/****************************************************************************
* Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
* Copyright 2023-2025 NXP
*
* SPDX-License-Identifier: MIT
*****************************************************************************/






#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/return_codes.h>
#include <common/misc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "vvsensor.h"

CREATE_TRACER( IMX258_INFO , "IMX258: ", INFO,    1);
CREATE_TRACER( IMX258_WARN , "IMX258: ", WARNING, 1);
CREATE_TRACER( IMX258_ERROR, "IMX258: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#endif

#define IMX258_PIDH_DEFAULT                        (0x02) //read only
#define IMX258_PIDL_DEFAULT                        (0x58) //read only
static const char SensorName[16] = "imx258";

typedef struct IMX258_Context_s
{
    IsiSensorContext_t  IsiCtx;
    struct vvcam_mode_info_s CurMode;
    IsiSensorAeInfo_t AeInfo;
    IsiSensorIntTime_t IntTime;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    IsiSensorGain_t SensorGain;
    uint32_t minAfps;
    uint64_t AEStartExposure;
    int motor_fd;
    uint32_t focus_mode;
} IMX258_Context_t;

static RESULT IMX258_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __func__);
    TRACE( IMX258_INFO, "%s: set power %d\n", __func__,on);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(IMX258_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( IMX258_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX258_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(IMX258_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(IMX258_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    uint32_t exp_line_time = pIMX258Ctx->CurMode.ae_info.one_line_exp_time_ns;

    IsiSensorAeInfo_t *pAeInfo = &pIMX258Ctx->AeInfo;
    pAeInfo->oneLineExpTime = (exp_line_time << ISI_EXPO_PARAS_FIX_FRACBITS) / 1000;

    if (pIMX258Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pIMX258Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->minIntTime.linearInt =
            pIMX258Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->maxAGain.linearGainParas = pIMX258Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pIMX258Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pIMX258Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pIMX258Ctx->CurMode.ae_info.min_dgain;
    }
    pAeInfo->gainStep = pIMX258Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pIMX258Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pIMX258Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pIMX258Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pIMX258Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pIMX258Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pIMX258Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pIMX258Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pIMX258Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pIMX258Ctx->minAfps != 0) {
        pAeInfo->minAfps = pIMX258Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pIMX258Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(IMX258_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX258Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    IMX258_UpdateIsiAEInfo(handle);

    TRACE(IMX258_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(IMX258_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(IMX258_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    IMX258_Context_t *pIMX258Ctx;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pIMX258Ctx = (IMX258_Context_t *) malloc(sizeof(IMX258_Context_t));
    if (!pIMX258Ctx)
        return RET_OUTOFMEM;

    memset(pIMX258Ctx, 0, sizeof(IMX258_Context_t));
    pIMX258Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX258Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pIMX258Ctx;

    result = IMX258_IsiSensorSetPowerIss(pIMX258Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = IMX258_IsiSensorGetClkIss(pIMX258Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = IMX258_IsiSensorSetClkIss(pIMX258Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = IMX258_IsiResetSensorIss(pIMX258Ctx);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = IMX258_IsiSetSensorModeIss(pIMX258Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX258_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(IMX258_INFO, "%s (enter) \n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    if (pIMX258Ctx == NULL)
        return (RET_WRONG_HANDLE);

    IMX258_IsiSensorSetStreamingIss(pIMX258Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    IMX258_IsiSensorGetClkIss(pIMX258Ctx, &clk);
    clk.status = 0;
    IMX258_IsiSensorSetClkIss(pIMX258Ctx, &clk);
    IMX258_IsiSensorSetPowerIss(pIMX258Ctx, BOOL_FALSE);
    free(pIMX258Ctx);
    pIMX258Ctx = NULL;

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX258_INFO, "%s (enter) \n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    result = IMX258_IsiHalQuerySensorIss(pIMX258Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(IMX258_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX258_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX258_INFO, "%s (enter) \n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = IMX258_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pIMX258Ctx->CurMode.index;

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX258_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pIMX258Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = IMX258_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(IMX258_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pIMX258Ctx->CurMode.size.bounds_width;
    format.format.height = pIMX258Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(IMX258_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(IMX258_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(IMX258_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = IMX258_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(IMX258_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != ((IMX258_PIDH_DEFAULT << 8) | (IMX258_PIDL_DEFAULT))) {
        TRACE(IMX258_ERROR,
            "%s:ChipID=0x2770,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pIMX258Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pIMX258Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT IMX258_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t IntLine;
    uint32_t oneLineTime;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pIMX258Ctx->AeInfo.oneLineExpTime;
    pIMX258Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = (pIntegrationTime->IntegrationTime.linearInt +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pIMX258Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX258_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pIMX258Ctx->IntLine = IntLine;
            }
            TRACE(IMX258_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pIMX258Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;

        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pIMX258Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t Gain;

    TRACE(IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pIMX258Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pIMX258Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX258_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pIMX258Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(IMX258_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(IMX258_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT IMX258_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(IMX258_INFO, "%s: (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pIMX258Ctx->CurMode.ae_info.cur_fps;

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(IMX258_INFO, "%s: (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(IMX258_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(IMX258_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX258Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    IMX258_UpdateIsiAEInfo(handle);

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT IMX258_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    TRACE(IMX258_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pIMX258Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pIMX258Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pIMX258Ctx->minAfps = minAfps;
    pIMX258Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    TRACE(IMX258_INFO, "%s: (enter)\n", __func__);

    if (pIMX258Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

#ifndef ISI_LITE

static RESULT IMX258_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( IMX258_INFO, "%s (enter)\n", __func__);

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX258Ctx->IsiCtx.HalHandle;

    struct sensor_test_pattern_s TestPattern;
    if (tpgMode == ISI_TPG_DISABLE) {
        TestPattern.enable = 0;
        TestPattern.pattern = 0;
    } else {
        TestPattern.enable = 1;
        TestPattern.pattern = (uint32_t)tpgMode - 1;
    }

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_TEST_PATTERN, &TestPattern);
    if (ret != 0)
    {
        TRACE(IMX258_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(IMX258_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX258_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( IMX258_INFO, "%s (enter)\n", __func__);
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    if (pIMX258Ctx->AEStartExposure == 0) {
        pIMX258Ctx->AEStartExposure =
            (uint64_t)pIMX258Ctx->CurMode.ae_info.start_exposure *
            pIMX258Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pIMX258Ctx->AEStartExposure;
    TRACE(IMX258_INFO, "%s:get start exposure %d\n", __func__, pIMX258Ctx->AEStartExposure);

    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX258_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( IMX258_INFO, "%s (enter)\n", __func__);
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *) handle;

    pIMX258Ctx->AEStartExposure = exposure;
    TRACE(IMX258_INFO, "set start exposure %d\n", __func__,pIMX258Ctx->AEStartExposure);
    TRACE(IMX258_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT IMX258_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( IMX258_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = IMX258_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = IMX258_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = IMX258_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = IMX258_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = IMX258_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = IMX258_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = IMX258_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = IMX258_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = IMX258_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = IMX258_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = IMX258_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = IMX258_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = IMX258_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = IMX258_IsiGetAeInfoIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = IMX258_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = IMX258_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = IMX258_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = IMX258_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = IMX258_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = IMX258_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = IMX258_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = IMX258_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiActivateTestPatternIss       = IMX258_IsiSetTestPatternIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = IMX258_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = IMX258_IsiGetAeStartExposureIs;
#endif
    TRACE( IMX258_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = ((IMX258_PIDH_DEFAULT << 8) | (IMX258_PIDL_DEFAULT)),
    .pIsiHalQuerySensor = IMX258_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = IMX258_IsiGetSensorIss,
};
