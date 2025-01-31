/*
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 * Copyright 2023-2025 NXP
 *
 * SPDX-License-Identifier: (GPL-2.0-only OR MIT)
 */

/****************************************************************************
 * Note: This software is released under dual MIT and GPL licenses. A
 * recipient may use this file under the terms of either the MIT license or
 * GPL License. If you wish to use only one license not the other, you can
 * indicate your decision by deleting one of the above license notices in your
 * version of this file.
 *****************************************************************************/

#ifndef _VVCAM_IMX258_REGS_1080P_H_
#define _VVCAM_IMX258_REGS_1080P_H_

#include "vvsensor.h"

/* 1080P30 RAW10 */
static struct vvcam_sccb_data_s imx258_init_setting_1080p[] = {
    { 0x0112,0x0A },        //  CSI_DT_FMT_H        
    { 0x0113,0x0A },        //  CSI_DT_FMT_L
    { 0x0114,0x03 },        //  CSI_LANE_MODE
    { 0x0301,0x04 },        //  IVTPXCK_DIV
    { 0x0303,0x02 },        //  IVTSYCK_DIV
    { 0x0305,0x03 },        //  PREPLLCK_IVT_DIV
    { 0x0306,0x00 },        //  PLL_IVT_MPY
    { 0x0307,0x41 },        //  PLL_IVT_MPY
    { 0x0309,0x0A },        //  IOPPXCK_DIV
    { 0x030B,0x01 },        //  IOPSYCK_DIV
    { 0x030D,0x02 },        //  PREPLLCK_IOP_DIV
    { 0x030E,0x00 },        //  PLL_IOP_MPY
    { 0x030F,0x48 },        //  PLL_IOP_MPY
    { 0x0310,0x00 },        //  PLL_MULT_DRIV
    { 0x0342,0x14 },        //  LINE_LENGTH_PCK
    { 0x0343,0xE8 },        //  LINE_LENGTH_PCK
    { 0x0340,0x06 },        //  FRM_LENGTH_LINES
    { 0x0341,0x54 },        //  FRM_LENGTH_LINES
    { 0x0344,0x00 },        //  X_ADD_STA
    { 0x0345,0x00 },        //  X_ADD_STA
    { 0x0346,0x01 },        //  Y_ADD_STA
    { 0x0347,0xE0 },        //  Y_ADD_STA
    { 0x0348,0x10 },        //  X_ADD_END
    { 0x0349,0x6F },        //  X_ADD_END
    { 0x034A,0x0A },        //  Y_ADD_END
    { 0x034B,0x4F },        //  Y_ADD_END
    { 0x0381,0x01 },        //  X_EVN_INC
    { 0x0383,0x01 },        //  X_ODD_INC
    { 0x0385,0x01 },        //  Y_EVN_INC
    { 0x0387,0x01 },        //  Y_ODD_INC
    { 0x0900,0x01 },        //  BINNING_MODE
    { 0x0901,0x12 },        //  [7:4]BINNING_TYPE_H[3:0]BINNING_TYPE_V
    { 0x0401,0x01 },        //  SCALE_MODE
    { 0x0404,0x00 },        //  SCALE_M
    { 0x0405,0x20 },        //  SCALE_M
    { 0x0408,0x00 },        //  DIG_CROP_X_OFFSET
    { 0x0409,0xB8 },        //  DIG_CROP_X_OFFSET
    { 0x040A,0x00 },        //  DIG_CROP_Y_OFFSET
    { 0x040B,0x00 },        //  DIG_CROP_Y_OFFSET
    { 0x040C,0x0F },        //  DIG_CROP_IMAGE_WIDTH
    { 0x040D,0x00 },        //  DIG_CROP_IMAGE_WIDTH
    { 0x040E,0x04 },        //  DIG_CROP_IMAGE_HEIGHT
    { 0x040F,0x38 },        //  DIG_CROP_IMAGE_HEIGHT
    { 0x3038,0x00 },        //  SCALE_MODE_EXT
    { 0x303A,0x00 },        //  SCALE_M_EXT
    { 0x303B,0x10 },        //  SCALE_M_EXT
    { 0x300D,0x00 },        //  FORCE_FD_SUM
    { 0x034C,0x07 },        //  X_OUT_SIZE
    { 0x034D,0x80 },        //  X_OUT_SIZE
    { 0x034E,0x04 },        //  Y_OUT_SIZE
    { 0x034F,0x38 },        //  Y_OUT_SIZE
    { 0x0202,0x04 },        //  COARSE_INTEG_TIME
    { 0x0203,0x28 },        //  COARSE_INTEG_TIME
    { 0x0204,0x03 },        //  ANA_GAIN_GLOBAL
    { 0x0205,0x00 },        //  ANA_GAIN_GLOBAL
    { 0x020E,0x01 },        //  DIG_GAIN_GR
    { 0x020F,0x00 },        //  DIG_GAIN_GR
    { 0x0210,0x01 },        //  DIG_GAIN_R
    { 0x0211,0x00 },        //  DIG_GAIN_R
    { 0x0212,0x01 },        //  DIG_GAIN_B
    { 0x0213,0x00 },        //  DIG_GAIN_B
    { 0x0214,0x01 },        //  DIG_GAIN_GB
    { 0x0215,0x00 },        //  DIG_GAIN_GB
    { 0x7BCD,0x00 },        //  AF_WINDOW_MODE
    { 0x94DC,0x20 },        //  
    { 0x94DD,0x20 },        //  
    { 0x94DE,0x20 },        //  
    { 0x95DC,0x20 },        //  
    { 0x95DD,0x20 },        //  
    { 0x95DE,0x20 },        //  
    { 0x7FB0,0x00 },        //  
    { 0x9010,0x3E },        //  
    { 0x9419,0x50 },        //  
    { 0x941B,0x50 },        //  
    { 0x9519,0x50 },        //  
    { 0x951B,0x50 },        //  
    { 0x3030,0x00 },        //  PHASE_PIX_OUTEN
    { 0x3032,0x00 },        //  PDPIX_DATA_RATE
    { 0x0220,0x00 },        //  HDR_MODE
    { 0x4084,0x00 },        //  
    { 0x4085,0x47 },        //  
    { 0x3170,0x00 },        //  
    { 0x3171,0x18 },        //  
    { 0x3172,0x10 },        //  
    { 0x3173,0x57 },        //  
    { 0x3174,0x00 },        //  
    { 0x3175,0x16 },        //  
    { 0x3176,0x0C },        //  
    { 0x3177,0x15 },        //  
};

#endif
