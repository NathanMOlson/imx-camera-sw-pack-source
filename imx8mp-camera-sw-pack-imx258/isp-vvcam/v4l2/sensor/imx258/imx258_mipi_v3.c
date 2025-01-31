/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd. 
 * Copyright 2018,2023-2025 NXP
 * 
 * SPDX-License-Identifier: GPL-2.0-only
 */ 
 
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "vvsensor.h"

#include "imx258_regs_1080p.h"
#include "imx258_regs_1080p60.h"
#include "imx258_regs_12MP.h"
#include "imx258_regs_8MP.h"

#define IMX258_SENS_PAD_SOURCE	0
#define IMX258_SENS_PADS_NUM	1

#define IMX258_XCLK_MAX  27000000
#define IMX258_XCLK_MIN  6000000

#define IMX258_REG_VALUE_08BIT		1
#define IMX258_REG_VALUE_16BIT		2

#define IMX258_REG_MODE_SELECT		0x0100
#define IMX258_MODE_STANDBY		0x00
#define IMX258_MODE_STREAMING		0x01

/* Chip ID */
#define IMX258_REG_CHIP_ID		0x0016
#define IMX258_CHIP_ID			0x0258

/* V_TIMING internal */
#define IMX258_VTS_30FPS		0x0c50
#define IMX258_VTS_30FPS_2K		0x0638
#define IMX258_VTS_30FPS_VGA		0x034c
#define IMX258_VTS_MAX			0xffff
#define IMX258_FRM_LENGTH_LINES 0x0340

/*Frame Length Line*/
#define IMX258_FLL_MIN			0x08a6
#define IMX258_FLL_MAX			0xffff
#define IMX258_FLL_STEP			1
#define IMX258_FLL_DEFAULT		0x0c98

/* HBLANK control - read only */
#define IMX258_PPL_DEFAULT		5352

#define IMX258_GRP_PARAM_HOLD	0x104
/* Exposure control */
#define IMX258_REG_EXPOSURE		0x0202
#define IMX258_EXPOSURE_MIN		4
#define IMX258_EXPOSURE_STEP		1
#define IMX258_EXPOSURE_DEFAULT		0x640
#define IMX258_EXPOSURE_MAX		65535

/* Analog gain control */
#define IMX258_REG_ANALOG_GAIN		0x0204
#define IMX258_ANA_GAIN_MIN			1
#define IMX258_ANA_GAIN_MAX			16
#define IMX258_ANA_GAIN_MAX_VAL		480

/* Digital gain control */
#define IMX258_REG_GR_DIGITAL_GAIN	0x020e
#define IMX258_REG_R_DIGITAL_GAIN	0x0210
#define IMX258_REG_B_DIGITAL_GAIN	0x0212
#define IMX258_REG_GB_DIGITAL_GAIN	0x0214
#define IMX258_DGTL_GAIN_MIN		1
#define IMX258_DGTL_GAIN_INT_SHIFT	8
#define IMX258_DGTL_GAIN_MAX		4096	/* Max = 0xFFF */
#define IMX258_DGTL_GAIN_DEFAULT	1024
#define IMX258_DGTL_GAIN_STEP		1

/* HDR control */
#define IMX258_REG_HDR			0x0220
#define IMX258_HDR_ON			BIT(0)
#define IMX258_REG_HDR_RATIO		0x0222
#define IMX258_HDR_RATIO_MIN		0
#define IMX258_HDR_RATIO_MAX		5
#define IMX258_HDR_RATIO_STEP		1
#define IMX258_HDR_RATIO_DEFAULT	0x0

/* Test Pattern Control */
#define IMX258_REG_TEST_PATTERN		0x0600
#define IMX258_NUM_TEST_PATTERN		0x03

/* Orientation */
#define REG_MIRROR_FLIP_CONTROL		0x0101
#define REG_CONFIG_MIRROR_FLIP		0x03
#define REG_CONFIG_FLIP_TEST_PATTERN	0x02

#define client_to_imx258(client)\
	container_of(i2c_get_clientdata(client), struct imx258, subdev)

struct imx258_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct imx258 {
	struct i2c_client *i2c_client;
	unsigned int pwn_gpio;
	unsigned int mclk;
	unsigned int mclk_source;
	struct clk *sensor_clk;
	unsigned int csi_id;
	struct imx258_capture_properties ocp;

	struct v4l2_subdev subdev;
	struct media_pad pads[IMX258_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	sensor_blc_t blc;
	sensor_white_balance_t wb;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
	u32 hcg_again;
	u32 hcg_dgain;
};

/* Stand-by OFF Sequence */
// External Clock Setting
static struct vvcam_sccb_data_s imx258_init_sequence[] = {
	// addr   data
	{ 0x0136, 0x18 },
	{ 0x0137, 0x00 },
	{ 0x3051, 0x00 },
	{ 0x6B11, 0xCF },
	{ 0x7FF0, 0x08 },
	{ 0x7FF1, 0x0F },
	{ 0x7FF2, 0x08 },
	{ 0x7FF3, 0x1B },
	{ 0x7FF4, 0x23 },
	{ 0x7FF5, 0x60 },
	{ 0x7FF6, 0x00 },
	{ 0x7FF7, 0x01 },
	{ 0x7FF8, 0x00 },
	{ 0x7FF9, 0x78 },
	{ 0x7FFA, 0x01 },
	{ 0x7FFB, 0x00 },
	{ 0x7FFC, 0x00 },
	{ 0x7FFD, 0x00 },
	{ 0x7FFE, 0x00 },
	{ 0x7FFF, 0x03 },
	{ 0x7F76, 0x03 },
	{ 0x7F77, 0xFE },
	{ 0x7FA8, 0x03 },
	{ 0x7FA9, 0xFE },
	{ 0x7B24, 0x81 },
	{ 0x7B25, 0x01 },
	{ 0x6564, 0x07 },
	{ 0x6B0D, 0x41 },
	{ 0x653D, 0x04 },
	{ 0x6B05, 0x8C },
	{ 0x6B06, 0xF9 },
	{ 0x6B08, 0x65 },
	{ 0x6B09, 0xFC },
	{ 0x6B0A, 0xCF },
	{ 0x6B0B, 0xD2 },
	{ 0x6700, 0x0E },
	{ 0x6707, 0x0E },
	{ 0x9104, 0x04 },
	{ 0x7421, 0x5C },
	{ 0x7423, 0xD7 },
	{ 0x5F04, 0x00 },
	{ 0x5F05, 0xED },
	{ 0x0106, 0x01 },
	{ 0x4648, 0x7F },
	{ 0x7420, 0x00 },
	{ 0x7422, 0x00 },
	{ 0x0C18, 0x10 },
	{ 0x0C19, 0x00 },
	{ 0x0C12, 0x05 },
	{ 0x7BC8, 0x01 },
	{ 0x7BC9, 0x01 },
};

static struct vvcam_sccb_data_s imx258_defectpixel_correct[] = {
	{ 0x94C7, 0xFF },
	{ 0x94C8, 0xFF },
	{ 0x94C9, 0xFF },
	{ 0x95C7, 0xFF },
	{ 0x95C8, 0xFF },
	{ 0x95C9, 0xFF },
	{ 0x94C4, 0x3F },
	{ 0x94C5, 0x3F },
	{ 0x94C6, 0x3F },
	{ 0x95C4, 0x3F },
	{ 0x95C5, 0x3F },
	{ 0x95C6, 0x3F },
	{ 0x94C1, 0x02 },
	{ 0x94C2, 0x02 },
	{ 0x94C3, 0x02 },
	{ 0x95C1, 0x02 },
	{ 0x95C2, 0x02 },
	{ 0x95C3, 0x02 },
	{ 0x94BE, 0x0C },
	{ 0x94BF, 0x0C },
	{ 0x94C0, 0x0C },
	{ 0x95BE, 0x0C },
	{ 0x95BF, 0x0C },
	{ 0x95C0, 0x0C },
	{ 0x94D0, 0x74 },
	{ 0x94D1, 0x74 },
	{ 0x94D2, 0x74 },
	{ 0x95D0, 0x74 },
	{ 0x95D1, 0x74 },
	{ 0x95D2, 0x74 },
	{ 0x94CD, 0x2E },
	{ 0x94CE, 0x2E },
	{ 0x94CF, 0x2E },
	{ 0x95CD, 0x2E },
	{ 0x95CE, 0x2E },
	{ 0x95CF, 0x2E },
	{ 0x94CA, 0x4C },
	{ 0x94CB, 0x4C },
	{ 0x94CC, 0x4C },
	{ 0x95CA, 0x4C },
	{ 0x95CB, 0x4C },
	{ 0x95CC, 0x4C },
	{ 0x900E, 0x32 },
	{ 0x94E2, 0xFF },
	{ 0x94E3, 0xFF },
	{ 0x94E4, 0xFF },
	{ 0x95E2, 0xFF },
	{ 0x95E3, 0xFF },
	{ 0x95E4, 0xFF },
	{ 0x94DF, 0x6E },
	{ 0x94E0, 0x6E },
	{ 0x94E1, 0x6E },
	{ 0x95DF, 0x6E },
	{ 0x95E0, 0x6E },
	{ 0x95E1, 0x6E },
	{ 0x7FCC, 0x01 },
	{ 0x7B78, 0x00 },
	{ 0x9401, 0x35 },
	{ 0x9403, 0x23 },
	{ 0x9405, 0x23 },
	{ 0x9406, 0x00 },
	{ 0x9407, 0x31 },
	{ 0x9408, 0x00 },
	{ 0x9409, 0x1B },
	{ 0x940A, 0x00 },
	{ 0x940B, 0x15 },
	{ 0x940D, 0x3F },
	{ 0x940F, 0x3F },
	{ 0x9411, 0x3F },
	{ 0x9413, 0x64 },
	{ 0x9415, 0x64 },
	{ 0x9417, 0x64 },
	{ 0x941D, 0x34 },
	{ 0x941F, 0x01 },
	{ 0x9421, 0x01 },
	{ 0x9423, 0x01 },
	{ 0x9425, 0x23 },
	{ 0x9427, 0x23 },
	{ 0x9429, 0x23 },
	{ 0x942B, 0x2F },
	{ 0x942D, 0x1A },
	{ 0x942F, 0x14 },
	{ 0x9431, 0x3F },
	{ 0x9433, 0x3F },
	{ 0x9435, 0x3F },
	{ 0x9437, 0x6B },
	{ 0x9439, 0x7C },
	{ 0x943B, 0x81 },
	{ 0x9443, 0x0F },
	{ 0x9445, 0x0F },
	{ 0x9447, 0x0F },
	{ 0x9449, 0x0F },
	{ 0x944B, 0x0F },
	{ 0x944D, 0x0F },
	{ 0x944F, 0x1E },
	{ 0x9451, 0x0F },
	{ 0x9453, 0x0B },
	{ 0x9455, 0x28 },
	{ 0x9457, 0x13 },
	{ 0x9459, 0x0C },
	{ 0x945D, 0x00 },
	{ 0x945E, 0x00 },
	{ 0x945F, 0x00 },
	{ 0x946D, 0x00 },
	{ 0x946F, 0x10 },
	{ 0x9471, 0x10 },
	{ 0x9473, 0x40 },
	{ 0x9475, 0x2E },
	{ 0x9477, 0x10 },
	{ 0x9478, 0x0A },
	{ 0x947B, 0xE0 },
	{ 0x947C, 0xE0 },
	{ 0x947D, 0xE0 },
	{ 0x947E, 0xE0 },
	{ 0x947F, 0xE0 },
	{ 0x9480, 0xE0 },
	{ 0x9483, 0x14 },
	{ 0x9485, 0x14 },
	{ 0x9487, 0x14 },
	{ 0x9501, 0x35 },
	{ 0x9503, 0x14 },
	{ 0x9505, 0x14 },
	{ 0x9507, 0x31 },
	{ 0x9509, 0x1B },
	{ 0x950B, 0x15 },
	{ 0x950D, 0x1E },
	{ 0x950F, 0x1E },
	{ 0x9511, 0x1E },
	{ 0x9513, 0x64 },
	{ 0x9515, 0x64 },
	{ 0x9517, 0x64 },
	{ 0x951D, 0x34 },
	{ 0x951F, 0x01 },
	{ 0x9521, 0x01 },
	{ 0x9523, 0x01 },
	{ 0x9525, 0x14 },
	{ 0x9527, 0x14 },
	{ 0x9529, 0x14 },
	{ 0x952B, 0x2F },
	{ 0x952D, 0x1A },
	{ 0x952F, 0x14 },
	{ 0x9531, 0x1E },
	{ 0x9533, 0x1E },
	{ 0x9535, 0x1E },
	{ 0x9537, 0x6B },
	{ 0x9539, 0x7C },
	{ 0x953B, 0x81 },
	{ 0x9543, 0x0F },
	{ 0x9545, 0x0F },
	{ 0x9547, 0x0F },
	{ 0x9549, 0x0F },
	{ 0x954B, 0x0F },
	{ 0x954D, 0x0F },
	{ 0x954F, 0x15 },
	{ 0x9551, 0x0B },
	{ 0x9553, 0x08 },
	{ 0x9555, 0x1C },
	{ 0x9557, 0x0D },
	{ 0x9559, 0x08 },
	{ 0x955D, 0x00 },
	{ 0x955E, 0x00 },
	{ 0x955F, 0x00 },
	{ 0x956D, 0x00 },
	{ 0x956F, 0x10 },
	{ 0x9571, 0x10 },
	{ 0x9573, 0x40 },
	{ 0x9575, 0x2E },
	{ 0x9577, 0x10 },
	{ 0x9578, 0x0A },
	{ 0x957B, 0xE0 },
	{ 0x957C, 0xE0 },
	{ 0x957D, 0xE0 },
	{ 0x957E, 0xE0 },
	{ 0x957F, 0xE0 },
	{ 0x9580, 0xE0 },
	{ 0x9583, 0x14 },
	{ 0x9585, 0x14 },
	{ 0x9587, 0x14 },
	{ 0x7F78, 0x00 },
	{ 0x7F89, 0x00 },
	{ 0x7F93, 0x00 },
	{ 0x924B, 0x1B },
	{ 0x924C, 0x0A },
	{ 0x9304, 0x04 },
	{ 0x9315, 0x04 },
	{ 0x9250, 0x50 },
	{ 0x9251, 0x3C },
	{ 0x9252, 0x14 },
};

static struct vvcam_mode_info_s pimx258_mode_info[] = {
	{
		.index          = 0,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = 0x654,
			.curr_frm_len_lines    = 0x654,
			.one_line_exp_time_ns  = 20576,

			.max_integration_line  = 0xffff - 10,
			.min_integration_line  = 1,

			.max_again             = 16 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = 16380,
			.min_dgain             = 1 * 1024,
			.gain_step             = 1,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx258_init_setting_1080p,
		.reg_data_count = ARRAY_SIZE(imx258_init_setting_1080p),
	},
	{
		.index          = 1,
		.size           = {
			.bounds_width  = 1920,
			.bounds_height = 1080,
			.top           = 0,
			.left          = 0,
			.width         = 1920,
			.height        = 1080,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = 0x654,
			.curr_frm_len_lines    = 0x654,
			.one_line_exp_time_ns  = 20576,

			.max_integration_line  = 0xffff - 10,
			.min_integration_line  = 1,

			.max_again             = 16 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = 16380,
			.min_dgain             = 1 * 1024,
			.gain_step             = 1,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 60 * 1024,
			.max_fps               = 60 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx258_init_setting_1080p60,
		.reg_data_count = ARRAY_SIZE(imx258_init_setting_1080p60),
	},
	{
		.index          = 2,
		.size           = {
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = 0xA20,
			.curr_frm_len_lines    = 0xA20,
			.one_line_exp_time_ns  = 12860,

			.max_integration_line  = 0xffff - 10,
			.min_integration_line  = 1,

			.max_again             = 16 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = 16380,
			.min_dgain             = 1 * 1024,
			.gain_step             = 1,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx258_init_setting_8MP,
		.reg_data_count = ARRAY_SIZE(imx258_init_setting_8MP),
	},
	{
		.index          = 3,
		.size           = {
			.bounds_width  = 4096,
			.bounds_height = 3072,
			.top           = 0,
			.left          = 0,
			.width         = 4096,
			.height        = 3072,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = 0xC4C,
			.curr_frm_len_lines    = 0xC4C,
			.one_line_exp_time_ns  = 10589,

			.max_integration_line  = 0xffff - 10,
			.min_integration_line  = 1,

			.max_again             = 16 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = 16380,
			.min_dgain             = 1 * 1024,
			.gain_step             = 1,
			.start_exposure        = 3 * 400 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx258_init_setting_12MP,
		.reg_data_count = ARRAY_SIZE(imx258_init_setting_12MP),
	},
};

static int imx258_write_reg_arry(struct imx258 *sensor,
				 struct vvcam_sccb_data_s *reg_arry,
				 u32 size);
static int imx258_set_clk_rate(struct imx258 *sensor);

static int imx258_restore_setting(struct imx258 *sensor)
{
	int ret;

	ret = imx258_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)imx258_init_sequence,
		ARRAY_SIZE(imx258_init_sequence));
	if (ret < 0) {
		pr_err("%s: imx258_init_sequence failed\n", __func__);
		return ret;
	}

	ret = imx258_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)imx258_defectpixel_correct,
		ARRAY_SIZE(imx258_defectpixel_correct));
	if (ret < 0) {
		pr_err("%s: imx258_defectpixel_correct failed\n", __func__);
		return ret;
	}
	return ret;
}

int imx258_get_clk(struct imx258 *sensor, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret = 0;
	vvcam_clk.sensor_mclk = clk_get_rate(sensor->sensor_clk);
	vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (ret != 0)
		ret = -EINVAL;
	return ret;
}

static int imx258_power_on(struct imx258 *sensor)
{
	int ret;
	pr_debug("enter %s\n", __func__);

	imx258_set_clk_rate(sensor);
	ret = clk_prepare_enable(sensor->sensor_clk);
	if (ret < 0) {
		pr_err("%s: enable sensor clk fail\n", __func__);
		return ret;
	}

	msleep(1);
	if (gpio_is_valid(sensor->pwn_gpio))
		gpio_set_value_cansleep(sensor->pwn_gpio, 1);

	msleep(15);
	imx258_restore_setting(sensor);

	return ret;
}

static int imx258_power_off(struct imx258 *sensor)
{
	pr_debug("enter %s\n", __func__);
	if (gpio_is_valid(sensor->pwn_gpio))
		gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	clk_disable_unprepare(sensor->sensor_clk);

	return 0;
}

static int imx258_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx258 *sensor = client_to_imx258(client);

	pr_debug("enter %s\n", __func__);
	if (on)
		imx258_power_on(sensor);
	else
		imx258_power_off(sensor);

	return 0;
}

static int imx258_write_reg(struct imx258 *sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(sensor->i2c_client, au8Buf, 3) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}

	return 0;
}

static int imx258_read_reg(struct imx258 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return 0;
}

static int imx258_write_reg_arry(struct imx258 *sensor,
				 struct vvcam_sccb_data_s *reg_arry,
				 u32 size)
{
	int i = 0;
	int ret = 0;
	struct i2c_msg msg;
	u8 *send_buf;
	u32 send_buf_len = 0;
	struct i2c_client *i2c_client = sensor->i2c_client;

	send_buf = (u8 *)kmalloc(size + 2, GFP_KERNEL);
	if (!send_buf)
		return -ENOMEM;

	send_buf[send_buf_len++] = (reg_arry[0].addr >> 8) & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].addr & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].data & 0xff;
	for (i=1; i < size; i++) {
		if (reg_arry[i].addr == (reg_arry[i-1].addr + 1)){
			send_buf[send_buf_len++] = reg_arry[i].data & 0xff;
		} else {
			msg.addr  = i2c_client->addr;
			msg.flags = i2c_client->flags;
			msg.buf   = send_buf;
			msg.len   = send_buf_len;
			ret = i2c_transfer(i2c_client->adapter, &msg, 1);
			if (ret < 0) {
				pr_err("%s:i2c transfer error\n",__func__);
				kfree(send_buf);
				return ret;
			}
			send_buf_len = 0;
			send_buf[send_buf_len++] =
				(reg_arry[i].addr >> 8) & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].addr & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].data & 0xff;
		}
	}

	if (send_buf_len > 0) {
		msg.addr  = i2c_client->addr;
		msg.flags = i2c_client->flags;
		msg.buf   = send_buf;
		msg.len   = send_buf_len;
		ret = i2c_transfer(i2c_client->adapter, &msg, 1);
		if (ret < 0)
			pr_err("%s:i2c transfer end meg error\n",__func__);
		else
			ret = 0;

	}
	kfree(send_buf);
	return ret;
}

static int imx258_query_capability(struct imx258 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "imx258");
	sprintf((char *)pcap->bus_info, "csi%d",sensor->csi_id);
	if(sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int imx258_query_supports(struct imx258 *sensor, void* parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(pimx258_mode_info);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, pimx258_mode_info,
			   sizeof(pimx258_mode_info));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;

}

static int imx258_get_sensor_id(struct imx258 *sensor, void* pchip_id)
{
	int ret = 0;
	u16 chip_id;
	u8 chip_id_high = 0;
	u8 chip_id_low = 0;

	imx258_read_reg(sensor, IMX258_REG_CHIP_ID, &chip_id_high);
	imx258_read_reg(sensor, IMX258_REG_CHIP_ID + 1, &chip_id_low);

	chip_id = ((chip_id_high & 0xff) << 8) | (chip_id_low & 0xff);

	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx258_get_reserve_id(struct imx258 *sensor, void* preserve_id)
{
	int ret = 0;
	u16 reserve_id = IMX258_CHIP_ID;
	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx258_get_sensor_mode(struct imx258 *sensor, void* pmode)
{
	int ret = 0;
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx258_set_sensor_mode(struct imx258 *sensor, void* pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;
	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(pimx258_mode_info); i++) {
		if (pimx258_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pimx258_mode_info[i],
				sizeof(struct vvcam_mode_info_s));

			return 0;
		}
	}

	return -ENXIO;
}


static int imx258_set_exp(struct imx258 *sensor, u16 exp)
{
	int ret = 0;
	u8 exp8;

	exp8 = exp >> 8;
	imx258_write_reg(sensor, IMX258_GRP_PARAM_HOLD, 1);
	ret |= imx258_write_reg(sensor, IMX258_REG_EXPOSURE, exp8);
	exp8 = exp & 0xff;
	ret |= imx258_write_reg(sensor, IMX258_REG_EXPOSURE + 1, exp8);
	imx258_write_reg(sensor, IMX258_GRP_PARAM_HOLD, 0);

	return ret;
}

static int imx258_calc_separate_gains(u32 total_gain, u32 *again, u32 *dgain)
{
	if(!again | !dgain)
		pr_err("%s: again or dgain empty\n", __func__);

	if(total_gain <= (IMX258_ANA_GAIN_MAX << SENSOR_FIX_FRACBITS)) {
		*dgain = IMX258_DGTL_GAIN_MIN << IMX258_DGTL_GAIN_INT_SHIFT;
		*again = (total_gain - (1 << SENSOR_FIX_FRACBITS)) * 512 / total_gain;
	}
	else {
		*again = IMX258_ANA_GAIN_MAX_VAL;
		*dgain = ((total_gain / IMX258_ANA_GAIN_MAX) >> (SENSOR_FIX_FRACBITS - IMX258_DGTL_GAIN_INT_SHIFT)) & 0xfff;
	}
	return 0;
}

static int imx258_update_analog_gain(struct imx258 *sensor, u16 val16)
{
	int ret;
	u8 val8;

	val8 = val16 >> 8;
	ret = imx258_write_reg(sensor, IMX258_REG_ANALOG_GAIN, val8);
	if (ret)
		return ret;
	val8 = val16 & 0xff;
	ret = imx258_write_reg(sensor, IMX258_REG_ANALOG_GAIN + 1, val8);
	if (ret)
		return ret;

	return 0;
}

static int imx258_update_digital_gain(struct imx258 *sensor, u16 val16)
{
	int ret;
	u8 val8;

	val8 = val16 >> 8;
	ret = imx258_write_reg(sensor, IMX258_REG_GR_DIGITAL_GAIN, val8);
	if (ret)
		return ret;
	val8 = val16 & 0xff;
	ret = imx258_write_reg(sensor, IMX258_REG_GR_DIGITAL_GAIN + 1, val8);
	if (ret)
		return ret;

	val8 = val16 >> 8;
	ret = imx258_write_reg(sensor, IMX258_REG_GB_DIGITAL_GAIN, val8);
	if (ret)
		return ret;
	val8 = val16 & 0xff;
	ret = imx258_write_reg(sensor, IMX258_REG_GB_DIGITAL_GAIN + 1, val8);
	if (ret)
		return ret;

	val8 = val16 >> 8;
	ret = imx258_write_reg(sensor, IMX258_REG_R_DIGITAL_GAIN, val8);
	if (ret)
		return ret;
	val8 = val16 & 0xff;
	ret = imx258_write_reg(sensor, IMX258_REG_R_DIGITAL_GAIN + 1, val8);
	if (ret)
		return ret;

	val8 = val16 >> 8;
	ret = imx258_write_reg(sensor, IMX258_REG_B_DIGITAL_GAIN, val8);
	if (ret)
		return ret;
	val8 = val16 & 0xff;
	ret = imx258_write_reg(sensor, IMX258_REG_B_DIGITAL_GAIN + 1, val8);
	if (ret)
		return ret;

	return 0;
}

static int imx258_set_gain(struct imx258 *sensor, u32 gain)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;
	u32 again = 0;
	u32 dgain = 0;

	if (gain < (1 << SENSOR_FIX_FRACBITS)){
		gain = 1  << SENSOR_FIX_FRACBITS;
	}

	imx258_calc_separate_gains(gain, &again, &dgain);

	dev_dbg(dev, "again:%d, dgain:%d\n", again, dgain);

	imx258_write_reg(sensor, IMX258_GRP_PARAM_HOLD, 1);
	imx258_update_analog_gain(sensor, again);
	imx258_update_digital_gain(sensor, dgain);
	imx258_write_reg(sensor, IMX258_GRP_PARAM_HOLD, 0);

	return ret;
}

static int imx258_get_fps(struct imx258 *sensor, u32 *pfps)
{
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int imx258_set_fps(struct imx258 *sensor, u32 fps)
{
	u32 vts;
	int ret = 0;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		fps = sensor->cur_mode.ae_info.max_fps;
	}
	else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}
	vts = sensor->cur_mode.ae_info.max_fps *
	      sensor->cur_mode.ae_info.def_frm_len_lines / fps;

	ret |= imx258_write_reg(sensor, IMX258_FRM_LENGTH_LINES, vts >> 8);
	ret |= imx258_write_reg(sensor, IMX258_FRM_LENGTH_LINES + 1, vts);
	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		sensor->cur_mode.ae_info.max_integration_line = vts - 1;
	}

	sensor->cur_mode.ae_info.curr_frm_len_lines = vts;
	return ret;
}

static int imx258_set_test_pattern(struct imx258 *sensor, void * arg)
{
	int ret;
	struct sensor_test_pattern_s test_pattern;

	ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
	if (ret != 0)
		return -ENOMEM;
	if (test_pattern.enable) {
		if (test_pattern.pattern <= IMX258_NUM_TEST_PATTERN) {
			ret = imx258_write_reg(sensor, IMX258_REG_TEST_PATTERN + 1, test_pattern.pattern + 1);
		}
		else {
			ret = -1;
		}
	} else {
		ret = imx258_write_reg(sensor, IMX258_REG_TEST_PATTERN + 1, 0);
	}
	return ret;
}

static int imx258_get_format_code(struct imx258 *sensor, u32 *code)
{
		
	*code = MEDIA_BUS_FMT_SRGGB10_1X10;
	
	return 0;
}

static int imx258_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct imx258 *sensor = client_to_imx258(client);

	dev_dbg(dev, "enter %s\n", __func__);
	sensor->stream_status = enable;
	if (enable) {
		imx258_write_reg(sensor, IMX258_REG_MODE_SELECT, IMX258_MODE_STREAMING);
	} else  {
		imx258_write_reg(sensor, IMX258_REG_MODE_SELECT, IMX258_MODE_STANDBY);
		msleep(50);
	}

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx258_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int imx258_enum_mbus_code(struct v4l2_subdev *sd,
			         struct v4l2_subdev_pad_config *cfg,
			         struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx258 *sensor = client_to_imx258(client);

	u32 cur_code = MEDIA_BUS_FMT_SRGGB10_1X10;

	if (code->index > 0)
		return -EINVAL;
	imx258_get_format_code(sensor,&cur_code);
	code->code = cur_code;

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx258_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
#else
static int imx258_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
#endif
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx258 *sensor = client_to_imx258(client);
	mutex_lock(&sensor->lock);

	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
	    (fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__,fmt->format.width,fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx258_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	if (ret < 0) {
		pr_err("%s:imx258_write_reg_arry error\n",__func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	imx258_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx258_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
#else
static int imx258_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx258 *sensor = client_to_imx258(client);

	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static long imx258_priv_ioctl(struct v4l2_subdev *sd,
                              unsigned int cmd,
                              void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx258 *sensor = client_to_imx258(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;
	uint32_t value = 0;

	mutex_lock(&sensor->lock);
	switch (cmd){
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = imx258_get_clk(sensor,arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = imx258_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = imx258_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = imx258_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = imx258_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx258_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx258_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx258_s_stream(&sensor->subdev, value);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx258_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx258_read_reg(sensor, sensor_reg.addr,
			(u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_EXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx258_set_exp(sensor, value);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx258_set_gain(sensor, value);
		break;
	case VVSENSORIOC_G_FPS:
		ret = imx258_get_fps(sensor, &value);
		ret |= copy_to_user(arg, &value, sizeof(value));
		break;
	case VVSENSORIOC_S_FPS:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= imx258_set_fps(sensor, value);
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret= imx258_set_test_pattern(sensor, arg);
		break;
	default:
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static struct v4l2_subdev_video_ops imx258_subdev_video_ops = {
	.s_stream = imx258_s_stream,
};

static const struct v4l2_subdev_pad_ops imx258_subdev_pad_ops = {
	.enum_mbus_code = imx258_enum_mbus_code,
	.set_fmt = imx258_set_fmt,
	.get_fmt = imx258_get_fmt,
};

static struct v4l2_subdev_core_ops imx258_subdev_core_ops = {
	.s_power = imx258_s_power,
	.ioctl = imx258_priv_ioctl,
};

static struct v4l2_subdev_ops imx258_subdev_ops = {
	.core  = &imx258_subdev_core_ops,
	.video = &imx258_subdev_video_ops,
	.pad   = &imx258_subdev_pad_ops,
};

static int imx258_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx258_sd_media_ops = {
	.link_setup = imx258_link_setup,
};

static int imx258_set_clk_rate(struct imx258 *sensor)
{
	int ret;
	unsigned int clk;

	clk = sensor->mclk;
	clk = min_t(u32, clk, (u32)IMX258_XCLK_MAX);
	clk = max_t(u32, clk, (u32)IMX258_XCLK_MIN);
	sensor->mclk = clk;

	pr_debug("   Setting mclk to %d MHz\n",sensor->mclk / 1000000);
	ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", sensor->mclk);
	return ret;
}

static int imx258_retrieve_capture_properties(
			struct imx258 *sensor,
			struct imx258_capture_properties* ocp)
{
	struct device *dev = &sensor->i2c_client->dev;
	__u64 mlf = 0;
	__u64 mpf = 0;
	__u64 mdr = 0;

	struct device_node *ep;
	int ret;
	/*Collecting the information about limits of capture path
	* has been centralized to the sensor
	* * also into the sensor endpoint itself.
	*/

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

	ret = fwnode_property_read_u64(of_fwnode_handle(ep),
		"max-lane-frequency", &mlf);
	if (ret || mlf == 0) {
		dev_dbg(dev, "no limit for max-lane-frequency\n");
	}

	ret = fwnode_property_read_u64(of_fwnode_handle(ep),
	        "max-pixel-frequency", &mpf);
	if (ret || mpf == 0) {
	        dev_dbg(dev, "no limit for max-pixel-frequency\n");
	}

	ret = fwnode_property_read_u64(of_fwnode_handle(ep),
	        "max-data-rate", &mdr);
	if (ret || mdr == 0) {
	        dev_dbg(dev, "no limit for max-data_rate\n");
	}

	ocp->max_lane_frequency = mlf;
	ocp->max_pixel_frequency = mpf;
	ocp->max_data_rate = mdr;

	return ret;
}

/* Verify chip ID */
static int imx258_check_chipid(struct imx258 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret;
	u32 chip_id = 0;
	u8 reg_val = 0;

	ret = imx258_read_reg(sensor, IMX258_REG_CHIP_ID, &reg_val);
	if (ret < 0) {
			dev_err(dev, "Failed to read IMX258_REG_CHIP_ID, ret=%d\n", ret);
			return ret;
	}
	chip_id |= reg_val << 8;
	ret = imx258_read_reg(sensor, IMX258_REG_CHIP_ID + 1, &reg_val);
	if (ret < 0) {
			dev_err(dev, "Failed to read IMX258_REG_CHIP_ID, ret=%d\n", ret);
			return ret;
	}
	chip_id |= reg_val;

	if (chip_id != IMX258_CHIP_ID) {
		dev_err(dev, "chip id mismatch: %x!=%x\n",
			IMX258_CHIP_ID, chip_id);
		return -EIO;
	}

	return 0;
}


static int imx258_probe(struct i2c_client *client)
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct imx258 *sensor;

	dev_info(dev, "enter %s\n", __func__);

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(*sensor));

	sensor->i2c_client = client;

	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
						GPIOF_OUT_INIT_LOW,
						"imx258_mipi_pwdn");
		if (retval < 0) {
			dev_err(dev, "Failed to set power pin\n");
			dev_err(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
				(u32 *)&(sensor->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi_id));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	retval = imx258_retrieve_capture_properties(sensor,&sensor->ocp);
	if (retval) {
		dev_warn(dev, "retrive capture properties error\n");
	}

	retval = imx258_power_on(sensor);
	if(retval < 0)
		goto probe_err_regulator_disable;

	retval = imx258_check_chipid(sensor);
	if (retval < 0) {
		dev_err(dev, "camera imx258 is not found\n");
		retval = -ENODEV;
		goto probe_err_power_off;
	}

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx258_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &imx258_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[IMX258_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				IMX258_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
	retval = v4l2_async_register_subdev_sensor(sd);
#else
	retval = v4l2_async_register_subdev_sensor_common(sd);
#endif
	if (retval < 0) {
		dev_err(&client->dev,"%s--Async register failed, ret=%d\n",
			__func__,retval);
		goto probe_err_free_entiny;
	}

	memcpy(&sensor->cur_mode, &pimx258_mode_info[0],
			sizeof(struct vvcam_mode_info_s));

	mutex_init(&sensor->lock);
	dev_info(dev, "%s camera mipi imx258, is found\n", __func__);

	return 0;

probe_err_free_entiny:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	imx258_power_off(sensor);

probe_err_regulator_disable:

	return retval;
}

static void imx258_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *sensor = client_to_imx258(client);

	pr_info("enter %s\n", __func__);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx258_power_off(sensor);
	mutex_destroy(&sensor->lock);

	// return 0;
}

static int __maybe_unused imx258_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx258 *sensor = client_to_imx258(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status) {
		imx258_s_stream(&sensor->subdev,0);
	}

	return 0;
}

static int __maybe_unused imx258_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx258 *sensor = client_to_imx258(client);

	if (sensor->resume_status) {
		imx258_s_stream(&sensor->subdev,1);
	}

	return 0;
}

static const struct dev_pm_ops imx258_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx258_suspend, imx258_resume)
};

static const struct i2c_device_id imx258_id[] = {
	{"imx258", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx258_id);

static const struct of_device_id imx258_of_match[] = {
	{ .compatible = "sony,imx258" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx258_of_match);

static struct i2c_driver imx258_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imx258",
		.pm = &imx258_pm_ops,
		.of_match_table	= imx258_of_match,
	},
	.probe  = imx258_probe,
	.remove = imx258_remove,
	.id_table = imx258_id,
};


module_i2c_driver(imx258_i2c_driver);
MODULE_DESCRIPTION("IMX258 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
