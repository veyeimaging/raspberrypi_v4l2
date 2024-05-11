// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for VEYE cssc132 cameras.
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx258 camera driver
 * Copyright (C) 2018 Intel Corporation
 *
 * DT / fwnode changes, and regulator / GPIO control taken from ov5640.c
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 * Copyright (C) 2020-2021 Tianjin Zhonganyijia Tech. All Rights Reserved.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

//#include "cssc132.h"

#define SENSOR_NAME "cssc132"

//#define DEBUG_PRINTK
#ifndef DEBUG_PRINTK
#define debug_printk(s , ... )
#define VEYE_TRACE 
#else
#define debug_printk printk
#define VEYE_TRACE printk("%s %s %d \n",__FILE__,__FUNCTION__,__LINE__);
#endif

/* External clock frequency is 24.0M */
// we do not need it
#define CSSC132_XCLK_FREQ		24000000

/* Pixel rate is fixed at 74.25M for all the modes */
#define CSSC132_PIXEL_RATE		74250000
/*mipi clk is 297Mhz */
#define CSSC132_DEFAULT_LINK_FREQ	297000000


#define CSSC132_XCLR_MIN_DELAY_US	6000
#define CSSC132_XCLR_DELAY_RANGE_US	1000

#define CSSC132_TABLE_WAIT_MS	0xFFFF
#define CSSC132_TABLE_END	1
#define CSSC132_MAX_RETRIES	3
#define CSSC132_WAIT_MS_STOP	1
#define CSSC132_WAIT_MS_START	30
#define CSSC132_WAIT_MS_CMD	5
#define CSSC132_WAIT_MS_STREAM	200

#define CSSC132_GAIN_TABLE_SIZE 255

typedef enum 
{
	CS_MIPI_IMX307 = 0x0037,	
	CS_LVDS_IMX307 = 0x0038,	
	CS_USB_IMX307 = 0x0039,	
	CS_MIPI_GS132 = 0x0132,	
	CS_LVDS_GS132 = 0x0133,	
	CS_USB_GS132 = 0x0134
}ENProductID;

typedef enum
{
    deviceID = 0x00,
    HardWare = 0x01,
    LoadingDone = 0x02,

    Csi2_Enable = 0x03,
    Fpga_CAP_L = 0x04,
    Fpga_CAP_H = 0x05,
    
    TriggerMode = 0x10,
    SlaveMode = 0x11,
    TrigDly_H = 0x12,
    TrigDly_M = 0x13,
    TrigDly_U = 0x14,
    TrigDly_L = 0x15,
    VerTotalTime_H = 0x16,
    VerTotalTime_L = 0x17,
    HorTotalTime_H = 0x18,
    HorTotalTime_L = 0x19,
    
    //for arm part
    ARM_VER_L = 0x0100,
    ARM_VER_H = 0x0101,
    PRODUCTID_L = 0x0102,
    PRODUCTID_H = 0x0103,
    SYSTEM_RESET = 0x0104,
    PARAM_SAVE = 0x0105,
    VIDEOFMT_CAP = 0x0106, 
    
    VIDEOFMT_NUM = 0x0107,

    FMTCAP_WIDTH_L = 0x0108,
    FMTCAP_WIDTH_H = 0x0109,
    FMTCAP_HEIGHT_L = 0x010A,
    FMTCAP_HEIGHT_H = 0x010B,
    FMTCAP_FRAMRAT_L = 0x010C,
    FMTCAP_FRAMRAT_H = 0x010D,
    
    FMT_WIDTH_L = 0x0180,
    FMT_WIDTH_H = 0x0181,
    FMT_HEIGHT_L = 0x0182,
    FMT_HEIGHT_H = 0x0183,
    FMT_FRAMRAT_L = 0x0184,
    FMT_FRAMRAT_H = 0x0185,
   
    //ISP cap
    ISP_CAP_L = 0x0200,
    ISP_CAP_M = 0x0201,
    ISP_CAP_H = 0x0202,
    ISP_CAP_E = 0x0203,
    POWER_HZ = 0x0204,
}ECAMERA_REG;

struct cssc132_reg {
	u16 address;
	u8 val;
};

struct cssc132_reg_list {
	u32 num_of_regs;
	const struct cssc132_reg *regs;
};

/* Mode : resolution and related config&values */
struct cssc132_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
    /* max framerate */
	u32 max_framerate;
	/* V-timing */
	//u32 vts_def;
	/* Default register values */
	struct cssc132_reg_list reg_list;
};

enum {
	SC132_MODE_1280X1080_45FPS,
    SC132_MODE_1080X1280_45FPS,
    SC132_MODE_1280X720_CROP_60FPS,
    SC132_MODE_720X1280_CROP_60FPS,
    SC132_MODE_640X480_CROP_120FPS,
    SC132_MODE_480X640_CROP_120FPS,
	//IMX307_MODE_TEST_PATTERN
};

static struct cssc132_reg cssc132_start_regs[] = {
	{CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_START},
    {Csi2_Enable,0x01},
	{CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00 }
};

static struct cssc132_reg cssc132_stop_regs[] = {
	{CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STOP},
    {Csi2_Enable,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
	//{CSSC132_TABLE_END, 0x00 }
};

static struct cssc132_reg sc132_reg_1280x1080_45fps[] = {
    {FMT_WIDTH_L,0x00},
    {FMT_WIDTH_H,0x5},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0x38},
    {FMT_HEIGHT_H,0x4},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x2D},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};

static struct cssc132_reg sc132_reg_1080x1280_45fps[] = {
    {FMT_WIDTH_L,0x38},
    {FMT_WIDTH_H,0x4},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0x00},
    {FMT_HEIGHT_H,0x5},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x2D},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};

static struct cssc132_reg sc132_reg_1280x720_crop_60fps[] = {
    {FMT_WIDTH_L,0x00},
    {FMT_WIDTH_H,0x5},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0xD0},
    {FMT_HEIGHT_H,0x2},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x3C},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};

static struct cssc132_reg sc132_reg_720x1280_crop_60fps[] = {
    {FMT_WIDTH_L,0xD0},
    {FMT_WIDTH_H,0x2},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0x00},
    {FMT_HEIGHT_H,0x5},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x3C},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};

static struct cssc132_reg sc132_reg_640x480_crop_120fps[] = {
    {FMT_WIDTH_L,0x80},
    {FMT_WIDTH_H,0x2},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0xE0},
    {FMT_HEIGHT_H,0x1},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x78},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};
static struct cssc132_reg sc132_reg_480x640_crop_120fps[] = {
    {FMT_WIDTH_L,0xE0},
    {FMT_WIDTH_H,0x1},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0x80},
    {FMT_HEIGHT_H,0x2},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x78},
    {FMT_FRAMRAT_H,0x00},
    {CSSC132_TABLE_WAIT_MS, CSSC132_WAIT_MS_STREAM},
	//{CSSC132_TABLE_END, 0x00}
};
#if 1
/* regulator supplies */
static const char * const cssc132_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define CSSC132_NUM_SUPPLIES ARRAY_SIZE(cssc132_supply_name)
#endif

/* Mode configs */
static const struct cssc132_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 1080,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_1280x1080_45fps),
			.regs = sc132_reg_1280x1080_45fps,
		},
	},
    {
		.width = 1080,
		.height = 1280,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_1080x1280_45fps),
			.regs = sc132_reg_1080x1280_45fps,
		},
	},
    {
		.width = 1280,
		.height = 720,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_1280x720_crop_60fps),
			.regs = sc132_reg_1280x720_crop_60fps,
		},
	},
    {
		.width = 720,
		.height = 1280,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_720x1280_crop_60fps),
			.regs = sc132_reg_720x1280_crop_60fps,
		},
	},
    {
		.width = 640,
		.height = 480,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_640x480_crop_120fps),
			.regs = sc132_reg_640x480_crop_120fps,
		},
	},
    {
		.width = 480,
		.height = 640,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(sc132_reg_480x640_crop_120fps),
			.regs = sc132_reg_480x640_crop_120fps,
		},
	},
};

struct cssc132 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;
	//add here
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to CSSC132 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[CSSC132_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct cssc132_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct cssc132 *to_cssc132(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct cssc132, sd);
}


static int cssc132_write_reg(struct cssc132 *cssc132, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	
    struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	ret = i2c_master_send(client, data, 3);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int cssc132_read_reg(struct cssc132 *cssc132, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2) {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	/*
	 * The only return value indicating success is 1. Anything else, even
	 * a non-negative value, indicates something went wrong.
	 */
	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/* Write a list of registers */
static int cssc132_write_regs(struct cssc132 *cssc132,
			     const struct cssc132_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
        if(regs[i].address == CSSC132_TABLE_WAIT_MS)
            msleep(regs[i].val);
        else
            ret = cssc132_write_reg(cssc132, regs[i].address, regs[i].val);
		
        if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);
			return ret;
		}
	}
	return 0;
}

static void cssc132_set_default_format(struct cssc132 *cssc132)
{
	struct v4l2_mbus_framefmt *fmt;
    VEYE_TRACE
	fmt = &cssc132->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
/*	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
*/
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int cssc132_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
    VEYE_TRACE
	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	try_fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int cssc132_set_ctrl(struct v4l2_ctrl *ctrl)
{
    VEYE_TRACE
    return 0;
    #if 0
	struct cssc132 *cssc132 =
		container_of(ctrl->handler, struct cssc132, ctrl_handler);
	//struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	int ret = 0;

    if ((ctrl->id == V4L2_CID_PIXEL_RATE) || (ctrl->id == V4L2_CID_LINK_FREQ)){
		ret = 0;
    }

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */

/*	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cssc132_write_reg(cssc132, CSSC132_REG_ANALOG_GAIN,
				       CSSC132_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cssc132_write_reg(cssc132, CSSC132_REG_EXPOSURE,
				       CSSC132_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cssc132_write_reg(cssc132, CSSC132_REG_DIGITAL_GAIN,
				       CSSC132_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cssc132_write_reg(cssc132, CSSC132_REG_ORIENTATION, 1,
				       cssc132->hflip->val |
				       cssc132->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = cssc132_write_reg(cssc132, CSSC132_REG_VTS,
				       CSSC132_REG_VALUE_16BIT,
				       cssc132->mode->height + ctrl->val);
		break;

	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}
*/
	//pm_runtime_put(&client->dev);

	return ret;
    #endif
}

static const struct v4l2_ctrl_ops cssc132_ctrl_ops = {
	.s_ctrl = cssc132_set_ctrl,
};

static int cssc132_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
    VEYE_TRACE
    if (code->index > 0)
            return -EINVAL;
     code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int cssc132_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
    VEYE_TRACE
	if (fse->code != MEDIA_BUS_FMT_UYVY8_1X16)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;
    
	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].height;

	return 0;
}

static int __cssc132_get_pad_format(struct cssc132 *cssc132,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
    //struct cssc132 *cssc132 = to_cssc132(sd);
    const struct cssc132_mode *mode = cssc132->mode;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(&cssc132->sd, sd_state, fmt->pad);
#else
                mutex_unlock(&cssc132->mutex);
                return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
        fmt->format.height = mode->height;
        fmt->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	return 0;
}

static int cssc132_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct cssc132 *cssc132 = to_cssc132(sd);
	int ret;
    VEYE_TRACE
	mutex_lock(&cssc132->mutex);
	ret = __cssc132_get_pad_format(cssc132, sd_state, fmt);
	mutex_unlock(&cssc132->mutex);

	return ret;
}

static int cssc132_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct cssc132 *cssc132 = to_cssc132(sd);
//    struct i2c_client *client = cssc132->i2c_client;
   // struct v4l2_mbus_framefmt *__format;
    const struct cssc132_mode *new_mode;
    int ret = 0,mode,flag=0;
	const struct cssc132_reg_list *reg_list;
    
	mutex_lock(&cssc132->mutex);

	//debug_printk(" %s\n",__func__);
	
        for(mode=0;mode<ARRAY_SIZE(supported_modes);mode++) {
           if((fmt->format.width==supported_modes[mode].width)&&
                   (fmt->format.height==supported_modes[mode].height)){
                     new_mode = &supported_modes[mode];
                     flag=1;
                     break;
            }
         }
         if(flag==0){
           ret = -EINVAL;
           goto error;
         }

	fmt->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->format.width = new_mode->width;
	fmt->format.height = new_mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
    cssc132->mode = new_mode;
	/* Apply default values of current mode */
	reg_list = &cssc132->mode->reg_list;
	ret = cssc132_write_regs(cssc132, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		//dev_err(&client->dev, "%s failed to set mode\n", __func__);
        VEYE_TRACE
		goto error;
	}

error:
	mutex_unlock(&cssc132->mutex);

	return ret;
}

static int cssc132_start_streaming(struct cssc132 *cssc132)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	const struct cssc132_reg_list *reg_list;
	int ret;
    VEYE_TRACE
	/* Apply default values of current mode */
	reg_list = &cssc132->mode->reg_list;
	ret = cssc132_write_regs(cssc132, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(cssc132->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = cssc132_write_regs(cssc132,
		cssc132_start_regs,ARRAY_SIZE(cssc132_start_regs));
	if (ret)
		return ret;
    VEYE_TRACE
	return 0;
}

static void cssc132_stop_streaming(struct cssc132 *cssc132)
{
    VEYE_TRACE
	/* set stream off register */
	 cssc132_write_regs(cssc132,
		cssc132_stop_regs,ARRAY_SIZE(cssc132_stop_regs));
}

static int cssc132_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct cssc132 *cssc132 = to_cssc132(sd);
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
    debug_printk("start streaming %d\n", enable );
	mutex_lock(&cssc132->mutex);
	if (cssc132->streaming == enable) {
		mutex_unlock(&cssc132->mutex);
		return 0;
	}
	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = cssc132_start_streaming(cssc132);
		if (ret)
			goto err_unlock;
	} else {
		cssc132_stop_streaming(cssc132);
	}
	cssc132->streaming = enable;
	mutex_unlock(&cssc132->mutex);

	return ret;
err_unlock:
	mutex_unlock(&cssc132->mutex);

	return ret;
}

/* Power/clock management functions */
static int cssc132_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cssc132 *cssc132 = to_cssc132(sd);
	int ret;
    debug_printk("cssc132_power_on power on \n" );
	ret = regulator_bulk_enable(CSSC132_NUM_SUPPLIES,
				    cssc132->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
    //veye do not need clk
	/*ret = clk_prepare_enable(cssc132->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}*/

	gpiod_set_value_cansleep(cssc132->reset_gpio, 1);
	usleep_range(CSSC132_XCLR_MIN_DELAY_US,
		     CSSC132_XCLR_MIN_DELAY_US + CSSC132_XCLR_DELAY_RANGE_US);

	return 0;

/*reg_off:
	regulator_bulk_disable(CSSC132_NUM_SUPPLIES, cssc132->supplies);

	return ret;*/
}

static int cssc132_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cssc132 *cssc132 = to_cssc132(sd);
    debug_printk("cssc132_power_off power off \n" );
	gpiod_set_value_cansleep(cssc132->reset_gpio, 0);
	regulator_bulk_disable(CSSC132_NUM_SUPPLIES, cssc132->supplies);
	clk_disable_unprepare(cssc132->xclk);

	return 0;
}

static int __maybe_unused cssc132_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cssc132 *cssc132 = to_cssc132(sd);

	if (cssc132->streaming)
		cssc132_stop_streaming(cssc132);

	return 0;
}

static int __maybe_unused cssc132_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cssc132 *cssc132 = to_cssc132(sd);
	int ret;

	if (cssc132->streaming) {
		ret = cssc132_start_streaming(cssc132);
		if (ret)
			goto error;
	}

	return 0;

error:
	cssc132_stop_streaming(cssc132);
	cssc132->streaming = 0;

	return ret;
}

static int cssc132_get_regulators(struct cssc132 *cssc132)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	unsigned int i;

	for (i = 0; i < CSSC132_NUM_SUPPLIES; i++)
		cssc132->supplies[i].supply = cssc132_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       CSSC132_NUM_SUPPLIES,
				       cssc132->supplies);
}

/* Verify chip ID */
static int cssc132_identify_module(struct cssc132 *cssc132)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	int err;
    u8 reg_val[2];
    u16 cameraid = 0;
    VEYE_TRACE
	/* Probe sensor model id registers */
	err = cssc132_read_reg(cssc132, PRODUCTID_L, &reg_val[0]);
	if (err) {
		dev_err(&client->dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
     err = cssc132_read_reg(cssc132, PRODUCTID_H, &reg_val[1]);
    if (err) {
		dev_err(&client->dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
    cameraid = ((u16)reg_val[1]<<8) + reg_val[0];
	dev_err(&client->dev,"read sensor id %04x \n", cameraid);
	if (cameraid == CS_MIPI_GS132) 
    {
        err = 0;
        dev_err(&client->dev, " camera id is cs-mipi-sc132\n");
    }
    else
    {
        err = -ENODEV;
		dev_err(&client->dev, "%s: invalid sensor model id: %d\n",
			__func__, cameraid);
    }
err_reg_probe:

	return err;
}

/*static const struct v4l2_subdev_core_ops cssc132_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};*/

static const struct v4l2_subdev_video_ops cssc132_video_ops = {
	.s_stream = cssc132_set_stream,
};

static const struct v4l2_subdev_pad_ops cssc132_pad_ops = {
	.enum_mbus_code = cssc132_enum_mbus_code,
	.get_fmt = cssc132_get_pad_format,
	.set_fmt = cssc132_set_pad_format,
    //?
	//.get_selection = cssc132_get_selection,
	.enum_frame_size = cssc132_enum_frame_size,
};

static const struct v4l2_subdev_ops cssc132_subdev_ops = {
	//.core = &cssc132_core_ops,
	.video = &cssc132_video_ops,
	.pad = &cssc132_pad_ops,
};

static const struct v4l2_subdev_internal_ops cssc132_internal_ops = {
	.open = cssc132_open,
};

/* Initialize control handlers */
static int cssc132_init_controls(struct cssc132 *cssc132)
{
	struct i2c_client *client = v4l2_get_subdevdata(&cssc132->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	//unsigned int height = cssc132->mode->height;
	//struct v4l2_fwnode_device_properties props;
	//int exposure_max, exposure_def, hblank;
	int  ret;
    VEYE_TRACE
	ctrl_hdlr = &cssc132->ctrl_handler;
    //v4l2 number
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 1);
	if (ret)
		return ret;

	mutex_init(&cssc132->mutex);
	ctrl_hdlr->lock = &cssc132->mutex;

	/* By default, PIXEL_RATE is read only */
	cssc132->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       CSSC132_PIXEL_RATE,
					       CSSC132_PIXEL_RATE, 1,
					       CSSC132_PIXEL_RATE);
#if 0
	/* Initial vblank/hblank/exposure parameters based on current mode */
	cssc132->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					   V4L2_CID_VBLANK, CSSC132_VBLANK_MIN,
					   CSSC132_VTS_MAX - height, 1,
					   cssc132->mode->vts_def - height);
	hblank = CSSC132_PPL_DEFAULT - cssc132->mode->width;
	cssc132->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (cssc132->hblank)
		cssc132->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = cssc132->mode->vts_def - 4;
	exposure_def = (exposure_max < CSSC132_EXPOSURE_DEFAULT) ?
		exposure_max : CSSC132_EXPOSURE_DEFAULT;
	cssc132->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     CSSC132_EXPOSURE_MIN, exposure_max,
					     CSSC132_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  CSSC132_ANA_GAIN_MIN, CSSC132_ANA_GAIN_MAX,
			  CSSC132_ANA_GAIN_STEP, CSSC132_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  CSSC132_DGTL_GAIN_MIN, CSSC132_DGTL_GAIN_MAX,
			  CSSC132_DGTL_GAIN_STEP, CSSC132_DGTL_GAIN_DEFAULT);

	cssc132->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (cssc132->hflip)
		cssc132->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	cssc132->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (cssc132->vflip)
		cssc132->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &cssc132_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(cssc132_test_pattern_menu) - 1,
				     0, 0, cssc132_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &cssc132_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  CSSC132_TESTP_COLOUR_MIN,
				  CSSC132_TESTP_COLOUR_MAX,
				  CSSC132_TESTP_COLOUR_STEP,
				  CSSC132_TESTP_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}
#endif
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	/*ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &cssc132_ctrl_ops,
					      &props);
	if (ret)
		goto error;*/

	cssc132->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&cssc132->mutex);

	return ret;
}

static void cssc132_free_controls(struct cssc132 *cssc132)
{
	v4l2_ctrl_handler_free(cssc132->sd.ctrl_handler);
	mutex_destroy(&cssc132->mutex);
}

static int cssc132_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
    VEYE_TRACE
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != CSSC132_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int cssc132_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cssc132 *cssc132;
	int ret;

	cssc132 = devm_kzalloc(&client->dev, sizeof(*cssc132), GFP_KERNEL);
	if (!cssc132)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&cssc132->sd, client, &cssc132_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (cssc132_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
/*	cssc132->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(cssc132->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(cssc132->xclk);
	}

	cssc132->xclk_freq = clk_get_rate(cssc132->xclk);
	if (cssc132->xclk_freq != CSSC132_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			cssc132->xclk_freq);
		return -EINVAL;
	}
*/
	ret = cssc132_get_regulators(cssc132);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	cssc132->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for cssc132_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = cssc132_power_on(dev);
	if (ret)
		return ret;

    usleep_range(100, 110);
    
    ret = cssc132_identify_module(cssc132);
	if (ret)
		goto error_power_off;
    
	/* Set default mode to max resolution */
	cssc132->mode = &supported_modes[0];

    ret = cssc132_init_controls(cssc132);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	cssc132->sd.internal_ops = &cssc132_internal_ops;
	cssc132->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	cssc132->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	/* Initialize source pad */
	cssc132->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	cssc132_set_default_format(cssc132);
    
	ret = media_entity_pads_init(&cssc132->sd.entity, 1, &cssc132->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&cssc132->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	//pm_runtime_set_active(&client->dev);
	//pm_runtime_enable(&client->dev);
	//pm_runtime_idle(&client->dev);
   // debug_printk("cssc132 camera probed\n");
   dev_err(&client->dev, "camera cssc132_mipi is found\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&cssc132->sd.entity);

error_handler_free:
	cssc132_free_controls(cssc132);

error_power_off:
	cssc132_power_off(dev);

	return ret;
}

static void cssc132_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cssc132 *cssc132 = to_cssc132(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	cssc132_free_controls(cssc132);

	//pm_runtime_disable(&client->dev);
	//pm_runtime_set_suspended(&client->dev);

	
}

static const struct of_device_id cssc132_dt_ids[] = {
	{ .compatible = "veye,cssc132" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cssc132_dt_ids);

static struct i2c_driver cssc132_i2c_driver = {
	.driver = {
		.name = "cssc132",
		.of_match_table	= cssc132_dt_ids,
	},
	.probe = cssc132_probe,
	.remove = cssc132_remove,
};

module_i2c_driver(cssc132_i2c_driver);

MODULE_AUTHOR("xumm <www.veye.cc>");
MODULE_DESCRIPTION("cssc132 sensor v4l2 driver");
MODULE_LICENSE("GPL v2");
