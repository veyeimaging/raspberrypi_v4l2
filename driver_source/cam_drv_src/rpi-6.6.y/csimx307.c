// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for VEYE csimx307 cameras.
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

//#include "csimx307.h"

#define SENSOR_NAME "csimx307"

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
#define CSSC307_XCLK_FREQ		24000000

/* Pixel rate is fixed at 74.25M for all the modes */
#define CSSC307_PIXEL_RATE		74250000
/*mipi clk is 297Mhz */
#define CSSC307_DEFAULT_LINK_FREQ	297000000


#define CSSC307_XCLR_MIN_DELAY_US	6000
#define CSSC307_XCLR_DELAY_RANGE_US	1000

#define CS307_TABLE_WAIT_MS	0xFFFF
#define CS307_TABLE_END	1
#define CS307_MAX_RETRIES	3
#define CS307_WAIT_MS_STOP	1
#define CS307_WAIT_MS_START	30
#define CS307_WAIT_MS_CMD	5
#define CS307_WAIT_MS_STREAM	200

#define CS307_GAIN_TABLE_SIZE 255

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

    USER_CSI2_En = 0xD3,
    MIPI_Clk_Continous = 0xD4,
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

struct csimx307_reg {
	u16 address;
	u8 val;
};

struct csimx307_reg_list {
	u32 num_of_regs;
	const struct csimx307_reg *regs;
};

/* Mode : resolution and related config&values */
struct csimx307_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
    /* max framerate */
	u32 max_framerate;
	/* V-timing */
	//u32 vts_def;
	/* Default register values */
	struct csimx307_reg_list reg_list;
};

enum {
	CS307_MODE_1920X1080_30FPS,
    CS307_MODE_1280X720_CROP_60FPS,
    CS307_MODE_640X480_CROP_130FPS,
	//IMX307_MODE_TEST_PATTERN
};

static struct csimx307_reg csimx307_start_regs[] = {
	{CS307_TABLE_WAIT_MS, CS307_WAIT_MS_START},
	{MIPI_Clk_Continous,0x0},//discontinous mode for rpi5
    {Csi2_Enable,0x01},
	{USER_CSI2_En,0x01},
	{CS307_TABLE_WAIT_MS, CS307_WAIT_MS_STREAM},
	//{CS307_TABLE_END, 0x00 }
};

static struct csimx307_reg csimx307_stop_regs[] = {
	{CS307_TABLE_WAIT_MS, CS307_WAIT_MS_STOP},
    {Csi2_Enable,0x00},
	{USER_CSI2_En,0x00},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
	//{CS307_TABLE_END, 0x00 }
};

static struct csimx307_reg mode_1920_1080_30_regs[] = {
    {FMT_WIDTH_L,0x80},
    {FMT_WIDTH_H,0x7},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0x38},
    {FMT_HEIGHT_H,0x4},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x1E},
    {FMT_FRAMRAT_H,0x00},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_STREAM},
	//{CS307_TABLE_END, 0x00}
};

static struct csimx307_reg mode_1280_720_60_crop_regs[] = {
    {FMT_WIDTH_L,0x00},
    {FMT_WIDTH_H,0x5},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0xD0},
    {FMT_HEIGHT_H,0x2},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x3C},
    {FMT_FRAMRAT_H,0x00},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_STREAM},
	//{CS307_TABLE_END, 0x00}
};

static struct csimx307_reg mode_640_480_130_crop_regs[] = {
    {FMT_WIDTH_L,0x80},
    {FMT_WIDTH_H,0x2},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_HEIGHT_L,0xE0},
    {FMT_HEIGHT_H,0x1},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_CMD},
    {FMT_FRAMRAT_L,0x82},
    {FMT_FRAMRAT_H,0x00},
    {CS307_TABLE_WAIT_MS, CS307_WAIT_MS_STREAM},
	//{CS307_TABLE_END, 0x00}
};

#if 1
/* regulator supplies */
static const char * const csimx307_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define CSSC307_NUM_SUPPLIES ARRAY_SIZE(csimx307_supply_name)
#endif
/* Mode configs */
static const struct csimx307_mode supported_modes[] = {
	{
		/* 1080P 30fps  */
		.width = 1920,
		.height = 1080,
		//.vts_def = csimx307_VTS_30FPS_1080P,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_30_regs),
			.regs = mode_1920_1080_30_regs,
		},
	},
    {
		/* 720P 60fps  */
		.width = 1280,
		.height = 720,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280_720_60_crop_regs),
			.regs = mode_1280_720_60_crop_regs,
		},
	},
     {
		/* vga 130fps  */
		.width = 640,
		.height = 480,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640_480_130_crop_regs),
			.regs = mode_640_480_130_crop_regs,
		},
	},
};

struct csimx307 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;
	//add here
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to CSSC307 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[CSSC307_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct csimx307_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct csimx307 *to_csimx307(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct csimx307, sd);
}


static int csimx307_write_reg(struct csimx307 *csimx307, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	
    struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
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

static int csimx307_read_reg(struct csimx307 *csimx307, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);

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
static int csimx307_write_regs(struct csimx307 *csimx307,
			     const struct csimx307_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	unsigned int i;
	int ret;
    debug_printk("csimx307_write_regs len %d \n", len);
	for (i = 0; i < len; i++) {
        if(regs[i].address == CS307_TABLE_WAIT_MS){
            debug_printk("msleep %d \n", regs[i].val);
            msleep(regs[i].val);
        }
        else{
            debug_printk("csimx307_write_reg addr 0x%x val 0x%d \n", regs[i].address, regs[i].val);
            ret = csimx307_write_reg(csimx307, regs[i].address, regs[i].val);
            if (ret) {
                dev_err_ratelimited(&client->dev,
                            "Failed to write reg 0x%4.4x. error = %d\n",
                            regs[i].address, ret);
                return ret;
            }
        }
	}
	return 0;
}

static void csimx307_set_default_format(struct csimx307 *csimx307)
{
	struct v4l2_mbus_framefmt *fmt;
    VEYE_TRACE
	fmt = &csimx307->fmt;
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

static int csimx307_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
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

static int csimx307_set_ctrl(struct v4l2_ctrl *ctrl)
{
    VEYE_TRACE
    return 0;
    #if 0
	struct csimx307 *csimx307 =
		container_of(ctrl->handler, struct csimx307, ctrl_handler);
	//struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
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
		ret = csimx307_write_reg(csimx307, CSSC307_REG_ANALOG_GAIN,
				       CSSC307_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = csimx307_write_reg(csimx307, CSSC307_REG_EXPOSURE,
				       CSSC307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = csimx307_write_reg(csimx307, CSSC307_REG_DIGITAL_GAIN,
				       CSSC307_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = csimx307_write_reg(csimx307, CSSC307_REG_ORIENTATION, 1,
				       csimx307->hflip->val |
				       csimx307->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = csimx307_write_reg(csimx307, CSSC307_REG_VTS,
				       CSSC307_REG_VALUE_16BIT,
				       csimx307->mode->height + ctrl->val);
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

static const struct v4l2_ctrl_ops csimx307_ctrl_ops = {
	.s_ctrl = csimx307_set_ctrl,
};

static int csimx307_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
    VEYE_TRACE
    if (code->index > 0)
            return -EINVAL;
     code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int csimx307_enum_frame_size(struct v4l2_subdev *sd,
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

static int __csimx307_get_pad_format(struct csimx307 *csimx307,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
    //struct csimx307 *csimx307 = to_csimx307(sd);
    const struct csimx307_mode *mode = csimx307->mode;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(&csimx307->sd, sd_state, fmt->pad);
#else
                mutex_unlock(&csimx307->mutex);
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

static int csimx307_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct csimx307 *csimx307 = to_csimx307(sd);
	int ret;
    VEYE_TRACE
	mutex_lock(&csimx307->mutex);
	ret = __csimx307_get_pad_format(csimx307, sd_state, fmt);
	mutex_unlock(&csimx307->mutex);

	return ret;
}

static int csimx307_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct csimx307 *csimx307 = to_csimx307(sd);
//    struct i2c_client *client = csimx307->i2c_client;
   // struct v4l2_mbus_framefmt *__format;
    const struct csimx307_mode *new_mode;
    int ret = 0,mode,flag=0;
	const struct csimx307_reg_list *reg_list;
    
	mutex_lock(&csimx307->mutex);

	VEYE_TRACE
	
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
    debug_printk("set format  %d ,width %d ,height %d \n", mode,new_mode->width,new_mode->height);
	fmt->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->format.width = new_mode->width;
	fmt->format.height = new_mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
    csimx307->mode = new_mode;
	/* Apply default values of current mode */
	reg_list = &csimx307->mode->reg_list;
	ret = csimx307_write_regs(csimx307, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		//dev_err(&client->dev, "%s failed to set mode\n", __func__);
        VEYE_TRACE
		goto error;
	}

error:
	mutex_unlock(&csimx307->mutex);

	return ret;
}

static int csimx307_start_streaming(struct csimx307 *csimx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	const struct csimx307_reg_list *reg_list;
	int ret;
    VEYE_TRACE
	/* Apply default values of current mode */
	reg_list = &csimx307->mode->reg_list;
	ret = csimx307_write_regs(csimx307, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(csimx307->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = csimx307_write_regs(csimx307,
		csimx307_start_regs,ARRAY_SIZE(csimx307_start_regs));
	if (ret)
		return ret;
    VEYE_TRACE
	return 0;
}
 
static void csimx307_stop_streaming(struct csimx307 *csimx307)
{
	//struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	//int ret;
    VEYE_TRACE
	/* set stream off register */
	csimx307_write_regs(csimx307,
		csimx307_stop_regs,ARRAY_SIZE(csimx307_stop_regs));
	return ;
}

static int csimx307_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csimx307 *csimx307 = to_csimx307(sd);
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
    debug_printk("start streaming %d\n", enable );
	mutex_lock(&csimx307->mutex);
	if (csimx307->streaming == enable) {
		mutex_unlock(&csimx307->mutex);
		return 0;
	}
	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = csimx307_start_streaming(csimx307);
		if (ret)
			goto err_unlock;
	} else {
		csimx307_stop_streaming(csimx307);
	}
	csimx307->streaming = enable;
	mutex_unlock(&csimx307->mutex);

	return ret;
err_unlock:
	mutex_unlock(&csimx307->mutex);

	return ret;
}

/* Power/clock management functions */
static int csimx307_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct csimx307 *csimx307 = to_csimx307(sd);
	int ret;
    debug_printk("csimx307_power_on power on \n" );
	ret = regulator_bulk_enable(CSSC307_NUM_SUPPLIES,
				    csimx307->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
    //veye do not need clk
	/*ret = clk_prepare_enable(csimx307->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}*/

	gpiod_set_value_cansleep(csimx307->reset_gpio, 1);
	usleep_range(CSSC307_XCLR_MIN_DELAY_US,
		     CSSC307_XCLR_MIN_DELAY_US + CSSC307_XCLR_DELAY_RANGE_US);

	return 0;

/*reg_off:
	regulator_bulk_disable(CSSC307_NUM_SUPPLIES, csimx307->supplies);

	return ret;*/
}

static int csimx307_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct csimx307 *csimx307 = to_csimx307(sd);
    debug_printk("csimx307_power_off power off \n" );
	gpiod_set_value_cansleep(csimx307->reset_gpio, 0);
	regulator_bulk_disable(CSSC307_NUM_SUPPLIES, csimx307->supplies);
	clk_disable_unprepare(csimx307->xclk);

	return 0;
}

static int __maybe_unused csimx307_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct csimx307 *csimx307 = to_csimx307(sd);

	if (csimx307->streaming)
		csimx307_stop_streaming(csimx307);

	return 0;
}

static int __maybe_unused csimx307_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct csimx307 *csimx307 = to_csimx307(sd);
	int ret;

	if (csimx307->streaming) {
		ret = csimx307_start_streaming(csimx307);
		if (ret)
			goto error;
	}

	return 0;

error:
	csimx307_stop_streaming(csimx307);
	csimx307->streaming = 0;

	return ret;
}

static int csimx307_get_regulators(struct csimx307 *csimx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	unsigned int i;

	for (i = 0; i < CSSC307_NUM_SUPPLIES; i++)
		csimx307->supplies[i].supply = csimx307_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       CSSC307_NUM_SUPPLIES,
				       csimx307->supplies);
}

/* Verify chip ID */
static int csimx307_identify_module(struct csimx307 *csimx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	int err;
    u8 reg_val[2];
    u16 cameraid = 0;
    VEYE_TRACE
	/* Probe sensor model id registers */
	err = csimx307_read_reg(csimx307, PRODUCTID_L, &reg_val[0]);
	if (err) {
		dev_err(&client->dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
     err = csimx307_read_reg(csimx307, PRODUCTID_H, &reg_val[1]);
    if (err) {
		dev_err(&client->dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
    cameraid = ((u16)reg_val[1]<<8) + reg_val[0];
	dev_err(&client->dev,"read sensor id %04x \n", cameraid);
	if (cameraid == CS_MIPI_IMX307) 
    {
        err = 0;
        dev_err(&client->dev, " camera id is cs-mipi-imx307\n");
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

/*static const struct v4l2_subdev_core_ops csimx307_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};*/

static const struct v4l2_subdev_video_ops csimx307_video_ops = {
	.s_stream = csimx307_set_stream,
};

static const struct v4l2_subdev_pad_ops csimx307_pad_ops = {
	.enum_mbus_code = csimx307_enum_mbus_code,
	.get_fmt = csimx307_get_pad_format,
	.set_fmt = csimx307_set_pad_format,
    //?
	//.get_selection = csimx307_get_selection,
	.enum_frame_size = csimx307_enum_frame_size,
};

static const struct v4l2_subdev_ops csimx307_subdev_ops = {
	//.core = &csimx307_core_ops,
	.video = &csimx307_video_ops,
	.pad = &csimx307_pad_ops,
};

static const struct v4l2_subdev_internal_ops csimx307_internal_ops = {
	.open = csimx307_open,
};

/* Initialize control handlers */
static int csimx307_init_controls(struct csimx307 *csimx307)
{
	struct i2c_client *client = v4l2_get_subdevdata(&csimx307->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	//unsigned int height = csimx307->mode->height;
	//struct v4l2_fwnode_device_properties props;
	//int exposure_max, exposure_def, hblank;
	int  ret;
    VEYE_TRACE
	ctrl_hdlr = &csimx307->ctrl_handler;
    //v4l2 number
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 1);
	if (ret)
		return ret;

	mutex_init(&csimx307->mutex);
	ctrl_hdlr->lock = &csimx307->mutex;

	/* By default, PIXEL_RATE is read only */
	csimx307->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       CSSC307_PIXEL_RATE,
					       CSSC307_PIXEL_RATE, 1,
					       CSSC307_PIXEL_RATE);
#if 0
	/* Initial vblank/hblank/exposure parameters based on current mode */
	csimx307->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					   V4L2_CID_VBLANK, CSSC307_VBLANK_MIN,
					   CSSC307_VTS_MAX - height, 1,
					   csimx307->mode->vts_def - height);
	hblank = CSSC307_PPL_DEFAULT - csimx307->mode->width;
	csimx307->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (csimx307->hblank)
		csimx307->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = csimx307->mode->vts_def - 4;
	exposure_def = (exposure_max < CSSC307_EXPOSURE_DEFAULT) ?
		exposure_max : CSSC307_EXPOSURE_DEFAULT;
	csimx307->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     CSSC307_EXPOSURE_MIN, exposure_max,
					     CSSC307_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  CSSC307_ANA_GAIN_MIN, CSSC307_ANA_GAIN_MAX,
			  CSSC307_ANA_GAIN_STEP, CSSC307_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  CSSC307_DGTL_GAIN_MIN, CSSC307_DGTL_GAIN_MAX,
			  CSSC307_DGTL_GAIN_STEP, CSSC307_DGTL_GAIN_DEFAULT);

	csimx307->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (csimx307->hflip)
		csimx307->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	csimx307->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (csimx307->vflip)
		csimx307->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &csimx307_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(csimx307_test_pattern_menu) - 1,
				     0, 0, csimx307_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &csimx307_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  CSSC307_TESTP_COLOUR_MIN,
				  CSSC307_TESTP_COLOUR_MAX,
				  CSSC307_TESTP_COLOUR_STEP,
				  CSSC307_TESTP_COLOUR_MAX);
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

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &csimx307_ctrl_ops,
					      &props);
	if (ret)
		goto error;*/

	csimx307->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&csimx307->mutex);

	return ret;
}

static void csimx307_free_controls(struct csimx307 *csimx307)
{
	v4l2_ctrl_handler_free(csimx307->sd.ctrl_handler);
	mutex_destroy(&csimx307->mutex);
}

static int csimx307_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
    
	int ret = -EINVAL;
    VEYE_TRACE
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
	    ep_cfg.link_frequencies[0] != CSSC307_DEFAULT_LINK_FREQ) {
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

static int csimx307_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct csimx307 *csimx307;
	int ret;

	csimx307 = devm_kzalloc(&client->dev, sizeof(*csimx307), GFP_KERNEL);
	if (!csimx307)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&csimx307->sd, client, &csimx307_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (csimx307_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
/*	csimx307->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(csimx307->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(csimx307->xclk);
	}

	csimx307->xclk_freq = clk_get_rate(csimx307->xclk);
	if (csimx307->xclk_freq != CSSC307_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			csimx307->xclk_freq);
		return -EINVAL;
	}
*/
	ret = csimx307_get_regulators(csimx307);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	csimx307->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for csimx307_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = csimx307_power_on(dev);
	if (ret)
		return ret;

    usleep_range(100, 110);
    
    ret = csimx307_identify_module(csimx307);
	if (ret)
		goto error_power_off;
    
	/* Set default mode to max resolution */
	csimx307->mode = &supported_modes[0];

    ret = csimx307_init_controls(csimx307);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	csimx307->sd.internal_ops = &csimx307_internal_ops;
	csimx307->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	csimx307->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	/* Initialize source pad */
	csimx307->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	csimx307_set_default_format(csimx307);
    // stop stream here
	csimx307_stop_streaming(csimx307);
	csimx307->streaming = 0;
	
	ret = media_entity_pads_init(&csimx307->sd.entity, 1, &csimx307->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&csimx307->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	//pm_runtime_set_active(&client->dev);
	//pm_runtime_enable(&client->dev);
	//pm_runtime_idle(&client->dev);
    //debug_printk("csimx307 camera probed\n");
    dev_err(&client->dev, "camera csimx307_mipi is found\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&csimx307->sd.entity);

error_handler_free:
	csimx307_free_controls(csimx307);

error_power_off:
	csimx307_power_off(dev);

	return ret;
}

static void csimx307_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct csimx307 *csimx307 = to_csimx307(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	csimx307_free_controls(csimx307);

	//pm_runtime_disable(&client->dev);
	//pm_runtime_set_suspended(&client->dev);

	
}

static const struct of_device_id csimx307_dt_ids[] = {
	{ .compatible = "veye,csimx307" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, csimx307_dt_ids);

static struct i2c_driver csimx307_i2c_driver = {
	.driver = {
		.name = "csimx307",
		.of_match_table	= csimx307_dt_ids,
	},
	.probe = csimx307_probe,
	.remove = csimx307_remove,
};

module_i2c_driver(csimx307_i2c_driver);

MODULE_AUTHOR("xumm <www.veye.cc>");
MODULE_DESCRIPTION("csimx307 sensor v4l2 driver");
MODULE_LICENSE("GPL v2");
