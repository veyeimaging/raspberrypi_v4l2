// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for VEYE veye327 cameras.
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

//#include "veye327.h"

#define SENSOR_NAME "veye327"

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
#define VEYE327_XCLK_FREQ		24000000

/* Pixel rate is fixed at 74.25M for all the modes */
#define VEYE327_PIXEL_RATE		74250000
/*mipi clk is 297Mhz */
#define VEYE327_DEFAULT_LINK_FREQ	297000000


#define VEYE327_XCLR_MIN_DELAY_US	6000
#define VEYE327_XCLR_DELAY_RANGE_US	1000

/* veye327 model register address */
#define VEYE327_MODEL_ID_ADDR		0x0001
#define VEYE327_DEVICE_ID 		0x06

#define SENSOR_TYPR_ADDR_L    0x20
#define SENSOR_TYPR_ADDR_H    0x21

#define BOARD_TYPR_ADDR    0x25

/* registers */
#define VEYECAM_STREAMING_ON    0x001D
#define VEYECAM_MODE_STANDBY		0x00
#define VEYECAM_MODE_STREAMING		0x01

//static int debug = 0;

struct veye327_reg {
	u16 address;
	u8 val;
};

struct veye327_reg_list {
	u32 num_of_regs;
	const struct veye327_reg *regs;
};

/* Mode : resolution and related config&values */
struct veye327_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
    /* max framerate */
	u32 max_framerate;
	/* V-timing */
	//u32 vts_def;
	/* Default register values */
	struct veye327_reg_list reg_list;
};

/*enum veye327_mode_id {
	VEYE327_MODE_1080P_1920_1080,
	VEYE327_NUM_MODES,
};*/

static const struct veye327_reg mode_1920_1080_regs[] = {

};
#if 1
/* regulator supplies */
static const char * const veye327_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define VEYE327_NUM_SUPPLIES ARRAY_SIZE(veye327_supply_name)
#endif
/* Mode configs */
static const struct veye327_mode supported_modes[] = {
	{
		/* 1080P 30fps  */
		.width = 1920,
		.height = 1080,
		//.vts_def = veye327_VTS_30FPS_1080P,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_regs),
			.regs = mode_1920_1080_regs,
		},
	},
};

struct veye327 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;
	//add here
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to VEYE327 */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[VEYE327_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct veye327_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct veye327 *to_veye327(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct veye327, sd);
}


static int veye327_write_reg(struct veye327 *veye327, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	
    struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
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

static int veye327_read_reg(struct veye327 *veye327, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);

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
static int veye327_write_regs(struct veye327 *veye327,
			     const struct veye327_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = veye327_write_reg(veye327, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);
			return ret;
		}
	}
	return 0;
}

static void veye327_set_default_format(struct veye327 *veye327)
{
	struct v4l2_mbus_framefmt *fmt;
    VEYE_TRACE
	fmt = &veye327->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
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

static int veye327_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
    VEYE_TRACE
	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	try_fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int veye327_set_ctrl(struct v4l2_ctrl *ctrl)
{
    VEYE_TRACE
    return 0;
    #if 0
	struct veye327 *veye327 =
		container_of(ctrl->handler, struct veye327, ctrl_handler);
	//struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
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
		ret = veye327_write_reg(veye327, VEYE327_REG_ANALOG_GAIN,
				       VEYE327_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = veye327_write_reg(veye327, VEYE327_REG_EXPOSURE,
				       VEYE327_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = veye327_write_reg(veye327, VEYE327_REG_DIGITAL_GAIN,
				       VEYE327_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = veye327_write_reg(veye327, VEYE327_REG_ORIENTATION, 1,
				       veye327->hflip->val |
				       veye327->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = veye327_write_reg(veye327, VEYE327_REG_VTS,
				       VEYE327_REG_VALUE_16BIT,
				       veye327->mode->height + ctrl->val);
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

static const struct v4l2_ctrl_ops veye327_ctrl_ops = {
	.s_ctrl = veye327_set_ctrl,
};

static int veye327_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
    VEYE_TRACE
    if (code->index > 0)
            return -EINVAL;
     code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

static int veye327_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
    VEYE_TRACE
	if (fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;
    
	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = supported_modes[fse->index].height;

	return 0;
}

static int __veye327_get_pad_format(struct veye327 *veye327,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *fmt)
{
    //struct veye327 *veye327 = to_veye327(sd);
    const struct veye327_mode *mode = veye327->mode;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(&veye327->sd, cfg, fmt->pad);
#else
                mutex_unlock(&veye327->mutex);
                return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
        fmt->format.height = mode->height;
        fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	return 0;
}

static int veye327_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct veye327 *veye327 = to_veye327(sd);
	int ret;
    VEYE_TRACE
	mutex_lock(&veye327->mutex);
	ret = __veye327_get_pad_format(veye327, cfg, fmt);
	mutex_unlock(&veye327->mutex);

	return ret;
}

static int veye327_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct veye327 *veye327 = to_veye327(sd);
//    struct i2c_client *client = veye327->i2c_client;
   // struct v4l2_mbus_framefmt *__format;
    const struct veye327_mode *new_mode;
    int ret = 0,mode,flag=0;
	const struct veye327_reg_list *reg_list;
    
	mutex_lock(&veye327->mutex);

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

	fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->format.width = new_mode->width;
	fmt->format.height = new_mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
    veye327->mode = new_mode;
	/* Apply default values of current mode */
	reg_list = &veye327->mode->reg_list;
	ret = veye327_write_regs(veye327, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		//dev_err(&client->dev, "%s failed to set mode\n", __func__);
        VEYE_TRACE
		goto error;
	}

error:
	mutex_unlock(&veye327->mutex);

	return ret;
}

static int veye327_start_streaming(struct veye327 *veye327)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	const struct veye327_reg_list *reg_list;
	int ret;
    VEYE_TRACE
	/* Apply default values of current mode */
	reg_list = &veye327->mode->reg_list;
	ret = veye327_write_regs(veye327, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(veye327->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return veye327_write_reg(veye327, VEYECAM_STREAMING_ON, VEYECAM_MODE_STREAMING);
}

static void veye327_stop_streaming(struct veye327 *veye327)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	int ret;
    VEYE_TRACE
	/* set stream off register */
	ret = veye327_write_reg(veye327, VEYECAM_STREAMING_ON, VEYECAM_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int veye327_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct veye327 *veye327 = to_veye327(sd);
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
    debug_printk("start streaming %d\n", enable );
	mutex_lock(&veye327->mutex);
	if (veye327->streaming == enable) {
		mutex_unlock(&veye327->mutex);
		return 0;
	}
	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = veye327_start_streaming(veye327);
		if (ret)
			goto err_unlock;
	} else {
		veye327_stop_streaming(veye327);
	}
	veye327->streaming = enable;
	mutex_unlock(&veye327->mutex);

	return ret;
err_unlock:
	mutex_unlock(&veye327->mutex);

	return ret;
}

/* Power/clock management functions */
static int veye327_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);
	int ret;
    debug_printk("veye327_power_on power on \n" );
	ret = regulator_bulk_enable(VEYE327_NUM_SUPPLIES,
				    veye327->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
    //veye do not need clk
	/*ret = clk_prepare_enable(veye327->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}*/

	gpiod_set_value_cansleep(veye327->reset_gpio, 1);
	usleep_range(VEYE327_XCLR_MIN_DELAY_US,
		     VEYE327_XCLR_MIN_DELAY_US + VEYE327_XCLR_DELAY_RANGE_US);

	return 0;

/*reg_off:
	regulator_bulk_disable(VEYE327_NUM_SUPPLIES, veye327->supplies);

	return ret;*/
}

static int veye327_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);
    debug_printk("veye327_power_off power off \n" );
	gpiod_set_value_cansleep(veye327->reset_gpio, 0);
	regulator_bulk_disable(VEYE327_NUM_SUPPLIES, veye327->supplies);
	clk_disable_unprepare(veye327->xclk);

	return 0;
}

static int __maybe_unused veye327_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);

	if (veye327->streaming)
		veye327_stop_streaming(veye327);

	return 0;
}

static int __maybe_unused veye327_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);
	int ret;

	if (veye327->streaming) {
		ret = veye327_start_streaming(veye327);
		if (ret)
			goto error;
	}

	return 0;

error:
	veye327_stop_streaming(veye327);
	veye327->streaming = 0;

	return ret;
}

static int veye327_get_regulators(struct veye327 *veye327)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	unsigned int i;

	for (i = 0; i < VEYE327_NUM_SUPPLIES; i++)
		veye327->supplies[i].supply = veye327_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       VEYE327_NUM_SUPPLIES,
				       veye327->supplies);
}

static int veye327_read_model(struct veye327 *veye327)
{
    struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	int ret;
    u8 snr_l;
    u8 snr_h;
    u8 board_no;
    ret = veye327_read_reg(veye327, SENSOR_TYPR_ADDR_L, &snr_l);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    ret = veye327_read_reg(veye327, SENSOR_TYPR_ADDR_H, &snr_h);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    ret = veye327_read_reg(veye327, BOARD_TYPR_ADDR, &board_no);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    if(snr_l == 0x03 && snr_h == 0x27){
        dev_err(&client->dev, "sensor is IMX327\n");
    }
    else if(snr_l == 0x04 && snr_h == 0x62){
        dev_err(&client->dev, "sensor is IMX462\n");
    }
    else if(snr_l == 0x03 && snr_h == 0x85){
        dev_err(&client->dev, "sensor is IMX385\n");
    }
     if(board_no == 0x4C){
        dev_err(&client->dev, "board type is ONE board\n");
    }else{
        dev_err(&client->dev, "board type is TWO board\n");
    }
    return 0;
}

/* Verify chip ID */
static int veye327_identify_module(struct veye327 *veye327)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	int ret;
	//u32 val;
    int err;
    u8 device_id;
    VEYE_TRACE
	ret = veye327_read_reg(veye327, VEYE327_MODEL_ID_ADDR, &device_id);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    if (device_id == VEYE327_DEVICE_ID) 
    {
        err = 0;
        dev_err(&client->dev, " camera id is veye327\n");
    }
    else
    {
        err = -ENODEV;
		dev_err(&client->dev, "%s: invalid sensor model id: %d\n",
			__func__, device_id);
    }
    veye327_read_model(veye327);
	return err;
}

/*static const struct v4l2_subdev_core_ops veye327_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};*/

static const struct v4l2_subdev_video_ops veye327_video_ops = {
	.s_stream = veye327_set_stream,
};

static const struct v4l2_subdev_pad_ops veye327_pad_ops = {
	.enum_mbus_code = veye327_enum_mbus_code,
	.get_fmt = veye327_get_pad_format,
	.set_fmt = veye327_set_pad_format,
    //?
	//.get_selection = veye327_get_selection,
	.enum_frame_size = veye327_enum_frame_size,
};

static const struct v4l2_subdev_ops veye327_subdev_ops = {
	//.core = &veye327_core_ops,
	.video = &veye327_video_ops,
	.pad = &veye327_pad_ops,
};

static const struct v4l2_subdev_internal_ops veye327_internal_ops = {
	.open = veye327_open,
};

/* Initialize control handlers */
static int veye327_init_controls(struct veye327 *veye327)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veye327->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	//unsigned int height = veye327->mode->height;
	//struct v4l2_fwnode_device_properties props;
	//int exposure_max, exposure_def, hblank;
	int  ret;
    VEYE_TRACE
	ctrl_hdlr = &veye327->ctrl_handler;
    //v4l2 number
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 1);
	if (ret)
		return ret;

	mutex_init(&veye327->mutex);
	ctrl_hdlr->lock = &veye327->mutex;

	/* By default, PIXEL_RATE is read only */
	veye327->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       VEYE327_PIXEL_RATE,
					       VEYE327_PIXEL_RATE, 1,
					       VEYE327_PIXEL_RATE);
#if 0
	/* Initial vblank/hblank/exposure parameters based on current mode */
	veye327->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					   V4L2_CID_VBLANK, VEYE327_VBLANK_MIN,
					   VEYE327_VTS_MAX - height, 1,
					   veye327->mode->vts_def - height);
	hblank = VEYE327_PPL_DEFAULT - veye327->mode->width;
	veye327->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (veye327->hblank)
		veye327->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = veye327->mode->vts_def - 4;
	exposure_def = (exposure_max < VEYE327_EXPOSURE_DEFAULT) ?
		exposure_max : VEYE327_EXPOSURE_DEFAULT;
	veye327->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     VEYE327_EXPOSURE_MIN, exposure_max,
					     VEYE327_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  VEYE327_ANA_GAIN_MIN, VEYE327_ANA_GAIN_MAX,
			  VEYE327_ANA_GAIN_STEP, VEYE327_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  VEYE327_DGTL_GAIN_MIN, VEYE327_DGTL_GAIN_MAX,
			  VEYE327_DGTL_GAIN_STEP, VEYE327_DGTL_GAIN_DEFAULT);

	veye327->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (veye327->hflip)
		veye327->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	veye327->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (veye327->vflip)
		veye327->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &veye327_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(veye327_test_pattern_menu) - 1,
				     0, 0, veye327_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &veye327_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  VEYE327_TESTP_COLOUR_MIN,
				  VEYE327_TESTP_COLOUR_MAX,
				  VEYE327_TESTP_COLOUR_STEP,
				  VEYE327_TESTP_COLOUR_MAX);
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

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &veye327_ctrl_ops,
					      &props);
	if (ret)
		goto error;*/

	veye327->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&veye327->mutex);

	return ret;
}

static void veye327_free_controls(struct veye327 *veye327)
{
	v4l2_ctrl_handler_free(veye327->sd.ctrl_handler);
	mutex_destroy(&veye327->mutex);
}

static int veye327_check_hwcfg(struct device *dev)
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
	    ep_cfg.link_frequencies[0] != VEYE327_DEFAULT_LINK_FREQ) {
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

static int veye327_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct veye327 *veye327;
	int ret;

	veye327 = devm_kzalloc(&client->dev, sizeof(*veye327), GFP_KERNEL);
	if (!veye327)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&veye327->sd, client, &veye327_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (veye327_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
/*	veye327->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(veye327->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(veye327->xclk);
	}

	veye327->xclk_freq = clk_get_rate(veye327->xclk);
	if (veye327->xclk_freq != VEYE327_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			veye327->xclk_freq);
		return -EINVAL;
	}
*/
	ret = veye327_get_regulators(veye327);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	veye327->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for veye327_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = veye327_power_on(dev);
	if (ret)
		return ret;

    //usleep_range(100, 110);
    msleep(100);
    
    ret = veye327_identify_module(veye327);
	if (ret)
		goto error_power_off;
    
	/* Set default mode to max resolution */
	veye327->mode = &supported_modes[0];
    //clk discontinues mode
    veye327_write_reg(veye327,0x000b, 0xfe);
    
    ret = veye327_init_controls(veye327);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	veye327->sd.internal_ops = &veye327_internal_ops;
	veye327->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	veye327->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	/* Initialize source pad */
	veye327->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	veye327_set_default_format(veye327);
    
	ret = media_entity_pads_init(&veye327->sd.entity, 1, &veye327->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor_common(&veye327->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	//pm_runtime_set_active(&client->dev);
	//pm_runtime_enable(&client->dev);
	//pm_runtime_idle(&client->dev);
    //debug_printk("veye327 camera probed\n");
    dev_err(&client->dev, "veye327 camera probed\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&veye327->sd.entity);

error_handler_free:
	veye327_free_controls(veye327);

error_power_off:
	veye327_power_off(dev);

	return ret;
}

static int veye327_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veye327 *veye327 = to_veye327(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	veye327_free_controls(veye327);

	//pm_runtime_disable(&client->dev);
	//pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id veye327_dt_ids[] = {
	{ .compatible = "veye,veye327" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, veye327_dt_ids);

static struct i2c_driver veye327_i2c_driver = {
	.driver = {
		.name = "veye327",
		.of_match_table	= veye327_dt_ids,
	},
	.probe_new = veye327_probe,
	.remove = veye327_remove,
};

module_i2c_driver(veye327_i2c_driver);

MODULE_AUTHOR("xumm <www.veye.cc");
MODULE_DESCRIPTION("veye327 sensor v4l2 driver");
MODULE_LICENSE("GPL v2");
