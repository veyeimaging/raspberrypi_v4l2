// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for VEYE veyecam2m cameras.
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
// VEYE-MIPI-IMX327S
// VEYE-MIPI-IMX462
// VEYE-MIPI-IMX385

//#include "veyecam2m.h"

#define SENSOR_NAME "veyecam2m"

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
#define VEYECAM2M_XCLK_FREQ		24000000

/* Pixel rate is fixed at 74.25M for all the modes */
#define VEYECAM2M_PIXEL_RATE		74250000
/*mipi clk is 297Mhz */
#define VEYECAM2M_DEFAULT_LINK_FREQ	297000000


#define VEYECAM2M_XCLR_MIN_DELAY_US	6000
#define VEYECAM2M_XCLR_DELAY_RANGE_US	1000

/* veyecam2m model register address */
#define VEYECAM2M_MODEL_ID_ADDR		0x0001
#define VEYECAM2M_DEVICE_ID 		0x06

#define SENSOR_TYPR_ADDR_L    0x20
#define SENSOR_TYPR_ADDR_H    0x21

#define BOARD_TYPR_ADDR    0x25

/* registers */
#define VEYECAM_STREAMING_ON    0x001D
#define VEYECAM_MODE_STANDBY		0x00
#define VEYECAM_MODE_STREAMING		0x01

//static int debug = 0;

struct veyecam2m_reg {
	u16 address;
	u8 val;
};

struct veyecam2m_reg_list {
	u32 num_of_regs;
	const struct veyecam2m_reg *regs;
};

/* Mode : resolution and related config&values */
struct veyecam2m_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
    /* max framerate */
	u32 max_framerate;
	/* V-timing */
	//u32 vts_def;
	/* Default register values */
	struct veyecam2m_reg_list reg_list;
};

/*enum veyecam2m_mode_id {
	VEYECAM2M_MODE_1080P_1920_1080,
	VEYECAM2M_NUM_MODES,
};*/

static const struct veyecam2m_reg mode_1920_1080_regs[] = {

};
#if 1
/* regulator supplies */
static const char * const veyecam2m_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define VEYECAM2M_NUM_SUPPLIES ARRAY_SIZE(veyecam2m_supply_name)
#endif
/* Mode configs */
static const struct veyecam2m_mode supported_modes[] = {
	{
		/* 1080P 30fps  */
		.width = 1920,
		.height = 1080,
		//.vts_def = veyecam2m_VTS_30FPS_1080P,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920_1080_regs),
			.regs = mode_1920_1080_regs,
		},
	},
};

struct veyecam2m {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;
	//add here
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to VEYECAM2M */
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[VEYECAM2M_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct veyecam2m_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct veyecam2m *to_veyecam2m(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct veyecam2m, sd);
}


static int veyecam2m_write_reg(struct veyecam2m *veyecam2m, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	
    struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
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

static int veyecam2m_read_reg(struct veyecam2m *veyecam2m, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);

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
static int veyecam2m_write_regs(struct veyecam2m *veyecam2m,
			     const struct veyecam2m_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = veyecam2m_write_reg(veyecam2m, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);
			return ret;
		}
	}
	return 0;
}

static void veyecam2m_set_default_format(struct veyecam2m *veyecam2m)
{
	struct v4l2_mbus_framefmt *fmt;
    VEYE_TRACE
	fmt = &veyecam2m->fmt;
	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->colorspace = V4L2_COLORSPACE_REC709;
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

static int veyecam2m_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->state, 0);
    VEYE_TRACE
	/* Initialize try_fmt */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	try_fmt->field = V4L2_FIELD_NONE;
	try_fmt->colorspace = V4L2_COLORSPACE_REC709;

	return 0;
}

static int veyecam2m_set_ctrl(struct v4l2_ctrl *ctrl)
{
    VEYE_TRACE
    return 0;
    #if 0
	struct veyecam2m *veyecam2m =
		container_of(ctrl->handler, struct veyecam2m, ctrl_handler);
	//struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
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
		ret = veyecam2m_write_reg(veyecam2m, VEYECAM2M_REG_ANALOG_GAIN,
				       VEYECAM2M_REG_VALUE_08BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = veyecam2m_write_reg(veyecam2m, VEYECAM2M_REG_EXPOSURE,
				       VEYECAM2M_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = veyecam2m_write_reg(veyecam2m, VEYECAM2M_REG_DIGITAL_GAIN,
				       VEYECAM2M_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = veyecam2m_write_reg(veyecam2m, VEYECAM2M_REG_ORIENTATION, 1,
				       veyecam2m->hflip->val |
				       veyecam2m->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = veyecam2m_write_reg(veyecam2m, VEYECAM2M_REG_VTS,
				       VEYECAM2M_REG_VALUE_16BIT,
				       veyecam2m->mode->height + ctrl->val);
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

static const struct v4l2_ctrl_ops veyecam2m_ctrl_ops = {
	.s_ctrl = veyecam2m_set_ctrl,
};

static int veyecam2m_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
    VEYE_TRACE
    if (code->index > 0)
            return -EINVAL;
     code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	return 0;
}

static int veyecam2m_enum_frame_size(struct v4l2_subdev *sd,
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

static int __veyecam2m_get_pad_format(struct veyecam2m *veyecam2m,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
    //struct veyecam2m *veyecam2m = to_veyecam2m(sd);
    const struct veyecam2m_mode *mode = veyecam2m->mode;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
                fmt->format = *v4l2_subdev_get_try_format(&veyecam2m->sd, sd_state, fmt->pad);
#else
                mutex_unlock(&veyecam2m->mutex);
                return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
        fmt->format.height = mode->height;
        fmt->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->format.colorspace = V4L2_COLORSPACE_REC709;
	}
	return 0;
}

static int veyecam2m_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
	int ret;
    VEYE_TRACE
	mutex_lock(&veyecam2m->mutex);
	ret = __veyecam2m_get_pad_format(veyecam2m, sd_state, fmt);
	mutex_unlock(&veyecam2m->mutex);

	return ret;
}

static int veyecam2m_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
//    struct i2c_client *client = veyecam2m->i2c_client;
   // struct v4l2_mbus_framefmt *__format;
    const struct veyecam2m_mode *new_mode;
    int ret = 0,mode,flag=0;
	const struct veyecam2m_reg_list *reg_list;
    
	mutex_lock(&veyecam2m->mutex);

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
	fmt->format.colorspace = V4L2_COLORSPACE_REC709;
    veyecam2m->mode = new_mode;
	/* Apply default values of current mode */
	reg_list = &veyecam2m->mode->reg_list;
	ret = veyecam2m_write_regs(veyecam2m, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		//dev_err(&client->dev, "%s failed to set mode\n", __func__);
        VEYE_TRACE
		goto error;
	}

error:
	mutex_unlock(&veyecam2m->mutex);

	return ret;
}

static int veyecam2m_start_streaming(struct veyecam2m *veyecam2m)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	const struct veyecam2m_reg_list *reg_list;
	int ret;
    VEYE_TRACE
	/* Apply default values of current mode */
	reg_list = &veyecam2m->mode->reg_list;
	ret = veyecam2m_write_regs(veyecam2m, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(veyecam2m->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return veyecam2m_write_reg(veyecam2m, VEYECAM_STREAMING_ON, VEYECAM_MODE_STREAMING);
}

static void veyecam2m_stop_streaming(struct veyecam2m *veyecam2m)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	int ret;
    VEYE_TRACE
	/* set stream off register */
	ret = veyecam2m_write_reg(veyecam2m, VEYECAM_STREAMING_ON, VEYECAM_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int veyecam2m_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
	//struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
    debug_printk("start streaming %d\n", enable );
	mutex_lock(&veyecam2m->mutex);
	if (veyecam2m->streaming == enable) {
		mutex_unlock(&veyecam2m->mutex);
		return 0;
	}
	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = veyecam2m_start_streaming(veyecam2m);
		if (ret)
			goto err_unlock;
	} else {
		veyecam2m_stop_streaming(veyecam2m);
	}
	veyecam2m->streaming = enable;
	mutex_unlock(&veyecam2m->mutex);

	return ret;
err_unlock:
	mutex_unlock(&veyecam2m->mutex);

	return ret;
}

/* Power/clock management functions */
static int veyecam2m_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
	int ret;
    debug_printk("veyecam2m_power_on power on \n" );
	ret = regulator_bulk_enable(VEYECAM2M_NUM_SUPPLIES,
				    veyecam2m->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
    //veye do not need clk
	/*ret = clk_prepare_enable(veyecam2m->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}*/

	gpiod_set_value_cansleep(veyecam2m->reset_gpio, 1);
	usleep_range(VEYECAM2M_XCLR_MIN_DELAY_US,
		     VEYECAM2M_XCLR_MIN_DELAY_US + VEYECAM2M_XCLR_DELAY_RANGE_US);

	return 0;

/*reg_off:
	regulator_bulk_disable(VEYECAM2M_NUM_SUPPLIES, veyecam2m->supplies);

	return ret;*/
}

static int veyecam2m_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
    debug_printk("veyecam2m_power_off power off \n" );
	gpiod_set_value_cansleep(veyecam2m->reset_gpio, 0);
	regulator_bulk_disable(VEYECAM2M_NUM_SUPPLIES, veyecam2m->supplies);
	clk_disable_unprepare(veyecam2m->xclk);

	return 0;
}

static int __maybe_unused veyecam2m_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);

	if (veyecam2m->streaming)
		veyecam2m_stop_streaming(veyecam2m);

	return 0;
}

static int __maybe_unused veyecam2m_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);
	int ret;

	if (veyecam2m->streaming) {
		ret = veyecam2m_start_streaming(veyecam2m);
		if (ret)
			goto error;
	}

	return 0;

error:
	veyecam2m_stop_streaming(veyecam2m);
	veyecam2m->streaming = 0;

	return ret;
}

static int veyecam2m_get_regulators(struct veyecam2m *veyecam2m)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	unsigned int i;

	for (i = 0; i < VEYECAM2M_NUM_SUPPLIES; i++)
		veyecam2m->supplies[i].supply = veyecam2m_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       VEYECAM2M_NUM_SUPPLIES,
				       veyecam2m->supplies);
}

static int veyecam2m_read_model(struct veyecam2m *veyecam2m)
{
    struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	int ret;
    u8 snr_l;
    u8 snr_h;
    u8 board_no;
    ret = veyecam2m_read_reg(veyecam2m, SENSOR_TYPR_ADDR_L, &snr_l);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    ret = veyecam2m_read_reg(veyecam2m, SENSOR_TYPR_ADDR_H, &snr_h);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    ret = veyecam2m_read_reg(veyecam2m, BOARD_TYPR_ADDR, &board_no);
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
static int veyecam2m_identify_module(struct veyecam2m *veyecam2m)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	int ret;
	//u32 val;
    int err;
    u8 device_id;
    VEYE_TRACE
	ret = veyecam2m_read_reg(veyecam2m, VEYECAM2M_MODEL_ID_ADDR, &device_id);
	if (ret) {
		dev_err(&client->dev, "probe failed \n");
		return -ENODEV;
	}
    if (device_id == VEYECAM2M_DEVICE_ID) 
    {
        err = 0;
        dev_err(&client->dev, " camera id is veyecam2m\n");
    }
    else
    {
        err = -ENODEV;
		dev_err(&client->dev, "%s: invalid sensor model id: %d\n",
			__func__, device_id);
    }
    veyecam2m_read_model(veyecam2m);
	return err;
}

/*static const struct v4l2_subdev_core_ops veyecam2m_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};*/

static const struct v4l2_subdev_video_ops veyecam2m_video_ops = {
	.s_stream = veyecam2m_set_stream,
};

static const struct v4l2_subdev_pad_ops veyecam2m_pad_ops = {
	.enum_mbus_code = veyecam2m_enum_mbus_code,
	.get_fmt = veyecam2m_get_pad_format,
	.set_fmt = veyecam2m_set_pad_format,
    //?
	//.get_selection = veyecam2m_get_selection,
	.enum_frame_size = veyecam2m_enum_frame_size,
};

static const struct v4l2_subdev_ops veyecam2m_subdev_ops = {
	//.core = &veyecam2m_core_ops,
	.video = &veyecam2m_video_ops,
	.pad = &veyecam2m_pad_ops,
};

static const struct v4l2_subdev_internal_ops veyecam2m_internal_ops = {
	.open = veyecam2m_open,
};

/* Initialize control handlers */
static int veyecam2m_init_controls(struct veyecam2m *veyecam2m)
{
	struct i2c_client *client = v4l2_get_subdevdata(&veyecam2m->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	//unsigned int height = veyecam2m->mode->height;
	//struct v4l2_fwnode_device_properties props;
	//int exposure_max, exposure_def, hblank;
	int  ret;
    VEYE_TRACE
	ctrl_hdlr = &veyecam2m->ctrl_handler;
    //v4l2 number
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 1);
	if (ret)
		return ret;

	mutex_init(&veyecam2m->mutex);
	ctrl_hdlr->lock = &veyecam2m->mutex;

	/* By default, PIXEL_RATE is read only */
	veyecam2m->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       VEYECAM2M_PIXEL_RATE,
					       VEYECAM2M_PIXEL_RATE, 1,
					       VEYECAM2M_PIXEL_RATE);
#if 0
	/* Initial vblank/hblank/exposure parameters based on current mode */
	veyecam2m->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					   V4L2_CID_VBLANK, VEYECAM2M_VBLANK_MIN,
					   VEYECAM2M_VTS_MAX - height, 1,
					   veyecam2m->mode->vts_def - height);
	hblank = VEYECAM2M_PPL_DEFAULT - veyecam2m->mode->width;
	veyecam2m->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);
	if (veyecam2m->hblank)
		veyecam2m->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	exposure_max = veyecam2m->mode->vts_def - 4;
	exposure_def = (exposure_max < VEYECAM2M_EXPOSURE_DEFAULT) ?
		exposure_max : VEYECAM2M_EXPOSURE_DEFAULT;
	veyecam2m->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     VEYECAM2M_EXPOSURE_MIN, exposure_max,
					     VEYECAM2M_EXPOSURE_STEP,
					     exposure_def);

	v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  VEYECAM2M_ANA_GAIN_MIN, VEYECAM2M_ANA_GAIN_MAX,
			  VEYECAM2M_ANA_GAIN_STEP, VEYECAM2M_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  VEYECAM2M_DGTL_GAIN_MIN, VEYECAM2M_DGTL_GAIN_MAX,
			  VEYECAM2M_DGTL_GAIN_STEP, VEYECAM2M_DGTL_GAIN_DEFAULT);

	veyecam2m->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (veyecam2m->hflip)
		veyecam2m->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	veyecam2m->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (veyecam2m->vflip)
		veyecam2m->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &veyecam2m_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(veyecam2m_test_pattern_menu) - 1,
				     0, 0, veyecam2m_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &veyecam2m_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  VEYECAM2M_TESTP_COLOUR_MIN,
				  VEYECAM2M_TESTP_COLOUR_MAX,
				  VEYECAM2M_TESTP_COLOUR_STEP,
				  VEYECAM2M_TESTP_COLOUR_MAX);
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

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &veyecam2m_ctrl_ops,
					      &props);
	if (ret)
		goto error;*/

	veyecam2m->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&veyecam2m->mutex);

	return ret;
}

static void veyecam2m_free_controls(struct veyecam2m *veyecam2m)
{
	v4l2_ctrl_handler_free(veyecam2m->sd.ctrl_handler);
	mutex_destroy(&veyecam2m->mutex);
}

static int veyecam2m_check_hwcfg(struct device *dev)
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
	    ep_cfg.link_frequencies[0] != VEYECAM2M_DEFAULT_LINK_FREQ) {
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

static int veyecam2m_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct veyecam2m *veyecam2m;
	int ret;
    //try waiting for fdplink
    if (request_module("ds90ub954") != 0) {
        dev_err(dev, "Unable to load ds90ub954 driver,will go on\n");
        //return -ENODEV;
    }
	veyecam2m = devm_kzalloc(&client->dev, sizeof(*veyecam2m), GFP_KERNEL);
	if (!veyecam2m)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&veyecam2m->sd, client, &veyecam2m_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (veyecam2m_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
/*	veyecam2m->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(veyecam2m->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(veyecam2m->xclk);
	}

	veyecam2m->xclk_freq = clk_get_rate(veyecam2m->xclk);
	if (veyecam2m->xclk_freq != VEYECAM2M_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			veyecam2m->xclk_freq);
		return -EINVAL;
	}
*/
	ret = veyecam2m_get_regulators(veyecam2m);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	veyecam2m->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for veyecam2m_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = veyecam2m_power_on(dev);
	if (ret)
		return ret;

    //usleep_range(100, 110);
    msleep(100);
    
    ret = veyecam2m_identify_module(veyecam2m);
	if (ret)
		goto error_power_off;
    
	/* Set default mode to max resolution */
	veyecam2m->mode = &supported_modes[0];
    //clk discontinues mode
    veyecam2m_write_reg(veyecam2m,0x000b, 0xfe);
    
    ret = veyecam2m_init_controls(veyecam2m);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	veyecam2m->sd.internal_ops = &veyecam2m_internal_ops;
	veyecam2m->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	veyecam2m->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	/* Initialize source pad */
	veyecam2m->pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize default format */
	veyecam2m_set_default_format(veyecam2m);
    
	ret = media_entity_pads_init(&veyecam2m->sd.entity, 1, &veyecam2m->pad);
	if (ret)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&veyecam2m->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	//pm_runtime_set_active(&client->dev);
	//pm_runtime_enable(&client->dev);
	//pm_runtime_idle(&client->dev);
    //debug_printk("veyecam2m camera probed\n");
    dev_err(&client->dev, "veyecam2m camera probed\n");
	return 0;

error_media_entity:
	media_entity_cleanup(&veyecam2m->sd.entity);

error_handler_free:
	veyecam2m_free_controls(veyecam2m);

error_power_off:
	veyecam2m_power_off(dev);

	return ret;
}

static void veyecam2m_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct veyecam2m *veyecam2m = to_veyecam2m(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	veyecam2m_free_controls(veyecam2m);

	//pm_runtime_disable(&client->dev);
	//pm_runtime_set_suspended(&client->dev);

	
}

static const struct of_device_id veyecam2m_dt_ids[] = {
	{ .compatible = "veye,veyecam2m" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, veyecam2m_dt_ids);

static struct i2c_driver veyecam2m_i2c_driver = {
	.driver = {
		.name = "veyecam2m",
		.of_match_table	= veyecam2m_dt_ids,
	},
	.probe = veyecam2m_probe,
	.remove = veyecam2m_remove,
};

module_i2c_driver(veyecam2m_i2c_driver);

MODULE_AUTHOR("xumm <www.veye.cc>");
MODULE_DESCRIPTION("veyecam2m sensor v4l2 driver");
MODULE_LICENSE("GPL v2");
