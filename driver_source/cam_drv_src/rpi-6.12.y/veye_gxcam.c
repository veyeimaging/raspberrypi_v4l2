// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, www.veye.cc
 */
#include "veye_gxcam.h"
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-event.h>
#include <linux/unaligned.h>

/*
	v1.0.2:
	rename trigger_mode-->work_mode.
	v1.0.1:
	support video mode registers.
	v1.0.0:
	first release version
*/

#define DRIVER_VERSION			KERNEL_VERSION(1, 0x00, 0x02) 

#define gxcam_NAME			"gxcam"

/* Embedded metadata stream structure */
#define VEYE_GX_EMBEDDED_LINE_WIDTH 16384
#define VEYE_GX_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
//	METADATA_PAD,//reserved
	NUM_PADS
};

//#define DEBUG_PRINTK
#ifndef DEBUG_PRINTK
static int debug = 0;
#define debug_printk(s , ... )
#define VEYE_TRACE 
#else
static int debug = 1;
#define debug_printk printk
#define VEYE_TRACE printk("%s %s %d \n",__FILE__,__FUNCTION__,__LINE__);
#endif

module_param(debug, int, 0644);

#define STARTUP_MIN_DELAY_US	50*1000//50ms
#define STARTUP_DELAY_RANGE_US	1000

#define WRITE_HEAD 0xAB
#define PRE_READ_HEAD 0xBC
#define POST_READ_HEAD 0xAC
#define DIR_READ_HEAD 0xDE

#pragma pack(push, 1)  
struct preread_regs {
	uint8_t	pre_head;
	uint16_t reg;
	uint8_t xor;
};
struct write_regs {
	uint8_t	head;
	uint16_t reg;
	uint32_t data;
	uint8_t xor;
};
#pragma pack(pop) 

struct  reg_gx {
	u16 addr;
	u32 val;
};

struct gxcam_reg_list {
	unsigned int num_of_regs;
	const struct  reg_gx *regs;
};

struct gxcam_format {
	u32 index;
	u32 mbus_code;//mbus format
	u32 data_type;//mv data format
};
#define GXCAM_MAX_VIDEO_MODE_NUM 8
struct gxcam_imgmode {
    u32 width;
    u32 height;
    u32 maxfps;
    enum enum_ReadOutMode readoutmode;
};

static s64 link_freq_menu_items[] = {
	GXCAM_DEFAULT_LINK_FREQ,
};
/* regulator supplies */
static const char * const gxcam_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define GXCAM_NUM_SUPPLIES ARRAY_SIZE(gxcam_supply_name)

//used for gxcam->ctrls[i] index, this must obey the same order as gxcam_v4l2_ctrls 
enum enum_v4l2_ctrls_index{
    CID_LINK_FREQ,
    CID_PIXEL_RATE,
	CID_VEYE_GX_WORK_MODE,
	CID_VEYE_GX_TRIGGER_SRC,
	CID_VEYE_GX_SOFT_TRGONE,
	CID_VEYE_GX_SYNC_ROLE,
	CID_VEYE_GX_FRAME_RATE,
	GXCAM_MAX_CTRLS,
};

struct gxcam {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];
	struct kobject kobj;

    u32    model_id; 
	struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[GXCAM_NUM_SUPPLIES];
    
    struct i2c_client *client;
    //data format 
	struct gxcam_format *supported_formats;
	int num_supported_formats;
	int current_format_idx;

	//image mode
	struct gxcam_imgmode *imgmode_list;
	int num_imgmodes;
	int current_imgmode_idx;
    u32 current_width;
	u32 current_height;

    u32 cur_fps;
	//image mode
	struct v4l2_rect crop;
    u32 h_flip;
    u32 v_flip;
    
    u32 lane_num;
	u32 lanecap;
    u32 mipi_datarate;
	u8 camera_model[32];
	u64 pixelrate;
	
	struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *ctrls[GXCAM_MAX_CTRLS];
	/* V4L2 Controls */
   //struct v4l2_ctrl *frmrate;
    
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static inline struct gxcam *to_gxcam(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct gxcam, sd);
}

static uint8_t xor8(const uint8_t *data, uint32_t len) 
{
    uint8_t checksum = 0;
    const uint8_t *p = data;
    while (len--) 
    {
        checksum ^= *p++;
    }
    return checksum;
}

//direct register access
static int gxcam_readl_d_reg(struct i2c_client *client,
								   u16 addr, u32 *val)
{
	uint8_t checksum;
	uint8_t buf[3] = {DIR_READ_HEAD,  addr >> 8, addr & 0xff };
    uint8_t bufout[8] = {0};
	struct i2c_msg msgs[2] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = 3,
			 .buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD ,
			.len = 6,
			.buf = bufout,
		},
	};

	if (i2c_transfer(client->adapter, msgs,2) != 2) {
		debug_printk( "%s: Failed to read register 0x%02x\n",
			 __func__, addr);
		return -EIO;
	}

	checksum = xor8(bufout, 5);
	if (checksum != bufout[5]) {
		v4l2_err(client, "%s: Read register 0x%02x checksum error\n",
			 __func__, addr);
		return -EIO;
	}
	*val = ntohl(*(uint32_t*)bufout);
	//v4l2_dbg(1, debug, client, "%s: 0x%02x 0x%04x\n",
	//		 __func__, addr, *val);
	
	return 0;
}
//common register access
static int gxcam_readl_reg(struct i2c_client *client,
								   u16 addr, u32 *val)
{
	struct preread_regs regs;
	uint8_t checksum = 0;
	uint8_t buf[3] = {POST_READ_HEAD,  addr >> 8, addr & 0xff };
    uint8_t bufout[8] = {0};

	struct i2c_msg  msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(regs),
		.buf = (u8 *)&regs,
	};
	struct i2c_msg msgs[2] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = 3,
			 .buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD ,
			.len = 6,
			.buf = bufout,
		},
	};

	regs.pre_head = PRE_READ_HEAD;
	regs.reg = htons(addr);
	regs.xor = xor8((uint8_t *)&regs, 3);

	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		v4l2_err(client, "%s: Failed to write register 0x%02x\n",
			 __func__, addr);
		return -EIO;
	}
	usleep_range(20000, 25000); //20ms

	if (i2c_transfer(client->adapter, msgs,2) != 2) {
		return -EIO;
	}
	checksum = xor8(bufout, 5);
	if (checksum != bufout[5]) {
		v4l2_err(client, "%s: Read register 0x%02x checksum error\n",
			 __func__, addr);
		return -EIO;
	}
	*val = ntohl(*(uint32_t*)bufout);
	debug_printk( "gxcam_readl_reg: 0x%02x 0x%04x\n", addr, *val);
	return 0;
}
				
static int gxcam_writel_reg(struct i2c_client *client,
									u16 addr, u32 val)
{
	struct write_regs wr = {
		.head = WRITE_HEAD,
		.reg = htons(addr),
		.data = htonl(val),
		.xor = xor8((uint8_t *)&wr, sizeof(wr) - 1),
	};
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(wr),
		.buf = (u8 *)&wr,
	};

	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		return -EIO;
	}
	
	return 0;
}

static int gxcam_read(struct i2c_client *client, u16 addr, u32 *value)
{
	int ret;
	int count = 0;
	while (count++ < I2C_READ_RETRY_COUNT) {
		ret = gxcam_readl_reg(client, addr, value);
		if(!ret) {
			//v4l2_dbg(1, debug, client, "%s: 0x%02x 0x%04x\n",
			//	__func__, addr, *value);
			return ret;
		}
	}
    
	v4l2_err(client, "%s: Reading register 0x%02x failed\n",
			 __func__, addr);
	return ret;
}
//read direct registers
static int gxcam_read_d(struct i2c_client *client, u16 addr, u32 *value)
{
	int ret;
	int count = 0;
	while (count++ < I2C_READ_RETRY_COUNT) {
		ret = gxcam_readl_d_reg(client, addr, value);
		if(!ret) {
			//v4l2_dbg(1, debug, client, "%s: 0x%02x 0x%04x\n",
			//	__func__, addr, *value);
			return ret;
		}
	}
    
	v4l2_err(client, "%s: Reading register 0x%02x failed\n",
			 __func__, addr);
	return ret;
}

static int gxcam_write(struct i2c_client *client, u16 addr, u32 value)
{
	int ret;
	int count = 0;
	while (count++ < I2C_WRITE_RETRY_COUNT) {
		ret = gxcam_writel_reg(client, addr, value);
		if(!ret)
			return ret;
	}
	v4l2_err(client, "%s: Write 0x%04x to register 0x%02x failed\n",
			 __func__, value, addr);
	return ret;
}

/* Write a list of registers */
static int __maybe_unused  gxcam_write_regs(struct i2c_client *client,
			     const struct  reg_gx *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = gxcam_write(client, regs[i].addr,regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].addr, ret);

			return ret;
		}
	}
	return 0;
}

static u32 bit_count(u32 n)
{
    n = (n &0x55555555) + ((n >>1) &0x55555555) ;
    n = (n &0x33333333) + ((n >>2) &0x33333333) ;
    n = (n &0x0f0f0f0f) + ((n >>4) &0x0f0f0f0f) ;
    n = (n &0x00ff00ff) + ((n >>8) &0x00ff00ff) ;
    n = (n &0x0000ffff) + ((n >>16) &0x0000ffff) ;

    return n ;
}

static int gxcam_get_wh(struct gxcam *gxcam)
{
    struct i2c_client *client = gxcam->client;
    gxcam_read_d(client, ROI_Width,&gxcam->current_width);
    gxcam_read_d(client, ROI_Height,&gxcam->current_height);
	//print
	v4l2_dbg(1, debug, gxcam->client, "%s:get wh(%d,%d)\n",
				__func__, gxcam->current_width,gxcam->current_height);
	return 0;
}
//read common registers for v4l2-ctrls
static int gxcam_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
    struct gxcam *gxcam = 
		container_of(ctrl->handler, struct gxcam, ctrl_handler);
    struct i2c_client *client = gxcam->client;
    
	switch (ctrl->id) {
	case V4L2_CID_VEYE_GX_WORK_MODE:
        ret = gxcam_read(client, Work_Mode,&ctrl->val);
		break;
	case V4L2_CID_VEYE_GX_TRIGGER_SRC:
        ret = gxcam_read(client, Trigger_Source,&ctrl->val);
		break;

	case V4L2_CID_VEYE_GX_FRAME_RATE:
        ret = gxcam_read(client, Framerate_ex,&ctrl->val);
        ctrl->val = ctrl->val/10000;
        gxcam->cur_fps = ctrl->val;
		break;
	case V4L2_CID_VEYE_GX_SYNC_ROLE:
		ret = gxcam_read(client, Sync_Role,&ctrl->val);
		break;	

	default:
		dev_info(&client->dev,
			 "gxcam_g_volatile_ctrl ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}
    
    v4l2_dbg(1, debug, gxcam->client, "%s: cid = (0x%X), value = (%d).\n",
                     __func__, ctrl->id, ctrl->val);

	return ret;
}

static int gxcam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
	struct gxcam *gxcam = 
		container_of(ctrl->handler, struct gxcam, ctrl_handler);
    struct i2c_client *client = gxcam->client;
    
	v4l2_dbg(1, debug, gxcam->client, "%s: cid = (0x%X), value = (%d).\n",
			 __func__, ctrl->id, ctrl->val);
	
    switch (ctrl->id) {
	case V4L2_CID_VEYE_GX_WORK_MODE:
        ret = gxcam_write(client, Work_Mode,ctrl->val);
		break;
	case V4L2_CID_VEYE_GX_TRIGGER_SRC:
        ret = gxcam_write(client, Trigger_Source,ctrl->val);
		break;
	case V4L2_CID_VEYE_GX_SOFT_TRGONE:
        ret = gxcam_write(client, Trigger_Software,1);
		break;
	case V4L2_CID_VEYE_GX_FRAME_RATE:
        ret = gxcam_write(client, Framerate_ex,ctrl->val*10000);
        gxcam->cur_fps = ctrl->val;
		break;
	case V4L2_CID_VEYE_GX_SYNC_ROLE:
		ret = gxcam_write(client, Sync_Role,ctrl->val);
		break;

	default:
		dev_info(&client->dev,
			 "gxcam_s_ctrl ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}


static const struct v4l2_ctrl_ops gxcam_ctrl_ops = {
    .g_volatile_ctrl = gxcam_g_volatile_ctrl,
	.s_ctrl = gxcam_s_ctrl,
};

//must add to enum_v4l2_ctrls_index once you add a new ctrl here
static struct v4l2_ctrl_config gxcam_v4l2_ctrls[] = {
    //standard v4l2_ctrls
    {
		.ops = NULL,
		.id = V4L2_CID_LINK_FREQ,
		.name = NULL,//kernel will fill it
		.type = V4L2_CTRL_TYPE_MENU ,
        .def = 0,
		.min = 0,
        .max = ARRAY_SIZE(link_freq_menu_items) - 1,
        .step = 0,
        .qmenu_int = link_freq_menu_items,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_PIXEL_RATE,
		.name = NULL,//kernel will fill it
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = GXCAM_PIXEL_RATE,
		.min = GXCAM_PIXEL_RATE,
		.max = GXCAM_PIXEL_RATE,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
	},

	//custom v4l2-ctrls
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_VEYE_GX_WORK_MODE,
		.name = "work_mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = Video_Streaming_mode,
		.min = 0,
		.max = Work_mode_num-1,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_VEYE_GX_TRIGGER_SRC,
		.name = "trigger_src",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = Trg_Hard,
		.min = 0,
		.max = Trg_src_num-1,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_VEYE_GX_SOFT_TRGONE,
		.name = "soft_trgone",
		.type = V4L2_CTRL_TYPE_BUTTON,
		.def = 0,
		.min = 0,
		.max = 0,
		.step = 0,
	},
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_VEYE_GX_SYNC_ROLE,
		.name = "sync_role",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = 0, //default is master
		.min = 0,
		.max = 1, //0:master, 1:slave
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
	{
		.ops = &gxcam_ctrl_ops,
		.id = V4L2_CID_VEYE_GX_FRAME_RATE,
		.name = "frame_rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = GXCAM_DEF_FPS,
		.min = 0,
		.max = GXCAM_DEF_FPS,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE|V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	},
};

//grab some ctrls while streaming
static void gxcam_v4l2_ctrl_grab(struct gxcam *gxcam, bool grabbed)
{
    if (gxcam->ctrls[CID_VEYE_GX_WORK_MODE])
        v4l2_ctrl_grab(gxcam->ctrls[CID_VEYE_GX_WORK_MODE], grabbed);
    if (gxcam->ctrls[CID_VEYE_GX_TRIGGER_SRC])
        v4l2_ctrl_grab(gxcam->ctrls[CID_VEYE_GX_TRIGGER_SRC], grabbed);
    if (gxcam->ctrls[CID_VEYE_GX_FRAME_RATE])
        v4l2_ctrl_grab(gxcam->ctrls[CID_VEYE_GX_FRAME_RATE], grabbed);
    if (gxcam->ctrls[CID_VEYE_GX_SYNC_ROLE])
        v4l2_ctrl_grab(gxcam->ctrls[CID_VEYE_GX_SYNC_ROLE], grabbed);
}

static void gxcam_v4l2_ctrl_init(struct gxcam *gxcam)
{
    int i = 0;
    u32 value = 0;
    struct i2c_client *client = gxcam->client;
    for (i = 0; i < ARRAY_SIZE(gxcam_v4l2_ctrls); ++i) {
		switch(gxcam_v4l2_ctrls[i].id)
        {
            case V4L2_CID_VEYE_GX_WORK_MODE:
				gxcam_read_d(client, Work_Mode,&value);
                gxcam_v4l2_ctrls[i].def = value;
                v4l2_dbg(1, debug, gxcam->client, "%s:default trigger mode %d\n", __func__, value);
            break;
            case V4L2_CID_VEYE_GX_TRIGGER_SRC:
                gxcam_read_d(client, Trigger_Source,&value);
                gxcam_v4l2_ctrls[i].def = value;
                v4l2_dbg(1, debug, gxcam->client, "%s:default trigger source %d\n", __func__, value);
            break;
			case V4L2_CID_VEYE_GX_SYNC_ROLE:
				gxcam_read_d(client, Sync_Role,&value);
				gxcam_v4l2_ctrls[i].def = value;
				v4l2_dbg(1, debug, gxcam->client, "%s:default sync role %d\n", __func__, value);
			break;
            case V4L2_CID_VEYE_GX_FRAME_RATE:
                gxcam_read_d(client, Framerate_ex,&value);
                gxcam_v4l2_ctrls[i].def = value/10000;
                gxcam_read_d(client, MaxFrame_Rate,&value);
                gxcam_v4l2_ctrls[i].max = value/10000;
                v4l2_dbg(1, debug, gxcam->client, "%s:default framerate %lld , max fps %lld \n", __func__, \
                    gxcam_v4l2_ctrls[i].def,gxcam_v4l2_ctrls[i].max);
            break;

			case V4L2_CID_PIXEL_RATE:
				if(gxcam->pixelrate != 0){
					gxcam_v4l2_ctrls[i].min = gxcam->pixelrate;
					gxcam_v4l2_ctrls[i].def = gxcam->pixelrate;
					gxcam_v4l2_ctrls[i].max = gxcam->pixelrate;
				}
            default:
            break;
        }
	}
}
static int gxcam_csi2_enum_mbus_code(
			struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_mbus_code_enum *code)
{
	struct gxcam *gxcam = to_gxcam(sd);
	struct gxcam_format *supported_formats = gxcam->supported_formats;
	int num_supported_formats = gxcam->num_supported_formats;
	
	if (code->pad >= NUM_PADS)
		return -EINVAL;
	
	if (code->pad == IMAGE_PAD) {
		if (code->index >= num_supported_formats)
			return -EINVAL;
		code->code = supported_formats[code->index].mbus_code;
        v4l2_dbg(1, debug, sd, "%s: index = (%d) mbus code (%x)\n", __func__, code->index,code->code);
	} else {
		if (code->index > 0)
			return -EINVAL;
		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int gxcam_csi2_enum_framesizes(
			struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct gxcam *gxcam = to_gxcam(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
			 __func__, fse->code, fse->index);

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= gxcam->num_imgmodes)
			return -EINVAL;
		if (fse->code == gxcam->supported_formats[gxcam->current_format_idx].mbus_code) {
			fse->min_width = gxcam->imgmode_list[fse->index].width;
			fse->max_width = gxcam->imgmode_list[fse->index].width;
			fse->min_height = gxcam->imgmode_list[fse->index].height;
			fse->max_height = gxcam->imgmode_list[fse->index].height;
			return 0;
		}
	}
	 else {
	 	//meta data
	}

	return -EINVAL;
}




static void gxcam_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = VEYE_GX_EMBEDDED_LINE_WIDTH;
	fmt->format.height = VEYE_GX_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}


static int gxcam_csi2_get_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_state *sd_state,
								struct v4l2_subdev_format *format)
{
	struct gxcam *gxcam = to_gxcam(sd);
	struct gxcam_format *current_format;
	struct v4l2_mbus_framefmt *fmt = &format->format;

	if (format->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&gxcam->mutex);
    if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
            struct v4l2_mbus_framefmt *try_fmt =
                v4l2_subdev_state_get_format(sd_state, format->pad);
			current_format = &gxcam->supported_formats[gxcam->current_format_idx];
            try_fmt->code = format->pad == IMAGE_PAD ? current_format->mbus_code : MEDIA_BUS_FMT_SENSOR_DATA;
            format->format = *try_fmt;
        } else {
    	if (format->pad == IMAGE_PAD) {
    		current_format = &gxcam->supported_formats[gxcam->current_format_idx];
    		format->format.width = gxcam->imgmode_list[gxcam->current_imgmode_idx].width;
    		format->format.height = gxcam->imgmode_list[gxcam->current_imgmode_idx].height;
            
    		format->format.code = current_format->mbus_code;
    		format->format.field = V4L2_FIELD_NONE;
			fmt->colorspace = V4L2_COLORSPACE_REC709;//color

			/*fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
			fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
									fmt->colorspace,
									fmt->ycbcr_enc);
			fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);*/

			v4l2_dbg(1, debug, sd, "%s: width: (%d) height: (%d) code: (0x%X)\n",
				__func__, format->format.width, format->format.height,
				format->format.code);
    	} else {
    		gxcam_update_metadata_pad_format(format);
    	}
    }

	mutex_unlock(&gxcam->mutex);
	return 0;
}

static int gxcam_read_sel(struct gxcam *gxcam, struct v4l2_rect *rect) {
	struct i2c_client *client = gxcam->client;
	int ret = 0;
	ret += gxcam_read_d(client, ROI_Offset_X, &rect->left);
	ret += gxcam_read_d(client, ROI_Offset_Y, &rect->top);
	ret += gxcam_read_d(client, ROI_Width, &rect->width);
	ret += gxcam_read_d(client, ROI_Height, &rect->height);

	if (ret || rect->width == 0 
		|| rect->height == 0) {
			v4l2_err(client, "%s: Failed to read selection.\n",
			 	 __func__);
			return -EINVAL;
		}
	return 0;
}
static const struct v4l2_rect *
__gxcam_get_pad_crop(struct gxcam *gxcam,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	int ret = 0;
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		ret = gxcam_read_sel(gxcam, &gxcam->crop);
		if (ret)
			return NULL;
        return &gxcam->crop;
	}

	return NULL;
}
static int gxcam_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct gxcam *gxcam = to_gxcam(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
			mutex_lock(&gxcam->mutex);
    		sel->r = *__gxcam_get_pad_crop(gxcam, sd_state, sel->pad,
    						sel->which);
			mutex_unlock(&gxcam->mutex);
    		break;
    	}
        case V4L2_SEL_TGT_NATIVE_SIZE:
    //active area
        case V4L2_SEL_TGT_CROP_DEFAULT:
        case V4L2_SEL_TGT_CROP_BOUNDS:
            sel->r.top = 0;
            sel->r.left = 0;
            sel->r.width = gxcam->current_width;
            sel->r.height = gxcam->current_height;
		break;
        default:
		return -EINVAL;
	}
    sel->flags = V4L2_SEL_FLAG_LE;
    return 0;
}

static int gxcam_csi2_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_state *sd_state,
								struct v4l2_subdev_format *format)
{
	//int i;
	struct gxcam *gxcam = to_gxcam(sd);
    struct v4l2_mbus_framefmt *framefmt;
    //struct v4l2_subdev_selection sel;
   // VEYE_TRACE
	if (format->pad >= NUM_PADS)
		return -EINVAL;
    
	if (format->pad == IMAGE_PAD) {
		//	VEYE_TRACE
			struct gxcam_format *new_format = NULL;
			int i;
			/* Find a matching format */
			for (i = 0; i < gxcam->num_supported_formats; i++) {
				if (format->format.code ==
				    gxcam->supported_formats[i].mbus_code) {
					new_format = &gxcam->supported_formats[i];
					gxcam->current_format_idx = i;
					break;
				}
			}
			if (!new_format) {
				v4l2_err(sd, "%s: Unsupported media bus format 0x%X\n",
					 __func__, format->format.code);

				return -EINVAL;
			}
			/* Update to the new format */
			gxcam_write(gxcam->client, Pixel_Format,new_format->data_type);
			//print
			v4l2_dbg(1, debug, sd, "%s: set Pixel_Format = (0x%X)\n",
					 __func__, new_format->data_type);
			//set image mode according to width and height
			for (i = 0; i < gxcam->num_imgmodes; i++) {
				if (format->format.width ==
				    gxcam->imgmode_list[i].width &&
				    format->format.height ==
				    gxcam->imgmode_list[i].height) {
					gxcam->current_imgmode_idx = i;
					//write registers to set image mode
					gxcam_write(gxcam->client, Video_Mode,i);
					break;
				}
			}
			//set roi
			format->format.code = new_format->mbus_code;
			format->format.field = V4L2_FIELD_NONE;
			//update_controls(gxcam);
	} else {
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_state_get_format(sd_state,
							      format->pad);
			*framefmt = format->format;
		} else {
			/* Only one embedded data mode is supported */
			gxcam_update_metadata_pad_format(format);
		}
	}
	return 0;
}

static void gxcam_free_controls(struct gxcam *gxcam)
{
    VEYE_TRACE
	v4l2_ctrl_handler_free(gxcam->sd.ctrl_handler);
	//mutex_destroy(&gxcam->mutex);
}

static u32 mvdatatype_to_mbus_code(int data_type)
{
    VEYE_TRACE
   // debug_printk("%s: data type %d\n",
	//				__func__, data_type);
    switch(data_type) {
	case MV_DT_Mono8:
        return MEDIA_BUS_FMT_Y8_1X8;
	case MV_DT_Mono10:
		return MEDIA_BUS_FMT_Y10_1X10;
	case MV_DT_Mono12:
		return MEDIA_BUS_FMT_Y12_1X12;
    case MV_DT_Mono14:
		return MEDIA_BUS_FMT_Y14_1X14;
	case MV_DT_UYVY:
		return MEDIA_BUS_FMT_UYVY8_1X16;
	case MV_DT_YUYV:
		return MEDIA_BUS_FMT_YUYV8_1X16;
	}
	return 0;
}

static int get_fmt_index(struct gxcam *gxcam,u32 datatype)
{
    int i = 0;
    for(;i < gxcam->num_supported_formats;++i)
    {
        if((gxcam->supported_formats[i].data_type) == datatype)
            return i;
    }
    return -1;
}

static int gxcam_enum_pixformat(struct gxcam *gxcam)
{
	int ret = 0;
	u32 mbus_code = 0;
	int pixformat_type;
	int index = 0;
    int bitindex = 0;
	int num_pixformat = 0;
    u32 fmtcap = 0;
    u32 cur_fmt;
	struct i2c_client *client = gxcam->client;
VEYE_TRACE
    ret = gxcam_read_d(client, Format_cap, &fmtcap);
    if (ret < 0)
		goto err;

	num_pixformat = bit_count(fmtcap);
	if (num_pixformat < 0)
		goto err;
    
    v4l2_dbg(1, debug, gxcam->client, "%s: format count: %d; format cap 0x%x\n",
					__func__, num_pixformat,fmtcap);
    
	gxcam->supported_formats = devm_kzalloc(&client->dev,
		sizeof(*(gxcam->supported_formats)) * (num_pixformat+1), GFP_KERNEL);
	while(fmtcap){
        if(fmtcap&1){
            //which bit is set?
            pixformat_type = bitindex;
            fmtcap >>= 1;
            bitindex++;
        }
        else{
            fmtcap >>= 1;
            bitindex++;
            continue;
        }
        mbus_code = mvdatatype_to_mbus_code(pixformat_type);
		gxcam->supported_formats[index].index = index;
		gxcam->supported_formats[index].mbus_code = mbus_code;
		gxcam->supported_formats[index].data_type = pixformat_type;
        v4l2_dbg(1, debug, gxcam->client, "%s support format index %d mbuscode %d datatype: %d\n",
					__func__, index,mbus_code,pixformat_type);
        index++;
	}
	gxcam->num_supported_formats = num_pixformat;
    gxcam_read_d(client, Pixel_Format, &cur_fmt);
	gxcam->current_format_idx = get_fmt_index(gxcam,cur_fmt);
    v4l2_dbg(1, debug, gxcam->client, "%s: cur format: %d\n",
					__func__, cur_fmt);
	// gxcam_add_extension_pixformat(gxcam);
	return 0;
VEYE_TRACE
err:
	return -ENODEV;
}

static int gxcam_enum_imgmode(struct gxcam *gxcam)
{
	int ret = 0;
	u32 width_height = 0;
	u32 frame_rate_param = 0;
	u32 video_mode_num = 0;
	int i = 0;
	struct i2c_client *client = gxcam->client;
VEYE_TRACE
	ret = gxcam_read_d(client, VideoModeNum, &video_mode_num);
	if (ret < 0)
		goto err;
	v4l2_dbg(1, debug, gxcam->client, "%s: video mode num %d\n",
					__func__, video_mode_num);
	if(video_mode_num > GXCAM_MAX_VIDEO_MODE_NUM)
		video_mode_num = GXCAM_MAX_VIDEO_MODE_NUM;
	
	gxcam->imgmode_list = devm_kzalloc(&client->dev,
		sizeof(*(gxcam->imgmode_list)) * video_mode_num, GFP_KERNEL);
	if(!gxcam->imgmode_list)
		goto err;
	
	for(i=0;i<video_mode_num;++i)
	{
		gxcam_read_d(client, VidoeMode_WH1 + i*8, &width_height);
		gxcam_read_d(client, VideoMode_Param1 + i*8, &frame_rate_param);
		gxcam->imgmode_list[i].width = (width_height >>16) & 0xFFFF;
		gxcam->imgmode_list[i].height = width_height & 0xFFFF;
		gxcam->imgmode_list[i].maxfps = frame_rate_param & 0xFFFF;
		gxcam->imgmode_list[i].readoutmode = (frame_rate_param >>16) & 0xFF;
		v4l2_dbg(1, debug, gxcam->client, "%s: mode %d: w %d h %d fps %d readout %d\n",
					__func__, i,
					gxcam->imgmode_list[i].width,
					gxcam->imgmode_list[i].height,
					gxcam->imgmode_list[i].maxfps,
					gxcam->imgmode_list[i].readoutmode);
	}
	gxcam->num_imgmodes = video_mode_num;
	//current_imgmode_idx是Video_Mode
	gxcam_read_d(client, Video_Mode, &gxcam->current_imgmode_idx);
	if(gxcam->current_imgmode_idx >= gxcam->num_imgmodes)
		gxcam->current_imgmode_idx = 0;
	v4l2_dbg(1, debug, gxcam->client, "%s: current imgmode idx %d\n",
					__func__, gxcam->current_imgmode_idx);
	return 0;
	err:
	return -ENODEV;
}

static void gxcam_get_mipifeature(struct gxcam *gxcam)
{
    u32 lane_num;
	u32 lanenum_cap;
    u32 mipi_datarate;
    struct i2c_client *client = gxcam->client;
    gxcam_read_d(client, Lane_Num, &lane_num);
    if(lane_num == 4){
        gxcam->lane_num = 4;
    }else{
        gxcam->lane_num = 2;
    }
    gxcam_read_d(client, LaneNum_Cap, &lanenum_cap);
	if (lanenum_cap <= 0xF)//
	{
		gxcam->lanecap = lanenum_cap;
	}
    gxcam_read_d(client, MIPI_DataRate, &mipi_datarate);
    if(mipi_datarate == 0xFFFFFFFF)
        mipi_datarate = GXCAM_DEFAULT_LINK_FREQ;
    else
        mipi_datarate *=1000;//register value is kbps
    
    gxcam->mipi_datarate = mipi_datarate;
    
	link_freq_menu_items[0] = gxcam->mipi_datarate>>1;//hz is half of datarate
    v4l2_dbg(1, debug, gxcam->client, "%s: lane num %d, datarate %d bps\n",
					__func__, gxcam->lane_num,gxcam->mipi_datarate);
    return;
}

/* Start streaming */
static int gxcam_start_streaming(struct gxcam *gxcam)
{
	struct i2c_client *client = gxcam->client;
	int ret;
    VEYE_TRACE
	/* Apply customized values from user */
 //   ret =  __v4l2_ctrl_handler_setup(gxcam->sd.ctrl_handler);
    debug_printk("gxcam_start_streaming \n");
	/* set stream on register */
    ret = gxcam_write(client, Image_Acquisition,1);
	if (ret)
		return ret;

	/* some v4l2 ctrls cannot change during streaming */
    gxcam_v4l2_ctrl_grab(gxcam,true);
	return ret;
}

/* Stop streaming */
static int gxcam_stop_streaming(struct gxcam *gxcam)
{
	struct i2c_client *client = gxcam->client;
	int ret;
VEYE_TRACE
	/* set stream off register */
    ret = gxcam_write(client, Image_Acquisition,0);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
    debug_printk("gxcam_stop_streaming \n");
    
   	gxcam_v4l2_ctrl_grab(gxcam,false);

	/*
	 * Return success even if it was an error, as there is nothing the
	 * caller can do about it.
	 */
	return 0;
}

static int gxcam_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct gxcam *gxcam = to_gxcam(sd);
    struct i2c_client *client = gxcam->client;
	int ret = 0;
	enable = !!enable;
	
	if (gxcam->streaming == enable) {
        dev_info(&client->dev, "%s streaming state not changed %d!\n", __func__,enable);
		return 0;
	}
VEYE_TRACE
	if (enable) {

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = gxcam_start_streaming(gxcam);
		if (ret)
			goto end;
	} else {
		gxcam_stop_streaming(gxcam);
	}
	gxcam->streaming = enable;
	return ret;
end:
	return ret;
}
/* Power management functions */
static int gxcam_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);
	int ret;

	ret = regulator_bulk_enable(GXCAM_NUM_SUPPLIES,
				    gxcam->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
    
	gpiod_set_value_cansleep(gxcam->reset_gpio, 1);
	usleep_range(STARTUP_MIN_DELAY_US,
		     STARTUP_MIN_DELAY_US + STARTUP_DELAY_RANGE_US);
    debug_printk("gxcam_power_on\n");
	return 0;
}

static int gxcam_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);
    //do not really power off, because we might use i2c script at any time
	gpiod_set_value_cansleep(gxcam->reset_gpio, 1);
	regulator_bulk_disable(GXCAM_NUM_SUPPLIES, gxcam->supplies);
    debug_printk("gxcam_power_off,not really off\n");
	return 0;
}

static int gxcam_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gxcam *gxcam = to_gxcam(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, IMAGE_PAD);
    
//	struct v4l2_mbus_framefmt *try_fmt_meta =
//		v4l2_subdev_state_get_format(fh->pad, METADATA_PAD);
    struct v4l2_rect *try_crop;
	/* Initialize try_fmt */
	try_fmt->width = gxcam->imgmode_list[gxcam->current_imgmode_idx].width;
	try_fmt->height = gxcam->imgmode_list[gxcam->current_imgmode_idx].height;
	try_fmt->code = gxcam->supported_formats[0].mbus_code;
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
/*	try_fmt_meta->width = GXCAM_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = GXCAM_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;
*/
    /* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_state_get_crop( fh->state, 0);
	try_crop->top = 0;
	try_crop->left = 0;
	try_crop->width = gxcam->imgmode_list[gxcam->current_imgmode_idx].width;
	try_crop->height = gxcam->imgmode_list[gxcam->current_imgmode_idx].height;
    
	return 0;
}
static const struct v4l2_subdev_internal_ops gxcam_internal_ops = {
	.open = gxcam_open,
};

static const struct v4l2_subdev_core_ops gxcam_core_ops = {
	// .s_power = gxcam_s_power,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops gxcam_video_ops = {
	.s_stream = gxcam_set_stream,
};

static const struct v4l2_subdev_pad_ops gxcam_pad_ops = {
	.enum_mbus_code = gxcam_csi2_enum_mbus_code,
	.get_fmt = gxcam_csi2_get_fmt,
	.set_fmt = gxcam_csi2_set_fmt,
	.enum_frame_size = gxcam_csi2_enum_framesizes,
    .get_selection = gxcam_get_selection,
	//.set_selection = gxcam_set_selection,

	//todo
	//.get_mbus_config =  //will add lane number setting here
};

static const struct v4l2_subdev_ops gxcam_subdev_ops = {
	.core = &gxcam_core_ops,
	.video = &gxcam_video_ops,
	.pad = &gxcam_pad_ops,
};


static int __maybe_unused gxcam_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);

	if (gxcam->streaming){
		gxcam_stop_streaming(gxcam);
		gxcam->streaming = 0;
	}

	return 0;
}

static int __maybe_unused gxcam_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);
	int ret;

	if (gxcam->streaming) {
		ret = gxcam_start_streaming(gxcam);
		if (ret)
			goto error;
		gxcam->streaming = 1;
	}

	return 0;

error:
	gxcam_stop_streaming(gxcam);
	gxcam->streaming = 0;
	return ret;
}

static int gxcam_enum_controls(struct gxcam *gxcam)
{
    struct i2c_client *client = gxcam->client;
    struct v4l2_ctrl_handler *ctrl_hdlr;
    struct v4l2_fwnode_device_properties props;
    int ret;
    int i;
    struct v4l2_ctrl *ctrl;
    ctrl_hdlr = &gxcam->ctrl_handler;
    ret = v4l2_ctrl_handler_init(ctrl_hdlr, ARRAY_SIZE(gxcam_v4l2_ctrls));
    if (ret)
        return ret;
VEYE_TRACE
   // mutex_init(&gxcam->mutex);
    ctrl_hdlr->lock = &gxcam->mutex;
    
    for (i = 0; i < ARRAY_SIZE(gxcam_v4l2_ctrls); ++i) {
		ctrl = v4l2_ctrl_new_custom(
			ctrl_hdlr,
			&gxcam_v4l2_ctrls[i],
			NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %d ctrl\n",i);
			gxcam->ctrls[i] = NULL;
			continue;
		}
		gxcam->ctrls[i] = ctrl;
        dev_dbg(&client->dev, "init control %s success\n",gxcam_v4l2_ctrls[i].name);
	}
    
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &gxcam_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	gxcam->sd.ctrl_handler = ctrl_hdlr;
    v4l2_ctrl_handler_setup(ctrl_hdlr);
VEYE_TRACE
    dev_info(&client->dev, "gxcam_enum_controls success\n");
	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	//mutex_destroy(&gxcam->mutex);

	return ret;

}

static int gxcam_get_regulators(struct gxcam *gxcam)
{
	struct i2c_client *client = v4l2_get_subdevdata(&gxcam->sd);
	unsigned int i;

	for (i = 0; i < GXCAM_NUM_SUPPLIES; i++)
		gxcam->supplies[i].supply = gxcam_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       GXCAM_NUM_SUPPLIES,
				       gxcam->supplies);
}

/* Verify chip ID */
static int gxcam_identify_module(struct gxcam * gxcam)
{
	int ret;
    u32 device_id;
	//u32 firmware_version;
	u32 value[8];
    struct i2c_client *client = v4l2_get_subdevdata(&gxcam->sd);
    
    ret = gxcam_read_d(client, Model_Name, &device_id);
    if (ret ) {
        dev_err(&client->dev, "failed to read chip id\n");
        ret = -ENODEV;
        return ret;
    }
	ret = gxcam_read_d(client, CameraModel0, &value[0]);
	ret |= gxcam_read_d(client, CameraModel1, &value[1]);
	ret |= gxcam_read_d(client, CameraModel2, &value[2]);
	ret |= gxcam_read_d(client, CameraModel3, &value[3]);
	ret |= gxcam_read_d(client, CameraModel4, &value[4]);
	ret |= gxcam_read_d(client, CameraModel5, &value[5]);
	ret |= gxcam_read_d(client, CameraModel6, &value[6]);
	ret |= gxcam_read_d(client, CameraModel7, &value[7]);

	if (ret) {
		dev_err(&client->dev, "failed to read camera model\n");
		ret = -ENODEV;
		return ret;
	}
	strncpy(gxcam->camera_model, (char *)value, sizeof(gxcam->camera_model));

	dev_info(&client->dev, "camera is: %s\n", gxcam->camera_model);

    /*ret = gxcam_read(client, Device_Version, &firmware_version);
    if (ret) {
        dev_err(&client->dev, "read firmware version failed\n");
    }
    dev_info(&client->dev, "firmware version: 0x%04X\n", firmware_version);
	*/
	return 0;
}

static int gxcam_check_hwcfg(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);
    
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
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
	if(ep_cfg.bus.mipi_csi2.num_data_lanes != gxcam->lane_num){
		dev_err(dev, "dts lane num %d mismatch camera lane num %d\n",ep_cfg.bus.mipi_csi2.num_data_lanes,gxcam->lane_num);
		ret = -ENOENT;
		goto error_out;
	}
	/* Check the number of MIPI CSI2 data lanes with camera capbility*/
	/*if ((0x1 << (ep_cfg.bus.mipi_csi2.num_data_lanes-1)) | gxcam->lanecap) {
		//set camera lane num accroding to dts
		gxcam->lane_num = ep_cfg.bus.mipi_csi2.num_data_lanes;
		dev_info(dev, "Success to get gxcam endpoint data lanes, dts uses %d lanes,will set to camera\n", ep_cfg.bus.mipi_csi2.num_data_lanes);
	}
	else{
		dev_err(dev, "dts lane num %d mismatch camera data lane capbility 0x%x\n",ep_cfg.bus.mipi_csi2.num_data_lanes,gxcam->lanecap);
		ret = -ENOENT;
		goto error_out;
	}*/
	ret = 0;
error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static ssize_t model_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct gxcam *cam = container_of(kobj, struct gxcam, kobj);
    return sprintf(buf, "%s\n", cam->camera_model);
}

static ssize_t width_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct gxcam *cam = container_of(kobj, struct gxcam, kobj);
    return sprintf(buf, "%d\n", cam->current_width);
}

static ssize_t height_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct gxcam *cam = container_of(kobj, struct gxcam, kobj);
    return sprintf(buf, "%d\n", cam->current_height);
}

static ssize_t fps_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct gxcam *cam = container_of(kobj, struct gxcam, kobj);
    return sprintf(buf, "%d\n", cam->cur_fps);
}

static struct kobj_attribute model_attribute = __ATTR(camera_model, 0444, model_show, NULL);
static struct kobj_attribute width_attribute = __ATTR(width, 0444, width_show, NULL);
static struct kobj_attribute height_attribute = __ATTR(height, 0444, height_show, NULL);
static struct kobj_attribute fps_attribute = __ATTR(fps, 0444, fps_show, NULL);


static struct attribute *gxcam_attrs[] = {
    &model_attribute.attr,
    &width_attribute.attr,
    &height_attribute.attr,
	&fps_attribute.attr,
    NULL,
};

static struct attribute_group gxcam_attr_group = {
    .attrs = gxcam_attrs,
};

static const struct attribute_group *gxcam_attr_groups[] = {
    &gxcam_attr_group,
    NULL,
};

static const struct kobj_type gxcam_ktype = {
    .sysfs_ops = &kobj_sysfs_ops,
    .default_groups = gxcam_attr_groups,
};

static int gxcam_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gxcam *gxcam;
    
	int ret;
	dev_info(dev, "veye gx series camera driver version: %02x.%02x.%02x\n",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);
    
	gxcam = devm_kzalloc(&client->dev, sizeof(struct gxcam), GFP_KERNEL);
	if (!gxcam)
		return -ENOMEM;
	/* Initialize subdev */
	v4l2_i2c_subdev_init(&gxcam->sd, client, &gxcam_subdev_ops);
	gxcam->client = client;
	
	ret = gxcam_get_regulators(gxcam);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}
	
    gxcam->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gxcam->reset_gpio)) {
		ret = PTR_ERR(gxcam->reset_gpio);
		dev_err(dev, "failed to get reset gpio: %d\n", ret);
		return ret;
	}

	if (gxcam->reset_gpio) {
		dev_info(dev, "reset gpio acquired\n");
		dev_info(dev, "reset gpio value = %d\n",
				 gpiod_get_value_cansleep(gxcam->reset_gpio));
	} else {
		dev_info(dev, "no reset gpio provided\n");
	}

    mutex_init(&gxcam->mutex);
	
	ret = gxcam_power_on(dev);
	if (ret)
		goto err_destroy_mutex;
    VEYE_TRACE
    ret = gxcam_identify_module(gxcam);
	if (ret){
        goto error_power_off;
    }
	VEYE_TRACE
	if (gxcam_enum_pixformat(gxcam)) {
		dev_err(dev, "enum pixformat failed.\n");
		ret = -ENODEV;
		goto error_power_off;
	}
	gxcam_enum_imgmode(gxcam);
    gxcam_get_mipifeature(gxcam);
    /* Check the hardware configuration in device tree */
    if(gxcam_check_hwcfg(dev))
		goto error_power_off; 
    
    //read roi
    gxcam_get_wh(gxcam);

    gxcam_v4l2_ctrl_init(gxcam);
	if (gxcam_enum_controls(gxcam)) {
		dev_err(dev, "enum controls failed.\n");
		ret = -ENODEV;
		goto error_power_off;
	}
    //set to video stream mode
//	gxcam_write(client, Work_Mode,0);
    //stop acquitsition
    gxcam_write(client, Image_Acquisition,0);
	
    //set camera lane num
	//gxcam_write(client, Lane_Num,gxcam->lane_num);

	/* Initialize subdev */
	gxcam->sd.internal_ops = &gxcam_internal_ops;
	gxcam->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gxcam->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	/* Initialize source pad */
	gxcam->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
//	gxcam->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gxcam->sd.entity, NUM_PADS, gxcam->pad);
	if (ret < 0){
		dev_err(dev, "media_entity_pads_init failed\n");
		goto error_power_off;
	}

	ret = kobject_init_and_add(&gxcam->kobj, &gxcam_ktype, &client->dev.kobj, "veye_gxcam");
    if (ret) {
        dev_err(dev, "kobject_init_and_add failed\n");
        goto error_media_entity;
    }

	ret = v4l2_async_register_subdev_sensor(&gxcam->sd);
	if (ret)
		goto error_media_entity;

	return 0;

error_media_entity:

    kobject_put(&gxcam->kobj);

	media_entity_cleanup(&gxcam->sd.entity);

	gxcam_free_controls(gxcam);

error_power_off:
	gxcam_power_off(dev);
	gxcam_free_controls(gxcam);

err_destroy_mutex:
	mutex_destroy(&gxcam->mutex);
	return ret;
}

static void gxcam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gxcam *gxcam = to_gxcam(sd);

	v4l2_async_unregister_subdev(sd);

    kobject_put(&gxcam->kobj);

	media_entity_cleanup(&sd->entity);
	gxcam_free_controls(gxcam);

	mutex_destroy(&gxcam->mutex);
}

static const struct of_device_id veyegx_cam_dt_ids[] = {
	{ .compatible = "veye,gxcam"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, veyegx_cam_dt_ids);

static struct i2c_driver veyegx_cam_i2c_driver = {
	.driver = {
		.name = "gxcam",
		.of_match_table	= veyegx_cam_dt_ids,
	},
	.probe = gxcam_probe,
	.remove = gxcam_remove,
};

module_i2c_driver(veyegx_cam_i2c_driver);

MODULE_AUTHOR("xumm <www.veye.cc>");
MODULE_DESCRIPTION("VEYE GX series mipi camera v4l2 driver");
MODULE_LICENSE("GPL v2");

