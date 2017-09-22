/*
 *  Copyright (C) 2004 Samsung Electronics
 *             SW.LEE <hitchcar@samsung.com>
 *            - based on Russell King : pcf8583.c
 * 	      - added  smdk24a0, smdk2440
 *            - added  poseidon (s3c24a0+wavecom)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Driver for FIMC2.x Camera Decoder
 *
 */

#include <linux/module.h>    
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
//#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>

//#include <mach/hardware.h>

//#include <plat/gpio-cfg.h>
//#include <plat/egpio.h>

#include "s3c_camif.h"

#include "s5k5ca.h" 
#include "s5k5ca_0723.h"                               
#include <linux/proc_fs.h>
#include "../../../fs/proc/internal.h"
/* Camera functional setting values configured by user concept */
struct s5k5ca_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;	/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int manual_wb;	/* V4L2_CID_WHITE_BALANCE_PRESET */
	unsigned int wb_temp;	/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;	/* Color FX (AKA Color tone) */
	unsigned int contrast;	/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct s5k5ca_platform_data {
	unsigned int default_width;
	unsigned int default_height;
	unsigned int pixelformat;
	int freq;	/* MCLK in KHz */

	/* This SoC supports Parallel & CSI-2 */
	int is_mipi;
};

struct s5k5ca_state {
	struct s5k5ca_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct s5k5ca_userset userset;
	int freq;	/* MCLK in KHz */
	int is_mipi;
	int isize;
	int ver;
	int fps;
};

// function define
//#define CONFIG_LOAD_FILE            
#define I2C_BURST_MODE //dha23 100325 카메라 기동 시간 줄이기 위해 I2C Burst mode 사용.
// Purpose of verifying I2C operaion. must be ignored later.    
//#define LOCAL_CONFIG_S5K5CA_I2C_TEST
static struct i2c_client * s_i2c_client = NULL;
static struct i2c_driver s5k5ca_driver;
int set_af_after_capture = 0; //dha23 100518
static struct proc_dir_entry * s_proc = NULL;
static uint8_t s5k5ca_init_flag = false;

static void s5k5ca_sensor_gpio_init(void);
void s5k5ca_sensor_enable(void);
static void s5k5ca_sensor_disable(void);

static int s5k5ca_sensor_init(void);
static void s5k5ca_sensor_exit(void);

static int s5k5ca_sensor_change_size(struct i2c_client *client, int size);

#ifdef CONFIG_FLASH_AAT1271A
	extern int aat1271a_flash_init(void);
	extern void aat1271a_flash_exit(void);
	extern void aat1271a_falsh_camera_control(int ctrl);
	extern void aat1271a_falsh_movie_control(int ctrl);
#endif

#ifdef CONFIG_LOAD_FILE
	static int s5k5ca_regs_table_write(char *name);
#endif

/* 
 * MCLK: 24MHz, PCLK: 54MHz
 * 
 * In case of PCLK 54MHz
 *
 * Preview Mode (1024 * 768)  
 * 
 * Capture Mode (2048 * 1536)
 * 
 * Camcorder Mode
 */
static camif_cis_t s5k5ca_data = {
	itu_fmt:       	CAMIF_ITU601,
	order422:      	CAMIF_CRYCBY,
	camclk:        	24000000,		
	source_x:      	1024,		
	source_y:      	768,
	win_hor_ofst:  	0,
	win_ver_ofst:  	0,
	win_hor_ofst2: 	0,
	win_ver_ofst2: 	0,
	polarity_pclk: 	0,
	polarity_vsync:	1,
	polarity_href: 	0,
	reset_type:		CAMIF_RESET,
	reset_udelay: 	5000,
};

/* #define S5K5CA_ID	0x78 */
#define S5K5CA_ID	0x78 
static unsigned short s5k5ca_normal_i2c[] = { (S5K5CA_ID >> 1), I2C_CLIENT_END };
static unsigned short s5k5ca_ignore[] = { I2C_CLIENT_END };
static unsigned short s5k5ca_probe[] = { I2C_CLIENT_END };

static int previous_scene_mode = -1;
static int previous_WB_mode = 0;
static int af_mode = -1;
static unsigned short lux_value = 0;
int cam_flash_on = 0; //insook0804
int locked_ae_awb = 0; //insook0901

bool isS5k5caEnabled = false; 



//static unsigned short AFPosition = 0x00FF; 
//static unsigned short DummyAFPosition = 0x00FE; 

static unsigned short AFPosition = 0x0000; 
static unsigned short DummyAFPosition = 0x0001; 


#if 0
static struct i2c_client_address_data s5k5ca_addr_data = {
	.normal_i2c = s5k5ca_normal_i2c,
	.ignore		= s5k5ca_ignore,
	.probe		= s5k5ca_probe,
};
#endif

static int s5k5ca_sensor_read(struct i2c_client *client,
		unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static int s5k5ca_sensor_write(struct i2c_client *client,unsigned short subaddr, unsigned short val)
{


	//printk("   s5k5ca_sensor_write   (%x)\n", val);

	if(subaddr == 0xdddd)
	{
			msleep(val);
			printk("delay time(%d msec)\n", val);
	}	
	else
	{					
		unsigned char buf[4];
		struct i2c_msg msg = { client->addr, 0, 4, buf };

		buf[0] = (subaddr >> 8);
		buf[1] = (subaddr & 0xFF);
		buf[2] = (val >> 8);
		buf[3] = (val & 0xFF);

//		return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
                if(i2c_transfer(client->adapter, &msg, 1) == 1)
                    return 0;
                else{
                    printk("[**ASWOOGI**] s5k5ca_sensor_write_list error\n"); 
                    return -EIO;
                }
	}
}




#if 1
#define S5K5CA_CHIP_ID 0x05ca
#define MODEL_TRULY 0
#define MODEL_SUNNY 1
#define S5K5CA_IS_ON 1
#define CDBG(fmt, args...) printk(KERN_INFO "s5k5ca.c: " fmt, ## args)


static struct  i2c_client *s5k5ca_client = NULL;
static unsigned int reg_num;

static uint16_t s5k5ca_model_id = MODEL_SUNNY;
static struct s5k5ca_i2c_reg_conf * p_s5k5ca_init_reg_config;

static int s5k5ca_i2c_rxdata(unsigned short saddr,
                             unsigned char *rxdata, int length)
{
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = 2,
            .buf = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    if (i2c_transfer(s5k5ca_client->adapter, msgs, 2) < 0)
    {
        CDBG("s5k5ca_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}
static int32_t s5k5ca_i2c_read_w(unsigned short raddr, unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00) >> 8;
    buf[1] = (raddr & 0x00FF);

    rc = s5k5ca_i2c_rxdata(s5k5ca_client->addr, buf, 2);
    if (rc < 0)
    {
        return rc;
    }

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
    {
        CDBG("s5k5ca_i2c_read failed!\n");
    }

    return rc;
}
static int32_t s5k5ca_i2c_txdata(unsigned short saddr,
                                 unsigned char *txdata, int length)
{
    int32_t i  = 0;
    int32_t rc = -EFAULT;
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    for (i = 0; i < 3; i++)
    {
        rc = i2c_transfer(s5k5ca_client->adapter, msg, 1);
        if (0 <= rc)
        {
            return 0;
        }
    }

    if (3 == i)
    {
        CDBG("s5k5ca_i2c_txdata faild\n");
        return -EIO;
    }

    return 0;
}


static int32_t s5k5ca_i2c_write_w(unsigned short waddr, unsigned short wdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (wdata & 0xFF00) >> 8;
    buf[3] = (wdata & 0x00FF);

    rc = s5k5ca_i2c_txdata(s5k5ca_client->addr, buf, 4);

    if (rc < 0)
    {
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
    }

    return rc;
}

static int32_t s5k5ca_i2c_write_w_table(struct s5k5ca_i2c_reg_conf const *reg_conf_tbl,
                                        int                               num_of_items_in_table)
{
    int i;
    int32_t rc = -EFAULT;

    for (i = 0; i < num_of_items_in_table; i++)
    {
        rc = s5k5ca_i2c_write_w(reg_conf_tbl->waddr, reg_conf_tbl->wdata);
        if (rc < 0)
        {
		printk("i2c write break;\n");
            break;
        }

        reg_conf_tbl++;
    }

    return rc;
}

static int s5k5ca_probe_init_sensor(struct i2c_client *client)
{
    int rc;
    unsigned short chipid;


    /* Set the sensor reset when camera is not initialization. */
    if (false == s5k5ca_init_flag)
    {
    }

    mdelay(20);

    /* Set the soft reset to reset the chip and read the chip ID when camera is not initialization. */
    if (false == s5k5ca_init_flag)
    {
        rc = s5k5ca_i2c_write_w(0x0010, 0x0001);
        if (rc < 0)
        {
            CDBG("s5k5ca_i2c_write_w 0x0010 0x0001 rc=%d", rc);
            goto init_probe_fail;
        }

        mdelay(10);

        rc = s5k5ca_i2c_write_w(0x0010, 0x0000);
        if (rc < 0)
        {
            CDBG("s5k5ca_i2c_write_w 0x0010 0x0000 rc=%d", rc);
            goto init_probe_fail;
        }

        mdelay(10);

        rc = s5k5ca_i2c_write_w(0x002c, 0x0000);
        if (rc < 0)
        {
            CDBG("s5k5ca_i2c_write_w 0x002c 0x0000 rc=%d", rc);
            goto init_probe_fail;
        }

        rc = s5k5ca_i2c_write_w(0x002e, 0x0040);
        if (rc < 0)
        {
            CDBG("s5k5ca_i2c_write_w 0x002e 0x0040 rc=%d", rc);
            goto init_probe_fail;
        }

        /* 3. Read sensor Model ID: */
        rc = s5k5ca_i2c_read_w(0x0f12, &chipid);
        if (rc < 0)
        {
            CDBG("s5k5ca_i2c_read_w Model_ID failed!! rc=%d", rc);
            goto init_probe_fail;
        }

        CDBG("s5k5ca chipid = 0x%x\n", chipid);

        /* 4. Compare sensor ID to S5K5CA ID: */
        if (chipid != S5K5CA_CHIP_ID)
        {
            CDBG("s5k5ca Model_ID error!!");
            rc = -ENODEV;
            goto init_probe_fail;
        }

        {
            /* Change the method of reading model id to fit socket and FPC packing models
             * Socket model : 0--3
             * FPC model : 8--11
             */
            rc = s5k5ca_i2c_write_w(0xFCFC, 0xD000);
            if (rc < 0)
            {
                goto init_probe_fail;
            }

            rc = s5k5ca_i2c_write_w(0x108E, 0x3333);
            if (rc < 0)
            {
                goto init_probe_fail;
            }

            mdelay(2);

            rc = s5k5ca_i2c_write_w(0x1090, 0x8888);
            if (rc < 0)
            {
                goto init_probe_fail;
            }

            rc = s5k5ca_i2c_read_w(0x100C, &s5k5ca_model_id);
            if (rc < 0)
            {
                CDBG("s5k5ca_i2c_read_w 0x002e rc=%d", rc);
            }

            CDBG("s5k5ca model = 0x%x\n", s5k5ca_model_id);

            /* If ID out of range ,set model_id MODEL_TRULY_FPC as default */
            if (s5k5ca_model_id > MODEL_SUNNY)
            {
                s5k5ca_model_id = MODEL_SUNNY;
            }

            rc = s5k5ca_i2c_write_w(0x108E, 0x0000);
            if (rc < 0)
            {
                goto init_probe_fail;
            }

            switch (s5k5ca_model_id)
            {
            case MODEL_TRULY:

            case MODEL_SUNNY:
                p_s5k5ca_init_reg_config = (struct s5k5ca_i2c_reg_conf *)(s5k5ca_regs.s5k5ca_init_reg_config_sunny);
                reg_num = s5k5ca_regs.s5k5ca_init_reg_config_sunny_size;

                //              strncpy((char *)data->sensor_name, "23060043SF-SAM-S", strlen("23060043SF-SAM-S"));
                CDBG("s5k5ca probe is  MODEL_SUNNY.");
                break;

            default:
                goto init_probe_fail;
                CDBG("s5k5ca is no this sensor model.\n");
                break;
            }
        }
      //  CDBG("sensor name is %s.", data->sensor_name);
    }

    /*delete one line*/
        //CDBG("s5k5ca  model is %d : init sensor!\n", s5k5ca_model_id);

        /* Write init sensor register */
	//int rc = 0;
     //   rc = s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_init_reg_sensor_start,
       //                               s5k5ca_regs.s5k5ca_init_reg_sensor_start_size);
        mdelay(100);


	rc = s5k5ca_i2c_write_w_table(p_s5k5ca_init_reg_config, reg_num);
       // rc = s5k5ca_i2c_write_w_table(p_s5k5ca_init_reg_config, reg_num);
        /*add a 10ms delay between registers writing*/
        mdelay(10);
        //rc = s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_init_reg_config_sunny_2,  
        //                        s5k5ca_regs.s5k5ca_init_reg_config_sunny_2_size);
        //           mdelay(100);
        //CDBG("s5k5ca model is %d: init sensor done!\n", s5k5ca_model_id);
        if(rc >= 0)
        {
	 	printk("set mirror mode %d\n", reg_num);
		//rc = s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_mirror_mode_reg_config,
                               //      s5k5ca_regs.s5k5ca_mirror_mode_reg_config_size);
           // rc = s5k5ca_set_mirror_mode();
        } 
       // break;

    goto init_probe_done;

init_probe_fail:
		printk("-----ini_probe_fail-------\n");
    //s5k5ca_sensor_init_done(data);
init_probe_done:
    return rc;
}
static int s5k5ca_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	return err;
}

static int s5k5ca_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	int err = 0;

//	s5k5ca_init(sd, 0);
	return err;
}

static int s5k5ca_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	int err = 0;

	return err;
}

static int s5k5ca_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	return err;
}

static enum v4l2_mbus_pixelcode s5k5ca_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
	V4L2_MBUS_FMT_YVYU8_2X8,
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_RGB565_2X8_LE,
};

static int s5k5ca_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{

	if (index >= ARRAY_SIZE(s5k5ca_codes))
		return -EINVAL;

	*code = s5k5ca_codes[index];
	return 0;
}

static int s5k5ca_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	//printk("--------s5k5ca_g_fmt-----\n");
	int err = 0;

	return err;
}

static int s5k5ca_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s: numerator %d, denominator: %d\n", \
		__func__, param->parm.capture.timeperframe.numerator, \
		param->parm.capture.timeperframe.denominator);

	return err;
}

static int s5k5ca_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	return err;
}

static int s5k5ca_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	int err = 0;

	return err;
}

static struct v4l2_queryctrl s5k5ca_controls[] = {
	{
		/*
		 * For now, we just support in preset type
		 * to be close to generic WB system,
		 * we define color temp range for each preset
		 */
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "White balance in kelvin",
		.minimum = 0,
		.maximum = 10000,
		.step = 1,
		.default_value = 0,	/* FIXME */
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = -80,
		.maximum = 80,
		.step = 1,
		.default_value = 0x08,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0x10,
		.maximum = 0x30,
		.step = 1,
		.default_value = 0x20,
	},
	{
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0x00,
		.maximum = 0x80,
		.step = 1,
		.default_value = 0x40,
	},
	{
		.id = V4L2_CID_SHARPNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sharpness",
		.minimum = 0x00,
		.maximum = 0x09,
		.step = 1,
		.default_value = 0x04,
	},
	{
		.id = V4L2_CID_HUE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Hue",
		.minimum = 0,
		.maximum = 21,
		.step = 1,
		.default_value = 10,
	},
	{
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Power line frequency",
		.minimum = 0,
		.maximum = 2,
		.step = 1,
		.default_value = 0,
	},
};

static inline struct v4l2_queryctrl const *s5k5ca_find_qctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s5k5ca_controls); i++)
		if (s5k5ca_controls[i].id == id)
			return &s5k5ca_controls[i];

	return NULL;
}

static int s5k5ca_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s5k5ca_controls); i++) {
		if (s5k5ca_controls[i].id == qc->id) {
			memcpy(qc, &s5k5ca_controls[i], \
				sizeof(struct v4l2_queryctrl));
			return 0;
		}
	}

	return -EINVAL;
}

static int s5k5ca_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err = 0;
	return err;
}
static unsigned char regABVal = 0x06;
static unsigned char hueTable[21][3] = {{0x0, 0x80, 0},
				{0x14, 0x7E, 0},
				{0x27, 0x79, 0},
				{0x3A, 0x72, 0},
				{0x4B, 0x67, 0},
				{0x5A, 0x5A, 0},
				{0x67, 0x4B, 0},
				{0x72, 0x3A, 0},
				{0x79, 0x27, 0},
				{0x7E, 0x14, 0},
				{0x80, 0x00, 1},
				{0x7E, 0x14, 1},
				{0x79, 0x27, 1},
				{0x72, 0x3A, 1},
				{0x67, 0x4B, 1},
				{0x5A, 0x5A, 1},
				{0x4B, 0x67, 1},
				{0x3A, 0x72, 1},
				{0x27, 0x79, 1},
				{0x14, 0x7E, 1},
				{0x00, 0x80, 1}};

void s5k5ca_color_effect(struct v4l2_subdev *sd, int which)
{return;}


static const struct v4l2_subdev_video_ops s5k5ca_video_ops = {
	.s_crystal_freq = s5k5ca_s_crystal_freq,
	.g_mbus_fmt = s5k5ca_g_fmt,
	.s_mbus_fmt = s5k5ca_s_fmt,
	.enum_framesizes = s5k5ca_enum_framesizes,
	.enum_frameintervals = s5k5ca_enum_frameintervals,
	.enum_mbus_fmt = s5k5ca_enum_fmt,
	.try_mbus_fmt = s5k5ca_try_fmt,
	.g_parm = s5k5ca_g_parm,
	.s_parm = s5k5ca_s_parm,
};
static const struct v4l2_subdev_core_ops s5k5ca_core_ops = {
	.init	= s5k5ca_probe_init_sensor,
	.queryctrl = s5k5ca_queryctrl,
	.g_ctrl = s5k5ca_g_ctrl,
	//.s_ctrl = s5k5ca_s_ctrl,
};
static const struct v4l2_subdev_ops s5k5ca_ops = {
	.core = &s5k5ca_core_ops,
	.video = &s5k5ca_video_ops,
};
static int s5k5ca_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long width_flag = flags & SOCAM_DATAWIDTH_MASK;

	/* Only one width bit may be set */
	if (icl->set_bus_param)
		return icl->set_bus_param(icl, width_flag);

	/*
	 * Without board specific bus width settings we support only the
	 * sensors native bus width witch are tested working
	 */
	if (width_flag & (SOCAM_DATAWIDTH_10 | SOCAM_DATAWIDTH_8))
		return 0;

	return 0;
}

static unsigned long s5k5ca_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH;

	if (icl->query_bus_param)
		flags |= icl->query_bus_param(icl) & SOCAM_DATAWIDTH_MASK;
	else
		flags |= SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}
static struct soc_camera_ops s5k5ca_camera_ops = {
	.set_bus_param		= s5k5ca_set_bus_param,
	.query_bus_param	= s5k5ca_query_bus_param,
	.controls		= s5k5ca_controls,
	.num_controls		= ARRAY_SIZE(s5k5ca_controls),
};
#define BURST_MODE_SET			1
#define BURST_MODE_END			2
#define NORMAL_MODE_SET			3
#define MAX_INDEX				1000
static int s5k5ca_sensor_burst_write_list(struct i2c_client *client, struct samsung_short_t *list,char *name)
{
	__u8 temp_buf[MAX_INDEX];
	int index_overflow = 1;
	int new_addr_start = 0;
	int burst_mode = NORMAL_MODE_SET;
	unsigned short pre_subaddr = 0;
	struct i2c_msg msg = { client->addr, 0, 4, temp_buf };
	int i=0, ret=0;
	unsigned int index = 0;
	
	printk("s5k5ca_sensor_burst_write_list( %s ) \n", name); 
	//printk("[PGH] on write func s5k5ca_client->addr : %x\n", client->addr); //reduced startup time.     
    
#ifdef CONFIG_LOAD_FILE 
	s5k5ca_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(list[i].subaddr == 0xdddd)
		{
                        msleep(list[i].value);
			printk("delay 0x%04x, value 0x%04x\n", list[i].subaddr, list[i].value);
		}	
		else
		{			
		//	printk("--------write i2c reg------\n");		
			if( list[i].subaddr == list[i+1].subaddr )
			{
				burst_mode = BURST_MODE_SET;
				if((list[i].subaddr != pre_subaddr) || (index_overflow == 1))
				{
					new_addr_start = 1;
					index_overflow = 0;
				}
			}
			else
			{
				if(burst_mode == BURST_MODE_SET)
				{
					burst_mode = BURST_MODE_END;
					if(index_overflow == 1)
					{
						new_addr_start = 1;
						index_overflow = 0;
					}
				}
				else
				{
					burst_mode = NORMAL_MODE_SET;
				}
			}

			if((burst_mode == BURST_MODE_SET) || (burst_mode == BURST_MODE_END))
			{
				if(new_addr_start == 1)
				{
					index = 0;
					memset(temp_buf, 0x00 ,1000);
					index_overflow = 0;

					temp_buf[index] = (list[i].subaddr >> 8);
					temp_buf[++index] = (list[i].subaddr & 0xFF);

					new_addr_start = 0;
				}
				
				temp_buf[++index] = (list[i].value >> 8);
				temp_buf[++index] = (list[i].value & 0xFF);
				
				if(burst_mode == BURST_MODE_END)
				{
					msg.len = ++index;

					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("i2c_transfer fail ! \n");
						return -1;
					}
				}
				else if( index >= MAX_INDEX-1 )
				{
					index_overflow = 1;
					msg.len = ++index;
					
					ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("I2C_transfer Fail ! \n");
						return -1;
					}
				}
				
			}
			else
			{
				memset(temp_buf, 0x00 ,4);
			
				temp_buf[0] = (list[i].subaddr >> 8);
				temp_buf[1] = (list[i].subaddr & 0xFF);
				temp_buf[2] = (list[i].value >> 8);
				temp_buf[3] = (list[i].value & 0xFF);

				msg.len = 4;
				ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
				if( ret < 0)
				{
					printk("I2C_transfer Fail ! \n");
					return -1;
				}
			}
		}
		
		pre_subaddr = list[i].subaddr;
	}
#endif
	return ret;
}

static int sensor_init(struct i2c_client *client)
{
	int i, size;
	int ret = 0;

	printk("[**ASWOOGI**] sensor_init\n"); 

#ifdef I2C_BURST_MODE //dha23 100325	
	if(s5k5ca_sensor_burst_write_list(client,s5k5ca_init_00,"s5k5ca_init_00") < 0)
		return -1;
#else
	if(s5k5ca_sensor_write_list(client,s5k5ca_init0,"s5k5ca_init0") < 0)
		return -1;

	msleep(100);	

	if(s5k5ca_sensor_write_list(client,s5k5ca_init1,"s5k5ca_init1") < 0)
		return -1;
#endif
	msleep(10);	
	af_mode = -1;

	/* Check Sensor ID */
	//s5k5ca_sensor_get_id(client);
	return 0;
}

#define S5K5CA_PROC_NAME	"s5k5ca"
static int s5k5ca_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("--------s5k5ca_i2c_probe-------\n");
	struct s5k5ca_state *state;
	struct v4l2_subdev *sd;
	struct soc_camera_device  *icd = client->dev.platform_data;
	struct i2c_adapter        *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link    *icl;

	state = kzalloc(sizeof(struct s5k5ca_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, "S5K5CA");

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&adapter->dev,
			"S5K5CA: Missing platform_data for driver\n");
		return -EINVAL;
	}

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5k5ca_ops);

	icd->ops = &s5k5ca_camera_ops;

	s5k5ca_client = client;
	//sensor_init(client);
	s5k5ca_probe_init_sensor(client);

	//s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_preview_reg_config,
    //                                      s5k5ca_regs.s5k5ca_preview_reg_config_size);
#if 0
	s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_init_reg_sensor_start,
                                      s5k5ca_regs.s5k5ca_init_reg_sensor_start_size);
        mdelay(100);

   s5k5ca_i2c_write_w_table(p_s5k5ca_init_reg_config, reg_num);
        /*add a 10ms delay between registers writing*/
        mdelay(10);
   s5k5ca_i2c_write_w_table(s5k5ca_regs.s5k5ca_init_reg_config_sunny_2,  
                                s5k5ca_regs.s5k5ca_init_reg_config_sunny_2_size);
	mdelay(50);
#endif
	dev_info(&client->dev, "s5k5ca has been probed\n");

	return 0;
}
static __exit s5k5ca_sensor_remove(void)
{
	printk("------s5k5ca_sensor_remove-------\n");
	if (s5k5ca_data.sensor != NULL)
		i2c_del_driver(&s5k5ca_driver);
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_exit();
#endif
	i2c_del_driver(&s5k5ca_driver);
//	s3c_camif_remove_sensor(&s5k5ca_input, &s5k5ca_input_handler);
}
static inline struct s5k5ca_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5ca_state, sd);
}
static int __devexit s5k5ca_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
//	if (s_proc != NULL)
	//	remove_proc_entry(GC0308_PROC_NAME, &proc_root);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
    printk("[CAM-SENSOR][s5k5ca] removed\n");
	return 0;
}
static int s5k5ca_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strlcpy(info->type, "S5K5CA", I2C_NAME_SIZE);
	return 0;
}
static struct i2c_device_id s5k5ca_id[] = {
	{ "S5K5CA", 0 },
	{}
};
static struct i2c_driver s5k5ca_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = "S5K5CA",
	},
	.class 		= I2C_CLASS_HWMON,
	.probe		= s5k5ca_i2c_probe,
	.remove		= __devexit_p(s5k5ca_i2c_remove),
	.detect		= s5k5ca_i2c_detect,
	//.command = s5k5ca_sensor_command,
	.id_table	= s5k5ca_id,
//	.address_data	= &s5k5ca_addr_data,
};


#endif 


static __init s5k5ca_sensor_init(void)
{
	printk("--------s5k5ca_sensor_init-------\n");
	return i2c_add_driver(&s5k5ca_driver);
}

static void s5k5ca_sensor_exit(void)
{
	s5k5ca_sensor_disable();

#ifdef CONFIG_LOAD_FILE
	s5k5ca_regs_table_exit();
#endif
	
//	if (s5k5ca_data.sensor != NULL)
	//	s3c_camif_unregister_sensor(&s5k5ca_data);
}

static struct v4l2_input s5k5ca_input = {
	.index		= 0,
	.name		= "Camera Input (S5K5CA)",
	.type		= V4L2_INPUT_TYPE_CAMERA,
	.audioset	= 1,
	.tuner		= 0,
	.std		= V4L2_STD_PAL_BG | V4L2_STD_NTSC_M,
	.status		= 0,
};

static struct v4l2_input_handler s5k5ca_input_handler = {
	s5k5ca_sensor_init,
	s5k5ca_sensor_exit	
};

int s5k5ca_sensor_add(void)
{
#ifdef CONFIG_FLASH_AAT1271A
	aat1271a_flash_init();
#endif	
#ifdef LOCAL_CONFIG_S5K5CA_I2C_TEST
	return s5k5ca_sensor_init();
#else
	//return s3c_camif_add_sensor(&s5k5ca_input, &s5k5ca_input_handler);
	return 0;
#endif
}


module_init(s5k5ca_sensor_init)
module_exit(s5k5ca_sensor_remove)
