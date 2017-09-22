/*
 * OmniVision OV96xx Camera Driver
 *
 * Copyright (C) 2009 Marek Vasut <marek.vasut@gmail.com>
 *
 * Based on ov772x camera driver:
 *
 * Copyright (C) 2008 Renesas Solutions Corp.
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * Based on ov7670 and soc_camera_platform driver,
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 * Copyright (C) 2008 Magnus Damm
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

#include "ls1c-ov9650.h"

#define to_ov9650_sensor(sd)	container_of(sd, struct ov9650_priv, subdev)


static const struct ov9650_reg ov9650_regs_dflt[] = {                   /* yuv  vga */
	{0x12, 0x80}, {0x11, 0x81}, {0x6b, 0x0a}, {0x6a, 0x3e}, {0x3b, 0x09},
	{0x13, 0xe0}, {0x01, 0x80}, {0x02, 0x80}, {0x00, 0x00}, {0x10, 0x00},
	{0x13, 0xe5}, 
	{0x39, 0x43}, {0x38, 0x12}, {0x37, 0x91}, {0x35, 0x91},
	{0x0e, 0x20}, {0x1e, 0x04},
	{0xA8, 0x80}, {0x12, 0x40}, {0x04, 0x00}, {0x0c, 0x04}, {0x0d, 0x80},   // reg0d: 0x80
	{0x18, 0xc6}, {0x17, 0x26}, {0x32, 0xad}, {0x03, 0x00}, {0x1a, 0x3d},   //reg18: 0xba, reg17:0x1a  {0x18, 0xc6}, {0x17, 0x26},  // {0x18, 0xba}, {0x17, 0x1a},
	{0x19, 0x01}, {0x3f, 0xa6}, {0x14, 0x2e}, {0x15, 0x02}, {0x41, 0x02},   //reg15: bit0:hsync  bit1:vsync  negative
	{0x42, 0x08}, 
	{0x1b, 0x00}, {0x16, 0x06}, {0x33, 0xe2}, {0x34, 0xbf},
	{0x96, 0x04}, {0x3a, 0x00}, {0x8e, 0x00},
	{0x3c, 0x77}, {0x8B, 0x06}, {0x94, 0x88}, {0x95, 0x88}, {0x40, 0xc1},
	{0x29, 0x3f}, {0x0f, 0x42},
	{0x3d, 0x92}, {0x69, 0x40}, {0x5C, 0xb9}, {0x5D, 0x96}, {0x5E, 0x10},
	{0x59, 0xc0}, {0x5A, 0xaf}, {0x5B, 0x55}, {0x43, 0xf0}, {0x44, 0x10},
	{0x45, 0x68}, {0x46, 0x96}, {0x47, 0x60}, {0x48, 0x80}, {0x5F, 0xe0},
	{0x60, 0x8c}, /* 0c for advanced AWB (related to lens) */
	{0x61, 0x20}, {0xa5, 0xd9}, {0xa4, 0x74}, {0x8d, 0x02}, {0x13, 0xe7},
	{0x4f, 0x3a}, {0x50, 0x3d}, {0x51, 0x03}, {0x52, 0x12}, {0x53, 0x26},
	{0x54, 0x38}, {0x55, 0x40}, {0x56, 0x40}, {0x57, 0x40}, {0x58, 0x0d},
	{0x8C, 0x23}, {0x3E, 0x02}, {0xa9, 0xb8}, {0xaa, 0x92}, {0xab, 0x0a},
	{0x8f, 0xdf}, {0x90, 0x00}, {0x91, 0x00}, {0x9f, 0x00}, {0xa0, 0x00},
	{0x3A, 0x01},  //change 
	{0x24, 0x70}, {0x25, 0x64}, {0x26, 0xc3},
	{0x2a, 0x00}, /* 10 for 50Hz */ 
	{0x2b, 0x00}, /* 40 for 50Hz */
	//;gamma
	{0x6c, 0x40}, {0x6d, 0x30}, {0x6e, 0x4b}, {0x6f, 0x60}, {0x70, 0x70},
	{0x71, 0x70}, {0x72, 0x70}, {0x73, 0x70}, {0x74, 0x60}, {0x75, 0x60},
	{0x76, 0x50}, {0x77, 0x48}, {0x78, 0x3a}, {0x79, 0x2e}, {0x7a, 0x28},
	{0x7b, 0x22}, {0x7c, 0x04}, {0x7d, 0x07}, {0x7e, 0x10}, {0x7f, 0x28},
	{0x80, 0x36}, {0x81, 0x44}, {0x82, 0x52}, {0x83, 0x60}, {0x84, 0x6c},
	{0x85, 0x78}, {0x86, 0x8c}, {0x87, 0x9e}, {0x88, 0xbb}, {0x89, 0xd2},
	{0x8a, 0xe6}
};



/* Configurations
 * NOTE: for YUV, alter the following registers:
 * 		COM12 |= OV9650_COM12_YUV_AVG
 *
 *	 for RGB, alter the following registers:
 *		COM7  |= OV9650_COM7_RGB
 *		COM13 |= OV9650_COM13_RGB_AVG
 *		COM15 |= proper RGB color encoding mode
 */
static const struct ov9650_reg ov9650_regs_qqcif[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x0f) },
	{ OV9650_COM1,	OV9650_COM1_QQFMT | OV9650_COM1_HREF_2SKIP },
	{ OV9650_COM4,	OV9650_COM4_QQ_VP | OV9650_COM4_RSVD },
	{ OV9650_COM7,	OV9650_COM7_QCIF },
	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};

static const struct ov9650_reg ov9650_regs_qqvga[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x07) },
	{ OV9650_COM1,	OV9650_COM1_QQFMT | OV9650_COM1_HREF_2SKIP },
	{ OV9650_COM4,	OV9650_COM4_QQ_VP | OV9650_COM4_RSVD },
	{ OV9650_COM7,	OV9650_COM7_QVGA },
	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};

static const struct ov9650_reg ov9650_regs_qcif[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x07) },
	{ OV9650_COM4,	OV9650_COM4_QQ_VP | OV9650_COM4_RSVD },
	{ OV9650_COM7,	OV9650_COM7_QCIF },
	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};

static const struct ov9650_reg ov9650_regs_qvga[] = {
	{0x04, 0x00},
	{0x0c, 0x04},
	{0x0d, 0x80},
	{0x11, 0x83},
	{0x12, 0x10},
	{0x37, 0x91},
	{0x38, 0x12},
	{0x39, 0x43},
	/*
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x03) },
	{ OV9650_COM4,	OV9650_COM4_QQ_VP | OV9650_COM4_RSVD },
	{ OV9650_COM7,	OV9650_COM7_QVGA },
//	{ OV9650_COM12,	OV9650_COM12_RSVD }, { OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
	*/
};

static const struct ov9650_reg ov9650_regs_cif[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x03) },
	{ OV9650_COM3,	OV9650_COM3_VP },
	{ OV9650_COM7,	OV9650_COM7_CIF },
//	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};

#if 0
static const struct ov9650_reg ov9650_regs_vga[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x01) },  //enable doublle clock, div_2
	{ OV9650_COM3,	OV9650_COM3_VP },
	{ OV9650_COM7,	OV9650_COM7_VGA },
	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};
#else
static const struct ov9650_reg ov9650_regs_vga[] = {
	{ 0x04, 0x00 },
	{ 0x0c, 0x04 },
	{ 0x0d, 0x80 },
	{ 0x11, 0x81 },
	{ 0x12, 0x40 },
	{ 0x37, 0x91 },
	{ 0x38, 0x12 },
	{ 0x39, 0x43 },
	//{ OV9650_COM12,	OV9650_COM12_RSVD },
//	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
//	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};
#endif
static const struct ov9650_reg ov9650_regs_sxga[] = {
	{ OV9650_CLKRC,	OV9650_CLKRC_DPLL_EN | OV9650_CLKRC_DIV(0x01) },
	{ OV9650_COM3,	OV9650_COM3_VP },
	{ OV9650_COM7,	0 },
	{ OV9650_COM12,	OV9650_COM12_RSVD },
	{ OV9650_COM13,	OV9650_COM13_GAMMA_RAW | OV9650_COM13_MATRIX_EN },
	{ OV9650_COM15,	OV9650_COM15_OR_10F0 },
};


static const struct ov9650_reg ov9650_regs_yuv[] = {
	{ OV9650_MTX1,	0x58 },
	{ OV9650_MTX2,	0x48 },
	{ OV9650_MTX3,	0x10 },
	{ OV9650_MTX4,	0x28 },
	{ OV9650_MTX5,	0x48 },
	{ OV9650_MTX6,	0x70 },
	{ OV9650_MTX7,	0x40 },
	{ OV9650_MTX8,	0x40 },
	{ OV9650_MTX9,	0x40 },
	{ OV9650_MTXS,	0x0f },
};

static const struct ov9650_reg ov9650_regs_rgb[] = {
	{ OV9650_MTX1,	0x71 },
	{ OV9650_MTX2,	0x3e },
	{ OV9650_MTX3,	0x0c },
	{ OV9650_MTX4,	0x33 },
	{ OV9650_MTX5,	0x72 },
	{ OV9650_MTX6,	0x00 },
	{ OV9650_MTX7,	0x2b },
	{ OV9650_MTX8,	0x66 },
	{ OV9650_MTX9,	0xd2 },
	{ OV9650_MTXS,	0x65 },
};

static enum v4l2_mbus_pixelcode ov9650_codes[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,  // dbg-yg add
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_RGB565_2X8_LE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
};

static const struct v4l2_queryctrl ov9650_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
	{
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
};



/***************************************************************************
 * Description: SCCB / i2c  for ov9650 
 * Parameters: 
 * Author  :Sunyoung_yg 
 * Date    : 2014-12-24
 ***************************************************************************/
#define I2C_CLOCK		400000		/* Hz. max 400 Kbits/sec */

/* registers */
#define OCI2C_PRELOW		0
#define OCI2C_PREHIGH		1
#define OCI2C_CONTROL		2
#define OCI2C_DATA			3
#define OCI2C_CMD			4 /* write only */
#define OCI2C_STATUS		4 /* read only, same address as OCI2C_CMD */

#define OCI2C_CTRL_IEN		0x40
#define OCI2C_CTRL_EN		0x80

#define OCI2C_CMD_START		0x90
#define OCI2C_CMD_STOP		0x40
#define OCI2C_CMD_READ		0x20
#define OCI2C_CMD_WRITE		0x10
#define OCI2C_CMD_READ_ACK	0x20
#define OCI2C_CMD_READ_NACK	0x28
#define OCI2C_CMD_IACK		0x00

#define OCI2C_STAT_IF		0x01
#define OCI2C_STAT_TIP		0x02
#define OCI2C_STAT_ARBLOST	0x20
#define OCI2C_STAT_BUSY		0x40
#define OCI2C_STAT_NACK		0x80



struct ls1x_i2c {
	void __iomem *base;
	struct i2c_adapter adap;
};

static inline void i2c_writeb(struct ls1x_i2c *i2c, int reg, u8 value)
{
	writeb(value, i2c->base + reg);
}

static inline u8 i2c_readb(struct ls1x_i2c *i2c, int reg)
{
	return readb(i2c->base + reg);
}


static void ls1x_i2c_hwinit(struct ls1x_i2c *i2c)
{
	struct clk *clk;
	int prescale;
	u8 ctrl = i2c_readb(i2c, OCI2C_CONTROL);
	/* make sure the device is disabled */
	i2c_writeb(i2c, OCI2C_CONTROL, ctrl & ~(OCI2C_CTRL_EN|OCI2C_CTRL_IEN));

	prescale = 0x141;
	i2c_writeb(i2c, OCI2C_PRELOW, prescale & 0xff);
	i2c_writeb(i2c, OCI2C_PREHIGH, prescale >> 8);

	/* Init the device */
	i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_IACK);
	i2c_writeb(i2c, OCI2C_CONTROL, ctrl | OCI2C_CTRL_IEN | OCI2C_CTRL_EN);
}


/*
 * Poll the i2c status register until the specified bit is set.
 * Returns 0 if timed out (100 msec).
 */
static short ls1x_poll_status(struct ls1x_i2c *i2c, unsigned long bit)
{
	int loop_cntr = 10000;
	do {
		udelay(10);
	} while ((i2c_readb(i2c, OCI2C_STATUS) & bit) && (--loop_cntr > 0));
	return (loop_cntr > 0);
}

/***************************************************************/
static int ls1x_xfer_read(struct ls1x_i2c *i2c, unsigned char *buf, int length) 
{
	int x;
	for (x=0; x<length; x++) {
		/* send ACK last not send ACK */
		if (x != (length -1)) 
			i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_READ_NACK); //dbg-yg   NACK test for ov9650 camera, 
		else
			i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_READ_NACK);
		if (!ls1x_poll_status(i2c, OCI2C_STAT_TIP)) {
			return -ETIMEDOUT;
		}
		*buf++ = i2c_readb(i2c, OCI2C_DATA);
	}
	i2c_writeb(i2c,OCI2C_CMD, OCI2C_CMD_STOP);
		
	return 0;
}

/***************************************************************/
static int ls1x_xfer_write(struct ls1x_i2c *i2c, unsigned char *buf, int length)
{
	int x;

	for (x=0; x<length; x++) {
		i2c_writeb(i2c, OCI2C_DATA, *buf++);
		i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_WRITE);
		if (!ls1x_poll_status(i2c, OCI2C_STAT_TIP)) {
			return -ETIMEDOUT;
		}
	}
	i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_STOP);
	return 0;
}

static int ls1x_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{

	struct ls1x_i2c *i2c = (struct ls1x_i2c *)adap->algo_data;
	int i, ret;

	for (i = 0; i < num; i++) {
		if (!ls1x_poll_status(i2c, OCI2C_STAT_BUSY)) {
			return -ETIMEDOUT;
		}

		i2c_writeb(i2c, OCI2C_DATA, (pmsg->addr << 1)
			| ((pmsg->flags & I2C_M_RD) ? 1 : 0));
		i2c_writeb(i2c, OCI2C_CMD, OCI2C_CMD_START);
		/* Wait until transfer is finished */
		if (!ls1x_poll_status(i2c, OCI2C_STAT_TIP)) {
			return -ETIMEDOUT;
		}
 		if (pmsg->flags & I2C_M_RD)
			ret = ls1x_xfer_read(i2c, pmsg->buf, pmsg->len);
  		else
			ret = ls1x_xfer_write(i2c, pmsg->buf, pmsg->len);
		if (ret)
			return ret;
		pmsg++;
	}
	return i;
}

/* read a register */
static int ov9650_reg_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	u8 data = reg;
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &data,
	};
	ret = ls1x_xfer(client->adapter, &msg, 1);

	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	ret = ls1x_xfer(client->adapter, &msg, 1);

	if (ret < 0)
		goto err;

	*val = data;
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/* write a register */
static int ov9650_reg_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	u8 _val;
	unsigned char data[2] = { reg, val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	ret = ls1x_xfer(client->adapter, &msg, 1);
	//ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	/* we have to read the register back ... no idea why, maybe HW bug */
	ret = ov9650_reg_read(client, reg, &_val);
	if (ret)
		dev_err(&client->dev,
			"Failed reading back register 0x%02x!\n", reg);

	return 0;
}


/* Read a register, alter its bits, write it back */
static int ov9650_reg_rmw(struct i2c_client *client, u8 reg, u8 set, u8 unset)
{
	u8 val;
	int ret;

	ret = ov9650_reg_read(client, reg, &val);
	if (ret) {
		dev_err(&client->dev,
			"[Read]-Modify-Write of register %02x failed!\n", reg);
		return val;
	}

	val |= set;
	val &= ~unset;

	ret = ov9650_reg_write(client, reg, val);
	if (ret)
		dev_err(&client->dev,
			"Read-Modify-[Write] of register %02x failed!\n", reg);

	return ret;
}

/* Soft reset the camera. This has nothing to do with the RESET pin! */
static int ov9650_reset(struct i2c_client *client)
{
	int ret;

	ret = ov9650_reg_write(client, OV9650_COM7, OV9650_COM7_SCCB_RESET);
	if (ret)
		dev_err(&client->dev,
			"An error occurred while entering soft reset!\n");

	return ret;
}

/* Start/Stop streaming from the device */
static int ov9650_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

/* Alter bus settings on camera side */
static int ov9650_set_bus_param(struct soc_camera_device *icd,
				unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long ov9650_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	/*
	 * REVISIT: the camera probably can do 10 bit transfers, but I don't
	 *          have those pins connected on my hardware.
	 */
	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8;

	return soc_camera_apply_sensor_flags(icl, flags);
}

/* Get status of additional camera capabilities */
static int ov9650_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov9650_priv *priv = to_ov9650_sensor(sd);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		ctrl->value = priv->flag_vflip;
		break;
	case V4L2_CID_HFLIP:
		ctrl->value = priv->flag_hflip;
		break;
	}
	return 0;
}

/* Set status of additional camera capabilities */
static int ov9650_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9650_priv *priv = to_ov9650_sensor(sd);

	int ret = 0;
	printk("oooooooooooooooooooo ov9650_s_ctrl oooooooooooooooooooo\r\n");
	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		priv->flag_vflip = ctrl->value;
		if (ctrl->value)
			ret = ov9650_reg_rmw(client, OV9650_MVFP,
							OV9650_MVFP_V, 0);
		else
			ret = ov9650_reg_rmw(client, OV9650_MVFP,
							0, OV9650_MVFP_V);
		break;
	case V4L2_CID_HFLIP:
		priv->flag_hflip = ctrl->value;
		if (ctrl->value)
			ret = ov9650_reg_rmw(client, OV9650_MVFP,
							OV9650_MVFP_H, 0);
		else
			ret = ov9650_reg_rmw(client, OV9650_MVFP,
							0, OV9650_MVFP_H);
		break;
	}

	return ret;
}

/* Get chip identification */
static int ov9650_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct ov9650_priv *priv = to_ov9650_sensor(sd);

	id->ident	= priv->model;
	id->revision	= priv->revision;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov9650_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xff)
		return -EINVAL;

	reg->size = 1;

	ret = ov9650_reg_read(client, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int ov9650_set_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xff || reg->val & ~0xff)
		return -EINVAL;

	return ov9650_reg_write(client, reg->reg, reg->val);
}
#endif

/* select nearest higher resolution for capture */
static void ov9650_res_roundup(u32 *width, u32 *height)
{
	int i;
	enum { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
	int res_x[] = { 88, 160, 176, 320, 352, 640, 800, 1280 };
	int res_y[] = { 72, 120, 144, 240, 288, 480, 480, 960 };

	for (i = 0; i < ARRAY_SIZE(res_x); i++) {
		if (res_x[i] >= *width && res_y[i] >= *height) {
			*width = res_x[i];
			*height = res_y[i];
			return;
		}
	}

	*width = res_x[SXGA];
	*height = res_y[SXGA];
}

/* Prepare necessary register changes depending on color encoding */
static void ov9650_alter_regs(enum v4l2_mbus_pixelcode code,
			      struct ov9650_reg_alt *alt)
{
	switch (code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		alt->com12	= OV9650_COM12_YUV_AVG;
		alt->com13	= OV9650_COM13_Y_DELAY_EN |
					OV9650_COM13_YUV_DLY(0x01);
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		printk(" mbusf fmt yuyv 2x8 ...\r\n");
		alt->com12	= OV9650_COM12_YUV_AVG;
		alt->com13	= OV9650_COM13_Y_DELAY_EN |
					OV9650_COM13_YUV_DLY(0x01);
		break;
	case V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE:
		alt->com7	= OV9650_COM7_RGB;
		alt->com13	= OV9650_COM13_RGB_AVG;
		alt->com15	= OV9650_COM15_RGB_555;
		break;
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		alt->com7	= OV9650_COM7_RGB;
//		alt->com13	= OV9650_COM13_RGB_AVG;   // no have this bit
		alt->com15	= OV9650_COM15_RGB_565;
		break;
	default:
		break;
	};
}

/* Setup registers according to resolution and color encoding */
static int ov9650_write_regs(struct i2c_client *client, u32 width,
		enum v4l2_mbus_pixelcode code, struct ov9650_reg_alt *alts)
{
	const struct ov9650_reg	*ov9650_regs, *matrix_regs;
	int			ov9650_regs_len, matrix_regs_len;
	int			i, ret;
	u8			val;

	/* select register configuration for given resolution */
	switch (width) {
	case W_QQCIF:
		ov9650_regs	= ov9650_regs_qqcif;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_qqcif);
		break;
	case W_QQVGA:
		ov9650_regs	= ov9650_regs_qqvga;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_qqvga);
		break;
	case W_QCIF:
		ov9650_regs	= ov9650_regs_qcif;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_qcif);
		break;
	case W_QVGA:
		ov9650_regs	= ov9650_regs_qvga;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_qvga);
		break;
	case W_CIF:
		ov9650_regs	= ov9650_regs_cif;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_cif);
		break;
	case W_VGA:
		ov9650_regs	= ov9650_regs_vga;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_vga);
		break;
	case W_SXGA:
		ov9650_regs	= ov9650_regs_sxga;
		ov9650_regs_len	= ARRAY_SIZE(ov9650_regs_sxga);
		break;
	default:
		dev_err(&client->dev, "Failed to select resolution!\n");
		return -EINVAL;
	}
#if 1  //dbg-yg ov9650
	/* select color matrix configuration for given color encoding */
	if (code == V4L2_MBUS_FMT_UYVY8_2X8 || V4L2_MBUS_FMT_YUYV8_2X8) {
		matrix_regs	= ov9650_regs_yuv;
		matrix_regs_len	= ARRAY_SIZE(ov9650_regs_yuv);
	} else {
		matrix_regs	= ov9650_regs_rgb;
		matrix_regs_len	= ARRAY_SIZE(ov9650_regs_rgb);
	}
	/* write register settings into the module */
	for (i = 0; i < ov9650_regs_len; i++) {
		val = ov9650_regs[i].val;

		switch (ov9650_regs[i].reg) {
		case OV9650_COM7:
			val |= alts->com7;
			break;
		case OV9650_COM12:
			val |= alts->com12;
			break;
		case OV9650_COM13:
			val |= alts->com13;
			break;
		case OV9650_COM15:
			val |= alts->com15;
			break;
		}
		ret = ov9650_reg_write(client, ov9650_regs[i].reg, val);
		if (ret)
			return ret;

#endif 
	}
	
#if 1
	/* write color matrix configuration into the module */
	for (i = 0; i < matrix_regs_len; i++) {
		ret = ov9650_reg_write(client, matrix_regs[i].reg,
						matrix_regs[i].val);
		if (ret)
			return ret;
	}
#endif
	return 0;
}

/* program default register values */
static int ov9650_prog_dflt(struct i2c_client *client)
{
	int i, ret;

//	printk("oooooooooooooooooooo ov9650_prog_dflt oooooooooooooooooooo\r\n");
	for (i = 0; i < ARRAY_SIZE(ov9650_regs_dflt); i++) {
		ret = ov9650_reg_write(client, ov9650_regs_dflt[i].reg,
						ov9650_regs_dflt[i].val);
		if (ret)
			return ret;
	}

	/* wait for the changes to actually happen, 140ms are not enough yet */
	mdelay(150);

	return 0;
}

/* set the format we will capture in */
static int ov9650_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov9650_reg_alt alts = {0};
	enum v4l2_colorspace cspace;
	enum v4l2_mbus_pixelcode code = mf->code;
	int ret;

//	printk("oooooooooooooooooooo ov9650_s_fmt  oooooooooooooooooooo\r\n");
//	ov9650_res_roundup(&mf->width, &mf->height);
//	ov9650_alter_regs(mf->code, &alts);

//	ov9650_reset(client);

	ret = ov9650_prog_dflt(client);  //write default data for ov9650
	if (ret)
		return ret;
	switch (code) {
	case V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		cspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		cspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		cspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		code = V4L2_MBUS_FMT_UYVY8_2X8;
		cspace = V4L2_COLORSPACE_JPEG;
		break;
	}

	ret = ov9650_write_regs(client, mf->width, code, &alts);  //dbg-yg ov9650
	if (!ret) {
		mf->code	= code;
		mf->colorspace	= cspace;
	}
	return ret;
}

static int ov9650_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	ov9650_res_roundup(&mf->width, &mf->height);

	mf->field = V4L2_FIELD_NONE;
	switch (mf->code) {
	case V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		mf->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
	//	mf->code = V4L2_MBUS_FMT_UYVY8_2X8;
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;

	}

	return 0;
}

static int ov9650_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov9650_codes))
		return -EINVAL;

	*code = ov9650_codes[index];
	return 0;
}

static int ov9650_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left	= 0;
	a->c.top	= 0;
	a->c.width	= W_SXGA;
	a->c.height	= H_SXGA;
	a->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ov9650_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= W_SXGA;
	a->bounds.height		= H_SXGA;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}



static int ov9650_video_probe(struct soc_camera_device *icd,
				struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9650_priv *priv = to_ov9650_sensor(sd);
	u8		pid, ver, midh, midl;
	const char	*devname;
	int		ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface) {
		dev_err(&client->dev, "Parent missing or invalid!\n");
		ret = -ENODEV;
		goto err;
	}

	/*
	 * check and show product ID and manufacturer ID
	 */

	ret = ov9650_reg_read(client, OV9650_PID, &pid);
	if (ret)
		goto err;

	printk(" CAMERA PID IS  :0x%x &&&&&&&&&&&&&&&&&&\r\n", pid);
#if 1
	ret = ov9650_reg_read(client, OV9650_VER, &ver);
	if (ret)
		goto err;

	ret = ov9650_reg_read(client, OV9650_MIDH, &midh);
	if (ret)
		goto err;

	ret = ov9650_reg_read(client, OV9650_MIDL, &midl);
	if (ret)
		goto err;
#endif 
	printk(" version:0x%x...\r\n", VERSION(pid, ver)); 
	switch (VERSION(pid, ver)) {
	case OV9650_V2:
		devname		= "ov9650";
		priv->model	= V4L2_IDENT_OV9650;
		priv->revision	= 2;
	case OV9650_V3:
		devname		= "ov9650";
		priv->model	= V4L2_IDENT_OV9650;
		priv->revision	= 3;
		break;
	case OV9650_V4:
		devname		= "OV9650";
		priv->model	= V4L2_IDENT_OV9650;
		priv->revision	= 4;
		break;
	default:
		dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
		ret = -ENODEV;
		goto err;
	}

	dev_info(&client->dev, "%s Product ID %0x:%0x Manufacturer ID %x:%x\n",
		 devname, pid, ver, midh, midl);

err:
	return ret;
}

static struct soc_camera_ops ov9650_ops = {
	.set_bus_param		= ov9650_set_bus_param,
	.query_bus_param	= ov9650_query_bus_param,
	.controls		= ov9650_controls,
	.num_controls		= ARRAY_SIZE(ov9650_controls),
};

static struct v4l2_subdev_core_ops ov9650_core_ops = {
	.g_ctrl			= ov9650_g_ctrl,
	.s_ctrl			= ov9650_s_ctrl,
	.g_chip_ident		= ov9650_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register		= ov9650_get_register,
	.s_register		= ov9650_set_register,
#endif

};

static struct v4l2_subdev_video_ops ov9650_video_ops = {
	.s_stream	= ov9650_s_stream,
	.s_mbus_fmt	= ov9650_s_fmt,
	.try_mbus_fmt	= ov9650_try_fmt,
	.enum_mbus_fmt	= ov9650_enum_fmt,
	.cropcap	= ov9650_cropcap,
	.g_crop		= ov9650_g_crop,

};

static struct v4l2_subdev_ops ov9650_subdev_ops = {
	.core	= &ov9650_core_ops,
	.video	= &ov9650_video_ops,
};

/*
 * i2c_driver function
 */
static int ov9650_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov9650_priv *priv;
	struct soc_camera_device *icd	= client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;
//	dump_stack();
	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(struct ov9650_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev,
			"Failed to allocate memory for private data!\n");
		return -ENOMEM;
	}
	
	v4l2_i2c_subdev_init(&priv->subdev, client, &ov9650_subdev_ops);

	icd->ops	= &ov9650_ops;

	ret = ov9650_video_probe(icd, client);

	if (ret) {
		icd->ops = NULL;
		kfree(priv);
	}

	return ret;
}

static int ov9650_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9650_priv *priv = to_ov9650_sensor(sd);

	kfree(priv);
	return 0;
}

static const struct i2c_device_id ov9650_id[] = {
	{ "OV9650", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9650_id);

static struct i2c_driver ov9650_i2c_driver = {
	.driver = {
		.name = "OV9650",
	},
	.probe    = ov9650_probe,
	.remove   = ov9650_remove,
	.id_table = ov9650_id,
};

static int __init ov9650_module_init(void)
{
	return i2c_add_driver(&ov9650_i2c_driver);
}

static void __exit ov9650_module_exit(void)
{
	i2c_del_driver(&ov9650_i2c_driver);
}

module_init(ov9650_module_init);
module_exit(ov9650_module_exit);

MODULE_DESCRIPTION("SoC Camera driver for OmniVision OV96xx");
MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_LICENSE("GPL v2");
