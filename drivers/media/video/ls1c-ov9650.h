/*
 * OmniVision OV96xx Camera Header File
 *
 * Copyright (C) 2009 Marek Vasut <marek.vasut@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef	__DRIVERS_MEDIA_VIDEO_OV9650_H__
#define	__DRIVERS_MEDIA_VIDEO_OV9650_H__

/* Register definitions */
#define	OV9650_GAIN	0x00
#define	OV9650_BLUE	0x01
#define	OV9650_RED	0x02
#define	OV9650_VFER	0x03
#define	OV9650_COM1	0x04
#define	OV9650_BAVE	0x05
#define	OV9650_GEAVE	0x06
#define	OV9650_RSID	0x07
#define	OV9650_RAVE	0x08
#define	OV9650_COM2	0x09
#define	OV9650_PID	0x0a
#define	OV9650_VER	0x0b
#define	OV9650_COM3	0x0c
#define	OV9650_COM4	0x0d
#define	OV9650_COM5	0x0e
#define	OV9650_COM6	0x0f
#define	OV9650_AECH	0x10
#define	OV9650_CLKRC	0x11
#define	OV9650_COM7	0x12
#define	OV9650_COM8	0x13
#define	OV9650_COM9	0x14
#define	OV9650_COM10	0x15
/* 0x16 - RESERVED */
#define	OV9650_HSTART	0x17
#define	OV9650_HSTOP	0x18
#define	OV9650_VSTART	0x19
#define	OV9650_VSTOP	0x1a
#define	OV9650_PSHFT	0x1b
#define	OV9650_MIDH	0x1c
#define	OV9650_MIDL	0x1d
#define	OV9650_MVFP	0x1e
#define	OV9650_LAEC	0x1f
#define	OV9650_BOS	0x20
#define	OV9650_GBOS	0x21
#define	OV9650_GROS	0x22
#define	OV9650_ROS	0x23
#define	OV9650_AEW	0x24
#define	OV9650_AEB	0x25
#define	OV9650_VPT	0x26
#define	OV9650_BBIAS	0x27
#define	OV9650_GBBIAS	0x28
/* 0x29 - RESERVED */
#define	OV9650_EXHCH	0x2a
#define	OV9650_EXHCL	0x2b
#define	OV9650_RBIAS	0x2c
#define	OV9650_ADVFL	0x2d
#define	OV9650_ADVFH	0x2e
#define	OV9650_YAVE	0x2f
#define	OV9650_HSYST	0x30
#define	OV9650_HSYEN	0x31
#define	OV9650_HREF	0x32
#define	OV9650_CHLF	0x33
#define	OV9650_ARBLM	0x34
/* 0x35..0x36 - RESERVED */
#define	OV9650_ADC	0x37
#define	OV9650_ACOM	0x38
#define	OV9650_OFON	0x39
#define	OV9650_TSLB	0x3a
#define	OV9650_COM11	0x3b
#define	OV9650_COM12	0x3c
#define	OV9650_COM13	0x3d
#define	OV9650_COM14	0x3e
#define	OV9650_EDGE	0x3f
#define	OV9650_COM15	0x40
#define	OV9650_COM16	0x41
#define	OV9650_COM17	0x42
/* 0x43..0x4e - RESERVED */
#define	OV9650_MTX1	0x4f
#define	OV9650_MTX2	0x50
#define	OV9650_MTX3	0x51
#define	OV9650_MTX4	0x52
#define	OV9650_MTX5	0x53
#define	OV9650_MTX6	0x54
#define	OV9650_MTX7	0x55
#define	OV9650_MTX8	0x56
#define	OV9650_MTX9	0x57
#define	OV9650_MTXS	0x58
/* 0x59..0x61 - RESERVED */
#define	OV9650_LCC1	0x62
#define	OV9650_LCC2	0x63
#define	OV9650_LCC3	0x64
#define	OV9650_LCC4	0x65
#define	OV9650_LCC5	0x66
#define	OV9650_MANU	0x67
#define	OV9650_MANV	0x68
#define	OV9650_HV	0x69
#define	OV9650_MBD	0x6a
#define	OV9650_DBLV	0x6b
#define	OV9650_GSP	0x6c	/* ... till 0x7b */
#define	OV9650_GST	0x7c	/* ... till 0x8a */

#define	OV9650_CLKRC_DPLL_EN	0x80
#define	OV9650_CLKRC_DIRECT	0x40
#define	OV9650_CLKRC_DIV(x)	((x) & 0x3f)

#define	OV9650_PSHFT_VAL(x)	((x) & 0xff)

#define	OV9650_ACOM_2X_ANALOG	0x80
#define	OV9650_ACOM_RSVD	0x12

#define	OV9650_MVFP_V		0x10
#define	OV9650_MVFP_H		0x20

#define	OV9650_COM1_HREF_NOSKIP	0x00
#define	OV9650_COM1_HREF_2SKIP	0x04
#define	OV9650_COM1_HREF_3SKIP	0x08
#define	OV9650_COM1_QQFMT	0x20

#define	OV9650_COM2_SSM		0x10

#define	OV9650_COM3_VP		0x04

#define	OV9650_COM4_QQ_VP	0x80
#define	OV9650_COM4_RSVD	0x40

#define	OV9650_COM5_SYSCLK	0x80
#define	OV9650_COM5_LONGEXP	0x01

#define	OV9650_COM6_OPT_BLC	0x40
#define	OV9650_COM6_ADBLC_BIAS	0x08
#define	OV9650_COM6_FMT_RST	0x82
#define	OV9650_COM6_ADBLC_OPTEN	0x01

#define	OV9650_COM7_RAW_RGB	0x01
#define	OV9650_COM7_RGB		0x04
#define	OV9650_COM7_QCIF	0x08
#define	OV9650_COM7_QVGA	0x10
#define	OV9650_COM7_CIF		0x20
#define	OV9650_COM7_VGA		0x40
#define	OV9650_COM7_SCCB_RESET	0x80

#define	OV9650_TSLB_YVYU_YUYV	0x04
#define	OV9650_TSLB_YUYV_UYVY	0x08

#define	OV9650_COM12_YUV_AVG	0x04
#define	OV9650_COM12_RSVD	0x40

#define	OV9650_COM13_GAMMA_NONE	0x00
#define	OV9650_COM13_GAMMA_Y	0x40
#define	OV9650_COM13_GAMMA_RAW	0x80
#define	OV9650_COM13_RGB_AVG	0x20
#define	OV9650_COM13_MATRIX_EN	0x10
#define	OV9650_COM13_Y_DELAY_EN	0x08
#define	OV9650_COM13_YUV_DLY(x)	((x) & 0x07)

#define	OV9650_COM15_OR_00FF	0x00
#define	OV9650_COM15_OR_01FE	0x40
#define	OV9650_COM15_OR_10F0	0xc0
#define	OV9650_COM15_RGB_NORM	0x00
#define	OV9650_COM15_RGB_565	0x10
#define	OV9650_COM15_RGB_555	0x30

#define	OV9650_COM16_RB_AVG	0x01

/* IDs */
#define	OV9650_V2		0x9648
#define	OV9650_V3		0x9649
#define	OV9650_V4		0x9652
#define	VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xFF))

/* supported resolutions */
enum {
	W_QQCIF	= 88,
	W_QQVGA	= 160,
	W_QCIF	= 176,
	W_QVGA	= 320,
	W_CIF	= 352,
	W_VGA	= 640,
	W_SXGA	= 1280
};
#define	H_SXGA	960

/* Misc. structures */
struct ov9650_reg_alt {
	u8	com7;
	u8	com12;
	u8	com13;
	u8	com15;
};

struct ov9650_reg {
	u8	reg;
	u8	val;
};

struct ov9650_priv {
	struct v4l2_subdev		subdev;

	int				model;
	int				revision;

	bool				flag_vflip;
	bool				flag_hflip;
};

#endif	/* __DRIVERS_MEDIA_VIDEO_OV9650_H__ */
