/*
 * Z-Star/Vimicro zc301/zc302p/vc30x driver
 *
 * Copyright (C) 2009-2012 Jean-Francois Moine <http://moinejf.free.fr>
 * Copyright (C) 2004 2005 2006 Michel Xhaard mxhaard@magic.fr
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/input.h>
#include "gspca.h"
#include "jpeg.h"

MODULE_AUTHOR("Jean-Francois Moine <http://moinejf.free.fr>, "
		"Serge A. Suchkov <Serge.A.S@tochka.ru>");
MODULE_DESCRIPTION("GSPCA ZC03xx/VC3xx USB Camera Driver");
MODULE_LICENSE("GPL");

static int force_sensor = -1;

#define REG08_DEF 3		/* default JPEG compression (70%) */
#include "zc3xx-reg.h"

/* controls */
enum e_ctrl {
	BRIGHTNESS,
	CONTRAST,
	EXPOSURE,
	GAMMA,
	AUTOGAIN,
	LIGHTFREQ,
	SHARPNESS,
	QUALITY,
	NCTRLS		/* number of controls */
};

#define AUTOGAIN_DEF 1

/* specific webcam descriptor */
struct sd {
	struct gspca_dev gspca_dev;	/* !! must be the first item */

	struct gspca_ctrl ctrls[NCTRLS];

	struct work_struct work;
	struct workqueue_struct *work_thread;

	u8 reg08;		/* webcam compression quality */

	u8 bridge;
	u8 sensor;		/* Type of image sensor chip */
	u16 chip_revision;

	u8 jpeg_hdr[JPEG_HDR_SZ];
};
enum bridges {
	BRIDGE_ZC301,
	BRIDGE_ZC303,
};
enum sensors {
	SENSOR_ADCM2700,
	SENSOR_CS2102,
//	SENSOR_CS2102K,
	SENSOR_GC0303,
	SENSOR_GC0305,
	SENSOR_HDCS2020,
	SENSOR_HV7131B,
	SENSOR_HV7131R,
	SENSOR_ICM105A,
	SENSOR_MC501CB,
	SENSOR_MT9V111_1,	/* (mi360soc) zc301 */
	SENSOR_MT9V111_3,	/* (mi360soc) zc303 */
	SENSOR_OV7620,		/* OV7648 - same values */
	SENSOR_OV7630C,
	SENSOR_PAS106,
	SENSOR_PAS202B,
	SENSOR_PB0330,
	SENSOR_PO2030,
	SENSOR_TAS5130C,
	SENSOR_MAX
};

/* V4L2 controls supported by the driver */
static void setcontrast(struct gspca_dev *gspca_dev);
static void setexposure(struct gspca_dev *gspca_dev);
static int sd_setautogain(struct gspca_dev *gspca_dev, __s32 val);
static void setlightfreq(struct gspca_dev *gspca_dev);
static void setsharpness(struct gspca_dev *gspca_dev);
static int sd_setquality(struct gspca_dev *gspca_dev, __s32 val);

static const struct ctrl sd_ctrls[NCTRLS] = {
[BRIGHTNESS] = {
	    {
		.id      = V4L2_CID_BRIGHTNESS,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Brightness",
		.minimum = 0,
		.maximum = 255,
		.step    = 1,
		.default_value = 128,
	    },
	    .set_control = setcontrast
	},
[CONTRAST] = {
	    {
		.id      = V4L2_CID_CONTRAST,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Contrast",
		.minimum = 0,
		.maximum = 255,
		.step    = 1,
		.default_value = 128,
	    },
	    .set_control = setcontrast
	},
[EXPOSURE] = {
	    {
		.id      = V4L2_CID_EXPOSURE,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Exposure",
		.minimum = 0x30d,
		.maximum	= 0x493e,
		.step		= 1,
		.default_value  = 0x927
	    },
	    .set_control = setexposure
	},
[GAMMA] = {
	    {
		.id      = V4L2_CID_GAMMA,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Gamma",
		.minimum = 1,
		.maximum = 6,
		.step    = 1,
		.default_value = 4,
	    },
	    .set_control = setcontrast
	},
[AUTOGAIN] = {
	    {
		.id      = V4L2_CID_AUTOGAIN,
		.type    = V4L2_CTRL_TYPE_BOOLEAN,
		.name    = "Auto Gain",
		.minimum = 0,
		.maximum = 1,
		.step    = 1,
		.default_value = AUTOGAIN_DEF,
		.flags   = V4L2_CTRL_FLAG_UPDATE
	    },
	    .set = sd_setautogain
	},
[LIGHTFREQ] = {
	    {
		.id	 = V4L2_CID_POWER_LINE_FREQUENCY,
		.type    = V4L2_CTRL_TYPE_MENU,
		.name    = "Light frequency filter",
		.minimum = 0,
		.maximum = 2,	/* 0: 0, 1: 50Hz, 2:60Hz */
		.step    = 1,
		.default_value = 0,
	    },
	    .set_control = setlightfreq
	},
[SHARPNESS] = {
	    {
		.id	 = V4L2_CID_SHARPNESS,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Sharpness",
		.minimum = 0,
		.maximum = 3,
		.step    = 1,
		.default_value = 2,
	    },
	    .set_control = setsharpness
	},
[QUALITY] = {
	    {
		.id	 = V4L2_CID_JPEG_COMPRESSION_QUALITY,
		.type    = V4L2_CTRL_TYPE_INTEGER,
		.name    = "Compression Quality",
		.minimum = 40,
		.maximum = 70,
		.step    = 1,
		.default_value = 70	/* updated in sd_init() */
	    },
	    .set = sd_setquality
	},
};

static const struct v4l2_pix_format vga_mode[] = {
	{320, 240, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 240 * 5 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 1},
	{640, 480, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 640,
		.sizeimage = 640 * 480 * 5 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 0},
};

static const struct v4l2_pix_format broken_vga_mode[] = {
	{320, 232, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 320,
		.sizeimage = 320 * 232 * 4 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 1},
	{640, 472, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 640,
		.sizeimage = 640 * 472 * 3 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 0},
};

static const struct v4l2_pix_format sif_mode[] = {
	{176, 144, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 176,
		.sizeimage = 176 * 144 * 3 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 1},
	{352, 288, V4L2_PIX_FMT_JPEG, V4L2_FIELD_NONE,
		.bytesperline = 352,
		.sizeimage = 352 * 288 * 3 / 8 + 590,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 0},
};

/* bridge reg08 -> JPEG quality conversion table */
static u8 jpeg_qual[] = {40, 50, 60, 70, /*80*/};

/* usb exchanges */
struct usb_action {
	u8	req;
	u8	val;
	u16	idx;
};

static const struct usb_action adcm2700_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc */
	{0xa0, 0x04, ZC3XX_R002_CLOCKSELECT},		/* 00,02,04,cc */
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING},		/* 00,08,03,cc */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xa0, 0xd3, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,d3,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xd8, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,d8,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,00,cc */
	{0xa0, 0xde, ZC3XX_R09C_WINHEIGHTLOW},		/* 00,9c,de,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,86,cc */
	{0xbb, 0x00, 0x0400},				/* 04,00,00,bb */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x0f, 0x140f},				/* 14,0f,0f,bb */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,37,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},		/* 01,89,06,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc */
	{0xa0, 0x58, ZC3XX_R116_RGAIN},			/* 01,16,58,cc */
	{0xa0, 0x5a, ZC3XX_R118_BGAIN},			/* 01,18,5a,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,02,cc */
	{0xa0, 0xd3, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,d3,cc */
	{0xbb, 0x00, 0x0408},				/* 04,00,08,bb */
	{0xdd, 0x00, 0x0200},				/* 00,02,00,dd */
	{0xbb, 0x00, 0x0400},				/* 04,00,00,bb */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x0f, 0x140f},				/* 14,0f,0f,bb */
	{0xbb, 0xe0, 0x0c2e},				/* 0c,e0,2e,bb */
	{0xbb, 0x01, 0x2000},				/* 20,01,00,bb */
	{0xbb, 0x96, 0x2400},				/* 24,96,00,bb */
	{0xbb, 0x06, 0x1006},				/* 10,06,06,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x5f, 0x2090},				/* 20,5f,90,bb */
	{0xbb, 0x01, 0x8000},				/* 80,01,00,bb */
	{0xbb, 0x09, 0x8400},				/* 84,09,00,bb */
	{0xbb, 0x86, 0x0002},				/* 00,86,02,bb */
	{0xbb, 0xe6, 0x0401},				/* 04,e6,01,bb */
	{0xbb, 0x86, 0x0802},				/* 08,86,02,bb */
	{0xbb, 0xe6, 0x0c01},				/* 0c,e6,01,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0000},				/* 00,fe,00,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0020},				/* 00,fe,20,aa */
/*mswin+*/
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},
	{0xaa, 0xfe, 0x0002},
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xaa, 0xb4, 0xcd37},
	{0xaa, 0xa4, 0x0004},
	{0xaa, 0xa8, 0x0007},
	{0xaa, 0xac, 0x0004},
/*mswin-*/
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0000},				/* 00,fe,00,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x04, 0x0400},				/* 04,04,00,bb */
	{0xdd, 0x00, 0x0100},				/* 00,01,00,dd */
	{0xbb, 0x01, 0x0400},				/* 04,01,00,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xbb, 0x41, 0x2803},				/* 28,41,03,bb */
	{0xbb, 0x40, 0x2c03},				/* 2c,40,03,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0010},				/* 00,fe,10,aa */
	{}
};
static const struct usb_action adcm2700_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},		/* 00,02,10,cc */
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING},		/* 00,08,03,cc */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xa0, 0xd3, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,d3,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xd0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,d0,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,00,cc */
	{0xa0, 0xd8, ZC3XX_R09C_WINHEIGHTLOW},		/* 00,9c,d8,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,88,cc */
	{0xbb, 0x00, 0x0400},				/* 04,00,00,bb */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x0f, 0x140f},				/* 14,0f,0f,bb */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,37,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},		/* 01,89,06,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc */
	{0xa0, 0x58, ZC3XX_R116_RGAIN},			/* 01,16,58,cc */
	{0xa0, 0x5a, ZC3XX_R118_BGAIN},			/* 01,18,5a,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,02,cc */
	{0xa0, 0xd3, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,d3,cc */
	{0xbb, 0x00, 0x0408},				/* 04,00,08,bb */
	{0xdd, 0x00, 0x0200},				/* 00,02,00,dd */
	{0xbb, 0x00, 0x0400},				/* 04,00,00,bb */
	{0xdd, 0x00, 0x0050},				/* 00,00,50,dd */
	{0xbb, 0x0f, 0x140f},				/* 14,0f,0f,bb */
	{0xbb, 0xe0, 0x0c2e},				/* 0c,e0,2e,bb */
	{0xbb, 0x01, 0x2000},				/* 20,01,00,bb */
	{0xbb, 0x96, 0x2400},				/* 24,96,00,bb */
	{0xbb, 0x06, 0x1006},				/* 10,06,06,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x5f, 0x2090},				/* 20,5f,90,bb */
	{0xbb, 0x01, 0x8000},				/* 80,01,00,bb */
	{0xbb, 0x09, 0x8400},				/* 84,09,00,bb */
	{0xbb, 0x86, 0x0002},				/* 00,88,02,bb */
	{0xbb, 0xe6, 0x0401},				/* 04,e6,01,bb */
	{0xbb, 0x86, 0x0802},				/* 08,88,02,bb */
	{0xbb, 0xe6, 0x0c01},				/* 0c,e6,01,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0000},				/* 00,fe,00,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0020},				/* 00,fe,20,aa */
	/*******/
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xaa, 0xfe, 0x0000},				/* 00,fe,00,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xdd, 0x00, 0x0010},				/* 00,00,10,dd */
	{0xbb, 0x04, 0x0400},				/* 04,04,00,bb */
	{0xdd, 0x00, 0x0100},				/* 00,01,00,dd */
	{0xbb, 0x01, 0x0400},				/* 04,01,00,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xbb, 0x41, 0x2803},				/* 28,41,03,bb */
	{0xbb, 0x40, 0x2c03},				/* 2c,40,03,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0010},				/* 00,fe,10,aa */
	{}
};
static const struct usb_action adcm2700_50HZ[] = {
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xbb, 0x05, 0x8400},				/* 84,05,00,bb */
	{0xbb, 0xd0, 0xb007},				/* b0,d0,07,bb */
	{0xbb, 0xa0, 0xb80f},				/* b8,a0,0f,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0010},				/* 00,fe,10,aa */
	{0xaa, 0x26, 0x00d0},				/* 00,26,d0,aa */
	{0xaa, 0x28, 0x0002},				/* 00,28,02,aa */
	{}
};
static const struct usb_action adcm2700_60HZ[] = {
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xbb, 0x07, 0x8400},				/* 84,07,00,bb */
	{0xbb, 0x82, 0xb006},				/* b0,82,06,bb */
	{0xbb, 0x04, 0xb80d},				/* b8,04,0d,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0010},				/* 00,fe,10,aa */
	{0xaa, 0x26, 0x0057},				/* 00,26,57,aa */
	{0xaa, 0x28, 0x0002},				/* 00,28,02,aa */
	{}
};
static const struct usb_action adcm2700_NoFliker[] = {
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0002},				/* 00,fe,02,aa */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0a,cc */
	{0xbb, 0x07, 0x8400},				/* 84,07,00,bb */
	{0xbb, 0x05, 0xb000},				/* b0,05,00,bb */
	{0xbb, 0xa0, 0xb801},				/* b8,a0,01,bb */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xaa, 0xfe, 0x0010},				/* 00,fe,10,aa */
	{}
};
static const struct usb_action cs2102_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x20, ZC3XX_R080_HBLANKHIGH},
	{0xa0, 0x21, ZC3XX_R081_HBLANKLOW},
	{0xa0, 0x30, ZC3XX_R083_RGAINADDR},
	{0xa0, 0x31, ZC3XX_R084_GGAINADDR},
	{0xa0, 0x32, ZC3XX_R085_BGAINADDR},
	{0xa0, 0x23, ZC3XX_R086_EXPTIMEHIGH},
	{0xa0, 0x24, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x25, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0xb3, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xaa, 0x02, 0x0008},
	{0xaa, 0x03, 0x0000},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x12, 0x0089},
	{0xaa, 0x13, 0x0000},
	{0xaa, 0x14, 0x00e9},
	{0xaa, 0x20, 0x0000},
	{0xaa, 0x22, 0x0000},
	{0xaa, 0x0b, 0x0004},
	{0xaa, 0x30, 0x0030},
	{0xaa, 0x31, 0x0030},
	{0xaa, 0x32, 0x0030},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x10, 0x01ae},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x68, ZC3XX_R18D_YTARGET},
	{0xa0, 0x00, 0x01ad},
	{}
};

static const struct usb_action cs2102_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x20, ZC3XX_R080_HBLANKHIGH},
	{0xa0, 0x21, ZC3XX_R081_HBLANKLOW},
	{0xa0, 0x30, ZC3XX_R083_RGAINADDR},
	{0xa0, 0x31, ZC3XX_R084_GGAINADDR},
	{0xa0, 0x32, ZC3XX_R085_BGAINADDR},
	{0xa0, 0x23, ZC3XX_R086_EXPTIMEHIGH},
	{0xa0, 0x24, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x25, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0xb3, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xaa, 0x02, 0x0008},
	{0xaa, 0x03, 0x0000},
	{0xaa, 0x11, 0x0001},
	{0xaa, 0x12, 0x0087},
	{0xaa, 0x13, 0x0001},
	{0xaa, 0x14, 0x00e7},
	{0xaa, 0x20, 0x0000},
	{0xaa, 0x22, 0x0000},
	{0xaa, 0x0b, 0x0004},
	{0xaa, 0x30, 0x0030},
	{0xaa, 0x31, 0x0030},
	{0xaa, 0x32, 0x0030},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x15, 0x01ae},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x68, ZC3XX_R18D_YTARGET},
	{0xa0, 0x00, 0x01ad},
	{}
};
static const struct usb_action cs2102_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0001},
	{0xaa, 0x24, 0x005f},
	{0xaa, 0x25, 0x0090},
	{0xaa, 0x21, 0x00dd},
	{0xa0, 0x02, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xbf, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x20, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x3a, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x98, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xdd, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xe4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action cs2102_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0000},
	{0xaa, 0x24, 0x00af},
	{0xaa, 0x25, 0x00c8},
	{0xaa, 0x21, 0x0068},
	{0xa0, 0x01, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x5f, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x90, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x1d, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x4c, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x68, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xe3, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action cs2102_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0001},
	{0xaa, 0x24, 0x0055},
	{0xaa, 0x25, 0x00cc},
	{0xaa, 0x21, 0x003f},
	{0xa0, 0x02, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xab, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x98, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x30, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0xd4, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x39, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x70, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xb0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action cs2102_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0000},
	{0xaa, 0x24, 0x00aa},
	{0xaa, 0x25, 0x00e6},
	{0xaa, 0x21, 0x003f},
	{0xa0, 0x01, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x55, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xcc, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x18, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x6a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x3f, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xa5, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action cs2102_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0001},
	{0xaa, 0x24, 0x005f},
	{0xaa, 0x25, 0x0000},
	{0xaa, 0x21, 0x0001},
	{0xa0, 0x02, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xbf, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x80, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x01, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x40, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xa0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action cs2102_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x23, 0x0000},
	{0xaa, 0x24, 0x00af},
	{0xaa, 0x25, 0x0080},
	{0xaa, 0x21, 0x0001},
	{0xa0, 0x01, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x5f, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x80, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x80, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x01, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x40, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xa0, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};

#if 0
/* from usbvm31b.inf 0ac8:301b KMC-90 02/11/15 */
/* defined as tas5130c - this sensor is a hdcs2020 */
static const struct usb_action cs2102k_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x55, ZC3XX_R08B_I2CDEVICEADDR},
	{0xaa, 0x18, 0x0000},	/* control 2 */
	{0xaa, 0x0a, 0x0002},	/* first window row */
	{0xaa, 0x0b, 0x0002},	/* first window column */
	{0xaa, 0x0c, 0x007b},	/* last window row */
	{0xaa, 0x0d, 0x00a3},	/* last window column */
	{0xaa, 0x03, 0x00fb},	/* pad control */
	{0xaa, 0x05, 0x0000},	/* interface control */
	{0xaa, 0x06, 0x0003},	/* interface timing */
	{0xaa, 0x09, 0x0008},	/* ADC control */
	{0xaa, 0x0e, 0x0004},	/* timing control */
	{0xaa, 0x0f, 0x0018},	/* green 1 gain */
	{0xaa, 0x10, 0x0018},	/* red gain */
	{0xaa, 0x11, 0x0018},	/* blue gain */
	{0xaa, 0x12, 0x0018},	/* green 2 gain */
	{0xaa, 0x15, 0x0000},	/* sub-row exposure */
	{0xaa, 0x16, 0x000c},	/* error control */
	{0xaa, 0x17, 0x000c},	/* interface timing 2 */
	{0xaa, 0x18, 0x0004},	/* control 2 */
	{0xa0, 0xf7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x01, 0x01b1},
	{}
};
static const struct usb_action cs2102k_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x11, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x55, ZC3XX_R08B_I2CDEVICEADDR},
	{0xaa, 0x18, 0x0000},
	{0xaa, 0x0a, 0x0002},
	{0xaa, 0x0b, 0x0002},
	{0xaa, 0x0c, 0x007c},
	{0xaa, 0x0d, 0x00a3},
	{0xaa, 0x03, 0x00fb},
	{0xaa, 0x05, 0x0000},
	{0xaa, 0x06, 0x0003},
	{0xaa, 0x09, 0x0008},
	{0xaa, 0x0e, 0x0004},
	{0xaa, 0x0f, 0x0018},
	{0xaa, 0x10, 0x0018},
	{0xaa, 0x11, 0x0018},
	{0xaa, 0x12, 0x0018},
	{0xaa, 0x15, 0x0000},
	{0xaa, 0x16, 0x000c},
	{0xaa, 0x17, 0x000c},
	{0xaa, 0x18, 0x0004},
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x01, 0x01b1},
	{}
};
static const struct usb_action cs2102k_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x18, 0x0000},	/* control 2 */
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x13, 0x0018},	/* row exposure low */
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x14, 0x0001},	/* row exposure high */
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x20, 0x0001},	/* ?? */
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x21, 0x0018},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x18, 0x0004},	/* control 2 */
	{0xdd, 0x00, 0x0020},
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH},
	{0xa0, 0x18, ZC3XX_R0A4_EXPOSURETIMELOW},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xee, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x46, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x0c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x28, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x04, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x0f, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x19, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x1f, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action cs2102k_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x18, 0x0000},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x13, 0x0022},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x14, 0x0001},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x20, 0x0001},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x21, 0x0022},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x18, 0x0004},
	{0xdd, 0x00, 0x0020},
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH},
	{0xa0, 0x22, ZC3XX_R0A4_EXPOSURETIMELOW},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xee, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x3a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x0c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x28, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x04, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x0f, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x19, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x1f, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action cs2102k_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x18, 0x0000},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x13, 0x0010},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x14, 0x0001},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x20, 0x0001},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x21, 0x0010},
	{0xdd, 0x00, 0x0020},
	{0xaa, 0x18, 0x0004},
	{0xdd, 0x00, 0x0020},
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH},
	{0xa0, 0x10, ZC3XX_R0A4_EXPOSURETIMELOW},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x04, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x0f, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x19, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x1f, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{}
};
#endif

static const struct usb_action gc0305_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},	/* 00,00,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00,08,03,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xa0, 0x04, ZC3XX_R002_CLOCKSELECT},	/* 00,02,04,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},	/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,e0,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},	/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},	/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},	/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},	/* 01,1c,00,cc */
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},	/* 00,9c,e6,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},	/* 00,9e,86,cc */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},	/* 00,8b,98,cc */
	{0xaa, 0x13, 0x0002},	/* 00,13,02,aa */
	{0xaa, 0x15, 0x0003},	/* 00,15,03,aa */
	{0xaa, 0x01, 0x0000},	/* 00,01,00,aa */
	{0xaa, 0x02, 0x0000},	/* 00,02,00,aa */
	{0xaa, 0x1a, 0x0000},	/* 00,1a,00,aa */
	{0xaa, 0x1c, 0x0017},	/* 00,1c,17,aa */
	{0xaa, 0x1d, 0x0080},	/* 00,1d,80,aa */
	{0xaa, 0x1f, 0x0008},	/* 00,1f,08,aa */
	{0xaa, 0x21, 0x0012},	/* 00,21,12,aa */
	{0xa0, 0x82, ZC3XX_R086_EXPTIMEHIGH},	/* 00,86,82,cc */
	{0xa0, 0x83, ZC3XX_R087_EXPTIMEMID},	/* 00,87,83,cc */
	{0xa0, 0x84, ZC3XX_R088_EXPTIMELOW},	/* 00,88,84,cc */
	{0xaa, 0x05, 0x0000},	/* 00,05,00,aa */
	{0xaa, 0x0a, 0x0000},	/* 00,0a,00,aa */
	{0xaa, 0x0b, 0x00b0},	/* 00,0b,b0,aa */
	{0xaa, 0x0c, 0x0000},	/* 00,0c,00,aa */
	{0xaa, 0x0d, 0x00b0},	/* 00,0d,b0,aa */
	{0xaa, 0x0e, 0x0000},	/* 00,0e,00,aa */
	{0xaa, 0x0f, 0x00b0},	/* 00,0f,b0,aa */
	{0xaa, 0x10, 0x0000},	/* 00,10,00,aa */
	{0xaa, 0x11, 0x00b0},	/* 00,11,b0,aa */
	{0xaa, 0x16, 0x0001},	/* 00,16,01,aa */
	{0xaa, 0x17, 0x00e6},	/* 00,17,e6,aa */
	{0xaa, 0x18, 0x0002},	/* 00,18,02,aa */
	{0xaa, 0x19, 0x0086},	/* 00,19,86,aa */
	{0xaa, 0x20, 0x0000},	/* 00,20,00,aa */
	{0xaa, 0x1b, 0x0020},	/* 00,1b,20,aa */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,b7,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},	/* 01,00,0d,cc */
	{0xa0, 0x76, ZC3XX_R189_AWBSTATUS},	/* 01,89,76,cc */
	{0xa0, 0x09, 0x01ad},	/* 01,ad,09,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},	/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},	/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},	/* 03,01,08,cc */
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},	/* 01,a8,60,cc */
	{0xa0, 0x85, ZC3XX_R18D_YTARGET},	/* 01,8d,85,cc */
	{0xa0, 0x00, 0x011e},	/* 01,1e,00,cc */
	{0xa0, 0x52, ZC3XX_R116_RGAIN},	/* 01,16,52,cc */
	{0xa0, 0x40, ZC3XX_R117_GGAIN},	/* 01,17,40,cc */
	{0xa0, 0x52, ZC3XX_R118_BGAIN},	/* 01,18,52,cc */
	{0xa0, 0x03, ZC3XX_R113_RGB03},	/* 01,13,03,cc */
	{}
};
static const struct usb_action gc0305_InitialScale[] = { /* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},	/* 00,00,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00,08,03,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},	/* 00,02,10,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},	/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,e0,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},	/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},	/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},	/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},	/* 01,1c,00,cc */
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},	/* 00,9c,e8,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},	/* 00,9e,88,cc */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},	/* 00,8b,98,cc */
	{0xaa, 0x13, 0x0000},	/* 00,13,00,aa */
	{0xaa, 0x15, 0x0001},	/* 00,15,01,aa */
	{0xaa, 0x01, 0x0000},	/* 00,01,00,aa */
	{0xaa, 0x02, 0x0000},	/* 00,02,00,aa */
	{0xaa, 0x1a, 0x0000},	/* 00,1a,00,aa */
	{0xaa, 0x1c, 0x0017},	/* 00,1c,17,aa */
	{0xaa, 0x1d, 0x0080},	/* 00,1d,80,aa */
	{0xaa, 0x1f, 0x0008},	/* 00,1f,08,aa */
	{0xaa, 0x21, 0x0012},	/* 00,21,12,aa */
	{0xa0, 0x82, ZC3XX_R086_EXPTIMEHIGH},	/* 00,86,82,cc */
	{0xa0, 0x83, ZC3XX_R087_EXPTIMEMID},	/* 00,87,83,cc */
	{0xa0, 0x84, ZC3XX_R088_EXPTIMELOW},	/* 00,88,84,cc */
	{0xaa, 0x05, 0x0000},	/* 00,05,00,aa */
	{0xaa, 0x0a, 0x0000},	/* 00,0a,00,aa */
	{0xaa, 0x0b, 0x00b0},	/* 00,0b,b0,aa */
	{0xaa, 0x0c, 0x0000},	/* 00,0c,00,aa */
	{0xaa, 0x0d, 0x00b0},	/* 00,0d,b0,aa */
	{0xaa, 0x0e, 0x0000},	/* 00,0e,00,aa */
	{0xaa, 0x0f, 0x00b0},	/* 00,0f,b0,aa */
	{0xaa, 0x10, 0x0000},	/* 00,10,00,aa */
	{0xaa, 0x11, 0x00b0},	/* 00,11,b0,aa */
	{0xaa, 0x16, 0x0001},	/* 00,16,01,aa */
	{0xaa, 0x17, 0x00e8},	/* 00,17,e8,aa */
	{0xaa, 0x18, 0x0002},	/* 00,18,02,aa */
	{0xaa, 0x19, 0x0088},	/* 00,19,88,aa */
	{0xaa, 0x20, 0x0000},	/* 00,20,00,aa */
	{0xaa, 0x1b, 0x0020},	/* 00,1b,20,aa */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,b7,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},	/* 01,00,0d,cc */
	{0xa0, 0x76, ZC3XX_R189_AWBSTATUS},	/* 01,89,76,cc */
	{0xa0, 0x09, 0x01ad},	/* 01,ad,09,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},	/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},	/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},	/* 03,01,08,cc */
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},	/* 01,a8,60,cc */
	{0xa0, 0x00, 0x011e},	/* 01,1e,00,cc */
	{0xa0, 0x52, ZC3XX_R116_RGAIN},	/* 01,16,52,cc */
	{0xa0, 0x40, ZC3XX_R117_GGAIN},	/* 01,17,40,cc */
	{0xa0, 0x52, ZC3XX_R118_BGAIN},	/* 01,18,52,cc */
	{0xa0, 0x03, ZC3XX_R113_RGB03},	/* 01,13,03,cc */
	{}
};
static const struct usb_action gc0305_50HZ[] = {
	{0xaa, 0x82, 0x0000},	/* 00,82,00,aa */
	{0xaa, 0x83, 0x0002},	/* 00,83,02,aa */
	{0xaa, 0x84, 0x0038},	/* 00,84,38,aa */	/* win: 00,84,ec */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x0b, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,0b,cc */
	{0xa0, 0x18, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,18,cc */
							/* win: 01,92,10 */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x8e, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,8e,cc */
							/* win: 01,97,ec */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},	/* 01,8c,0e,cc */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,15,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},	/* 00,1d,62,cc */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},	/* 00,1e,90,cc */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},	/* 00,1f,c8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},	/* 00,20,ff,cc */
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},	/* 01,1d,60,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc */
/*	{0xa0, 0x85, ZC3XX_R18D_YTARGET},	 * 01,8d,85,cc *
						 * if 640x480 */
	{}
};
static const struct usb_action gc0305_60HZ[] = {
	{0xaa, 0x82, 0x0000},	/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},	/* 00,83,00,aa */
	{0xaa, 0x84, 0x00ec},	/* 00,84,ec,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x0b, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,0b,cc */
	{0xa0, 0x10, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,10,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0xec, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,ec,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},	/* 01,8c,0e,cc */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,15,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},	/* 00,1d,62,cc */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},	/* 00,1e,90,cc */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},	/* 00,1f,c8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},	/* 00,20,ff,cc */
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},	/* 01,1d,60,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc */
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},	/* 01,8d,80,cc */
	{}
};

static const struct usb_action gc0305_NoFliker[] = {
	{0xa0, 0x0c, ZC3XX_R100_OPERATIONMODE},	/* 01,00,0c,cc */
	{0xaa, 0x82, 0x0000},	/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},	/* 00,83,00,aa */
	{0xaa, 0x84, 0x0020},	/* 00,84,20,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x00, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,00,cc */
	{0xa0, 0x48, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,48,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,10,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},	/* 01,8c,0e,cc */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,15,cc */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},	/* 00,1d,62,cc */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},	/* 00,1e,90,cc */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},	/* 00,1f,c8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},	/* 00,20,ff,cc */
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},	/* 01,1d,60,cc */
	{0xa0, 0x03, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,03,cc */
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},	/* 01,8d,80,cc */
	{}
};

/* from usbvm303.inf for 0ac8:303b 07/08/30 */
static const struct usb_action hdcs2020_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x11, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x55, ZC3XX_R08B_I2CDEVICEADDR},	/* added for kmc-90 */
	{0xaa, 0x1c, 0x0000},
	{0xaa, 0x0a, 0x0001},
	{0xaa, 0x0b, 0x0006},
	{0xaa, 0x0c, 0x007b},
	{0xaa, 0x0d, 0x00a7},
	{0xaa, 0x03, 0x00fb},
	{0xaa, 0x05, 0x0000},
	{0xaa, 0x06, 0x0003},
	{0xaa, 0x09, 0x0008},

	{0xaa, 0x0f, 0x0018},	/* set sensor gain */
	{0xaa, 0x10, 0x0018},
	{0xaa, 0x11, 0x0018},
	{0xaa, 0x12, 0x0018},

	{0xaa, 0x15, 0x004e},
/*	{0xaa, 0x1c, 0x0004},	 * removed for kmc-90 */
	{0xaa, 0x16, 0x000c},	/* added for kmc-90 */
	{0xaa, 0x17, 0x000c},
	{0xaa, 0x18, 0x0004},
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{}
};
static const struct usb_action hdcs2020_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x55, ZC3XX_R08B_I2CDEVICEADDR},	/* added for kmc-90 */
	{0xaa, 0x1c, 0x0000},	/* control */
	{0xaa, 0x0a, 0x0001},	/* first window row */
	{0xaa, 0x0b, 0x0006},	/* first window column */
	{0xaa, 0x0c, 0x007a},	/* last window row */
	{0xaa, 0x0d, 0x00a7},	/* last window column */
	{0xaa, 0x03, 0x00fb},	/* pad control */
	{0xaa, 0x05, 0x0000},	/* interface control */
	{0xaa, 0x06, 0x0003},	/* interface timing */
	{0xaa, 0x09, 0x0008},	/* ADC control */
	{0xaa, 0x0f, 0x0018},	/* green 1 gain */
	{0xaa, 0x10, 0x0018},	/* red gain */
	{0xaa, 0x11, 0x0018},	/* blue gain */
	{0xaa, 0x12, 0x0018},	/* green 2 gain */
	{0xaa, 0x15, 0x004e},	/* sub-row exposure */
/*	{0xaa, 0x1c, 0x0004},	 * control - removed for kmc-90 */
	{0xaa, 0x16, 0x000c},	/* error control - added for kmc-90 */
	{0xaa, 0x17, 0x000c},	/* interface timing 2 */
	{0xaa, 0x18, 0x0004},	/* control 2 */
	{0xa0, 0xf7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{}
};
static const struct usb_action hdcs2020_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x13, 0x0018},	/* row exposure low */
	{0xaa, 0x14, 0x0001},	/* row exposure high */
	{0xaa, 0x0e, 0x0005},	/* timing control */
	{0xaa, 0x19, 0x001f},	/* horizontal blank */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,02,cc */
	{0xa0, 0x76, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,76,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x46, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,46,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x0c, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,0c,cc */
	{0xa0, 0x28, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,28,cc */
	{0xa0, 0x05, ZC3XX_R01D_HSYNC_0}, /* 00,1d,05,cc */
	{0xa0, 0x1f, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x2f, ZC3XX_R01F_HSYNC_2}, /* 00,1f,2f,cc */
	{0xa0, 0x44, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hdcs2020_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x13, 0x0031},			/* 00,13,31,aa */
	{0xaa, 0x14, 0x0001},			/* 00,14,01,aa */
	{0xaa, 0x0e, 0x0004},			/* 00,0e,04,aa */
	{0xaa, 0x19, 0x00cd},			/* 00,19,cd,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,02,cc */
	{0xa0, 0x62, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,62,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x3d, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,3d,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x0c, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,0c,cc */
	{0xa0, 0x28, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,28,cc */
	{0xa0, 0x04, ZC3XX_R01D_HSYNC_0}, /* 00,1d,04,cc */
	{0xa0, 0xcd, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x2c, ZC3XX_R01F_HSYNC_2}, /* 00,1f,2c,cc */
	{0xa0, 0x41, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hdcs2020_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x13, 0x0010},			/* 00,13,10,aa */
	{0xaa, 0x14, 0x0001},			/* 00,14,01,aa */
	{0xaa, 0x0e, 0x0004},			/* 00,0e,04,aa */
	{0xaa, 0x19, 0x0000},			/* 00,19,00,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,02,cc */
	{0xa0, 0x70, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,70,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,00,cc */
	{0xa0, 0x04, ZC3XX_R01D_HSYNC_0}, /* 00,1d,04,cc */
	{0xa0, 0x00, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x2a, ZC3XX_R01F_HSYNC_2}, /* 00,1f,2a,cc */
	{0xa0, 0x3d, ZC3XX_R020_HSYNC_3},
	{}
};

static const struct usb_action hv7131b_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xaa, 0x30, 0x002d},
	{0xaa, 0x01, 0x0005},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x13, 0x0001},	/* {0xaa, 0x13, 0x0000}, */
	{0xaa, 0x14, 0x0001},
	{0xaa, 0x15, 0x00e8},
	{0xaa, 0x16, 0x0002},
	{0xaa, 0x17, 0x0086},		/* 00,17,88,aa */
	{0xaa, 0x31, 0x0038},
	{0xaa, 0x32, 0x0038},
	{0xaa, 0x33, 0x0038},
	{0xaa, 0x5b, 0x0001},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x68, ZC3XX_R18D_YTARGET},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0xc0, 0x019b},
	{0xa0, 0xa0, 0x019c},
	{0xa0, 0x02, ZC3XX_R188_MINGAIN},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xaa, 0x02, 0x0090},			/* 00,02,80,aa */
	{}
};

static const struct usb_action hv7131b_Initial[] = {	/* 640x480*/
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xaa, 0x30, 0x002d},
	{0xaa, 0x01, 0x0005},
	{0xaa, 0x11, 0x0001},
	{0xaa, 0x13, 0x0000},	/* {0xaa, 0x13, 0x0001}; */
	{0xaa, 0x14, 0x0001},
	{0xaa, 0x15, 0x00e6},
	{0xaa, 0x16, 0x0002},
	{0xaa, 0x17, 0x0086},
	{0xaa, 0x31, 0x0038},
	{0xaa, 0x32, 0x0038},
	{0xaa, 0x33, 0x0038},
	{0xaa, 0x5b, 0x0001},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0xc0, 0x019b},
	{0xa0, 0xa0, 0x019c},
	{0xa0, 0x02, ZC3XX_R188_MINGAIN},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xaa, 0x02, 0x0090},	/* {0xaa, 0x02, 0x0080}, */
	{}
};
static const struct usb_action hv7131b_50HZ[] = {	/* 640x480*/
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0007},			/* 00,25,07,aa */
	{0xaa, 0x26, 0x0053},			/* 00,26,53,aa */
	{0xaa, 0x27, 0x0000},			/* 00,27,00,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x0050},			/* 00,21,50,aa */
	{0xaa, 0x22, 0x001b},			/* 00,22,1b,aa */
	{0xaa, 0x23, 0x00fc},			/* 00,23,fc,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0x9b, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,9b,cc */
	{0xa0, 0x80, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,80,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0xea, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,ea,cc */
	{0xa0, 0x60, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,60,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},	/* 01,8c,0c,cc */
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,18,cc */
	{0xa0, 0x18, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,18,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0x50, ZC3XX_R01E_HSYNC_1},	/* 00,1e,50,cc */
	{0xa0, 0x1b, ZC3XX_R01F_HSYNC_2},	/* 00,1f,1b,cc */
	{0xa0, 0xfc, ZC3XX_R020_HSYNC_3},	/* 00,20,fc,cc */
	{}
};
static const struct usb_action hv7131b_50HZScale[] = {	/* 320x240 */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0007},			/* 00,25,07,aa */
	{0xaa, 0x26, 0x0053},			/* 00,26,53,aa */
	{0xaa, 0x27, 0x0000},			/* 00,27,00,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x0050},			/* 00,21,50,aa */
	{0xaa, 0x22, 0x0012},			/* 00,22,12,aa */
	{0xaa, 0x23, 0x0080},			/* 00,23,80,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0x9b, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,9b,cc */
	{0xa0, 0x80, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,80,cc */
	{0xa0, 0x01, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,01,cc */
	{0xa0, 0xd4, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,d4,cc */
	{0xa0, 0xc0, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,c0,cc */
	{0xa0, 0x07, ZC3XX_R18C_AEFREEZE},	/* 01,8c,07,cc */
	{0xa0, 0x0f, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,0f,cc */
	{0xa0, 0x18, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,18,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0x50, ZC3XX_R01E_HSYNC_1},	/* 00,1e,50,cc */
	{0xa0, 0x12, ZC3XX_R01F_HSYNC_2},	/* 00,1f,12,cc */
	{0xa0, 0x80, ZC3XX_R020_HSYNC_3},	/* 00,20,80,cc */
	{}
};
static const struct usb_action hv7131b_60HZ[] = {	/* 640x480*/
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0007},			/* 00,25,07,aa */
	{0xaa, 0x26, 0x00a1},			/* 00,26,a1,aa */
	{0xaa, 0x27, 0x0020},			/* 00,27,20,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x0040},			/* 00,21,40,aa */
	{0xaa, 0x22, 0x0013},			/* 00,22,13,aa */
	{0xaa, 0x23, 0x004c},			/* 00,23,4c,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0x4d, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,4d,cc */
	{0xa0, 0x60, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,60,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0xc3, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,c3,cc */
	{0xa0, 0x50, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,50,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},	/* 01,8c,0c,cc */
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,18,cc */
	{0xa0, 0x18, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,18,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0x40, ZC3XX_R01E_HSYNC_1},	/* 00,1e,40,cc */
	{0xa0, 0x13, ZC3XX_R01F_HSYNC_2},	/* 00,1f,13,cc */
	{0xa0, 0x4c, ZC3XX_R020_HSYNC_3},	/* 00,20,4c,cc */
	{}
};
static const struct usb_action hv7131b_60HZScale[] = {	/* 320x240 */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0007},			/* 00,25,07,aa */
	{0xaa, 0x26, 0x00a1},			/* 00,26,a1,aa */
	{0xaa, 0x27, 0x0020},			/* 00,27,20,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x00a0},			/* 00,21,a0,aa */
	{0xaa, 0x22, 0x0016},			/* 00,22,16,aa */
	{0xaa, 0x23, 0x0040},			/* 00,23,40,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0x4d, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,4d,cc */
	{0xa0, 0x60, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,60,cc */
	{0xa0, 0x01, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,01,cc */
	{0xa0, 0x86, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,86,cc */
	{0xa0, 0xa0, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,a0,cc */
	{0xa0, 0x07, ZC3XX_R18C_AEFREEZE},	/* 01,8c,07,cc */
	{0xa0, 0x0f, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,0f,cc */
	{0xa0, 0x18, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,18,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0xa0, ZC3XX_R01E_HSYNC_1},	/* 00,1e,a0,cc */
	{0xa0, 0x16, ZC3XX_R01F_HSYNC_2},	/* 00,1f,16,cc */
	{0xa0, 0x40, ZC3XX_R020_HSYNC_3},	/* 00,20,40,cc */
	{}
};
static const struct usb_action hv7131b_NoFliker[] = {	/* 640x480*/
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0003},			/* 00,25,03,aa */
	{0xaa, 0x26, 0x0000},			/* 00,26,00,aa */
	{0xaa, 0x27, 0x0000},			/* 00,27,00,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x0010},			/* 00,21,10,aa */
	{0xaa, 0x22, 0x0000},			/* 00,22,00,aa */
	{0xaa, 0x23, 0x0003},			/* 00,23,03,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0xf8, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,f8,cc */
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,00,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x02, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,02,cc */
	{0xa0, 0x00, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,00,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},	/* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,00,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0x10, ZC3XX_R01E_HSYNC_1},	/* 00,1e,10,cc */
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},	/* 00,1f,00,cc */
	{0xa0, 0x03, ZC3XX_R020_HSYNC_3},	/* 00,20,03,cc */
	{}
};
static const struct usb_action hv7131b_NoFlikerScale[] = { /* 320x240 */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},	/* 00,19,00,cc */
	{0xaa, 0x25, 0x0003},			/* 00,25,03,aa */
	{0xaa, 0x26, 0x0000},			/* 00,26,00,aa */
	{0xaa, 0x27, 0x0000},			/* 00,27,00,aa */
	{0xaa, 0x20, 0x0000},			/* 00,20,00,aa */
	{0xaa, 0x21, 0x00a0},			/* 00,21,a0,aa */
	{0xaa, 0x22, 0x0016},			/* 00,22,16,aa */
	{0xaa, 0x23, 0x0040},			/* 00,23,40,aa */
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,2f,cc */
	{0xa0, 0xf8, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,f8,cc */
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,00,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x02, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,02,cc */
	{0xa0, 0x00, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,00,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},	/* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,00,cc */
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},	/* 00,1d,00,cc */
	{0xa0, 0xa0, ZC3XX_R01E_HSYNC_1},	/* 00,1e,a0,cc */
	{0xa0, 0x16, ZC3XX_R01F_HSYNC_2},	/* 00,1f,16,cc */
	{0xa0, 0x40, ZC3XX_R020_HSYNC_3},	/* 00,20,40,cc */
	{}
};

/* from lPEPI264v.inf (hv7131b!) */
static const struct usb_action hv7131r_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x000c},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x13, 0x0000},
	{0xaa, 0x14, 0x0001},
	{0xaa, 0x15, 0x00e8},
	{0xaa, 0x16, 0x0002},
	{0xaa, 0x17, 0x0088},
	{0xaa, 0x30, 0x000b},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0xc0, 0x019b},
	{0xa0, 0xa0, 0x019c},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{}
};
static const struct usb_action hv7131r_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x000c},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x13, 0x0000},
	{0xaa, 0x14, 0x0001},
	{0xaa, 0x15, 0x00e6},
	{0xaa, 0x16, 0x0002},
	{0xaa, 0x17, 0x0086},
	{0xaa, 0x30, 0x000b},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0xc0, 0x019b},
	{0xa0, 0xa0, 0x019c},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{}
};
static const struct usb_action hv7131r_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x06, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x68, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xa0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0xea, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x60, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x18, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hv7131r_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x0c, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xd1, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x40, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x01, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0xd4, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0xc0, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x18, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hv7131r_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x06, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x1a, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x80, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0xc3, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x50, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x18, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hv7131r_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x0c, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x35, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x01, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x86, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0xa0, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x18, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hv7131r_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xf8, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x02, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x58, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action hv7131r_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xa0, 0x2f, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0xf8, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x04, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0xb0, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x00, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x00, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0x08, ZC3XX_R020_HSYNC_3},
	{}
};

static const struct usb_action icm105a_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0c, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x00, ZC3XX_R097_WINYSTARTHIGH},
	{0xa0, 0x01, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R099_WINXSTARTHIGH},
	{0xa0, 0x01, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x01, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x01, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xaa, 0x01, 0x0010},
	{0xaa, 0x03, 0x0000},
	{0xaa, 0x04, 0x0001},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0001},
	{0xaa, 0x04, 0x0011},
	{0xaa, 0x05, 0x00a0},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0002},
	{0xaa, 0x04, 0x0013},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0003},
	{0xaa, 0x04, 0x0015},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0004},
	{0xaa, 0x04, 0x0017},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x000d},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0005},
	{0xaa, 0x04, 0x0019},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0006},
	{0xaa, 0x04, 0x0017},
	{0xaa, 0x05, 0x0026},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0007},
	{0xaa, 0x04, 0x0019},
	{0xaa, 0x05, 0x0022},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0008},
	{0xaa, 0x04, 0x0021},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0009},
	{0xaa, 0x04, 0x0023},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x000d},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000a},
	{0xaa, 0x04, 0x0025},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000b},
	{0xaa, 0x04, 0x00ec},
	{0xaa, 0x05, 0x002e},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000c},
	{0xaa, 0x04, 0x00fa},
	{0xaa, 0x05, 0x002a},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x07, 0x000d},
	{0xaa, 0x01, 0x0005},
	{0xaa, 0x94, 0x0002},
	{0xaa, 0x90, 0x0000},
	{0xaa, 0x91, 0x001f},
	{0xaa, 0x10, 0x0064},
	{0xaa, 0x9b, 0x00f0},
	{0xaa, 0x9c, 0x0002},
	{0xaa, 0x14, 0x001a},
	{0xaa, 0x20, 0x0080},
	{0xaa, 0x22, 0x0080},
	{0xaa, 0x24, 0x0080},
	{0xaa, 0x26, 0x0080},
	{0xaa, 0x00, 0x0084},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xaa, 0xa8, 0x01c0},
	{}
};

static const struct usb_action icm105a_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0c, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x00, ZC3XX_R097_WINYSTARTHIGH},
	{0xa0, 0x02, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R099_WINXSTARTHIGH},
	{0xa0, 0x02, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x02, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x02, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xaa, 0x01, 0x0010},
	{0xaa, 0x03, 0x0000},
	{0xaa, 0x04, 0x0001},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0001},
	{0xaa, 0x04, 0x0011},
	{0xaa, 0x05, 0x00a0},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0002},
	{0xaa, 0x04, 0x0013},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0001},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0003},
	{0xaa, 0x04, 0x0015},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0004},
	{0xaa, 0x04, 0x0017},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x000d},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0005},
	{0xaa, 0x04, 0x0019},
	{0xaa, 0x05, 0x0020},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0006},
	{0xaa, 0x04, 0x0017},
	{0xaa, 0x05, 0x0026},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0007},
	{0xaa, 0x04, 0x0019},
	{0xaa, 0x05, 0x0022},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0008},
	{0xaa, 0x04, 0x0021},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x0009},
	{0xaa, 0x04, 0x0023},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x000d},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000a},
	{0xaa, 0x04, 0x0025},
	{0xaa, 0x05, 0x00aa},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000b},
	{0xaa, 0x04, 0x00ec},
	{0xaa, 0x05, 0x002e},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x03, 0x000c},
	{0xaa, 0x04, 0x00fa},
	{0xaa, 0x05, 0x002a},
	{0xaa, 0x06, 0x0005},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x07, 0x000d},
	{0xaa, 0x01, 0x0005},
	{0xaa, 0x94, 0x0002},
	{0xaa, 0x90, 0x0000},
	{0xaa, 0x91, 0x0010},
	{0xaa, 0x10, 0x0064},
	{0xaa, 0x9b, 0x00f0},
	{0xaa, 0x9c, 0x0002},
	{0xaa, 0x14, 0x001a},
	{0xaa, 0x20, 0x0080},
	{0xaa, 0x22, 0x0080},
	{0xaa, 0x24, 0x0080},
	{0xaa, 0x26, 0x0080},
	{0xaa, 0x00, 0x0084},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xaa, 0xa8, 0x0180},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},
	{}
};
static const struct usb_action icm105a_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x008c},
	{0xaa, 0x0e, 0x000d},
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x0008},
	{0xaa, 0x1d, 0x0001},
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x20, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x84, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xe3, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xec, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf5, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};
static const struct usb_action icm105a_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x0020},
	{0xaa, 0x0e, 0x000e},
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x000d},
	{0xaa, 0x1d, 0x0002}, /* 00,1d,02,aa */
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x1a, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x4b, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xc8, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd8, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xea, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};
static const struct usb_action icm105a_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x0008},
	{0xaa, 0x0e, 0x000d}, /* 00,0e,0d,aa */
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x0083},
	{0xaa, 0x1d, 0x0001},
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x08, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x81, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xc2, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd6, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xea, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};
static const struct usb_action icm105a_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x0004},
	{0xaa, 0x0e, 0x000d},
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x0004},
	{0xaa, 0x1d, 0x0001},
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x10, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x41, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xc1, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xd4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xe8, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};
static const struct usb_action icm105a_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x0004}, /* 00,0c,04,aa */
	{0xaa, 0x0e, 0x000d}, /* 00,0e,0d,aa */
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x0000}, /* 00,1c,00,aa */
	{0xaa, 0x1d, 0x0001},
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x20, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,20,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,10,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,00,cc */
	{0xa0, 0xc1, ZC3XX_R01D_HSYNC_0}, /* 00,1d,c1,cc */
	{0xa0, 0xd4, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d4,cc */
	{0xa0, 0xe8, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};
static const struct usb_action icm105a_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0x0d, 0x0003}, /* 00,0d,03,aa */
	{0xaa, 0x0c, 0x0004}, /* 00,0c,04,aa */
	{0xaa, 0x0e, 0x000d},
	{0xaa, 0x0f, 0x0002}, /* 00,0f,02,aa */
	{0xaa, 0x1c, 0x0000},
	{0xaa, 0x1d, 0x0001},
	{0xaa, 0x00, 0x0084}, /* 00,00,84,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x20, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,20,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,10,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE}, /* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,00,cc */
	{0xa0, 0xc1, ZC3XX_R01D_HSYNC_0}, /* 00,1d,c1,cc */
	{0xa0, 0xd4, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d4,cc */
	{0xa0, 0xe8, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{}
};

static const struct usb_action mc501cb_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT}, /* 00,02,00,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,01,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING}, /* 00,08,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xd8, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,d8,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW}, /* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW}, /* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW}, /* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW}, /* 01,1c,00,cc */
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH}, /* 00,9b,01,cc */
	{0xa0, 0xde, ZC3XX_R09C_WINHEIGHTLOW}, /* 00,9c,de,cc */
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH}, /* 00,9d,02,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW}, /* 00,9e,86,cc */
	{0xa0, 0x33, ZC3XX_R086_EXPTIMEHIGH}, /* 00,86,33,cc */
	{0xa0, 0x34, ZC3XX_R087_EXPTIMEMID}, /* 00,87,34,cc */
	{0xa0, 0x35, ZC3XX_R088_EXPTIMELOW}, /* 00,88,35,cc */
	{0xa0, 0xb0, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,b0,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xaa, 0x01, 0x0001}, /* 00,01,01,aa */
	{0xaa, 0x01, 0x0003}, /* 00,01,03,aa */
	{0xaa, 0x01, 0x0001}, /* 00,01,01,aa */
	{0xaa, 0x03, 0x0000}, /* 00,03,00,aa */
	{0xaa, 0x10, 0x0000}, /* 00,10,00,aa */
	{0xaa, 0x11, 0x0080}, /* 00,11,80,aa */
	{0xaa, 0x12, 0x0000}, /* 00,12,00,aa */
	{0xaa, 0x13, 0x0000}, /* 00,13,00,aa */
	{0xaa, 0x14, 0x0000}, /* 00,14,00,aa */
	{0xaa, 0x15, 0x0000}, /* 00,15,00,aa */
	{0xaa, 0x16, 0x0000}, /* 00,16,00,aa */
	{0xaa, 0x17, 0x0001}, /* 00,17,01,aa */
	{0xaa, 0x18, 0x00de}, /* 00,18,de,aa */
	{0xaa, 0x19, 0x0002}, /* 00,19,02,aa */
	{0xaa, 0x1a, 0x0086}, /* 00,1a,86,aa */
	{0xaa, 0x20, 0x00a8}, /* 00,20,a8,aa */
	{0xaa, 0x22, 0x0000}, /* 00,22,00,aa */
	{0xaa, 0x23, 0x0000}, /* 00,23,00,aa */
	{0xaa, 0x24, 0x0000}, /* 00,24,00,aa */
	{0xaa, 0x40, 0x0033}, /* 00,40,33,aa */
	{0xaa, 0x41, 0x0077}, /* 00,41,77,aa */
	{0xaa, 0x42, 0x0053}, /* 00,42,53,aa */
	{0xaa, 0x43, 0x00b0}, /* 00,43,b0,aa */
	{0xaa, 0x4b, 0x0001}, /* 00,4b,01,aa */
	{0xaa, 0x72, 0x0020}, /* 00,72,20,aa */
	{0xaa, 0x73, 0x0000}, /* 00,73,00,aa */
	{0xaa, 0x80, 0x0000}, /* 00,80,00,aa */
	{0xaa, 0x85, 0x0050}, /* 00,85,50,aa */
	{0xaa, 0x91, 0x0070}, /* 00,91,70,aa */
	{0xaa, 0x92, 0x0072}, /* 00,92,72,aa */
	{0xaa, 0x03, 0x0001}, /* 00,03,01,aa */
	{0xaa, 0x10, 0x00a0}, /* 00,10,a0,aa */
	{0xaa, 0x11, 0x0001}, /* 00,11,01,aa */
	{0xaa, 0x30, 0x0000}, /* 00,30,00,aa */
	{0xaa, 0x60, 0x0000}, /* 00,60,00,aa */
	{0xaa, 0xa0, 0x001a}, /* 00,a0,1a,aa */
	{0xaa, 0xa1, 0x0000}, /* 00,a1,00,aa */
	{0xaa, 0xa2, 0x003f}, /* 00,a2,3f,aa */
	{0xaa, 0xa3, 0x0028}, /* 00,a3,28,aa */
	{0xaa, 0xa4, 0x0010}, /* 00,a4,10,aa */
	{0xaa, 0xa5, 0x0020}, /* 00,a5,20,aa */
	{0xaa, 0xb1, 0x0044}, /* 00,b1,44,aa */
	{0xaa, 0xd0, 0x0001}, /* 00,d0,01,aa */
	{0xaa, 0xd1, 0x0085}, /* 00,d1,85,aa */
	{0xaa, 0xd2, 0x0080}, /* 00,d2,80,aa */
	{0xaa, 0xd3, 0x0080}, /* 00,d3,80,aa */
	{0xaa, 0xd4, 0x0080}, /* 00,d4,80,aa */
	{0xaa, 0xd5, 0x0080}, /* 00,d5,80,aa */
	{0xaa, 0xc0, 0x00c3}, /* 00,c0,c3,aa */
	{0xaa, 0xc2, 0x0044}, /* 00,c2,44,aa */
	{0xaa, 0xc4, 0x0040}, /* 00,c4,40,aa */
	{0xaa, 0xc5, 0x0020}, /* 00,c5,20,aa */
	{0xaa, 0xc6, 0x0008}, /* 00,c6,08,aa */
	{0xaa, 0x03, 0x0004}, /* 00,03,04,aa */
	{0xaa, 0x10, 0x0000}, /* 00,10,00,aa */
	{0xaa, 0x40, 0x0030}, /* 00,40,30,aa */
	{0xaa, 0x41, 0x0020}, /* 00,41,20,aa */
	{0xaa, 0x42, 0x002d}, /* 00,42,2d,aa */
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x1c, 0x0050}, /* 00,1C,50,aa */
	{0xaa, 0x11, 0x0081}, /* 00,11,81,aa */
	{0xaa, 0x3b, 0x001d}, /* 00,3b,1D,aa */
	{0xaa, 0x3c, 0x004c}, /* 00,3c,4C,aa */
	{0xaa, 0x3d, 0x0018}, /* 00,3d,18,aa */
	{0xaa, 0x3e, 0x006a}, /* 00,3e,6A,aa */
	{0xaa, 0x01, 0x0000}, /* 00,01,00,aa */
	{0xaa, 0x52, 0x00ff}, /* 00,52,FF,aa */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,37,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS}, /* 01,89,06,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05}, /* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS}, /* 03,01,08,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,02,cc */
	{0xaa, 0x03, 0x0002}, /* 00,03,02,aa */
	{0xaa, 0x51, 0x0027}, /* 00,51,27,aa */
	{0xaa, 0x52, 0x0020}, /* 00,52,20,aa */
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x50, 0x0010}, /* 00,50,10,aa */
	{0xaa, 0x51, 0x0010}, /* 00,51,10,aa */
	{0xaa, 0x54, 0x0010}, /* 00,54,10,aa */
	{0xaa, 0x55, 0x0010}, /* 00,55,10,aa */
	{0xa0, 0xf0, 0x0199}, /* 01,99,F0,cc */
	{0xa0, 0x80, 0x019a}, /* 01,9A,80,cc */
	{}
};

static const struct usb_action mc501cb_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT}, /* 00,02,10,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,01,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING}, /* 00,08,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xd0, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,d0,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW}, /* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW}, /* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW}, /* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW}, /* 01,1c,00,cc */
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH}, /* 00,9b,01,cc */
	{0xa0, 0xd8, ZC3XX_R09C_WINHEIGHTLOW}, /* 00,9c,d8,cc */
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH}, /* 00,9d,02,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW}, /* 00,9e,88,cc */
	{0xa0, 0x33, ZC3XX_R086_EXPTIMEHIGH}, /* 00,86,33,cc */
	{0xa0, 0x34, ZC3XX_R087_EXPTIMEMID}, /* 00,87,34,cc */
	{0xa0, 0x35, ZC3XX_R088_EXPTIMELOW}, /* 00,88,35,cc */
	{0xa0, 0xb0, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,b0,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xaa, 0x01, 0x0001}, /* 00,01,01,aa */
	{0xaa, 0x01, 0x0003}, /* 00,01,03,aa */
	{0xaa, 0x01, 0x0001}, /* 00,01,01,aa */
	{0xaa, 0x03, 0x0000}, /* 00,03,00,aa */
	{0xaa, 0x10, 0x0000}, /* 00,10,00,aa */
	{0xaa, 0x11, 0x0080}, /* 00,11,80,aa */
	{0xaa, 0x12, 0x0000}, /* 00,12,00,aa */
	{0xaa, 0x13, 0x0000}, /* 00,13,00,aa */
	{0xaa, 0x14, 0x0000}, /* 00,14,00,aa */
	{0xaa, 0x15, 0x0000}, /* 00,15,00,aa */
	{0xaa, 0x16, 0x0000}, /* 00,16,00,aa */
	{0xaa, 0x17, 0x0001}, /* 00,17,01,aa */
	{0xaa, 0x18, 0x00d8}, /* 00,18,d8,aa */
	{0xaa, 0x19, 0x0002}, /* 00,19,02,aa */
	{0xaa, 0x1a, 0x0088}, /* 00,1a,88,aa */
	{0xaa, 0x20, 0x00a8}, /* 00,20,a8,aa */
	{0xaa, 0x22, 0x0000}, /* 00,22,00,aa */
	{0xaa, 0x23, 0x0000}, /* 00,23,00,aa */
	{0xaa, 0x24, 0x0000}, /* 00,24,00,aa */
	{0xaa, 0x40, 0x0033}, /* 00,40,33,aa */
	{0xaa, 0x41, 0x0077}, /* 00,41,77,aa */
	{0xaa, 0x42, 0x0053}, /* 00,42,53,aa */
	{0xaa, 0x43, 0x00b0}, /* 00,43,b0,aa */
	{0xaa, 0x4b, 0x0001}, /* 00,4b,01,aa */
	{0xaa, 0x72, 0x0020}, /* 00,72,20,aa */
	{0xaa, 0x73, 0x0000}, /* 00,73,00,aa */
	{0xaa, 0x80, 0x0000}, /* 00,80,00,aa */
	{0xaa, 0x85, 0x0050}, /* 00,85,50,aa */
	{0xaa, 0x91, 0x0070}, /* 00,91,70,aa */
	{0xaa, 0x92, 0x0072}, /* 00,92,72,aa */
	{0xaa, 0x03, 0x0001}, /* 00,03,01,aa */
	{0xaa, 0x10, 0x00a0}, /* 00,10,a0,aa */
	{0xaa, 0x11, 0x0001}, /* 00,11,01,aa */
	{0xaa, 0x30, 0x0000}, /* 00,30,00,aa */
	{0xaa, 0x60, 0x0000}, /* 00,60,00,aa */
	{0xaa, 0xa0, 0x001a}, /* 00,a0,1a,aa */
	{0xaa, 0xa1, 0x0000}, /* 00,a1,00,aa */
	{0xaa, 0xa2, 0x003f}, /* 00,a2,3f,aa */
	{0xaa, 0xa3, 0x0028}, /* 00,a3,28,aa */
	{0xaa, 0xa4, 0x0010}, /* 00,a4,10,aa */
	{0xaa, 0xa5, 0x0020}, /* 00,a5,20,aa */
	{0xaa, 0xb1, 0x0044}, /* 00,b1,44,aa */
	{0xaa, 0xd0, 0x0001}, /* 00,d0,01,aa */
	{0xaa, 0xd1, 0x0085}, /* 00,d1,85,aa */
	{0xaa, 0xd2, 0x0080}, /* 00,d2,80,aa */
	{0xaa, 0xd3, 0x0080}, /* 00,d3,80,aa */
	{0xaa, 0xd4, 0x0080}, /* 00,d4,80,aa */
	{0xaa, 0xd5, 0x0080}, /* 00,d5,80,aa */
	{0xaa, 0xc0, 0x00c3}, /* 00,c0,c3,aa */
	{0xaa, 0xc2, 0x0044}, /* 00,c2,44,aa */
	{0xaa, 0xc4, 0x0040}, /* 00,c4,40,aa */
	{0xaa, 0xc5, 0x0020}, /* 00,c5,20,aa */
	{0xaa, 0xc6, 0x0008}, /* 00,c6,08,aa */
	{0xaa, 0x03, 0x0004}, /* 00,03,04,aa */
	{0xaa, 0x10, 0x0000}, /* 00,10,00,aa */
	{0xaa, 0x40, 0x0030}, /* 00,40,30,aa */
	{0xaa, 0x41, 0x0020}, /* 00,41,20,aa */
	{0xaa, 0x42, 0x002d}, /* 00,42,2d,aa */
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x1c, 0x0050}, /* 00,1c,50,aa */
	{0xaa, 0x11, 0x0081}, /* 00,11,81,aa */
	{0xaa, 0x3b, 0x003a}, /* 00,3b,3A,aa */
	{0xaa, 0x3c, 0x0098}, /* 00,3c,98,aa */
	{0xaa, 0x3d, 0x0030}, /* 00,3d,30,aa */
	{0xaa, 0x3e, 0x00d4}, /* 00,3E,D4,aa */
	{0xaa, 0x01, 0x0000}, /* 00,01,00,aa */
	{0xaa, 0x52, 0x00ff}, /* 00,52,FF,aa */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,37,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS}, /* 01,89,06,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05}, /* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS}, /* 03,01,08,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,02,cc */
	{0xaa, 0x03, 0x0002}, /* 00,03,02,aa */
	{0xaa, 0x51, 0x004e}, /* 00,51,4E,aa */
	{0xaa, 0x52, 0x0041}, /* 00,52,41,aa */
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x50, 0x0010}, /* 00,50,10,aa */
	{0xaa, 0x51, 0x0010}, /* 00,51,10,aa */
	{0xaa, 0x54, 0x0010}, /* 00,54,10,aa */
	{0xaa, 0x55, 0x0010}, /* 00,55,10,aa */
	{0xa0, 0xf0, 0x0199}, /* 01,99,F0,cc */
	{0xa0, 0x80, 0x019a}, /* 01,9A,80,cc */
	{}
};

static const struct usb_action mc501cb_50HZ[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x001d}, /* 00,36,1D,aa */
	{0xaa, 0x37, 0x004c}, /* 00,37,4C,aa */
	{0xaa, 0x3b, 0x001d}, /* 00,3B,1D,aa */
	{0xaa, 0x3c, 0x004c}, /* 00,3C,4C,aa */
	{0xaa, 0x3d, 0x001d}, /* 00,3D,1D,aa */
	{0xaa, 0x3e, 0x004c}, /* 00,3E,4C,aa */
	{}
};

static const struct usb_action mc501cb_50HZScale[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x003a}, /* 00,36,3A,aa */
	{0xaa, 0x37, 0x0098}, /* 00,37,98,aa */
	{0xaa, 0x3b, 0x003a}, /* 00,3B,3A,aa */
	{0xaa, 0x3c, 0x0098}, /* 00,3C,98,aa */
	{0xaa, 0x3d, 0x003a}, /* 00,3D,3A,aa */
	{0xaa, 0x3e, 0x0098}, /* 00,3E,98,aa */
	{}
};

static const struct usb_action mc501cb_60HZ[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x0018}, /* 00,36,18,aa */
	{0xaa, 0x37, 0x006a}, /* 00,37,6A,aa */
	{0xaa, 0x3d, 0x0018}, /* 00,3D,18,aa */
	{0xaa, 0x3e, 0x006a}, /* 00,3E,6A,aa */
	{0xaa, 0x3b, 0x0018}, /* 00,3B,18,aa */
	{0xaa, 0x3c, 0x006a}, /* 00,3C,6A,aa */
	{}
};

static const struct usb_action mc501cb_60HZScale[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x0030}, /* 00,36,30,aa */
	{0xaa, 0x37, 0x00d4}, /* 00,37,D4,aa */
	{0xaa, 0x3d, 0x0030}, /* 00,3D,30,aa */
	{0xaa, 0x3e, 0x00d4}, /* 00,3E,D4,aa */
	{0xaa, 0x3b, 0x0030}, /* 00,3B,30,aa */
	{0xaa, 0x3c, 0x00d4}, /* 00,3C,D4,aa */
	{}
};

static const struct usb_action mc501cb_NoFliker[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x0018}, /* 00,36,18,aa */
	{0xaa, 0x37, 0x006a}, /* 00,37,6A,aa */
	{0xaa, 0x3d, 0x0018}, /* 00,3D,18,aa */
	{0xaa, 0x3e, 0x006a}, /* 00,3E,6A,aa */
	{0xaa, 0x3b, 0x0018}, /* 00,3B,18,aa */
	{0xaa, 0x3c, 0x006a}, /* 00,3C,6A,aa */
	{}
};

static const struct usb_action mc501cb_NoFlikerScale[] = {
	{0xaa, 0x03, 0x0003}, /* 00,03,03,aa */
	{0xaa, 0x10, 0x00fc}, /* 00,10,fc,aa */
	{0xaa, 0x36, 0x0030}, /* 00,36,30,aa */
	{0xaa, 0x37, 0x00d4}, /* 00,37,D4,aa */
	{0xaa, 0x3d, 0x0030}, /* 00,3D,30,aa */
	{0xaa, 0x3e, 0x00d4}, /* 00,3E,D4,aa */
	{0xaa, 0x3b, 0x0030}, /* 00,3B,30,aa */
	{0xaa, 0x3c, 0x00d4}, /* 00,3C,D4,aa */
	{}
};

/* from zs211.inf */
static const struct usb_action ov7620_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x40, ZC3XX_R002_CLOCKSELECT}, /* 00,02,40,cc */
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING}, /* 00,08,00,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x06, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,06,cc */
	{0xa0, 0x02, ZC3XX_R083_RGAINADDR}, /* 00,83,02,cc */
	{0xa0, 0x01, ZC3XX_R085_BGAINADDR}, /* 00,85,01,cc */
	{0xa0, 0x80, ZC3XX_R086_EXPTIMEHIGH}, /* 00,86,80,cc */
	{0xa0, 0x81, ZC3XX_R087_EXPTIMEMID}, /* 00,87,81,cc */
	{0xa0, 0x10, ZC3XX_R088_EXPTIMELOW}, /* 00,88,10,cc */
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,a1,cc */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE}, /* 00,8d,08,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xd8, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,d8,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW}, /* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW}, /* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW}, /* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW}, /* 01,1c,00,cc */
	{0xa0, 0xde, ZC3XX_R09C_WINHEIGHTLOW}, /* 00,9c,de,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW}, /* 00,9e,86,cc */
	{0xaa, 0x12, 0x0088}, /* 00,12,88,aa */
	{0xaa, 0x12, 0x0048}, /* 00,12,48,aa */
	{0xaa, 0x75, 0x008a}, /* 00,75,8a,aa */
	{0xaa, 0x13, 0x00a3}, /* 00,13,a3,aa */
	{0xaa, 0x04, 0x0000}, /* 00,04,00,aa */
	{0xaa, 0x05, 0x0000}, /* 00,05,00,aa */
	{0xaa, 0x14, 0x0000}, /* 00,14,00,aa */
	{0xaa, 0x15, 0x0004}, /* 00,15,04,aa */
	{0xaa, 0x17, 0x0018}, /* 00,17,18,aa */
	{0xaa, 0x18, 0x00ba}, /* 00,18,ba,aa */
	{0xaa, 0x19, 0x0002}, /* 00,19,02,aa */
	{0xaa, 0x1a, 0x00f1}, /* 00,1a,f1,aa */
	{0xaa, 0x20, 0x0040}, /* 00,20,40,aa */
	{0xaa, 0x24, 0x0088}, /* 00,24,88,aa */
	{0xaa, 0x25, 0x0078}, /* 00,25,78,aa */
	{0xaa, 0x27, 0x00f6}, /* 00,27,f6,aa */
	{0xaa, 0x28, 0x00a0}, /* 00,28,a0,aa */
	{0xaa, 0x21, 0x0000}, /* 00,21,00,aa */
	{0xaa, 0x2a, 0x0083}, /* 00,2a,83,aa */
	{0xaa, 0x2b, 0x0096}, /* 00,2b,96,aa */
	{0xaa, 0x2d, 0x0005}, /* 00,2d,05,aa */
	{0xaa, 0x74, 0x0020}, /* 00,74,20,aa */
	{0xaa, 0x61, 0x0068}, /* 00,61,68,aa */
	{0xaa, 0x64, 0x0088}, /* 00,64,88,aa */
	{0xaa, 0x00, 0x0000}, /* 00,00,00,aa */
	{0xaa, 0x06, 0x0080}, /* 00,06,80,aa */
	{0xaa, 0x01, 0x0090}, /* 00,01,90,aa */
	{0xaa, 0x02, 0x0030}, /* 00,02,30,aa */
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,77,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS}, /* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad}, /* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05}, /* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS}, /* 03,01,08,cc */
	{0xa0, 0x68, ZC3XX_R116_RGAIN}, /* 01,16,68,cc */
	{0xa0, 0x52, ZC3XX_R118_BGAIN}, /* 01,18,52,cc */
	{0xa0, 0x40, ZC3XX_R11D_GLOBALGAIN}, /* 01,1d,40,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,02,cc */
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN}, /* 01,a8,50,cc */
	{}
};
static const struct usb_action ov7620_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x50, ZC3XX_R002_CLOCKSELECT},	/* 00,02,50,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00,08,00,cc */
						/* mx change? */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x06, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,06,cc */
	{0xa0, 0x02, ZC3XX_R083_RGAINADDR},	/* 00,83,02,cc */
	{0xa0, 0x01, ZC3XX_R085_BGAINADDR},	/* 00,85,01,cc */
	{0xa0, 0x80, ZC3XX_R086_EXPTIMEHIGH},	/* 00,86,80,cc */
	{0xa0, 0x81, ZC3XX_R087_EXPTIMEMID},	/* 00,87,81,cc */
	{0xa0, 0x10, ZC3XX_R088_EXPTIMELOW},	/* 00,88,10,cc */
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,a1,cc */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE}, /* 00,8d,08,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xd0, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,d0,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},	/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},	/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},	/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},	/* 01,1c,00,cc */
	{0xa0, 0xd6, ZC3XX_R09C_WINHEIGHTLOW},	/* 00,9c,d6,cc */
						/* OV7648 00,9c,d8,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},	/* 00,9e,88,cc */
	{0xaa, 0x12, 0x0088}, /* 00,12,88,aa */
	{0xaa, 0x12, 0x0048}, /* 00,12,48,aa */
	{0xaa, 0x75, 0x008a}, /* 00,75,8a,aa */
	{0xaa, 0x13, 0x00a3}, /* 00,13,a3,aa */
	{0xaa, 0x04, 0x0000}, /* 00,04,00,aa */
	{0xaa, 0x05, 0x0000}, /* 00,05,00,aa */
	{0xaa, 0x14, 0x0000}, /* 00,14,00,aa */
	{0xaa, 0x15, 0x0004}, /* 00,15,04,aa */
	{0xaa, 0x24, 0x0088}, /* 00,24,88,aa */
	{0xaa, 0x25, 0x0078}, /* 00,25,78,aa */
	{0xaa, 0x17, 0x0018}, /* 00,17,18,aa */
	{0xaa, 0x18, 0x00ba}, /* 00,18,ba,aa */
	{0xaa, 0x19, 0x0002}, /* 00,19,02,aa */
	{0xaa, 0x1a, 0x00f2}, /* 00,1a,f2,aa */
	{0xaa, 0x20, 0x0040}, /* 00,20,40,aa */
	{0xaa, 0x27, 0x00f6}, /* 00,27,f6,aa */
	{0xaa, 0x28, 0x00a0}, /* 00,28,a0,aa */
	{0xaa, 0x21, 0x0000}, /* 00,21,00,aa */
	{0xaa, 0x2a, 0x0083}, /* 00,2a,83,aa */
	{0xaa, 0x2b, 0x0096}, /* 00,2b,96,aa */
	{0xaa, 0x2d, 0x0005}, /* 00,2d,05,aa */
	{0xaa, 0x74, 0x0020}, /* 00,74,20,aa */
	{0xaa, 0x61, 0x0068}, /* 00,61,68,aa */
	{0xaa, 0x64, 0x0088}, /* 00,64,88,aa */
	{0xaa, 0x00, 0x0000}, /* 00,00,00,aa */
	{0xaa, 0x06, 0x0080}, /* 00,06,80,aa */
	{0xaa, 0x01, 0x0090}, /* 00,01,90,aa */
	{0xaa, 0x02, 0x0030}, /* 00,02,30,aa */
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,77,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},	/* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad},			/* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},	/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},	/* 03,01,08,cc */
	{0xa0, 0x68, ZC3XX_R116_RGAIN},		/* 01,16,68,cc */
	{0xa0, 0x52, ZC3XX_R118_BGAIN},		/* 01,18,52,cc */
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},	/* 01,1d,50,cc */
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,02,cc */
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN},	/* 01,a8,50,cc */
	{}
};
static const struct usb_action ov7620_50HZ[] = {
	{0xaa, 0x13, 0x00a3},	/* 00,13,a3,aa */
	{0xdd, 0x00, 0x0100},	/* 00,01,00,dd */
	{0xaa, 0x2b, 0x0096},	/* 00,2b,96,aa */
	{0xaa, 0x75, 0x008a},	/* 00,75,8a,aa */
	{0xaa, 0x2d, 0x0005},	/* 00,2d,05,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,04,cc */
	{0xa0, 0x18, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,18,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x83, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,83,cc */
	{0xaa, 0x10, 0x0082},				/* 00,10,82,aa */
	{0xaa, 0x76, 0x0003},				/* 00,76,03,aa */
/*	{0xa0, 0x40, ZC3XX_R002_CLOCKSELECT},		 * 00,02,40,cc
							 * if mode0 (640x480) */
	{}
};
static const struct usb_action ov7620_60HZ[] = {
	{0xaa, 0x13, 0x00a3},			/* 00,13,a3,aa */
						/* (bug in zs211.inf) */
	{0xdd, 0x00, 0x0100},			/* 00,01,00,dd */
	{0xaa, 0x2b, 0x0000},			/* 00,2b,00,aa */
	{0xaa, 0x75, 0x008a},			/* 00,75,8a,aa */
	{0xaa, 0x2d, 0x0005},			/* 00,2d,05,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x18, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,18,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x83, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,83,cc */
	{0xaa, 0x10, 0x0020},			/* 00,10,20,aa */
	{0xaa, 0x76, 0x0003},			/* 00,76,03,aa */
/*	{0xa0, 0x40, ZC3XX_R002_CLOCKSELECT},	 * 00,02,40,cc
						 * if mode0 (640x480) */
/* ?? in gspca v1, it was
	{0xa0, 0x00, 0x0039},  * 00,00,00,dd *
	{0xa1, 0x01, 0x0037},		*/
	{}
};
static const struct usb_action ov7620_NoFliker[] = {
	{0xaa, 0x13, 0x00a3},			/* 00,13,a3,aa */
						/* (bug in zs211.inf) */
	{0xdd, 0x00, 0x0100},			/* 00,01,00,dd */
	{0xaa, 0x2b, 0x0000},			/* 00,2b,00,aa */
	{0xaa, 0x75, 0x008e},			/* 00,75,8e,aa */
	{0xaa, 0x2d, 0x0001},			/* 00,2d,01,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,04,cc */
	{0xa0, 0x18, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,18,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x01, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,01,cc */
/*	{0xa0, 0x44, ZC3XX_R002_CLOCKSELECT},	 * 00,02,44,cc
						 * if mode1 (320x240) */
/* ?? was
	{0xa0, 0x00, 0x0039},  * 00,00,00,dd *
	{0xa1, 0x01, 0x0037},		*/
	{}
};

static const struct usb_action ov7630c_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x06, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x12, 0x0080},
	{0xa0, 0x02, ZC3XX_R083_RGAINADDR},
	{0xa0, 0x01, ZC3XX_R085_BGAINADDR},
	{0xa0, 0x90, ZC3XX_R086_EXPTIMEHIGH},
	{0xa0, 0x91, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x10, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xd8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xaa, 0x12, 0x0069},
	{0xaa, 0x04, 0x0020},
	{0xaa, 0x06, 0x0050},
	{0xaa, 0x13, 0x0083},
	{0xaa, 0x14, 0x0000},
	{0xaa, 0x15, 0x0024},
	{0xaa, 0x17, 0x0018},
	{0xaa, 0x18, 0x00ba},
	{0xaa, 0x19, 0x0002},
	{0xaa, 0x1a, 0x00f6},
	{0xaa, 0x1b, 0x0002},
	{0xaa, 0x20, 0x00c2},
	{0xaa, 0x24, 0x0060},
	{0xaa, 0x25, 0x0040},
	{0xaa, 0x26, 0x0030},
	{0xaa, 0x27, 0x00ea},
	{0xaa, 0x28, 0x00a0},
	{0xaa, 0x21, 0x0000},
	{0xaa, 0x2a, 0x0081},
	{0xaa, 0x2b, 0x0096},
	{0xaa, 0x2d, 0x0094},
	{0xaa, 0x2f, 0x003d},
	{0xaa, 0x30, 0x0024},
	{0xaa, 0x60, 0x0000},
	{0xaa, 0x61, 0x0040},
	{0xaa, 0x68, 0x007c},
	{0xaa, 0x6f, 0x0015},
	{0xaa, 0x75, 0x0088},
	{0xaa, 0x77, 0x00b5},
	{0xaa, 0x01, 0x0060},
	{0xaa, 0x02, 0x0060},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x04, ZC3XX_R1A7_CALCGLOBALMEAN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R116_RGAIN},
	{0xa0, 0x46, ZC3XX_R118_BGAIN},
	{0xa0, 0x04, ZC3XX_R113_RGB03},
/* 0x10, */
	{0xa1, 0x01, 0x0002},
	{0xa0, 0x50, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xf8, ZC3XX_R10B_RGB01},
	{0xa0, 0xf8, ZC3XX_R10C_RGB02},
	{0xa0, 0xf8, ZC3XX_R10D_RGB10},
	{0xa0, 0x50, ZC3XX_R10E_RGB11},
	{0xa0, 0xf8, ZC3XX_R10F_RGB12},
	{0xa0, 0xf8, ZC3XX_R110_RGB20},
	{0xa0, 0xf8, ZC3XX_R111_RGB21},
	{0xa0, 0x50, ZC3XX_R112_RGB22},
	{0xa1, 0x01, 0x0008},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* clock ? */
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},	/* sharpness+ */
	{0xa1, 0x01, 0x01c8},
	{0xa1, 0x01, 0x01c9},
	{0xa1, 0x01, 0x01ca},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},	/* sharpness- */
	{0xa0, 0x01, ZC3XX_R120_GAMMA00},	/* gamma 2 ?*/
	{0xa0, 0x0c, ZC3XX_R121_GAMMA01},
	{0xa0, 0x1f, ZC3XX_R122_GAMMA02},
	{0xa0, 0x3a, ZC3XX_R123_GAMMA03},
	{0xa0, 0x53, ZC3XX_R124_GAMMA04},
	{0xa0, 0x6d, ZC3XX_R125_GAMMA05},
	{0xa0, 0x85, ZC3XX_R126_GAMMA06},
	{0xa0, 0x9c, ZC3XX_R127_GAMMA07},
	{0xa0, 0xb0, ZC3XX_R128_GAMMA08},
	{0xa0, 0xc2, ZC3XX_R129_GAMMA09},
	{0xa0, 0xd1, ZC3XX_R12A_GAMMA0A},
	{0xa0, 0xde, ZC3XX_R12B_GAMMA0B},
	{0xa0, 0xe9, ZC3XX_R12C_GAMMA0C},
	{0xa0, 0xf2, ZC3XX_R12D_GAMMA0D},
	{0xa0, 0xf9, ZC3XX_R12E_GAMMA0E},
	{0xa0, 0xff, ZC3XX_R12F_GAMMA0F},
	{0xa0, 0x05, ZC3XX_R130_GAMMA10},
	{0xa0, 0x0f, ZC3XX_R131_GAMMA11},
	{0xa0, 0x16, ZC3XX_R132_GAMMA12},
	{0xa0, 0x1a, ZC3XX_R133_GAMMA13},
	{0xa0, 0x19, ZC3XX_R134_GAMMA14},
	{0xa0, 0x19, ZC3XX_R135_GAMMA15},
	{0xa0, 0x17, ZC3XX_R136_GAMMA16},
	{0xa0, 0x15, ZC3XX_R137_GAMMA17},
	{0xa0, 0x12, ZC3XX_R138_GAMMA18},
	{0xa0, 0x10, ZC3XX_R139_GAMMA19},
	{0xa0, 0x0e, ZC3XX_R13A_GAMMA1A},
	{0xa0, 0x0b, ZC3XX_R13B_GAMMA1B},
	{0xa0, 0x09, ZC3XX_R13C_GAMMA1C},
	{0xa0, 0x08, ZC3XX_R13D_GAMMA1D},
	{0xa0, 0x06, ZC3XX_R13E_GAMMA1E},
	{0xa0, 0x03, ZC3XX_R13F_GAMMA1F},
	{0xa0, 0x50, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xf8, ZC3XX_R10B_RGB01},
	{0xa0, 0xf8, ZC3XX_R10C_RGB02},
	{0xa0, 0xf8, ZC3XX_R10D_RGB10},
	{0xa0, 0x50, ZC3XX_R10E_RGB11},
	{0xa0, 0xf8, ZC3XX_R10F_RGB12},
	{0xa0, 0xf8, ZC3XX_R110_RGB20},
	{0xa0, 0xf8, ZC3XX_R111_RGB21},
	{0xa0, 0x50, ZC3XX_R112_RGB22},

	{0xa1, 0x01, 0x0180},
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xaa, 0x10, 0x001b},
	{0xaa, 0x76, 0x0002},
	{0xaa, 0x2a, 0x0081},
	{0xaa, 0x2b, 0x0000},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x01, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xb8, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x37, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x26, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x40, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xaa, 0x13, 0x0083},	/* 40 */
	{0xa1, 0x01, 0x0180},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};

static const struct usb_action ov7630c_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x06, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0xa1, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},

	{0xaa, 0x12, 0x0080},
	{0xa0, 0x02, ZC3XX_R083_RGAINADDR},
	{0xa0, 0x01, ZC3XX_R085_BGAINADDR},
	{0xa0, 0x90, ZC3XX_R086_EXPTIMEHIGH},
	{0xa0, 0x91, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x10, ZC3XX_R088_EXPTIMELOW},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},
	{0xaa, 0x12, 0x0069},	/* i2c */
	{0xaa, 0x04, 0x0020},
	{0xaa, 0x06, 0x0050},
	{0xaa, 0x13, 0x00c3},
	{0xaa, 0x14, 0x0000},
	{0xaa, 0x15, 0x0024},
	{0xaa, 0x19, 0x0003},
	{0xaa, 0x1a, 0x00f6},
	{0xaa, 0x1b, 0x0002},
	{0xaa, 0x20, 0x00c2},
	{0xaa, 0x24, 0x0060},
	{0xaa, 0x25, 0x0040},
	{0xaa, 0x26, 0x0030},
	{0xaa, 0x27, 0x00ea},
	{0xaa, 0x28, 0x00a0},
	{0xaa, 0x21, 0x0000},
	{0xaa, 0x2a, 0x0081},
	{0xaa, 0x2b, 0x0096},
	{0xaa, 0x2d, 0x0084},
	{0xaa, 0x2f, 0x003d},
	{0xaa, 0x30, 0x0024},
	{0xaa, 0x60, 0x0000},
	{0xaa, 0x61, 0x0040},
	{0xaa, 0x68, 0x007c},
	{0xaa, 0x6f, 0x0015},
	{0xaa, 0x75, 0x0088},
	{0xaa, 0x77, 0x00b5},
	{0xaa, 0x01, 0x0060},
	{0xaa, 0x02, 0x0060},
	{0xaa, 0x17, 0x0018},
	{0xaa, 0x18, 0x00ba},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x77, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x04, ZC3XX_R1A7_CALCGLOBALMEAN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R116_RGAIN},
	{0xa0, 0x46, ZC3XX_R118_BGAIN},
	{0xa0, 0x04, ZC3XX_R113_RGB03},

	{0xa1, 0x01, 0x0002},
	{0xa0, 0x4e, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xfe, ZC3XX_R10B_RGB01},
	{0xa0, 0xf4, ZC3XX_R10C_RGB02},
	{0xa0, 0xf7, ZC3XX_R10D_RGB10},
	{0xa0, 0x4d, ZC3XX_R10E_RGB11},
	{0xa0, 0xfc, ZC3XX_R10F_RGB12},
	{0xa0, 0x00, ZC3XX_R110_RGB20},
	{0xa0, 0xf6, ZC3XX_R111_RGB21},
	{0xa0, 0x4a, ZC3XX_R112_RGB22},

	{0xa1, 0x01, 0x0008},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* clock ? */
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},	/* sharpness+ */
	{0xa1, 0x01, 0x01c8},
	{0xa1, 0x01, 0x01c9},
	{0xa1, 0x01, 0x01ca},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},	/* sharpness- */
	{0xa0, 0x16, ZC3XX_R120_GAMMA00},	/* gamma ~4 */
	{0xa0, 0x3a, ZC3XX_R121_GAMMA01},
	{0xa0, 0x5b, ZC3XX_R122_GAMMA02},
	{0xa0, 0x7c, ZC3XX_R123_GAMMA03},
	{0xa0, 0x94, ZC3XX_R124_GAMMA04},
	{0xa0, 0xa9, ZC3XX_R125_GAMMA05},
	{0xa0, 0xbb, ZC3XX_R126_GAMMA06},
	{0xa0, 0xca, ZC3XX_R127_GAMMA07},
	{0xa0, 0xd7, ZC3XX_R128_GAMMA08},
	{0xa0, 0xe1, ZC3XX_R129_GAMMA09},
	{0xa0, 0xea, ZC3XX_R12A_GAMMA0A},
	{0xa0, 0xf1, ZC3XX_R12B_GAMMA0B},
	{0xa0, 0xf7, ZC3XX_R12C_GAMMA0C},
	{0xa0, 0xfc, ZC3XX_R12D_GAMMA0D},
	{0xa0, 0xff, ZC3XX_R12E_GAMMA0E},
	{0xa0, 0xff, ZC3XX_R12F_GAMMA0F},
	{0xa0, 0x20, ZC3XX_R130_GAMMA10},
	{0xa0, 0x22, ZC3XX_R131_GAMMA11},
	{0xa0, 0x20, ZC3XX_R132_GAMMA12},
	{0xa0, 0x1c, ZC3XX_R133_GAMMA13},
	{0xa0, 0x16, ZC3XX_R134_GAMMA14},
	{0xa0, 0x13, ZC3XX_R135_GAMMA15},
	{0xa0, 0x10, ZC3XX_R136_GAMMA16},
	{0xa0, 0x0d, ZC3XX_R137_GAMMA17},
	{0xa0, 0x0b, ZC3XX_R138_GAMMA18},
	{0xa0, 0x09, ZC3XX_R139_GAMMA19},
	{0xa0, 0x07, ZC3XX_R13A_GAMMA1A},
	{0xa0, 0x06, ZC3XX_R13B_GAMMA1B},
	{0xa0, 0x05, ZC3XX_R13C_GAMMA1C},
	{0xa0, 0x04, ZC3XX_R13D_GAMMA1D},
	{0xa0, 0x00, ZC3XX_R13E_GAMMA1E},
	{0xa0, 0x01, ZC3XX_R13F_GAMMA1F},
	{0xa0, 0x4e, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xfe, ZC3XX_R10B_RGB01},
	{0xa0, 0xf4, ZC3XX_R10C_RGB02},
	{0xa0, 0xf7, ZC3XX_R10D_RGB10},
	{0xa0, 0x4d, ZC3XX_R10E_RGB11},
	{0xa0, 0xfc, ZC3XX_R10F_RGB12},
	{0xa0, 0x00, ZC3XX_R110_RGB20},
	{0xa0, 0xf6, ZC3XX_R111_RGB21},
	{0xa0, 0x4a, ZC3XX_R112_RGB22},

	{0xa1, 0x01, 0x0180},
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xaa, 0x10, 0x000d},
	{0xaa, 0x76, 0x0002},
	{0xaa, 0x2a, 0x0081},
	{0xaa, 0x2b, 0x0000},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x00, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xd8, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x1b, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x26, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x40, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xaa, 0x13, 0x00c3},

	{0xa1, 0x01, 0x0180},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};

static const struct usb_action pas106b_Initial_com[] = {
/* Sream and Sensor specific */
	{0xa1, 0x01, 0x0010},	/* CMOSSensorSelect */
/* System */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},	/* SystemControl */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},	/* SystemControl */
/* Picture size */
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},	/* ClockSelect */
	{0xa0, 0x03, 0x003a},
	{0xa0, 0x0c, 0x003b},
	{0xa0, 0x04, 0x0038},
	{}
};

static const struct usb_action pas106b_InitialScale[] = {	/* 176x144 */
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
/* Sream and Sensor specific */
	{0xa0, 0x0f, ZC3XX_R010_CMOSSENSORSELECT},
/* Picture size */
	{0xa0, 0x00, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0xb0, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x00, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0x90, ZC3XX_R006_FRAMEHEIGHTLOW},
/* System */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
/* Sream and Sensor specific */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
/* Sensor Interface */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},
/* Window inside sensor array */
	{0xa0, 0x03, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x03, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x28, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x68, ZC3XX_R09E_WINWIDTHLOW},
/* Init the sensor */
	{0xaa, 0x02, 0x0004},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x09, 0x0005},
	{0xaa, 0x0a, 0x0002},
	{0xaa, 0x0b, 0x0002},
	{0xaa, 0x0c, 0x0005},
	{0xaa, 0x0d, 0x0000},
	{0xaa, 0x0e, 0x0002},
	{0xaa, 0x14, 0x0081},
/* Other registers */
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
/* Frame retreiving */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
/* Gains */
	{0xa0, 0xa0, ZC3XX_R1A8_DIGITALGAIN},
/* Unknown */
	{0xa0, 0x00, 0x01ad},
/* Sharpness */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
/* Other registers */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
/* Auto exposure and white balance */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
/*Dead pixels */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
/* EEPROM */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},
/* Other registers */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
/* Auto exposure and white balance */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
/*Dead pixels */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
/* EEPROM */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},

	{0xa0, 0x58, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xf4, ZC3XX_R10B_RGB01},
	{0xa0, 0xf4, ZC3XX_R10C_RGB02},
	{0xa0, 0xf4, ZC3XX_R10D_RGB10},
	{0xa0, 0x58, ZC3XX_R10E_RGB11},
	{0xa0, 0xf4, ZC3XX_R10F_RGB12},
	{0xa0, 0xf4, ZC3XX_R110_RGB20},
	{0xa0, 0xf4, ZC3XX_R111_RGB21},
	{0xa0, 0x58, ZC3XX_R112_RGB22},
/* Auto correction */
	{0xa0, 0x03, ZC3XX_R181_WINXSTART},
	{0xa0, 0x08, ZC3XX_R182_WINXWIDTH},
	{0xa0, 0x16, ZC3XX_R183_WINXCENTER},
	{0xa0, 0x03, ZC3XX_R184_WINYSTART},
	{0xa0, 0x05, ZC3XX_R185_WINYWIDTH},
	{0xa0, 0x14, ZC3XX_R186_WINYCENTER},
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
/* Auto exposure and white balance */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x03, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xb1, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x87, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
/* sensor on */
	{0xaa, 0x07, 0x00b1},
	{0xaa, 0x05, 0x0003},
	{0xaa, 0x04, 0x0001},
	{0xaa, 0x03, 0x003b},
/* Gains */
	{0xa0, 0x20, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x26, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xa0, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
/* Auto correction */
	{0xa0, 0x40, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa1, 0x01, 0x0180},				/* AutoCorrectEnable */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
/* Gains */
	{0xa0, 0x40, ZC3XX_R116_RGAIN},
	{0xa0, 0x40, ZC3XX_R117_GGAIN},
	{0xa0, 0x40, ZC3XX_R118_BGAIN},
	{}
};

static const struct usb_action pas106b_Initial[] = {	/* 352x288 */
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
/* Sream and Sensor specific */
	{0xa0, 0x0f, ZC3XX_R010_CMOSSENSORSELECT},
/* Picture size */
	{0xa0, 0x01, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x60, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0x20, ZC3XX_R006_FRAMEHEIGHTLOW},
/* System */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
/* Sream and Sensor specific */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
/* Sensor Interface */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},
/* Window inside sensor array */
	{0xa0, 0x03, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x03, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0x28, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x68, ZC3XX_R09E_WINWIDTHLOW},
/* Init the sensor */
	{0xaa, 0x02, 0x0004},
	{0xaa, 0x08, 0x0000},
	{0xaa, 0x09, 0x0005},
	{0xaa, 0x0a, 0x0002},
	{0xaa, 0x0b, 0x0002},
	{0xaa, 0x0c, 0x0005},
	{0xaa, 0x0d, 0x0000},
	{0xaa, 0x0e, 0x0002},
	{0xaa, 0x14, 0x0081},
/* Other registers */
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
/* Frame retreiving */
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
/* Gains */
	{0xa0, 0xa0, ZC3XX_R1A8_DIGITALGAIN},
/* Unknown */
	{0xa0, 0x00, 0x01ad},
/* Sharpness */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
/* Other registers */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
/* Auto exposure and white balance */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},
/*Dead pixels */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
/* EEPROM */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},
/* Other registers */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
/* Auto exposure and white balance */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
/*Dead pixels */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
/* EEPROM */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
/* JPEG control */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x08, ZC3XX_R1C6_SHARPNESS00},
	{0xa0, 0x0f, ZC3XX_R1CB_SHARPNESS05},

	{0xa0, 0x58, ZC3XX_R10A_RGB00},	/* matrix */
	{0xa0, 0xf4, ZC3XX_R10B_RGB01},
	{0xa0, 0xf4, ZC3XX_R10C_RGB02},
	{0xa0, 0xf4, ZC3XX_R10D_RGB10},
	{0xa0, 0x58, ZC3XX_R10E_RGB11},
	{0xa0, 0xf4, ZC3XX_R10F_RGB12},
	{0xa0, 0xf4, ZC3XX_R110_RGB20},
	{0xa0, 0xf4, ZC3XX_R111_RGB21},
	{0xa0, 0x58, ZC3XX_R112_RGB22},
/* Auto correction */
	{0xa0, 0x03, ZC3XX_R181_WINXSTART},
	{0xa0, 0x08, ZC3XX_R182_WINXWIDTH},
	{0xa0, 0x16, ZC3XX_R183_WINXCENTER},
	{0xa0, 0x03, ZC3XX_R184_WINYSTART},
	{0xa0, 0x05, ZC3XX_R185_WINYWIDTH},
	{0xa0, 0x14, ZC3XX_R186_WINYCENTER},
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},

/* Auto exposure and white balance */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x03, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xb1, ZC3XX_R192_EXPOSURELIMITLOW},

	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x87, ZC3XX_R197_ANTIFLICKERLOW},

	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
/* sensor on */
	{0xaa, 0x07, 0x00b1},
	{0xaa, 0x05, 0x0003},
	{0xaa, 0x04, 0x0001},
	{0xaa, 0x03, 0x003b},
/* Gains */
	{0xa0, 0x20, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x26, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
/* Auto correction */
	{0xa0, 0x40, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa1, 0x01, 0x0180},				/* AutoCorrectEnable */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
/* Gains */
	{0xa0, 0x40, ZC3XX_R116_RGAIN},
	{0xa0, 0x40, ZC3XX_R117_GGAIN},
	{0xa0, 0x40, ZC3XX_R118_BGAIN},

	{0xa0, 0x00, 0x0007},			/* AutoCorrectEnable */
	{0xa0, 0xff, ZC3XX_R018_FRAMELOST},	/* Frame adjust */
	{}
};
static const struct usb_action pas106b_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x06, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,06,cc */
	{0xa0, 0x54, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,54,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x87, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,87,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},	/* 01,8c,10,cc */
	{0xa0, 0x30, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,30,cc */
	{0xaa, 0x03, 0x0021},			/* 00,03,21,aa */
	{0xaa, 0x04, 0x000c},			/* 00,04,0c,aa */
	{0xaa, 0x05, 0x0002},			/* 00,05,02,aa */
	{0xaa, 0x07, 0x001c},			/* 00,07,1c,aa */
	{0xa0, 0x04, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,04,cc */
	{}
};
static const struct usb_action pas106b_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x06, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,06,cc */
	{0xa0, 0x2e, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,2e,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x71, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,71,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},	/* 01,8c,10,cc */
	{0xa0, 0x30, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,30,cc */
	{0xaa, 0x03, 0x001c},			/* 00,03,1c,aa */
	{0xaa, 0x04, 0x0004},			/* 00,04,04,aa */
	{0xaa, 0x05, 0x0001},			/* 00,05,01,aa */
	{0xaa, 0x07, 0x00c4},			/* 00,07,c4,aa */
	{0xa0, 0x04, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,04,cc */
	{}
};
static const struct usb_action pas106b_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x06, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,06,cc */
	{0xa0, 0x50, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,50,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,10,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},	/* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},	/* 01,8f,20,cc */
	{0xaa, 0x03, 0x0013},			/* 00,03,13,aa */
	{0xaa, 0x04, 0x0000},			/* 00,04,00,aa */
	{0xaa, 0x05, 0x0001},			/* 00,05,01,aa */
	{0xaa, 0x07, 0x0030},			/* 00,07,30,aa */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{}
};

/* from lvWIMv.inf 046d:08a2/:08aa 2007/06/03 */
static const struct usb_action pas202b_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0e, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0e,cc */
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},		/* 00,02,00,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,e0,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},	/* 00,8d,08,cc */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,00,cc */
	{0xa0, 0x03, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,03,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,00,cc */
	{0xa0, 0x03, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,03,cc */
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},		/* 00,9b,01,cc */
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},		/* 00,9c,e6,cc */
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},		/* 00,9d,02,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,86,cc */
	{0xaa, 0x02, 0x0002},			/* 00,02,04,aa --> 02 */
	{0xaa, 0x07, 0x0006},				/* 00,07,06,aa */
	{0xaa, 0x08, 0x0002},				/* 00,08,02,aa */
	{0xaa, 0x09, 0x0006},				/* 00,09,06,aa */
	{0xaa, 0x0a, 0x0001},				/* 00,0a,01,aa */
	{0xaa, 0x0b, 0x0001},				/* 00,0b,01,aa */
	{0xaa, 0x0c, 0x0006},
	{0xaa, 0x0d, 0x0000},				/* 00,0d,00,aa */
	{0xaa, 0x10, 0x0000},				/* 00,10,00,aa */
	{0xaa, 0x12, 0x0005},				/* 00,12,05,aa */
	{0xaa, 0x13, 0x0063},				/* 00,13,63,aa */
	{0xaa, 0x15, 0x0070},				/* 00,15,70,aa */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,b7,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},		/* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad},				/* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc */
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},		/* 01,8d,70,cc */
	{}
};
static const struct usb_action pas202b_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0e, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,0e,cc */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},		/* 00,02,10,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc */
	{0xa0, 0x08, ZC3XX_R08D_COMPABILITYMODE},	/* 00,8d,08,cc */
	{0xa0, 0x08, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,08,cc */
	{0xa0, 0x02, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,02,cc */
	{0xa0, 0x08, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,08,cc */
	{0xa0, 0x02, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,02,cc */
	{0xa0, 0x01, ZC3XX_R09B_WINHEIGHTHIGH},		/* 00,9b,01,cc */
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},		/* 00,9d,02,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,88,cc */
	{0xaa, 0x02, 0x0002},				/* 00,02,02,aa */
	{0xaa, 0x07, 0x0006},				/* 00,07,06,aa */
	{0xaa, 0x08, 0x0002},				/* 00,08,02,aa */
	{0xaa, 0x09, 0x0006},				/* 00,09,06,aa */
	{0xaa, 0x0a, 0x0001},				/* 00,0a,01,aa */
	{0xaa, 0x0b, 0x0001},				/* 00,0b,01,aa */
	{0xaa, 0x0c, 0x0006},
	{0xaa, 0x0d, 0x0000},				/* 00,0d,00,aa */
	{0xaa, 0x10, 0x0000},				/* 00,10,00,aa */
	{0xaa, 0x12, 0x0005},				/* 00,12,05,aa */
	{0xaa, 0x13, 0x0063},				/* 00,13,63,aa */
	{0xaa, 0x15, 0x0070},				/* 00,15,70,aa */
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,37,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},		/* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad},				/* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc */
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},		/* 01,8d,70,cc */
	{0xa0, 0xff, ZC3XX_R097_WINYSTARTHIGH},
	{0xa0, 0xfe, ZC3XX_R098_WINYSTARTLOW},
	{}
};
static const struct usb_action pas202b_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0002},				/* 00,20,02,aa */
	{0xaa, 0x21, 0x001b},
	{0xaa, 0x03, 0x0044},				/* 00,03,44,aa */
	{0xaa, 0x04, 0x0008},
	{0xaa, 0x05, 0x001b},
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x1c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x1b, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x4d, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,4d,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1b, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x44, ZC3XX_R01D_HSYNC_0},		/* 00,1d,44,cc */
	{0xa0, 0x6f, ZC3XX_R01E_HSYNC_1},		/* 00,1e,6f,cc */
	{0xa0, 0xad, ZC3XX_R01F_HSYNC_2},		/* 00,1f,ad,cc */
	{0xa0, 0xeb, ZC3XX_R020_HSYNC_3},		/* 00,20,eb,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};
static const struct usb_action pas202b_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0004},
	{0xaa, 0x21, 0x003d},
	{0xaa, 0x03, 0x0041},				/* 00,03,41,aa */
	{0xaa, 0x04, 0x0010},
	{0xaa, 0x05, 0x003d},
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x1c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x3d, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x9b, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,9b,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1b, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x41, ZC3XX_R01D_HSYNC_0},		/* 00,1d,41,cc */
	{0xa0, 0x6f, ZC3XX_R01E_HSYNC_1},		/* 00,1e,6f,cc */
	{0xa0, 0xad, ZC3XX_R01F_HSYNC_2},		/* 00,1f,ad,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};
static const struct usb_action pas202b_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0002},				/* 00,20,02,aa */
	{0xaa, 0x21, 0x0000},				/* 00,21,00,aa */
	{0xaa, 0x03, 0x0045},				/* 00,03,45,aa */
	{0xaa, 0x04, 0x0008},				/* 00,04,08,aa */
	{0xaa, 0x05, 0x0000},				/* 00,05,00,aa */
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x1c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x40, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,40,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1b, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x45, ZC3XX_R01D_HSYNC_0},		/* 00,1d,45,cc */
	{0xa0, 0x8e, ZC3XX_R01E_HSYNC_1},		/* 00,1e,8e,cc */
	{0xa0, 0xc1, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c1,cc */
	{0xa0, 0xf5, ZC3XX_R020_HSYNC_3},		/* 00,20,f5,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};
static const struct usb_action pas202b_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0004},
	{0xaa, 0x21, 0x0008},
	{0xaa, 0x03, 0x0042},				/* 00,03,42,aa */
	{0xaa, 0x04, 0x0010},
	{0xaa, 0x05, 0x0008},
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x1c, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x08, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x81, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,81,cc */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1b, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x42, ZC3XX_R01D_HSYNC_0},		/* 00,1d,42,cc */
	{0xa0, 0x6f, ZC3XX_R01E_HSYNC_1},		/* 00,1e,6f,cc */
	{0xa0, 0xaf, ZC3XX_R01F_HSYNC_2},		/* 00,1f,af,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};
static const struct usb_action pas202b_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0002},				/* 00,20,02,aa */
	{0xaa, 0x21, 0x0006},
	{0xaa, 0x03, 0x0040},				/* 00,03,40,aa */
	{0xaa, 0x04, 0x0008},				/* 00,04,08,aa */
	{0xaa, 0x05, 0x0006},
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x02, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x06, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x01, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},		/* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,00,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x40, ZC3XX_R01D_HSYNC_0},		/* 00,1d,40,cc */
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},		/* 00,1e,60,cc */
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},		/* 00,1f,90,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};
static const struct usb_action pas202b_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},		/* 00,19,00,cc */
	{0xa0, 0x20, ZC3XX_R087_EXPTIMEMID},		/* 00,87,20,cc */
	{0xa0, 0x21, ZC3XX_R088_EXPTIMELOW},		/* 00,88,21,cc */
	{0xaa, 0x20, 0x0004},
	{0xaa, 0x21, 0x000c},
	{0xaa, 0x03, 0x0040},				/* 00,03,40,aa */
	{0xaa, 0x04, 0x0010},
	{0xaa, 0x05, 0x000c},
	{0xaa, 0x0e, 0x0001},				/* 00,0e,01,aa */
	{0xaa, 0x0f, 0x0000},				/* 00,0f,00,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x0c, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc */
	{0xa0, 0x02, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,02,cc */
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},		/* 01,8c,10,cc */
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,20,cc */
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,00,cc */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x40, ZC3XX_R01D_HSYNC_0},		/* 00,1d,40,cc */
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},		/* 00,1e,60,cc */
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},		/* 00,1f,90,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc */
	{0xa0, 0x0f, ZC3XX_R087_EXPTIMEMID},		/* 00,87,0f,cc */
	{0xa0, 0x0e, ZC3XX_R088_EXPTIMELOW},		/* 00,88,0e,cc */
	{}
};

/* mt9v111 (mi0360soc) and pb0330 from vm30x.inf 0ac8:301b 07/02/13 */
static const struct usb_action mt9v111_1_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0001},
	{0xaa, 0x06, 0x0000},
	{0xaa, 0x08, 0x0483},
	{0xaa, 0x01, 0x0004},
	{0xaa, 0x08, 0x0006},
	{0xaa, 0x02, 0x0011},
	{0xaa, 0x03, 0x01e5},			/*jfm: was 01e7*/
	{0xaa, 0x04, 0x0285},			/*jfm: was 0287*/
	{0xaa, 0x07, 0x3002},
	{0xaa, 0x20, 0x5100},
	{0xaa, 0x35, 0x507f},
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xaa, 0x2b, 0x007f},
	{0xaa, 0x2c, 0x007f},			/*jfm: was 0030*/
	{0xaa, 0x2d, 0x007f},			/*jfm: was 0030*/
	{0xaa, 0x2e, 0x007f},			/*jfm: was 0030*/
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x09, 0x01ad},			/*jfm: was 00*/
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x6c, ZC3XX_R18D_YTARGET},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},
	{0xa0, 0x65, ZC3XX_R118_BGAIN},
	{}
};
static const struct usb_action mt9v111_1_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0001},
	{0xaa, 0x06, 0x0000},
	{0xaa, 0x08, 0x0483},
	{0xaa, 0x01, 0x0004},
	{0xaa, 0x08, 0x0006},
	{0xaa, 0x02, 0x0011},
	{0xaa, 0x03, 0x01e7},
	{0xaa, 0x04, 0x0287},
	{0xaa, 0x07, 0x3002},
	{0xaa, 0x20, 0x5100},
	{0xaa, 0x35, 0x007f},			/*jfm: was 0050*/
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xaa, 0x2b, 0x007f},			/*jfm: was 28*/
	{0xaa, 0x2c, 0x007f},			/*jfm: was 30*/
	{0xaa, 0x2d, 0x007f},			/*jfm: was 30*/
	{0xaa, 0x2e, 0x007f},			/*jfm: was 28*/
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x09, 0x01ad},			/*jfm: was 00*/
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x6c, ZC3XX_R18D_YTARGET},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},
	{0xa0, 0x65, ZC3XX_R118_BGAIN},
	{}
};
static const struct usb_action mt9v111_1_AE50HZ[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0562},
	{0xbb, 0x01, 0x09aa},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x03, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x9b, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x47, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_1_AE50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0509},
	{0xbb, 0x01, 0x0934},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xd2, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x9a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_1_AE60HZ[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x003d},
	{0xaa, 0x09, 0x016e},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xdd, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x3d, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_1_AE60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0509},
	{0xbb, 0x01, 0x0983},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x8f, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x81, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_1_AENoFliker[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0509},
	{0xbb, 0x01, 0x0960},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x09, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x40, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_1_AENoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0534},
	{0xbb, 0x02, 0x0960},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x34, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
/* from usbvm303.inf 0ac8:303b 07/03/25 (3 - tas5130c) */
static const struct usb_action mt9v111_3_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0001},		/* select IFP/SOC registers */
	{0xaa, 0x06, 0x0000},		/* operating mode control */
	{0xaa, 0x08, 0x0483},		/* output format control */
					/* H red first, V red or blue first,
					 * raw Bayer, auto flicker */
	{0xaa, 0x01, 0x0004},		/* select sensor core registers */
	{0xaa, 0x08, 0x0006},		/* row start */
	{0xaa, 0x02, 0x0011},		/* column start */
	{0xaa, 0x03, 0x01e5},		/* window height - 1 */
	{0xaa, 0x04, 0x0285},		/* window width - 1 */
	{0xaa, 0x07, 0x3002},		/* output control */
	{0xaa, 0x20, 0x1100},		/* read mode: bits 8 & 12 (?) */
	{0xaa, 0x35, 0x007f},		/* global gain */
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xaa, 0x2b, 0x007f},		/* green1 gain */
	{0xaa, 0x2c, 0x007f},		/* blue gain */
	{0xaa, 0x2d, 0x007f},		/* red gain */
	{0xaa, 0x2e, 0x007f},		/* green2 gain */
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},
	{0xa0, 0x65, ZC3XX_R118_BGAIN},
	{}
};
static const struct usb_action mt9v111_3_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xdc, ZC3XX_R08B_I2CDEVICEADDR},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0001},
	{0xaa, 0x06, 0x0000},
	{0xaa, 0x08, 0x0483},
	{0xaa, 0x01, 0x0004},
	{0xaa, 0x08, 0x0006},
	{0xaa, 0x02, 0x0011},
	{0xaa, 0x03, 0x01e7},
	{0xaa, 0x04, 0x0287},
	{0xaa, 0x07, 0x3002},
	{0xaa, 0x20, 0x1100},
	{0xaa, 0x35, 0x007f},
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xaa, 0x2b, 0x007f},
	{0xaa, 0x2c, 0x007f},
	{0xaa, 0x2d, 0x007f},
	{0xaa, 0x2e, 0x007f},
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},
	{0xa0, 0x65, ZC3XX_R118_BGAIN},
	{}
};
static const struct usb_action mt9v111_3_AE50HZ[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0009},		/* horizontal blanking */
	{0xaa, 0x09, 0x01ce},		/* shutter width */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xd2, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x9a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_3_AE50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0009},
	{0xaa, 0x09, 0x01ce},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xd2, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x9a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_3_AE60HZ[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0009},
	{0xaa, 0x09, 0x0083},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x8f, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x81, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_3_AE60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0009},
	{0xaa, 0x09, 0x0083},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x8f, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x81, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf9, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_3_AENoFliker[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0034},
	{0xaa, 0x09, 0x0260},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x34, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};
static const struct usb_action mt9v111_3_AENoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R180_AUTOCORRECTENABLE},
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xaa, 0x05, 0x0034},
	{0xaa, 0x09, 0x0260},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1c, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x34, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},
	{}
};

static const struct usb_action pb0330_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0006},
	{0xaa, 0x02, 0x0011},
	{0xaa, 0x03, 0x01e5},			/*jfm: was 1e7*/
	{0xaa, 0x04, 0x0285},			/*jfm: was 0287*/
	{0xaa, 0x06, 0x0003},
	{0xaa, 0x07, 0x3002},
	{0xaa, 0x20, 0x1100},
	{0xaa, 0x2f, 0xf7b0},
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x34, 0x0100},
	{0xaa, 0x35, 0x0060},
	{0xaa, 0x3d, 0x068f},
	{0xaa, 0x40, 0x01e0},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x09, 0x01ad},			/*jfm: was 00 */
	{0xa0, 0x15, 0x01ae},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},	/*jfm: was 6c*/
	{}
};
static const struct usb_action pb0330_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},	/* 00 */
	{0xa0, 0x0a, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x07, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},
	{0xdd, 0x00, 0x0200},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xaa, 0x01, 0x0006},
	{0xaa, 0x02, 0x0011},
	{0xaa, 0x03, 0x01e7},
	{0xaa, 0x04, 0x0287},
	{0xaa, 0x06, 0x0003},
	{0xaa, 0x07, 0x3002},
	{0xaa, 0x20, 0x1100},
	{0xaa, 0x2f, 0xf7b0},
	{0xaa, 0x30, 0x0005},
	{0xaa, 0x31, 0x0000},
	{0xaa, 0x34, 0x0100},
	{0xaa, 0x35, 0x0060},
	{0xaa, 0x3d, 0x068f},
	{0xaa, 0x40, 0x01e0},
	{0xaa, 0x58, 0x0078},
	{0xaa, 0x62, 0x0411},
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x09, 0x01ad},
	{0xa0, 0x15, 0x01ae},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x78, ZC3XX_R18D_YTARGET},	/*jfm: was 6c*/
	{}
};
static const struct usb_action pb0330_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x055c},
	{0xbb, 0x01, 0x09aa},
	{0xbb, 0x00, 0x1001},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xc4, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x47, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1a, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x5c, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action pb0330_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0566},
	{0xbb, 0x02, 0x09b2},
	{0xbb, 0x00, 0x1002},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x8c, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x8a, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1a, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd7, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0xf0, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0xf8, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action pb0330_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0535},
	{0xbb, 0x01, 0x0974},
	{0xbb, 0x00, 0x1001},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xfe, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x3e, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1a, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x35, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x50, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xd0, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action pb0330_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0535},
	{0xbb, 0x02, 0x096c},
	{0xbb, 0x00, 0x1002},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xc0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x7c, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x1a, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x14, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x66, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x35, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x50, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xd0, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action pb0330_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0509},
	{0xbb, 0x02, 0x0940},
	{0xbb, 0x00, 0x1002},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x01, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x09, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x40, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{}
};
static const struct usb_action pb0330_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS},
	{0xbb, 0x00, 0x0535},
	{0xbb, 0x01, 0x0980},
	{0xbb, 0x00, 0x1001},
	{0xa0, 0x60, ZC3XX_R11D_GLOBALGAIN},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xf0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x01, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x10, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x20, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x35, ZC3XX_R01D_HSYNC_0},
	{0xa0, 0x60, ZC3XX_R01E_HSYNC_1},
	{0xa0, 0x90, ZC3XX_R01F_HSYNC_2},
	{0xa0, 0xe0, ZC3XX_R020_HSYNC_3},
	{}
};

/* from oem9.inf */
static const struct usb_action po2030_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x04, ZC3XX_R002_CLOCKSELECT},	/* 00,02,04,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,01,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x04, ZC3XX_R080_HBLANKHIGH}, /* 00,80,04,cc */
	{0xa0, 0x05, ZC3XX_R081_HBLANKLOW}, /* 00,81,05,cc */
	{0xa0, 0x16, ZC3XX_R083_RGAINADDR}, /* 00,83,16,cc */
	{0xa0, 0x18, ZC3XX_R085_BGAINADDR}, /* 00,85,18,cc */
	{0xa0, 0x1a, ZC3XX_R086_EXPTIMEHIGH}, /* 00,86,1a,cc */
	{0xa0, 0x1b, ZC3XX_R087_EXPTIMEMID}, /* 00,87,1b,cc */
	{0xa0, 0x1c, ZC3XX_R088_EXPTIMELOW}, /* 00,88,1c,cc */
	{0xa0, 0xee, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,ee,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING}, /* 00,08,03,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,e0,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,42,cc */
	{0xaa, 0x8d, 0x0008},			/* 00,8d,08,aa */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},	/* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},	/* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},	/* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},	/* 01,1c,00,cc */
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},	/* 00,9c,e6,cc */
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},	/* 00,9e,86,cc */
	{0xaa, 0x09, 0x00ce}, /* 00,09,ce,aa */
	{0xaa, 0x0b, 0x0005}, /* 00,0b,05,aa */
	{0xaa, 0x0d, 0x0054}, /* 00,0d,54,aa */
	{0xaa, 0x0f, 0x00eb}, /* 00,0f,eb,aa */
	{0xaa, 0x87, 0x0000}, /* 00,87,00,aa */
	{0xaa, 0x88, 0x0004}, /* 00,88,04,aa */
	{0xaa, 0x89, 0x0000}, /* 00,89,00,aa */
	{0xaa, 0x8a, 0x0005}, /* 00,8a,05,aa */
	{0xaa, 0x13, 0x0003}, /* 00,13,03,aa */
	{0xaa, 0x16, 0x0040}, /* 00,16,40,aa */
	{0xaa, 0x18, 0x0040}, /* 00,18,40,aa */
	{0xaa, 0x1d, 0x0002}, /* 00,1d,02,aa */
	{0xaa, 0x29, 0x00e8}, /* 00,29,e8,aa */
	{0xaa, 0x45, 0x0045}, /* 00,45,45,aa */
	{0xaa, 0x50, 0x00ed}, /* 00,50,ed,aa */
	{0xaa, 0x51, 0x0025}, /* 00,51,25,aa */
	{0xaa, 0x52, 0x0042}, /* 00,52,42,aa */
	{0xaa, 0x53, 0x002f}, /* 00,53,2f,aa */
	{0xaa, 0x79, 0x0025}, /* 00,79,25,aa */
	{0xaa, 0x7b, 0x0000}, /* 00,7b,00,aa */
	{0xaa, 0x7e, 0x0025}, /* 00,7e,25,aa */
	{0xaa, 0x7f, 0x0025}, /* 00,7f,25,aa */
	{0xaa, 0x21, 0x0000}, /* 00,21,00,aa */
	{0xaa, 0x33, 0x0036}, /* 00,33,36,aa */
	{0xaa, 0x36, 0x0060}, /* 00,36,60,aa */
	{0xaa, 0x37, 0x0008}, /* 00,37,08,aa */
	{0xaa, 0x3b, 0x0031}, /* 00,3b,31,aa */
	{0xaa, 0x44, 0x000f}, /* 00,44,0f,aa */
	{0xaa, 0x58, 0x0002}, /* 00,58,02,aa */
	{0xaa, 0x66, 0x00c0}, /* 00,66,c0,aa */
	{0xaa, 0x67, 0x0044}, /* 00,67,44,aa */
	{0xaa, 0x6b, 0x00a0}, /* 00,6b,a0,aa */
	{0xaa, 0x6c, 0x0054}, /* 00,6c,54,aa */
	{0xaa, 0xd6, 0x0007}, /* 00,d6,07,aa */
	{0xa0, 0xf7, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,f7,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS}, /* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad}, /* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05}, /* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS}, /* 03,01,08,cc */
	{0xa0, 0x7a, ZC3XX_R116_RGAIN}, /* 01,16,7a,cc */
	{0xa0, 0x4a, ZC3XX_R118_BGAIN}, /* 01,18,4a,cc */
	{}
};

/* from oem9.inf */
static const struct usb_action po2030_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL}, /* 00,00,01,cc */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT}, /* 00,02,10,cc */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT}, /* 00,10,01,cc */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING}, /* 00,01,01,cc */
	{0xa0, 0x04, ZC3XX_R080_HBLANKHIGH}, /* 00,80,04,cc */
	{0xa0, 0x05, ZC3XX_R081_HBLANKLOW}, /* 00,81,05,cc */
	{0xa0, 0x16, ZC3XX_R083_RGAINADDR}, /* 00,83,16,cc */
	{0xa0, 0x18, ZC3XX_R085_BGAINADDR}, /* 00,85,18,cc */
	{0xa0, 0x1a, ZC3XX_R086_EXPTIMEHIGH}, /* 00,86,1a,cc */
	{0xa0, 0x1b, ZC3XX_R087_EXPTIMEMID}, /* 00,87,1b,cc */
	{0xa0, 0x1c, ZC3XX_R088_EXPTIMELOW}, /* 00,88,1c,cc */
	{0xa0, 0xee, ZC3XX_R08B_I2CDEVICEADDR}, /* 00,8b,ee,cc */
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING}, /* 00,08,03,cc */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,03,cc */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,01,cc */
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH}, /* 00,03,02,cc */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW}, /* 00,04,80,cc */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH}, /* 00,05,01,cc */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW}, /* 00,06,e0,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,42,cc */
	{0xaa, 0x8d, 0x0008},			/* 00,8d,08,aa */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW}, /* 00,98,00,cc */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW}, /* 00,9a,00,cc */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW}, /* 01,1a,00,cc */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW}, /* 01,1c,00,cc */
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW}, /* 00,9c,e8,cc */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW}, /* 00,9e,88,cc */
	{0xaa, 0x09, 0x00cc}, /* 00,09,cc,aa */
	{0xaa, 0x0b, 0x0005}, /* 00,0b,05,aa */
	{0xaa, 0x0d, 0x0058}, /* 00,0d,58,aa */
	{0xaa, 0x0f, 0x00ed}, /* 00,0f,ed,aa */
	{0xaa, 0x87, 0x0000}, /* 00,87,00,aa */
	{0xaa, 0x88, 0x0004}, /* 00,88,04,aa */
	{0xaa, 0x89, 0x0000}, /* 00,89,00,aa */
	{0xaa, 0x8a, 0x0005}, /* 00,8a,05,aa */
	{0xaa, 0x13, 0x0003}, /* 00,13,03,aa */
	{0xaa, 0x16, 0x0040}, /* 00,16,40,aa */
	{0xaa, 0x18, 0x0040}, /* 00,18,40,aa */
	{0xaa, 0x1d, 0x0002}, /* 00,1d,02,aa */
	{0xaa, 0x29, 0x00e8}, /* 00,29,e8,aa */
	{0xaa, 0x45, 0x0045}, /* 00,45,45,aa */
	{0xaa, 0x50, 0x00ed}, /* 00,50,ed,aa */
	{0xaa, 0x51, 0x0025}, /* 00,51,25,aa */
	{0xaa, 0x52, 0x0042}, /* 00,52,42,aa */
	{0xaa, 0x53, 0x002f}, /* 00,53,2f,aa */
	{0xaa, 0x79, 0x0025}, /* 00,79,25,aa */
	{0xaa, 0x7b, 0x0000}, /* 00,7b,00,aa */
	{0xaa, 0x7e, 0x0025}, /* 00,7e,25,aa */
	{0xaa, 0x7f, 0x0025}, /* 00,7f,25,aa */
	{0xaa, 0x21, 0x0000}, /* 00,21,00,aa */
	{0xaa, 0x33, 0x0036}, /* 00,33,36,aa */
	{0xaa, 0x36, 0x0060}, /* 00,36,60,aa */
	{0xaa, 0x37, 0x0008}, /* 00,37,08,aa */
	{0xaa, 0x3b, 0x0031}, /* 00,3b,31,aa */
	{0xaa, 0x44, 0x000f}, /* 00,44,0f,aa */
	{0xaa, 0x58, 0x0002}, /* 00,58,02,aa */
	{0xaa, 0x66, 0x00c0}, /* 00,66,c0,aa */
	{0xaa, 0x67, 0x0044}, /* 00,67,44,aa */
	{0xaa, 0x6b, 0x00a0}, /* 00,6b,a0,aa */
	{0xaa, 0x6c, 0x0054}, /* 00,6c,54,aa */
	{0xaa, 0xd6, 0x0007}, /* 00,d6,07,aa */
	{0xa0, 0xf7, ZC3XX_R101_SENSORCORRECTION}, /* 01,01,f7,cc */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC}, /* 00,12,05,cc */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE}, /* 01,00,0d,cc */
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS}, /* 01,89,06,cc */
	{0xa0, 0x00, 0x01ad}, /* 01,ad,00,cc */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE}, /* 01,c5,03,cc */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05}, /* 01,cb,13,cc */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE}, /* 02,50,08,cc */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS}, /* 03,01,08,cc */
	{0xa0, 0x7a, ZC3XX_R116_RGAIN}, /* 01,16,7a,cc */
	{0xa0, 0x4a, ZC3XX_R118_BGAIN}, /* 01,18,4a,cc */
	{}
};

static const struct usb_action po2030_50HZ[] = {
	{0xaa, 0x8d, 0x0008}, /* 00,8d,08,aa */
	{0xaa, 0x1a, 0x0001}, /* 00,1a,01,aa */
	{0xaa, 0x1b, 0x000a}, /* 00,1b,0a,aa */
	{0xaa, 0x1c, 0x00b0}, /* 00,1c,b0,aa */
	{0xa0, 0x05, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,05,cc */
	{0xa0, 0x35, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,35,cc */
	{0xa0, 0x70, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,70,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x85, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,85,cc */
	{0xa0, 0x58, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,58,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE}, /* 01,8c,0c,cc */
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,18,cc */
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN}, /* 01,a8,60,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x22, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,22,cc */
	{0xa0, 0x88, ZC3XX_R18D_YTARGET}, /* 01,8d,88,cc */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN}, /* 01,1d,58,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,42,cc */
	{}
};

static const struct usb_action po2030_60HZ[] = {
	{0xaa, 0x8d, 0x0008}, /* 00,8d,08,aa */
	{0xaa, 0x1a, 0x0000}, /* 00,1a,00,aa */
	{0xaa, 0x1b, 0x00de}, /* 00,1b,de,aa */
	{0xaa, 0x1c, 0x0040}, /* 00,1c,40,aa */
	{0xa0, 0x08, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,08,cc */
	{0xa0, 0xae, ZC3XX_R191_EXPOSURELIMITMID}, /* 01,91,ae,cc */
	{0xa0, 0x80, ZC3XX_R192_EXPOSURELIMITLOW}, /* 01,92,80,cc */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x6f, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,6f,cc */
	{0xa0, 0x20, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,20,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE}, /* 01,8c,0c,cc */
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE}, /* 01,8f,18,cc */
	{0xa0, 0x60, ZC3XX_R1A8_DIGITALGAIN}, /* 01,a8,60,cc */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,10,cc */
	{0xa0, 0x22, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,22,cc */
	{0xa0, 0x88, ZC3XX_R18D_YTARGET},		/* 01,8d,88,cc */
							/* win: 01,8d,80 */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc */
	{}
};

static const struct usb_action po2030_NoFliker[] = {
	{0xa0, 0x02, ZC3XX_R180_AUTOCORRECTENABLE}, /* 01,80,02,cc */
	{0xaa, 0x8d, 0x000d}, /* 00,8d,0d,aa */
	{0xaa, 0x1a, 0x0000}, /* 00,1a,00,aa */
	{0xaa, 0x1b, 0x0002}, /* 00,1b,02,aa */
	{0xaa, 0x1c, 0x0078}, /* 00,1c,78,aa */
	{0xaa, 0x46, 0x0000}, /* 00,46,00,aa */
	{0xaa, 0x15, 0x0000}, /* 00,15,00,aa */
	{}
};

static const struct usb_action tas5130c_InitialScale[] = {	/* 320x240 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x50, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x03, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x02, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x00, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},

	{0xa0, 0x04, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x0f, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x04, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x0f, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x06, ZC3XX_R08D_COMPABILITYMODE},
	{0xa0, 0xf7, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x07, ZC3XX_R0A5_EXPOSUREGAIN},
	{0xa0, 0x02, ZC3XX_R0A6_EXPOSUREBLACKLVL},
	{}
};
static const struct usb_action tas5130c_Initial[] = {	/* 640x480 */
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},
	{0xa0, 0x40, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x00, ZC3XX_R008_CLOCKSETTING},
	{0xa0, 0x02, ZC3XX_R010_CMOSSENSORSELECT},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x00, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},
	{0xa0, 0x05, ZC3XX_R098_WINYSTARTLOW},
	{0xa0, 0x0f, ZC3XX_R09A_WINXSTARTLOW},
	{0xa0, 0x05, ZC3XX_R11A_FIRSTYLOW},
	{0xa0, 0x0f, ZC3XX_R11C_FIRSTXLOW},
	{0xa0, 0xe6, ZC3XX_R09C_WINHEIGHTLOW},
	{0xa0, 0x02, ZC3XX_R09D_WINWIDTHHIGH},
	{0xa0, 0x86, ZC3XX_R09E_WINWIDTHLOW},
	{0xa0, 0x06, ZC3XX_R08D_COMPABILITYMODE},
	{0xa0, 0x37, ZC3XX_R101_SENSORCORRECTION},
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},
	{0xa0, 0x06, ZC3XX_R189_AWBSTATUS},
	{0xa0, 0x70, ZC3XX_R18D_YTARGET},
	{0xa0, 0x50, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x00, 0x01ad},
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},
	{0xa0, 0x07, ZC3XX_R0A5_EXPOSUREGAIN},
	{0xa0, 0x02, ZC3XX_R0A6_EXPOSUREBLACKLVL},
	{}
};
static const struct usb_action tas5130c_50HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0063}, /* 00,a4,63,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x63, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,63,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x04, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xfe, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x47, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,47,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x08, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xd3, ZC3XX_R01D_HSYNC_0}, /* 00,1d,d3,cc */
	{0xa0, 0xda, ZC3XX_R01E_HSYNC_1}, /* 00,1e,da,cc */
	{0xa0, 0xea, ZC3XX_R01F_HSYNC_2}, /* 00,1f,ea,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x03, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,03,cc */
	{0xa0, 0x4c, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action tas5130c_50HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0077}, /* 00,a4,77,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x77, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,77,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x07, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xd0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x7d, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,7d,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x08, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xf0, ZC3XX_R01D_HSYNC_0}, /* 00,1d,f0,cc */
	{0xa0, 0xf4, ZC3XX_R01E_HSYNC_1}, /* 00,1e,f4,cc */
	{0xa0, 0xf8, ZC3XX_R01F_HSYNC_2}, /* 00,1f,f8,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x03, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,03,cc */
	{0xa0, 0xc0, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action tas5130c_60HZ[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0036}, /* 00,a4,36,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x36, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,36,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x05, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x54, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x3e, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,3e,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x08, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xca, ZC3XX_R01D_HSYNC_0}, /* 00,1d,ca,cc */
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d0,cc */
	{0xa0, 0xe0, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e0,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x03, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,03,cc */
	{0xa0, 0x28, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action tas5130c_60HZScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0077}, /* 00,a4,77,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x77, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,77,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x09, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x47, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH}, /* 01,95,00,cc */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID}, /* 01,96,00,cc */
	{0xa0, 0x7d, ZC3XX_R197_ANTIFLICKERLOW}, /* 01,97,7d,cc */
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x08, ZC3XX_R1A9_DIGITALLIMITDIFF},
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0xc8, ZC3XX_R01D_HSYNC_0}, /* 00,1d,c8,cc */
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d0,cc */
	{0xa0, 0xe0, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e0,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x03, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,03,cc */
	{0xa0, 0x20, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};
static const struct usb_action tas5130c_NoFliker[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0040}, /* 00,a4,40,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x40, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,40,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x05, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0xa0, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,00,cc */
	{0xa0, 0xbc, ZC3XX_R01D_HSYNC_0}, /* 00,1d,bc,cc */
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d0,cc */
	{0xa0, 0xe0, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e0,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x02, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,02,cc */
	{0xa0, 0xf0, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};

static const struct usb_action tas5130c_NoFlikerScale[] = {
	{0xa0, 0x00, ZC3XX_R019_AUTOADJUSTFPS}, /* 00,19,00,cc */
	{0xaa, 0xa3, 0x0001}, /* 00,a3,01,aa */
	{0xaa, 0xa4, 0x0090}, /* 00,a4,90,aa */
	{0xa0, 0x01, ZC3XX_R0A3_EXPOSURETIMEHIGH}, /* 00,a3,01,cc */
	{0xa0, 0x90, ZC3XX_R0A4_EXPOSURETIMELOW}, /* 00,a4,90,cc */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH}, /* 01,90,00,cc */
	{0xa0, 0x0a, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x00, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},
	{0xa0, 0x04, ZC3XX_R197_ANTIFLICKERLOW},
	{0xa0, 0x0c, ZC3XX_R18C_AEFREEZE},
	{0xa0, 0x18, ZC3XX_R18F_AEUNFREEZE},
	{0xa0, 0x00, ZC3XX_R1A9_DIGITALLIMITDIFF}, /* 01,a9,00,cc */
	{0xa0, 0x00, ZC3XX_R1AA_DIGITALGAINSTEP}, /* 01,aa,00,cc */
	{0xa0, 0xbc, ZC3XX_R01D_HSYNC_0}, /* 00,1d,bc,cc */
	{0xa0, 0xd0, ZC3XX_R01E_HSYNC_1}, /* 00,1e,d0,cc */
	{0xa0, 0xe0, ZC3XX_R01F_HSYNC_2}, /* 00,1f,e0,cc */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3}, /* 00,20,ff,cc */
	{0xa0, 0x02, ZC3XX_R09F_MAXXHIGH}, /* 00,9f,02,cc */
	{0xa0, 0xf0, ZC3XX_R0A0_MAXXLOW},
	{0xa0, 0x50, ZC3XX_R11D_GLOBALGAIN},
	{}
};

/* from usbvm305.inf 0ac8:305b 07/06/15 (3 - tas5130c) */
static const struct usb_action gc0303_Initial[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc, */
	{0xa0, 0x02, ZC3XX_R008_CLOCKSETTING},		/* 00,08,02,cc, */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc, */
	{0xa0, 0x00, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc, */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc, */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc, */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,e0,cc, */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,98,cc, */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc, */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc, */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc, */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,00,cc, */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,00,cc, */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,00,cc, */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,00,cc, */
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},		/* 00,9c,e6,cc,
							 * 6<->8 */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,86,cc,
							 * 6<->8 */
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},		/* 00,87,10,cc, */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,98,cc, */
	{0xaa, 0x01, 0x0000},
	{0xaa, 0x1a, 0x0000},		/* 00,1a,00,aa, */
	{0xaa, 0x1c, 0x0017},		/* 00,1c,17,aa, */
	{0xaa, 0x1b, 0x0000},
	{0xa0, 0x82, ZC3XX_R086_EXPTIMEHIGH},		/* 00,86,82,cc, */
	{0xa0, 0x83, ZC3XX_R087_EXPTIMEMID},		/* 00,87,83,cc, */
	{0xa0, 0x84, ZC3XX_R088_EXPTIMELOW},		/* 00,88,84,cc, */
	{0xaa, 0x05, 0x0010},		/* 00,05,10,aa, */
	{0xaa, 0x0a, 0x0002},
	{0xaa, 0x0b, 0x0000},
	{0xaa, 0x0c, 0x0002},
	{0xaa, 0x0d, 0x0000},
	{0xaa, 0x0e, 0x0002},
	{0xaa, 0x0f, 0x0000},
	{0xaa, 0x10, 0x0002},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x16, 0x0001},		/* 00,16,01,aa, */
	{0xaa, 0x17, 0x00e8},		/* 00,17,e6,aa, (e6 -> e8) */
	{0xaa, 0x18, 0x0002},		/* 00,18,02,aa, */
	{0xaa, 0x19, 0x0088},		/* 00,19,86,aa, */
	{0xaa, 0x20, 0x0020},		/* 00,20,20,aa, */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,b7,cc, */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc, */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc, */
	{0xa0, 0x76, ZC3XX_R189_AWBSTATUS},		/* 01,89,76,cc, */
	{0xa0, 0x09, 0x01ad},				/* 01,ad,09,cc, */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc, */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc, */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc, */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc, */
	{0xa0, 0x58, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},			/* 01,16,61,cc, */
	{0xa0, 0x65, ZC3XX_R118_BGAIN},			/* 01,18,65,cc */
	{0xaa, 0x1b, 0x0000},
	{}
};

static const struct usb_action gc0303_InitialScale[] = {
	{0xa0, 0x01, ZC3XX_R000_SYSTEMCONTROL},		/* 00,00,01,cc, */
	{0xa0, 0x02, ZC3XX_R008_CLOCKSETTING},		/* 00,08,02,cc, */
	{0xa0, 0x01, ZC3XX_R010_CMOSSENSORSELECT},	/* 00,10,01,cc, */
	{0xa0, 0x10, ZC3XX_R002_CLOCKSELECT},
	{0xa0, 0x02, ZC3XX_R003_FRAMEWIDTHHIGH},	/* 00,03,02,cc, */
	{0xa0, 0x80, ZC3XX_R004_FRAMEWIDTHLOW},		/* 00,04,80,cc, */
	{0xa0, 0x01, ZC3XX_R005_FRAMEHEIGHTHIGH},	/* 00,05,01,cc, */
	{0xa0, 0xe0, ZC3XX_R006_FRAMEHEIGHTLOW},	/* 00,06,e0,cc, */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,98,cc, */
	{0xa0, 0x01, ZC3XX_R001_SYSTEMOPERATING},	/* 00,01,01,cc, */
	{0xa0, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,03,cc, */
	{0xa0, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,01,cc, */
	{0xa0, 0x00, ZC3XX_R098_WINYSTARTLOW},		/* 00,98,00,cc, */
	{0xa0, 0x00, ZC3XX_R09A_WINXSTARTLOW},		/* 00,9a,00,cc, */
	{0xa0, 0x00, ZC3XX_R11A_FIRSTYLOW},		/* 01,1a,00,cc, */
	{0xa0, 0x00, ZC3XX_R11C_FIRSTXLOW},		/* 01,1c,00,cc, */
	{0xa0, 0xe8, ZC3XX_R09C_WINHEIGHTLOW},		/* 00,9c,e8,cc,
							 * 8<->6 */
	{0xa0, 0x88, ZC3XX_R09E_WINWIDTHLOW},		/* 00,9e,88,cc,
							 * 8<->6 */
	{0xa0, 0x10, ZC3XX_R087_EXPTIMEMID},		/* 00,87,10,cc, */
	{0xa0, 0x98, ZC3XX_R08B_I2CDEVICEADDR},		/* 00,8b,98,cc, */
	{0xaa, 0x01, 0x0000},
	{0xaa, 0x1a, 0x0000},		/* 00,1a,00,aa, */
	{0xaa, 0x1c, 0x0017},		/* 00,1c,17,aa, */
	{0xaa, 0x1b, 0x0000},
	{0xa0, 0x82, ZC3XX_R086_EXPTIMEHIGH},	/* 00,86,82,cc, */
	{0xa0, 0x83, ZC3XX_R087_EXPTIMEMID},	/* 00,87,83,cc, */
	{0xa0, 0x84, ZC3XX_R088_EXPTIMELOW},	/* 00,88,84,cc, */
	{0xaa, 0x05, 0x0010},		/* 00,05,10,aa, */
	{0xaa, 0x0a, 0x0001},
	{0xaa, 0x0b, 0x0000},
	{0xaa, 0x0c, 0x0001},
	{0xaa, 0x0d, 0x0000},
	{0xaa, 0x0e, 0x0001},
	{0xaa, 0x0f, 0x0000},
	{0xaa, 0x10, 0x0001},
	{0xaa, 0x11, 0x0000},
	{0xaa, 0x16, 0x0001},		/* 00,16,01,aa, */
	{0xaa, 0x17, 0x00e8},		/* 00,17,e6,aa (e6 -> e8) */
	{0xaa, 0x18, 0x0002},		/* 00,18,02,aa, */
	{0xaa, 0x19, 0x0088},		/* 00,19,88,aa, */
	{0xa0, 0xb7, ZC3XX_R101_SENSORCORRECTION},	/* 01,01,b7,cc, */
	{0xa0, 0x05, ZC3XX_R012_VIDEOCONTROLFUNC},	/* 00,12,05,cc, */
	{0xa0, 0x0d, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0d,cc, */
	{0xa0, 0x76, ZC3XX_R189_AWBSTATUS},		/* 01,89,76,cc, */
	{0xa0, 0x09, 0x01ad},				/* 01,ad,09,cc, */
	{0xa0, 0x03, ZC3XX_R1C5_SHARPNESSMODE},		/* 01,c5,03,cc, */
	{0xa0, 0x13, ZC3XX_R1CB_SHARPNESS05},		/* 01,cb,13,cc, */
	{0xa0, 0x08, ZC3XX_R250_DEADPIXELSMODE},	/* 02,50,08,cc, */
	{0xa0, 0x08, ZC3XX_R301_EEPROMACCESS},		/* 03,01,08,cc, */
	{0xa0, 0x58, ZC3XX_R1A8_DIGITALGAIN},
	{0xa0, 0x61, ZC3XX_R116_RGAIN},		/* 01,16,61,cc, */
	{0xa0, 0x65, ZC3XX_R118_BGAIN},		/* 01,18,65,cc */
	{0xaa, 0x1b, 0x0000},
	{}
};
static const struct usb_action gc0303_50HZ[] = {
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0001},		/* 00,83,01,aa */
	{0xaa, 0x84, 0x0063},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc, */
	{0xa0, 0x06, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,0d,cc, */
	{0xa0, 0xa8, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,50,cc, */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc, */
	{0xa0, 0x47, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,47,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,8c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,15,cc, */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,10,cc, */
	{0xa0, 0x48, ZC3XX_R1AA_DIGITALGAINSTEP},
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,1d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,1e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc, */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc, */
	{0xa0, 0x7f, ZC3XX_R18D_YTARGET},
	{}
};

static const struct usb_action gc0303_50HZScale[] = {
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0003},		/* 00,83,03,aa */
	{0xaa, 0x84, 0x0054},		/* 00,84,54,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc, */
	{0xa0, 0x0d, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,0d,cc, */
	{0xa0, 0x50, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,50,cc, */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc, */
	{0xa0, 0x8e, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,8e,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,8c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,15,cc, */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,10,cc, */
	{0xa0, 0x48, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc, */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,1d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,1e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc, */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc, */
	{0xa0, 0x7f, ZC3XX_R18D_YTARGET},
	{}
};

static const struct usb_action gc0303_60HZ[] = {
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},
	{0xaa, 0x84, 0x003b},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc, */
	{0xa0, 0x05, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,91,05,cc, */
	{0xa0, 0x88, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,92,88,cc, */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc, */
	{0xa0, 0x3b, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,3b,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,8c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,15,cc, */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,a9,10,cc, */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,aa,24,cc, */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,1d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,1e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc, */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc, */
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},
	{}
};

static const struct usb_action gc0303_60HZScale[] = {
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},
	{0xaa, 0x84, 0x0076},
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc, */
	{0xa0, 0x0b, ZC3XX_R191_EXPOSURELIMITMID},	/* 01,1,0b,cc, */
	{0xa0, 0x10, ZC3XX_R192_EXPOSURELIMITLOW},	/* 01,2,10,cc, */
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,5,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,6,00,cc, */
	{0xa0, 0x76, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,7,76,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,f,15,cc, */
	{0xa0, 0x10, ZC3XX_R1A9_DIGITALLIMITDIFF},	/* 01,9,10,cc, */
	{0xa0, 0x24, ZC3XX_R1AA_DIGITALGAINSTEP},	/* 01,a,24,cc, */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,0,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,d,58,cc, */
	{0xa0, 0x42, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,42,cc, */
	{0xa0, 0x80, ZC3XX_R18D_YTARGET},
	{}
};

static const struct usb_action gc0303_NoFliker[] = {
	{0xa0, 0x0c, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0c,cc, */
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},		/* 00,83,00,aa */
	{0xaa, 0x84, 0x0020},		/* 00,84,20,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,0,00,cc, */
	{0xa0, 0x00, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x48, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc, */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,10,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,8c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,15,cc, */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,1d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,1e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc, */
	{0xa0, 0x03, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,03,cc */
	{}
};

static const struct usb_action gc0303_NoFlikerScale[] = {
	{0xa0, 0x0c, ZC3XX_R100_OPERATIONMODE},		/* 01,00,0c,cc, */
	{0xaa, 0x82, 0x0000},		/* 00,82,00,aa */
	{0xaa, 0x83, 0x0000},		/* 00,83,00,aa */
	{0xaa, 0x84, 0x0020},		/* 00,84,20,aa */
	{0xa0, 0x00, ZC3XX_R190_EXPOSURELIMITHIGH},	/* 01,90,00,cc, */
	{0xa0, 0x00, ZC3XX_R191_EXPOSURELIMITMID},
	{0xa0, 0x48, ZC3XX_R192_EXPOSURELIMITLOW},
	{0xa0, 0x00, ZC3XX_R195_ANTIFLICKERHIGH},	/* 01,95,00,cc, */
	{0xa0, 0x00, ZC3XX_R196_ANTIFLICKERMID},	/* 01,96,00,cc, */
	{0xa0, 0x10, ZC3XX_R197_ANTIFLICKERLOW},	/* 01,97,10,cc, */
	{0xa0, 0x0e, ZC3XX_R18C_AEFREEZE},		/* 01,8c,0e,cc, */
	{0xa0, 0x15, ZC3XX_R18F_AEUNFREEZE},		/* 01,8f,15,cc, */
	{0xa0, 0x62, ZC3XX_R01D_HSYNC_0},		/* 00,1d,62,cc, */
	{0xa0, 0x90, ZC3XX_R01E_HSYNC_1},		/* 00,1e,90,cc, */
	{0xa0, 0xc8, ZC3XX_R01F_HSYNC_2},		/* 00,1f,c8,cc, */
	{0xa0, 0xff, ZC3XX_R020_HSYNC_3},		/* 00,20,ff,cc, */
	{0xa0, 0x58, ZC3XX_R11D_GLOBALGAIN},		/* 01,1d,58,cc, */
	{0xa0, 0x03, ZC3XX_R180_AUTOCORRECTENABLE},	/* 01,80,03,cc */
	{}
};

static u8 reg_r(struct gspca_dev *gspca_dev,
		u16 index)
{
	int ret;

	if (gspca_dev->usb_err < 0)
		return 0;
	ret = usb_control_msg(gspca_dev->dev,
			usb_rcvctrlpipe(gspca_dev->dev, 0),
			0xa1,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x01,			/* value */
			index, gspca_dev->usb_buf, 1,
			500);
	if (ret < 0) {
		pr_err("reg_r err %d\n", ret);
		gspca_dev->usb_err = ret;
		return 0;
	}
	return gspca_dev->usb_buf[0];
}

static void reg_w(struct gspca_dev *gspca_dev,
			u8 value,
			u16 index)
{
	int ret;

	if (gspca_dev->usb_err < 0)
		return;
	ret = usb_control_msg(gspca_dev->dev,
			usb_sndctrlpipe(gspca_dev->dev, 0),
			0xa0,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value, index, NULL, 0,
			500);
	if (ret < 0) {
		pr_err("reg_w_i err %d\n", ret);
		gspca_dev->usb_err = ret;
	}
}

static u16 i2c_read(struct gspca_dev *gspca_dev,
			u8 reg)
{
	u8 retbyte;
	u16 retval;

	if (gspca_dev->usb_err < 0)
		return 0;
	reg_w(gspca_dev, reg, 0x0092);
	reg_w(gspca_dev, 0x02, 0x0090);			/* <- read command */
	msleep(20);
	retbyte = reg_r(gspca_dev, 0x0091);		/* read status */
	if (retbyte != 0x00)
		pr_err("i2c_r status error %02x\n", retbyte);
	retval = reg_r(gspca_dev, 0x0095);		/* read Lowbyte */
	retval |= reg_r(gspca_dev, 0x0096) << 8;	/* read Hightbyte */
	return retval;
}

static u8 i2c_write(struct gspca_dev *gspca_dev,
			u8 reg,
			u8 valL,
			u8 valH)
{
	u8 retbyte;

	if (gspca_dev->usb_err < 0)
		return 0;
	reg_w(gspca_dev, reg, 0x92);
	reg_w(gspca_dev, valL, 0x93);
	reg_w(gspca_dev, valH, 0x94);
	reg_w(gspca_dev, 0x01, 0x90);		/* <- write command */
	msleep(1);
	retbyte = reg_r(gspca_dev, 0x0091);		/* read status */
	if (retbyte != 0x00)
		pr_err("i2c_w status error %02x\n", retbyte);
	return retbyte;
}

static void usb_exchange(struct gspca_dev *gspca_dev,
			const struct usb_action *action)
{
	while (action->req) {
		switch (action->req) {
		case 0xa0:	/* write register */
			reg_w(gspca_dev, action->val, action->idx);
			break;
		case 0xa1:	/* read status */
			reg_r(gspca_dev, action->idx);
			break;
		case 0xaa:
			i2c_write(gspca_dev,
				  action->val,			/* reg */
				  action->idx & 0xff,		/* valL */
				  action->idx >> 8);		/* valH */
			break;
		case 0xbb:
			i2c_write(gspca_dev,
				  action->idx >> 8,		/* reg */
				  action->idx & 0xff,		/* valL */
				  action->val);			/* valH */
			break;
		default:
/*		case 0xdd:	 * delay */
			msleep(action->idx);
			break;
		}
		action++;
		msleep(1);
	}
}

static void setmatrix(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i;
	const u8 *matrix;
	static const u8 adcm2700_matrix[9] =
/*		{0x66, 0xed, 0xed, 0xed, 0x66, 0xed, 0xed, 0xed, 0x66}; */
/*ms-win*/
		{0x74, 0xed, 0xed, 0xed, 0x74, 0xed, 0xed, 0xed, 0x74};
	static const u8 gc0305_matrix[9] =
		{0x50, 0xf8, 0xf8, 0xf8, 0x50, 0xf8, 0xf8, 0xf8, 0x50};
	static const u8 hdcs2020_matrix[9] =
		{0x66, 0xed, 0xed, 0xed, 0x66, 0xed, 0xed, 0xed, 0x66};
	static const u8 ov7620_matrix[9] =
		{0x58, 0xf4, 0xf4, 0xf4, 0x58, 0xf4, 0xf4, 0xf4, 0x58};
	static const u8 pas202b_matrix[9] =
/*		{0x4c, 0xf5, 0xff, 0xf9, 0x51, 0xf5, 0xfb, 0xed, 0x5f}; */
		{0x3e, 0x01, 0x01, 0x00, 0x47, 0xf9, 0x02, 0xfc, 0x49};
	static const u8 po2030_matrix[9] =
		{0x60, 0xf0, 0xf0, 0xf0, 0x60, 0xf0, 0xf0, 0xf0, 0x60};
	static const u8 tas5130c_matrix[9] =
		{0x68, 0xec, 0xec, 0xec, 0x68, 0xec, 0xec, 0xec, 0x68};
	static const u8 gc0303_matrix[9] =
		{0x6c, 0xea, 0xea, 0xea, 0x6c, 0xea, 0xea, 0xea, 0x6c};
	static const u8 *matrix_tb[SENSOR_MAX] = {
		[SENSOR_ADCM2700] =	adcm2700_matrix,
		[SENSOR_CS2102] =	ov7620_matrix,
//		[SENSOR_CS2102K] =	NULL,
		[SENSOR_GC0303] =	gc0303_matrix,
		[SENSOR_GC0305] =	gc0305_matrix,
		[SENSOR_HDCS2020] =	hdcs2020_matrix,
		[SENSOR_HV7131B] =	gc0305_matrix,
		[SENSOR_HV7131R] =	po2030_matrix,
		[SENSOR_ICM105A] =	po2030_matrix,
		[SENSOR_MC501CB] =	po2030_matrix,
		[SENSOR_MT9V111_1] =	gc0305_matrix,
		[SENSOR_MT9V111_3] =	gc0305_matrix,
		[SENSOR_OV7620] =	ov7620_matrix,
		[SENSOR_OV7630C] =	NULL,
		[SENSOR_PAS106] =	NULL,
		[SENSOR_PAS202B] =	pas202b_matrix,
		[SENSOR_PB0330] =	gc0305_matrix,
		[SENSOR_PO2030] =	po2030_matrix,
		[SENSOR_TAS5130C] =	tas5130c_matrix,
	};

	matrix = matrix_tb[sd->sensor];
	if (matrix == NULL)
		return;		/* matrix already loaded */
	for (i = 0; i < ARRAY_SIZE(ov7620_matrix); i++)
		reg_w(gspca_dev, matrix[i], 0x010a + i);
}

static void setsharpness(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int sharpness;
	static const u8 sharpness_tb[][2] = {
		{0x02, 0x03},
		{0x04, 0x07},
		{0x08, 0x0f},
		{0x10, 0x1e}
	};

	sharpness = sd->ctrls[SHARPNESS].val;
	reg_w(gspca_dev, sharpness_tb[sharpness][0], 0x01c6);
	reg_r(gspca_dev, 0x01c8);
	reg_r(gspca_dev, 0x01c9);
	reg_r(gspca_dev, 0x01ca);
	reg_w(gspca_dev, sharpness_tb[sharpness][1], 0x01cb);
}

static void setcontrast(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	const u8 *Tgamma;
	int g, i, brightness, contrast, adj, gp1, gp2;
	u8 gr[16];
	static const u8 delta_b[16] =		/* delta for brightness */
		{0x50, 0x38, 0x2d, 0x28, 0x24, 0x21, 0x1e, 0x1d,
		 0x1d, 0x1b, 0x1b, 0x1b, 0x19, 0x18, 0x18, 0x18};
	static const u8 delta_c[16] =		/* delta for contrast */
		{0x2c, 0x1a, 0x12, 0x0c, 0x0a, 0x06, 0x06, 0x06,
		 0x04, 0x06, 0x04, 0x04, 0x03, 0x03, 0x02, 0x02};
	static const u8 gamma_tb[6][16] = {
		{0x00, 0x00, 0x03, 0x0d, 0x1b, 0x2e, 0x45, 0x5f,
		 0x79, 0x93, 0xab, 0xc1, 0xd4, 0xe5, 0xf3, 0xff},
		{0x01, 0x0c, 0x1f, 0x3a, 0x53, 0x6d, 0x85, 0x9c,
		 0xb0, 0xc2, 0xd1, 0xde, 0xe9, 0xf2, 0xf9, 0xff},
		{0x04, 0x16, 0x30, 0x4e, 0x68, 0x81, 0x98, 0xac,
		 0xbe, 0xcd, 0xda, 0xe4, 0xed, 0xf5, 0xfb, 0xff},
		{0x13, 0x38, 0x59, 0x79, 0x92, 0xa7, 0xb9, 0xc8,
		 0xd4, 0xdf, 0xe7, 0xee, 0xf4, 0xf9, 0xfc, 0xff},
		{0x20, 0x4b, 0x6e, 0x8d, 0xa3, 0xb5, 0xc5, 0xd2,
		 0xdc, 0xe5, 0xec, 0xf2, 0xf6, 0xfa, 0xfd, 0xff},
		{0x24, 0x44, 0x64, 0x84, 0x9d, 0xb2, 0xc4, 0xd3,
		 0xe0, 0xeb, 0xf4, 0xff, 0xff, 0xff, 0xff, 0xff},
	};

	Tgamma = gamma_tb[sd->ctrls[GAMMA].val - 1];

	contrast = ((int) sd->ctrls[CONTRAST].val - 128); /* -128 / 127 */
	brightness = ((int) sd->ctrls[BRIGHTNESS].val - 128); /* -128 / 92 */
	adj = 0;
	gp1 = gp2 = 0;
	for (i = 0; i < 16; i++) {
		g = Tgamma[i] + delta_b[i] * brightness / 256
				- delta_c[i] * contrast / 256 - adj / 2;
		if (g > 0xff)
			g = 0xff;
		else if (g < 0)
			g = 0;
		reg_w(gspca_dev, g, 0x0120 + i);	/* gamma */
		if (contrast > 0)
			adj--;
		else if (contrast < 0)
			adj++;
		if (i > 1)
			gr[i - 1] = (g - gp2) / 2;
		else if (i != 0)
			gr[0] = gp1 == 0 ? 0 : (g - gp1);
		gp2 = gp1;
		gp1 = g;
	}
	gr[15] = (0xff - gp2) / 2;
	for (i = 0; i < 16; i++)
		reg_w(gspca_dev, gr[i], 0x0130 + i);	/* gradient */
}

static void getexposure(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->sensor != SENSOR_HV7131R)
		return;
	sd->ctrls[EXPOSURE].val = (i2c_read(gspca_dev, 0x25) << 9)
		| (i2c_read(gspca_dev, 0x26) << 1)
		| (i2c_read(gspca_dev, 0x27) >> 7);
}

static void setexposure(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int val;

	if (sd->sensor != SENSOR_HV7131R)
		return;
	val = sd->ctrls[EXPOSURE].val;
	i2c_write(gspca_dev, 0x25, val >> 9, 0x00);
	i2c_write(gspca_dev, 0x26, val >> 1, 0x00);
	i2c_write(gspca_dev, 0x27, val << 7, 0x00);
}

static void setquality(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	s8 reg07;

	reg07 = 0;
	switch (sd->sensor) {
	case SENSOR_OV7620:
		reg07 = 0x30;
		break;
	case SENSOR_HV7131R:
	case SENSOR_PAS202B:
		return;			/* done by work queue */
	}
	reg_w(gspca_dev, sd->reg08, ZC3XX_R008_CLOCKSETTING);
	if (reg07 != 0)
		reg_w(gspca_dev, reg07, 0x0007);
}

/* Matches the sensor's internal frame rate to the lighting frequency.
 * Valid frequencies are:
 *	50Hz, for European and Asian lighting (default)
 *	60Hz, for American lighting
 *	0 = No Fliker (for outdoore usage)
 */
static void setlightfreq(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i, mode;
	const struct usb_action *zc3_freq;
	static const struct usb_action *freq_tb[SENSOR_MAX][6] = {
	[SENSOR_ADCM2700] =
		{adcm2700_NoFliker, adcm2700_NoFliker,
		 adcm2700_50HZ, adcm2700_50HZ,
		 adcm2700_60HZ, adcm2700_60HZ},
	[SENSOR_CS2102] =
		{cs2102_NoFliker, cs2102_NoFlikerScale,
		 cs2102_50HZ, cs2102_50HZScale,
		 cs2102_60HZ, cs2102_60HZScale},
//	[SENSOR_CS2102K] =
//		{cs2102k_NoFliker, cs2102k_NoFliker,
//		 cs2102k_50HZ, cs2102k_50HZ,
//		 cs2102k_60HZ, cs2102k_60HZ},
	[SENSOR_GC0303] =
		{gc0303_NoFliker, gc0303_NoFlikerScale,
		 gc0303_50HZ, gc0303_50HZScale,
		 gc0303_60HZ, gc0303_60HZScale},
	[SENSOR_GC0305] =
		{gc0305_NoFliker, gc0305_NoFliker,
		 gc0305_50HZ, gc0305_50HZ,
		 gc0305_60HZ, gc0305_60HZ},
	[SENSOR_HDCS2020] =
		{hdcs2020_NoFliker, hdcs2020_NoFliker,
		 hdcs2020_50HZ, hdcs2020_50HZ,
		 hdcs2020_60HZ, hdcs2020_60HZ},
	[SENSOR_HV7131B] =
		{hv7131b_NoFliker, hv7131b_NoFlikerScale,
		 hv7131b_50HZ, hv7131b_50HZScale,
		 hv7131b_60HZ, hv7131b_60HZScale},
	[SENSOR_HV7131R] =
		{hv7131r_NoFliker, hv7131r_NoFlikerScale,
		 hv7131r_50HZ, hv7131r_50HZScale,
		 hv7131r_60HZ, hv7131r_60HZScale},
	[SENSOR_ICM105A] =
		{icm105a_NoFliker, icm105a_NoFlikerScale,
		 icm105a_50HZ, icm105a_50HZScale,
		 icm105a_60HZ, icm105a_60HZScale},
	[SENSOR_MC501CB] =
		{mc501cb_NoFliker, mc501cb_NoFlikerScale,
		 mc501cb_50HZ, mc501cb_50HZScale,
		 mc501cb_60HZ, mc501cb_60HZScale},
	[SENSOR_MT9V111_1] =
		{mt9v111_1_AENoFliker, mt9v111_1_AENoFlikerScale,
		 mt9v111_1_AE50HZ, mt9v111_1_AE50HZScale,
		 mt9v111_1_AE60HZ, mt9v111_1_AE60HZScale},
	[SENSOR_MT9V111_3] =
		{mt9v111_3_AENoFliker, mt9v111_3_AENoFlikerScale,
		 mt9v111_3_AE50HZ, mt9v111_3_AE50HZScale,
		 mt9v111_3_AE60HZ, mt9v111_3_AE60HZScale},
	[SENSOR_OV7620] =
		{ov7620_NoFliker, ov7620_NoFliker,
		 ov7620_50HZ, ov7620_50HZ,
		 ov7620_60HZ, ov7620_60HZ},
	[SENSOR_OV7630C] =
		{NULL, NULL,
		 NULL, NULL,
		 NULL, NULL},
	[SENSOR_PAS106] =
		{pas106b_NoFliker, pas106b_NoFliker,
		 pas106b_50HZ, pas106b_50HZ,
		 pas106b_60HZ, pas106b_60HZ},
	[SENSOR_PAS202B] =
		{pas202b_NoFliker, pas202b_NoFlikerScale,
		 pas202b_50HZ, pas202b_50HZScale,
		 pas202b_60HZ, pas202b_60HZScale},
	[SENSOR_PB0330] =
		{pb0330_NoFliker, pb0330_NoFlikerScale,
		 pb0330_50HZ, pb0330_50HZScale,
		 pb0330_60HZ, pb0330_60HZScale},
	[SENSOR_PO2030] =
		{po2030_NoFliker, po2030_NoFliker,
		 po2030_50HZ, po2030_50HZ,
		 po2030_60HZ, po2030_60HZ},
	[SENSOR_TAS5130C] =
		{tas5130c_NoFliker, tas5130c_NoFlikerScale,
		 tas5130c_50HZ, tas5130c_50HZScale,
		 tas5130c_60HZ, tas5130c_60HZScale},
	};

	i = sd->ctrls[LIGHTFREQ].val * 2;
	mode = gspca_dev->cam.cam_mode[gspca_dev->curr_mode].priv;
	if (mode)
		i++;			/* 320x240 */
	zc3_freq = freq_tb[sd->sensor][i];
	if (zc3_freq == NULL)
		return;
	switch (sd->sensor) {
	case SENSOR_HV7131B:
	case SENSOR_HV7131R:
		reg_w(gspca_dev, 0x10,			/* reset level */
				ZC3XX_R180_AUTOCORRECTENABLE);
		break;
	}
	usb_exchange(gspca_dev, zc3_freq);
	switch (sd->sensor) {
	case SENSOR_GC0305:
		if (mode				/* if 320x240 */
		    && sd->ctrls[LIGHTFREQ].val == 1)	/* and 50Hz */
			reg_w(gspca_dev, 0x85, 0x018d);
					/* win: 0x80, 0x018d */
		break;
	case SENSOR_OV7620:
		if (!mode) {				/* if 640x480 */
			if (sd->ctrls[LIGHTFREQ].val != 0) /* and filter */
				reg_w(gspca_dev, 0x40, 0x0002);
			else
				reg_w(gspca_dev, 0x44, 0x0002);
		}
		break;
	case SENSOR_PAS202B:
		reg_w(gspca_dev, 0x00, 0x01a7);
		break;
	}
}

static void setautogain(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u8 autoval;

	if (sd->ctrls[AUTOGAIN].val)
		autoval = 0x42;
	else
		autoval = 0x02;
	reg_w(gspca_dev, autoval, 0x0180);
}

/* update the transfer parameters */
/* This function is executed from a work queue. */
/* The exact use of the bridge registers 07 and 08 is not known.
 * The following algorithm has been adapted from ms-win traces */
static void transfer_update(struct work_struct *work)
{
	struct sd *sd = container_of(work, struct sd, work);
	struct gspca_dev *gspca_dev = &sd->gspca_dev;
	int change, good;
	u8 reg07, reg11;

	/* synchronize with the main driver and initialize the registers */
	mutex_lock(&gspca_dev->usb_lock);
	reg07 = 0;					/* max */
	reg_w(gspca_dev, reg07, 0x0007);
	reg_w(gspca_dev, sd->reg08, ZC3XX_R008_CLOCKSETTING);
	mutex_unlock(&gspca_dev->usb_lock);

	good = 0;
	for (;;) {
		msleep(100);

		/* get the transfer status */
		/* the bit 0 of the bridge register 11 indicates overflow */
		mutex_lock(&gspca_dev->usb_lock);
		if (!gspca_dev->present || !gspca_dev->streaming)
			goto err;
		reg11 = reg_r(gspca_dev, 0x0011);
		if (gspca_dev->usb_err < 0
		 || !gspca_dev->present || !gspca_dev->streaming)
			goto err;

		change = reg11 & 0x01;
		if (change) {				/* overflow */
			switch (reg07) {
			case 0:				/* max */
				reg07 = sd->sensor == SENSOR_HV7131R
						? 0x30 : 0x32;
				if (sd->reg08 != 0) {
					change = 3;
					sd->reg08--;
				}
				break;
			case 0x32:
				reg07 -= 4;
				break;
			default:
				reg07 -= 2;
				break;
			case 2:
				change = 0;		/* already min */
				break;
			}
			good = 0;
		} else {				/* no overflow */
			if (reg07 != 0) {		/* if not max */
				good++;
				if (good >= 10) {
					good = 0;
					change = 1;
					reg07 += 2;
					switch (reg07) {
					case 0x30:
						if (sd->sensor == SENSOR_PAS202B)
							reg07 += 2;
						break;
					case 0x32:
					case 0x34:
						reg07 = 0;
						break;
					}
				}
			} else {			/* reg07 max */
				if (sd->reg08 < sizeof jpeg_qual - 1) {
					good++;
					if (good > 10) {
						sd->reg08++;
						change = 2;
					}
				}
			}
		}
		if (change) {
			if (change & 1) {
				reg_w(gspca_dev, reg07, 0x0007);
				if (gspca_dev->usb_err < 0
				 || !gspca_dev->present
				 || !gspca_dev->streaming)
					goto err;
			}
			if (change & 2) {
				reg_w(gspca_dev, sd->reg08,
						ZC3XX_R008_CLOCKSETTING);
				if (gspca_dev->usb_err < 0
				 || !gspca_dev->present
				 || !gspca_dev->streaming)
					goto err;
				sd->ctrls[QUALITY].val = jpeg_qual[sd->reg08];
				jpeg_set_qual(sd->jpeg_hdr,
						jpeg_qual[sd->reg08]);
			}
		}
		mutex_unlock(&gspca_dev->usb_lock);
	}
	return;
err:
	mutex_unlock(&gspca_dev->usb_lock);
}

static void send_unknown(struct gspca_dev *gspca_dev, int sensor)
{
	reg_w(gspca_dev, 0x01, 0x0000);		/* bridge reset */
	switch (sensor) {
	case SENSOR_PAS106:
		reg_w(gspca_dev, 0x03, 0x003a);
		reg_w(gspca_dev, 0x0c, 0x003b);
		reg_w(gspca_dev, 0x08, 0x0038);
		break;
	case SENSOR_ADCM2700:
	case SENSOR_GC0305:
	case SENSOR_OV7620:
	case SENSOR_MT9V111_1:
	case SENSOR_MT9V111_3:
	case SENSOR_PB0330:
	case SENSOR_PO2030:
		reg_w(gspca_dev, 0x0d, 0x003a);
		reg_w(gspca_dev, 0x02, 0x003b);
		reg_w(gspca_dev, 0x00, 0x0038);
		break;
	case SENSOR_HV7131R:
	case SENSOR_PAS202B:
		reg_w(gspca_dev, 0x03, 0x003b);
		reg_w(gspca_dev, 0x0c, 0x003a);
		reg_w(gspca_dev, 0x0b, 0x0039);
		if (sensor == SENSOR_PAS202B)
			reg_w(gspca_dev, 0x0b, 0x0038);
		break;
	}
}

/* start probe 2 wires */
static void start_2wr_probe(struct gspca_dev *gspca_dev, int sensor)
{
	struct sd *sd = (struct sd *) gspca_dev;

	reg_w(gspca_dev, 0x01, 0x0000);
	if (sd->sensor != SENSOR_GC0303)
		reg_w(gspca_dev, sensor, 0x0010);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0x03, 0x0012);
	reg_w(gspca_dev, 0x01, 0x0012);
/*	msleep(2); */
}

static int sif_probe(struct gspca_dev *gspca_dev)
{
	u16 checkword;

	start_2wr_probe(gspca_dev, 0x0f);		/* PAS106 */
	reg_w(gspca_dev, 0x08, 0x008d);
	msleep(150);
	checkword = ((i2c_read(gspca_dev, 0x00) & 0x0f) << 4)
			| ((i2c_read(gspca_dev, 0x01) & 0xf0) >> 4);
	PDEBUG(D_PROBE, "probe sif 0x%04x", checkword);
	if (checkword == 0x0007) {
		send_unknown(gspca_dev, SENSOR_PAS106);
		return 0x0f;			/* PAS106 */
	}
	return -1;
}

static int vga_2wr_probe(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	u16 retword;

	start_2wr_probe(gspca_dev, 0x00);	/* HV7131B */
	i2c_write(gspca_dev, 0x01, 0xaa, 0x00);
	retword = i2c_read(gspca_dev, 0x01);
	if (retword != 0)
		return 0x00;			/* HV7131B */

	if (sd->sensor != SENSOR_GC0303) {
		start_2wr_probe(gspca_dev, 0x04);	/* CS2102 */
		i2c_write(gspca_dev, 0x01, 0xaa, 0x00);
		retword = i2c_read(gspca_dev, 0x01);
		if (retword != 0)
			return 0x04;			/* CS2102 */
	}

	start_2wr_probe(gspca_dev, 0x06);	/* OmniVision */
	reg_w(gspca_dev, 0x08, 0x008d);
	i2c_write(gspca_dev, 0x11, 0xaa, 0x00);
	retword = i2c_read(gspca_dev, 0x11);
	if (retword != 0) {
		/* (should have returned 0xaa) --> Omnivision? */
		/* reg_r 0x10 -> 0x06 -->  */
		goto ov_check;
	}

	start_2wr_probe(gspca_dev, 0x08);	/* HDCS2020 */
	i2c_write(gspca_dev, 0x1c, 0x00, 0x00);
	i2c_write(gspca_dev, 0x15, 0xaa, 0x00);
	retword = i2c_read(gspca_dev, 0x15);
	if (retword != 0)
		return 0x08;			/* HDCS2020 */

	start_2wr_probe(gspca_dev, 0x0a);	/* PB0330 */
	i2c_write(gspca_dev, 0x07, 0xaa, 0xaa);
	retword = i2c_read(gspca_dev, 0x07);
	if (retword != 0)
		return 0x0a;			/* PB0330 */
	retword = i2c_read(gspca_dev, 0x03);
	if (retword != 0)
		return 0x0a;			/* PB0330 ?? */
	retword = i2c_read(gspca_dev, 0x04);
	if (retword != 0)
		return 0x0a;			/* PB0330 ?? */

	start_2wr_probe(gspca_dev, 0x0c);	/* ICM105A */
	i2c_write(gspca_dev, 0x01, 0x11, 0x00);
	retword = i2c_read(gspca_dev, 0x01);
	if (retword != 0)
		return 0x0c;			/* ICM105A */

	start_2wr_probe(gspca_dev, 0x0e);	/* PAS202BCB */
	reg_w(gspca_dev, 0x08, 0x008d);
	i2c_write(gspca_dev, 0x03, 0xaa, 0x00);
	msleep(50);
	retword = i2c_read(gspca_dev, 0x03);
	if (retword != 0) {
		send_unknown(gspca_dev, SENSOR_PAS202B);
		return 0x0e;			/* PAS202BCB */
	}

	start_2wr_probe(gspca_dev, 0x02);	/* TAS5130C */
	i2c_write(gspca_dev, 0x01, 0xaa, 0x00);
	retword = i2c_read(gspca_dev, 0x01);
	if (retword != 0)
		return 0x02;			/* TAS5130C */
	if (sd->sensor == SENSOR_GC0303)
		return -1;
ov_check:
	reg_r(gspca_dev, 0x0010);		/* ?? */
	reg_r(gspca_dev, 0x0010);

	reg_w(gspca_dev, 0x01, 0x0000);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0x06, 0x0010);		/* OmniVision */
	reg_w(gspca_dev, 0xa1, 0x008b);
	reg_w(gspca_dev, 0x08, 0x008d);
	msleep(500);
	reg_w(gspca_dev, 0x01, 0x0012);
	i2c_write(gspca_dev, 0x12, 0x80, 0x00);	/* sensor reset */
	retword = i2c_read(gspca_dev, 0x0a) << 8;
	retword |= i2c_read(gspca_dev, 0x0b);
	PDEBUG(D_PROBE, "probe 2wr ov vga 0x%04x", retword);
	switch (retword) {
	case 0x7631:				/* OV7630C */
		reg_w(gspca_dev, 0x06, 0x0010);
		break;
	case 0x7620:				/* OV7620 */
	case 0x7648:				/* OV7648 */
		break;
	default:
		return -1;			/* not OmniVision */
	}
	return retword;
}

struct sensor_by_chipset_revision {
	u16 revision;
	u8 internal_sensor_id;
};
static const struct sensor_by_chipset_revision chipset_revision_sensor[] = {
//fixme: test - don't use these values
//	{0xc000, 0x12},		/* TAS5130C */
	{0xc001, 0x13},		/* MT9V111 */
	{0xe001, 0x13},
	{0x8001, 0x13},
////	{0x8000, 0x14},		/* CS2102K */
//fixme: test - don't use these values
//	{0x8000, 0x14},		/* HDCS2020 */
//	{0x8400, 0x15},		/* MT9V111 */
//	{0xe400, 0x15},
};

static int vga_3wr_probe(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i;
	u16 retword;

/*fixme: lack of 8b=b3 (11,12)-> 10, 8b=e0 (14,15,16)-> 12 found in gspcav1*/
#if 0
	reg_w(gspca_dev, 0x00, ZC3XX_R010_CMOSSENSORSELECT);
	reg_w(gspca_dev, 0xe0, ZC3XX_R08B_I2CDEVICEADDR);
//inex in traces
	reg_w(gspca_dev, 0x03, ZC3XX_R012_VIDEOCONTROLFUNC);
	reg_w(gspca_dev, 0x01, ZC3XX_R012_VIDEOCONTROLFUNC);
	retword = i2c_read(gspca_dev, 0x14);
	if (retword != 0)
		return 0x12;			/* TAS5130C */
	retword = i2c_read(gspca_dev, 0x15);
	if (retword != 0)
		return 0x12;			/* TAS5130C */
	retword = i2c_read(gspca_dev, 0x16);
	if (retword != 0)
		return 0x12;			/* TAS5130C */
#endif

	if (sd->sensor != SENSOR_GC0303)
		reg_w(gspca_dev, 0x02, 0x0010);
	reg_r(gspca_dev, 0x0010);
	reg_w(gspca_dev, 0x01, 0x0000);
	reg_w(gspca_dev, 0x00, 0x0010);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0x91, 0x008b);
	reg_w(gspca_dev, 0x03, 0x0012);
	reg_w(gspca_dev, 0x01, 0x0012);
	reg_w(gspca_dev, 0x05, 0x0012);
	retword = i2c_read(gspca_dev, 0x14);
	if (retword != 0)
		return 0x11;			/* HV7131R */
	retword = i2c_read(gspca_dev, 0x15);
	if (retword != 0)
		return 0x11;			/* HV7131R */
	retword = i2c_read(gspca_dev, 0x16);
	if (retword != 0)
		return 0x11;			/* HV7131R */

	if (sd->sensor != SENSOR_GC0303)
		reg_w(gspca_dev, 0x02, 0x0010);
	retword = reg_r(gspca_dev, 0x000b) << 8;
	retword |= reg_r(gspca_dev, 0x000a);
	PDEBUG(D_PROBE, "probe 3wr vga 1 0x%04x", retword);
	reg_r(gspca_dev, 0x0010);
	if ((retword & 0xff00) == 0x6400)
		return 0x02;		/* TAS5130C */
	for (i = 0; i < ARRAY_SIZE(chipset_revision_sensor); i++) {
		if (chipset_revision_sensor[i].revision == retword) {
			sd->chip_revision = retword;
			send_unknown(gspca_dev, SENSOR_PB0330);
			return chipset_revision_sensor[i].internal_sensor_id;
		}
	}

	if (sd->sensor != SENSOR_GC0303) {
		reg_w(gspca_dev, 0x01, 0x0000);	/* check PB0330 */
		reg_w(gspca_dev, 0x01, 0x0001);
		reg_w(gspca_dev, 0xdd, 0x008b);
		reg_w(gspca_dev, 0x0a, 0x0010);
		reg_w(gspca_dev, 0x03, 0x0012);
		reg_w(gspca_dev, 0x01, 0x0012);
		retword = i2c_read(gspca_dev, 0x00);
		if (retword != 0) {
			PDEBUG(D_PROBE, "probe 3wr vga type 0a");
			return 0x0a;			/* PB0330 */
		}
	}

	/* probe gc0303 / gc0305 */
	reg_w(gspca_dev, 0x01, 0x0000);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0x98, 0x008b);
	reg_w(gspca_dev, 0x01, 0x0010);
	reg_w(gspca_dev, 0x03, 0x0012);
	msleep(2);
	reg_w(gspca_dev, 0x01, 0x0012);
	retword = i2c_read(gspca_dev, 0x00);
	if (retword != 0) {
		PDEBUG(D_PROBE, "probe 3wr vga type %02x", retword);
		if (retword == 0x0011)			/* gc0303 */
			return 0x0303;
		if (retword == 0x0029)			/* gc0305 */
			send_unknown(gspca_dev, SENSOR_GC0305);
		return retword;
	}

	reg_w(gspca_dev, 0x01, 0x0000);	/* check OmniVision */
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0xa1, 0x008b);
	reg_w(gspca_dev, 0x08, 0x008d);
	reg_w(gspca_dev, 0x06, 0x0010);
	reg_w(gspca_dev, 0x01, 0x0012);
	reg_w(gspca_dev, 0x05, 0x0012);
	if (i2c_read(gspca_dev, 0x1c) == 0x007f	/* OV7610 - manufacturer ID */
	    && i2c_read(gspca_dev, 0x1d) == 0x00a2) {
		send_unknown(gspca_dev, SENSOR_OV7620);
		return 0x06;		/* OmniVision confirm ? */
	}

	reg_w(gspca_dev, 0x01, 0x0000);
	reg_w(gspca_dev, 0x00, 0x0002);
	reg_w(gspca_dev, 0x01, 0x0010);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0xee, 0x008b);
	reg_w(gspca_dev, 0x03, 0x0012);
	reg_w(gspca_dev, 0x01, 0x0012);
	reg_w(gspca_dev, 0x05, 0x0012);
	retword = i2c_read(gspca_dev, 0x00) << 8;	/* ID 0 */
	retword |= i2c_read(gspca_dev, 0x01);		/* ID 1 */
	PDEBUG(D_PROBE, "probe 3wr vga 2 0x%04x", retword);
	if (retword == 0x2030) {
#ifdef GSPCA_DEBUG
		u8 retbyte;

		retbyte = i2c_read(gspca_dev, 0x02);	/* revision number */
		PDEBUG(D_PROBE, "sensor PO2030 rev 0x%02x", retbyte);
#endif
		send_unknown(gspca_dev, SENSOR_PO2030);
		return retword;
	}

	reg_w(gspca_dev, 0x01, 0x0000);
	reg_w(gspca_dev, 0x0a, 0x0010);
	reg_w(gspca_dev, 0xd3, 0x008b);
	reg_w(gspca_dev, 0x01, 0x0001);
	reg_w(gspca_dev, 0x03, 0x0012);
	reg_w(gspca_dev, 0x01, 0x0012);
	reg_w(gspca_dev, 0x05, 0x0012);
	reg_w(gspca_dev, 0xd3, 0x008b);
	retword = i2c_read(gspca_dev, 0x01);
	if (retword != 0) {
		PDEBUG(D_PROBE, "probe 3wr vga type 0a ? ret: %04x", retword);
		return 0x16;			/* adcm2700 (6100/6200) */
	}
	return -1;
}

static int zcxx_probeSensor(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int sensor;

	switch (sd->sensor) {
	case SENSOR_MC501CB:
		return -1;		/* don't probe */
	case SENSOR_PAS106:
		sensor =  sif_probe(gspca_dev);
		if (sensor >= 0)
			return sensor;
		break;
	}
	sensor = vga_2wr_probe(gspca_dev);
	if (sensor >= 0)
		return sensor;
	return vga_3wr_probe(gspca_dev);
}

/* this function is called at probe time */
static int sd_config(struct gspca_dev *gspca_dev,
			const struct usb_device_id *id)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (id->idProduct == 0x301b)
		sd->bridge = BRIDGE_ZC301;
	else
		sd->bridge = BRIDGE_ZC303;

	/* define some sensors from the vendor/product */
	sd->sensor = id->driver_info;

	gspca_dev->cam.ctrls = sd->ctrls;
	sd->reg08 = REG08_DEF;

	INIT_WORK(&sd->work, transfer_update);

	return 0;
}

/* this function is called at probe and resume time */
static int sd_init(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	struct cam *cam;
	int sensor;
	static const u8 gamma[SENSOR_MAX] = {
		[SENSOR_ADCM2700] =	4,
		[SENSOR_CS2102] =	4,
//		[SENSOR_CS2102K] =	5,
		[SENSOR_GC0303] =	3,
		[SENSOR_GC0305] =	4,
		[SENSOR_HDCS2020] =	4,
		[SENSOR_HV7131B] =	4,
		[SENSOR_HV7131R] =	4,
		[SENSOR_ICM105A] =	4,
		[SENSOR_MC501CB] =	4,
		[SENSOR_MT9V111_1] =	4,
		[SENSOR_MT9V111_3] =	4,
		[SENSOR_OV7620] =	3,
		[SENSOR_OV7630C] =	4,
		[SENSOR_PAS106] =	4,
		[SENSOR_PAS202B] =	4,
		[SENSOR_PB0330] =	4,
		[SENSOR_PO2030] =	4,
		[SENSOR_TAS5130C] =	3,
	};
	static const u8 mode_tb[SENSOR_MAX] = {
		[SENSOR_ADCM2700] =	2,
		[SENSOR_CS2102] =	1,
//		[SENSOR_CS2102K] =	1,
		[SENSOR_GC0303] =	1,
		[SENSOR_GC0305] =	1,
		[SENSOR_HDCS2020] =	1,
		[SENSOR_HV7131B] =	1,
		[SENSOR_HV7131R] =	1,
		[SENSOR_ICM105A] =	1,
		[SENSOR_MC501CB] =	2,
		[SENSOR_MT9V111_1] =	1,
		[SENSOR_MT9V111_3] =	1,
		[SENSOR_OV7620] =	2,
		[SENSOR_OV7630C] =	1,
		[SENSOR_PAS106] =	0,
		[SENSOR_PAS202B] =	1,
		[SENSOR_PB0330] =	1,
		[SENSOR_PO2030] =	1,
		[SENSOR_TAS5130C] =	1,
	};
	static const u8 reg08_tb[SENSOR_MAX] = {
		[SENSOR_ADCM2700] =	1,
		[SENSOR_CS2102] =	3,
//		[SENSOR_CS2102K] =	3,
		[SENSOR_GC0303] =	2,
		[SENSOR_GC0305] =	3,
		[SENSOR_HDCS2020] =	1,
		[SENSOR_HV7131B] =	3,
		[SENSOR_HV7131R] =	3,
		[SENSOR_ICM105A] =	3,
		[SENSOR_MC501CB] =	3,
		[SENSOR_MT9V111_1] =	3,
		[SENSOR_MT9V111_3] =	3,
		[SENSOR_OV7620] =	1,
		[SENSOR_OV7630C] =	3,
		[SENSOR_PAS106] =	3,
		[SENSOR_PAS202B] =	3,
		[SENSOR_PB0330] =	3,
		[SENSOR_PO2030] =	2,
		[SENSOR_TAS5130C] =	3,
	};

	sensor = zcxx_probeSensor(gspca_dev);
	if (sensor >= 0)
		PDEBUG(D_PROBE, "probe sensor -> %04x", sensor);
	if ((unsigned) force_sensor < SENSOR_MAX) {
		sd->sensor = force_sensor;
		PDEBUG(D_PROBE, "sensor forced to %d", force_sensor);
	} else {
		switch (sensor) {
		case -1:
			switch (sd->sensor) {
			case SENSOR_MC501CB:
				PDEBUG(D_PROBE, "Sensor MC501CB");
				break;
			case SENSOR_GC0303:
				PDEBUG(D_PROBE, "Sensor GC0303");
				break;
			default:
				pr_warn("Unknown sensor - set to TAS5130C\n");
				sd->sensor = SENSOR_TAS5130C;
			}
			break;
		case 0:
			/* check the sensor type */
			sensor = i2c_read(gspca_dev, 0x00);
			PDEBUG(D_PROBE, "Sensor hv7131 type %d", sensor);
			switch (sensor) {
			case 0:			/* hv7131b */
			case 1:			/* hv7131e */
				PDEBUG(D_PROBE, "Find Sensor HV7131B");
				sd->sensor = SENSOR_HV7131B;
				break;
			default:
/*			case 2:			 * hv7131r */
				PDEBUG(D_PROBE, "Find Sensor HV7131R");
				sd->sensor = SENSOR_HV7131R;
				break;
			}
			break;
		case 0x02:
			PDEBUG(D_PROBE, "Sensor TAS5130C");
			sd->sensor = SENSOR_TAS5130C;
			break;
		case 0x04:
			PDEBUG(D_PROBE, "Find Sensor CS2102");
			sd->sensor = SENSOR_CS2102;
			break;
		case 0x08:
			PDEBUG(D_PROBE, "Find Sensor HDCS2020");
			sd->sensor = SENSOR_HDCS2020;
			break;
		case 0x0a:
			PDEBUG(D_PROBE,
				"Find Sensor PB0330. Chip revision %x",
				sd->chip_revision);
			sd->sensor = SENSOR_PB0330;
			break;
		case 0x0c:
			PDEBUG(D_PROBE, "Find Sensor ICM105A");
			sd->sensor = SENSOR_ICM105A;
			break;
		case 0x0e:
			PDEBUG(D_PROBE, "Find Sensor PAS202B");
			sd->sensor = SENSOR_PAS202B;
			break;
		case 0x0f:
			PDEBUG(D_PROBE, "Find Sensor PAS106");
			sd->sensor = SENSOR_PAS106;
			break;
		case 0x10:
		case 0x12:
			PDEBUG(D_PROBE, "Find Sensor TAS5130C");
			sd->sensor = SENSOR_TAS5130C;
			break;
		case 0x11:
			PDEBUG(D_PROBE, "Find Sensor HV7131R");
			sd->sensor = SENSOR_HV7131R;
			break;
		case 0x13:
		case 0x15:
			PDEBUG(D_PROBE,
				"Sensor MT9V111. Chip revision %04x",
				sd->chip_revision);
			sd->sensor = sd->bridge == BRIDGE_ZC301
					? SENSOR_MT9V111_1
					: SENSOR_MT9V111_3;
			break;
		case 0x14:
			PDEBUG(D_PROBE,
				"Find Sensor HDCS2020. Chip revision %x",
				sd->chip_revision);
			sd->sensor = SENSOR_HDCS2020;
			break;
		case 0x16:
			PDEBUG(D_PROBE, "Find Sensor ADCM2700");
			sd->sensor = SENSOR_ADCM2700;
			break;
		case 0x29:
			PDEBUG(D_PROBE, "Find Sensor GC0305");
			sd->sensor = SENSOR_GC0305;
			break;
		case 0x0303:
			PDEBUG(D_PROBE, "Sensor GC0303");
			sd->sensor =  SENSOR_GC0303;
			break;
		case 0x2030:
			PDEBUG(D_PROBE, "Find Sensor PO2030");
			sd->sensor = SENSOR_PO2030;
			sd->ctrls[SHARPNESS].def = 0;	/* from win traces */
			break;
		case 0x7620:
			PDEBUG(D_PROBE, "Find Sensor OV7620");
			sd->sensor = SENSOR_OV7620;
			break;
		case 0x7631:
			PDEBUG(D_PROBE, "Find Sensor OV7630C");
			sd->sensor = SENSOR_OV7630C;
			break;
		case 0x7648:
			PDEBUG(D_PROBE, "Find Sensor OV7648");
			sd->sensor = SENSOR_OV7620;	/* same sensor (?) */
			break;
		default:
			pr_err("Unknown sensor %04x\n", sensor);
			return -EINVAL;
		}
	}
	if (sensor < 0x20) {
		if (sensor == -1 || sensor == 0x10 || sensor == 0x12)
			reg_w(gspca_dev, 0x02, 0x0010);
		reg_r(gspca_dev, 0x0010);
	}

	cam = &gspca_dev->cam;
	switch (mode_tb[sd->sensor]) {
	case 0:
		cam->cam_mode = sif_mode;
		cam->nmodes = ARRAY_SIZE(sif_mode);
		break;
	case 1:
		cam->cam_mode = vga_mode;
		cam->nmodes = ARRAY_SIZE(vga_mode);
		break;
	default:
/*	case 2: */
		cam->cam_mode = broken_vga_mode;
		cam->nmodes = ARRAY_SIZE(broken_vga_mode);
		break;
	}

	sd->ctrls[GAMMA].def = gamma[sd->sensor];
	sd->reg08 = reg08_tb[sd->sensor];
	sd->ctrls[QUALITY].def = jpeg_qual[sd->reg08];
	sd->ctrls[QUALITY].min = jpeg_qual[0];
	sd->ctrls[QUALITY].max = jpeg_qual[ARRAY_SIZE(jpeg_qual) - 1];

	switch (sd->sensor) {
	case SENSOR_HV7131R:
		gspca_dev->ctrl_dis = (1 << QUALITY);
		break;
	case SENSOR_OV7630C:
		gspca_dev->ctrl_dis = (1 << LIGHTFREQ) | (1 << EXPOSURE);
		break;
	case SENSOR_PAS202B:
		gspca_dev->ctrl_dis = (1 << QUALITY) | (1 << EXPOSURE);
		break;
	default:
		gspca_dev->ctrl_dis = (1 << EXPOSURE);
		break;
	}
#if AUTOGAIN_DEF
	if (sd->ctrls[AUTOGAIN].val)
		gspca_dev->ctrl_inac = (1 << EXPOSURE);
#endif

	/* switch off the led */
	reg_w(gspca_dev, 0x01, 0x0000);
	return gspca_dev->usb_err;
}

static int sd_start(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int mode;
	static const struct usb_action *init_tb[SENSOR_MAX][2] = {
	[SENSOR_ADCM2700] =
			{adcm2700_Initial, adcm2700_InitialScale},
	[SENSOR_CS2102]	=
			{cs2102_Initial, cs2102_InitialScale},
//	[SENSOR_CS2102K] =
//			{cs2102k_Initial, cs2102k_InitialScale},
	[SENSOR_GC0303] =
		{gc0303_Initial, gc0303_InitialScale},
	[SENSOR_GC0305] =
			{gc0305_Initial, gc0305_InitialScale},
	[SENSOR_HDCS2020] =
			{hdcs2020_Initial, hdcs2020_InitialScale},
	[SENSOR_HV7131B] =
			{hv7131b_Initial, hv7131b_InitialScale},
	[SENSOR_HV7131R] =
			{hv7131r_Initial, hv7131r_InitialScale},
	[SENSOR_ICM105A] =
			{icm105a_Initial, icm105a_InitialScale},
	[SENSOR_MC501CB] =
			{mc501cb_Initial, mc501cb_InitialScale},
	[SENSOR_MT9V111_1] =
			{mt9v111_1_Initial, mt9v111_1_InitialScale},
	[SENSOR_MT9V111_3] =
			{mt9v111_3_Initial, mt9v111_3_InitialScale},
	[SENSOR_OV7620] =
			{ov7620_Initial, ov7620_InitialScale},
	[SENSOR_OV7630C] =
			{ov7630c_Initial, ov7630c_InitialScale},
	[SENSOR_PAS106] =
			{pas106b_Initial, pas106b_InitialScale},
	[SENSOR_PAS202B] =
			{pas202b_Initial, pas202b_InitialScale},
	[SENSOR_PB0330] =
			{pb0330_Initial, pb0330_InitialScale},
	[SENSOR_PO2030] =
			{po2030_Initial, po2030_InitialScale},
	[SENSOR_TAS5130C] =
			{tas5130c_Initial, tas5130c_InitialScale},
	};

	/* create the JPEG header */
	jpeg_define(sd->jpeg_hdr, gspca_dev->height, gspca_dev->width,
			0x21);		/* JPEG 422 */

	mode = gspca_dev->cam.cam_mode[gspca_dev->curr_mode].priv;
	switch (sd->sensor) {
//	case SENSOR_HV7131R:
//		zcxx_probeSensor(gspca_dev);
//		break;
	case SENSOR_PAS106:
		usb_exchange(gspca_dev, pas106b_Initial_com);
		break;
	}
	usb_exchange(gspca_dev, init_tb[sd->sensor][mode]);

	switch (sd->sensor) {
	case SENSOR_ADCM2700:
	case SENSOR_GC0305:
	case SENSOR_OV7620:
	case SENSOR_PO2030:
	case SENSOR_TAS5130C:
	case SENSOR_GC0303:
/*		msleep(100);			 * ?? */
		reg_r(gspca_dev, 0x0002);	/* --> 0x40 */
		reg_w(gspca_dev, 0x09, 0x01ad);	/* (from win traces) */
		reg_w(gspca_dev, 0x15, 0x01ae);
		if (sd->sensor == SENSOR_TAS5130C)
			break;
		reg_w(gspca_dev, 0x0d, 0x003a);
		reg_w(gspca_dev, 0x02, 0x003b);
		reg_w(gspca_dev, 0x00, 0x0038);
		break;
	case SENSOR_HV7131R:
	case SENSOR_PAS202B:
		reg_w(gspca_dev, 0x03, 0x003b);
		reg_w(gspca_dev, 0x0c, 0x003a);
		reg_w(gspca_dev, 0x0b, 0x0039);
		if (sd->sensor == SENSOR_HV7131R)
			reg_w(gspca_dev, 0x50, ZC3XX_R11D_GLOBALGAIN);
		break;
	}

	setmatrix(gspca_dev);
	switch (sd->sensor) {
	case SENSOR_ADCM2700:
	case SENSOR_OV7620:
		reg_r(gspca_dev, 0x0008);
		reg_w(gspca_dev, 0x00, 0x0008);
		break;
	case SENSOR_PAS202B:
	case SENSOR_GC0305:
	case SENSOR_HV7131R:
	case SENSOR_TAS5130C:
		reg_r(gspca_dev, 0x0008);
		/* fall thru */
	case SENSOR_PO2030:
		reg_w(gspca_dev, 0x03, 0x0008);
		break;
	}
	setsharpness(gspca_dev);

	/* set the gamma tables when not set */
	switch (sd->sensor) {
	case SENSOR_OV7630C:		/* gamma set in xxx_Initial */
		break;
	default:
		setcontrast(gspca_dev);
		break;
	}
	setmatrix(gspca_dev);			/* one more time? */
	switch (sd->sensor) {
	case SENSOR_OV7620:
	case SENSOR_PAS202B:
		reg_r(gspca_dev, 0x0180);	/* from win */
		reg_w(gspca_dev, 0x00, 0x0180);
		break;
	}
	setquality(gspca_dev);
	jpeg_set_qual(sd->jpeg_hdr, jpeg_qual[sd->reg08]);
	setlightfreq(gspca_dev);

	switch (sd->sensor) {
	case SENSOR_ADCM2700:
		reg_w(gspca_dev, 0x09, 0x01ad);	/* (from win traces) */
		reg_w(gspca_dev, 0x15, 0x01ae);
		reg_w(gspca_dev, 0x02, 0x0180);
						/* ms-win + */
		reg_w(gspca_dev, 0x40, 0x0117);
		break;
	case SENSOR_HV7131R:
		setexposure(gspca_dev);
		reg_w(gspca_dev, 0x00, ZC3XX_R1A7_CALCGLOBALMEAN);
		break;
	case SENSOR_GC0303:
	case SENSOR_GC0305:
	case SENSOR_TAS5130C:
		reg_w(gspca_dev, 0x09, 0x01ad);	/* (from win traces) */
		reg_w(gspca_dev, 0x15, 0x01ae);
		/* fall thru */
	case SENSOR_PAS202B:
	case SENSOR_PO2030:
/*		reg_w(gspca_dev, 0x40, ZC3XX_R117_GGAIN); in win traces */
		reg_r(gspca_dev, 0x0180);
		break;
	case SENSOR_OV7620:
		reg_w(gspca_dev, 0x09, 0x01ad);
		reg_w(gspca_dev, 0x15, 0x01ae);
		i2c_read(gspca_dev, 0x13);	/*fixme: returns 0xa3 */
		i2c_write(gspca_dev, 0x13, 0xa3, 0x00);
					/*fixme: returned value to send? */
		reg_w(gspca_dev, 0x40, 0x0117);
		reg_r(gspca_dev, 0x0180);
		break;
	}

	setautogain(gspca_dev);

	/* start the transfer update thread if needed */
	if (gspca_dev->usb_err >= 0) {
		switch (sd->sensor) {
		case SENSOR_HV7131R:
		case SENSOR_PAS202B:
			sd->work_thread =
				create_singlethread_workqueue(KBUILD_MODNAME);
			queue_work(sd->work_thread, &sd->work);
			break;
		}
	}

	return gspca_dev->usb_err;
}

/* called on streamoff with alt 0 and on disconnect */
static void sd_stop0(struct gspca_dev *gspca_dev)
{
	struct sd *sd = (struct sd *) gspca_dev;

	if (sd->work_thread != NULL) {
		mutex_unlock(&gspca_dev->usb_lock);
		destroy_workqueue(sd->work_thread);
		mutex_lock(&gspca_dev->usb_lock);
		sd->work_thread = NULL;
	}
	if (!gspca_dev->present)
		return;
	send_unknown(gspca_dev, sd->sensor);
}

static void sd_pkt_scan(struct gspca_dev *gspca_dev,
			u8 *data,
			int len)
{
	struct sd *sd = (struct sd *) gspca_dev;

	/* check the JPEG end of frame */
	if (len >= 3
	 && data[len - 3] == 0xff && data[len - 2] == 0xd9) {
/*fixme: what does the last byte mean?*/
		gspca_frame_add(gspca_dev, LAST_PACKET,
					data, len - 1);
		return;
	}

	/* check the JPEG start of a frame */
	if (data[0] == 0xff && data[1] == 0xd8) {
		/* put the JPEG header in the new frame */
		gspca_frame_add(gspca_dev, FIRST_PACKET,
			sd->jpeg_hdr, JPEG_HDR_SZ);

		/* remove the webcam's header:
		 * ff d8 ff fe 00 0e 00 00 ss ss 00 01 ww ww hh hh pp pp
		 *	- 'ss ss' is the frame sequence number (BE)
		 *	- 'ww ww' and 'hh hh' are the window dimensions (BE)
		 *	- 'pp pp' is the packet sequence number (BE)
		 */
		data += 18;
		len -= 18;
	}
	gspca_frame_add(gspca_dev, INTER_PACKET, data, len);
}

static int sd_setautogain(struct gspca_dev *gspca_dev, __s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;

	sd->ctrls[AUTOGAIN].val = val;
	if (val) {
		gspca_dev->ctrl_inac |= (1 << EXPOSURE);
	} else {
		gspca_dev->ctrl_inac &= ~(1 << EXPOSURE);
		if (gspca_dev->streaming)
			getexposure(gspca_dev);
	}
	if (gspca_dev->streaming)
		setautogain(gspca_dev);
	return gspca_dev->usb_err;
}

static int sd_querymenu(struct gspca_dev *gspca_dev,
			struct v4l2_querymenu *menu)
{
	switch (menu->id) {
	case V4L2_CID_POWER_LINE_FREQUENCY:
		switch (menu->index) {
		case 0:		/* V4L2_CID_POWER_LINE_FREQUENCY_DISABLED */
			strcpy((char *) menu->name, "NoFliker");
			return 0;
		case 1:		/* V4L2_CID_POWER_LINE_FREQUENCY_50HZ */
			strcpy((char *) menu->name, "50 Hz");
			return 0;
		case 2:		/* V4L2_CID_POWER_LINE_FREQUENCY_60HZ */
			strcpy((char *) menu->name, "60 Hz");
			return 0;
		}
		break;
	}
	return -EINVAL;
}

static int sd_setquality(struct gspca_dev *gspca_dev, __s32 val)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(jpeg_qual) - 1; i++) {
		if (val <= jpeg_qual[i])
			break;
	}
	if (i > 0
	 && i == sd->reg08
	 && val < jpeg_qual[sd->reg08])
		i--;
	sd->reg08 = i;
	sd->ctrls[QUALITY].val = jpeg_qual[i];
	if (gspca_dev->streaming)
		jpeg_set_qual(sd->jpeg_hdr, sd->ctrls[QUALITY].val);
	return gspca_dev->usb_err;
}

static int sd_set_jcomp(struct gspca_dev *gspca_dev,
			struct v4l2_jpegcompression *jcomp)
{
	struct sd *sd = (struct sd *) gspca_dev;

	sd_setquality(gspca_dev, jcomp->quality);
	jcomp->quality = sd->ctrls[QUALITY].val;
	return gspca_dev->usb_err;
}

static int sd_get_jcomp(struct gspca_dev *gspca_dev,
			struct v4l2_jpegcompression *jcomp)
{
	struct sd *sd = (struct sd *) gspca_dev;

	memset(jcomp, 0, sizeof *jcomp);
	jcomp->quality = sd->ctrls[QUALITY].val;
	jcomp->jpeg_markers = V4L2_JPEG_MARKER_DHT
			| V4L2_JPEG_MARKER_DQT;
	return 0;
}

#if defined(CONFIG_INPUT) || defined(CONFIG_INPUT_MODULE)
static int sd_int_pkt_scan(struct gspca_dev *gspca_dev,
			u8 *data,		/* interrupt packet data */
			int len)		/* interrput packet length */
{
	if (len == 8 && data[4] == 1) {
		input_report_key(gspca_dev->input_dev, KEY_CAMERA, 1);
		input_sync(gspca_dev->input_dev);
		input_report_key(gspca_dev->input_dev, KEY_CAMERA, 0);
		input_sync(gspca_dev->input_dev);
	}

	return 0;
}
#endif

static const struct sd_desc sd_desc = {
	.name = KBUILD_MODNAME,
	.ctrls = sd_ctrls,
	.nctrls = ARRAY_SIZE(sd_ctrls),
	.config = sd_config,
	.init = sd_init,
	.start = sd_start,
	.stop0 = sd_stop0,
	.pkt_scan = sd_pkt_scan,
	.querymenu = sd_querymenu,
	.get_jcomp = sd_get_jcomp,
	.set_jcomp = sd_set_jcomp,
#if defined(CONFIG_INPUT) || defined(CONFIG_INPUT_MODULE)
	.int_pkt_scan = sd_int_pkt_scan,
#endif
};

static const struct usb_device_id device_table[] = {
	{USB_DEVICE(0x03f0, 0x1b07)},
	{USB_DEVICE(0x041e, 0x041e)},
	{USB_DEVICE(0x041e, 0x4017)},
	{USB_DEVICE(0x041e, 0x401c), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x041e, 0x401e)},
	{USB_DEVICE(0x041e, 0x401f)},
	{USB_DEVICE(0x041e, 0x4022)},
	{USB_DEVICE(0x041e, 0x4029)},
	{USB_DEVICE(0x041e, 0x4034), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x041e, 0x4035), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x041e, 0x4036)},
	{USB_DEVICE(0x041e, 0x403a)},
	{USB_DEVICE(0x041e, 0x4051), .driver_info = SENSOR_GC0303},
	{USB_DEVICE(0x041e, 0x4053), .driver_info = SENSOR_GC0303},
	{USB_DEVICE(0x0458, 0x7007)},
	{USB_DEVICE(0x0458, 0x700c)},
	{USB_DEVICE(0x0458, 0x700f)},
	{USB_DEVICE(0x0461, 0x0a00)},
	{USB_DEVICE(0x046d, 0x089d), .driver_info = SENSOR_MC501CB},
	{USB_DEVICE(0x046d, 0x08a0)},
	{USB_DEVICE(0x046d, 0x08a1)},
	{USB_DEVICE(0x046d, 0x08a2)},
	{USB_DEVICE(0x046d, 0x08a3)},
	{USB_DEVICE(0x046d, 0x08a6)},
	{USB_DEVICE(0x046d, 0x08a7)},
	{USB_DEVICE(0x046d, 0x08a9)},
	{USB_DEVICE(0x046d, 0x08aa)},
	{USB_DEVICE(0x046d, 0x08ac)},
	{USB_DEVICE(0x046d, 0x08ad)},
	{USB_DEVICE(0x046d, 0x08ae)},
	{USB_DEVICE(0x046d, 0x08af)},
	{USB_DEVICE(0x046d, 0x08b9)},
	{USB_DEVICE(0x046d, 0x08d7)},
	{USB_DEVICE(0x046d, 0x08d8)},
	{USB_DEVICE(0x046d, 0x08d9)},
	{USB_DEVICE(0x046d, 0x08da)},
	{USB_DEVICE(0x046d, 0x08dd), .driver_info = SENSOR_MC501CB},
	{USB_DEVICE(0x0471, 0x0325), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x0471, 0x0326), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x0471, 0x032d), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x0471, 0x032e), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x055f, 0xc005)},
	{USB_DEVICE(0x055f, 0xd003)},
	{USB_DEVICE(0x055f, 0xd004)},
	{USB_DEVICE(0x0698, 0x2003)},
	{USB_DEVICE(0x0ac8, 0x0301), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x0ac8, 0x0302), .driver_info = SENSOR_PAS106},
	{USB_DEVICE(0x0ac8, 0x301b)},
	{USB_DEVICE(0x0ac8, 0x303b)},
	{USB_DEVICE(0x0ac8, 0x305b)},
	{USB_DEVICE(0x0ac8, 0x307b)},
	{USB_DEVICE(0x10fd, 0x0128)},
	{USB_DEVICE(0x10fd, 0x804d)},
	{USB_DEVICE(0x10fd, 0x8050)},
	{}			/* end of entry */
};
MODULE_DEVICE_TABLE(usb, device_table);

/* -- device connect -- */
static int sd_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	return gspca_dev_probe(intf, id, &sd_desc, sizeof(struct sd),
				THIS_MODULE);
}

/* USB driver */
static struct usb_driver sd_driver = {
	.name = KBUILD_MODNAME,
	.id_table = device_table,
	.probe = sd_probe,
	.disconnect = gspca_disconnect,
#ifdef CONFIG_PM
	.suspend = gspca_suspend,
	.resume = gspca_resume,
	.reset_resume = gspca_resume,
#endif
};

module_usb_driver(sd_driver);

module_param(force_sensor, int, 0644);
MODULE_PARM_DESC(force_sensor,
	"Force sensor. Only for experts!!!");
