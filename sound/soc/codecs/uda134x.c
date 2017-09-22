/*
 * uda134x.c  --  UDA134X ALSA SoC Codec driver
 *
 * Modifications by Christian Pellegrin <chripell@evolware.org>
 *
 * Copyright 2007 Dension Audio Systems Ltd.
 * Author: Zoltan Devai
 *
 * Based on the WM87xx drivers by Liam Girdwood and Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>

#include <sound/uda134x.h>
#include <sound/l3.h>

#include "uda134x.h"


#define UDA134X_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
                SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 |\
                SNDRV_PCM_RATE_48000)

#define UDA134X_FORMATS (SNDRV_PCM_FMTBIT_S8 |\
                               SNDRV_PCM_FMTBIT_S16_LE |\
                               SNDRV_PCM_FMTBIT_S16_BE |\
                               SNDRV_PCM_FMTBIT_S20_3LE |\
                               SNDRV_PCM_FMTBIT_S20_3BE |\
                               SNDRV_PCM_FMTBIT_S24_3LE |\
                               SNDRV_PCM_FMTBIT_S24_3BE |\
                               SNDRV_PCM_FMTBIT_S32_LE |\
			       SNDRV_PCM_FMTBIT_U16_LE	|\
                               SNDRV_PCM_FMTBIT_S32_BE)

extern int ls1a_i2c_write ( unsigned char reg, unsigned short val);
extern unsigned short ls1a_i2c_read (unsigned short reg);

/* In-data addresses are hard-coded into the reg-cache values */
static const char uda134x_reg[UDA134X_REGS_NUM] = {
	/* Extended address registers */
	0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* Status, data regs */
	0x00, 0x83, 0x00, 0x40, 0x80, 0xC0, 0x00,
};

/*
 * The codec has no support for reading its registers except for peak level...
 */
static inline unsigned int uda134x_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = (u8 *)uda134x_reg;
	if (reg >= UDA134X_REGS_NUM)
		return -1;
	return cache[reg];
}


/*
 * The codec has no support for reading its registers except for peak level...
 */
/*
	FIXME
	i2c read unsigned short;but uda134x_read int.   
*/
static inline unsigned int uda134x_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	return 	ls1a_i2c_read(reg );
}


/* * Write to the uda134x registers */
static int uda134x_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	ls1a_i2c_write(reg, value);
	return 0;
}

static struct snd_soc_dai_driver uda134x_dai = {
	.name = "uda134x-hifi",
	//.ac97_control = 1,		//这个代表着 ac97 设备有platform data 
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = UDA134X_RATES,
		.formats = UDA134X_FORMATS,
	},
	/* capture capabilities */
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = UDA134X_RATES,
		.formats = UDA134X_FORMATS,
	},
};

static inline void uda134x_reset(struct snd_soc_codec *codec)
{
	u8 reset_reg = uda134x_read_reg_cache(codec, UDA134X_STATUS0);
	uda134x_write(codec, UDA134X_STATUS0, reset_reg | (1<<6));
	msleep(1);
	uda134x_write(codec, UDA134X_STATUS0, reset_reg & ~(1<<6));
}

static int uda134x_soc_probe(struct snd_soc_codec *codec)
{
	u8 reg;
	struct snd_ac97_bus *ac97_bus;
	struct snd_ac97_template ac97_template;
	int ret;
	
	uda134x_reset(codec);
	reg = uda134x_read_reg_cache(codec, UDA134X_STATUS1);
	uda134x_write(codec, UDA134X_STATUS1, reg | 0x03);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_uda134x = {
	.probe = uda134x_soc_probe,
	.read = uda134x_read,
	.write = uda134x_write,
};

static int __devexit uda134x_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}


static int __devinit uda134x_codec_probe(struct platform_device *pdev)
{
        return snd_soc_register_codec(&pdev->dev,
                        &soc_codec_dev_uda134x, &uda134x_dai, 1);
}

static struct platform_driver uda134x_codec_driver = {
        .driver = {
                .name = "uda134x-codec",
                .owner = THIS_MODULE,
        },
        .probe = uda134x_codec_probe,
        .remove = __devexit_p(uda134x_codec_remove),
};

static int __init uda134x_codec_init(void)
{
	return platform_driver_register(&uda134x_codec_driver);
}
module_init(uda134x_codec_init);

static void __exit uda134x_codec_exit(void)
{
	platform_driver_unregister(&uda134x_codec_driver);
}
module_exit(uda134x_codec_exit);

MODULE_DESCRIPTION("UDA134X ALSA soc codec driver");
MODULE_AUTHOR("Zoltan Devai, Christian Pellegrin <chripell@evolware.org>");
MODULE_LICENSE("GPL");
