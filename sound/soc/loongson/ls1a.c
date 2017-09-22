/*
 * ls1a.c  --  SoC audio for ls1a
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "ls1a-pcm.h"
#include "ls1a-ac97.h"

static struct 	snd_soc_card ls1a;
extern struct 	snd_soc_ops ls1a_ac97_hifi_dai_ops;
extern int	uda134x_init(struct snd_soc_pcm_runtime *rtd);

static struct snd_soc_dai_link ls1a_dai[] = {
	{
		.name = "AC97",
		.stream_name = "LS1A<-->ALC203",

		.platform_name	= "ls1a-pcm-audio",
		.cpu_dai_name ="ls1a-ac97",
		.codec_name	= "uda134x-codec",	
		.codec_dai_name ="uda134x-hifi",
		.ops	= &ls1a_ac97_hifi_dai_ops,
		.init	= uda134x_init,
	},
};

static struct snd_soc_card ls1a = {
	.name = "LS1A",
	.owner= THIS_MODULE,
	.dai_link = ls1a_dai,
	.num_links = ARRAY_SIZE(ls1a_dai),
};


/*zqx----platform_device代表整个音频子系统*/
static struct platform_device *ls1a_snd_ac97_device;

static int __init ls1a_init(void)
{
	int ret;

	pr_debug("Entered %s\n", __func__);
	ls1a_snd_ac97_device = platform_device_alloc("soc-audio", -1);
	if (!ls1a_snd_ac97_device){
		printk("err platform devoce alloc failed!\n");
		return -ENOMEM;
	}

	platform_set_drvdata(ls1a_snd_ac97_device, &ls1a);

	ret = platform_device_add(ls1a_snd_ac97_device);

	if (ret)
		platform_device_put(ls1a_snd_ac97_device);

	return ret;
}

static void __exit ls1a_exit(void)
{
	pr_debug("Entered %s\n", __func__);
	platform_device_unregister(ls1a_snd_ac97_device);
	pr_debug("Exited %s\n", __func__);
}

module_init(ls1a_init);
module_exit(ls1a_exit);

/* Module information */
MODULE_AUTHOR("Zhuo Qixiang");
MODULE_DESCRIPTION("ALSA SoC LS1A(SB2F)");
MODULE_LICENSE("GPL");

