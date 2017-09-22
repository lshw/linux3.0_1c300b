/*
 * linux/sound/ls1a-ac97.c -- AC97 support for the Loongson 1A chip.
 *
 * Author:	Nicolas Pitre
 * Created:	Dec 02, 2004
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/ac97_codec.h>
#include <sound/soc.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/ls1a-lib.h>

#include <linux/interrupt.h>
#include "ls1a-pcm.h"
#include "ls1a-ac97.h"

#if 0	///DEBUG
#define ZPRINTK(fmt, args...) printk(KERN_ALERT "<%s>: " fmt, __FUNCTION__ , ## args)
#else
#define ZPRINTK(fmt, args...)
#endif


#define GS_SOC_I2C_BASE    0xbfe58000  		///I2C 0 模块寄存器地址

#define GS_SOC_I2C_PRER_LO (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x0)
#define GS_SOC_I2C_PRER_HI (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x1)
#define GS_SOC_I2C_CTR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x2)
#define GS_SOC_I2C_TXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_RXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_CR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)
#define GS_SOC_I2C_SR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)


#define CR_START 0x80
#define CR_WRITE 0x10
#define SR_NOACK 0x80
#define CR_STOP  0x40
#define SR_BUSY  0x40 
#define CR_START 0x80
#define SR_TIP   0x2

#define CR_READ  0x20
#define I2C_WACK 0x8

#define uda1342_base    0xbfe60000

#define IISSTATE  (volatile unsigned int *)(uda1342_base + 0x8)

int ls1a_i2c_write ( unsigned char reg, unsigned short val)
{
	unsigned char value;

	/*write slave_addr*/
	* GS_SOC_I2C_TXR = UDA_DEV_ADDR;
	* GS_SOC_I2C_CR  = CR_START|CR_WRITE; 	/* start on first addr */
	while(*GS_SOC_I2C_SR & SR_TIP);

	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	* GS_SOC_I2C_TXR = reg++;   //lxy: reg++ after write once
	* GS_SOC_I2C_CR  = CR_WRITE;
	while(*GS_SOC_I2C_SR & SR_TIP);
	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	value = (val >> 8); * GS_SOC_I2C_TXR = value;
	* GS_SOC_I2C_CR  = CR_WRITE;
	//                printk ("hi data = 0x%x ,", value);
	while(*GS_SOC_I2C_SR & SR_TIP);
	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	value = val;
	* GS_SOC_I2C_TXR = value;
	* GS_SOC_I2C_CR  = CR_WRITE;

	while(*GS_SOC_I2C_SR & SR_TIP);
	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	* GS_SOC_I2C_CR = CR_WRITE|CR_STOP;
	while(*GS_SOC_I2C_SR & SR_TIP);

	while(*GS_SOC_I2C_SR & SR_BUSY);
	return 1;
}
EXPORT_SYMBOL_GPL(ls1a_i2c_write);

unsigned short ls1a_i2c_read (unsigned short reg)
{
	unsigned short retval;		//!!
	unsigned char value;

	/*write slave_addr*/
	* GS_SOC_I2C_TXR = UDA_DEV_ADDR;
	* GS_SOC_I2C_CR  = CR_START|CR_WRITE; 		/* start on first addr */
	while(*GS_SOC_I2C_SR & SR_TIP);

	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	* GS_SOC_I2C_TXR = reg++;   			//lxy reg++ after read once
	* GS_SOC_I2C_CR  = CR_WRITE;
	while(*GS_SOC_I2C_SR & SR_TIP);
	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	/*write slave_addr*/
	* GS_SOC_I2C_TXR = UDA_DEV_ADDR|1;
	* GS_SOC_I2C_CR  = CR_START|CR_WRITE; /* start on first addr */
	while(*GS_SOC_I2C_SR & SR_TIP);

	if((* GS_SOC_I2C_SR) & SR_NOACK) return -1;

	* GS_SOC_I2C_CR  = CR_READ; /*read send ack*/
	while(*GS_SOC_I2C_SR & SR_TIP);
	value = * GS_SOC_I2C_TXR;
	retval =  value << 8;

	* GS_SOC_I2C_CR  = CR_READ|I2C_WACK; /*last read not send ack*/
	while(*GS_SOC_I2C_SR & SR_TIP);
	value = * GS_SOC_I2C_TXR;
	retval |= value;

	* GS_SOC_I2C_CR  = CR_STOP;
	while(*GS_SOC_I2C_SR & SR_TIP);
	while(*GS_SOC_I2C_SR & SR_BUSY);

	return retval;		/// count = 1
}
EXPORT_SYMBOL_GPL(ls1a_i2c_read);


static void ls1a_ac97_cold_reset(struct snd_ac97 *ac97)
{
	udelay(5000);

	ls1a_i2c_write( 0, 0);
	udelay(5000);		//FIXME,This is very very necessary,Zhuo Qixiang!
	printk(KERN_ALERT "+++++++++ Here Enter into %s.+++++++++\n", __func__);
}


static struct ls1a_pcm_dma_params ls1a_ac97_pcm_stereo_out = {
	.name			= "AC97 PCM Stereo out",
	.dev_addr		= 0x1fe60010 | (1<< 31),		///0x0fe72420 | (1<<31) | (0<<30)//播放,
};

static struct ls1a_pcm_dma_params ls1a_ac97_pcm_stereo_in = {
	.name			= "AC97 PCM Stereo in",
	.dev_addr		= 0x1fe6000c,				///0x0fe74c4c | (1<<31) | (0<<30)//录音,
									///IISRxData 地址是 0xbfe6_000c 后面在设置通用DMA的时候
									///设置为 dma_daddr
};


static int ls1a_ac97_probe(struct snd_soc_dai *dai)
{
	ls1a_ac97_cold_reset(NULL);

	printk(KERN_ERR "%s--------------\n",__func__);
	return 0;
}

static int ls1a_ac97_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ls1a_pcm_dma_params *dma_data;

	//采样位数;设置声道对于I2S 如何设置???
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			ls1a_ac97_pcm_stereo_out.dev_addr |= (1<<28);
			break;
		default:
			printk("NOTICE!___May Not Surpport!\n");
			break;
		}
		dma_data = &ls1a_ac97_pcm_stereo_out;
		ls1a_i2c_write( 0x20, 0x84);
		ls1a_i2c_write( 0x21, 0x84);
                * IISSTATE = 0x0d280;   	///0x0200;
	}
	else{
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			ls1a_ac97_pcm_stereo_in.dev_addr |= (1<<28);
			break;
		default:
			printk("capture NOTICE!___May Not Surpport!\n");
			break;
		}
		dma_data = &ls1a_ac97_pcm_stereo_in;
		ls1a_i2c_write( 0x20, 0x30);
		ls1a_i2c_write( 0x21, 0x30);
                * IISSTATE = 0x0e800;   	///0x0200;
	}
	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);

	return 0;
}

#define LS1A_AC97_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

struct snd_soc_ops ls1a_ac97_hifi_dai_ops = {
	.hw_params	= &ls1a_ac97_hw_params,
};

#define AC97_FORMATS (SNDRV_PCM_FMTBIT_S8 |\
                               SNDRV_PCM_FMTBIT_S16_LE |\
                               SNDRV_PCM_FMTBIT_S16_BE |\
                               SNDRV_PCM_FMTBIT_S20_3LE |\
                               SNDRV_PCM_FMTBIT_S20_3BE |\
                               SNDRV_PCM_FMTBIT_S24_3LE |\
                               SNDRV_PCM_FMTBIT_S24_3BE |\
                               SNDRV_PCM_FMTBIT_S32_LE |\
			       SNDRV_PCM_FMTBIT_U16_LE	|\
                               SNDRV_PCM_FMTBIT_S32_BE)


/*
 * There is only 1 physical AC97 interface for ls1a, but it
 * has extra fifo's that can be used for aux DACs and ADCs.
 */
struct snd_soc_dai_driver ls1a_ac97_dai[] = {
	{
		.name = "ls1a-ac97",
		.id = 0,
		//.ac97_control = 1,
		.probe = &ls1a_ac97_probe,
		.playback = {
			.stream_name = "AC97 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = LS1A_AC97_RATES,
			.formats = AC97_FORMATS,},
		.capture = {
			.stream_name = "AC97 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = LS1A_AC97_RATES,
			.formats = AC97_FORMATS  /*SNDRV_PCM_FMTBIT_S16_LE*/ ,},
	},
};

EXPORT_SYMBOL_GPL(ls1a_ac97_dai);

static int __devinit ls1a_ac97_dev_probe(struct platform_device *pdev)
{
	
	/* Punt most of the init to the SoC probe; we may need the machine
	 * driver to do interesting things with the clocking to get us up
	 * and running.
	 */
	return snd_soc_register_dais(&pdev->dev,ls1a_ac97_dai, ARRAY_SIZE(ls1a_ac97_dai));
}

static int __devexit ls1a_ac97_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dais(ls1a_ac97_dai, ARRAY_SIZE(ls1a_ac97_dai));

	return 0;
}

static struct platform_driver ls1a_ac97_driver = {
        .probe          = ls1a_ac97_dev_probe,
        .remove         = __devexit_p(ls1a_ac97_dev_remove),
        .driver         = {
                .name   = "ls1a-ac97",
                .owner  = THIS_MODULE,
        },
};

static int __init ls1a_ac97_init(void)
{
	pr_debug("Entered %s\n", __func__);
	return platform_driver_register(&ls1a_ac97_driver);
	pr_debug("Exit %s\n", __func__);
}
module_init(ls1a_ac97_init);

static void __exit ls1a_ac97_exit(void)
{
	pr_debug("Entered %s\n", __func__);
	platform_driver_unregister(&ls1a_ac97_driver);
	pr_debug("Exit %s\n", __func__);
}
module_exit(ls1a_ac97_exit);


MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("AC97 driver for the Intel PXA2xx chip");
MODULE_LICENSE("GPL");
