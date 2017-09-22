/*
 * linux/sound/mips/ls1a-pcm.c -- ALSA PCM interface for the Loongson 1A chip
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	(C) 2004 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/ls1a-lib.h>

#include <linux/interrupt.h>

#include "ls1a-pcm.h"
#include "../../mips/ls1a-pcm.h"

#define SA_SHIRQ 0x00000080

#define GS_SOC_I2C_BASE    0xbfe58000           
#define GS_SOC_I2C_PRER_LO (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x0)
#define GS_SOC_I2C_PRER_HI (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x1)
#define GS_SOC_I2C_CTR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x2)
#define GS_SOC_I2C_TXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_RXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_CR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)
#define GS_SOC_I2C_SR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)

#define I2C_SINGLE 0 
#define I2C_BLOCK 1
#define I2C_SMB_BLOCK 2
#define I2C_HW_SINGLE 3 

#define uda1342_base    0xbfe60000

#define IISVERSION  (volatile unsigned int *)(uda1342_base + 0x0)
#define IISCONFIG   (volatile unsigned int *)(uda1342_base + 0x4)

unsigned short reg1[7] = {
        //    0x1c02,
        //    0x1a02,
        0x1402,
        //    0x1402,
        //    0x1002,
        //    0x1202,
        0x0014,
        //    0x0010,
        //    0xc003,
        //    0xff63,
        //    0xff00,
        0xff03,
        //    0xfc00,
        //    0xcf83,
        0x0000,
        0x0000,
        0x0130,
        0x042F,
};

static int ls1a_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls1a_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct ls1a_pcm_dma_params *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	int ret;

	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!dma)	//于ls1a-ac97.c里赋值
		return 0;
	
	/*请求dma_ch,释放dma_ch*/
	if (prtd->params != dma || prtd->params == NULL) {
		prtd->params = dma;
		
		ret = request_irq(LS1B_BOARD_DMA1_IRQ , ls1a_pcm_dma_irq, SA_SHIRQ,		///xsc
			"ac97dma-read", substream);
		if(ret < 0){
			printk("-+-+-+-+%s request_irq failed.\n", __func__);
			return ret;
		}
		
		ret = request_irq(LS1B_BOARD_DMA2_IRQ , ls1a_pcm_dma_irq, SA_SHIRQ,		///xsc
			"ac97dma-write", substream);
		if(ret < 0){
			printk("-+-+-+-+%s request_irq failed.\n", __func__);
			return ret;
		}
	}
	printk(KERN_ERR "-+-+-+-+Here in func:%s May show A LOT TIMES!\n", __func__);
	
	return __ls1a_pcm_hw_params(substream, params);
}

int	uda134x_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	unsigned short data1[7];
	unsigned char i;

	*(volatile unsigned int *)(0xbfd010c0) = 0x0;
	*(volatile unsigned int *)(0xbfd010c4) = 0x0;
	*(volatile unsigned int *)(0xbfd010c8) = 0x0;

	* GS_SOC_I2C_PRER_LO = 0x64;
	* GS_SOC_I2C_PRER_HI = 0;
	* GS_SOC_I2C_CTR = 0x80;

	ret = ls1a_i2c_write( 0x0 , reg1[0] );
	ret = ls1a_i2c_write( 0x1 , reg1[1] );
	ret = ls1a_i2c_write( 0x10, reg1[2] );
	ret = ls1a_i2c_write( 0x11, reg1[3] );
	ret = ls1a_i2c_write( 0x12, reg1[4] );
	ret = ls1a_i2c_write( 0x20, 0x84 );
	ret = ls1a_i2c_write( 0x21, 0x84 );

	data1[0] 	= ls1a_i2c_read(0x0  );
	data1[1] 	= ls1a_i2c_read(0x1  );
	data1[2] 	= ls1a_i2c_read(0x10 );
	data1[3] 	= ls1a_i2c_read(0x11 );
	data1[4] 	= ls1a_i2c_read(0x12 );
	data1[5] 	= ls1a_i2c_read(0x20 );
	data1[6] 	= ls1a_i2c_read(0x21 );

	* IISCONFIG = (16<<24) | (16<<16) | (0x27 /*rat_bitdiv*/ <<8) | (0x4 /*rat_cddiv*/ <<0) ;

	/// 检查I2C是否读写正确
	for (i=0; i<7; i++)
	{
		printk ("data1[%d] = 0x%x ---> 0x%x!\n", i, data1[i], reg1[i]);
	}
}


static int ls1a_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct ls1a_runtime_data *prtd = substream->runtime->private_data;

	free_irq(LS1B_BOARD_DMA1_IRQ,substream);
	free_irq(LS1B_BOARD_DMA2_IRQ,substream);

	__ls1a_pcm_hw_free(substream);

	return 0;
}

static struct snd_pcm_ops ls1a_pcm_ops = {
	.open		= __ls1a_pcm_open,
	.close		= __ls1a_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= ls1a_pcm_hw_params,
	.hw_free	= ls1a_pcm_hw_free,
	.prepare	= __ls1a_pcm_prepare,
	.trigger	= ls1a_pcm_trigger,
	.pointer	= ls1a_pcm_pointer,
};

static u64 ls1a_pcm_dmamask = DMA_BIT_MASK(32);

static int ls1a_soc_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;
	printk("func %s,line%i\n",__func__,__LINE__);
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &ls1a_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = ls1a_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret){	
			printk("func %s,line%i\n",__func__,__LINE__);
			goto out;
		}
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream ) {
		ret = ls1a_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret){			
			printk("func %s,line%i\n",__func__,__LINE__);
			goto out;
		}
	}
 out:
	return ret;
}

struct snd_soc_platform_driver ls1a_soc_platform = {
	.ops 		= &ls1a_pcm_ops,
	.pcm_new	= ls1a_soc_pcm_new,
	.pcm_free	= ls1a_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(ls1a_soc_platform);;

static int __devinit ls1a_soc_platform_probe(struct platform_device *pdev)
{
	int ret;
        ret	= snd_soc_register_platform(&pdev->dev, &ls1a_soc_platform);
	return ret;
}

static int __devexit ls1a_soc_platform_remove(struct platform_device *pdev)
{
        snd_soc_unregister_platform(&pdev->dev);
        return 0;
}

static struct platform_driver ls1a_pcm_driver = {
        .driver = {
                        .name = "ls1a-pcm-audio",
                        .owner = THIS_MODULE,
        },
        .probe = ls1a_soc_platform_probe,
        .remove = __devexit_p(ls1a_soc_platform_remove),
};

static int __init ls1a_soc_platform_init(void)
{
	return platform_driver_register(&ls1a_pcm_driver);
}
module_init(ls1a_soc_platform_init);

static void __exit ls1a_soc_platform_exit(void)
{
	platform_driver_unregister(&ls1a_pcm_driver);
}
module_exit(ls1a_soc_platform_exit);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("Intel PXA2xx PCM DMA module");
MODULE_LICENSE("GPL");
