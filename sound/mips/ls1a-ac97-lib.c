/*
 * Based on sound/arm/ls1a-ac97.c and sound/soc/pxa/ls1a-ac97.c
 * which contain:
 *
 * Author:	Nicolas Pitre
 * Created:	Dec 02, 2004
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <sound/ac97_codec.h>
#include <sound/ls1a-lib.h>

//#include <mach/audio.h>

bool ls1a_ac97_try_warm_reset(struct snd_ac97 *ac97)
{
	return true;
}
EXPORT_SYMBOL_GPL(ls1a_ac97_try_warm_reset);

bool ls1a_ac97_try_cold_reset(struct snd_ac97 *ac97)
{
	return true;
}
EXPORT_SYMBOL_GPL(ls1a_ac97_try_cold_reset);


void ls1a_ac97_finish_reset(struct snd_ac97 *ac97)
{
	
}
EXPORT_SYMBOL_GPL(ls1a_ac97_finish_reset);

/*
static irqreturn_t ls1a_ac97_irq(int irq, void *dev_id)
{
	
	return IRQ_HANDLED;
}
*/
#ifdef CONFIG_PM
int ls1a_ac97_hw_suspend(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(ls1a_ac97_hw_suspend);

int ls1a_ac97_hw_resume(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(ls1a_ac97_hw_resume);
#endif

int __devinit ls1a_ac97_hw_probe(struct platform_device *dev)
{
	int ret = 0;

	return ret;
}
EXPORT_SYMBOL_GPL(ls1a_ac97_hw_probe);

void ls1a_ac97_hw_remove(struct platform_device *dev)
{

}
EXPORT_SYMBOL_GPL(ls1a_ac97_hw_remove);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("Intel/Marvell PXA sound library");
MODULE_LICENSE("GPL");
