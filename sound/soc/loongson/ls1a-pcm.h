/*
 * linux/sound/arm/ls1a-pcm.h -- ALSA PCM interface for the Intel PXA2xx chip
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LS1A_PCM_H
#define _LS1A_PCM_H

#include <asm/delay.h>

/* platform data */
extern struct snd_soc_platform_driver ls1a_soc_platform;

/*Zhuo Qixiang REGS OPRERATION*/


static inline u32 read_reg(volatile u32 * reg)
{
	return (*reg);
}
static inline void write_reg(volatile u32 * reg, u32 val)
{
	*(reg) = (val);
}

int ls1a_i2c_write ( unsigned char reg, unsigned short val);
unsigned short ls1a_i2c_read (unsigned short reg);


#endif
