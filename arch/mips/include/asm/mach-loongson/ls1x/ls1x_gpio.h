/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_GPIO_H__
#define __ASM_ARCH_GPIO_H__

//#define ARCH_NR_GPIOS		105
#include <asm-generic/gpio.h>

#define	GPIO_ID_NAME		"gpio"
/* use gpiolib dispatchers */
#if 0
#define gpio_get_value          __gpio_get_value
#define gpio_set_value          __gpio_set_value
#define gpio_cansleep           __gpio_cansleep
#endif
#define gpio_to_irq				__gpio_to_irq

/******* define by yg******************************************************/
//#ifdef CONFIG_CPU_LS1C  

#define GPIO_REG_CFG		0x0
#define GPIO_REG_DIR		0x10
#define GPIO_REG_EN			GPIO_REG_DIR  /*手册里把输入输出寄存器　叫做GPIO_EN,不直白*/
#define GPIO_REG_IN			0x20
#define GPIO_REG_OUT		0x30

#define GPIO_00     0
#define GPIO_01     1
#define GPIO_02     2
#define GPIO_03     3
#define GPIO_04     4
#define GPIO_05     5
#define GPIO_06     6
#define GPIO_07     7
#define GPIO_08     8
#define GPIO_09     9
#define GPIO_10    10 
#define GPIO_11    11 
#define GPIO_12    12
#define GPIO_13    13
#define GPIO_14    14
#define GPIO_15    15
#define GPIO_16    16
#define GPIO_17    17
#define GPIO_18    18
#define GPIO_19    19
#define GPIO_20    20
#define GPIO_21    21
#define GPIO_22    22
#define GPIO_23    23
#define GPIO_24    24
#define GPIO_25    25
#define GPIO_26    26
#define GPIO_27    27
#define GPIO_28    28
#define GPIO_29    29
#define GPIO_30    30
#define GPIO_31    31
#define GPIO_32    32
#define GPIO_33    33
#define GPIO_34    34
#define GPIO_35    35
#define GPIO_36    36
#define GPIO_37    37
#define GPIO_38    38
#define GPIO_39    39
#define GPIO_40    40
#define GPIO_41    41
#define GPIO_42    42
#define GPIO_43    43
#define GPIO_44    44
#define GPIO_45    45
#define GPIO_46    46
#define GPIO_47    47
#define GPIO_48    48
#define GPIO_49    49
#define GPIO_50    50
#define GPIO_51    51
#define GPIO_52    52
#define GPIO_53    53
#define GPIO_54    54
#define GPIO_55    55
#define GPIO_56    56
#define GPIO_57    57
#define GPIO_58    58
#define GPIO_59    59
#define GPIO_60    60
#define GPIO_61    61
#define GPIO_62    62
#define GPIO_63    63
#define GPIO_64    64
#define GPIO_65    65
#define GPIO_66    66
#define GPIO_67    67
#define GPIO_68    68
#define GPIO_69    69
#define GPIO_70	   70
#define GPIO_71    71
#define GPIO_72    72
#define GPIO_73    73
#define GPIO_74    74
#define GPIO_75    75
#define GPIO_76    76
#define GPIO_77    77
#define GPIO_78    78
#define GPIO_79    79
#define GPIO_80    80
#define GPIO_81    81
#define GPIO_82    82
#define GPIO_83    83
#define GPIO_84    84
#define GPIO_85    85
#define GPIO_86	   86
#define GPIO_87    87
#define GPIO_88    88
#define GPIO_89    89
#define GPIO_90    90
#define GPIO_91    91
#define GPIO_92    92
#define GPIO_93    93
#define GPIO_94    94
#define GPIO_95    95


#define GPIO_MAX  GPIO_95


#define GPIO_MIN  GPIO_00
#define GPIO_OUTPUT     0
#define GPIO_INPUT      1
#define GPIO_HIGH       1
#define GPIO_LOW        0


#define PIN_IDX(pin)        ((pin)/32)
#define PIN_OFFSET(pin)     ((pin)%32)
#define PIN_BITSET(pin)     (0x1<<((pin)%32))
//#define GPIO_BIT(x)    (1<<(x))
 

#define LS1C_BANK0_BASE				0xbfd010c0
#define LS1C_BANK1_BASE				0xbfd010c4
#define LS1C_BANK2_BASE				0xbfd010c8
//#define LS1C_BANK3_BASE				0xbfd010cc

#define LS1X_PAD_FUNC_MAX0_BASE	0xbfd011c0
#define LS1X_PAD_FUNC_MAX1_BASE	0xbfd011c4
#define LS1X_PAD_FUNC_MAX2_BASE	0xbfd011c8
//#define LS1X_PAD_FUNC_MAX3_BASE	0xbfd011cc

#define LS1C_BANK0_IRQ_START	64
#define LS1C_BANK1_IRQ_START	96
#define LS1C_BANK2_IRQ_START	128
//#define LS1C_BANK3_IRQ_START	54


#define GPIO_BANK_COUNT			3
#define GPIO_IRQ_START			64
#define GPIO_IRQ_END			159

enum PAD_FUNC_TYPE
{
	PAD_FUNC_1 = 1,
	PAD_FUNC_2,
	PAD_FUNC_3,
	PAD_FUNC_4,
	PAD_FUNC_5,
	PAD_FUNC_GPIO,
	PAD_FUNC_DEFAULT,
};


struct gpio_bank {
	unsigned long pbase;	//bank base = gpio base
	u32 pad_muxfunc_base;	//pad mux func base
	u32 virtual_irq_start;
	spinlock_t lock;
	struct gpio_chip chip;
	u32 mod_usage;
};
#if 0
struct gpio_bank gpio_bank_ls1c[GPIO_BANK_COUNT] = {
	{ LS1C_BANK0_BASE,  LS1X_PAD_FUNC_MAX0_BASE, LS1C_BANK0_IRQ_START},
	{ LS1C_BANK1_BASE,  LS1X_PAD_FUNC_MAX1_BASE, LS1C_BANK1_IRQ_START},
	{ LS1C_BANK2_BASE,  LS1X_PAD_FUNC_MAX2_BASE, LS1C_BANK2_IRQ_START},
//	{ LS1C_BANK3_BASE,  LS1X_PAD_FUNC_MAX3_BASE, LS1C_BANK3_IRQ_START}, 
};
#endif

int gpio_2irq(struct gpio_chip *chip, unsigned offset);
int gpio_output(struct gpio_chip *chip, unsigned offset, int value);
void gpio_set(struct gpio_chip *chip, unsigned offset, int value);
int gpio_input(struct gpio_chip *chip, unsigned offset);
int gpio_is_input(struct gpio_bank *bank, int offset);
int ls1c_gpio_request(struct gpio_chip *chip, unsigned offset);
void ls1c_gpio_free(struct gpio_chip *chip, unsigned offset);
int gpio_irq_type(unsigned irq, unsigned type);
void _set_gpio_irqenable(struct gpio_bank *bank, int gpio, int enable);
void _clear_gpio_irqstatus(struct gpio_bank *bank, int gpio);

#endif /* __ASM_ARCH_GPIO_H__ */
