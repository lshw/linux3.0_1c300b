/*
 *
 * Support functions for ls1c GPIO
 *
 * Copyright (C) 2003-2005 Loongson Corporation
 * Written by Sunyoung_yg <yangguang@loongson.cn>
 *
 * Copyright (C) 2015 Loongson China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Ls1c have 105 gpios, but gpio96~104 is mux with sdram pin, now we can't
 * test and use these pins.
 * So we assume the gpio number range is 0~95.
 * the change relevant data: 
 * GPIO_BANK_COUNT,
 * struct gpio_bank gpio_bank_ls1c,
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <asm/irq.h>

#include <ls1x_gpio.h>

/*
 * ls1c specific GPIO registers
 */
#if 1
 struct gpio_bank gpio_bank_ls1c[GPIO_BANK_COUNT] = {
	{ LS1C_BANK0_BASE,  LS1X_PAD_FUNC_MAX0_BASE, LS1C_BANK0_IRQ_START},
	{ LS1C_BANK1_BASE,  LS1X_PAD_FUNC_MAX1_BASE, LS1C_BANK1_IRQ_START},
	{ LS1C_BANK2_BASE,  LS1X_PAD_FUNC_MAX2_BASE, LS1C_BANK2_IRQ_START},
//	{ LS1C_BANK3_BASE,  LS1X_PAD_FUNC_MAX3_BASE, LS1C_BANK3_IRQ_START}, 
};
#endif


struct gpio_bank *gpio_bank;

int gpio_2irq(struct gpio_chip *chip, unsigned offset);
int irq_to_gpio(unsigned irq);

inline struct gpio_bank *get_gpio_bank(int gpio)
{
	if(PIN_IDX(gpio) > GPIO_BANK_COUNT)
		return NULL;
	return &gpio_bank[PIN_IDX(gpio)];	
}

 inline int get_gpio_index(int gpio)
{
	return PIN_OFFSET(gpio);
}

/***************************************************************************
 * Description:
 * Parameters: 
 * Rutern  : 
 * Author  :Sunyoung_yg 
 * Date    : 2015-07-31
 ***************************************************************************/
 inline int gpio_valid(int gpio)
{
	if ((gpio < 0) || (gpio> GPIO_MAX))  /*104 is max gpio num*/
		return -1;
	return 0;
}

 int check_gpio(struct gpio_bank *bank, int offset)
{
	unsigned int gpio = irq_to_gpio(gpio_2irq(&bank->chip, offset)); 

	if (unlikely(gpio_valid(gpio))) {
		printk(KERN_ERR "ls1c-gpio: invalid GPIO %d\n", gpio);
		return -1;
	}
	return 0;
}
/***************************************************************************
 * Description: set gpio direction input or output
 * Parameters:  bank: 抽象的一组32位gpio （同一个寄存器）　
 *				gpio: gpio no. or gpio offset 
 * Author  :Sunyoung_yg 
 * Date    : 2015-02-11
 ***************************************************************************/

void _set_gpio_direction(struct gpio_bank *bank, int offset, int is_input)
{
	u32 reg = bank->pbase+GPIO_REG_DIR;
	u32 l;
	if (check_gpio(bank, offset) < 0)
		return -EINVAL;
	if (!(bank->mod_usage & (1<<offset)))
		return -EINVAL;

	l = __raw_readl(reg);
	if (is_input)
		l |= PIN_BITSET(offset);
	else
		l &= ~PIN_BITSET(offset);
	__raw_writel(l, reg);
}
/***************************************************************************
 * Description: set gpio value
 * Parameters: offset: gpio no.  or  gpio offset; enable: gpio value
 * Rutern  :void
 * Author  :Sunyoung_yg
 * Date    : 2015-02-12
 ***************************************************************************/
 void _set_gpio_dataout(struct gpio_bank *bank, int offset, int enable)
{
	u32 reg = bank->pbase+GPIO_REG_OUT;
	u32 l = 0;
	if (check_gpio(bank, offset) < 0)
		return -EINVAL;
	if (!(bank->mod_usage & (1<<offset)))
		return -EINVAL;

	l = __raw_readl(reg);
	if(enable)
		l |= PIN_BITSET(offset);
	else
		l &= ~(PIN_BITSET(offset));
	__raw_writel(l, reg);
}
/***************************************************************************
 * Description: read gpio input level
 * Parameters: 
 * Rutern  : 
 * Author  :Sunyoung_yg 
 * Date    : 2015-02-12
 ***************************************************************************/
 int _get_gpio_datain(struct gpio_bank *bank, int offset)
{
	u32 reg;
	if (check_gpio(bank, offset) < 0)
		return -EINVAL;
	if (!(bank->mod_usage & (1<<offset)))
		return -EINVAL;

	reg = bank->pbase + GPIO_REG_IN;
	return (__raw_readl(reg) & (1<<offset));
}
/***************************************************************************
 * Description: read gpio output  level
 * Parameters:
 * Rutern  :
 * Author  :Sunyoung_yg
 * Date    : 2015-02-12
 ***************************************************************************/

 int _get_gpio_dataout(struct gpio_bank *bank, int offset)
{
	u32 reg;
	if (check_gpio(bank, offset) < 0)
		return -EINVAL;
	if (!(bank->mod_usage & (1<<offset)))
		return -EINVAL;

	reg = bank->pbase + GPIO_REG_OUT;
	return (__raw_readl(reg) & (1<<offset));
}



/*
 * Note that ENAWAKEUP needs to be enabled in GPIO_SYSCONFIG register.
 * 1510 does not seem to have a wake-up register. If JTAG is connected
 * to the target, system will wake up always on GPIO events. While
 * system is running all registered GPIO interrupts need to have wake-up
 * enabled. When system is suspended, only selected GPIO interrupts need
 * to have wake-up enabled.
 */
int _set_gpio_wakeup(struct gpio_bank *bank, int gpio, int enable)
{

}
/***************************************************************************
 * Description: reset gpio,set gpio input, and disable irq, clear irq status.
 * Parameters:
 * Author  :Sunyoung_yg
 * Date    : 2015-02-11
 ***************************************************************************/

void _reset_gpio(struct gpio_bank *bank, int gpio)
{
	_set_gpio_direction(bank, get_gpio_index(gpio), GPIO_INPUT);
}

/***************************************************************************
 * Description:	set gpio function
 * Parameters: 
 * Author  :Sunyoung_yg 
 * Date    : 2015-02-11
 ***************************************************************************/
int set_pad_func(struct gpio_chip *chip, unsigned offset, unsigned int type)
{
	u32 flags,reg,i;
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);

	spin_lock_irqsave(&bank->lock, flags);
	/*清除5个复用功能配置寄存器 pad 对应位*/
	for(i=0; i<5; i++)  
	{
		reg = __raw_readl(bank->pad_muxfunc_base+ i*0x10);
		reg &= ~(1<<offset);
		__raw_writel(reg,bank->pad_muxfunc_base+ i*0x10);
	}

	switch(type)
	{
		case PAD_FUNC_1:
		case PAD_FUNC_2:
		case PAD_FUNC_3:
		case PAD_FUNC_4:
		case PAD_FUNC_5:
			/*置位　对应复用功能寄存器的位*/
			reg = __raw_readl(bank->pad_muxfunc_base+ (type-1)*0x10);
			reg |= (1<<offset);
			__raw_writel(reg,bank->pad_muxfunc_base+ (type-1)*0x10);
			break;
		case PAD_FUNC_GPIO:
			reg = __raw_readl(bank->pbase);
			reg |= (1<<offset);
			__raw_writel(reg,bank->pbase);
			break;
		case PAD_FUNC_DEFAULT: 
			reg = __raw_readl(bank->pbase);
			reg &= ~(1<<offset);
			__raw_writel(reg,bank->pbase);
			break;
		default:
			break;
	}

	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}
/***************************************************************************
 * Description:
 * Parameters:	chip:抽象出来的一个控制器，实际使同使用一个寄存器的gpio集合
 *				offset: gpio 在32位寄存器中　偏移量
 * Author  :Sunyoung_yg 
 * Date    : 2015-02-11
 ***************************************************************************/

int ls1c_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	unsigned long flags;
	int ret;
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);

	if (check_gpio(bank, offset) < 0)
		return -EINVAL;
	spin_lock_irqsave(&bank->lock, flags);
	/* Set trigger to none. You need to enable the desired trigger with
	 * request_irq() or set_irq_type().
	 */
	if (!(bank->mod_usage & (1<<offset))) {
		set_pad_func(chip, offset, PAD_FUNC_GPIO);
		bank->mod_usage |= 1 << offset;
		ret = 0;
	}
	else
		ret = -EINVAL;
	spin_unlock_irqrestore(&bank->lock, flags);

	return ret;
}

void ls1c_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);
	unsigned int tmp, flags;

	spin_lock_irqsave(&bank->lock, flags);

	bank->mod_usage &= ~(1 << offset);
	tmp = __raw_readl(bank->pbase + GPIO_REG_CFG);
	__raw_writel((tmp & ~(1<<offset)), bank->pbase + GPIO_REG_CFG);
	_reset_gpio(bank, bank->chip.base + offset);
	spin_unlock_irqrestore(&bank->lock, flags);
}


/*---------------------------------------------------------------------*/

/* REVISIT these are stupid implementations!  replace by ones that
 * don't switch on METHOD_* and which mostly avoid spinlocks
 */
/***************************************************************************
 * Description: set gpio direction is input
 * Parameters: chip: offset:
 * Author  :Sunyoung_yg
 * Date    : 2015-02-11
 ***************************************************************************/

int gpio_input(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	unsigned long flags;
	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_direction(bank, offset, GPIO_INPUT);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

/***************************************************************************
 * Description:
 * Parameters:
 * Rutern  : 1:input ;  0:output
 * Author  :Sunyoung_yg
 * Date    : 2015-02-12
 ***************************************************************************/
int gpio_is_input(struct gpio_bank *bank, int offset)
{
	u32 reg = bank->pbase+GPIO_REG_DIR;
	if(__raw_readl(reg) & offset)  /*ls1c gpio: 0 is output; 1 is input*/
		return GPIO_INPUT;
	else 
		return GPIO_OUTPUT;
}
/***************************************************************************
 * Description: get gpio level 
 * Parameters: 
 * Author  : Sunyoung_yg 
 * Date    : 2015-02-11
 ***************************************************************************/

int gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	void __iomem *reg;
	int gpio;
	u32 mask;
	bank = container_of(chip, struct gpio_bank, chip);

	if (gpio_is_input(bank, offset))
		return _get_gpio_datain(bank, offset);
	else
		return _get_gpio_dataout(bank, offset);
}
/***************************************************************************
 * Description: set gpio direction is output, and set gpio value.
 * Parameters: value : default is 0.
 * Rutern  : 0
 * Author  :Sunyoung_yg
 * Date    : 2015-02-12
 ***************************************************************************/

int gpio_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_dataout(bank, offset, value);
	_set_gpio_direction(bank, offset, GPIO_OUTPUT);
	spin_unlock_irqrestore(&bank->lock, flags);
	return 0;
}

/***************************************************************************
 * Description:
 * Parameters: 
 * Rutern  : 
 * Author  :Sunyoung_yg 
 * Date    : 2015-02-12
 ***************************************************************************/

void gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	unsigned long flags;

	bank = container_of(chip, struct gpio_bank, chip);
	spin_lock_irqsave(&bank->lock, flags);
	_set_gpio_dataout(bank, offset, value);
	spin_unlock_irqrestore(&bank->lock, flags);
}
/***************************************************************************
 * Description: because in gpiolib.c have the function "gpio_to_irq",and
 *              it is equlvalent to __gpio_to_irq ,defined in ls1x_gpio.h ,
 *              it invoke this function(gpio_2irq)finally.
 * Notice     : ls1c gpio0~95 int reg is int2~int4 regs, but gpio96~104 is in
 *				int1 register; So gpio0~95 irq number is 64~159, and gpio96~104
 *				irq number is 54~62. Now we ignore gpio96~104.
 * Parameters :
 * Rutern	  :
 * Author     :Sunyoung_yg
 * Date       : 2015-07-31
 ***************************************************************************/

int gpio_2irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;

	bank = container_of(chip, struct gpio_bank, chip);
//	if (bank->virtual_irq_start == LS1C_BANK3_IRQ_START)
//		if(offset > LS1C_GPIO_MAX%32) return -EINVAL;
	if (offset > 31)
		return -EINVAL;
	return bank->virtual_irq_start + offset;
}

int irq_to_gpio(unsigned irq)
{
	int i;
	if ( (irq < GPIO_IRQ_START)||(irq > GPIO_IRQ_END ))
		return -EINVAL;
	return irq - GPIO_IRQ_START;

#if 0
	struct gpio_bank *bank;
	for(i=0;i<GPIO_BANK_COUNT;i++)
	{
		bank = &gpio_bank[i];
		if(irq>=bank->virtual_irq_start && irq<bank->virtual_irq_start+32)
		break;
	}
	if(i == GPIO_BANK_COUNT)
		return -EINVAL;
	return i*32 + (irq - bank->virtual_irq_start);
#endif
}

/*---------------------------------------------------------------------*/

 int initialized;


/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

 int __init _ls1c_gpio_init(void)
{
	int i;
	int gpio = 0;
	struct gpio_bank *bank;

	initialized = 1;

	gpio_bank = gpio_bank_ls1c;

	for (i = 0; i < GPIO_BANK_COUNT; i++) {
		int j, gpio_count = 32;

		bank = &gpio_bank[i];
		spin_lock_init(&bank->lock);
		bank->mod_usage = 0;
		/* REVISIT eventually switch from ls1c-specific gpio structs
		 * over to the generic ones
		 */
		bank->chip.request = ls1c_gpio_request;
		bank->chip.free = ls1c_gpio_free;
		bank->chip.direction_input = gpio_input;
		bank->chip.get = gpio_get;
		bank->chip.direction_output = gpio_output;
		bank->chip.set = gpio_set;
		bank->chip.to_irq = gpio_2irq;

		bank->chip.label = "gpio";
		bank->chip.base = gpio;
		gpio += gpio_count;

		bank->chip.ngpio = gpio_count;

		gpiochip_add(&bank->chip);
	}
	return 0;
}

/*
 * This may get called early from board specific init
 * for boards that have interrupts routed via FPGA.
 */
int __init ls1c_gpio_init(void)
{
	if (!initialized)
		return _ls1c_gpio_init();
	else
		return 0;
}

 int __init ls1c_gpio_sysinit(void)
{
	int ret = 0;

	if (!initialized)
		ret = _ls1c_gpio_init();
	return ret;
}

arch_initcall(ls1c_gpio_sysinit);
