#ifndef __LS1B_IRQ_H
#define __LS1B_IRQ_H

#define LS1B_BOARD_ACPI_IRQ		0
#define LS1A_BOARD_HPET_IRQ		1

#if defined(CONFIG_LS1C_MACH)
#define LS1X_UART0_IRQ	2
#define LS1X_UART1_IRQ	4
#define LS1X_UART2_IRQ	5
#define LS1X_UART3_IRQ	29
#else 
#define LS1X_UART0_IRQ	2
#define LS1X_UART1_IRQ  3
#define LS1X_UART2_IRQ	4
#define LS1X_UART3_IRQ	5
#endif 


#define LS1X_BOARD_CAN0_IRQ	6
#define LS1X_BOARD_CAN1_IRQ	7
#define LS1B_BOARD_SPI0_IRQ	8
#define LS1B_BOARD_SPI1_IRQ	9
#define LS1B_BOARD_AC97_IRQ	10
#define LS1B_BOARD_MS_IRQ	11
#define LS1B_BOARD_KB_IRQ	12
#define LS1B_BOARD_DMA0_IRQ	13
#define LS1B_BOARD_DMA1_IRQ	14
#define LS1B_BOARD_DMA2_IRQ	15

#define SB2F_BOARD_DMA1_IRQ	14
#define SB2F_BOARD_DMA2_IRQ	15

#define LS1A_LPC_INT_IRQ	16

#define LS1B_BOARD_PWM0_IRQ	17
#define LS1B_BOARD_PWM1_IRQ	18
#define LS1B_BOARD_PWM2_IRQ	19
#define LS1B_BOARD_PWM3_IRQ	20

#define LS1X_RTC_INT0_IRQ	21
#define LS1X_RTC_INT1_IRQ	22
#define LS1X_RTC_INT2_IRQ	23
#define LS1X_TOY_INT0_IRQ	24
#define LS1X_TOY_INT1_IRQ	25
#define LS1X_TOY_INT2_IRQ	26
#define LS1X_RTC_TICK_IRQ	27
#define LS1X_TOY_TICK_IRQ	28

#ifdef	CONFIG_LS1B_MACH
#define LS1X_UART4_IRQ	29
#define LS1X_UART5_IRQ	30
#endif

#define LS1C_SDIO_IRQ   31

#define LS1X_EHCI_IRQ	32
#define LS1X_OHCI_IRQ	33

#if defined(CONFIG_LS1C_MACH)
#define LS1X_OTG_IRQ	34
#define LS1X_GMAC0_IRQ	35
#define LS1X_CAM_IRQ	36
#else
#define LS1X_GMAC0_IRQ	34
#define LS1X_GMAC1_IRQ	35
#define LS1A_BOARD_SATA_IRQ	36
#define LS1X_GPU_IRQ	37
#endif

#define LS1A_BOARD_PCI_INTA_IRQ LS1X_GPIO_FIRST_IRQ
#define LS1A_BOARD_PCI_INTB_IRQ (LS1X_GPIO_FIRST_IRQ + 1)
//#define LS1A_BOARD_PCI_INTA_IRQ 38
//#define LS1A_BOARD_PCI_INTB_IRQ 39
//#define LS1A_BOARD_PCI_INTC_IRQ 40
//#define LS1A_BOARD_PCI_INTD_IRQ 41

#define INT_PCI_INTA	(1<<0)
#define INT_PCI_INTB	(1<<1)
#define INT_PCI_INTC	(1<<2)
#define INT_PCI_INTD	(1<<3)
//#define INT_PCI_INTA	(1<<6)
//#define INT_PCI_INTB	(1<<7)
//#define INT_PCI_INTC	(1<<8)
//#define INT_PCI_INTD	(1<<9)

#define LS1X_GPIO_FIRST_IRQ 64

#if defined(CONFIG_LS1A_MACH)
#define LS1X_LAST_IRQ 159
#define INTN 5
#elif defined(CONFIG_LS1B_MACH)
#define LS1X_LAST_IRQ 127
#define INTN 4
#elif defined(CONFIG_LS1C_MACH)
#define LS1X_LAST_IRQ 159
#define INTN 5
#endif

/*
 * CPU core Interrupt Numbers
 */
#define MIPS_CPU_IRQ_BASE		(LS1X_LAST_IRQ + 1)
#define MIPS_CPU_IRQ(x)			(MIPS_CPU_IRQ_BASE + (x))

#define SOFTINT0_IRQ			MIPS_CPU_IRQ(0)
#define SOFTINT1_IRQ			MIPS_CPU_IRQ(1)
#define INT0_IRQ			MIPS_CPU_IRQ(2)
#define INT1_IRQ			MIPS_CPU_IRQ(3)
#define INT2_IRQ			MIPS_CPU_IRQ(4)
#define INT3_IRQ			MIPS_CPU_IRQ(5)
#define INT4_IRQ			MIPS_CPU_IRQ(6)
#define TIMER_IRQ			MIPS_CPU_IRQ(7)		/* cpu timer */

#define NR_IRQS 	  320

struct ls1x_intc_regs
{
	volatile unsigned int int_isr;
	volatile unsigned int int_en;
	volatile unsigned int int_set;
	volatile unsigned int int_clr;		/* offset 0x10*/
	volatile unsigned int int_pol;
   	volatile unsigned int int_edge;		/* offset 0 */
};

#endif	/* __LS1B_IRQ_H */
