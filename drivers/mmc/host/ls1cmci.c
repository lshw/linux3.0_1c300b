/*
 *  linux/drivers/mmc/ls1cmci.h - Samsung ls1c MCI driver
 *
 *  Copyright (C) 2004-2006 maintech GmbH, Thomas Kleffel <tk@maintech.de>
 *
 * Current driver maintained by Ben Dooks and Simtec Electronics
 *  Copyright (C) 2008 Simtec Electronics <ben-linux@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
/*
#include <mach/dma.h>

#include <mach/regs-sdi.h>
#include <mach/regs-gpio.h>

#include <plat/mci.h>
*/
#include "ls1cmci.h"

#define DRIVER_NAME "ls1c_sdio"

#define dbg-yg  0



enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

static const int dbgmap_err   = dbg_fail;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_err | dbg_debug;

#define dbg(host, channels, args...)		  \
	do {					  \
	if (dbgmap_err & channels) 		  \
		dev_err(&host->pdev->dev, args);  \
	else if (dbgmap_info & channels)	  \
		dev_info(&host->pdev->dev, args); \
	else if (dbgmap_debug & channels)	  \
		dev_dbg(&host->pdev->dev, args);  \
	} while (0)
#if 0
static struct ls1c_dma_client ls1cmci_dma_client = {
	.name		= "ls1c_sdio",
};
#endif
static void finalize_request(struct ls1cmci_host *host);
static void ls1cmci_send_request(struct mmc_host *mmc);
static void ls1cmci_reset(struct ls1cmci_host *host);

#ifdef CONFIG_MMC_DEBUG

static void dbg_dumpregs(struct ls1cmci_host *host, char *prefix)
{
	u32 con, pre, cmdarg, cmdcon, cmdsta, r0, r1, r2, r3, timer, bsize;
	u32 datcon, datcnt, datsta, fsta, imask;

	con 	= readl(host->base + ls1c_SDICON);
	pre 	= readl(host->base + ls1c_SDIPRE);
	cmdarg 	= readl(host->base + ls1c_SDICMDARG);
	cmdcon 	= readl(host->base + ls1c_SDICMDCON);
	cmdsta 	= readl(host->base + ls1c_SDICMDSTAT);
	r0 	= readl(host->base + ls1c_SDIRSP0);
	r1 	= readl(host->base + ls1c_SDIRSP1);
	r2 	= readl(host->base + ls1c_SDIRSP2);
	r3 	= readl(host->base + ls1c_SDIRSP3);
	timer 	= readl(host->base + ls1c_SDITIMER);
	bsize 	= readl(host->base + ls1c_SDIBSIZE);
	datcon 	= readl(host->base + ls1c_SDIDCON);
	datcnt 	= readl(host->base + ls1c_SDIDCNT);
	datsta 	= readl(host->base + ls1c_SDIDSTA);
	fsta 	= readl(host->base + ls1c_SDIFSTA);
	imask   = readl(host->base + host->sdiimsk);

	dbg(host, dbg_debug, "%s  CON:[%08x]  PRE:[%08x]  TMR:[%08x]\n",
				prefix, con, pre, timer);

	dbg(host, dbg_debug, "%s CCON:[%08x] CARG:[%08x] CSTA:[%08x]\n",
				prefix, cmdcon, cmdarg, cmdsta);

	dbg(host, dbg_debug, "%s DCON:[%08x] FSTA:[%08x]"
			       " DSTA:[%08x] DCNT:[%08x]\n",
				prefix, datcon, fsta, datsta, datcnt);

	dbg(host, dbg_debug, "%s   R0:[%08x]   R1:[%08x]"
			       "   R2:[%08x]   R3:[%08x]\n",
				prefix, r0, r1, r2, r3);
}

static void prepare_dbgmsg(struct ls1cmci_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct ls1cmci_host *host, struct mmc_command *cmd,
			int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s DCNT:0x%08x\n",
			cmd->data->error, host->dbgmsg_dat,
			readl(host->base + ls1c_SDIDCNT));
	}
}
#else
static void dbg_dumpcmd(struct ls1cmci_host *host,
			struct mmc_command *cmd, int fail) { }

static void prepare_dbgmsg(struct ls1cmci_host *host, struct mmc_command *cmd,
			   int stop) { }

static void dbg_dumpregs(struct ls1cmci_host *host, char *prefix) { }

#endif /* CONFIG_MMC_DEBUG */


/**********************************************************************************************************/
/**********************************************************************************************************/
/*********************************  ls1c-sdio-start ***
 * *************************************************************************/
#define SDIO_DEV_ADDR   0x1fe6c040


/**
 * ls1cmci_enable_irq - enable IRQ, after having disabled it.
 * @host: The device state.
 * @more: True if more IRQs are expected from transfer.
 *
 * Enable the main IRQ if needed after it has been disabled.
 *
 * The IRQ can be one of the following states:
 *	- disabled during IDLE
 *	- disabled whilst processing data
 *	- enabled during transfer
 *	- enabled whilst awaiting SDIO interrupt detection
 */
static void ls1cmci_enable_irq(struct ls1cmci_host *host, bool more)
{
	unsigned long flags;
	bool enable = false;

	local_irq_save(flags);

	host->irq_enabled = more;
	host->irq_disabled = false;

	enable = more | host->sdio_irqen;

	if (host->irq_state != enable) {
		host->irq_state = enable;

		if (enable)
			enable_irq(host->irq);
		else
			disable_irq(host->irq);
	}

	local_irq_restore(flags);
}


static inline bool ls1cmci_host_usedma(struct ls1cmci_host *host)
{
	return true;
/*	
#ifdef CONFIG_MMC_LS1C_PIO
	return false;
#elif defined(CONFIG_MMC_LS1C_DMA)
	return true;
#else
	return host->dodma;
#endif
*/
}


static void ls1cmci_check_sdio_irq(struct ls1cmci_host *host)
{
	if (host->sdio_irqen) {
		if (gpio_get_value(84) == 0) {
			printk(KERN_DEBUG "%s: signalling irq\n", __func__);
			mmc_signal_sdio_irq(host->mmc);
		}
	}
}

/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : zhanghualiang
 * Language: C
 * Date    : 2013-07-01
 ***************************************************************************/

static void ls1cmci_disable_irq(struct ls1cmci_host *host, bool transfer)
{
	unsigned long flags;

	local_irq_save(flags);

	//printk(KERN_DEBUG "%s: transfer %d\n", __func__, transfer);
	host->irq_disabled = transfer;

	if (transfer && host->irq_state) {
		host->irq_state = false;
		disable_irq(host->irq);
	}
	local_irq_restore(flags);
}


/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static void ls1cmci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ls1cmci_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 con;
#if 1
	local_irq_save(flags);

	con = readl(host->base + SDICON);
	host->sdio_irqen = enable;

	if (enable == host->sdio_irqen)
		goto same_state;

	if (enable) {
		//con |= INT0_EN_SDIOIRQ;
		//enable_imask(host, SDIIMSK_SDIOIRQ);
		*(volatile unsigned int *)(0xbfd01044) |= 0x1 << 31;  //int0_en bit[31]  //sdio
		writel(readl(0xbfd01044)|(0x1 << 31),0xbfd01044);	

		if (!host->irq_state && !host->irq_disabled) {
			host->irq_state = true;
			enable_irq(host->irq);
		}
	} else {

		writel(readl(0xbfd01044)&(~(0x1 << 31)),0xbfd01044);	
//		disable_imask(host, ls1c_SDIIMSK_SDIOIRQ);
//		con &= ~ls1c_SDICON_SDIOIRQ;

		if (!host->irq_enabled && host->irq_state) {
			disable_irq_nosync(host->irq);
			host->irq_state = false;
		}
	}
//	writel(con, host->base + ls1c_SDICON);

 same_state:
	local_irq_restore(flags);

	ls1cmci_check_sdio_irq(host);
#endif
}




static inline void clear_imask(struct ls1cmci_host *host)
{
	u32 mask = readl(host->base + host->sdiimsk);

	/* preserve the SDIO IRQ mask state */
//	mask &= SDIIMSK_SDIOIRQ;
	writel(mask, host->base + host->sdiimsk);
}



/***************************************************************************
 * Description:  call by pio_tasklet()
 *      
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-11-21
 ***************************************************************************/
static void finalize_request(struct ls1cmci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	int debug_as_failure = 0;
//	printk(" dbg-yg ======================> finalize_request().............\r\n");
	if (host->complete_what != COMPLETION_FINALIZE)
		return;

//	printk(" dbg-yg ======================> 1 .............\r\n");
	if (!mrq)
		return;
	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	if (cmd->data && (cmd->error == 0) &&
	    (cmd->data->error == 0)) {
		if (ls1cmci_host_usedma(host) && (!host->dma_complete)) {
			dbg(host, dbg_dma, "DMA Missing (%d)!\n",
			    host->dma_complete);
			return;
		}
	}

//	printk(" dbg-yg ======================> 2 .............\r\n");
	/* Read response from controller. */
	cmd->resp[0] = readl(host->base + SDIRSP0);
	cmd->resp[1] = readl(host->base + SDIRSP1);
	cmd->resp[2] = readl(host->base + SDIRSP2);
	cmd->resp[3] = readl(host->base + SDIRSP3);

//	writel(host->prescaler, host->base + S3C2410_SDIPRE);

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

//	dbg_dumpcmd(host, cmd, debug_as_failure);

	/* Cleanup controller */
	writel(0, host->base + SDICMDARG);
//	writel(SDIDCON_STOP, host->base + SDIDCON);
	writel(0, host->base + SDICMDCON);
	clear_imask(host);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		ls1cmci_send_request(host->mmc);
		return;
	}

//	printk(" dbg-yg ======================> 3 .............\r\n");
	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

//	printk(" dbg-yg ======================> 4 .............\r\n");
	/* Calculate the amout of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
//		printk(" dbg-yg =============> mrq->data->bytes_xfered = %d ...\r\n", mrq->data->bytes_xfered);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	/* If we had an error while transferring data we flush the
	 * DMA channel and the fifo to clear out any garbage. */
	if (mrq->data->error != 0) {
		if (ls1cmci_host_usedma(host))
		//	s3c2410_dma_ctrl(host->dma, S3C2410_DMAOP_FLUSH);
			printk(" dbg-yg =============> ls1cmci dma data error!\r\n");
#if 0
		if (host->is2440) {
			/* Clear failure register and reset fifo. */
			writel(S3C2440_SDIFSTA_FIFORESET |
			       S3C2440_SDIFSTA_FIFOFAIL,
			       host->base + S3C2410_SDIFSTA);
		} else {
			u32 mci_con;

			/* reset fifo */
			mci_con = readl(host->base + S3C2410_SDICON);
			mci_con |= S3C2410_SDICON_FIFORESET;

			writel(mci_con, host->base + S3C2410_SDICON);
		}
#endif
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;

//	ls1cmci_check_sdio_irq(host);
	mmc_request_done(host->mmc, mrq);
//	printk(" finalize request ==================================================end !\r\n");
}   



/***************************************************************************
 * Description:  sdio irq tasklet
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-11-21
 ***************************************************************************/


static void pio_tasklet(unsigned long data)
{
	struct ls1cmci_host *host = (struct ls1cmci_host *) data;

	ls1cmci_disable_irq(host, true);
	//printk(" dbg-yg .....enter pio_tasklet()........\r\n");

/*
	if (host->pio_active == XFER_WRITE)
		do_pio_write(host);

	if (host->pio_active == XFER_READ)
		do_pio_read(host);
*/
	if (host->complete_what == COMPLETION_FINALIZE) {
	//	printk(" host->complete_whta == COMPLETION_FINALIZE\r\n");
		clear_imask(host);
		/*
		if (host->pio_active != XFER_NONE) {
			dbg(host, dbg_err, "unfinished %s "
			    "- pio_count:[%u] pio_bytes:[%u]\n",
			    (host->pio_active == XFER_READ) ? "read" : "write",
			    host->pio_count, host->pio_bytes);

			if (host->mrq->data)
				host->mrq->data->error = -EINVAL;
		}
		*/
		ls1cmci_enable_irq(host, false);
		finalize_request(host);
	} else
		ls1cmci_enable_irq(host, true);
	//printk(" ...................... finish pio_tasklet().................\r\n");
}


/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung_yg
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/
 static irqreturn_t ls1cmci_irq(int irq, void *dev_id)
{
	struct ls1cmci_host *host = dev_id;
	struct mmc_command *cmd;
	u32 mci_csta, mci_dsta, mci_fsta, mci_dcnt, mci_imsk;
	u32 mci_cclear = 0, mci_dclear;
	unsigned long iflags;
	
//	spin_lock_irqsave(&host->complete_lock, iflags);
//	printk("\r\n\r\nenter ===============================> ls1cmci_irq()\r\n");

	mci_csta = readl(host->base + SDICMDSTA);
	mci_imsk = readl(host->base + SDIINTMSK);
	mci_dsta = readl(host->base + SDIDSTA);
//	printk(" =========> dbg-yg ls1cmci_irq: sdio csta:0x%08x  dsta:0x%08x  imsk:0x%08x \r\n", mci_csta, mci_dsta, mci_imsk );


	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		clear_imask(host);
		goto irq_out;
	}
//	printk("  ls1cmci_irq  1 \r\n");
	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

//	printk("  ls1cmci_irq  2 \r\n");
	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;
/*	
	//若有数据操作，需要等待 数据搬运完成
	while((mci_dsta = readl(host->base + SDIDSTA)) & 0x03);
	mci_imsk = readl(host->base + SDIINTMSK);
*/
	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

//	printk("  ls1cmci_irq  3 \r\n");
	//if (mci_csta & SDICMDSTAT_CMDTIMEOUT) {
	if (mci_imsk & SDIIMSK_CMDTIMEOUT) {
		dbg(host, dbg_err, "CMDSTAT: error CMDTIMEOUT\n");
		cmd->error = -ETIMEDOUT;
		host->status = "error: command timeout";
		goto fail_transfer;
	}

//	printk("  ls1cmci_irq  4 \r\n");
//	if ((mci_csta & SDICMDSTAT_CMDSENT) {
	if((mci_imsk & SDIIMSK_CMDSENT)){
//		printk("  ls1cmci_irq  4-1 \r\n");
		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		}
//		mci_cclear |= S3C2410_SDICMDSTAT_CMDSENT;
	}

//	printk("  ls1cmci_irq  5 \r\n");
//	if (mci_csta & SDICMDSTAT_CRCFAIL) {
	if (mci_imsk & SDIIMSK_RESPONSECRC){
		if (cmd->flags & MMC_RSP_CRC) {
			if (host->mrq->cmd->flags & MMC_RSP_136) {
				dbg(host, dbg_irq,
				    "fixup: ignore CRC fail with long rsp\n");
			} else {
				/* note, we used to fail the transfer
				 * here, but it seems that this is just
				 * the hardware getting it wrong.
				 *
				 * cmd->error = -EILSEQ;
				 * host->status = "error: bad command crc";
				 * goto fail_transfer;
				*/
			}
		}
	//	mci_cclear |= S3C2410_SDICMDSTAT_CRCFAIL;
	}

//	if ((mci_csta & SDICMDSTAT_RSPFIN)) {
	if ((mci_imsk & SDIIMSK_CMDSENT)) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			host->status = "ok: command response received";
//			printk(" mci_csta: 0x%08x ... status: %s ...\r\n", mci_csta, host->status);
			goto close_transfer;
		}
//		printk("  ls1cmci_irq  6 \r\n");
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
	//	mci_cclear |= S3C2410_SDICMDSTAT_RSPFIN;
	}
	
	if (!cmd->data)
		goto clear_status_bits;

//	printk("  ls1cmci_irq  7 \r\n");
//	if (mci_dsta & SDIDSTA_RXCRCFAIL) {
	if (mci_imsk & SDIIMSK_RXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (outgoing)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (outgoing)";
		goto fail_transfer;
	}

//	printk("  ls1cmci_irq  8 \r\n");
	//if (mci_dsta & SDIDSTA_CRCFAIL) {
	if (mci_imsk & SDIIMSK_TXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (incoming)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (incoming)";
		goto fail_transfer;
	}

//	printk("  ls1cmci_irq  9 \r\n");
	//if (mci_dsta & SDIDSTA_DATATIMEOUT) {
	if (mci_imsk & SDIIMSK_DATATIMEOUT) {
		dbg(host, dbg_err, "data timeout\n");
		cmd->data->error = -ETIMEDOUT;
		host->status = "error: data timeout";
		goto fail_transfer;
	}

//	printk("  ls1cmci_irq  10 \r\n");

	//if ((mci_dsta & SDIDSTA_XFERFINISH) || (mci_imsk & SDIIMSK_DATAFINISH )){
	if ((mci_imsk & SDIIMSK_DATAFINISH )){
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			host->dma_complete = 1;
		//	host->complete_what = COMPLETION_FINALIZE;
			goto close_transfer;
		}

//		printk("  ls1cmci_irq  11 \r\n");
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
//		mci_dclear |= S3C2410_SDIDSTA_XFERFINISH;
	}
	//mci_dcnt = readl(host->base + SDIDCNT);

//	printk(" resp[0]:0x%08x   resp[1]:0x%08x  resp[2]:0x%08x  resp[3]:0x%08x ...\r\n", cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

//	printk("  no int????????????????????????\r\n");

#if 0	
	if (mci_csta == 0x1ff) {
//		printk(" 2 \r\n");
		 cmd->error = EPFNOSUPPORT;
		 host->status = "error: not support";
	mmc_request_done(host->mmc, host->mrq);
			return IRQ_HANDLED;
	}
	if (mci_imsk & (1 << 7)) {   //cmd time out

		printk(" \r\n cmd time out \r\n\r\n");
		dbg(host, dbg_err, "CMDSTAT: error CMDTIMEOUT\n");
		cmd->error = ETIMEDOUT;
		host->status = "error: command timeout";
		mmc_request_done(host->mmc, host->mrq);
	//	printk(" \r\n return \r\n\r\n");
			return IRQ_HANDLED;
	}
#endif
#if 0
	mci_csta = readl(host->base + SDICMDSTA);
	mci_imsk = readl(host->base + SDIINTMSK);
	mci_dsta = readl(host->base + SDIDSTA);
#endif
//	printk(" end =========> dbg-yg ls1cmci_irq: sdio csta:0x%08x  dsta:0x%08x  imsk:0x%08x \r\n", mci_csta, mci_dsta, mci_imsk );
	

clear_status_bits:
//	writel(mci_cclear, host->base + S3C2410_SDICMDSTAT);
//	writel(mci_dclear, host->base + S3C2410_SDIDSTA);

	goto irq_out;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;

	writel(mci_imsk, host->base + SDIINTMSK);  //write "1" clear  imsk ,
	clear_imask(host);
	tasklet_schedule(&host->pio_tasklet);
//	printk(" host->status : %s ...\r\n", host->status);
	goto irq_out;

irq_out:
//	dbg(host, dbg_irq,	    "csta:0x%08x dsta:0x%08x fsta:0x%08x dcnt:0x%08x status:%s.\n", mci_csta, mci_dsta, mci_fsta, mci_dcnt, host->status);
//	printk(" irq_out....\r\n");	
//	spin_unlock_irqrestore(&host->complete_lock, iflags);

	writel(mci_imsk, host->base + SDIINTMSK);  //write "1" clear  imsk ,
//	printk(" after clear imsk: 0x%08x ...\r\n", readl(host->base + SDIINTMSK));	
//	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return IRQ_HANDLED;

}





/*******************************************************/

static void ls1cmci_set_clk(struct ls1cmci_host *host, struct mmc_ios *ios)
{
	u32 mci_psc;
//	printk(" in  ls1cmci_set_clk()...\r\n");
	/* Set clock */
	for (mci_psc = 0; mci_psc < 255; mci_psc++) {
		host->real_rate = host->clk_rate / ((mci_psc+1));

		if (host->real_rate <= ios->clock)
			break;
	}
	printk(" in ls1cmci_set_clk() ...host->clk_rate:%d ...host->real_rate:%d\r\n",host->clk_rate,host->real_rate);
	if (mci_psc > 255)
		mci_psc = 255;

	host->prescaler = mci_psc;
	writel(host->prescaler, host->base + SDIPRE);
//	printk("host->prescaler is :%d \r\n",host->prescaler);
	/* If requested clock is 0, real_rate will be 0, too */
	if (ios->clock == 0)
		host->real_rate = 0;
//	printk(" ios->clock:%d  \r\n",ios->clock);
}



static void ls1cmci_reset(struct ls1cmci_host *host)
{
	u32 con = readl(host->base + SDICON);

	con |= SDICON_SDRESET;
	writel(con, host->base + SDICON);
	printk(" ls1cmci host reset()");
}
/************************************************************************/

/************************************************/
/**
 *ls1c_dma_enqueue - set dma desc chain
 *
 *
 */
static void ls1c_dma_enqueue(int dma_num, int dma_len, struct ls1cmci_host *host, dma_addr_t addr, unsigned int size)
{
#if 0	
	struct ls1c_dma_desc * dma_desc =  (struct ls1c_dma_desc *) (host->dma_send_phy_mem + (dma_num*0x40));
	
		dma_desc->order_addr =  (unsigned int ) (dma_desc + 0x40);
		if ( dma_num == (dma_len - 1))
			dma_desc->order_addr &= ~(1<<0);
		else 
			dma_desc->order_addr |= 1;  //next desc in effect
		dma_desc->saddr = addr;
		dma_desc->daddr  = SDIO_DEV_ADDR;
		dma_desc->length = size/4;
		dma_desc->step_length  = 0;   //??? question  ???
		dma_desc->step_times = 1;   //???
		dma_desc->cmd = 0x1000;  //  write deva
#endif
#if 0
	struct ls1c_dma_desc * first_desc =  (struct ls1c_dma_desc *) host->dma_send_phy_mem;
	unsigned int offset = dma_num * 0x40;

		first_desc->order_addr + offset = first_desc + (offset + 0x40); //0x40 means low 6bit align
		if ( dma_num == (dma_len - 1))
			first_desc->order_addr + offset &= ~(1<<0);
		else 
			first_desc->order_addr + offset |= 1;  //next desc in effect
		first_desc->saddr + offset = addr;
		first_desc->daddr + offset = SDIO_DEV_ADDR;
		first_desc->length + offset = size/4;
		first_desc->step_length + offset = 0;   //??? question  ???
		first_desc->step_times + offset = 1;   //???
		first_desc->dma_cmd + offset = 0x1000;  //  write dev
#endif 
}


static void ls1cmci_send_command(struct ls1cmci_host *host,
					struct mmc_command *cmd)
{
	u32 ccon, imsk;
/*
	imsk  = ls1c_SDIIMSK_CRCSTATUS | ls1c_SDIIMSK_CMDTIMEOUT |
		ls1c_SDIIMSK_RESPONSEND | ls1c_SDIIMSK_CMDSENT |
		ls1c_SDIIMSK_RESPONSECRC;
	enable_imask(host, imsk);
*/
//	printk(" enter ls1cmci_send_commend()...\r\n cmd_index:%d\r\n",cmd->opcode & SDICMDCON_INDEX);

//	printk(" ls1c send cmd:%d .................\r\n", cmd->opcode);
	if (cmd->data){
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	}
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	writel(cmd->arg, host->base + SDICMDARG);
//	printk("write  regs cmd->arg(08): 0x%08x...\r\n",cmd->arg);
	ccon  = cmd->opcode & SDICMDCON_INDEX;
	ccon |= SDICMDCON_SENDERHOST | SDICMDCON_CMDSTART;

	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= SDICMDCON_WAITRSP;

	if (cmd->flags & MMC_RSP_136)
		ccon |= SDICMDCON_LONGRSP;
//	printk("dbg-yg  ls1cmci_send_command write  regs cmd_con(0c): 0x%08x...\r\n",ccon);
	writel(ccon, host->base + SDICMDCON);
//	printk("read  regs cmd_con(0c): 0x%08x...\r\n",readl(host->base+SDICMDCON ));
}
/*******************************************************************/
#if 0
static void ls1cmci_dma_setup(struct ls1cmci_host *host,
			     enum ls1c_dmasrc source)
{
	static enum ls1c_dmasrc last_source = -1;
	static int setup_ok;

	if (last_source == source)
		return;

	last_source = source;

	s3c2410_dma_devconfig(host->dma, source,
			      host->mem->start + host->sdidata);

	if (!setup_ok) {
		ls1c_dma_config(host->dma, 4);
		ls1c_dma_set_buffdone_fn(host->dma,
					    ls1cmci_dma_done_callback);
		ls1c_dma_setflags(host->dma, ls1c_DMAF_AUTOSTART);
		setup_ok = 1;
	}
}
#endif

static int ls1cmci_prepare_dma(struct ls1cmci_host *host, struct mmc_data *data)
{
	int dma_len, i, j;
	int rw = data->flags & MMC_DATA_WRITE;
	unsigned int  dcon;
	unsigned char *ptr;

//	printk(" enter ls1cmci_prepare_dma().......\r\n");    //dbg-yg
//	writel(data->blksz, host->base + SDIBSIZE);    //write in  setup_data();
	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     rw ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
//	printk(" dma_len:%d   blksz:0x%08x   blocks:%d  \r\n",dma_len, data->blksz, data->blocks);    //dbg-yg
	if (dma_len == 0)
		return -ENOMEM;

	host->dma_complete = 0;
	host->dmatogo = dma_len;
	for (i = 0; i < dma_len; i++) {
		host->sg_cpu[i].length = sg_dma_len(&data->sg[i])  / 4;  //unit is  word
//		printk(" dbg-yg  sg_dma_len :0x%08x ...\r\n",sg_dma_len(&data->sg[i]));
		host->sg_cpu[i].step_length = 0;
		host->sg_cpu[i].step_times = data->blocks;
		host->sg_cpu[i].saddr = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].daddr = SDIO_DEV_ADDR;
//		printk(" dbg-yg  =================> sg_cpu[%d].saddr:0x%08x ...\r\n", i, host->sg_cpu[i].saddr);
		if (data->flags & MMC_DATA_READ) {						  //  dma read
		    
			host->sg_cpu[i].cmd = 0x1<<0;   //bit12:0(default) & disable dma int 
	//		printk(">>>>>>>>>>>read sd to sg_cpu[%d].saddr:0x%08x\r\n", i, host->sg_cpu[i].saddr);
		#if  dbg-yg // dbg-yg  debug test sd card  read 	
			ptr = (unsigned char *) (host->sg_cpu[i].saddr|0xa0000000);
			for( j=0; j<20; j++)
			{
				if (j % 16 == 0)  printk("\r\n");
				printk(" %3x   ",(ptr[j]));
			}
			printk("\r\n");
		#endif	
		} else {												  //  dma write
			host->sg_cpu[i].cmd = ((0x1<<12) | (0x1<<0));   //ddr > dev  & disable dma int 
#if dbg-yg
			/*  watch the write data*/
			//printk(">>>>>>>>>>>>write sd from sg_cpu[%d].saddr:0x%08x\r\n", i, host->sg_cpu[i].saddr);
			ptr = (unsigned char *) (host->sg_cpu[i].saddr|0xa0000000);
			for( j=0; j<20; j++)
			{
				if (j%16 == 0)  printk("\r\n");
				printk(" %3x   ",(ptr[j]));
			}
			printk("\r\n");
#endif 
		}
		

		host->sg_cpu[i].order_addr = host->sg_dma+(i+1)*sizeof(struct ls1c_dma_desc);
		host->sg_cpu[i].order_addr |= 0x1<<0;   //next desc in effect  
	}
		host->sg_cpu[dma_len-1].order_addr &= ~(0x1<<0);  // last order no effect 

		*(volatile unsigned int *) (0xbfd01160) = (host->sg_dma | 0x8 | 0x1);    //dma start  use dma1
//	printk("host->sg_cpu[dma_len-1].order_addr:0x%08x ...\r\n",host->sg_cpu[dma_len-1].order_addr);

#if 0
		dbg(host, dbg_dma, "enqueue %i: %08x@%u\n", i,
		    sg_dma_address(&data->sg[i]),
		    sg_dma_len(&data->sg[i]));
	    ls1c_dma_enqueue(i, dma_len, host,sg_dma_address(&data->sg[i]),
							   sg_dma_len(&data->sg[i]));
	}
#endif
/*		if (res) {
			ls1c_dma_ctrl(host->dma, ls1c_DMAOP_FLUSH);
			return -EBUSY;
		}
*/

	return 0;
}



static int ls1cmci_setup_data(struct ls1cmci_host *host, struct mmc_data *data)
{
	u32 dcon, imsk, stoptries = 3, tmp;

	/* write DCON register */

	if (!data) {
		writel(0, host->base + SDIDCON);
		return 0;
	}

	if ((data->blksz & 3) != 0) {
		/* We cannot deal with unaligned blocks with more than
		 * one block being transferred. */

		if (data->blocks > 1) {
			pr_warning("%s: can't do non-word sized block transfers (blksz %d)\n", __func__, data->blksz);
			return -EINVAL;
		}
	}
//	printk(" dma setup data ,sdi data stat: 0x%08x  ...\r\n" , readl(host->base + SDIDSTA));
#if 0
	while (readl(host->base + SDIDSTA) & (SDIDSTA_TXDATAON | SDIDSTA_RXDATAON)) 
	{
		dbg(host, dbg_err,"mci_setup_data() transfer stillin progress.\n");
	//	writel(SDIDCON_STOP, host->base + SDIDCON);
		ls1cmci_reset(host);
		if ((stoptries--) == 0) {
			dbg_dumpregs(host, "DRF");
			return -EINVAL;
		}
	}
#endif
	dcon  = data->blocks & SDIDCON_BLKNUM_MASK;
	if (ls1cmci_host_usedma(host)){
		tmp = readl(MISC_CTRL);
		tmp &= ~(0x3 << 23);
		writel(tmp | (2<<23), MISC_CTRL);  // use dma1
	}
	if (host->bus_width == MMC_BUS_WIDTH_4){
		printk(" sdio  bus width : 4 \r\n");
		dcon |= SDIDCON_WIDEBUS;    //bit16
	}
/*
	if (!(data->flags & MMC_DATA_STREAM))
		dcon |= SDIDCON_BLOCKMODE;
*/
/*	if (data->flags & MMC_DATA_WRITE) {
		dcon |= SDIDCON_TXAFTERRESP;
		dcon |= SDIDCON_XFER_TXSTART;
	}

	if (data->flags & MMC_DATA_READ) {
		dcon |= SDIDCON_RXAFTERCMD;
		dcon |= SDIDCON_XFER_RXSTART;
	}

	if (host->is) {
		dcon |= SDIDCON_DS_WORD;
		dcon |= SDIDCON_DATSTART;
	}
*/
	dcon |= 3 << 14;  //bit15:dma_en bit14:  start
	//printk(" sdi dat con:0x%08x ...\r\n",dcon);
	writel(dcon, host->base + SDIDCON);
	writel(data->blksz, host->base + SDIBSIZE);  //block  size 
	//printk("block size:%d \r\n", data->blksz);
	/* write TIMER register */
	writel(0x007FFFFF, host->base + SDITIMER);

		/* FIX: set slow clock to prevent timeouts on read */
//	if (data->flags & MMC_DATA_READ)  //???
	//	writel(0xFF, host->base + SDIPRE);

	return 0;
}


/*********************************************************************************/
static void ls1cmci_send_request(struct mmc_host *mmc)
{
	int res;
	struct ls1cmci_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;
	host->ccnt++;
//	printk(" send request:  cmd_index:%d ...\r\n", cmd->opcode);
	if (cmd->data) {
		host->dcnt++;

			res = ls1cmci_setup_data(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "setup data error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
			res = ls1cmci_prepare_dma(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "data prepare error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
	}

	/* Send command */
	ls1cmci_send_command(host, cmd);
//		ls1c_dma_ctrl(host->dma, DMAOP_START);
	/* Enable Interrupt */
	ls1cmci_enable_irq(host, true);
}


/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static void ls1cmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ls1cmci_host *host = mmc_priv(mmc);
	u32 mci_con, val;
	/* Set the power state */
#if 1
	mci_con = readl(host->base + SDICON);

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
/* config mux: sdio mux with spi0,*/
		val = readl(MISC_CTRL);
		val &= ~(0x3 << 16);
		val |= (1<<16);
		*(volatile unsigned int *)(MISC_CTRL) = val;

		*(volatile unsigned int *)(SDIO_BASE + SDICON) = 0x100; // reset sdio ctl reg
		mdelay(100);
		*(volatile unsigned int *)(SDIO_BASE + SDICON) = 0x01; // enable clk
		*(volatile unsigned int *)(SDIO_BASE + SDIINTEN) = 0x1ff; // enable  int *************
	//	*(volatile unsigned int *)(SDIO_BASE + SDIPRE) = 0x8000000f; // config pre_scale???
	//  *(volatile unsigned int *)(SDIO_BASE + SDIPRE) = 2; // config pre_scale
	
	//**********************************************************

	//	if (host->pdata->set_power)
	//		host->pdata->set_power(ios->power_mode, ios->vdd);

		break;

	case MMC_POWER_OFF:
	default:
			mci_con |= SDICON_SDRESET;
//		if (host->pdata->set_power)
//			host->pdata->set_power(ios->power_mode, ios->vdd);
		break;
	}
	ls1cmci_set_clk(host, ios);
	/* Set CLOCK_ENABLE */
	if (ios->clock)
		mci_con |= SDICON_CLOCKTYPE;   //clk enable
//	else
//		mci_con &= ~SDICON_CLOCKTYPE;
	writel(mci_con, host->base + SDICON);
	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_conf, "running at %lukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_conf, "powered down.\n");
	}
//	printk(" bus_width:%d \r\n",ios->bus_width);
	host->bus_width = ios->bus_width;
#endif	
}
/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static int ls1cmci_card_present(struct mmc_host *mmc)
{
	struct ls1cmci_host *host = mmc_priv(mmc);
	struct ls1c_mci_pdata *pdata = host->pdata;
	int ret = 0;

		return 1; // beijing banzi  dont have  detect io
#if 1
	if (pdata->no_detect)
		return -ENOSYS;
	ret = gpio_get_value(pdata->gpio_detect) ? 0 : 1;
#endif
	return ret ^ pdata->detect_invert;
}

/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/
 static void ls1cmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ls1cmci_host *host = mmc_priv(mmc);
//	printk(" enter ls1cmci_request()...\r\n");
	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;

	if (ls1cmci_card_present(mmc) == 0) {
		dbg(host, dbg_err, "%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
	} else
		ls1cmci_send_request(mmc);

}

/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static int ls1cmci_get_ro(struct mmc_host *mmc)
{
	struct ls1cmci_host *host = mmc_priv(mmc);
	struct ls1c_mci_pdata *pdata = host->pdata;
	int ret=0;
	//printk(" enter ls1cmci_get_ro()...\r\n");
#if 0
	if (pdata->no_wprotect)
		return 0;

	ret = gpio_get_value(pdata->gpio_wprotect) ? 1 : 0;
	ret ^= pdata->wprotect_invert;
#endif
	return ret;
}

/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung_yg
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/
 static struct mmc_host_ops ls1cmci_ops = {
	.request	= ls1cmci_request,
	.set_ios	= ls1cmci_set_ios,
	.get_ro		= ls1cmci_get_ro,
	.get_cd		= ls1cmci_card_present,
	.enable_sdio_irq = ls1cmci_enable_sdio_irq,
};
/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/
static irqreturn_t ls1cmci_irq_cd(int irq, void *dev_id)
{
	struct ls1cmci_host *host = (struct ls1cmci_host *)dev_id;
	int ret;
	dbg(host, dbg_irq, "card detect\n");
	ret = gpio_get_value(84) ? 0 : 1;
	if (ret)
		printk(" sd card is detect\r\n");
	else
		printk("sd card is  removed\r\n");
	mmc_detect_change(host->mmc, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}
 static irqreturn_t mci_detect_irq(int irq, void *dev_id)
{
	printk(" int mci_detect_irq() \r\n");
	return IRQ_HANDLED;
}

/***************************************************************************
 * Description:
 *  
 * Version : 1.00
 * Author  : Sunyoung_yg 
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/

static struct ls1c_mci_pdata ls1cmci_def_pdata = {
	/* This is currently here to avoid a number of if (host->pdata)
	 * checks. Any zero fields to ensure reasonable defaults are picked. */
	 .no_wprotect = 1,
	 .no_detect = 0,
	 .gpio_detect = 84,
};

/***************************************************************************
 * Description: probe()
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/

/* #################### probe #######################*/
static int __devinit ls1cmci_probe(struct platform_device *pdev)
{
	struct ls1cmci_host *host;
	struct mmc_host	*mmc;
	int ret;
	int i;
	unsigned int tmp;
	mmc = mmc_alloc_host(sizeof(struct ls1cmci_host), &pdev->dev);//* alloc a    mmc_host + ls1cmci_host  */ 
	if (!mmc) {
		ret = -ENOMEM;
//		goto probe_out;
	}

	//config sdio use dma1, & spi0 pin.
	tmp = *(volatile unsigned int *)(MISC_CTRL);
	tmp &= ~(0x3<<23);
	tmp &= ~(0x3<<16);
	tmp |= (0x2<<23) | (0x1<<16);
	*(volatile unsigned int *)(MISC_CTRL) = tmp;

	tmp = *(volatile unsigned int *)(MISC_CTRL);
	printk(" --------------------dbg-yg  misc_ctrl:0x%08x ....\r\n", tmp);
//	*(volatile unsigned int *)(SDIO_BASE + SDICON) = 0x100; // reset sdio ctl reg
	mdelay(1);

//**********************************************************
	
//	printk(" config spi0 gpio mux to sdio\r\n");
	/* config gpio mux */
	*(volatile unsigned int *)(0xbfd011c8) |= (0x1f << (79-64));
	*(volatile unsigned int *)(0xbfd011d8) &= ~(0x3f << (79-64));
	*(volatile unsigned int *)(0xbfd011e8) &= ~(0x3f << (79-64));
	*(volatile unsigned int *)(0xbfd011f8) &= ~(0x3f << (79-64));
	
	/* config gpio84 in INT mode */
	*(volatile unsigned int *)(0xbfd011c8) &= ~(0x1 << (84-64));

	*(volatile unsigned int *)(0xbfd010c8) |= (0x1 << (84-64)); //cfg enable
	*(volatile unsigned int *)(0xbfd010d8) |= (0x1 << (84-64)); //input enable
	*(volatile unsigned int *)(0xbfd010f8) |= (0x1 << (84-64)); //  output 1

	*(volatile unsigned int *)(0xbfd010a4) |= (0x1 << (84-64)); //  int enable
	*(volatile unsigned int *)(0xbfd010b4) |= (0x1 << (84-64));   // edge int
	
	/* sdio  int */	
//	*(volatile unsigned int *)(0xbfd01054) |= (0x1 << (31));   // sdio edge triger
	*(volatile unsigned int *)(0xbfd01050) |= (0x1 << (31));   // sdio high level triger | edge = raise edge triger

//***********************************************************
	*(volatile unsigned int *)(SDIO_BASE + SDIINTEN) = 0x1ff; // enable all sdio int *************



	host = mmc_priv(mmc);
	host->mmc 	= mmc;
	host->pdev	= pdev;
	//host->is	= is;

	host->pdata = pdev->dev.platform_data;
	if (!host->pdata) {
		pdev->dev.platform_data = &ls1cmci_def_pdata;
		host->pdata = &ls1cmci_def_pdata;
	}
	
	spin_lock_init(&host->complete_lock);
	tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);
	host->sdiimsk	= 0x3c;
	host->sdidata	= 0x40;//ls1c sd card 只能dma 传输 
	host->clk_div	= 0x10; //0x7;

	host->complete_what 	= COMPLETION_NONE;
	host->pio_active 	= XFER_NONE;


    host->base = 0xbfe6c000;
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		dev_err(&pdev->dev, "failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	if (request_irq(host->irq, ls1cmci_irq, 0, DRIVER_NAME, host)) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	disable_irq(host->irq);
//	printk("  host->irq is :%d ... \r\n",host->irq);
	host->irq_state = false;

   // gpio interupt 
	if (!host->pdata->no_detect) {
		host->irq_cd = gpio_to_irq(host->pdata->gpio_detect);
		host->irq_cd = host->pdata->gpio_detect + LS1X_GPIO_FIRST_IRQ;  //64 is  gpio irq base
//		printk(" host->irq_cd (gpio_detect irq is:%d\r\n",host->irq_cd);
		if (host->irq_cd >= 0) {
			if (request_irq(host->irq_cd, ls1cmci_irq_cd,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					DRIVER_NAME, host)) {
				dev_err(&pdev->dev,
					"can't get card detect irq.\n");
				ret = -ENOENT;
				goto probe_free_gpio_cd;
			}
		} else {
			dev_warn(&pdev->dev,
				 "host detect has no irq available\n");
			//gpio_direction_input(host->pdata->gpio_detect);
		}
	} else
		host->irq_cd = -1;
	
	disable_irq(host->irq_cd);

	/* depending on the dma state, get a dma channel to use. */

	if (ls1cmci_host_usedma(host)) {
		host->dma = 0; 
	}
	host->sg_cpu = dma_alloc_coherent(&pdev->dev, 0x100*28, &host->sg_dma, GFP_KERNEL);
	memset(host->sg_cpu, 0 , 0x100*28);
//	printk(" DMA_MEM sg_cpu:0x%08x  sg_dma:0x%08x ...\r\n", host->sg_cpu, host->sg_dma); 

//	host->clk_rate = clk_get_rate(host->clk);
	host->clk_rate = 120000000;  //sdram rate
	mmc->ops 	= &ls1cmci_ops;
	mmc->ocr_avail	= MMC_VDD_32_33 | MMC_VDD_33_34;
#ifdef CONFIG_MMC_ls1c_HW_SDIO_IRQ
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;
#else
	mmc->caps	= MMC_CAP_4_BIT_DATA;
#endif
	mmc->f_min 	= host->clk_rate / (host->clk_div * 256);
	mmc->f_max 	= host->clk_rate / host->clk_div;

	if (host->pdata->ocr_avail)
		mmc->ocr_avail = host->pdata->ocr_avail;

	mmc->max_blk_count	= 4095;
	mmc->max_blk_size	= 4095;
	mmc->max_req_size	= 4095 * 512;
	mmc->max_seg_size	= mmc->max_req_size;

	mmc->max_segs		= 128;
  	dbg(host, dbg_debug, "mapped mci_base:%p irq:%u irq_cd:%u dma:%u.\n",host->base, host->irq, host->irq_cd, host->dma);

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto free_cpufreq;
	}
//	ls1cmci_debugfs_attach(host);
	platform_set_drvdata(pdev, mmc);
	dev_info(&pdev->dev, "%s - using %s, %s SDIO IRQ\n", mmc_hostname(mmc),
		 ls1cmci_host_usedma(host) ? "dma" : "pio",
		 mmc->caps & MMC_CAP_SDIO_IRQ ? "hw" : "sw");

//	enable_irq(host->irq_cd);  // beijing board  dont have  card detect gpio
	tmp = read_c0_status();//???	
	tmp |= 1 << 14;
	write_c0_status(tmp);
	return 0;

 free_cpufreq:
//	ls1cmci_cpufreq_deregister(host);

 free_dmabuf:
	clk_disable(host->clk);

 clk_free:
	clk_put(host->clk);

 probe_free_dma:
//	if (ls1cmci_host_usedma(host))
//		ls1c_dma_free(host->dma, &ls1cmci_dma_client);

 probe_free_gpio_wp:
	if (!host->pdata->no_wprotect)
		gpio_free(host->pdata->gpio_wprotect);

 probe_free_gpio_cd:
	if (!host->pdata->no_detect)
		gpio_free(host->pdata->gpio_detect);

 probe_free_irq_cd:
	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

 probe_free_irq:
	free_irq(host->irq, host);

 probe_iounmap:
	iounmap(host->base);

 probe_free_mem_region:
//	release_mem_region(host->mem->start, resource_size(host->mem));

 probe_free_gpio:
/*	for (i = ls1c_GPE(5); i <= ls1c_GPE(10); i++)
		gpio_free(i);
*/
 probe_free_host:
	mmc_free_host(mmc);

 probe_out:
	return ret;
	return 0;
}

static void ls1cmci_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct ls1cmci_host *host = mmc_priv(mmc);

	if (host->irq_cd >= 0)
		free_irq(host->irq_cd, host);

//	ls1cmci_debugfs_remove(host);
//	ls1cmci_cpufreq_deregister(host);
	mmc_remove_host(mmc);
	clk_disable(host->clk);
}

static int __devexit ls1cmci_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct ls1cmci_host	*host = mmc_priv(mmc);
	struct ls1c_mci_pdata *pd = host->pdata;
	int i;

	ls1cmci_shutdown(pdev);

	clk_put(host->clk);

//	tasklet_disable(&host->pio_tasklet);

//	if (ls1cmci_host_usedma(host))
//		ls1c_dma_free(host->dma, &ls1cmci_dma_client);

	free_irq(host->irq, host);
#if 0
	if (!pd->no_wprotect)
		gpio_free(pd->gpio_wprotect);

	if (!pd->no_detect)
		gpio_free(pd->gpio_detect);

	for (i = ls1c_GPE(5); i <= ls1c_GPE(10); i++)
		gpio_free(i);
#endif

	iounmap(host->base);
	release_mem_region(host->mem->start, resource_size(host->mem));

	mmc_free_host(mmc);
	return 0;
}

static struct platform_device_id ls1cmci_driver_ids[] = {
	{
		.name	= "ls1c_sdio",
		.driver_data	= 0,
	}, 
	{ }
};

MODULE_DEVICE_TABLE(platform, ls1cmci_driver_ids);


#ifdef CONFIG_PM

static int ls1cmci_suspend(struct device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(to_platform_device(dev));

	return mmc_suspend_host(mmc);
}

static int ls1cmci_resume(struct device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(to_platform_device(dev));

	return mmc_resume_host(mmc);
}

static const struct dev_pm_ops ls1cmci_pm = {
	.suspend	= ls1cmci_suspend,
	.resume		= ls1cmci_resume,
};

#define ls1cmci_pm_ops &ls1cmci_pm
#else /* CONFIG_PM */
#define ls1cmci_pm_ops NULL
#endif /* CONFIG_PM */


static struct platform_driver ls1cmci_driver = {
	.driver	= {
		.name	= "ls1c_sdio",
		.owner	= THIS_MODULE,
		.pm	= ls1cmci_pm_ops,
	},
	.id_table	= ls1cmci_driver_ids,
	.probe		= ls1cmci_probe,
	.remove		= __devexit_p(ls1cmci_remove),
	.shutdown	= ls1cmci_shutdown,
};

static int __init ls1cmci_init(void)
{
	return platform_driver_register(&ls1cmci_driver);
}

static void __exit ls1cmci_exit(void)
{
	platform_driver_unregister(&ls1cmci_driver);
}

module_init(ls1cmci_init);
module_exit(ls1cmci_exit);

MODULE_DESCRIPTION("Loongson ls1c MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(" Loongson embed team");
