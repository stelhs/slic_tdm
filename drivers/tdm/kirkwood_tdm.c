/*
 * kirkwood_tdm.c
 *
 *  Created on: 19.01.2012
 *      Author: Michail Kurochkin
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tdm/tdm.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/inet_lro.h>
#include <linux/slab.h>
#include <asm/system.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/mbus.h>
#include <linux/miscdevice.h>


#include <linux/platform_device.h>
#include "kirkwood_tdm.h"


/**
 * Init hardware tdm controller
 * @param tdm - tdm_controller descriptor
 * @return
 */
static int kirkwood_tdm_hw_init(struct tdm_controller *tdm)
{
	struct kirkwood_tdm *onchip_tdm =
	    (struct kirkwood_tdm *)dev_get_drvdata(&tdm->dev);
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;
	struct tdm_controller_hw_settings *hw = tdm->settings;
	u32 control_val = 0;
	u32 pclk_freq;
	unsigned long flags;
	spinlock_t lock;
	spin_lock_init(&lock);


	/* Errata GL-CODEC-10: resolve problem witch FS */
	spin_lock_irqsave(&lock, flags);
	writel(0x03ffff00, &regs->pcm_ctrl_reg + 0x1000);
	writel(5, &regs->pcm_ctrl_reg + 0x101C);
	spin_unlock_irqrestore(&lock, flags);


	writel(0, &regs->int_reset_sel);
	writel(0x3ffff, &regs->int_event_mask);
	writel(0, &regs->int_status_mask);
	writel(0, &regs->int_status);

	writeb(0x41, &regs->pcm_ctrl_reg + 0xC41);
	writeb(0x10, &regs->pcm_ctrl_reg + 0xC40);

	/* Configuring PCLK and FS frequency */
	pclk_freq = hw->count_time_slots * 8 * hw->fs_freq;
	switch (pclk_freq) {
	case 256:
		writel(0x1, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x4, &regs->num_time_slot);
		break;

	case 512:
		writel(0x2, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x8, &regs->num_time_slot);
		break;

	case 1024:
		writel(0x4, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x10, &regs->num_time_slot);
		break;

	case 2048:
		writel(0x8, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x20, &regs->num_time_slot);
		break;

	case 4096:
		writel(0x10, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x40, &regs->num_time_slot);
		break;

	case 8192:
		writel(0x20, &regs->tdm_pcm_clock_div);
		writel(0x0, &regs->dummy_rx_write);
		writel(0x80, &regs->num_time_slot);
		break;

	default:
		dev_err(&tdm->dev, "Incorrect count of time slots parameter\n");
		return -EINVAL;
	}

	control_val = readl(&regs->pcm_ctrl_reg);

	if(hw->clock_direction == TDM_CLOCK_OUTPUT)
		control_val &= (u32)~(1 << 0);
	else
		control_val |= (1 << 0);


	if(hw->fs_clock_direction == TDM_CLOCK_OUTPUT)
		control_val &= (u32) ~(1 << 1);
	else
		control_val |= (1 << 1);


	if(hw->fs_polarity == TDM_POLAR_POSITIV)
		control_val &= (u32) ~(1 << 4);
	else
		control_val |= (1 << 4);


	if(hw->data_polarity == TDM_POLAR_POSITIV)
		control_val &= (u32) ~(1 << 2);
	else
		control_val |= (1 << 2);


	if(hw->channel_size == 1)
		control_val &= (u32) ~(1 << 6);
	else
		control_val |= (1 << 6);

	writel(control_val, &regs->pcm_ctrl_reg);
	writel(0, &regs->chan_time_slot_ctrl);
	writel(0, &regs->chan0_enable_disable);
	writel(0, &regs->chan0_enable_disable + 4);

	return 0;
}

/**
 * deInitialization hardware tdm controller
 * @param tdm - tdm_controller descriptor
 */
static void kirkwood_tdm_hw_deinit(struct tdm_controller *tdm)
{
	struct kirkwood_tdm *onchip_tdm =
	    (struct kirkwood_tdm *)dev_get_drvdata(&tdm->dev);
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;

	writel(0, &regs->pcm_ctrl_reg);
	writel(0, &regs->chan_time_slot_ctrl);
}

/**
 * Setup hardware voice channel
 * @param ch - voice hardware channel
 * @return 0 - OK
 */
int kirkwood_setup_voice_channel(struct tdm_voice_channel* ch)
{
	struct tdm_device *tdm_dev = to_tdm_device(ch->dev);
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_controller_hw_settings *hw = tdm->settings;
	struct kirkwood_tdm *onchip_tdm =
	    (struct kirkwood_tdm *)dev_get_drvdata(&tdm->dev);
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;
	unsigned long timeout;
	u32 state;
	int i;


	u8 voice_num = ch->channel_num;

	/* calculate buffer size */
	ch->buffer_len = tdm_dev->buffer_sample_count * hw->channel_size;

	/* Request timeslot for voice channel */
	writeb(ch->tdm_channel, (u8*)&regs->chan_time_slot_ctrl + 2 * voice_num);
	writeb(ch->tdm_channel, (u8*)&regs->chan_time_slot_ctrl + 1 + 2 * voice_num);

	/* FIXME: move coherent_dma_mask to board specific */
	tdm_dev->dev.coherent_dma_mask = 0xffffffff;

	/* Allocate rx and tx buffers */
	for(i = 0; i < COUNT_DMA_BUFFERS_PER_CHANNEL; i++) {
		/* allocate memory for DMA receiver */
		onchip_ch->rx_buf[i] =
		        dma_alloc_coherent(&tdm_dev->dev,
		        ch->buffer_len, onchip_ch->rx_buff_phy + i, GFP_DMA);

		if (onchip_ch->rx_buf == NULL) {
			dev_err(ch->dev, "Can't allocate memory for TDM receiver DMA buffer\n");
			return -ENOMEM;
		}
		memset(onchip_ch->rx_buf[i], 0, ch->buffer_len);


		/* allocate memory for DMA transmitter */
		onchip_ch->tx_buf[i] =
		        dma_alloc_coherent(&tdm_dev->dev,
		        ch->buffer_len, onchip_ch->tx_buff_phy + i, GFP_DMA);

		if (onchip_ch->tx_buf == NULL) {
			dev_err(ch->dev, "Can't allocate memory for TDM transmitter DMA buffer\n");
			return -ENOMEM;
		}
		memset(onchip_ch->tx_buf[i], 0, ch->buffer_len);
	}

	atomic_set(&onchip_ch->write_rx_buf_num, 0);
	atomic_set(&onchip_ch->write_tx_buf_num, 0);
	atomic_set(&onchip_ch->read_rx_buf_num, 0);
	atomic_set(&onchip_ch->read_tx_buf_num, 0);

	/* Set length for DMA */
	writel(((tdm_dev->buffer_sample_count - 32) << 8) | tdm_dev->buffer_sample_count,
	       &regs->chan0_total_sample + voice_num);

	/* Waiting for program transmit DMA */
	timeout = jiffies + HZ;
	do {
		state = readl(&regs->chan0_buff_ownership + 4 * voice_num) & 0x100;
		if(time_after(jiffies, timeout)) {
			dev_err(ch->dev, "Can`t program DMA tx buffer\n");
			return -EBUSY;
		}
	} while(state);

	/* Set DMA buffers fo transmitter */
	writel((u32)onchip_ch->tx_buff_phy[0],
	    &regs->chan0_transmit_start_addr + 4 * voice_num);
	writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num) + 1);

	/* Waiting for program received DMA */
	timeout = jiffies + HZ;
	do {
		state = readl(&regs->chan0_buff_ownership + 4 * voice_num) & 1;
		if(time_after(jiffies, timeout)) {
			dev_err(ch->dev, "Can`t program DMA rx buffer\n");
			return -EBUSY;
		}
	} while(state);

	/* Set DMA buffers for receiver */
	writel((u32)onchip_ch->rx_buff_phy[0],
	    &regs->chan0_receive_start_addr + 4 * voice_num);
	writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num) );

	return 0;
}


/**
 * Run tdm transmitter and receiver
 * @param tdm_dev - tdm device
 * @return 0 - ok
 */
int kirkwood_tdm_run_audio(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;
	struct kirkwood_tdm *onchip_tdm =
	        (struct kirkwood_tdm *)dev_get_drvdata(&tdm->dev);
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;
	struct tdm_voice_channel *ch = tdm_dev->ch;
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;

	memset(onchip_ch->tx_buf[0], 0, ch->buffer_len);
	memset(onchip_ch->tx_buf[1], 0, ch->buffer_len);

	writeb(0x1, (u8 *)(&regs->chan0_enable_disable + 4 * ch->channel_num) + 1);
	writeb(0x1, (u8 *)(&regs->chan0_enable_disable + 4 * ch->channel_num) );

	/* enable Tx interrupts */
	writel(0, &regs->int_status);
	writel((readl(&regs->int_status_mask) | TDM_INT_TX(ch->channel_num)),
	        &regs->int_status_mask);

	/* enable Rx interrupts */
	writel(0, &regs->int_status);
	writel((readl(&regs->int_status_mask) | TDM_INT_RX(ch->channel_num)),
	        &regs->int_status_mask);

	writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * ch->channel_num));
	writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * ch->channel_num) + 1);

	return 0;
}


/**
 * Stop tdm transmitter and receiver
 * @param tdm_dev - tdm device
 * @return 0 - ok
 */
int kirkwood_tdm_stop_audio(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;
	struct kirkwood_tdm *onchip_tdm = (struct kirkwood_tdm *)dev_get_drvdata(&tdm->dev);
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;
	struct tdm_voice_channel *ch = tdm_dev->ch;

	/* disable Tx interrupts */
	writel((readl(&regs->int_status_mask) & (~TDM_INT_TX(ch->channel_num))),
	    &regs->int_status_mask);
	writel(0, &regs->int_status);

	/* disable Rx interrupts */
	writel((readl(&regs->int_status_mask) & (~TDM_INT_RX(ch->channel_num))),
	    &regs->int_status_mask);
	writel(0, &regs->int_status);

	writeb(0x0, (u8 *)(&regs->chan0_enable_disable + 4 * ch->channel_num) + 1);
	writeb(0x0, (u8 *)(&regs->chan0_enable_disable + 4 * ch->channel_num) );

	return 0;
}


/**
 * Get DMA tx buffers latency
 * @param onchip_ch - kirkwood based voice channel
 * @return latency in buffers
 */
int get_tx_latency(struct kirkwood_tdm_voice *onchip_ch)
{
	if (atomic_read(&onchip_ch->read_tx_buf_num) <= atomic_read(&onchip_ch->write_tx_buf_num))
		return atomic_read(&onchip_ch->write_tx_buf_num) - atomic_read(&onchip_ch->read_tx_buf_num);
	else
		return COUNT_DMA_BUFFERS_PER_CHANNEL - atomic_read(&onchip_ch->read_tx_buf_num)
		       + atomic_read(&onchip_ch->write_tx_buf_num);
}


/**
 * Get DMA rx buffers latency
 * @param onchip_ch - kirkwood based voice channel
 * @return latency in buffers
 */
int get_rx_latency(struct kirkwood_tdm_voice *onchip_ch)
{
	if (atomic_read(&onchip_ch->read_rx_buf_num) <= atomic_read(&onchip_ch->write_rx_buf_num))
		return atomic_read(&onchip_ch->write_rx_buf_num) - atomic_read(&onchip_ch->read_rx_buf_num);
	else
		return COUNT_DMA_BUFFERS_PER_CHANNEL - atomic_read(&onchip_ch->read_rx_buf_num)
		       + atomic_read(&onchip_ch->write_rx_buf_num);
}


/**
 * Check rx audio buffer for exist new data
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought data, 1 - data exist
 */
int kirkwood_poll_rx(struct tdm_device *tdm_dev)
{
	struct tdm_voice_channel *ch = tdm_dev->ch;
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;

	return get_rx_latency(onchip_ch) > 1;
}


/**
 * Check tx audio buffer for free space
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought free space, 1 - exist free space
 */
int kirkwood_poll_tx(struct tdm_device *tdm_dev)
{
	struct tdm_voice_channel *ch = tdm_dev->ch;
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;

	return get_tx_latency(onchip_ch) > 1;
}



/**
 * Get next dma buffer number
 * @param num - current buffer number
 * @return next buffer number
 */
static int inc_next_buf_num(int num)
{
	num++;
	if (num >= COUNT_DMA_BUFFERS_PER_CHANNEL)
		num = 0;

	return num;
}



/**
 * Send voice data block to tdm voice channel controller.
 * @param ch - voice channel attendant to transmit data in TDM frame
 * @param data - data to be transmit. Length of data must be equal to
 * 		value returned by get_voice_block_size()
 *
 * Context: can sleep
 * @return 0 on success; negative errno on failure
 */
static int kirkwood_send(struct tdm_voice_channel *ch, u8 *data)
{
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;

	wait_event_interruptible(ch->tx_queue,
	                         get_tx_latency(onchip_ch) > 1);

	memcpy(onchip_ch->tx_buf[atomic_read(&onchip_ch->read_tx_buf_num)], data,
	       ch->buffer_len);

	atomic_set(&onchip_ch->read_tx_buf_num, inc_next_buf_num(atomic_read(&onchip_ch->read_tx_buf_num)));

	return 0;
}


/**
 * Receive voice data block from TDM voice channel controller.
 * @param ch - voice channel attendant to transmit data in TDM frame
 * @param data - pointer to read data received by DMA.
                 Length data for read equal to value returned by get_tdm_voice_block_size()
 *
 * Context: can sleep
 * @return 0 on success; negative errno on failure
 */
static int kirkwood_recv(struct tdm_voice_channel *ch, u8 *data)
{
	struct kirkwood_tdm_voice *onchip_ch = ch->private_data;

	wait_event_interruptible(ch->rx_queue,
	                         get_rx_latency(onchip_ch) > 1);

	memcpy(data, onchip_ch->rx_buf[atomic_read(&onchip_ch->read_rx_buf_num)],
	       ch->buffer_len);

	atomic_set(&onchip_ch->read_rx_buf_num,
	           inc_next_buf_num(atomic_read(&onchip_ch->read_rx_buf_num)));

	return 0;
}



/**
 * kirkwood_tdm_irq - IRQ handler for Kirkwood TDM
 * @irq: IRQ number for this TDM controller
 * @context_data: structure for TDM controller kirkwood_tdm
 * Context: can not sleep
 */
static irqreturn_t kirkwood_tdm_irq(s32 irq, void *context_data)
{
	struct kirkwood_tdm *onchip_tdm = context_data;
	struct kirkwood_tdm_regs *regs = onchip_tdm->regs;
	struct tdm_controller *tdm = to_tdm_controller(onchip_tdm->controller_dev);
	struct tdm_voice_channel *ch;
	struct kirkwood_tdm_voice *onchip_ch;
	struct tdm_device *tdm_dev;

	irqreturn_t ret = IRQ_NONE;
	u32 status;
	u8 i;

	int voice_num; /* current voice channel */
	int next_buf_num; /* number of next buffer */
	int mode; /* irq event mode: */
	int overflow = 0;
	int full = 0;

	enum irq_event_mode {
		IRQ_RECEIVE,
		IRQ_TRANSMIT,
	};

	status = readl(&regs->int_status);

	if ((status & 0xFF) == 0)
		return ret;

	/*  Check first 8 bit in status mask register for detect event type */
	for(i = 0; i < 8; i++) {
		if((status & (1 << i)) == 0)
			continue;

		writel(status & ~(1 << i), &regs->int_status);

		switch(i) {
		case 0:
			mode = IRQ_RECEIVE;
			voice_num = 0;
			overflow = 1;
			break;

		case 1:
			mode = IRQ_TRANSMIT;
			voice_num = 0;
			overflow = 1;
			break;

		case 2:
			mode = IRQ_RECEIVE;
			voice_num = 1;
			overflow = 1;
			break;

		case 3:
			mode = IRQ_TRANSMIT;
			voice_num = 1;
			overflow = 1;
			break;

		case 4:
			mode = IRQ_RECEIVE;
			voice_num = 0;
			overflow = 0;
			full = 0;
			break;

		case 5:
			mode = IRQ_TRANSMIT;
			voice_num = 0;
			overflow = 0;
			full = 0;
			break;

		case 6:
			mode = IRQ_RECEIVE;
			voice_num = 1;
			overflow = 0;
			full = 0;
			break;

		case 7:
			mode = IRQ_TRANSMIT;
			voice_num = 1;
			overflow = 0;
			full = 0;
			break;
		}

		/* ñurrent voice channel struct */
		ch = get_voice_channel_by_num(tdm, voice_num);
		onchip_ch = ch->private_data;

		/* TDM device attached to current voice channel */
		tdm_dev = to_tdm_device(ch->dev);

		switch(mode) {
		case IRQ_RECEIVE: {
			/* get next buffer number, and move write/read pointer */
			next_buf_num = inc_next_buf_num(atomic_read(&onchip_ch->write_rx_buf_num));
			atomic_set(&onchip_ch->write_rx_buf_num, next_buf_num);
			if(next_buf_num == atomic_read(&onchip_ch->read_rx_buf_num))
				atomic_set(&onchip_ch->read_rx_buf_num,
				           inc_next_buf_num(atomic_read(&onchip_ch->read_rx_buf_num)));

			/* if receive overflow event */
			if (overflow) {
				/* set next buffer address */
				writel((u32)onchip_ch->rx_buff_phy[next_buf_num],
				       &regs->chan0_receive_start_addr + 4 * voice_num);
				writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num));

				/* enable receiver */
				writeb(0x1, (u8 *)(&regs->chan0_enable_disable + 4 * voice_num));

				ret = IRQ_HANDLED;
				break;
			}

			/* waiting while dma providing access to buffer */
			while(readl(&regs->chan0_buff_ownership + 4 * voice_num) & 1);

			/* set next buffer address */
			writel((u32)onchip_ch->rx_buff_phy[next_buf_num],
			       &regs->chan0_receive_start_addr + 4 * voice_num);
			writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num));

			wake_up_interruptible(&ch->rx_queue);

			ret = IRQ_HANDLED;
		}
		break;

		case IRQ_TRANSMIT: {
			/* get next buffer number, and move write/read pointer */
			next_buf_num = inc_next_buf_num(atomic_read(&onchip_ch->write_tx_buf_num));
			atomic_set(&onchip_ch->write_tx_buf_num, next_buf_num);
			if(next_buf_num == atomic_read(&onchip_ch->read_tx_buf_num))
				atomic_set(&onchip_ch->read_tx_buf_num,
				           inc_next_buf_num(atomic_read(&onchip_ch->read_tx_buf_num)));

			/* if transmit overflow event */
			if (overflow) {
				/* set next buffer address */
				writel((u32)onchip_ch->tx_buff_phy[next_buf_num],
				       &regs->chan0_transmit_start_addr + 4 * voice_num);
				writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num) + 1);

				/* enable transmitter */
				writeb(0x1, (u8 *)(&regs->chan0_enable_disable + 4 * voice_num) + 1);

				ret = IRQ_HANDLED;
				break;
			}

			/* waiting while dma providing access to buffer */
			while(readl(&regs->chan0_buff_ownership + 4 * voice_num) & 0x100);

			/* set next buffer address */
			writel((u32)onchip_ch->tx_buff_phy[next_buf_num],
			       &regs->chan0_transmit_start_addr + 4 * voice_num);
			writeb(0x1, (u8*)(&regs->chan0_buff_ownership + 4 * voice_num) + 1);

			wake_up_interruptible(&ch->tx_queue);

			ret = IRQ_HANDLED;
		}
		break;
		}
	}

	return ret;
}


/**
 * Configuring mbus windows for correct access to DMA memory
 * @param base_regs - base address for tdm registers
 * @param dram - dram settings
 */
static void kirkwood_tdm_mbus_windows(void *base_regs,
                                      struct mbus_dram_target_info *dram)
{
	int i;

	for (i = 0; i < 4; i++) {
		writel(0, (u8 *)base_regs + TDM_WINDOW_CTRL(i));
		writel(0, (u8 *)base_regs + TDM_WINDOW_BASE(i));
	}

	for (i = 0; i < dram->num_cs; i++) {
		struct mbus_dram_window *cs = dram->cs + i;

		writel(((cs->size - 1) & 0xffff0000) | (cs->mbus_attr << 8) |
		       (dram->mbus_dram_target_id << 4) | 1,
		       (u8 *)base_regs + TDM_WINDOW_CTRL(i));

		writel(cs->base, (u8 *)base_regs + TDM_WINDOW_BASE(i));
	}
}



__devinit
static int kirkwood_tdm_probe(struct platform_device *pdev)
{
	int err = 0;
	struct tdm_controller *tdm;
	struct kirkwood_tdm *onchip_tdm = NULL;
	struct resource *res;
	struct tdm_controller_hw_settings *hw = pdev->dev.platform_data;
	struct tdm_voice_channel *ch;
	int i;

	tdm = tdm_alloc_controller(&pdev->dev, sizeof(struct kirkwood_tdm));
	if (tdm == NULL) {
		dev_err(&pdev->dev, "Can`t alloc memory\n");
		err = -ENOMEM;
		goto out0;
	}

	platform_set_drvdata(pdev, tdm);
	onchip_tdm = tdm_controller_get_devdata(tdm);

	if (pdev->id != -1)
		tdm->bus_num = pdev->id;

	/* Check hardware settings */
	if (hw->fs_freq != 8 && hw->fs_freq != 16) {
		dev_err(&pdev->dev, "Fs frequency may be 8kHz o 16kHz. "
		    "Frequency %d is not incorrect\n", hw->fs_freq);
		err = -EINVAL;
		goto out1;
	}

	if (hw->count_time_slots > 64) {
		dev_err(&pdev->dev, "Incorrect count time slots. "
		    "No more than 64 timeslots per one FS. "
		    "Current set %d\n", hw->count_time_slots);
		err = -EINVAL;
		goto out1;
	}

	if (hw->channel_size > 2 || hw->channel_size == 0) {
		dev_err(&pdev->dev, "Incorrect count time slots. "
		    "No more than 64 timeslots per one FS. "
		    "Current set %d\n", hw->count_time_slots);
		err = -EINVAL;
		goto out1;
	}

	if (hw->clock_direction != TDM_CLOCK_INPUT &&
	    hw->clock_direction != TDM_CLOCK_OUTPUT) {
		dev_err(&pdev->dev, "Incorrect PCLK clock direction value\n");
		err = -EINVAL;
		goto out1;
	}

	if (hw->fs_clock_direction != TDM_CLOCK_INPUT &&
	    hw->fs_clock_direction != TDM_CLOCK_OUTPUT) {
		dev_err(&pdev->dev, "Incorrect FS clock direction value\n");
		err = -EINVAL;
		goto out1;
	}

	if (hw->fs_polarity != TDM_POLAR_NEGATIVE &&
	    hw->fs_polarity != TDM_POLAR_POSITIV) {
		dev_err(&pdev->dev, "Incorrect FS polarity value\n");
		err = -EINVAL;
		goto out1;
	}

	if (hw->data_polarity != TDM_POLAR_NEGATIVE &&
	    hw->data_polarity != TDM_POLAR_POSITIV) {
		dev_err(&pdev->dev, "Incorrect data polarity value\n");
		err = -EINVAL;
		goto out1;
	}

	/* Set controller data */
	tdm->bus_num = pdev->id;
	tdm->settings = hw;
	tdm->setup_voice_channel = kirkwood_setup_voice_channel;
	tdm->recv = kirkwood_recv;
	tdm->send = kirkwood_send;
	tdm->run_audio = kirkwood_tdm_run_audio;
	tdm->stop_audio = kirkwood_tdm_stop_audio;
	tdm->poll_rx = kirkwood_poll_rx;
	tdm->poll_tx = kirkwood_poll_tx;

	for (i = 0; i < KIRKWOOD_MAX_VOICE_CHANNELS; i++)
	{
	        ch = tdm_alloc_voice_channel();
	        if (ch == NULL) {
	                dev_err(&pdev->dev, "Can`t alloc voice channel %d\n", i);
	                goto out1;
	        }

                tdm_register_new_voice_channel(tdm, ch, (void *)(onchip_tdm->voice_channels + i));
	}


	/* Get resources(memory, IRQ) associated with the device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Can`t request registers memory\n");
		err = -ENODEV;
		goto out2;
	}

	onchip_tdm->regs = ioremap(res->start, resource_size(res));
	if (!onchip_tdm->regs) {
		dev_err(&pdev->dev, "Incorrect setup registers memory area\n");
		err = -EINVAL;
		goto out2;
	}

	/* If need remap mbus windows */
	if (hw->dram != NULL)
		kirkwood_tdm_mbus_windows(onchip_tdm->regs, hw->dram);

	/* Get IRQ number for all controllers events */
	onchip_tdm->irq = platform_get_irq(pdev, 0);
	if (onchip_tdm->irq < 0) {
		dev_err(&pdev->dev, "Can`t request IRQ\n");
		err = onchip_tdm->irq;
		goto out3;
	}

	/* Set interrupt callback kirkwood_tdm_irq */
	err = request_irq(onchip_tdm->irq, kirkwood_tdm_irq, IRQF_DISABLED,
	                  dev_name(&pdev->dev), onchip_tdm);
	if (err) {
		dev_err(&pdev->dev, "Can`t setup IRQ callback\n");
		err = -EINVAL;
		goto out4;
	}

	onchip_tdm->controller_dev = &tdm->dev;

	/* Initialization TDM controller */
	err = kirkwood_tdm_hw_init(tdm);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't initialization tdm controller, %d\n",
		        err);
		goto out4;
	}

	err = tdm_controller_register(tdm);
	if(err) {
		dev_err(&pdev->dev, "cannot register tdm controller, %d\n", err);
		goto out5;
	}

	dev_dbg(&tdm->dev, "tdm controller registred sucessfully\n");
	return 0;


out5:
	kirkwood_tdm_hw_deinit(tdm);

out4:
	free_irq(onchip_tdm->irq, onchip_tdm);

out3:
	if (onchip_tdm->regs)
		iounmap(onchip_tdm->regs);

out2:
	tdm_free_voice_channels(tdm);

out1:
	tdm_free_controller(tdm);

out0:
	return err;
}


static int __devexit kirkwood_tdm_remove(struct platform_device *pdev)
{
	struct tdm_controller *tdm = NULL;
	struct kirkwood_tdm *onchip_tdm = NULL;

	tdm = platform_get_drvdata(pdev);
	if (tdm == NULL)
		goto out0;

	onchip_tdm = tdm_controller_get_devdata(tdm);

	kirkwood_tdm_hw_deinit(tdm);
	free_irq(onchip_tdm->irq, onchip_tdm);

	tdm_free_voice_channels(tdm);
	tdm_controller_unregister(tdm);
	if (onchip_tdm->regs)
		iounmap(onchip_tdm->regs);

	tdm_free_controller(tdm);
out0:
	return 0;
}


static struct platform_driver kirkwood_tdm_driver = {
	.probe	= kirkwood_tdm_probe,
	.remove	= __devexit_p(kirkwood_tdm_remove),
	.driver = {
		.name = "kirkwood_tdm",
		.owner = THIS_MODULE,
	},
	.suspend = NULL,
	.resume  = NULL,
};


static int __init kirkwood_init_tdm(void)
{
	return platform_driver_register(&kirkwood_tdm_driver);
}

static void __exit kirkwood_exit_tdm(void)
{
	platform_driver_unregister(&kirkwood_tdm_driver);
}

module_init(kirkwood_init_tdm);
module_exit(kirkwood_exit_tdm);
MODULE_AUTHOR("Michail Kurochkin <stelhs@yandex.ru>");
MODULE_DESCRIPTION("TDM controller driver for Marvel kirkwood arch.");
MODULE_LICENSE("GPL");


