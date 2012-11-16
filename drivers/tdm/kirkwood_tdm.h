/*
 * kirkwood_tdm.h
 *
 *  Created on: 25.01.2012
 *      Author: Michail Kurochkin
 */

#ifndef KIRKWOOD_TDM_H_
#define KIRKWOOD_TDM_H_

#include <linux/module.h>
#include <linux/kernel.h>

struct kirkwood_tdm_regs {
	u32 pcm_ctrl_reg; /*  0x0 PCM control register */
	u32 chan_time_slot_ctrl; /*  0x4 Channel Time Slot Control Register */
	u32 chan0_delay_ctrl; /*  0x8 Channel 0 Delay Control Register */
	u32 chan1_delay_ctrl; /*  0xC Channel 1 Delay Control Register */
	u32 chan0_enable_disable; /*  0x10 Channel 0 Enable and Disable Register */
	u32 chan0_buff_ownership; /*  0x14 Channel 0 Buffer Ownership Register */
	u32 chan0_transmit_start_addr; /*  0x18 Channel 0 Transmit Data Start Address Register */
	u32 chan0_receive_start_addr; /*  0x1C Channel 0 Receive Data Start Address Register */
	u32 chan1_enable_disable; /*  0x20 Channel 1 Enable and Disable Register */
	u32 chan1_buff_ownership; /*  0x24 Channel 1 Buffer Ownership Register */
	u32 chan1_transmit_start_addr; /*  0x28 Channel 1 Transmit Data Start Address Register */
	u32 chan1_receive_start_addr; /*  0x2C Channel 1 Receive Data Start Address Register */
	u32 chan0_total_sample; /*  0x30 Channel 0 Total Sample Count Register */
	u32 chan1_total_sample; /*  0x34 Channel 1 Total Sample Count Register */
	u32 num_time_slot; /*  0x38 Number of Time Slot Register */
	u32 tdm_pcm_clock_div; /*  0x3C TDM PCM Clock Rate Divisor Register */
	u32 int_event_mask; /*  0x40 Interrupt Event Mask Register */
	u32 reserved_44h; /*  0x44 */
	u32 int_status_mask; /*  0x48 Interrupt Status Mask Register */
	u32 int_reset_sel; /*  0x4C Interrupt Reset Selection Register */
	u32 int_status; /*  0x50 Interrupt Status Register */
	u32 dummy_rx_write;/*  0x54 Dummy Data for Dummy RX Write Register */
	u32 misc_control; /*  0x58 Miscellaneous Control Register */
	u32 reserved_5Ch; /*  0x5C */
	u32 chan0_current_tx_addr; /*  0x60 Channel 0 Transmit Data Current Address Register (for DMA) */
	u32 chan0_current_rx_addr;  /*  0x64 Channel 0 Receive Data Current Address Register (for DMA) */
	u32 chan1_current_tx_addr; /*  0x68 Channel 1 Transmit Data Current Address Register (for DMA) */
	u32 chan1_current_rx_addr; /*  0x6C Channel 1 Receive Data Current Address Register (for DMA) */
	u32 curr_time_slot; /*  0x70 Current Time Slot Register */
	u32 revision; /*  0x74 TDM Revision Register */
	u32 chan0_debug; /*  0x78 TDM Channel 0 Debug Register */
	u32 chan1_debug; /*  0x7C TDM Channel 1 Debug Register */
	u32 tdm_dma_abort_1; /*  0x80 TDM DMA Abort Register 1 */
	u32 tdm_dma_abort_2; /*  0x84 TDM DMA Abort Register 2 */
	u32 chan0_wideband_delay_ctrl; /*  0x88 TDM Channel 0 Wideband Delay Control Register */
	u32 chan1_wideband_delay_ctrl; /*  0x8C TDM Channel 1 Wideband Delay Control Register */
};


#define KIRKWOOD_MAX_VOICE_CHANNELS 2 /*  Max count hardware channels */
#define COUNT_DMA_BUFFERS_PER_CHANNEL 6 /*  Count of dma buffers for tx or rx path by one channel */

/*
 * Data specified for kirkwood hardware voice channel
 */
struct kirkwood_tdm_voice {
	/* Transmitter and receiver buffers split to half.
	 * While first half buffer is filling by DMA controller,
	 * second half buffer is used by consumer and etc.
	*/
	u8 *tx_buf[COUNT_DMA_BUFFERS_PER_CHANNEL]; /*  transmitter voice buffers */
	u8 *rx_buf[COUNT_DMA_BUFFERS_PER_CHANNEL]; /*  receiver voice buffers pointer */

	dma_addr_t tx_buff_phy[COUNT_DMA_BUFFERS_PER_CHANNEL]; /*  two physical pointers to tx_buf */
	dma_addr_t rx_buff_phy[COUNT_DMA_BUFFERS_PER_CHANNEL]; /*  two physical pointers to rx_buf */
	atomic_t write_tx_buf_num; /*  current writing transmit buffer number */
	atomic_t read_tx_buf_num; /*  current reading transmit buffer number */
	atomic_t write_rx_buf_num; /*  current writing receive buffer number */
	atomic_t read_rx_buf_num; /*  current reading receive buffer number */
};


/*
 * Data specified for kirkwood tdm controller
 */
struct kirkwood_tdm {
	struct kirkwood_tdm_regs *regs; /*  Registers for hardware TDM */
	u32			irq; /*  Irq number for all TDM operations */

	struct kirkwood_tdm_voice voice_channels[KIRKWOOD_MAX_VOICE_CHANNELS];
	struct device *controller_dev;
};


#define TDM_WINDOW_CTRL(i)		(0x4030 + ((i) << 4))
#define TDM_WINDOW_BASE(i)	(0x4034 + ((i) << 4))


/* INT_STATUS_REG bits */
#define RX_OVERFLOW_BIT(ch)	(1<<(0+(ch)*2))
#define TX_UNDERFLOW_BIT(ch)	(1<<(1+((ch)*2)))
#define RX_BIT(ch)		(1<<(4+((ch)*2)))
#define TX_BIT(ch)		(1<<(5+((ch)*2)))
#define RX_IDLE_BIT(ch)		(1<<(8+((ch)*2)))
#define TX_IDLE_BIT(ch)		(1<<(9+((ch)*2)))
#define RX_FIFO_FULL(ch)	(1<<(12+((ch)*2)))
#define TX_FIFO_EMPTY(ch)	(1<<(13+((ch)*2)))
#define DMA_ABORT_BIT		(1<<16)
#define SLIC_INT_BIT		(1<<17)
#define TDM_INT_TX(ch)	(TX_UNDERFLOW_BIT(ch) | TX_BIT(ch))
#define TDM_INT_RX(ch)	(RX_OVERFLOW_BIT(ch) | RX_BIT(ch))


#endif /* KIRKWOOD_TDM_H_ */


