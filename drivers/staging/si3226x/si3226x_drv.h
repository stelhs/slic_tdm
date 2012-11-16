/*
 * si3226x_drv.h
 * Driver SLIC Silabs si3226x
 *
 *  Created on: 01.03.2012
 *      Author: Michail Kurochkin
 */

#ifndef SI3226X_H_
#define SI3226X_H_

#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/tdm/tdm.h>
#include <linux/slic_si3226x.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include "fifo.h"


#define DRIVER_NAME "si3226x"

/**
 * Calculate jiffies value for miliseconds
 * @param ms - time in miliseconds
 */
#define MSEC(ms) msecs_to_jiffies(ms)

#define RESET_SLIC_PERIOD 100 /*  100ms */
#define DTMF_DIGIT_PERIOD 70 /*  DTMF tone delay */
#define INCOMING_DTMF_BUF_SIZE 10 /*  Buffer size for incommint DTMF digits */
#define CALLERID_BUF_SIZE 128 /*  buffer size for FSK caller id information */
#define CALIBRATE_TIMEOUT 5000 /*  5s */
#define ACCESS_TIMEOUT MSEC(1000) /*  register access timeout */
#define UNLOCK_ATTEMPT_CNT 3 /*  Count retry for lock/unlock slic */

#define SI3226X_MAJOR 40
/*
 * Caller ID modes
 */
enum callerid_modes
{
	SI_FSK_BELLCORE,
	SI_FSK_ETSI,
	SI_CALLERID_DTMF,
	SI_CALLERID_NONE,
};


/*
 * high level line states
 */
enum line_states
{
	SI_LINE_DISABLE, /*  disable line and voltage */
	SI_LINE_SILENCE, /*  hook on */
	SI_LINE_WAIT, /*  generate long tone signals */
	SI_LINE_INVITATION, /*  generate permanent tone to line */
	SI_LINE_BUSY, /*  generate short tone signals to line */
	SI_LINE_TALK, /*  voice transaction, audio enable */
};

/*
 * current telephone hook states
 */
enum hook_states
{
	SI_HOOK_ON,
	SI_HOOK_OFF,
};

/*
 * si3226x line structure
 */
struct si3226x_line
{
	u8 ch; /*  number of fxs channel */

	dev_t devt; /*  major/minor number of line device */

	enum line_states state; /*  current line state */
	enum hook_states hook_state; /*  current telephone hook state */

	enum companding_modes companding_mode; /*  a-law or m-law */

	/*  audio buffers */
	u8 *tx_buffer; /*  private transmit buffer */
	u8 *rx_buffer; /*  private receive buffer */
	int audio_buffer_size; /*  audio buffer size */

	wait_queue_head_t *rx_wait_queue;
	wait_queue_head_t *tx_wait_queue;

	struct tdm_device *tdm_dev; /*  tdm device for transfer audio data */

	u8 file_opened : 1;  /*  flag indicate opened device file */
	u8 audio_start_flag;  /*  flag indicate audio start on line */

	enum callerid_modes callerid_mode; /*  SI_CALLERID_DTMF, SI_FSK_BELLCORE, SI_FSK_ETSI */
	struct work_struct line_call_work; /*  work queue for line call and caller_id routine */
	u8 caller_id_buf[255]; /*  caller_id data buffer for send in line */
	int caller_id_buf_size; /*  caller_id size of data buffer */

	struct fifo_buffer dtmf; /* buffer for incomming all DTMF digits */
	struct fifo_buffer fsk; /* buffer for transmit fsk */
};


/*
 * si3226x controller structure
 */
struct si3226x_slic
{
	dev_t devt;

	struct platform_device *pdev;
	struct spi_device *spi_dev;
	struct si3226x_line lines[SI3226X_MAX_CHANNELS];

	int irq; /*  IRQ number for slic */
	int reset_gpio; /*  GPIO for reset slic */
	int int_gpio; /*  GPIO interrupt for slic */
	struct work_struct irq_work;

#ifdef CONFIG_SI3226X_POLLING
	struct delayed_work delayed_work;
#endif

	u8 file_opened : 1;  /*  flag indicate opened device file */
};


/*
 * slic device types
 */
enum slic_dev_type
{
	PLAT_DEVICE,
	SPI_DEVICE,
	TDM_DEVICE,
};

/*
 * container for slic list probed devices
 */
struct slic_dev_list
{
	struct list_head list;
	struct device *dev;
	enum slic_dev_type type;
};

/*
 * device data types
 */
enum chr_dev_type
{
	SLIC_CHR_DEV, /*  type si3226x_slic */
	LINE_CHR_DEV, /*  type si3226x_line */
};

/*
 * container for list registred character devices
 */
struct slic_chr_dev
{
	struct list_head list;
	struct device *dev; /*  character device; */
	enum chr_dev_type type; /*  character device type; */
};


static inline struct si3226x_slic *to_si3226x_slic(struct si3226x_line *line)
{
	return line ? container_of(line - line->ch, struct si3226x_slic, lines[0]) : NULL;
}

void release_slic_drv(struct si3226x_slic *slic);
int free_slic(struct si3226x_slic *slic);

#endif /* SI3226X_H_ */
