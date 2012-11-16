/*
 * si3226x_hw.c
 *
 *  Created on: 14.03.2012
 *      Author: Michail Kurochkin
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slic_si3226x.h>
#include <linux/tdm/tdm.h>
#include "si3226x_drv.h"
#include "si3226x_hw.h"
#include "fifo.h"
#include <linux/mutex.h>

/*  Include patch for si3226x */
#include "si3226x_patch_C_FB_2011MAY19.c"

/*  Include generated early setup file */
#include "si3226x_setup.c"

/*  Include generated dtmf digit table */
#include "slic_dtmf_table.c"

static DEFINE_MUTEX(spi_add_lock);


/*
 * si3226x register configurations for line tones
 */
struct si3226x_timer_regs slic_tones[] =
{
		{ /*  signal invitation */
			.osc_amp = 0x784000,
			.osc_freq = 0x78F0000,
			.o_talo = 0,
			.o_tahi = 0,
			.o_tilo = 0,
			.o_tihi = 0,
		},
		{ /*  signal busy */
			.osc_amp = 0x784000,
			.osc_freq = 0x78F0000,
			.o_talo = 0xF0,
			.o_tahi = 0xA,
			.o_tilo = 0xF0,
			.o_tihi = 0xA,
		},
		{ /*  signal wait */
			.osc_amp = 0x7FC000,
			.osc_freq = 0x7810000,
			.o_talo = 0xE0,
			.o_tahi = 0x2E,
			.o_tilo = 0x0,
			.o_tihi = 0x7D,
		},
};

/**
 * convert heximal dtmf code to ASCII code
 * @param dtmf - heximal dtmf code
 * @return ASCII code
 */
static char conv_dtmf_to_char(u8 dtmf)
{
	return dtmf_codes[dtmf];
}

/**
 * ASCII code to convert heximal dtmf code
 * @param �� - ASCII code
 * @return heximal dtmf code or -EINVAL
 */
static int conv_char_to_dtmf(char ch)
{
	int i;

	for(i = 0; i < sizeof dtmf_codes - 1; i++)
		if(dtmf_codes[i] == ch)
			return i;

	return -EINVAL;
}


/**
 * Reset SLIC
 */
static int slic_reset(struct si3226x_slic *slic)
{
	unsigned long timeout;

	gpio_set_value(slic->reset_gpio, 0);

	timeout = jiffies + MSEC(RESET_SLIC_PERIOD);

	while(!(time_after(jiffies, timeout)))
		schedule();

	gpio_set_value(slic->reset_gpio, 1);

	return 0;
}


/**
 * Write value into SLIC register
 * @param channel - channel number
 * @param addr - address SLIC register
 * @param data - register value
 * @return
 * 		0 - OK
 * 		1 - ERROR
 */
int slic_write_reg(struct si3226x_line *line, u8 addr, u8 data)
{
	u8 ch;
	int rc;
	int i;
	struct si3226x_slic *slic = to_si3226x_slic(line);

	mutex_lock(&spi_add_lock);

	if (line && line->ch >= SI3226X_MAX_CHANNELS)
	{
		mutex_unlock(&spi_add_lock);
		dev_err(&line->tdm_dev->dev, "Incorrect slic line number\n");
		return -EINVAL;
	}

	if(line == NULL)
		ch = 0;
	else
		ch = line->ch;


	/**
	 * bit 7 - BRDCST - Indicates a broadcast operation that is intended
	 * for all devices in the daisy chain. This is
	 * only valid for write operations since it would cause contention
	 * on the SDO pin during a read.
		bit 6 - R/W - Read/Write Bit.
			0 = Write operation.
			1 = Read operation.
		bit 5 - REG/RAM - Register/RAM Access Bit.
			0 = RAM access.
			1 = Register access.
		bit 4:0 -  CID[4:0] - Indicates the channel that is targeted by
		 	the operation. Note that the 4-bit channel value is
		 	provided LSB first. The devices reside on the daisy
		 	chain such that device 0 is nearest to the controller,
		 	and device 15 is furthest down the SDI/SDU_THRU chain.
		 	(See Figure 41.)
			As the CID information propagates down the daisy chain,
			each channel decrements the CID by 1. The SDI nodes
			between devices reflect a decrement of 2 per device
			since each device contains two channels. The device
			receiving a value of 0 in the CID field responds
			to the SPI transaction. (See Figure 42.) If a broadcast
			to all devices connected to the chain is requested,
			the CID does not  decrement. In this case, the same
			8-bit or 16-bit data is pre-sented to all channels
			regardless of the CID values.
	 */
	{

		u8 write_transaction[] =
		 {
				 (1 << 5) | ((ch & 1) << 4), /*  control byte */
				 addr,
				 data,
		 };

		for(i = 0; i < sizeof write_transaction; i++)
		{
			rc = spi_write(slic->spi_dev, write_transaction + i, 1);
			if (rc)
			{
				mutex_unlock(&spi_add_lock);
				return rc;
			}
		}
	}

	mutex_unlock(&spi_add_lock);
	return rc;
}


/**
 * Read value from SLIC register
 * @param line - line descriptor or NULL if read GLOBAL register
 * @param addr - address SLIC register
 * @param data - pointer to read data
 * @return 0 - ok
 */
static int slic_read_reg(struct si3226x_line *line, u8 addr, u8 *data)
{
	int rc;
	u8 ch;
	int i;
	struct si3226x_slic *slic = to_si3226x_slic(line);

	mutex_lock(&spi_add_lock);

	if (line && line->ch >= SI3226X_MAX_CHANNELS)
	{
		mutex_unlock(&spi_add_lock);
		return -EINVAL;
	}

	ch = line->ch;

	{
		u8 read_transaction[] =
		{
			(1 << 5) | (1 << 6) | ((ch & 1) << 4), /*  control byte */
			addr,
		};

		for(i = 0; i < sizeof read_transaction; i++)
		{
			rc = spi_write(slic->spi_dev, read_transaction + i, 1);
			if (rc)
			{
				mutex_unlock(&spi_add_lock);
				return rc;
			}
		}
	}

	rc = spi_read(slic->spi_dev, data, 1);

	mutex_unlock(&spi_add_lock);
	return rc;
}


#ifdef DEBUG
/**
 * Read value from SLIC RAM
 * @param channel - channel number
 * @param addr - address SLIC RAM
 * @return value 29bit of 32bit
 */
static int slic_read_ram(struct si3226x_line *line, u16 addr, u32 *value)
{
	int rc;
	u8 *val = (u8 *)value;
	unsigned long timeout;
	u8 state;

	rc = slic_write_reg(line, SI_RAM_ADDR_HI, (addr >> 3) & 0xe0);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_RAM_ADDR_LO, addr & 0xFF);
	if (rc)
		return rc;

	timeout = jiffies + ACCESS_TIMEOUT;
	do
	{
		rc = slic_read_reg(line, SI_REG_RAMSTAT, &state);
		if (rc)
			return rc;

		state &= 1;
		if(time_after(jiffies, timeout))
		{
			dev_err(&line->tdm_dev->dev,
					"Can`t read access to slic RAM\n");
			return -EBUSY;
		}
	}
	while(state);

	rc = slic_read_reg(line, SI_RAM_DATA_B0, val + 0);
	if (rc)
		return rc;

	rc = slic_read_reg(line, SI_RAM_DATA_B1, val + 1);
	if (rc)
		return rc;

	rc = slic_read_reg(line, SI_RAM_DATA_B2, val + 2);
	if (rc)
		return rc;

	rc = slic_read_reg(line, SI_RAM_DATA_B3, val + 3);
	if (rc)
		return rc;

	*value >>= 3;

	return 0;
}
#endif


/**
 * Write value into SLIC RAM
 * @param channel - channel number
 * @param addr - address SLIC RAM
 * @param value - value of SLIC RAM
 * @return
 *
 */
int slic_write_ram(struct si3226x_line *line, u16 addr, u32 value)
{
        int rc;
        unsigned long timeout;
        u8 state;
        u8 *write_data;
        u32 val = value << 3;

        write_data = (u8 *)&val;

        if((!line) || line->ch >= SI3226X_MAX_CHANNELS)
                return -EINVAL;

        value <<= 3;

        rc = slic_write_reg(line, SI_RAM_ADDR_HI, (addr >> 3) & 0xe0);
        if (rc)
                return rc;

        rc = slic_write_reg(line, SI_RAM_DATA_B0,  write_data[0] & 0xF8);
        if (rc)
                return rc;

        rc = slic_write_reg(line, SI_RAM_DATA_B1,  write_data[1]);
        if (rc)
                return rc;

        rc = slic_write_reg(line, SI_RAM_DATA_B2,  write_data[2]);
        if (rc)
                return rc;

        rc = slic_write_reg(line, SI_RAM_DATA_B3,  write_data[3]);
        if (rc)
                return rc;

        rc = slic_write_reg(line, SI_RAM_ADDR_LO, addr & 0xFF);
        if (rc)
                return rc;

        timeout = jiffies + ACCESS_TIMEOUT;
        do
        {
                rc = slic_read_reg(line, SI_REG_RAMSTAT, &state);
                if (rc)
                        return rc;

                state &= 1;
                if(time_after(jiffies, timeout))
                {
                        dev_err(&line->tdm_dev->dev, "Can`t write access to slic RAM\n");
                        return -EBUSY;
                }
        }
        while (state);

        return 0;
}



/**
 * Lock channel. To protect against undersirable/unintended
 * system operation, there are certain critical register control
 * bits that are protected by default against unintentional
 * modification.
 * @param channel - channel number
 * @return 0 - ok
 */
int slic_lock_channel(struct si3226x_line *line)
{
	int rc;
	int i;
	u8 state;
	int attempt_cnt = UNLOCK_ATTEMPT_CNT;
	u8 unlock_data[] = {0x2, 0x8, 0xE, 0};

	do
	{
		rc = slic_read_reg(line, SI_REG_UAM, &state);
		if (rc)
			return rc;

		if((state & 1) == 0) /*  if channel already locked */
			return 0;

		for(i = 0; i < sizeof unlock_data; i++)
		{
			rc = slic_write_reg(line, SI_REG_UAM, unlock_data[i]);
			if (rc)
				return rc;
		}

		rc = slic_read_reg(line, SI_REG_UAM, &state);
		if (rc)
			return rc;

		attempt_cnt--;
	}
	while ((state & 1) && attempt_cnt);

	if (attempt_cnt == 0)
	{
		dev_err(&line->tdm_dev->dev,
				"can`t lock slic channel %d\n", line->ch);
		return -EBUSY;
	}

	return 0;
}


/**
 * Unlock channel. While in protected mode, any writes to
 * protected register bits or protected RAM space are simply
 * ignored by the si3226x.
 * @param channel - channel number
 * @return 0 - ok
 */
int slic_unlock_channel(struct si3226x_line *line)
{
	int rc;
	int i;
	u8 state;
	int attempt_cnt = UNLOCK_ATTEMPT_CNT;

	/* unlock data consecution */
	u8 unlock_data[] = {0x2, 0x8, 0xE, 0};

	do
	{
		rc = slic_read_reg(line, SI_REG_UAM, &state);
		if (rc)
			return rc;

		if((state & 1) != 0) /*  if channel already unlocked */
			return 0;

		for(i = 0; i < sizeof unlock_data; i++)
		{
			rc = slic_write_reg(line, SI_REG_UAM, unlock_data[i]);
			if (rc)
				return rc;
		}

		rc = slic_read_reg(line, SI_REG_UAM, &state);
		if (rc)
			return rc;

		attempt_cnt--;
	}
	while ((!(state & 1)) && attempt_cnt);

	if (attempt_cnt == 0)
	{
		dev_err(&line->tdm_dev->dev,
				"can`t unlock slic channel %d\n", line->ch);
		return -EBUSY;
	}

	return 0;
}


/**
 * Calibrate slic
 * @param slic - slic descriptor
 * @return 0 - ok
 */
int slic_calibrate(struct si3226x_line *line)
{
	int rc;
	u8 data;
	unsigned long timeout;

	rc = slic_write_reg(line, SI_REG_CALR3, SI_VAL_CAL_EN);
	if (rc)
		return rc;

	timeout = jiffies + MSEC(CALIBRATE_TIMEOUT);
	do
	{
		rc = slic_read_reg(line, SI_REG_CALR3, &data);
		if (rc)
			return rc;

		data &= SI_VAL_CAL_EN;

		if(time_after(jiffies, timeout))
		{
			dev_err(&line->tdm_dev->dev,
					"Can`t calibrate slic line %d\n",
					line->ch);
			return -EBUSY;
		}
	}
	while(data);


	return 0;
}


/**
 * Enable hardware echo cancelation
 * @param line - line
 * @return 0 - ok
 */
int slic_enable_echo(struct si3226x_line *line)
{
	int rc;
	u8 data;

	rc = slic_read_reg(line, SI_REG_DIGCON, &data);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_DIGCON, data & (~SI_VAL_HYB_DIS));
	if (rc)
		return rc;

	return 0;
}


/**
 * Disable hardware echo cancelation
 * @param line - line descriptor
 * @return 0 - ok
 */
int slic_disable_echo(struct si3226x_line *line)
{
	int rc;
	u8 data;

	rc = slic_read_reg(line, SI_REG_DIGCON, &data);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_DIGCON, data | SI_VAL_HYB_DIS);
	if (rc)
		return rc;

	return 0;
}


/**
 * function send FSK data from fifo to FXS line.
 * called by irq when hardware fsk buffer is empty
 * @param line - line descriptor
 */
static void do_fsk(struct si3226x_line *line)
{
	u8 data;
	int rc;

	/*  put 8 byte into hardware fsk fifo */
	rc = fifo_pop(&line->fsk, &data);
	if (rc)
	{
		mdelay(300);
		slic_write_reg(line, SI_REG_OCON, 0);
		return;
	}

	slic_write_reg(line, SI_REG_FSKDAT, data);
}


/**
 * Run sending fsk caller id procedure.
 * asynchronous send fsk data by irq
 * @param line - line descriptor
 * @param mode - type of caller id. Declared in enum callerid_modes
 * @param buf - caller id data
 * @param size - caller id size
 * @return 0 - ok
 */
static int
slic_send_fsk(struct si3226x_line *line, int mode, u8 *buf, int size)
{
	int rc;
	int i;
	u8 data;

	if (!size)
		return -EINVAL;

	if (!fifo_is_empty(&line->fsk))
	{
		rc = init_fifo(&line->fsk, 1, CALLERID_BUF_SIZE);
		if (rc)
			return rc;
	}

	data = 0x55;
	for (i = 0; i < 38; i++)
		fifo_push(&line->fsk, &data);

	data = 0xFF;
	for (i = 0; i < 19; i++)
		fifo_push(&line->fsk, &data);

	for (i = 0; i < size; i++)
		fifo_push(&line->fsk, buf + i);

	rc = slic_write_reg(line, SI_REG_OMODE,
			SI_VAL_OSC1_FSK | SI_VAL_ROUTING_1_2);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_FSKDEPTH, SI_VAL_FSK_FLUSH);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_FSKDEPTH, 0);
	if (rc)
		return rc;


	switch (mode)
	{
		case SI_FSK_BELLCORE:
			rc = slic_write_ram(line, SI_RAM_FSKAMP0, 0x105E000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKAMP1, 0x8BE000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKFREQ0, 0x6B60000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKFREQ1, 0x79C0000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSK01, 0x2232000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSK10, 0x77C2000);
			if (rc)
				return rc;

			break;

		case SI_FSK_ETSI:
			rc = slic_write_ram(line, SI_RAM_FSKAMP0, 0x340000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKAMP1, 0x1FA000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKFREQ0, 0x6D20000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSKFREQ1, 0x78B0000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSK01, 0x26E4000);
			if (rc)
				return rc;

			rc = slic_write_ram(line, SI_RAM_FSK10, 0x694C000);
			if (rc)
				return rc;

			break;
	}

	rc = slic_write_reg(line, SI_REG_O1TALO, 0x14);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_O1TAHI, 0);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_OCON, SI_VAL_OSC1_EN | SI_VAL_OSC1_TA_EN);
	if (rc)
		return rc;

	do_fsk(line);
	return 0;
}


/**
 * Disable DTMF recognizing on FXS line
 * @param line
 * @return 0 - ok
 */
static int disable_dtmf_detect(struct si3226x_line *line)
{
	return slic_write_reg(line, SI_REG_TONEN, 0xe3);
}

/**
 * Enable DTMF recognizing on FXS line
 * @param line
 * @return 0 - ok
 */
static int enable_dtmf_detect(struct si3226x_line *line)
{
	return slic_write_reg(line, SI_REG_TONEN, 0xe0);
}

/**
 * Send callerID data
 * @param line - line
 * @param buf - callerID buffer
 * @param size - buffer size
 * @return 0 - ok
 */
static int slic_send_callerid(struct si3226x_line *line, char *buf, int size)
{
	int rc;
	int i;

	switch(line->callerid_mode) {
	case SI_CALLERID_DTMF:
		rc = disable_dtmf_detect(line);
		if (rc)
			return rc;

		for(i = 0; i < size; i++)
		{
			rc = slic_send_dtmf_digit(line, buf[i]);

			if(rc || line->hook_state)
				return enable_dtmf_detect(line);

			msleep(70);
		}

		rc = enable_dtmf_detect(line);
		if (rc)
			return rc;

		break;


	case SI_FSK_BELLCORE:
		rc = slic_send_fsk(line, SI_FSK_BELLCORE, buf, size);
		if (rc)
			return rc;

		break;

	case SI_FSK_ETSI:
		rc = slic_send_fsk(line, SI_FSK_ETSI, buf, size);
		if (rc)
			return rc;

		break;

	case SI_CALLERID_NONE:
		break;
	}

	return 0;
}



/**
 * Set hardware slic channel state
 * @param line - line descriptor
 * @param state - hardware line state
 * @return 0 - ok
 */
static int
slic_set_linefeed_state(struct si3226x_line *line, enum fxs_states state)
{
	int rc;

	rc = slic_write_reg(line, SI_REG_LINEFEED, state);
	if (rc)
		return rc;

	return 0;
}


/**
 * send ring signal to line and send callerid if not null.
 * Functon is blocking while sending callerid
 * @param line - line descriptor
 * @param callerid_buf - buffer witch callerid
 * @param size - caller id buffer size
 * @return 0 - ok
 */
int slic_line_call(struct si3226x_line *line, u8 *callerid_buf, int size)
{
	int stored_size;

	stored_size = size > (sizeof(line->caller_id_buf) - 1) ?
			(sizeof(line->caller_id_buf) - 1) : size;

	memcpy(line->caller_id_buf, callerid_buf, stored_size);
	line->caller_id_buf_size = stored_size;

	schedule_work(&line->line_call_work);

	return 0;
}

/**
 * Work queue IRQ callback handled all slic events
 * @param work - work queue item
 */
void do_line_call(struct work_struct *work)
{
	struct si3226x_line *line = container_of(work, struct si3226x_line, line_call_work);

	u8 data;
	int rc;

	rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_OHT);
	if (rc)
		return;

	rc = slic_set_linefeed_state(line, SI_FXS_RINGING);
	if (rc)
		return;

	if(line->callerid_mode == SI_CALLERID_NONE)
		return;

	msleep(30);

	do
	{
		rc = slic_read_reg(line, SI_REG_LINEFEED, &data);
		if (rc)
			return;

		schedule();
	}
	while (data & (1 << 6));

	msleep(200);

	rc = slic_send_callerid(line, line->caller_id_buf, line->caller_id_buf_size);
	if (rc)
		return;
}

/**
 * Setup slic hardware timer
 * @param line - line descriptor
 * @param timer_num - number of timer
 * @param timer_regs - list specified timer registers values
 * @return 0 - ok
 */
static int
slic_set_timer(struct si3226x_line *line, u8 timer_num,
		struct si3226x_timer_regs *timer_regs)
{
	int rc;

	if(timer_num > 1)
		return -EINVAL;

	rc = slic_write_ram(line, SI_RAM_OSC1AMP + timer_num * 3,
			timer_regs->osc_amp);
	if (rc)
		return rc;

	rc = slic_write_ram(line, SI_RAM_OSC1FREQ + timer_num * 3,
			timer_regs->osc_freq);
	if (rc)
		return rc;

	rc = slic_write_ram(line, SI_RAM_OSC1PHAS + timer_num * 3, 0);
	if (rc)
		return rc;

	/*  configure timer delay */
	rc = slic_write_reg(line, SI_REG_O1TALO + timer_num * 4,
			timer_regs->o_talo);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_O1TAHI + timer_num * 4,
			timer_regs->o_tahi);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_O1TILO + timer_num * 4,
			timer_regs->o_tilo);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_O1TIHI + timer_num * 4,
			timer_regs->o_tihi);
	if (rc)
		return rc;

	return 0;
}


/**
 * Send tone signal to line
 * @param line - line descriptor
 * @param type - type of signal (defined in array slic_tones[])
 * @return 0 - ok
 */
static int
slic_set_signal_to_line(struct si3226x_line *line, enum tone_types type)
{
	int rc;

	if (type > ARRAY_SIZE(slic_tones))
		return -EINVAL;

	/*  disable tone if needed */
	if(type == SI_TONE_NONE)
	{
		rc = slic_write_reg(line, SI_REG_OCON, 0);
		return rc;
	}

	rc = slic_set_timer(line, 0, slic_tones +(int)type);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_OMODE,
			SI_VAL_ROUTING_1_3 | SI_VAL_ZERO_EN_1);
	if (rc)
		return rc;

	switch(type) {
	case SI_TONE_INVITATION:
		rc = slic_write_reg(line, SI_REG_OCON, SI_VAL_OSC1_EN);
		if (rc)
			return rc;

		break;

	case SI_TONE_BUSY:
	case SI_TONE_WAIT:
		rc = slic_write_reg(line, SI_REG_OCON,
			SI_VAL_OSC1_EN | SI_VAL_OSC1_TA_EN | SI_VAL_OSC1_TI_EN);
		if (rc)
			return rc;

		break;

	case SI_TONE_NONE:
		break;
	}

	return 0;
}


/**
 * Set high level line state
 * @param line - line descriptor
 * @param state - needed line state
 * @return 0 - ok
 */
int slic_set_line_state(struct si3226x_line *line, int state)
{
	int rc;

	if (state > SI_LINE_TALK)
		return -EINVAL;

	switch(state) {
	case SI_LINE_DISABLE:
		rc = slic_set_linefeed_state(line, SI_FXS_OPEN);
		if (rc)
			return rc;

		break;

	case SI_LINE_SILENCE:
		rc = slic_set_signal_to_line(line, SI_TONE_NONE);
		if (rc)
			return rc;

		rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_ACTIVE);
		if (rc)
			return rc;

		break;

	case SI_LINE_INVITATION:
		rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_ACTIVE);
		if (rc)
			return rc;

		rc = slic_set_signal_to_line(line, SI_TONE_INVITATION);
		if (rc)
			return rc;

		break;

	case SI_LINE_WAIT:
		rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_ACTIVE);
		if (rc)
			return rc;

		rc = slic_set_signal_to_line(line, SI_TONE_WAIT);
		if (rc)
			return rc;

		break;

	case SI_LINE_BUSY:
		rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_ACTIVE);
		if (rc)
			return rc;

		rc = slic_set_signal_to_line(line, SI_TONE_BUSY);
		if (rc)
			return rc;

		break;

	case SI_LINE_TALK:
		rc = slic_set_signal_to_line(line, SI_TONE_NONE);
		if (rc)
			return rc;

		rc = slic_set_linefeed_state(line, SI_FXS_FORWARD_ACTIVE);
		if (rc)
			return rc;

		break;
	}

	return 0;
}



/**
 * Send one digit by DTMF
 * @param line - line
 * @param ch - heximal DTMF code
 * @return 0 - ok
 */
int slic_send_dtmf_digit(struct si3226x_line *line, char ch)
{
	int rc;
	int i;
	unsigned long timeout;
	u8 data;
	int dtmf_ch;
	struct si3226x_timer_regs timer_regs[2];
	u16 dtmf_delay = DTMF_DIGIT_PERIOD * 1000 / 125;

	memset(timer_regs, 0, sizeof timer_regs);
	dtmf_ch = conv_char_to_dtmf(ch);
	if (dtmf_ch < 0)
		return dtmf_ch;

	if(dtmf_ch > 0xF)
		return -EINVAL;

	timer_regs[0].osc_amp = slic_dtmf_table[dtmf_ch].osc1amp;
	timer_regs[0].osc_freq = slic_dtmf_table[dtmf_ch].osc1freq;
	timer_regs[0].o_talo = (u8)(dtmf_delay & 0xFF);
	timer_regs[0].o_tahi = (u8)((dtmf_delay >> 8) & 0xFF);

	timer_regs[1].osc_amp = slic_dtmf_table[dtmf_ch].osc2amp;
	timer_regs[1].osc_freq = slic_dtmf_table[dtmf_ch].osc2freq;
	timer_regs[1].o_talo = (u8)(dtmf_delay & 0xFF);
	timer_regs[1].o_tahi = (u8)((dtmf_delay >> 8) & 0xFF);

	for(i = 0; i < 2; i++)
	{
		rc = slic_set_timer(line, i, timer_regs + i);
		if (rc)
			return rc;
	}

	/*  Sending tone to */
	rc = slic_write_reg(line, SI_REG_OMODE,
			SI_VAL_ROUTING_1_3 | SI_VAL_ZERO_EN_1 |
			SI_VAL_ROUTING_2_3 | SI_VAL_ZERO_EN_2);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_OCON,
			SI_VAL_OSC1_EN | SI_VAL_OSC1_TA_EN |
			SI_VAL_OSC2_EN | SI_VAL_OSC2_TA_EN);
	if (rc)
		return rc;

	/*  sleeping by DTMF is signalling */
	msleep(DTMF_DIGIT_PERIOD);

	timeout = jiffies + ACCESS_TIMEOUT;
	do
	{
		rc = slic_read_reg(line, SI_REG_OCON, &data);
		if (rc)
			return rc;

		data &= (SI_VAL_EN_SYNC_1 | SI_VAL_EN_SYNC_2);
		if(time_after(jiffies, timeout))
			return -EBUSY;
	}
	while(data);

	return 0;
}


/**
 * Mask irq for slic line
 * @param line - line descriptor
 * @return 0 - ok
 */
static int slic_mask_irq(struct si3226x_line *line)
{
	int rc;
	u8 data;
	int i;

	/*  Cleanup all irq flags */
	for(i = SI_REG_IRQ0; i < SI_REG_IRQ4; i++)
	{
		rc = slic_read_reg(line, i, &data);
		if (rc)
			return rc;
	}

	/*  FSK empty buffer */
	rc = slic_write_reg(line, SI_REG_IRQEN1, SI_VAL_FSKBUF_AVAIL_IA);
	if (rc)
		return rc;

	/*  RX,TX modem tone detector, DTMF, on/off hook */
	rc = slic_write_reg(line, SI_REG_IRQEN2,
			SI_VAL_TXMDM_IA | SI_VAL_RXMDM_IA | SI_VAL_DTMF_IA | SI_VAL_LCR_IA);
	if (rc)
		return rc;

	/*  Termo alarm */
	rc = slic_write_reg(line, SI_REG_IRQEN3, SI_VAL_P_TERM_IA);
	if (rc)
		return rc;

	/*  disable unused irq */
	rc = slic_write_reg(line, SI_REG_IRQEN4, 0);
	if (rc)
		return rc;

	return 0;
}


/**
 * Upload patch for si3226x
 * @param line - patch upload for line
 * @return 0 - ok
 */
static int upload_patch(struct si3226x_line *line)
{
	int rc;
	int i;

	rc = slic_write_reg(line, SI_REG_JMPEN, 0);
	if (rc)
		return rc;

	/*  load patch ram */
	rc = slic_write_ram(line, SI_RAM_PRAM_ADDR, 0);
	if (rc)
		return rc;

	for(i = 0; i < ARRAY_SIZE(si3226x_patch_data); i++)
	{
		rc = slic_write_ram(line, SI_RAM_PRAM_DATA,
				si3226x_patch_data[i]);
		if (rc)
			return rc;
	}

	/*  load jump table */
	for(i = SI_REG_JMP0LO; i < SI_REG_JMP7HI; i++)
	{
		rc = slic_write_reg(line, i,
				si3226x_patch_entries[i - SI_REG_JMP0LO]);
		if (rc)
			return rc;
	}

	/*  load RAM */
	for(i = 0; i < ARRAY_SIZE(si3226x_patch_support_addr); i++)
	{
		rc = slic_write_ram(line, si3226x_patch_support_addr[i],
				si3226x_patch_support_data[i]);
		if (rc)
			return rc;
	}

	return 0;
}


/**
 * Setup slic hardware audio settings
 * @param line - line descriptor
 * @return 0 - ok
 */
int slic_setup_audio(struct si3226x_line *line)
{
	struct tdm_device *tdm_dev = line->tdm_dev;
	struct tdm_controller_hw_settings *tdm_controller_hw =
			tdm_dev->controller->settings;
	u16 time_slot_pos =
		tdm_dev->tdm_channel_num * tdm_controller_hw->channel_size * 8;
	u8 tx_edge;
	u8 bus_format = 0;
	int rc;

	switch(line->companding_mode) {
	case SI_ALAW:
		bus_format = SI_VAL_PCM_FMT_0 | SI_VAL_PCM_ALAW_1;
		break;

	case SI_MLAW:
		bus_format = SI_VAL_PCM_FMT_1;
		break;

	default:
		return -EINVAL;
	}

	tx_edge = (tdm_controller_hw->data_polarity == TDM_POLAR_NEGATIVE) << 4;

	rc = slic_write_reg(line, SI_REG_PCMTXLO, (u8)(time_slot_pos & 0xFF));
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_PCMTXHI,
			(u8)((time_slot_pos >> 8) & 0x3) | tx_edge);
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_PCMRXLO, (u8)(time_slot_pos & 0xFF));
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_PCMRXHI,
			(u8)((time_slot_pos >> 8) & 0x3));
	if (rc)
		return rc;

	rc = slic_write_reg(line, SI_REG_PCMMODE, bus_format | SI_VAL_PCM_EN);
	if (rc)
		return rc;

	if (!line->audio_start_flag)
	{
		rc = tdm_run_audio(tdm_dev);
		if (!rc)
			line->audio_start_flag = 1;
	}

	return 0;
}


/**
 * Init hardware slic line
 * @param line - line descriptor
 * @return 0 - ok
 */
static int init_line(struct si3226x_line *line)
{
	int rc;

	dev_info(&line->tdm_dev->dev, "run init line\n");

	/*  Init default Caller ID mode */
	line->callerid_mode = SI_CALLERID_NONE;

	rc = init_fifo(&line->dtmf, 1, INCOMING_DTMF_BUF_SIZE);
	if (rc)
		return rc;

	rc = init_fifo(&line->fsk, 1, CALLERID_BUF_SIZE);
	if (rc)
		return rc;


	rc = slic_setup_audio(line);
	if (rc)
		return rc;

	rc = slic_set_line_state(line, SI_LINE_SILENCE);
	if (rc)
		return rc;

	rc = slic_mask_irq(line);
	if (rc)
		return rc;

	dev_info(&line->tdm_dev->dev, "init line done\n");

	return 0;
}


/**
 * Init hardware slic
 * @param slic - slic descriptor
 * @return 0 - ok
 */
int init_slic(struct si3226x_slic *slic)
{
	int rc;
	int i;
	struct si3226x_line *line = slic->lines;
	struct platform_device *pdev = slic->pdev;
	u8 chip_id;

	dev_info(&pdev->dev, "run slic initialization\n");

	rc = slic_reset(slic);
	if (rc)
	{
		dev_err(&pdev->dev, "failed to reset SLIC\n");
			return rc;
	}

	printk("slic_reset\n");

	mdelay(50);

	rc = slic_read_reg(line, 0, &chip_id);
	if (rc)
		return rc;

	if (chip_id != 0xC3)
	{
		dev_err(&pdev->dev, "Incorrect slic chip ID: 0x%X\n", chip_id);
		return -ENODEV;
	}



	for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
	{
		if (line->state == SI_LINE_DISABLE)
			continue;

		printk("slic_unlock_channel = %d\n", i);
		rc = slic_unlock_channel(line);
		if (rc)
			return rc;

		printk("upload_patch = %d\n", i);
		rc = upload_patch(line);
		if (rc)
			return rc;

		printk("slic_lock_channel = %d\n", i);
		rc = slic_lock_channel(line);
		if (rc)
			return rc;

		dev_info(&pdev->dev, "line: %d, patch uploaded\n", line->ch);
	}

	printk("slic_load_settings\n");
	rc = slic_load_settings(slic);
	if (rc)
		return rc;

	dev_info(&pdev->dev, "settings loaded\n");

	for (i = 0, line = slic->lines; i < SI3226X_MAX_CHANNELS; i++, line++)
	{
		if (line->state == SI_LINE_DISABLE)
			continue;

		rc = init_line(line);
		if (rc)
			return rc;
	}

	return 0;
}


/**
 * deInit hardware slic. stop slic.
 * @param slic - slic descriptor
 * @return 0 - ok
 */
int deinit_slic(struct si3226x_slic *slic)
{
	struct si3226x_line *line;
	int i;

	for (i = 0, line = slic->lines; i < SI3226X_MAX_CHANNELS; i++, line++)
	{
		if (line->state == SI_LINE_DISABLE)
			continue;

		slic_set_line_state(line, SI_LINE_SILENCE);

		free_fifo(&line->dtmf);
		free_fifo(&line->fsk);
	}

	return 0;
}



/*
 * IRQ event handlers - event_xxx...
 */
static void event_modem_detect(struct si3226x_line *line)
{

}

static void event_dtmf_detect(struct si3226x_line *line)
{
	u8 ch;
	u8 data = 0;

	slic_read_reg(line, SI_REG_TONDTMF, &data);

	/*  if dtmf not recognized */
	if (!(data & SI_VAL_VALID))
		return;

	ch = conv_dtmf_to_char(data & 0xF);

	fifo_push(&line->dtmf, &ch);
}

static void event_hook_detect(struct si3226x_line *line)
{
	u8 data = 0;

	slic_read_reg(line, SI_REG_LCRRTP, &data);

	if (data & SI_VAL_LCR)
		line->hook_state = 1;
	else
		line->hook_state = 0;
}

static void event_error_detect(struct si3226x_line *line, enum si3226x_errors err)
{
	struct si3226x_slic *slic = to_si3226x_slic(line);

	switch(err) {
	case SI_ERR_TERMAL_SHOCK:
		dev_err(&slic->pdev->dev, "SLIC Termal shock! Stopped driver\n");
		release_slic_drv(slic);
		free_slic(slic);
		break;
	}
}



#ifdef CONFIG_SI3226X_POLLING
void slic_delayed_work(struct delayed_work *work)
{
	struct si3226x_slic *slic =
			container_of(work, struct si3226x_slic, delayed_work);

	slic_irq_callback(&slic->irq_work);

	schedule_delayed_work(work, MSEC(50));
}
#endif

/**
 * Interrupt handler for slic
 * @param irq - irq number
 * @param context_data - slic private data
 */
irqreturn_t slic_irq(s32 irq, void *context_data)
{
	struct si3226x_slic *slic = context_data;
	int value = gpio_get_value(slic->int_gpio);

	if (value == 0)
		schedule_work(&slic->irq_work);

	return IRQ_HANDLED;
}


/**
 * Work queue IRQ callback handled all slic events
 * @param work - work queue item
 */
void slic_irq_callback(struct work_struct *work)
{
	struct si3226x_slic *slic =
			container_of(work, struct si3226x_slic, irq_work);

	struct si3226x_line *line = 0;
	u8 global_irq_status;
	u8 data;
	int rc;
	int i;

	rc = slic_read_reg(slic->lines, SI_REG_IRQ0, &global_irq_status);
	if (rc)
		return;

	/*  identificate irq reason */
	for (i = 0; i < 8; i++)
	{
		if (!(global_irq_status & (1 << i)))
			continue;

		switch (i) {
		case 0:
			line = slic->lines + 0;
			rc = slic_read_reg(line, SI_REG_IRQ1, &data);
			if (rc)
				return;

			if (data & SI_VAL_FSKBUF_AVAIL_IA)
				do_fsk(line);
			break;

		case 1:
			line = slic->lines + 0;
			rc = slic_read_reg(line, SI_REG_IRQ2, &data);
			if (rc)
				return;

			if (data & SI_VAL_RXMDM_IA)
				event_modem_detect(line);

			if (data & SI_VAL_TXMDM_IA)
				event_modem_detect(line);

			if (data & SI_VAL_DTMF_IA)
				event_dtmf_detect(line);

			if (data & SI_VAL_LCR_IA)
				event_hook_detect(line);
			break;

		case 2:
			line = slic->lines + 0;
			rc = slic_read_reg(line, SI_REG_IRQ3, &data);
			if (rc)
				return;

			if (data & SI_VAL_P_TERM_IA)
				event_error_detect(line, SI_ERR_TERMAL_SHOCK);
			break;

		case 3:
			line = slic->lines + 0;
			rc = slic_read_reg(line, SI_REG_IRQ4, &data);
			if (rc)
				return;

			break;

		case 4:
			line = slic->lines + 1;
			rc = slic_read_reg(line, SI_REG_IRQ1, &data);
			if (rc)
				return;

			if (data & SI_VAL_FSKBUF_AVAIL_IA)
				do_fsk(line);
			break;

		case 5:
			line = slic->lines + 1;
			rc = slic_read_reg(line, SI_REG_IRQ2, &data);
			if (rc)
				return;

			if (data & SI_VAL_RXMDM_IA)
				event_modem_detect(line);

			if (data & SI_VAL_TXMDM_IA)
				event_modem_detect(line);

			if (data & SI_VAL_DTMF_IA)
				event_dtmf_detect(line);

			if (data & SI_VAL_LCR_IA)
				event_hook_detect(line);
			break;

		case 6:
			line = slic->lines + 1;
			rc = slic_read_reg(line, SI_REG_IRQ3, &data);
			if (rc)
				return;

			if (data & SI_VAL_P_TERM_IA)
				event_error_detect(line, 0);
			break;

		case 7:
			line = slic->lines + 1;
			rc = slic_read_reg(line, SI_REG_IRQ4, &data);
			if (rc)
				return;

			break;
		}
	}
}


