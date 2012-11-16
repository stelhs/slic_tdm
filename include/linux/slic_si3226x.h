/*
 * si3226x.h
 *
 *  Created on: 02.03.2012
 *      Author: Michail Kurochkin
 */

#ifndef SLIC_SI3226X_H_
#define SLIC_SI3226X_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#define SI3226X_MAX_CHANNELS 2 // Count channels

#define SI3226X_IOC_MAGIC 0xde

// SLIC control methods
#define SI3226X_SET_COMPANDING_MODE  _IO(SI3226X_IOC_MAGIC, 0)
#define SI3226X_SET_CALLERID_MODE		_IO(SI3226X_IOC_MAGIC, 1)

// Line control methods
#define SI3226X_SET_ECHO_CANCELATION _IO(SI3226X_IOC_MAGIC, 10)
#define SI3226X_SET_LINE_STATE 		_IO(SI3226X_IOC_MAGIC, 11)
#define SI3226X_CALL					 		_IO(SI3226X_IOC_MAGIC, 12)
#define SI3226X_SEND_DTMF		 		_IO(SI3226X_IOC_MAGIC, 13)
#define SI3226X_GET_HOOK_STATE 		_IO(SI3226X_IOC_MAGIC, 14)
#define SI3226X_GET_DTMF_DIGIT 		_IO(SI3226X_IOC_MAGIC, 15)
#define SI3226X_GET_AUDIO_BLOCK_SIZE _IO(SI3226X_IOC_MAGIC, 16)
#define SI3226X_ENABLE_AUDIO 			_IO(SI3226X_IOC_MAGIC, 17)
#define SI3226X_DISABLE_AUDIO 		_IO(SI3226X_IOC_MAGIC, 18)

/*
 * conteiner for transfer caller id data over ioctl
 */
struct si3226x_caller_id {
	u8 *data;
	int size;
};


/*
 * audio compaunding modes
 */
enum companding_modes {
	SI_ALAW,
	SI_MLAW,
};


/*
 * Board specific data for setup driver SLIC Silabs si3226x
 */
struct si3226x_platform_data {
	int reset_gpio; // number GPIO for reset SLIC
	int int_gpio; // number GPIO for interrupt SLIC

	/*
	 *  fxs_tdm_ch array FXS channels.
	 *  one item is a number of tdm channel number for
	 *  corresponding FXS channel.
	 *  if fxs_tdm_ch[x] == -1 then disable this FXS port
	 */
	int fxs_tdm_ch[SI3226X_MAX_CHANNELS];

	/**
	 * spi chip select requested for si3226x
	 */
	u16 spi_chip_select;

	enum companding_modes companding_mode;
};

#endif /* SLIC_SI3226X_H_ */
