/*
 * si3226x_hw.h
 *
 *  Created on: 14.03.2012
 *      Author: Michail Kurochkin
 */

#ifndef SI3226X_HW_H_
#define SI3226X_HW_H_

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include "si3226x_drv.h"

// define ram registers
#define SI_RAM_ADDR_HI 5
#define SI_RAM_DATA_B0 6
#define SI_RAM_DATA_B1 7
#define SI_RAM_DATA_B2 8
#define SI_RAM_DATA_B3 9
#define SI_RAM_ADDR_LO 10

#define SI_RAM_OSC1FREQ 26
#define SI_RAM_OSC1AMP 27
#define SI_RAM_OSC1PHAS 28
#define SI_RAM_OSC2FREQ 29
#define SI_RAM_OSC2AMP 30
#define SI_RAM_OSC2PHAS 31

#define SI_RAM_FSKAMP0 836
#define SI_RAM_FSKAMP1 837
#define SI_RAM_FSKFREQ0 834
#define SI_RAM_FSKFREQ1 835
#define SI_RAM_FSK01 838
#define SI_RAM_FSK10 839

#define SI_RAM_PRAM_ADDR 1358
#define SI_RAM_PRAM_DATA 1359

#define SI_RAM_PD_DCDC 1538

// define ram values
#define SI_RAM_VAL_REG (u32)(1 << 23)
#define SI_RAM_VAL_OFF (u32)(1 << 20)

// define registers
#define SI_REG_MSTRSTAT 3
#define SI_REG_RAMSTAT 4

#define SI_REG_PCMMODE 11
#define SI_REG_PCMTXLO 12
#define SI_REG_PCMTXHI 13
#define SI_REG_PCMRXLO 14
#define SI_REG_PCMRXHI 15

#define SI_REG_IRQ0 17
#define SI_REG_IRQ1 18
#define SI_REG_IRQ2 19
#define SI_REG_IRQ3 20
#define SI_REG_IRQ4 21

#define SI_REG_IRQEN1 22
#define SI_REG_IRQEN2 23
#define SI_REG_IRQEN3 24
#define SI_REG_IRQEN4 25

#define SI_REG_CALR3 29

#define SI_REG_LINEFEED 30

#define SI_REG_LCRRTP 34

#define SI_REG_DIGCON 44
#define SI_REG_OMODE 48
#define SI_REG_OCON 49
#define SI_REG_TONEN 62

#define SI_REG_O1TALO 50
#define SI_REG_O1TAHI 51
#define SI_REG_O1TILO 52
#define SI_REG_O1TIHI 53
#define SI_REG_O2TALO 54
#define SI_REG_O2TAHI 55
#define SI_REG_O2TILO 56
#define SI_REG_O2TIHI 57

#define SI_REG_FSKDAT 58
#define SI_REG_FSKDEPTH 59
#define SI_REG_TONDTMF 60

#define SI_REG_JMPEN 81
#define SI_REG_JMP0LO 82
#define SI_REG_JMP7HI 97
#define SI_REG_UAM 126


// define register values
#define SI_VAL_FSKBUF_AVAIL_IA ((u8)0x40)
#define SI_VAL_RXMDM_IA ((u8)0x80)
#define SI_VAL_TXMDM_IA ((u8)0x40)
#define SI_VAL_DTMF_IA ((u8)0x10)
#define SI_VAL_LCR_IA ((u8)0x2)
#define SI_VAL_P_TERM_IA ((u8)0x2)
#define SI_VAL_HYB_DIS ((u8)0x10)
#define SI_VAL_CAL_EN ((u8)0x80)
#define SI_VAL_ROUTING_1_3 ((u8)0x3)
#define SI_VAL_ROUTING_1_2 ((u8)0x2)
#define SI_VAL_ROUTING_2_3 ((u8)0x30)
#define SI_VAL_OSC1_TA_EN ((u8)0x4)
#define SI_VAL_OSC1_TI_EN ((u8)0x2)
#define SI_VAL_OSC1_EN ((u8)0x1)
#define SI_VAL_OSC2_TA_EN ((u8)0x40)
#define SI_VAL_OSC2_TI_EN ((u8)0x20)
#define SI_VAL_OSC2_EN ((u8)0x10)

#define SI_VAL_VALID ((u8)0x20)

#define SI_VAL_ZERO_EN_1 ((u8)0x4)
#define SI_VAL_ZERO_EN_2 ((u8)0x40)

#define SI_VAL_OSC1_FSK ((u8)0x8)

#define SI_VAL_FSK_FLUSH ((u8)0x8)
#define SI_VAL_FSKBUF_DEPTH_7 ((u8)0x7)

#define SI_VAL_PCM_EN ((u8)0x10)
#define SI_VAL_PCM_FMT_0 ((u8)0x0)
#define SI_VAL_PCM_FMT_1 ((u8)0x1)

#define SI_VAL_PCM_ALAW_0 ((u8)0x0)
#define SI_VAL_PCM_ALAW_1 ((u8)0x4)
#define SI_VAL_PCM_ALAW_2 ((u8)0x8)
#define SI_VAL_PCM_ALAW_3 ((u8)0xC)

#define SI_VAL_LCR ((u8)0x2)

#define SI_VAL_EN_SYNC_1 ((u8)0x8)
#define SI_VAL_EN_SYNC_2 ((u8)0x80)
/*
 * si3226x hardware line states
 */
enum fxs_states {
	SI_FXS_OPEN,
	SI_FXS_FORWARD_ACTIVE,
	SI_FXS_FORWARD_OHT,
	SI_FXS_TIP_OPEN,
	SI_FXS_RINGING,
	SI_FXS_REVERSE_ACTIVE,
	SI_FXS_REVERSE_OHT,
	SI_FXS_RING_OPEN,
};

/*
 * registers content for generate one DTMF digit
 */
struct si3226x_dtmf_digit {
	u32 osc1amp;
	u32 osc2amp;
	u32 osc1freq;
	u32 osc2freq;
};

/*
 * ATS answer tone register values
 */
struct si3226x_timer_regs {
	u32 osc_amp;
	u32 osc_freq;
	u8 o_talo;
	u8 o_tahi;
	u8 o_tilo;
	u8 o_tihi;
	u8 o_con;
};

/*
 * ATS answer tone types
 */
enum tone_types {
	SI_TONE_INVITATION,
	SI_TONE_BUSY,
	SI_TONE_WAIT,
	SI_TONE_NONE,
};


/*
 * si3226x hardware errors
 */
enum si3226x_errors {
	SI_ERR_TERMAL_SHOCK,
};

//extern struct si3226x_dtmf_digit slic_dtmf_table[];

int init_slic(struct si3226x_slic *slic);
int deinit_slic(struct si3226x_slic *slic);
int slic_setup_audio(struct si3226x_line *line);
int slic_set_line_state(struct si3226x_line *line, int state);
int slic_line_call(struct si3226x_line *line, u8 *callerid_buf, int size);
int slic_send_dtmf_digit(struct si3226x_line *line, char ch);
int slic_calibrate(struct si3226x_line *line);
int slic_enable_echo(struct si3226x_line *line);
int slic_disable_echo(struct si3226x_line *line);
irqreturn_t slic_irq(s32 irq, void *context_data);
int slic_unlock_channel(struct si3226x_line *line);
int slic_lock_channel(struct si3226x_line *line);
int slic_write_ram(struct si3226x_line *line, u16 addr, u32 value);
int slic_write_reg(struct si3226x_line *line, u8 addr, u8 data);
void slic_irq_callback(struct work_struct *work);
void do_line_call(struct work_struct *work);


/*
 * Sleep current process during timeout "msec"
 */
#define si_msleep(msec) \
{	\
	unsigned long timeout = jiffies + MSEC(msec);	\
	while(!time_after(jiffies, timeout))	\
		schedule();	\
}



#ifdef CONFIG_SI3226X_POLLING
void slic_delayed_work(struct delayed_work *work);
#endif

#endif /* SI3226X_HW_H_ */
