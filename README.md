Description of the TDM subsystem:

The code which registers new TDM bus:
/drivers/tdm/tdm_code.c
/include/linux/tdm/tdm.h

TDM subsystem provides API for registering of various TDM controller drivers:

    tdm_alloc_controller() - alloc memory for TDM controller driver
    tdm_free_controller() - free memory allocated for TDM controller driver
    tdm_controller_register() - register TDM controller driver on TDM bus
    tdm_controller_unregister() - unregister TDM controller driver

    tdm_alloc_voice_channel() - allocate memory for voice channel
    tdm_register_new_voice_channel() - register voice channel on TDM controller
    tdm_free_voice_channels() - unregister and free voice channels
    tdm_register_board_info() - register TDM devices list from board_specific code

TDM controller driver attached to TDM bus. Provides the following API for TDM device drivers:

    tdm_new_device() - allocate memory and create tdm_device
    tdm_add_device() - add tdm device to TDM bus
    request_voice_channel() - attempt to request TDM voice channel for TDM device
    release_voice_channel() - release TDM voice channel
    tdm_run_audio() - enable audio receive/transmit
    tdm_stop_audio() - disable audio receive/transmit
    tdm_recv() - receive one block audio data
    tdm_send() - transmit one block audio data
    tdm_poll_rx() - check for exist incomming data
    tdm_poll_tx() - check free space for transmitting data
    tdm_get_voice_block_size() - return audio block size

TDM controller driver for ARM Marvell kirkwood CPU.
Was tested on CPU ARM9 Marvell 88F6283. This CPU supports two TDM audio channels.

/drivers/tdm/kirkwood_tdm.c
/drivers/tdm/kirkwood_tdm.h

Driver for SLIC Silabs si3226x based on SPI and TDM frameworks.

/drivers/staging/si3226x/si3226x_drv.c
/drivers/staging/si3226x/si3226x_hw.c

Provides two FXS channels and creates the following device files:

/dev/si3226x_cnt.x
/dev/si3226x_fxs0.x
/dev/si3226x_fxs1.x