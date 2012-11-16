/*
 * tdm.c
 *
 *  Created on: 19.01.2012
 *      Author: Michail Kurochkin
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/spinlock.h>
//#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/tdm/tdm.h>

#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>

/**
 *  Release TDM device
 * @param dev
 */
static void tdmdev_release(struct device *dev)
{
	struct tdm_device	*tdm_dev = to_tdm_device(dev);

	/* spi masters may cleanup for released devices */
	if (tdm_dev->controller->cleanup)
		tdm_dev->controller->cleanup(tdm_dev);

	put_device(&tdm_dev->controller->dev);
	kfree(tdm_dev);
}

static ssize_t
modalias_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct tdm_device	*tdm_dev = to_tdm_device(dev);

	return sprintf(buf, "%s\n", tdm_dev->modalias);
}

static struct device_attribute tdm_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};



static int tdm_match_device(struct device *dev, struct device_driver *drv)
{
	const struct tdm_device	*tdm_dev = to_tdm_device(dev);

	return strcmp(tdm_dev->modalias, drv->name) == 0;
}


static int tdm_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct tdm_device	*tdm_dev = to_tdm_device(dev);

	add_uevent_var(env, "MODALIAS=%s%s", "tdm:", tdm_dev->modalias);
	return 0;
}


static void tdm_release(struct device *dev)
{
	struct tdm_device *tdm_dev;

	tdm_dev = to_tdm_device(dev);

	kfree(tdm_dev);
}


static struct class tdm_class = {
		.name		= "tdm",
		.owner		= THIS_MODULE,
		.dev_release	= tdm_release,
	};



/**
 * Struct for TDM bus
 */
struct bus_type tdm_bus_type = {
	.name		= "tdm",
	.dev_attrs	= tdm_dev_attrs,
	.match		= tdm_match_device,
	.uevent		= tdm_uevent,
	.suspend	= NULL,
	.resume		= NULL,
};
EXPORT_SYMBOL_GPL(tdm_bus_type);



static int tdm_drv_probe(struct device *dev)
{
	const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

	return tdm_drv->probe(to_tdm_device(dev));
}

static int tdm_drv_remove(struct device *dev)
{
	const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

	return tdm_drv->remove(to_tdm_device(dev));
}

static void tdm_drv_shutdown(struct device *dev)
{
	const struct tdm_driver *tdm_drv = to_tdm_driver(dev->driver);

	tdm_drv->shutdown(to_tdm_device(dev));
}


/**
 * tdm_register_driver - register a TDM driver
 * @sdrv: the driver to register
 * Context: can sleep
 */
int tdm_register_driver(struct tdm_driver *tdm_drv)
{
	tdm_drv->driver.bus = &tdm_bus_type;

	if (tdm_drv->probe)
		tdm_drv->driver.probe = tdm_drv_probe;

	if (tdm_drv->remove)
		tdm_drv->driver.remove = tdm_drv_remove;

	if (tdm_drv->shutdown)
		tdm_drv->driver.shutdown = tdm_drv_shutdown;

	return driver_register(&tdm_drv->driver);
}
EXPORT_SYMBOL_GPL(tdm_register_driver);


/**
 * tdm_unregister_driver - reverse effect of tdm_register_driver
 * @sdrv: the driver to unregister
 * Context: can sleep
 */
void tdm_unregister_driver(struct tdm_driver *tdm_dev)
{
	if (tdm_dev)
		driver_unregister(&tdm_dev->driver);
}
EXPORT_SYMBOL_GPL(tdm_unregister_driver);



/**
 * Request unused voice channel
 * @param tdm_dev - TDM device requested voice channel
 * @return pointer to voice channel
 */
struct tdm_voice_channel *request_voice_channel(struct tdm_device *tdm_dev) {
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_voice_channel *ch = tdm->voice_channels;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ch->lock, flags);

	for (i = 0; i < tdm->count_voice_channels; i++, ch++)
		if (ch->dev == NULL) {
			ch->dev = &tdm_dev->dev;
			tdm_dev->ch = ch;
			spin_unlock_irqrestore(&ch->lock, flags);
			return ch;
		}

	spin_unlock_irqrestore(&ch->lock, flags);

	return NULL;
}


/**
 *
 * @param tdm_dev
 * @return
 */
struct tdm_voice_channel *
get_requested_tdm_voice_channel(struct tdm_device *tdm_dev) {
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_voice_channel *ch = tdm_dev->controller->voice_channels;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ch->lock, flags);

	for (i = 0; i < tdm->count_voice_channels; i++, ch++)
		if (ch->dev == &tdm_dev->dev) {
			spin_unlock_irqrestore(&ch->lock, flags);
			return ch;
		}

	spin_unlock_irqrestore(&ch->lock, flags);

	return NULL;
}

/**
 * Release requested voice channel
 * @param TDM device requested early voice channel
 * @return 0 - OK
 */
int release_voice_channel(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_voice_channel *ch = tdm_dev->controller->voice_channels;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ch->lock, flags);

	for (i = 0; i < tdm->count_voice_channels; i++, ch++)
		if (ch->dev == &tdm_dev->dev) {
			ch->dev = NULL;
			return 0;
		}

	spin_unlock_irqrestore(&ch->lock, flags);

	return -ENODEV;
}



/*
 * Container for union board info of all TDM devices
 */
struct tdm_board_devices {
	struct list_head	list;
	struct tdm_board_info	bi; // Board specific info
};

static LIST_HEAD(tdm_devices_list); // List of all board specific TDM devices
static LIST_HEAD(tdm_controller_list); // List of all registred TDM controllers

/*
 * Used to protect add/del opertion for board_info list and
 * tdm_master list, and their matching process
 */
static DEFINE_MUTEX(board_lock);


static void tdm_match_controller_to_boardinfo(struct tdm_controller *tdm,
        struct tdm_board_info *bi)
{
	struct tdm_device *tdm_dev;

	printk("tdm_match_controller_to_boardinfo\n");

	if (tdm->bus_num != bi->bus_num)
		return;

	printk("tdm->bus_num == bi->bus_num\n");

	tdm_dev = tdm_new_device(tdm, bi);
	if (!tdm_dev)
		dev_err(tdm->dev.parent, "can't create new device for %s\n",
		        bi->modalias);
}



/**
 * Allocate memory for TDM controller device
 * @dev: the controller, possibly using the platform_bus
 * @size: how much zeroed driver-private data to allocate; the pointer to this
 *	memory is in the driver_data field of the returned device,
 *	accessible with tdm_controller_get_devdata().
 * Context: can sleep
 */
struct tdm_controller *tdm_alloc_controller(struct device *dev, unsigned size) {
	struct tdm_controller	*tdm;

	if (!dev)
		return NULL;

	tdm = kzalloc(sizeof *tdm + size, GFP_KERNEL);
	if (!tdm)
		return NULL;

	device_initialize(&tdm->dev);
	tdm->dev.class = &tdm_class;
	tdm->dev.parent = get_device(dev);

	dev_set_drvdata(&tdm->dev, tdm + 1);

	return tdm;
}
EXPORT_SYMBOL_GPL(tdm_alloc_controller);


/**
 * Free memory for TDM controller device, allocated by tdm_alloc_controller
 * @dev: the controller, possibly using the platform_bus
 * Context: can sleep
 */
int tdm_free_controller(struct tdm_controller *tdm)
{
	if (!tdm)
		return -EINVAL;

	dev_set_drvdata(&tdm->dev, NULL);
	put_device(&tdm->dev);
	kzfree(tdm);

	return 0;
}
EXPORT_SYMBOL_GPL(tdm_free_controller);


/**
 * Allocate memory for voice channels for tdm_controller
 * @param count_channels - count supported voice channels
 * @return array of struct voice_channel or NULL if error
 */
struct tdm_voice_channel *tdm_alloc_voice_channels(u8 count_channels) {
	struct tdm_voice_channel *alloc_ch;
	struct tdm_voice_channel *ch;
	int i;

	alloc_ch = kzalloc(count_channels * sizeof *alloc_ch, GFP_KERNEL);
	if (!alloc_ch)
		return NULL;

	memset(alloc_ch, 0, count_channels * sizeof *alloc_ch);

	ch = 	alloc_ch;
	for (i = 0; i < count_channels; i++, ch++) {
		ch->channel_num = i;
		spin_lock_init(&ch->lock);
		init_waitqueue_head(&ch->tx_queue);
		init_waitqueue_head(&ch->rx_queue);
	}

	return alloc_ch;
}
EXPORT_SYMBOL_GPL(tdm_alloc_voice_channels);

/**
 * Free memory for voice channels allocated by tdm_alloc_voice_channels
 * @param tdm - TDM controller
 * @param count_channels - count supported voice channels
 * @return
 */
int tdm_free_voice_channels(struct tdm_voice_channel *ch)
{
	printk("tdm_free_voice_channels\n");

	if (!ch)
		return -EINVAL;

	kzfree(ch);
	return 0;
}
EXPORT_SYMBOL_GPL(tdm_free_voice_channels);


/**
 * tdm_controller_register - register TDM controller
 * @master: initialized master, originally from spi_alloc_master()
 * Context: can sleep
 *
 * This must be called from context that can sleep.  It returns zero on
 * success, else a negative error code (dropping the master's refcount).
 * After a successful return, the caller is responsible for calling
 * tdm_controller_unregister().
 */
int tdm_controller_register(struct tdm_controller *tdm)
{
	static atomic_t		dyn_bus_id = ATOMIC_INIT((1<<15) - 1);
	struct device		*dev = tdm->dev.parent;
	struct tdm_board_devices	*bi;
	int			status = -ENODEV;
	int			dynamic = 0;

	if (!dev) {
		dev_err(dev, "parent device not exist\n");
		return -ENODEV;
	}

	/* convention:  dynamically assigned bus IDs count down from the max */
	if (tdm->bus_num < 0) {
		tdm->bus_num = atomic_dec_return(&dyn_bus_id);
		dynamic = 1;
	}

	/* register the device, then userspace will see it.
	 * registration fails if the bus ID is in use.
	 */
	dev_set_name(&tdm->dev, "tdm%u", tdm->bus_num);
	status = device_add(&tdm->dev);
	if (status < 0) {
		dev_err(dev, "cannot added controller device, %d\n", status);
		goto done;
	}
	dev_dbg(dev, "registered controller %s%s\n", dev_name(&tdm->dev),
	        dynamic ? " (dynamic)" : "");

	mutex_lock(&board_lock);
	list_add_tail(&tdm->list, &tdm_controller_list);
	list_for_each_entry(bi, &tdm_devices_list, list) // Run all devices connected to this TDM controller
	tdm_match_controller_to_boardinfo(tdm, &bi->bi);
	mutex_unlock(&board_lock);

	status = 0;

done:
	return status;
}
EXPORT_SYMBOL_GPL(tdm_controller_register);


static int __unregister(struct device *dev, void *null)
{
	device_unregister(dev);
	return 0;
}


/**
 * tdm_controller_unregister - unregister TDM controller
 * @tdm: the controller being unregistered
 * Context: can sleep
 *
 * This call is used only by TDM controller drivers, which are the
 * only ones directly touching chip registers.
 *
 * This must be called from context that can sleep.
 */
void tdm_controller_unregister(struct tdm_controller *tdm)
{
	int dummy;

	mutex_lock(&board_lock);
	list_del(&tdm->list);
	mutex_unlock(&board_lock);

	dummy = device_for_each_child(&tdm->dev, NULL, __unregister);
	device_unregister(&tdm->dev);
}
EXPORT_SYMBOL_GPL(tdm_controller_unregister);


/**
 * tdm_new_device - instantiate one new TDM device
 * @tdm: TDM Controller to which device is connected
 * @chip: Describes the TDM device
 * Context: can sleep
 *
 * Returns the new device, or NULL.
 */
struct tdm_device *tdm_new_device(struct tdm_controller *tdm,
                                  struct tdm_board_info *chip) {
	struct tdm_device 	*tdm_dev;
	int			status;

	printk("tdm_new_device 1\n");

	if (!tdm_controller_get(tdm))
		return NULL;

	printk("tdm_new_device 2\n");

	tdm_dev = kzalloc(sizeof *tdm_dev, GFP_KERNEL);
	if (!tdm_dev) {
		dev_err(tdm->dev.parent, "cannot alloc for TDM device\n");
		tdm_controller_put(tdm);
		return NULL;
	}

	printk("tdm_new_device 3\n");

	tdm_dev->controller = tdm;
	tdm_dev->dev.parent = tdm->dev.parent;
	tdm_dev->dev.bus = &tdm_bus_type;
	tdm_dev->dev.release = tdmdev_release;
	device_initialize(&tdm_dev->dev);

	WARN_ON(strlen(chip->modalias) >= sizeof(tdm_dev->modalias));

	tdm_dev->tdm_channel_num = chip->tdm_channel_num;
	tdm_dev->mode_wideband = chip->mode_wideband;
	tdm_dev->buffer_sample_count = chip->buffer_sample_count;
	strlcpy(tdm_dev->modalias, chip->modalias, sizeof(tdm_dev->modalias));

	status = tdm_add_device(tdm_dev);
	if (status < 0) {
		put_device(&tdm_dev->dev);
		return NULL;
	}
	printk("tdm_new_device 4 %s\n", dev_name(&tdm_dev->dev));

	return tdm_dev;
}
EXPORT_SYMBOL_GPL(tdm_new_device);



/**
 * tdm_add_device - Add tdm_device
 * @tdm_dev: TDM device to register
 *
 * Returns 0 on success; negative errno on failure
 */
int tdm_add_device(struct tdm_device *tdm_dev)
{
	static DEFINE_MUTEX(tdm_add_lock);
	struct tdm_controller *tdm = tdm_dev->controller;
	struct tdm_controller_hw_settings *hw = tdm->settings;
	struct device *dev = tdm->dev.parent;
	struct device *d;
	struct tdm_voice_channel *ch;
	int status;
	u8 count_tdm_channels;


	if (tdm_dev->mode_wideband) {
		dev_err(dev, "mode_wideband is not supported by this driver\n");
		status = -EINVAL;
		goto done;
	}

	count_tdm_channels = hw->count_time_slots / hw->channel_size;

	if (tdm_dev->tdm_channel_num >= count_tdm_channels) {
		dev_err(dev, "Incorrect requested TDM channel.\n"
		        "Requested %d TDM channel, %d TDM channels available.\n",
		        tdm_dev->tdm_channel_num, count_tdm_channels);

		status = -EINVAL;
		goto done;
	}

	if (tdm_dev->mode_wideband &&
	    (tdm_dev->tdm_channel_num > count_tdm_channels / 2)) {
		dev_err(dev, "Incorrect requested TDM channel in wideband mode.\n"
		        "Requested %d TDM channel, %d TDM channels available\n"
		        "in wideband mode\n", tdm_dev->tdm_channel_num, count_tdm_channels / 2);

		status = -EINVAL;
		goto done;
	}


	/* Set the bus ID string */
	dev_set_name(&tdm_dev->dev, "%s.%u", dev_name(&tdm_dev->controller->dev),
	             tdm_dev->tdm_channel_num);

	/* We need to make sure there's no other device with this
	 * chipselect **BEFORE** we call setup(), else we'll trash
	 * its configuration.  Lock against concurrent add() calls.
	 */
	mutex_lock(&tdm_add_lock);

	d = bus_find_device_by_name(&tdm_bus_type, NULL, dev_name(&tdm_dev->dev));
	if (d != NULL) {
		dev_err(dev, "TDM channel %d already in use\n",
		        tdm_dev->tdm_channel_num);
		put_device(d);
		status = -EBUSY;
		goto done;
	}

	ch = request_voice_channel(tdm_dev);
	if (ch == NULL) {
		dev_err(dev, "Can't request TDM voice channel. All voice channels is busy\n");
		status = -EBUSY;
		goto done;
	}

	// Configuring voice channel
	ch->mode_wideband = tdm_dev->mode_wideband;
	ch->tdm_channel = tdm_dev->tdm_channel_num;

	// Run setup voice channel
	status = tdm_dev->controller->setup_voice_channel(ch);
	if (status < 0) {
		dev_err(dev, "can't setup voice channel, status %d\n", status);
		goto done;
	}

	/* Device may be bound to an active driver when this returns */
	status = device_add(&tdm_dev->dev);
	if (status < 0)
		dev_err(dev, "can't add %s, status %d\n",
		        dev_name(&tdm_dev->dev), status);
	else
		dev_dbg(dev, "registered child %s\n", dev_name(&tdm_dev->dev));

done:
	mutex_unlock(&tdm_add_lock);
	return status;
}
EXPORT_SYMBOL_GPL(tdm_add_device);



/**
 * Receive audio-data for tdm device.
 * @param tdm_dev - tdm device registered on TDM bus
 * @param data - pointer to receive block data.
 * 					Allocated data size must be equal value
 * 					returned by get_tdm_voice_block_size();
 * @return 0 - success
 */
int tdm_recv(struct tdm_device *tdm_dev, u8 *data)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	if (tdm_dev->ch == NULL)
		return -ENODEV;

	return tdm->recv(tdm_dev->ch, data);
}
EXPORT_SYMBOL_GPL(tdm_recv);


/**
 * Transmit audio-data from tdm device.
 * @param tdm_dev - tdm device registered on TDM bus
 * @param data - pointer to transmit block data.
 * 					Transmit data size must be equal value
 * 					returned by get_tdm_voice_block_size();
 * @return 0 - success
 */
int tdm_send(struct tdm_device *tdm_dev, u8 *data)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	if (tdm_dev->ch == NULL)
		return -ENODEV;

	return tdm->send(tdm_dev->ch, data);
}
EXPORT_SYMBOL_GPL(tdm_send);


/**
 * Enable audio transport
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - success
 */
int tdm_run_audio(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	if (!tdm_dev->ch) {
		dev_err(&tdm_dev->dev, "Can't run audio because not allocated tdm voice channel\n");
		return -ENODEV;
	}

	return tdm->run_audio(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_run_audio);


/**
 * Disable audio transport
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - success
 */
int tdm_stop_audio(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	if (!tdm_dev->ch) {
		dev_err(&tdm_dev->dev, "Can't stop audio because not allocated tdm voice channel\n");
		return -ENODEV;
	}

	return tdm->stop_audio(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_stop_audio);



/**
 * Check rx audio buffer for exist new data
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought data, 1 - data exist
 */
int tdm_poll_rx(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	return tdm->poll_rx(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_poll_rx);


/**
 * Check tx audio buffer for free space
 * @param tdm_dev - tdm device registered on TDM bus
 * @return 0 - not enought free space, 1 - exist free space
 */
int tdm_poll_tx(struct tdm_device *tdm_dev)
{
	struct tdm_controller *tdm = tdm_dev->controller;

	return tdm->poll_tx(tdm_dev);
}
EXPORT_SYMBOL_GPL(tdm_poll_tx);


/**
 * Get voice block size for transmit or receive operations
 * @param tdm_dev - tdm device registered on TDM bus
 * @return voice block size, or error if returned value less 0
 */
int get_tdm_voice_block_size(struct tdm_device *tdm_dev)
{
	struct tdm_voice_channel *ch;

	ch = get_requested_tdm_voice_channel(tdm_dev);
	if (ch == NULL)
		return -ENODEV;

	return ch->buffer_len;
}
EXPORT_SYMBOL_GPL(get_tdm_voice_block_size);



/**
 * tdm_register_board_info - register TDM devices for a given board
 * @info: array of chip descriptors
 * @n: how many descriptors are provided
 * Context: can sleep
 */
int __init
tdm_register_board_info(struct tdm_board_info const *info, unsigned n)
{
	struct tdm_board_devices *bi;
	int i;

	bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	for (i = 0; i < n; i++, bi++, info++) {
		struct tdm_controller *tdm;

		memcpy(&bi->bi, info, sizeof(*info));
		mutex_lock(&board_lock);

		list_add_tail(&bi->list, &tdm_devices_list);
		list_for_each_entry(tdm, &tdm_controller_list, list)
		tdm_match_controller_to_boardinfo(tdm, &bi->bi);

		mutex_unlock(&board_lock);
	}

	return 0;
}



static int __init tdm_core_init(void)
{
	int	status;

	status = bus_register(&tdm_bus_type);
	if (status < 0)
		goto err0;

	status = class_register(&tdm_class);
	if (status < 0)
		goto err1;
	return 0;

err1:
	bus_unregister(&tdm_bus_type);
err0:
	return status;
}

postcore_initcall(tdm_core_init);

