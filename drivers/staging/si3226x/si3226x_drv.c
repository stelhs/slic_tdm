/*
 * si3226x_drv.c
 *
 *  Created on: 01.03.2012
 *      Author: Michail Kurochkin
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slic_si3226x.h>
#include <linux/tdm/tdm.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include "si3226x_drv.h"
#include "si3226x_hw.h"
#include "fifo.h"

#include <asm/uaccess.h>


/**
 * spi_dev_list necessary for
 * accumulate corresponding devices while
 * this registering. SLIC requirement one spi_device,
 * two tdm_device, and platform_device
 */
static LIST_HEAD(slic_dev_list);
static DEFINE_MUTEX(dev_list_lock);


/*
 * list all existing character devices
 */
static LIST_HEAD(slic_chr_dev_list);
static DEFINE_MUTEX(slic_chr_dev_lock);

/*
 * class for character devices
 */
static struct class *slic_class;


/**
 * slic controls over ioctl
 * @param slic - slic descriptor
 * @param cmd - command
 * @param arg - argument
 * @return 0 - ok
 */
int slic_control(struct si3226x_slic *slic, unsigned int cmd, unsigned long arg)
{
	int rc;
	struct si3226x_line *line = slic->lines;
	int i;

	switch(cmd)
	{
		case SI3226X_SET_COMPANDING_MODE:
			if(	arg != SI_ALAW && arg !=	SI_MLAW)
				return -EINVAL;

			for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
			{
				if (line->state == SI_LINE_DISABLE)
					continue;

				line->companding_mode = arg;

				rc = slic_setup_audio(line);
				if (rc)
					return rc;
			}
			break;

		case SI3226X_SET_CALLERID_MODE:
			if(	arg != SI_FSK_BELLCORE && arg !=	SI_FSK_ETSI
					&& arg != SI_CALLERID_DTMF)
				return -EINVAL;

			for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
			{
				if (line->state == SI_LINE_DISABLE)
					continue;

				line->callerid_mode = arg;
			}
			break;
	}

	return 0;
}

/**
 * slic line controls over ioctl
 * @param line - line descriptor
 * @param cmd - command
 * @param arg - argument
 * @return 0 - ok
 */
int line_control(struct si3226x_line *line, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	u8 data;
	struct si3226x_caller_id caller_id;
	u8 callerid_buffer[CALLERID_BUF_SIZE];

	switch(cmd)
	{
		case SI3226X_SET_CALLERID_MODE:
			if(	arg != SI_FSK_BELLCORE && arg !=	SI_FSK_ETSI
					&& arg != SI_CALLERID_DTMF && arg != SI_CALLERID_NONE)
				return -EINVAL;

			line->callerid_mode = arg;
			break;

		case SI3226X_SET_ECHO_CANCELATION:
			if(arg)
				rc = slic_enable_echo(line);
			else
				rc = slic_disable_echo(line);
			break;

		case SI3226X_SET_LINE_STATE:
			rc = slic_set_line_state(line, arg);
			break;

		case SI3226X_CALL:
			copy_from_user(&caller_id, (__u8 __user *)arg, sizeof caller_id);

			if(caller_id.size > CALLERID_BUF_SIZE)
				caller_id.size = CALLERID_BUF_SIZE;

			copy_from_user(callerid_buffer, (__u8 __user *)caller_id.data, caller_id.size);
			rc = slic_line_call(line, callerid_buffer, caller_id.size);
			break;

		case SI3226X_SEND_DTMF:
			rc = slic_send_dtmf_digit(line, arg);
			break;

		case SI3226X_GET_HOOK_STATE:
			__put_user(line->hook_state, (__u8 __user *)arg);
			break;

		case SI3226X_GET_DTMF_DIGIT:
			rc = fifo_pop(&line->dtmf, &data);
			if (rc)
				return -ENODATA;

			__put_user(data, (__u8 __user *)arg);
			break;

		case SI3226X_GET_AUDIO_BLOCK_SIZE:
			__put_user(line->audio_buffer_size, (__u8 __user *)arg);
			break;

		case SI3226X_SET_COMPANDING_MODE:
			if(	arg != SI_ALAW && arg != SI_MLAW)
				return -EINVAL;

			line->companding_mode = arg;

			rc = slic_setup_audio(line);
			if (rc)
				return rc;

			break;

		case SI3226X_ENABLE_AUDIO:
			if (line->tdm_dev == NULL)
				return -ENODEV;

			printk("SI3226X_ENABLE_AUDIO %d \n", line->ch);

			/* TODO: workaround to fix sound
			 * distortion after asterisk restart */
			if (!line->audio_start_flag)
			{
				rc = tdm_run_audio(line->tdm_dev);
				if (!rc)
					line->audio_start_flag = 1;
			}

			break;

		case SI3226X_DISABLE_AUDIO:
			if (line->tdm_dev == NULL)
				return -ENODEV;

			printk("SI3226X_DISABLE_AUDIO %d \n", line->ch);
			rc = tdm_stop_audio(line->tdm_dev);
			break;
	}

	return rc;
}

/*
 * method "open" for character device
 */
static int slic_chr_open(struct inode *inode, struct file *file)
{
	struct slic_chr_dev *chr_dev;
	struct si3226x_slic *slic;
	struct si3226x_line *line;
	struct tdm_device *tdm_dev;
	struct tdm_voice_channel *ch;
	int status = -ENXIO;

	mutex_lock(&slic_chr_dev_lock);

	list_for_each_entry(chr_dev, &slic_chr_dev_list, list)
	{
		switch (chr_dev->type) {
		case SLIC_CHR_DEV:
			slic = dev_get_drvdata(chr_dev->dev);

			if (slic->devt != inode->i_rdev)
				continue;;

			if (slic->file_opened)
			{
				status = -EBUSY;
				goto out;
			}

			slic->file_opened = 1;
			status = 0;
			break;

		case LINE_CHR_DEV:
			line = dev_get_drvdata(chr_dev->dev);
			tdm_dev = line->tdm_dev;

			if (line->devt != inode->i_rdev)
				continue;

			if (line->file_opened)
			{
				status = -EBUSY;
				goto out;
			}

			line->audio_buffer_size = tdm_get_voice_block_size(tdm_dev);

			line->rx_buffer = kzalloc(line->audio_buffer_size, GFP_KERNEL);
			if (!line->rx_buffer)
			{
				status = -ENOMEM;
				goto out;
			}

			line->tx_buffer = kzalloc(line->audio_buffer_size, GFP_KERNEL);
			if (!line->tx_buffer)
			{
				status = -ENOMEM;
				goto out;
			}

			/*  store pointer to transmit wait_queue_head_t
			 *  for use in poll() */
			ch = tdm_dev->ch;
			line->tx_wait_queue = &ch->tx_queue;
			line->rx_wait_queue = &ch->rx_queue;
			line->file_opened = 1;
			status = 0;

			break;
		}

		file->private_data = chr_dev;
		status = 0;
		goto out;
	}

out:
	mutex_unlock(&slic_chr_dev_lock);
	return status;
}


/*
 * method "release" for character device
 */
static int slic_chr_release(struct inode *inode, struct file *file)
{
	struct slic_chr_dev *chr_dev = file->private_data;
	struct si3226x_slic *slic;
	struct si3226x_line *line;
	struct tdm_device *tdm_dev;
	int status = -ENXIO;

	mutex_lock(&slic_chr_dev_lock);

	switch (chr_dev->type) {
	case SLIC_CHR_DEV:
		slic = dev_get_drvdata(chr_dev->dev);

		if(!slic->file_opened)
		{
			status = -ENODEV;
			goto out;
		}

		slic->file_opened = 0;
		status = 0;
		break;

	case LINE_CHR_DEV:
		line = dev_get_drvdata(chr_dev->dev);
		tdm_dev = line->tdm_dev;

		if(!line->file_opened)
		{
			status = -ENODEV;
			goto out;
		}

		tdm_stop_audio(tdm_dev);
		kfree(line->rx_buffer);
		kfree(line->tx_buffer);
		line->file_opened = 0;
		status = 0;
	}

	file->private_data = NULL;

out:
	mutex_unlock(&slic_chr_dev_lock);
    return status;
}

/*
 * method "ioctl" for character device
 */
static long
slic_chr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct slic_chr_dev *chr_dev = file->private_data;
	struct si3226x_slic *slic;
	struct si3226x_line *line;
	int status = -ENXIO;

	if (_IOC_TYPE(cmd) != SI3226X_IOC_MAGIC)
		return -ENOTTY;

	mutex_lock(&slic_chr_dev_lock);

	if (!file->private_data)
	{
		status = -EPERM;
		goto out;
	}

	switch(chr_dev->type) {
	case SLIC_CHR_DEV:
		slic = dev_get_drvdata(chr_dev->dev);
		if (!slic->file_opened)
		{
			status = -ENODEV;
			goto out;
		}

		status = slic_control(slic, cmd, arg);
		break;

	case LINE_CHR_DEV:
		line = dev_get_drvdata(chr_dev->dev);
		if (!line->file_opened)
		{
			status = -ENODEV;
			goto out;
		}

		status = line_control(line, cmd, arg);
		break;
	}

out:
	mutex_unlock(&slic_chr_dev_lock);
    return status;
}


/*
 * method "read" for character device
 */
static ssize_t
slic_chr_read(struct file *file, char *buff, size_t count, loff_t *offp)
{
	struct slic_chr_dev *chr_dev = file->private_data;
	struct si3226x_line *line;
	struct tdm_device *tdm_dev;
	int rc;

	if (!file->private_data)
		return -EPERM;

	if(chr_dev->type != LINE_CHR_DEV)
		return -EPERM;

	mutex_lock(&slic_chr_dev_lock);

	line = dev_get_drvdata(chr_dev->dev);
	tdm_dev = line->tdm_dev;

	if (count != line->audio_buffer_size)
	{
		rc = -ENODATA;
		goto out;
	}

	rc = tdm_recv(tdm_dev, line->rx_buffer);
	if (rc)
	{
		rc = -EFAULT;
		goto out;
	}

	rc = copy_to_user(buff, line->rx_buffer, count);
	if (rc)
	{
		rc = -EFAULT;
		goto out;
	}

	rc = count;
out:
	mutex_unlock(&slic_chr_dev_lock);
    return rc;
}


/*
 * method "write" for character device
 */
static ssize_t
slic_chr_write(struct file *file, const char *buf, size_t count, loff_t *offp)
{
	struct slic_chr_dev *chr_dev = file->private_data;
	struct si3226x_line *line;
	struct tdm_device *tdm_dev;
	int rc;

	if (!file->private_data)
		return -EPERM;

	if (chr_dev->type != LINE_CHR_DEV)
		return -EPERM;

	mutex_lock(&slic_chr_dev_lock);

	line = dev_get_drvdata(chr_dev->dev);
	tdm_dev = line->tdm_dev;

	if (count != line->audio_buffer_size)
	{
		rc = -EMSGSIZE;
		goto out;
	}

	rc = copy_from_user(line->tx_buffer, buf, count);
	if (rc)
	{
		rc = -EFAULT;
		goto out;
	}

	rc = tdm_send(tdm_dev, line->tx_buffer);
	if (rc)
	{
		rc = -EFAULT;
		goto out;
	}

	rc = line->audio_buffer_size;
out:
	mutex_unlock(&slic_chr_dev_lock);
    return rc;
}


/*
 * method "poll" for character device
 */
static unsigned int slic_chr_poll(struct file *file, poll_table *wait)
{
	struct slic_chr_dev *chr_dev = file->private_data;
	struct si3226x_line *line;
	int mask = 0;

	if (!file->private_data)
		return -EPERM;

	if (chr_dev->type != LINE_CHR_DEV)
		return -EPERM;

	mutex_lock(&slic_chr_dev_lock);

	line = dev_get_drvdata(chr_dev->dev);

	poll_wait(file, line->rx_wait_queue,  wait);
	poll_wait(file, line->tx_wait_queue,  wait);

	if (tdm_poll_rx(line->tdm_dev))
		mask |= POLLIN;

	if (tdm_poll_tx(line->tdm_dev))
		mask |= POLLOUT;

	mutex_unlock(&slic_chr_dev_lock);

   return mask;
}


/*
 * file operations for slic character devices
 */
static struct file_operations slic_cnt_ops =
{
    .unlocked_ioctl = slic_chr_ioctl,
    .open = slic_chr_open,
	.read = slic_chr_read,
	.write = slic_chr_write,
	.poll = slic_chr_poll,
    .release = slic_chr_release,
	.llseek = no_llseek,
    .owner = THIS_MODULE,
};


/**
 * add to list slic character device.
 * @param dev - character device
 * @param type - type of character device
 * @return 0 - ok
 */
static int add_slic_chr_dev(struct device *dev, enum chr_dev_type type)
{
	struct slic_chr_dev *chr_dev;

	chr_dev = kzalloc(sizeof(*chr_dev), GFP_KERNEL);
	if (!chr_dev)
		return -ENOMEM;

	chr_dev->dev = dev;
	chr_dev->type = type;
	list_add(&chr_dev->list, &slic_chr_dev_list);

	return 0;
}


/**
 * delete slic character device form list
 * @param dev - character device
 * @return 0 - ok
 */
static int del_slic_chr_dev(struct device *dev)
{
	struct slic_chr_dev *chr_dev, *chr_dev_tmp;

	list_for_each_entry_safe(chr_dev, chr_dev_tmp, &slic_chr_dev_list, list)
		if (chr_dev->dev == dev)
		{
			list_del(&chr_dev->list);
			kfree(chr_dev);
			return 0;
		}

	return -ENODEV;
}


/**
 * Init slic driver
 * @return 0 - ok
 */
static int init_slic_drv(struct si3226x_slic *slic)
{
	struct platform_device *pdev = slic->pdev;
	struct si3226x_platform_data *plat = pdev->dev.platform_data;
	struct si3226x_line *line = slic->lines;
	struct device *chr_dev;
	int rc;
	int i;

	dev_info(&pdev->dev, "run Initialization slic driver\n");

	/*  set default companding_mode for all lines */
	for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
		line->companding_mode = plat->companding_mode;

	/*  request reset GPIO */
	rc = gpio_request(slic->reset_gpio, DRIVER_NAME "_reset");
	if (rc < 0)
	{
		dev_err(&pdev->dev, "failed to request " DRIVER_NAME "_reset\n");
		goto out0;
	}
	gpio_direction_output(slic->reset_gpio, 1);

	/*  request interrupt GPIO */
	rc = gpio_request(slic->int_gpio, DRIVER_NAME "_irq");
	if (rc < 0)
	{
		dev_err(&pdev->dev, "failed to request " DRIVER_NAME "_irq\n");
		goto out1;
	}
	gpio_direction_input(slic->int_gpio);
	dev_info(&pdev->dev, "GPIO requested\n");

	slic->irq = gpio_to_irq(slic->int_gpio);

	INIT_WORK(&slic->irq_work, slic_irq_callback);

#ifdef CONFIG_SI3226X_POLLING
	INIT_DELAYED_WORK(&slic->delayed_work, slic_delayed_work);
#endif

	/*  register slic character device */
	rc = register_chrdev(SI3226X_MAJOR, DRIVER_NAME, &slic_cnt_ops);
	if (rc < 0)
	{
		dev_err(&pdev->dev, "can not register character device\n");
		goto out2;
	}

	/*  added class device and create /dev/si3226x_cnt.X */
	slic->devt = MKDEV(SI3226X_MAJOR, pdev->id);
	chr_dev = device_create(slic_class, &pdev->dev, slic->devt,
			    slic, DRIVER_NAME "_cnt.%d", pdev->id);
	if (IS_ERR(chr_dev))
	{
		dev_err(&pdev->dev, "can not added class device\n");
		goto out3;
	}

	/* added character device into device list
	    for use in file operations
	*/
	rc = add_slic_chr_dev(chr_dev, SLIC_CHR_DEV);
	if (rc)
	{
		dev_err(&pdev->dev, "can not added character device\n");
		goto out4;
	}

	line = slic->lines;
	for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
	{
		printk("line = %d\n", line->ch);

		if (plat->fxs_tdm_ch[i] < 0)
		{
			line->state = SI_LINE_DISABLE;
			continue;
		}

		INIT_WORK(&line->line_call_work, do_line_call);

		line->state = SI_LINE_SILENCE;

		/*  added class device and create /dev/si3226x_fxsX.X */
		line->devt = MKDEV(SI3226X_MAJOR, pdev->id + 1 + i);
		chr_dev = device_create(slic_class, &pdev->dev, line->devt,
			line, DRIVER_NAME "_fxs%d.%d", i, pdev->id);
		if (IS_ERR(chr_dev))
		{
			dev_err(&pdev->dev, "can not added class device\n");
			goto out4;
		}

		/*  added created character device into device list
		    for use in file operations
		*/
		rc = add_slic_chr_dev(chr_dev, LINE_CHR_DEV);
		if (rc)
		{
			dev_err(&pdev->dev, "can't added character device\n");
			goto out4;
		}
	}

	rc = init_slic(slic);
	if (rc)
	{
		dev_err(&pdev->dev, "slic initialization fail\n");
		goto out4;
	}

#ifdef CONFIG_SI3226X_POLLING
	dev_info(&pdev->dev, "schedule delayed work\n");
	schedule_delayed_work(&slic->delayed_work, MSEC(50));
#else
	/*  set interrupt callback slic_irq */
	dev_info(&pdev->dev, "request irq\n");
	rc = request_irq(slic->irq, slic_irq, IRQF_TRIGGER_FALLING | IRQF_DISABLED,
			  dev_name(&slic->pdev->dev), slic);
	if (rc)
	{
		dev_err(&pdev->dev, "can not request IRQ\n");
		rc = -EINVAL;
		goto out5;
	}
#endif
	dev_info(&pdev->dev, "success initialization slic driver\n");

	return 0;

out5:
	free_irq(slic->irq, slic);
	deinit_slic(slic);

out4:
	{
		/*  remove all character devices */
		struct slic_chr_dev *chr_dev, *chr_dev_tmp;
		list_for_each_entry_safe(chr_dev, chr_dev_tmp, &slic_chr_dev_list, list)
		{
			device_del(chr_dev->dev);
			del_slic_chr_dev(chr_dev->dev);
		}
	}

out3:
	unregister_chrdev(SI3226X_MAJOR, DRIVER_NAME);

out2:
	gpio_free(slic->int_gpio);

out1:
	gpio_free(slic->reset_gpio);

out0:
	return rc;
}



/**
 * DeInit slic driver
 */
void release_slic_drv(struct si3226x_slic *slic)
{
	struct platform_device *pdev = slic->pdev;
	struct slic_chr_dev *chr_dev, *chr_dev_tmp;
	int i;

	dev_dbg(&pdev->dev, "release slic\n");

	deinit_slic(slic);

	free_irq(slic->irq, slic);

	/*  delete slic character device */
	list_for_each_entry_safe(chr_dev, chr_dev_tmp, &slic_chr_dev_list, list)
	{
		struct si3226x_line *line;
		struct si3226x_slic *s;
		struct si3226x_line *l;

		switch (chr_dev->type) {
		case SLIC_CHR_DEV:
			s = dev_get_drvdata(chr_dev->dev);
			if (s == slic)
			{
			    device_del(chr_dev->dev);
				list_del(&chr_dev->list);
				kfree(chr_dev);
			}
			break;

		case LINE_CHR_DEV:
			l = dev_get_drvdata(chr_dev->dev);
			line = slic->lines;
			for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
				if (l == line)
				{
					device_del(chr_dev->dev);
					list_del(&chr_dev->list);
					kfree(chr_dev);
				}
			break;
		}
	}

	unregister_chrdev(SI3226X_MAJOR, DRIVER_NAME);
	gpio_free(slic->int_gpio);
	gpio_free(slic->reset_gpio);
}



/**
 * Add device to slic devices list
 * @param dev - device (spi, tdm or platform)
 * @return 0 - ok
 */
static int add_to_slic_devices_list(struct device *dev, enum slic_dev_type type)
{
	struct slic_dev_list *slic_dev_item;

	slic_dev_item = kzalloc(sizeof *slic_dev_item, GFP_KERNEL);
	if (!slic_dev_item)
		return -ENOMEM;

	slic_dev_item->dev = dev;
	slic_dev_item->type = type;
	list_add_tail(&slic_dev_item->list, &slic_dev_list);

	return 0;
}


/**
 * remove device from slic devices list
 * @param dev - device (spi, tdm or platform)
 * @return 0 - ok
 */
static int remove_from_slic_devices_list(struct device *dev)
{
	struct slic_dev_list *slic_dev_item, *slic_dev_item_tmp;

	list_for_each_entry_safe(slic_dev_item, slic_dev_item_tmp, &slic_dev_list, list)
		if(slic_dev_item->dev == dev)
		{
			list_del(&slic_dev_item->list);
			kzfree(slic_dev_item);
			return 0;
		}

	return -ENODEV;
}


/**
 * allocate memory and configure slic descriptor.
 * @param pdev - platform device
 * @return allocated slic controller descriptor
 */
struct si3226x_slic *alloc_slic(struct platform_device *pdev)
{
	int i;
	struct si3226x_slic 	*slic;
	struct si3226x_line *line;

	slic = kzalloc(sizeof *slic, GFP_KERNEL);
	if (!slic)
		return NULL;

	memset(slic, 0, sizeof *slic);
	platform_set_drvdata(pdev, slic);

	slic->pdev = pdev;

	line = slic->lines;
	for (i = 0; i < SI3226X_MAX_CHANNELS; i++, line++)
	{
		line->ch = i;
		line->audio_start_flag = 0;
	}

	return slic;
}


/**
 * free memory allocated alloc_slic()
 * @param slic - slic controller descriptor
 * @return 0 - ok
 */
int free_slic(struct si3226x_slic *slic)
{
	if (slic == NULL)
		return -EINVAL;

	platform_set_drvdata(slic->pdev, NULL);
	kfree(slic);

	return 0;
}

/**
 * match all devices necessery for slic
 * @param slic - pointer to pointer on slic private data
 * @return 0 - ok, 1 - incomplete device list, 0< - any errors
 */
int slic_match_bus_devices(struct si3226x_slic **slic)
{
	struct slic_dev_list *plat_list_item;
	struct slic_dev_list *spi_list_item;
	struct slic_dev_list *tdm_list_item;
	int i;


	/* foreach all slic platform devices */
	list_for_each_entry(plat_list_item, &slic_dev_list, list)
	{
		struct platform_device *pdev =
				to_platform_device(plat_list_item->dev);
		struct si3226x_platform_data *plat = pdev->dev.platform_data;
		struct tdm_device *found_tdm_devices[SI3226X_MAX_CHANNELS];
		struct spi_device *found_spi_dev = NULL;
		struct si3226x_line *slic_line;
		int count_found_tdm;
		int count_fxs = 0;

		if (plat_list_item->type != PLAT_DEVICE)
			continue;

		/*  increment count enabled fxs ports */
		for (i = 0; i < SI3226X_MAX_CHANNELS; i++)
			if(plat->fxs_tdm_ch[i] >= 0)
				count_fxs++;

		if (!count_fxs)
		{
			dev_err(&pdev->dev, "Init slic fail. "
					"Disabled all FXS ports\n");
			return -EINVAL;
		}

		dev_dbg(&pdev->dev, "count_fxs = %d\n", count_fxs);

		/*  foreach all spi devices and match with current slic */
		list_for_each_entry(spi_list_item, &slic_dev_list, list)
		{
			struct spi_device *spi_dev =
					to_spi_device(spi_list_item->dev);

			if (spi_list_item->type != SPI_DEVICE)
				continue;

			if(plat->spi_chip_select != spi_dev->chip_select)
				continue;

			found_spi_dev = to_spi_device(spi_list_item->dev);
		}

		if (found_spi_dev == NULL)
		{
			dev_dbg(&pdev->dev, "not found spi_dev\n");
			return 1;
		}

		dev_dbg(&pdev->dev, "found spi_dev = %s\n",
					dev_name(&found_spi_dev->dev));

		/*  foreach all registered tdm devices for current slic */
		/*  and match tdm channel with fxs port tdm channel */
		memset(found_tdm_devices, 0, sizeof *found_tdm_devices);
		count_found_tdm = 0;
		for (i = 0; i < SI3226X_MAX_CHANNELS; i++)
		{
			if(plat->fxs_tdm_ch[i] < 0)
				continue;

			list_for_each_entry(tdm_list_item, &slic_dev_list, list)
			{
				struct tdm_device *tdm_dev =
						to_tdm_device(tdm_list_item->dev);

				if (tdm_list_item->type != TDM_DEVICE)
					continue;

				/*  Match device tdm channel with corresponding fxs port */
				if (plat->fxs_tdm_ch[i] == tdm_dev->tdm_channel_num)
				{
					found_tdm_devices[i] = to_tdm_device(tdm_list_item->dev);
					count_found_tdm++;
				}
			}
		}

		/*  if found needed tdm devices for current slic */
		if (count_fxs != count_found_tdm)
		{
			dev_dbg(&pdev->dev, "tdm devices not found\n");
			return 1;
		}

		*slic = alloc_slic(pdev);
		if (!*slic)
		{
			dev_err(&pdev->dev, "can't alloc slic\n");
			return -ENOMEM;
		}

		(*slic)->spi_dev = found_spi_dev;
		(*slic)->reset_gpio = plat->reset_gpio;
		(*slic)->int_gpio = plat->int_gpio;


		slic_line = (*slic)->lines;
		for (i = 0; i < SI3226X_MAX_CHANNELS; i++, slic_line++)
		{
			slic_line->tdm_dev = found_tdm_devices[i];

			/*  Remove devices from list slic devices */
			remove_from_slic_devices_list(&found_tdm_devices[i]->dev);
		}

		/*  Remove devices from list slic devices */
		remove_from_slic_devices_list(&found_spi_dev->dev);
		remove_from_slic_devices_list(plat_list_item->dev);

		return 0;
	}

	return 1;
}



/*
 * probe spi device
 */
static int probe_spi_slic(struct spi_device *spi_dev)
{
	struct si3226x_slic *slic;
	int rc;

	rc = add_to_slic_devices_list(&spi_dev->dev, SPI_DEVICE);
	if (rc < 0)
	{
		dev_err(&spi_dev->dev, "can't added spi device "
				"to slic list devices: %d\n", rc);
		goto out;
	}

	rc = slic_match_bus_devices(&slic);
	if (rc < 0)
	{
		dev_err(&spi_dev->dev, "Error: %d\n", rc);
		goto out;
	}

	if (rc)
	{
		rc = 0;
		goto out;
	}

	rc = init_slic_drv(slic);

	if (rc)
		free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}


/*
 * probe tdm device
 */
static int probe_tdm_slic(struct tdm_device *tdm_dev)
{
	struct si3226x_slic *slic;
	int rc;

	mutex_lock(&dev_list_lock);

	rc = add_to_slic_devices_list(&tdm_dev->dev, TDM_DEVICE);
	if (rc < 0)
	{
		dev_err(&tdm_dev->dev, "can't added tdm device to "
				"slic list devices: %d\n", rc);
		goto out;
	}

	rc = slic_match_bus_devices(&slic);
	if (rc < 0)
	{
		dev_err(&tdm_dev->dev, "Error: %d\n", rc);
		goto out;
	}

	if (rc)
	{
		rc = 0;
		goto out;
	}

	rc = init_slic_drv(slic);

	if (rc)
		free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}


/*
 * probe slic platform device
 */
static int probe_slic(struct platform_device *pdev)
{
	struct si3226x_slic *slic;
	int rc;

	mutex_lock(&dev_list_lock);

	rc = add_to_slic_devices_list(&pdev->dev, PLAT_DEVICE);
	if (rc < 0)
	{
		dev_err(&pdev->dev, "can't added platform device to "
				"slic list devices: %d\n", rc);
		goto out;
	}

	rc = slic_match_bus_devices(&slic);
	if (rc < 0)
	{
		dev_err(&pdev->dev, "Error: %d\n", rc);
		goto out;
	}

	if (rc)
	{
		rc = 0;
		goto out;
	}

	rc = init_slic_drv(slic);
	if (rc)
		free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}



/*
 * called when remove spi device
 */
static int remove_spi_slic(struct spi_device *spi_dev)
{
	struct si3226x_slic *slic = NULL;
	struct slic_chr_dev *chr_dev;
	int rc = 0;

	mutex_lock(&dev_list_lock);

	/*  find slic device supported current spi device */
	list_for_each_entry(chr_dev, &slic_chr_dev_list, list)
	{
		struct si3226x_slic *p;
		p = dev_get_drvdata(chr_dev->dev);

		if (p->spi_dev == spi_dev)
		{
			slic = p;
			break;
		}
	}

	/*  If line device not found in character devices list */
	if (!slic)
	{
		remove_from_slic_devices_list(&spi_dev->dev);
		goto out;
	}

	release_slic_drv(slic);
	rc = free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}


/*
 * remove tdm device
 */
static int remove_tdm_slic(struct tdm_device *tdm_dev)
{
	struct si3226x_slic *slic = NULL;
	struct si3226x_line *line = NULL;
	struct slic_chr_dev *chr_dev;
	int rc = 0;

	mutex_lock(&dev_list_lock);

	/*  find slic device supported current tdm device */
	list_for_each_entry(chr_dev, &slic_chr_dev_list, list)
	{
		struct si3226x_line *p;
		p = dev_get_drvdata(chr_dev->dev);

		if (p->tdm_dev == tdm_dev)
		{
			line = p;
			break;
		}
	}

	/*  If line device not found in character devices list */
	if (!line)
	{
		remove_from_slic_devices_list(&tdm_dev->dev);
		goto out;
	}

	slic = to_si3226x_slic(line);

	release_slic_drv(slic);
	rc = free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}


/*
 * remove slic platform device
 */
static int remove_slic(struct platform_device *pdev)
{
	struct si3226x_slic *slic = NULL;
	struct slic_chr_dev *chr_dev;
	int rc = 0;

	mutex_lock(&dev_list_lock);

	/*  find slic device supported current tdm device */
	list_for_each_entry(chr_dev, &slic_chr_dev_list, list)
	{
		struct si3226x_slic *p;
		p = dev_get_drvdata(&pdev->dev);

		if (p->pdev == pdev)
		{
			slic = p;
			break;
		}
	}

	/*  If line device not found in character devices list */
	if (!slic)
	{
		remove_from_slic_devices_list(&pdev->dev);
		goto out;
	}

	release_slic_drv(slic);
	rc = free_slic(slic);

out:
	mutex_unlock(&dev_list_lock);
	return rc;
}


/*
 * spi interface
 */
static struct spi_driver spi_slic_driver =
{
    .driver =
    {
        .name = "si3226x",
        .bus    = &spi_bus_type,
        .owner    = THIS_MODULE,
    },
    .probe    = probe_spi_slic,
    .remove    = __devexit_p(remove_spi_slic),
};


/*
 * tdm Interface
 */
static struct tdm_driver tdm_slic_driver =
{
	.driver =
	{
		.name = "si3226x",
		.bus    = &tdm_bus_type,
		.owner    = THIS_MODULE,
	},
	.probe    = probe_tdm_slic,
	.remove    = __devexit_p(remove_tdm_slic),
};


/*
 * slic platform interface
 */
static struct platform_driver slic_driver =
{
	.driver =
	{
		.name = "si3226x",
		.bus    = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.probe	= probe_slic,
	.remove	= __devexit_p(remove_slic),
};


/*
 * init kernel module
 */
static int __init si3226x_mod_init(void)
{
	int rc;

	slic_class = class_create(THIS_MODULE, "slic");

	rc = spi_register_driver(&spi_slic_driver);
	if (rc)
	{
		pr_debug("failed to register SPI for SLIC, error: %d\n", rc);
		goto out0;
	}


	rc = tdm_register_driver(&tdm_slic_driver);
	if (rc)
	{
		pr_debug("failed to register TDM for SLIC, error: %d\n", rc);
		goto out1;
	}


	rc = platform_driver_register(&slic_driver);
	if (rc)
	{
		pr_debug("failed to register SLIC on platform, error: %d\n", rc);
		goto out2;
	}

	return 0;

out2:
	tdm_unregister_driver(&tdm_slic_driver);

out1:
	spi_unregister_driver(&spi_slic_driver);

out0:
 	return rc;
}


/*
 * deinit kernel module
 */
static void __exit si3226x_mod_exit(void)
{
	tdm_unregister_driver(&tdm_slic_driver);
	spi_unregister_driver(&spi_slic_driver);
	platform_driver_unregister(&slic_driver);

	class_destroy(slic_class);
}

module_init(si3226x_mod_init);
module_exit(si3226x_mod_exit);
MODULE_AUTHOR("Kurochkin Michail");
MODULE_LICENSE("GPL");
