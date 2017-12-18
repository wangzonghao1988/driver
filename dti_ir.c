/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/adc.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/kthread.h>
#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/pmu.h>



struct dti_ir_data {
	int irq;
	int fe;
	struct input_dev *input;
	struct wake_lock gwake_lock;
};
struct dti_ir_data gdata = {
	.fe = 1,
};

static irqreturn_t ir_isr(int irq, void *dev_id)
{
	//printk("%s(): %d.\n", __func__, irq);
#if 1
	wake_lock_timeout(&gdata.gwake_lock, HZ);
	input_report_key(gdata.input, KEY_WAKEUP, 1);
	input_sync(gdata.input);
	input_report_key(gdata.input, KEY_WAKEUP, 0);
	input_sync(gdata.input);
#endif
	return IRQ_HANDLED;
}

static int dti_ir_probe(struct platform_device *pdev)
{
	int error = 0;
	struct irda_info *mach_info = NULL;

	struct device *dev = &pdev->dev;
	printk("%s().\n", __func__);

	mach_info = pdev->dev.platform_data;
	if (mach_info)
		mach_info->iomux_init();

	gdata.input = input_allocate_device();
	gdata.input->name = pdev->name;
	input_set_capability(gdata.input, EV_KEY, KEY_WAKEUP);
	error = input_register_device(gdata.input);
	if (error) {
		printk("Unable to register input device error: %d\n", error);
		goto fail;
	}

	gdata.irq = gpio_to_irq(mach_info->intr_pin);
	printk("%s(): irq=%d.\n", __func__, gdata.irq);

	error = request_irq(gdata.irq, ir_isr, IRQ_TYPE_EDGE_RISING, NULL, dev);
	if (error) {
		printk("Unable to claim irq %d; error %d\n", gdata.irq, error);
		goto fail;
	}

	wake_lock_init(&gdata.gwake_lock, WAKE_LOCK_SUSPEND, "wakelock_ir");

fail:
	return error;
}

static int dti_ir_remove(struct platform_device *pdev)
{
	wake_lock_destroy(&gdata.gwake_lock);
	input_unregister_device(gdata.input);

	return 0;
}

#ifdef CONFIG_PM
static int dti_ir_suspend(struct device *dev)
{
	printk("%s().\n", __func__);
#if 1
	enable_irq_wake(gdata.irq);
#endif
	return 0;
}

static int dti_ir_resume(struct device *dev)
{
	printk("%s().\n", __func__);
#if 1
	disable_irq_wake(gdata.irq);
#endif
	return 0;
}
#else
#define dti_ir_suspend	NULL
#define dti_ir_resume	NULL
#endif

static struct platform_driver pd_dti_ir_driver = {
	.probe		= dti_ir_probe,
	.remove		= dti_ir_remove,
	.driver		= {
		.name	= "dti_ir",
		.owner	= THIS_MODULE,
	},
	.suspend	= dti_ir_suspend,
	.resume		= dti_ir_resume,
};

static ssize_t enable_show(struct device_driver *drv, char *buf)
{
	if (gdata.fe) {
		return sprintf(buf, "enable\n");
	} else {
		return sprintf(buf, "disable\n");
	}
}

static ssize_t enable_store(struct device_driver *drv, const char *buf, size_t count)
{
	unsigned int val;
	int error = 0;
	//printk("%s(): %s.\n", __func__, buf);

	error = kstrtouint(buf, 10, &val);
	if (val == 1) {
		gdata.fe = 1;
	} else {
		gdata.fe = 0;
	}

	return count;
}
static DRIVER_ATTR(enable, 0666, enable_show, enable_store);

static int __init m_ir_init(void)
{
	int ret;
	ret = platform_driver_register(&pd_dti_ir_driver);
	if ( ret != 0) {
		printk("Could not register dti ir driver\n");
		return -EINVAL;
	}
	ret = driver_create_file(&pd_dti_ir_driver.driver, &driver_attr_enable);
	return ret;
}

static void __exit m_ir_exit(void)
{
	platform_driver_unregister(&pd_dti_ir_driver);
}

module_init(m_ir_init);
module_exit(m_ir_exit);
