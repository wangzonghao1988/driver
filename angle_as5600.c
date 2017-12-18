/*
 * NJDATI AS5600 I2C driver
 *
 * Copyright (c) 2016  NJDTI Co., Ltd.
 * Henglei Hu <huhenglei@njdti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/version.h>

#define I2C_MASTER_CLK    (400 * 1000)
#define AS5600_NAME       "as5600"
#define AS5600_DRV_VER    "1.0"

static struct class *as5600_class = NULL;

struct as5600_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work as_work;
	int flag;
	int freq;
	int delay;
	int start_angle;
	int end_angle;
	int idx;
};

struct as5600_data g_data;

static int ap;
static int ag;

static DEFINE_MUTEX(as5600_mutex);

static int as5600_i2c_tx(struct i2c_client *client, void *txdata, size_t txlen)
{
	int ret;

	struct i2c_msg msg = {
		.addr		= client->addr,
		.flags		= 0,
		.len		= txlen,
		.buf		= txdata,
		.scl_rate	= I2C_MASTER_CLK,

	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	return (ret == 1) ? txlen : ret;
}

static int as5600_i2c_xfer(struct i2c_client *client,
			    void *txdata, size_t txlen,
			    void *rxdata, size_t rxlen)
{
	int error = 0;
	int ret;
	int array_sz = 1;

	struct i2c_msg msgs[] = {
		{
			.addr		= client->addr,
			.flags		= 0,
			.len		= txlen,
			.buf		= txdata,
			.scl_rate	= I2C_MASTER_CLK,
		},
		{
			.addr		= client->addr,
			.flags		= I2C_M_RD,
			.len		= rxlen,
			.buf		= rxdata,
			.scl_rate	= I2C_MASTER_CLK,
		},
	};

	ret = i2c_transfer(client->adapter, &msgs[0], 1);
	if (ret == array_sz) {
		mdelay(2);
		ret = i2c_transfer(client->adapter, &msgs[1], 1);
	}

	if (ret != array_sz) {
		error = ret < 0 ? ret : -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed: %d\n", __func__, error);
		return error;
	}

	return error;
}

static int read_rawangle(void)
{
	u8 tx_buf[1];
	u8 rx_buf[1];
	int angle;

	tx_buf[0] = 0x0C;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = rx_buf[0]&0xF;
	angle = angle<<8;

	tx_buf[0] = 0x0D;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = angle + rx_buf[0];

	return angle;
}

static int read_angle(void)
{
	u8 tx_buf[1];
	u8 rx_buf[1];
	int angle;

	tx_buf[0] = 0x0E;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = rx_buf[0]&0xF;
	angle = angle<<8;

	tx_buf[0] = 0x0F;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = angle + rx_buf[0];

	return angle;
}

static int read_zmco(void)
{
	u8 tx_buf[1];
	u8 rx_buf[1];

	tx_buf[0] = 0x00;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);

	return 0;
}

static int read_zpos(void)
{
	u8 tx_buf[1];
	u8 rx_buf[1];
	int angle;

	tx_buf[0] = 0x01;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = rx_buf[0]&0xF;
	angle = angle<<8;

	tx_buf[0] = 0x02;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = angle + rx_buf[0];

	return angle;
}

static int write_zpos(void)
{
	u8 tx_buf[2];

	tx_buf[0] = 0x02;
	tx_buf[1] = g_data.start_angle & 0xFF;
	as5600_i2c_tx(g_data.client, tx_buf, 2);

	tx_buf[0] = 0x01;
	tx_buf[1] = g_data.start_angle>>8;
	tx_buf[1] = tx_buf[1]&0xF;
	as5600_i2c_tx(g_data.client, tx_buf, 2);

	return 0;
}

static int read_mpos(void)
{
	u8 tx_buf[1];
	u8 rx_buf[1];
	int angle;

	tx_buf[0] = 0x03;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = rx_buf[0]&0xF;
	angle = angle<<8;

	tx_buf[0] = 0x04;
	rx_buf[0] = 0;
	as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
	angle = angle + rx_buf[0];

	return angle;
}

static int write_mpos(void){
	u8 tx_buf[2];

	tx_buf[0] = 0x04;
	tx_buf[1] = g_data.end_angle& 0xFF;
	as5600_i2c_tx(g_data.client, tx_buf, 2);

	tx_buf[0] = 0x03;
	tx_buf[1] = g_data.end_angle>>8;
	tx_buf[1] = tx_buf[1]&0xF;
	as5600_i2c_tx(g_data.client, tx_buf, 2);

	return 0;
}

int write_burn(void)
{
	u8 tx_buf[4];

	tx_buf[0] = 0xFF;
	tx_buf[1] = 0x80;
	as5600_i2c_tx(g_data.client, tx_buf, 2);

	mdelay(1);

	tx_buf[0] = 0xFF;
	tx_buf[1] = 0x01;
	tx_buf[2] = 0x11;
	tx_buf[3] = 0x10;
	as5600_i2c_tx(g_data.client, tx_buf, 4);

	return 0;
}

static int as5600_init(void)
{
	int error = 0;
	int retry_count = 0;

	u8 tx_buf[1];
	u8 rx_buf[1];


	tx_buf[0] = 0x00;
	rx_buf[0] = 0;

	for (retry_count = 0; retry_count < 3; retry_count++) {
		error = as5600_i2c_xfer(g_data.client, tx_buf, 1, rx_buf, 1);
		if (error < 0) {
			msleep(1000);
			continue;
		}

		break;
	}

	if (error < 0) return 1;
	g_data.idx = rx_buf[0];
	g_data.flag = 0;
	g_data.freq = 60;
	g_data.delay = HZ/60;
	g_data.start_angle = read_rawangle();
	g_data.end_angle = read_mpos();

	return 0;
}

static ssize_t start_angle_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "start_angle: %d\n", g_data.start_angle);
}

static ssize_t start_angle_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int error;
	int start;

	error = kstrtoint(buf, 10, &start);
	printk("%s(): start=%d.\n", __func__, start);

	if (start == 1) {
		g_data.start_angle = read_rawangle();
		g_data.flag = g_data.flag | 0x1;
	}
	return error ? error : count;
}

static ssize_t end_angle_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "end_angle: %d\n", g_data.end_angle);
}

static ssize_t end_angle_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int error;
	int end;
	error = kstrtoint(buf, 10, &end);
	printk("%s(): end=%d.\n", __func__, end);

	if(end == 1) {
		g_data.end_angle = read_rawangle();
		g_data.flag = g_data.flag | 0x10;
	}

	return error ? error : count;
}
static ssize_t download_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "flag: %d\n", g_data.flag);
}

static ssize_t download_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int error;
	int flag;

	error = kstrtoint(buf, 10, &flag);
	printk("%s(): flag=%d.\n", __func__, flag);

	if(flag == 1 && g_data.flag == 0x11 && g_data.idx < 2) {
		write_zpos();
		mdelay(1);
		write_mpos();
		mdelay(1);
		read_zmco();
		write_burn();
		mdelay(100);
		g_data.start_angle = read_zpos();
		g_data.end_angle = read_mpos();
		g_data.flag = 0;
	}

	return error ? error : count;
}

static ssize_t freq_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "freq: %d\n", g_data.freq);
}

static ssize_t freq_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int error;
	int freq;

	error = kstrtoint(buf, 10, &freq);
	g_data.freq = freq;
	g_data.delay = HZ/freq;
	printk("%s(): g_data.freq=%d, g_data.delay=%d.\n", __func__, g_data.freq, g_data.delay);

	return error ? error : count;
}

static DEVICE_ATTR(start_angle, S_IRUGO|S_IWUSR, start_angle_show, start_angle_store);
static DEVICE_ATTR(end_angle, S_IRUGO|S_IWUSR, end_angle_show, end_angle_store);
static DEVICE_ATTR(download, S_IRUGO|S_IWUSR, download_show, download_store);
static DEVICE_ATTR(freq, S_IRUGO|S_IWUSR, freq_show, freq_store);

static struct attribute *as5600_attrs[] = {
	&dev_attr_start_angle.attr,
	&dev_attr_end_angle.attr,
	&dev_attr_download.attr,
	&dev_attr_freq.attr,
	NULL
};

static const struct attribute_group as5600_attr_group = {
	.attrs = as5600_attrs,
};

static void as_td(struct work_struct *work)
{
	u32 tt;


	if (g_data.flag == 0) {
		mutex_lock(&as5600_mutex);
		ag = read_rawangle();
		//read_angle();
		//read_zpos();
		//read_mpos();
		ag = g_data.start_angle - ag;
		if (ag < 0) ag = ag + 4096;
		tt = ag * 360;
		ap = tt / 4096;
		//printk("**report %d:%d.\n", ag, ap);
		input_report_abs(g_data.input, ABS_X, ap);
		input_sync(g_data.input);
		mutex_unlock(&as5600_mutex);
	}


	schedule_delayed_work(&g_data.as_work, g_data.delay);
}

static ssize_t adjust_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
	mutex_lock(&as5600_mutex);
	_buf[0] = ap & 0xff;
	_buf[1] = (ap >> 8) & 0xff;
	_buf[2] = ag & 0xff;
	_buf[3] = (ag >> 8) & 0xff;
	mutex_unlock(&as5600_mutex);

	return 4;
}

static ssize_t adjust_store(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	mutex_lock(&as5600_mutex);
	g_data.start_angle = read_rawangle();
	mutex_unlock(&as5600_mutex);
	return _count;
}

static CLASS_ATTR(adjust, 0666, adjust_show, adjust_store);

static int as5600_create_input_device(struct as5600_data *as)
{
	struct device *dev = &as->client->dev;
	struct input_dev *input;
	int error;

	input = input_allocate_device();

	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	as->input = input;

	input->name = "AS5600";

	__set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, 0, 65534, 0, 0);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input: %d\n", error);
		return error;
	}
	
	as5600_class = class_create(THIS_MODULE, "as5600");
	if (IS_ERR(as5600_class)) {
		printk("fail to create class as5600\n");
		return -1;
	}

	printk("success to create class as5600\n");

	error = class_create_file(as5600_class, &class_attr_adjust);
	if (error) {
		printk("fail to create class as5600\n");
		return error;
	}

	printk("success to create class file as5600\n");

	return error;
}

static int as5600_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;

	dev_info(&client->dev, "adapter=%d, client irq: %d, addr=0x%x, name=%s.\n",
		client->adapter->nr, client->irq, client->addr, client->name);

	g_data.client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENXIO;

	error = as5600_init();
	if (error) {
		printk("%s(): as5600 init error.\n", __func__);
		return error;
	}
	error = as5600_create_input_device(&g_data);

	INIT_DELAYED_WORK(&g_data.as_work, as_td);
	schedule_delayed_work(&g_data.as_work, 30*HZ);

	error = sysfs_create_group(&client->dev.kobj, &as5600_attr_group);
	if (error) {
		dev_err(&client->dev, "create sysfs failed: %d\n", error);
		return error;
	}

	return error;
}

static int as5600_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &as5600_attr_group);
	return 0;
}

static const struct i2c_device_id as5600_dev_id[] = {
	{ AS5600_NAME, 0 },
	{ }
};

static struct i2c_driver as5600_driver = {
	.probe		= as5600_probe,
	.remove		= as5600_remove,
	.id_table	= as5600_dev_id,
	.driver		= {
		.name	= AS5600_NAME,
	},
};

static int __init as5600_driver_init(void)
{
	printk("%s()\n", __func__);
	return i2c_add_driver(&as5600_driver);
}

static void __exit as5600_driver_exit(void)
{
	i2c_del_driver(&as5600_driver);
}

module_init(as5600_driver_init);
module_exit(as5600_driver_exit);

MODULE_AUTHOR("leisure <huhenglei@njdti.com>");
MODULE_DESCRIPTION("ams as5600 driver");
MODULE_VERSION(AS5600_DRV_VER);
MODULE_LICENSE("GPL");
