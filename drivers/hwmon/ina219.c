/*
 * An hwmon driver for the TI INA219 Power sensor
 *
 *
 * Copyright (c) 2010 DATA RESPONS AS
 *   Hans Christian Lonstad <hcl@datarespons.no>
 *
 *	The chip supports a voltage to current scaler by using a multiplier
 *	on the 12 bit shunt A/D voltage meter.
 *	This driver does not setup the calibration register to support this, but
 *	relies on sw to perform the multiplication step for reading rail voltage and
 *	rail current.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c/ina219_cal.h>

/* INA219 registers */
#define INA219_REG_CFG		0x00
#define INA219_REG_SHUNTV	0x01
#define INA219_REG_BUSV		0x02
#define INA219_REG_POWER	0x03
#define INA219_REG_CURRENT	0x04
#define INA219_REG_CAL		0x05

/*******************************************
 * Full scale shunt voltage range table
 */
static int fs_mvolt[4] = {
	40,
	80,
	160,
	320
};

/*******************************************
 * Rail voltage range table
 */
static int fs_rail_mvolt[2] = {
	16000,
	32000
};

/******************************************
 * Device object
 */
struct ina219_data {
	struct device *hwmon_dev;
	struct mutex lock;
	struct ina219_cal *cal;
	int current_lsb_uamp;
	int rail_lsb_uv;
	int pga;
	int brng;
	int last_vshunt;
	int last_vrail;
	bool valid;
	unsigned long last_updated;
};

static inline int ina219_read(struct i2c_client *client, u8 reg)
{
	int val = i2c_smbus_read_word_data(client, reg);

	if (val < 0)
		return val;

	return swab16(val);
}

static inline int ina219_write(struct i2c_client *client, u8 reg, u16 value)
{
	return i2c_smbus_write_word_data(client, reg, swab16(value));
}

static struct ina219_data *ina219_update_regs(struct device *dev) {

	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	mutex_lock(&data->lock);
	if (time_after(jiffies, data->last_updated + HZ + HZ / 2) || !data->valid) {

		data->last_vrail = ina219_read(client, INA219_REG_BUSV);
		data->last_vrail >>= 3;
		udelay(30);
		data->last_vshunt = ina219_read(client, INA219_REG_SHUNTV);
		data->last_updated = jiffies;
		data->valid = true;
	}
	mutex_unlock(&data->lock);
	return data;
}

static ssize_t show_railv(struct device *dev, struct device_attribute *attr, char *buf) {
	struct ina219_data *data = ina219_update_regs(dev);
	return sprintf(buf, "%d\n", (data->last_vrail * data->rail_lsb_uv)/1000);
}

static ssize_t show_current(struct device *dev, struct device_attribute *attr, char *buf) {
	struct ina219_data *data = ina219_update_regs(dev);
	return sprintf(buf, "%d\n", (data->last_vshunt * data->current_lsb_uamp)/1000);
}

static ssize_t show_power(struct device *dev, struct device_attribute *attr, char *buf) {
	struct ina219_data *data = ina219_update_regs(dev);
	return sprintf(buf, "%d\n",
			((data->last_vshunt * data->current_lsb_uamp)/1000) *
			((data->last_vrail * data->rail_lsb_uv)/1000)
			);
}

static ssize_t show_current_lsb(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->current_lsb_uamp);
}

static ssize_t show_railv_lsb(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ina219_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->rail_lsb_uv);
}


static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_railv, NULL, 0);
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, show_current, NULL, 1);
static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, show_power, NULL, 2);
static SENSOR_DEVICE_ATTR(current_lsb_ua, S_IRUGO, show_current_lsb, NULL, 3);
static SENSOR_DEVICE_ATTR(railv_lsb_uv, S_IRUGO, show_railv_lsb, NULL, 4);

static struct attribute *ina219_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_current_lsb_ua.dev_attr.attr,
	&sensor_dev_attr_railv_lsb_uv.dev_attr.attr,
	NULL
};

static const struct attribute_group ina219_group = {
	.attrs = ina219_attributes,
};

static void ina219_calibrate(struct i2c_client *client, struct ina219_data *data) {
	int vshunt_uvolt;
	int n, conf;
	if ( !data->cal ) {
		dev_warn(&client->dev, "%s No calibration data!.\n", __func__);
		ina219_write(client, INA219_REG_CAL, 4096);
		return;
	}
	conf = ina219_read(client, INA219_REG_CFG);

	vshunt_uvolt = data->cal->shunt_mohm * data->cal->max_current_mamp;

	for (n=0; n < 4; n++)
		if (vshunt_uvolt <= (1000*fs_mvolt[n]))
			break;

	data->pga = n < 4 ? n : 3;
	data->brng = data->cal->vbus_max_mvolt <= fs_rail_mvolt[0] ? 0 : 1;

	// Calculate current lsb in uA
	data->current_lsb_uamp = (320*1000000)/(data->cal->shunt_mohm*0x7fff);
	data->rail_lsb_uv = (16000*1000)/4096;

	dev_info(&client->dev, "%s lsb_uA = %d, lsb_uV = %d",
			__func__, data->current_lsb_uamp, data->rail_lsb_uv);

	conf &= ~0x3800;
	conf |= (data->pga&3) << 11 | data->brng << 13;
	ina219_write(client, INA219_REG_CFG, conf);

}

static int ina219_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	struct ina219_data *data;
	int conf;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		err = -ENODEV;
		goto exit;
	}
	data = kzalloc(sizeof(struct ina219_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->cal = client->dev.platform_data;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	dev_info(&client->dev, "chip found\n");

	/* Make sure the chip is powered up. */
	conf = ina219_read(client, INA219_REG_CFG);
	if (conf < 0)
		dev_warn(&client->dev, "ina219_probe unable to read config register.\n");
	else {
		ina219_write(client, INA219_REG_CFG, conf | 0x8000);	//Reset
	}
	ina219_calibrate(client, data);
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ina219_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &ina219_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int __devexit ina219_remove(struct i2c_client *client)
{
	struct ina219_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &ina219_group);
	kfree(data);
	return 0;
}

static const struct i2c_device_id ina219_id[] = {
	{ "ina219", 0 },
	{}
};

static struct i2c_driver ina219_driver = {
	.driver = {
		.name	= "ina219",
	},
	.probe	= ina219_probe,
	.remove	= __devexit_p(ina219_remove),
	.id_table = ina219_id,
};

static int __init ina219_init(void)
{
	return i2c_add_driver(&ina219_driver);
}
module_init(ina219_init);

static void __exit ina219_exit(void)
{
	i2c_del_driver(&ina219_driver);
}
module_exit(ina219_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");

MODULE_DESCRIPTION("INA219 driver");
MODULE_LICENSE("GPL");
