/*
 * I2C client/driver for the Laerdal RC
 *
 * Copyright (C) Laerdal Medical
 *
 * Author: Hans Christian Lonstad <hcl@datarespns.no>

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/swab.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/power_supply.h>
#include <linux/slab.h>


#define RC_REG_VERSION 0x00
#define RC_REG_LED_STATUS 0x01
#define RC_REG_CHARGE_STATUS 0x02
#define RC_REG_VOLT_MSB	0x03
#define RC_REG_VOLT_LSB	0x04
#define RC_REG_TEMP_MSB	0x05
#define RC_REG_TEMP_LSB	0x06
#define RC_REG_STATE_CHARGE	0x07
#define RC_REG_STATE_CHARGE_SOC	0x08

struct rc_info;


#define to_rc_info(x) container_of(x, struct rc_info, battery)

struct rc_info {
	struct i2c_client	*client;
	struct power_supply	battery;
	int	id;
};

static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_lock);

static inline int rc_read_reg(struct rc_info *info, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(info->client, reg);
	if (ret < 0) {
		dev_err(&info->client->dev, "register read failed\n");
		return ret;
	}

	*val = ret;
	return 0;
}

static int rc_get_temp(struct rc_info *info, int *temp)
{
	u8 msb, lsb;
	int err;
	int temp_raw;

	/*
	 * Temperature is measured in units of 0.25 degrees Kelvin, the
	 * power_supply class measures temperature in tenths of degrees
	 * celsius.
	 */
	err = rc_read_reg(info, RC_REG_TEMP_MSB, &msb);
	if (err)
		return err;

	err = rc_read_reg(info, RC_REG_TEMP_LSB, &lsb);
	if (err)
		return err;

	temp_raw = (msb << 8) | lsb;

	*temp = (10*temp_raw)/4 - 2700;
	return 0;
}



static int rc_get_voltage(struct rc_info *info, int *voltage_uV)
{
	int volt_raw;
	u8 msb, lsb;
	int err;

	/*
	 * Voltage is measured in units of 1mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	err = rc_read_reg(info, RC_REG_VOLT_MSB, &msb);
	if (err)
		return err;

	err = rc_read_reg(info, RC_REG_VOLT_LSB, &lsb);
	if (err)
		return err;

	volt_raw = (msb << 8) | lsb;
	*voltage_uV = volt_raw * 1000;
	return 0;
}

static int rc_get_capacity(struct rc_info *info, int *percent_cap)
{

	u8 cap_raw;
	int err;

	/*
	 * Voltage is measured in units of 1mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	err = rc_read_reg(info, RC_REG_STATE_CHARGE_SOC, &cap_raw);
	if (err)
		return err;

	*percent_cap = cap_raw;
	return 0;
}

static int rc_get_version(struct rc_info *info, int *ver)
{

	u8 ver_raw;
	int err;

	/*
	 * Voltage is measured in units of 1mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	err = rc_read_reg(info, RC_REG_VERSION, &ver_raw);
	if (err)
		return err;

	*ver = ver_raw;
	return 0;
}

static int rc_get_charge_status(struct rc_info *info, int *status)
{

	u8 status_raw;
	int err;

	/*
	 * Voltage is measured in units of 1mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	err = rc_read_reg(info, RC_REG_STATE_CHARGE, &status_raw);
	if (err)
		return err;

	if ( status_raw )
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	return 0;
}



static int rc_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct rc_info *info = to_rc_info(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = rc_get_charge_status(info, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = rc_get_capacity(info, &val->intval);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = rc_get_voltage(info, &val->intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = rc_get_temp(info, &val->intval);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property rc_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static void rc_power_supply_init(struct power_supply *battery)
{
	battery->type			= POWER_SUPPLY_TYPE_BATTERY;
	battery->properties		= rc_battery_props;
	battery->num_properties		= ARRAY_SIZE(rc_battery_props);
	battery->get_property		= rc_battery_get_property;
	battery->external_power_changed	= NULL;
}

static int rc_battery_remove(struct i2c_client *client)
{
	struct rc_info *info = i2c_get_clientdata(client);

	power_supply_unregister(&info->battery);
	kfree(info->battery.name);

	mutex_lock(&battery_lock);
	idr_remove(&battery_id, info->id);
	mutex_unlock(&battery_lock);

	kfree(info);
	return 0;
}

static int rc_battery_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct rc_info *info;
	int ret;
	int ver;
	int num;

	/* Get an ID for this battery */
	ret = idr_pre_get(&battery_id, GFP_KERNEL);
	if (ret == 0) {
		ret = -ENOMEM;
		goto fail_id;
	}

	mutex_lock(&battery_lock);
	ret = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_lock);
	if (ret < 0)
		goto fail_id;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto fail_info;
	}

	info->battery.name = kasprintf(GFP_KERNEL, "%s-%d", client->name, num);
	if (!info->battery.name) {
		ret = -ENOMEM;
		goto fail_name;
	}



	i2c_set_clientdata(client, info);
	info->client = client;
	info->id = num;

	rc_power_supply_init(&info->battery);

	ret = power_supply_register(&client->dev, &info->battery);
	if (ret) {
		dev_err(&client->dev, "failed to register battery\n");
		goto fail_register;
	}

	ret = rc_get_version(info, &ver);
	if (ret)
		return ret;
	dev_info(&client->dev, "Version is %x\n", ver);

	return 0;

fail_register:
	kfree(info->battery.name);
fail_name:
	kfree(info);
fail_info:
	mutex_lock(&battery_lock);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_lock);
fail_id:
	return ret;
}

static const struct i2c_device_id rc_id[] = {
	{"rc-battery", 0},
	{},
};

static struct i2c_driver rc_battery_driver = {
	.driver 	= {
		.name	= "rc-battery",
	},
	.probe		= rc_battery_probe,
	.remove		= rc_battery_remove,
	.id_table	= rc_id,
};

static int __init rc_battery_init(void)
{
	return i2c_add_driver(&rc_battery_driver);
}
module_init(rc_battery_init);

static void __exit rc_battery_exit(void)
{
	i2c_del_driver(&rc_battery_driver);
}
module_exit(rc_battery_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Fuel Gauge driver for Laerdal RC");
MODULE_LICENSE("GPL");
