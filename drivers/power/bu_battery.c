/*
 * I2C client/driver for the Laerdal Base Unit
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
#include <linux/delay.h>

#undef BU_BC_NOHW

typedef enum  {
	BU_BC_CMD_VERSION = 0x02,
	BU_BC_CMD_SET_P1 = 0x03,
	BU_BC_CMD_SET_P2 = 0x04,
	BU_BC_CMD_SET_P3 = 0x05,
	BU_BC_CMD_SET_ADAPTER_VOLTAGE = 0x06,
	BU_BC_CMD_SET_VBUS_VOLTAGE = 0x07,
	BU_BC_CMD_GET_BAT_STATE = 0x30,
	BU_BC_CMD_SET_BATTERY = 0x31,
	BU_BC_CMD_GET_POWER_STATE = 0x32,
	BU_BC_CMD_INVALID = 0xFF,
} BU_BC_CMDS;

typedef enum {
	BU_BC_BAT_CMD_RESUME = 0x00,
	BU_BC_BAT_CMD_SUSPEND = 0x01,
	BU_BC_BAT_CMD_BAT1PRI = 0x02,
	BU_BC_BAT_CMD_BAT2PRI = 0x03,
	BU_BC_BAT_CMD_INVALID = 0xff,
} BU_BC_BAT_CMD;


struct bu_bc_info;


#define to_bu_bc_info(x) container_of(x, struct bu_bc_info, battery)

struct bu_bc_info {
	struct i2c_client	*client;
	u16 p1;
	u16 p2;
	u16 p3;
	u16 vbus_voltage;
	u16 adapter_voltage;
	u16 fw_ver;
	BU_BC_BAT_CMD bat_state;
};


static int bu_bc_read(struct bu_bc_info *info, u8 *vals) {
	int ret;
	u8 buf[4];
	u8 chkSum;

	struct i2c_msg msg[] = {
			{
				.addr = info->client->addr,
				.flags = I2C_M_RD,
				.len = 4,
				.buf = buf,
			},
		};

	ret = i2c_transfer(info->client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&info->client->dev, "%s read failed\n", __func__);
		return ret;
	}

	chkSum = buf[0] + buf[1] + buf[2] + buf[3];
	if ( chkSum != 0 ) {
		dev_err(&info->client->dev, "%s bad checksum\n", __func__);
		return -EIO;
	}
	vals[0] = buf[1];
	vals[1] = buf[2];

	return 0;
}

static int bu_bc_write_cmd(struct bu_bc_info *info, u8 cmd, u8 p1, u8 p2, u8* v1, u8* v2) {
	int ret;
	u8 buf[4];
	u8 vals[2];

#ifdef BU_BC_NOHW
	*v1 = p1;
	*v2 = p2;
	msleep(1000);
#else
	struct i2c_msg msg[] = {
			{
				.addr = info->client->addr,
				.flags = 0,
				.len = 4,
				.buf = buf,
			},
		};

	buf[0] = cmd;
	buf[1] = p1;
	buf[2] = p2;
	buf[3] = ~(cmd + p1 + p2) + 1;

	ret = i2c_transfer(info->client->adapter, msg, 1);
	if (ret < 0) {
	    if (ret == -EREMOTEIO)
	        dev_err(&info->client->dev, "%s BC NACK'ed\n", __func__);
	    else
	        dev_err(&info->client->dev, "%s write failed with %d\n", __func__, ret);
		return ret;
	}
	msleep(1000);
	ret = bu_bc_read(info, vals);
	if ( ret < 0 )
		return ret;

	*v1 = vals[0];
	*v2 = vals[1];
#endif
	return 0;
}

/*****************************************************************************
 *
 * 	SYSFS interface
 */

static int bu_bc_long_to_param(const char* buf, u8* par) {
	unsigned long res;
	int status = strict_strtoul(buf, 10, &res);
	if ( status )
		return status;

	par[0] = (res >> 8) & 0xff;
	par[1] = res & 0xff;
	return 0;
}

static ssize_t bu_bc_led_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf) {
	int status;

	u8 ret_val[2];
	u8 val[2];
	struct bu_bc_info* info = dev_get_drvdata(dev);


	if (strcmp(attr->attr.name, "fw_version") == 0)
	    return sprintf(buf, "%d.%d\n", info->fw_ver >> 8, info->fw_ver & 0xff);

	if (strcmp(attr->attr.name, "power_level_1") == 0)
	    return sprintf(buf, "%d\n", info->p1);

	if (strcmp(attr->attr.name, "power_level_2") == 0)
	    return sprintf(buf, "%d\n", info->p2);

	if (strcmp(attr->attr.name, "power_level_3") == 0)
	    return sprintf(buf, "%d\n", info->p3);

	if (strcmp(attr->attr.name, "adapter_voltage") == 0)
	    return sprintf(buf, "%d\n", info->adapter_voltage);

	if (strcmp(attr->attr.name, "vbus_voltage") == 0)
		return sprintf(buf, "%d\n", info->vbus_voltage);

	if (strcmp(attr->attr.name, "battery_command") == 0) {
	    switch (info->bat_state) {

        case BU_BC_BAT_CMD_RESUME:
            return sprintf(buf, "resume\n");
            break;
        case BU_BC_BAT_CMD_SUSPEND:
            return sprintf(buf, "suspend\n");
            break;
        case BU_BC_BAT_CMD_BAT1PRI:
            return sprintf(buf, "bat1pri\n");
            break;
        case BU_BC_BAT_CMD_BAT2PRI:
            return sprintf(buf, "bat2pri\n");
            break;

        default:
            return 0;
            break;

        }
	}

	if (strcmp(attr->attr.name, "battery_state") == 0) {
	    val[0] = val[1] = 0xCF;
	    status = bu_bc_write_cmd(info, BU_BC_CMD_GET_BAT_STATE, val[0], val[1], &ret_val[0], &ret_val[1]);
	    if (status == 0) {
	        switch(ret_val[0]) {

	        case 0:
	            return sprintf(buf, "idle\n");
	            break;
	        case 1:
	            return sprintf(buf, "suspend\n");
	            break;
	        case 2:
	            return sprintf(buf, "charge_bat1\n");
	            break;
	        case 3:
	            return sprintf(buf, "charge_bat1_wait_bat2\n");
	            break;
	        case 4:
	            return sprintf(buf, "charge_bat2\n");
	            break;
	        case 5:
	            return sprintf(buf, "charge_bat2_wait_bat1\n");
	            break;
	        case 6:
	            return sprintf(buf, "charge_both\n");
	            break;
	        case 7:
	            return sprintf(buf, "bat1_charged\n");
	            break;
	        case 8:
	            return sprintf(buf, "bat2_charged\n");
	            break;
	        case 9:
	            return sprintf(buf, "both_charged\n");
	            break;
	        case 0xFF:
	            return sprintf(buf, "no_dc_present\n");
	            break;

	        default:
	            return 0;
	            break;

	        }
	    }
	    else
	        return 0;

	}

	if (strcmp(attr->attr.name, "power_source_state") == 0) {
	    val[0] = val[1] = 0xCD;
	    status = bu_bc_write_cmd(info, BU_BC_CMD_GET_POWER_STATE, val[0], val[1], &ret_val[0], &ret_val[1]);
	    if (status == 0) {
	        return sprintf(buf, "%d\n", ret_val[0]);
	    }
	}
	return 0;
}

static ssize_t bu_bc_led_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count) {

	int status=0;
	u8 ret_val[2];
	u8 val[2];
	BU_BC_BAT_CMD bat_state = BU_BC_BAT_CMD_INVALID;
	struct bu_bc_info* info = dev_get_drvdata(dev);
	BU_BC_CMDS cmd = BU_BC_CMD_INVALID;
	char *p = strchr(buf, '\n');
	if (p)
		*p = '\0';

	if (strcmp(attr->attr.name, "power_level_1") == 0) {
		cmd = BU_BC_CMD_SET_P1;
		status = bu_bc_long_to_param(buf, val);
	}

	if (strcmp(attr->attr.name, "power_level_2") == 0) {
		cmd = BU_BC_CMD_SET_P2;
		status = bu_bc_long_to_param(buf, val);
	}

	if (strcmp(attr->attr.name, "power_level_3") == 0) {
		cmd = BU_BC_CMD_SET_P3;
		status = bu_bc_long_to_param(buf, val);
	}

	if (strcmp(attr->attr.name, "adapter_voltage") == 0) {
		cmd = BU_BC_CMD_SET_ADAPTER_VOLTAGE;
		status = bu_bc_long_to_param(buf, val);
	}

	if (strcmp(attr->attr.name, "vbus_voltage") == 0) {
		cmd = BU_BC_CMD_SET_VBUS_VOLTAGE;
		status = bu_bc_long_to_param(buf, val);
	}

	if (strcmp(attr->attr.name, "battery_command") == 0) {
			cmd = BU_BC_CMD_SET_BATTERY;
			status = 0;
			if (strcmp(buf, "resume") == 0)
				bat_state = BU_BC_BAT_CMD_RESUME;
			else if (strcmp(buf, "suspend") == 0)
				bat_state = BU_BC_BAT_CMD_SUSPEND;
			else if (strcmp(buf, "bat1pri") == 0)
				bat_state = BU_BC_BAT_CMD_BAT1PRI;
			else if (strcmp(buf, "bat2pri") == 0)
				bat_state = BU_BC_BAT_CMD_BAT2PRI;
			else
				status = -EINVAL;
			val[0] = val[1] = bat_state;
	}

	if ( cmd == BU_BC_CMD_INVALID )
		return -ENODATA;

	if ( status )
		return -EINVAL;

	dev_info(dev, "Set: Command 0x%02x (%s) with 0x%02x  0x%02x\n", cmd, attr->attr.name, val[0], val[1]);
	status = bu_bc_write_cmd(info, cmd, val[0], val[1], &ret_val[0], &ret_val[1]);
	if (status)
		return status;
	switch (cmd) {
	case BU_BC_CMD_SET_P1:
		info->p1 = val[0] << 8 | val[1];
		break;

	case BU_BC_CMD_SET_P2:
		info->p2 = val[0] << 8 | val[1];
		break;

	case BU_BC_CMD_SET_P3:
		info->p3 = val[0] << 8 | val[1];
		break;

	case BU_BC_CMD_SET_ADAPTER_VOLTAGE:
		info->adapter_voltage = val[0] << 8 | val[1];
		break;

	case BU_BC_CMD_SET_VBUS_VOLTAGE:
		info->vbus_voltage = val[0] << 8 | val[1];
		break;

	case BU_BC_CMD_SET_BATTERY:
		info->bat_state = bat_state;
		break;

	default:
		break;
	}
	return count;
}

#define BU_BC_ATTR(_name)					\
{									\
	.attr = { .name = #_name, \
			  .mode = S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP, \
				  },	\
	.show = bu_bc_led_show_property,				\
	.store = bu_bc_led_store_property,				\
}

#define BU_BC_ATTR_RO(_name)					\
{									\
	.attr = { .name = #_name, \
			  .mode = S_IRUSR | S_IRGRP | S_IROTH, \
				  },	\
	.show = bu_bc_led_show_property,				\
	.store = bu_bc_led_store_property,				\
}

static struct device_attribute bu_bc_led_attrs[] = {
	/* Properties of type string */
	BU_BC_ATTR_RO(fw_version),
	BU_BC_ATTR(power_level_1),
	BU_BC_ATTR(power_level_2),
	BU_BC_ATTR(power_level_3),
	BU_BC_ATTR(adapter_voltage),
	BU_BC_ATTR(vbus_voltage),
	BU_BC_ATTR(battery_command),
	BU_BC_ATTR_RO(battery_state),
	BU_BC_ATTR_RO(power_source_state),
};

static int bu_bc_init_attrs(struct device *dev)
{
	int res;
	int n;
	for (n=0; n < ARRAY_SIZE(bu_bc_led_attrs); n++) {
		res = device_create_file(dev, &bu_bc_led_attrs[n]);
		if (res)
			break;
	}
	return res;
}



void bu_bc_say_goodbye(void) {

}
EXPORT_SYMBOL(bu_bc_say_goodbye);


static int bu_bc_remove(struct i2c_client *client)
{
	struct bu_bc_info *info = i2c_get_clientdata(client);
	kfree(info);
	return 0;
}

static int bu_bc_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int status = 0;
	u8 val[2];
	u8 ret_val[2];
	struct bu_bc_info *info;
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}
	info->fw_ver = 0;
	info->p1 = 0;
	info->p2 = 0;
	info->p3 = 0;
	info->adapter_voltage = 0;
	info->vbus_voltage = 0;

	i2c_set_clientdata(client, info);
	info->client = client;



#ifndef BU_BC_NOHW
	val[0] = val[1] = 0xFD;
	status = bu_bc_write_cmd(info, BU_BC_CMD_VERSION, val[0], val[1], &ret_val[0], &ret_val[1]);
	if ( status ) {
		dev_err(&client->dev, "No device found\n");
		kfree(info);
		return status;
	}
	else {
		bu_bc_init_attrs(&client->dev);
		info->fw_ver = ret_val[0] << 8 | ret_val[1];
	}
#else
	bu_bc_init_attrs(&client->dev);
#endif
	dev_info(&client->dev, "Probed with fw version %d.%d\n", info->fw_ver >> 8, info->fw_ver & 0xff);
	return status;

}

static const struct i2c_device_id bu_bc_id[] = {
	{"bu-bc", 0},
	{},
};

static struct i2c_driver bu_bc_driver = {
	.driver 	= {
		.name	= "bu-bc",
	},
	.probe		= bu_bc_probe,
	.remove		= bu_bc_remove,
	.id_table	= bu_bc_id,
};

static int __init bu_bc_init(void)
{
	return i2c_add_driver(&bu_bc_driver);
}
module_init(bu_bc_init);

static void __exit bu_bc_exit(void)
{
	i2c_del_driver(&bu_bc_driver);
}
module_exit(bu_bc_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("Fuel Gauge driver for Laerdal BU");
MODULE_LICENSE("GPL");
