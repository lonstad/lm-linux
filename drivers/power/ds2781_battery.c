/*************************************************************************
 * Driver for batteries with DS2781 chips inside.
 *
 * Copyright © 2011 Laerdal Medical
 *
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * Author:  Hans Chr Lonstad <hcl@datarespons.no>
 *	    February 2011
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2781.h"


/*****************************************************************
 *
 *  Device class
 */
struct ds2781_device_info {
    struct device *dev;

    unsigned long update_time; /* jiffies when data read */
    u8 raw[DS2781_DATA_SIZE]; /* raw DS2781 data */
    int voltage_raw; /* units of 4.88 mV */
    int voltage_uV; /* units of µV */
    int current_raw; /* units depends on res */
    int current_uA; /* units of µA */
    int current_avg_uA; /* units of uA */
    int accum_current_uAh; /* units of µAh */
    int temp_raw; /* units of 0.125 °C */
    int temp_C; /* units of 0.1 °C */
    int rated_capacity; /* units of µAh */
    int rem_capacity; /* percentage */
    int full_active_uAh; /* units of µAh */
    int empty_uAh; /* units of µAh */
    int life_sec; /* units of seconds */
    int charge_status; /* POWER_SUPPLY_STATUS_* */

    struct power_supply bat;
    struct device *w1_dev;
    struct workqueue_struct *monitor_wqueue;
    struct delayed_work monitor_work;
    struct delayed_work set_charged_work;
};

static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static unsigned int pmod_enabled;
module_param(pmod_enabled, bool, 0644);
MODULE_PARM_DESC(pmod_enabled, "PMOD enable bit");


/***********************************************************
 *
 *  Battery model
 */
struct battery_info {
    int acc_bias_mA;
    int aging_cap_mAh;
    int charge_voltage_mV;
    int min_charge_current_mA;
    int active_empty_voltage_mV;
    int active_empty_current_mA;
    int active_empty40_mAh;
    int sense_resistor_mhos;
    int full40_mAh;
    int full_slopes[4];
    int ae_slopes[4];
    int se_slopes[4];
    int rs_gain_x1000;
    int current_bias_mAh;
    int temp_breaks[3];

};

static struct battery_info dummy_bat = {
    .acc_bias_mA = 0,
    .aging_cap_mAh = 1220,
    .charge_voltage_mV = 8400,
    .min_charge_current_mA = 50,
    .active_empty_voltage_mV = 7000,
    .active_empty_current_mA = 240,
    .active_empty40_mAh = 6,
    .sense_resistor_mhos = 50,
    .full40_mAh = 1051,
    .full_slopes = { 822, 1110, 3013, 3330 },
    .ae_slopes = { 303, 634, 1110, 2260 },
    .se_slopes = { 173, 264, 317, 1308 },
    .rs_gain_x1000 = 1000,
    .current_bias_mAh = 0,
    .temp_breaks = { 18, 0, -12 }, };

/***********************************************************************
 *
 *  Parameter calculation and installation
 */
static int ds2781_battery_set_eeprom(struct ds2781_device_info *di) {

    static u8 buf[0x80];
    int val;
    u64 tmp;
    buf[0x61] = (dummy_bat.acc_bias_mA * dummy_bat.sense_resistor_mhos * 10000) / 15625;

    val = (dummy_bat.aging_cap_mAh * dummy_bat.sense_resistor_mhos * 100) / 625;
    buf[0x62] = val >> 8;
    buf[0x63] = val & 0xff;

    buf[0x64] = (dummy_bat.charge_voltage_mV * 100) / 3904;
    buf[0x65] = (dummy_bat.min_charge_current_mA * dummy_bat.sense_resistor_mhos) / 50;
    buf[0x66] = (dummy_bat.active_empty_voltage_mV * 100) / 3904;
    buf[0x67] = (dummy_bat.active_empty_current_mA * dummy_bat.sense_resistor_mhos) / 200;
    buf[0x68] = (dummy_bat.active_empty40_mAh * 1024) / dummy_bat.full40_mAh;
    buf[0x69] = dummy_bat.sense_resistor_mhos;

    tmp = (dummy_bat.full40_mAh * 100000) / (dummy_bat.sense_resistor_mhos * 625);
    val = tmp;
    buf[0x6A] = val >> 8;
    buf[0x6B] = val & 0xff;

    buf[0x6C] = (dummy_bat.full_slopes[0] * (1 << 14)) / 10 ^ 6;
    buf[0x6D] = (dummy_bat.full_slopes[1] * (1 << 14)) / 10 ^ 6;
    buf[0x6E] = (dummy_bat.full_slopes[2] * (1 << 14)) / 10 ^ 6;
    buf[0x6F] = (dummy_bat.full_slopes[3] * (1 << 14)) / 10 ^ 6;

    buf[0x70] = (dummy_bat.ae_slopes[0] * (1 << 14)) / 10 ^ 6;
    buf[0x71] = (dummy_bat.ae_slopes[1] * (1 << 14)) / 10 ^ 6;
    buf[0x72] = (dummy_bat.ae_slopes[2] * (1 << 14)) / 10 ^ 6;
    buf[0x73] = (dummy_bat.ae_slopes[3] * (1 << 14)) / 10 ^ 6;

    buf[0x74] = (dummy_bat.se_slopes[0] * (1 << 14)) / 10 ^ 6;
    buf[0x75] = (dummy_bat.se_slopes[1] * (1 << 14)) / 10 ^ 6;
    buf[0x76] = (dummy_bat.se_slopes[2] * (1 << 14)) / 10 ^ 6;
    buf[0x77] = (dummy_bat.se_slopes[3] * (1 << 14)) / 10 ^ 6;

    val = (dummy_bat.rs_gain_x1000 * 1024) / 1000;
    buf[0x78] = val >> 8;
    buf[0x79] = val & 0xff;

    buf[0x7A] = 0;
    buf[0x7B] = (dummy_bat.current_bias_mAh * dummy_bat.sense_resistor_mhos * 10000) / 15625;
    buf[0x7C] = (char) dummy_bat.temp_breaks[0];
    buf[0x7D] = (char) dummy_bat.temp_breaks[1];
    buf[0x7E] = (char) dummy_bat.temp_breaks[2];

    w1_ds2781_write(di->w1_dev, &buf[0x61], 0x61, 0x7E - 0x61 + 1);

    w1_ds2781_store_eeprom(di->w1_dev, DS2781_EEPROM_BLOCK1);
    w1_ds2781_recall_eeprom(di->w1_dev, DS2781_EEPROM_BLOCK1);
    return 0;
}

static int ds2781_battery_read_status(struct ds2781_device_info *di) {
    int ret, start, count;
    u16 ds2781_rcap;
    int val;
    u64 tmp;

    if (di->update_time && time_before(jiffies, di->update_time +
            msecs_to_jiffies(cache_time)))
        return 0;

    // Read full EEPROM first time, thereafter only dynamic content
    if (di->update_time == 0) {
        //ds2781_battery_set_eeprom(di);
        ret = w1_ds2781_read(di->w1_dev, di->raw, 0, DS2781_DATA_SIZE);
        ds2781_rcap = (di->raw[DS2781_FULL40_CAPACITY_MSB] << 8) | di->raw[DS2781_FULL40_CAPACITY_LSB];
        tmp = (ds2781_rcap * 625 * di->raw[DS2781_ACTIVE_RSNSP]) / 100; /* convert to µAh */
        di->rated_capacity = (int) tmp;
    } else {
        start = DS2781_STATUS_REG;
        count = 0x1b;
        ret = w1_ds2781_read(di->w1_dev, di->raw + start, start, count);
    }

    di->update_time = jiffies;

    //DS2781 reports voltage in units of 9.76mV
    di->voltage_raw = (di->raw[DS2781_VOLTAGE_MSB] << 3) | (di->raw[DS2781_VOLTAGE_LSB] >> 5);
    di->voltage_uV = di->voltage_raw * 9760;

    // Read and convert current
    di->current_raw = ((signed char) (di->raw[DS2781_CURRENT_MSB]) << 8) | di->raw[DS2781_CURRENT_LSB];
    di->current_uA = (di->current_raw * di->raw[DS2781_ACTIVE_RSNSP] * 78) / 50;

    // Average current
    val = ((signed char) (di->raw[DS2781_CURRENT_AVG_MSB]) << 8) | di->raw[DS2781_CURRENT_AVG_LSB];
    di->current_avg_uA = (val * di->raw[DS2781_ACTIVE_RSNSP] * 78) / 50;

    // Accumulated current is in units of 6.25uVh/RSNS
    di->accum_current_uAh = (((di->raw[DS2781_CURRENT_ACCUM_MSB] << 8) | di->raw[DS2781_CURRENT_ACCUM_LSB]) * 625
            * di->raw[DS2781_ACTIVE_RSNSP]) / 100;

    // Temperature for DS2781 is in 0.125, convert to 0.1
    di->temp_raw = (((signed char) di->raw[DS2781_TEMP_MSB]) << 3) | (di->raw[DS2781_TEMP_LSB] >> 5);
    di->temp_C = (5*di->temp_raw)/4;

    // Full level
    tmp = di->raw[DS2781_ACTIVE_FULL] << 8 | di->raw[DS2781_ACTIVE_FULL + 1];
    tmp = (tmp * di->rated_capacity) / (1 << 15);
    di->full_active_uAh = (int) tmp;

    // Active Empty
    tmp = di->raw[DS2781_ACTIVE_EMPTY] << 8 | di->raw[DS2781_ACTIVE_EMPTY + 1];
    tmp = (tmp * di->rated_capacity) / (1 << 15);
    di->empty_uAh = (int) tmp;

    return 0;
}

static void ds2781_battery_set_current_accum(struct ds2781_device_info *di, unsigned int acr_val) {
    unsigned char acr[2];

    acr_val = (acr_val * 100) / (di->raw[DS2781_ACTIVE_RSNSP] * 625);

    acr[0] = acr_val >> 8;
    acr[1] = acr_val & 0xff;

    if (w1_ds2781_write(di->w1_dev, acr, DS2781_CURRENT_ACCUM_MSB, 2) < 2)
        dev_warn(di->dev, "ACR write failed\n");
}

static void ds2781_battery_update_status(struct ds2781_device_info *di) {
    int val;
    ds2781_battery_read_status(di);
    if (di->current_uA < -5000)
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    else if (di->current_uA > 5000)
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    else
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    di->rem_capacity = di->raw[DS2781_RARC];
    val = (di->raw[DS2781_RAAC_MSB] << 8) | di->raw[DS2781_RAAC_LSB];

    if (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
        if ( di->current_avg_uA >= 0)
            di->life_sec = (val * 1600 * 36) / (-di->current_uA / 100);
        else
            di->life_sec = (val * 1600 * 36) / (-di->current_avg_uA / 100);
    }
}

static void ds2781_battery_write_control(struct ds2781_device_info *di, char control) {
    if (control == di->raw[DS2781_CONTROL_REG])
        return;

    w1_ds2781_write(di->w1_dev, &control, DS2781_CONTROL_REG, 1);
    w1_ds2781_store_eeprom(di->w1_dev, DS2781_EEPROM_BLOCK1);
    w1_ds2781_recall_eeprom(di->w1_dev, DS2781_EEPROM_BLOCK1);
}

static void ds2781_battery_work(struct work_struct *work) {
    struct ds2781_device_info
    *di = container_of(work, struct ds2781_device_info, monitor_work.work);
    const int interval = HZ * 60;

    dev_dbg(di->dev, "%s\n", __func__);

    ds2781_battery_update_status(di);
    queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

#define to_ds2781_device_info(x) container_of((x), struct ds2781_device_info, bat);

static void ds2781_battery_external_power_changed(struct power_supply *psy) {
    struct ds2781_device_info *di = to_ds2781_device_info(psy);

    dev_dbg(di->dev, "%s\n", __func__);

    cancel_delayed_work(&di->monitor_work);
    queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ / 10);
}


static int ds2781_battery_get_property(struct power_supply *psy, enum power_supply_property psp,
        union power_supply_propval *val) {

    struct ds2781_device_info *di = to_ds2781_device_info(psy);

    ds2781_battery_update_status(di);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = di->charge_status;
        return 0;
    default:
        break;
    }

    switch (psp) {
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = di->voltage_uV;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = di->current_uA;
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = di->current_avg_uA;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = di->rated_capacity;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = di->full_active_uAh;
        break;
    case POWER_SUPPLY_PROP_CHARGE_EMPTY:
        val->intval = di->empty_uAh;
        break;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        val->intval = di->accum_current_uAh;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = di->temp_C;
        break;
    case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        val->intval = di->life_sec;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = di->rem_capacity;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int ds2781_battery_set_property(struct power_supply *psy, enum power_supply_property psp,
        const union power_supply_propval *val) {

    struct ds2781_device_info *di = to_ds2781_device_info(psy);

    switch (psp) {

    case POWER_SUPPLY_PROP_CHARGE_NOW:
        /* ds2781_battery_set_current_accum() does the conversion */
        ds2781_battery_set_current_accum(di, val->intval);
        break;

    default:
        return -EPERM;
    }

    return 0;
}

static int ds2781_battery_property_is_writeable(struct power_supply *psy, enum power_supply_property psp) {
    switch (psp) {
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        return 1;

    default:
        break;
    }

    return 0;
}

static enum power_supply_property ds2781_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_EMPTY,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_CAPACITY, };

static int ds2781_battery_probe(struct platform_device *pdev) {
    char control;
    int retval = 0;
    struct ds2781_device_info *di;

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
        return -ENOMEM;

    platform_set_drvdata(pdev, di);

    di->dev = &pdev->dev;
    di->w1_dev = pdev->dev.parent;
    di->bat.name = dev_name(&pdev->dev);
    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    di->bat.properties = ds2781_battery_props;
    di->bat.num_properties = ARRAY_SIZE(ds2781_battery_props);
    di->bat.get_property = ds2781_battery_get_property;
    di->bat.set_property = ds2781_battery_set_property;
    di->bat.property_is_writeable = ds2781_battery_property_is_writeable;
    di->bat.external_power_changed = ds2781_battery_external_power_changed;

    di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

    ds2781_battery_read_status(di);
    control = di->raw[DS2781_CONTROL_REG];

    if (pmod_enabled)
        control |= DS2781_CONTROL_PMOD;
    else
        control &= ~DS2781_CONTROL_PMOD;

    ds2781_battery_write_control(di, control);

    retval = power_supply_register(&pdev->dev, &di->bat);
    if (retval) {
        dev_err(di->dev, "failed to register battery\n");
        goto batt_failed;
    }

    INIT_DELAYED_WORK(&di->monitor_work, ds2781_battery_work);
    di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
    if (!di->monitor_wqueue) {
        retval = -ESRCH;
        goto workqueue_failed;
    }
    queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 1);

    return 0;

workqueue_failed:
    power_supply_unregister(&di->bat);
batt_failed:
    kfree(di);

    return retval;
}

static int ds2781_battery_remove(struct platform_device *pdev) {
    struct ds2781_device_info *di = platform_get_drvdata(pdev);

    cancel_delayed_work_sync(&di->monitor_work);
    cancel_delayed_work_sync(&di->set_charged_work);
    destroy_workqueue(di->monitor_wqueue);
    power_supply_unregister(&di->bat);
    kfree(di);

    return 0;
}

#ifdef CONFIG_PM

static int ds2781_battery_suspend(struct platform_device *pdev,
        pm_message_t state)
{
    //struct ds2781_device_info *di = platform_get_drvdata(pdev);

    return 0;
}

static int ds2781_battery_resume(struct platform_device *pdev)
{
    struct ds2781_device_info *di = platform_get_drvdata(pdev);

    cancel_delayed_work(&di->monitor_work);
    queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ);

    return 0;
}

#else

#define ds2781_battery_suspend NULL
#define ds2781_battery_resume NULL

#endif /* CONFIG_PM */

MODULE_ALIAS("platform:ds2781-battery");

static struct platform_driver ds2781_battery_driver = {
    .driver = { .name = "ds2781-battery", },
    .probe = ds2781_battery_probe,
    .remove = ds2781_battery_remove,
    .suspend = ds2781_battery_suspend,
    .resume = ds2781_battery_resume, };

static int __init ds2781_battery_init(void)
{
    return platform_driver_register(&ds2781_battery_driver);
}

static void __exit ds2781_battery_exit(void)
{
    platform_driver_unregister(&ds2781_battery_driver);
}

module_init(ds2781_battery_init);
module_exit(ds2781_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hans Chr Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ds2781 battery driver");
