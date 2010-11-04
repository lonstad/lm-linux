/*
 * pll1708.c
 *
 *  Created on: Sep 10, 2010
 *      Author: hcl
 * drivers/gpio/pll1708.c
 *
 * Copyright (C) 2010 Laerdal Medical
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/pll1708.h>

#define MODE_REG 0x7000
#define CONFIG_REG 0x6C00
#define REG_MASK 0xFC00
#define VAL_MASK 0x03FF

#define SCKO0_MASK 0x010
#define SCKO1_MASK 0x080
#define SCKO2_MASK 0x020
#define SCKO3_MASK 0x040
#define MCKO1_MASK 0x100
#define MCKO2_MASK 0x200

#define SR_MASK 0x00C
#define SR_SHIFT 2
#define FS_MASK 3

#define CFG1_MASK 0x010
#define DEBUG

/* A write to the pll1708 means one message with one transfer */
static int pll1708_spi_write(struct device *dev, unsigned int reg,
				unsigned int val)
{
	u16 word;
	struct spi_device *spi = to_spi_device(dev);

	switch (reg)
	{
	case 0:
		word = MODE_REG | (val & VAL_MASK);
		break;
	case 1:
		word = CONFIG_REG | (val & VAL_MASK);
		break;
	default:
		printk(KERN_ERR "%s; Illegal register val %d\n", __func__, reg);
		return -EINVAL;
		break;

	}

	return spi_write(spi, (const u8 *)&word, sizeof(word));
}

static int __devinit pll1708_probe(struct spi_device *spi)
{
	struct pll1708 *ts;
	int ret;

	/* bits_per_word cannot be configured in platform data */
	printk(KERN_INFO "%s called\n", __func__);
	dev_dbg(&spi->dev,"%s called\n", __func__);
	spi->bits_per_word = 16;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ts = kzalloc(sizeof(struct pll1708), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	// ts->read = pll1708_spi_read;
	//ts->write = pll1708_spi_write;
	ts->dev = &spi->dev;
	dev_set_drvdata(&spi->dev, ts);
	ts->freq = PLL1708_FS_48KHZ;
	ts->sr = PLL1708_SR_STANDARD;
	ts->cfg1 = 1;

	pll1708_spi_write(ts->dev, 0, SCKO1_MASK | SCKO2_MASK | (ts->freq) | (ts->sr << SR_SHIFT));
	pll1708_spi_write(ts->dev, 1, CFG1_MASK);
	return 0;
}

static int __devexit pll1708_remove(struct spi_device *spi)
{
	struct pll1708 *ts = dev_get_drvdata(&spi->dev);

	if (ts == NULL)
		return -ENODEV;

	dev_set_drvdata(&spi->dev, NULL);


	//ts->write(dev, 0x04, 0x00);

	return 0;
}

static const struct spi_device_id pll1708_id[] = {
	{ "pll1708", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, pll1708_id);

static struct spi_driver pll1708_driver = {
	.driver = {
		.name = "pll1708",
		.owner = THIS_MODULE,
	},
	.probe = pll1708_probe,
	.remove = __devexit_p(pll1708_remove),
	.id_table = pll1708_id,
};

static int __init pll1708_init(void)
{
	return spi_register_driver(&pll1708_driver);
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pll1708_init);

static void __exit pll1708_exit(void)
{
	spi_unregister_driver(&pll1708_driver);
}
module_exit(pll1708_exit);

MODULE_AUTHOR("Hans Christian Lonstad");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI PLL1708 Clock Gen Driver");

