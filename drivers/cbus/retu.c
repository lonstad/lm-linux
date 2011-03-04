/**
 * drivers/cbus/retu.c
 *
 * Support functions for Retu ASIC
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Juha Yrj�l� <juha.yrjola@nokia.com>,
 *	      David Weinehall <david.weinehall@nokia.com>, and
 *	      Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <plat/mux.h>
#include <plat/board.h>
#include <plat/cbus.h>

#include "cbus.h"
#include "retu.h"

struct retu {
	/* Device lock */
	struct mutex		mutex;
	struct device		*dev;

	int			devid;

	int			irq_base;
	int			irq_end;

	int			irq;

	int			ack;
	bool			ack_pending;

	int			mask;
	bool			mask_pending;

	bool			is_vilma;
};

static struct retu *the_retu;

/**
 * __retu_read_reg - Read a value from a register in Retu
 * @retu: pointer to retu structure
 * @reg: the register address to read from
 */
static int __retu_read_reg(struct retu *retu, unsigned reg)
{
	return cbus_read_reg(retu->devid, reg);
}

/**
 * __retu_write_reg - Writes a value to a register in Retu
 * @retu: pointer to retu structure
 * @reg: the register address to write to
 * @val: the value to write to the register
 */
static void __retu_write_reg(struct retu *retu, unsigned reg, u16 val)
{
	cbus_write_reg(retu->devid, reg, val);
}

/**
 * retu_read_reg - Read a value from a register in Retu
 * @child: device pointer for the calling child
 * @reg: the register to read from
 *
 * This function returns the contents of the specified register
 */
int retu_read_reg(struct device *child, unsigned reg)
{
	struct retu		*retu = dev_get_drvdata(child->parent);

	return __retu_read_reg(retu, reg);
}
EXPORT_SYMBOL_GPL(retu_read_reg);

/**
 * retu_write_reg - Write a value to a register in Retu
 * @child: the pointer to our calling child
 * @reg: the register to write to
 * @val: the value to write to the register
 *
 * This function writes a value to the specified register
 */
void retu_write_reg(struct device *child, unsigned reg, u16 val)
{
	struct retu		*retu = dev_get_drvdata(child->parent);

	mutex_lock(&retu->mutex);
	__retu_write_reg(retu, reg, val);
	mutex_unlock(&retu->mutex);
}
EXPORT_SYMBOL_GPL(retu_write_reg);

/**
 * retu_set_clear_reg_bits - helper function to read/set/clear bits
 * @child: device pointer to calling child
 * @reg: the register address
 * @set: mask for setting bits
 * @clear: mask for clearing bits
 */
void retu_set_clear_reg_bits(struct device *child, unsigned reg, u16 set,
		u16 clear)
{
	struct retu		*retu = dev_get_drvdata(child->parent);
	u16			w;

	mutex_lock(&retu->mutex);
	w = __retu_read_reg(retu, reg);
	w &= ~clear;
	w |= set;
	__retu_write_reg(retu, reg, w);
	mutex_unlock(&retu->mutex);
}
EXPORT_SYMBOL_GPL(retu_set_clear_reg_bits);

#define ADC_MAX_CHAN_NUMBER	13

/**
 * retu_read_adc - Reads AD conversion result
 * @child: device pointer to calling child
 * @channel: the ADC channel to read from
 */
int retu_read_adc(struct device *child, int channel)
{
	struct retu		*retu = dev_get_drvdata(child->parent);
	int			res;

	if (!retu)
		return -ENODEV;

	if (channel < 0 || channel > ADC_MAX_CHAN_NUMBER)
		return -EINVAL;

	mutex_lock(&retu->mutex);

	if ((channel == 8) && retu->is_vilma) {
		int scr = __retu_read_reg(retu, RETU_REG_ADCSCR);
		int ch = (__retu_read_reg(retu, RETU_REG_ADCR) >> 10) & 0xf;
		if (((scr & 0xff) != 0) && (ch != 8))
			__retu_write_reg(retu, RETU_REG_ADCSCR, (scr & ~0xff));
	}

	/* Select the channel and read result */
	__retu_write_reg(retu, RETU_REG_ADCR, channel << 10);
	res = __retu_read_reg(retu, RETU_REG_ADCR) & 0x3ff;

	if (retu->is_vilma)
		__retu_write_reg(retu, RETU_REG_ADCR, (1 << 13));

	/* Unlock retu */
	mutex_unlock(&retu->mutex);

	return res;
}
EXPORT_SYMBOL_GPL(retu_read_adc);

static irqreturn_t retu_irq_handler(int irq, void *_retu)
{
	struct retu		*retu = _retu;

	int			i;

	u16			idr;
	u16			imr;

	mutex_lock(&retu->mutex);
	idr = __retu_read_reg(retu, RETU_REG_IDR);
	imr = __retu_read_reg(retu, RETU_REG_IMR);
	mutex_unlock(&retu->mutex);

	idr &= ~imr;
	if (!idr) {
		dev_vdbg(retu->dev, "No IRQ, spurious?\n");
		return IRQ_NONE;
	}

	for (i = retu->irq_base; idr != 0; i++, idr >>= 1) {
		if (!(idr & 1))
			continue;

		handle_nested_irq(i);
	}

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------- */

static void retu_irq_mask(struct irq_data *data)
{
	struct retu		*retu = irq_data_get_irq_chip_data(data);
	int			irq = data->irq;

	retu->mask |= (1 << (irq - retu->irq_base));
	retu->mask_pending = true;
}

static void retu_irq_unmask(struct irq_data *data)
{
	struct retu		*retu = irq_data_get_irq_chip_data(data);
	int			irq = data->irq;

	retu->mask &= ~(1 << (irq - retu->irq_base));
	retu->mask_pending = true;

}

static void retu_irq_ack(struct irq_data *data)
{
	struct retu		*retu = irq_data_get_irq_chip_data(data);
	int			irq = data->irq;

	retu->ack |= (1 << (irq - retu->irq_base));
	retu->ack_pending = true;
}

static void retu_bus_lock(struct irq_data *data)
{
	struct retu		*retu = irq_data_get_irq_chip_data(data);

	mutex_lock(&retu->mutex);
}

static void retu_bus_sync_unlock(struct irq_data *data)
{
	struct retu		*retu = irq_data_get_irq_chip_data(data);

	if (retu->mask_pending) {
		__retu_write_reg(retu, RETU_REG_IMR, retu->mask);
		retu->mask_pending = false;
	}

	if (retu->ack_pending) {
		__retu_write_reg(retu, RETU_REG_IDR, retu->ack);
		retu->ack_pending = false;
	}

	mutex_unlock(&retu->mutex);
}

static struct irq_chip retu_irq_chip = {
	.name			= "retu",
	.irq_bus_lock		= retu_bus_lock,
	.irq_bus_sync_unlock	= retu_bus_sync_unlock,
	.irq_mask		= retu_irq_mask,
	.irq_unmask		= retu_irq_unmask,
	.irq_ack		= retu_irq_ack,
};

static inline void retu_irq_setup(int irq)
{
#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	set_irq_noprobe(irq);
#endif
}

static void retu_irq_init(struct retu *retu)
{
	int			base = retu->irq_base;
	int			end = retu->irq_end;
	int			irq;

	for (irq = base; irq < end; irq++) {
		set_irq_chip_data(irq, retu);
		set_irq_chip_and_handler(irq, &retu_irq_chip,
				handle_simple_irq);
		set_irq_nested_thread(irq, 1);
		retu_irq_setup(irq);
	}
}

static void retu_irq_exit(struct retu *retu)
{
	int			base = retu->irq_base;
	int			end = retu->irq_end;
	int			irq;

	for (irq = base; irq < end; irq++) {
#ifdef CONFIG_ARM
		set_irq_flags(irq, 0);
#endif
		set_irq_chip_and_handler(irq, NULL, NULL);
		set_irq_chip_data(irq, NULL);
	}
}

/* -------------------------------------------------------------------------- */

/**
 * retu_power_off - Shut down power to system
 *
 * This function puts the system in power off state
 */
static void retu_power_off(void)
{
	struct retu		*retu = the_retu;
	unsigned		reg;

	reg = __retu_read_reg(retu, RETU_REG_CC1);

	/* Ignore power button state */
	__retu_write_reg(retu, RETU_REG_CC1, reg | 2);
	/* Expire watchdog immediately */
	__retu_write_reg(retu, RETU_REG_WATCHDOG, 0);
	/* Wait for poweroff*/
	for (;;);
}

static struct resource generic_resources[] = {
	{
		.start	= -EINVAL,	/* fixed later */
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= -EINVAL,	/* fixed later */
		.flags	= IORESOURCE_IRQ,
	},
};

/**
 * retu_allocate_child - Allocates one Retu child
 * @name: name of new child
 * @parent: parent device for this child
 */
static struct device *retu_allocate_child(char *name, struct device *parent,
		int irq_base, int irq1, int irq2, int num)
{
	struct platform_device		*pdev;
	int				status;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		dev_dbg(parent, "can't allocate %s\n", name);
		goto err;
	}

	pdev->dev.parent = parent;

	if (num) {
		generic_resources[0].start = irq_base + irq1;
		generic_resources[1].start = irq_base + irq2;

		status = platform_device_add_resources(pdev,
				generic_resources, num);
		if (status < 0) {
			dev_dbg(parent, "can't add resources to %s\n", name);
			goto err;
		}
	}

	status = platform_device_add(pdev);
	if (status < 0) {
		dev_dbg(parent, "can't add %s\n", name);
		goto err;
	}

	return &pdev->dev;

err:
	platform_device_put(pdev);

	return NULL;
}

/**
 * retu_allocate_children - Allocates Retu's children
 */
static int retu_allocate_children(struct device *parent, int irq_base)
{
	struct device	*child;

	child = retu_allocate_child("retu-pwrbutton", parent, irq_base,
			RETU_INT_PWR, -1, 1);
	if (!child)
		return -ENOMEM;

	child = retu_allocate_child("retu-headset", parent, irq_base,
			RETU_INT_HOOK, -1, 1);
	if (!child)
		return -ENOMEM;

	child = retu_allocate_child("retu-rtc", parent, irq_base,
			RETU_INT_RTCS, RETU_INT_RTCA, 2);
	if (!child)
		return -ENOMEM;

	child = retu_allocate_child("retu-wdt", parent, -1, -1, -1, 0);
	if (!child)
		return -ENOMEM;

	return 0;
}

/**
 * retu_probe - Probe for Retu ASIC
 * @dev: the Retu device
 *
 * Probe for the Retu ASIC and allocate memory
 * for its device-struct if found
 */
static int __init retu_probe(struct platform_device *pdev)
{
	struct retu	*retu;
	struct cbus_retu_platform_data *pdata = pdev->dev.platform_data;

	int		ret = -ENOMEM;
	int		rev;

	retu = kzalloc(sizeof(*retu), GFP_KERNEL);
	if (!retu) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err0;
	}

	platform_set_drvdata(pdev, retu);

	retu->irq	= platform_get_irq(pdev, 0);
	retu->irq_base	= pdata->irq_base;
	retu->irq_end	= pdata->irq_end;
	retu->devid	= pdata->devid;
	the_retu	= retu;

	mutex_init(&retu->mutex);

	retu_irq_init(retu);

	rev = __retu_read_reg(retu, RETU_REG_ASICR) & 0xff;
	if (rev & (1 << 7))
		retu->is_vilma = true;

	dev_info(&pdev->dev, "%s v%d.%d found\n",
			retu->is_vilma ? "Vilma" : "Retu",
			(rev >> 4) & 0x07, rev & 0x0f);

	/* Mask all RETU interrupts */
	__retu_write_reg(retu, RETU_REG_IMR, 0xffff);

	ret = request_threaded_irq(retu->irq, NULL, retu_irq_handler, 0,
			  "retu", retu);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register IRQ handler\n");
		goto err1;
	}

	set_irq_wake(retu->irq, 1);

	/* Register power off function */
	pm_power_off = retu_power_off;

	ret = retu_allocate_children(&pdev->dev, retu->irq_base);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to allocate Retu children\n");
		goto err2;
	}

	return 0;

err2:
	pm_power_off = NULL;
	__retu_write_reg(retu, RETU_REG_IMR, 0xffff);
	free_irq(retu->irq, retu);

err1:
	kfree(retu);
	the_retu = NULL;

err0:
	return ret;
}

static int __exit retu_remove(struct platform_device *pdev)
{
	struct retu		*retu = platform_get_drvdata(pdev);

	pm_power_off = NULL;
	the_retu = NULL;

	/* Mask all RETU interrupts */
	__retu_write_reg(retu, RETU_REG_IMR, 0xffff);

	free_irq(retu->irq, retu);
	retu_irq_exit(retu);
	kfree(retu);

	return 0;
}

static struct platform_driver retu_driver = {
	.remove		= __exit_p(retu_remove),
	.driver		= {
		.name	= "retu",
	},
};

static int __init retu_init(void)
{
	return platform_driver_probe(&retu_driver, retu_probe);
}
subsys_initcall(retu_init);

static void __exit retu_exit(void)
{
	platform_driver_unregister(&retu_driver);
}
module_exit(retu_exit);

MODULE_DESCRIPTION("Retu ASIC control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrj�l�");
MODULE_AUTHOR("David Weinehall");
MODULE_AUTHOR("Mikko Ylinen");
