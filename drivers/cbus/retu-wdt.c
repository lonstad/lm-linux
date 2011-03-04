/**
 * drivers/cbus/retu-wdt.c
 *
 * Driver for Retu watchdog
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Amit Kucheria <amit.kucheria@nokia.com>
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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>

#include <asm/uaccess.h>

#include <plat/prcm.h>

#include "cbus.h"
#include "retu.h"

/* Watchdog timeout in seconds */
#define RETU_WDT_MIN_TIMER 0
#define RETU_WDT_DEFAULT_TIMER 32
#define RETU_WDT_MAX_TIMER 63

static DEFINE_MUTEX(retu_wdt_mutex);

/* Current period of watchdog */
static unsigned int period_val = RETU_WDT_DEFAULT_TIMER;

struct retu_wdt_dev {
	struct device		*dev;
	unsigned long		users;
	struct miscdevice	retu_wdt_miscdev;
	struct delayed_work	ping_work;
};

static struct retu_wdt_dev *retu_wdt;

static int _retu_modify_counter(unsigned int new)
{
	if (retu_wdt)
		retu_write_reg(retu_wdt->dev, RETU_REG_WATCHDOG, (u16)new);

	return 0;
}

static int retu_modify_counter(unsigned int new)
{
	if (new < RETU_WDT_MIN_TIMER || new > RETU_WDT_MAX_TIMER)
		return -EINVAL;

	mutex_lock(&retu_wdt_mutex);
	period_val = new;
	_retu_modify_counter(period_val);
	mutex_unlock(&retu_wdt_mutex);

	return 0;
}

/*
 * Since retu watchdog cannot be disabled in hardware, we must kick it
 * with a timer until userspace watchdog software takes over. Do this
 * unless /dev/watchdog is open or CONFIG_WATCHDOG_NOWAYOUT is set.
 */
static void retu_wdt_ping_enable(struct retu_wdt_dev *wdev)
{
	_retu_modify_counter(RETU_WDT_MAX_TIMER);
	schedule_delayed_work(&wdev->ping_work,
			      round_jiffies_relative(RETU_WDT_DEFAULT_TIMER * HZ));
}

static void retu_wdt_ping_disable(struct retu_wdt_dev *wdev)
{
	_retu_modify_counter(RETU_WDT_MAX_TIMER);
	cancel_delayed_work_sync(&wdev->ping_work);
}

static void retu_wdt_ping_work(struct work_struct *work)
{
	struct retu_wdt_dev *wdev = container_of(to_delayed_work(work),
					struct retu_wdt_dev, ping_work);
	retu_wdt_ping_enable(wdev);
}

static int retu_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &retu_wdt->users))
		return -EBUSY;

	file->private_data = (void *)retu_wdt;
	retu_wdt_ping_disable(retu_wdt);

	return nonseekable_open(inode, file);
}

static int retu_wdt_release(struct inode *inode, struct file *file)
{
	struct retu_wdt_dev *wdev = file->private_data;

#ifndef CONFIG_WATCHDOG_NOWAYOUT
	retu_wdt_ping_enable(wdev);
#endif
	clear_bit(0, &wdev->users);

	return 0;
}

static ssize_t retu_wdt_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	if (len)
		retu_modify_counter(RETU_WDT_MAX_TIMER);

	return len;
}

static long retu_wdt_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int new_margin;

	static struct watchdog_info ident = {
		.identity = "Retu Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOTTY;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
							sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_GETBOOTSTATUS:
		if (cpu_is_omap16xx())
			return put_user(omap_readw(ARM_SYSST),
					(int __user *)arg);
		if (cpu_is_omap24xx())
			return put_user(omap_prcm_get_reset_sources(),
					(int __user *)arg);
	case WDIOC_KEEPALIVE:
		retu_modify_counter(RETU_WDT_MAX_TIMER);
		break;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;
		retu_modify_counter(new_margin);
		/* Fall through */
	case WDIOC_GETTIMEOUT:
		return put_user(period_val, (int __user *)arg);
	}

	return 0;
}

static const struct file_operations retu_wdt_fops = {
	.owner		= THIS_MODULE,
	.write		= retu_wdt_write,
	.unlocked_ioctl	= retu_wdt_ioctl,
	.open		= retu_wdt_open,
	.release	= retu_wdt_release,
};

static int __init retu_wdt_probe(struct platform_device *pdev)
{
	struct retu_wdt_dev *wdev;
	int ret;

	wdev = kzalloc(sizeof(struct retu_wdt_dev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->dev = &pdev->dev;

	platform_set_drvdata(pdev, wdev);
	retu_wdt = wdev;
	wdev->retu_wdt_miscdev.parent = &pdev->dev;
	wdev->retu_wdt_miscdev.minor = WATCHDOG_MINOR;
	wdev->retu_wdt_miscdev.name = "watchdog";
	wdev->retu_wdt_miscdev.fops = &retu_wdt_fops;

	ret = misc_register(&(wdev->retu_wdt_miscdev));
	if (ret)
		goto err_free_wdev;

	INIT_DELAYED_WORK(&wdev->ping_work, retu_wdt_ping_work);

	/* Kick the watchdog for kernel booting to finish.
	 * If nowayout is not set, we start the ping work. */
#ifdef CONFIG_WATCHDOG_NOWAYOUT
	retu_modify_counter(RETU_WDT_MAX_TIMER);
#else
	retu_wdt_ping_enable(retu_wdt);
#endif

	return 0;

err_free_wdev:
	kfree(wdev);

	return ret;
}

static int __devexit retu_wdt_remove(struct platform_device *pdev)
{
	struct retu_wdt_dev *wdev;

	wdev = platform_get_drvdata(pdev);
	misc_deregister(&wdev->retu_wdt_miscdev);
	cancel_delayed_work_sync(&wdev->ping_work);
	kfree(wdev);

	return 0;
}

static struct platform_driver retu_wdt_driver = {
	.remove		= __exit_p(retu_wdt_remove),
	.driver		= {
		.name	= "retu-wdt",
	},
};

static int __init retu_wdt_init(void)
{
	return platform_driver_probe(&retu_wdt_driver, retu_wdt_probe);
}

static void __exit retu_wdt_exit(void)
{
	platform_driver_unregister(&retu_wdt_driver);
}

module_init(retu_wdt_init);
module_exit(retu_wdt_exit);

MODULE_DESCRIPTION("Retu WatchDog");
MODULE_AUTHOR("Amit Kucheria");
MODULE_LICENSE("GPL");
