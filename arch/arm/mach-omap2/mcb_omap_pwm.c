/*
 * OMAP PWM driver. Enabled userspace usage of the PWM outputs
 *
 * Copyright (C) 2010 Laerdal Medical
 * Author:	Svein Seldal <sveinse@seldal.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/device.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <plat/dmtimer.h>


/* Maximum number of devices to allocate */
#define DEVICES 1
#define DEVICE_NAME "omap_pwm"

struct omap_pwm_dev {
    struct cdev cdev;
    struct omap_dm_timer *timer;
};


static struct class *omap_pwm_class;            /* Tie with the device model */

static struct omap_pwm_dev *omap_pwm_devp;

static dev_t omap_pwm_dev_number;               /* Allotted device number */

static int omap_pwm_timerid = 10;

#define DEFAULT_PRESCALE 0
#define DEFAULT_PERIOD 0xFF
#define DEFAULT_MATCH (DEFAULT_PERIOD/2)


#ifdef HACK
struct omap_dm_timer {
	unsigned long phys_base;
	int irq;
#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3) || \
			defined(CONFIG_ARCH_OMAP4)
	struct clk *iclk, *fclk;
#endif
	void __iomem *io_base;
	unsigned reserved:1;
	unsigned enabled:1;
	unsigned posted:1;
};

static inline u32 timer_read_reg(struct omap_dm_timer *timer, u32 reg)
{
	if (timer->posted)
		while (readl(timer->io_base + (0x34 & 0xff))
				& (reg >> 16))
			cpu_relax();
	return readl(timer->io_base + (reg & 0xff));
}
#endif

static int omap_pwm_open(struct inode *inode, struct file *file)
{
    struct omap_pwm_dev *devp;
    struct omap_dm_timer *gpt;
    u32 tick_rate;

    //pr_err("omap_pwm: open()\n");

    devp = container_of(inode->i_cdev, struct omap_pwm_dev, cdev);
    file->private_data = devp;

    gpt = omap_dm_timer_request_specific(omap_pwm_timerid);
    if (!gpt)
    {
        pr_err("omap_pwm: Cannot allocate timer channel\n");
        return -EFAULT;
    }

    devp->timer = gpt;

    omap_dm_timer_set_source(gpt, OMAP_TIMER_SRC_SYS_CLK);

    //tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gpt));
    //pr_err("omap_pwm: Tick rate: %d\n", tick_rate);

    omap_dm_timer_stop(gpt);

    omap_dm_timer_set_prescaler(gpt, DEFAULT_PRESCALE);

    // gpt, enable=0, VAL
    omap_dm_timer_set_match(gpt, 1, (0xFFFFFFFF - (DEFAULT_MATCH)));

    // gtp, def_on=0, toggle=1, trigger=2 
    omap_dm_timer_set_pwm(gpt, 0, 1, 2);

    // gpt, autoreload=1, VAL
    omap_dm_timer_set_load_start(gpt, 1, (0xFFFFFFFF - (DEFAULT_PERIOD)));

    return 0;
}


static int omap_pwm_release(struct inode *inode, struct file *file)
{
    struct omap_pwm_dev *devp = file->private_data;
    if (devp->timer)
        omap_dm_timer_free(devp->timer);

    pr_err("omap_pwm: release()\n");

    return 0;
}


static ssize_t omap_pwm_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    // EOF
    return 0;
}


static ssize_t omap_pwm_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct omap_pwm_dev *devp = file->private_data;
    struct omap_dm_timer *gpt = devp->timer;
    unsigned char val, *kbuf;
    size_t i;

    //pr_err("omap_pwm: write() %d bytes\n", count);

    kbuf = kmalloc(count,GFP_KERNEL);
    if(!kbuf)
    {
        return -ENOMEM;
    }
    if (copy_from_user(kbuf,buf,count))
    {
        kfree(kbuf);
        return -EFAULT;
    }

    for(i=0;i<count;i++)
    {
        // Safeguard against overflow (value >0xFE is not permitted by HW). See Datasheet
        val = kbuf[i];
        val = val>0xFD ? 0xFD : val;

        omap_dm_timer_set_match(gpt, 1, (0xFFFFFFFF - 0xFF + val));
    }
    kfree(kbuf);

#ifdef HACK
    pr_err("omap_pwm: STATUS %08x, TIMER %08x\n", omap_dm_timer_read_status(gpt), omap_dm_timer_read_counter(gpt));
    pr_err("omap_pwm: CTRL: %08x\n", timer_read_reg(gpt, 0x24));
    pr_err("omap_pwm: COUNTER: %08x\n", timer_read_reg(gpt, 0x28));
    pr_err("omap_pwm: LOAD: %08x\n", timer_read_reg(gpt, 0x2C));
    pr_err("omap_pwm: TRIGGER: %08x\n", timer_read_reg(gpt, 0x30));
    pr_err("omap_pwm: MATCH: %08x\n", timer_read_reg(gpt, 0x38));
#endif

    return count;
}


static struct file_operations omap_pwm_fops = {
        .owner   = THIS_MODULE,
        .open    = omap_pwm_open,
        .release = omap_pwm_release,
        .read    = omap_pwm_read,
        .write   = omap_pwm_write,
};


static int __init omap_pwm_init(void)
{
    int ret;
    pr_err("omap_pwm: omap_pwm_init()\n");

    // Driver is not available on other than omap
    if (!(cpu_is_omap16xx() || cpu_class_is_omap2()))
        return -ENODEV;
    if (!cpu_is_omap34xx())
    {
        pr_warning("omap_pwm: Driver only tested for omap34xx. Use at own risk\n");
    }

    // Request dynamic allocation of a device major number 
    if (0 > alloc_chrdev_region(&omap_pwm_dev_number, 0, DEVICES, DEVICE_NAME))
    {
        pr_err("omap_pwm: Unable to get dynamic major number for device\n");
        ret = -EIO;
        goto err_alloc_chrdev_region;
    }

    // Allocate memory for the per-device structure
    omap_pwm_devp = kzalloc(sizeof(struct omap_pwm_dev), GFP_KERNEL);
    if (!omap_pwm_devp)
    {
        pr_err("omap_pwm: Memory allocation failure\n");
        ret = -ENOMEM;
        goto err_kzalloc;
    }

    // Populate the sysfs entries
    omap_pwm_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(omap_pwm_class))
    {
        pr_err("omap_pwm: Unable to register sysfs class\n");
        ret = -ENOMEM;
        goto err_class_create;
    }

    // Connect the major/minor number to the cdev
    cdev_init(&(omap_pwm_devp->cdev), &omap_pwm_fops);
    omap_pwm_devp->cdev.owner = THIS_MODULE;

    // Connect the major/minor number to the cdev
    if (cdev_add(&(omap_pwm_devp->cdev), omap_pwm_dev_number, 1))
    {
        pr_err("omap_pwm: Bad cdev\n");
        ret = -EINVAL;
        goto err_cdev_add;
    }

    // Link the cdev nodes to the /class/omap_pwm in sysfs
    device_create(omap_pwm_class, NULL, omap_pwm_dev_number, NULL, DEVICE_NAME);

    pr_err("omap_pwm: Driver initialized\n");
    pr_err("omap_pwm: Please remember to enable PWM output on the pin mux, i.e. echo 0x2 > /sys/kernel/debug/omap_mux/uart2_rts\n");

    return 0;

err_cdev_add:
err_class_create:
    kfree(omap_pwm_devp);

err_kzalloc:
err_alloc_chrdev_region:
    return ret;
}


static void __exit omap_pwm_exit(void)
{
    pr_err("omap_pwm: omap_pwm_exit()\n");

    // Destroy the class/omap_pwm binding
    device_destroy(omap_pwm_class, omap_pwm_dev_number);

    // Delete the cdev entry
    cdev_del(&(omap_pwm_devp->cdev));

    // Destroy the overall class
    class_destroy(omap_pwm_class);

    // Release the major numbers
    unregister_chrdev_region(MAJOR(omap_pwm_dev_number), DEVICES);

    pr_err("omap_pwm: Exit done\n");
    return;
}

module_init(omap_pwm_init);
module_exit(omap_pwm_exit);


MODULE_AUTHOR("Svein Seldal <sveinse@seldal.com>");
MODULE_DESCRIPTION("Omap PWM char driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("char:omap_pwm");

