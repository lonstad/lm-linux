/*
 * omap3_pwm.c
 * Driver for OMAP3 type pwm
 *
 * Copyright (C) 2010 DATA RESPONS AS
 * Author: Hans Christian Lonstad <hcl@datarespons.no>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pwm.h>
#include <linux/types.h>
#include <linux/device.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <plat/dmtimer.h>


#define DEFAULT_PRESCALE 0
#define DEFAULT_PERIOD 0xFF
#define DEFAULT_MATCH (DEFAULT_PERIOD/2)

// Opaque device structure
struct pwm_device {
	const char *label;
	unsigned int pwm_id;
	struct omap_dm_timer *gpt;
};

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{

	u32 tick_rate, period, period_on;

	if (pwm == NULL || period_ns <= 0 || duty_ns > period_ns)
			return -EINVAL;



	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(pwm->gpt));
	printk(KERN_INFO "%s %s with timer #%d, tick_rate = %d, duty_ns=%d period_ns=%d\n",
			__FILE__, __func__, pwm->pwm_id, tick_rate, duty_ns, period_ns);
	period = period_ns/(1000000000/tick_rate);
	period_on = duty_ns/(1000000000/tick_rate);


	if ( (period - period_on) < 3)
		period_on = period-3;

	if (period_on < 4)
		period_on = 4;

	// gpt, enable=0, VAL
	omap_dm_timer_enable(pwm->gpt);
	__delay(15000);	// TODO: HW is aborting, to early for clocks?
	omap_dm_timer_stop(pwm->gpt);
	omap_dm_timer_set_match(pwm->gpt, 1, (0xFFFFFFFF - period_on));

	// gtp, def_on=1, toggle=1, trigger=2
	omap_dm_timer_set_pwm(pwm->gpt, 1, 1, 2);

	// gpt, autoreload=1, VAL
	omap_dm_timer_set_load_start(pwm->gpt, 1, (0xFFFFFFFF - period));
	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	omap_dm_timer_enable(pwm->gpt);
	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	omap_dm_timer_stop(pwm->gpt);
	omap_dm_timer_disable(pwm->gpt);
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		pr_err("%s: failed to allocate memory\n", label);
		return NULL;
	}
	pwm->label = label;
	pwm->pwm_id = pwm_id;

	pwm->gpt = omap_dm_timer_request_specific(pwm->pwm_id);
	if (!pwm->gpt)
	{
		printk(KERN_ERR "%s Cannot allocate timer channel %d\n", __func__, pwm->pwm_id);
		kfree(pwm);
		return 0;
	}

	omap_dm_timer_set_source(pwm->gpt, OMAP_TIMER_SRC_SYS_CLK);

	//tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gpt));
	//pr_err("omap_pwm: Tick rate: %d\n", tick_rate);

	omap_dm_timer_stop(pwm->gpt);

	omap_dm_timer_set_prescaler(pwm->gpt, DEFAULT_PRESCALE);



	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	pwm_disable(pwm);
	kfree(pwm);
}
EXPORT_SYMBOL(pwm_free);
