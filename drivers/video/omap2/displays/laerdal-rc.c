/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
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
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings rc_panel_timings = {
	/* 800 x 600 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */
	.x_res		= 640,
	.y_res		= 480,
	.pixel_clock	= 25000,
	.hfp		= 16,
	.hsw		= 30,
	.hbp		= 84,
	.vfp		= 3,
	.vsw		= 3,
	.vbp		= 32,
};

static int rc_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void rc_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int rc_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = rc_panel_timings;

	return 0;
}

static void rc_panel_remove(struct omap_dss_device *dssdev)
{
}

static int rc_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = rc_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void rc_panel_disable(struct omap_dss_device *dssdev)
{
	if ( dssdev )
		rc_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int rc_panel_suspend(struct omap_dss_device *dssdev)
{
	rc_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int rc_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = rc_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void rc_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void rc_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int rc_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver rc_driver = {
	.probe		= rc_panel_probe,
	.remove		= rc_panel_remove,

	.enable		= rc_panel_enable,
	.disable	= rc_panel_disable,
	.suspend	= rc_panel_suspend,
	.resume		= rc_panel_resume,

	.set_timings	= rc_panel_set_timings,
	.get_timings	= rc_panel_get_timings,
	.check_timings	= rc_panel_check_timings,

	.driver         = {
		.name   = "rc_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init rc_panel_drv_init(void)
{
	return omap_dss_register_driver(&rc_driver);
}

static void __exit rc_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&rc_driver);
}

module_init(rc_panel_drv_init);
module_exit(rc_panel_drv_exit);
MODULE_LICENSE("GPL");
