/*
 * laerdal-rc.c --  SoC audio for Laerdal RC
 *
 * Author: Steve Sakoman <steve@sakoman.com>
 * 		   Hans Christian Lonstad <hcl@datarespons.no> (Modified from overo.c)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#define AMP_SD 158

static int rc_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops rc_ops = {
	.hw_params = rc_hw_params,
};

static int rc_hp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(AMP_SD, 0);
	else
		gpio_set_value(AMP_SD, 1);

	return 0;
}

static const struct snd_soc_dapm_widget rc_out_dapm_widgets[] = {
	SND_SOC_DAPM_PGA_E("Line Amplifier", SND_SOC_NOPM,
			   0, 0, NULL, 0, rc_hp_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_LINE("Line Out", NULL),
};

static const struct snd_soc_dapm_route rc_out_map[] = {
	{"Line Amplifier", NULL, "PREDRIVEL"},
	{"Line Out", NULL, "Line Amplifier"},
};

static int rc_machine_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* Not comnnected */
	snd_soc_dapm_nc_pin(codec, "HSMIC");
	snd_soc_dapm_nc_pin(codec, "CARKITMIC");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC0");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC1");
	snd_soc_dapm_nc_pin(codec, "AUXR");
	snd_soc_dapm_nc_pin(codec, "AUXL");
	snd_soc_dapm_nc_pin(codec, "MAINMIC");
	snd_soc_dapm_nc_pin(codec, "SUBMIC");

	snd_soc_dapm_enable_pin(codec, "PREDRIVEL");
	/* Some TWL4030 output pins are floating */
	snd_soc_dapm_nc_pin(codec, "EARPIECE");
	snd_soc_dapm_nc_pin(codec, "PREDRIVER");
	snd_soc_dapm_nc_pin(codec, "HSOL");
	snd_soc_dapm_nc_pin(codec, "HSOR");
	snd_soc_dapm_nc_pin(codec, "CARKITL");
	snd_soc_dapm_nc_pin(codec, "CARKITR");
	snd_soc_dapm_nc_pin(codec, "HFL");
	snd_soc_dapm_nc_pin(codec, "HFR");
	snd_soc_dapm_nc_pin(codec, "VIBRA");

	ret = snd_soc_dapm_new_controls(codec, rc_out_dapm_widgets,
				ARRAY_SIZE(rc_out_dapm_widgets));
	if (ret < 0)
		return ret;

	snd_soc_dapm_add_routes(codec, rc_out_map, ARRAY_SIZE(rc_out_map));
	return snd_soc_dapm_sync(codec);
}
/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link rc_dai = {
	.name = "TWL4030",
	.stream_name = "TWL4030",
	.cpu_dai_name = "omap-mcbsp-dai.1",
	.codec_dai_name = "twl4030-hifi",
	.platform_name = "omap-pcm-audio",
	.codec_name = "twl4030-codec",
	.ops = &rc_ops,
	.init = rc_machine_init,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_card_rc = {
	.name = "rc",
	.dai_link = &rc_dai,
	.num_links = 1,
};

static struct platform_device *rc_snd_device;

static int __init rc_soc_init(void)
{
	int ret;

	printk(KERN_INFO "RC SoC init\n");

	ret = gpio_request(AMP_SD, "AMP_SD");
	if (ret) {
		printk(KERN_ERR "Failed to get amp power GPIO\n");
		return ret;
	}

	ret = gpio_direction_output(AMP_SD, 1);

	rc_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rc_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(rc_snd_device, &snd_soc_card_rc);

	ret = platform_device_add(rc_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(rc_snd_device);

	return ret;
}
module_init(rc_soc_init);

static void __exit rc_soc_exit(void)
{
	platform_device_unregister(rc_snd_device);
}
module_exit(rc_soc_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ALSA SoC RC");
MODULE_LICENSE("GPL");
