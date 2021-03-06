/*
 * mcb.c  -- ALSA SoC support for OMAP3517 / AM3517 EVM
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic3105.h"

//#define CODEC_CLOCK 	12288000
#define CODEC_CLOCK 	4096000
#define LINE_IN_DET	90	// Mic/line in plugged
#define LINE_OUT_DET 91	// Audio jack

static int mcbline_hw_params(struct snd_pcm_substream *substream,
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


	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_CLKR_SRC_CLKX, CODEC_CLOCK,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_CLKR_SRC_CLKX\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT, CODEC_CLOCK,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_SYSCLK_CLKS_EXT\n");
		return ret;
	}

	snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_FSR_SRC_FSX, CODEC_CLOCK,
				SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_FSR_SRC_FSX\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops mcbline_ops = {
	.hw_params = mcbline_hw_params,
};

/* Headset jack */
static struct snd_soc_jack hs_jack;

/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Mic In",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Line Out",
		.mask = SND_JACK_HEADPHONE,
	},
};

/* Headset jack detection gpios */
static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	{
		.gpio = LINE_IN_DET,
		.name = "mic-gpio",
		.report = SND_JACK_MICROPHONE,
		.debounce_time = 200,
		.invert = 1,
	},
	{
		.gpio = LINE_OUT_DET,
		.name = "hp-gpio",
		.report = SND_JACK_HEADPHONE,
		.debounce_time = 200,
		.invert = 1,
	},
};

/* mcbline machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic3105_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Line Out", NULL),
	//SND_SOC_DAPM_LINE("Line In R", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "HPLOUT"},
	{"Line Out", NULL, "HPROUT"},

	{"LINE1R", NULL, "Mic Bias 2V"},
	{"LINE1L", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic In"},
};

static int mcbline_aic23_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_new_controls(dapm, tlv320aic3105_dapm_widgets,
				  ARRAY_SIZE(tlv320aic3105_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic In");
	//snd_soc_dapm_enable_pin(codec, "Line In R");

	// Disable unconnected pins
	snd_soc_dapm_nc_pin(dapm, "LLOUT");
	snd_soc_dapm_nc_pin(dapm, "RLOUT");
	snd_soc_dapm_nc_pin(dapm, "MIC3L");
	snd_soc_dapm_nc_pin(dapm, "MIC3R");
	snd_soc_dapm_nc_pin(dapm, "LINE2L");
	snd_soc_dapm_nc_pin(dapm, "LINE2R");
	snd_soc_dapm_nc_pin(dapm, "LLOUT");
	snd_soc_dapm_nc_pin(dapm, "RLOUT");
	snd_soc_dapm_nc_pin(dapm, "HPLCOM");
	snd_soc_dapm_nc_pin(dapm, "HPRCOM");

	snd_soc_dapm_sync(dapm);

	/* Headset jack detection */
	ret = snd_soc_jack_new(codec, "Headset Jack", SND_JACK_HEADSET, &hs_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins), hs_jack_pins);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios), hs_jack_gpios);
	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mcbline_dai = {
	.name = "TLV320AIC3105",
	.stream_name = "AIC23",
	.cpu_dai_name ="omap-mcbsp-dai.0",
	.codec_dai_name = "tlv320aic3105",
	.platform_name = "omap-pcm-audio",
	.codec_name = "tlv320aic3105-codec.2-0018",
	.init = mcbline_aic23_init,
	.ops = &mcbline_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mcbline = {
	.name = "mcbline",
	.dai_link = &mcbline_dai,
	.num_links = 1,
};

static struct platform_device *mcbline_snd_device;

static int __init mcbline_soc_init(void)
{
	int ret;

	pr_info("Laerdal MCB SoC init\n");

	mcbline_snd_device = platform_device_alloc("soc-audio", 0);
	if (!mcbline_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(mcbline_snd_device, &snd_soc_mcbline);
	ret = platform_device_add(mcbline_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(mcbline_snd_device);

	return ret;
}

static void __exit mcbline_soc_exit(void)
{
	snd_soc_jack_free_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios), hs_jack_gpios);
	platform_device_unregister(mcbline_snd_device);
}

module_init(mcbline_soc_init);
module_exit(mcbline_soc_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ALSA SoC Laerdal BU line");
MODULE_LICENSE("GPL v2");
