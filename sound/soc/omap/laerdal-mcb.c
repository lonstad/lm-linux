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
#include <sound/pcm_params.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic3105.h"
#include "../codecs/pcm1681.h"

//#define CODEC_CLOCK 	12288000
#define CODEC_CLOCK 	4096000

/**********************************************************************
 *
 *
 * 	Line IO
 */
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

/* mcb machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic3105_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Line Out", NULL),
	//SND_SOC_DAPM_LINE("Line In R", NULL),
	SND_SOC_DAPM_MIC("Mic In", NULL),
};

static const struct snd_soc_dapm_route audio_map_3105[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "HPLOUT"},
	{"Line Out", NULL, "HPROUT"},

	{"LINE1R", NULL, "Mic Bias 2V"},
	{"LINE1L", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic In"},
};

static int mcb_aic23_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_new_controls(dapm, tlv320aic3105_dapm_widgets,
				  ARRAY_SIZE(tlv320aic3105_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, audio_map_3105, ARRAY_SIZE(audio_map_3105));

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



/*******************************************************************************
 *
 * 	8 Channel D/A
 */

static int mcb1681_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned int fmt_dai, fmt_cpu;
	unsigned ch = params_channels(params);
	unsigned fmt = params_format(params);

	if ((fmt == SNDRV_PCM_FORMAT_S24_LE || fmt == SNDRV_PCM_FORMAT_S32_LE) && ch == 8 && params_rate(params) == 16000) {
		fmt_dai = SND_SOC_DAIFMT_TDM32_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBS_CFS;
		fmt_cpu = SND_SOC_DAIFMT_TDM32_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBS_CFS;
	}
	else {
		printk(KERN_WARNING "%s: Invalid format %d or channels %d or rate %d\n", __func__, fmt, ch, params_rate(params));
		return -EINVAL;
	}
	printk(KERN_INFO "%s rate is %d\n", __func__, params_rate(params));
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt_dai);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt_cpu);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}


	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT, CODEC_CLOCK,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set CPU system clock OMAP_MCBSP_SYSCLK_CLKS_EXT\n");
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}


	return 0;
}

static struct snd_soc_ops mcb1681_ops = {
	.hw_params = mcb1681_hw_params,
};

static const struct snd_soc_dapm_widget soc_pcm1681_dapm_widgets[] = {
	SND_SOC_DAPM_HP("OUT1", NULL),
	SND_SOC_DAPM_HP("OUT2", NULL),
	SND_SOC_DAPM_HP("OUT3", NULL),
	SND_SOC_DAPM_HP("OUT4", NULL),
	SND_SOC_DAPM_HP("OUT5", NULL),
	SND_SOC_DAPM_HP("OUT6", NULL),
	SND_SOC_DAPM_HP("OUT7", NULL),
	SND_SOC_DAPM_HP("OUT8", NULL),

};

static const struct snd_soc_dapm_route audio_map_1681[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"OUT1", NULL, "LOUT1"},
	{"OUT2", NULL, "ROUT1"},
	{"OUT3", NULL, "LOUT2"},
	{"OUT4", NULL, "ROUT2"},
	{"OUT5", NULL, "LOUT3"},
	{"OUT6", NULL, "ROUT3"},
	{"OUT7", NULL, "LOUT4"},
	{"OUT8", NULL, "ROUT4"},

};

static int mcb1681_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	snd_soc_dapm_new_controls(dapm, soc_pcm1681_dapm_widgets,
				  ARRAY_SIZE(soc_pcm1681_dapm_widgets));
	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map_1681, ARRAY_SIZE(audio_map_1681));

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "OUT1");
	snd_soc_dapm_enable_pin(dapm, "OUT2");
	snd_soc_dapm_enable_pin(dapm, "OUT3");
	snd_soc_dapm_enable_pin(dapm, "OUT4");
	snd_soc_dapm_enable_pin(dapm, "OUT5");
	snd_soc_dapm_enable_pin(dapm, "OUT6");
	snd_soc_dapm_enable_pin(dapm, "OUT7");
	snd_soc_dapm_enable_pin(dapm, "OUT8");

	snd_soc_dapm_sync(dapm);

	return 0;
}

/*******************************************************************************
 *
 * 	Integration
 */
/* Audio machine driver */

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mcb_dais[] = {
	{
		.name = "PCM1681",
		.stream_name = "PCM1681_STREAM",
		.cpu_dai_name ="omap-mcbsp-dai.1",
		.codec_dai_name = "PCM1681",
		.platform_name = "omap-pcm-audio",
		.codec_name = "pcm1681-codec.2-004c",
		.init = mcb1681_init,
		.ops = &mcb1681_ops,
	},
	{
		.name = "TLV320AIC3105",
		.stream_name = "AIC23",
		.cpu_dai_name ="omap-mcbsp-dai.0",
		.codec_dai_name = "tlv320aic3105",
		.platform_name = "omap-pcm-audio",
		.codec_name = "tlv320aic3105-codec.2-0018",
		.init = mcb_aic23_init,
		.ops = &mcbline_ops,
	},

};

static struct snd_soc_card snd_soc_mcb = {
	.name = "mcb",
	.dai_link = mcb_dais,
	.num_links = 2,
};

static struct platform_device *mcb_snd_device;

static int __init mcb_soc_init(void)
{
	int ret;

	pr_info("Laerdal MCB SoC init\n");

	mcb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!mcb_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(mcb_snd_device, &snd_soc_mcb);
	ret = platform_device_add(mcb_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(mcb_snd_device);

	return ret;
}

static void __exit mcb_soc_exit(void)
{
	snd_soc_jack_free_gpios(&hs_jack, ARRAY_SIZE(hs_jack_gpios), hs_jack_gpios);
	platform_device_unregister(mcb_snd_device);
}

module_init(mcb_soc_init);
module_exit(mcb_soc_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ALSA SoC Laerdal Base Unit");
MODULE_LICENSE("GPL v2");
