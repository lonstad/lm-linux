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

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/tlv320aic3105.h"

#define CODEC_CLOCK 	12288000

static int mcbline_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
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

/* mcbline machine dapm widgets */
static const struct snd_soc_dapm_widget tlv320aic3105_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	//SND_SOC_DAPM_MIC("Mic In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "HPLOUT"},
	{"Line Out", NULL, "HPROUT"},

	{"LINE1L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
};

static int mcbline_aic23_init(struct snd_soc_codec *codec)
{
	/* Add am3517-evm specific widgets */
	snd_soc_dapm_new_controls(codec, tlv320aic3105_dapm_widgets,
				  ARRAY_SIZE(tlv320aic3105_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Line In");

	// Disable unconnected pins
	snd_soc_dapm_disable_pin(codec, "LLOUT");
	snd_soc_dapm_disable_pin(codec, "RLOUT");
	snd_soc_dapm_disable_pin(codec, "MIC3L");
	snd_soc_dapm_disable_pin(codec, "MIC3R");
	snd_soc_dapm_disable_pin(codec, "LINE2L");
	snd_soc_dapm_disable_pin(codec, "LINE2R");
	//snd_soc_dapm_enable_pin(codec, "Mic In");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mcbline_dai = {
	.name = "TLV320AIC3105",
	.stream_name = "AIC23",
	.cpu_dai = &omap_mcbsp_dai[0],
	.codec_dai = &aic3105_dai,
	.init = mcbline_aic23_init,
	.ops = &mcbline_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mcbline = {
	.name = "mcbline",
	.platform = &omap_soc_platform,
	.dai_link = &mcbline_dai,
	.num_links = 1,
};

/* Audio subsystem */
static struct snd_soc_device mcbline_snd_devdata = {
	.card = &snd_soc_mcbline,
	.codec_dev = &soc_codec_dev_aic3105,
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

	platform_set_drvdata(mcbline_snd_device, &mcbline_snd_devdata);
	mcbline_snd_devdata.dev = &mcbline_snd_device->dev;
	*(unsigned int *)mcbline_dai.cpu_dai->private_data = 0; /* McBSP1 */

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
	platform_device_unregister(mcbline_snd_device);
}

module_init(mcbline_soc_init);
module_exit(mcbline_soc_exit);

MODULE_AUTHOR("Hans Christian Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ALSA SoC Laerdal BU line");
MODULE_LICENSE("GPL v2");
