

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

#include "../codecs/pcm1681.h"

//#define CODEC_CLOCK 	12288000
#define CODEC_CLOCK 	4096000

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

static const struct snd_soc_dapm_route audio_map[] = {
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
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

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

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mcb1681_dai = {
	.name = "PCM1681",
	.stream_name = "PCM1681_STREAM",
	.cpu_dai_name ="omap-mcbsp-dai.1",
	.codec_dai_name = "PCM1681",
	.platform_name = "omap-pcm-audio",
	.codec_name = "pcm1681-codec.2-004c",
	.init = mcb1681_init,
	.ops = &mcb1681_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mcb1681 = {
	.name = "mcb1681",
	.dai_link = &mcb1681_dai,
	.num_links = 1,
};

static struct platform_device *mcb1681_snd_device;

static int __init mcb1681_soc_init(void)
{
	int ret;

	pr_info("Laerdal MCB1681 SoC init\n");


	mcb1681_snd_device = platform_device_alloc("soc-audio", 1);
	if (!mcb1681_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(mcb1681_snd_device, &snd_soc_mcb1681);
	ret = platform_device_add(mcb1681_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(mcb1681_snd_device);

	return ret;
}

static void __exit mcb1681_soc_exit(void)
{
	platform_device_unregister(mcb1681_snd_device);
}

module_init(mcb1681_soc_init);
module_exit(mcb1681_soc_exit);

MODULE_AUTHOR("Hans Chr Lonstad <hcl@datarespons.no>");
MODULE_DESCRIPTION("ALSA SoC for MCB 1681 codec");
MODULE_LICENSE("GPL v2");
