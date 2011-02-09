#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "pcm1681.h"


#define PCM1681_NUM_SUPPLIES 3
static const char *pcm1681_supply_names[PCM1681_NUM_SUPPLIES] = {
	"VDD",
	"VCC1",
	"VCC2",
};

struct pcm1681_priv;

struct pcm1681_disable_nb {
	struct notifier_block nb;
	struct pcm1681_priv *pcm1681;
};
/* codec private data */
struct pcm1681_priv {
	struct snd_soc_codec *codec;
	struct regulator_bulk_data supplies[PCM1681_NUM_SUPPLIES];
	struct pcm1681_disable_nb disable_nb[PCM1681_NUM_SUPPLIES];
	void *control_data;
	enum snd_soc_control_type control_type;
	unsigned int sysclk;
	int power;

};

static const u8 pcm1681_reg[PCM1681_CACHEREGNUM] = {
	0x00, //register 0 is reserved for factory use
	0xff, //ATT1
	0xff, //ATT2
	0xff, //ATT3
	0xff, //ATT4
	0xff, //ATT5
	0xff, //ATT6
	0x00, //MUTE
	0x00, //DAC
	0x07, //IFACE
	0x00, //APDIGI
	0xff, //PHASEO
	0x0f, //FLTRO
	0x00, //ZEROFL
	0x00, //ZEROD  (Bit 7:0 default N/A)
	0x00, //register 15 is reserved for factory use
	0xff, //ATT7
	0xff, //ATT8
	0x00, //MUTEOR
	0x00, //DACOR
};

//static const char *pcm1681_input_select[] = {"Data1", "Data2", "Data3", "Data4"};

static const char *pcm1681_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};

static const struct soc_enum pcm1681_enum = SOC_ENUM_SINGLE(PCM1681_APDIGI, 3, 4, pcm1681_deemph);

static const DECLARE_TLV_DB_SCALE(dac_tlv, -6350, 50, 0);

/*
 * write to the pcm1681 register space
 */
static int pcm1681_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	int ret;
	u8 data[2];
	struct i2c_client *client = codec->control_data;
	/* data is
	 *   D15..D8 pcm1681 register offset
	 *   D7...D0 register data
	 */
	data[0] = reg & 0x7f;
	data[1] = value & 0xff;

	ret = i2c_master_send(client, data, 2);
#if 0
	pcm1681_write_reg_cache(codec, data[0], data[1]);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
#endif
	return ret;
}

/*
 * read from the pcm1681 register space
 */
static unsigned int pcm1681_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;
	//u8 *cache = codec->reg_cache;
	struct i2c_client *client = codec->control_data;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg[2];

	u8 buf[2];
	u8 ret_buf[2];
	buf[0] = reg;
	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = (char *)buf;
	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = ret_buf;


	ret = i2c_transfer(adap, msg, 2);
	return msg[1].buf[0];

	//pcm1681_write_reg_cache(codec, reg, *value);

}


static int snd_soc_pcm1681_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int val = ucontrol->value.integer.value[0] + 128;

	pcm1681_write(codec, reg, val);
	return 0;
}

static int snd_soc_pcm1681_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;

	unsigned int val = pcm1681_read(codec, reg);

	ucontrol->value.integer.value[0] = val - 128;
	return 0;
}

#define SOC_SINGLE_TLV_PCM1681(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_pcm1681_get_volsw,\
	.put = snd_soc_pcm1681_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

static const struct snd_kcontrol_new pcm1681_snd_controls[] = {
	SOC_SINGLE_TLV_PCM1681("SPKR1 Playback Volume",PCM1681_ATT1,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR2 Playback Volume",PCM1681_ATT2,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR3 Playback Volume",PCM1681_ATT3,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR4 Playback Volume",PCM1681_ATT4,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR5 Playback Volume",PCM1681_ATT5,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR6 Playback Volume",PCM1681_ATT6,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR7 Playback Volume",PCM1681_ATT7,0,127,0, dac_tlv),
	SOC_SINGLE_TLV_PCM1681("SPKR8 Playback Volume",PCM1681_ATT8,0,127,0, dac_tlv),
	SOC_ENUM("Deemphasis", pcm1681_enum),
};



static const struct snd_soc_dapm_widget pcm1681_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("PB DAC 1", "Playback", PCM1681_DAC, 0, 1),
	SND_SOC_DAPM_DAC("PB DAC 2", "Playback", PCM1681_DAC, 1, 1),
	SND_SOC_DAPM_DAC("PB DAC 3", "Playback", PCM1681_DAC, 2, 1),
	SND_SOC_DAPM_DAC("PB DAC 4", "Playback", PCM1681_DAC, 3, 1),
	SND_SOC_DAPM_DAC("PB DAC 5", "Playback", PCM1681_DAC, 4, 1),
	SND_SOC_DAPM_DAC("PB DAC 6", "Playback", PCM1681_DAC, 5, 1),
	SND_SOC_DAPM_DAC("PB DAC 7", "Playback", PCM1681_DAC, 6, 1),
	SND_SOC_DAPM_DAC("PB DAC 8", "Playback", PCM1681_DAC, 7, 1),
	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	SND_SOC_DAPM_OUTPUT("LOUT3"),
	SND_SOC_DAPM_OUTPUT("ROUT3"),
	SND_SOC_DAPM_OUTPUT("LOUT4"),
	SND_SOC_DAPM_OUTPUT("ROUT4"),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"LOUT1", NULL, "PB DAC 1"},
	{"ROUT1", NULL, "PB DAC 2"},
	{"LOUT2", NULL, "PB DAC 3"},
	{"ROUT2", NULL, "PB DAC 4"},
	{"LOUT3", NULL, "PB DAC 5"},
	{"ROUT3", NULL, "PB DAC 6"},
	{"LOUT4", NULL, "PB DAC 7"},
	{"ROUT4", NULL, "PB DAC 8"},
};


/*
 * read pcm1681 register cache
 */
static inline unsigned int pcm1681_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= PCM1681_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write pcm1681 register cache
 */
static inline void pcm1681_write_reg_cache(struct snd_soc_codec *codec,
					 u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= PCM1681_CACHEREGNUM)
		return;
	cache[reg] = value;
}


static int pcm1681_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(&codec->dapm, pcm1681_dapm_widgets,
				  ARRAY_SIZE(pcm1681_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, intercon, ARRAY_SIZE(intercon));

	return 0;
}

static int pcm1681_pcm_prepare(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	snd_soc_write(rtd->codec, PCM1681_DAC, 0x00);

	return 0;
}

static int pcm1681_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	/*
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct pcm1681_priv *pcm1681 = snd_soc_codec_get_drvdata(codec);
	 */

	return 0;
}

static void pcm1681_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;


	if (!rtd->codec->active) {
		udelay(50);
		snd_soc_write(rtd->codec, PCM1681_DAC, 0xff);
	}
}


static int pcm1681_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	if (mute)
		snd_soc_write(codec, PCM1681_MUTE, 0xff);
	else
		snd_soc_write(codec, PCM1681_MUTE, 0);
	return 0;
}


static int pcm1681_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm1681_priv *pcm1681 = snd_soc_codec_get_drvdata(codec);

	pcm1681->sysclk = freq;
	return 0;
}

static int pcm1681_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	u8 iface;
	/* set master/slave audio interface */
	switch (fmt & (SND_SOC_DAIFMT_MASTER_MASK | SND_SOC_DAIFMT_INV_MASK)){
	case (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_IF):
		break;
	default:
		return -EINVAL;
		break;
	}
	/*
	 * match both interface format and signal polarities since they
	 * are fixed
	 */
	iface = pcm1681_read_reg_cache(codec_dai->codec, PCM1681_IFACE) & 0xf0;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S :
		iface |= 4;
		break;

	case SND_SOC_DAIFMT_TDM32_I2S:
		iface |= 7;
		break;

	case SND_SOC_DAIFMT_DSP_A:
		iface |= 8;
		break;

	case SND_SOC_DAIFMT_DSP_B:
		iface |= 9;
		break;

	case SND_SOC_DAIFMT_RIGHT_J:
		iface |= 0;
		break;

	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 5;
		break;

	default:
		return -EINVAL;
	}

	snd_soc_write(codec_dai->codec, PCM1681_IFACE, iface);
	return 0;
}


static int pcm1681_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct pcm1681_priv *pcm1681 = snd_soc_codec_get_drvdata(codec);
	int i, ret;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regulator_bulk_enable(ARRAY_SIZE(pcm1681->supplies),
						    pcm1681->supplies);
			if (ret != 0)
				return ret;

			/* Sync reg_cache with the hardware */
			for (i = 0; i < ARRAY_SIZE(pcm1681_reg); i++) {
				if (cache[i] == pcm1681_reg[i])
					continue;

				data[0] = (i << 1) | ((cache[i] >> 8)
						      & 0x0001);
				data[1] = cache[i] & 0x00ff;
				codec->hw_write(codec->control_data, data, 2);
			}
		}

		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, PCM1681_DAC, 0x08ff);
		regulator_bulk_disable(ARRAY_SIZE(pcm1681->supplies),
				       pcm1681->supplies);
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}


#define PCM1681_RATES SNDRV_PCM_RATE_16000

#define PCM1681_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops pcm1681_dai_ops = {
	.prepare	= pcm1681_pcm_prepare,
	.hw_params	= pcm1681_hw_params,
	.shutdown	= pcm1681_shutdown,
	.digital_mute	= pcm1681_mute,
	.set_sysclk	= pcm1681_set_dai_sysclk,
	.set_fmt	= pcm1681_set_dai_fmt,
};

struct snd_soc_dai_driver pcm1681_dai = {
	.name = "PCM1681",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 8,
		.channels_max = 8,
		.rates = PCM1681_RATES,
		.formats = PCM1681_FORMATS,
	},
	.ops = &pcm1681_dai_ops,
};
EXPORT_SYMBOL_GPL(pcm1681_dai);


static int pcm1681_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	pcm1681_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}


static int pcm1681_resume(struct snd_soc_codec *codec)
{
	pcm1681_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int pcm1681_regulator_event(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct pcm1681_disable_nb *disable_nb =
		container_of(nb, struct pcm1681_disable_nb, nb);
	struct pcm1681_priv *pcm1681 = disable_nb->pcm1681;

	if (event & REGULATOR_EVENT_DISABLE) {
		pcm1681->codec->cache_sync = 1;
	}

	return 0;
}

static int pcm1681_probe(struct snd_soc_codec *codec)
{
	struct pcm1681_priv *pcm1681 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int i;

	codec->control_data = pcm1681->control_data;
	pcm1681->codec = codec;
	codec->dapm.idle_bias_off = 1;
#if 0
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, pcm1681->control_type);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

#endif

	snd_soc_write(codec, PCM1681_MUTE, 0xff);

	for (i = 0; i < ARRAY_SIZE(pcm1681->supplies); i++)
		pcm1681->supplies[i].supply = pcm1681_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(pcm1681->supplies),
				 pcm1681->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto err_get;
	}
	for (i = 0; i < ARRAY_SIZE(pcm1681->supplies); i++) {
		pcm1681->disable_nb[i].nb.notifier_call = pcm1681_regulator_event;
		pcm1681->disable_nb[i].pcm1681 = pcm1681;
		ret = regulator_register_notifier(pcm1681->supplies[i].consumer,
						  &pcm1681->disable_nb[i].nb);
		if (ret) {
			dev_err(codec->dev,
				"Failed to request regulator notifier: %d\n",
				 ret);
			goto err_notif;
		}
	}

	codec->cache_only = 0;
	snd_soc_add_controls(codec, pcm1681_snd_controls,
				 ARRAY_SIZE(pcm1681_snd_controls));

	pcm1681_add_widgets(codec);

	return ret;


err_notif:
	while (i--)
		regulator_unregister_notifier(pcm1681->supplies[i].consumer,
						  &pcm1681->disable_nb[i].nb);
	regulator_bulk_free(ARRAY_SIZE(pcm1681->supplies), pcm1681->supplies);
err_get:
	kfree(pcm1681);
	return ret;
}


static int pcm1681_remove(struct snd_soc_codec *codec)
{
	struct pcm1681_priv *pcm1681 = snd_soc_codec_get_drvdata(codec);
	int i;

	pcm1681_set_bias_level(codec, SND_SOC_BIAS_OFF);
	for (i = 0; i < ARRAY_SIZE(pcm1681->supplies); i++)
		regulator_unregister_notifier(pcm1681->supplies[i].consumer,
						  &pcm1681->disable_nb[i].nb);
	regulator_bulk_free(ARRAY_SIZE(pcm1681->supplies), pcm1681->supplies);

	return 0;
}


struct snd_soc_codec_driver soc_codec_dev_pcm1681 = {
	.set_bias_level = pcm1681_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(pcm1681_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = pcm1681_reg,
	.probe = 	pcm1681_probe,
	.remove = 	pcm1681_remove,
	.suspend = 	pcm1681_suspend,
	.resume =	pcm1681_resume,
	.read = pcm1681_read,
	.write = pcm1681_write,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_pcm1681);

static __devinit int pcm1681_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	int ret;
	struct pcm1681_priv *pcm1681;

	pcm1681 = kzalloc(sizeof(struct pcm1681_priv), GFP_KERNEL);
	if (pcm1681 == NULL)
		return -ENOMEM;

	pcm1681->control_data = i2c;
	pcm1681->control_type = SND_SOC_I2C;

	i2c_set_clientdata(i2c, pcm1681);
	ret = snd_soc_register_codec(&i2c->dev,	&soc_codec_dev_pcm1681, &pcm1681_dai, 1);
	if (ret < 0)
		kfree(pcm1681);

	return ret;
}

static __devexit int pcm1681_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id pcm1681_i2c_id[] = {
	{ "pcm1681", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcm1681_i2c_id);

static struct i2c_driver pcm1681_i2c_driver = {
	.driver = {
		.name = "pcm1681-codec",
		.owner = THIS_MODULE,
	},
	.probe =    pcm1681_i2c_probe,
	.remove =   __devexit_p(pcm1681_i2c_remove),
	.id_table = pcm1681_i2c_id,
};



static int __init pcm1681_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&pcm1681_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register pcm1681 I2C driver: %d\n",
		       ret);
	}
	return 0;
}
module_init(pcm1681_modinit);

static void __exit pcm1681_exit(void)
{
	i2c_del_driver(&pcm1681_i2c_driver);

}
module_exit(pcm1681_exit);

MODULE_DESCRIPTION("ASoC PCM1681 codec driver");
MODULE_AUTHOR("Hans Christian Lonstad");
MODULE_LICENSE("GPL");
