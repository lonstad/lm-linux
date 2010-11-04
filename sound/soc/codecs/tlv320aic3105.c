/*
 * ALSA SoC TLV320aic3105 codec driver
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  The aic3105 is a driver for a low power stereo audio
 *  codecs aic31, aic32, aic33.
 *
 *  It supports full aic33 codec functionality.
 *  The compatibility with aic32, aic31 is as follows:
 *        aic32        |        aic31
 *  ---------------------------------------
 *   MONO_LOUT -> N/A  |  MONO_LOUT -> N/A
 *                     |  IN1L -> LINE1L
 *                     |  IN1R -> LINE1R
 *                     |  IN2L -> LINE2L
 *                     |  IN2R -> LINE2R
 *                     |  MIC3L/R -> N/A
 *   truncated internal functionality in
 *   accordance with documentation
 *  ---------------------------------------
 *
 *  Hence the machine layer should disable unsupported inputs/outputs by
 *  snd_soc_dapm_disable_pin(codec, "MONO_LOUT"), etc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/tlv320aic3105.h>

#include "tlv320aic3105.h"

#define AIC3105_NUM_SUPPLIES	4
static const char *aic3105_supply_names[AIC3105_NUM_SUPPLIES] = {
	"IOVDD",	/* I/O Voltage */
	"DVDD",		/* Digital Core Voltage */
	"AVDD",		/* Analog DAC Voltage */
	"DRVDD",	/* ADC Analog and Output Driver Voltage */
};

/* codec private data */
struct aic3105_priv {
	struct snd_soc_codec codec;
	struct regulator_bulk_data supplies[AIC3105_NUM_SUPPLIES];
	unsigned int sysclk;
	int master;
	int gpio_reset;
};

/*
 * AIC3105 register cache
 * We can't read the AIC3105 register space when we are
 * using 2 wire for device control, so we cache them instead.
 * There is no point in caching the reset register
 */
static const u8 aic3105_reg[AIC3105_CACHEREGNUM] = {
	0x00, 0x00, 0x00, 0x10,	/* 0 */
	0x04, 0x00, 0x00, 0x00,	/* 4 */
	0x00, 0x00, 0x00, 0x01,	/* 8 */
	0x00, 0x00, 0x00, 0x80,	/* 12 */
	0x80, 0xff, 0xff, 0x78,	/* 16 */
	0x78, 0x78, 0x78, 0x78,	/* 20 */
	0x78, 0x00, 0x00, 0xfe,	/* 24 */
	0x00, 0x00, 0xfe, 0x00,	/* 28 */
	0x18, 0x18, 0x00, 0x00,	/* 32 */
	0x00, 0x00, 0x00, 0x00,	/* 36 */
	0x00, 0x00, 0x00, 0x80,	/* 40 */
	0x80, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x04,	/* 48 */
	0x00, 0x00, 0x00, 0x00,	/* 52 */
	0x00, 0x00, 0x04, 0x00,	/* 56 */
	0x00, 0x00, 0x00, 0x00,	/* 60 */
	0x00, 0x04, 0x00, 0x00,	/* 64 */
	0x00, 0x00, 0x00, 0x00,	/* 68 */
	0x04, 0x00, 0x00, 0x00,	/* 72 */
	0x00, 0x00, 0x00, 0x00,	/* 76 */
	0x00, 0x00, 0x00, 0x00,	/* 80 */
	0x00, 0x00, 0x00, 0x00,	/* 84 */
	0x00, 0x00, 0x00, 0x00,	/* 88 */
	0x00, 0x00, 0x00, 0x00,	/* 92 */
	0x00, 0x00, 0x00, 0x00,	/* 96 */
	0x00, 0x00, 0x02,	/* 100 */
};

/*
 * read aic3105 register cache
 */
static inline unsigned int aic3105_read_reg_cache(struct snd_soc_codec *codec,
						unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= AIC3105_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write aic3105 register cache
 */
static inline void aic3105_write_reg_cache(struct snd_soc_codec *codec,
					 u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= AIC3105_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the aic3105 register space
 */
static int aic3105_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D8 aic3105 register offset
	 *   D7...D0 register data
	 */
	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	aic3105_write_reg_cache(codec, data[0], data[1]);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

/*
 * read from the aic3105 register space
 */
static int aic3105_read(struct snd_soc_codec *codec, unsigned int reg,
		      u8 *value)
{
	*value = reg & 0xff;

	value[0] = i2c_smbus_read_byte_data(codec->control_data, value[0]);

	aic3105_write_reg_cache(codec, reg, *value);
	return 0;
}

#define SOC_DAPM_SINGLE_AIC3105(xname, reg, shift, mask, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_dapm_get_volsw, .put = snd_soc_dapm_put_volsw_aic3105, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, mask, invert) }

/*
 * All input lines are connected when !0xf and disconnected with 0xf bit field,
 * so we have to use specific dapm_put call for input mixer
 */
static int snd_soc_dapm_put_volsw_aic3105(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned short val, val_mask;
	int ret;
	struct snd_soc_dapm_path *path;
	int found = 0;

	val = (ucontrol->value.integer.value[0] & mask);

	mask = 0xf;
	if (val)
		val = mask;

	if (invert)
		val = mask - val;
	val_mask = mask << shift;
	val = val << shift;

	mutex_lock(&widget->codec->mutex);

	if (snd_soc_test_bits(widget->codec, reg, val_mask, val)) {
		/* find dapm widget path assoc with kcontrol */
		list_for_each_entry(path, &widget->codec->dapm_paths, list) {
			if (path->kcontrol != kcontrol)
				continue;

			/* found, now check type */
			found = 1;
			if (val)
				/* new connection */
				path->connect = invert ? 0 : 1;
			else
				/* old connection must be powered down */
				path->connect = invert ? 1 : 0;
			break;
		}

		if (found)
			snd_soc_dapm_sync(widget->codec);
	}

	ret = snd_soc_update_bits(widget->codec, reg, val_mask, val);

	mutex_unlock(&widget->codec->mutex);
	return ret;
}

static const char *aic3105_left_dac_mux[] = { "DAC_L1", "DAC_L3", "DAC_L2" };
static const char *aic3105_right_dac_mux[] = { "DAC_R1", "DAC_R3", "DAC_R2" };
static const char *aic3105_left_hpcom_mux[] =
    { "differential of HPLOUT", "constant VCM", "single-ended" };
static const char *aic3105_right_hpcom_mux[] =
    { "differential of HPROUT", "constant VCM", "single-ended",
      "differential of HPLCOM", "external feedback" };

static const char *aic3105_adc_hpf[] =
    { "Disabled", "0.0045xFs", "0.0125xFs", "0.025xFs" };

#define LDAC_ENUM	0
#define RDAC_ENUM	1
#define LHPCOM_ENUM	2
#define RHPCOM_ENUM	3
#define ADC_HPF_ENUM 4

static const struct soc_enum aic3105_enum[] = {
	SOC_ENUM_SINGLE(DAC_LINE_MUX, 6, 3, aic3105_left_dac_mux),
	SOC_ENUM_SINGLE(DAC_LINE_MUX, 4, 3, aic3105_right_dac_mux),
	SOC_ENUM_SINGLE(HPLCOM_CFG, 4, 3, aic3105_left_hpcom_mux),
	SOC_ENUM_SINGLE(HPRCOM_CFG, 3, 5, aic3105_right_hpcom_mux),
	SOC_ENUM_DOUBLE(AIC3105_CODEC_DFILT_CTRL, 6, 4, 4, aic3105_adc_hpf),
};

/*
 * DAC digital volumes. From -63.5 to 0 dB in 0.5 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, -6350, 50, 0);
/* ADC PGA gain volumes. From 0 to 59.5 dB in 0.5 dB steps */
static DECLARE_TLV_DB_SCALE(adc_tlv, 0, 50, 0);
/*
 * Output stage volumes. From -78.3 to 0 dB. Muted below -78.3 dB.
 * Step size is approximately 0.5 dB over most of the scale but increasing
 * near the very low levels.
 * Define dB scale so that it is mostly correct for range about -55 to 0 dB
 * but having increasing dB difference below that (and where it doesn't count
 * so much). This setting shows -50 dB (actual is -50.3 dB) for register
 * value 100 and -58.5 dB (actual is -78.3 dB) for register value 117.
 */
static DECLARE_TLV_DB_SCALE(output_stage_tlv, -5900, 50, 1);
static DECLARE_TLV_DB_SCALE(output_stage_gain, 0, 1, 0);

static const struct snd_kcontrol_new aic3105_snd_controls[] = {
	/* Output */
	SOC_DOUBLE_R_TLV("PCM Playback Volume",	LDAC_VOL, RDAC_VOL, 0, 0x7f, 1, dac_tlv),

	SOC_DOUBLE_R_TLV("HP DAC_1 Playback Volume", DACL1_2_HPLOUT_VOL, DACR1_2_HPROUT_VOL, 0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("HP Ana Playback Volume", HPLOUT_CTRL, HPROUT_CTRL, 4, 9, 0, output_stage_gain),
	SOC_DOUBLE_R("HP DAC Playback Switch", HPLOUT_CTRL, HPROUT_CTRL, 3, 0x01, 0),

	SOC_DOUBLE_R_TLV("HPCOM DAC_1 Playback Volume",	DACL1_2_HPLCOM_VOL, DACR1_2_HPRCOM_VOL,	0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("HPCOM Ana Playback Volume", HPLCOM_CTRL, HPRCOM_CTRL, 4, 9, 0, output_stage_gain),
	SOC_DOUBLE_R("HPCOM DAC Playback Switch", HPLCOM_CTRL, HPRCOM_CTRL, 3, 0x01, 0),

#ifdef ENABLE_EXTRA_CTL
	SOC_DOUBLE_R_TLV("Line DAC Playback Volume",
			 DACL1_2_LLOPM_VOL, DACR1_2_RLOPM_VOL,
			 0, 118, 1, output_stage_tlv),
	SOC_SINGLE("LineL Playback Switch", LLOPM_CTRL, 3, 0x01, 0),
	SOC_SINGLE("LineR Playback Switch", RLOPM_CTRL, 3, 0x01, 0),
	SOC_DOUBLE_R_TLV("LineL DAC Playback Volume",
			 DACL1_2_LLOPM_VOL, DACR1_2_LLOPM_VOL,
			 0, 118, 1, output_stage_tlv),
	SOC_SINGLE_TLV("LineL Left PGA Bypass Playback Volume",
		       PGAL_2_LLOPM_VOL, 0, 118, 1, output_stage_tlv),
	SOC_SINGLE_TLV("LineR Right PGA Bypass Playback Volume",
		       PGAR_2_RLOPM_VOL, 0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("LineL Line2 Bypass Playback Volume",
			 LINE2L_2_LLOPM_VOL, LINE2R_2_LLOPM_VOL,
			 0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("LineR Line2 Bypass Playback Volume",
			 LINE2L_2_RLOPM_VOL, LINE2R_2_RLOPM_VOL,
			 0, 118, 1, output_stage_tlv),


	SOC_DOUBLE_R_TLV("HP Right PGA Bypass Playback Volume",
			 PGAR_2_HPLOUT_VOL, PGAR_2_HPROUT_VOL,
			 0, 118, 1, output_stage_tlv),
	SOC_SINGLE_TLV("HPL PGA Bypass Playback Volume",
		       PGAL_2_HPLOUT_VOL, 0, 118, 1, output_stage_tlv),
	SOC_SINGLE_TLV("HPR PGA Bypass Playback Volume",
		       PGAL_2_HPROUT_VOL, 0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("HP Line2 Bypass Playback Volume",
			 LINE2L_2_HPLOUT_VOL, LINE2R_2_HPROUT_VOL,
			 0, 118, 1, output_stage_tlv),

	SOC_SINGLE_TLV("HPLCOM PGA Bypass Playback Volume",
		       PGAL_2_HPLCOM_VOL, 0, 118, 1, output_stage_tlv),
	SOC_SINGLE_TLV("HPRCOM PGA Bypass Playback Volume",
		       PGAL_2_HPRCOM_VOL, 0, 118, 1, output_stage_tlv),
	SOC_DOUBLE_R_TLV("HPCOM Line2 Bypass Playback Volume",
			 LINE2L_2_HPLCOM_VOL, LINE2R_2_HPRCOM_VOL,
			 0, 118, 1, output_stage_tlv),
#endif
	/*
	 * Note: enable Automatic input Gain Controller with care. It can
	 * adjust PGA to max value when ADC is on and will never go back.
	*/
	SOC_DOUBLE_R("AGC Switch", LAGC_CTRL_A, RAGC_CTRL_A, 7, 0x01, 0),

	/* Input */
	SOC_DOUBLE_R_TLV("PGA Capture Volume", LADC_VOL, RADC_VOL, 0, 119, 0, adc_tlv),
	SOC_DOUBLE_R("PGA Capture Switch", LADC_VOL, RADC_VOL, 7, 0x01, 1),

	SOC_ENUM("ADC HPF Cut-off", aic3105_enum[ADC_HPF_ENUM]),
};

/* Left DAC Mux */
static const struct snd_kcontrol_new aic3105_left_dac_mux_controls =
SOC_DAPM_ENUM("Route", aic3105_enum[LDAC_ENUM]);

/* Right DAC Mux */
static const struct snd_kcontrol_new aic3105_right_dac_mux_controls =
SOC_DAPM_ENUM("Route", aic3105_enum[RDAC_ENUM]);

/* Left HPCOM Mux */
static const struct snd_kcontrol_new aic3105_left_hpcom_mux_controls =
SOC_DAPM_ENUM("Route", aic3105_enum[LHPCOM_ENUM]);

/* Right HPCOM Mux */
static const struct snd_kcontrol_new aic3105_right_hpcom_mux_controls =
SOC_DAPM_ENUM("Route", aic3105_enum[RHPCOM_ENUM]);

/* Left PGA Bypass Mixer */
static const struct snd_kcontrol_new aic3105_left_pga_bp_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", PGAL_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", PGAL_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPL Switch", PGAL_2_HPLOUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPR Switch", PGAL_2_HPROUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPLCOM Switch", PGAL_2_HPLCOM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPRCOM Switch", PGAL_2_HPRCOM_VOL, 7, 1, 0),
};

/* Right PGA Bypass Mixer */
static const struct snd_kcontrol_new aic3105_right_pga_bp_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", PGAR_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", PGAR_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPL Switch", PGAR_2_HPLOUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPR Switch", PGAR_2_HPROUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPLCOM Switch", PGAR_2_HPLCOM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPRCOM Switch", PGAR_2_HPRCOM_VOL, 7, 1, 0),
};

/* Left DAC_L1 Mixer */
static const struct snd_kcontrol_new aic3105_left_dac_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", DACL1_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", DACL1_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HP Switch", DACL1_2_HPLOUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPCOM Switch", DACL1_2_HPLCOM_VOL, 7, 1, 0),
};

/* Right DAC_R1 Mixer */
static const struct snd_kcontrol_new aic3105_right_dac_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", DACR1_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", DACR1_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HP Switch", DACR1_2_HPROUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPCOM Switch", DACR1_2_HPRCOM_VOL, 7, 1, 0),
};

/* Left PGA Mixer */
static const struct snd_kcontrol_new aic3105_left_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE_AIC3105("Line1L Switch", LINE1L_2_LADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Line1R Switch", LINE1R_2_LADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Line2L Switch", LINE2L_2_LADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Mic3L Switch", MIC3LR_2_LADC_CTRL, 4, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Mic3R Switch", MIC3LR_2_LADC_CTRL, 0, 1, 1),
};

/* Right PGA Mixer */
static const struct snd_kcontrol_new aic3105_right_pga_mixer_controls[] = {
	SOC_DAPM_SINGLE_AIC3105("Line1R Switch", LINE1R_2_RADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Line1L Switch", LINE1L_2_RADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Line2R Switch", LINE2R_2_RADC_CTRL, 3, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Mic3L Switch", MIC3LR_2_RADC_CTRL, 4, 1, 1),
	SOC_DAPM_SINGLE_AIC3105("Mic3R Switch", MIC3LR_2_RADC_CTRL, 0, 1, 1),
};


/* Left Line2 Bypass Mixer */
static const struct snd_kcontrol_new aic3105_left_line2_bp_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", LINE2L_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", LINE2L_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HP Switch", LINE2L_2_HPLOUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPLCOM Switch", LINE2L_2_HPLCOM_VOL, 7, 1, 0),
};

/* Right Line2 Bypass Mixer */
static const struct snd_kcontrol_new aic3105_right_line2_bp_mixer_controls[] = {
	SOC_DAPM_SINGLE("LineL Switch", LINE2R_2_LLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("LineR Switch", LINE2R_2_RLOPM_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HP Switch", LINE2R_2_HPROUT_VOL, 7, 1, 0),
	SOC_DAPM_SINGLE("HPRCOM Switch", LINE2R_2_HPRCOM_VOL, 7, 1, 0),
};

static const struct snd_soc_dapm_widget aic3105_dapm_widgets[] = {
	/* Left DAC to Left Outputs */
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", DAC_PWR, 7, 0),
	SND_SOC_DAPM_MUX("Left DAC Mux", SND_SOC_NOPM, 0, 0,
			 &aic3105_left_dac_mux_controls),
	SND_SOC_DAPM_MIXER("Left DAC_L1 Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_left_dac_mixer_controls[0],
			   ARRAY_SIZE(aic3105_left_dac_mixer_controls)),
	SND_SOC_DAPM_MUX("Left HPCOM Mux", SND_SOC_NOPM, 0, 0,
			 &aic3105_left_hpcom_mux_controls),
	SND_SOC_DAPM_PGA("Left Line Out", LLOPM_CTRL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left HP Out", HPLOUT_CTRL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left HP Com", HPLCOM_CTRL, 0, 0, NULL, 0),

	/* Right DAC to Right Outputs */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", DAC_PWR, 6, 0),
	SND_SOC_DAPM_MUX("Right DAC Mux", SND_SOC_NOPM, 0, 0,
			 &aic3105_right_dac_mux_controls),
	SND_SOC_DAPM_MIXER("Right DAC_R1 Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_right_dac_mixer_controls[0],
			   ARRAY_SIZE(aic3105_right_dac_mixer_controls)),
	SND_SOC_DAPM_MUX("Right HPCOM Mux", SND_SOC_NOPM, 0, 0,
			 &aic3105_right_hpcom_mux_controls),
	SND_SOC_DAPM_PGA("Right Line Out", RLOPM_CTRL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right HP Out", HPROUT_CTRL, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right HP Com", HPRCOM_CTRL, 0, 0, NULL, 0),


	/* Inputs to Left ADC */
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", LINE1L_2_LADC_CTRL, 2, 0),
	SND_SOC_DAPM_MIXER("Left PGA Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_left_pga_mixer_controls[0],
			   ARRAY_SIZE(aic3105_left_pga_mixer_controls)),




	/* Inputs to Right ADC */
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture",
			 LINE1R_2_RADC_CTRL, 2, 0),
	SND_SOC_DAPM_MIXER("Right PGA Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_right_pga_mixer_controls[0],
			   ARRAY_SIZE(aic3105_right_pga_mixer_controls)),


	/* Mic Bias */
	SND_SOC_DAPM_REG(snd_soc_dapm_micbias, "Mic Bias 2V",
			 MICBIAS_CTRL, 6, 3, 1, 0),
	SND_SOC_DAPM_REG(snd_soc_dapm_micbias, "Mic Bias 2.5V",
			 MICBIAS_CTRL, 6, 3, 2, 0),
	SND_SOC_DAPM_REG(snd_soc_dapm_micbias, "Mic Bias AVDD",
			 MICBIAS_CTRL, 6, 3, 3, 0),

	/* Left PGA to Left Output bypass */
	SND_SOC_DAPM_MIXER("Left PGA Bypass Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_left_pga_bp_mixer_controls[0],
			   ARRAY_SIZE(aic3105_left_pga_bp_mixer_controls)),

	/* Right PGA to Right Output bypass */
	SND_SOC_DAPM_MIXER("Right PGA Bypass Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_right_pga_bp_mixer_controls[0],
			   ARRAY_SIZE(aic3105_right_pga_bp_mixer_controls)),

	/* Left Line2 to Left Output bypass */
	SND_SOC_DAPM_MIXER("Left Line2 Bypass Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_left_line2_bp_mixer_controls[0],
			   ARRAY_SIZE(aic3105_left_line2_bp_mixer_controls)),

	/* Right Line2 to Right Output bypass */
	SND_SOC_DAPM_MIXER("Right Line2 Bypass Mixer", SND_SOC_NOPM, 0, 0,
			   &aic3105_right_line2_bp_mixer_controls[0],
			   ARRAY_SIZE(aic3105_right_line2_bp_mixer_controls)),

	SND_SOC_DAPM_OUTPUT("LLOUT"),
	SND_SOC_DAPM_OUTPUT("RLOUT"),
	SND_SOC_DAPM_OUTPUT("HPLOUT"),
	SND_SOC_DAPM_OUTPUT("HPROUT"),
	SND_SOC_DAPM_OUTPUT("HPLCOM"),
	SND_SOC_DAPM_OUTPUT("HPRCOM"),

	SND_SOC_DAPM_INPUT("MIC3L"),
	SND_SOC_DAPM_INPUT("MIC3R"),
	SND_SOC_DAPM_INPUT("LINE1L"),
	SND_SOC_DAPM_INPUT("LINE1R"),
	SND_SOC_DAPM_INPUT("LINE2L"),
	SND_SOC_DAPM_INPUT("LINE2R"),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Left Output */
	{"Left DAC Mux", "DAC_L1", "Left DAC"},
	{"Left DAC Mux", "DAC_L2", "Left DAC"},
	{"Left DAC Mux", "DAC_L3", "Left DAC"},

	{"Left DAC_L1 Mixer", "LineL Switch", "Left DAC Mux"},
	{"Left DAC_L1 Mixer", "LineR Switch", "Left DAC Mux"},
	{"Left DAC_L1 Mixer", "HP Switch", "Left DAC Mux"},
	{"Left DAC_L1 Mixer", "HPCOM Switch", "Left DAC Mux"},
	{"Left Line Out", NULL, "Left DAC Mux"},
	{"Left HP Out", NULL, "Left DAC Mux"},

	{"Left HPCOM Mux", "differential of HPLOUT", "Left DAC_L1 Mixer"},
	{"Left HPCOM Mux", "constant VCM", "Left DAC_L1 Mixer"},
	{"Left HPCOM Mux", "single-ended", "Left DAC_L1 Mixer"},

	{"Left Line Out", NULL, "Left DAC_L1 Mixer"},
	{"Left HP Out", NULL, "Left DAC_L1 Mixer"},
	{"Left HP Com", NULL, "Left HPCOM Mux"},

	{"LLOUT", NULL, "Left Line Out"},
	{"LLOUT", NULL, "Left Line Out"},
	{"HPLOUT", NULL, "Left HP Out"},
	{"HPLCOM", NULL, "Left HP Com"},

	/* Right Output */
	{"Right DAC Mux", "DAC_R1", "Right DAC"},
	{"Right DAC Mux", "DAC_R2", "Right DAC"},
	{"Right DAC Mux", "DAC_R3", "Right DAC"},

	{"Right DAC_R1 Mixer", "LineL Switch", "Right DAC Mux"},
	{"Right DAC_R1 Mixer", "LineR Switch", "Right DAC Mux"},
	{"Right DAC_R1 Mixer", "HP Switch", "Right DAC Mux"},
	{"Right DAC_R1 Mixer", "HPCOM Switch", "Right DAC Mux"},
	{"Right Line Out", NULL, "Right DAC Mux"},
	{"Right HP Out", NULL, "Right DAC Mux"},

	{"Right HPCOM Mux", "differential of HPROUT", "Right DAC_R1 Mixer"},
	{"Right HPCOM Mux", "constant VCM", "Right DAC_R1 Mixer"},
	{"Right HPCOM Mux", "single-ended", "Right DAC_R1 Mixer"},
	{"Right HPCOM Mux", "differential of HPLCOM", "Right DAC_R1 Mixer"},
	{"Right HPCOM Mux", "external feedback", "Right DAC_R1 Mixer"},

	{"Right Line Out", NULL, "Right DAC_R1 Mixer"},
	{"Right HP Out", NULL, "Right DAC_R1 Mixer"},
	{"Right HP Com", NULL, "Right HPCOM Mux"},

	{"RLOUT", NULL, "Right Line Out"},
	{"RLOUT", NULL, "Right Line Out"},
	{"HPROUT", NULL, "Right HP Out"},
	{"HPRCOM", NULL, "Right HP Com"},


	/* Left Input */

	{"Left PGA Mixer", "Line1L Switch", "LINE1L"},
	{"Left PGA Mixer", "Line1R Switch", "LINE1R"},
	{"Left PGA Mixer", "Line2L Switch", "LINE2L"},
	{"Left PGA Mixer", "Mic3L Switch", "MIC3L"},
	{"Left PGA Mixer", "Mic3R Switch", "MIC3R"},

	{"Left ADC", NULL, "Left PGA Mixer"},

	/* Right Input */


	{"Right PGA Mixer", "Line1L Switch", "LINE1L"},
	{"Right PGA Mixer", "Line1R Switch", "LINE1R"},
	{"Right PGA Mixer", "Line2R Switch", "LINE2R"},
	{"Right PGA Mixer", "Mic3L Switch", "MIC3L"},
	{"Right PGA Mixer", "Mic3R Switch", "MIC3R"},

	{"Right ADC", NULL, "Right PGA Mixer"},


};

static int aic3105_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, aic3105_dapm_widgets,
				  ARRAY_SIZE(aic3105_dapm_widgets));

	/* set up audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	return 0;
}

static int aic3105_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct aic3105_priv *aic3105 = snd_soc_codec_get_drvdata(codec);
	int codec_clk = 0, bypass_pll = 0, fsref, last_clk = 0;
	u8 data, j, r, p, pll_q, pll_p = 1, pll_r = 1, pll_j = 1;
	u16 d, pll_d = 1;
	u8 reg;
	int clk;

	/* select data word length */
	data =
	    aic3105_read_reg_cache(codec, AIC3105_ASD_INTF_CTRLB) & (~(0x3 << 4));
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x01 << 4);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x02 << 4);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x03 << 4);
		break;
	}
	aic3105_write(codec, AIC3105_ASD_INTF_CTRLB, data);

	/* Fsref can be 44100 or 48000 */
	fsref = (params_rate(params) % 11025 == 0) ? 44100 : 48000;

	/* Try to find a value for Q which allows us to bypass the PLL and
	 * generate CODEC_CLK directly. */
	for (pll_q = 2; pll_q < 18; pll_q++)
		if (aic3105->sysclk / (128 * pll_q) == fsref) {
			bypass_pll = 1;
			break;
		}

	if (bypass_pll) {
		pll_q &= 0xf;
		aic3105_write(codec, AIC3105_PLL_PROGA_REG, pll_q << PLLQ_SHIFT);
		aic3105_write(codec, AIC3105_GPIOB_REG, CODEC_CLKIN_CLKDIV);
		/* disable PLL if it is bypassed */
		reg = aic3105_read_reg_cache(codec, AIC3105_PLL_PROGA_REG);
		aic3105_write(codec, AIC3105_PLL_PROGA_REG, reg & ~PLL_ENABLE);

	} else {
		aic3105_write(codec, AIC3105_GPIOB_REG, CODEC_CLKIN_PLLDIV);
		/* enable PLL when it is used */
		reg = aic3105_read_reg_cache(codec, AIC3105_PLL_PROGA_REG);
		aic3105_write(codec, AIC3105_PLL_PROGA_REG, reg | PLL_ENABLE);
	}

	/* Route Left DAC to left channel input and
	 * right DAC to right channel input */
	data = (LDAC2LCH | RDAC2RCH);
	data |= (fsref == 44100) ? FSREF_44100 : FSREF_48000;
	if (params_rate(params) >= 64000)
		data |= DUAL_RATE_MODE;
	aic3105_write(codec, AIC3105_CODEC_DATAPATH_REG, data);

	/* codec sample rate select */
	data = (fsref * 20) / params_rate(params);
	if (params_rate(params) < 64000)
		data /= 2;
	data /= 5;
	data -= 2;
	data |= (data << 4);
	aic3105_write(codec, AIC3105_SAMPLE_RATE_SEL_REG, data);

	if (bypass_pll)
		return 0;

	/* Use PLL, compute apropriate setup for j, d, r and p, the closest
	 * one wins the game. Try with d==0 first, next with d!=0.
	 * Constraints for j are according to the datasheet.
	 * The sysclk is divided by 1000 to prevent integer overflows.
	 */

	codec_clk = (2048 * fsref) / (aic3105->sysclk / 1000);

	for (r = 1; r <= 16; r++)
		for (p = 1; p <= 8; p++) {
			for (j = 4; j <= 55; j++) {
				/* This is actually 1000*((j+(d/10000))*r)/p
				 * The term had to be converted to get
				 * rid of the division by 10000; d = 0 here
				 */
				int tmp_clk = (1000 * j * r) / p;

				/* Check whether this values get closer than
				 * the best ones we had before
				 */
				if (abs(codec_clk - tmp_clk) <
					abs(codec_clk - last_clk)) {
					pll_j = j; pll_d = 0;
					pll_r = r; pll_p = p;
					last_clk = tmp_clk;
				}

				/* Early exit for exact matches */
				if (tmp_clk == codec_clk)
					goto found;
			}
		}

	/* try with d != 0 */
	for (p = 1; p <= 8; p++) {
		j = codec_clk * p / 1000;

		if (j < 4 || j > 11)
			continue;

		/* do not use codec_clk here since we'd loose precision */
		d = ((2048 * p * fsref) - j * aic3105->sysclk)
			* 100 / (aic3105->sysclk/100);

		clk = (10000 * j + d) / (10 * p);

		/* check whether this values get closer than the best
		 * ones we had before */
		if (abs(codec_clk - clk) < abs(codec_clk - last_clk)) {
			pll_j = j; pll_d = d; pll_r = 1; pll_p = p;
			last_clk = clk;
		}

		/* Early exit for exact matches */
		if (clk == codec_clk)
			goto found;
	}

	if (last_clk == 0) {
		printk(KERN_ERR "%s(): unable to setup PLL\n", __func__);
		return -EINVAL;
	}

found:
	data = aic3105_read_reg_cache(codec, AIC3105_PLL_PROGA_REG);
	aic3105_write(codec, AIC3105_PLL_PROGA_REG, data | (pll_p << PLLP_SHIFT));
	aic3105_write(codec, AIC3105_OVRF_STATUS_AND_PLLR_REG, pll_r << PLLR_SHIFT);
	aic3105_write(codec, AIC3105_PLL_PROGB_REG, pll_j << PLLJ_SHIFT);
	aic3105_write(codec, AIC3105_PLL_PROGC_REG, (pll_d >> 6) << PLLD_MSB_SHIFT);
	aic3105_write(codec, AIC3105_PLL_PROGD_REG,
		    (pll_d & 0x3F) << PLLD_LSB_SHIFT);

	return 0;
}

static int aic3105_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 ldac_reg = aic3105_read_reg_cache(codec, LDAC_VOL) & ~MUTE_ON;
	u8 rdac_reg = aic3105_read_reg_cache(codec, RDAC_VOL) & ~MUTE_ON;

	if (mute) {
		aic3105_write(codec, LDAC_VOL, ldac_reg | MUTE_ON);
		aic3105_write(codec, RDAC_VOL, rdac_reg | MUTE_ON);
	} else {
		aic3105_write(codec, LDAC_VOL, ldac_reg);
		aic3105_write(codec, RDAC_VOL, rdac_reg);
	}

	return 0;
}

static int aic3105_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3105_priv *aic3105 = snd_soc_codec_get_drvdata(codec);

	aic3105->sysclk = freq;
	return 0;
}

static int aic3105_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic3105_priv *aic3105 = snd_soc_codec_get_drvdata(codec);
	u8 iface_areg, iface_breg;
	int delay = 0;

	iface_areg = aic3105_read_reg_cache(codec, AIC3105_ASD_INTF_CTRLA) & 0x3f;
	iface_breg = aic3105_read_reg_cache(codec, AIC3105_ASD_INTF_CTRLB) & 0x3f;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic3105->master = 1;
		iface_areg |= BIT_CLK_MASTER | WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic3105->master = 0;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * match both interface format and signal polarities since they
	 * are fixed
	 */
	switch (fmt & (SND_SOC_DAIFMT_FORMAT_MASK |
		       SND_SOC_DAIFMT_INV_MASK)) {
	case (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF):
		break;
	case (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF):
		delay = 1;
	case (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF):
		iface_breg |= (0x01 << 6);
		break;
	case (SND_SOC_DAIFMT_RIGHT_J | SND_SOC_DAIFMT_NB_NF):
		iface_breg |= (0x02 << 6);
		break;
	case (SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF):
		iface_breg |= (0x03 << 6);
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	aic3105_write(codec, AIC3105_ASD_INTF_CTRLA, iface_areg);
	aic3105_write(codec, AIC3105_ASD_INTF_CTRLB, iface_breg);
	aic3105_write(codec, AIC3105_ASD_INTF_CTRLC, delay);

	return 0;
}

static int aic3105_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	struct aic3105_priv *aic3105 = snd_soc_codec_get_drvdata(codec);
	u8 reg;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		if (aic3105->master) {
			/* enable pll */
			reg = aic3105_read_reg_cache(codec, AIC3105_PLL_PROGA_REG);
			aic3105_write(codec, AIC3105_PLL_PROGA_REG,
				    reg | PLL_ENABLE);
		}
		break;
	case SND_SOC_BIAS_STANDBY:
		/* fall through and disable pll */
	case SND_SOC_BIAS_OFF:
		if (aic3105->master) {
			/* disable pll */
			reg = aic3105_read_reg_cache(codec, AIC3105_PLL_PROGA_REG);
			aic3105_write(codec, AIC3105_PLL_PROGA_REG,
				    reg & ~PLL_ENABLE);
		}
		break;
	}
	codec->bias_level = level;

	return 0;
}


int aic3105_headset_detected(struct snd_soc_codec *codec)
{
	u8 val;
	aic3105_read(codec, AIC3105_HEADSET_DETECT_CTRL_B, &val);
	return (val >> 4) & 1;
}
EXPORT_SYMBOL_GPL(aic3105_headset_detected);

int aic3105_button_pressed(struct snd_soc_codec *codec)
{
	u8 val;
	aic3105_read(codec, AIC3105_HEADSET_DETECT_CTRL_B, &val);
	return (val >> 5) & 1;
}
EXPORT_SYMBOL_GPL(aic3105_button_pressed);

#define AIC3105_RATES	SNDRV_PCM_RATE_8000_96000
#define AIC3105_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops aic3105_dai_ops = {
	.hw_params	= aic3105_hw_params,
	.digital_mute	= aic3105_mute,
	.set_sysclk	= aic3105_set_dai_sysclk,
	.set_fmt	= aic3105_set_dai_fmt,
};

struct snd_soc_dai aic3105_dai = {
	.name = "tlv320aic3105",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3105_RATES,
		.formats = AIC3105_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AIC3105_RATES,
		.formats = AIC3105_FORMATS,},
	.ops = &aic3105_dai_ops,
};
EXPORT_SYMBOL_GPL(aic3105_dai);

static int aic3105_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	aic3105_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int aic3105_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u8 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(aic3105_reg); i++) {
		data[0] = i;
		data[1] = cache[i];
		codec->hw_write(codec->control_data, data, 2);
	}

	aic3105_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

/*
 * initialise the AIC3105 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int aic3105_init(struct snd_soc_codec *codec)
{
	int reg;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "tlv320aic3105";
	codec->owner = THIS_MODULE;
	codec->read = aic3105_read_reg_cache;
	codec->write = aic3105_write;
	codec->set_bias_level = aic3105_set_bias_level;
	codec->dai = &aic3105_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(aic3105_reg);
	codec->reg_cache = kmemdup(aic3105_reg, sizeof(aic3105_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;

	aic3105_write(codec, AIC3105_PAGE_SELECT, PAGE0_SELECT);
	aic3105_write(codec, AIC3105_RESET, SOFT_RESET);

	/* DAC default volume and mute */
	aic3105_write(codec, AIC3105_ASD_INTF_CTRLB, 0x01);	// Soft mute
	aic3105_write(codec, LDAC_VOL, DEFAULT_VOL | MUTE_ON);
	aic3105_write(codec, RDAC_VOL, DEFAULT_VOL | MUTE_ON);

	/* DAC to HP default volume and route to Output mixer */
	aic3105_write(codec, DACL1_2_HPLOUT_VOL, DEFAULT_VOL | ROUTE_ON);
	aic3105_write(codec, DACR1_2_HPROUT_VOL, DEFAULT_VOL | ROUTE_ON);
	aic3105_write(codec, DACL1_2_HPLCOM_VOL, DEFAULT_VOL | ROUTE_ON);
	aic3105_write(codec, DACR1_2_HPRCOM_VOL, DEFAULT_VOL | ROUTE_ON);
	/* DAC to Line Out default volume and route to Output mixer */
	aic3105_write(codec, DACL1_2_LLOPM_VOL, DEFAULT_VOL | ROUTE_ON);
	aic3105_write(codec, DACR1_2_RLOPM_VOL, DEFAULT_VOL | ROUTE_ON);

	/* unmute all outputs */
	reg = aic3105_read_reg_cache(codec, LLOPM_CTRL);
	aic3105_write(codec, LLOPM_CTRL, reg | UNMUTE);
	reg = aic3105_read_reg_cache(codec, RLOPM_CTRL);
	aic3105_write(codec, RLOPM_CTRL, reg | UNMUTE);
	reg = aic3105_read_reg_cache(codec, HPLOUT_CTRL);
	aic3105_write(codec, HPLOUT_CTRL, reg | UNMUTE);
	reg = aic3105_read_reg_cache(codec, HPROUT_CTRL);
	aic3105_write(codec, HPROUT_CTRL, reg | UNMUTE);
	reg = aic3105_read_reg_cache(codec, HPLCOM_CTRL);
	aic3105_write(codec, HPLCOM_CTRL, reg | UNMUTE);
	reg = aic3105_read_reg_cache(codec, HPRCOM_CTRL);
	aic3105_write(codec, HPRCOM_CTRL, reg | UNMUTE);

	/* ADC default volume and unmute */
	aic3105_write(codec, LADC_VOL, DEFAULT_GAIN);
	aic3105_write(codec, RADC_VOL, DEFAULT_GAIN);
	/* By default route Line1 to ADC PGA mixer */
	aic3105_write(codec, LINE1L_2_LADC_CTRL, 0x0);
	aic3105_write(codec, LINE1R_2_RADC_CTRL, 0x0);

	/* PGA to HP Bypass default volume, disconnect from Output Mixer */
	aic3105_write(codec, PGAL_2_HPLOUT_VOL, DEFAULT_VOL);
	aic3105_write(codec, PGAR_2_HPROUT_VOL, DEFAULT_VOL);
	aic3105_write(codec, PGAL_2_HPLCOM_VOL, DEFAULT_VOL);
	aic3105_write(codec, PGAR_2_HPRCOM_VOL, DEFAULT_VOL);
	/* PGA to Line Out default volume, disconnect from Output Mixer */
	aic3105_write(codec, PGAL_2_LLOPM_VOL, DEFAULT_VOL);
	aic3105_write(codec, PGAR_2_RLOPM_VOL, DEFAULT_VOL);

	/* Line2 to HP Bypass default volume, disconnect from Output Mixer */
	aic3105_write(codec, LINE2L_2_HPLOUT_VOL, DEFAULT_VOL);
	aic3105_write(codec, LINE2R_2_HPROUT_VOL, DEFAULT_VOL);
	aic3105_write(codec, LINE2L_2_HPLCOM_VOL, DEFAULT_VOL);
	aic3105_write(codec, LINE2R_2_HPRCOM_VOL, DEFAULT_VOL);
	/* Line2 Line Out default volume, disconnect from Output Mixer */
	aic3105_write(codec, LINE2L_2_LLOPM_VOL, DEFAULT_VOL);
	aic3105_write(codec, LINE2R_2_RLOPM_VOL, DEFAULT_VOL);

	/* off, with power on */
	aic3105_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static struct snd_soc_codec *aic3105_codec;

static int aic3105_register(struct snd_soc_codec *codec)
{
	int ret;

	ret = aic3105_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to initialise device\n");
		return ret;
	}

	aic3105_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret) {
		dev_err(codec->dev, "Failed to register codec\n");
		return ret;
	}

	ret = snd_soc_register_dai(&aic3105_dai);
	if (ret) {
		dev_err(codec->dev, "Failed to register dai\n");
		snd_soc_unregister_codec(codec);
		return ret;
	}

	return 0;
}

static int aic3105_unregister(struct aic3105_priv *aic3105)
{
	aic3105_set_bias_level(&aic3105->codec, SND_SOC_BIAS_OFF);

	snd_soc_unregister_dai(&aic3105_dai);
	snd_soc_unregister_codec(&aic3105->codec);

	if (aic3105->gpio_reset >= 0) {
		gpio_set_value(aic3105->gpio_reset, 0);
		gpio_free(aic3105->gpio_reset);
	}
	regulator_bulk_disable(ARRAY_SIZE(aic3105->supplies), aic3105->supplies);
	regulator_bulk_free(ARRAY_SIZE(aic3105->supplies), aic3105->supplies);

	kfree(aic3105);
	aic3105_codec = NULL;

	return 0;
}

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
/*
 * AIC3105 2 wire address can be up to 4 devices with device addresses
 * 0x18, 0x19, 0x1A, 0x1B
 */

/*
 * If the i2c layer weren't so broken, we could pass this kind of data
 * around
 */
static int aic3105_i2c_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	struct snd_soc_codec *codec;
	struct aic3105_priv *aic3105;
	struct aic3105_pdata *pdata = i2c->dev.platform_data;
	int ret, i;

	aic3105 = kzalloc(sizeof(struct aic3105_priv), GFP_KERNEL);
	if (aic3105 == NULL) {
		dev_err(&i2c->dev, "failed to create private data\n");
		return -ENOMEM;
	}

	codec = &aic3105->codec;
	codec->dev = &i2c->dev;
	snd_soc_codec_set_drvdata(codec, aic3105);
	codec->control_data = i2c;
	codec->hw_write = (hw_write_t) i2c_master_send;

	i2c_set_clientdata(i2c, aic3105);

	aic3105->gpio_reset = -1;
	if (pdata && pdata->gpio_reset >= 0) {
		ret = gpio_request(pdata->gpio_reset, "tlv320aic3105 reset");
		if (ret != 0)
			goto err_gpio;
		aic3105->gpio_reset = pdata->gpio_reset;
		gpio_direction_output(aic3105->gpio_reset, 0);
	}

	for (i = 0; i < ARRAY_SIZE(aic3105->supplies); i++)
		aic3105->supplies[i].supply = aic3105_supply_names[i];

	ret = regulator_bulk_get(codec->dev, ARRAY_SIZE(aic3105->supplies),
				 aic3105->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to request supplies: %d\n", ret);
		goto err_get;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(aic3105->supplies),
				    aic3105->supplies);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to enable supplies: %d\n", ret);
		goto err_enable;
	}

	if (aic3105->gpio_reset >= 0) {
		udelay(1);
		gpio_set_value(aic3105->gpio_reset, 1);
	}

	return aic3105_register(codec);

err_enable:
	regulator_bulk_free(ARRAY_SIZE(aic3105->supplies), aic3105->supplies);
err_get:
	if (aic3105->gpio_reset >= 0)
		gpio_free(aic3105->gpio_reset);
err_gpio:
	kfree(aic3105);
	return ret;
}

static int aic3105_i2c_remove(struct i2c_client *client)
{
	struct aic3105_priv *aic3105 = i2c_get_clientdata(client);

	return aic3105_unregister(aic3105);
}

static const struct i2c_device_id aic3105_i2c_id[] = {
	{ "tlv320aic3105", 0 },
	{ "tlv320aic33", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic3105_i2c_id);

/* machine i2c codec control layer */
static struct i2c_driver aic3105_i2c_driver = {
	.driver = {
		.name = "aic3105 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe	= aic3105_i2c_probe,
	.remove = aic3105_i2c_remove,
	.id_table = aic3105_i2c_id,
};

static inline void aic3105_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&aic3105_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: error regsitering i2c driver, %d\n",
		       __func__, ret);
}

static inline void aic3105_i2c_exit(void)
{
	i2c_del_driver(&aic3105_i2c_driver);
}
#else
static inline void aic3105_i2c_init(void) { }
static inline void aic3105_i2c_exit(void) { }
#endif

static int aic3105_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct aic3105_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	codec = aic3105_codec;
	if (!codec) {
		dev_err(&pdev->dev, "Codec not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = codec;
	setup = socdev->codec_data;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "aic3105: failed to create pcms\n");
		goto pcm_err;
	}

	snd_soc_add_controls(codec, aic3105_snd_controls,
			     ARRAY_SIZE(aic3105_snd_controls));

	aic3105_add_widgets(codec);

	return ret;

pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static int aic3105_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* power down chip */
	if (codec->control_data)
		aic3105_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec->reg_cache);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_aic3105 = {
	.probe = aic3105_probe,
	.remove = aic3105_remove,
	.suspend = aic3105_suspend,
	.resume = aic3105_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_aic3105);

static int __init aic3105_modinit(void)
{
	aic3105_i2c_init();

	return 0;
}
module_init(aic3105_modinit);

static void __exit aic3105_exit(void)
{
	aic3105_i2c_exit();
}
module_exit(aic3105_exit);

MODULE_DESCRIPTION("ASoC TLV320AIC3105 codec driver");
MODULE_AUTHOR("Hans Christian Lonstad");
MODULE_LICENSE("GPL");
