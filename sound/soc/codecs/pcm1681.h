/* PCM1681.h */


#ifndef _PCM1681_H
#define _PCM1681_H


/* Register */

#define PCM1681_ATT1      0x01
#define PCM1681_ATT2      0x02
#define PCM1681_ATT3      0x03
#define PCM1681_ATT4      0x04
#define PCM1681_ATT5      0x05
#define PCM1681_ATT6      0x06
#define PCM1681_MUTE      0x07
#define PCM1681_DAC       0x08
#define PCM1681_IFACE     0x09
#define PCM1681_APDIGI    0x0a
#define PCM1681_PHASEO    0x0b
#define PCM1681_FLTRO     0x0c
#define PCM1681_ZEROFL    0x0d
#define PCM1681_ZEROD     0x0e
#define PCM1681_ATT7      0x10
#define PCM1681_ATT8      0x11
#define PCM1681_MUTEOR    0x12
#define PCM1681_DACOR     0x13


#define PCM1681_CACHEREGNUM    20

extern struct snd_soc_dai pcm1681_dai;
extern struct snd_soc_codec_device soc_codec_dev_pcm1681;


#endif
