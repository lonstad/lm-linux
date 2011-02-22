/*
 * 1-Wire implementation for the ds2781 chip
 *
 * Copyright © Laerdal Medical
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#ifndef __w1_ds2781_h__
#define __w1_ds2781_h__

// Commands
#define W1_DS2781_SWAP			0xAA
#define W1_DS2781_READ_DATA		0x69
#define W1_DS2781_WRITE_DATA	0x6C
#define W1_DS2781_COPY_DATA		0x48
#define W1_DS2781_RECALL_DATA	0xB8
#define W1_DS2781_LOCK			0x6A

// EEPROM size (actually 256, but w1 system wraps on uchar)
#define DS2781_DATA_SIZE		0x80

#define DS2781_PROTECTION_REG		0x00
#define DS2781_STATUS_REG		    0x01
	#define DS2781_STATUS_CHGTF		0x80	// Charge termination flag
	#define DS2781_STATUS_AEF		0x40	// Active Empty Flag
	#define DS2781_STATUS_SEF		0x20	// Standby Empty Flag
	#define DS2781_STATUS_LEARNF	0x10	// Learn flag
	#define DS2781_STATUS_UVF		0x04	// Undervoltage Flag
	#define DS2781_STATUS_PROF		0x02	// Power on reset Flag

#define DS2781_CONTROL_REG		    0x60
	#define DS2781_CONTROL_PMOD	 	(1 << 5)

#define DS2781_EEPROM_REG		    0x1F
#define DS2781_SPECIAL_FEATURE_REG	0x15
#define DS2781_RAAC_MSB             0x02	// Remaining active cap in mAh
#define DS2781_RAAC_LSB             0x03
#define DS2781_RARC                 0x06	// Remaining cap in percent
#define DS2781_VOLTAGE_MSB          0x0c
#define DS2781_VOLTAGE_LSB	        0x0d
#define DS2781_CURRENT_AVG_MSB	    0x08
#define DS2781_CURRENT_AVG_LSB	    0x09
#define DS2781_CURRENT_MSB		    0x0e
#define DS2781_CURRENT_LSB		    0x0f
#define DS2781_CURRENT_ACCUM_MSB	0x10
#define DS2781_CURRENT_ACCUM_LSB	0x11
#define DS2781_TEMP_MSB			    0x0A
#define DS2781_TEMP_LSB			    0x0B
#define DS2781_EEPROM_BLOCK0		0x20
#define DS2781_ACTIVE_FULL		    0x16
#define DS2781_ACTIVE_EMPTY		    0x18
#define DS2781_EEPROM_BLOCK1		0x60

#define DS2781_FULL40_CAPACITY_MSB	0x6A
#define DS2781_FULL40_CAPACITY_LSB	0x6B
#define DS2781_CURRENT_OFFSET_BIAS	0x7B
#define DS2781_ACTIVE_EMPTY40		0x68
#define DS2781_ACTIVE_RSNSP		    0x69

extern int w1_ds2781_read(struct device *dev, char *buf, int addr,
			  size_t count);
extern int w1_ds2781_write(struct device *dev, char *buf, int addr,
			   size_t count);
extern int w1_ds2781_store_eeprom(struct device *dev, int addr);
extern int w1_ds2781_recall_eeprom(struct device *dev, int addr);

#endif /* !__w1_ds2781_h__ */
