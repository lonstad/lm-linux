/**************************************************************************'
 *
 * 	Board file for Laerdal RC
 * 	arch/arm/mach-omap2/board-rc.c
 */


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/usb/otg.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/if_ether.h>
#include <linux/pwm_backlight.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <mach/gpio.h>
#include <plat/gpmc.h>
#include <mach/hardware.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <linux/smsc911x.h>

#include <linux/gpio_keys.h>
#include "mux.h"
#include "hsmmc.h"
#include "sdram-micron-mt46h32m32lf-6.h"

#define CONFIG_RC_WIFI
#undef USE_MT_TOUCH


#include <linux/ft5x0x_ts.h>
#include <linux/input/cy8ctma3001.h>


#define G_SENSOR_INT1 156
#define G_SENSOR_INT2 157
#define NAND_BLOCK_SIZE SZ_128K

#define RC_SMSC911X_CS      3
#define RC_SMSC911X_IRQ    	25
#define RC_SMSC911X_NRESET 	167


#define RC_TOUCH_PWREN	142
#define RC_TOUCH_IRQ	143

#define AMP_SD	158		// Handled in SOC file

#define RC_WLAN_NWAKEUP 173
#define RC_WLAN_NPD		174
#define PWREN_WIFI	159

#define RC_LCD_LR	171
#define RC_LCD_UD	172

#define RC_3V_PWREN 96

#define BST5V_PWREN 98
#define LCD_PWRENB 55
#define LED_PWRENB 58
#define BACKL_ADJ 57

#define PHY_PWREN 111
#define RC_USBH_NRESET	97

#define OTG5V_EN 26
#define OTG_OC 27

#define USUS 126

#define PWR_BTN 31

#if CONFIG_RC_VERSION > 1
#define HOST_CHG_EN	24	// Enable charge from USB
#define RC_SMSC911X_PWREN 	95
#define IUSB 94			// Enable 500 mA from USB

#define RESET_WLAN	170
#define USER_KEY 141

#else
#define RC_SMSC911X_PWREN 	99
#define RESET_WLAN	16
#define USER_KEY 140
#endif

void twl_poweroff(void);

static int rc_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio);
/***********************************************************************
 *
 * 	Utilities
 */

static int config_gpio_in(int gpio_pin, int gpio_pin_type, const char* name) {
	int r;
	omap_mux_init_gpio(gpio_pin, gpio_pin_type);
	r = gpio_request(gpio_pin, name);
	if (r < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d\n", gpio_pin);
		return r;
	}
	r = gpio_direction_input(gpio_pin);

	if (r < 0) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as %d\n",
				gpio_pin, gpio_pin_type);
		gpio_free(gpio_pin);
		return r;
	}
	gpio_export(gpio_pin, 0);
	return 0;
}

static int config_gpio_out(int gpio_pin, int gpio_pin_type, const char* name, int val) {
	int r;
	omap_mux_init_gpio(gpio_pin, gpio_pin_type);
	r = gpio_request(gpio_pin, name);
	if (r < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d\n", gpio_pin);
		return r;
	}
	r = gpio_direction_output(gpio_pin, val);

	if (r < 0) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as %d\n",
				gpio_pin, gpio_pin_type);
		gpio_free(gpio_pin);
		return r;
	}
	gpio_export(gpio_pin, 0);
	return 0;
}


/******************************************************************
 *
 * 	Ethernet
 */

#ifdef CONFIG_SMSC911X

static struct smsc911x_platform_config rc_smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_FORCE_INTERNAL_PHY ,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};
static struct resource rc_smsc911x_resources[] = {
	{
		.name	= "smsc911x-memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};
static struct platform_device rc_smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(rc_smsc911x_resources),
	.resource	= rc_smsc911x_resources,
	.dev		= {
		.platform_data = &rc_smsc911x_config,
	},
};

static struct platform_device *smsc911x_devices[] = {
	&rc_smsc911x_device,
};

static int __init rc_eth_addr_setup(char *str) {
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		rc_smsc911x_config.mac[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}
/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", rc_eth_addr_setup);



static inline void rc_init_smsc911x(void)
{
	unsigned long cs_mem_base;

	if (gpmc_cs_request(RC_SMSC911X_CS, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC mem for smsc911x\n");
		return;
	}

	rc_smsc911x_resources[0].start = cs_mem_base + 0x0;
	rc_smsc911x_resources[0].end   = cs_mem_base + 0xff;

	if (config_gpio_in(RC_SMSC911X_IRQ, OMAP_PIN_INPUT_PULLUP, "SMSC911X IRQ") == 0) {
		rc_smsc911x_resources[1].start = gpio_to_irq(RC_SMSC911X_IRQ);
		rc_smsc911x_resources[1].end	  = 0;
	}
	config_gpio_out(RC_SMSC911X_NRESET, OMAP_PIN_OUTPUT, "RC_SMSC911X_NRESET", 1);
	config_gpio_out(RC_SMSC911X_PWREN, OMAP_PIN_OUTPUT, "RC_SMSC911X_PWREN", 1);
	udelay(200);

	platform_add_devices(smsc911x_devices, ARRAY_SIZE(smsc911x_devices));
}

#endif

#ifdef SUPPORT_NAND
/****************************************************************************
 *
 * 	Flash
 */
static struct mtd_partition rc_nand_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,			/* Offset = 0x00000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size           = 14 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data rc_nand_data = {
	.parts = rc_nand_partitions,
	.nr_parts = ARRAY_SIZE(rc_nand_partitions),
	.devsize = NAND_BUSWIDTH_16,
	.dma_channel = -1,	/* disable DMA in OMAP NAND driver */
};

static void __init rc_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		rc_nand_data.cs = nandcs;

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (gpmc_nand_init(&rc_nand_data) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

#endif

/******************************************************************
 *
 * 	DSS
 */

static int rc_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PWRENB, 1);
	mdelay(1);
	gpio_set_value(LED_PWRENB, 1);
	//gpio_set_value(BACKL_ADJ, 1);
	return 0;
}

static void rc_disable_lcd(struct omap_dss_device *dssdev)
{
	//gpio_set_value(BACKL_ADJ, 0);
	gpio_set_value(LED_PWRENB, 0);
	gpio_set_value(LCD_PWRENB, 0);
}

static struct omap_dss_device rc_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.driver_name = "rc_panel",
	.phy.dpi.data_lines = 18,
	.platform_enable = rc_enable_lcd,
	.platform_disable = rc_disable_lcd,
};

static struct omap_dss_device *rc_dss_devices[] = {
	&rc_lcd_device,
};

static struct omap_dss_board_info rc_dss_data = {
	.num_devices = ARRAY_SIZE(rc_dss_devices),
	.devices = rc_dss_devices,
	.default_device = &rc_lcd_device,
};

static struct platform_device rc_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &rc_dss_data,
	},
};



#ifdef CONFIG_BACKLIGHT_PWM
/*****************************************************************
 *
 * 	Backlight
 */

static struct platform_pwm_backlight_data rc_backlight_data = {
	.pwm_id = 11,
	.max_brightness = 100,
	.dft_brightness = 25,
	.pwm_period_ns = 25000,
};

static struct platform_device rc_backlight_device = {
	.name          = "pwm-backlight",
	.id            = -1,
	.dev            = {
		.platform_data = &rc_backlight_data,
	},
};

#endif


/******************************************************************
 *
 * 	MMC
 */
static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA ,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
#ifdef CONFIG_RC_WIFI
	{
		.mmc		= 2,
		//.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.transceiver	= true,
		.ocr_mask	= 0x00100000,	/* 3.3V */
	},
#endif
	{}	/* Terminator */
};


/**************************************************************************************
 *
 * 	Regulators in TPS65930
 */
static struct regulator_consumer_supply rc_vmmc1_supply = {
	.supply = "vmmc",
};


static struct regulator_consumer_supply rc_vdvi_supplies[] =
{
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
};


/* VPLL2 for digital video outputs */
static struct regulator_init_data rc_vpll1 = {
	.constraints = {
		.name			= "VPLL1",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask	= REGULATOR_CHANGE_MODE	| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(rc_vdvi_supplies),
	.consumer_supplies	= rc_vdvi_supplies,
};



static struct twl4030_gpio_platform_data rc_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= rc_twl_gpio_setup,
};

static struct twl4030_usb_data rc_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct regulator_init_data rc_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &rc_vmmc1_supply,
};

static struct twl4030_codec_audio_data rc_audio_data = {

};

static struct twl4030_codec_data rc_codec_data = {
	.audio_mclk = 26000000,
	.audio = &rc_audio_data,
};

/* mmc2 (WLAN) and Bluetooth don't use twl4030 regulators */

static struct twl4030_platform_data rc_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,
	.gpio		= &rc_gpio_data,
	.usb		= &rc_usb_data,
	.codec		= &rc_codec_data,
	.vmmc1		= &rc_vmmc1,
	.vpll1		= &rc_vpll1,
};

/********************************************************************
 *
 * 	Other consumer/supplies
 */

static struct regulator_consumer_supply rc_vcc = {
		.supply = "vcc",
};

// The power gate to WiFI
static struct regulator_consumer_supply rc_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

static struct regulator_init_data vcc_wlan_reg_data = {
	//.supply_regulator = "VCC33",
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &rc_vmmc2_supply,
};

static struct fixed_voltage_config vcc_wlan_config = {
	.supply_name = "VCC_WLAN",
	.microvolts = 3300000,
	.enabled_at_boot = 0,
	.enable_high = 1,
	.startup_delay = 10000,
	.gpio = RC_WLAN_NPD,
	.init_data = &vcc_wlan_reg_data,
};


static struct platform_device reg_wlan = {
	.name = "reg-fixed-voltage",
	.id = 2,
	.dev = {
		.platform_data = &vcc_wlan_config,
	},
};

// USB Phy
static struct regulator_consumer_supply rc_usb_phy_supply =
	REGULATOR_SUPPLY("hsusb0", "ehci-omap.0");

static struct regulator_init_data usb_phy_reg_data = {
	//.supply_regulator = "VCC33",
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &rc_usb_phy_supply,
};

static struct fixed_voltage_config usb_phy_config = {
	.supply_name = "HSUSB1_PHY",
	.microvolts = 1800000,
	.enabled_at_boot = 0,
	.enable_high = 1,
	.startup_delay = 10000,
	.gpio = PHY_PWREN,
	.init_data = &usb_phy_reg_data,
};

static struct platform_device reg_usb_phy = {
	.name = "reg-fixed-voltage",
	.id = 3,
	.dev = {
		.platform_data = &usb_phy_config,
	},
};
/************************************************/

static int rc_twl_gpio_setup(struct device *dev, unsigned gpio, unsigned ngpio)
{
	omap2_hsmmc_init(mmc);
	rc_vmmc1_supply.dev = mmc[0].dev;
#ifdef CONFIG_RC_WIFI
	rc_vmmc2_supply.dev = mmc[1].dev;
#endif

	return 0;
}

/*********************************************************************
 *
 * Touch
 */


static struct cy8_platform_data tc_single_data = {
	.maxx = 767,
	.maxy = 1023,
	.flags =  CY8F_REVERSE_Y | CY8F_XY_AXIS_FLIPPED,
};



static struct i2c_board_info __initdata rc_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65930", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &rc_twldata,
	},
};

static struct i2c_board_info __initdata rc_i2c2_boardinfo[] = {

	{
			I2C_BOARD_INFO("EDTTouch", 0x38),
			.type = FT5X0X_NAME,

	},
	{
		I2C_BOARD_INFO("TrueTouch", 0x24),

		.type = "cy8ctma300",
		.platform_data = &tc_single_data,
	},

#if CONFIG_RC_VERSION > 1
	{
		I2C_BOARD_INFO("rc-bat1", 0x34),
		.type = "rc-battery",
	},
#endif

};

static struct i2c_board_info __initdata rc_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("ADXL345", 0x1D ),
		.type  = "adxl34x",
	},
	{
		I2C_BOARD_INFO("AT24C16BN", 0x50),
		.type = "24c16",
	},
};

static int __init rc_i2c_init(void)
{
	int res = config_gpio_in(G_SENSOR_INT1, OMAP_PIN_INPUT_PULLUP, "g_sensor_int1");
	if ( res == 0 )	{
		rc_i2c3_boardinfo[0].irq = gpio_to_irq(G_SENSOR_INT1);
		printk(KERN_INFO "%s: Got IRQ %d for G_SENSOR_INT1\n", __func__, rc_i2c3_boardinfo[0].irq);
	}

	res = config_gpio_in(G_SENSOR_INT2, OMAP_PIN_INPUT_PULLUP, "g_sensor_int2");


	if ( gpio_is_valid(RC_TOUCH_IRQ) )
	{
		rc_i2c2_boardinfo[0].irq = gpio_to_irq(RC_TOUCH_IRQ);
		rc_i2c2_boardinfo[1].irq = rc_i2c2_boardinfo[0].irq;
		printk(KERN_INFO "%s: Got IRQ %d for RC_TOUCH_IRQ\n", __func__, rc_i2c2_boardinfo[0].irq);
	}


	omap_register_i2c_bus(1, 400, rc_i2c1_boardinfo, ARRAY_SIZE(rc_i2c1_boardinfo));
	omap_register_i2c_bus(2, 100, rc_i2c2_boardinfo, ARRAY_SIZE(rc_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, rc_i2c3_boardinfo, ARRAY_SIZE(rc_i2c3_boardinfo));
	return 0;
}

/****************************************************************
 *
 * Buttons (GPIO keyboard)
 */

static struct gpio_keys_button buttons[] = {
	{
		.code = KEY_F12,
		.gpio = USER_KEY,
		.active_low = 1,
		.desc = "USER_BUTTON",
		.type = EV_KEY,
		.debounce_interval = 10,
	},
};

static struct gpio_keys_platform_data rc_buttons_data = {
	.buttons = buttons,
	.nbuttons = 1,
	.rep = 0,
};

static struct platform_device rc_buttons_device = {
	.name          = "gpio-keys",
	.id            = -1,
	.dev            = {
		.platform_data = &rc_buttons_data,
	},
};

static struct platform_device rc_lcd_pdevice = {
	.name		= "rc_lcd",
	.id		= -1,
};

static struct omap_lcd_config rc_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel rc_config[] __initdata = {
	{ OMAP_TAG_LCD,		&rc_lcd_config },
};


static void __init rc_init_early(void)
{
    omap2_init_common_infrastructure();
    omap2_init_common_devices(mt46h32m32lf6_sdrc_params, NULL);
}

static struct platform_device *rc_devices[] __initdata = {
#ifdef CONFIG_BACKLIGHT_PWM
	&rc_backlight_device,
#endif
	&rc_buttons_device,
	&rc_dss_device,
};

static const struct ehci_hcd_omap_platform_data rc_ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = RC_USBH_NRESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(GPMC_NCS6, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),		// Set backlight as pwm
	OMAP3_MUX(CAM_STROBE, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),	// GPIO_126
	OMAP3_MUX(SDMMC1_DAT4, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),		// Remove gpio_126 multipath
	OMAP3_MUX(SDMMC1_DAT5, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT6, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(SDMMC1_DAT7, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(CHASSIS_IDLEACK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(CHASSIS_MSTDBY, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#ifdef CONFIG_USB_MUSB_HDRC
static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
	.extvbus	 	= 1,
};
#endif

void rc_battery_say_goodbye(void);

void shutdown_system(void) {
	gpio_set_value(BST5V_PWREN, 0);
	rc_battery_say_goodbye();
	twl_poweroff();
}
static void __init rc_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);
	config_gpio_in(PWR_BTN, OMAP_PIN_INPUT, "PWR_BTN");
	config_gpio_out(OTG5V_EN, OMAP_PIN_OUTPUT, "OTG5V_EN", 0);

	// Turn on 3.3V and delay
	config_gpio_out(RC_3V_PWREN, OMAP_PIN_OUTPUT, "RC_3V_PWREN", 1);
	mdelay(20);

	// Configure the WiFI power gate
	platform_device_register(&reg_wlan);

	platform_device_register(&reg_usb_phy);
	// Turn on 5V and delay
	config_gpio_out(BST5V_PWREN, OMAP_PIN_OUTPUT, "BST5V_PWREN", 1);
	mdelay(20);

	config_gpio_out(LED_PWRENB, OMAP_PIN_OUTPUT, "LED_PWRENB", 0);
	config_gpio_out(LCD_PWRENB, OMAP_PIN_OUTPUT, "LCD_PWRENB", 0);


#if CONFIG_RC_VERSION > 1
	config_gpio_out(IUSB, OMAP_PIN_OUTPUT, "IUSB", 0);
	config_gpio_out(USUS, OMAP_PIN_OUTPUT, "USUS", 1);
	config_gpio_out(HOST_CHG_EN, OMAP_PIN_OUTPUT, "HOST_CHG_EN", 0);
	config_gpio_out(RC_WLAN_NWAKEUP, OMAP_PIN_OUTPUT, "RC_WLAN_NWAKEUP", 1);
	//config_gpio_out(RC_WLAN_NPD, OMAP_PIN_OUTPUT, "RC_WLAN_NPD", 1);
	config_gpio_out(PWREN_WIFI, OMAP_PIN_OUTPUT, "RC_WLAN_NPD", 1);
	udelay(200);
#else
	config_gpio_in(RC_WLAN_NWAKEUP, OMAP_PIN_INPUT_PULLUP, "RC_WLAN_NWAKEUP");
	config_gpio_in(RC_WLAN_NPD, OMAP_PIN_INPUT_PULLUP, "RC_WLAN_NPD");
#endif
	config_gpio_out(RC_TOUCH_PWREN, OMAP_PIN_OUTPUT, "RC_TOUCH_PWREN", 1);
	//config_gpio_out(PHY_PWREN, OMAP_PIN_OUTPUT, "PHY_PWREN", 1);
	config_gpio_in(RC_TOUCH_IRQ, OMAP_PIN_INPUT, "RC_TOUCH_IRQ");
	config_gpio_in(OTG_OC, OMAP_PIN_INPUT, "OTG_OC");

	rc_i2c_init();
	//platform_device_register(&reg_vcc);


	config_gpio_out(RESET_WLAN, OMAP_PIN_OUTPUT, "RESET_WLAN", 0);
	mdelay(2);
	gpio_set_value(RESET_WLAN, 1);
	platform_add_devices(rc_devices, ARRAY_SIZE(rc_devices));
	omap_serial_init();

#ifdef SUPPORT_NAND
	rc_flash_init();
#endif

#ifdef CONFIG_USB_MUSB_HDRC
	usb_musb_init(&musb_board_data);
#endif

#if defined(CONFIG_USB_EHCI_HCD) && CONFIG_RC_VERSION > 1
	usb_ehci_init(&rc_ehci_pdata);
#endif

#ifdef CONFIG_SMSC911X
	rc_init_smsc911x();
#endif

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	config_gpio_out(RC_LCD_LR, OMAP_PIN_OUTPUT, "lcd_lr", 1);
	config_gpio_out(RC_LCD_UD, OMAP_PIN_OUTPUT, "lcd_ud", 0);
	pm_power_off = shutdown_system;
	printk(KERN_INFO "Laerdal RC v%d init done\n", CONFIG_RC_VERSION);
}

MACHINE_START(OVERO, "Laerdal Remote Control")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap_init_irq,
	.init_early = rc_init_early,
	.init_machine	= rc_init,
	.timer		= &omap_timer,
MACHINE_END
