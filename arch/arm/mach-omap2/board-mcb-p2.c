/*
 * linux/arch/arm/mach-omap2/board-mcbc-p2.c
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/am35xx_emac.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/i2c/ina219_cal.h>
#include <linux/usb/otg.h>
#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/pll1708.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <plat/nand.h>
#include <plat/gpmc.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <control.h>
#include <linux/regulator/machine.h>
#include "mux.h"
#include "hsmmc.h"


int omap_pwm_init(void);
/***********************************************************************
 *
 * 	GPIO definitions
 *
 */

// RTC
#define RTC_IRQ_N 	55	// GPIO interrupt for RTC

// BC
#define PSU_PWTBN_CTL1	63	// The power button is pressed for 3s
#define PSU_PWTBN_CTL2	64	// Shuts power when high
#define LED_ARB			99
#define I2C_REQ	  		100	// Master select 0 (default) is BC, 1 is omap
#define I2C_ACK			101	// Interrupt from BC (low) TODO: Check level
#define PWR_BUTTON		92	// Power button event
// Wifi
#define WIFI_SD			69	// Powerdown WiFI when 0
#define WIFI_PD_N		86
#define WIFI_LED_CTL_R	182
#define WIFI_LED_CTL_G	181
#define WIFI_LED_CTL_B	180



// Resets
#define LAN_PHY_RST_N 	25
#define WIFI_RST_N	 	27
#define AC2_RST_N	26
#define RTC_RST_N	28	// Active low RTC reset (has pullup)

// Manikin
#define PWR_MANIKIN_EN	89	// Keep this off under boot

// Pwr LEDS
#define PWR_LED_CTL_R	94
#define PWR_LED_CTL_G	179
#define PWR_LED_CTL_B	178

// MCU
#define MCU_BOOT0	66
#define MCU_BOOT1	93
#define CPU2MCU_RST	67

// USB
#define USB_HOST_EN_N 65	// USB HOST VBUS enable
#define USB_OC_N 0			// Flag from HOST USB VBUS (fault when 0)
#define USB_HOST_RST_N 24

// 5 watt apml shutdown
#define AC5W_SD_N 84

#define UART4_SEL 68

// MMC
#define SD1_CDS_N 127

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

/***********************************************************************
 *
 * 	Export GPIO
 *
 */

static void mcb_export_gpio(void) {
	config_gpio_out(RTC_RST_N, OMAP_PIN_OUTPUT, "RTC_RST_N", 1);
	config_gpio_out(LAN_PHY_RST_N, OMAP_PIN_OUTPUT, "LAN_PHY_RST_N", 1);
	config_gpio_out(AC2_RST_N, OMAP_PIN_OUTPUT, "AC2_RST_N", 1);
	config_gpio_in(WIFI_PD_N, OMAP_PIN_INPUT_PULLUP, "WIFI_PD_N");
	config_gpio_out(WIFI_RST_N, OMAP_PIN_OUTPUT, "WIFI_RST_N", 0);
	config_gpio_out(WIFI_SD, OMAP_PIN_INPUT_PULLUP, "WIFI_SD", 1);
	config_gpio_in(PSU_PWTBN_CTL1, OMAP_PIN_INPUT, "PSU_PWTBN_CTL1");

	config_gpio_out(PWR_MANIKIN_EN, OMAP_PIN_OUTPUT, "PWR_MANIKIN_EN", 0);
	config_gpio_out(MCU_BOOT0, OMAP_PIN_OUTPUT, "MCU_BOOT0", 0);
	config_gpio_out(MCU_BOOT1, OMAP_PIN_OUTPUT, "MCU_BOOT1", 0);

	config_gpio_out(PWR_LED_CTL_R, OMAP_PIN_OUTPUT, "PWR_LED_CTL_R", 0);
	config_gpio_out(PWR_LED_CTL_G, OMAP_PIN_OUTPUT, "PWR_LED_CTL_G", 1);
	config_gpio_out(PWR_LED_CTL_B, OMAP_PIN_OUTPUT, "PWR_LED_CTL_B", 0);

	config_gpio_out(WIFI_LED_CTL_R, OMAP_PIN_OUTPUT, "WIFI_LED_CTL_R", 0);
	config_gpio_out(WIFI_LED_CTL_G, OMAP_PIN_OUTPUT, "WIFI_LED_CTL_G", 0);
	config_gpio_out(WIFI_LED_CTL_B, OMAP_PIN_OUTPUT, "WIFI_LED_CTL_B", 0);

	config_gpio_out(CPU2MCU_RST, OMAP_PIN_OUTPUT, "CPU2MCU_RST", 1);
	//config_gpio_in(CPU2MCU_RST, OMAP_PIN_INPUT_PULLUP, "CPU2MCU_RST");	// TODO: Should become output
	config_gpio_out(AC5W_SD_N, OMAP_PIN_OUTPUT, "AC5W_SD_N", 0);
	config_gpio_out(UART4_SEL, OMAP_PIN_OUTPUT, "UART4_SEL", 1);
	config_gpio_out(USB_HOST_EN_N, OMAP_PIN_OUTPUT, "USB_HOST_EN_N", 0);
	config_gpio_in(USB_OC_N, OMAP_PIN_INPUT_PULLUP, "USB_OC_N");

	//config_gpio_in(LINE_IN_DET, OMAP_PIN_INPUT_PULLUP, "LINE_IN_DET");
	//config_gpio_in(LINE_OUT_DET, OMAP_PIN_INPUT_PULLUP, "LINE_OUT_DET");

}
/***********************************************************************
 *
 * 	I2C buses
 */
static struct i2c_board_info __initdata mcb_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("rtc-ds1374", 0x68),
		.type		= "ds1374",
	},
	{
		I2C_BOARD_INFO("AT24C16BN", 0x50),
		.type = "24c16",
	},
	{
		I2C_BOARD_INFO("one_wire_bridge_0", 0x30 >> 1),
		.type = "ds2482",
	},
	{
		I2C_BOARD_INFO("one_wire_bridge_1", 0x32 >> 1),
		.type = "ds2482",
	},

};

static struct i2c_board_info __initdata mcb_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("8ChCodec", 0x98 >> 1),
		.type = "pcm1681",
	},
	{
		I2C_BOARD_INFO("StereoOut", 0x30 >> 1),
		.type = "tlv320aic3105",
	},
};

static struct ina219_cal adapter_ina219_cal = {
	.shunt_mohm = 10,
	.max_current_mamp = 6000,
	.vbus_max_mvolt = 12,
};

static struct ina219_cal system_ina219_cal = {
	.shunt_mohm = 10,
	.max_current_mamp = 6000,
	.vbus_max_mvolt = 12,
};

static struct i2c_board_info __initdata mcb_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("PSU", 0xF0 >> 1),
		.type = "mcb-bc",
	},

	{
		I2C_BOARD_INFO("pwrmon_adapter", 0x80 >> 1),
		.type = "ina219",
		.platform_data = &adapter_ina219_cal,
	},
	{
		I2C_BOARD_INFO("pwrmon_system", 0x82 >> 1),
		.type = "ina219",
		.platform_data = &system_ina219_cal,
	},

};

/*******************************************************************************
 *
 * 	SPI
 */

static struct pll1708 pll1708_private = {
	.freq = PLL1708_FS_32KHZ,
	.sr = PLL1708_SR_HALF,

};


static struct omap2_mcspi_device_config mcb_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info mcb_spi_board_info[] __initdata = {
	{
		.modalias			= "pll1708",
		.bus_num			= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1000000,
		.controller_data	= &mcb_mcspi_config,
		.platform_data 		= &pll1708_private,
	}
};


/***********************************************************************'
 *
 * 	RTC - DS1374
 */

static void __init mcb_rtc_init(void) {

	int r = config_gpio_in(RTC_IRQ_N, OMAP_PIN_INPUT_PULLUP, "rtc-irq");
	if (r == 0) {
		mcb_i2c1_boardinfo[0].irq = gpio_to_irq(RTC_IRQ_N);
		printk("%s: got irq %d for gpio_%d\n", __func__, mcb_i2c1_boardinfo[0].irq, RTC_IRQ_N);
	}
}

/*********************************************************************
 *
 * 	Board Controller
 */

static void mcb_bc_init(void) {
	int r;
	r = config_gpio_out(I2C_REQ, OMAP_PIN_OUTPUT, "I2C3_REQ", 1);
	r = config_gpio_in(I2C_ACK, OMAP_PIN_INPUT, "I2C3_ACK");
	config_gpio_out(LED_ARB, OMAP_PIN_OUTPUT, "LED_ARB", 1);
	mcb_i2c3_boardinfo[0].platform_data = NULL;
}



/*********************************************************************
 *
 * 	Board initialization
 */

static struct omap_board_config_kernel mcb_config[] __initdata = {
};

static int __init mcb_i2c_init(void) {
	omap_register_i2c_bus(1, 400, mcb_i2c1_boardinfo, ARRAY_SIZE(mcb_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, mcb_i2c2_boardinfo, ARRAY_SIZE(mcb_i2c2_boardinfo));
	if (gpio_get_value(I2C_ACK) == 0)
		printk(KERN_ERR "%s BC did not grant us I2C3 - \n", __func__);
	else
		omap_register_i2c_bus(3, 400, mcb_i2c3_boardinfo, ARRAY_SIZE(mcb_i2c3_boardinfo));
	return 0;
}

static void mcb_reset_all(void) {
	gpio_set_value(WIFI_RST_N, 0);
	gpio_set_value(LAN_PHY_RST_N, 0);
	gpio_set_value(AC2_RST_N, 0);
	udelay(1000);
	gpio_set_value(WIFI_RST_N, 1);
	gpio_set_value(LAN_PHY_RST_N, 1);
	gpio_set_value(AC2_RST_N, 1);
	udelay(1000);

}
/*********************************************************************
 *
 * MMC
 */

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.caps			= MMC_CAP_4_BIT_DATA,
		.gpio_cd        = -EINVAL, //SD1_CDS_N,
		.gpio_wp        = -EINVAL,
		.ocr_mask	    = MMC_VDD_32_33,
	},
	// Libertas SDIO WiFI
	{
		.mmc            = 2,
		.caps			= MMC_CAP_4_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.transceiver	= true,
		.ocr_mask	    = MMC_VDD_32_33,
		},
	{}      /* Terminator */
};

/************************************************************
 *
 * Ethernet
 */


static struct emac_platform_data  mcb_emac_pdata = {
	.phy_mask       = 0xf,
	.mdio_max_freq  = 1000000,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str) {
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		mcb_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource mcb_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device mcb_emac_device = {
	.name           = "am35xx_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(mcb_emac_resources),
	.resource       = mcb_emac_resources,
};

static void mcb_enable_ethernet_int(void) {
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR
			| AM35XX_CPGMAC_C0_TX_PULSE_CLR | AM35XX_CPGMAC_C0_MISC_PULSE_CLR
			| AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void mcb_disable_ethernet_int(void) {
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void mcb_ethernet_init(struct emac_platform_data *pdata) {
	unsigned int regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 		= mcb_enable_ethernet_int;
	pdata->interrupt_disable 		= mcb_disable_ethernet_int;
	mcb_emac_device.dev.platform_data     = pdata;

	platform_device_register(&mcb_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval,AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}

/*******************************************************************
 *
 * 	Power off hook
 */

// TODO: Check BC implementation
static void mcb_power_off(void) {
	gpio_set_value(PSU_PWTBN_CTL2, 1);
	mdelay(100);
	gpio_set_value(PSU_PWTBN_CTL2, 0);
}

static void  __init mcb_init_power_off(void) {
	int r = config_gpio_out(PSU_PWTBN_CTL2, OMAP_PIN_OUTPUT, "PSU_PWTBN_CTL2", 0);
	if ( r )
		printk(KERN_ERR "%s %s can not provide BC power off\n", __FILE__, __func__);
	pm_power_off = mcb_power_off;
}

/****************************************************************
 *
 * Buttons (GPIO keyboard)
 */

static struct gpio_keys_button buttons[] = {
	{
		.code = KEY_POWER,
		.gpio = PWR_BUTTON,
		.active_low = 0,
		.desc = "PWR_BUTTON",
		.type = EV_KEY,
		.debounce_interval = 10,
	},
};

static struct gpio_keys_platform_data mcb_buttons_data = {
	.buttons = buttons,
	.nbuttons = 1,
	.rep = 0,
};

static struct platform_device mcb_buttons_device = {
	.name          = "gpio-keys",
	.id            = -1,
	.dev            = {
		.platform_data = &mcb_buttons_data,
	},
};

#ifdef CONFIG_CAN_TI_HECC
/******************************************************************
 *
 * 	CAN
 */
static struct resource mcb_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mcb_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(mcb_hecc_resources),
	.resource	= mcb_hecc_resources,
};

static struct ti_hecc_platform_data mcb_evm_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
};

static void mcb_evm_hecc_init(struct ti_hecc_platform_data *pdata)
{
	mcb_hecc_device.dev.platform_data = pdata;
	platform_device_register(&mcb_hecc_device);
}

#endif
/*******************************************************************
 *
 * 	Init
 */


static void __init mcb_init_irq(void)
{
	omap_board_config = mcb_config;
	omap_board_config_size = ARRAY_SIZE(mcb_config);
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();

}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = USB_HOST_RST_N,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART2_RTS, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },

};
#else
#define board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 500,
	.extvbus	 	= 1,
};

static __init void mcb_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_26MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}


static struct platform_device *mcb_devices[] __initdata = {
	&mcb_buttons_device,
};

static void __init mcb_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);
	mcb_init_power_off();
	mcb_export_gpio();
	mcb_reset_all();
	platform_add_devices(mcb_devices, ARRAY_SIZE(mcb_devices));
	omap_serial_init();
	mcb_bc_init();
	usb_ehci_init(&ehci_pdata);

#ifdef CONFIG_CAN_TI_HECC
	mcb_evm_hecc_init(&mcb_evm_hecc_pdata);
#endif
	/* NET */
	mcb_ethernet_init(&mcb_emac_pdata);
	/* RTC */
	mcb_rtc_init();
	/* I2C */
	mcb_i2c_init();
	/* SPI */
	spi_register_board_info(mcb_spi_board_info, ARRAY_SIZE(mcb_spi_board_info));
	/* MMC */
	omap2_hsmmc_init(mmc);
	mcb_musb_init();

}

MACHINE_START(OMAP3517EVM, "DR MCB-P2 board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_irq	= mcb_init_irq,
	.reserve = omap_reserve,
	.init_machine	= mcb_init,
	.timer		= &omap_timer,
MACHINE_END
