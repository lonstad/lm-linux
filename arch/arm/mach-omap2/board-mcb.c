/*
 * linux/arch/arm/mach-omap2/board-mcbc
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/am35xx_emac.h>
#include <linux/can/platform/ti_hecc.h>

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


/***********************************************************************
 *
 * 	GPIO definitions
 *
 */
#define RTC_IRQ_GPIO 	55	// GPIO interrupt for RTC
// BC

#define PSU_PWTBN_CTL1	63	// Communication with BC
#define PSU_PWTBN_CTL2	64	// Communication with BC
#define LED_ARB			99
#define I2C_ARB	  		100	// Master select 0 (default) is BC, 1 is omap
#define I2C_MODE		101	// Interrupt from BC (low) TODO: Check level
#define ISP_MODE		102 // Sets update mode on BC when 1


#define WIFI_SD			69	// Powerdown WiFI when 0
#define WIFI_PDN		86
#define WIFI_RESETN	 	85
#define PWR_MANIKIN_EN	89	// Keep this off under boot
#define CPU2MCU_RST		67
// WIFI Leds
#define WIFI_LED_CTL_R	182
#define WIFI_LED_CTL_G	181
#define WIFI_LED_CTL_B	180
// Pwr LEDS
#define PWR_LED_CTL_R	94
#define PWR_LED_CTL_G	179
#define PWR_LED_CTL_B	178
// MCU
#define MCU_BOOT0	66
#define MCU_BOOT1	93

#define SYS_RST 30

#define AC5W_SD 84
#define UART4_SEL 68

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
	config_gpio_in(WIFI_PDN, OMAP_PIN_INPUT_PULLUP, "wifi_pdn");
	config_gpio_out(WIFI_RESETN, OMAP_PIN_INPUT_PULLUP, "WIFI_RESETN", 0);
	config_gpio_out(WIFI_SD, OMAP_PIN_INPUT_PULLUP, "wifi_sd", 1);
	udelay(1000);
	gpio_set_value(WIFI_RESETN, 1);
	udelay(1000);
	config_gpio_out(PWR_MANIKIN_EN, OMAP_PIN_OUTPUT, "pwr_manikin_en", 1);
	config_gpio_out(MCU_BOOT0, OMAP_PIN_OUTPUT, "mcu_boot0", 0);
	config_gpio_out(MCU_BOOT1, OMAP_PIN_OUTPUT, "mcu_boot1", 0);
	config_gpio_out(SYS_RST, OMAP_PIN_OUTPUT, "sys_rst", 1);
	//config_gpio_out(CPU2MCU_RST, OMAP_PIN_OUTPUT, "mcu_rst", 0);
	config_gpio_in(CPU2MCU_RST, OMAP_PIN_INPUT_PULLUP, "CPU2MCU_RST");
	//config_gpio_out(AC5W_SD, OMAP_PIN_OUTPUT, "ac5sw_sd", 0);
	config_gpio_in(AC5W_SD, OMAP_PIN_INPUT_PULLUP, "ac5sw_sd");
	config_gpio_out(UART4_SEL, OMAP_PIN_OUTPUT, "UART4_SEL", 1);
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

static struct i2c_board_info __initdata mcb_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("PSU", 0x98 >> 1),
		.type = "mcb-bc",
	},
	{
		I2C_BOARD_INFO("one_wire_bridge_0", 0x30 >> 1),
		.type = "ds2482",
	},
	{
		I2C_BOARD_INFO("one_wire_bridge_1", 0x32 >> 1),
		.type = "ds2482",
	},
	{
		I2C_BOARD_INFO("pwrmon_0", 0x80 >> 1),
		.type = "ina219",
	},
	{
		I2C_BOARD_INFO("pwrmon_1", 0x82 >> 1),
		.type = "ina219",
	},
};

/*******************************************************************************
 *
 * 	SPI
 */

static struct omap2_mcspi_device_config mcb_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info mcb_spi_board_info[] __initdata = {
	{
		.modalias		= "pll1708",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1000000,
		.controller_data	= &mcb_mcspi_config,
	}
};

/************************************************************************
 *
 * 	LEDS
 */
static struct gpio_led mcb_gpio_leds[] = {
	{
		.name	= "wifi:red",
		.gpio	= WIFI_LED_CTL_R,
	},
	{
		.name	= "wifi:green",
		.gpio	= WIFI_LED_CTL_G,
	},
	{
		.name	= "wifi:blue",
		.gpio	= WIFI_LED_CTL_B,
	},
	{
		.name	= "pwr:red",
		.gpio	= PWR_LED_CTL_R,
	},
	{
		.name	= "pwr:green",
		.gpio	= PWR_LED_CTL_G,
	},
	{
		.name	= "pwr:blue",
		.gpio	= PWR_LED_CTL_B,
	},

};

static struct gpio_led_platform_data mcb_led_data = {
	.leds	= mcb_gpio_leds,
	.num_leds	= ARRAY_SIZE(mcb_gpio_leds),
};

static struct platform_device mcb_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &mcb_led_data,
	},
};


/***********************************************************************'
 *
 * 	RTC - DS1374
 */

static void __init mcb_rtc_init(void) {

	int r = config_gpio_in(RTC_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP, "rtc-irq");
	if (r == 0) {
		mcb_i2c1_boardinfo[0].irq = gpio_to_irq(RTC_IRQ_GPIO);
		printk("%s: got irq %d for gpio_%d\n", __func__, mcb_i2c1_boardinfo[0].irq, RTC_IRQ_GPIO);
	}
}

/*********************************************************************
 *
 * 	Board Controller
 */

static void mcb_bc_init(void) {
	int r;
	r = config_gpio_out(I2C_ARB, OMAP_PIN_OUTPUT, "i2c_arb1", 1);	// Take control of interface
	r = config_gpio_out(I2C_MODE, OMAP_PIN_OUTPUT, "i2c_mode", 1);
	r = config_gpio_out(ISP_MODE, OMAP_PIN_OUTPUT, "isp_mode", 0);	// TODO: Check interrupt level
	mcb_i2c3_boardinfo[0].platform_data = NULL;
}

#ifdef HAS_FLASH
/****************************************************************************
 *
 * 	FLASH (1GiB)
 */

#define NAND_BLOCK_SIZE SZ_128K

static struct mtd_partition mcb_nand_partitions[] = {
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

static struct omap_nand_platform_data mcb_nand_data = {
	.parts = mcb_nand_partitions,
	.nr_parts = ARRAY_SIZE(mcb_nand_partitions),
	.dma_channel = -1,	/* disable DMA in OMAP NAND driver */
};

static void __init mcb_flash_init(void)
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
		mcb_nand_data.cs = nandcs;

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (gpmc_nand_init(&mcb_nand_data) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}
#endif

/*********************************************************************
 *
 * 	Board initialization
 */

static struct omap_board_config_kernel mcb_config[] __initdata = {
};

static struct platform_device *mcb_devices[] __initdata = {
	&mcb_leds_gpio,
};


static int __init mcb_i2c_init(void) {
	omap_register_i2c_bus(1, 400, mcb_i2c1_boardinfo, ARRAY_SIZE(mcb_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, mcb_i2c2_boardinfo, ARRAY_SIZE(mcb_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void mcb_reset_all(void) {
	//gpio_set_value(SYS_RST, 0);
	//gpio_set_value(WIFI_SD, 0);	// Shut down WLAN
	//udelay(10);
	//gpio_set_value(WIFI_SD, 1);	// Shut down WLAN
	//udelay(1000);
	//gpio_set_value(SYS_RST, 1);

}
/*********************************************************************
 *
 * MMC
 */

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd        = -EINVAL,
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
	.name           = "davinci_emac",
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

static void mcb_power_off(void) {
	gpio_set_value(PSU_PWTBN_CTL1, 1);
	mdelay(1);
	gpio_set_value(PSU_PWTBN_CTL1, 0);
}

static void  __init mcb_init_power_off(void) {
	int r = config_gpio_out(PSU_PWTBN_CTL1, OMAP_PIN_OUTPUT, "PSU_PWTBN_CTL1", 0);
	if ( r )
		printk(KERN_ERR "%s %s can not provide BC power off\n", __FILE__, __func__);
	pm_power_off = mcb_power_off;
}


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

/*******************************************************************
 *
 * 	Init
 */
static void __init mcb_init_irq(void)
{
	omap_board_config = mcb_config;
	omap_board_config_size = ARRAY_SIZE(mcb_config);
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
		/* MMC 2 */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT5, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	OMAP3_MUX(SAD2D_MCAD25, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(SAD2D_MCAD28, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },

};
#else
#define board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
	//.extvbus	 	= 1,
};


static void __init mcb_init(void)
{
	mcb_init_power_off();

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	mcb_export_gpio();
	mcb_reset_all();
	platform_add_devices(mcb_devices, ARRAY_SIZE(mcb_devices));
	omap_serial_init();
	mcb_bc_init();
	usb_ehci_init(&ehci_pdata);
	usb_musb_init(&musb_board_data);
	mcb_evm_hecc_init(&mcb_evm_hecc_pdata);
	/* NET */
	mcb_ethernet_init(&mcb_emac_pdata);
	/* RTC */
	mcb_rtc_init();
	/* I2C */
	mcb_i2c_init();
	i2c_register_board_info(3, mcb_i2c3_boardinfo, ARRAY_SIZE(mcb_i2c3_boardinfo));

#ifdef HAS_FLASH
	mcb_flash_init();
#endif

	spi_register_board_info(mcb_spi_board_info, ARRAY_SIZE(mcb_spi_board_info));
	//mdelay(200);

	/* MMC */
	omap2_hsmmc_init(mmc);
}

MACHINE_START(OMAP3517EVM, "DR MCB board")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.init_irq	= mcb_init_irq,
	.reserve = omap_reserve,
	.init_machine	= mcb_init,
	.timer		= &omap_timer,
MACHINE_END
