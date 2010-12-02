#
# Makefile for the kernel mmc core.
#

obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o

mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o

CFLAGS_REMOVE_core.o = -Os
CFLAGS_REMOVE_sdio_bus.o = -Os
CFLAGS_REMOVE_bus.o = -Os
CFLAGS_REMOVE_sdio.o = -Os
CFLAGS_REMOVE_sdio_ops.o = -Os
