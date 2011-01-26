#ifndef LINUX_SPI_PLL1708_H
#define LINUX_SPI_PLL1708_H

typedef enum freq_sel {PLL1708_FS_48KHZ=0, PLL1708_FS_44KHZ=1, PLL1708_FS_32KHZ=2, PLL1708_FS_RESERVED=3} freq_sel;
typedef enum {PLL1708_SR_STANDARD=0, PLL1708_SR_DOUBLE=1, PLL1708_SR_HALF=2, PLL1708_SR_RESERVED=3} sample_rate;

struct pll1708
{
	struct device *dev;
	freq_sel freq;
	sample_rate sr;
};

#endif
