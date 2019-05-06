/*
 * Copyright (c) 2014 Google, Inc
 *
 * (C) Copyright 2002
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com.
 *
 * Influenced by code from:
 * Wolfgang Denk, DENX sunxiware Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <malloc.h>
#include <spi.h>
#include <asm/gpio.h>
#include <spl.h>
#include <asm/io.h>
#include <libfdt.h>
#define _DEBUG	0

#ifdef CONFIG_SPL_OS_BOOT
#error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*
 * This is a very simple U-Boot image loading implementation, trying to
 * replicate what the boot ROM is doing when loading the SPL. Because we
 * know the exact pins where the SPI Flash is connected and also know
 * that the Read Data Bytes (03h) command is supported, the hardware
 * configuration is very simple and we don't need the extra flexibility
 * of the SPI framework. Moreover, we rely on the default settings of
 * the SPI controler hardware registers and only adjust what needs to
 * be changed. This is good for the code size and this implementation
 * adds less than 400 bytes to the SPL.
 *
 * There are two variants of the SPI controller in Allwinner SoCs:
 * A10/A13/A20 (sun4i variant) and everything else (sun6i variant).
 * Both of them are supported.
 *
 * The pin mixing part is SoC specific and only A10/A13/A20/H3/A64 are
 * supported at the moment.
 */

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             (0x01C05000 + 0x1C)
#define SUN4I_SPI0_CTL              (0x01C05000 + 0x08)
#define SUN4I_SPI0_RX               (0x01C05000 + 0x00)
#define SUN4I_SPI0_TX               (0x01C05000 + 0x04)
#define SUN4I_SPI0_FIFO_STA         (0x01C05000 + 0x28)
#define SUN4I_SPI0_BC               (0x01C05000 + 0x20)
#define SUN4I_SPI0_TC               (0x01C05000 + 0x24)

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             (0x01C68000 + 0x24)
#define SUN6I_SPI0_GCR              (0x01C68000 + 0x04)
#define SUN6I_SPI0_TCR              (0x01C68000 + 0x08)
#define SUN6I_SPI0_FIFO_STA         (0x01C68000 + 0x1C)
#define SUN6I_SPI0_MBC              (0x01C68000 + 0x30)
#define SUN6I_SPI0_MTC              (0x01C68000 + 0x34)
#define SUN6I_SPI0_BCC              (0x01C68000 + 0x38)
#define SUN6I_SPI0_TXD              (0x01C68000 + 0x200)
#define SUN6I_SPI0_RXD              (0x01C68000 + 0x300)

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#define CCM_AHB_GATING0             (0x01C20000 + 0x60)
#define CCM_SPI0_CLK                (0x01C20000 + 0xA0)
#define SUN6I_BUS_sunxi_RST_REG0     (0x01C20000 + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x8000

/*****************************************************************************/

DECLARE_GLOBAL_DATA_PTR;

struct sunxi_spi_platdata {
	struct gpio_desc cs;
	struct gpio_desc sclk;
	struct gpio_desc mosi;
	struct gpio_desc miso;
	int spi_delay_us;
	int flags;
};

#define SPI_MASTER_NO_RX        BIT(0)
#define SPI_MASTER_NO_TX        BIT(1)

struct sunxi_spi_priv {
	unsigned int mode;
};

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(2); pin++)
		sunxi_gpio_set_cfgpin(pin, pin_function);


	//sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

/*
 * Setup 6 MHz from OSC24M (because the BROM is doing the same).
 */
static void spi0_enable_clock(void)
{

	/* Deassert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		setbits_le32(SUN6I_BUS_sunxi_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));

	/* Open the SPI0 gate */
	setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Divide by 4 */
	writel(SPI0_CLK_DIV_BY_4, IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ?
				  SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL);
	/* 24MHz from OSC24M */
	writel((1 << 31), CCM_SPI0_CLK);

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		/* Enable SPI in the master mode and do a sunxi reset */
		setbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE |
					     SUN6I_CTL_SRST);
		/* Wait for completion */
		while (readl(SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
			;
	} else {
		/* Enable SPI in the master mode and reset FIFO */
		setbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE |
					     SUN4I_CTL_TF_RST |
					     SUN4I_CTL_RF_RST);
	}
}

#if 0
static void spi0_disable_clock(void)
{

	/* Disable the SPI0 controller */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE);
	else
		clrbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE);

	/* Disable the SPI0 clock */
	writel(0, CCM_SPI0_CLK);

	/* Close the SPI0 gate */
	clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Assert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_BUS_sunxi_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));
}
#endif

static void spi0_init(void)
{
	unsigned int pin_function = SUNXI_GPC_SPI0;


	spi0_pinmux_setup(pin_function);
	spi0_enable_clock();
}



static int sunxi_spi_cs_activate(struct udevice *dev)
{
	struct udevice *bus = dev_get_parent(dev);
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);

	dm_gpio_set_value(&plat->cs, 1);
		/* Open the SPI0 gate */
	setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));
	dm_gpio_set_value(&plat->cs, 0);

	return 0;
}

static int sunxi_spi_cs_deactivate(struct udevice *dev)
{
	struct udevice *bus = dev_get_parent(dev);
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);

	dm_gpio_set_value(&plat->cs, 1);
		/* Close the SPI0 gate */
	clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));
	return 0;
}

static int sunxi_spi_claim_bus(struct udevice *dev)
{
	/*
	 * Make sure the SPI clock is in idle state as defined for
	 * this slave.
	 */
	 
	return 0;
}

static int sunxi_spi_release_bus(struct udevice *dev)
{
	/* Nothing to do */
	return 0;
}
/*****************************************************************************/

#define SPI_READ_MAX_SIZE 64 /* FIFO size, minus 4 bytes of the header */

static void sunxi_spi0_xfer_data(u8 *recv_buf8, u8 *send_buf8, u32 bufsize,
				 ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	int i = 0;

	writel(bufsize, spi_bc_reg); /* Burst counter (total bytes) */
	
	if(send_buf8 != NULL)
	{
		writel(bufsize, spi_tc_reg);           /* Transfer counter (bytes to send) */
		if (spi_bcc_reg)
			writel(bufsize, spi_bcc_reg);  /* SUN6I also needs this */
		
	
		/* Send the Read Data Bytes (03h) command header */
		for(i=0;i<bufsize;i++)
		{
			writeb((u8)send_buf8[i], spi_tx_reg);
		}

	}
	

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < bufsize)
		;

	/* Read the data */
	while (bufsize-- > 0)
		*recv_buf8++ = readb(spi_rx_reg);
	

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	udelay(1);
}
static void spi0_xfer_data(void *recv_buf, const void *send_buf, u32 len)
{
	u8 *recv_buf8 = recv_buf;
	u8 *send_buf8 = send_buf;
	u32 chunk_len;

	while (len > 0) {
		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE)
			chunk_len = SPI_READ_MAX_SIZE;

		if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
			sunxi_spi0_xfer_data(recv_buf8, send_buf8, chunk_len,
					     SUN6I_SPI0_TCR,
					     SUN6I_TCR_XCH,
					     SUN6I_SPI0_FIFO_STA,
					     SUN6I_SPI0_TXD,
					     SUN6I_SPI0_RXD,
					     SUN6I_SPI0_MBC,
					     SUN6I_SPI0_MTC,
					     SUN6I_SPI0_BCC);
		} else {
			sunxi_spi0_xfer_data(recv_buf8, send_buf8, chunk_len,
					     SUN4I_SPI0_CTL,
					     SUN4I_CTL_XCH,
					     SUN4I_SPI0_FIFO_STA,
					     SUN4I_SPI0_TX,
					     SUN4I_SPI0_RX,
					     SUN4I_SPI0_BC,
					     SUN4I_SPI0_TC,
					     0);
		}

		len  -= chunk_len;
		recv_buf8 += chunk_len;
		send_buf8 += chunk_len;
	}
}

/*-----------------------------------------------------------------------
 * SPI transfer
 *
 * This writes "bitlen" bits out the SPI MOSI port and simultaneously clocks
 * "bitlen" bits in the SPI MISO port.  That's just the way SPI works.
 *
 * The source of the outgoing bits is the "dout" parameter and the
 * destination of the input bits is the "din" parameter.  Note that "dout"
 * and "din" can point to the same memory location, in which case the
 * input data overwrites the output data (since both are buffered by
 * temporary variables, this is OK).
 */
static int sunxi_spi_xfer(struct udevice *dev, unsigned int bitlen,
			 const void *dout, void *din, unsigned long flags)
{

	if (flags & SPI_XFER_BEGIN)
		sunxi_spi_cs_activate(dev);
	
	spi0_xfer_data(din,dout,bitlen/8);

	if (flags & SPI_XFER_END)
		sunxi_spi_cs_deactivate(dev);



	return 0;
}

static int sunxi_spi_set_speed(struct udevice *dev, unsigned int speed)
{
	/* Accept any speed */
	return 0;
}

static int sunxi_spi_set_mode(struct udevice *dev, unsigned int mode)
{
	struct sunxi_spi_priv *priv = dev_get_priv(dev);

	priv->mode = mode;

	return 0;
}

static const struct dm_spi_ops sunxi_spi_ops = {
	.claim_bus	= sunxi_spi_claim_bus,
	.release_bus	= sunxi_spi_release_bus,
	.xfer		= sunxi_spi_xfer,
	.set_speed	= sunxi_spi_set_speed,
	.set_mode	= sunxi_spi_set_mode,
};

static int sunxi_spi_ofdata_to_platdata(struct udevice *dev)
{

	return 0;
}

static int sunxi_spi_probe(struct udevice *dev)
{
	struct sunxi_spi_platdata *plat = dev->platdata;

	if (gpio_request_by_name(dev, "cs-gpios", 0, &plat->cs,
					 GPIOD_IS_OUT))
			return -EINVAL;		


	spi0_init();

	return 0;
}

static const struct udevice_id sunxi_spi_ids[] = {
	{ .compatible = "sunxi_spi" },
	{ }
};

U_BOOT_DRIVER(sunxi_spi) = {
	.name	= "sunxi_spi",
	.id	= UCLASS_SPI,
	.of_match = sunxi_spi_ids,
	.ops	= &sunxi_spi_ops,
	.ofdata_to_platdata = sunxi_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct sunxi_spi_platdata),
	.priv_auto_alloc_size = sizeof(struct sunxi_spi_priv),
	.probe	= sunxi_spi_probe,
};
