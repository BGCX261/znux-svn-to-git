/***************************************************************************/
/*
 *	linux/arch/m68knommu/platform/520x/spi-mcf520x.c
 *
 *	Sub-architcture dependant initialization code for the Freescale
 *	520x SPI module
 *
 *	Yaroslav Vinogradov yaroslav.vinogradov@freescale.com
 *	Copyright Freescale Semiconductor, Inc 2006
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or 
 *	(at your option) any later version.
 */
/***************************************************************************/


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcfqspi.h>

#include <asm/dma.h>
#include <asm/traps.h>
#include <asm/machdep.h>
#include <asm/coldfire.h>
#include <asm/mcfsim.h>
#include <asm/mcfdma.h>

#define SPI_NUM_CHIPSELECTS 	0x04
// on the 5208EVB, the mc13192 is on CS2
#define SPI_PAR_VAL		0x3F  /* Enable DIN, DOUT, CLK, CS2 */

#define MCF520x_QSPI_IRQ_SOURCE	(31)
#define MCF520x_QSPI_IRQ_VECTOR	(64 + MCF520x_QSPI_IRQ_SOURCE)

#define MCF520x_QSPI_PAR	(0xFC0A4034)
#define MCF520x_QSPI_QMR	(0xFC05C000)
#define MCF520x_INTC_ICR	(0xFC048040)
#define MCF520x_INTC_IMRL	(0xFC04800C)

#define MCF520x_QSPI_DDR      (0xFC0A4010)
#define MCF520x_QSPI_PPDSDR   (0xFC0A401C)
#define MCF520x_QSPI_PCLRR    (0xFC0A4028)

#define MC13192_IRQ_SOURCE 1
#define MC13192_IRQ_VECTOR (64+MC13192_IRQ_SOURCE)
#define MC13192_IRQ_LEVEL	4

#define MC13192_CS      2

void coldfire_qspi_cs_control(u8 cs, u8 command)
{   
#if defined(CONFIG_MC13192)
   /* 
    * we will control the chip selects ourselves since the CS must be driven
    * low at the beginning of a message, kept low for the message, and then
    * driven high at the end
    */
   if (cs == MC13192_CS)
   {
      if (command & QSPI_CS_INIT)
      {
         // take over control of the CS2 pin
         *((u8*)MCF520x_QSPI_PAR) &= 0x3F;
         // start high (disabled)
         *((u8*)MCF520x_QSPI_PPDSDR) = 0x08;
         *((u8*)MCF520x_QSPI_DDR) |= 0x08;
      }
      
      if (command & QSPI_CS_ASSERT)
      {
         *((u8*)MCF520x_QSPI_PCLRR) = 0xF7;
      }
      
      if (command & QSPI_CS_DROP)
      {
         *((u8*)MCF520x_QSPI_PPDSDR) = 0x08;
      }
   }
#endif
}

#if defined(CONFIG_MC13192)
static struct coldfire_spi_chip mc13192_chip_info = {
	.mode = SPI_MODE_0,
	.bits_per_word = 8,
	.del_cs_to_clk = 1,
	.del_after_trans = 1,
	.void_write_data = 0
};
#endif

static struct spi_board_info spi_board_info[] = {
#if defined(CONFIG_MC13192)
	{
		.modalias = "mc13192",
		.max_speed_hz = 8000000,
		.bus_num = 1,
		.chip_select = MC13192_CS,
		.irq = MC13192_IRQ_VECTOR,
		.platform_data = NULL,
		.controller_data = &mc13192_chip_info
	}
#endif
};

static struct coldfire_spi_master coldfire_master_info = {
	.bus_num = 1,
	.num_chipselect = SPI_NUM_CHIPSELECTS,
	.irq_source = MCF520x_QSPI_IRQ_SOURCE,
	.irq_vector = MCF520x_QSPI_IRQ_VECTOR,
	.irq_mask = (0x01 << MCF520x_QSPI_IRQ_SOURCE),
	.irq_lp = 0x5,  /* Level */
	.par_val = SPI_PAR_VAL,
	.par_val16 = 0,   /* not used on 520x */
	.cs_control = coldfire_qspi_cs_control,
};

static struct resource coldfire_spi_resources[] = {
	[0] = {
		.name = "qspi-par",
		.start = MCF520x_QSPI_PAR,
		.end = MCF520x_QSPI_PAR,
		.flags = IORESOURCE_MEM
	},

	[1] = {
		.name = "qspi-module",
		.start = MCF520x_QSPI_QMR,
		.end = MCF520x_QSPI_QMR + 0x18,
		.flags = IORESOURCE_MEM
	},

	[2] = {
		.name = "qspi-int-level",
		.start = MCF520x_INTC_ICR + MCF520x_QSPI_IRQ_SOURCE,
		.end = MCF520x_INTC_ICR + MCF520x_QSPI_IRQ_SOURCE,
		.flags = IORESOURCE_MEM
	},

	[3] = {
		.name = "qspi-int-mask",
		.start = MCF520x_INTC_IMRL,
		.end = MCF520x_INTC_IMRL,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device coldfire_spi = {
	.name = "coldfire-qspi",
	.id = -1,
	.resource = coldfire_spi_resources,
	.num_resources = ARRAY_SIZE(coldfire_spi_resources),
	.dev = {
		.platform_data = &coldfire_master_info,
	}
};

#if defined(CONFIG_MC13192)
static int __init init_mc13192(void)
{
   int retval = 0;
   
	/* EPORT initialization */
	MCF_EPORT_EPPAR &= ~(MCF_EPORT_EPPAR_EPPA1(MCF_EPORT_EPPAR_BOTH));
	MCF_EPORT_EPPAR |= MCF_EPORT_EPPAR_EPPA1(MCF_EPORT_EPPAR_FALLING);
	MCF_EPORT_EPDDR &= ~0x02;
	MCF_EPORT_EPIER |= MCF_EPORT_EPIER_EPIE1;
	/* enable interrupt source */
	MCF_INTC0_ICR1 = MC13192_IRQ_LEVEL;
	MCF_INTC0_CIMR = MC13192_IRQ_SOURCE;
	
	// for testing
	MCF_GPIO_PCLRR_TIMER = ~0x01;
	MCF_GPIO_PAR_TIMER   &= ~0x03;
	MCF_GPIO_PDDR_TIMER  |= 0x01;
	
	return retval;
}
#endif

static int __init spi_dev_init(void)
{
	int retval = 0;	
	
#if defined(CONFIG_MC13192)
	retval = init_mc13192();
	if (retval < 0)
	   goto out;
#endif

	retval = platform_device_register(&coldfire_spi);
	if (retval < 0)
		goto out;

	if (ARRAY_SIZE(spi_board_info))
		retval = spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));


out:
	return retval;
}

arch_initcall(spi_dev_init);
