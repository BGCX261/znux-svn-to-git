/*
 * mc13192 wireless transceiver driver
 *
 *  Andrew Webster (awebster@arcx.com)
 *  Copyright (c) 2007 Andrew Webster
 *
 * Using code from:
 *  -ads7843.c
 *  Copyright Freescale Semiconductor, Inc 2006
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#if defined(CONFIG_M520x)
#include <asm/coldfire.h>
#include <asm/mcfsim.h>

#define coldfire_enable_irq(irq) mcf_enable_irq(irq-64)
#define coldfire_disable_irq(irq) mcf_disable_irq(irq-64)

#else
#define coldfire_enable_irq(irq)
#define coldfire_disable_irq(irq)
#endif

static const int watchdog = 1000;

// what should this be?
#define SAMPLE_BITS (32)

typedef u8 mc13192_msg_buf[3];

/* store this information for the driver.. */
struct mc13192_local {
	/*
	 * Since the chip has to be operated in stream mode due to a bug in packet
	 * mode, this is where we store the packet we are currently sending
	 */
	struct sk_buff *pending_tx_skb;
	/*
	 * Here is a pointer to the next byte to be sent
	 */
	char* pending_tx_data;
	int pending_tx_len;
	struct tasklet_struct tx_task;
	
	struct sk_buff *pending_rx_skb;
	int pending_rx_len;

 	/*
	 * these are things that the kernel wants me to keep
	 */
	struct net_device_stats stats;
	
	spinlock_t		lock;
	
	struct spi_device* spi;
	
	struct work_struct start_tx;
	struct work_struct check_status;
	
	struct net_device* ndev;
	
	/*
	 * Here is an SPI message buffer for async operation
	 */
   mc13192_msg_buf buf_req;
   mc13192_msg_buf buf_res;
   struct spi_transfer buf_xfer;
   struct spi_message  buf_msg;
};

static void
mc13192_check_status(void* data);

#if defined(CONFIG_M520x)

#define RXTXEN_ENABLE()  (MCF_GPIO_PPDSDR_UART = MCF_GPIO_PAR_UART_PAR_URTS0)
#define RXTXEN_DISABLE() (MCF_GPIO_PCLRR_UART = ~MCF_GPIO_PAR_UART_PAR_URTS0)

#define ATTN_ENABLE()    (MCF_GPIO_PCLRR_UART = ~MCF_GPIO_PAR_UART_PAR_URTS1)
#define ATTN_DISABLE()   (MCF_GPIO_PPDSDR_UART = MCF_GPIO_PAR_UART_PAR_URTS1)

#endif

#define READ_ADDR(addr)  (addr | 0x80)
#define WRITE_ADDR(addr) (addr)

static inline void
mc13192_spi_read(mc13192_msg_buf buf, u8 addr)
{
   buf[0] = READ_ADDR(addr);
   // doesn't matter what data is sent for a read
   buf[1] = 0;
   buf[2] = 0;
}

static inline void
mc13192_spi_write(mc13192_msg_buf buf, u8 addr, u16 data)
{
   buf[0] = WRITE_ADDR(addr);
   buf[1] = data >> 8;
   buf[2] = data & 0xFF;
}

// SPI registers
#define MC13192_REG_RESET           0x00
#define MC13192_REG_RX_PKT_RAM      0x01
#define MC13192_REG_TX_PKT_RAM      0x02
#define MC13192_REG_TX_PKT_CTL      0x03
#define MC13192_REG_CCA_THRESH      0x04
#define MC13192_REG_IRQ_MASK        0x05
#define MC13192_REG_CONTROL_A       0x06
#define MC13192_REG_CONTROL_B       0x07
#define MC13192_REG_PA_ENABLE       0x08
#define MC13192_REG_CONTROL_C       0x09
#define MC13192_REG_CLKO_CTL        0x0A
#define MC13192_REG_GPIO_DIR        0x0B
#define MC13192_REG_GPIO_DATA_OUT   0x0C
#define MC13192_REG_LO1_INT_DIV     0x0F
#define MC13192_REG_LO1_NUM         0x10
#define MC13192_REG_HIDDEN_3        0x11
#define MC13192_REG_PA_LVL          0x12
#define MC13192_REG_TMR_CMP1_A      0x1B
#define MC13192_REG_TMR_CMP1_B      0x1C
#define MC13192_REG_TMR_CMP2_A      0x1D
#define MC13192_REG_TMR_CMP2_B      0x1E
#define MC13192_REG_TMR_CMP3_A      0x1F
#define MC13192_REG_TMR_CMP3_B      0x20
#define MC13192_REG_TMR_CMP4_A      0x21
#define MC13192_REG_TMR_CMP4_B      0x22
#define MC13192_REG_TC2_PRIME       0x23
#define MC13192_REG_IRQ_STATUS      0x24
#define MC13192_REG_RST_IND         0x25
#define MC13192_REG_CURRENT_TIME_A  0x26
#define MC13192_REG_CURRENT_TIME_B  0x27
#define MC13192_REG_GPIO_DATA_IN    0x28
#define MC13192_REG_CHIP_ID         0x2C
#define MC13192_REG_RX_STATUS       0x2D
#define MC13192_REG_TIMESTAMP_A     0x2E
#define MC13192_REG_TIMESTAMP_B     0x2F
#define MC13192_REG_BER_ENABLE      0x30
#define MC13192_REG_PSM_MODE        0x31

#define MC13192_REG_CONTROL_A_MODE_MASK   0x0003
#define MC13192_REG_CONTROL_A_MODE_IDLE   0x0000
#define MC13192_REG_CONTROL_A_MODE_CCA    0x0001
#define MC13192_REG_CONTROL_A_MODE_RX     0x0002
#define MC13192_REG_CONTROL_A_MODE_TX     0x0003

#define MC13192_REG_CONTROL_A_STRM_MASK   0x1800
#define MC13192_REG_CONTROL_A_IDLE_STRM   0x0000
#define MC13192_REG_CONTROL_A_TX_STRM     0x1000
#define MC13192_REG_CONTROL_A_RX_STRM     0x0800

#define MC13192_REG_IRQ_STATUS_PLL_LOCK   0x8000
#define MC13192_REG_IRQ_STATUS_TX_DONE    0x4000
#define MC13192_REG_IRQ_STATUS_RX_DONE    0x2000
#define MC13192_REG_IRQ_STATUS_STRM_ERR   0x1000
#define MC13192_REG_IRQ_STATUS_ATTN       0x0400
#define MC13192_REG_IRQ_STATUS_DOZE       0x0200
#define MC13192_REG_IRQ_STATUS_TMR1       0x0100
#define MC13192_REG_IRQ_STATUS_RX_STRM    0x0080
#define MC13192_REG_IRQ_STATUS_TX_STRM    0x0040
#define MC13192_REG_IRQ_STATUS_CCA_DONE   0x0020
#define MC13192_REG_IRQ_STATUS_TMR3       0x0010
#define MC13192_REG_IRQ_STATUS_TMR4       0x0008
#define MC13192_REG_IRQ_STATUS_TMR2       0x0004
#define MC13192_REG_IRQ_STATUS_CCA        0x0002
#define MC13192_REG_IRQ_STATUS_CCA_VALID  0x0001

static void
mc13192_show_regs(struct mc13192_local *lp);

static int
mc13192_spi_flush(struct spi_device *spi, struct spi_message* msg)
{
   int status;
   
   //disable_irq(spi->irq);
   //coldfire_disable_irq(spi->irq);
   status = spi_sync(spi, msg);
   //enable_irq(spi->irq);
   //coldfire_enable_irq(spi->irq);
   
   if (msg->status)
   {
      printk("SPI transfer failed\n");
      status = msg->status;
   }
   
   return status;
}

static int
mc13192_read_single_async(struct mc13192_local *lp, u8 addr, void (*complete)(void*))
{
   struct spi_device *spi = lp->spi;
   
   mc13192_spi_read(lp->buf_req, addr);
   
   INIT_LIST_HEAD(&(lp->buf_msg.transfers));
   lp->buf_xfer.tx_buf = lp->buf_req;
   lp->buf_xfer.rx_buf = lp->buf_res;
   lp->buf_xfer.len = 3;
   
   lp->buf_msg.complete = complete;
   lp->buf_msg.context = lp;
   
   spi_message_add_tail(&lp->buf_xfer, &lp->buf_msg);
   
   return spi_async(spi, &lp->buf_msg);
}

static int
mc13192_read_single(struct mc13192_local *lp, u8 addr, u16* data)
{
   int status;
   struct spi_device	*spi = lp->spi;
   mc13192_msg_buf read_req;
   mc13192_msg_buf read_res;
   struct spi_transfer xfer;
   struct spi_message msg;
   
   mc13192_spi_read(read_req, addr);
   
   INIT_LIST_HEAD(&msg.transfers);
   xfer.tx_buf = read_req;
   xfer.rx_buf = read_res;
   xfer.len = 3;
   
   spi_message_add_tail(&xfer, &msg);
   
   status = mc13192_spi_flush(spi, &msg);
   
   if (status == 0)
   {
      *data = read_res[1] << 8 | read_res[2];
   }
   
   return status;      
}

static int
mc13192_write_single(struct mc13192_local *lp, u8 addr, u16 data)
{
   int status;
   struct spi_device	*spi = lp->spi;
   mc13192_msg_buf write_req;
   struct spi_transfer xfer;
   struct spi_message msg;
   
   mc13192_spi_write(write_req, addr, data);
   
   INIT_LIST_HEAD(&msg.transfers);
   xfer.tx_buf = write_req;
   xfer.rx_buf = NULL;
   xfer.len = 3;
   
   spi_message_add_tail(&xfer, &msg);
   
   status = mc13192_spi_flush(spi, &msg);
   
   return status;      
}

static inline int
mc13192_read_id(struct mc13192_local *lp, u16* id)
{
   return mc13192_read_single(lp, MC13192_REG_CHIP_ID, id);
}

static int
mc13192_start_rx(struct mc13192_local *lp)
{
   int status;
   u16 ctrl_a;
   
   printk("mc13192_start_rx\n");
   
   if (lp->pending_rx_skb)
   {
      printk("Start RX with unexpected pending RX packet\n");
      dev_kfree_skb(lp->pending_rx_skb);
   }
   lp->pending_rx_skb = NULL;
   
   RXTXEN_DISABLE();
   
   status = mc13192_read_single(lp, MC13192_REG_CONTROL_A, &ctrl_a);
   if (status)
   {
      return status;
   }
   
   ctrl_a &= ~MC13192_REG_CONTROL_A_STRM_MASK;   
   ctrl_a |= MC13192_REG_CONTROL_A_RX_STRM;
   status = mc13192_write_single(lp, MC13192_REG_CONTROL_A, ctrl_a);
   RXTXEN_ENABLE();
   
   return status;
}

static int mc13192_next_tx(struct mc13192_local* lp)
{
   if (lp->pending_tx_len)
   {
      // TODO: what should the byte order be?
      u16 data = *lp->pending_tx_data++;
      lp->pending_tx_len--;
      
      if (lp->pending_tx_len)
      {
         data |= (*lp->pending_tx_data++) << 8;
         lp->pending_tx_len--;
      }
      
      return mc13192_write_single(lp, MC13192_REG_TX_PKT_RAM, data);
   }
   /*
    * What to do if there is no data left? Time out the tx I guess
    */
    
   return -EINVAL;
}

#define NUM_START_TX_REGS 3
static void
mc13192_start_tx(void* data)
{
   struct mc13192_local *lp = (struct mc13192_local*)data;
   int status;
   u16 ctrl_a;
   int i;
   u16 reg;
   
   if (lp->pending_rx_skb)
   {
      /* 
       * this means we're in the middle of receiving, so hold off until
       * the RX is done
       */
      printk("Deffering TX until after RX\n");
      return;
   }
   
   printk("Starting TX\n");      
   
   if (lp->pending_tx_len < 2) return; /* paranoid */
   
   RXTXEN_DISABLE();
   
   status = mc13192_read_single(lp, MC13192_REG_CONTROL_A, &ctrl_a);
   if (status)
   {
      return;
   }
   
   ctrl_a &= ~MC13192_REG_CONTROL_A_STRM_MASK;
   ctrl_a |= MC13192_REG_CONTROL_A_TX_STRM;
   
   mc13192_write_single(lp, MC13192_REG_TX_PKT_CTL, lp->pending_tx_skb->len + 2);
   mc13192_next_tx(lp);
   mc13192_write_single(lp, MC13192_REG_CONTROL_A, ctrl_a);
   
   lp->pending_tx_data += 2;
   lp->pending_tx_len -= 2;
   
   RXTXEN_ENABLE();
   
   //mc13192_show_regs(lp);
}

static void
mc13192_show_regs(struct mc13192_local *lp)
{
   u16 reg;
   
   mc13192_read_single(lp, MC13192_REG_TX_PKT_CTL, &reg);
   printk("TX_PKT_CTL: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_TX_PKT_RAM, &reg);
   printk("TX_PKT_RAM: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_HIDDEN_3, &reg);
   printk("HIDDEN_3: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_TMR_CMP1_A, &reg);
   printk("TMR_CMP1_A: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_TMR_CMP2_A, &reg);
   printk("TMR_CMP1_A: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_TMR_CMP3_A, &reg);
   printk("TMR_CMP3_A: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_TMR_CMP4_A, &reg);
   printk("TMR_CMP4_A: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_CONTROL_B, &reg);
   printk("CONTROL_B: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_GPIO_DATA_OUT, &reg);
   printk("GPIO_DATA_OUT: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_CCA_THRESH, &reg);
   printk("CCA_THRESH: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_PA_ENABLE, &reg);
   printk("PA_ENABLE: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_IRQ_MASK, &reg);
   printk("IRQ_MASK: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_CONTROL_A, &reg);
   printk("CONTROL_A: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_LO1_INT_DIV, &reg);
   printk("LO1_INT_DIV: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_LO1_NUM, &reg);
   printk("LO1_NUM: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_PA_LVL, &reg);
   printk("PA_LVL: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_CONTROL_C, &reg);
   printk("CONTROL_C: %04X\n", reg);
   mc13192_read_single(lp, MC13192_REG_IRQ_STATUS, &reg);
   printk("IRQ_STATUS: %04X\n", reg);
}

#define NUM_INIT_XFER (16)

/*
 * Open and Initialize the board
 *
 * Set up everything, reset the chip, etc..
 */
static int
mc13192_open(struct net_device *dev)
{
	struct mc13192_local *lp = netdev_priv(dev);
	struct spi_device	*spi = lp->spi;
	int status;
	int i;
	u16 reg;
	
	/*
	 * Check that the address is valid.  If its not, refuse
	 * to bring the device up.  The user must specify an
	 * address using ifconfig wpanlrX hw ether xx:xx:xx:xx:xx:xx
	 */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		printk("%s: no valid ethernet hw addr\n", __FUNCTION__);
		return -EINVAL;
	}
	
	printk("mc13192_open\n");

   /*
    * It would be nice to do this all at once instead of waiting for each
    * message to complete before starting the next, but it has proven to be
    * difficult to get the CS to properly frame the transfers.  Revisit later?
    */
   status = mc13192_write_single(lp, MC13192_REG_HIDDEN_3, 0x80FF); /* Eliminate Unlock Conditions due to L01 */
	if (!status) status = mc13192_write_single(lp, MC13192_REG_TMR_CMP1_A, 0x8000); /* Disable TC1. */
	if (!status) status = mc13192_write_single(lp, MC13192_REG_TMR_CMP2_A, 0x8000); /* Disable TC2. */
	if (!status) status = mc13192_write_single(lp, MC13192_REG_TMR_CMP3_A, 0x8000); /* Disable TC3. */
	if (!status) status = mc13192_write_single(lp, MC13192_REG_TMR_CMP4_A, 0x8000); /* Disable TC4. */
	/*
	 * There is a bug in the hardware that prevents an even number of bytes from
	 * being received sometimes.  For that reason, streaming mode will be used
	 * so the compatibility with 802.15.4 will not be a problem later
	 */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_CONTROL_B, 0x0EE0); /* Enable CLKo in Doze */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_GPIO_DATA_OUT, 0x0300); /* IRQ pull-up disable. */
   if (!status) status = mc13192_read_single(lp, MC13192_REG_RST_IND, &reg); /* Sets the reset indicator bit */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_CCA_THRESH, 0xA08D); /* New cal value */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_PA_ENABLE, 0xFFF7); /* Preferred injection */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_IRQ_MASK, 0x8750); /* Acoma, TC1, Doze, ATTN masks, LO1, CRC */
   if (!status) status = mc13192_write_single(lp, MC13192_REG_CONTROL_A, 0x4720); /* CCA, TX, RX, energy detect */ 
   /* Read the status register to clear any undesired IRQs. */
   if (!status) status = mc13192_read_single(lp, MC13192_REG_IRQ_STATUS, &reg); /* Clear the status register, if set */
   printk("IRQ_STATUS: %04X\n", reg);
   
   // set to channel 0 always for now
   if (!status) status = mc13192_write_single(lp, MC13192_REG_LO1_INT_DIV, 0x0F95);
   if (!status) status = mc13192_write_single(lp, MC13192_REG_LO1_NUM, 0x5000);
   
   // enable alt GPIO functions
   if (!status) status = mc13192_write_single(lp, MC13192_REG_CONTROL_C, 0xF3CB);

   if (status)
   {
      return status;
   }
   
   // set the power level
   status = mc13192_read_single(lp, MC13192_REG_PA_LVL, &reg);
   if (status)
   {
      return status;
   }
   
   reg &= 0xFF00;
   reg |= 0x00BC; // nominal power
   
   status = mc13192_write_single(lp, MC13192_REG_PA_LVL, reg);
   if (status)
   {
      return status;
   }
   
   // finally, set it to RX mode
   //status = mc13192_start_rx(lp);
   
   if (!status)
   {
      netif_start_queue(dev);   
      printk("mc13192 opened successfully\n");
   }
   
   /*{
      u16 id;
      mc13192_read_id(lp, &id);
      printk("ID is %04X\n", id);
   }*/
   
	return status;
}

/*
 * this makes the board clean up everything that it can
 * and not talk to the outside world.   Caused by
 * an 'ifconfig ethX down'
 */
static int mc13192_close(struct net_device *dev)
{
	struct mc13192_local *lp = netdev_priv(dev);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	printk("mc13192_close\n");
	
	return mc13192_write_single(lp, MC13192_REG_RESET, 0x0000);
}

static int mc13192_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct mc13192_local *lp = netdev_priv(dev);
	
	printk("mc13192_tx\n");
	
	lp->pending_tx_skb = skb;
	lp->pending_tx_data = skb->data;
	lp->pending_tx_len = skb->len;
	
	// only accept one packet at a time
	netif_stop_queue(dev);
	
	// start the transmit process
	schedule_work(&lp->start_tx);

	return 0;
}

static void mc13192_timeout(struct net_device *dev)
{
	struct smc_local *lp = netdev_priv(dev);
	
	printk("mc13192_timeout\n");
	
	// TODO: we timed out on TX, fix the problem (reset?)

	/* We can accept TX packets again */
	dev->trans_start = jiffies;
	netif_wake_queue(dev);
}

static struct net_device_stats *mc13192_query_statistics(struct net_device *dev)
{
	struct mc13192_local *lp = netdev_priv(dev);

	return &lp->stats;
}

static void init_gpio(void)
{
#if defined(CONFIG_M520x)
   unsigned short *uart_par;
   uart_par = (unsigned short *)(MCF_IPSBAR + MCF_GPIO_PAR_UART);
   // RXTXEN is on UCTS0
   // ATTN is on UCTS1
   *uart_par &= ~(MCF_GPIO_PAR_UART_PAR_URTS0 |
                          MCF_GPIO_PAR_UART_PAR_URTS1);
   MCF_GPIO_PDDR_UART |= (MCF_GPIO_PAR_UART_PAR_URTS0 |
                          MCF_GPIO_PAR_UART_PAR_URTS1);
#endif
}

static void mc13192_status_complete(void *context)
{
   struct mc13192_local *lp = (struct mc13192_local*)context;
   u16 status = (lp->buf_res[1] << 8) | lp->buf_res[2];
   
   MCF_GPIO_PCLRR_TIMER = ~0x01;
   
   printk("mc13192 status: %04X (%02X)\n", status, lp->buf_res[0]);
}

static irqreturn_t mc13192_irq(int irq, void *handle)
{
#if defined(CONFIG_M520x)
	/* On M5208EVB we must confirm the EPORT interrupt to not to trigger it again */
	MCF_EPORT_EPFR = 0x82; //0x02;
#endif
   //printk("MC13192_irq\n");
   MCF_GPIO_PPDSDR_TIMER = 0x01;
   
   if (mc13192_read_single_async((struct mc13192_local*)handle, MC13192_REG_IRQ_STATUS, mc13192_status_complete))
   {
      printk("mc13192_irq: Failed to read status\n");
   }
   //schedule_work(&(((struct mc13192_local*)handle)->check_status));
   
	return IRQ_HANDLED;
}

static void
mc13192_check_status(void* data)
{
   struct mc13192_local* lp = (struct mc13192_local*)data;
   u16 status;
   
   
   if (mc13192_read_single(lp, MC13192_REG_IRQ_MASK, &status))
   {
      return;
   }
   printk("IRQ_MASK: %04X\n", status);
   
   /*
    * This will not work in an interrupt context!
    * 
    * In order to read the status register, the spi_async function may
    * be a better choice
    */
   if (mc13192_read_single(lp, MC13192_REG_IRQ_STATUS, &status))
   {
      return;
   }
   
   printk("mc13192 status: %04X\n", status);
   
   if (status & MC13192_REG_IRQ_STATUS_PLL_LOCK)
   {
      // TODO
   }
   
   if (status & MC13192_REG_IRQ_STATUS_TX_DONE)
   {
      lp->stats.tx_packets++;
      lp->stats.tx_bytes += lp->pending_tx_skb->len;
      dev_kfree_skb(lp->pending_tx_skb);
      
      lp->pending_tx_skb = NULL;
      lp->pending_tx_data = NULL;
      lp->pending_tx_len = 0;
      
      netif_wake_queue(lp->ndev);
      
      printk("TX done\n");
      
      //mc13192_start_rx(lp);
   }
   
   if (status & MC13192_REG_IRQ_STATUS_RX_DONE)
   {
#if 0
      if (lp->pending_rx_skb)
      {
         struct sk_buff* skb = lp->pending_rx_skb;
         skb->dev = lp->ndev;
         skb->protocol = eth_type_trans(skb, lp->ndev);
         /*
          * actually, the 802.15.4 packet is checked by the hardware, but
          * not the 802.3!
          */
         skb->ip_summed = CHECKSUM_NONE;
         lp->stats.rx_packets++;
         lp->stats.rx_bytes += lp->pending_rx_len;
         netif_rx(skb);
         
         lp->pending_rx_skb = NULL;
      }
      else
      {
         printk("RX finished without a packet\n");
      }
      
      if (lp->pending_tx_skb)
      {
         // there is a packet waiting to TX
         mc13192_start_tx((unsigned long)lp);
      }
      else
      {
         // does RX need to be started again?
         //mc13192_start_rx(lp);
      }
#endif
   }
   
   if (status & MC13192_REG_IRQ_STATUS_STRM_ERR)
   {
      printk("Stream error\n");
      
#if 0      
      if (lp->pending_rx_skb)
      {
         dev_kfree_skb(lp->pending_rx_skb);
         lp->stats.rx_dropped++;
         lp->pending_rx_skb = NULL;
         
         if (lp->pending_tx_skb)
         {
            mc13192_start_tx((unsigned long)lp);
         }
         else
         {
            mc13192_start_rx(lp);
         }
      }
      else if (lp->pending_tx_skb)
      {
         dev_kfree_skb(lp->pending_tx_skb);
         lp->stats.tx_dropped++;
         lp->pending_tx_skb = NULL;
         
         netif_wake_queue(lp->ndev);
         
         mc13192_start_rx(lp);
      }
#endif      
   }
   
   if (status & MC13192_REG_IRQ_STATUS_ATTN)
   {
      // TODO
   }
   
   if (status & MC13192_REG_IRQ_STATUS_DOZE)
   {
      // TODO
   }
   
   if (status & MC13192_REG_IRQ_STATUS_RX_STRM)
   {
#if 0
      if (lp->pending_rx_skb)
      {
         // read the next data word
         u16 data;
         
         if (mc13192_read_single(lp, MC13192_REG_RX_PKT_RAM, &data) == 0)
         {
            if (lp->pending_rx_len > 1)
            {
               memcpy(skb_put(lp->pending_rx_skb, 2), &data, 2);
               lp->pending_rx_len -= 2;
            }
            else if (lp->pending_rx_len == 1)
            {
               // data is in the lower bits?
               memcpy(skb_put(lp->pending_rx_skb, 1), ((char*)data)+1, 1);
               lp->pending_rx_len--;
            }
            else
            {
               printk("Unexpected RX_STRM\n");
            }
         }
      }
      else
      {
         // read the size of the next message
         u16 size;
         
         if (mc13192_read_single(lp, MC13192_REG_RX_STATUS, &size) == 0)
         {
            size &= 0x7F;
            
            if (size)
            {
               lp->pending_rx_skb = dev_alloc_skb(size + 2);
               
               if (!lp->pending_rx_skb)
               {
                  /*
                   * actually, this will not work very well and will likely lead
                   * to an RX error.  Revisit.
                   */
                  printk("Oops, out of memory, dropping RX packet\n");
                  
                  lp->stats.rx_dropped++;
                  
                  // restart the RX process
                  mc13192_start_rx(lp);
               }
               else
               {
                  printk("New RX message of size %u\n", size);
                  lp->pending_rx_len = size;
               }
            }
         }
      }
#endif
   }
   
   if (status & MC13192_REG_IRQ_STATUS_TX_STRM)
   {
      printk("Next TX\n");
      mc13192_next_tx(lp);
   }
   
   if (status & MC13192_REG_IRQ_STATUS_CCA_DONE)
   {
      // TODO
   }
   
   if (status & MC13192_REG_IRQ_STATUS_CCA)
   {
      // TODO
   }
   
   if (status & MC13192_REG_IRQ_STATUS_CCA_VALID)
   {
      // TODO
   }
}

/*--------------------------------------------------------------------------*/

static int
mc13192_suspend(struct spi_device *spi, pm_message_t message)
{
	struct net_device *ndev = dev_get_drvdata(&spi->dev);
	struct mc13192_local *priv = netdev_priv(ndev);
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);

	spi->dev.power.power_state = message;

	//disable_irq(spi->irq);
	//coldfire_disable_irq(spi->irq);
	
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static int mc13192_resume(struct spi_device *spi)
{
	//enable_irq(spi->irq);
	//coldfire_enable_irq(spi->irq);
	spi->dev.power.power_state = PMSG_ON;
	return 0;
}

static int __devinit mc13192_probe(struct spi_device *spi)
{
   struct net_device	*ndev;
   struct spi_transfer		*x;
   int				err;
   struct mc13192_local *lp;
   u16 id;

   printk(/*KERN_INFO*/ "mc13192: probe\n");

   if (!spi->irq) {
      printk(KERN_INFO "mc13192: no IRQ?\n");
      dev_dbg(&spi->dev, "no IRQ?\n");
      return -ENODEV;
   }
	
	if (spi->max_speed_hz > 8000000) {
		dev_dbg(&spi->dev, "f(sample) %d KHz?\n",
				(spi->max_speed_hz/SAMPLE_BITS)/1000);
		return -EINVAL;
	}

	ndev = alloc_etherdev(sizeof(struct mc13192_local));
   if (!ndev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	lp = netdev_priv(ndev);

	dev_set_drvdata(&spi->dev, ndev);
	spi->dev.power.power_state = PMSG_ON;
	lp->spi = spi;
	lp->ndev = ndev;
	
	if (mc13192_read_id(lp, &id) || id == 0 || id == 0xFFFF)
	{
	   err = -ENODEV;
      goto err_free_mem;
   }

	if (request_irq(spi->irq, mc13192_irq,
			/* SA_SAMPLE_RANDOM | SA_TRIGGER_FALLING*/ SA_INTERRUPT ,
			spi->dev.bus_id, lp)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}
	
	if (request_irq(67, mc13192_irq,
			/* SA_SAMPLE_RANDOM | SA_TRIGGER_FALLING*/ SA_INTERRUPT ,
			spi->dev.bus_id, lp)) {
		dev_dbg(&spi->dev, "irq 67 busy?\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	dev_info(&spi->dev, "transceiver, irq %d\n", spi->irq);
	
	/* Fill in the fields of the device structure with ethernet values. */
   ether_setup(ndev);

   ndev->open = mc13192_open;
   ndev->stop = mc13192_close;
   ndev->hard_start_xmit = mc13192_tx;
   ndev->tx_timeout = mc13192_timeout;
   ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
   ndev->get_stats = mc13192_query_statistics;
   strcpy(ndev->name, "wpanlr%d");
   ndev->mtu = 125; // override default ethernet value
   
   err = register_netdev(ndev);
   if (err)
      goto err_free_irq;
      
   init_gpio();
   RXTXEN_DISABLE();
   ATTN_DISABLE();
   
   INIT_WORK(&lp->start_tx, mc13192_start_tx, lp);
   INIT_WORK(&lp->check_status, mc13192_check_status, lp);

	printk(/*KERN_INFO*/ "mc13192: Probed successfully, chip ID is %04X\n", id);

	return 0;

 err_free_irq:
	free_irq(spi->irq, ndev);
 err_free_mem:
	free_netdev(ndev);
	return err;
}

static int __devexit mc13192_remove(struct spi_device *spi)
{
	struct net_device	*ndev = dev_get_drvdata(&spi->dev);

	mc13192_suspend(spi, PMSG_SUSPEND);
	dev_set_drvdata(&spi->dev, NULL);
	free_irq(ndev->irq, ndev);
	
	unregister_netdev(ndev);

	dev_dbg(&spi->dev, "unregistered mc13192\n");
	return 0;
}

static struct spi_driver mc13192_driver = {
	.driver = {
		.name	= "mc13192",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe      = mc13192_probe,
	.remove     = __devexit_p(mc13192_remove),
	.suspend    = mc13192_suspend,
	.resume     = mc13192_resume,
};

static int __init mc13192_init(void)
{
   int ret = 0;
   printk("init mc13192 Transceiver Driver\n");

   ret = spi_register_driver(&mc13192_driver);

   printk("mc13192: spi registered driver\n");
  return ret ;
}
module_init(mc13192_init);

static void __exit mc13192_exit(void)
{
	spi_unregister_driver(&mc13192_driver);
}
module_exit(mc13192_exit);

MODULE_DESCRIPTION("mc13192 Transceiver Driver");
MODULE_LICENSE("GPL");
