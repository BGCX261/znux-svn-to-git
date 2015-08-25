/****************************************************************************/

/*
 *	coldfire.c - Master QSPI controller for the ColdFire processors
 *
	Yaroslav Vinogradov (yaroslav.vinogradov@freescale.com)
 	Copyright Freescale Semiconductor, Inc. 2006

 *	(C) Copyright 2005, Intec Automation,
 *			    Mike Lavender (mike@steroidmicros)

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */


/****************************************************************************/

/*
 * Includes
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <asm/delay.h>
#include <asm/mcfsim.h>
#include <linux/spi/mcfqspi.h>
#include <asm/coldfire.h>


/****************************************************************************/

/*
 * Local constants and macros
 */

#define QSPI_RAM_SIZE		0x10    /* 16 word table */

#define QSPI_TRANSMIT_RAM 	0x00
#define QSPI_RECEIVE_RAM  	0x10
#define QSPI_COMMAND_RAM  	0x20

#define QSPI_COMMAND		0x7000
//#define QSPI_COMMAND		0x2000
					/* 15:   X = Continuous CS
					 * 14:   1 = Get BITSE from QMR[BITS]
					 * 13:   1 = Get DT    from QDLYR[DTL]
					 * 12:   1 = Get DSK   from QDLYR[QCD]
					 * 8-11: XXXX = next 4 bytes for CS
					 * 0-7:  0000 0000 Reserved
					 */

#define QIR_WCEF                0x0008  /* write collison */
#define QIR_ABRT                0x0004  /* abort */
#define QIR_SPIF                0x0001  /* finished */

#define QIR_WCEFE              	0x0800
#define QIR_ABRTE              	0x0400
#define QIR_SPIFE              	0x0100

#define QIR_WCEFB              	0x8000
#define QIR_ABRTB              	0x4000
#define QIR_ABRTL              	0x1000

#define QMR_BITS             	0x3C00
#define QMR_BITS_8              0x2000

#define QCR_CONT              	0x8000

#define QDLYR_SPE		0x8000

#define QDLYR_DTL(x) 	((x)&0x00FF)
#define QDLYR_QCD(x)	(((x)&0x007F)<<8)

#define QWR_ENDQP_MASK		0x0FF0
#define QWR_CSIV		0x1000  /* 1 = active low chip selects */

/****************************************************************************/

/*
 * Local Data Structures
 */

struct transfer_state {
	u32 index;
	u32 len;
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;
	char flags;
#define TRAN_STATE_RX_VOID	   0x01
#define TRAN_STATE_TX_VOID 	   0x02
#define TRAN_STATE_WORD_ODD_NUM	   0x04
	u8 cs;
	u16 void_write_data;
	unsigned cs_change:1;
};

typedef struct {
	unsigned master:1;
	unsigned dohie:1;
	unsigned bits:4;
	unsigned cpol:1;
	unsigned cpha:1;
	unsigned baud:8;
} QMR;

typedef struct {
	unsigned spe:1;
	unsigned qcd:7;
	unsigned dtl:8;
} QDLYR;

typedef struct {
	unsigned halt:1;
	unsigned wren:1;
	unsigned wrto:1;
	unsigned csiv:1;
	unsigned endqp:4;
	unsigned cptqp:4;
	unsigned newqp:4;
} QWR;


struct chip_data {
	union {
		u16 qmr_val;
		QMR qmr;
	};
	union {
		u16 qdlyr_val;
		QDLYR qdlyr;
	};
	union {
		u16 qwr_val;
		QWR qwr;
	};
	u16 void_write_data;
};


struct master_data {
	spinlock_t lock;
	struct spi_master *master;
	struct list_head queue;
	struct tasklet_struct pump_messages;
	struct tasklet_struct pump_transfers;
	struct spi_message* cur_msg;
	struct transfer_state cur_state;
	u32 trans_cnt;
	u32 wce_cnt;
	u32 abrt_cnt;
 	u16 *qmr;          /* QSPI mode register      */
 	u16 *qdlyr;        /* QSPI delay register     */
 	u16 *qwr;	   /* QSPI wrap register      */
 	u16 *qir;          /* QSPI interrupt register */
 	u16 *qar;          /* QSPI address register   */
 	u16 *qdr;          /* QSPI data register      */
 	u16 *qcr;	   /* QSPI command register   */
#if defined(CONFIG_M532x)
 	u16  *par;	   /* Pin assignment register */
#else
	u8  *par;	   /* Pin assignment register */
#endif
 	u8  *int_icr;	   /* Interrupt level and priority register */
 	u32 *int_mr;       /* Interrupt mask register */
 	void (*cs_control)(u8 cs, u8 command);
};



/****************************************************************************/

/*
 * SPI local functions
 */

#undef SPI_COLDFIRE_DEBUG
//#define SPI_COLDFIRE_DEBUG

static int write(struct master_data *drv_data, struct transfer_state *state)
{
	u16 tx_data;
 	int tx_count = 0;
 	int cmd_count = 0;
 	int tx_word = ((*drv_data->qmr & QMR_BITS) == QMR_BITS_8) ? 0 : 1;
	u16 qwr;

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: write\n");
#endif

	// If we are in word mode, but only have a single byte to transfer
	// then switch to byte mode temporarily.  Will switch back at the
	// end of the transfer.
 	if (tx_word && ((state->tx_end - state->tx) == 1)) {
 		state->flags |= TRAN_STATE_WORD_ODD_NUM;
 		*drv_data->qmr |= (*drv_data->qmr & ~QMR_BITS) | QMR_BITS_8;
 		tx_word = 0;
	}

 	*drv_data->qar = QSPI_TRANSMIT_RAM;
 	while ((state->tx < state->tx_end) && (tx_count < QSPI_RAM_SIZE)) {
 		if (tx_word) {
			if ((state->tx_end - state->tx) == 1)
				break;

 			if (!(state->flags & TRAN_STATE_TX_VOID)) {
				tx_data = *(u16 *)state->tx;
 				*drv_data->qdr = *(u16 *)state->tx;
#ifdef SPI_COLDFIRE_DEBUG
				printk("data16=%x\n", tx_data);
#endif
 			} else {
 				*drv_data->qdr = state->void_write_data;
#ifdef SPI_COLDFIRE_DEBUG
				printk("data16=void %x\n", state->void_write_data);
#endif
			}
 			state->tx += 2;
 		} else {
 			if (!(state->flags & TRAN_STATE_TX_VOID)) {
				tx_data = *(u8 *)state->tx;
 				*(drv_data->qdr) = tx_data;//*(u8 *)state->tx;
#ifdef SPI_COLDFIRE_DEBUG
				printk("data8=%x\n", tx_data);
#endif
			} else {
 				*drv_data->qdr = *(u8 *)&state->void_write_data;
#ifdef SPI_COLDFIRE_DEBUG
				printk("data8=%x\n", *(u8 *)&state->void_write_data);
#endif
			}
 			state->tx++;
 		}
 		tx_count++;
 	}


 	*drv_data->qar = QSPI_COMMAND_RAM;
 	while (cmd_count < tx_count) {
		u16 qcr =   QSPI_COMMAND
			  | QCR_CONT
			  | (~((0x01 << state->cs) << 8) & 0x0F00);

		if ((cmd_count == tx_count - 1)
			&& (state->tx == state->tx_end)
			&& (state->cs_change) ) {
			qcr &= ~QCR_CONT;
		}
#ifdef SPI_COLDFIRE_DEBUG
		printk("command=%x\n", qcr);
#endif

		*drv_data->qcr = qcr;
		cmd_count++;
	}

	qwr = (*drv_data->qwr & ~QWR_ENDQP_MASK) | ((cmd_count - 1) << 8);

	*drv_data->qwr = qwr;
#ifdef SPI_COLDFIRE_DEBUG
	printk("tx_count=%d\n", tx_count);
#endif
 	/* Fire it up! */
 	*drv_data->qdlyr |= QDLYR_SPE;

	//*drv_data->qdlyr = QDLYR_SPE | QDLYR_DTL(16) | QDLYR_QCD(10);


 	return tx_count;
}


static int read(struct master_data *drv_data, struct transfer_state *state)
{
	int rx_data;
	int rx_count = 0;
	int rx_word = ((*drv_data->qmr & QMR_BITS) == QMR_BITS_8) ? 0 : 1;

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: read\n");
#endif

	*drv_data->qar = QSPI_RECEIVE_RAM;
	while ((state->rx < state->rx_end) && (rx_count < QSPI_RAM_SIZE)) {
		if (rx_word) {
			if ((state->rx_end - state->rx) == 1)
				break;

			if (!(state->flags & TRAN_STATE_RX_VOID)) {
				*(u16 *)state->rx = *drv_data->qdr;
				rx_data = *(u16 *)state->rx;
#ifdef SPI_COLDFIRE_DEBUG
				printk("rx_data16=%x\n", rx_data);
#endif
			}
			state->rx += 2;
		} else {
			if (!(state->flags & TRAN_STATE_RX_VOID)) {
				rx_data = *drv_data->qdr;
				*(u8 *)state->rx = rx_data;

#ifdef SPI_COLDFIRE_DEBUG
				printk("rx_data8=%x\n", rx_data);
#endif
			}
			state->rx++;
		}
		rx_count++;
	}
#ifdef SPI_COLDFIRE_DEBUG
	printk("rx_count=%d\n", rx_count);
#endif
	return rx_count;
}


static inline void qspi_setup_chip(struct master_data *drv_data,
	struct chip_data *chip)
{

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: qspi_setup_chip\n");
	printk("QSPI: qmr=%x\n", chip->qmr_val);
	printk("QSPI: qdlyr=%x\n", chip->qdlyr_val);
	printk("QSPI: qwr=%x\n", chip->qwr_val);
#endif

	*drv_data->qmr = chip->qmr_val;
	*drv_data->qdlyr = chip->qdlyr_val;
	*drv_data->qwr = chip->qwr_val;

	/*
	 * Enable all the interrupts and clear all the flags
	 */
	*drv_data->qir =  (QIR_SPIFE | QIR_ABRTE | QIR_WCEFE)
			| (QIR_WCEFB | QIR_ABRTB | QIR_ABRTL)
			| (QIR_SPIF  | QIR_ABRT  | QIR_WCEF);
}


static irqreturn_t qspi_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct master_data *drv_data = (struct master_data *)dev_id;
	struct transfer_state *state = (struct transfer_state *)drv_data->cur_msg->state;
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer	*t = NULL;
	u32 i;
	u16 irq_status = *drv_data->qir;

	/* Clear all flags immediately */
	*drv_data->qir |= (QIR_SPIF | QIR_ABRT | QIR_WCEF);

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: qspi_interrupt\n");
#endif

	if (!drv_data->cur_msg || !drv_data->cur_msg->state) {
		printk(KERN_ERR "coldfire-qspi: bad message or transfer "
		"state in interrupt handler\n");
		return IRQ_NONE;
	}

	if (irq_status & QIR_SPIF) {
		/*
		 * Read the data into the buffer and reload and start
		 * queue with new data if not finished.  If finished
		 * then setup the next transfer
		 */
		 read(drv_data, state);

		 if (state->rx == state->rx_end) {
			/*
			 * Finished now - fall through and schedule next
			 * transfer tasklet
			 */
			if (state->flags & TRAN_STATE_WORD_ODD_NUM)
				*drv_data->qmr &= ~QMR_BITS;

			/* Ugly fix: to be fixed later */
			/* msg->actual_length += msg->transfers[state->index].len; */
			i = state->index;
			list_for_each_entry (t, &msg->transfers, transfer_list) {
				if (i==0) {
					msg->actual_length += t->len;
					break;
				}
				i--;
			}
			++state->index;
		 } else {
			/* not finished yet - keep going */
		 	write(drv_data, state);
		 	return IRQ_HANDLED;
		}
	} else {
		if (irq_status & QIR_WCEF)
			drv_data->wce_cnt++;

		if (irq_status & QIR_ABRT)
			drv_data->abrt_cnt++;

		state->index = -2;
	}

	tasklet_schedule(&drv_data->pump_transfers);

	return IRQ_HANDLED;
}


static void pump_transfers(unsigned long data)
{
	struct master_data *drv_data = (struct master_data *)data;
	struct spi_message *message = drv_data->cur_msg;
	struct chip_data *chip;
	struct transfer_state *state;
	struct spi_transfer *transfer;
	unsigned long flags;

	/* Ugly fix: */
	unsigned int s_index;
	unsigned int n_transfer = 0;
	list_for_each_entry (transfer, &message->transfers, transfer_list) {
		n_transfer++;
	}
	/* end of ugly fix */

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: pump_transfers\n");
#endif

	if (!message) {
		printk(KERN_ERR "coldfire-qspi: bad pump_transfers "
			"schedule\n");
		tasklet_schedule(&drv_data->pump_messages);
		return;
	}

	state = (struct transfer_state *)message->state;
	if (!state) {
		printk(KERN_ERR "coldfire-qspi: bad message state\n");
		drv_data->cur_msg = NULL;
 		tasklet_schedule(&drv_data->pump_messages);
		return;
	}

	chip = spi_get_ctldata(message->spi);
	if (!chip) {
		printk(KERN_ERR "coldfire-qspi: bad chip data\n");
		drv_data->cur_msg = NULL;
		tasklet_schedule(&drv_data->pump_messages);
		return;
	}

	/* Handle for abort */
	if (state->index == -2) {
		message->status = -EIO;
		if (message->complete) {
			message->complete(message->context);
		}
		drv_data->cur_msg = NULL;
		tasklet_schedule(&drv_data->pump_messages);
		return;
	}

	/* Handle end of message */
	/* ugly fix */
	
	/* if (state->index == message->n_transfer) { */
	if (state->index == n_transfer) {
		if (drv_data->cs_control) {
			drv_data->cs_control(state->cs, QSPI_CS_DROP);
		}

		message->status = 0;
		if (message->complete)
			message->complete(message->context);
		drv_data->cur_msg = NULL;
		tasklet_schedule(&drv_data->pump_messages);
		return;
	}

	/* Handle start of message */
	if (state->index == -1) {
		qspi_setup_chip(drv_data, chip);

		if (drv_data->cs_control) {
		   drv_data->cs_control(message->spi->chip_select, QSPI_CS_ASSERT);
		}

		++state->index;
	}

	/* ugly fix */
	s_index = state->index;
	list_for_each_entry (transfer, &message->transfers, transfer_list) {
		if (s_index == 0) break;
		s_index--;
	}


	/* Delay if requested at end of transfer*/
	if (state->index >= 1) {
		/* ugly fix
		transfer = message->transfers + (state->index - 1); */

		if (drv_data->cs_control && transfer->cs_change)
			drv_data->cs_control(message->spi->chip_select, QSPI_CS_DROP);

		if (transfer->delay_usecs) {
#ifdef SPI_COLDFIRE_DEBUG
			printk( "QSPI: delay %d\n", transfer->delay_usecs );
#endif
			udelay(transfer->delay_usecs);
		}

		if (drv_data->cs_control && transfer->cs_change)
			drv_data->cs_control(message->spi->chip_select, QSPI_CS_ASSERT);
	}

	/* Setup the transfer state based on the type of transfer */
	drv_data->trans_cnt++;
	/* ugly fix, but calculated once up in the code
	transfer = message->transfers + state->index; */
	state->len = transfer->len;

	/* Message has no length skip it and move on */
	if (!state->len) {
		++state->index;
		tasklet_schedule(&drv_data->pump_transfers);
	} else {
		state->flags = 0;
		state->tx = (void *)transfer->tx_buf;
		state->tx_end = state->tx + transfer->len;
		state->rx = transfer->rx_buf;
		state->rx_end = state->rx + transfer->len;
		if (!state->rx)
			state->flags |= TRAN_STATE_RX_VOID;
		if (!state->tx)
			state->flags |= TRAN_STATE_TX_VOID;
		state->cs = message->spi->chip_select;
		state->cs_change = transfer->cs_change;
		state->void_write_data = chip->void_write_data;

		/* Go baby, go */
		local_irq_save(flags);
		write(drv_data, state);
		local_irq_restore(flags);
	}
}


static void pump_messages(unsigned long data)
{
	struct master_data *drv_data = (struct master_data *)data;
	unsigned long flags;

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: pump_messages\n");
#endif

	spin_lock_irqsave(&drv_data->lock, flags);

	/* Check for list empty */
	if (list_empty(&drv_data->queue)) {
		spin_unlock(&drv_data->lock);
		return;
	}

	/* Check to see if we are already running */
	if (drv_data->cur_msg) {
		spin_unlock(&drv_data->lock);
		return;
	}

	/* Extract head of queue and check for tasklet reschedule */
	drv_data->cur_msg = list_entry(drv_data->queue.next,
		struct spi_message, queue);
	list_del_init(&drv_data->cur_msg->queue);

	/* Setup message transfer and schedule transfer pump */
	drv_data->cur_msg->state = &drv_data->cur_state;
	drv_data->cur_state.index = -1;
	drv_data->cur_state.len = 0;
	drv_data->cur_state.cs_change = 0;

	//tasklet_schedule(&drv_data->pump_transfers);
	pump_transfers(drv_data);

	spin_unlock_irqrestore(&drv_data->lock, flags);
}

/****************************************************************************/

/*
 * SPI master implementation
 */

static int transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct master_data *drv_data = class_get_devdata(&spi->master->cdev);
	unsigned long flags;

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: transfer\n");
#endif

	msg->actual_length = 0;
	msg->status = 0;

	spin_lock_irqsave(&drv_data->lock, flags);
	list_add_tail(&msg->queue, &drv_data->queue);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	//tasklet_schedule(&drv_data->pump_messages);
	pump_messages(drv_data);

	return 0;
}


static int setup(struct spi_device *spi)
{
	struct coldfire_spi_chip *chip_info;
	struct chip_data *chip;
	u32 baud_divisor = 255;

#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: setup\n");
#endif

	chip_info = (struct coldfire_spi_chip *)spi->controller_data;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
 		chip = kcalloc(1, sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi->mode = chip_info->mode;
		spi->bits_per_word = chip_info->bits_per_word;
	}

	chip->qwr.csiv = 1;    // Chip selects are active low
	chip->qmr.master = 1;  // Must set to master mode
	chip->qmr.dohie = 1;   // Data output high impediance enabled
	chip->void_write_data = chip_info->void_write_data;

	chip->qdlyr.qcd = chip_info->del_cs_to_clk;
	chip->qdlyr.dtl = chip_info->del_after_trans;

	if (spi->max_speed_hz != 0)
		baud_divisor = (MCF_BUSCLK/(2*spi->max_speed_hz));

	if (baud_divisor < 2)
		baud_divisor = 2;

	if (baud_divisor > 255)
		baud_divisor = 255;

	chip->qmr.baud = baud_divisor;

	printk( "QSPI: spi->max_speed_hz %d\n", spi->max_speed_hz );
	printk( "QSPI: Baud set to %d\n", chip->qmr.baud );

	if (spi->mode & SPI_CPHA)
		chip->qmr.cpha = 1;

	if (spi->mode & SPI_CPOL)
		chip->qmr.cpol = 1;

	if (spi->bits_per_word == 16) {
		chip->qmr.bits = 0;
	} else if ((spi->bits_per_word >= 8) && (spi->bits_per_word <= 15)) {
		chip->qmr.bits = spi->bits_per_word;
	} else {
		printk(KERN_ERR "coldfire-qspi: invalid wordsize\n");
		kfree(chip);
		return -ENODEV;
	}

 	spi_set_ctldata(spi, chip);

 	return 0;
}


static void cleanup(const struct spi_device *spi)
{
 	dev_dbg(&spi->dev, "spi_device %u.%u cleanup\n",
 		spi->master->bus_num, spi->chip_select);
}


/****************************************************************************/

/*
 * Generic Device driver routines and interface implementation
 */

static int probe(struct device *dev)
{
 	struct platform_device *pdev = to_platform_device(dev);
 	struct coldfire_spi_master *platform_info;
 	struct spi_master *master;
 	struct master_data *drv_data = 0;
 	struct resource *memory_resource;
	int status = 0;
	int i;


#ifdef SPI_COLDFIRE_DEBUG
	printk("QSPI: probe\n");
#endif

 	platform_info = (struct coldfire_spi_master *)pdev->dev.platform_data;

  	master = spi_alloc_master(dev, sizeof(struct master_data));
  	if (!master)
 		return -ENOMEM;

 	drv_data = class_get_devdata(&master->cdev);
 	drv_data->master = master;

	INIT_LIST_HEAD(&drv_data->queue);
	spin_lock_init(&drv_data->lock);

	master->bus_num = platform_info->bus_num;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = cleanup;
	master->setup = setup;
	master->transfer = transfer;


 	drv_data->cs_control = platform_info->cs_control;
 	if (drv_data->cs_control)
 		for(i = 0; i < master->num_chipselect; i++)
 			drv_data->cs_control(i, QSPI_CS_INIT | QSPI_CS_DROP);

	/* Setup register addresses */
 	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi-module");
 	if (!memory_resource) {
 		dev_dbg(dev, "can not find platform module memory\n");
 		status = -ENODEV;
 		return status;
 	}

 	drv_data->qmr   = (void *)(memory_resource->start + 0x00000000);
 	drv_data->qdlyr = (void *)(memory_resource->start + 0x00000004);
 	drv_data->qwr   = (void *)(memory_resource->start + 0x00000008);
 	drv_data->qir   = (void *)(memory_resource->start + 0x0000000c);
 	drv_data->qar   = (void *)(memory_resource->start + 0x00000010);
 	drv_data->qdr   = (void *)(memory_resource->start + 0x00000014);
 	drv_data->qcr   = (void *)(memory_resource->start + 0x00000014);

	/* Setup register addresses */
 	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi-par");
 	if (!memory_resource) {
 		dev_dbg(dev, "can not find platform par memory\n");
 		status = -ENODEV;
 		return status;
 	}

 	drv_data->par = (void *)memory_resource->start;

	/* Setup register addresses */
 	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi-int-level");
 	if (!memory_resource) {
 		dev_dbg(dev, "can not find platform par memory\n");
 		status = -ENODEV;
 		return status;
 	}

 	drv_data->int_icr = (void *)memory_resource->start;

	/* Setup register addresses */
 	memory_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi-int-mask");
 	if (!memory_resource) {
 		dev_dbg(dev, "can not find platform par memory\n");
 		status = -ENODEV;
 		return status;
 	}

 	drv_data->int_mr = (void *)memory_resource->start;

	/* Now that we have all the addresses etc.  Let's set it up */
#if defined(CONFIG_M532x)
	*drv_data->par = platform_info->par_val16;
#else
	*drv_data->par = platform_info->par_val;
#endif

	if (request_irq(platform_info->irq_vector, qspi_interrupt, SA_INTERRUPT, dev->bus_id, drv_data)) {
			dev_dbg(dev, "unable to attach ColdFire QSPI interrupt ");
			status = -EINVAL;
			return status;
	}

	*drv_data->int_icr = platform_info->irq_lp;
	*drv_data->int_mr &= ~platform_info->irq_mask;

	tasklet_init(&drv_data->pump_messages,
		pump_messages, (unsigned long)drv_data);

	tasklet_init(&drv_data->pump_transfers,
		pump_transfers, (unsigned long)drv_data);

	/* Register with the SPI framework */
	dev_set_drvdata(dev, master);
	status = spi_register_master(master);

	printk( "COLDFIRE-QSPI: probed and master registered\n" );

	return status;
}


static int remove(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);

	spi_unregister_master(master);

	return 0;
}

static struct device_driver driver = {
	.name = "coldfire-qspi",
	.bus = &platform_bus_type,
	.owner = THIS_MODULE,
	.probe = probe,
	.remove = remove,
};

static int coldfire_spi_init(void)
{
	driver_register(&driver);

	return 0;
}

static void coldfire_spi_exit(void)
{
	driver_unregister(&driver);
}

module_init(coldfire_spi_init);
module_exit(coldfire_spi_exit);

MODULE_DESCRIPTION("ColdFire QSPI Contoller");
MODULE_LICENSE("GPL");
