/****************************************************************************/

/*
 *  m520xsim.h -- ColdFire 5207/5208 System Integration Module support.
 *
 *  (C) Copyright 2005, Intec Automation (mike@steroidmicros.com)
 */

/****************************************************************************/
#ifndef m520xsim_h
#define m520xsim_h
/****************************************************************************/

#define MCF_REG32(x) (*(volatile unsigned long  *)(x))
#define MCF_REG16(x) (*(volatile unsigned short *)(x))
#define MCF_REG08(x) (*(volatile unsigned char  *)(x))


/*
 *  Define the 5282 SIM register set addresses.
 */
#define MCFICM_INTC0        0x48000     /* Base for Interrupt Ctrl 0 */
#define MCFINTC_IPRH        0x00        /* Interrupt pending 32-63 */
#define MCFINTC_IPRL        0x04        /* Interrupt pending 1-31 */
#define MCFINTC_IMRH        0x08        /* Interrupt mask 32-63 */
#define MCFINTC_IMRL        0x0c        /* Interrupt mask 1-31 */
#define MCFINTC_INTFRCH     0x10        /* Interrupt force 32-63 */
#define MCFINTC_INTFRCL     0x14        /* Interrupt force 1-31 */
#define MCFINTC_ICR0        0x40        /* Base ICR register */

#define MCFINT_VECBASE      64
#define MCFINT_UART0        26          /* Interrupt number for UART0 */
#define MCFINT_UART1        27          /* Interrupt number for UART1 */
#define MCFINT_UART2        28          /* Interrupt number for UART2 */
#define MCFINT_QSPI         31          /* Interrupt number for QSPI */
#define MCFINT_PIT1         4           /* Interrupt number for PIT1 (PIT0 in processor) */

/*
 *  SDRAM configuration registers.
 */
#define MCFSIM_SDMR         0x000a8000	/* SDRAM Mode/Extended Mode Register */
#define MCFSIM_SDCR         0x000a8004	/* SDRAM Control Register */
#define MCFSIM_SDCFG1       0x000a8008	/* SDRAM Configuration Register 1 */
#define MCFSIM_SDCFG2       0x000a800c	/* SDRAM Configuration Register 2 */
#define MCFSIM_SDCS0        0x000a8110	/* SDRAM Chip Select 0 Configuration */
#define MCFSIM_SDCS1        0x000a8114	/* SDRAM Chip Select 1 Configuration */


#define MCF_GPIO_PAR_UART                   (0xA4036)
#define MCF_GPIO_PAR_FECI2C                 (0xA4033)
#define MCF_GPIO_PAR_FEC                    (0xA4038)

#define MCF_GPIO_PDDR_UART                  MCF_REG08(0xFC0A4012)

#define MCF_GPIO_PPDSDR_UART                MCF_REG08(0xFC0A401E)
#define MCF_GPIO_PCLRR_UART                 MCF_REG08(0xFC0A402A)

#define MCF_GPIO_PAR_UART_PAR_URXD0         (0x0001)
#define MCF_GPIO_PAR_UART_PAR_UTXD0         (0x0002)

#define MCF_GPIO_PAR_UART_PAR_URTS0         (0x000C)
#define MCF_GPIO_PAR_UART_PAR_UCTS0         (0x0030)

#define MCF_GPIO_PAR_UART_PAR_URXD1         (0x0040)
#define MCF_GPIO_PAR_UART_PAR_UTXD1         (0x0080)

#define MCF_GPIO_PAR_UART_PAR_URTS1         (0x0300)
#define MCF_GPIO_PAR_UART_PAR_UCTS1         (0x0C00)

#define MCF_GPIO_UART_U0RTS                 (0x04)
#define MCF_GPIO_UART_U1RTS                 (0x40)


#define MCF_GPIO_PAR_FECI2C_PAR_SDA_URXD2   (0x02)
#define MCF_GPIO_PAR_FECI2C_PAR_SCL_UTXD2   (0x04)

#define MCF_GPIO_PAR_TIMER                  MCF_REG08(0xFC0A4035)
#define MCF_GPIO_PDDR_TIMER                 MCF_REG08(0xFC0A4011)
#define MCF_GPIO_PPDSDR_TIMER               MCF_REG08(0xFC0A401D)
#define MCF_GPIO_PCLRR_TIMER                MCF_REG08(0xFC0A4029)

#define ICR_INTRCONF		0x05
#define MCFPIT_IMR		MCFINTC_IMRL
#define MCFPIT_IMR_IBIT		(1 << MCFINT_PIT1)

#define MCFSIM_IMR_SIMR    0xFC04801C
#define MCFSIM_IMR_CIMR    0xFC04801D

#define mcf_enable_irq(irq)		\
	*((volatile unsigned char*) (MCFSIM_IMR_CIMR)) = (irq);

#define mcf_disable_irq(irq)		\
	*((volatile unsigned char*) (MCFSIM_IMR_SIMR)) = (irq);
	
/*********************************************************************
 *
 * Edge Port Module (EPORT)
 *
 *********************************************************************/

/* Register read/write macros */
#define MCF_EPORT_EPPAR                MCF_REG16(0xFC088000)
#define MCF_EPORT_EPDDR                MCF_REG08(0xFC088002)
#define MCF_EPORT_EPIER                MCF_REG08(0xFC088003)
#define MCF_EPORT_EPDR                 MCF_REG08(0xFC088004)
#define MCF_EPORT_EPPDR                MCF_REG08(0xFC088005)
#define MCF_EPORT_EPFR                 MCF_REG08(0xFC088006)

/* Bit definitions and macros for MCF_EPORT_EPPAR */
#define MCF_EPORT_EPPAR_EPPA1(x)       (((x)&0x0003)<<2)
#define MCF_EPORT_EPPAR_EPPA2(x)       (((x)&0x0003)<<4)
#define MCF_EPORT_EPPAR_EPPA3(x)       (((x)&0x0003)<<6)
#define MCF_EPORT_EPPAR_EPPA4(x)       (((x)&0x0003)<<8)
#define MCF_EPORT_EPPAR_EPPA5(x)       (((x)&0x0003)<<10)
#define MCF_EPORT_EPPAR_EPPA6(x)       (((x)&0x0003)<<12)
#define MCF_EPORT_EPPAR_EPPA7(x)       (((x)&0x0003)<<14)
#define MCF_EPORT_EPPAR_LEVEL          (0)
#define MCF_EPORT_EPPAR_RISING         (1)
#define MCF_EPORT_EPPAR_FALLING        (2)
#define MCF_EPORT_EPPAR_BOTH           (3)
#define MCF_EPORT_EPPAR_EPPA7_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA7_RISING   (0x4000)
#define MCF_EPORT_EPPAR_EPPA7_FALLING  (0x8000)
#define MCF_EPORT_EPPAR_EPPA7_BOTH     (0xC000)
#define MCF_EPORT_EPPAR_EPPA6_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA6_RISING   (0x1000)
#define MCF_EPORT_EPPAR_EPPA6_FALLING  (0x2000)
#define MCF_EPORT_EPPAR_EPPA6_BOTH     (0x3000)
#define MCF_EPORT_EPPAR_EPPA5_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA5_RISING   (0x0400)
#define MCF_EPORT_EPPAR_EPPA5_FALLING  (0x0800)
#define MCF_EPORT_EPPAR_EPPA5_BOTH     (0x0C00)
#define MCF_EPORT_EPPAR_EPPA4_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA4_RISING   (0x0100)
#define MCF_EPORT_EPPAR_EPPA4_FALLING  (0x0200)
#define MCF_EPORT_EPPAR_EPPA4_BOTH     (0x0300)
#define MCF_EPORT_EPPAR_EPPA3_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA3_RISING   (0x0040)
#define MCF_EPORT_EPPAR_EPPA3_FALLING  (0x0080)
#define MCF_EPORT_EPPAR_EPPA3_BOTH     (0x00C0)
#define MCF_EPORT_EPPAR_EPPA2_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA2_RISING   (0x0010)
#define MCF_EPORT_EPPAR_EPPA2_FALLING  (0x0020)
#define MCF_EPORT_EPPAR_EPPA2_BOTH     (0x0030)
#define MCF_EPORT_EPPAR_EPPA1_LEVEL    (0x0000)
#define MCF_EPORT_EPPAR_EPPA1_RISING   (0x0004)
#define MCF_EPORT_EPPAR_EPPA1_FALLING  (0x0008)
#define MCF_EPORT_EPPAR_EPPA1_BOTH     (0x000C)

/* Bit definitions and macros for MCF_EPORT_EPDDR */
#define MCF_EPORT_EPDDR_EPDD1          (0x02)
#define MCF_EPORT_EPDDR_EPDD2          (0x04)
#define MCF_EPORT_EPDDR_EPDD3          (0x08)
#define MCF_EPORT_EPDDR_EPDD4          (0x10)
#define MCF_EPORT_EPDDR_EPDD5          (0x20)
#define MCF_EPORT_EPDDR_EPDD6          (0x40)
#define MCF_EPORT_EPDDR_EPDD7          (0x80)

/* Bit definitions and macros for MCF_EPORT_EPIER */
#define MCF_EPORT_EPIER_EPIE1          (0x02)
#define MCF_EPORT_EPIER_EPIE2          (0x04)
#define MCF_EPORT_EPIER_EPIE3          (0x08)
#define MCF_EPORT_EPIER_EPIE4          (0x10)
#define MCF_EPORT_EPIER_EPIE5          (0x20)
#define MCF_EPORT_EPIER_EPIE6          (0x40)
#define MCF_EPORT_EPIER_EPIE7          (0x80)

/* Bit definitions and macros for MCF_EPORT_EPDR */
#define MCF_EPORT_EPDR_EPD1            (0x02)
#define MCF_EPORT_EPDR_EPD2            (0x04)
#define MCF_EPORT_EPDR_EPD3            (0x08)
#define MCF_EPORT_EPDR_EPD4            (0x10)
#define MCF_EPORT_EPDR_EPD5            (0x20)
#define MCF_EPORT_EPDR_EPD6            (0x40)
#define MCF_EPORT_EPDR_EPD7            (0x80)

/* Bit definitions and macros for MCF_EPORT_EPPDR */
#define MCF_EPORT_EPPDR_EPPD1          (0x02)
#define MCF_EPORT_EPPDR_EPPD2          (0x04)
#define MCF_EPORT_EPPDR_EPPD3          (0x08)
#define MCF_EPORT_EPPDR_EPPD4          (0x10)
#define MCF_EPORT_EPPDR_EPPD5          (0x20)
#define MCF_EPORT_EPPDR_EPPD6          (0x40)
#define MCF_EPORT_EPPDR_EPPD7          (0x80)

/* Bit definitions and macros for MCF_EPORT_EPFR */
#define MCF_EPORT_EPFR_EPF1            (0x02)
#define MCF_EPORT_EPFR_EPF2            (0x04)
#define MCF_EPORT_EPFR_EPF3            (0x08)
#define MCF_EPORT_EPFR_EPF4            (0x10)
#define MCF_EPORT_EPFR_EPF5            (0x20)
#define MCF_EPORT_EPFR_EPF6            (0x40)
#define MCF_EPORT_EPFR_EPF7            (0x80)

/*********************************************************************
 *
 * Interrupt Controller (INTC)
 *
 *********************************************************************/

/* Register read/write macros */
#define MCF_INTC0_IPRH             MCF_REG32(0xFC048000)
#define MCF_INTC0_IPRL             MCF_REG32(0xFC048004)
#define MCF_INTC0_IMRH             MCF_REG32(0xFC048008)
#define MCF_INTC0_IMRL             MCF_REG32(0xFC04800C)
#define MCF_INTC0_INTFRCH          MCF_REG32(0xFC048010)
#define MCF_INTC0_INTFRCL          MCF_REG32(0xFC048014)
#define MCF_INTC0_ICONFIG          MCF_REG16(0xFC04801A)
#define MCF_INTC0_SIMR             MCF_REG08(0xFC04801C)
#define MCF_INTC0_CIMR             MCF_REG08(0xFC04801D)
#define MCF_INTC0_CLMASK           MCF_REG08(0xFC04801E)
#define MCF_INTC0_SLMASK           MCF_REG08(0xFC04801F)
#define MCF_INTC0_ICR0             MCF_REG08(0xFC048040)
#define MCF_INTC0_ICR1             MCF_REG08(0xFC048041)
#define MCF_INTC0_ICR2             MCF_REG08(0xFC048042)
#define MCF_INTC0_ICR3             MCF_REG08(0xFC048043)
#define MCF_INTC0_ICR4             MCF_REG08(0xFC048044)
#define MCF_INTC0_ICR5             MCF_REG08(0xFC048045)
#define MCF_INTC0_ICR6             MCF_REG08(0xFC048046)
#define MCF_INTC0_ICR7             MCF_REG08(0xFC048047)
#define MCF_INTC0_ICR8             MCF_REG08(0xFC048048)
#define MCF_INTC0_ICR9             MCF_REG08(0xFC048049)
#define MCF_INTC0_ICR10            MCF_REG08(0xFC04804A)
#define MCF_INTC0_ICR11            MCF_REG08(0xFC04804B)
#define MCF_INTC0_ICR12            MCF_REG08(0xFC04804C)
#define MCF_INTC0_ICR13            MCF_REG08(0xFC04804D)
#define MCF_INTC0_ICR14            MCF_REG08(0xFC04804E)
#define MCF_INTC0_ICR15            MCF_REG08(0xFC04804F)
#define MCF_INTC0_ICR16            MCF_REG08(0xFC048050)
#define MCF_INTC0_ICR17            MCF_REG08(0xFC048051)
#define MCF_INTC0_ICR18            MCF_REG08(0xFC048052)
#define MCF_INTC0_ICR19            MCF_REG08(0xFC048053)
#define MCF_INTC0_ICR20            MCF_REG08(0xFC048054)
#define MCF_INTC0_ICR21            MCF_REG08(0xFC048055)
#define MCF_INTC0_ICR22            MCF_REG08(0xFC048056)
#define MCF_INTC0_ICR23            MCF_REG08(0xFC048057)
#define MCF_INTC0_ICR24            MCF_REG08(0xFC048058)
#define MCF_INTC0_ICR25            MCF_REG08(0xFC048059)
#define MCF_INTC0_ICR26            MCF_REG08(0xFC04805A)
#define MCF_INTC0_ICR27            MCF_REG08(0xFC04805B)
#define MCF_INTC0_ICR28            MCF_REG08(0xFC04805C)
#define MCF_INTC0_ICR29            MCF_REG08(0xFC04805D)
#define MCF_INTC0_ICR30            MCF_REG08(0xFC04805E)
#define MCF_INTC0_ICR31            MCF_REG08(0xFC04805F)
#define MCF_INTC0_ICR32            MCF_REG08(0xFC048060)
#define MCF_INTC0_ICR33            MCF_REG08(0xFC048061)
#define MCF_INTC0_ICR34            MCF_REG08(0xFC048062)
#define MCF_INTC0_ICR35            MCF_REG08(0xFC048063)
#define MCF_INTC0_ICR36            MCF_REG08(0xFC048064)
#define MCF_INTC0_ICR37            MCF_REG08(0xFC048065)
#define MCF_INTC0_ICR38            MCF_REG08(0xFC048066)
#define MCF_INTC0_ICR39            MCF_REG08(0xFC048067)
#define MCF_INTC0_ICR40            MCF_REG08(0xFC048068)
#define MCF_INTC0_ICR41            MCF_REG08(0xFC048069)
#define MCF_INTC0_ICR42            MCF_REG08(0xFC04806A)
#define MCF_INTC0_ICR43            MCF_REG08(0xFC04806B)
#define MCF_INTC0_ICR44            MCF_REG08(0xFC04806C)
#define MCF_INTC0_ICR45            MCF_REG08(0xFC04806D)
#define MCF_INTC0_ICR46            MCF_REG08(0xFC04806E)
#define MCF_INTC0_ICR47            MCF_REG08(0xFC04806F)
#define MCF_INTC0_ICR48            MCF_REG08(0xFC048070)
#define MCF_INTC0_ICR49            MCF_REG08(0xFC048071)
#define MCF_INTC0_ICR50            MCF_REG08(0xFC048072)
#define MCF_INTC0_ICR51            MCF_REG08(0xFC048073)
#define MCF_INTC0_ICR52            MCF_REG08(0xFC048074)
#define MCF_INTC0_ICR53            MCF_REG08(0xFC048075)
#define MCF_INTC0_ICR54            MCF_REG08(0xFC048076)
#define MCF_INTC0_ICR55            MCF_REG08(0xFC048077)
#define MCF_INTC0_ICR56            MCF_REG08(0xFC048078)
#define MCF_INTC0_ICR57            MCF_REG08(0xFC048079)
#define MCF_INTC0_ICR58            MCF_REG08(0xFC04807A)
#define MCF_INTC0_ICR59            MCF_REG08(0xFC04807B)
#define MCF_INTC0_ICR60            MCF_REG08(0xFC04807C)
#define MCF_INTC0_ICR61            MCF_REG08(0xFC04807D)
#define MCF_INTC0_ICR62            MCF_REG08(0xFC04807E)
#define MCF_INTC0_ICR63            MCF_REG08(0xFC04807F)
#define MCF_INTC0_ICR(x)           MCF_REG08(0xFC048040+((x)*0x001))

/****************************************************************************/
#endif  /* m520xsim_h */
