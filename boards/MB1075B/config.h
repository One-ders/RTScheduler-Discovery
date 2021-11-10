

//#define SYS_CLOCK	180000000
#define SYS_CLOCK	168000000

#define MAX_TASKS 	256
#define TQ_SIZE         1024

#define USB_TX_BSIZE	64
#define USB_RX_BSIZE 	512

#define USART_TX_BSIZE  1024
#define USART_RX_BSIZE	16

#define GPIO_PINMEM	256

/* define board */
#define MB1075B

#define SDRAM_SIZE	256*1024
#define BYTES_PER_LONG	4

#define HAVE_SDRAM
#define SDRAM_BITS_PER_COLUMNS 	8
#define SDRAM_BITS_PER_ROW 	11
#define SDRAM_MEM_WIDTH 	16
#define SDRAM_INTERNAL_BANKS 	4
#define SDRAM_CAS_LATENCY 	3

#define SDRAM_LOAD_TO_ACTIVE_DELAY	2
#define SDRAM_EXIT_SELF_REFRESH_DELAY	6
#define SDRAM_SELF_REFRESH_TIME 	4
#define SDRAM_ROWCYCLE_DELAY		6
#define SDRAM_WRITE_RECOVERY_TIME	2
#define SDRAM_RP_DELAY			2
#define SDRAM_RCD_DELAY			2

/* SDRAM Pins */
#define FMC_A0  GPIO_PIN(PF,0)
#define FMC_A1  GPIO_PIN(PF,1)
#define FMC_A2  GPIO_PIN(PF,2)
#define FMC_A3  GPIO_PIN(PF,3)
#define FMC_A4  GPIO_PIN(PF,4)
#define FMC_A5  GPIO_PIN(PF,5)
#define FMC_A6  GPIO_PIN(PF,12)
#define FMC_A7  GPIO_PIN(PF,13)
#define FMC_A8  GPIO_PIN(PF,14)
#define FMC_A9  GPIO_PIN(PF,15)
#define FMC_A10 GPIO_PIN(PG,0)
#define FMC_A11 GPIO_PIN(PG,1)

#define FMC_BA0 GPIO_PIN(PG,4)
#define FMC_BA1 GPIO_PIN(PG,5)

#define FMC_D0  GPIO_PIN(PD,14)
#define FMC_D1  GPIO_PIN(PD,15)
#define FMC_D2  GPIO_PIN(PD,0)
#define FMC_D3  GPIO_PIN(PD,1)
#define FMC_D4  GPIO_PIN(PE,7)
#define FMC_D5  GPIO_PIN(PE,8)
#define FMC_D6  GPIO_PIN(PE,9)
#define FMC_D7  GPIO_PIN(PE,10)
#define FMC_D8  GPIO_PIN(PE,11)
#define FMC_D9  GPIO_PIN(PE,12)
#define FMC_D10 GPIO_PIN(PE,13)
#define FMC_D11 GPIO_PIN(PE,14)
#define FMC_D12 GPIO_PIN(PE,15)
#define FMC_D13 GPIO_PIN(PD,8)
#define FMC_D14 GPIO_PIN(PD,9)
#define FMC_D15 GPIO_PIN(PD,10)

#define FMC_NBL0 GPIO_PIN(PE,0)
#define FMC_NBL1 GPIO_PIN(PE,1)
#define FMC_SDCLK  GPIO_PIN(PG,8)
#define FMC_SDNWE  GPIO_PIN(PC,0)
#define FMC_SDNRAS GPIO_PIN(PF,11)
#define FMC_SDNCAS GPIO_PIN(PG,15)
#define FMC_SDNE1  GPIO_PIN(PB,6)
#define FMC_SDCKE1 GPIO_PIN(PB,5)

#if 0

#define FMC_A12 PG2

#define FMC_NBL2 PI4
#define FMC_NBL3 PI5

#define FMC_SDNWE  PC0 | PH5
#define FMC_SDNE0  PC2 | PH3
#define FMC_SDCKE0 PC3 | PH2
#define FMC_SDNE1  PH6 | PB6
#define FMC_SDCKE1 PH7 | PB5

#define FMC_A23 PE2
#define FMC_A19 PE3
#define FMC_A20 PE4
#define FMC_A21 PE5
#define FMC_A22 PE6

#define FMC_D30 PI9
#define FMC_D31 PI10
#define FMC_NIORD PF6
#define FMC_NREG PF7
#define FMC_NIOWR PF8
#define FMC_CD  PF9
#define FMC_INTR PF10
#define FMC_D16 PH8
#define FMC_D17 PH9
#define FMC_D18 PH10
#define FMC_D19 PH11
#define FMC_D20 PH12
#define FMC_A16 PD11
#define FMC_A17 PD12
#define FMC_A18 PD13
#define FMC_A13 PG3
#define FMC_BA0|FMC_A14 PG4
#define FMC_BA1|FMC_A15 PG5
#define FMC_INT2 PG6
#define FMC_INT3 PG7
#define FMC_D21 PH13
#define FMC_D22 PH14
#define FMC_D23 PH15
#define FMC_D24 PI0
#define FMC_D25 PI1
#define FMC_D26 PI2
#define FMC_D27 PI3
#define FMC_CLK PD3
#define FMC_NOE PD4
#define FMC_NWE PD5
#define FMC_NWAIT PD6
#define FMC_NE1/FMC_NCE2 PD7
#define FMC_NE2/FMC_NCE3 PG9
#define FMC_NCE4/FMC_NE3 PG10
#define FMC_NCE4_2 PG11
#define FMC_NE4 PG12
#define FMC_A24 PG13
#define FMC_A25 PG14
#define FMC_SDCKE1 PB5
#define FMC_SDNE1 PB6
#define FMC_NL PB7
#define FMC_D28  PI6
#define FMC_D29  PI7
#endif

#define SYS_CONSOLE_DEV		"usart0"
#define USER_CONSOLE_DEV	"usart0"


/* User Leds */
#define USER_LEDS	2
#define LED_PINS	(GPIO_PIN(PG,13)|(GPIO_PIN(PG,14)<<8))
#define LED_GREEN	1
#define LED_RED  	2

/* Usart*/
#define USE_USART	1
#define USART_TX_PIN	GPIO_PIN(PA,9)
#define USART_RX_PIN	GPIO_PIN(PA,10)

/* Usb */
#define USB_ID		GPIO_PIN(PB,12)
#define USB_VBUS	GPIO_PIN(PB,13)
#define USB_DM		GPIO_PIN(PB,14)
#define USB_DP		GPIO_PIN(PB,15)
#if 0
#define USB_PSO		GPIO_PIN(PC,4)
#define USB_QC		GPIO_PIN(PC,5)

/* Cec */
#if 1
#define CEC_PIN		GPIO_PIN(PC,3);
#else
#define CEC_PIN		GPIO_PIN(PB,7);
#endif

/* Sony A1 */
#define A1_PIN		GPIO_PIN(PB,4);
#endif
