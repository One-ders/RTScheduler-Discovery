
#define I2C_CR1_PE	0x00000001	/* Peripheral Enable */
#define I2C_CR1_SMBUS	0x00000002	/* 1=SMBUS, 0=I2C    */
#define I2C_CR1_SMBTYPE 0x00000008      /* 1=SMBus_host, 0=SMBus_device */
#define I2C_CR1_ENARP   0x00000010      /* 1=ARP Enable */
#define I2C_CR1_ENPEC   0x00000020	/* 1=PEC calc enable */
#define I2C_CR1_ENGC    0x00000040	/* 1=General call enabled */
#define I2C_CR1_NOSTRETCH 0x00000080    /* 1=Clock strech disable, 0=enable */
#define I2C_CR1_START	0x00000100	/* 1=start generation, 0=no startg. */
#define I2C_CR1_STOP	0x00000200	/* 1=Stop generation, 0=no stopg. */
#define I2C_CR1_ACK	0x00000400	/* 1=Ack return, 0=No ack */
#define I2C_CR1_POS	0x00000800	/* 1=Ack next byte, 0=ack current */
#define I2C_CR1_PEC	0x00001000	/* 1=PEC transfer, 0=No PEC transfer */
#define I2C_CR1_ALERT	0x00002000	/* 1=SMBUS Alert, 0=No SMBA */
#define I2C_CR1_SWRST	0x00008000	/* 1=RESET */

#define I2C_CR2_FREQ_MASK 0x0000003f	/* FRQ field */
#define I2C_CR2_ITERREN	0x00000100	/* 1=Error irq enable */
#define I2C_CR2_ITEVTEN	0x00000200	/* 1=Event Irq enable */
#define I2C_CR2_ITBUFEN	0x00000400	/* 1=Buf Irq enable */
#define I2C_CR2_DMAEN	0x00000800	/* 1=DMA Enable */
#define I2C_CR2_LAST	0x00001000	/* 1=Next DMA is last */

#define I2C_OAR1_ADD0	0x00000001	/* bit 0 of 10 bit addr  */
#define I2C_OAR1_ADD_MASK 0x000000fe	/* addr, 7 bit, part when 10 bit */
#define I2C_OAR1_ADD_SHIFT 1
#define I2C_OAR1_ADD89  0x00000300	/* top addr bits when 10 bit */
#define I2C_OAR1_7BADD	0x000000fe
#define I2C_OAR1_10BADD 0x000003ff
#define I2C_OAR1_AMODE	0x00008000	/* 1=10 bit address */

#define I2C_OAR2_ENDUAL	0x00000001	/* 1=Dual address mode */
#define I2C_OAR2_ADD2_MASK 0x000000fe	/* 7 bit mode sec. addr */
#define I2C_OAR2_ADD2_SHIFT 1

#define I2C_DR_MASK	0x000000ff	/* data */

#define I2C_SR1_SB	0x00000001	/* 1=Start condition generated */
#define I2C_SR1_ADDR	0x00000002	/* 1=Received addr match */
#define I2C_SR1_BTF	0x00000004	/* 1=Byte transfer finished */
#define I2C_SR1_ADD10	0x00000008	/* 1=Master sent first address */
#define I2C_SR1_STOPF	0x00000010	/* 1=Stop condition detect */
#define I2C_SR1_RXNE	0x00000040	/* 1=Data reg. not empty */
#define I2C_SR1_TXE	0x00000080	/* 1=Data reg. Empty */
#define I2C_SR1_BERR	0x00000100	/* 1=Misplaced start or stop */
#define I2C_SR1_ARLO	0x00000200	/* 1=Arbitration Lost */
#define I2C_SR1_AF	0x00000400	/* 1=Ack failure */
#define I2C_SR1_OVR	0x00000800	/* 1=Over or underrun */
#define I2C_SR1_PECERR	0x00001000	/* 1=PEC error */
#define I2C_SR1_TOUT	0x00004000	/* 1=Timeout */
#define I2C_SR1_SMBALRT 0x00008000	/* 1=SMB Alert */

#define I2C_SR2_MSL	0x00000001	/* 1=Master mode */
#define I2C_SR2_BUSY	0x00000002	/* 1=BUS com ongoing */
#define I2C_SR2_TRA	0x00000004	/* 1=data tx'ed, 0=data rx'ed */
#define I2C_SR2_GENCALL 0x00000010	/* 1=General Call address recv. */
#define I2C_SR2_SMBDEFAULT 0x00000020   /* 1=SMBus Dev default add. recv. */
#define I2C_SR2_SMBHOST 0x00000040	/* 1=SMBus Host address recieved */
#define I2C_SR2_DUALF   0x00000080	/* 1=recaddr=OAR2, 0=recaddr=OAR1 */
#define I2C_SR2_PEC_MASK 0x0000ff00	/* Packet Error check reg */
#define I2C_SR2_PEC_SHIFT 8

#define I2C_CCR_CCR_MASK 0x00000fff	/* Clock Control register */
#define I2C_CCR_DUTY	0x00004000	/* 1=tl/th=16/9, 0=tl/th=2 */
#define I2C_CCR_FS	0x00008000	/* Master mode 1=Fm, 0=Sm */

#define I2C_TRISE_TRISE_MASK 0x0000003f /* Max rise time */

#define I2C_FLTR_DNF_MASK 0x0000000f	/* Digital noise filter mask */
#define I2C_FLTR_ANOF	0x00000010	/* Analog noise filter 1=disable */

struct I2C {
	volatile unsigned int CR1;	/* 0  */
	volatile unsigned int CR2;	/* 4  */
	volatile unsigned int OAR1;	/* 8  */
	volatile unsigned int OAR2;	/* 12 */
	volatile unsigned int DR;	/* 16 */
	volatile unsigned int SR1;	/* 20 */
	volatile unsigned int SR2;	/* 24 */
	volatile unsigned int CCR;      /* 28 */
	volatile unsigned int TRISE;    /* 32 */
	volatile unsigned int FLTR;     /* 36 */
};
