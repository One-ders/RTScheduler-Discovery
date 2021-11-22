
#include <sys.h>
#include <string.h>
#include <stm32f407.h>
#include <devices.h>

#include <gpio_drv.h>

#ifdef MB1075B

extern struct driver_ops gpio_drv_ops;

static int fmc_sram_open(void *instance, DRV_CBH cb_handler, void *dum) {
	return -1;
}

static int fmc_sram_control(struct device_handle *dh, int cmd, void *arg1, int arg2) {
	return -1;
}

static int fmc_sram_close(struct device_handle *dh) {
	return 0;
}

static int fmc_sram_start(void *inst) {
	return 0;
}

// Mem at 90 Mhz = (180/2)
// Timings
// SDTR.TMRD: 1-16,  2 cc
#define LoadToActiveDelay	2
// SDTR.TXSR: min 70ns (7x11.11ns) 1-16 (v-1)<<4
#define ExitSelfRefreshDelay	7
// SDTR.TRAS: min 42, max 120000 ns (4x11.11ns), 1-16 -> (v-1)<<8
#define SelfRefreshTime		4
// SDTR.TRC: min 70 (7x11.11ns), 1-16 -> (v-1)<<12
#define RowCycleDelay		7
// SDTR.TWR: min 1+7ns	(1+1x11.11ns), 1-16 -> (v-1)<<16
#define WriteRecoveryTime	2
// SDTR.TRP:  20ns (2x11.11ns), 1-16 -> (v-1)<<20
#define RPDelay			2
// SDTR.TRCD: 20ns (2x11.11ns), 1-16 -> (v-1)<<24
#define RCDelay			2

// Controll config
// SDRAM Bank 0 or 1
#define Bank			0
// SDRC1,2 Col addressing: [7:0], value 8-11 is col-8
#define ColumnBits		8
// SDRC1,2 Row addressing: [11:0], value is 11-13 -> (v-11)<<2
#define RowBits			12
// SDRC1,2 MWID 8,16,32 -> (v>>4)<<4)
#define MemDataWidth		16
// SDRC1,2 NB 2 or 4 -> (v>>2)<<6
#define InternalBank		4
// SDRC1,2 0,1,2,3 cycles,  v<<7
#define CASLatency		3
// SDRC1,2 WP  0 or 1 v<<9
#define WriteProtect		0
// SDRC1 00=disable, 2=2xhclk or 3=3xhclk
#define SDClockPeriod		2
// SDRC1 readburst disable 0 or 1  -> v<<12
#define ReadBurst		0
// SDRC1 0,1 or 2 -> v<<13
#define ReadPipeDelay		1

#define PORT(a) ((a>>4)&0xf)
#define PIN(a) (a&0xf)

static int fmc_sram_init(void *inst) {
	struct device_handle *gpiob_dh;
	struct device_handle *gpioc_dh;
	struct device_handle *gpiod_dh;
	struct device_handle *gpioe_dh;
	struct device_handle *gpiof_dh;
	struct device_handle *gpiog_dh;
//	struct driver *gpio_drv;
	struct pin_spec ps[GPIO_PG+1];
	int rc;
	unsigned int tmpreg;
	unsigned int tout;
	unsigned int flags;

//	gpio_drv=driver_lookup(GPIO_DRV);
//	if (!gpio_drv) return 0;

	gpio_drv_ops.init(0);

	gpiob_dh=gpio_drv_ops.open(0,0,0);
	gpioc_dh=gpio_drv_ops.open(0,0,0);
	gpiod_dh=gpio_drv_ops.open(0,0,0);
	gpioe_dh=gpio_drv_ops.open(0,0,0);
	gpiof_dh=gpio_drv_ops.open(0,0,0);
	gpiog_dh=gpio_drv_ops.open(0,0,0);

	flags=GPIO_DIR(0,GPIO_ALTFN_PIN);
//	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
	flags=GPIO_SPEED(flags,GPIO_SPEED_FAST);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_ALTFN(flags,0xc);

	ps[GPIO_PB].port=GPIO_PB;
	ps[GPIO_PB].flags=flags;
	ps[GPIO_PB].pins=0;

	ps[GPIO_PC].port=GPIO_PC;
	ps[GPIO_PC].flags=flags;
	ps[GPIO_PC].pins=0;

	ps[GPIO_PD].port=GPIO_PD;
	ps[GPIO_PD].flags=flags;
	ps[GPIO_PD].pins=0;

	ps[GPIO_PE].port=GPIO_PE;
	ps[GPIO_PE].flags=flags;
	ps[GPIO_PE].pins=0;

	ps[GPIO_PF].port=GPIO_PF;
	ps[GPIO_PF].flags=flags;
	ps[GPIO_PF].pins=0;

	ps[GPIO_PG].port=GPIO_PG;
	ps[GPIO_PG].flags=flags;
	ps[GPIO_PG].pins=0;


// Port F
	ps[PORT(FMC_A0)].pins|=(1<<PIN(FMC_A0));
	ps[PORT(FMC_A1)].pins|=(1<<PIN(FMC_A1));
	ps[PORT(FMC_A2)].pins|=(1<<PIN(FMC_A2));
	ps[PORT(FMC_A3)].pins|=(1<<PIN(FMC_A3));
	ps[PORT(FMC_A4)].pins|=(1<<PIN(FMC_A4));
	ps[PORT(FMC_A5)].pins|=(1<<PIN(FMC_A5));
	ps[PORT(FMC_A6)].pins|=(1<<PIN(FMC_A6));
	ps[PORT(FMC_A7)].pins|=(1<<PIN(FMC_A7));
	ps[PORT(FMC_A8)].pins|=(1<<PIN(FMC_A8));
	ps[PORT(FMC_A9)].pins|=(1<<PIN(FMC_A9));
//---
//	Port G
	ps[PORT(FMC_A10)].pins|=(1<<PIN(FMC_A10));
	ps[PORT(FMC_A11)].pins|=(1<<PIN(FMC_A11));
	ps[PORT(FMC_BA0)].pins|=(1<<PIN(FMC_BA0));
	ps[PORT(FMC_BA1)].pins|=(1<<PIN(FMC_BA1));
	/*********************/
//	Port D
	ps[PORT(FMC_D0)].pins|=(1<<PIN(FMC_D0));
	ps[PORT(FMC_D1)].pins|=(1<<PIN(FMC_D1));
	ps[PORT(FMC_D2)].pins|=(1<<PIN(FMC_D2));
	ps[PORT(FMC_D3)].pins|=(1<<PIN(FMC_D3));
//	Port E
	ps[PORT(FMC_D4)].pins|=(1<<PIN(FMC_D4));
	ps[PORT(FMC_D5)].pins|=(1<<PIN(FMC_D5));
	ps[PORT(FMC_D6)].pins|=(1<<PIN(FMC_D6));
	ps[PORT(FMC_D7)].pins|=(1<<PIN(FMC_D7));
	ps[PORT(FMC_D8)].pins|=(1<<PIN(FMC_D8));
	ps[PORT(FMC_D9)].pins|=(1<<PIN(FMC_D9));
	ps[PORT(FMC_D10)].pins|=(1<<PIN(FMC_D10));
	ps[PORT(FMC_D11)].pins|=(1<<PIN(FMC_D11));
	ps[PORT(FMC_D12)].pins|=(1<<PIN(FMC_D12));
//	Port D
	ps[PORT(FMC_D13)].pins|=(1<<PIN(FMC_D13));
	ps[PORT(FMC_D14)].pins|=(1<<PIN(FMC_D14));
	ps[PORT(FMC_D15)].pins|=(1<<PIN(FMC_D15));
	/**************************/
// 	Port E
	ps[PORT(FMC_NBL0)].pins|=(1<<PIN(FMC_NBL0));
	ps[PORT(FMC_NBL1)].pins|=(1<<PIN(FMC_NBL1));
//	Port G
	ps[PORT(FMC_SDCLK)].pins|=(1<<PIN(FMC_SDCLK));
//	Port C
	ps[PORT(FMC_SDNWE)].pins|=(1<<PIN(FMC_SDNWE));
//	Port F
	ps[PORT(FMC_SDNRAS)].pins|=(1<<PIN(FMC_SDNRAS));
//	Port G
	ps[PORT(FMC_SDNCAS)].pins|=(1<<PIN(FMC_SDNCAS));
//	Port B
	ps[PORT(FMC_SDNE1)].pins|=(1<<PIN(FMC_SDNE1));
	ps[PORT(FMC_SDCKE1)].pins|=(1<<PIN(FMC_SDCKE1));

	rc=gpio_drv_ops.control(gpiob_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PB],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port B\n");
		return -1;
	}

	rc=gpio_drv_ops.control(gpioc_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PC],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port C\n");
		return -1;
	}

	rc=gpio_drv_ops.control(gpiod_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PD],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port D\n");
		return -1;
	}

	rc=gpio_drv_ops.control(gpioe_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PE],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port E\n");
		return -1;
	}

	rc=gpio_drv_ops.control(gpiof_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PF],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port F\n");
		return -1;
	}

	rc=gpio_drv_ops.control(gpiog_dh,GPIO_BUS_ASSIGN_PINS,&ps[GPIO_PG],sizeof(struct pin_spec));
	if (rc<0) {
		sys_printf("extmem; failed to assign port G\n");
		return -1;
	}

	/* */
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
	sys_udelay(100);

	/* Config and enable SDRAM bank1 */
#if 0
	FMC->SDCR_1=(1<<FMC_SDCR_RPIPE_SHIFT)|(2<<FMC_SDCR_SDCLK_SHIFT)|
			(3<<FMC_SDCR_CAS_SHIFT)|FMC_SDCR_NB|
			(1<<FMC_SDCR_MWID_SHIFT);
#endif
	FMC->SDCR_1=(ReadPipeDelay<<FMC_SDCR_RPIPE_SHIFT)|
			(SDClockPeriod<<FMC_SDCR_SDCLK_SHIFT)|
			(CASLatency<<FMC_SDCR_CAS_SHIFT)|
			((InternalBank>>2)<<FMC_SDCR_NB_SHIFT)|
			((MemDataWidth>>4)<<FMC_SDCR_MWID_SHIFT)|
			((RowBits-11)<<FMC_SDCR_NR_SHIFT)|
			(ColumnBits-8);

	FMC->SDCR_2=(ReadPipeDelay<<FMC_SDCR_RPIPE_SHIFT)|
			(SDClockPeriod<<FMC_SDCR_SDCLK_SHIFT)|
			(CASLatency<<FMC_SDCR_CAS_SHIFT)|
			((InternalBank>>2)<<FMC_SDCR_NB_SHIFT)|
			((MemDataWidth>>4)<<FMC_SDCR_MWID_SHIFT)|
			((RowBits-11)<<FMC_SDCR_NR_SHIFT)|
			(ColumnBits-8);


#if 0
	FMC->SDTR1=(1<<FMC_SDTR_TRCD_SHIFT)|(1<<FMC_SDTR_TRP_SHIFT)|
		(1<<FMC_SDTR_TWR_SHIFT)|(5<<FMC_SDTR_TRC_SHIFT)|
		(3<<FMC_SDTR_TRAS_SHIFT)|(5<<FMC_SDTR_TXSR_SHIFT)|
		(1<<FMC_SDTR_TMRD_SHIFT);
#endif
	FMC->SDTR1=((RCDelay-1)<<FMC_SDTR_TRCD_SHIFT)|
		((RPDelay-1)<<FMC_SDTR_TRP_SHIFT)|
		((WriteRecoveryTime-1)<<FMC_SDTR_TWR_SHIFT)|
		((RowCycleDelay-1)<<FMC_SDTR_TRC_SHIFT)|
		((SelfRefreshTime-1)<<FMC_SDTR_TRAS_SHIFT)|
		((ExitSelfRefreshDelay-1)<<FMC_SDTR_TXSR_SHIFT)|
		((LoadToActiveDelay-1)<<FMC_SDTR_TMRD_SHIFT);

	FMC->SDTR2=((RCDelay-1)<<FMC_SDTR_TRCD_SHIFT)|
		((RPDelay-1)<<FMC_SDTR_TRP_SHIFT)|
		((WriteRecoveryTime-1)<<FMC_SDTR_TWR_SHIFT)|
		((RowCycleDelay-1)<<FMC_SDTR_TRC_SHIFT)|
		((SelfRefreshTime-1)<<FMC_SDTR_TRAS_SHIFT)|
		((ExitSelfRefreshDelay-1)<<FMC_SDTR_TXSR_SHIFT)|
		((LoadToActiveDelay-1)<<FMC_SDTR_TMRD_SHIFT);


	FMC->SDCMR=FMC_SDCMR_CTB2|(1<<FMC_SDCMR_MODE_SHIFT); // Clock conf ena

	tout=0xffff;
	tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	while(tmpreg&&(tout--)) {
		tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	}

	for(tout=0;tout<1000;tout++);

	FMC->SDCMR=FMC_SDCMR_CTB2|(2<<FMC_SDCMR_MODE_SHIFT); // PALL
	tout=0xffff;
	tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	while(tmpreg&&(tout--)) {
		tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	}

	FMC->SDCMR=(3<<FMC_SDCMR_NRFS_SHIFT)|FMC_SDCMR_CTB2|
			(3<<FMC_SDCMR_MODE_SHIFT); // Auto refresh
	tout=0xffff;
	tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	while(tmpreg&&(tout--)) {
		tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	}

	FMC->SDCMR=(3<<FMC_SDCMR_NRFS_SHIFT)|FMC_SDCMR_CTB2|
			(3<<FMC_SDCMR_MODE_SHIFT); // Auto refresh
	tout=0xffff;
	tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	while(tmpreg&&(tout--)) {
		tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	}


	FMC->SDCMR=(0x231<<FMC_SDCMR_MRD_SHIFT)|FMC_SDCMR_CTB2|
			(4<<FMC_SDCMR_MODE_SHIFT); // load mode reg
	tout=0xffff;
	tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	while(tmpreg&&(tout--)) {
		tmpreg=FMC->SDSR&FMC_SDSR_BUSY;
	}

//	FMC->SDRTR|=(0x27c<<FMC_SDRTR_COUNT_SHIFT);
	FMC->SDRTR=(0x56a<<FMC_SDRTR_COUNT_SHIFT);

	FMC->SDCR_2&=~FMC_SDCR_WP;

	return 0;
}

static struct driver_ops fmc_sram_ops = {
	fmc_sram_open,
	fmc_sram_close,
	fmc_sram_control,
	fmc_sram_init,
	fmc_sram_start,
};

static struct driver fmc_sram_drv = {
	"fmc_sram",
	0,
	&fmc_sram_ops,
};

void init_fmc_sram(void) {
	driver_publish(&fmc_sram_drv);
}

INIT_FUNC(init_fmc_sram);

#endif
