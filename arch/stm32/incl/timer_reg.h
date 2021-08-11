
/* TIMER control register 1 */
#define TIM_CR1_CEN		0x00000001
#define TIM_CR1_UDIS		0x00000002
#define TIM_CR1_URS		0x00000004
#define TIM_CR1_OPM		0x00000008
#define TIM_CR1_DIR		0x00000010
#define TIM_CR1_CMS_MASK	0x00000060
#define TIM_CR1_CMS_SHIFT	5
#define TIM_CR1_ARPE		0x00000080
#define TIM_CR1_CKD_MASK	0x00000300
#define TIM_CR1_CKD_SHIFT	8

/* TIMER control register 2 */
#define TIM_CR2_CCPC		0x00000001
#define TIM_CR2_CCUS		0x00000004
#define TIM_CR2_CCDS		0x00000008
#define TIM_CR2_MMS_MASK	0x00000070
#define TIM_CR2_MMS_SHIFT	4
#define TIM_CR2_TI1S		0x00000080
#define TIM_CR2_OIS1		0x00000100
#define TIM_CR2_OIS1N		0x00000200
#define TIM_CR2_OIS2		0x00000400
#define TIM_CR2_OIS2N		0x00000800
#define TIM_CR2_OIS3		0x00001000
#define TIM_CR2_OIS3N		0x00002000
#define TIM_CR2_OIS4		0x00004000

/* TIMER slave mode control register */
#define TIM_SMCR_SMS_MASK	0x00000007
#define TIM_SMCR_SMS_SHIFT	0
#define TIM_SMCR_TS_MASK	0x00000070
#define TIM_SMCR_TS_SHIFT	4
#define TIM_SMCR_MSM		0x00000080
#define TIM_SMCR_ETF_MASK	0x00000f00
#define TIM_SMCR_ETF_SHIFT	8
#define TIM_SMCR_ETPS_MASK	0x00003000
#define TIM_SMCR_ETPS_SHIFT	12
#define TIM_SMCR_ECE		0x00004000
#define TIM_SMCR_ETP		0x00008000

/* TIMER DMA/interrupt enable register */
#define TIM_DIER_UIE		0x00000001
#define TIM_DIER_CC1IE		0x00000002
#define TIM_DIER_CC2IE		0x00000004
#define TIM_DIER_CC3IE		0x00000008
#define TIM_DIER_CC4IE		0x00000010
#define TIM_DIER_COMIE		0x00000020
#define TIM_DIER_TIE		0x00000040
#define TIM_DIER_BIE		0x00000080
#define TIM_DIER_UDE		0x00000100
#define TIM_DIER_CC1DE		0x00000200
#define TIM_DIER_CC2DE		0x00000400
#define TIM_DIER_CC3DE		0x00000800
#define TIM_DIER_CC4DE		0x00001000
#define TIM_DIER_COMDE		0x00002000
#define TIM_DIER_TDE		0x00004000

/* Timer status register */
#define TIM_SR_UIF		0x00000001
#define TIM_SR_CC1IF		0x00000002
#define TIM_SR_CC2IF		0x00000004
#define TIM_SR_CC3IF		0x00000008
#define TIM_SR_CC4IF		0x00000010
#define TIM_SR_COMIF		0x00000020
#define TIM_SR_TIF		0x00000040
#define TIM_SR_BIF		0x00000080
#define TIM_SR_CC1OF		0x00000200
#define TIM_SR_CC2OF		0x00000400
#define TIM_SR_CC3OF            0x00000800
#define TIM_SR_CC4OF		0x00001000

/* Timer event generation register */
#define TIM_EGR_UG		0x00000001
#define TIM_EGR_CC1G		0x00000002
#define TIM_EGR_CC2G		0x00000004
#define TIM_EGR_CC3G		0x00000008
#define TIM_EGR_CC4G		0x00000010
#define TIM_EGR_COMG		0x00000020
#define TIM_EGR_TG		0x00000040
#define TIM_EGR_BG		0x00000080

/* Timer capture/compare mode register 1 */
/* as output */
#define TIM_CCMR1_CC1S_MASK	0x00000003
#define TIM_CCMR1_CC1S_SHIFT	0
#define TIM_CCMR1_OC1FE		0x00000004
#define TIM_CCMR1_OC1PE		0x00000008
#define TIM_CCMR1_OC1M_MASK	0x00000070
#define TIM_CCMR1_OC1M_SHIFT	4
#define TIM_CCMR1_OC1CE		0x00000080
#define TIM_CCMR1_CC2S_MASK	0x00000300
#define TIM_CCMR1_CC2S_SHIFT	8
#define TIM_CCMR1_OC2FE		0x00000400
#define TIM_CCMR1_OC2PE		0x00000800
#define TIM_CCMR1_OC2M_MASK	0x00007000
#define TIM_CCMR1_OC2M_SHIFT	12
#define TIM_CCMR1_OC2CE		0x00008000

/* as input */
#define TIM_CCMR1_IC1PSC_MASK	0x0000000c
#define TIM_CCMR1_IC1PSC_SHIFT	2
#define TIM_CCMR1_IC1F_MASK	0x000000f0
#define TIM_CCMR1_IC1F_SHIFT	4
#define TIM_CCMR1_IC2PSC_MASK	0x00000c00
#define TIM_CCMR1_IC2PSC_SHIFT  10
#define TIM_CCMR1_IC2F_MASK	0x0000f000
#define TIM_CCMR1_IC2F_SHIFT	12


/* Timer capture/compare mode register 2 */
/* as output */
#define TIM_CCMR2_CC3S_MASK	0x00000003
#define TIM_CCMR2_CC3S_SHIFT	0
#define TIM_CCMR2_OC3FE		0x00000004
#define TIM_CCMR2_OC3PE		0x00000008
#define TIM_CCMR2_OC3M_MASK	0x00000070
#define TIM_CCMR2_OC3M_SHIFT	4
#define TIM_CCMR2_OC3CE		0x00000080
#define TIM_CCMR2_CC4S_MASK	0x00000300
#define TIM_CCMR2_CC4S_SHIFT	8
#define TIM_CCMR2_OC4FE		0x00000400
#define TIM_CCMR2_OC4PE		0x00000800
#define TIM_CCMR2_OC4M_MASK	0x00007000
#define TIM_CCMR2_OC4M_SHIFT	12
#define TIM_CCMR2_OC4CE		0x00008000

/* as input */
#define TIM_CCMR2_IC3PSC_MASK	0x0000000c
#define TIM_CCMR2_IC3PSC_SHIFT	2
#define TIM_CCMR2_IC3F_MASK	0x000000f0
#define TIM_CCMR2_IC3F_SHIFT	4
#define TIM_CCMR2_IC4PSC_MASK	0x00000c00
#define TIM_CCMR2_IC4PSC_SHIFT  10
#define TIM_CCMR2_IC4F_MASK	0x0000f000
#define TIM_CCMR2_IC4F_SHIFT	12


/* Timer capture/compare enable register */
#define TIM_CCER_CC1E		0x00000001
#define TIM_CCER_CC1P		0x00000002
#define TIM_CCER_CC1NE		0x00000004
#define TIM_CCER_CC1NP		0x00000008
#define TIM_CCER_CC2E		0x00000010
#define TIM_CCER_CC2P		0x00000020
#define TIM_CCER_CC2NE		0x00000040
#define TIM_CCER_CC2NP		0x00000080
#define TIM_CCER_CC3E		0x00000100
#define TIM_CCER_CC3P		0x00000200
#define TIM_CCER_CC3NE		0x00000400
#define TIM_CCER_CC3NP		0x00000800
#define TIM_CCER_CC4E		0x00001000
#define TIM_CCER_CC4P		0x00002000

/* Timer break and dead-time register */
#define TIM_BDTR_DTG_MASK	0x000000ff
#define TIM_BDTR_DTG_SHIFT	0
#define TIM_BDTR_LOCK_MASK	0x00000300
#define TIM_BDTR_LOCK_SHIFT	8
#define TIM_BDTR_OSSI		0x00000400
#define TIM_BDTR_OSSR		0x00000800
#define TIM_BDTR_BKE		0x00001000
#define TIM_BDTR_BKP		0x00002000
#define TIM_BDTR_AOE		0x00004000
#define TIM_BDTR_MOE		0x00008000

/* Timer DMA control register */
#define TIM_DCR_DBA_MASK	0x0000001f
#define TIM_DCR_DBA_SHIFT	0
#define TIM_DCR_DBL_MASK	0x00001f00
#define TIM_DCR_DBL_SHIFT	8





struct TIMER {
	volatile unsigned int CR1;	// 0x00
	volatile unsigned int CR2;	// 0x04
	volatile unsigned int SMCR;	// 0x08
	volatile unsigned int DIER;	// 0x0c
	volatile unsigned int SR;	// 0x10
	volatile unsigned int EGR;	// 0x14
	volatile unsigned int CCMR1;	// 0x18
	volatile unsigned int CCMR2;	// 0x1c
	volatile unsigned int CCER;	// 0x20
	volatile unsigned int CNT;	// 0x24
	volatile unsigned int PSC;	// 0x28
	volatile unsigned int ARR;	// 0x2c
	volatile unsigned int RCR;	// 0x30
	volatile unsigned int CCR1;	// 0x34
	volatile unsigned int CCR2;	// 0x38
	volatile unsigned int CCR3;	// 0x3c
	volatile unsigned int CCR4;	// 0x40
	volatile unsigned int BDTR;	// 0x44
	volatile unsigned int DCR;	// 0x48
	volatile unsigned int DMAR;	// 0x4c
};
