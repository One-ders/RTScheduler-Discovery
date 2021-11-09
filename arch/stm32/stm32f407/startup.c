
#include <sys.h>
#include <string.h>
#include <stm32f407.h>
#include <devices.h>

#define PLL_M	8

#ifdef MB1075B
#define PLL_N	360
#else
#define PLL_N	336
#endif
#define PLL_P	2
#define PLL_q	7

#define HSE_STARTUP_TIMEOUT	0x0500

void clk_init(void) {
	volatile unsigned int startup_cnt=0;
	volatile unsigned int HSE_status=0;
	/* Reset RCC clocks */
	RCC->CR |= RCC_CR_HSION;

	RCC->CFGR = 0;

	RCC->CR &=
		~(RCC_CR_HSEON |
		 RCC_CR_CSSON |
		 RCC_CR_PLLON);

	// write reset value 0x24003010
	RCC->PLLCFGR= 0x20000000 |
			(RCC_PLLCFGR_PLLQ2 |
			 (0xc0 << RCC_PLLCFGR_PLLN_SHIFT) |
			RCC_PLLCFGR_PLLM4);

	RCC->CR &= ~RCC_CR_HSEBYP;

	RCC->CIR = 0;

	/* Config clocks */
	RCC->CR |= RCC_CR_HSEON;
	do {
		HSE_status=RCC->CR&RCC_CR_HSERDY;
		startup_cnt++;
	} while ((!HSE_status) && (startup_cnt!=HSE_STARTUP_TIMEOUT));

	if (RCC->CR&RCC_CR_HSERDY) {
		HSE_status=1;
	} else {
		HSE_status=0;
		/* Failed to start external clock */
		return;
		ASSERT(0);
	}

	/* Enable high perf mode, system clock to 168/180 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
#ifdef MB1075B
	PWR->CR |= (3<<PWR_CR_VOS_SHIFT);
#else
	PWR->CR |= (1<<PWR_CR_VOS_SHIFT);
#endif

	/* HCLK = SYSCLK/1 */
//	RCC->CFGR |= (1<<RCC_CFGR_HPRE_SHIFT);

	/* PCLK2 = HCLK/2 */
	RCC->CFGR |= (4<<RCC_CFGR_PPRE2_SHIFT);

	/* PCLK1 = HCLK/4 */
	RCC->CFGR |= (5<<RCC_CFGR_PPRE1_SHIFT);

	/* Configure the main PLL */
	RCC->PLLCFGR = RCC_PLLCFGR_PLLM3 |
			(PLL_N<<RCC_PLLCFGR_PLLN_SHIFT) |
			RCC_PLLCFGR_PLLSRC |
			RCC_PLLCFGR_PLLQ0 |
			RCC_PLLCFGR_PLLQ1 |
			RCC_PLLCFGR_PLLQ2;

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

#ifdef MB1075B
	PWR->CR |= PWR_CR_ODEN;
	while(!(PWR->CSR&PWR_CSR_ODRDY));
	PWR->CR |= PWR_CR_ODSW;
	while(!(PWR->CSR&PWR_CSR_ODSWRDY));
#endif

	/* Wait for main PLL ready */
	while(!(RCC->CR&RCC_CR_PLLRDY));

	/* Configure flash prefetch, Icache, dcache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN| FLASH_ACR_DCEN | (FLASH_ACR_LATENCY_MASK&0x5);

	/* Select main PLL as system clock source */
	RCC->CFGR &= ~RCC_CFGR_SW0;
	RCC->CFGR |= RCC_CFGR_SW1;

	while((RCC->CFGR&RCC_CFGR_SWS_MASK)!=(0x2<<RCC_CFGR_SWS_SHIFT));

}

void init_irq(void) {
//      NVIC_SetPriority(SVCall_IRQn,0xe);
	NVIC_SetPriority(SVCall_IRQn,0xf);  /* try to share level with pendsv */
	NVIC_SetPriority(SysTick_IRQn,0x1);  /* preemptive tics... */
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn,0x0);
	NVIC_SetPriority(EXTI0_IRQn,0x2);
	NVIC_SetPriority(EXTI1_IRQn,0x2);
	NVIC_SetPriority(EXTI2_IRQn,0x2);
	NVIC_SetPriority(EXTI3_IRQn,0x2);
	NVIC_SetPriority(EXTI4_IRQn,0x2);
	NVIC_SetPriority(EXTI9_5_IRQn,0x2);
	NVIC_SetPriority(EXTI15_10_IRQn,0x2);
	NVIC_SetPriority(USART3_IRQn,0xe);
}
