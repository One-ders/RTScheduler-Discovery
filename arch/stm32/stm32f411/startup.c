
#include <stm32f411.h>
#include <devices.h>
#include <core_cm4.h>

#define VECT_TAB_OFFSET  0x00U
#define FLASH_BASE 0x08000000

#define __FPU_PRESENT 0
#define __FPU_USED 0
/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */

void led_flash(int oncnt, int offcnt) {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER&=~GPIO_MODER_MODER13_MASK;
	GPIOC->MODER = (1<<26);
	while(1) {
		volatile int i;
		for(i=0;i<oncnt;i++);
		GPIOC->BSRR = GPIO_BSRR_BS13;
		for(i=0;i<offcnt;i++);
		GPIOC->BSRR = GPIO_BSRR_BR13;
	}
}

void clk_init(void) {
	/* FPU settings ------------------------------------------------------------*/
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif


// Set flash wait states
	FLASH->ACR=(FLASH->ACR&~0xf)|2;
	PWR->CR|=PWR_CR_VOS_MASK;

	RCC->CR|=RCC_CR_HSEON;
	while(!(RCC->CR&RCC_CR_HSERDY));

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR|=PWR_CR_DBP;

	RCC->BDCR|=RCC_BDCR_BDRST;
	RCC->BDCR&=~RCC_BDCR_BDRST;
	RCC->BDCR|=RCC_BDCR_LSEON;
	while(!(RCC->BDCR&RCC_BDCR_LSERDY));
	RCC->BDCR=(RCC->BDCR&~RCC_BDCR_RTCSEL_MASK)|(1<<RCC_BDCR_RTCSEL_SHIFT);
	RCC->BDCR|=RCC_BDCR_RTCEN;

// PLLM = 0x19  el  25
// PLLN = 0x150 el 336
// PLLP = 4 for value 1
// 	VCO clk = 25x(336/25)=336
// 	PLL clk = 336/4 = 84 Mhz
// PLLQ = 7  ---> 48 Mhz for usb
	RCC->PLLCFGR=(RCC->PLLCFGR&0xffbf8000)|0x405419;
	RCC->PLLCFGR=(RCC->PLLCFGR&~0x30000)|0x10000; // pllp=01

	RCC->PLLCFGR=(RCC->PLLCFGR&0xf0bf8000)|0x07405419;

	RCC->CR|=RCC_CR_PLLON;
	while(!(RCC->CR&RCC_CR_PLLRDY));

	RCC->CFGR=RCC->CFGR&~RCC_CFGR_HPRE_MASK;
	RCC->CFGR=(RCC->CFGR&~RCC_CFGR_PPRE1_MASK)|(4<RCC_CFGR_PPRE1_SHIFT);
	RCC->CFGR=(RCC->CFGR&~RCC_CFGR_PPRE2_MASK);
	RCC->CFGR=(RCC->CFGR&~3)|2;
	while((RCC->CFGR&0xc)!=8);

        /* Configure flash prefetch, Icache, dcache and wait state */
        FLASH->ACR |= (FLASH_ACR_ICEN| FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);

	*((&RCC->CR)+35)&=~0x1000000;

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
        NVIC_SetPriority(USART2_IRQn,0xe);
}

