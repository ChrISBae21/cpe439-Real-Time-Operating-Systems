#include "main.h"
#define WDG_REFRESH 0x0000AAAA

void initButtonInterrupt(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; 	// enable GPIOC Clock
	GPIOC->MODER &= ~GPIO_MODER_MODE13;		// Clear PC13 to be input mode
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13;
	GPIOC->PUPDR |=  GPIO_PUPDR_PUPD13_1;	// Pull-up PC13

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// enable SYSCFG clock

	// set EXTI line for PC13
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->IMR1 |=   EXTI_IMR1_IM13;			// unmask interrupt requests from line 13
	EXTI->RTSR1 |=  EXTI_RTSR1_RT13;		// enable rising trigger for line 13
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT13;		// disable falling trigger for line 13
	NVIC->ISER[1] |= (0x1 << (EXTI15_10_IRQn & 0x1F));	// EXTI1 Interrupt enabled

	__enable_irq();
}

void EXTI15_10_IRQHandler(void) {
	// interrupt by PC13
	if(EXTI->PR1 & EXTI_PR1_PIF13) {
		IWDG->KR = WDG_REFRESH;			// refresh counter
		EXTI->PR1 |= EXTI_PR1_PIF13; 	// clear the bit by writing a 1
	}
}
