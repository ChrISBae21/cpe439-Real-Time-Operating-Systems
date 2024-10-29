// CCMR CONFIGURATION ON CHANNEL 2
#include "main.h"
#include "timer.h"

void PWM_GPIO_init() {
	// Configure GPIO and AFR
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOCEN;		// enable GPIOC Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;		// turn on clock to TIM2

	GPIOA->MODER &= ~GPIO_MODER_MODE1;			// clear PA0 and PA1
	GPIOA->MODER |=  GPIO_MODER_MODE1_1;		// set PA0 and PA1 to AF

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL1;			// clear AF
	GPIOA->AFR[0] |=  GPIO_AFRL_AFSEL1_0;		// set PA0 and PA1 to AF1
}


void PWM_init(uint32_t clock, uint32_t freq) {
	PWM_GPIO_init();
	TIM2->CCER 	&= ~TIM_CCER_CC2P;		// CC2P (Polarity) Active High

	TIM2->CR1 	|= TIM_CR1_ARPE;		// enable the Auto-Reload Preload Register

	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;		// Set the Preload Register for CCR2

	TIM2->CCMR1 |=  ((TIM_CCMR1_OC2M_1)	// set bit 14 for OC2M Channel 2 Mode 2
					|(TIM_CCMR1_OC2M_2)	// set bit 13 for OC2M Channel 2 Mode 2
	);

	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;		// clear to set Channel 2 as output

	TIM2->ARR  = clock / freq - 1;
	TIM2->CCR2 = (clock / (2*freq)) - 1;

	TIM2->EGR	|= TIM_EGR_UG;			// set the UG bit
	TIM2->CCER	|= TIM_CCER_CC2E;		// enable Capture Mode for Channel 2
	TIM2->CR1	|= TIM_CR1_CEN;			// enable counter
}
