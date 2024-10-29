

#include "main.h"
#include "timer.h"

void PWM_GPIO_init() {
	// Configure GPIO and AFR
	RCC->AHB2ENR |= (0x1 << 0);		// enable GPIOC Clock
	RCC->APB1ENR1 |= (1 << 0);		// turn on clock to TIM2

	GPIOA->MODER &= ~((0x3 << 2));		// clear PA0 and PA1
	GPIOA->MODER |=  ((0x2 << 2));		// set PA0 and PA1 to AF

	GPIOA->AFR[0] &= ~((0xF << 4));	// clear AF
	GPIOA->AFR[0] |=  ((0x1 << 4));	// set PA0 and PA1 to AF1
}


void PWM_init(uint32_t clock, uint32_t freq) {
	PWM_GPIO_init();
	TIM2->CCER 	&= ~(0x1 << 5);					// CC2P (Polarity) Active High

	TIM2->CR1 	|= (0x1 << 7);					// set the Auto-Reload Preload Register

	// CCMR CONFIGURATION OR CHANNEL 2
	TIM2->CCMR1 |= (0x1 << 11);					// Set the Preload Register for CCR2
	TIM2->CCMR1 &= ~((0x1 << 24)				// clear bit 24 for OC2M Channel 2
					|(0x3 << 12)				// clear bit 14, 13, and 12 for OC2M Channel 2
	);

	TIM2->CCMR1 |=  (0x6 << 12); 				// set bit 14, 12, and 12 for OC2M Channel 2 Mode 2
	TIM2->CCMR1 &= ~(0x3 << 8);					// clear to set Channel 2 as output

	TIM2->CR2 |= (0x5 << 4);

	TIM2->ARR  = clock / freq - 1;
	TIM2->CCR2 = (clock / (2*freq)) - 1;

	TIM2->EGR	|= (0x1 << 0);				// set the UG bit
	TIM2->CCER	|= (0x1 << 4);				// enable Capture Mode for Channel 2
	TIM2->CR1	|= (0x1 << 0);				// enable counter
}
