#include "main.h"
#include "adc.h"

void ADC_init() {

	// CLOCK INITIALIZATION
	RCC->AHB2ENR  |= ( (1 << 0) );		// enable GPIOA clock
	RCC->AHB2ENR  |= ( (1 << 13) );		// enable ADC clock
	RCC->CCIPR |=  (0x3 << 28);			//ADC Peripheral Clock set to SYSCLK

	// GPIO INITIALIZATION
	// DATASHEET SAYS PA0 IS ADC12_IN5, CHANNEL 5
	GPIOA->MODER &= ~(0x3 << 0); 			// clear PA0
	GPIOA->MODER |=  (0x3 << 0);			// set PA0 to Analog Input
	GPIOA->AFR[0] &= ~(0xF);				// Clear PA0
	GPIOA->AFR[0] |= 0x7;					// set PA0 as ADC input pin
	GPIOA->ASCR |= (0x1 << 0);				// Analog Switch Control Register

	ADC123_COMMON->CCR &= ~(0xF << 18);		// clear ADC prescaler
//	ADC123_COMMON->CCR |=  (0x3 << 0);		// set ADC prescaler divided by 6
	ADC123_COMMON->CCR &= ~(0x3 << 16);		// clear (this is also asynchronous mode; give ADC own clock from PLL)
	ADC123_COMMON->CCR |= (0x1 << 16);		// set clock to SYSCLK
	ADC123_COMMON->CCR |= (0x1 << 13);		// DMA circular Mode
	ADC1->CFGR |= (0x1 << 0);				// Enable ADC DMA
	ADC1->CFGR |= (0x1 << 1);				// Enable ADC DMA Circular Mode

	ADC1->CR &= ~(0x1 << 29);				// get out of deep-power-down
	ADC1->CR |=  (0x1 << 28);				// turn on voltage regulator

	while( !(ADC1->CR & (0x1 << 28)) );		// wait for voltage regulator to turn on

	ADC1->CR &= ~(0x1 << 0);				// ADEN = 0; Disable ADC
	while( ADC1->CR & (0x1 << 0));			// wait for ADC to disable

	ADC1->CR |= (0x1 << 30);				// ADCALDIF = 0; single-input calibration
	ADC1->CR |=  (0x1 << 31);				// ADCAL = 1; start calibration
	while( ADC1->CR & (0x1 << 31) );		// wait for calibration to finish

	ADC1->DIFSEL = 0;						// Set all inputs to Single-Input

	ADC1->CFGR |= (0x1 << 13);				// CONT = 1; Continuous Conversion Mode
	ADC1->CFGR &= ~(0x3 << 3);				// 12-bit res

	ADC1->SMPR1 &= ~(0x1 << 31);			// no additional clocks

	// because the DMA operates at 4MHz, Tconv >= 0.25us
	ADC1->SMPR1 &= ~(0x7 << 15);			// clear sampling Rate for channel 5 (2.5 clocks)
//	ADC1->SMPR1 |=  (0x1 << 15);			// set the sampling rate for channel 5

	ADC1->SQR1 &= ~(0xF << 0);				// sequence length of 1
	ADC1->SQR1 &= ~(0x1F << 6);				// clear first sequence select
	ADC1->SQR1 |= (0x5 << 6);				// set the first sequence to be CH5

	ADC1->CFGR &= ~(0x3 << 10);				// EXTEN[1:0] = 0; disable hardware trigger detection


	ADC1->CR  |= (0x1 << 0);				// ADEN = 1
	while( !(ADC1->ISR & (0x1 << 0)) );		// wait for ADC ready flag
	ADC1->ISR |= (0x1 << 0);				// clear ADC ready flag (sets it ready)
//	ADC1->IER |= ((0x1 << 2) 				// enable EOC interrupt
//				 |(0x1 << 3) 				// enable EOS interrupt
//	);
	ADC1->ISR |= ((0x1 << 2) 				// clear EOC flag
			 	 |(0x1 << 3) 				// clear EOS flag

	);
//	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));		// enable USART2 ISR (bit 18)
//	__enable_irq();
	ADC1->CR |= (0x1 << 2);					// ADSTART = 1

}

//void ADC1_2_IRQHandler(void) {
//
//	//EOC
//	if(ADC1->ISR & (0x1 << 2)) {
//		ADC1->ISR |= (0x1 << 2);
//	}
//	//EOS
//	else if(ADC1->ISR & (0x1 << 3)) {
//		ADC1->ISR |= (0x1 << 3);
//	}
//}



