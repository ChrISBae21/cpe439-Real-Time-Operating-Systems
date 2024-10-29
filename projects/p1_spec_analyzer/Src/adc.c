// DATASHEET SAYS PA0 IS ADC12_IN5, CHANNEL 5
#include "main.h"
#include "adc.h"

void ADC_init() {

	// CLOCK INITIALIZATION
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;		// enable GPIOA clock
	RCC->AHB2ENR  |= RCC_AHB2ENR_ADCEN;			// enable ADC clock
	RCC->CCIPR 	  |= RCC_CCIPR_ADCSEL;			// ADC Peripheral Clock set to SYSCLK

	// GPIO INITIALIZATION
	GPIOA->MODER &= ~GPIO_MODER_MODE0; 			// clear PA0
	GPIOA->MODER |=  GPIO_MODER_MODE0;			// set PA0 to Analog Input

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0;			// Clear PA0
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL0_0		// set PA0 as ADC input pin (0x7)
				     |GPIO_AFRL_AFSEL0_1
					 |GPIO_AFRL_AFSEL0_2);

	GPIOA->ASCR |= GPIO_ASCR_ASC0;				// Analog Switch Control Register

	ADC123_COMMON->CCR &= ~ADC_CCR_PRESC;		// clear ADC prescaler
//	ADC123_COMMON->CCR |= ADC_CCR_PRESC_1;
	ADC123_COMMON->CCR |= ADC_CCR_PRESC_2;
	ADC123_COMMON->CCR |= ADC_CCR_CKMODE_0;		// set clock to SYSCLK no Prescaler
	ADC123_COMMON->CCR |= ADC_CCR_DMACFG;		// DMA circular Mode

	ADC1->CFGR |= ADC_CFGR_DMAEN;				// Enable ADC DMA
	ADC1->CFGR |= ADC_CFGR_DMACFG;				// Enable ADC DMA Circular Mode

	ADC1->CR &= ~ADC_CR_DEEPPWD;				// get out of deep-power-down
	ADC1->CR |=  ADC_CR_ADVREGEN;				// turn on voltage regulator

	while( !(ADC1->CR & ADC_CR_ADVREGEN) );		// wait for voltage regulator to turn on

	ADC1->CR &= ~ADC_CR_ADEN;				// ADEN = 0; Disable ADC
	while( ADC1->CR & ADC_CR_ADEN);			// wait for ADC to disable

	ADC1->CR &= ~ADC_CR_ADCALDIF;			// ADCALDIF = 0; single-input calibration
	ADC1->CR |=  ADC_CR_ADCAL;				// ADCAL = 1; start calibration
	while( ADC1->CR & ADC_CR_ADCAL );		// wait for calibration to finish

	ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL;		// Set all inputs to Single-Input

	ADC1->CFGR &= ~ADC_CFGR_CONT;			// CONT = 0; Single Conversion Mode
	ADC1->CFGR &= ~ADC_CFGR_RES;			// 12-bit res

	ADC1->SMPR1 &= ~ADC_SMPR1_SMP5;			// clear sampling Rate for channel 5 (2.5 clocks)

	ADC1->SQR1 &= ~ADC_SQR1_L;				// sequence length of 1
	ADC1->SQR1 &= ~ADC_SQR1_SQ1;			// clear first sequence select
	ADC1->SQR1 |= (ADC_SQR1_SQ1_0			// set the first sequence to be CH5 (0x5)
				  |ADC_SQR1_SQ1_2);

	ADC1->CFGR |=  (ADC_CFGR_EXTSEL_0		// EXTEN SEL set to TIM2 CH2 (0x3)
				   |ADC_CFGR_EXTSEL_1);

	ADC1->CFGR |=  ADC_CFGR_EXTEN_0;		// EXTEN[1:0] = 1; Rising Edge Trigger

	ADC1->CR  |= ADC_CR_ADEN;				// ADEN = 1
	while( !(ADC1->ISR & ADC_ISR_ADRDY) );	// wait for ADC ready flag

	ADC1->ISR |= (ADC_ISR_EOC 				// clear EOC flag
			 	 |ADC_ISR_EOS); 			// clear EOS flag
}




