// PA0 is
// PA1 is Ramp (Motor1)

#include "main.h"
#include "motorLib.h"


#define NUM_ANGLES 180
#define MAX_DC 12.5
#define MIN_DC 3
#define MAX_ANGLE 180
#define MIN_ANGLE 0
float MOTOR_LUT[181];
uint32_t ARR;



/* Initializes the LUT for Duty Cycles */
void initMotorLUT() {
	uint16_t angle;
	for(angle = 0; angle < NUM_ANGLES+1; angle++) {
		MOTOR_LUT[angle] = MIN_DC + ((MAX_DC - MIN_DC) * angle) / (MAX_ANGLE - MIN_ANGLE);
	}
}

void MOTOR_GPIO_init() {
	// Configure GPIO and AFR
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;		// enable GPIOC Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;		// turn on clock to TIM2

	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);			// clear PA0 and PA1
	GPIOA->MODER |=  GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;		// set PA0 and PA1 to AF

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);			// clear AF
	GPIOA->AFR[0] |=  (GPIO_AFRL_AFSEL0_0 | GPIO_AFRL_AFSEL1_0);		// set PA0 and PA1 to AF1
}


void MOTOR1_init() {
	MOTOR_GPIO_init();

	TIM2->CCER 	&= ~TIM_CCER_CC2P;		// CC2P (Polarity) Active High

	TIM2->CR1 	|= TIM_CR1_ARPE;		// enable the Auto-Reload Preload Register

	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;		// Set the Preload Register for CCR2

	TIM2->CCMR1 |=  ((TIM_CCMR1_OC2M_1)	// set bit 13 for OC2M Channel 2 Mode 2
					|(TIM_CCMR1_OC2M_2)	// set bit 14 for OC2M Channel 2 Mode 2
	);

	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;		// clear to set Channel 2 as output

	ARR = (SYS_CLK / MOTOR_FREQ);
	TIM2->ARR  = ARR - 1;
	TIM2->CCR2 = (uint32_t)((MOTOR_LUT[90]/100) * ARR-1) - 1;

	TIM2->EGR	|= TIM_EGR_UG;			// set the UG bit
	TIM2->CCER	|= TIM_CCER_CC2E;		// enable Capture Mode for Channel 2
	TIM2->CR1	|= TIM_CR1_CEN;			// enable counter
}


void MOTOR2_init() {
	MOTOR_GPIO_init();
	TIM2->CCER 	&= ~TIM_CCER_CC1P;		// CC2P (Polarity) Active High

	TIM2->CR1 	|= TIM_CR1_ARPE;		// enable the Auto-Reload Preload Register

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;		// Set the Preload Register for CCR2

	TIM2->CCMR1 |=  ((TIM_CCMR1_OC1M_1)	// set bit 5 for OC2M Channel 1 Mode 1
					|(TIM_CCMR1_OC1M_2)	// set bit 6 for OC2M Channel 1 Mode 1
	);

	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;		// clear to set Channel 1 as output

	ARR = (SYS_CLK / MOTOR_FREQ);
	TIM2->ARR  = ARR - 1;
	TIM2->CCR1 = (uint32_t)((MOTOR_LUT[5]/100) * ARR-1) - 1;

	TIM2->EGR	|= TIM_EGR_UG;			// set the UG bit
	TIM2->CCER	|= TIM_CCER_CC1E;		// enable Capture Mode for Channel 1
	TIM2->CR1	|= TIM_CR1_CEN;			// enable counter
}




void setMotor1Angle(uint16_t angle) {
	TIM2->CCER	&= ~TIM_CCER_CC2E;			// disable Capture Mode for Channel 2
	TIM2->CCR2 = (uint32_t)(((float)MOTOR_LUT[angle]/100) * ARR) - 1;

	TIM2->CCER	|= TIM_CCER_CC2E;			// enable Capture Mode for Channel 2


}

void setMotor2Angle(uint16_t angle) {
	TIM2->CCER	&= ~TIM_CCER_CC1E;			// disable Capture Mode for Channel 2
	TIM2->CCR1 = (uint32_t)(((float)MOTOR_LUT[angle]/100) * ARR) - 1;

	TIM2->CCER	|= TIM_CCER_CC1E;			// enable Capture Mode for Channel 2
}
void disableMotor1() {

}
void disableMotor2() {

}


