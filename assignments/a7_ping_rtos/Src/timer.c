#include "main.h"
#include "timer.h"
#include "nodes.h"
#include "usart.h"
#include "networks.h"
#include <stdio.h>
#include "spsgrf.h"


#define REFRESH_TIME 	3
#define HEARTBEAT_TIME 	30


void INIT_timer(uint32_t sys_clk, uint32_t timer_freq) {
	RCC->APB1ENR1	|= (RCC_APB1ENR1_TIM2EN);		// turn on clock to TIM2
	TIM2->ARR 		= sys_clk / timer_freq - 1;		// set the ARR value
	TIM2->DIER 		|= TIM_DIER_UIE;				// enable the auto-reload register interrupt, or U.I. (ref man pg 975)
	TIM2->SR 		&= ~TIM_SR_UIF;					// clear interrupt flag (ref man pg 978)

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	// set NVIC interrupt priority
	NVIC_SetPriority(TIM2_IRQn, 7);					// set interrupt priority
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));		// enable DMA1_CH1 ISR (bit 11)

	TIM2->CR1 	|= TIM_CR1_CEN;						// enable counter: ref man pg 969

}

void TIM2_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;		// clear status register of U.I. because we are servicing it
		TIME_COUNT++;					// increment the global time count

		if((TIME_COUNT % HEARTBEAT_TIME) == 0){
			xSemaphoreGiveFromISR( heartbeatSema, &xHigherPriorityTaskWoken );	// give heartbeatTask semaphore
		}
		else if((TIME_COUNT % REFRESH_TIME) == 0) {
			xSemaphoreGiveFromISR( refreshSema, &xHigherPriorityTaskWoken );	// give refreshTask semaphore
		}
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// allow tasks to unblock

	}

}
