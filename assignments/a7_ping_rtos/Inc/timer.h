#ifndef SRC_TIMER_INTERRUPT_H_
#define SRC_TIMER_INTERRUPT_H_
#include "FreeRTOS.h"
#include "semphr.h"

void TIM2_IRQHandler(void);
void INIT_timer(uint32_t sys_clk, uint32_t timer_freq);
extern uint32_t TIME_COUNT;
extern SemaphoreHandle_t startSema;	// Semaphore Handles
extern SemaphoreHandle_t refreshSema;
extern SemaphoreHandle_t heartbeatSema;
#endif /* SRC_TIMER_INTERRUPT_H_ */
