#ifndef SRC_BUTTONINTERRUPT_H_
#define SRC_BUTTONINTERRUPT_H_
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

extern SemaphoreHandle_t getSkittleSema;	// Semaphore Handles
void initButtonInterrupt(void);
void EXTI1_IRQHandler(void);

#endif /* SRC_BUTTONINTERRUPT_H_ */


