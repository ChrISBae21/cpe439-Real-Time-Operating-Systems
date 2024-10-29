#ifndef SRC_BUTTONINTERRUPT_H_
#define SRC_BUTTONINTERRUPT_H_

extern SemaphoreHandle_t taskSema;	// Semaphore Handles
void initButtonInterrupt(void);
void EXTI1_IRQHandler(void);

#endif /* SRC_BUTTONINTERRUPT_H_ */


