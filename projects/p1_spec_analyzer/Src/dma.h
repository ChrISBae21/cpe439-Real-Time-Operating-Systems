#ifndef SRC_DMA_H_
#define SRC_DMA_H_
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#endif /* SRC_DMA_H_ */
void init_P2M_DMA(volatile uint32_t *peripheral, uint32_t *dest);
void DMA1_Channel1_IRQHandler(void);

