/*
 * dma.h
 *
 *  Created on: Apr 12, 2024
 *      Author: cbgno
 */

#ifndef SRC_DMA_H_
#define SRC_DMA_H_
#endif /* SRC_DMA_H_ */
void init_M2M_DMA(uint16_t *src, uint16_t *dest);
void init_P2M_DMA(volatile uint32_t *peripheral, uint16_t *dest);
void DMA1_Channel1_IRQHandler(void);
