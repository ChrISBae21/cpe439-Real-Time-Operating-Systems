/*
 * dma.c
 *
 *  Created on: Apr 12, 2024
 *      Author: Chris Bae
 */
#include "main.h"

void init_M2M_DMA(uint16_t *src, uint16_t *dest) {
	RCC->AHB1ENR |= (0x1 << 0);			// enable clock to DMA1

	DMA1_Channel1->CCR &= ~(0x1 << 0);	// disable channel
	DMA1_Channel1->CCR |=  (0x1 << 14);	// memory-to-memory mode
	DMA1_Channel1->CCR &= ~((0x3 << 10) // clear transfer data size for Source
						   |(0x3 << 8)	// clear transfer data size for Destination
	);
	DMA1_Channel1->CCR |= ((0x1 << 10) 	// 16-bit Source data transfers
						  |(0x1 << 8)	// 16-bit Destination data transfers
	);
	DMA1_Channel1->CCR |= ((0x1 << 7)	// Source Memory Increment Mode Enabled
			              |(0x1 << 6)	// Destination Memory Increment Mode Enabled
	);
	DMA1_Channel1->CCR &= ~(0x1 << 5);	// Circular Mode Disabled
	DMA1_Channel1->CCR |=  (0x1 << 4);	// Direction bit

	DMA1_Channel1->CNDTR = 1000;		// number of data transfers
	DMA1_Channel1->CPAR = (uint32_t) dest;			// Destination Address
	DMA1_Channel1->CMAR = (uint32_t) src;			// Source Address


	DMA1_Channel1->CCR |= (0x1 << 0);	// enable channel
}


void init_P2M_DMA(volatile uint32_t *peripheral, uint16_t *dest) {
	RCC->AHB1ENR |= (0x1 << 0);			// enable clock to DMA1

	DMA1_Channel1->CCR &= ~(0x1 << 0);	// disable channel
	DMA1_Channel1->CCR &= ~(0x1 << 14);	// disable memory-to-memory mode
	DMA1_Channel1->CCR &= ~((0x3 << 10) // clear transfer data size for Memory
						   |(0x3 << 8)	// clear transfer data size for Peripheral
	);
	DMA1_Channel1->CCR |= ((0x1 << 10) 	// 16-bit Memory data transfers
						  |(0x1 << 8)	// 16-bit Peripheral data transfers
	);
	DMA1_Channel1->CCR |= (0x1 << 7);	// Enable Memory Increment Mode
	DMA1_Channel1->CCR &= ~(0x1 << 6);	// Disable Peripheral Increment Mode

	DMA1_Channel1->CCR |= (0x1 << 5);	// Enable Circular Mode
	DMA1_Channel1->CCR &=  ~(0x1 << 4);	// Read from Peripheral
	DMA1_Channel1->CCR |= ((0x1 << 2)	// Enable Half-Transfer Interrupts
						  |(0x1 << 1)	// Enable Complete Transfer Interrupts
	);

	DMA1_Channel1->CNDTR = 1000;		// number of data transfers
	DMA1_CSELR->CSELR &= ~(0xF);

	DMA1_Channel1->CPAR = (uint32_t) peripheral;	// Peripheral Address
	DMA1_Channel1->CMAR = (uint32_t) dest;			// Destination Memory Address

	NVIC->ISER[0] = (1 << (11 & 0x1F));		// enable DMA1_CH1 ISR (bit 11)

	__enable_irq();

	DMA1_Channel1->CCR |= (0x1 << 0);	// enable channel

}


void DMA1_Channel1_IRQHandler(void) {
	// Half-Transfer
	if((DMA1->ISR & (0x1 << 2))) {
		DMA1->IFCR |= (0x1 << 2);
	}
	// Complete Transfer
	else if((DMA1->ISR & (0x1 << 1))) {
		DMA1->IFCR |= (0x1 << 1);
	}
	GPIOC->ODR ^= (0x1 << 0);	// Toggle GPIO Pin
}
