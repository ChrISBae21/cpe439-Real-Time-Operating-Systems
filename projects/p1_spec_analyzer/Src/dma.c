#include "dma.h"

#define FFT_LEN 2048

void init_P2M_DMA(volatile uint32_t *peripheral, uint32_t *dest) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;			// enable clock to DMA1

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;			// disable channel
	DMA1_Channel1->CCR &= ~DMA_CCR_MEM2MEM;		// disable memory-to-memory mode
	DMA1_Channel1->CCR &= ~(DMA_CCR_MSIZE 		// clear transfer data size for Memory
						   |DMA_CCR_PSIZE);		// clear transfer data size for Peripheral

	DMA1_Channel1->CCR |= (DMA_CCR_MSIZE_0 		// 32-bit Memory data transfers
						  |DMA_CCR_PSIZE_0);	// 16-bit Peripheral data transfers

	DMA1_Channel1->CCR |=  DMA_CCR_MINC;		// Enable Memory Increment Mode
	DMA1_Channel1->CCR &= ~DMA_CCR_PINC;		// Disable Peripheral Increment Mode

	DMA1_Channel1->CCR |=  DMA_CCR_CIRC;		// Enable Circular Mode
	DMA1_Channel1->CCR &=  ~DMA_CCR_DIR;		// Read from Peripheral

	DMA1_Channel1->CCR |= (DMA_CCR_HTIE				// Enable Half-Transfer Interrupts
						  |DMA_CCR_TCIE);			// Enable Complete Transfer Interrupts


	DMA1_Channel1->CNDTR = FFT_LEN * 2;				// number of data transfers
	DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S;			// Channel 1 Select to ADC1

	DMA1_Channel1->CPAR = (uint32_t) peripheral;	// Peripheral Address
	DMA1_Channel1->CMAR = (uint32_t) dest;			// Destination Memory Address

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 6);
	NVIC->ISER[0] = (1 << (DMA1_Channel1_IRQn & 0x1F));				// enable DMA1_CH1 ISR (bit 11)

	__enable_irq();

	DMA1_Channel1->CCR |= DMA_CCR_EN;	// enable DMA channel for data transfers

}



