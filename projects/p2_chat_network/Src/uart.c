/*
 *
 * GPIO Pins
 * PA2: USART2_TX
 * PA3: USART2_RX
 *
 *
 */

#define BAUDRATE 115200
#define CARRIAGE_RETURN 13

#include "main.h"
#include "uart.h"




void USART2_IRQHandler(void) {
	static uint32_t ind = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// if the interrupt was from a reception of a byte
	if(USART2->ISR & USART_ISR_RXNE) {
		txPayload[ind] = USART2->RDR;
		USART2->TDR = USART2->RDR;
		ind++;
		if(USART2->RDR == CARRIAGE_RETURN) {
			ind--;
			txPayload[ind] = '\0';
			UART_print("\x1B[E");
			UART_print("\x1B[1B");
			xSemaphoreGiveFromISR( transmitSema , xHigherPriorityTaskWoken);
//			txPayload[0] = '\0';
			ind = 0;
		}
	USART2->RQR |= USART_RQR_RXFRQ;		// clears RXNE bit using the RQR, RXFRQ bit (do I need this?)
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);		// allow tasks to unblock

}


void UART_ESC_Code(char* code) {
	while(!(USART2->ISR & USART_ISR_TXE));		// wait until transmit data register is empty
	USART2->TDR = '\x1B';
	UART_print(code);
}

void UART_print(char* data) {
	uint8_t i;
	for(i = 0; data[i] != '\0'; i++) {
		while(!(USART2->ISR & USART_ISR_TXE));		// wait until transmit data register is empty
		USART2->TDR = data[i];
	}
}

void UART_init() {
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;			// enable USART2 clock
	RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;			// enable GPIOA clock

	// UART GPIO INIT
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);			// clears GPIOA pins 2-3
	GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);		// sets GPIOA pins 2-3 as alternate function mode (10)

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3); 							// clears AFR for GPIOA pins 2-3
	GPIOA->AFR[0] |=  (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2		// sets AFR GPIOA pins 2 as AF7
				      |GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2);	// sets AFR GPIOA pins 3 as AF7

	USART2->CR1 &= ~(USART_CR1_M1		// clear M1 for 1 start bit and 8-bit data 		(28)
					|USART_CR1_OVER8	// clear OVER8 bit to oversample by 16			(15)
					|USART_CR1_M0 		// clear M0 for 1 start bit and 8-bit data		(12)
					|USART_CR1_TE		// clear TE: don't enable transmit yet			(3)
					|USART_CR1_RE		// clear RE: don't enable receive yet			(2)
					|USART_CR1_UE		// clear UE: don't enable USART yet				(0)
					);
	USART2->CR1 |=  USART_CR1_RXNEIE;	// set RXNEIE to enable RXN interrupt			(5)

	USART2->CR2 &= ~(USART_CR2_MSBFIRST	// clear MSBFIRST to start LSB					(19)
					|USART_CR2_TXINV	// clear TXINV for TX to idle high				(17)
					|USART_CR2_RXINV	// clear RXINV for RX to idle high				(16)
					|USART_CR2_STOP		// set STOP to have 1 stop bit					(12, 13)
					|USART_CR2_CLKEN	// clear CLKEN for asynchronous mode			(11)
					|USART_CR2_LBDIE	// clear LBDIE to disable LIN interrupt			(6)
					);

	USART2->BRR = SYS_CLK / BAUDRATE;


	USART2->CR1 |= (USART_CR1_TE		// enables transmitter	(3)
				   |USART_CR1_RE		// enables receiver		(2)
				   |USART_CR1_UE		// enables USART		(0)
				   );
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	// set NVIC interrupt priority
	NVIC_SetPriority(USART2_IRQn, 6);					// set interrupt priority
	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR (bit 38)


}

void int_to_string(uint32_t num, char* buffer, uint32_t buff_len) {
	for(uint32_t i = 0; i < buff_len; i++) {
		buffer[i] = 0;
	}

    // Convert integer part
	uint32_t temp = (uint32_t)num;
	uint32_t len = 0;
    while (temp != 0) {
        temp /= 10;
        len++;
    }

    if (len == 0) {
        *buffer++ = '0';
    }
    temp = (uint32_t)num;
    while (len > 0) {
    	uint32_t rem = 1;
        for (int i = 1; i < len; i++) {
            rem *= 10;
        }
        *buffer++ = '0' + (temp / rem);
        temp %= rem;
        len--;
    }
    *buffer = '\0';
}


uint32_t string_to_int(char* strnum) {
	uint32_t num = 0;
//	uint32_t mult = 1;
	for(int i = 0; strnum[i] != '\0'; i++) {
		num = num * 10 + (strnum[i] - 48);
		strnum[i] = '\0';			// reset the array
	}
	return num;
}



