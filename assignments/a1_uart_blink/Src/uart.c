/*
 *
 * GPIO Pins
 * PA2: USART2_TX
 * PA3: USART2_RX
 *
 *
 */

/*
 * Character Reception Procedure on pp. 1350 of Reference Manual
 * Character Transmission Procedure on pp. 1347 of Reference Manual
 */

#include "main.h"
#include "uart.h"


void USART_ESC_Code(char* code) {
	while(!(USART2->ISR & (1 << 7)));		// wait until transmit data register is empty
	USART2->TDR = 0x1B;
	UART_print(code);
}

void UART_print(char* data) {
	uint8_t i;
	for(i = 0; data[i] != 0; i++) {
		while(!(USART2->ISR & (1 << 7)));		// wait until transmit data register is empty
		USART2->TDR = data[i];
	}
}

void UART_init() {
	RCC->APB1ENR1 |= (1 << 17);			// enable USART2 clock
	RCC->AHB2ENR |= (0x1 << 0);			// enable GPIOA clock

	// UART GPIO INIT
	GPIOA->MODER &= ~(0x3 << 4 | 0x3 << 6);			// clears GPIOA pins 2-3
	GPIOA->MODER |=  (0x2 << 4 | 0x2 << 6);			// sets GPIOA pins 2-3 as alternate function mode (10)

	GPIOA->AFR[0] &= ~(0xF << 8 | 0xF << 12); 		// clears AFR for GPIOA pins 2-3
	GPIOA->AFR[0] |=  (0x7 << 8 | 0x7 << 12); 		// sets AFR GPIOA pins 2-3 as AF7

	/*
	 *	Bit 11 is useless because bit 13 is disabled
	 * 	Bit 9 is useless because bit 10 is disabled
	 *
	 */

	USART2->CR1 &= ~((1 << 28)		// clear M1 for 1 start bit and 8-bit data 		(28)
					|(0x3 << 26)	// clear EOBIE and RTOIE to prevent interrupts	(26, 27)
					|(1 << 15)		// clear OVER8 bit to oversample by 16			(15)
					|(1 << 14)		// clear CMIE to prevent interrupt				(14)
					|(1 << 13)		// clear MME to disable Mute Mode				(13)
					|(1 << 12) 		// clear M0 for 1 start bit and 8-bit data		(12)
					|(1 << 10)		// clear PCE to disable parity					(10)
					|(1 << 8)		// clear PEIE to disable parity error interrupt	(8)
					|(1 << 7) 		// clear TXEIE to disable TXE interrupt			(7)
					|(1 << 6)		// clear TCIE to prevent interrupt 				(6)
					|(1 << 4)		// clear IDLEIE to disable idle interrupts		(4)
					|(1 << 3)		// clear TE: don't enable transmit yet			(3)
					|(1 << 2)		// clear RE: don't enable receive yet			(2)
					|(1 << 0)		// clear UE: don't enable USART yet				(0)
					);
	USART2->CR1 |=   (1 << 5);		// set RXNEIE to enable RXN interrupt		(5)

	/*
	 * Bit 18 is used by a different mode (smartcard)
	 *
	 */
	USART2->CR2 &= ~((1 << 23)		// clear RTOEN to disable reciever timeout		(23)
					|(1 << 20)		// clear ABREN to disable auto baud rate		(20)
					|(1 << 19)		// clear MSBFIRST to start LSB					(19)
					|(1 << 17)		// clear TXINV for TX to idle high				(17)
					|(1 << 16)		// clear RXINV for RX to idle high				(16)
					|(1 << 15)		// clear SWAP to use default pinout				(15)
					|(1 << 14)		// clear LINEN to disable LIN mode				(14)
					|(0x3 << 12)	// set STOP to have 1 stop bit					(12, 13)
					|(1 << 11)		// clear CLKEN for asynchronous mode			(11)
					|(1 << 6)		// clear LBDIE to disable LIN interrupt			(6)
					);

	USART2->CR3 &= ~((1 << 24)		// clear TCBGTIE to disable guard time inter.	(24)
					|(1 << 14)		// clear DEM to  disable driver enable 			(14)
					|(1 << 9) 		// disable CTS									(9)
					|(1 << 8)		// disable RTS									(8)
					|(1 << 7)		// disable DMA transmitter						(7)
					|(1 << 6)		// disable DMA reciever							(6)
					|(1 << 5)		// disable smartcard							(5)
					|(1 << 4)		// disable NACK									(4)
					|(1 << 3)		// disable half-duplex							(3)
					|(1 << 1)		// disable IrDA mode							(1)
					|(1 << 0)		// disable error interrupts						(0)
					);

	/*USART2->CR3 |= 	((1 << 12)		// disable OVERRUN								(12)
					|(1 << 11)		// ONEBIT to sample with one bit				(11)
					);*/

	//USART2->BRR = 35;				// value for 115.2 kbps for a clock at 4 MHz
	USART2->BRR = 35;				// value for 115.2 kbps for a clock at 4 MHz


	USART2->CR1 |= ((1 << 3)		// enables transmitter	(3)
				   |(1 << 2)		// enables receiver		(2)
				   |(1 << 0)		// enables USART		(0)
				   );

	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));		// enable USART2 ISR (bit 38)
//	__enable_irq();

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



