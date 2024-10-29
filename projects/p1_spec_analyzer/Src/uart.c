#include "main.h"
#include "uart.h"
#include "stdio.h"
#define SYS_CLK 32000000

#define BAUD_RATE 2000000	// value for 115.2 kbps for a clock at 4 MHz
//#define NUM_ROWS 	29
#define NUM_ROWS 	56
#define NUM_COLS 	128

#define ESC_CODE	0x1B
#define DOWN_ONE	"[1B"
#define LEFT_ONE	"[1D"

#define HOME_ESC	 	"[H"
#define COL_OFFSET 		"[5B"
#define SPEC_START_POS 	"[1C"
#define NEW_LINE 		"\n\r"

//#define GRID_START_POS 	"[6;18H"
#define GRID_START_POS 	"[1;1H"
#define FIRST_NUM_POS  	"[36;19H"
#define SECOND_NUM_POS 	"[36;33H"
#define THIRD_NUM_POS  	"[36;49H"
#define FOURTH_NUM_POS 	"[36;64H"
#define FIFTH_NUM_POS  	"[36;80H"

// prints the axis for the spectrum
void printAxis() {
	USART_ESC_Code("[2J");
	USART_ESC_Code(GRID_START_POS);
	for(uint8_t i = 0; i < NUM_ROWS; i++) {
		UART_print("|");
		USART_ESC_Code(DOWN_ONE);
		USART_ESC_Code(LEFT_ONE);
	}
	UART_print("|__________________________________________________________________________________________________________________________________");
//	printNumbers();
}

void printNumbers() {
	USART_ESC_Code(FIRST_NUM_POS);
	UART_print("1");
	USART_ESC_Code(SECOND_NUM_POS);
	UART_print("250");
	USART_ESC_Code(THIRD_NUM_POS);
	UART_print("500");
	USART_ESC_Code(FOURTH_NUM_POS);
	UART_print("750");
	USART_ESC_Code(FIFTH_NUM_POS);
	UART_print("1000");
}

void printGrid(int16_t *mags) {
	char cmd[3000] = {};
	char *cmd_ptr = cmd;
	USART_ESC_Code(HOME_ESC);
//	USART_ESC_Code(COL_OFFSET);


	for(uint32_t i = 0; i < NUM_ROWS; i++) {
		cmd_ptr = cmd;
		*cmd_ptr++ = ESC_CODE;
		memcpy(cmd_ptr, SPEC_START_POS, 3);
		cmd_ptr += 3;
		for(uint32_t j = 0; j < NUM_COLS; j++) {
//			if(mags[j] >= (NUM_ROWS-i)) {
//				*cmd_ptr++ = 'x';
//			}
			if(mags[j] <= (i/2)) {
				*cmd_ptr++ = 'x';
			}
			else {
				*cmd_ptr++ = ' ';
			}
		}
		memcpy(cmd_ptr, NEW_LINE, 2);
		cmd_ptr+=2;
		cmd_ptr[0] = '\0';
		UART_print(cmd);
	}


}

void USART_ESC_Code(char* code) {
	while(!(USART2->ISR & USART_ISR_TXE));		// wait until transmit data register is empty
	USART2->TDR = ESC_CODE;
	UART_print(code);
}

void UART_print(char* data) {
	uint32_t i;
	for(i = 0; data[i] != 0; i++) {
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

	USART2->CR1 &= ~(USART_CR1_TE		// disables transmitter	(3)
				    |USART_CR1_UE);		// disables USART		(0)

	// Sets 8-bit Mode
	USART2->CR1 &= ~(USART_CR1_M0
					|USART_CR1_M1);

	USART2->BRR = SYS_CLK / BAUD_RATE;

	USART2->CR1 |= (USART_CR1_TE		// enables transmitter	(3)
				   |USART_CR1_UE);		// enables USART		(0)
}

void float_to_string(float32_t num, char* buffer, uint8_t buff_len) {
	int32_t intPart;
	uint32_t divisor;
	float32_t decimalPart;
	uint32_t decimalDigit;

	for(uint8_t i = 0; i < buff_len; i++) {
		buffer[i] = 0;
	}

    // Convert integer part
    intPart = (int32_t)num;		// casts off fractional portion
    divisor = 1;

    while (intPart != 0) {
        intPart /= 10;			// divide until integer portion is 0
        divisor *= 10;			// multiply to get a "mask"
    }
    divisor /= 10;				// divide out extra digits place mask

	if (divisor == 0) {			// special case, already zero
		*buffer++ = '0';
	}

    intPart = (int32_t)num;		// casts off fractional portion
    while (divisor > 0) {		// go until there are still integers
        *buffer++ = '0' + (intPart / divisor);	// place into the string
        intPart %= divisor;		// grabs remainder
        divisor /= 10;			// next digit mask
    }

    // Convert fractional part, rounding to 2 decimal places
    *buffer++ = '.';
    decimalPart = num - (int32_t)num;		// grab decimal portion
    for (uint8_t i = 0; i < 2; i++) {
        decimalPart *= 10;					// "shift left" by one digit
        decimalDigit = (uint32_t)decimalPart;	// truncate remaining decimals
        *buffer++ = '0' + decimalDigit;		// place into the string
        decimalPart -= decimalDigit;			// subtract out the current decimal
    }

    *buffer = '\0';
}
