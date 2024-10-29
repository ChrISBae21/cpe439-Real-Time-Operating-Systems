


#ifndef UART_H_
#define UART_H_
#include "FreeRTOS.h"
#include "semphr.h"
void UART_init();
void UART_print(char* data);
void UART_ESC_Code(char* code);
void USART2_IRQHandler(void);
void int_to_string(uint32_t num, char* buffer, uint32_t buff_len);
uint32_t string_to_int(char* strnum);

extern uint8_t txPayload[280];


extern SemaphoreHandle_t transmitSema;
#endif /* UART_H_ */
