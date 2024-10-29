
#ifndef UART_H_
#define UART_H_
#include "arm_math.h"
#endif /* UART_H_ */
void UART_init();
void UART_print(char* data);
void USART_ESC_Code(char* code);
void USART2_IRQHandler(void);
void float_to_string(float32_t num, char* buffer, uint8_t buff_len);
