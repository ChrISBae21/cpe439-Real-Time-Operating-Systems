void UART_init();
void UART_print(char* data);
void USART_ESC_Code(char* code);
void USART2_IRQHandler(void);
void int_to_string(uint32_t num, char* buffer, uint32_t buff_len);
uint32_t string_to_int(char* strnum);
