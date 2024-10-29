

#include "main.h"
#include "uart.h"
#include "timer_interrupt.h"
void SystemClock_Config(void);

char RDR[32];
uint8_t ind;

int main(void) {
	HAL_Init();
	SystemClock_Config();

	// GPIO INIT
	RCC->AHB2ENR &= ~(0x1 << 0);
	RCC->AHB2ENR |=  (0x1 << 0);	// enable GPIOA clock

	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |=  (0x1 << 10);	// PA5 output
	GPIOA->ODR &= ~(0x1 << 5);		// output LOW to LED
	UART_init();
	INIT_timer();
	__enable_irq();

	UART_print("Enter a blinking period (ms): ");

	while(1) {

	}
}// end main

void USART2_IRQHandler(void) {
	uint32_t num = 1000;			// default to 1 second

	// if the interrupt was from a reception of a byte
	if(USART2->ISR & (1 << 5)) {
		RDR[ind] = USART2->RDR;
		USART2->TDR = USART2->RDR;
		ind++;
		if(USART2->RDR == 13) {
			UART_print("\n");
			UART_print("Enter a blinking period (ms): ");
			RDR[ind-1] = '\0';
			ind = 0;
			num = string_to_int(RDR);
			set_timer(num);
		}
	USART2->RQR |= (1 << 3);		// clears RXNE bit using the RQR, RXFRQ bit (do I need this?)
	}
}

void TIM2_IRQHandler(void) {
	if(TIM2->SR & 1)
		TIM2->SR &= ~(0x1);		// clear status register of U.I. because we are servicing it
	GPIOA->ODR ^= (1 << 5);		// toggles the LED
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	//RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	/* from stm32l4xx_hal_rcc.h==
	#define RCC_MSIRANGE_0                 MSI = 100 KHz
	#define RCC_MSIRANGE_1                 MSI = 200 KHz
	#define RCC_MSIRANGE_2                 MSI = 400 KHz
	#define RCC_MSIRANGE_3                 MSI = 800 KHz
	#define RCC_MSIRANGE_4                 MSI = 1 MHz
	#define RCC_MSIRANGE_5                 MSI = 2 MHz
	#define RCC_MSIRANGE_6                 MSI = 4 MHz
	#define RCC_MSIRANGE_7                 MSI = 8 MHz
	#define RCC_MSIRANGE_8                 MSI = 16 MHz
	#define RCC_MSIRANGE_9                 MSI = 24 MHz
	#define RCC_MSIRANGE_10                MSI = 32 MHz
	#define RCC_MSIRANGE_11                MSI = 48 MHz   dont use this one*/
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void) {
/* USER CODE BEGIN Error_Handler_Debug */
/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1){}
/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
/* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
