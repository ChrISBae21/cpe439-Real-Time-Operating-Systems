/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "adc.h"

void SystemClock_Config(void);
void initGPIOC(void);
int main(void) {
	HAL_Init();
	SystemClock_Config();

	initGPIOC();

	uint16_t src[1000] = {};
	uint16_t dest[1000] = {};
	for(uint16_t i = 0; i < 1000; i++)
		src[i] = i;	// populate source array

	/*
	 * Copying data using a for loop
	 */
	GPIOC->BSRR |= (0x1 << 0);		// Set PC0 High: start
	for(uint16_t i = 0; i < 1000; i++) {
		dest[i] = src[i];
	}
	GPIOC->BSRR |= (0x1 << 16);		// Set PC0 Low: end
	HAL_Delay(1000);				// Pause for one second before moving on to the next test

	// CLEAR THE DESTINATION ARRAY FOR DMA NOW
	for(uint16_t i = 0; i < 1000; i++) {
			dest[i] = 0;
	}

	/*
	 * Copying data using DMA
	 */
	init_M2M_DMA(src, dest);
	GPIOC->BSRR |= (0x1 << 0);		// Set PC0 High: start
	while(!(DMA1->ISR & (0x1 << 1)));	// Wait until DMA is done
	GPIOC->BSRR |= (0x1 << 16);		// Set PC0 Low: end

	HAL_Delay(1000);				// Pause for one second before moving on to the next test

//	 CLEAR THE DESTINATION ARRAY FOR DMA NOW
	for(uint16_t i = 0; i < 1000; i++) {
		dest[i] = 0;
	}
	/* ----------------------------------------------------------------------------------------- */
	init_P2M_DMA(&(ADC1->DR), dest);

	ADC_init();



	while (1) {
	}

}

void initGPIOC(void) {
	RCC->AHB2ENR |= (0x1 << 2);		// enable GPIOC Clock
	GPIOC->MODER &= ~(0x3 << 0);	// clear PC0 Moder
	GPIOC->MODER |=  (0x1 << 0);	// PC0 Output Mode
	GPIOC->OTYPER &= ~(0x1 << 0);	// Push-Pull
	GPIOC->OSPEEDR &= ~(0x3 << 0);	// Low Speed
	GPIOC->PUPDR &= ~(0x3 << 0);	// No Pull-up, Pull-down

	GPIOC->BSRR |= (0x1 << 16);		// Set PC0 Low
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
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1);
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
