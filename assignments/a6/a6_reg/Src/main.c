// PC0 is a timing pin
// PC1 is the pin to be tested
#include "main.h"
void SystemClock_Config(void);

void initTestGPIOC(void);
void initPin(void);

int main(void) {
	HAL_Init();
	SystemClock_Config();
	initTestGPIOC();
	GPIOA->BSRR |= GPIO_BSRR_BS0;			// Set PC0 High
	initPin();
	GPIOA->BSRR |= GPIO_BSRR_BR0;			// Set PC0 Low

	while (1) {
		GPIOC->BSRR |= GPIO_BSRR_BS1;			// Set PC0 Low
		GPIOC->BSRR |= GPIO_BSRR_BR1;			// Set PC0 Low
	}
}

void initPin(void) {
	RCC->AHB2ENR   |=  RCC_AHB2ENR_GPIOCEN;		// enable GPIOC Clock
	GPIOC->MODER   &= ~GPIO_MODER_MODE1;		// clear PC0 Moder
	GPIOC->MODER   |=  GPIO_MODER_MODE1_0;		// PC0 Output Mode
	GPIOC->OTYPER  &= ~GPIO_OTYPER_OT1;			// Push-Pull
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;	// Low Speed
	GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPD1;		// No Pull-up, Pull-down
	GPIOC->BSRR    |= GPIO_BSRR_BR1;			// Set PC0 Low
}

void initTestGPIOC(void) {
	RCC->AHB2ENR   |=  RCC_AHB2ENR_GPIOAEN;		// enable GPIOC Clock
	GPIOA->MODER   &= ~GPIO_MODER_MODE0;		// clear PC0 Moder
	GPIOA->MODER   |=  GPIO_MODER_MODE0_0;		// PC0 Output Mode
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT0;			// Push-Pull
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;	// Low Speed
	GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPD0;		// No Pull-up, Pull-down
	GPIOA->BSRR    |= GPIO_BSRR_BR0;			// Set PC0 Low
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
