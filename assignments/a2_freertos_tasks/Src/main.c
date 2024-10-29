/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

TaskHandle_t task1Handler, task2Handler, task3Handler, task4Handler;
volatile unsigned long ulHighFrequencyTimerTicks;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PortC_Init(void);
void RTOS_Stats_Timer_Init(void);

/* Task function prototypes  ------------------------------------------------*/
void Task1(void *argument);
void Task2(void *argument);
void Task3(void *argument);
void Task4(void *argument);

/* main --------------------------------------------------------------------*/
int main(void)
{
  BaseType_t retVal;	// used for checking task creation

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  // Initialize GPIO
  PortC_Init();

  /* Create the tasks */
  retVal = xTaskCreate(Task1, "task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task1Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task2, "task2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task2Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task3, "task3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task3Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task4, "task4", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task4Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
  }
}

/* Define Tasks --------------------------------------------------------------*/
void Task1(void *argument)
{
	for(;;)
	{
		GPIOC->ODR ^= (0x1 << 0);			// toggle PC0
		//for(int i=0; i<4000; i++);
		vTaskDelay(2 / portTICK_PERIOD_MS);
	}
}

void Task2(void *argument)
{
	for(;;)
	{
		GPIOC->ODR ^= (0x1 << 1);			// toggle PC1
		//for(int i=0; i<20000; i++);
		vTaskDelay(3 / portTICK_PERIOD_MS);
	}
}

void Task3(void *argument)
{
	for(;;)
	{
		GPIOC->ODR ^= (0x1 << 2);			// toggle on PC2
		//for(int i=0; i<10000; i++);
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void Task4(void *argument)
{
	for(;;)
	{
		GPIOC->ODR ^= (0x1 << 3);			// toggle on PC3
		//for(int i=0; i<10000; i++);
		vTaskDelay(15 / portTICK_PERIOD_MS);
	}
}
/*
 * Configure PC0-PC3 for GPIO Output
 * push-pull, low speed, no pull-up/pull-down resistors
 * Initialize all to 0s
 */
void PortC_Init(void)
{
	// turn on clock to GPIOC
	RCC->AHB2ENR |= (0x1 << 2);

	// Configure PC0-PC3 for GPIO output to control LED
	GPIOC->MODER   &= ~((0x3 << 0) | (0x3 << 2) |
					    (0x3 << 4) | (0x3 << 6));

	GPIOC->MODER   |=  ((0x1 << 0) |		// PC0 Output (01)
					    (0x1 << 2) |		// PC1 Output (01)
					    (0x1 << 4) |		// PC2 Output (01)
					    (0x1 << 6));		// PC3 Output (01)

	// Output push-pull (1)
	GPIOA->OTYPER  &= ~((0x1 << 0) | (0x1 << 2) |
					    (0x1 << 3) | (0x1 << 4));
	// Low speed (00)
	GPIOA->OSPEEDR &= ~((0x3 << 0) | (0x3 << 2) |
					    (0x3 << 4) | (0x3 << 6));
	// No pull-up, pull-down (00)
	GPIOA->PUPDR   &= ~((0x3 << 0) | (0x3 << 2) |
		    			(0x3 << 4) | (0x3 << 6));

	GPIOA->BSRR    =   ((0x1 << 16) | (0x1 << 17) |
					    (0x1 << 18) | (0x1 << 19));
}

/* Configure Timer to interrupt 100 kHz (100 times every Tick) */
void RTOS_Stats_Timer_Init(void)
{
	RCC->APB1ENR1 |= ((0x1 << 0));	// turn on TIM2
	TIM2->DIER |= ((0x1 << 0));		// enable interrupts
	TIM2->SR  &= ~((0x1 << 0));		// clear interrupt flag
	TIM2->ARR = 400 - 1;
	TIM2->CR1 |= (0x1 << 0);		// start timer

	// enable interrupts
	NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));
}

/* Timer 2 is used to collect runtime stats for FreeRTOS tasks*/
void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~((0x1 << 0));		// clear interrupt flag
	ulHighFrequencyTimerTicks++;
}

/* Built in functions for using FreeRTOS runtime stats need to be defined*/
void configureTimerForRunTimeStats(void)
{
    ulHighFrequencyTimerTicks = 0;
    RTOS_Stats_Timer_Init();
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
