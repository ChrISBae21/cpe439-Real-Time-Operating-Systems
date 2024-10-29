#include "main.h"
#include "cmsis_os.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "buttonInterrupt.h"

#define NUM_LED_BLINK 5
#define LED_PERIOD 200

TaskHandle_t task1Handler, task2Handler, initTaskHandler;
volatile unsigned long ulHighFrequencyTimerTicks;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PortC_Init(void);
void initLED();

/* Task function prototypes  ------------------------------------------------*/
void Task1(void *argument);
void Task2(void *argument);

SemaphoreHandle_t taskSema;	// Semaphore Handles


/* main --------------------------------------------------------------------*/
int main(void)
{
  BaseType_t retVal;	// used for checking task creation

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  // Initialize GPIO
  initLED();


  /* Create the tasks */
  retVal = xTaskCreate(Task1, "task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task1Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed

  retVal = xTaskCreate(Task2, "task2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &task2Handler);
  if (retVal != pdPASS) { while(1);}	// check if task creation failed


  taskSema = xSemaphoreCreateBinary();
  if( taskSema == NULL ) { while(1); } 	// check if semaphore creation failed
  initButtonInterrupt();

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
  }
}

/* Define Tasks --------------------------------------------------------------*/
void Task1(void *argument) {
	uint8_t count = 0;
	for(;;) {
		count = 0;
		if( xSemaphoreTake( taskSema, portMAX_DELAY ) == pdTRUE) {
			while(count < NUM_LED_BLINK) {
				/* Enable LED */
				GPIOA->BSRR |= GPIO_BSRR_BS5;
//				GPIOC->BSRR |= GPIO_BSRR_BS0;
				vTaskDelay((LED_PERIOD/2) / portTICK_PERIOD_MS);	// wait for some time
				/* Disable LED */
				GPIOA->BSRR |= GPIO_BSRR_BR5;
//				GPIOC->BSRR |= GPIO_BSRR_BR0;
				vTaskDelay((LED_PERIOD/2) / portTICK_PERIOD_MS);	// wait for some time
				count++;
			}
		}
	}
}

void Task2(void *argument) {
	for(;;) {
		xSemaphoreGive( taskSema );
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}




void initLED() {
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |=  RCC_AHB2ENR_GPIOAEN;	// enable GPIOA clock

	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |=  GPIO_MODER_MODE5_0;	// PA5 output

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


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
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
