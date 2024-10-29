#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"

#include "buttonInterrupt.h"
#include "watchdog.h"

#define NUM_LED_BLINK 5
#define LED_PERIOD 400

TaskHandle_t setupTaskHandler, countingTaskHandler;	// Task Handles
uint8_t flag = 0;

void setupTask(void *argument);
void countingTask(void *argument);
void initLED();
void initGPIOC(void);
void SystemClock_Config(void);


int main(void) {
	BaseType_t retVal;
	HAL_Init();

	SystemClock_Config();
	initLED();
	initGPIOC();

	retVal = xTaskCreate(setupTask, "setupTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &setupTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(countingTask, "countingTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &countingTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	vTaskStartScheduler();			// start the Scheduler

	while (1);
}

void setupTask(void *argument) {
	uint8_t count = 0;
	for(;;) {
		while(count < NUM_LED_BLINK) {
			GPIOA->BSRR |= GPIO_BSRR_BS5;
			vTaskDelay((LED_PERIOD/2) / portTICK_PERIOD_MS);
			GPIOA->BSRR |= GPIO_BSRR_BR5;
			vTaskDelay((LED_PERIOD/2) / portTICK_PERIOD_MS);
			count++;
		}
		initButtonInterrupt();
		initWatchdog();
		flag = 1;
		vTaskDelete(NULL);
	}
}
void countingTask(void *argument) {


	while(!flag);
	for(;;) {
		GPIOC->BSRR |= GPIO_BSRR_BS0;
		vTaskDelay(10 / portTICK_PERIOD_MS);
		GPIOC->BSRR |= GPIO_BSRR_BR0;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}


void initLED() {
	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |=  RCC_AHB2ENR_GPIOAEN;	// enable GPIOA clock

	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |=  GPIO_MODER_MODE5_0;	// PA5 output
}

void initGPIOC(void) {
	RCC->AHB2ENR   |=  RCC_AHB2ENR_GPIOCEN;		// enable GPIOC Clock
	GPIOC->MODER   &= ~GPIO_MODER_MODE0;		// clear PC0 Moder
	GPIOC->MODER   |=  GPIO_MODER_MODE0_0;		// PC0 Output Mode
	GPIOC->OTYPER  &= ~GPIO_OTYPER_OT0;			// Push-Pull
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;	// Low Speed
	GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPD0;		// No Pull-up, Pull-down
	GPIOC->BSRR    |= GPIO_BSRR_BR0;			// Set PC0 Low
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}


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



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }

}

void Error_Handler(void) {
	__disable_irq();
	while (1);
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
