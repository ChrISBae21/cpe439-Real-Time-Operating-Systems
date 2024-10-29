#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include <ctype.h>

#include "i2c.h"
#include "color_sensor.h"
#include "motorLib.h"
#include "buttonInterrupt.h"

#define NUM_TASKS 3

/* Angle for each skittle bin */
#define BIN_1 30
#define BIN_2 65
#define BIN_3 100
#define BIN_4 130
#define BIN_5 165

/* Angles for the transfer ring */
#define DROP_ANGLE 180
#define SCAN_ANGLE 90
#define RESET_ANGLE 5

/* Task Handles */
TaskHandle_t initTaskHandler, testGiveTaskHandler, colorSenseTaskHandler, moveFunnelTaskHandler, getSkittleTaskHandler;
/* Semaphore and Mutex Handles */
SemaphoreHandle_t startSema, colorSenseSema, getSkittleSema;
QueueHandle_t maxColorQueue;

void initTask(void *argument);
void colorSenseTask(void *argument);
void moveFunnelTask(void *argument);
void getSkittleTask(void *argument);

int main(void) {
	BaseType_t retVal;	// used for checking task creation
	HAL_Init();
	SystemClock_Config();
	initMotorLUT();

	retVal = xTaskCreate(initTask, "initTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 15, &initTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(colorSenseTask, "colorSenseTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 4, &colorSenseTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(moveFunnelTask, "moveFunnelTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 4, &moveFunnelTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(getSkittleTask, "getSkittleTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 4, &getSkittleTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	startSema = xSemaphoreCreateCounting(NUM_TASKS, 0);
	if( startSema == NULL ) { while(1); } 		// check if semaphore creation failed

	colorSenseSema = xSemaphoreCreateBinary();
	if( colorSenseSema == NULL ) { while(1); } 		// check if semaphore creation failed

	getSkittleSema = xSemaphoreCreateBinary();
	if( getSkittleSema == NULL ) { while(1); } 		// check if semaphore creation failed

	maxColorQueue = xQueueCreate( 1, 1 );	// (number of items, number of bytes)
	if( maxColorQueue == NULL ) { while(1); } 		// check if queue creation failed

	vTaskStartScheduler();	// start the Scheduler

	while (1);
}


void initTask(void *argument) {
	MOTOR1_init();		// initialize first servo
	MOTOR2_init();		// initialize second servo
	I2C_init();			// initialize I2C
	sensorInit(SENSOR_INTEGRATIONTIME_50MS, SENSOR_GAIN_4X); // initialize color sensor

	/* start givig access to other tasks */
	xSemaphoreGive(startSema);
	xSemaphoreGive(startSema);
	xSemaphoreGive(startSema);
	initButtonInterrupt();		// initialize button interrupt
	vTaskDelete(NULL);			// delete this task
	for(;;);
}


void getSkittleTask(void *argument) {
	/* unblocks after the join task finishes */
	if( xSemaphoreTake( startSema, portMAX_DELAY ) == pdTRUE) {
		for(;;) {
			/* wait until the system is ready for the next skittle */
			if( xSemaphoreTake( getSkittleSema, portMAX_DELAY ) == pdTRUE) {
				setMotor2Angle(SCAN_ANGLE);
				xSemaphoreGive(colorSenseSema);	// give a semaphore to start color sensing
			}
		}
	}
}

void colorSenseTask(void *argument) {

	/* unblocks after the join task finishes */
	if( xSemaphoreTake( startSema, portMAX_DELAY ) == pdTRUE) {
		uint8_t ready = 0;
		int r, g, b;
		uint8_t color;

		for(;;) {
			/* wait until the system grabs a skittle */
			if( xSemaphoreTake( colorSenseSema, portMAX_DELAY ) == pdTRUE) {
				vTaskDelay(500 / portTICK_PERIOD_MS);	// wait until the skittle reaches the color sensor
				ready = 0;
				/* wait for color sensor to be ready to take samples */
				while(!ready) {
					ready = data_ready();	// ready to start color sampling
				}
				getRGB(&r, &g, &b);			// grabs RGB values
				color = maxColor(r, g);		// finds the max color

				/* passes the max color to the moveFunnelTask for sorting */
				xQueueSend( maxColorQueue, &color, portMAX_DELAY);
			}
		}
	}

}


void moveFunnelTask(void *argument) {
	/* unblocks after the join task finishes */
	if( xSemaphoreTake( startSema, portMAX_DELAY ) == pdTRUE) {
		uint8_t maxColor, active = 1;
		for(;;) {
			/* waits until a maximum color is received  */
			if( xQueueReceive(maxColorQueue, &maxColor, portMAX_DELAY) == pdTRUE) {
				active = 1;
				switch(maxColor) {
				case PURPLE:
					/* move to the first bin */
					setMotor1Angle(BIN_1);
					break;
				case RED:
					/* move to the second bin */
					setMotor1Angle(BIN_2);
					break;
				case GREEN:
					/* move to the third bin */
					setMotor1Angle(BIN_3);
					break;
				case ORANGE:
					/* move to the fourth bin */
					setMotor1Angle(BIN_4);
					break;
				case YELLOW:
					/* move to the fifth bin */
					setMotor1Angle(BIN_5);
					break;
					/* no skittle detected */
				case NO_COLOR:
					active = 0;
					break;
				}
				/* wait until ramp has moved to the bin */
				vTaskDelay(500 / portTICK_PERIOD_MS);
				/* spins the skittle transfer ring to the drop hole */
				setMotor2Angle(DROP_ANGLE);
				/* wait until the skittle has been dropped */
				vTaskDelay(500 / portTICK_PERIOD_MS);
				/* resets the skittle transfer ring */
				setMotor2Angle(RESET_ANGLE);
				/* wait for the skittle transfer ring has reset */
				vTaskDelay(1000 / portTICK_PERIOD_MS);


				if(active) {	// if a skittle was detected, continue
					NVIC->ISER[1] &= ~(0x1 << (EXTI15_10_IRQn & 0x1F));	// EXTI1 Interrupt enabled
					xSemaphoreGive(getSkittleSema);
				}
				else {			// no skittle detected, wait for a button start
					RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;		// turn off clock to TIM2 (for motors)
					NVIC->ISER[1] |= (0x1 << (EXTI15_10_IRQn & 0x1F));	// EXTI1 Interrupt enabled
				}
			}
		}
	}
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
