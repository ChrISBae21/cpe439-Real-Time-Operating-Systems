#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "nodes.h"
#include "networks.h"
#include "timer.h"

#include <stdio.h>
#include "spsgrf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#define SYS_CLK 		40000000
#define START 			0x01
#define ACK 			0x02
#define HEARTBEAT 		0x03
#define MAX_PAYLOAD 280

uint8_t payload[MAX_PAYLOAD] = {0};

TaskHandle_t joinTaskHandler, recvTaskHandler, refreshTaskHandler, heartbeatTaskHandler;	// Task Handles
SemaphoreHandle_t startSema, refreshSema, heartbeatSema, RxTxMutex;	// Semaphore and Mutex Handles


volatile SpiritFlagStatus xRxTimeoutFlag;
void joinTask(void *argument);
void recvTask(void *argument);
void refreshTask(void *argument);
void heartbeatTask(void *argument);
void SystemClock_Config(void);

int main(void) {
	BaseType_t retVal;	// used for checking task creation
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();


	retVal = xTaskCreate(joinTask, "joinTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 8, &joinTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 4, &recvTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(refreshTask, "refreshTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 5, &refreshTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(heartbeatTask, "heartbeatTask", configMINIMAL_STACK_SIZE, NULL,
						 tskIDLE_PRIORITY + 5, &heartbeatTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	startSema = xSemaphoreCreateBinary();
	if( startSema == NULL ) { while(1); } 		// check if semaphore creation failed

	heartbeatSema = xSemaphoreCreateBinary();
	if( heartbeatSema == NULL ) { while(1); } 	// check if semaphore creation failed

	refreshSema = xSemaphoreCreateBinary();
	if( refreshSema == NULL ) { while(1); } 	// check if semaphore creation failed

	RxTxMutex = xSemaphoreCreateMutex();
    if( RxTxMutex == NULL ) { while(1); }		// check if mutex creation failed

	vTaskStartScheduler();	// start the Scheduler

	while (1);
}


/* Join task which initializes needed timers and RF and joins the network
 * Deletes itself after it's done */
void joinTask(void *argument) {
	SPSGRF_Init();					// initializes the RF
	xRxTimeoutFlag = S_RESET;
	joinNetwork();					// send initial join packet to the network
	INIT_timer(SYS_CLK, 1);			// initialize 1 Hz timer
	xSemaphoreGive( startSema );	// gives program flow to the rest of the tasks
	vTaskDelete(NULL);				// deletes itself
	for(;;);
}

/* Task that receives packets via the RF board */
void recvTask(void *argument) {
	uint8_t type, recNode;
	/* take semaphore from the join task after initialization */
	if( xSemaphoreTake( startSema, portMAX_DELAY ) == pdTRUE) {
		for(;;) {
			payload[0] = 0;	// clear the flag in the payload
			xRxTimeoutFlag = S_RESET;
			/* Start transmission by taking the Mutex */
			if( xSemaphoreTake( RxTxMutex, portMAX_DELAY ) == pdTRUE ) {

				/* receive incoming data */
				receiveData(payload, &recNode);

				/* give the mutex back */
				xSemaphoreGive( RxTxMutex );
			}
			if(!xRxTimeoutFlag) {
				type = payload[0];		// type of packet

				switch(type) {

				/* New Node on the Network */
				case START:
					/* transmit the ACK PDU */
					sendAck(recNode);

				/* Received Heartbeat */
				case HEARTBEAT:
				/* ACKs after joining */
				case ACK:
					/* update the Node List */
					updateNodes(recNode);
					break;
				default:
					updateNodes(recNode);
					break;
				}
			}
			xRxTimeoutFlag = S_RESET;
		}
	}
}

/* Task that checks and refreshes the online nodes and prints to the serial terminal */
void refreshTask(void *argument) {
	for(;;) {
		if( xSemaphoreTake( refreshSema, portMAX_DELAY ) == pdTRUE) {	// grab semaphore from TIM2
			refreshNodes();		// refresh the nodes
		}
	}
}

/* Task that transmits a heartbeat to all other nodes on the network */
void heartbeatTask(void *argument) {
	for(;;) {
		if( xSemaphoreTake( heartbeatSema, portMAX_DELAY ) == pdTRUE) {		// grab semaphore from TIM2
			refreshNodes();			// refresh the nodes
			if( xSemaphoreTake( RxTxMutex, portMAX_DELAY ) == pdTRUE ) {	// grab semaphore from Rx/Tx
				sendHeartbeat();	// send a heartbeat
				xSemaphoreGive( RxTxMutex );	// give the mutex back
			}

		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	SpiritIrqs xIrqStatus = {0};

	if (GPIO_Pin != SPIRIT1_GPIO3_Pin) {
		return;
	}

	SpiritIrqGetStatus(&xIrqStatus);
	/* Error with the FIFO */
	if (xIrqStatus.IRQ_TX_FIFO_ERROR) {
		xTxDoneFlag = S_SET;
	}

	/* Transmit Data Sent*/
	if (xIrqStatus.IRQ_TX_DATA_SENT) {
		xTxDoneFlag = S_SET;
	}

	/* Receive Data Ready */
	if (xIrqStatus.IRQ_RX_DATA_READY) {
		xRxDoneFlag = S_SET;
	}

	/* Receive Timeout*/
	if (xIrqStatus.IRQ_RX_TIMEOUT) {
		xRxTimeoutFlag = S_SET;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

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
