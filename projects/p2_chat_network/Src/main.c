#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "uart.h"
#include "gpio.h"

#include "nodes.h"
#include "networks.h"
#include "timer.h"

#include <stdio.h>
#include "spsgrf.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include <ctype.h>


#define START 			0x01
#define ACK 			0x02
#define HEARTBEAT 		0x03
#define MESSAGE			0x04

uint8_t rxPayload[280] = {0};
char myName[21] = {0};
uint8_t txPayload[280] = {0};



TaskHandle_t joinTaskHandler, recvTaskHandler, refreshTaskHandler, heartbeatTaskHandler, transmitMessageTaskHandler;	// Task Handles
SemaphoreHandle_t startSema, refreshSema, heartbeatSema, RxTxMutex, transmitSema;	// Semaphore and Mutex Handles


volatile SpiritFlagStatus xRxTimeoutFlag;
void joinTask(void *argument);
void recvTask(void *argument);
void refreshTask(void *argument);
void heartbeatTask(void *argument);
void transmitMessageTask(void *argument);
uint8_t getName(uint8_t *inputData, uint8_t *destName);
void SystemClock_Config(void);

int main(void) {
	BaseType_t retVal;	// used for checking task creation
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();


	retVal = xTaskCreate(joinTask, "joinTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 10, &joinTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(recvTask, "recvTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY +5, &recvTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(refreshTask, "refreshTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &refreshTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(heartbeatTask, "heartbeatTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 6, &heartbeatTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(transmitMessageTask, "transmitMessageTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, &transmitMessageTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	startSema = xSemaphoreCreateBinary();
	if( startSema == NULL ) { while(1); } 		// check if semaphore creation failed

	heartbeatSema = xSemaphoreCreateBinary();
	if( heartbeatSema == NULL ) { while(1); } 	// check if semaphore creation failed

	refreshSema = xSemaphoreCreateBinary();
	if( refreshSema == NULL ) { while(1); } 	// check if semaphore creation failed

	transmitSema = xSemaphoreCreateBinary();
	if( transmitSema == NULL ) { while(1); } 		// check if semaphore creation failed

	RxTxMutex = xSemaphoreCreateMutex();
    if( RxTxMutex == NULL ) { while(1); }		// check if mutex creation failed

	vTaskStartScheduler();	// start the Scheduler

	while (1);
}

/*
 * Join task which initializes needed timers and RF and joins the network
 * Deletes itself after it's done
 */
void joinTask(void *argument) {
	MX_SPI1_Init();
	SPSGRF_Init();					// initializes the RF
	UART_init();
	UART_print("\x1B[2J");
	UART_print("\x1B[H");
	UART_print("Enter Username:");
	UART_print("\x20");

	if( xSemaphoreTake( transmitSema, portMAX_DELAY ) == pdTRUE ) {
		strcpy(myName, (char*)txPayload);
	}

	xRxTimeoutFlag = S_RESET;
	if( xSemaphoreTake( RxTxMutex, portMAX_DELAY ) == pdTRUE ) {
		joinNetwork();					// send initial join packet to the network
		xSemaphoreGive( RxTxMutex );	// give the mutex back
	}
	INIT_timer(SYS_CLK, 1);			// initialize 1 Hz timer
	xSemaphoreGive( startSema );	// gives program flow to the rest of the tasks
	vTaskDelete(NULL);				// deletes itself
	for(;;);
}

/*
 * Task that receives packets via the RF board
 */
void recvTask(void *argument) {
	uint8_t type, recNode, destNode;
	if( xSemaphoreTake( startSema, portMAX_DELAY ) == pdTRUE) {
		for(;;) {

			xRxTimeoutFlag = S_RESET;

			/* Take the Mutex so start receiving */
			if( xSemaphoreTake( RxTxMutex, portMAX_DELAY ) == pdTRUE ) {
				receiveData(rxPayload, &recNode, &destNode);	// Recevie incoming data
				xSemaphoreGive( RxTxMutex );					// Give the mutex back
			}
			if(!xRxTimeoutFlag) {
				type = rxPayload[0]; // type of packet

				switch(type) {

				/* New Node on the Network */
				case START:
					/* ACK new node */
					sendAck(recNode);
					/* update the Node List */
					updateNodes(recNode, rxPayload+1);
					break;

				/* Incoming Message */
				case MESSAGE:
					printMessage(rxPayload+1, recNode, destNode);

				/* Received Heartbeat */
				case HEARTBEAT:
				/* ACKs after joining */
				case ACK:
					/* update the Node List */
					updateNodes(recNode, rxPayload+1);
					break;
				/* Do nothing if the packet is invalid */
				default:
					break;
				}
			}

			xRxTimeoutFlag = S_RESET;
		}
	}
}

/*
 * Task that checks and refreshes the online nodes and prints to the serial terminal
 */
void refreshTask(void *argument) {
	for(;;) {
		if( xSemaphoreTake( refreshSema, portMAX_DELAY ) == pdTRUE) {	// grab semaphore from TIM2
			refreshNodes();		// refresh the nodes
		}
	}
}

/*
 * Task that transmits a heartbeat to all other nodes on the network
 */
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

void transmitMessageTask(void *argument) {
	char flag;
	uint16_t len = 0;
	uint8_t *rdrPtr = txPayload;
	uint8_t destAddr = BROADCAST_ADDRESS;
	uint8_t pduBuffer[280], destName[21] = {0};

	for(;;) {
		if( xSemaphoreTake( transmitSema, portMAX_DELAY ) == pdTRUE ) {	// grab semaphore from Rx/Tx
			strcpy((char*)(pduBuffer+1), myName);
			rdrPtr = txPayload;
			flag = (char) *rdrPtr;
			rdrPtr+=2;	// increment space and NULL

			switch(tolower(flag)) {
			case 'l':
				printNodes();
				break;

			case 'm':
				rdrPtr += 1 + getName(rdrPtr, destName); //space
				if((destAddr = getNodeAddress((char*)destName)) == 1) {
						UART_print("Username '");
						UART_print((char*) destName);
						UART_print("' does not exist\n\r");
					break;			// doesn't exist
				}

			case 'b':
				len = buildPDU(pduBuffer, rdrPtr, MESSAGE_CODE[0]);

				if( xSemaphoreTake( RxTxMutex, portMAX_DELAY ) == pdTRUE ) {	// grab semaphore from Rx/Tx
					transmitData(destAddr, pduBuffer, len);
					xSemaphoreGive( RxTxMutex );	// give the mutex back
				}
				break;

			default:
				break;
			}

//			xSemaphoreGive( transmitSema );	// give the mutex back
		}
	}
}

uint8_t getName(uint8_t *inputData, uint8_t *destName) {
	uint8_t len = 0;
	while(((char)inputData[len] != ' ')) {
		destName[len] = inputData[len];
		len++;
	}
	destName[len] = '\0';
	return len;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus = {0};

  if (GPIO_Pin != SPIRIT1_GPIO3_Pin)
  {
    return;
  }

  SpiritIrqGetStatus(&xIrqStatus);
  if (xIrqStatus.IRQ_TX_FIFO_ERROR)
  {
    xTxDoneFlag = S_SET;
  }
  if (xIrqStatus.IRQ_TX_DATA_SENT)
  {
    xTxDoneFlag = S_SET;
  }
  if (xIrqStatus.IRQ_RX_DATA_READY)
  {
    xRxDoneFlag = S_SET;
  }
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    xRxTimeoutFlag = S_SET;
    xRxDoneFlag = S_SET;
  }
  if(xIrqStatus.IRQ_TX_FIFO_ALMOST_EMPTY) {
	  xTxFifoFlag = S_SET;
  }
  if(xIrqStatus.IRQ_RX_FIFO_ALMOST_FULL) {
	  xRxFifoFlag = S_SET;
  }
}

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
