#include "adc.h"
#include "dma.h"
#include "timer.h"
#include "uart.h"

#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"

#define FFT_LEN 2048
#define SAMPLE_FREQ 2048
#define SYS_CLK 32000000
#define SPEC_MAG 54
#define NUM_BINS 128
#define MAX_16INT 65535

uint16_t sampleInBuf[FFT_LEN*2];	// ADC Sample Buffer
int16_t fftOutBuf[FFT_LEN*2] = {};	// FFT Result Buffer
int16_t fftMagBuf[FFT_LEN/2];		// FFT Magnitude Buffer
int16_t avgs[NUM_BINS] = {};				// Frequency Average Bin Buffer

TaskHandle_t fftTaskHandler, uartTaskHandler;	// Task Handles
SemaphoreHandle_t dmaToFFTSema, fftToUARTSema;	// Semaphore Handles
arm_rfft_instance_q15 fftHandler;				// FFT Handle


void SystemClock_Config(void);
void StartDefaultTask(void *argument);
void initGPIOC(void);
void fftTask(void *argument);
void uartTask(void *argument);


int main(void) {
	BaseType_t retVal;	// used for checking task creation
	HAL_Init();

	SystemClock_Config();
	initGPIOC();	// GPIOC pin for Timing Calculations

	retVal = xTaskCreate(fftTask, "fftTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &fftTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(uartTask, "uartTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, &uartTaskHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	dmaToFFTSema = xSemaphoreCreateBinary();
	if( dmaToFFTSema == NULL ) { while(1); } 	// check if semaphore creation failed

	fftToUARTSema = xSemaphoreCreateBinary();
	if( fftToUARTSema == NULL ) { while(1); } 	// check if semaphore creation failed

	arm_rfft_init_q15(&fftHandler, FFT_LEN, 0, 1 );	// initialize the FFT

	UART_init();	// initialize the UART peripheral
	printAxis();	// print the spectrum axis

	init_P2M_DMA(&(ADC1->DR), (uint32_t*) sampleInBuf);	// initialize the DMA
	ADC_init();		// initialize the ADC
	ADC1->CR |= ADC_CR_ADSTART;							// ADSTART = 1; Start ADC Conversions

	PWM_init(SYS_CLK, SAMPLE_FREQ);	// initialize the PWM for ADC sampling
	vTaskStartScheduler();			// start the Scheduler

	while (1);

}

void fftTask(void *argument){
	uint16_t max_mag = 0;
	uint8_t blkSize = 8;
	uint16_t toggle = 0;
	uint8_t ind = 0;

	for(;;) {
		// Take semaphore from the DMA interrupt
		if( xSemaphoreTake( dmaToFFTSema, portMAX_DELAY ) == pdTRUE) {
			ind = 0;
			max_mag = 0;

			GPIOC->BSRR    |= GPIO_BSRR_BS0;			// Set PC0 Low


			arm_rfft_q15(&fftHandler, (int16_t * )(sampleInBuf+toggle), fftOutBuf);	  	// do the FFT
			arm_shift_q15(fftOutBuf, 8, fftOutBuf, FFT_LEN*2);						  	// Shift Fixed-Point
			arm_cmplx_mag_q15(fftOutBuf, fftMagBuf, FFT_LEN/2);		// Find Complex Mag
			arm_shift_q15(fftMagBuf, 1, fftMagBuf, FFT_LEN/2);


			fftMagBuf[0] = 0;
			// calculate the averages of 16 frequencies into one bin
			for(uint32_t i = 0; i < FFT_LEN / 2; i+=8) {
				arm_mean_q15(&fftMagBuf[i], blkSize, &(avgs[ind]));
				(avgs[ind] > max_mag) ? max_mag = avgs[ind] : max_mag;
				ind++;
			}
			// normalize with respect to max magnitude
//			for(uint32_t i = 0; i < NUM_BINS; i++) {
//				avgs[i] = ((float)avgs[i] / max_mag) * SPEC_MAG;
//				(avgs[i] > max_dB) ? max_dB = avgs[i] : max_dB;
//			}


			for(uint32_t i = 0; i < NUM_BINS; i++) {
				if(avgs[i] == 0) avgs[i] = 1;
				avgs[i] = 20*log10((float)avgs[i] / max_mag);
//				(avgs[i] > max_dB) ? max_dB = avgs[i] : max_dB;
//				(avgs[i] < min_dB) ? min_dB = avgs[i] : min_dB;
			}
			arm_abs_q15(avgs, avgs, NUM_BINS);
//			for(uint32_t i = 0; i < NUM_BINS; i++) {
//
//			}
//			for(uint32_t i = 0; i < NUM_BINS; i++) {
//				avgs[i] = ((float)avgs[i] / max_dB) * SPEC_MAG;
//			}
//			for(uint32_t i = 0; i < NUM_BINS; i++) {
//				avgs[i] = avgs[i] + (-1 * min_dB);
//			}


			// give semaphore to the UART Task
			xSemaphoreGive( fftToUARTSema );
			toggle ^= FFT_LEN;
		}
	}
}

void uartTask(void *argument) {
	for(;;) {
		// take semaphore from FFT Task
		if( xSemaphoreTake( fftToUARTSema, portMAX_DELAY ) == pdTRUE) {
			printGrid(avgs);
			GPIOC->BSRR    |= GPIO_BSRR_BR0;			// Set PC0 Low

		}
	}
}

void DMA1_Channel1_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Half-Transfer
	if((DMA1->ISR & DMA_ISR_HTIF1)) {
		xSemaphoreGiveFromISR( dmaToFFTSema, &xHigherPriorityTaskWoken );
		DMA1->IFCR |= DMA_IFCR_CHTIF1;	// clear half-transfer flag
	}
	// Complete-Transfer
	if((DMA1->ISR & DMA_ISR_TCIF1)) {
		xSemaphoreGiveFromISR( dmaToFFTSema, &xHigherPriorityTaskWoken );
		DMA1->IFCR |= DMA_IFCR_CTCIF1;		// clear complete-transfer flag
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * Configure PC0-PC3 for GPIO Output
 * push-pull, low speed, no pull-up/pull-down resistors
 * Initialize all to 0s
 */
void initGPIOC(void) {
	RCC->AHB2ENR   |=  RCC_AHB2ENR_GPIOCEN;		// enable GPIOC Clock
	GPIOC->MODER   &= ~GPIO_MODER_MODE0;		// clear PC0 Moder
	GPIOC->MODER   |=  GPIO_MODER_MODE0_0;		// PC0 Output Mode
	GPIOC->OTYPER  &= ~GPIO_OTYPER_OT0;			// Push-Pull
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;	// Low Speed
	GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPD0;		// No Pull-up, Pull-down
	GPIOC->BSRR    |= GPIO_BSRR_BR0;			// Set PC0 Low
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
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
