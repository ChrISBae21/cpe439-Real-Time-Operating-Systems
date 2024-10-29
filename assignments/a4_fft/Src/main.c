/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "adc.h"
#include "timer.h"
#include "uart.h"
#include "arm_math.h"



#define FFT_LEN 2048
#define SAMPLE_FREQ 2048
#define SYS_CLK 4000000
#define AMP 1.5
#define OFFSET 1.5

uint16_t sampleInBuf[FFT_LEN] = {};
int16_t fftInBuf[FFT_LEN] = {};
int16_t fftOutBuf[FFT_LEN*2] = {};

void SystemClock_Config(void);
void testFFT(float32_t freq, float32_t *xin);
void initGPIOC(void);
void processHalf(uint16_t index);

int main(void) {
	int16_t fftMagBuf[FFT_LEN/2];
//	int16_t fftMagBuf[FFT_LEN/2];
	int16_t max_mag = 0;
	int16_t max_freq = 0;
	uint32_t i;
	char freq_str[16] = {};


	HAL_Init();
	SystemClock_Config();

	initGPIOC();



	UART_init();
	init_P2M_DMA(&(ADC1->DR), (uint32_t*) sampleInBuf);
	ADC_init();
	ADC1->CR |= (0x1 << 2);					// ADSTART = 1; Start ADC Conversions

	PWM_init(SYS_CLK, SAMPLE_FREQ);


	while (1) {
		arm_rfft_instance_q15 fftHandler;			// initialize FFT struct
		arm_rfft_init_q15(&fftHandler, FFT_LEN, 0, 1 );

		while(!(DMA1->ISR & (0x1 << 2)));	// Waits until first half of the array is filled with samples
		processHalf(0);						// Does a calibration on the samples
		DMA1->IFCR |= (0x1 << 2);

		while(!(DMA1->ISR & (0x1 << 1))); 	// Waits until second hald of the array is filled with samples
		processHalf(FFT_LEN / 2);			// Does a calibration on the samples
		DMA1->IFCR |= (0x1 << 1);

		GPIOC->BSRR |= (0x1 << 0);									// PC0 High
		arm_rfft_q15(&fftHandler, fftInBuf, fftOutBuf);				// do the FFT
											// PC0 Low

		arm_cmplx_mag_q15(fftOutBuf, fftMagBuf, FFT_LEN);
		arm_max_q15(&fftMagBuf[1], (FFT_LEN/2) - 1, &max_mag, &i);			// Find index of the max magnitude
		max_freq = ((float32_t) (i * SAMPLE_FREQ) / (FFT_LEN));		// Find frequency

		float_to_string(max_freq, freq_str, 16);
		UART_print(freq_str);
		UART_print("\n\r");
		GPIOC->BSRR |= (0x1 << 16);
	}
}


void processHalf(uint16_t index) {

	// Process the data (calibrate)
	for(int i = 0; i < FFT_LEN / 2; i++) {
		fftInBuf[index] = sampleInBuf[index];
//		fftInBuf[index] = (float32_t) sampleInBuf[index];
		index++;
	}

}
void testFFT(float32_t freq, float32_t *xin) {
	for(int i = 0; i < FFT_LEN; i++) {
		xin[i] = 1.5*sin(((2*PI*freq)/ SAMPLE_FREQ)*i) + 1.5;
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






//
//
//
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "dma.h"
//#include "adc.h"
//#include "timer.h"
//#include "uart.h"
//#include "arm_math.h"
//
//
//
//#define FFT_LEN 2048
//#define SAMPLE_FREQ 2048
//#define SYS_CLK 4000000
//#define AMP 1.5
//#define OFFSET 1.5
//
//uint32_t sampleInBuf[FFT_LEN] = {};
//float32_t fftInBuf[FFT_LEN] = {};
//float32_t fftOutBuf[FFT_LEN*2] = {};
//
//void SystemClock_Config(void);
//void testFFT(float32_t freq, float32_t *xin);
//void initGPIOC(void);
//void processHalf(uint16_t index);
//
//int main(void) {
//	float32_t fftMagBuf[FFT_LEN/2];
//	float32_t max_mag = 0;
//	float32_t max_freq = 0;
//	uint32_t i;
//	char freq_str[16] = {};
//
//
//	HAL_Init();
//	SystemClock_Config();
//
//	initGPIOC();
//
//	arm_rfft_fast_instance_f32 fftHandler;			// initialize FFT struct
//	arm_rfft_fast_init_f32(&fftHandler, FFT_LEN);	// initialize FFT
//
//	arm_rfft_init_q15();
//
//	UART_init();
//	init_P2M_DMA(&(ADC1->DR), (uint32_t*) sampleInBuf);
//	ADC_init();
//	ADC1->CR |= (0x1 << 2);					// ADSTART = 1; Start ADC Conversions
//
//	PWM_init(SYS_CLK, SAMPLE_FREQ);
//
//	while (1) {
//
//
//		while(!(DMA1->ISR & (0x1 << 2)));	// Waits until first half of the array is filled with samples
//		processHalf(0);						// Does a calibration on the samples
//		DMA1->IFCR |= (0x1 << 2);
//
//		while(!(DMA1->ISR & (0x1 << 1))); 	// Waits until second hald of the array is filled with samples
//		processHalf(FFT_LEN / 2);			// Does a calibration on the samples
//		DMA1->IFCR |= (0x1 << 1);
//
//		GPIOC->BSRR |= (0x1 << 0);									// PC0 High
//		arm_rfft_fast_f32(&fftHandler, fftInBuf, fftOutBuf, 0);		// do the FFT
//		GPIOC->BSRR |= (0x1 << 16);									// PC0 Low
//		arm_cmplx_mag_f32(fftOutBuf, fftMagBuf, FFT_LEN/2);			// Calculate magnitudes of the FFT
//		arm_max_f32(fftMagBuf, FFT_LEN/2, &max_mag, &i);			// Find index of the max magnitude
//		max_freq = ((float32_t) (i * SAMPLE_FREQ) / FFT_LEN);		// Find frequency
//
//		float_to_string(max_freq, freq_str, 16);
//		UART_print(freq_str);
//		UART_print("\n\r");
//
//	}
//}
//
//
//void processHalf(uint16_t index) {
//
//	// Process the data (calibrate)
//	for(int i = 0; i < FFT_LEN / 2; i++) {
//		fftInBuf[index] = (0.000848*sampleInBuf[index] + .0191 - OFFSET)/AMP;
////		fftInBuf[index] = (float32_t) sampleInBuf[index];
//		index++;
//	}
//
//}
//void testFFT(float32_t freq, float32_t *xin) {
//	for(int i = 0; i < FFT_LEN; i++) {
//		xin[i] = 1.5*sin(((2*PI*freq)/ SAMPLE_FREQ)*i) + 1.5;
//	}
//}
//void initGPIOC(void) {
//	RCC->AHB2ENR |= (0x1 << 2);		// enable GPIOC Clock
//	GPIOC->MODER &= ~(0x3 << 0);	// clear PC0 Moder
//	GPIOC->MODER |=  (0x1 << 0);	// PC0 Output Mode
//	GPIOC->OTYPER &= ~(0x1 << 0);	// Push-Pull
//	GPIOC->OSPEEDR &= ~(0x3 << 0);	// Low Speed
//	GPIOC->PUPDR &= ~(0x3 << 0);	// No Pull-up, Pull-down
//
//	GPIOC->BSRR |= (0x1 << 16);		// Set PC0 Low
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void) {
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	*/
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	* in the RCC_OscInitTypeDef structure.
//	*/
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	*/
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
//		Error_Handler();
//	}
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void) {
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1);
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line) {
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
