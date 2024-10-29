#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

void Error_Handler(void);

#define SYS_CLK 		40000000
extern char myName[21];

#define SPIRIT1_GPIO3_Pin GPIO_PIN_7
#define SPIRIT1_GPIO3_GPIO_Port GPIOC
#define SPIRIT1_GPIO3_EXTI_IRQn EXTI9_5_IRQn
#define SPIRIT1_SDN_Pin GPIO_PIN_10
#define SPIRIT1_SDN_GPIO_Port GPIOA
#define SPIRIT1_SPI_CSn_Pin GPIO_PIN_6
#define SPIRIT1_SPI_CSn_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
