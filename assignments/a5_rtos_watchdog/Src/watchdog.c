#include "main.h"

#define WDG_ENABLE 0x0000CCCC
#define WDG_WRITE_ACCESS 0x00005555
#define WDG_REFRESH 0x0000AAAA


void initWatchdog(void) {

	RCC->CSR |= RCC_CSR_LSION;				// enable LSI clock
	while(!(RCC->CSR & RCC_CSR_LSIRDY));	// wait for LSI clock to turn on

	IWDG->KR = WDG_ENABLE;					// enable IWDG
	IWDG->KR = WDG_WRITE_ACCESS;			// enable write access

	while((IWDG->SR & IWDG_SR_PVU));		// wait for prescaler to set
	IWDG->PR &= ~IWDG_PR_PR;				// clear prescaler
	while((IWDG->SR & IWDG_SR_PVU));		// wait for prescaler to set
	IWDG->PR |=  IWDG_PR_PR_2;				// prescaler of 64: 32kHz / 64

	while((IWDG->SR & IWDG_SR_RVU));		// wait for reload value to set
	IWDG->RLR &= ~IWDG_RLR_RL;				// clear reload register
	while((IWDG->SR & IWDG_SR_RVU));		// wait for reload value to set
	IWDG->RLR |= 2500;						// 5 seconds: Reload Value = time * (32000 / prescaler)

	while((IWDG->SR != 0));					// wait for all registers to set

	IWDG->KR = WDG_REFRESH;					// refresh counter

}
