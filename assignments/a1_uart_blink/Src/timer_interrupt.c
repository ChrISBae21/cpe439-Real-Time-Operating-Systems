#include "main.h"
#include "timer_interrupt.h"
#include <math.h>


void INIT_timer() {
  NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));// TIM2 bit 28 of ISER0 	In Programming Manual NVIC_ISERx, and Reference Manual 13.3
  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);	// turn on clock to TIM2
  TIM2->ARR = 3999999;						// set count reload value 3999999 (4000000 total, f = 1Hz from 4MHz)
  TIM2->DIER |= (0x1);						// enable the auto-reload register interrupt, or U.I. (ref man pg 975)

  // interrupt flags basically "block" interrupts
  TIM2->SR &= ~(0x1);						// clear interrupt flag (ref man pg 978)
  TIM2->CR1 |= (0x1 << 0);					// enable counter: ref man pg 969

}

void set_timer(float time) {
    TIM2->CR1 &= ~(TIM_CR1_CEN);            // disable timer to edit tim2 properties
    float arr = 4000000 * time * pow(10, -3);
    TIM2->EGR |= 0x1; 					//trigger an update interupt to restart the counter
    TIM2->ARR = round(arr - 1);			// set ARR value to adjust with frequency
    //TIM2->DIER |= (0x1);                // enable the auto-reload register interrupt, or U.I.
    TIM2->SR &= ~(0x1);            		// clear interrupt flag
    TIM2->CR1 |= TIM_CR1_CEN;           // enable timer
}


