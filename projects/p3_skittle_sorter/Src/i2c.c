#include "main.h"
#include "i2c.h"
#include "color_sensor.h"

#define ONE_BYTE 1
#define TWO_BYTES 2

/*
 * Initialized with AUTOEND off
 */
void I2C_init() {

	/* I2C Clock Init */
	RCC->AHB2ENR 	|=  RCC_AHB2ENR_GPIOCEN;		// Enable GPIOC Clock
	RCC->CCIPR 		&= ~RCC_CCIPR_I2C3SEL;			// Set the I2C3 Clock to PCLK from APB1
	RCC->APB1ENR1 	|=  RCC_APB1ENR1_I2C3EN;		// Enable I2C3 Clock on APB1 (PCLK)

	/* I2C GPIO Init */
	GPIOC->OTYPER 	|=  ( GPIO_OTYPER_OT0      | GPIO_OTYPER_OT1 );			// set PB6 and PB7 to open-drain
	GPIOC->PUPDR 	|=  ( GPIO_PUPDR_PUPD0_0   | GPIO_PUPDR_PUPD1_0 );		// PB6 and PB7 pull-up
	GPIOC->OSPEEDR 	|=  ( GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1 );	// set the speed
	GPIOC->MODER 	&= ~( GPIO_MODER_MODE0     | GPIO_MODER_MODE1 );		// clear PB6 and PB7 Pins
	GPIOC->MODER 	|=  ( GPIO_MODER_MODE0_1   | GPIO_MODER_MODE1_1 );		// set PB6 and PB7 as Alternate Function
	GPIOC->AFR[0] 	&= ~( GPIO_AFRL_AFSEL0     | GPIO_AFRL_AFSEL1 );		// clear AFR for PB6 and PB7
	GPIOC->AFR[0] 	|=  ( GPIO_AFRL_AFSEL0_2   | GPIO_AFRL_AFSEL1_2 );		// set AFR4 for PB6 and PB7

	/* I2C Register Init */
	I2C3->CR1 &= ~I2C_CR1_PE;			// disable the I2C1 before setting registers

	// Before enabling I2C1, Filters must be set
	I2C3->CR1 &= ~I2C_CR1_ANFOFF_Msk;	// Enable Analog Filter
	I2C3->CR1 &= ~I2C_CR1_DNF_Msk;		// disable Digital Noise Filter
	I2C3->TIMINGR &= ~(0xFFFFFFFF);		// clear timing register
	I2C3->TIMINGR |= 0x00000102;		// timing from ioc for 400khz, 30ns rise/fall times
	I2C3->CR1 &= ~I2C_CR1_NOSTRETCH;	// NOSTRETCH must be cleared in Master Mode

	I2C3->CR1 &= ~I2C_CR1_GCEN;			// clear General Call Enable
	I2C3->OAR1 &= ~(I2C_OAR1_OA1EN);	// Clear Own Address Register 1
	I2C3->OAR2 &= ~(I2C_OAR2_OA2EN);	// Clear Own Address Register 2

	I2C3->CR2 &= ~I2C_CR2_AUTOEND; 		// AUTOEND OFF
	I2C3->CR2 &= ~I2C_CR2_RELOAD;

	I2C3->CR1 |=  I2C_CR1_PE;			// enable the I2C1
}


/* reads an 8 bit value over I2C with sensor commands*/
uint8_t I2C_read8(uint8_t periphAddr, uint8_t regAddr) {
	uint8_t recvData;
	uint8_t txBuf[1] = {(uint8_t)(SENSOR_COMMAND_BIT | regAddr)};
	I2C_write(periphAddr, txBuf, 1);

	I2C_periph_addr(periphAddr);	// set the peripheral address
	I2C3->CR2 |= I2C_CR2_RD_WRN;	// Controller Read Request
	I2C3->CR2 &= ~I2C_CR2_NBYTES;	// Clear the number of bytes
	I2C3->CR2 |= (ONE_BYTE << I2C_CR2_NBYTES_Pos);			// 1 byte to transfer

	I2C3->CR2 |= I2C_CR2_START;		// START the data transfer

    while (!(I2C3->ISR & I2C_ISR_RXNE));		// wait until all data is received
    recvData = I2C3->RXDR;

    // Send NACK and stop condition
    while (!(I2C3->ISR & I2C_ISR_TC));		// wait for Transfer Complete
    I2C3->CR2 |= I2C_CR2_STOP;				// issue I2C STOP

    return recvData;
}


/* reads a 16 bit values over I2C with sensor commands */
uint16_t I2C_read16(uint8_t periphAddr, uint8_t regAddr) {
	uint16_t retVal = 0;
	uint8_t bytes = 0;

	uint8_t rxBuf[2];
	uint8_t txBuf[1] = {(uint8_t)(SENSOR_COMMAND_BIT | regAddr)};
	I2C_write(periphAddr, txBuf, 1);

	I2C_periph_addr(periphAddr);	// set the peripheral address
	I2C3->CR2 |= I2C_CR2_RD_WRN;	// Controller Read Request
	I2C3->CR2 &= ~I2C_CR2_NBYTES;	// Clear the number of bytes
	I2C3->CR2 |= (TWO_BYTES << I2C_CR2_NBYTES_Pos);		// 2 bytes to transfer

	I2C3->CR2 |= I2C_CR2_START;		// START the data transfer

	for(bytes = 0; bytes < TWO_BYTES; bytes++) {	// receive 2 bytes
		while (!(I2C3->ISR & I2C_ISR_RXNE));	// wait until all data is received
		rxBuf[bytes] = I2C3->RXDR;
	}

    // Send NACK and stop condition
    while (!(I2C3->ISR & I2C_ISR_TC));		// wait for Transfer Complete
    I2C3->CR2 |= I2C_CR2_STOP;				// issue I2C STOP

    /* shift data to fit 16-bits */
    retVal = rxBuf[1];		// upper byte
    retVal <<= 8;
    retVal |= rxBuf[0] & 0xFF;	// lower byte
    return retVal;
}


/* writes a register and an 8 bit value over I2C with sensor commands */
void I2C_write8(uint8_t periphAddr, uint8_t regAddr, uint8_t data) {
	/* packs the register address and data to be written*/
	uint8_t txBuf[2] = {(uint8_t)(SENSOR_COMMAND_BIT | regAddr), data};
	/* writes the data */
	I2C_write(periphAddr, txBuf, 2);
}

/* writes a register and 'numBytes' amount of bytes over I2C with sensor commands */

void I2C_write(uint8_t periphAddr, uint8_t *data, uint8_t numBytes) {
	uint8_t remainingBytes;
	I2C_periph_addr(periphAddr);		// set the peripheral address

	I2C3->CR2 &= ~I2C_CR2_RD_WRN;		// Controller Write Request
	I2C3->CR2 &= ~I2C_CR2_NBYTES;		// Clear the number of bytes
	I2C3->CR2 |=  (numBytes << I2C_CR2_NBYTES_Pos);		// set the number of bytes to be transfered

	I2C3->CR2 |= I2C_CR2_START;			// START the data transfer

	/* continuously write all bytes */
	for(remainingBytes = 0; remainingBytes < numBytes; remainingBytes++) {
		while(!(I2C3->ISR & I2C_ISR_TXIS));		// wait for previous transfer to complete
		I2C3->TXDR = data[remainingBytes];		// send a new byte
	}

	while (!(I2C3->ISR & I2C_ISR_TC));		// wait for Transfer Complete
	I2C3->CR2 |= I2C_CR2_STOP;				// issue I2C STOP Bit
	while (!(I2C3->ISR & I2C_ISR_STOPF));	// wait for STOP flag
	I2C3->ICR |= I2C_ICR_STOPCF;			// clear STOP flag
}

/* sets the peripheral address before transmission/reception */
void I2C_periph_addr(uint8_t addr) {
	I2C3->CR2 &= ~(0x1 << I2C_CR2_ADD10_Pos);	// 7 bit address
	I2C3->CR2 &= ~(0x3FF << I2C_CR2_SADD_Pos);	// clear SADD (peripheral addr)
	I2C3->CR2 |=  (addr << 1);					// shift peripheral addr by 1 for 7-bit addr
}



