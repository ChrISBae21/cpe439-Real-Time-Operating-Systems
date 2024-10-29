
#ifndef INC_I2C_H_
#define INC_I2C_H_

void I2C_init();
void I2C_write8(uint8_t periphAddr, uint8_t regAddr, uint8_t data);
uint8_t I2C_read8(uint8_t periphAddr, uint8_t regAddr);
uint16_t I2C_read16(uint8_t periphAddr, uint8_t regAddr);
void I2C_write(uint8_t periphAddr, uint8_t *data, uint8_t numBytes);
void I2C_periph_addr(uint8_t addr);

#endif /* INC_I2C_H_ */
