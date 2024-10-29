
#ifndef INC_COLOR_SENSOR_H_
#define INC_COLOR_SENSOR_H_

/* Color definitions */
#define NUM_COLORS 5
#define PURPLE 0
#define RED 1
#define GREEN 2
#define ORANGE 3
#define YELLOW 4
#define BLUE 5
#define NO_COLOR 6

/* Sensor Configuration addresses*/
#define SENSOR_ADDR 		(0x29)  /* Sensor I2C address */
#define SENSOR_COMMAND_BIT 	(0x80) 	/* Command bit */
#define SENSOR_ENABLE 		(0x00)  /* Interrupt Enable register */
#define SENSOR_ENABLE_AEN 	(0x02) 	/* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define SENSOR_ENABLE_PON 	(0x01) 	/* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define SENSOR_ATIME 		(0x01) 	/* Integration time */

#define SENSOR_CONTROL 	(0x0F) /* Set the gain level for the sensor */
#define SENSOR_STATUS 	(0x13) /* Device status */

/* Sensor color channels */
#define SENSOR_CDATAL (0x14) /* Clear channel data low byte */
#define SENSOR_CDATAH (0x15) /* Clear channel data high byte */
#define SENSOR_RDATAL (0x16) /* Red channel data low byte */
#define SENSOR_RDATAH (0x17) /* Red channel data high byte */
#define SENSOR_GDATAL (0x18) /* Green channel data low byte */
#define SENSOR_GDATAH (0x19) /* Green channel data high byte */
#define SENSOR_BDATAL (0x1A) /* Blue channel data low byte */
#define SENSOR_BDATAH (0x1B) /* Blue channel data high byte */

/* Integration time settings for the Color Sensor */
#define SENSOR_INTEGRATIONTIME_2_4MS 	(0xFF) /* 2.4ms - 1 cycle - Max Count: 1024 */
#define SENSOR_INTEGRATIONTIME_24MS 	(0xF6) /* 24.0ms - 10 cycles - Max Count: 10240 */
#define SENSOR_INTEGRATIONTIME_50MS 	(0xEB) /* 50.4ms - 21 cycles - Max Count: 21504 */
#define SENSOR_INTEGRATIONTIME_60MS 	(0xE7) /* 60.0ms - 25 cycles - Max Count: 25700 */
#define SENSOR_INTEGRATIONTIME_101MS 	(0xD6) /* 100.8ms - 42 cycles - Max Count: 43008 */
#define SENSOR_INTEGRATIONTIME_120MS 	(0xCE) /* 120.0ms - 50 cycles - Max Count: 51200 */
#define SENSOR_INTEGRATIONTIME_154MS 	(0xC0) /* 153.6ms - 64 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_180MS 	(0xB5) /* 180.0ms - 75 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_199MS 	(0xAD) /* 199.2ms - 83 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_240MS 	(0x9C) /* 240.0ms - 100 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_300MS 	(0x83) /* 300.0ms - 125 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_360MS 	(0x6A) /* 360.0ms - 150 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_401MS 	(0x59) /* 400.8ms - 167 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_420MS 	(0x51) /* 420.0ms - 175 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_480MS 	(0x38) /* 480.0ms - 200 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_499MS 	(0x30) /* 499.2ms - 208 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_540MS 	(0x1F) /* 540.0ms - 225 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_600MS 	(0x06) /* 600.0ms - 250 cycles - Max Count: 65535 */
#define SENSOR_INTEGRATIONTIME_614MS 	(0x00) /* 614.4ms - 256 cycles - Max Count: 65535 */

/* Gain settings for TCS34725  */
#define  SENSOR_GAIN_1X  0x00  /*  No gain  */
#define  SENSOR_GAIN_4X  0x01  /*  4x gain  */
#define  SENSOR_GAIN_16X  0x02 /*  16x gain */
#define  SENSOR_GAIN_60X  0x03  /*  60x gain */

/* Placeholder reference variables */
extern uint8_t sensorIntegrationTime;
extern uint8_t sensorGain;


uint8_t maxColor (int red, int green);
void enable();
void disable();
void sensorInit(uint8_t intTime, uint8_t gain);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(int *r, int *g, int *b);
uint8_t data_ready();


#endif /* INC_COLOR_SENSOR_H_ */
