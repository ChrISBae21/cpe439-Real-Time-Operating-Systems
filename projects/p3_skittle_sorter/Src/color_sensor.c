#include "main.h"
#include "i2c.h"
#include "color_sensor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

uint8_t sensorIntegrationTime;
uint8_t sensorGain;


/* "Calibrated" values found through sampling RGB values for each color */
int colorRedTrigger 	= 134;		/* Trigger for Red to determine Yellow, Orange, Red from Green & Purple */
int colorGreenTriggerG 	= 115;      /* Trigger for Green to determine Green */
int colorGreenTriggerP 	= 101;		/* Trigger for Green to determine Green Purple */
int colorGreenTriggerY 	= 98;       /* Trigger for Green to determine Yellow  */
int colorGreenTriggerO 	= 82;    	/* Trigger for Green to determine Orange  */


/* finds the color (Red, Green, Yellow, Orange, or Purple) based on
 * calibrated trigger-values */
uint8_t maxColor (int red, int green) {
	uint8_t color;

	/* if red is high */
	if (red >= colorRedTrigger) {

		/* if green is high */
		if (green >= colorGreenTriggerY) {
		  color = YELLOW;	// color is yellow
		}

		/* if green is low */
		else if (green <= colorGreenTriggerO) {
		  color = ORANGE;	// color is orange
		}

		/* green is medium */
		else {
		  color = RED;		// color is red
		}
	}

	/* red is low or medium */
	else {

		/* green is high */
		if (green >= colorGreenTriggerG){
		  color = GREEN;	// color is green
		}

		/* green is low */
		else if (green <= colorGreenTriggerP) {
			color = PURPLE;	// color is purple
		}

		/* green is medium */
		else {
			color = NO_COLOR;	// no color
		}
	}
	return color;
}


/* Enables the device */
void enable() {
  I2C_write8(SENSOR_ADDR, SENSOR_ENABLE, SENSOR_ENABLE_PON);
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  I2C_write8(SENSOR_ADDR, SENSOR_ENABLE, SENSOR_ENABLE_PON | SENSOR_ENABLE_AEN);
  /* Set a delay for the integration time */
  vTaskDelay(((256 - sensorIntegrationTime) * 12 / 5 + 1) / portTICK_PERIOD_MS);

}

/* disables the device (putting it in lower power sleep mode) */
void disable() {
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = I2C_read8(SENSOR_ADDR, SENSOR_ENABLE);
  I2C_write8(SENSOR_ADDR, SENSOR_ENABLE, reg & ~(SENSOR_ENABLE_PON | SENSOR_ENABLE_AEN));
}



/* initializes the sensor */
void sensorInit(uint8_t intTime, uint8_t gain) {
  setIntegrationTime(intTime);
  setGain(gain);
  enable();
}

/* sets the integration time for the sensor */
void setIntegrationTime(uint8_t it) {

  /* Update the timing register */
  I2C_write8(SENSOR_ADDR, SENSOR_ATIME, it);
  sensorIntegrationTime = it;
}

/* adjusts the gain (sensitivity to light) on the Sensor */
void setGain(uint8_t gain) {

  /* Update the timing register */
	I2C_write8(SENSOR_ADDR, SENSOR_CONTROL, gain);
	sensorGain = gain;
}

/* reads the raw red, green, blue and clear channel values */
void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {

  *c = I2C_read16(SENSOR_ADDR, SENSOR_CDATAL);
  *r = I2C_read16(SENSOR_ADDR, SENSOR_RDATAL);
  *g = I2C_read16(SENSOR_ADDR, SENSOR_GDATAL);
  *b = I2C_read16(SENSOR_ADDR, SENSOR_BDATAL);

  /* Set a delay for the integration time */
  vTaskDelay(((256 - sensorIntegrationTime) * 12 / 5 + 1) / portTICK_PERIOD_MS);
}


/* reads the raw red, green, blue and clear channel values in
 * one-shot mode (wake from sleep, take measurement, enter
 */
void getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {

  enable();
  getRawData(r, g, b, c);
  disable();
}

/* Read the RGB color detected by the sensor normalized 0-255 */
void getRGB(int *r, int *g, int *b) {
  uint16_t red, green, blue, clear;
  getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = 0;
    *g = 0;
    *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

/* determines if data is ready  */
uint8_t data_ready() {
	return (I2C_read8(SENSOR_ADDR, SENSOR_STATUS) & 0x01);
}


