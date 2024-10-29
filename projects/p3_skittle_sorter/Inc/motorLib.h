/*
 * motorLib.h
 *
 *  Created on: May 31, 2024
 *      Author: cbgno
 */

#ifndef INC_MOTORLIB_H_
#define INC_MOTORLIB_H_

#define MOTOR_FREQ 50

void setMotorAngle(uint16_t angle);
void MOTOR1_init();
void MOTOR2_init();
void setMotor1Angle(uint16_t angle);
void setMotor2Angle(uint16_t angle);
void MOTOR_GPIO_init();
void initMotorLUT();

#endif /* INC_MOTORLIB_H_ */
