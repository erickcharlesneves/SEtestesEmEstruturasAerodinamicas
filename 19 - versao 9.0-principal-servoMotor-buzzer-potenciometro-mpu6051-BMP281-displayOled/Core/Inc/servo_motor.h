#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

#include "main.h"

void ServoMotor_Init(TIM_HandleTypeDef *htim);
void ServoMotor_Control(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

#endif // SERVO_MOTOR_H
