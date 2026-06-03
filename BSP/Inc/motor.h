#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f4xx_hal.h"

int abs(int x);
void Load(float moto1, float moto2);
void Motor_Brake(float pwm);
void Limit(int *MotoA, int *MotoB);

#endif
