#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "stm32f4xx_hal.h"

/**
 * 读取电机转速
 * @param 电机编码器对应寄存器地址
 * return 转速
 */
int Read_Speed(TIM_HandleTypeDef *htim);
int16_t Encoder_GetPulseDelta(TIM_HandleTypeDef *htim, uint16_t *lastCount);

#endif
