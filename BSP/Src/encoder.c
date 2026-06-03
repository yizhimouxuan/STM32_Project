#include "encoder.h"

#include "tim.h"

#define TIMER_MAX 65536

int Read_Speed(TIM_HandleTypeDef *htim){
    int temp;
    temp = (short)__HAL_TIM_GetCounter(htim);
    __HAL_TIM_SetCounter(htim,0);
    return temp;
}

int16_t Encoder_GetPulseDelta(TIM_HandleTypeDef *htim, uint16_t *lastCount) {
    /* Read_Speed already returns signed pulse delta since last call. */
    (void)lastCount;
    return (int16_t)Read_Speed(htim);
}
