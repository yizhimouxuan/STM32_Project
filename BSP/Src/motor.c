#include "motor.h"

extern TIM_HandleTypeDef htim1;

#define MAX_PWM 1000
#define MIN_PWM -1000

static uint32_t pwm_abs_limit(float pwm)
{
	if (pwm < 0.0f)
	{
		pwm = -pwm;
	}
	if (pwm > (float)MAX_PWM)
	{
		pwm = (float)MAX_PWM;
	}
	return (uint32_t)pwm;
}

int abs(int x){
	if(x > 0){
		return x;
	}else{
		return -x;
	}
}

//取值（-1000，1000）1000对应100%占空比//取值（-1000，1000）1000对应100%占空比
void Load(float motoA, float motoB)
{
	if (motoA < 0)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_abs_limit(motoA));
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
	}
	else if (motoA > 0)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm_abs_limit(motoA));
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
	}

	if (motoB < 0)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_abs_limit(motoB));
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	}
	else if (motoB > 0)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm_abs_limit(motoB));
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
	}
}

void Motor_Brake(float pwm)
{
	uint32_t duty = pwm_abs_limit(pwm);

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, duty);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty);
}

void Limit(int *MotoA, int *MotoB){
	if(*MotoA > MAX_PWM) *MotoA = MAX_PWM;
	if(*MotoA < MIN_PWM) *MotoA = MIN_PWM;
	if(*MotoB > MAX_PWM) *MotoB = MAX_PWM;
	if(*MotoB < MIN_PWM) *MotoB = MIN_PWM;
}
