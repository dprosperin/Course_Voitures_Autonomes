/*
 * pwm_api.c
 *
 *  Created on: Oct 14, 2024
 *      Author: joseph.munoz-saltos
 */
#include "pwm_api.h"


void PWM_write(TIM_HandleTypeDef *htim, uint32_t pwm_channel, float duty_cycle)
{
	uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t CRR = duty_cycle*(ARR+1)-1;
    __HAL_TIM_SET_COMPARE(htim,pwm_channel,CRR);
}
void PWM_dir(int dir)
{
	HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, dir);
}
