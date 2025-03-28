/*
 * pwm_api.c
 *
 *  Created on: Oct 14, 2024
 *      Author: joseph.munoz-saltos
 */
#include "pwm_api.h"

/**
 * @brief Écrit un signal PWM sur le canal spécifié.
 *
 * @param channel Le canal PWM sur lequel le signal doit être écrit.
 * @param duty_cycle Le cycle de service du signal PWM, spécifié en flotant compris entre 0 et 1.
 */

void PWM_write(TIM_HandleTypeDef *htim, uint32_t pwm_channel, float duty_cycle)
{
	uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t CRR = duty_cycle*(ARR+1)-1;
    __HAL_TIM_SET_COMPARE(htim,pwm_channel,CRR);
}

/**
 * @brief Changer le sens de rotation du moteur CC
 *
 * @param dir 0 ou 1 correspond au sens de rotation
 */

void PWM_dir(int dir)
{
	HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, dir);
}

/**
 * @brief Écrit un signal PWM sur le canal spécifié.
 *
 * @param dir 0 ou 1 correspond au sens de rotation
 * @param channel Le canal PWM sur lequel le signal doit être écrit.
 * @param duty_cycle Le cycle de service du signal PWM, spécifié en flotant compris entre 0 et 1.
 */
void PWM_dir_and_cycle(int dir,TIM_HandleTypeDef *htim, uint32_t pwm_channel, float duty_cycle)
{
	HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, dir);
	uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t CRR = duty_cycle*(ARR+1)-1;
    __HAL_TIM_SET_COMPARE(htim,pwm_channel,CRR);

}

