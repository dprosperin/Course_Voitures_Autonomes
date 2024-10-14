/*
 * pwm_api.h
 *
 *  Created on: Oct 14, 2024
 *      Author: joseph.munoz-saltos
 */

#ifndef INC_PWM_API_H_
#define INC_PWM_API_H_
#include "tim.h"
void PWM_write(TIM_HandleTypeDef *htim, uint32_t pwm_channel, float duty_cycle);
void PWM_dir(int dir);
#endif /* INC_PWM_API_H_ */
