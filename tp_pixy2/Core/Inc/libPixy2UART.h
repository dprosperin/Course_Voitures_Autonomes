/*
 * automate.h
 *
 *  Created on: Dec 10, 2024
 *      Author: davidprosperin
 */

#ifndef INC_LIBPIXY2UART_H_
#define INC_LIBPIXY2UART_H_

#include <stdint.h>
#include "usart.h"

#define PIXY2_HUART huart1

void automate_decode(void);
HAL_StatusTypeDef Pixy2_setLED(uint8_t red, uint8_t green, uint8_t blue);

#endif /* INC_LIBPIXY2UART_H_ */
