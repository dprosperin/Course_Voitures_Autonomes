/*
 * herculex.h
 *
 *  Created on: Nov 25, 2024
 *      Author: joseph.munoz-saltos
 */

#ifndef INC_HERCULEX_H_
#define INC_HERCULEX_H_
#include <stdint.h>
#include "main.h"
#include "usart.h"


uint8_t calcul_checksum1(uint8_t size, uint8_t *trame);
uint8_t calcul_checksum2(uint8_t checksum1);
void send_trame(uint8_t id,uint8_t size, uint8_t *trame);
void reboot(uint8_t id);
void send_color(uint8_t id, uint8_t color);
void send_torque_on(uint8_t id);
void send_pos(uint8_t id,uint16_t pos_16b);
void send_angle(uint8_t id,float angle);
#endif /* INC_HERCULEX_H_ */
