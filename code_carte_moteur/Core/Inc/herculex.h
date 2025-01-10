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

#define TORQUE_FREE 0x00 //
#define BREAK_ON 0x40   //Mode du torque
#define TORQUE_ON 0x60 //

uint8_t calcul_cheksum1(uint8_t size, uint8_t* trame);

uint8_t calcul_checksum2(uint8_t Csum1);

void send_trame(uint8_t id, uint8_t size, uint8_t* trame);

void read_trame(uint8_t id, uint8_t size, uint8_t* trame);

void send_reboot(uint8_t id);

void send_torque(uint8_t id, uint8_t torq);

void send_pos(uint8_t id, uint16_t pos);

uint16_t calcul_angle(double angle);

#endif /* INC_HERCULEX_H_ */
