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
uint8_t reboot(uint8_t id);
#endif /* INC_HERCULEX_H_ */
