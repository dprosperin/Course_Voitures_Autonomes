/*
 * herculex.h
 *
 *  Created on: Nov 4, 2024
 *      Author: joseph.munoz-saltos
 */

#ifndef HERCULEX_H_
#define HERCULEX_H_


uint8_t calcul_checksum1(uint8_t size,uint8_t trame);
uint8_t calcul_checksum2(uint8_t checksum1);
void send_trame(uint8_t id,uint8_t size, uint8_t trame);
#endif /* HERCULEX_H_ */
