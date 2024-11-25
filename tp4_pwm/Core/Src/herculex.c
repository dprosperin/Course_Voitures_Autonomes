

/*
 * herculex.c
 *
 *  Created on: Nov 4, 2024
 *      Author: joseph.munoz-saltos
 */

#include "herculex.h"

uint8_t calcul_checksum1(uint8_t size,uint8_t trame)
{
	uint8_t checksum1;
	checksum1=size ^ trame[3] ^ trame[4];
	for(int i=7;isize;i++)
	{
		checksum1=checksum1 ^ trame[i];
	}
	checksum1=checksum1 & 0xfe;
	return checksum1;
}
uint8_t calcul_checksum2(uint8_t checksum1)
{
	uint8_t checksum2;
	checksum2=(~checksum1) & 0xfe;
	return checksum2;
}
void send_trame(uint8_t id,uint8_t size, uint8_t trame)
{
	trame[3]=id;
	trame[5]=calcul_checksum1(size,trame);
	trame[6]=calcul_checksum2(trame[5]);
	HAL_UART_Transmit(&huart2,trame,size, HAL_MAX_DELAY);
}
