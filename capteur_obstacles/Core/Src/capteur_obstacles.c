/*
 * capteur_obstacles.c
 *
 *  Created on: Feb 14, 2025
 *      Author: davidprosperin
 */
#include "dma.h"
#include "usart.h"
#include <stdio.h>
#include "capteur_obstacles.h"
#include <stdbool.h>

uint8_t buffer_DMA_reception[BUFFER_DMA_RECEPTION_SIZE] = {0};

#define TOF_FRAME_HEADER 0x57
#define TOF_FUNCTION_MARK 0x00

/**
 * @brief Démarre le DMA en mode normal pour la reception des trames
 */
HAL_StatusTypeDef capteur_obstacles_init()
{
	//return HAL_UART_Receive_DMA(&huart1, buffer_DMA_reception, BUFFER_DMA_RECEPTION_SIZE);

	return HAL_UART_Receive_IT(&huart1, buffer_DMA_reception, BUFFER_DMA_RECEPTION_SIZE);
}

void capteur_obstacles_decode_frame(uint8_t *rx_buf)
{
	tof_parameter tof0;

	bool checksum_pass = 0;
	uint8_t sum_byte = 0;

	for (size_t i = 0; i < 15; i++)
	{
		sum_byte += rx_buf[i];
	}

	checksum_pass = sum_byte == rx_buf[15];


	if((rx_buf[0] == TOF_FRAME_HEADER)&&(rx_buf[1] == TOF_FUNCTION_MARK) && (checksum_pass))
	{
		tof0.id=rx_buf[3];
		tof0.system_time=(unsigned long)(((unsigned long)rx_buf[7])<<24|((unsigned long)rx_buf[6])<<16|((unsigned long)rx_buf[5])<<8|(unsigned long)rx_buf[4]);
		tof0.dis=((float)(((long)(((unsigned long)rx_buf[10]<<24)|((unsigned long)rx_buf[9]<<16)|((unsigned long)rx_buf[8]<<8)))/256))/1000.0;
		tof0.dis_status=rx_buf[11];
		tof0.signal_strength=(unsigned int)(((unsigned int)rx_buf[13]<<8)|(unsigned int)rx_buf[12]);
		tof0.range_precision=rx_buf[14];

		printf("id: %d", tof0.id);
		printf("\nsystem_time: %lu", tof0.system_time);
		printf("\ndis: %f", tof0.dis);
		printf("\ndis_status: %d", tof0.dis_status);
		printf("\nsignal_strength: %d", tof0.signal_strength);
		printf("\nrange_precision: %d\n", tof0.range_precision);
	}

}

