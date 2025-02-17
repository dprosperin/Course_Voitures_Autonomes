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
#define TOF_FUNCTION_MARK_READ_FRAME 0x10
#define TOF_SIZE_READ_FRAME 16
#define TOF_SIZE_PARAMETER 32

/**
 * @brief Démarre le DMA en mode normal pour la réception des trames.
 *
 * Cette fonction initialise la réception DMA des trames du capteur TOFSense
 * en utilisant l'UART configuré préalablement.
 *
 * @return HAL_StatusTypeDef Retourne le statut de l'initialisation DMA.
 *         HAL_OK si l'initialisation a réussi, ou un autre code d'erreur sinon.
 */
HAL_StatusTypeDef capteur_obstacles_init()
{
	return HAL_UART_Receive_DMA(&huart1, buffer_DMA_reception, BUFFER_DMA_RECEPTION_SIZE);
}

/**
 * @brief Décode une trame reçue du capteur TOFSense.
 *
 * Cette fonction analyse le buffer reçu par UART, vérifie le checksum
 * et extrait les paramètres pertinents (distance, signal, etc.).
 *
 * @param[in] rx_buf Buffer contenant les données reçues via UART.
 * @param[out] tof0 Structure dans laquelle les paramètres extraits seront stockés.
 */
void capteur_obstacles_decode_frame(uint8_t *rx_buf, tof_parameter *tof0)
{
	tof0->checksum_pass = capteur_obstacles_compute_checksum(rx_buf, TOF_SIZE_READ_FRAME) == rx_buf[15];

	if((rx_buf[0] == TOF_FRAME_HEADER)&&(rx_buf[1] == TOF_FUNCTION_MARK) && (tof0->checksum_pass))
	{
		tof0->id=rx_buf[3];
		tof0->system_time=(unsigned long)(((unsigned long)rx_buf[7])<<24|((unsigned long)rx_buf[6])<<16|((unsigned long)rx_buf[5])<<8|(unsigned long)rx_buf[4]);
		tof0->dis=((float)(((long)(((unsigned long)rx_buf[10]<<24)|((unsigned long)rx_buf[9]<<16)|((unsigned long)rx_buf[8]<<8)))/256))/1000.0;
		tof0->dis_status=rx_buf[11];
		tof0->signal_strength=(unsigned int)(((unsigned int)rx_buf[13]<<8)|(unsigned int)rx_buf[12]);
		tof0->range_precision=rx_buf[14];
	}

}

/**
 * @brief Affiche les paramètres extraits d'une trame TOFSense.
 *
 * Cette fonction imprime sur la console les différents champs décodés
 * d'une trame TOFSense.
 *
 * @param[in] tof0 Pointeur vers la structure contenant les paramètres TOFSense.
 */
void capteur_obstacles_print_frame(tof_parameter *tof0)
{
	printf("\nid: %d", tof0->id);
	printf("\nsystem_time: %lu", tof0->system_time);
	printf("\ndis: %f", tof0->dis);
	printf("\ndis_status: %d", tof0->dis_status);
	printf("\nsignal_strength: %d", tof0->signal_strength);
	printf("\nrange_precision: %d", tof0->range_precision);
	printf("\nchecksum_pass: %d", tof0->checksum_pass);
}

/**
 * @brief Calcule le checksum d'une trame TOFSense.
 *
 * Cette fonction additionne tous les octets du buffer,
 * sauf le dernier, pour calculer le checksum attendu.
 *
 * @param[in] rx_buf Buffer contenant les données reçues via UART.
 * @param[in] rx_buf_length Longueur totale du buffer rx_buf.
 * @return uint8_t Valeur calculée du checksum.
 */
uint8_t capteur_obstacles_compute_checksum(uint8_t *rx_buf, size_t rx_buf_length)
{
	uint8_t sum_byte = 0;

	for (size_t i = 0; i < (rx_buf_length - 1); i++)
	{
		sum_byte += rx_buf[i];
	}

	return sum_byte;
}

/**
 * @brief Envoie une trame de lecture au capteur TOFSense.
 *
 * Cette fonction construit une trame de lecture pour interroger un capteur
 * TOFSense avec un identifiant donné, puis l'envoie via UART.
 *
 * @param[in] id Identifiant du capteur TOFSense à interroger.
 * @return HAL_StatusTypeDef Retourne le statut de la transmission UART.
 *         HAL_OK si la transmission a réussi, ou un autre code d'erreur sinon.
 */
HAL_StatusTypeDef capteur_obstacles_send_read_frame(uint8_t id)
{
	uint8_t read_frame[TOF_SIZE_READ_FRAME] = {
			TOF_FRAME_HEADER,
			TOF_FUNCTION_MARK_READ_FRAME,
			0xFF,
			0xFF,
			id,
			0xFF,
			0xFF,
			0
	};

	read_frame[15] = capteur_obstacles_compute_checksum(read_frame, TOF_SIZE_READ_FRAME);

	return HAL_UART_Transmit(&huart1, read_frame, TOF_SIZE_READ_FRAME, HAL_MAX_DELAY);
}

/**
 * @brief Configure le mode de sortie des données du capteur TOFSense.
 *
 * Cette fonction envoie une trame pour configurer le mode de sortie des données
 * (actif ou sur requête) pour un capteur TOFSense identifié par son ID.
 *
 * @param[in] id Identifiant du capteur TOFSense à configurer.
 * @param[in] tof_data_output_mode Mode de sortie des données (énumération TOF_ACTIVE ou TOF_INQUIRE).
 * @return HAL_StatusTypeDef Retourne le statut de la transmission UART.
 *         HAL_OK si la transmission a réussi, ou un autre code d'erreur sinon.
 */
HAL_StatusTypeDef capteur_obstacles_set_data_output_mode(uint8_t id, tof_data_output_mode_t tof_data_output_mode)
{
	uint8_t write_frame[TOF_SIZE_PARAMETER] = {
			0x54,
			0x20,
			0x00,
			0xFF,
			id,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00, // Data output mode
			0xFF,
			0xFF,
			0x00,
			0x10,
			0x0E,
			0x1B,
			0x1B,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			0xFF,
			0x01,
			0x00,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0x00
	};

	if (tof_data_output_mode == TOF_ACTIVE)
	{
		write_frame[9] = 0x08;
	} else if (tof_data_output_mode == TOF_INQUIRE)
	{
		write_frame[9] = 0x0A;
	} else {
		return HAL_ERROR;
	}

	write_frame[31] = capteur_obstacles_compute_checksum(write_frame, TOF_SIZE_PARAMETER);

	return HAL_UART_Transmit(&huart1, write_frame, TOF_SIZE_PARAMETER, HAL_MAX_DELAY);
}
