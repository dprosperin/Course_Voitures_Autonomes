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
bool flag_decoding_frame_complete = false;
tof_parameter global_tf0 = {0};

#define TOF_FRAME_HEADER 0x57
#define TOF_FUNCTION_MARK 0x00
#define TOF_FUNCTION_MARK_READ_FRAME 0x10
#define TOF_SIZE_READ_FRAME 16
#define TOF_SIZE_PARAMETER 32

/**
 * @brief Démarre la réception par interruption
 *
 * Cette fonction initialise la réception UART octet par octet.
 *
 * @return HAL_StatusTypeDef Retourne le statut de l'initialisation UART par interruption.
 *         HAL_OK si l'initialisation a réussi, ou un autre code d'erreur sinon.
 */
HAL_StatusTypeDef capteur_obstacles_init()
{
	return HAL_UART_Receive_IT(&huart1, buffer_DMA_reception, BUFFER_DMA_RECEPTION_SIZE);
}


void capteur_obstacles_automate_decode(uint8_t received_char)
{
	static uint8_t cpt_octet = 0;
    static tof_automate_frame etat_futur = TOF_STATE_FRAME_HEADER; //ligne exécutée une seule fois au premier appel de robot_fsm.
    tof_automate_frame etat_actuel;

    static tof_parameter tf0 = {0};

    static uint8_t system_time_octets[4] = {0};
    static uint8_t dis_octets[3] = {0};

    static uint8_t sum_byte = 0;

    etat_actuel = etat_futur; //on met à jour l'état actuel

    switch(etat_actuel){
          case TOF_STATE_FRAME_HEADER:
        	    sum_byte = 0;
                if (received_char == TOF_FRAME_HEADER)
                {
                	etat_futur = TOF_STATE_FUNCTION_MARK;
                	sum_byte += received_char;
                }
              break;
          case TOF_STATE_FUNCTION_MARK:
                if (received_char == TOF_FUNCTION_MARK)
                {
                	etat_futur = TOF_STATE_ID;
                	sum_byte += received_char;
                }
                else
                {
                	sum_byte = 0;
                	etat_futur = TOF_STATE_FRAME_HEADER;
                }
              break;
          case TOF_STATE_ID:
        	  sum_byte += received_char;

        	  if (cpt_octet == 0)
        	  {
        		  cpt_octet++;
        	  }
        	  else if (cpt_octet == 1)
        	  {
        		  cpt_octet = 0;
        		  tf0.id = received_char;
        		  etat_futur = TOF_STATE_SYSTEM_TIME;
        	  }
          break;

          case TOF_STATE_SYSTEM_TIME:
        	  sum_byte += received_char;
        	  system_time_octets[cpt_octet] = received_char;
        	  cpt_octet++;

        	  if (cpt_octet >= 4)
        	  {
        		  cpt_octet = 0;
        		  // Décodage de system time
        		  tf0.system_time = (uint32_t)(((uint32_t)system_time_octets[3]) << 24
        				  |((uint32_t)system_time_octets[2]) << 16 |
						  ((uint32_t)system_time_octets[1]) << 8
						  |(uint32_t)system_time_octets[0]);

        		  etat_futur = TOF_STATE_DIS;
        	  }
          break;

          case TOF_STATE_DIS:
        	  sum_byte += received_char;
        	  dis_octets[cpt_octet] = received_char;
        	  cpt_octet++;

        	  if (cpt_octet >= 3)
        	  {
        		  cpt_octet = 0;
        		  tf0.dis = (((uint32_t)dis_octets[2]<<24)
        				  | ((uint32_t)dis_octets[1] << 16)
						  | ((uint32_t)dis_octets[0]<<8));

        		  etat_futur = TOF_STATE_DIS_STATUS;
        	  }
          break;

          case TOF_STATE_DIS_STATUS:
        	  sum_byte += received_char;
        	  tf0.dis_status = received_char;

        	  etat_futur = TOF_STATE_SIGNAL_STRENGTH;
          break;

          case TOF_STATE_SIGNAL_STRENGTH:
        	  sum_byte += received_char;
        	  tf0.signal_strength = received_char;
        	  cpt_octet++;

        	  if (cpt_octet == 2)
        	  {
        		  cpt_octet = 0;
        		  etat_futur = TOF_STATE_RANGE_PRECISION;
        	  }

          break;

          case TOF_STATE_RANGE_PRECISION:
        	  sum_byte += received_char;
        	  tf0.range_precision = received_char;

        	  etat_futur = TOF_STATE_SUMCHECK;
          break;

          case TOF_STATE_SUMCHECK:
        	  tf0.checksum_pass = sum_byte == received_char;

        	  if (tf0.checksum_pass == 1)
        	  {
        		  // Fin du décodage
        		  flag_decoding_frame_complete = true;
        		  global_tf0 = tf0;
        	  }

        	  etat_futur = TOF_STATE_FRAME_HEADER;
          break;
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
	printf("\ndis: %f", tof0->dis/1000.0);
	printf("\ndis_status: %d", tof0->dis_status);
	printf("\nsignal_strength: %d", tof0->signal_strength);
	printf("\nrange_precision: %d", tof0->range_precision);
	printf("\nchecksum_pass: %d", tof0->checksum_pass);
}

void capteur_obstacles_print_frame_teleplot_format(tof_parameter *tof0)
{
	printf(">capteur_obstacles_%d:%lu|xy\n", tof0->id, tof0->dis);
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
 * @attention Le paramétrage met plusieurs secondes à prendre effet
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
