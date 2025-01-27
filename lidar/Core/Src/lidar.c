/*
 * lidar.c
 *
 *  Created on: Jan 23, 2025
 *      Author: davidprosperin
 */
#include "lidar.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"
#include <main.h>

command_lidar_t command_requested = LIDAR_UNKNOWN_COMMAND;

uint8_t buffer_scan[10] = {0};

uint8_t buffer[BUFFER_SIZE] = {0};
int16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];

/**
 * @brief Decode l'angle et la distance
 * 
 * @param buffer Tableau de réception des valeurs reçu du LIDAR
 * @param angle Pointeur vers la variable angle
 * @param distance Pointeur vers la variable distance
 * @note  on verifie si la valeur de S est bien differente de la valeur de /S pour pouvoir passer a la suite
 * @note  on va reprendre les forumles pour calculer les angles et distances de chaques points recu avec extraction des octets d'angle et distancr
 * 
 */

void lidar_decode_angle_and_distance(uint8_t *buffer, float *angle, float *distance)
{
	static uint8_t distance_low_byte = 0;
	static uint8_t distance_high_byte = 0;

	static uint8_t angle_low_byte = 0;
	static uint8_t angle_high_byte = 0;

	if (
			(((rplidar_measurement_data_result_response_t *)(buffer))->S == !((rplidar_measurement_data_result_response_t *)(buffer))->not_S)
			&& ((rplidar_measurement_data_result_response_t *)(buffer))->C
	)
	{
		angle_low_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->angle_q6_6_0;
		angle_high_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->angle_q6_14_7;

		distance_low_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->distance_q2_7_0;
		distance_high_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->distance_q2_15_8;

		*angle = (((uint16_t)(angle_high_byte) << 7) | ((uint16_t)(angle_low_byte) & 0x00FF)) / 64.0;
		*distance = ((((uint16_t) distance_high_byte << 8) & 0xFF00 ) | ((uint16_t) distance_low_byte & 0x00FF)) / 4.0;
	}

}

void lidar_decode_get_health(uint8_t *buffer)
{
	char status_msg[16] = "";

	uint8_t status = ((rplidar_device_health_data_response_t *)(buffer + 7))->status;

	uint8_t error_code_low_byte = ((rplidar_device_health_data_response_t *)(buffer + 7))->error_code_7_0;

	uint8_t error_code_high_byte = ((rplidar_device_health_data_response_t *)(buffer + 7))->error_code_15_8;

	uint16_t error_code = ((((uint16_t) error_code_high_byte << 8) & 0xFF00 ) | ((uint16_t) error_code_low_byte & 0x00FF));

	switch (status) {
	case 0:
		sprintf(status_msg, "Good");
		break;
	case 1:
		sprintf(status_msg, "Warning");
		break;
	case 2:
		sprintf(status_msg, "Error");
		break;
	default:
		sprintf(status_msg, "Unknown status");
		break;
	}
	printf("=== Health Report ===\n"
			"status : %s\n"
			"error code : 0x%x\n"
			"======================\n", status_msg, error_code);
}

/**
 * @brief Calcul des coordonnées cartésiennes (x, y) sur teleplot

 */
void lidar_print_single_point_teleplot_format(float angle, float distance)
{
	static float x = 0,
			y = 0,
			angle_rad = 0;

	printf(">single_point:");
	angle_rad  = angle * (M_PI / 180);

	x = cos(angle_rad) * distance;
	y = sin(angle_rad) * distance;

	printf("%.2f:%.2f|xy\n",x,y);
}

/**
 * @brief Calcul des coordonnées cartésiennes (x, y) sur teleplot

 */
void lidar_print_array_distance_teleplot_format(int16_t *points, float num_points_scan)
{
	static float angle_rad = 0,
			x = 0,
			y = 0;

	printf(">data:");
	for(uint16_t i = 0; i < num_points_scan; i++){
		if(points[i] > 0){
			angle_rad = ((float) i * 2 * M_PI) / num_points_scan;
			x = cos(angle_rad) * (float)points[i];
			y = sin(angle_rad) * (float)points[i];

			printf("%.2f:%.2f;",x,y);
		}
	}
	printf("|xy\n");
}


void handle_receive_character()
{
	static char message[40] = "";
	static int i = 0;

	if (flag_reception_uart2 == 1) {
		if (caractere == '\n') {
			if (strstr(message, "START_SCAN") != NULL)
			{
				command_requested = LIDAR_START_SCAN;
				printf("Demarrage du scan normal\n");
				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_START_SCAN, LIDAR_COMMAND_START_SCAN_LEN, HAL_MAX_DELAY);
				HAL_UART_Receive_IT(&LIDAR_HUART, buffer, 7);
			} else if (strstr(message, "STOP") != NULL)
			{
				command_requested = LIDAR_STOP;
				printf("Arret\n");
				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_STOP, LIDAR_COMMAND_STOP_LEN, HAL_MAX_DELAY);
			} else if (strstr(message, "RESET") != NULL)
			{
				command_requested = LIDAR_RESET;
				printf("Reset\n");
				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_RESET, LIDAR_COMMAND_RESET_LEN, HAL_MAX_DELAY);
			} else if (strstr(message, "GET_INFO") != NULL)
			{
				printf("RTFM ! <*_*>\n");
				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_INFO, LIDAR_COMMAND_GET_INFO_LEN, HAL_MAX_DELAY);
			} else if (strstr(message, "GET_HEALTH") != NULL)
			{
				command_requested = LIDAR_GET_HEALTH;
				printf("GET_HEALTH\n");
				HAL_UART_Receive_IT(&LIDAR_HUART, buffer, 10);
				HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_HEALTH, LIDAR_COMMAND_GET_HEALTH_LEN, HAL_MAX_DELAY);
			} else {
				command_requested = LIDAR_UNKNOWN_COMMAND;
				printf("Commande non reconnue : %s\n", message);
			}

			message[0] = '\0';
			i = 0;
		}

		message[i++] = caractere;
		flag_reception_uart2 = 0;

		HAL_UART_Receive_IT(&PC_HUART, &caractere, 1);
	}
}
