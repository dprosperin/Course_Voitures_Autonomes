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
#include <stm32g4xx_hal.h>
#include <main.h>
#include <stm32g4xx_hal_def.h>

command_lidar_t command_requested = LIDAR_UNKNOWN_COMMAND;

uint8_t buffer_DMA_scan[BUFFER_DMA_SIZE] = {0};
uint8_t buffer_UART[BUFFER_UART_SIZE] = {0};
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

void lidar_decode_angle_and_distance(uint8_t *buffer, float *angle, float *distance, bool *is_first_scan_point)
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
		*is_first_scan_point = ((rplidar_measurement_data_result_response_t *)(buffer))->S;
	}

}

/**
 * @brief Décode et affiche l'état de santé du LiDAR.
 *
 * Cette fonction interprète les données du paquet de réponse "GET_HEALTH"
 * envoyé par le LiDAR et extrait les informations suivantes :
 * - L'état de santé (Good, Warning, Error).
 * - Le code d'erreur associé (si applicable).
 *
 * @param buffer Un pointeur vers le tableau contenant les données brutes du
 * paquet de réponse "GET_HEALTH". Ces données sont organisées comme suit :
 * - Byte 0-6 : En-tête et données précédentes.
 * - Byte 7+  : Contient les informations de l'état de santé.
 *
 * @note
 * Les informations de santé sont définies comme suit :
 * - **status** (1 octet) :
 *   - 0 : Good (le LiDAR fonctionne normalement).
 *   - 1 : Warning (risque potentiel détecté, mais le LiDAR fonctionne encore).
 *   - 2 : Error (le LiDAR est en mode "Protection Stop").
 * - **error_code** (2 octets) : Code d'erreur associé (pour les états Warning ou Error).
 *
 */
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
 * @brief Décode et affiche les informations générales du LiDAR.
 *
 * Cette fonction interprète les données du paquet de réponse "GET_INFO"
 * envoyé par le LiDAR et extrait les informations suivantes :
 * - Modèle du LiDAR.
 * - Version du firmware (major et minor).
 * - Version du matériel.
 * - Numéro de série unique (128 bits, imprimé en hexadécimal).
 *
 * @param buffer Un pointeur vers le tableau contenant les données brutes
 * du paquet de réponse "GET_INFO". Les données doivent respecter le format suivant :
 * - Byte 0   : Modèle du LiDAR.
 * - Byte 1-2 : Version du firmware (minor et major).
 * - Byte 3   : Version du matériel.
 * - Byte 4-19 : Numéro de série (16 octets).
 *
 * @note
 * Les informations extraites sont affichées sous un format lisible, comme suit :
 * - **model** : L'identifiant du modèle utilisé.
 * - **firmware ver** : La version du firmware, séparée en major et minor.
 * - **hardware ver** : La version du matériel.
 * - **serial number** : Une séquence hexadécimale représentant le numéro de série unique.
 */
void lidar_decode_get_info(uint8_t *buffer)
{
	uint8_t model = ((rplidar_device_info_data_response_t *)(buffer))->model;
	uint8_t firmware_minor = ((rplidar_device_info_data_response_t *)(buffer + 8))->firmware_minor;
	uint8_t firmware_major = ((rplidar_device_info_data_response_t *)(buffer + 8 * 2))->firmware_major;
	uint8_t hardware = ((rplidar_device_info_data_response_t *)(buffer + 8 * 3))->hardware;

	printf("=== Info ===\n"
			   "model : %d\n"
			   "firmware ver : %d.%d\n"
			   "hardware ver : %d\n"
			   "serial number : ", model, firmware_major, firmware_minor, hardware);

	for (size_t i = (8 * 4); i <= (8 * 19); i += 8)
	{
		if (i == (8 * 19))
		{
			printf("%X\n", buffer[i]);
		} else
		{
			printf("%X:", buffer[i]);
		}
	}

    printf("======================\n");
}

/**
 * @brief Affiche les coordonnées d'un point en format Teleplot.
 *
 * Cette fonction convertit un point défini par un angle et une distance
 * (en coordonnées polaires) en coordonnées cartésiennes (x, y), puis
 * imprime le résultat dans un format compatible avec Teleplot.
 *
 * @param angle L'angle du point en degrés (float).
 * @param distance La distance du point en mètre (float).
 *
 * Exemple de sortie pour `angle = 45` et `distance = 10` :
 * ```
 * >single_point:7.07:7.07|xy
 * ```
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
 * @brief Affiche un tableau de distances en format Teleplot.
 *
 * Cette fonction prend un tableau de distances mesurées par le LiDAR et les
 * convertit en coordonnées cartésiennes (x, y), basées sur leur index
 * correspondant à un angle. Elle imprime les résultats dans un format
 * compatible avec Teleplot pour visualisation.
 *
 * @param points Tableau des distances mesurées par le LiDAR (int16_t*).
 *               Chaque élément représente la distance pour un angle donné.
 * @param num_points_scan Nombre total de points dans un balayage LiDAR (float).
 *
 * Exemple de sortie pour un tableau de distances :
 * ```
 * >data:5.00:0.00;4.24:2.42;3.54:3.54;|xy
 * ```
 * @note Les coordonnées sont imprimées avec deux décimales de précision.
 * @note Les distances égales ou inférieures à 0 sont ignorées.
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

/**
 * @brief Gère la réception de caractères pour interpréter les commandes du LiDAR.
 *
 * Cette fonction analyse les caractères reçus via UART et détecte des commandes
 * spécifiques liées au fonctionnement du LiDAR. Lorsqu'une commande complète
 * est reçue (terminée par un caractère `\n`), elle est interprétée et la commande
 * correspondante est exécutée.
 *
 * @note
 * Les commandes reconnues sont les suivantes :
 * - **START_SCAN** : Démarre le scan normal du LiDAR.
 * - **STOP** : Arrête le LiDAR.
 * - **RESET** : Réinitialise le LiDAR.
 * - **GET_INFO** : Récupère les informations générales du LiDAR.
 * - **GET_HEALTH** : Vérifie l'état de santé du LiDAR.
 * - Toute autre commande est considérée comme inconnue et une notification est imprimée.
 *
 * La fonction utilise un buffer statique pour assembler les caractères reçus en une commande complète.
 * Une fois une commande détectée, elle est exécutée et le buffer est réinitialisé.
 *
 */
void lidar_handle_receive_character()
{
	static char message[40] = "";
	static int i = 0;

	if (flag_reception_uart2 == 1) {
		if (caractere == '\n') {
			if (strstr(message, "START_SCAN") != NULL)
			{
				command_requested = LIDAR_START_SCAN;
				printf("Demarrage du scan normal\n");
				lidar_send_start_scan();
				HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_START_SCAN);
			} else if (strstr(message, "STOP") != NULL)
			{
				command_requested = LIDAR_STOP;
				printf("Arret\n");
				lidar_send_stop();
			} else if (strstr(message, "RESET") != NULL)
			{
				command_requested = LIDAR_RESET;
				printf("Reset\n");
				lidar_send_reset();
			} else if (strstr(message, "GET_INFO") != NULL)
			{
				printf("GET_INFO\n");
				lidar_send_get_info();
			} else if (strstr(message, "GET_HEALTH") != NULL)
			{
				command_requested = LIDAR_GET_HEALTH;
				printf("GET_HEALTH\n");
				HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_GET_HEALTH);
				lidar_send_get_health();
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

/**
 * @brief Envoie une commande pour démarrer le balayage du LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande de
 * démarrage de balayage au capteur LiDAR.
 *
 * @return HAL_StatusTypeDef Le statut de la transmission UART.
 *         - HAL_OK : Transmission réussie.
 *         - HAL_ERROR : Erreur de transmission.
 *         - HAL_BUSY : Périphérique occupé.
 *         - HAL_TIMEOUT : Temps d'attente dépassé.
 */
HAL_StatusTypeDef lidar_send_start_scan(void)
{
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_START_SCAN, LIDAR_COMMAND_START_SCAN_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour arrêter le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande d'arrêt
 * au capteur LiDAR.
 *
 * @return HAL_StatusTypeDef Le statut de la transmission UART.
 *         - HAL_OK : Transmission réussie.
 *         - HAL_ERROR : Erreur de transmission.
 *         - HAL_BUSY : Périphérique occupé.
 *         - HAL_TIMEOUT : Temps d'attente dépassé.
 */
HAL_StatusTypeDef lidar_send_stop(void)
{
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_STOP, LIDAR_COMMAND_STOP_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour obtenir l'état de santé du LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande de
 * récupération des informations de santé du capteur LiDAR.
 *
 * @return HAL_StatusTypeDef Le statut de la transmission UART.
 *         - HAL_OK : Transmission réussie.
 *         - HAL_ERROR : Erreur de transmission.
 *         - HAL_BUSY : Périphérique occupé.
 *         - HAL_TIMEOUT : Temps d'attente dépassé.
 */
HAL_StatusTypeDef lidar_send_get_health(void)
{
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_HEALTH, LIDAR_COMMAND_GET_HEALTH_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour réinitialiser le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande de
 * réinitialisation au capteur LiDAR.
 *
 * @return HAL_StatusTypeDef Le statut de la transmission UART.
 *         - HAL_OK : Transmission réussie.
 *         - HAL_ERROR : Erreur de transmission.
 *         - HAL_BUSY : Périphérique occupé.
 *         - HAL_TIMEOUT : Temps d'attente dépassé.
 */
HAL_StatusTypeDef lidar_send_reset(void)
{
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_RESET, LIDAR_COMMAND_RESET_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour obtenir des informations sur le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande de
 * récupération des informations générales sur le capteur LiDAR.
 *
 * @return HAL_StatusTypeDef Le statut de la transmission UART.
 *         - HAL_OK : Transmission réussie.
 *         - HAL_ERROR : Erreur de transmission.
 *         - HAL_BUSY : Périphérique occupé.
 *         - HAL_TIMEOUT : Temps d'attente dépassé.
 */
HAL_StatusTypeDef lidar_send_get_info(void)
{
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_INFO, LIDAR_COMMAND_GET_INFO_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 90°.
 *
 */
__weak void lidar_half_complete_scan_callback()
{

}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 180°.
 *
 */
__weak void lidar_complete_scan_callback()
{

}

