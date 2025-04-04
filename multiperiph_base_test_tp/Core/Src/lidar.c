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
uint16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];


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

void lidar_decode_angle_and_distance(uint8_t *buffer, uint16_t *angle, uint16_t *distance, bool *is_first_scan_point)
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

	uint8_t status = ((rplidar_device_health_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_HEALTH))->status;

	uint8_t error_code_low_byte = ((rplidar_device_health_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_HEALTH))->error_code_7_0;

	uint8_t error_code_high_byte = ((rplidar_device_health_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_HEALTH))->error_code_15_8;

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
 *
 * @todo Tester et valider la fonction lidar_decode_get_info(uint8_t *buffer)
 */
void lidar_decode_get_info(uint8_t *buffer)
{
	uint8_t model = ((rplidar_device_info_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO))->model;
	uint8_t firmware_minor = ((rplidar_device_info_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO))->firmware_minor;
	uint8_t firmware_major = ((rplidar_device_info_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO))->firmware_major;
	uint8_t hardware = ((rplidar_device_info_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO))->hardware;

	printf("=== Info ===\n"
			"model : %d\n"
			"firmware ver : %d.%d\n"
			"hardware ver : %d\n"
			"serial number : ", model, firmware_major, firmware_minor, hardware);

	for (size_t i = (LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO + 4); i < LIDAR_RESPONSE_SIZE_GET_INFO; i++)
	{
		if (i == LIDAR_RESPONSE_SIZE_GET_INFO - 1)
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
 * @brief Décode et affiche les durées de mesure pour les modes de scan standard et express.
 *
 * Cette fonction extrait les données du paquet de réponse "GET_SAMPLERATE" envoyé par le LiDAR
 * et affiche les durées de mesure pour chaque mode de balayage.
 *
 * @param buffer Pointeur vers le tampon contenant les données brutes du paquet de réponse.
 *
 * @details
 * - **Tstandard** : Durée nécessaire pour effectuer une mesure laser unique en mode standard.
 *   - Unité : Microsecondes (µs).
 *   - Utilisée pour calculer la vitesse de rotation du LiDAR en mode de scan standard.
 * - **Texpress** : Durée nécessaire pour effectuer une mesure laser unique en mode express.
 *   - Unité : Microsecondes (µs).
 *   - Utilisée pour calculer la vitesse de rotation du LiDAR en mode de scan express.
 *
 * Ces valeurs sont particulièrement utiles pour le débogage et l'ajustement précis
 * de la vitesse de rotation du LiDAR.
 *
 * @note
 * - Les valeurs décodées sont affichées sous un format lisible via `printf`.
 * - Le tampon `buffer` doit être correctement initialisé avec une réponse valide
 *   du LiDAR pour éviter des résultats incorrects.
 *
 * @example Exemple de sortie :
 * Si le paquet de réponse contient les valeurs Tstandard = 500 et Texpress = 250,
 * la sortie sera :
 * ```
 * === GET_SAMPLERATE ===
 * Tstandard : 500
 * Texpress  : 250
 * ======================
 * ```
 * @TODO Tester et valider la fonction lidar_decode_get_samplerate(uint8_t *buffer)
 */
void lidar_decode_get_samplerate(uint8_t *buffer)
{
	uint16_t Tstandard = ((rplidar_sample_rate_data_response_t *)(buffer + LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_SAMPLERATE))->Tstandard;
	uint16_t Texpress = ((rplidar_sample_rate_data_response_t *)(buffer+ LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_SAMPLERATE))->Texpress;

	printf("=== GET_SAMPLERATE ===\n");
	printf("Tstandard : %d\n", Tstandard);
	printf("Texpress  : %d\n", Texpress);
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

	//printf(">data:");
	for(uint16_t i = 0; i < num_points_scan; i++){
		if(points[i] > 0)

		{
			angle_rad = ((float) i * 2 * M_PI) / num_points_scan;
			x = cos(angle_rad) * (float)points[i];
			y = sin(angle_rad) * (float)points[i];

			printf("%.2f:%.2f;",x,y);
		}
	}
	//printf("|xy\n");
}

/*
void PolairesACartesiens(uint16_t* data_lidar_mm_main, uint16_t* angle)
{
	X[*angle] = ((float)data_lidar_mm_main[*angle])*cos((((float)*angle) * M_PI) / 180.0);
	Y[*angle] = ((float)data_lidar_mm_main[*angle])*sin((((float)*angle) * M_PI) / 180.0);
}
*/
void polToCart(lidar_point_t* p, uint16_t dist, int16_t angle)
{
	p->x = ((float)dist)*cos((((float)- angle) * M_PI) / 180.0);
	p->y = ((float)dist)*sin((((float)- angle) * M_PI) / 180.0);
	p->i = angle;
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
				printf("Demarrage du scan normal\n");
				memset(buffer_DMA_scan, 0, BUFFER_DMA_SIZE);
				lidar_send_start_scan();
			} else if (strstr(message, "STOP") != NULL)
			{
				printf("Arret\n");
				lidar_send_stop();
			} else if (strstr(message, "RESET") != NULL)
			{
				printf("Reset\n");
				lidar_send_reset();
			} else if (strstr(message, "GET_INFO") != NULL)
			{
				printf("GET_INFO\n");
				lidar_send_get_info();
			} else if (strstr(message, "GET_HEALTH") != NULL)
			{
				printf("GET_HEALTH\n");
				lidar_send_get_health();
			} else if (strstr(message, "GET_SAMPLERATE") != NULL)
			{
				printf("GET_SAMPLERATE\n");
				lidar_send_get_samplerate();
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
 * @todo Tester et valider la fonction lidar_send_get_samplerate(void)
 */
HAL_StatusTypeDef lidar_send_get_samplerate(void)
{
	HAL_StatusTypeDef return_get_samplerate;
	command_requested = LIDAR_GET_SAMPLERATE;
	if((return_get_samplerate =  HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_SAMPLERATE, LIDAR_COMMAND_GET_SAMPLERATE_LEN, HAL_MAX_DELAY)) == HAL_OK)
	{
		HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_GET_SAMPLERATE);
	}
	return return_get_samplerate;
}

/**
 * @brief Envoie une commande pour démarrer un scan avec le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande START_SCAN (0x20)
 * au LiDAR. Une fois la commande reçue, le LiDAR passe en mode de balayage et commence
 * à envoyer des paquets de données correspondant aux mesures réalisées.
 *
 * @details
 * - Le LiDAR entre dans l'état de balayage sauf s'il est en état de "Protection Stop".
 * - Si le LiDAR est déjà en état de balayage, il arrête le cycle de mesure en cours et
 *   démarre un nouveau balayage.
 * - Une fois la commande acceptée, un descripteur de réponse est envoyé immédiatement par le LiDAR.
 *   Les données des résultats de mesure sont envoyées en continu après stabilisation de la rotation.
 * - Chaque résultat de mesure est encapsulé dans un paquet de données avec le format suivant :
 *   - **Quality** : Qualité de l'échantillon (relié à la force du signal réfléchi).
 *   - **angle_q6** : Angle de la mesure (0-360 degrés, représenté en point fixe).
 *   - **distance_q2** : Distance mesurée par rapport au centre de rotation du LiDAR
 *     (en millimètres, représentée en point fixe).
 *
 * @return HAL_StatusTypeDef
 * - **HAL_OK** : La commande a été envoyée avec succès.
 * - **HAL_ERROR** : Une erreur de transmission s'est produite.
 * - **HAL_BUSY** : L'UART est occupé.
 * - **HAL_TIMEOUT** : La transmission a dépassé le délai maximal autorisé.
 */

HAL_StatusTypeDef lidar_send_start_scan(void)
{
	HAL_StatusTypeDef return_start_scan;
	command_requested = LIDAR_START_SCAN;
	if((return_start_scan =  HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_START_SCAN, LIDAR_COMMAND_START_SCAN_LEN, HAL_MAX_DELAY)) == HAL_OK)
	{
		HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_START_SCAN);
	}
	return return_start_scan;
}

/**
 * @brief Envoie une commande pour arrêter le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande STOP (0x25)
 * au LiDAR. Une fois cette commande reçue, le LiDAR quitte l'état de scan en cours,
 * désactive la diode laser et le système de mesure, et entre dans l'état Idle.
 *
 * @details
 * - La commande STOP est ignorée si le LiDAR est déjà dans l'état Idle ou
 *   dans l'état "Protection Stop".
 * - Aucune réponse n'est envoyée par le LiDAR suite à cette commande. Par conséquent,
 *   le système hôte doit attendre au moins **1 milliseconde** avant d'envoyer une autre commande.
 *
 * @return HAL_StatusTypeDef
 * - **HAL_OK** : La commande a été envoyée avec succès.
 * - **HAL_ERROR** : Une erreur de transmission s'est produite.
 * - **HAL_BUSY** : L'UART est occupé.
 * - **HAL_TIMEOUT** : La transmission a dépassé le délai maximal autorisé.
 */
HAL_StatusTypeDef lidar_send_stop(void)
{
	command_requested = LIDAR_STOP;
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_STOP, LIDAR_COMMAND_STOP_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour vérifier l'état de santé du LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer la commande GET_HEALTH
 * au LiDAR, permettant au système hôte de récupérer son état de santé.
 *
 * @details
 * Après réception de cette commande, le LiDAR renvoie un paquet contenant :
 * - **status** (1 octet) : Indique l'état de santé du LiDAR. Les valeurs possibles sont :
 *   - 0 : **Good** - Le LiDAR fonctionne normalement.
 *   - 1 : **Warning** - Un risque potentiel a été détecté, mais le LiDAR peut encore fonctionner.
 *   - 2 : **Error** - Le LiDAR est en mode "Protection Stop" à cause d'une panne matérielle.
 * - **error_code** (2 octets) : Contient le code d'erreur lié à l'état Warning ou Error.
 *
 * Si le LiDAR entre dans l'état "Protection Stop", il est possible d'envoyer une commande
 * RESET pour le redémarrer. Cependant, des occurrences répétées de cet état peuvent indiquer
 * une panne matérielle irréversible.
 *
 * @return HAL_StatusTypeDef
 * - **HAL_OK** : La commande a été envoyée avec succès.
 * - **HAL_ERROR** : Une erreur de transmission s'est produite.
 * - **HAL_BUSY** : L'UART est occupé.
 * - **HAL_TIMEOUT** : La transmission a dépassé le délai maximal autorisé.
 */
HAL_StatusTypeDef lidar_send_get_health(void)
{
	HAL_StatusTypeDef return_get_health;
	command_requested = LIDAR_GET_HEALTH;

	if ((return_get_health = HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_HEALTH, LIDAR_COMMAND_GET_HEALTH_LEN, HAL_MAX_DELAY)) == HAL_OK)
	{
		HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_GET_HEALTH);
	}
	return return_get_health;
}

/**
 * @brief Envoie une commande pour réinitialiser le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer une commande permettant
 * de réinitialiser le LiDAR. Cette opération force le LiDAR à redémarrer,
 * revenant à un état similaire à celui d'un démarrage après mise sous tension.
 *
 * @details
 * - Une réinitialisation est particulièrement utile lorsque le LiDAR entre dans
 *   l'état de protection ("Protection Stop state") en raison d'une erreur matérielle.
 * - Après un redémarrage, le LiDAR revient à l'état d'attente (idle state),
 *   dans lequel il peut accepter de nouvelles commandes, comme une commande de
 *   démarrage de scan.
 * - **Aucune réponse** n'est envoyée par le LiDAR suite à cette commande.
 *   Par conséquent, le système hôte doit attendre au moins 2 millisecondes avant
 *   d'envoyer une nouvelle requête.
 *
 * @return HAL_StatusTypeDef
 * - **HAL_OK** : La commande a été envoyée avec succès.
 * - **HAL_ERROR** : Une erreur de transmission s'est produite.
 * - **HAL_BUSY** : L'UART est occupé.
 * - **HAL_TIMEOUT** : La transmission a dépassé le délai maximal autorisé.
 */
HAL_StatusTypeDef lidar_send_reset(void)
{
	command_requested = LIDAR_RESET;
	return HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_RESET, LIDAR_COMMAND_RESET_LEN, HAL_MAX_DELAY);
}

/**
 * @brief Envoie une commande pour obtenir des informations générales sur le LiDAR.
 *
 * Cette fonction utilise l'interface UART pour envoyer une commande au LiDAR,
 * demandant des informations générales telles que le modèle, la version du firmware,
 * la version du matériel et le numéro de série. Une fois cette requête reçue,
 * le LiDAR envoie ses informations au système hôte.
 *
 * @return HAL_StatusTypeDef
 * - **HAL_OK** : La commande a été envoyée avec succès.
 * - **HAL_ERROR** : Une erreur de transmission s'est produite.
 * - **HAL_BUSY** : L'UART est occupé.
 * - **HAL_TIMEOUT** : La transmission a dépassé le délai maximal autorisé.
 */
HAL_StatusTypeDef lidar_send_get_info(void)
{
	HAL_StatusTypeDef return_get_info;

	command_requested = LIDAR_GET_INFO;
	if ((return_get_info = HAL_UART_Transmit(&LIDAR_HUART, LIDAR_COMMAND_GET_INFO, LIDAR_COMMAND_GET_INFO_LEN, HAL_MAX_DELAY)) == HAL_OK)
	{
		HAL_UART_Receive_IT(&LIDAR_HUART, buffer_UART, LIDAR_RESPONSE_SIZE_GET_INFO);
	}

	return return_get_info;
}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 90°.
 * @todo Tester et valider la fonction lidar_half_complete_scan_callback()
 */
__weak void lidar_half_complete_scan_callback()
{

}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 180°.
 * @todo Tester et valider la fonction void lidar_complete_scan_callback()
 */
__weak void lidar_complete_scan_callback()
{

}

/**
 * @brief Calcul la somme de controle
 * @todo gérer le cas de dépassement de tableau
 */
uint8_t lidar_calculate_checksum(uint8_t *buffer, size_t number_of_bytes)
{
	uint8_t checksum = 0;

	for (size_t i = 0; i < number_of_bytes; i++)
	{
		checksum ^= buffer[i];
	}

	return checksum;
}
