#include "herculex.h"

uint8_t reboot [7] = {0xFF, 0xFF, 0x07, 0, 0x09, 0,0}; // Trame de la fonction reboot
uint8_t torque[10] = {0xFF, 0xFF, 0x0A, 0, 0x03, 0,0, 0x34, 0x01,0}; //Trame pour la fonction send_torque
uint8_t Iposition[12] = {0xFF, 0xFF, 0x0c, 0, 0x05, 0,0, 0,0, 0x04,0 , 0x3c}; //Trame pour la fonction send_Iposition

uint16_t calcul_angle(double angle) // Calcul la valeur à envoyé en fonction d'un angle donné (+ ou - 90° de chaque côté)
{
	uint16_t valeur;

	valeur = 512 + (angle/0.326);

	return valeur;
}

uint8_t calcul_cheksum1(uint8_t size, uint8_t *trame) //Calcul une valeur à mettre dans la trame en fonction de toutes les
{													  //autres valeurs de la trame
	uint8_t checksum1=0;
	checksum1=trame[2]^trame[3]^trame[4];
	for (int i = 7; i<size; i++)
	{

			checksum1 = checksum1 ^ trame[i];
	}

	checksum1 = checksum1 & 0xFE;
	return checksum1;

}

uint8_t calcul_checksum2(uint8_t Csum1) //Pareil que checksum2
{
	uint8_t checksum2;

	checksum2 = (~Csum1) & 0xFE;
	return checksum2;
}

void send_trame(uint8_t id, uint8_t size, uint8_t *trame) //Fonction global qui façonne la trame pour l'envoyer correctement
{
	uint8_t Csum1;
	uint8_t Csum2;

	trame[3] = id;
	Csum1 = calcul_cheksum1(size, trame);
	Csum2 = calcul_checksum2(Csum1);
	trame[5] = Csum1;
	trame[6] = Csum2;

	HAL_UART_Transmit(&huart2, trame, size, HAL_MAX_DELAY);

}

void read_trame(uint8_t id, uint8_t size, uint8_t *trame) //Fonction global pour recevoir des valeurs
{
	uint8_t Csum1;
	uint8_t Csum2;
	uint8_t buffer_rx[12];

	trame[3] = id;
	Csum1 = calcul_cheksum1(size, trame);
	Csum2 = calcul_checksum2(Csum1);
	trame[5] = Csum1;
	trame[6] = Csum2;

	//HAL_UART_Receive_IT(&huart2, buffer_rx, 4);
	HAL_UART_Transmit(&huart2, trame, size, HAL_MAX_DELAY);
	for (int i = 0; i<12; i++)
	{
		HAL_UART_Receive(&huart2, &buffer_rx[i], 1, 2000);
	}

	HAL_Delay(100);
	/*IHM_LCD_clear();
	HAL_Delay(250);
	IHM_LCD_printf("pos:%x",buffer_rx[9]);
	HAL_Delay(250);*/
}

void send_reboot(uint8_t id) //Reboot les herkulex (Les autre fonction ne fonctionnent pas si il y a un reboot dans le code)
{
	send_trame(id, 7, reboot);
}

void send_torque(uint8_t id, uint8_t torq) //Permet de modifier le torque du herkulex (TORQUE_FREE, TORQUE_ON, TORQUE_OFF)
{
	torque[9] = torq;
	send_trame(id, 10, torque);
}

void send_pos(uint8_t id, uint16_t pos) //Donne une position pour un herkulex + une couleur
{																// position -> entre -90° et +90°
	Iposition[10] = id;
	Iposition[7] = pos&0x00FF;
	Iposition[8] = (pos&0xFF00) >>8;
	send_torque(id, TORQUE_ON);
	send_trame(id, 12, Iposition);
}

