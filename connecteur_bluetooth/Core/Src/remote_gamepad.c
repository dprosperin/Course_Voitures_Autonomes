/*
 * remote_gamepad.c
 *
 *  Created on: Jan 25, 2025
 *      Author: davidprosperin
 */
#include "remote_gamepad.h"
#include <stdint.h>
#include <stdio.h>
#include "fdcan.h"
#include "deplacement.h"
#include "main.h"

/*
 *	Tableau de correspondance
 * | Bouton           | Caractère reçu | Action associé                             |
 * | ---------------- | -------------- | ------------------------------------------ |
 * | Flèche du haut   | w              | Marche avant                               |
 * | Flèche du bas    | s              | Marche arrière                             |
 * | Flèche du gauche | a              | Tourner à gauche                           |
 * | Flèche du droite | d              | Tourner à gauche                           |
 * | L2               | o              | Diminuer la vitesse du moteur CC           |
 * | L1               | l              | Augmenter la vitesse du moteur CC          |
 * | Start            | n              | Démarrer du programme de conduite autonome |
 * | Select           | m              | Arret du programme de conduite autonome    |
 * | B                | v              | Arret d'urgence du véhicule                |
 * | R1               | p              | Augmenter kp                               |
 * | R2               | k              | Diminuer kp                                |
 *
 */


void handle_receive_character(uint8_t receive_character)
{
	switch (receive_character) {
	case GAMEPAD_TOP_ARROW:
		printf("Marche avant\n");
		sens = 1;
		set_rapport_cyclique_et_sens(rapport_cyclique, sens);
		break;
	case GAMEPAD_BOTTOM_ARROW:
		printf("Marche arriere\n");
		sens = 0;
		set_rapport_cyclique_et_sens(rapport_cyclique, sens);
		break;
	case GAMEPAD_LEFT_ARROW:
		printf("Tourner a gauche\n");
		break;
	case GAMEPAD_RIGHT_ARROW:
		printf("Tourner a droite\n");
		break;
	case GAMEPAD_L2:
		printf("Diminuer la vitesse du moteur CC\n");
		rapport_cyclique -= 0.1;
		set_rapport_cyclique_et_sens(rapport_cyclique, sens);
		break;
	case GAMEPAD_L1:
		printf("Augmenter la vitesse du moteur CC \n");
		rapport_cyclique += 0.1;
		set_rapport_cyclique_et_sens(rapport_cyclique, sens);
		break;
	case GAMEPAD_START:
		printf("Démarrer du programme de conduite autonome \n");
		send_start_autonomous_driving();
		break;
	case GAMEPAD_SELECT:
		printf("Arret du programme de conduite autonome \n");
		send_stop_autonomous_driving();
		break;
	case GAMEPAD_B:
		printf("Arret d'urgence du véhicule \n");
		break;
	case GAMEPAD_STOP_PRESSED:
		printf("GAMEPAD_STOP_PRESSED \n");
		break;
	case GAMEPAD_R1:
		printf("Augmenter kp\n");
		kp += 0.01;
		printf(">kp:%2.5f\n", kp);
		set_kp_value(kp);
		break;
	case GAMEPAD_R2:
		printf("Diminuer kp\n");
		kp -= 0.01;
		printf(">kp:%2.5f\n", kp);
		set_kp_value(kp);
		break;
	}
}

void send_start_autonomous_driving(void)
{
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = CAN_ID_START_AUTONOMOUS_DRIVING;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
}

void send_stop_autonomous_driving(void)
{
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = CAN_ID_STOP_AUTONOMOUS_DRIVING;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
}

void set_kp_value(float new_kp_value)
{
	uint8_t txData[2] = {0};
	uint8_t octet_faible_angle = 0;
	uint8_t octet_fort_angle = 0;
	int16_t kp_multiply_1000 = kp * 1000;

	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = CAN_ID_SET_KP_VALUE;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = 2;

	octet_faible_angle = (uint8_t) kp_multiply_1000;
	octet_fort_angle   = (kp_multiply_1000 >> 8) & 0x00FF;

	txData[0] = octet_fort_angle;
	txData[1] = octet_faible_angle;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &txData);
}
