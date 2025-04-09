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
 * | B                | v              | Arret d'urgence du véhicule                |                               |
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
		send_diminuer_vitesse_moyenne();
		break;
	case GAMEPAD_L1:
		printf("Augmenter la vitesse du moteur CC \n");
		send_augmenter_vitesse_moyenne();
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

void send_augmenter_vitesse_moyenne(void)
{
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = CAN_ID_AUGMENTER_VITESSE_MOYENNE;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
}

void send_diminuer_vitesse_moyenne(void)
{
	FDCAN_TxHeaderTypeDef pTxHeader = { 0 };

	pTxHeader.Identifier = CAN_ID_DIMINUER_VITESSE_MOYENNE;
	pTxHeader.IdType = FDCAN_STANDARD_ID;
	pTxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
	pTxHeader.DataLength = 0;

	uint8_t pTxData;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, &pTxData);
}
