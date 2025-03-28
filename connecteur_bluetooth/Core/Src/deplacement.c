/*
 * deplacement.c
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#include <deplacement.h>
#include "fdcan.h"
#include <stdint.h>

// Base motrice
float rapport_cyclique = 0.0;
bool  sens = 0;
float angle = -100;


////  Fonctions de déplacements

/**
 * @brief
 * @param  nouvelle_vitesse : un floatant compris entre 0 et 1
 * @param  nouveau_sens : sens de rotation des roues
 * @retval None
 */
void set_rapport_cyclique_et_sens(float nouveau_rapport_cyclique, bool nouveau_sens)
{
	uint8_t octet_nouveau_rapport_cyclique;
	uint8_t octet_nouveau_sens;
	uint8_t txData[2];
	FDCAN_TxHeaderTypeDef header;

	// On gère les cas limites
	if (nouveau_rapport_cyclique > 1)
		rapport_cyclique = 1;
	else if (nouveau_rapport_cyclique < 0)
		rapport_cyclique = 0;
	else
		rapport_cyclique = nouveau_rapport_cyclique;

	octet_nouveau_rapport_cyclique = (uint8_t) (rapport_cyclique * 100);
	octet_nouveau_sens     =  nouveau_sens;

	// Premier octet la vitesse * 100
	txData[0] = octet_nouveau_rapport_cyclique;

	// Second octet le sens de rotation
	txData[1] = octet_nouveau_sens;

	/******************* NE PAS TOUCHER **************************************/
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;
	/*************************************************************************/

	header.Identifier = CAN_ID_MOTEUR; // Set your CAN identifier
	header.IdType = FDCAN_STANDARD_ID; // Standard ID
	header.TxFrameType = FDCAN_DATA_FRAME; // Data frame
	header.DataLength = 2; // Data length

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, txData);

	// TODO : Mettre une tempo
}

/**
 * @brief  Fait reculer à la vitesse définit en global
 * @retval None
 */
void recule(void)
{
	set_rapport_cyclique_et_sens(rapport_cyclique, 0);
}

/**
 * @brief Fait avancer à la vitesse définit en global
 * @retval None
 */
void avance(void)
{
	set_rapport_cyclique_et_sens(rapport_cyclique, 1);
}

void set_angle(float nouvelle_angle)
{
	uint8_t octet_fort_angle;
	uint8_t octet_faible_angle;
	uint8_t txData[2];
	uint16_t cmd_angle;
	FDCAN_TxHeaderTypeDef header;

	//Fixer les limites de rotation du robot

	if (nouvelle_angle > ANGLE_HERKULEX_MAX)
		angle = ANGLE_HERKULEX_MAX;
	else if (nouvelle_angle < ANGLE_HERKULEX_MIN)
		angle = ANGLE_HERKULEX_MIN;
	else
		angle = nouvelle_angle;


	/******************* NE PAS TOUCHER **************************************/
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;
	/*************************************************************************/


	header.Identifier = CAN_ID_HERKULEX; // Set your CAN identifier
	header.IdType = FDCAN_STANDARD_ID; // Standard ID
	header.TxFrameType = FDCAN_DATA_FRAME; // Data frame
	header.DataLength = 2; // Data length

	cmd_angle          = 0x200+(angle/0.35);

	octet_faible_angle = (uint8_t) cmd_angle;
	octet_fort_angle   = (cmd_angle >> 8) & 0x00FF;

	txData[0] = octet_fort_angle;
	txData[1] = octet_faible_angle;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, txData);

	// TODO : Mettre une tempo
}
