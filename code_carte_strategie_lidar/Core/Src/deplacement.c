/*
 * deplacement.c
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#include <deplacement.h>
#include "fdcan.h"
#include <stdint.h>
#include "utils.h"

// Base motrice
float rapport_cyclique = 0.0;
bool  sens = 0;
float angle = -100;
float kp = 0.6;
float vitesse_lineaire = 1;

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
	static float angle = -100;

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

/**
 * @brief Fait touner les roues à un angle donnée dans la plage de ANGLE_ROUE_BRAQUAGE_GAUCHE à ANGLE_ROUE_BRAQUAGE_DROITE
 * @param angle_roue angle en degrée dans la plage comprise de ANGLE_ROUE_BRAQUAGE_GAUCHE à ANGLE_ROUE_BRAQUAGE_DROITE
 */
void set_angle_roue(float angle_roue)
{
	if (angle_roue < ANGLE_ROUE_BRAQUAGE_GAUCHE)
		angle_roue = ANGLE_ROUE_BRAQUAGE_GAUCHE;
	else if (angle_roue > ANGLE_ROUE_BRAQUAGE_DROITE)
		angle_roue = ANGLE_ROUE_BRAQUAGE_DROITE;

	float angle_herkulex = mapf(angle_roue, ANGLE_ROUE_BRAQUAGE_DROITE, ANGLE_ROUE_BRAQUAGE_GAUCHE, ANGLE_HERKULEX_MAX, ANGLE_HERKULEX_MIN);

	set_angle(angle_herkulex);
}


/**
 * @brief envoi la vitesse consigne linéaire
 * @param vitesse consigne en linéaire en m/s
 */
void set_consigne_vitesse(float vitesse, bool sens)
{
	uint16_t octets_vitesse_linaire_mm_par_seconde;
	uint8_t octet_sens;
	uint8_t txData[3];
	FDCAN_TxHeaderTypeDef header;

	octets_vitesse_linaire_mm_par_seconde = (uint16_t) (vitesse * 1000);
	octet_sens     =  sens;

	txData[0] = (int8_t) (octets_vitesse_linaire_mm_par_seconde >> 8);
	txData[1] =  (int8_t)(octets_vitesse_linaire_mm_par_seconde);

	// Troisième octet le sens de rotation
	txData[2] = octet_sens;

	/******************* NE PAS TOUCHER **************************************/
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;
	/*************************************************************************/

	header.Identifier = CAN_ID_VITESSE_LINEAIRE; // Set your CAN identifier
	header.IdType = FDCAN_STANDARD_ID; // Standard ID
	header.TxFrameType = FDCAN_DATA_FRAME; // Data frame
	header.DataLength = 3; // Data length

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, txData);

}
