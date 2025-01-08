/*
 * mymain.c
 *
 *  Created on: Jan 6, 2025
 *      Author: yassine.mesbahi
 */
#include "fdcan.h"
#include "mymain.h"
#include <stm32g4xx_hal.h>
#include <stdbool.h>

FDCAN_TxHeaderTypeDef header;

float vitesse = 0.0;
bool  sens = 0;
float angle = 0.0;

void setup()
{

	/******************* NE PAS TOUCHER **************************************/
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;
	/*************************************************************************/

	header.Identifier = 0x700; // Set your CAN identifier
	header.IdType = FDCAN_STANDARD_ID; // Standard ID
	header.TxFrameType = FDCAN_DATA_FRAME; // Data frame
	header.DataLength = 3; // Data length

}

void loop()
{

	set_angle_test();
    HAL_Delay(1000);
}

////  Fonctions de navigation
void set_angle_test(void)
{
	 float data_lidar_main[360];

	 data_lidar_main [345] = 1405.0; // Assign float value
	 data_lidar_main[45] = 1404.0;  // Assign another float value

	 float difference  = data_lidar_main[345]-data_lidar_main [45] ;
	 float kp = 0.60 ;
	 float angle = 0.0 ;

	 angle = kp * difference ;

	set_angle(angle);
}

////  Fonctions de déplacements

/**
 * @brief
 * @param  nouvelle_vitesse : un floatant compris entre 0 et 1
 * @param  nouveau_sens : sens de rotation des roues
 * @retval None
 */
void set_vitesse_et_sens(float nouvelle_vitesse, bool nouveau_sens)
{
	uint8_t octet_nouvelle_vitesse;
	uint8_t octet_nouveau_sens;
	uint8_t txData[2];

	// On gère les cas limites
	if (nouvelle_vitesse > 1)
		vitesse = 1;
	else if (nouvelle_vitesse < 0)
		vitesse = 0;
	else
		vitesse = nouvelle_vitesse;

	octet_nouvelle_vitesse = (uint8_t) (vitesse * 100);
	octet_nouveau_sens     =  nouveau_sens;

	// Premier octet la vitesse * 100
	txData[0] = octet_nouvelle_vitesse;

	// Second octet le sens de rotation
	txData[1] = octet_nouveau_sens;

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
	set_vitesse_et_sens(vitesse, 0);
}

/**
 * @brief Fait avancer à la vitesse définit en global
 * @retval None
 */
void avance(void)
{
	set_vitesse_et_sens(vitesse, 1);
}

void set_angle(float nouvelle_angle)
{
	uint8_t octet_fort_angle;
	uint8_t octet_faible_angle;
	uint8_t txData[2];
	uint16_t cmd_angle;

	//TODO : Fixer les limites de rotation du robot

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

