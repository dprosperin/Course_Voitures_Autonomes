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
#include "ident.h"
#include <stdio.h>
#include <stdarg.h>

FDCAN_TxHeaderTypeDef header;

// IHM
uint8_t curseur = 0;
char tableau_ecran[65];

// Base motrice
float rapport_cyclique = 0.0;
bool  sens = 0;
float angle = -100;

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

	//set_angle_test();

	set_angle(-120);
	set_rapport_cyclique_et_sens(0.2, 1);
    HAL_Delay(1000 * 2);



    set_angle(-100);
    HAL_Delay(1000 * 2);

    set_angle(-90);
       HAL_Delay(1000 * 2);


       set_angle(-85);
           HAL_Delay(1000 * 2);

    //set_angle(-80.8);
    //HAL_Delay(1000 * 2);
    /*
    set_angle(90);
    HAL_Delay(1000 * 2);


    set_angle(120);
    HAL_Delay(1000 * 2);


    set_angle(0);
    HAL_Delay(1000 * 2);*/

}

//// Fonctions utilitaire
float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
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
void set_rapport_cyclique_et_sens(float nouveau_rapport_cyclique, bool nouveau_sens)
{
	uint8_t octet_nouveau_rapport_cyclique;
	uint8_t octet_nouveau_sens;
	uint8_t txData[2];

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

	//Fixer les limites de rotation du robot

	if (nouvelle_angle > -80.8)
		angle = -80.8;
	else if (nouvelle_angle < -120.8)
		angle = -120.8;
	else
		angle = nouvelle_angle;

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

/// Fonctions IHM
void BAR_set(uint16_t motif_BAR) {

	header.Identifier = BAR_SET;
	header.IdType = FDCAN_STANDARD_ID;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.DataLength = 2;

    uint8_t pTxData[2] = {(uint8_t)(motif_BAR>>8), (uint8_t)(motif_BAR)};
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header , pTxData);

    // TODO : Tempo : ? HAL_Delay(1);
}

void LCD_gotoxy (uint8_t x, uint8_t y)
{
    curseur=(y*16+x)%32;
}

void LCD_clear(void)
{
    uint8_t i;

    curseur=0;

    for(i=0; i<32; i++)
        tableau_ecran[i]=32;

    header.Identifier = LCD_CLEAR;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_REMOTE_FRAME;
    header.DataLength = 0;

    uint8_t pTxData = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header , &pTxData);
    // TODO : Tempo : ? HAL_Delay(1);
}

void LCD_printf(const char* format, ...)
{
	uint8_t i, j;
	va_list arg;
	va_start(arg, format);

	curseur = curseur + vsprintf(tableau_ecran+curseur % 64, format, arg);

	if(curseur>31)
	{
	   for(i = 32; i < curseur; i++)
	   {
	        tableau_ecran[i % 32] = tableau_ecran[i];
	   }
	   curseur=curseur % 32;
	}
	else
	{
	   for(i = 0; i < 32;i++)
	   {
	      if(tableau_ecran[i] == 0)
	           tableau_ecran[i] = 20;
	   }
	 }
	 va_end(arg);
	 tableau_ecran[32] = '\0';


	 for(j = 0; j < 4; j++) {
	     uint8_t pTxData[8];

	     header.Identifier = LCD_CHAR0 + j;
	     header.IdType = FDCAN_STANDARD_ID;
	     header.TxFrameType = FDCAN_DATA_FRAME;
	     header.DataLength = 8;

	     for(i = 0; i < 8; i++)
	    	 pTxData[i] = tableau_ecran[i + j * 8];

	     HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header , pTxData);
	     // TODO : Tempo : ? HAL_Delay(1);
	 }
}
