/*
 * ihm.c
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#include "ident.h"
#include <stdarg.h>
#include <stdint.h>
#include "fdcan.h"

// IHM
uint8_t curseur = 0;
char tableau_ecran[65];


/// Fonctions IHM
void BAR_set(uint16_t motif_BAR)
{
	FDCAN_TxHeaderTypeDef header;

	/******************* NE PAS TOUCHER **************************************/
		header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		header.BitRateSwitch = FDCAN_BRS_OFF;
		header.FDFormat = FDCAN_CLASSIC_CAN;
		header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		header.MessageMarker = 0;
		/*************************************************************************/

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
    FDCAN_TxHeaderTypeDef header;

    curseur=0;

    for(i=0; i<32; i++)
        tableau_ecran[i]=32;

    /******************* NE PAS TOUCHER **************************************/
    	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    	header.BitRateSwitch = FDCAN_BRS_OFF;
    	header.FDFormat = FDCAN_CLASSIC_CAN;
    	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    	header.MessageMarker = 0;
    	/*************************************************************************/

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
	FDCAN_TxHeaderTypeDef header;
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

	     /******************* NE PAS TOUCHER **************************************/
	     	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	     	header.BitRateSwitch = FDCAN_BRS_OFF;
	     	header.FDFormat = FDCAN_CLASSIC_CAN;
	     	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	     	header.MessageMarker = 0;
	     	/*************************************************************************/

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
