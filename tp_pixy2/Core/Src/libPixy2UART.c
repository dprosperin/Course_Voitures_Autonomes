/*
 * automate.c
 *
 *  Created on: Dec 10, 2024
 *      Author: davidprosperin
 */
#include "libPixy2UART.h"

extern uint8_t buffer_rx[1000];
extern uint8_t index_read;

void automate_decode(void)
{
    typedef enum {
        FIRST_SYNC_BYTE,
        SECOND_SYNC_BYTE,
        TYPE_OF_PACKET,
		LENGTH_OF_PAYLOAD,
		FIRST_CHECKSUM_BYTE,
		SECOND_CHECKSUM_BYTE,
		FIRST_RESULT_BYTE,
		SECOND_RESULT_BYTE,
		THIRD_RESULT_BYTE,
		FOURTH_RESULT_BYTE
    } fsm_states;


    static fsm_states etat_futur = FIRST_SYNC_BYTE; //ligne exécutée une seule fois au premier appel de robot_fsm.
    fsm_states etat_actuel;

    uint8_t first_checksum_byte,
		  second_checksum_byte;

   uint16_t checksum;

    etat_actuel = etat_futur; //on met à jour l'état actuel

    //on réfléchi (mécanique de la machine d'état)
    switch(etat_actuel){
          case FIRST_SYNC_BYTE:
                if (buffer_rx[index_read] == 0xAF)
		{
			etat_futur = SECOND_SYNC_BYTE;
		}
              break;
          case SECOND_SYNC_BYTE:
		 if (buffer_rx[index_read] == 0xC1)
		{
			etat_futur = TYPE_OF_PACKET;
		}
              break;
          case TYPE_OF_PACKET:
                 if (buffer_rx[index_read] == 0x1)
		{
			etat_futur = LENGTH_OF_PAYLOAD;
		}
          break;
	  case LENGTH_OF_PAYLOAD:
                 if (buffer_rx[index_read] == 0x4)
		{
			etat_futur = FIRST_CHECKSUM_BYTE;
		}
          break;
	case FIRST_CHECKSUM_BYTE:
		if (buffer_rx[index_read] == 0x0)
				{
			etat_futur = SECOND_CHECKSUM_BYTE;
				}

          break;
	case SECOND_CHECKSUM_BYTE:
		if (buffer_rx[index_read] == 0x0)
		{
			etat_futur = FIRST_RESULT_BYTE;
	    }
          break;

	case FIRST_RESULT_BYTE:
		if (buffer_rx[index_read] == 0x0)
		{
			etat_futur = SECOND_RESULT_BYTE;
		}
	break;

	case SECOND_RESULT_BYTE:
		if (buffer_rx[index_read] == 0x0)
		{
			etat_futur = THIRD_RESULT_BYTE;
		}
	break;

	case THIRD_RESULT_BYTE:
		if (buffer_rx[index_read] == 0x0)
		{
			etat_futur = FOURTH_RESULT_BYTE;
		}
	break;

	case FOURTH_RESULT_BYTE:
			if (buffer_rx[index_read] == 0x0)
			{
				etat_futur = FIRST_SYNC_BYTE;
			}
		break;

    }

    printf("Etat num %d\n", etat_actuel);
}

HAL_StatusTypeDef Pixy2_getVersion(void)
{
	uint8_t getVersion[4] = {0xAE, 0xC1, 0xE, 0x0};
	return  HAL_UART_Transmit(&PIXY2_HUART, getVersion, 4, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_getResolution(void)
{
	uint8_t getResolution_request[5] = {0xAE, 0xC1, 0xC, 0x1};
	return  HAL_UART_Transmit(&PIXY2_HUART, getResolution_request, 5, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_setLED(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t setLed[7] = {0xAE, 0xC1, 0x14, 0x3, red, green, blue};
	return  HAL_UART_Transmit(&PIXY2_HUART, setLed, 7, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_setCameraBrightness(uint8_t brightness)
{
	uint8_t setCameraBrightness_request[5] = {0xAE, 0xC1, 0x10, 0x1, brightness};
	return  HAL_UART_Transmit(&PIXY2_HUART, setCameraBrightness_request, 5, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_getRGB(uint16_t x, uint16_t y, bool saturate)
{
	if (x > 315 || y > 207)
		return HAL_ERROR;

	uint8_t getRGB_request[9] = {0xAE, 0xC1, 0x70, 0x5, (uint8_t) x, (uint8_t)(x >> 8), (uint8_t) y, (uint8_t) (y >> 8), saturate};

	return  HAL_UART_Transmit(&PIXY2_HUART, getRGB_request, 9, HAL_MAX_DELAY);
}
