/*
 * automate.c
 *
 *  Created on: Dec 10, 2024
 *      Author: davidprosperin
 */
#include "libPixy2UART.h"

uint8_t received_byte_array[PIXY2_MAX_PAYLOAD_LENGTH] = {0};
uint8_t blue_pixel_value = 0,  green_pixel_value = 0, red_pixel_value = 0;

uint8_t color_code_number = 0,
	position_x_of_blocks = 0,
	position_y_of_blocks = 0,
	width_of_blocks = 0,
	height_of_blocks = 0,
	angle_of_color_code = 0,
	tracking_index_of_blocks = 0,
	age = 0;

void Pixy2_automate_decode(uint8_t received_byte)
{
	static uint8_t index_received_byte = 0,
			payload_length = 0,
			type_of_packet = 0;

			// Checksum variablee
	static uint16_t checksum_value = 0,
			first_byte_checksum = 0,
			byte_sum = 0;

	static Pixy2_automate_states_e automate_current_state = PIXY2_FIRST_SYNC_BYTE;

	switch (automate_current_state)
	{
	case PIXY2_FIRST_SYNC_BYTE:
		if (received_byte == 0xAF)
		{
			automate_current_state = PIXY2_SECOND_SYNC_BYTE;
		}
	break;

	case PIXY2_SECOND_SYNC_BYTE:
		if (received_byte == 0xC1)
		{
			automate_current_state = PIXY2_TYPE_OF_PACKET;
		} else
		{
			automate_current_state = PIXY2_FIRST_SYNC_BYTE;
		}
	break;

	case PIXY2_TYPE_OF_PACKET:
		type_of_packet = received_byte;
		automate_current_state = PIXY2_LENGTH_OF_PAYLOAD;
	break;

	case PIXY2_LENGTH_OF_PAYLOAD:
		payload_length = received_byte;
		automate_current_state = PIXY2_FIRST_CHECKSUM_BYTE;
	break;

	case PIXY2_FIRST_CHECKSUM_BYTE:
		first_byte_checksum = received_byte;
		automate_current_state = PIXY2_SECOND_CHECKSUM_BYTE;
	break;

	case PIXY2_SECOND_CHECKSUM_BYTE:
		checksum_value = (first_byte_checksum << 8) | received_byte;
		index_received_byte = 0;
		byte_sum = 0;
		automate_current_state = PIXY2_RECEIVE_PAYLOAD_BYTES;
	break;

	case PIXY2_RECEIVE_PAYLOAD_BYTES:
		if (index_received_byte < (payload_length - 2))
		{
			received_byte_array[index_received_byte] = received_byte;
			index_received_byte++;
			byte_sum += received_byte;
		}

		if (index_received_byte == (payload_length - 2)
				&& checksum_value == byte_sum)
		{
			 // @todo À vérifier car la documentation mentionne une taille de charge utile à 4 pour la requête getRGB
			if (type_of_packet == PIXY2_ACK_RES && payload_length == 5)
			{
				blue_pixel_value = received_byte_array[0];
				green_pixel_value = received_byte_array[1];
				red_pixel_value = received_byte_array[2];
			}

			if (type_of_packet == PIXY2_GET_BLOCKS_RES && payload_length == 16)
			{
				color_code_number = received_byte_array[1] >> 8 | received_byte_array[0];
				position_x_of_blocks = received_byte_array[3] >> 8 | received_byte_array[2];
				position_y_of_blocks = received_byte_array[5] >> 8 | received_byte_array[4];
				width_of_blocks = received_byte_array[7] >> 8 | received_byte_array[6];
				height_of_blocks = received_byte_array[9] >> 8 | received_byte_array[8];
				angle_of_color_code = received_byte_array[10] >> 8 | received_byte_array[9];
				tracking_index_of_blocks = received_byte_array[11];
				age = received_byte_array[12];
			}

			// Décodage
			automate_current_state = PIXY2_FIRST_SYNC_BYTE;
		}
	break;

	}
}

HAL_StatusTypeDef Pixy2_getVersion(void)
{
	uint8_t getVersion[4] = {0xAE, 0xC1, PIXY2_GET_VERSION_REQ, 0x0};
	return  HAL_UART_Transmit(&PIXY2_HUART, getVersion, 4, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_getResolution(void)
{
	uint8_t getResolution_request[5] = {0xAE, 0xC1, PIXY2_GET_RESOLUTION_REQ, 0x1};
	return  HAL_UART_Transmit(&PIXY2_HUART, getResolution_request, 5, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_setLED(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t setLed[7] = {0xAE, 0xC1, PIXY2_SET_LED_REQ, 0x3, red, green, blue};
	return  HAL_UART_Transmit(&PIXY2_HUART, setLed, 7, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_setCameraBrightness(uint8_t brightness)
{
	uint8_t setCameraBrightness_request[5] = {0xAE, 0xC1, PIXY2_SET_CAMERA_BRIGHTNESS_REQ, 0x1, brightness};
	return  HAL_UART_Transmit(&PIXY2_HUART, setCameraBrightness_request, 5, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_getRGB(uint16_t x, uint16_t y, bool saturate)
{
	if (x > 315 || y > 207)
		return HAL_ERROR;

	uint8_t getRGB_request[9] = {0xAE, 0xC1, PIXY2_GET_RGB_REQ, 0x5, (uint8_t) x, (uint8_t)(x >> 8), (uint8_t) y, (uint8_t) (y >> 8), saturate};

	return  HAL_UART_Transmit(&PIXY2_HUART, getRGB_request, 9, HAL_MAX_DELAY);
}

HAL_StatusTypeDef Pixy2_getBlocks(uint8_t sigmap, uint8_t maxBlocks)
{
	uint8_t getBlocks_request[6] = {0xAE, 0xC1, PIXY2_GET_BLOCKS_REQ, 2, sigmap, maxBlocks};

	return HAL_UART_Transmit(&PIXY2_HUART, getBlocks_request, 6, HAL_MAX_DELAY);
}

