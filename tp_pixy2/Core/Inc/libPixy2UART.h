/*
 * automate.h
 *
 *  Created on: Dec 10, 2024
 *      Author: davidprosperin
 */

#ifndef INC_LIBPIXY2UART_H_
#define INC_LIBPIXY2UART_H_

#include <stdint.h>
#include "usart.h"
#include <stdbool.h>
#include <stdio.h>

#define PIXY2_HUART huart1
#define PIXY2_MAX_PAYLOAD_LENGTH 12

typedef enum {
	PIXY2_FIRST_SYNC_BYTE,
	PIXY2_SECOND_SYNC_BYTE,
	PIXY2_TYPE_OF_PACKET,
	PIXY2_LENGTH_OF_PAYLOAD,
	PIXY2_FIRST_CHECKSUM_BYTE,
	PIXY2_SECOND_CHECKSUM_BYTE,
	PIXY2_RECEIVE_PAYLOAD_BYTES
} Pixy2_automate_states_e;

typedef enum {
	PIXY2_GET_VERSION_RES = 15,
	PIXY2_GET_RESOLUTION_RES = 13,
	PIXY2_ACK_RES = 1,
	PIXY2_GET_BLOCKS_RES = 33,
	PIXY2_GET_MAIN_FEATURES_RES = 49,
} Pixy2_type_response_e;

typedef enum {
	PIXY2_GET_VERSION_REQ = 14,
	PIXY2_GET_RESOLUTION_REQ = 12,
	PIXY2_SET_CAMERA_BRIGHTNESS_REQ = 16,
	PIXY2_SET_SERVOS_REQ = 18,
	PIXY2_SET_LED_REQ = 20,
	PIXY2_SET_LAMP_REQ = 22,
	PIXY2_GET_FPS_REQ = 24,
	PIXY2_GET_BLOCKS_REQ = 32,
	PIXY2_GET_MAIN_FEATURES_REQ = 48,
	PIXY2_SET_MODE_REQ = 54,
	PIXY2_SET_NEXT_TURN_REQ = 58,
	PIXY2_SET_DEFAULT_TURN_REQ = 60,
	PIXY2_SET_VECTOR_REQ = 56,
	PIXY2_REVERSE_VECTOR_REQ  = 62,
	PIXY2_GET_RGB_REQ = 112,
} Pixy2_type_request_e;

void Pixy2_automate_decode(uint8_t received_byte);
HAL_StatusTypeDef Pixy2_setLED(uint8_t red, uint8_t green, uint8_t blue);
HAL_StatusTypeDef Pixy2_getVersion(void);
HAL_StatusTypeDef Pixy2_getResolution(void);
HAL_StatusTypeDef Pixy2_setCameraBrightness(uint8_t brightness);
HAL_StatusTypeDef Pixy2_getRGB(uint16_t x, uint16_t y, bool saturate);

extern uint8_t received_byte_array[PIXY2_MAX_PAYLOAD_LENGTH];
extern uint8_t blue_pixel_value,  green_pixel_value, red_pixel_value;

#endif /* INC_LIBPIXY2UART_H_ */
