/*
 * automate.c
 *
 *  Created on: Nov 14, 2024
 *      Author: davidprosperin
 */
#include <automate.h>
#include <stdio.h>
#include "stm32g4xx_hal.h"

extern UART_HandleTypeDef huart1;


void automate_scan_decode(uint8_t receivedByte)
{
    static state_automate_scan_t next_state = SCAN_FLAG_START1;
    static state_automate_scan_t current_state;

    static uint8_t quality = 0;
    uint8_t constant_bit = 0;
    static uint8_t s = 0;
    uint8_t not_s = 0;

    static uint8_t distance_low_byte = 0;
    static uint8_t distance_high_byte = 0;

    static uint8_t angle_low_byte = 0;
    static uint8_t angle_high_byte = 0;

    static float distance = 0;
    static float angle = 0;

    current_state = next_state;

    switch (current_state)
    {
    case SCAN_FLAG_START1 :
    	if (receivedByte == 0xA5)
        {
             next_state = SCAN_FLAG_START2;
        }
    break;

    case SCAN_FLAG_START2:
        if (receivedByte == 0x5A)
        {
            next_state = SCAN_RESPONSE_DESCRIPTOR1;
        } else {
        	next_state = SCAN_FLAG_START1;
        }
    break;

    case SCAN_RESPONSE_DESCRIPTOR1:
    	if (receivedByte == 0x05)
    	{
    		next_state = SCAN_RESPONSE_DESCRIPTOR2;
    	}
    	break;
    case SCAN_RESPONSE_DESCRIPTOR2:
    	if (receivedByte == 0x00)
    	{
    		next_state = SCAN_RESPONSE_DESCRIPTOR3;
    	}
    	break;

    case SCAN_RESPONSE_DESCRIPTOR3:
        if (receivedByte == 0x00)
        {
        	next_state = SCAN_RESPONSE_DESCRIPTOR4;
        }
        break;
    case SCAN_RESPONSE_DESCRIPTOR4:
    	if (receivedByte == 0x40)
    	{
    	    next_state = SCAN_RESPONSE_DESCRIPTOR5;
    	}
    	break;

    case SCAN_RESPONSE_DESCRIPTOR5:
        if (receivedByte == 0x81)
        {
        	printf("Response descriptor correctement lu debut a %d\n", index_read);
        	next_state = SCAN_QUALITY;
        }
        break;

    case SCAN_QUALITY:
    		quality = receivedByte >> 2;
    		not_s = (receivedByte >> 1) & 1;
    		s = receivedByte & 1;

            if ((!not_s) == s)
            {
            	next_state = SCAN_ANGLE_FIRST_PART;

            } else {
            	next_state = SCAN_QUALITY;
            }
    break;

    case SCAN_ANGLE_FIRST_PART:
    	constant_bit = receivedByte & 0b1;
    	angle_low_byte = receivedByte;

        if (constant_bit)
        {
            next_state = SCAN_ANGLE_SECOND_PART;
        } else {
        	next_state = SCAN_QUALITY;
        }
    break;

    case SCAN_ANGLE_SECOND_PART:
    	angle_high_byte = receivedByte;

    	angle = (((uint16_t)(angle_high_byte) << 7) | ((uint16_t)(angle_low_byte) & 0x00FF)) / 64.0;

    	next_state = SCAN_DISTANCE_FIRST_PART;
    break;

    case SCAN_DISTANCE_FIRST_PART:
    	distance_low_byte = receivedByte;

    	next_state = SCAN_DISTANCE_SECOND_PART;

    break;

    case SCAN_DISTANCE_SECOND_PART:
    	distance_high_byte = receivedByte;

    	distance = ((((uint16_t) distance_high_byte << 8) & 0xFF00 ) | ((uint16_t) distance_low_byte & 0x00FF)) / 4.0;

    	if (distance > 0)
    	{
    		ajout_mesure(angle, distance);
    		printf("(%4.3f, %4.3f)\n", angle, distance);
    	}
        next_state = SCAN_QUALITY;
    break;
    }
}


void automate_health_decode(uint8_t receivedByte)
{
    static state_automate_health_t next_state = HEALTH_FLAG_START1;
    static state_automate_health_t current_state;

    static uint8_t status = 0;
    static uint8_t error_code_low_byte = 0;
    static uint8_t error_code_high_byte = 0;
    static uint16_t error_code = 0;

    static char status_msg[16] = "";

    current_state = next_state;

    switch (current_state)
    {
    case HEALTH_FLAG_START1 :
    	if (receivedByte == 0xA5)
        {
             next_state = HEALTH_FLAG_START2;
        }
    break;

    case HEALTH_FLAG_START2:
        if (receivedByte == 0x5A)
        {
            next_state = HEALTH_RESPONSE_DESCRIPTOR1;
        } else {
        	next_state = HEALTH_FLAG_START1;
        }
    break;

    case HEALTH_RESPONSE_DESCRIPTOR1:
    	if (receivedByte == 0x03)
    	{
    		next_state = HEALTH_RESPONSE_DESCRIPTOR2;
    	}
    	break;
    case HEALTH_RESPONSE_DESCRIPTOR2:
    	if (receivedByte == 0x00)
    	{
    		next_state = HEALTH_RESPONSE_DESCRIPTOR3;
    	}
    	break;

    case HEALTH_RESPONSE_DESCRIPTOR3:
        if (receivedByte == 0x00)
        {
        	next_state = HEALTH_RESPONSE_DESCRIPTOR4;
        }
        break;
    case HEALTH_RESPONSE_DESCRIPTOR4:
    	if (receivedByte == 0x00)
    	{
    	    next_state = HEALTH_RESPONSE_DESCRIPTOR5;
    	}
    	break;

    case HEALTH_RESPONSE_DESCRIPTOR5:
        if (receivedByte == 0x06)
        {
        	printf("Response descriptor correctement lu debut a %d\n", index_read);
        	next_state = HEALTH_STATUS;
        }
        break;

    case HEALTH_STATUS:
    	status = receivedByte;
    	next_state = HEALTH_ERROR_CODE0;
    break;

    case HEALTH_ERROR_CODE0:
    	error_code_low_byte = receivedByte;
    	next_state = HEALTH_ERROR_CODE1;
    break;

    case HEALTH_ERROR_CODE1:
    	error_code_high_byte = receivedByte;

    	error_code = ((((uint16_t) error_code_high_byte << 8) & 0xFF00 ) | ((uint16_t) error_code_low_byte & 0x00FF));

    	switch (status) {
			case 0:
				sprintf(status_msg, "Good");
				break;
			case 1:
				sprintf(status_msg, "Warning");
				break;
			case 2:
				sprintf(status_msg, "Error");
				break;
			default:
				sprintf(status_msg, "Unknown status");
				break;
		}

    	printf("=== Health Report ===\n"
    		   "status : %s\n"
    		   "error code : 0x%x\n"
    		   "======================\n", status_msg, error_code);

        next_state = HEALTH_FLAG_START1;
    break;
    }
}
