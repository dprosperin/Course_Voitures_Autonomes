#include <stdio.h>

#include "lidar_start_scan.h"

/**
Programme : test l'automate de décodage de trame
Author : David PROSPERIN

Compilation : clang test_automate_decode.c -o test_automate_decode && ./test_automate_decode | less
*/

// Fonction de filtre à réponse impulsionnelle infinie
float valeurFiltreeRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII){
  return valeurFiltreePrecedente  * (1-coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
}

float lidar_distance_mm[361] = {};

typedef enum {
     FLAG_START1,
     FLAG_START2,
     RESPONSE_DESCRIPTOR1,
	 RESPONSE_DESCRIPTOR2,
	 RESPONSE_DESCRIPTOR3,
	 RESPONSE_DESCRIPTOR4,
	 RESPONSE_DESCRIPTOR5,
     QUALITY,
     ANGLE_FIRST_PART,
     ANGLE_SECOND_PART,
     DISTANCE_FIRST_PART,
     DISTANCE_SECOND_PART
} state_automate_t;

void automate_decode(uint8_t receivedByte)
{
    static state_automate_t next_state = FLAG_START1;
    static state_automate_t current_state;

    uint8_t quality = 0;
    uint8_t constant_bit = 0;
    uint8_t s = 0;
    uint8_t not_s = 0;

    uint8_t distance_low_byte = 0;
    uint8_t distance_high_byte = 0;

    uint8_t angle_low_byte = 0;
    uint8_t angle_high_byte = 0;

    float distance;
    float angle;

    current_state = next_state;

    switch (current_state)
    {
    case FLAG_START1 :
    	if (receivedByte == 0xA5)
        {
             next_state = FLAG_START2;
        }
    break;

    case FLAG_START2:
        if (receivedByte == 0x5A)
        {
            next_state = RESPONSE_DESCRIPTOR1;
        } else {
        	next_state = FLAG_START1;
        }
    break;

    case RESPONSE_DESCRIPTOR1:
    	if (receivedByte == 0x05)
    	{
    		next_state = RESPONSE_DESCRIPTOR2;
    	}
    	break;
    case RESPONSE_DESCRIPTOR2:
    	if (receivedByte == 0x00)
    	{
    		next_state = RESPONSE_DESCRIPTOR3;
    	}
    	break;

    case RESPONSE_DESCRIPTOR3:
        if (receivedByte == 0x00)
        {
        	next_state = RESPONSE_DESCRIPTOR4;
        }
        break;
    case RESPONSE_DESCRIPTOR4:
    	if (receivedByte == 0x40)
    	{
    	    next_state = RESPONSE_DESCRIPTOR5;
    	}
    	break;

    case RESPONSE_DESCRIPTOR5:
        if (receivedByte == 0x81)
        {
        	//printf("Response descriptor correctement lu\n");
        	next_state = QUALITY;
        }
        break;

    case QUALITY:
    		quality = receivedByte >> 2;
    		not_s = (receivedByte >> 1) & 1;
    		s = receivedByte & 1;

            if (!not_s == s)
            {
            	//printf("Pass QUALITY : not S : %d S : %d, Quality : %d\n", not_s, s, quality);
            	next_state = ANGLE_FIRST_PART;

            } else {
            	//printf("No pass QUALITY : not S : %d S : %d, Quality : %d\n", not_s, s, quality);
            	next_state = QUALITY;
            }
    break;

    case ANGLE_FIRST_PART:
    	constant_bit = receivedByte & 0b1;
    	angle_low_byte = receivedByte;

        if (constant_bit)
        {
            next_state = ANGLE_SECOND_PART;
        } else {
        	next_state = QUALITY;
        }
    break;

    case ANGLE_SECOND_PART:
    	angle_high_byte = receivedByte;

    	angle = (((uint16_t)(angle_high_byte) << 7) | ((uint16_t)(angle_low_byte) & 0x00FF)) / 64.0;

    	next_state = DISTANCE_FIRST_PART;
    break;

    case DISTANCE_FIRST_PART:
    	distance_low_byte = receivedByte;

    	next_state = DISTANCE_SECOND_PART;

    break;

    case DISTANCE_SECOND_PART:
    	distance_high_byte = receivedByte;

    	distance = ((((uint16_t) distance_high_byte << 8) & 0xFF00 ) | ((uint16_t) distance_low_byte & 0x00FF)) / 4.0;

    	if (distance > 0)
        {
            //printf("(%f, %f)\n", angle, distance);
            angle = (int) angle;

            if (angle >= 0 && angle <= 360)
            {
                if (lidar_distance_mm[(int) angle] == -1)
                {
                    lidar_distance_mm[(int) angle] = distance;
                } else {
                    lidar_distance_mm[(int) angle] = valeurFiltreeRII(lidar_distance_mm[(int) angle], distance, 0.1);
                }
            }

        }
        next_state = QUALITY;
    break;
    }
}

int main(void)
{
    for (int i = 0; i < 361; i++)
    {
        lidar_distance_mm[i] = -1;
    }


    uint8_t buffer[] = {
        0xA5, 0x5A, 0x05,0, 0, 0x40, 0x81, 
        
        0x02, 0xED, 0x7A, 0, 0, 
        0x02, 0x09, 0x7B, 0, 0,
        0x02, 0x27, 0x7B, 0, 0,
        0x02, 0x43, 0x7B, 0, 0,
        0x02, 0x2B, 0x38, 0, 0,
        0x02, 0x65, 0x65, 0, 0,
        0x01, 0x6F, 0x23, 0,0,
        0x02, 0x2F, 0x24, 0, 0,
        0x02, 0x25, 0x23, 0, 0};
    size_t buffer_size = sizeof(buffer) / sizeof(buffer[0]);

    

    for (size_t i = 0; i < big_buffer_size; i++)
    {
        automate_decode(big_buffer[i]);
    }

    for (int i = 0; i < 361; i++)
    {
        printf("(%f,%f)\n", (float) i, lidar_distance_mm[i]);
    }

    return 0;
}