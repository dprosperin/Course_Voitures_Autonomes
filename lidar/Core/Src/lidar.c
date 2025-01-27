/*
 * lidar.c
 *
 *  Created on: Jan 23, 2025
 *      Author: davidprosperin
 */
#include "lidar.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>

/**
 * @brief Decode l'angle et la distance
 * 
 * @param buffer Tableau de réception des valeurs reçu du LIDAR
 * @param angle Pointeur vers la variable angle
 * @param distance Pointeur vers la variable distance
 */
void lidar_decode_angle_and_distance(uint8_t *buffer, float *angle, float *distance)
{
	static uint8_t distance_low_byte = 0;
	static uint8_t distance_high_byte = 0;

	static uint8_t angle_low_byte = 0;
	static uint8_t angle_high_byte = 0;

	if (
			(((rplidar_measurement_data_result_response_t *)(buffer))->S == !((rplidar_measurement_data_result_response_t *)(buffer))->not_S)
			&& ((rplidar_measurement_data_result_response_t *)(buffer))->C
	   )
	{
		angle_low_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->angle_q6_6_0;
		angle_high_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->angle_q6_14_7;

		distance_low_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->distance_q2_7_0;
		distance_high_byte = ((rplidar_measurement_data_result_response_t *)(buffer))->distance_q2_15_8;

		*angle = (((uint16_t)(angle_high_byte) << 7) | ((uint16_t)(angle_low_byte) & 0x00FF)) / 64.0;
		*distance = ((((uint16_t) distance_high_byte << 8) & 0xFF00 ) | ((uint16_t) distance_low_byte & 0x00FF)) / 4.0;
	}

}

void lidar_print_single_point_teleplot_format(float angle, float distance)
{
	static float x = 0,
			     y = 0,
				 angle_rad = 0;

	printf(">single_point:");
	angle_rad  = angle * (M_PI / 180);
	x = cos(angle_rad) * distance;
	y = sin(angle_rad) * distance;

	printf("%.2f:%.2f|xy\n",x,y);
}

void lidar_print_array_distance_teleplot_format(int16_t *points, float num_points_scan)
{
	static float angle_rad = 0,
			x = 0,
			y = 0;

	printf(">data:");
	for(uint16_t i = 0; i < num_points_scan; i++){
		if(points[i] > 0){
			angle_rad = ((float) i * 2 * M_PI) / num_points_scan;
	        x = cos(angle_rad) * (float)points[i];
	        y = sin(angle_rad) * (float)points[i];

	        printf("%.2f:%.2f;",x,y);
		}
	}
	printf("|xy\n");
}
