/*
 * lidar.h
 *
 *  Created on: Jan 23, 2025
 *      Author: davidprosperin
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include <stdint.h>

typedef struct {
	uint8_t  start_flag1          : 8;
	uint8_t  start_flag2          : 8;
	uint32_t data_response_length : 30;
	uint8_t  send_mode            : 2;
	uint8_t  data_type            : 8;
} __attribute__((packed)) rplidar_response_descriptor_t;

typedef struct {
    uint8_t S		         : 1;
    uint8_t not_S            : 1;
	uint8_t quality          : 6;

	uint8_t C                : 1;
	uint8_t angle_q6_6_0     : 7;
	uint8_t	angle_q6_14_7    : 8;

	uint8_t distance_q2_7_0  : 8;
	uint8_t distance_q2_15_8 : 8;
} __attribute__((packed)) rplidar_measurement_data_result_response_t;


typedef struct {
	uint8_t status	        : 8;
	uint8_t error_code_7_0  : 8;
	uint8_t error_code_15_8 : 8;
} __attribute__((packed)) rplidar_device_health_data_response_t;


void lidar_decode_angle_and_distance(uint8_t *buffer, float *angle, float *distance);
void lidar_print_single_point_teleplot_format(float angle, float distance);

#endif /* INC_LIDAR_H_ */
