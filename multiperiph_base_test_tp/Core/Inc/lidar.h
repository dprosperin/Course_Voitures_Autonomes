/*
 * lidar.h
 *
 *  Created on: Jan 23, 2025
 *      Author: davidprosperin
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include <stdint.h>
#include "stdbool.h"
#include "stm32g4xx_hal.h"

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

typedef struct {
	uint8_t model	        : 8;
	uint8_t firmware_minor  : 8;
	uint8_t firmware_major  : 8;
	uint8_t hardware        : 8;
	uint8_t serialnumber_0  : 8;
	uint8_t serialnumber_1  : 8;
	uint8_t serialnumber_2  : 8;
	uint8_t serialnumber_3  : 8;
	uint8_t serialnumber_4  : 8;
	uint8_t serialnumber_5  : 8;
	uint8_t serialnumber_6  : 8;
	uint8_t serialnumber_7  : 8;
	uint8_t serialnumber_8  : 8;
	uint8_t serialnumber_9  : 8;
	uint8_t serialnumber_10 : 8;
	uint8_t serialnumber_11 : 8;
	uint8_t serialnumber_12 : 8;
	uint8_t serialnumber_13 : 8;
	uint8_t serialnumber_14 : 8;
	uint8_t serialnumber_15 : 8;
} __attribute__((packed)) rplidar_device_info_data_response_t;

typedef enum {
	LIDAR_RESET,
	LIDAR_STOP,
	LIDAR_START_SCAN,
	LIDAR_GET_HEALTH,
	LIDAR_GET_INFO,
	LIDAR_UNKNOWN_COMMAND,
	LIDAR_SCAN_IN_PROGESS
} command_lidar_t;

#define LIDAR_COMMAND_STOP ((uint8_t*)"\xA5\x25")
#define LIDAR_COMMAND_STOP_LEN 2

#define LIDAR_COMMAND_START_SCAN ((uint8_t*)"\xA5\x20")
#define LIDAR_COMMAND_START_SCAN_LEN 2
#define LIDAR_RESPONSE_SIZE_START_SCAN 7

#define LIDAR_COMMAND_RESET ((uint8_t*)"\xA5\x40")
#define LIDAR_COMMAND_RESET_LEN 2

#define LIDAR_COMMAND_GET_INFO ((uint8_t*)"\xA5\x5")
#define LIDAR_COMMAND_GET_INFO_LEN 2
#define LIDAR_RESPONSE_SIZE_GET_INFO 20

#define LIDAR_COMMAND_GET_HEALTH ((uint8_t*)"\xA5\x52")
#define LIDAR_COMMAND_GET_HEALTH_LEN 2
#define LIDAR_RESPONSE_SIZE_GET_HEALTH 10

#define BUFFER_UART_SIZE 20
#define DATA_LIDAR_MM_MAIN_SIZE 360 + 1

#define BUFFER_DMA_SIZE 10

extern command_lidar_t command_requested;
extern uint8_t buffer_DMA_scan[BUFFER_DMA_SIZE];

extern uint8_t buffer_UART[BUFFER_UART_SIZE];
extern int16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];

void lidar_decode_angle_and_distance(uint8_t *buffer, float *angle, float *distance, bool *is_first_scan_point);
void lidar_print_single_point_teleplot_format(float angle, float distance);
void lidar_print_array_distance_teleplot_format(int16_t *points, float num_points_scan);
void lidar_handle_receive_character();
void lidar_decode_get_health(uint8_t *buffer);
void lidar_decode_get_info(uint8_t *buffer);

HAL_StatusTypeDef lidar_send_start_scan(void);
HAL_StatusTypeDef lidar_send_stop(void);
HAL_StatusTypeDef lidar_send_get_health(void);
HAL_StatusTypeDef lidar_send_reset(void);
HAL_StatusTypeDef lidar_send_get_info(void);

void lidar_half_complete_scan_callback();
void lidar_complete_scan_callback();

#endif /* INC_LIDAR_H_ */
