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

typedef struct {
	uint16_t Tstandard : 16;
	uint16_t Texpress  : 16;
} __attribute__((packed)) rplidar_sample_rate_data_response_t;

typedef enum {
	LIDAR_RESET,
	LIDAR_STOP,
	LIDAR_START_SCAN,
	LIDAR_GET_HEALTH,
	LIDAR_GET_INFO,
	LIDAR_GET_SAMPLERATE,
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

#define LIDAR_COMMAND_GET_INFO ((uint8_t*)"\xA5\x50")
#define LIDAR_COMMAND_GET_INFO_LEN 2
#define LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO 7
#define LIDAR_RESPONSE_SIZE_info_DATA 20
#define LIDAR_RESPONSE_SIZE_GET_INFO (LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_INFO + LIDAR_RESPONSE_SIZE_info_DATA)

#define LIDAR_COMMAND_GET_HEALTH ((uint8_t*)"\xA5\x52")
#define LIDAR_COMMAND_GET_HEALTH_LEN 2
#define LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_HEALTH 7
#define LIDAR_RESPONSE_SIZE_HEALTH_DATA 3
#define LIDAR_RESPONSE_SIZE_GET_HEALTH (LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_HEALTH + LIDAR_RESPONSE_SIZE_HEALTH_DATA)

#define LIDAR_COMMAND_GET_SAMPLERATE ((uint8_t*)"\xA5\x5A")
#define LIDAR_COMMAND_GET_SAMPLERATE_LEN 2
/**
 * @note
 * Taille du response descriptor  : 7
 * Taille de Sample Rate Data Response Packet : 4
 * La taille totale de la réponse est de 11 octets
 */
#define LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_SAMPLERATE 7
#define LIDAR_RESPONSE_SIZE_SAMPLERATE_DATA 4
#define LIDAR_RESPONSE_SIZE_GET_SAMPLERATE (LIDAR_RESPONSE_DESCRIPTOR_SIZE_GET_SAMPLERATE + LIDAR_RESPONSE_SIZE_SAMPLERATE_DATA)

#define BUFFER_UART_SIZE 20
#define DATA_LIDAR_MM_MAIN_SIZE 360 + 1

#define BUFFER_DMA_SIZE 10

extern command_lidar_t command_requested;
extern uint8_t buffer_DMA_scan[BUFFER_DMA_SIZE];

extern uint8_t buffer_UART[BUFFER_UART_SIZE];

typedef struct __attribute__((packed)){
	int16_t x;
	int16_t y;
	int16_t i;

	// ignored by can tx behind this point

	bool valid;
} lidar_point_t;

void polToCart(lidar_point_t* p, uint16_t dist, int16_t angle);

void lidar_decode_angle_and_distance(uint8_t *buffer, uint16_t *angle, uint16_t *distance, bool *is_first_scan_point);
void lidar_print_single_point_teleplot_format(float angle, float distance);
void lidar_print_array_distance_teleplot_format(int16_t *points, float num_points_scan);
void lidar_handle_receive_character();
void lidar_decode_get_health(uint8_t *buffer);
void lidar_decode_get_info(uint8_t *buffer);
void lidar_decode_get_samplerate(uint8_t *buffer);

void PolairesACartesiens(uint16_t* data_lidar_mm_main, uint16_t* angle);

void distance(int32_t x, int32_t y);

HAL_StatusTypeDef lidar_send_start_scan(void);
HAL_StatusTypeDef lidar_send_get_health(void);
HAL_StatusTypeDef lidar_send_get_info(void);
HAL_StatusTypeDef lidar_send_stop(void);
HAL_StatusTypeDef lidar_send_reset(void);
HAL_StatusTypeDef lidar_send_get_samplerate(void);

void lidar_half_complete_scan_callback();
void lidar_complete_scan_callback();

uint8_t lidar_calculate_checksum(uint8_t *buffer, size_t number_of_bytes);

#endif /* INC_LIDAR_H_ */
