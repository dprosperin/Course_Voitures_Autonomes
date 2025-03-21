/*
 * capteur_obstacles.h
 *
 *  Created on: Feb 14, 2025
 *      Author: davidprosperin
 */

#ifndef INC_CAPTEUR_OBSTACLES_H_
#define INC_CAPTEUR_OBSTACLES_H_

#include <stdbool.h>

#define BUFFER_DMA_RECEPTION_SIZE 1

extern uint8_t buffer_DMA_reception[BUFFER_DMA_RECEPTION_SIZE];

typedef struct {
  uint8_t id;
  uint32_t system_time;
  uint32_t dis;
  uint8_t dis_status;
  uint16_t signal_strength;
  uint8_t range_precision;
  bool checksum_pass;
} tof_parameter;

typedef enum {
	TOF_ACTIVE,
	TOF_INQUIRE
} tof_data_output_mode_t;

typedef enum {
	TOF_STATE_FRAME_HEADER,
	TOF_STATE_FUNCTION_MARK, // Puis skip un octet
	TOF_STATE_ID,
	TOF_STATE_SYSTEM_TIME, // Sur 4 octets
	TOF_STATE_DIS,  // Sur 3 octets
	TOF_STATE_DIS_STATUS,
	TOF_STATE_SIGNAL_STRENGTH,
	TOF_STATE_RANGE_PRECISION,
	TOF_STATE_SUMCHECK
} tof_automate_frame;

#define TOF_LEFT_SENSOR_ID 93
#define TOF_RIGHT_SENSOR_ID 92

HAL_StatusTypeDef capteur_obstacles_init();
void capteur_obstacles_print_frame(tof_parameter *tof0);
HAL_StatusTypeDef capteur_obstacles_send_read_frame(uint8_t id);
HAL_StatusTypeDef capteur_obstacles_set_data_output_mode(uint8_t id, tof_data_output_mode_t tof_data_output_mode);
void capteur_obstacles_automate_decode(uint8_t received_char);
uint8_t capteur_obstacles_compute_checksum(uint8_t *rx_buf, size_t rx_buf_length);

extern bool flag_decoding_frame_complete;
extern tof_parameter global_tf0;
#endif /* INC_CAPTEUR_OBSTACLES_H_ */
