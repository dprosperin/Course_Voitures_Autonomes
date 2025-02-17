/*
 * capteur_obstacles.h
 *
 *  Created on: Feb 14, 2025
 *      Author: davidprosperin
 */

#ifndef INC_CAPTEUR_OBSTACLES_H_
#define INC_CAPTEUR_OBSTACLES_H_

#include <stdbool.h>

#define BUFFER_DMA_RECEPTION_SIZE 16

extern uint8_t buffer_DMA_reception[BUFFER_DMA_RECEPTION_SIZE];

typedef struct {
  unsigned char id;
  unsigned long system_time;
  float dis;
  unsigned char dis_status;
  unsigned int signal_strength;
  unsigned char range_precision;
  bool checksum_pass;
} tof_parameter;

typedef enum {
	TOF_ACTIVE,
	TOF_INQUIRE
} tof_data_output_mode_t;

HAL_StatusTypeDef capteur_obstacles_init();
void capteur_obstacles_decode_frame(uint8_t *rx_buf, tof_parameter *tof0);
void capteur_obstacles_print_frame(tof_parameter *tof0);
uint8_t capteur_obstacles_compute_checksum(uint8_t *rx_buf, size_t rx_buf_length);
HAL_StatusTypeDef capteur_obstacles_send_read_frame(uint8_t id);
HAL_StatusTypeDef capteur_obstacles_set_data_output_mode(uint8_t id, tof_data_output_mode_t tof_data_output_mode);
#endif /* INC_CAPTEUR_OBSTACLES_H_ */
