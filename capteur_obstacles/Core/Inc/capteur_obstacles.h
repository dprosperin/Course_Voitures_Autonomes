/*
 * capteur_obstacles.h
 *
 *  Created on: Feb 14, 2025
 *      Author: davidprosperin
 */

#ifndef INC_CAPTEUR_OBSTACLES_H_
#define INC_CAPTEUR_OBSTACLES_H_

#define BUFFER_DMA_RECEPTION_SIZE 16

extern uint8_t buffer_DMA_reception[BUFFER_DMA_RECEPTION_SIZE];

HAL_StatusTypeDef capteur_obstacles_init();
void capteur_obstacles_decode_frame(uint8_t *rx_buf);

typedef struct {
  unsigned char id;
  unsigned long system_time;
  float dis;
  unsigned char dis_status;
  unsigned int signal_strength;
  unsigned char range_precision;
} tof_parameter;

#endif /* INC_CAPTEUR_OBSTACLES_H_ */
