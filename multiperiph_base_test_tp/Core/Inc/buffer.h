/*
 * buffer.h
 *
 *  Created on: Nov 28, 2024
 *      Author: davidprosperin
 */

#ifndef INC_BUFFER_H_
#define INC_BUFFER_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 2048
#define DATA_LIDAR_MM_MAIN_SIZE 360 + 1

extern size_t index_write;
extern size_t index_read;
extern uint8_t buffer[BUFFER_SIZE];
extern int16_t data_lidar_mm_main[DATA_LIDAR_MM_MAIN_SIZE];

bool dequeue(uint8_t *value);

void init_data_lidar_mm_main();
bool ajout_mesure(float angle, float distance);
void print_init_data_lidar_mm_main();

#endif /* INC_BUFFER_H_ */
