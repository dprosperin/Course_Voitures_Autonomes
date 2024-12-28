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

extern size_t index_write;
extern size_t index_read;
extern uint8_t buffer[BUFFER_SIZE];

bool dequeue(uint8_t *value);

#endif /* INC_BUFFER_H_ */
