/*
 * buffer.c
 *
 *  Created on: Nov 28, 2024
 *      Author: davidprosperin
 */

#include <stddef.h>
#include <stdint.h>

size_t index_write = 0;
size_t index_read = 0;
uint8_t buffer[2048] = {0};

