/*
 * buffer.c
 *
 *  Created on: Dec 10, 2024
 *      Author: davidprosperin
 */
#include <stdio.h>


uint8_t buffer [100]; // note: only (N - 1) elements can be stored at a given time
uint8_t writeIndx = 0;
uint8_t readIndx  = 0;

int put (uint8_t item)
{
  if ((writeIndx + 1) % N == readIndx)
  {
     // buffer is full, avoid overflow
     return 0;
  }
  buffer[writeIndx] = item;
  writeIndx = (writeIndx + 1) % N;
  return 1;
}

int get (uint8_t * value)
{
  if (readIndx == writeIndx)
  {
     // buffer is empty
     return 0;
  }

  *value = buffer[readIndx];
  readIndx = (readIndx + 1) % N;
  return 1;
}

