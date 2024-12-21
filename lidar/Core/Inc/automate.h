/*
 * automate.h
 *
 *  Created on: Nov 14, 2024
 *      Author: davidprosperin
 */

#ifndef SRC_AUTOMATE_H_
#define SRC_AUTOMATE_H_

#include <buffer.h>

typedef enum {
     FLAG_START1,
     FLAG_START2,
     RESPONSE_DESCRIPTOR1,
	 RESPONSE_DESCRIPTOR2,
	 RESPONSE_DESCRIPTOR3,
	 RESPONSE_DESCRIPTOR4,
	 RESPONSE_DESCRIPTOR5,
     QUALITY,
     ANGLE_FIRST_PART,
     ANGLE_SECOND_PART,
     DISTANCE_FIRST_PART,
     DISTANCE_SECOND_PART
} state_automate_t;

void automate_decode(uint8_t receivedByte);

#endif /* SRC_AUTOMATE_H_ */
