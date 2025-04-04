/*
 * mymain.h
 *
 *  Created on: Jan 6, 2025
 *      Author: yassine.mesbahi
 */

#ifndef INC_CONDUITE_AUTONOME_H_
#define INC_CONDUITE_AUTONOME_H_

#include <stm32g4xx_hal.h>
#include "deplacement.h"
#include "lidar.h"
#include "utils.h"
#include <stdio.h>
#include <math.h>
#include "ihm.h"
#include "test.h"

void conduite_autonome(void);
void lidar_complete_scan_callback(void);

void discontinuite() ;
void recherches_locaux();
void autonomous () ;
void clear();


#endif /* INC_CONDUITE_AUTONOME_H_ */
