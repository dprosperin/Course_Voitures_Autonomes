/*
 * mymain.c
 *
 *  Created on: Jan 6, 2025
 *      Author: david.prosperin
 */
#include "conduite_autonome.h"
#include <stm32g4xx_hal.h>
#include "deplacement.h"
#include "lidar.h"

void conduite_autonome(void)
{
	 //float difference  = data_lidar_mm_main[345] - data_lidar_mm_main[45] ;

	 float difference  = data_lidar_mm_main[45] - data_lidar_mm_main[345];
	 float kp = 0.60 ;
	 float angle = 0.0 ;

	 angle = kp * difference ;

	set_angle(angle);
}
