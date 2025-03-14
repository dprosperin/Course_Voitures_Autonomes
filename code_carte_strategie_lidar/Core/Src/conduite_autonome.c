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
#include "utils.h"
#include <stdio.h>

void conduite_autonome(void)
{
	//float difference  = data_lidar_mm_main[345] - data_lidar_mm_main[45] ;

	float difference  = data_lidar_mm_main[135] - data_lidar_mm_main[45];
	float angle = 0.0;
	float angle_mapped = 0.0;

	angle = kp * difference;

	angle_mapped = mapf(angle, -16, 16, ANGLE_HERKULEX_MIN, ANGLE_HERKULEX_MAX);

	set_angle(angle_mapped);
}

/**
 * @brief Callback déclenché lorsque le scan LiDAR atteint 180°.
 * @todo Tester et valider la fonction void lidar_complete_scan_callback()
 */
void lidar_complete_scan_callback()
{
	printf(">demi_tour:%lu|xy\n", HAL_GetTick());
}
