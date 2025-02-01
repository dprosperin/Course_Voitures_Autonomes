/*
 * mymain.c
 *
 *  Created on: Jan 6, 2025
 *      Author: david.prosperin
 */
#include "mymain.h"
#include <stm32g4xx_hal.h>
#include <stdbool.h>
#include <stdio.h>
#include "deplacement.h"
#include "lidar.h"

//// Fonctions utilitaire
float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}

void conduite_autonome(void)
{
	 //float difference  = data_lidar_mm_main[345] - data_lidar_mm_main[45] ;

	 float difference  = data_lidar_mm_main[45] - data_lidar_mm_main[345];
	 float kp = 0.60 ;
	 float angle = 0.0 ;

	 angle = kp * difference ;

	set_angle(angle);
}

void print_vitesse_moteur_teleplot()
{
	printf(">vitesse_moteur:%f|xy\n", rapport_cyclique);
}

void print_angle_herkulex_teleplot()
{
	printf(">angle_herkulex:%f|xy\n", angle);
}
