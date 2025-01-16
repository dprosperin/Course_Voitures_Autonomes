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
#include <deplacement.h>


void setup()
{
}

void loop()
{

	//set_angle_test();

	set_angle(-120);
	set_rapport_cyclique_et_sens(0.2, 1);
    //HAL_Delay(1000 * 2);

    set_angle(-100);
    //HAL_Delay(1000 * 2);

    set_angle(-90);
       //HAL_Delay(1000 * 2);


    set_angle(-85);
    HAL_Delay(1000 * 1);

    //set_angle(-80.8);
    //HAL_Delay(1000 * 2);
    /*
    set_angle(90);
    HAL_Delay(1000 * 2);


    set_angle(120);
    HAL_Delay(1000 * 2);


    set_angle(0);
    HAL_Delay(1000 * 2);*/

}

//// Fonctions utilitaire
float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}

////  Fonctions de navigation
void set_angle_test(void)
{
	 float data_lidar_main[360];

	 data_lidar_main [345] = 1405.0; // Assign float value
	 data_lidar_main[45] = 1404.0;  // Assign another float value

	 float difference  = data_lidar_main[345]-data_lidar_main [45] ;
	 float kp = 0.60 ;
	 float angle = 0.0 ;

	 angle = kp * difference ;

	set_angle(angle);
}
