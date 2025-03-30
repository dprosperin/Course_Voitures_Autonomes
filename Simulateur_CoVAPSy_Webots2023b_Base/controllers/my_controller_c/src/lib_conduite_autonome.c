//
// Created by David PROSPÉRIN on 29/03/2025.
//

#include "lib_conduite_autonome.h"
#include <stdio.h>

/**
* @note Use the Disparity Extender approach
* @todo 1. Find disparities in the lidar readings
* @todo 2. For each disparity, extend it half the width of the car
* @todo 3. Choose a path based on virtual lidar readings
*/

void test(float angle_degre)
{
   printf(">direction4:%f|xy\n", angle_degre);
}

/**
*  @brief Effectue une rotation de 90° dans le sens antihoraire
*  @param angle_degre Un angle compris entre 0° et 360°
*  @return L'angle décalé de -90°
*/
int rotation_angle_90_antihoraire(int angle_degre)
{
  return mod_python(angle_degre - 90, 360);
}

/**
*  @brief Effectue une rotation de 90° dans le sens horaire
*  @param angle_degre Un angle compris entre 0° et 360°
*  @return L'angle décalé de +90°
*/
int rotation_angle_90_horaire(int angle_degre)
{
    return mod_python(angle_degre + 90, 360);
}

int mod_python(int a, int b)
{
  return ((a % b) + b) % b;
}