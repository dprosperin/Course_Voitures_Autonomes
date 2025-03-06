/*
 * utils.c
 *
 *  Created on: Feb 1, 2025
 *      Author: david
 */

#include <stdio.h>
#include "deplacement.h"

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}

void print_vitesse_moteur_teleplot()
{
	printf(">vitesse_moteur:%f|xy\n", rapport_cyclique);
}

void print_angle_herkulex_teleplot()
{
	printf(">angle_herkulex:%f|xy\n", angle);
}
