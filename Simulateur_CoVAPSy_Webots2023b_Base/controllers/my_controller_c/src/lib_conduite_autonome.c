//
// Created by David PROSPÉRIN on 29/03/2025.
//

#include "lib_conduite_autonome.h"
#include "lib_voiture.h"
#include <stdio.h>
#include <stdlib.h>

/**
* @note Use the Disparity Extender approach
* @todo 1. Find disparities in the lidar readings
* @todo 2. For each disparity, extend it half the width of the car
* @todo 3. Choose a path based on virtual lidar readings
*/

unsigned int nombre_discontinuitees = 0;
unsigned int tableau_discontinuitees[TAILLE_TABLEAU_DISCONTINUITEES] = {0};

void test(float angle_degre)
{
   printf(">direction4:%f|xy\n", angle_degre);
}

/**
* @brief Cherche les discontinuitées et les place dans le tableau discontinuitées à partir d'un seuil
* @param seuil
*/
void cherche_discontinuitee(unsigned int seuil)
{
  nombre_discontinuitees = 0;
  for (unsigned int i = 0; i < TAILLE_TABLEAU_DISCONTINUITEES; i++)
  {
    if (abs(data_lidar_mm_main[i] - data_lidar_mm_main[i + 1]) >= seuil)
    {
      tableau_discontinuitees[i] = data_lidar_mm_main[i];
      nombre_discontinuitees++;
    }
    else
    {
      tableau_discontinuitees[i] = 0;
    }
  }
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