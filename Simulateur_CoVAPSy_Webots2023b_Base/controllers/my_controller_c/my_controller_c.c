/*
 * File:          my_controller_c.c
 * Date:          23 mai 2023
 * Description:
 * Author: Bruno Larnaudie, Anthony Juton
 * Modifications: 24 août 2023
 */
#include "my_controller_c.h"

int main(int argc, char **argv) 
{
  voiture_init();
  lidar_init();
  affichage_consigne();
  set_direction_degres(0);
  set_vitesse_m_s(0);

  while (wbu_driver_step() != -1) 
  {
    lidar_read();
    gestion_appui_clavier();

    if(modeAuto)
    {
        cherche_discontinuitee(200);

        for (unsigned int i = 0; i < TAILLE_TABLEAU_DISCONTINUITEES; i++)
        {
            if (tableau_discontinuitees[i] > 0)
              printf("(%d °, %d mm), ", i, tableau_discontinuitees[i]);
        }
         printf("\n");
        printf("nombre discontinuité : %d\n", nombre_discontinuitees);
    }
  }
  /* This is necessary to cleanup webots resources */
  wbu_driver_cleanup();
  return 0;
}