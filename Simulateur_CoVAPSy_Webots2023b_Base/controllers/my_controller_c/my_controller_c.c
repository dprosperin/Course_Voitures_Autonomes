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
        /****************************************/
        /* Programme etudiant avec              */
        /*  - le tableau data_lidar_mm_main     */
        /*  - la fonction set_direction_degre(.)*/
        /*  - la fonction set_vitesse_m_s(...)  */
        /*  - la fonction recule()              */
        /****************************************/
        angle_degre = 0.02*(data_lidar_mm_main[30]-data_lidar_mm_main[150]); //distance à 60° - distance à -60°
        set_direction_degres(angle_degre);
        vitesse_m_s = 0.5;
        test(angle_degre);

        set_vitesse_m_s(vitesse_m_s);
    }
  }
  /* This is necessary to cleanup webots resources */
  wbu_driver_cleanup();
  return 0;
}