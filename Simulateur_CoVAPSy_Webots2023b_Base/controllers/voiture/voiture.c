#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>

#define TIME_STEP 32
#define SIZE_TABLEAU 200
#define MAX_SPEED 6.28  // Vitesse maximale des moteurs


const float* range_donnees;
//float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto=0;
// prototype des fonctions
void affichage_consigne();
//void set_direction_degres(float angle_degre);
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
void set_angle_test() ; 
//vitesse en km/h
float speed = 0;
float maxSpeed = 28; //km/h
signed int data_lidar_mm_main[360];



int main(int argc, char **argv) 
{
 gestion_appui_clavier();     
    if(modeAuto)
    { 
       set_angle_test() ; 
        /****************************************/
        /* Programme etudiant avec              */
        /*  - le tableau data_lidar_mm_main     */
        /*  - la fonction set_direction_degre(.)*/
        /*  - la fonction set_vitesse_m_s(...)  */
        /*  - la fonction recule()              */
        /****************************************/
        vitesse_m_s = 0.5;
        set_vitesse_m_s(vitesse_m_s); 
    }
  }
  /* This is necessary to cleanup webots resources */
  wbu_driver_cleanup();
  return 0;
}

unsigned char gestion_appui_clavier(void)
{
  int key;
  key=wb_keyboard_get_key();
  switch(key)
  {
    case -1:
      break;
         
    case 'n':
    case 'N':
      if (modeAuto)
      {
        modeAuto = 0;
        printf("--------Mode Auto Désactivé-------");
      }
      break;
    
    case 'a':
    case 'A':
      if (!modeAuto)
      {
        modeAuto = 1;
        printf("------------Mode Auto Activé-----------------");
         printf("c\n");
      }
      break;
      
    default:
      break; 
  }
  return key;  
}















}











































































































































