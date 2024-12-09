/*
 * File:          my_controller_c.c
 * Date:          23 mai 2023
 * Description:
 * Author: Bruno Larnaudie, Anthony Juton
 * Modifications: 24 août 2023
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>
/*
 * You may want to add macros here.
 */
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

// angle max de la direction
//float maxangle_degre = 16; 

float degree_vers_radian(float degree)
{
 return degree*( 3.14/180);
}
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
int main(int argc, char **argv) 
{ 
  unsigned int i;
  
  float vitesse_m_s;
  /* necessary to initialize webots stuff */
  //intialisation du conducteur de voiture
  wbu_driver_init();
  //enable keyboard
  wb_keyboard_enable(TIME_STEP);
  // enable lidar
  WbDeviceTag lidar = wb_robot_get_device("RpLidarA2");
  wb_lidar_enable(lidar,TIME_STEP);
  // affichage des points lidar sur la piste
  wb_lidar_enable_point_cloud(lidar);
 
  affichage_consigne();
  set_vitesse_m_s(0);
  while (wbu_driver_step() != -1) 
  {
    float distance;
    /* lire le lidar et traiter les données :   */
    range_donnees=wb_lidar_get_range_image(lidar);
    distance = range_donnees[0];
    if((distance > 0.0))
      data_lidar_mm_main[0]=1000*distance;
      else data_lidar_mm_main[0] = 0;
    for(i = 1; i<360 ; i++)
    {
      distance = range_donnees[360-i];
      if((distance > 0.0))
        data_lidar_mm_main[i]=1000*distance;
      else data_lidar_mm_main[i] = 0;
    }       
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

void affichage_consigne()
{
  printf("cliquer sur la vue 3D pour commencer\n");
  printf("a pour mode auto, n pour stop\n");
}
void set_vitesse_m_s(float vitesse_m_s){
  float speed;
  speed = vitesse_m_s*3.6;
  if(speed > maxSpeed)
    speed = maxSpeed;
  if(speed < 0)
    speed = 0;
 wbu_driver_set_cruising_speed(speed);
}

void recule(void){
    wbu_driver_set_cruising_speed(-1);
}

void set_angle_test()
{   
  int max_distance_d = 0.0 ; 
  int max_distance_g = 0.0; 
  int max_distance = 0.0 ;  
  int angle_d =0.0 ;  
  int angle_g =0.0; 
  int max_angle=0.0 ; 
  float angle_voiture =0.0 ; 
 
 
 if ((data_lidar_mm_main[270]-data_lidar_mm_main[90])!=0)
 {
   for (int j = 0 ; j<91;j++)
   {
    if (data_lidar_mm_main[j]>max_distance_d)
     {
     max_distance_d=data_lidar_mm_main[j];
     angle_d = j ; 
     }
    else 
     {
      max_distance_d=max_distance_d;}
      angle_d = angle_d ; 
     }
 
    for (int k=270;k<360;k++)
    {
     if (data_lidar_mm_main[k]>max_distance_g)
     {
      max_distance_g=data_lidar_mm_main[k];
      angle_g = k ; 
     }
      else 
     { 
     max_distance_g=max_distance_g;
     angle_g = angle_g ; 
     }
   }
 
 if (max_distance_d>max_distance_g)
 {
  max_distance=max_distance_d;
  max_angle = angle_d ; 
 }
 else
 {
 max_distance=max_distance_d;
 max_angle = -angle_g ;
 }

angle_voiture = degree_vers_radian(max_angle); 

if (angle_voiture>0.35)
 {
  angle_voiture=0.35;
 }
  else
  {
   angle_voiture=angle_voiture;
  }

if (angle_voiture<-0.35) 
  { 
   angle_voiture=-0.31;
  }
else 
 {
angle_voiture=angle_voiture;
 }

} 
  
 printf("--------distance-----------\n");  
 printf("dd=%d,dg=%d,dmax=%d \n",max_distance_d,max_distance_g,max_distance);  
 printf("-----angle--------\n");  
 printf("ad=%d,ag=%d,amax=%d \n",angle_d,angle_g,max_angle);  
 printf("-----angle_voiture--------\n"); 
 printf("voiture_angle=%4.4f\n",angle_voiture);  
  
   wbu_driver_set_steering_angle(-angle_voiture);
}

