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
#include <webots/display.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 30
#define SIZE_TABLEAU 200
#define MAX_SPEED 6.28  // Vitesse maximale des moteurs




const float* range_donnees;
//float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto=0;
// prototype des fonctions
void affichage_consigne(); 
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
//vitesse en km/h
float speed = 0;
float maxSpeed = 28; //km/h
signed int data_lidar_mm_main[360];
typedef enum {INIT,MIN_LOCAL,MAX_LOCAL,AUGMENTER,DIMINUER} etat_discontuinuite ; 
static etat_discontuinuite etat  = INIT ; 


void reculer () ; 
void recherches_locaux() ;
void discontinuite() ;
int tab_discontuinuite [50][2] = {0} ; 


WbDeviceTag lidar;


  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
int main(int argc, char **argv) 
{ 
  unsigned int i;
  float vitesse_m_s;

  /* necessary to initialize webots stuff */
  // initialisation du conducteur de voiture
  wbu_driver_init();
  // enable keyboard
  wb_keyboard_enable(TIME_STEP);
  // enable lidar
  lidar = wb_robot_get_device("RpLidarA2");
  wb_lidar_enable(lidar, TIME_STEP);
  // affichage des points lidar sur la piste
  wb_lidar_enable_point_cloud(lidar);

  affichage_consigne();
  set_vitesse_m_s(0);

  while (wbu_driver_step() != -1) 
  {
    float distance;


    /* lire le lidar et traiter les données */
    range_donnees = wb_lidar_get_range_image(lidar);
    distance = range_donnees[0];
    if (distance > 0.0)
      data_lidar_mm_main[0] = 1000 * distance;
    else 
      data_lidar_mm_main[0] = 0;

    for (i = 1; i < 360; i++) 
    {
      distance = range_donnees[360 - i];
      if (distance > 0.0)
        data_lidar_mm_main[i] = 1000 * distance;
      else 
        data_lidar_mm_main[i] = 0;
    }

    gestion_appui_clavier();

    if (modeAuto) 
    {  
      
          recherches_locaux() ; 
          discontinuite() ; 
          vitesse_m_s = 0.0;
          set_vitesse_m_s(vitesse_m_s);
    }
  } // Fin de la boucle while

  wbu_driver_cleanup(); // Nettoyage des ressources Webots

  return 0;  // Retour correct de la fonction main
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

  void reculer () 
{
   if (data_lidar_mm_main[359]<250)
          {
          recule(); // Reculer
          wb_robot_step(2500);
          }
}


void discontinuite() {
    int distance_courante = 0;
    int distance_suivante = 0 ; 
    int diff = 0 ;
    int seuil_discontinuite = 100;  // Seuil pour considérer une discontinuité
    int cpt = 0; 
    
    for (int i = 1; i < 359; i++) 
    {
        if ((i >= 0 && i <= 90) || (i >= 270 && i <= 359)) 
        {   
            distance_courante = data_lidar_mm_main[i]; 
            distance_suivante =  data_lidar_mm_main[i + 1];

            diff = (int)fabs(distance_suivante - distance_courante);

            if (diff >= seuil_discontinuite) 
            {
              tab_discontuinuite [cpt][0]  = diff ; 
               tab_discontuinuite [cpt][1]  = i ; 
              cpt = cpt + 1 ; 
            //printf ("%d,%d\n",tab_discontuinuite [cpt][0],tab_discontuinuite [cpt][1]) ; 
            }
        }
    }
}





void recherches_locaux() 
{
    int distance_actuelle = 0;
    int distance_apres = 0;

   
    
    // Parcours des angles du lidar (de 1 à 358 pour éviter de sortir du tableau avec i+1)
    for (int i = 1; i < 359; i++) 
    {
        // Filtrage des angles souhaités (ici 0-90° et 315-359°)
        if ((i >= 0 && i <= 90) || (i >= 315 && i <= 359))
        {   
            distance_actuelle = data_lidar_mm_main[i];     
            distance_apres = data_lidar_mm_main[i+1];   
            
            switch (etat)
            { 
                case INIT:
                    if (distance_actuelle < distance_apres)
                    {
                        etat = AUGMENTER;
                    }
                    else if (distance_actuelle > distance_apres)
                    {
                        etat = DIMINUER;
                    }
                    break;

                case AUGMENTER:
                    if (distance_actuelle < distance_apres)
                    {
                        etat = AUGMENTER;
                    }
                    else if (distance_actuelle > distance_apres)
                    {
                        etat = MAX_LOCAL;
                    }
                    break;

                case DIMINUER:
                    if (distance_actuelle > distance_apres)
                    {
                        etat = DIMINUER;
                    }
                    else if (distance_actuelle < distance_apres)
                    {
                        etat = MIN_LOCAL;
                    }
                    break;

                case MIN_LOCAL:
                   // printf("Minimum local (distance = %d mm à l'angle %d°)\n", distance_actuelle, i) ; 
                    etat = INIT; 

                case MAX_LOCAL:
                   // printf("Maximum local (distance = %d mm à l'angle %d°)\n", distance_actuelle, i) ; 
                    wbu_driver_set_steering_angle(0) ; 
                    etat = INIT; // Réinitialiser pour le prochain cycle
                    break;

                default:
                    break;
            }
        }
    } 
 //   printf("Fin de tour du lidar\n");
}
