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

void reculer () ; 

float* distance_mur () ;  
void virage () ; 
void discontinuite() ; 
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
            //distance_mur () ; 
            //virage () ; 
          //reculer () ; 
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

float* distance_mur () 
{
static float distance_au_mur [3] = {0.0} ;
  float somme_face = 0.0, somme_gauche = 0.0, somme_droite = 0.0;
  int count_face = 0, count_gauche = 0, count_droite = 0;

    for (int i = 0; i < 360; i++) 
    {
        if ((i >= 0 && i <= 45) || (i >= 315 && i < 360)) 
        {  // Face
            somme_face += data_lidar_mm_main[i];
            count_face++;
        } 
        else if (i >= 45 && i <= 90) 
        {  // Gauche
            somme_gauche += data_lidar_mm_main[i];
            count_gauche++;
        } 
        else if (i >= 270 && i <= 315) {  // Droite
            somme_droite += data_lidar_mm_main[i];
            count_droite++;
        }
    }

    float distance_face = somme_face / count_face;
    float distance_gauche = somme_gauche / count_gauche ;
    float distance_droite = somme_droite / count_droite ;
    distance_au_mur [0] = distance_gauche  ; 
    distance_au_mur [1] = distance_face ; 
    distance_au_mur [2] = distance_droite ; 




   // printf("Distance au mur en face  : %.1f mm\n", distance_face);
   // printf("Distance au mur à gauche : %.1f mm\n", distance_gauche);
   // printf("Distance au mur à droite : %.1f mm\n", distance_droite);
  return distance_au_mur ;
}

void virage() 
{
    float* distances = distance_mur(); 

    // Affichage pour débogage
    printf("Gauche: %.1f, Face: %.1f, Droite: %.1f\n", distances[0], distances[1], distances[2]);

    // Vérification si la voiture est dans un virage à gauche
    if (distances[0] < distances[1] && distances[0] < distances[2] && 
        (distances[1] - distances[0] > 50.0 || distances[2] - distances[0] > 50.0)) {
        printf("Virage à gauche détecté.\n");
    } 
    // Vérification si la voiture est dans un virage à droite
    else if (distances[2] < distances[1] && distances[2] < distances[0] && 
             (distances[1] - distances[2] > 50.0 || distances[0] - distances[2] > 50.0)) {
        printf("Virage à droite détecté.\n");
    }
    // Vérification si la voiture est en ligne droite (distances similaires entre gauche, face et droite)
    else if (fabs(distances[0] - distances[1]) < 50.0 && fabs(distances[1] - distances[2]) < 50.0) {
        printf("Ligne droite détectée.\n");
    } 
    else {
        printf("Situation ambiguë ou virage serré détecté.\n");
    } 
}

float deg_to_rad(float deg) {
    return deg * M_PI / 180.0;
}

void discontinuite() {
    float distance_actuelle = 0.0;
    float distance_apres = 0.0;
    float diff = 0.0;
    float seuil_discontinuite = 100.0;  // Seuil pour considérer une discontinuité

    for (int i = 1; i < 359; i++) {
        if ((i >= 0 && i <= 45) || (i >= 315 && i <= 359)) {   
            distance_actuelle = (float) data_lidar_mm_main[i]; 
            distance_apres = (float) data_lidar_mm_main[i + 1];

            diff = fabs(distance_apres - distance_actuelle);

            if (diff >= seuil_discontinuite) 
            {
                if (distance_actuelle < distance_apres) 
                {
                    printf("  -> Minimum local à l'angle %d : %4.2f mm\n", i, distance_actuelle);
                } 
                else if (distance_actuelle > distance_apres) {
                    printf("  -> Maximum local à l'angle %d : %4.2f mm\n", i, distance_actuelle);

                }
            }
        }
    }
}




