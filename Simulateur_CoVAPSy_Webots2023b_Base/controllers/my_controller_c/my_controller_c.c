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
#define MAX_SPEED 6.28 // Vitesse maximale des moteurs
#define N 180 
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

const float *range_donnees;
// float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto = 0;
// prototype des fonctions
void affichage_consigne();
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
double radius_to_degrees(double radius, double angle_deg);
// vitesse en km/h
float speed = 0;
float maxSpeed = 28; // km/h
signed int data_lidar_mm_main[360];
typedef enum
{
  INIT,
  MIN_LOCAL,
  MAX_LOCAL,
  AUGMENTER,
  DIMINUER
} etat_discontuinuite;
static etat_discontuinuite etat = INIT;

double map(double x, double in_min, double in_max, double out_min, double out_max);
int get_mapped_angle(int angle_unmapped);
int get_unmapped_angle(int angle_mapped);
double get_angle_cap(double angle_mapped);
void reculer();
void recherches_locaux();
void discontinuite();
int tab_discontuinuite[180][2] = {0};
void clear();
int cpt_discontuinuiter = 0;
int min_locaux[180][2] = {0};
int cpt_min_locaux = 0;
void conduite_autonome();

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
      recherches_locaux();
      discontinuite();
      conduite_autonome();
      vitesse_m_s = 0.8;
      set_vitesse_m_s(vitesse_m_s);
      clear();
    }
  } // Fin de la boucle while

  wbu_driver_cleanup(); // Nettoyage des ressources Webots

  return 0; // Retour correct de la fonction main
}

unsigned char gestion_appui_clavier(void)
{
  int key;
  key = wb_keyboard_get_key();
  switch (key)
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
void set_vitesse_m_s(float vitesse_m_s)
{
  float speed;
  speed = vitesse_m_s * 3.6;
  if (speed > maxSpeed)
    speed = maxSpeed;
  if (speed < 0)
    speed = 0;
  wbu_driver_set_cruising_speed(speed);
}

void recule(void)
{
  wbu_driver_set_cruising_speed(-1);
}

void reculer()
{
  if (data_lidar_mm_main[359] < 250)
  {
    recule(); // Reculer
    wb_robot_step(2500);
  }
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_mapped_angle(int angle_unmapped)
{
  if (angle_unmapped >= 0 && angle_unmapped <= 90)
  {
    return 90 - angle_unmapped;
  }
  else if (angle_unmapped >= 270 && angle_unmapped <= 359)
  {
    return 450 - angle_unmapped;
  }
  else
  {
    return angle_unmapped;
  }
}

int get_unmapped_angle(int angle_mapped)
{
  if (angle_mapped >= 0 && angle_mapped <= 90)
  {
    return 90 - angle_mapped;
  }
  else if (angle_mapped >= 91 && angle_mapped <= 180)
  {
    return 450 - angle_mapped;
  }
  else
  {
    return 0;
  }
}

double get_angle_cap(double angle_mapped)
{
  if (angle_mapped >= 0 && angle_mapped <= 180)
  {
    return map(angle_mapped, 0, 180, 0.31, -0.31);
  }
  else
  {
    return 0;
  }
}

void discontinuite()
{
  printf("--------------------------------\n") ; 
  int distance_courante = 0;
  int distance_suivante = 0;
  int diff = 0;
  int seuil_discontinuite = 100; // Seuil pour considérer une discontinuité

  for (int i = 1; i < 359; i++)
  {
    if ((i >= 0 && i <= 90) || (i >= 270 && i <= 359))
    {
      distance_courante = data_lidar_mm_main[i];
      distance_suivante = data_lidar_mm_main[i + 1];

      diff = (int)fabs(distance_suivante - distance_courante);

      if (diff >= seuil_discontinuite)
      {
        tab_discontuinuite[cpt_discontuinuiter][0] = diff;
        tab_discontuinuite[cpt_discontuinuiter][1] = get_mapped_angle(i);
        cpt_discontuinuiter = cpt_discontuinuiter + 1;
      }
    }
  }
  printf("--------------------------------\n") ; 
}

void recherches_locaux()
{
  int distance_actuelle = 0;
  int distance_apres = 0;

  for (int i = 1; i < 359; i++)
  {
    // Filtrage des angles souhaités (ici 0-90° et 315-359°)
    if ((i >= 0 && i <= 90) || (i >= 315 && i <= 359))
    {
      distance_actuelle = data_lidar_mm_main[i];
      distance_apres = data_lidar_mm_main[i + 1];

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
        min_locaux[cpt_min_locaux][0] = distance_actuelle;
        min_locaux[cpt_min_locaux][1] = get_mapped_angle(i);
        cpt_min_locaux++;
        etat = INIT;
        break;

      case MAX_LOCAL:
        etat = INIT; 
        break;

      default:
        break;
      }
    }
  }
  //   printf("Fin de tour du lidar\n");
}

void conduite_autonome()
{
    float angle = 0;
    int max_discontuinuite_1 = 0;
    int max_discontuinuite_2 = 0;

    int max_min_locaux_1 = 0;
    int angle_max_min_locaux_1 = 0;

    int max_min_locaux_2 = 0;
    int angle_max_min_locaux_2 = 0;

    int angle_1 = 0;
    int angle_2 = 0;

    if (cpt_discontuinuiter == 1)
    {
        for (int j = 0; j < min(180, N); j++)
        {
            if (tab_discontuinuite[j][0] > max_discontuinuite_1)
            {
                max_discontuinuite_1 = tab_discontuinuite[j][0];
                angle_1 = tab_discontuinuite[j][1];
            }
        }

        for (int i = angle_1; i > max(0, angle_1 - 20); i--)
        {
            if (min_locaux[i][1] > max_min_locaux_1)
            {
                max_min_locaux_1 = min_locaux[i][1];
                angle_max_min_locaux_1 = min_locaux[cpt_min_locaux][1];
            }
        }

        for (int k = angle_1; k < min(N, angle_1 + 20); k++)
        {
            if (min_locaux[k][1] > max_min_locaux_2)
            {
                max_min_locaux_2 = min_locaux[k][1];
                angle_max_min_locaux_2 = min_locaux[cpt_min_locaux][1];
            }
        }

        if (max_min_locaux_1 > max_min_locaux_2)
        {
            angle_2 = angle_max_min_locaux_1;
        }
        else if (max_min_locaux_1 < max_min_locaux_2)
        {
            angle_2 = angle_max_min_locaux_2;
        }

        angle = ((angle_1 + angle_2) /2);
    }

    else if (cpt_discontuinuiter > 1)
    {
        for (int i = 0; i < min(50, N); i++)
        {
            if (tab_discontuinuite[i][0] > max_discontuinuite_1)
            {
                max_discontuinuite_2 = max_discontuinuite_1;
                angle_2 = angle_1;
                max_discontuinuite_1 = tab_discontuinuite[i][0];
                angle_1 = tab_discontuinuite[i][1];
            }
            else if (tab_discontuinuite[i][0] > max_discontuinuite_2)
            {
                max_discontuinuite_2 = tab_discontuinuite[i][0];  
                angle_2 = tab_discontuinuite[i][1];
            }
        }
        angle = (angle_1 + angle_2) / 2;
    }
    printf("%4.4f\n", (-(get_angle_cap(angle * 0.9))*180.0/M_PI )      );
    wbu_driver_set_steering_angle(-(get_angle_cap(angle * 0.9)));
}

void clear()
{
  for (int i = 0; i < 50; i++)
  {
    tab_discontuinuite[i][0] = 0;
    tab_discontuinuite[i][1] = 0;
  }

  for (int i = 0; i < 180; i++)
  {
    min_locaux[i][0] = 0;
    min_locaux[i][1] = 0;
  }

  cpt_discontuinuiter = 0;
  cpt_min_locaux = 0;
}


