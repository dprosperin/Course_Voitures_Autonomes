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
#define MAX_DROITE 16
#define MAX_GAUCHE -16

const float *range_donnees;
// float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto = 0;
// prototype des fonctions
void affichage_consigne();
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);

// vitesse en km/h
float speed = 0;
float maxSpeed = 28; // km/h
signed int data_lidar_mm_main[360];

// Code Etudiant
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
void marche_arriere();
void recherches_locaux();
void discontinuite();
void conduite_autonome();
void clear();
double radius_to_degrees(double radius, double angle_deg);
void set_direction_degre(float angle_degre);

#define TAB_DISCONTINUITE_TAILLE 180
int tab_discontinuite[TAB_DISCONTINUITE_TAILLE][2] = {0};

int cpt_discontuinuiter = 0;

#define MIN_LOCAUX_TAILLE 180
int min_locaux[MIN_LOCAUX_TAILLE][2] = {0};
int cpt_min_locaux = 0;

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

void marche_arriere()
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
        tab_discontinuite[cpt_discontuinuiter][0] = diff;
        tab_discontinuite[cpt_discontuinuiter][1] = get_mapped_angle(i);
        cpt_discontuinuiter = cpt_discontuinuiter + 1;
      }
    }
  }
}

void recherches_locaux()
{
  int distance_actuelle = 0;
  int distance_suivante = 0;

  for (int i = 1; i < 359; i++)
  {
    // Filtrage des angles souhaités (ici 0-90° et 315-359°)
    if ((i >= 0 && i <= 90) || (i >= 315 && i <= 359))
    {
      distance_actuelle = data_lidar_mm_main[i];
      distance_suivante = data_lidar_mm_main[i + 1];

      switch (etat)
      {
      case INIT:
        if (distance_actuelle < distance_suivante)
        {
          etat = AUGMENTER;
        }
        else if (distance_actuelle > distance_suivante)
        {
          etat = DIMINUER;
        }
        break;

      case AUGMENTER:
        if (distance_actuelle < distance_suivante)
        {
          etat = AUGMENTER;
        }
        else if (distance_actuelle > distance_suivante)
        {
          etat = MAX_LOCAL;
        }
        break;

      case DIMINUER:
        if (distance_actuelle > distance_suivante)
        {
          etat = DIMINUER;
        }
        else if (distance_actuelle < distance_suivante)
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
}

void conduite_autonome()
{
  /**
   * @note Cas avec plusieurs discontinuitées
   */
  int max_distance_discontinuite_1 = 0;
  int max_distance_discontinuite_2 = 0;

  int max_distance_discontinuite_angle_1 = 0, max_distance_discontinuite_angle_2 = 0;

  int angle = 0;

  if (cpt_discontuinuiter > 1)
  {
    for (int i = 0; i < TAB_DISCONTINUITE_TAILLE; i++)
    {
      if (tab_discontinuite[i][0] > max_distance_discontinuite_1)
      {
        max_distance_discontinuite_2 = max_distance_discontinuite_1;
        max_distance_discontinuite_angle_2 = tab_discontinuite[i][1];
        max_distance_discontinuite_1 = tab_discontinuite[i][0];
        max_distance_discontinuite_angle_1 = tab_discontinuite[i][1];
      }
      else if (tab_discontinuite[i][0] >= max_distance_discontinuite_2 && tab_discontinuite[i][0] <= max_distance_discontinuite_1)
      {
        max_distance_discontinuite_2 = tab_discontinuite[i][0];
        max_distance_discontinuite_angle_2 = tab_discontinuite[i][1];
      }
      else if (tab_discontinuite[i][0] == max_distance_discontinuite_1)
      {
        max_distance_discontinuite_2 = max_distance_discontinuite_1;
        max_distance_discontinuite_angle_2 = tab_discontinuite[i][1];
      }
    }
    angle = ((max_distance_discontinuite_angle_1 + max_distance_discontinuite_angle_2) / 2);
  }
  printf("angle : %d\n", angle);

  float commande_servomoteur = map(angle, 0, 180, -16, 16);

  set_direction_degre(commande_servomoteur);
}

void clear()
{
  for (int i = 0; i < MIN_LOCAUX_TAILLE; i++)
  {
    tab_discontinuite[i][0] = 0;
    tab_discontinuite[i][1] = 0;
  }

  for (int i = 0; i < 180; i++)
  {
    min_locaux[i][0] = 0;
    min_locaux[i][1] = 0;
  }

  cpt_discontuinuiter = 0;
  cpt_min_locaux = 0;
}

void set_direction_degre(float angle_degre)
{
  if (angle_degre > MAX_DROITE)
  {
    angle_degre = MAX_DROITE;
  }
  else if (angle_degre < MAX_GAUCHE)
  {
    angle_degre = MAX_GAUCHE;
  }
  angle_degre = angle_degre * 3.14 / 180.0;
  printf("angle_degre : %f\n", angle_degre);
  wbu_driver_set_steering_angle(angle_degre);
}