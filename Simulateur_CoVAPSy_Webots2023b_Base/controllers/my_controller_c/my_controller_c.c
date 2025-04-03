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
#define MAX_SPEED 6.28 // Vitesse maximale des moteurs
#define TAILLE_TABLEAU_MAPPED 180
#define SEUIL_MIN_LOCAUX 2

#define KP_1_DSCONTINUITE 0.90
#define KP_2_DSCONTINUITE 1.1
#define KP_0_DSCONTINUITE 0.91

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

// angle max de la direction
float maxangle_degre = 16;
// variable étudiant
int tab_mapped[180] = {0};
int tab_discontuinuite[180][2] = {0};
int cpt_discontuinuiter = 0;
float commande_moteur = 0.0 ; 
typedef enum
{
  INIT,
  MIN_LOCAL,
  MAX_LOCAL,
  AUGMENTER,
  DIMINUER
} etat_discontuinuite;

typedef enum
{
  DEBUT,
  AVANCER,
  RECULER,
  BLOQUE
} etat_deplacement_e;
int min_locaux[180][2] = {0};

etat_deplacement_e etat_deplacement = DEBUT;

// Fonction Etudiant
int get_mapped_tableau(signed int data_lidar[360], int data_maped[180]);
void discontinuite();
void recherches_locaux();
void conduite_autonome();
double get_angle_cap(double angle_mapped);
double map(double x, double in_min, double in_max, double out_min, double out_max);
 void clear();
void afficher(int angle_start , int angle_end) ; 

/* main loop
 * Perform simulation steps of TIME_STEP milliseconds
 * and leave the loop when the simulation is over
 */

int main(int argc, char **argv)
{
  unsigned int i;
  signed int data_lidar_mm_main[360];
  float vitesse_m_s;
  /* necessary to initialize webots stuff */
  // intialisation du conducteur de voiture
  wbu_driver_init();
  // enable keyboard
  wb_keyboard_enable(TIME_STEP);
  // enable lidar
  WbDeviceTag lidar = wb_robot_get_device("RpLidarA2");
  wb_lidar_enable(lidar, TIME_STEP);
  // affichage des points lidar sur la piste
  wb_lidar_enable_point_cloud(lidar);

  affichage_consigne();
  set_vitesse_m_s(0);
 
  while (wbu_driver_step() != -1)
  {
    float distance;
    /* lire le lidar et traiter les données :   */
    range_donnees = wb_lidar_get_range_image(lidar);
    distance = range_donnees[0];
    if ((distance > 0.0) && (distance < 20.0))
      data_lidar_mm_main[0] = 1000 * distance;
    else
      data_lidar_mm_main[0] = 0;
    for (i = 1; i < 360; i++)
    {
      distance = range_donnees[i];
      if ((distance > 0.0) && (distance < 20.0))
        data_lidar_mm_main[i] = 1000 * distance;
      else
        data_lidar_mm_main[i] = 0;
    }

    gestion_appui_clavier();
    if (modeAuto)
    {
      get_mapped_tableau(data_lidar_mm_main, tab_mapped);    
     
     switch (etat_deplacement)
     {
       case DEBUT:
          etat_deplacement = AVANCER;
        break;
        
       case AVANCER:
         discontinuite();
         recherches_locaux();
         //afficher(0,180);
         conduite_autonome() ; 
         vitesse_m_s = 0.5;
         set_vitesse_m_s(vitesse_m_s);
         clear();
       if(tab_mapped[90] < 185)
       {
       etat_deplacement = BLOQUE ; 
       }
       
        break;
        
        case BLOQUE:
         etat_deplacement = RECULER ; 
        
        break;
        
        case RECULER:
        vitesse_m_s = 0.5;
        set_vitesse_m_s(vitesse_m_s);
       if (tab_mapped [90] < 250) 
        {
        commande_moteur = -commande_moteur*1.9 , 
        recule();
        wb_robot_step(2500); 
        }
        else 
        {
        etat_deplacement = AVANCER ; 
        }
        break;
        
        default:
        break;
     }
    }
  }

  /* This is necessary to cleanup webots resources */
  wbu_driver_cleanup();
  return 0;
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
int get_mapped_tableau(signed int data_lidar[360], int data_maped[180])
{
  for (int i = 0; i < 90; i++)
  {
    tab_mapped[i + 90] = data_lidar[i];
  }
  for (int i = 270; i < 360; i++)
  {
    tab_mapped[i - 270] = data_lidar[i];
  }
  return 0;
}
void discontinuite()
{
  int distance_courante = 0;
  int distance_suivante = 0;
  int diff = 0;
  int seuil_discontinuite = 250;
  cpt_discontuinuiter = 0;

  for (int i = 0; i < 179; i++)
  {
    distance_courante = tab_mapped[i];
    distance_suivante = tab_mapped[i + 1];
    diff = (int)fabs(distance_suivante - distance_courante);

    if (diff >= seuil_discontinuite)
    {
      tab_discontuinuite[i][0] = distance_courante;
      tab_discontuinuite[i][1] = i;
      cpt_discontuinuiter++;
  //    printf("v_discontinuiter : %d , a:%d\n",distance_courante,i);
    } 
  }
//printf("cpt_discontinuiter : %d \n",cpt_discontuinuiter) ;
}

void recherches_locaux()
{

 static etat_discontuinuite etat = INIT;
  int distance_actuelle = 0;
  int distance_apres = 0;

  for (int i = 0; i < 179; i++)
  {
    distance_actuelle = tab_mapped[i];
    distance_apres = tab_mapped[i + 1];

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
      min_locaux[i][0] = distance_actuelle;
      min_locaux[i][1] = i;
      etat = AUGMENTER;
      break;

    case MAX_LOCAL:
      etat = DIMINUER;
      break;

    default:
      break;
    }
  }
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double get_angle_cap(double angle_mapped)
{
  if (angle_mapped >= 0 && angle_mapped <= 180)
  {
    return map(angle_mapped, 0, 180, -0.31, 0.31);
  }
  else
  {
    return 0;
  }
}

void conduite_autonome()
{
  int angle = 0;
  //
  int max_discontinuite_2_distance_1 = 0;
  int max_discontinuite_2_distance_2 = 0;
  int angle_discontinuite_2_1 = 0;
  int angle_discontinuite_2_2 = 0;
  //
  int max_distance_min_locaux_1 = 0;
  int angle_min_locaux_1 = 0;
  int max_distance_1_discontinuite = 0;
  int angle_1_discontinuite = 0;
  //
  int max_distance_0_discontinuite = 0;
  int angle_0_discontinuite = 0;
  //
  if (cpt_discontuinuiter > 1)
  {
    for (int i = 0; i < TAILLE_TABLEAU_MAPPED; i++)
    {
      if (max_discontinuite_2_distance_1 < tab_discontuinuite[i][0])
      {
        max_discontinuite_2_distance_2 = max_discontinuite_2_distance_1;
        angle_discontinuite_2_2 = angle_discontinuite_2_1;
        max_discontinuite_2_distance_1 = tab_discontuinuite[i][0];
        angle_discontinuite_2_1 = i;
      }
      else if (max_discontinuite_2_distance_2 < tab_discontuinuite[i][0] && max_discontinuite_2_distance_1 > tab_discontuinuite[i][0])
      {
        max_discontinuite_2_distance_2 = tab_discontuinuite[i][0];
        angle_discontinuite_2_2 = i;
      }
      else if (max_discontinuite_2_distance_1 == tab_discontuinuite[i][0] && max_discontinuite_2_distance_2 < tab_discontuinuite[i][0])
      {
        max_discontinuite_2_distance_2 = tab_discontuinuite[i][0];
        angle_discontinuite_2_2 = i;
      }
    }
    angle = (angle_discontinuite_2_1 + angle_discontinuite_2_2) / 2;
    
    angle *= KP_2_DSCONTINUITE;
  }

  else if (cpt_discontuinuiter == 1)
  {
    for (int i = 0; i < TAILLE_TABLEAU_MAPPED; i++)
    {
      if (tab_discontuinuite[i][0] > max_distance_1_discontinuite)
      {
        max_distance_1_discontinuite = tab_discontuinuite[i][0];
        angle_1_discontinuite = i;
      }
    }
  // printf("-----------------------\n");
    for (int i = 0; i < TAILLE_TABLEAU_MAPPED; i++)
    {
      if (min_locaux[i][0] > max_distance_min_locaux_1)
      {
        max_distance_min_locaux_1 = min_locaux[i][0];
        angle_min_locaux_1 = i;
      }
    }
    //   printf("-----------------------\n");
    //printf("min_locaux_d:%d,a:%d\n",max_distance_min_locaux_1,angle_min_locaux_1) ; 
    angle = (angle_min_locaux_1 + angle_1_discontinuite) / 2;
    
    angle *= KP_1_DSCONTINUITE;
  }

  else if (cpt_discontuinuiter == 0)
  {
    for (int i = 0; i < TAILLE_TABLEAU_MAPPED; i++)
    {
      if (tab_mapped[i] > max_distance_0_discontinuite)
      {
        max_distance_0_discontinuite = tab_mapped[i];
        angle_0_discontinuite = i;
      }
    }
  angle   = angle_0_discontinuite ; 

  angle *= KP_0_DSCONTINUITE;
  }
  commande_moteur = get_angle_cap(angle);
  wbu_driver_set_steering_angle(commande_moteur);
}

 
 void clear()
 {
   for (int i = 0; i < 180; i++)
   {
     tab_discontuinuite[i][0] = 0;
     tab_discontuinuite[i][1] = 0;
   }
 
   for (int i = 0; i < 180; i++)
   {
     min_locaux[i][0] = 0;
     min_locaux[i][1] = 0;
   }
 }
   
void afficher(int angle_start , int angle_end) 
 {
    for (int i = angle_start; i < angle_end; i++)
    {
   //   printf("%d, ",tab_mapped[i]);
    }
 }