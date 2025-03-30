//
// Created by David PROSPÉRIN on 30/03/2025.
//

#include "lib_voiture.h"

const float* range_donnees;
//float range_donnees[SIZE_TABLEAU];
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto=0;

signed int data_lidar_mm_main[360];
float angle_degre, vitesse_m_s;

//vitesse en km/h
float speed = 0;
float maxSpeed = 28; //km/h

// angle max de la direction
float maxangle_degre = 16;

WbDeviceTag lidar;

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

void set_direction_degres(float angle_degre)
{
    float angle=0;
    if(angle_degre > maxangle_degre)
        angle_degre = maxangle_degre;
    else if(angle_degre < -maxangle_degre)
        angle_degre = -maxangle_degre;
    angle = -angle_degre * 3.14/180;
    wbu_driver_set_steering_angle(angle);
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

void lidar_init(void){
    // enable lidar
    lidar = wb_robot_get_device("RpLidarA2");
    wb_lidar_enable(lidar,TIME_STEP);
    // affichage des points lidar sur la piste
    wb_lidar_enable_point_cloud(lidar);
}

void voiture_init(void){
    /* necessary to initialize webots stuff */
    //intialisation du conducteur de voiture
    wbu_driver_init();
    //enable keyboard
    wb_keyboard_enable(TIME_STEP);
}

/**
 * @note On a touné virtuellement le lidar de 90° vers la gauche pour qu'il correspond à la configuration réelle
 *
 */
void lidar_read(void){
    float distance;
    /* lire le lidar et traiter les données :   */
    range_donnees=wb_lidar_get_range_image(lidar);
    distance = range_donnees[0];
    if((distance > 0.0) && (distance <20.0))
        data_lidar_mm_main[rotation_angle_90_horaire(0)]=1000*distance;
    else data_lidar_mm_main[rotation_angle_90_horaire(0)] = 0;
    for(unsigned int i = 1; i<360 ; i++)
    {
        if((distance > 0.0) && (distance <20.0))
            data_lidar_mm_main[rotation_angle_90_horaire(i)]=1000*range_donnees[i];
        else data_lidar_mm_main[rotation_angle_90_horaire(i)] = 0;
    }
}