
#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>

#define TIME_STEP 32
#define SIZE_TABLEAU 200
#define MAX_SPEED 120 // Vitesse maximale des moteurs

const float *range_donnees;
unsigned char gestion_appuie_clavier(void);
unsigned char modeAuto = 0;

void affichage_consigne();
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
void set_angle_etudiant();

// vitesse en km/h
float speed = 100;
float maxSpeed = 110; // km/hp

// angle max de la direction
float maxangle_degre = 318;

// varible etudiant
float sommed1;
float sommed2;
float sommed3;
float sommed4;
float sommeg1;
float sommeg2;
float sommeg3;
float sommeg4;

float moyenned1 = 0;
float moyenned2 = 0;
float moyenned3 = 0;
float moyenned4 = 0;
float moyenneg1 = 0;
float moyenneg2 = 0;
float moyenneg3 = 0;
float moyenneg4 = 0;
float angle_degre_etudiant = 0.0;
float maxangle_degre_etudiant = 360.0;
float angle_etudiant = 0.0;

/* main loop
 * Perform simulation steps of TIME_STEP milliseconds
 * and leave the loop when the simulation is over
 */
signed int data_lidar_mm_main[360];
int main(int argc, char **argv)
{
    unsigned int i;
    float angle_degre, vitesse_m_s;
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
    set_angle_etudiant(0);
    set_vitesse_m_s(0);
    while (wbu_driver_step() != -1)
    {
        printf("moyenne  : g4 = %4.4f  , g3 = %4.4f  , g2 = %4.4f  , g1 = %4.4f , d1 = %4.4f , d2 =  %4.4f, d3 = %4.4f , d4 = %4.4f , angle_roue = %4.4f\r\n", moyenneg4, moyenneg3, moyenneg2, moyenneg1, moyenned4, moyenned3, moyenned2, moyenned1, angle_etudiant);

        angle_degre_etudiant = 0.0;
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
            distance = range_donnees[360 - i];
            if ((distance > 0.0) && (distance < 20.0))
                data_lidar_mm_main[i] = 1000 * distance;
            else
                data_lidar_mm_main[i] = 0;
        }
        gestion_appui_clavier();

        if (modeAuto)
        {

            set_angle_etudiant();

            vitesse_m_s = 0.95;
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
        // case 'r' :
        // case 'R' :
        // printf ("moyenne  : g4 = %4.4f  , g3 = %4.4f  , g2 = %4.4f  , g1 = %4.4f , d1 = %4.4f , d2 =  %4.4f, d3 = %4.4f , d4 = %4.4f , angle_roue = %4.4f \r",moyenneg4,moyenneg3,moyenneg2,moyenneg1,moyenned4,moyenned3,moyenned2,moyenned1,angle_etudiant);

        // break ;

    default:
        break;
    }
    return key;
}

void affichage_consigne()
{
    printf("cliquer sur la vue 3D pour commencer\n");
    printf("a pour mode auto, n pour stop\n");
    printf("w pour valeur lidar\n");
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

void set_angle_etudiant()
{
    sommed1 = 0;
    sommed2 = 0;
    sommed3 = 0;
    sommed4 = 0;
    sommeg1 = 0;
    sommeg2 = 0;
    sommeg3 = 0;
    sommeg4 = 0;

    for (int i = 0; i < 22; i++)
    {
        sommed1 = sommed1 + data_lidar_mm_main[i];
    }
    moyenned1 = sommed1 / 22;

    for (int i = 22; i < 45; i++)
    {
        sommed2 = sommed2 + data_lidar_mm_main[i];
    }
    moyenned2 = sommed2 / 23;

    for (int i = 45; i < 67; i++)
    {
        sommed3 = sommed3 + data_lidar_mm_main[i];
    }
    moyenned3 = sommed3 / 22;

    for (int i = 67; i < 90; i++)
    {
        sommed4 = sommed4 + data_lidar_mm_main[i];
    }
    moyenned4 = sommed4 / 23;

    for (int i = 270; i < 292; i++)
    {
        sommeg1 = sommeg1 + data_lidar_mm_main[i];
    }
    moyenneg1 = sommeg1 / 22;

    for (int i = 292; i < 315; i++)
    {
        sommeg2 = sommeg2 + data_lidar_mm_main[i];
    }
    moyenneg2 = sommeg2 / 23;

    for (int i = 315; i < 337; i++)
    {
        sommeg3 = sommeg3 + data_lidar_mm_main[i];
    }
    moyenneg3 = sommeg3 / 22;

    for (int i = 337; i < 360; i++)
    {
        sommeg4 = sommeg4 + data_lidar_mm_main[i];
    }
    moyenneg4 = sommeg4 / 23;

    // Évaluer chaque moyenne pour ajuster l'angle de direction
    // Côté droit
    if (moyenned1 < moyenned2 && moyenned1 < moyenned3 && moyenned1 < moyenned4)
    {
        // angle_degre_etudiant = angle_degre_etudiant + 10*0.9; // Tourner à gauche si d1 est faible
        angle_degre_etudiant = data_lidar_mm_main[0];
        angle_degre_etudiant = angle_degre_etudiant + data_lidar_mm_main[22] * 0.2;
    }
    else if (moyenned2 < moyenned1 && moyenned2 < moyenned3 && moyenned2 < moyenned4)
    {
        // angle_degre_etudiant = angle_degre_etudiant + 5*0.9; // Tourner un peu à gauche si d2 est faible
        angle_degre_etudiant = data_lidar_mm_main[22];
        angle_degre_etudiant = angle_degre_etudiant + data_lidar_mm_main[45] * 0.2;
    }
    else if (moyenned3 < moyenned1 && moyenned3 < moyenned2 && moyenned3 < moyenned4)
    {
        // angle_degre_etudiant = angle_degre_etudiant +  3*0.9; // Tourner légèrement à gauche si d3 est faible
        angle_degre_etudiant = data_lidar_mm_main[45];
        angle_degre_etudiant = angle_degre_etudiant + data_lidar_mm_main[67] * 0.2;
    }
    else if (moyenned4 < moyenned1 && moyenned4 < moyenned2 && moyenned4 < moyenned3)
    {
        // angle_degre_etudiant = angle_degre_etudiant +  1*0.9; // Tourner très légèrement à gauche si d4 est faible
        angle_degre_etudiant = data_lidar_mm_main[67];
        angle_degre_etudiant = angle_degre_etudiant + data_lidar_mm_main[90] * 0.2;
    }
    else
    {
        // angle_degre_etudiant = angle_degre_etudiant +  1 *0.9;
        angle_degre_etudiant = data_lidar_mm_main[0] * 0.2;
    }

    // Côté gauche
    if (moyenneg1 < moyenneg2 && moyenneg1 < moyenneg3 && moyenneg1 < moyenneg4)
    {
        // angle_degre_etudiant = angle_degre_etudiant - 10*0.9;// Tourner à droite si g1 est faible
        angle_degre_etudiant = data_lidar_mm_main[270];
        angle_degre_etudiant = angle_degre_etudiant - data_lidar_mm_main[292] * 0.2;
    }
    else if (moyenneg2 < moyenneg1 && moyenneg2 < moyenneg3 && moyenneg2 < moyenneg4)
    {
        // angle_degre_etudiant = angle_degre_etudiant - 5*0.9; // Tourner un peu à droite si g2 est faible
        angle_degre_etudiant = data_lidar_mm_main[292];
        angle_degre_etudiant = angle_degre_etudiant - data_lidar_mm_main[315] * 0.2;
    }
    else if (moyenneg3 < moyenneg1 && moyenneg3 < moyenneg2 && moyenneg3 < moyenneg4)
    {
        // angle_degre_etudiant = angle_degre_etudiant - 3*0.9; // Tourner légèrement à droite si g3 est faible
        angle_degre_etudiant = data_lidar_mm_main[315];
        angle_degre_etudiant = angle_degre_etudiant - data_lidar_mm_main[337] * 0.2;
    }
    else if (moyenneg4 < moyenneg1 && moyenneg4 < moyenneg2 && moyenneg4 < moyenneg3)
    {
        // angle_degre_etudiant = angle_degre_etudiant - 1*0.9; // Tourner très légèrement à droite si g4 est faible
        angle_degre_etudiant = data_lidar_mm_main[337];
        angle_degre_etudiant = angle_degre_etudiant - data_lidar_mm_main[360] * 0.2;
    }
    else
    {
        // angle_degre_etudiant = angle_degre_etudiant - 1*0.9;
        angle_degre_etudiant = data_lidar_mm_main[0] * 0.2;
    }

    // Limiter l'angle de direction
    if (angle_degre_etudiant > maxangle_degre_etudiant)
    {
        angle_degre_etudiant = maxangle_degre_etudiant;
    }
    else if (angle_degre_etudiant < -maxangle_degre_etudiant)
    {
        angle_degre_etudiant = -maxangle_degre_etudiant;
    }
    angle_etudiant = angle_degre_etudiant * 3.14 / 180;
    wbu_driver_set_steering_angle(angle_etudiant);
}
