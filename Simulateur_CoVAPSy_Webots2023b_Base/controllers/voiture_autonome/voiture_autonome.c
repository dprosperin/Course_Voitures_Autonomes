/*
 * File:          my_controller_c.c
 * Description:   Gestion d'une voiture autonome avec états avancés
 */

#include <math.h>
#include <webots/robot.h>
#include <webots/vehicle/car.h>
#include <webots/vehicle/driver.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/lidar.h>

#define TIME_STEP 30
#define MAX_SPEED 6.28 // Vitesse maximale des moteurs
#define LIMITE_DISTANCE 1000 // Distance seuil en mm pour reculer

typedef enum {
    ETAT_DECISION,
    ETAT_AVANCER,
    ETAT_RECULER
} EtatVoiture;

const float *range_donnees;
float speed = 0;
float maxSpeed = 28; // km/h
signed int data_lidar_mm_main[360];
EtatVoiture etat_actuel = ETAT_DECISION;

// Prototypes des fonctions
void affichage_consigne();
void set_vitesse_m_s(float vitesse_m_s);
void recule();
float set_angle();
unsigned char gestion_appui_clavier(void);

int main(int argc, char **argv) {
    unsigned int i;
    float vitesse_m_s;
    
    // Initialisation de Webots
    wbu_driver_init();
    wb_keyboard_enable(TIME_STEP);

    // Initialisation du LIDAR
    WbDeviceTag lidar = wb_robot_get_device("RpLidarA2");
    wb_lidar_enable(lidar, TIME_STEP);
    wb_lidar_enable_point_cloud(lidar);
    
    affichage_consigne();
    set_vitesse_m_s(0);

    while (wbu_driver_step() != -1) {
        float distance_avant;
        
        // Lire les données LIDAR
        range_donnees = wb_lidar_get_range_image(lidar);
        distance_avant = range_donnees[0] * 1000; // Distance en mm

        for (i = 0; i < 360; i++) {
            float distance = range_donnees[360 - i];
            data_lidar_mm_main[i] = (distance > 0.0) ? distance * 1000 : 0;
        }

        gestion_appui_clavier(); // Gestion des touches clavier

        switch (etat_actuel) {
            case ETAT_DECISION:
                if (distance_avant < LIMITE_DISTANCE) {
                    etat_actuel = ETAT_RECULER;
                    printf("Collision imminente ! Passage en mode recul.\n");
                } else {
                    etat_actuel = ETAT_AVANCER;
                }
                break;

            case ETAT_AVANCER:
                set_angle(); // Ajuster la direction
                vitesse_m_s = 0.8;
                set_vitesse_m_s(vitesse_m_s);
                
                // Revenir à l'état décision si un obstacle est détecté
                if (distance_avant < LIMITE_DISTANCE) {
                    etat_actuel = ETAT_DECISION;
                }
                break;

            case ETAT_RECULER:
                recule(); // Reculer
                wb_robot_step(1000); // Reculer pendant une seconde
                etat_actuel = ETAT_DECISION; // Retour à la décision
                break;

            default:
                printf("Erreur : État inconnu !\n");
                etat_actuel = ETAT_DECISION;
                break;
        }
    }

    wbu_driver_cleanup();
    return 0;
}

// Gestion des touches clavier
unsigned char gestion_appui_clavier(void) {
    int key = wb_keyboard_get_key();
    switch (key) {
        case -1:
            break;

        case 'n':
        case 'N':
            if (etat_actuel != ETAT_DECISION) {
                printf("Mode désactivé. Retour à la décision.\n");
                etat_actuel = ETAT_DECISION;
            }
            break;

        case 'a':
        case 'A':
            printf("Mode Auto Activé.\n");
            break;

        default:
            break;
    }
    return key;
}

// Affichage des consignes
void affichage_consigne() {
    printf("Cliquez sur la vue 3D pour commencer.\n");
    printf("Appuyez sur 'A' pour activer le mode auto, 'N' pour désactiver.\n");
}

// Ajuste la vitesse de la voiture
void set_vitesse_m_s(float vitesse_m_s) {
    float speed = vitesse_m_s * 3.6; // Convertir en km/h
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < 0) speed = 0;
    wbu_driver_set_cruising_speed(speed);
}

// Faire reculer la voiture
void recule() {
    wbu_driver_set_cruising_speed(-1);
}

// Ajuster l'angle en fonction des données LIDAR
float set_angle() {
    float distance_droite = data_lidar_mm_main[315];
    float distance_gauche = data_lidar_mm_main[45];
    float distance = distance_droite - distance_gauche;
    float kp = 0.002;
    float angle = kp * distance;

    if (angle > 0.31) angle = 0.31; // Limite droite
    if (angle < -0.31) angle = -0.31; // Limite gauche

    wbu_driver_set_steering_angle(angle);
    return distance;
}
