//
// Created by David PROSPÉRIN on 30/03/2025.
//

#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

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

// prototype des fonctions
void affichage_consigne();
void set_direction_degres(float angle_degre);
void set_vitesse_m_s(float vitesse_m_s);
unsigned char gestion_appui_clavier(void);
void recule(void);
void lidar_init(void);
void voiture_init(void);
void lidar_read(void);

//vitesse en km/h
extern float speed;
extern float maxSpeed; //km/h

// angle max de la direction
extern float maxangle_degre;

extern const float* range_donnees;
//float range_donnees[SIZE_TABLEAU];
extern unsigned char gestion_appuie_clavier(void);
extern unsigned char modeAuto;

extern signed int data_lidar_mm_main[360];
extern float angle_degre, vitesse_m_s;

#endif //DEPLACEMENT_H
