/*
 * test.c
 *
 *  Created on: Jan 27, 2025
 *      Author: yassi
 */
#include <test.h>
#include <main.h>
#include <deplacement.h>
#include <ihm.h>
#include "lidar.h"

etat_test etat_actuelle = TEST_CHOIX;


//variable pour test herkulex
int16_t herkulex_test = 0.0;


//variable pour test vitesse
float vitesse;
bool direction = true;
bool maintenir = 0;
uint8_t valeur_cod = 0;

//variable pour test lidar ;
uint16_t cod_lidar = 0;

void test_composants_voiture ()
{}
