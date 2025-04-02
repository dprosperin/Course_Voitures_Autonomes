/*
 * deplacement.h
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	TEST_CHOIX, TEST_VITESSE, TEST_LIDAR, TEST_HERKULEX, TEST_BLUETOOTH
} etat_test;

extern etat_test etat_actuelle;

//variable pour test herkulex
extern int16_t herkulex_test;

//variable pour test vitesse
extern float vitesse;
extern bool direction;
extern bool maintenir;
extern uint8_t valeur_cod;

//variable pour test lidar ;
extern uint16_t cod_lidar;

void test_composants_voiture ();
void test_herculex_balayage_plage();
void test_roues_balayage_plage_0_180();

#endif /* INC_DEPLACEMENT_H_ */
