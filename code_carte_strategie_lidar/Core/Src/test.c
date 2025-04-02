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
{
	switch (etat_actuelle) {
			case TEST_CHOIX:
				HAL_Delay(10);
				LCD_gotoxy(0, 0);
				LCD_printf("Test a choisir");
				LCD_gotoxy(0, 1);
				LCD_printf("             ");

				if (jog_value == JOG_HAUT) {
					etat_actuelle = TEST_VITESSE;
				} else if (jog_value == 16) {
					etat_actuelle = TEST_HERKULEX;
				} else if (jog_value == 1) {
					etat_actuelle = TEST_LIDAR;
				} else if (jog_value == JOG_GAUCHE) {
					etat_actuelle = TEST_BLUETOOTH;
				}
				break;

			case TEST_VITESSE:
				HAL_Delay(10);
				valeur_cod = cod_value % 101;
				vitesse = (float) valeur_cod / 100.0;
				set_rapport_cyclique_et_sens(vitesse, direction);
				LCD_gotoxy(0, 0);
				LCD_printf("choix:vitesse");
				LCD_gotoxy(0, 1);
				LCD_printf("vitesse=%4.2f", vitesse);

				if (jog_value == 24) {
					direction = true;
				} else if (jog_value == 10) {
					direction = false;
				}
				if (jog_value == 2) {
					etat_actuelle = TEST_BLUETOOTH;
				} else if (jog_value == 16) {
					etat_actuelle = TEST_HERKULEX;
				} else if (jog_value == 1) {
					etat_actuelle = TEST_LIDAR;
				}
				break;

			case TEST_HERKULEX:
				HAL_Delay(10);
				LCD_gotoxy(0, 0);
				LCD_printf("choix:herkulex");
				herkulex_test = ((cod_value%41)+80)*(-1);
				LCD_gotoxy(0, 1);
				LCD_printf("%d   ", herkulex_test);

				set_angle(herkulex_test);

				if (jog_value == 8) {
					etat_actuelle = TEST_VITESSE;
				} else if (jog_value == 1) {
					etat_actuelle = TEST_LIDAR;
				} else if (jog_value == 2) {
					etat_actuelle = TEST_BLUETOOTH;
				}
				break;

			case TEST_LIDAR:
				HAL_Delay(10);
				LCD_gotoxy(0, 0);
				LCD_printf("choix:lidar     ");
				LCD_gotoxy(0,1);
			    LCD_printf("%d         ", data_lidar_mm_main[cod_lidar]);

			    if (cod_value<128)
				{
			   cod_lidar = (cod_value* 90) / 127;

				}
				else if (cod_value>127)
				{
				 cod_lidar = 270 + ((cod_value - 128) * 90) / 127;
				}


				if (jog_value == 8) {
					etat_actuelle = TEST_VITESSE;
				} else if (jog_value == 16) {
					etat_actuelle = TEST_HERKULEX;
				} else if (jog_value == 2) {
					etat_actuelle = TEST_BLUETOOTH;
				}
				break;

			case TEST_BLUETOOTH:
				HAL_Delay(10);
				LCD_gotoxy(0, 0);
				LCD_printf("choix:bluetooth   ");
				LCD_gotoxy(0, 1);
				LCD_printf("           ");
				if (jog_value == 8) {
					etat_actuelle = TEST_VITESSE;
				}

				else if (jog_value == 16) {
					etat_actuelle = TEST_HERKULEX;
				} else if (jog_value == 1) {
					etat_actuelle = TEST_LIDAR;
				}

				break;

			default:
				break;

			}
}

void test_herculex_balayage_plage()
{
	static float angle = ANGLE_HERKULEX_MIN;

	for (; angle <= ANGLE_HERKULEX_MAX; angle++)
	{
		LCD_gotoxy(0, 0);
		LCD_printf("servo %2.2f  ", angle);
		set_angle(angle);

		HAL_Delay(500);

		if (angle >= ANGLE_HERKULEX_MAX)
		{
			angle = ANGLE_HERKULEX_MIN;
		}
	}

}
