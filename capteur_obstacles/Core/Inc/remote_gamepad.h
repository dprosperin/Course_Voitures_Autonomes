/*
 * remote_gamepad.c
 *
 *  Created on: Jan 25, 2025
 *      Author: davidprosperin
 */

#ifndef INC_REMOTE_GAMEPAD_H_
#define INC_REMOTE_GAMEPAD_H_

#include <stdint.h>

/*
 *	Tableau de correspondance
 * | Bouton           | Caractère reçu | Action associé                             |
 * | ---------------- | -------------- | ------------------------------------------ |
 * | Flèche du haut   | w              | Marche avant                               |
 * | Flèche du bas    | s              | Marche arrière                             |
 * | Flèche du gauche | a              | Tourner à gauche                           |
 * | Flèche du droite | d              | Tourner à gauche                           |
 * | L2               | o              | Diminuer la vitesse du moteur CC           |
 * | L1               | l              | Augmenter la vitesse du moteur CC          |
 * | Start            | n              | Démarrer du programme de conduite autonome |
 * | Select           | m              | Arret du programme de conduite autonome    |
 * | B                | v              | Arret d'urgence du véhicule                |                              |
 *
 */

typedef enum {
	GAMEPAD_TOP_ARROW    = 'w',
	GAMEPAD_BOTTOM_ARROW = 's',
	GAMEPAD_LEFT_ARROW   = 'a',
	GAMEPAD_RIGHT_ARROW  = 'd',
	GAMEPAD_L2           = 'o',
	GAMEPAD_L1           = 'l',
	GAMEPAD_START        = 'n',
	GAMEPAD_SELECT       = 'm',
	GAMEPAD_B            = 'v',
	GAMEPAD_STOP_PRESSED = '0',
	GAMEPAD_R1           = 'p',
	GAMEPAD_R2           = 'k'
} gamepad_button_t;

void handle_receive_character(uint8_t receive_character);
void send_start_autonomous_driving(void);
void send_stop_autonomous_driving(void);
void send_augmenter_vitesse_moyenne(void);
void send_diminuer_vitesse_moyenne(void);

#define CAN_ID_START_AUTONOMOUS_DRIVING 0x500
#define CAN_ID_STOP_AUTONOMOUS_DRIVING 0x501
#define CAN_ID_AUGMENTER_VITESSE_MOYENNE 0x300
#define CAN_ID_DIMINUER_VITESSE_MOYENNE 0x301

#endif /* INC_REMOTE_GAMEPAD_C_ */
