/*
 * deplacement.h
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#ifndef INC_DEPLACEMENT_H_
#define INC_DEPLACEMENT_H_

#include <stdbool.h>

void set_rapport_cyclique_et_sens(float nouveau_rapport_cyclique, bool nouveau_sens);
void recule(void);
void avance(void);
void set_angle(float nouvelle_angle);
void set_angle_roue(float angle_roue);
void set_consigne_vitesse(float vitesse, bool sens);

#define CAN_ID_HERKULEX 0x601
#define CAN_ID_MOTEUR 0x602
#define CAN_ID_VITESSE_LINEAIRE 0x603

/**
 * @note ANGLE_HERKULEX_MIN les roues braquent totalement à gauche
 */
#define ANGLE_HERKULEX_MIN -150.0f
/**
 * @note ANGLE_HERKULEX_MAX les roues braquent totalement à droite
 */
#define ANGLE_HERKULEX_MAX -79.0f

#define ANGLE_ROUE_BRAQUAGE_GAUCHE 0
#define ANGLE_ROUE_BRAQUAGE_DROITE 180

extern float rapport_cyclique;
extern bool  sens;
extern float angle;
extern float kp;
extern float vitesse_lineaire;


#endif /* INC_DEPLACEMENT_H_ */
