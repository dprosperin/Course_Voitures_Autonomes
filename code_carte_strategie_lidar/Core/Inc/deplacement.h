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

#define CAN_ID_HERKULEX 0x601
#define CAN_ID_MOTEUR 0x602

#define ANGLE_HERKULEX_MIN -150.0f
#define ANGLE_HERKULEX_MAX -79.0f

extern float rapport_cyclique;
extern bool  sens;
extern float angle;
extern float kp;
extern float vitesse_lineaire;


#endif /* INC_DEPLACEMENT_H_ */
