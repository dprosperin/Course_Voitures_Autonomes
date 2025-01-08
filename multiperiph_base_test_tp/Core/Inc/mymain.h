/*
 * mymain.h
 *
 *  Created on: Jan 6, 2025
 *      Author: yassine.mesbahi
 */

#ifndef INC_MYMAIN_H_
#define INC_MYMAIN_H_

#include <stdbool.h>

void setup();

void loop();

void set_angle_test(void);
void set_vitesse_et_sens(float nouvelle_vitesse, bool nouveau_sens);
void set_angle(float nouvelle_angle);
void recule(void);
void avance(void);

#define CAN_ID_MOTEUR 0x700
#define CAN_ID_HERKULEX 0x701

#endif /* INC_MYMAIN_H_ */
