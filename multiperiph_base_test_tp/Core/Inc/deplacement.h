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


#endif /* INC_DEPLACEMENT_H_ */
