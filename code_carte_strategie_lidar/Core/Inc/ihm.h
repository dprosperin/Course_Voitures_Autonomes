/*
 * ihm.h
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#ifndef INC_IHM_H_
#define INC_IHM_H_

#include "fdcan.h"

void BAR_set(uint16_t motif_BAR);
void LCD_gotoxy (uint8_t x, uint8_t y);
void LCD_clear(void);
void LCD_printf(const char* format, ...);
void automate_decode_IHM(void);
void COD_read(void);
void JOG_read(void);

extern uint16_t marker1;
extern T_FDCAN_trame_rx trame_rx;
extern T_FDCAN_trame_rx buffer_trame_rx[32];
extern uint8_t jog_value;
extern uint8_t cod_value;

#define JOG_GAUCHE 2
#define JOG_DROITE 10
#define JOG_HAUT 8
#define JOG_BAS 1
#define JOG_CENTRE 4

#endif /* INC_IHM_H_ */
