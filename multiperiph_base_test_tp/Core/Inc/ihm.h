/*
 * ihm.h
 *
 *  Created on: Jan 16, 2025
 *      Author: davidprosperin
 */

#ifndef INC_IHM_H_
#define INC_IHM_H_

void BAR_set(uint16_t motif_BAR);
void LCD_gotoxy (uint8_t x, uint8_t y);
void LCD_clear(void);
void LCD_printf(const char* format, ...);

#endif /* INC_IHM_H_ */
