/*
 * keypad.h
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */


#ifndef KEYPAD_H_
#define KEYPAD_H_
#include "main.h"

#define KEYPAD_PORT         GPIOA

#define ROW_PIN_1           GPIO_PIN_7
#define ROW_PIN_2           GPIO_PIN_6
#define ROW_PIN_3           GPIO_PIN_5
#define ROW_PIN_4           GPIO_PIN_4

#define COL_PIN_1           GPIO_PIN_3
#define COL_PIN_2           GPIO_PIN_2
#define COL_PIN_3           GPIO_PIN_1
#define COL_PIN_4           GPIO_PIN_0

void keypad_init(void);
char get_char(void);



#endif /* KEYPAD_H_ */
