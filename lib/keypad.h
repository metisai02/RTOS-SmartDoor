/*
 * keypad.h
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal.h"


#ifndef KEYPAD_H_
#define KEYPAD_H_

#define KEYPAD_PORT         GPIOA

#define ROW_PIN_1           GPIO_PIN_4
#define ROW_PIN_2           GPIO_PIN_5
#define ROW_PIN_3           GPIO_PIN_6
#define ROW_PIN_4           GPIO_PIN_7

#define COL_PIN_1           GPIO_PIN_0
#define COL_PIN_2           GPIO_PIN_1
#define COL_PIN_3           GPIO_PIN_2
#define COL_PIN_4           GPIO_PIN_3

void keypad_init(void);
char get_char(void);



#endif /* KEYPAD_H_ */
