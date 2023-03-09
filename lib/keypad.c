/*
 * keypad.c
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

#include "keypad.h"

static char keyPad[4][4] = {{'1', '2', '3', 'A'},
                            {'4', '5', '6', 'B'},
                            {'7', '8', '9', 'C'},
                            {'*', '0', '#', 'D'}};

uint8_t tableColum[4] = {COL_PIN_1, COL_PIN_2, COL_PIN_3, COL_PIN_4};
uint8_t tableRow[4] = {ROW_PIN_1, ROW_PIN_2, ROW_PIN_3, ROW_PIN_4};

void keypad_init(void)
{
    for (uint8_t colum = 0; colum < 4; colum++)
    {
        HAL_GPIO_WritePin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_SET);
    }
}

char get_char(void)
{
    for (uint8_t colum = 0; colum < 4; colum++)
    {
        HAL_GPIO_WritePin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_RESET);
        for (uint8_t row = 0; row < 4; row++)
        {
            if (!HAL_GPIO_ReadPin(KEYPAD_PORT, tableRow[row]))
                {
                    HAL_Delay(100);
                    while (!HAL_GPIO_ReadPin(KEYPAD_PORT, tableRow[row]))
                    {
                    };
                    return keyPad[row][colum];
            }
        }
        HAL_GPIO_WritePin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_SET);
    }
    return 0;
}
