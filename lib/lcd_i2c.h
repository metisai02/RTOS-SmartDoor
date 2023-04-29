/*
 * lcd_i2c.h
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "main.h"

#define LCD_COMMAND 0x00
#define LCD_DATA 0x01

#define LCD_EN 0x04 // Enable bit
#define LCD_RW 0x02 // Read/Write bit
#define LCD_RS 0x01 // Register select bit

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

typedef struct
{
  I2C_HandleTypeDef *I2C;
  uint8_t ADDRESS;
  uint8_t ENTRYMODE;
  uint8_t DISPLAYCTRL;
  uint8_t CURSORSHIFT;
  uint8_t FUNCTIONSET;
  uint8_t BACKLIGHT;
} I2C_LCD_Handle_t;

void I2C_LCD_init(I2C_LCD_Handle_t *LCD, I2C_HandleTypeDef *I2CHandle, uint8_t SlaveAddr);
void I2C_LCD_print_char(I2C_LCD_Handle_t *LCD, char data);
void I2C_LCD_display_clear(I2C_LCD_Handle_t *LCD);
void I2C_LCD_display_return_home(I2C_LCD_Handle_t *LCD);
void I2C_LCD_print_string(I2C_LCD_Handle_t *LCD, char *str);
void I2C_LCD_set_cursor(I2C_LCD_Handle_t *LCD, uint8_t row, uint8_t column);


#endif /* LCD_I2C_H_ */
