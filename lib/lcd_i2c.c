/*
 * lcd_i2c.c
 *
 *  Created on: Mar 6, 2023
 *      Author: DELL
 */

// this thread has been managed by hanh

#include "lcd_i2c.h"

static void mdelay(uint32_t time)
{
  for (uint32_t i = 0; i < (time * 1000); i++)
    ;
}

static void udelay(uint32_t time)
{
  for (uint32_t i = 0; i < (time * 1); i++)
    ;
}

static void I2C_LCD_Write(I2C_LCD_Handle_t *LCD, uint8_t data, uint8_t mode)
{
  char data_h;
  char data_l;
  uint8_t data_t[4];
  data_h = data & 0xF0;
  data_l = (data << 4) & 0xF0;

  if (LCD->BACKLIGHT)
  {
    data_h |= LCD_BACKLIGHT;
    data_l |= LCD_BACKLIGHT;
  }

  if (mode == LCD_DATA)
  {
    data_h |= LCD_RS;
    data_l |= LCD_RS;
  }
  else if (mode == LCD_COMMAND)
  {
    data_h &= ~LCD_RS;
    data_l &= ~LCD_RS;
  }
  data_t[0] = data_h | LCD_EN;
  mdelay(1);
  data_t[1] = data_h;
  data_t[2] = data_l | LCD_EN;
  mdelay(1);
  data_t[3] = data_l;
  HAL_I2C_Master_Transmit(LCD->I2C, LCD->ADDRESS, (uint8_t *)data_t, sizeof(data_t), HAL_MAX_DELAY);
}

void I2C_LCD_init(I2C_LCD_Handle_t *LCD, I2C_HandleTypeDef *I2CHandle, uint8_t SlaveAddr)
{
  LCD->I2C = I2CHandle;
  LCD->ADDRESS = SlaveAddr;
  LCD->FUNCTIONSET = LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  LCD->ENTRYMODE = LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  LCD->DISPLAYCTRL = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  LCD->CURSORSHIFT = LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT;
  LCD->BACKLIGHT = LCD_BACKLIGHT;

  mdelay(50);
  I2C_LCD_Write(LCD, 0x33, LCD_COMMAND);
  mdelay(5);
  I2C_LCD_Write(LCD, 0x33, LCD_COMMAND);
  udelay(200);
  I2C_LCD_Write(LCD, 0x32, LCD_COMMAND);
  mdelay(5);
  I2C_LCD_Write(LCD, 0x20, LCD_COMMAND);
  mdelay(5);

  I2C_LCD_Write(LCD, LCD->ENTRYMODE, LCD_COMMAND);
  I2C_LCD_Write(LCD, LCD->DISPLAYCTRL, LCD_COMMAND);
  I2C_LCD_Write(LCD, LCD->CURSORSHIFT, LCD_COMMAND);
  I2C_LCD_Write(LCD, LCD->FUNCTIONSET, LCD_COMMAND);
  I2C_LCD_Write(LCD, LCD_CLEARDISPLAY, LCD_COMMAND);
  I2C_LCD_Write(LCD, LCD_RETURNHOME, LCD_COMMAND);
}

void I2C_LCD_print_char(I2C_LCD_Handle_t *LCD, char data)
{
  I2C_LCD_Write(LCD, data, LCD_DATA);
}

void I2C_LCD_display_clear(I2C_LCD_Handle_t *LCD)
{
  I2C_LCD_Write(LCD, LCD_CLEARDISPLAY, LCD_COMMAND);
}

void I2C_LCD_display_return_home(I2C_LCD_Handle_t *LCD)
{
  I2C_LCD_Write(LCD, LCD_RETURNHOME, LCD_COMMAND);
}

void I2C_LCD_print_string(I2C_LCD_Handle_t *LCD, char *str)
{
  while (*str)
    I2C_LCD_Write(LCD, *str++, LCD_DATA);
}

void I2C_LCD_set_cursor(I2C_LCD_Handle_t *LCD, uint8_t row, uint8_t column)
{
  switch (row)
  {
  case 0:
    column |= 0x80;
    break;
  case 1:
    column |= 0xC0;
    break;
  }
  I2C_LCD_Write(LCD, column, LCD_COMMAND);
}