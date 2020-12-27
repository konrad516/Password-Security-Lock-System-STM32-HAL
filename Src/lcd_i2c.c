/*
 * lcd_i2c.c
 *
 *  Created on: Nov 30, 2020
 *      Author: lunqe
 */

#include "main.h"
#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c1;

void LCD_Init(void)
{
	//figure 24 datasheet HD44780U
	HAL_Delay(42);
	LCD_Command (0x30);
	HAL_Delay(5);
	LCD_Command (0x30);
	HAL_Delay(1);
	LCD_Command (0x30);
	HAL_Delay(10);
	LCD_Command (0x20);	//4bit mode
	HAL_Delay(10);

  	LCD_Command (0x28); // N=1 2line, F=0 5x8
	HAL_Delay(1);
	LCD_Command (0x08); //display off, cursor off, blink off
	HAL_Delay(1);
	LCD_Command (0x01);  // clear display
	HAL_Delay(1);
	LCD_Command (0x06); // I/D=1 increment cursor, S=0 no shift
	HAL_Delay(1);
	LCD_Command (0x0C); //Display on, cursor off, blink off
	HAL_Delay(1);
}
void LCD_Write(char data)
{
	char data_u = data & 0xF0;
	char data_l = (data << 4) & 0xF0;
	uint8_t data_s[4];
	data_s[0] = data_u | 0b00001101; //RS=1 RW=0 E=1 BACKLIGHT
	data_s[1] = data_u | 0b00001001; //RS=1 RW=0 E=0 BACKLIGHT
	data_s[2] = data_l | 0b00001101; //RS=1 RW=0 E=1 BACKLIGHT
	data_s[3] = data_l | 0b00001001; //RS=1 RW=0 E=0 BACKLIGHT
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS<<1, (uint8_t *)data_s, sizeof(data_s), 100);
}
void LCD_Command(char command)
{
	char data_u = command & 0xF0;
	char data_l = (command << 4) & 0xF0;
	uint8_t data_s[4];
	data_s[0] = data_u | 0b00001100; //RS=0 RW=0 E=1 BACKLIGHT
	data_s[1] = data_u | 0b00001000; //RS=0 RW=0 E=0 BACKLIGHT
	data_s[2] = data_l | 0b00001100; //RS=0 RW=0 E=1 BACKLIGHT
	data_s[3] = data_l | 0b00001000; //RS=0 RW=0 E=0 BACKLIGHT
	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS<<1, (uint8_t *)data_s, sizeof(data_s), 100);
}
void LCD_Print(char *text)
{
	while(*text)
		LCD_Write(*text++);
}
void LCD_SetCursor(uint8_t columnes, uint8_t rows)
{
	if(rows>LCD_ROWS-1)
		rows=LCD_ROWS-1;
	if(columnes>LCD_COLUMNS-1)
		columnes=LCD_COLUMNS-1;
	LCD_Command(LCD_SETDDRAMADDR + columnes + (LCD_FULLLINE * rows));
}
void LCD_PrintXY(char *text, uint8_t columnes, uint8_t rows)
{
	LCD_SetCursor(columnes, rows);
	LCD_Print(text);
}

void LCD_Clear(void)
{
	LCD_Command (LCD_CLEAR);
}
