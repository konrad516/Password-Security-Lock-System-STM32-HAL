/*
 * lcd_i2c.h
 *
 *  Created on: Nov 30, 2020
 *      Author: lunqe
 */

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

#define LCD_ADDRESS 0x27

#define LCD_ROWS 2
#define LCD_COLUMNS 14
#define LCD_SETDDRAMADDR 0x80
#define LCD_FULLLINE 0x40

#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_BLINK_ON 0x0D
#define LCD_BLINK_OFF 0x0C

void LCD_Init(void);
void LCD_Print(char *text);
void LCD_SetCursor(uint8_t columnes, uint8_t rows);
void LCD_PrintXY(char *text, uint8_t columnes, uint8_t rows);
void LCD_Command(char command);
void LCD_Clear(void);

#endif /* INC_LCD_I2C_H_ */
