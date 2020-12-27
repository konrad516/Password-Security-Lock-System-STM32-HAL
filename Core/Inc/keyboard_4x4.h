/*
 * keyboard_4x4.h
 *
 *  Created on: Dec 1, 2020
 *      Author: lunqe
 */

#ifndef INC_KEYBOARD_4X4_H_
#define INC_KEYBOARD_4X4_H_

#define KB_GPIO GPIOD

#define KB_R1 GPIO_PIN_0
#define KB_R2 GPIO_PIN_1
#define KB_R3 GPIO_PIN_2
#define KB_R4 GPIO_PIN_3

#define KB_C1 GPIO_PIN_4
#define KB_C2 GPIO_PIN_5
#define KB_C3 GPIO_PIN_6
#define KB_C4 GPIO_PIN_7



void KB_init(void);
char KB_read(void);

#endif /* INC_KEYBOARD_4X4_H_ */
