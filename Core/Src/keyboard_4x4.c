/*
 * keyboard_4x4.c
 *
 *  Created on: Dec 1, 2020
 *      Author: lunqe
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "keyboard_4x4.h"

char KB_map[4][4]={
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}};

char KB_rows[4]={KB_R1,KB_R2,KB_R3,KB_R4};
char KB_columnes[4]={KB_C1,KB_C2,KB_C3,KB_C4};

void KB_init(void)
{
	GPIO_InitTypeDef GPIO_KB_init;

	GPIO_KB_init.Pin = KB_R1 | KB_R2 | KB_R3 | KB_R4;
	GPIO_KB_init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_KB_init.Pull = GPIO_NOPULL;
	GPIO_KB_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_GPIO, &GPIO_KB_init);

	GPIO_KB_init.Pin = KB_C1 | KB_C2 | KB_C3 | KB_C4;
	GPIO_KB_init.Mode = GPIO_MODE_INPUT;
	GPIO_KB_init.Pull = GPIO_PULLDOWN;
	GPIO_KB_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_GPIO, &GPIO_KB_init);
}

char KB_read(void)
{
	char result = 0x00;

	for(uint8_t i=0; i<4; i++)
	{
		HAL_GPIO_WritePin(KB_GPIO, KB_rows[i], GPIO_PIN_SET);
		for(uint8_t j=0; j<4; j++)
		{
			if(HAL_GPIO_ReadPin(KB_GPIO, KB_columnes[j])==GPIO_PIN_SET) result=KB_map[i][j];
		}
		HAL_GPIO_WritePin(KB_GPIO, KB_rows[i], GPIO_PIN_RESET);
	}
	return result;
}
