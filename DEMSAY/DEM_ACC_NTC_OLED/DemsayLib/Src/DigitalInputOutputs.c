/*
 * DigitalInputOutputs.c
 *
 *  Created on: 11 Eki 2023
 *      Author: Onur
 */
#include "DigitalInputOutputs.h"

void RGB_LED_Control(uint8_t LED_Red, uint8_t LED_Green, uint8_t LED_Blue) {
	if (LED_Red == 1)
		HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 1);
	else
		HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 0);

	if (LED_Green == 1)
		HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 1);
	else
		HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 0);

	if (LED_Blue == 1)
		HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 1);
	else
		HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 0);
}

void LED_Control(uint8_t LED1_Red, uint8_t LED2_Green, uint8_t LED3_Blue,
		uint8_t LED4_White, uint8_t LED5_Yellow) {

	if (LED1_Red == 1)
		HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, 1);
	else
		HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, 0);

	if (LED2_Green == 1)
		HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, 1);
	else
		HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, 0);

	if (LED3_Blue == 1)
		HAL_GPIO_WritePin(LED3_BLUE_GPIO_Port, LED3_BLUE_Pin, 1);
	else

		HAL_GPIO_WritePin(LED3_BLUE_GPIO_Port, LED3_BLUE_Pin, 0);

	if (LED4_White == 1)
		HAL_GPIO_WritePin(LED4_WHITE_GPIO_Port, LED4_WHITE_Pin, 1);
	else
		HAL_GPIO_WritePin(LED4_WHITE_GPIO_Port, LED4_WHITE_Pin, 0);

	if (LED5_Yellow == 1)
		HAL_GPIO_WritePin(LED5_YELLOW_GPIO_Port, LED5_YELLOW_Pin, 1);
	else
		HAL_GPIO_WritePin(LED5_YELLOW_GPIO_Port, LED5_YELLOW_Pin, 0);
}

void Buzzer_Control(uint8_t BuzzerState) {
	if (BuzzerState == 1)
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	else
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);

}

