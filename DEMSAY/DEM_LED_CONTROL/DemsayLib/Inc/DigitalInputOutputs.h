/*
 * DigitalInputOutputs.h
 *
 *  Created on: 11 Eki 2023
 *      Author: Onur
 */
#include "main.h"
#include "stm32g0xx_hal.h"
#ifndef DEMSAYLIB_INC_DIGITALINPUTOUTPUTS_H_
#define DEMSAYLIB_INC_DIGITALINPUTOUTPUTS_H_

typedef enum {
	Passive = 0, Active,
NotState
} Status_t;

void RGB_LED_Control(uint8_t LED_Red, uint8_t LED_Green, uint8_t LED_Blue);
void LED_Control(uint8_t LED1_Red, uint8_t LED2_Green, uint8_t LED3_Blue,
	uint8_t LED4_White, uint8_t LED5_Yellow);
void Buzzer_Control(uint8_t BuzzerState);

#endif /* DEMSAYLIB_INC_DIGITALINPUTOUTPUTS_H_ */

