/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A1_IN4_NTC_Pin GPIO_PIN_4
#define A1_IN4_NTC_GPIO_Port GPIOA
#define RGB_R_Pin GPIO_PIN_6
#define RGB_R_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_8
#define RGB_G_GPIO_Port GPIOA
#define LED1_RED_Pin GPIO_PIN_15
#define LED1_RED_GPIO_Port GPIOA
#define LED2_GREEN_Pin GPIO_PIN_0
#define LED2_GREEN_GPIO_Port GPIOD
#define LED3_BLUE_Pin GPIO_PIN_1
#define LED3_BLUE_GPIO_Port GPIOD
#define LED4_WHITE_Pin GPIO_PIN_2
#define LED4_WHITE_GPIO_Port GPIOD
#define LED5_YELLOW_Pin GPIO_PIN_3
#define LED5_YELLOW_GPIO_Port GPIOD
#define RGB_B_Pin GPIO_PIN_3
#define RGB_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct
{
	uint8_t _1msn;
	uint8_t _10msn;
	uint8_t _50msn;
	uint8_t _100msn;
	uint8_t _250msn;
	uint8_t _500msn;
	uint8_t _750msn;
	uint8_t _1sn;
	uint8_t _2sn;
	uint8_t _5sn;

}SystemClockTimer_t;
extern SystemClockTimer_t SysClkTim;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
