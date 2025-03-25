/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "lis2dw12_reg.h"
#include "ssd1306.h"
#include "stdio.h"
#include "string.h"
#include "ssd1306_tests.h"
#include "DigitalInputOutputs.h"

double Termistor(uint32_t analogValue)
{
double temperature;
uint32_t adcval = 4096 - analogValue;

temperature = log((adcval * 10000) / (4095 - adcval));
temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
temperature = temperature - 273.15;
return temperature;
}
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t counter = 0;
uint8_t rgbcounter = 0;
double Temp1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define 	SENSOR_BUS 	hi2c1
#define 	BOOT_TIME 20 //ms
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
volatile uint64_t SYSTickTimer = 0; //Sistemin zamanlayıcısının (SysTick) sayaç değeri için kullanılır. Zamanlayıcı tabanlı olayları izlemek için kullanılır.

stmdev_ctx_t dev_ctx; // Bir sensör veya cihazla ilgili işlemleri yönetmek için kullanılan bir yapı.

static int16_t data_raw_acceleration[3]; // İvmeölçerden gelen ham ivme verilerini tutar.
static float acceleration_mg[3]; //Ham ivme verilerini milig cinsine çevirip saklar
static uint8_t whoamI, rst; //Bir sensörün veya cihazın sıfırlama durumunu izlemek için kullanılır.
SystemClockTimer_t SysClkTim;  // Gerçek tanımlama burada!
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef ret = HAL_ERROR;//ret değişkeni, HAL fonksiyonlarının sonuçlarını kontrol etmek için kullanılır.

uint8_t rec_buf[10] =
{ 0 }; //rec_buf, veri almak veya iletmek için kullanılan bir tamponu temsil eder.
volatile uint32_t adcValues[10]; //adcValues, ADC'den alınan verileri saklamak için kullanılan bir dizi olup, volatile anahtar kelimesi ile donanım tarafından değişebileceğini belirtir.
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//  ssd1306_TestDrawBitmap();//Ekranın doğru şekilde çalıştığını test etmek için bir bitmap veya test görüntüsü çizer.
//  ssd1306_Init();// OLED ekran modülünü başlatır ve gerekli yapılandırmaları yapar.
//  Buzzer_Control(1);
//  HAL_Delay(200);
//  Buzzer_Control(0);
//  HAL_Delay(200);
//  ssd1306_SetCursor(15, 2);
//  ssd1306_WriteString("DEMSAY", Font_16x26, White);
//  ssd1306_SetCursor(14, 27);
//  ssd1306_WriteString("EDUCATION", Font_11x18, White);
//  ssd1306_SetCursor(45, 46);
//  ssd1306_WriteString("KIT", Font_11x18, White);
//  ssd1306_UpdateScreen();
//  HAL_Delay(3000);
//  ssd1306_Fill(Black);
//  RGB_LED_Control(1, 0, 0);
//  LED_Control(1, 0, 0, 0, 0);
//  HAL_Delay(200);
//  RGB_LED_Control(0, 1, 0);
//  LED_Control(0, 1, 0, 0, 0);
//  HAL_Delay(200);
//  RGB_LED_Control(0, 0, 1);
//  LED_Control(0, 1, 0, 0, 0);
//  HAL_Delay(200);
//  RGB_LED_Control(1, 0, 1);
//  LED_Control(0, 0, 1, 0, 0);
//  HAL_Delay(200);
//  RGB_LED_Control(1, 1, 0);
//  LED_Control(0, 0, 0, 1, 0);
//  HAL_Delay(200);
//  RGB_LED_Control(1, 1, 1);
//  LED_Control(0, 0, 0, 0, 1);
//  HAL_Delay(200);
//  RGB_LED_Control(0, 0, 0);
//  LED_Control(1, 1, 1, 1, 1);
//  HAL_Delay(200);
//  LED_Control(0, 0, 0, 0, 0);
//  Buzzer_Control(1);
//  HAL_Delay(200);
//  Buzzer_Control(0);
//  HAL_Delay(200);

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  lis2dw12_device_id_get(&dev_ctx, &whoamI);
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do
  {
  lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
  /* Configure filtering chain
   * Accelerometer - filter path / bandwidth
   */
  lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
  lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
  /* Configure power mode */
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t reg;
	   	 		/* Read output only if new value is available */
	   	 		lis2dw12_flag_data_ready_get(&dev_ctx, &reg);

	   	 		if (reg)
	   	 		{
	   	 			/* Read acceleration data */
	   	 			memset(data_raw_acceleration, 0, sizeof(data_raw_acceleration));
	   	 			lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);				//reading values from the accelerometer
	   	 			acceleration_mg[0] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[0]);
	   	 			acceleration_mg[1] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[1]);
	   	 			acceleration_mg[2] = lis2dw12_from_fs2_to_mg(data_raw_acceleration[2]);
	   	 		}
	   	 		if (SysClkTim._1sn == 1)
	   	 		{
	   	 			HAL_ADC_Start(&hadc1);
	   	 			HAL_ADC_PollForConversion(&hadc1, 50); 							//reading values from the Temperature sensor with ADC
	   	 			adcValues[2] = HAL_ADC_GetValue(&hadc1);
	   	 			HAL_ADC_Stop(&hadc1);
	   	 			Temp1 = Termistor(adcValues[2]) - 10;

	   	 			HAL_GPIO_TogglePin(LED1_RED_GPIO_Port, LED1_RED_Pin);
	   	 			SysClkTim._1sn = 0;
	   	 		}
	   	 		if (SysClkTim._50msn == 1)
	   	 		{
	   	 			HAL_GPIO_TogglePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin);
	   	 			SysClkTim._50msn = 0;
	   	 		}
	   	 		if (SysClkTim._100msn == 1)
	   	 		{

	   	 			char str[10] =
	   	 			{ 0 };
	   	 			char Tmp[10] =
	   	 			{ 0 };
	   	 			sprintf(str, "%.2f m/s2", acceleration_mg[2] / 100);
	   	 			HAL_GPIO_TogglePin(LED3_BLUE_GPIO_Port, LED3_BLUE_Pin);

	   	 			ssd1306_SetCursor(2, 0);
	   	 			ssd1306_WriteString("G:", Font_11x18, White);
	   	 			ssd1306_SetCursor(25, 0);
	   	 			ssd1306_WriteString(str, Font_11x18, White);								// printing the values from the accelerometer
	   	 			ssd1306_UpdateScreen();
	   	 			sprintf(Tmp, "%.2fC", Temp1);
	   	 			ssd1306_SetCursor(2, 25);
	   	 			ssd1306_WriteString("Temp:", Font_11x18, White);
	   	 			ssd1306_SetCursor(55, 25);
	   	 			ssd1306_WriteString(Tmp, Font_11x18, White);								// printing the values from the Temperature Sensor (NTC)

	   	 			SysClkTim._100msn = 0;
	   	 		}
	   	 		if (SysClkTim._250msn == 1)
	   	 		{
	   	 			HAL_GPIO_TogglePin(LED4_WHITE_GPIO_Port, LED4_WHITE_Pin);
	   	 			SysClkTim._250msn = 0;
	   	 		}

	   	 		if (SysClkTim._500msn == 1)
	   	 		{
	   	 			counter++;
	   	 			if (counter % 2 == 1)
	   	 				HAL_GPIO_WritePin(LED5_YELLOW_GPIO_Port, LED5_YELLOW_Pin, 1);
	   	 			else
	   	 				HAL_GPIO_WritePin(LED5_YELLOW_GPIO_Port, LED5_YELLOW_Pin, 0);

	   	 			if (counter > 2)
	   	 			{
	   	 				counter = 0;
	   	 			}

	   	 			SysClkTim._500msn = 0;
	   	 		}

	   	 		if (SysClkTim._2sn == 1)
	   	 		{
	   	 			rgbcounter++;
	   	 			switch (rgbcounter)
	   	 			{
	   	 			case 0:
	   	 				HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 1);
	   	 				HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 0);
	   	 				break;
	   	 			case 1:
	   	 				HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 1);
	   	 				HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 0);
	   	 				break;
	   	 			case 2:
	   	 				HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 1);
	   	 				break;
	   	 			case 3:
	   	 				HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 1);
	   	 				HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 1);
	   	 				HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 1);
	   	 				break;
	   	 			case 4:
	   	 				HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, 0);
	   	 				HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, 0);
	   	 				break;
	   	 			}
	   	 			if (rgbcounter > 4)
	   	 			{
	   	 				rgbcounter = 0;
	   	 			}

	   	 			SysClkTim._2sn = 0;
	   	 		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_R_Pin|RGB_G_Pin|LED1_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|RGB_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_GREEN_Pin|LED3_BLUE_Pin|LED4_WHITE_Pin|LED5_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGB_R_Pin RGB_G_Pin LED1_RED_Pin */
  GPIO_InitStruct.Pin = RGB_R_Pin|RGB_G_Pin|LED1_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin RGB_B_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|RGB_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_GREEN_Pin LED3_BLUE_Pin LED4_WHITE_Pin LED5_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED2_GREEN_Pin|LED3_BLUE_Pin|LED4_WHITE_Pin|LED5_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// SysTick Zamanlayıcı Geri Çağırma Fonksiyonu
void HAL_SYSTICK_Callback(void)
{
SYSTickTimer++;

if (SYSTickTimer % 1 == 0)
{
SysClkTim._1msn = 1;
}
if (SYSTickTimer % 10 == 0)
{
SysClkTim._10msn = 1;
}
if (SYSTickTimer % 50 == 0)
{
SysClkTim._50msn = 1;
}
if (SYSTickTimer % 100 == 0)
{
SysClkTim._100msn = 1;
}
if (SYSTickTimer % 250 == 0)
{
SysClkTim._250msn = 1;
}
if (SYSTickTimer % 500 == 0)
{
SysClkTim._500msn = 1;
}
if (SYSTickTimer % 1000 == 0)
{
SysClkTim._1sn = 1;
}
if (SYSTickTimer % 2000 == 0)
{
SysClkTim._2sn = 1;
}
if (SYSTickTimer % 5000 == 0)
{
SysClkTim._5sn = 1;
}

}
//platform_write ve platform_read Fonksiyonları
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

return 0;
}
/*
* @brief Read generic device register (platform dependent)
*
* @param handle customizable argument. In this examples is used in
* order to select the correct sensor bus handler.
* @param reg register to read
* @param bufp pointer to buffer that store the data read
* @param len number of consecutive register to read
*
*/static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
