/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#define EEPROM_ADDRESS_START	0x08080000
#define PULSE_PER_360 			720

uint32_t imp_count;
uint32_t gradus;
signed int dir_gradus;
uint32_t isPushCW;
uint32_t man_azimuth;
signed int dir_azimuth;
uint32_t time_on_cw;
uint32_t time_on_ccw;
char str_rx[10];
char str_tx[10];
uint8_t flag_stop;
uint8_t flag_status;
uint8_t flag_move;

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
#define BTN_UP_Pin LL_GPIO_PIN_14
#define BTN_UP_GPIO_Port GPIOC
#define BTN_DWN_Pin LL_GPIO_PIN_15
#define BTN_DWN_GPIO_Port GPIOC
#define OE_Pin LL_GPIO_PIN_1
#define OE_GPIO_Port GPIOA
#define LED_Pin LL_GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define CW_Pin LL_GPIO_PIN_6
#define CW_GPIO_Port GPIOA
#define CCW_Pin LL_GPIO_PIN_7
#define CCW_GPIO_Port GPIOA
#define BTN_START_Pin LL_GPIO_PIN_1
#define BTN_START_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
