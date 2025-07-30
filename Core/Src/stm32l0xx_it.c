/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern uint32_t imp_count;
extern uint32_t gradus;
extern signed int dir_gradus;
extern signed int dir_azimuth;
extern char str_rx[10];
extern char str_tx[10];
extern uint32_t time_on_cw;
extern uint32_t time_on_ccw;
extern uint8_t flag_stop;
extern uint8_t flag_status;
extern uint8_t flag_move;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);

        const uint32_t max_count = (PULSE_PER_360)/(timer_preload+1) - 1;

        if (dir_azimuth > dir_gradus)
        {
            imp_count = (imp_count < max_count) ? imp_count + 1 : 0;
        }
        else if (dir_azimuth < dir_gradus)
        {
            imp_count = (imp_count > 0) ? imp_count - 1 : max_count;
        }

        gradus = imp_count * step;
        dir_gradus = (gradus >= 180) ? gradus - 360 : gradus;

        uint32_t current_time = HAL_GetTick();
        time_on_cw = current_time;
        time_on_ccw = current_time;
    }
}

/**
  * @brief This function handles TIM21 global interrupt.
  */
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM21)) {
				LL_TIM_ClearFlag_UPDATE(TIM21);
				LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			}
  /* USER CODE END TIM21_IRQn 0 */
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    LL_USART_DisableIT_RXNE(USART2);

    static uint8_t i = 0;
    char letter = LL_USART_ReceiveData8(USART2);

    if (letter != '\n') {
        str_rx[i] = letter;
        i = (i < 8) ? i + 1 : 0;
    }
    else {
        str_rx[i] = '\0';
        i = 0;

        uint8_t cmd = str_rx[0];
        flag_stop   = (cmd == 'S');
        flag_move   = (cmd == 'M');
        flag_status = (cmd == 'C');
    }

    LL_USART_EnableIT_RXNE(USART2);
    /* USER CODE END USART2_IRQn 0 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
