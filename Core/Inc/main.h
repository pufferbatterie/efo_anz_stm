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
#include "stm32u0xx_hal.h"

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
#define adc0_Voltage_Pin GPIO_PIN_0
#define adc0_Voltage_GPIO_Port GPIOC
#define adc3_Vdd_Pin GPIO_PIN_3
#define adc3_Vdd_GPIO_Port GPIOC
#define NH_right_Pin GPIO_PIN_1
#define NH_right_GPIO_Port GPIOB
#define NH_right_EXTI_IRQn EXTI0_1_IRQn
#define NH_down_Pin GPIO_PIN_2
#define NH_down_GPIO_Port GPIOB
#define NH_down_EXTI_IRQn EXTI2_3_IRQn
#define NH_up_Pin GPIO_PIN_10
#define NH_up_GPIO_Port GPIOB
#define NH_up_EXTI_IRQn EXTI4_15_IRQn
#define NH_enter_Pin GPIO_PIN_11
#define NH_enter_GPIO_Port GPIOB
#define NH_enter_EXTI_IRQn EXTI4_15_IRQn
#define NH_left_Pin GPIO_PIN_12
#define NH_left_GPIO_Port GPIOB
#define NH_left_EXTI_IRQn EXTI4_15_IRQn
#define LED_RIGHT_3_Pin GPIO_PIN_15
#define LED_RIGHT_3_GPIO_Port GPIOB
#define LED_RIGHT_2_Pin GPIO_PIN_6
#define LED_RIGHT_2_GPIO_Port GPIOC
#define LED_RIGHT_1_Pin GPIO_PIN_7
#define LED_RIGHT_1_GPIO_Port GPIOC
#define LED_LEFT_3_Pin GPIO_PIN_8
#define LED_LEFT_3_GPIO_Port GPIOC
#define LED_LEFT_2_Pin GPIO_PIN_9
#define LED_LEFT_2_GPIO_Port GPIOC
#define LED_LEFT_1_Pin GPIO_PIN_8
#define LED_LEFT_1_GPIO_Port GPIOA
#define LED_LCD_Pin GPIO_PIN_6
#define LED_LCD_GPIO_Port GPIOB
#define LCD_NRESET_Pin GPIO_PIN_7
#define LCD_NRESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
