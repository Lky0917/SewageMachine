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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

  void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTO1_BK_Pin GPIO_PIN_13
#define MOTO1_BK_GPIO_Port GPIOC

#define MOTO1_DIR_Pin GPIO_PIN_0
#define MOTO1_DIR_GPIO_Port GPIOC

#define MOTO4_BK_Pin GPIO_PIN_2
#define MOTO4_BK_GPIO_Port GPIOC

#define MOTO4_DIR_Pin GPIO_PIN_3
#define MOTO4_DIR_GPIO_Port GPIOC

#define KEY_IN_OFF_Pin GPIO_PIN_0
#define KEY_IN_OFF_GPIO_Port GPIOA
#define LED_POWER_Pin GPIO_PIN_7
#define LED_POWER_GPIO_Port GPIOA
#define LED_SOC1_Pin GPIO_PIN_4
#define LED_SOC1_GPIO_Port GPIOC
#define LED_SOC2_Pin GPIO_PIN_5
#define LED_SOC2_GPIO_Port GPIOC
#define LED_SOC3_Pin GPIO_PIN_0
#define LED_SOC3_GPIO_Port GPIOB
#define LED_SOC4_Pin GPIO_PIN_1
#define LED_SOC4_GPIO_Port GPIOB
#define QMI8658A_INT_Pin GPIO_PIN_2
#define QMI8658A_INT_GPIO_Port GPIOB

#define MOTO3_BK_Pin GPIO_PIN_14
#define MOTO3_BK_GPIO_Port GPIOB

#define MOTO3_DIR_Pin GPIO_PIN_15
#define MOTO3_DIR_GPIO_Port GPIOB

#define MOTO2_DIR_Pin GPIO_PIN_9
#define MOTO2_DIR_GPIO_Port GPIOC

#define MOTO4_EN_Pin GPIO_PIN_8
#define MOTO4_EN_GPIO_Port GPIOA

#define MOTO3_EN_Pin GPIO_PIN_10
#define MOTO3_EN_GPIO_Port GPIOA
#define MOTO2_BK_Pin GPIO_PIN_12
#define MOTO2_BK_GPIO_Port GPIOA
#define HIT_IN1_Pin GPIO_PIN_2
#define HIT_IN1_GPIO_Port GPIOD
#define HIT_IN2_Pin GPIO_PIN_3
#define HIT_IN2_GPIO_Port GPIOB
#define HIT_IN3_Pin GPIO_PIN_4
#define HIT_IN3_GPIO_Port GPIOB
#define HIT_IN4_Pin GPIO_PIN_5
#define HIT_IN4_GPIO_Port GPIOB

  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
