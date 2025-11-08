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
#include "stm32f4xx_hal.h"

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
#define M1_PWM_Pin GPIO_PIN_2
#define M1_PWM_GPIO_Port GPIOE
#define M2_PWM_Pin GPIO_PIN_3
#define M2_PWM_GPIO_Port GPIOE
#define BAT_VOT_Pin GPIO_PIN_1
#define BAT_VOT_GPIO_Port GPIOC
#define KEY_1_Pin GPIO_PIN_4
#define KEY_1_GPIO_Port GPIOA
#define KEY_2_Pin GPIO_PIN_5
#define KEY_2_GPIO_Port GPIOA
#define SYS_CURRENT_Pin GPIO_PIN_0
#define SYS_CURRENT_GPIO_Port GPIOB
#define SYS_CONTROL_Pin GPIO_PIN_7
#define SYS_CONTROL_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOD
#define AIN2_Pin GPIO_PIN_0
#define AIN2_GPIO_Port GPIOE
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
