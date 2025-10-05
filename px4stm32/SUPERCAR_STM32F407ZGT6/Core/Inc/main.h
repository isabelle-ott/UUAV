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
#define driver_PWM_1_Pin GPIO_PIN_0
#define driver_PWM_1_GPIO_Port GPIOA
#define driver_PWM_2_Pin GPIO_PIN_1
#define driver_PWM_2_GPIO_Port GPIOA
#define driver_PWM_3_Pin GPIO_PIN_2
#define driver_PWM_3_GPIO_Port GPIOA
#define driver_PWM_4_Pin GPIO_PIN_3
#define driver_PWM_4_GPIO_Port GPIOA
#define driver_eder_2_A_Pin GPIO_PIN_6
#define driver_eder_2_A_GPIO_Port GPIOA
#define driver_eder_2_B_Pin GPIO_PIN_7
#define driver_eder_2_B_GPIO_Port GPIOA
#define driver_GPIO_1_Pin GPIO_PIN_0
#define driver_GPIO_1_GPIO_Port GPIOB
#define driver_eder_1_A_Pin GPIO_PIN_9
#define driver_eder_1_A_GPIO_Port GPIOE
#define driver__eder_1_B_Pin GPIO_PIN_11
#define driver__eder_1_B_GPIO_Port GPIOE
#define driver_GPIO_4_Pin GPIO_PIN_10
#define driver_GPIO_4_GPIO_Port GPIOB
#define Light_Pin GPIO_PIN_9
#define Light_GPIO_Port GPIOD
#define ARM1_Pin GPIO_PIN_6
#define ARM1_GPIO_Port GPIOC
#define ARM2_Pin GPIO_PIN_7
#define ARM2_GPIO_Port GPIOC
#define ARM3_Pin GPIO_PIN_8
#define ARM3_GPIO_Port GPIOC
#define driver_eder_3_A_Pin GPIO_PIN_15
#define driver_eder_3_A_GPIO_Port GPIOA
#define driver__GPIO_3_Pin GPIO_PIN_0
#define driver__GPIO_3_GPIO_Port GPIOD
#define driver_eder_3_B_Pin GPIO_PIN_3
#define driver_eder_3_B_GPIO_Port GPIOB
#define driver_eder_4_A_Pin GPIO_PIN_6
#define driver_eder_4_A_GPIO_Port GPIOB
#define driver_eder_4_B_Pin GPIO_PIN_7
#define driver_eder_4_B_GPIO_Port GPIOB
#define CAMERA_pwm_Pin GPIO_PIN_8
#define CAMERA_pwm_GPIO_Port GPIOB
#define driver__GPIO_2_Pin GPIO_PIN_0
#define driver__GPIO_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
