/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define LGMRA_Pin GPIO_PIN_0
#define LGMRA_GPIO_Port GPIOA
#define LGMRB_Pin GPIO_PIN_1
#define LGMRB_GPIO_Port GPIOA
#define RGMRA_Pin GPIO_PIN_6
#define RGMRA_GPIO_Port GPIOA
#define RGMRB_Pin GPIO_PIN_7
#define RGMRB_GPIO_Port GPIOA
#define MPU6050_INT_Pin GPIO_PIN_5
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI9_5_IRQn
#define SERVO_PWM_Pin GPIO_PIN_6
#define SERVO_PWM_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_10
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_11
#define MPU6050_SDA_GPIO_Port GPIOB
#define LAIN1_Pin GPIO_PIN_12
#define LAIN1_GPIO_Port GPIOB
#define LAIN2_Pin GPIO_PIN_13
#define LAIN2_GPIO_Port GPIOB
#define RAIN1_Pin GPIO_PIN_14
#define RAIN1_GPIO_Port GPIOB
#define RAIN2_Pin GPIO_PIN_15
#define RAIN2_GPIO_Port GPIOB
#define LPWM_Pin GPIO_PIN_8
#define LPWM_GPIO_Port GPIOA
#define RPWM_Pin GPIO_PIN_11
#define RPWM_GPIO_Port GPIOA
#define MOTOR_STBY_Pin GPIO_PIN_8
#define MOTOR_STBY_GPIO_Port GPIOB
#define START_BTN_Pin GPIO_PIN_9
#define START_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
