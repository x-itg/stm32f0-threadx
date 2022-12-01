/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define CIN3_Pin GPIO_PIN_13
#define CIN3_GPIO_Port GPIOC
#define CIN4_Pin GPIO_PIN_14
#define CIN4_GPIO_Port GPIOC
#define USART2_DE_Pin GPIO_PIN_4
#define USART2_DE_GPIO_Port GPIOA
#define MOT3_EN_Pin GPIO_PIN_7
#define MOT3_EN_GPIO_Port GPIOA
#define MOT3_IN1_Pin GPIO_PIN_1
#define MOT3_IN1_GPIO_Port GPIOB
#define MOT3_Fault_Pin GPIO_PIN_2
#define MOT3_Fault_GPIO_Port GPIOB
#define LED_ERR_Pin GPIO_PIN_10
#define LED_ERR_GPIO_Port GPIOB
#define LED_RUN_Pin GPIO_PIN_11
#define LED_RUN_GPIO_Port GPIOB
#define MOT1_Fault_Pin GPIO_PIN_15
#define MOT1_Fault_GPIO_Port GPIOB
#define MOT1_EN_Pin GPIO_PIN_8
#define MOT1_EN_GPIO_Port GPIOA
#define MOT1_IN1_Pin GPIO_PIN_10
#define MOT1_IN1_GPIO_Port GPIOA
#define MOT2_IN1_Pin GPIO_PIN_12
#define MOT2_IN1_GPIO_Port GPIOA
#define MOT2_EN_Pin GPIO_PIN_6
#define MOT2_EN_GPIO_Port GPIOF
#define MOT2_Fault_Pin GPIO_PIN_7
#define MOT2_Fault_GPIO_Port GPIOF
#define SIN4_Pin GPIO_PIN_15
#define SIN4_GPIO_Port GPIOA
#define SIN3_Pin GPIO_PIN_3
#define SIN3_GPIO_Port GPIOB
#define SIN2_Pin GPIO_PIN_4
#define SIN2_GPIO_Port GPIOB
#define SIN1_Pin GPIO_PIN_5
#define SIN1_GPIO_Port GPIOB
#define CIN1_Pin GPIO_PIN_8
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_9
#define CIN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
