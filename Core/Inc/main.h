/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOC
#define S0_Pin GPIO_PIN_15
#define S0_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOA
#define SIG_Pin GPIO_PIN_3
#define SIG_GPIO_Port GPIOA
#define BL_Pin GPIO_PIN_4
#define BL_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_5
#define CLK_GPIO_Port GPIOA
#define DIN_Pin GPIO_PIN_7
#define DIN_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_0
#define DC_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_1
#define CE_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_2
#define RST_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_9
#define AI2_GPIO_Port GPIOA
#define AI1_Pin GPIO_PIN_10
#define AI1_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_11
#define STBY_GPIO_Port GPIOA
#define BI1_Pin GPIO_PIN_12
#define BI1_GPIO_Port GPIOA
#define BI2_Pin GPIO_PIN_15
#define BI2_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_3
#define PWM_B_GPIO_Port GPIOB
#define BOT2_Pin GPIO_PIN_8
#define BOT2_GPIO_Port GPIOB
#define BOT2_EXTI_IRQn EXTI9_5_IRQn
#define BOT1_Pin GPIO_PIN_9
#define BOT1_GPIO_Port GPIOB
#define BOT1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
