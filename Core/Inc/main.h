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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Buttom1_Pin GPIO_PIN_15
#define Buttom1_GPIO_Port GPIOC
#define Buttom3_Pin GPIO_PIN_0
#define Buttom3_GPIO_Port GPIOB
#define Buttom2_Pin GPIO_PIN_1
#define Buttom2_GPIO_Port GPIOB
#define GrayL2_Pin GPIO_PIN_12
#define GrayL2_GPIO_Port GPIOB
#define GrayL1_Pin GPIO_PIN_13
#define GrayL1_GPIO_Port GPIOB
#define GrayM_Pin GPIO_PIN_14
#define GrayM_GPIO_Port GPIOB
#define GrayR2_Pin GPIO_PIN_15
#define GrayR2_GPIO_Port GPIOB
#define GrayR1_Pin GPIO_PIN_11
#define GrayR1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
