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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
void SystemClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI4_IRQn
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOF
#define LED0_Pin GPIO_PIN_10
#define LED0_GPIO_Port GPIOF
#define IOUTA_Pin GPIO_PIN_0
#define IOUTA_GPIO_Port GPIOC
#define IOUTB_Pin GPIO_PIN_1
#define IOUTB_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_2
#define VBUS_GPIO_Port GPIOC
#define CBUS_Pin GPIO_PIN_3
#define CBUS_GPIO_Port GPIOC
#define KEYUP_Pin GPIO_PIN_0
#define KEYUP_GPIO_Port GPIOA
#define KEYUP_EXTI_IRQn EXTI0_IRQn
#define NTC_Pin GPIO_PIN_4
#define NTC_GPIO_Port GPIOC
#define FAULT_Pin GPIO_PIN_7
#define FAULT_GPIO_Port GPIOE
#define INLA_Pin GPIO_PIN_8
#define INLA_GPIO_Port GPIOE
#define INHA_Pin GPIO_PIN_9
#define INHA_GPIO_Port GPIOE
#define INLB_Pin GPIO_PIN_10
#define INLB_GPIO_Port GPIOE
#define INHB_Pin GPIO_PIN_11
#define INHB_GPIO_Port GPIOE
#define INLC_Pin GPIO_PIN_12
#define INLC_GPIO_Port GPIOE
#define INHC_Pin GPIO_PIN_13
#define INHC_GPIO_Port GPIOE
#define DRV_EN_Pin GPIO_PIN_14
#define DRV_EN_GPIO_Port GPIOE
#define CAL_Pin GPIO_PIN_15
#define CAL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
