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
#include "stm32g4xx_hal.h"

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
#define HC4067_S0_Pin GPIO_PIN_13
#define HC4067_S0_GPIO_Port GPIOC
#define STDBY_Pin GPIO_PIN_14
#define STDBY_GPIO_Port GPIOC
#define CHRG_Pin GPIO_PIN_15
#define CHRG_GPIO_Port GPIOC
#define VBAT_ADC_Pin GPIO_PIN_0
#define VBAT_ADC_GPIO_Port GPIOA
#define CH15_Pin GPIO_PIN_1
#define CH15_GPIO_Port GPIOA
#define CH14_Pin GPIO_PIN_2
#define CH14_GPIO_Port GPIOA
#define CH13_Pin GPIO_PIN_3
#define CH13_GPIO_Port GPIOA
#define CH12_Pin GPIO_PIN_4
#define CH12_GPIO_Port GPIOA
#define CH11_Pin GPIO_PIN_5
#define CH11_GPIO_Port GPIOA
#define CH10_Pin GPIO_PIN_6
#define CH10_GPIO_Port GPIOA
#define CH9_Pin GPIO_PIN_7
#define CH9_GPIO_Port GPIOA
#define CH8_Pin GPIO_PIN_4
#define CH8_GPIO_Port GPIOC
#define CH7_Pin GPIO_PIN_0
#define CH7_GPIO_Port GPIOB
#define CH6_Pin GPIO_PIN_1
#define CH6_GPIO_Port GPIOB
#define CH5_Pin GPIO_PIN_2
#define CH5_GPIO_Port GPIOB
#define CH4_Pin GPIO_PIN_10
#define CH4_GPIO_Port GPIOB
#define CH3_Pin GPIO_PIN_12
#define CH3_GPIO_Port GPIOB
#define CH2_Pin GPIO_PIN_13
#define CH2_GPIO_Port GPIOB
#define CH1_Pin GPIO_PIN_14
#define CH1_GPIO_Port GPIOB
#define CH0_Pin GPIO_PIN_15
#define CH0_GPIO_Port GPIOB
#define HC4067_S1_Pin GPIO_PIN_6
#define HC4067_S1_GPIO_Port GPIOC
#define POWER_KEY_Pin GPIO_PIN_11
#define POWER_KEY_GPIO_Port GPIOA
#define POWER_CTRL_Pin GPIO_PIN_12
#define POWER_CTRL_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_10
#define LED_R_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOB
#define HC4067_S3_Pin GPIO_PIN_6
#define HC4067_S3_GPIO_Port GPIOB
#define HC4067_EN_Pin GPIO_PIN_7
#define HC4067_EN_GPIO_Port GPIOB
#define HC4067_S2_Pin GPIO_PIN_9
#define HC4067_S2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
