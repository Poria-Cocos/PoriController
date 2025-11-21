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
#define Toggle2A_Pin GPIO_PIN_13
#define Toggle2A_GPIO_Port GPIOC
#define Toggle2B_Pin GPIO_PIN_14
#define Toggle2B_GPIO_Port GPIOC
#define Toggle1A_Pin GPIO_PIN_15
#define Toggle1A_GPIO_Port GPIOC
#define Toggle1B_Pin GPIO_PIN_0
#define Toggle1B_GPIO_Port GPIOC
#define SW9_Pin GPIO_PIN_1
#define SW9_GPIO_Port GPIOC
#define SW8_Pin GPIO_PIN_2
#define SW8_GPIO_Port GPIOC
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_0
#define SW3_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_1
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOA
#define RockerLY_Pin GPIO_PIN_3
#define RockerLY_GPIO_Port GPIOA
#define RockerLSW_Pin GPIO_PIN_5
#define RockerLSW_GPIO_Port GPIOA
#define RockerLX_Pin GPIO_PIN_7
#define RockerLX_GPIO_Port GPIOA
#define EncoderLB_Pin GPIO_PIN_0
#define EncoderLB_GPIO_Port GPIOB
#define EncoderLA_Pin GPIO_PIN_1
#define EncoderLA_GPIO_Port GPIOB
#define EncoderLSW_Pin GPIO_PIN_2
#define EncoderLSW_GPIO_Port GPIOB
#define RockerRX_Pin GPIO_PIN_11
#define RockerRX_GPIO_Port GPIOB
#define RockerRY_Pin GPIO_PIN_13
#define RockerRY_GPIO_Port GPIOB
#define RockerRSW_Pin GPIO_PIN_15
#define RockerRSW_GPIO_Port GPIOB
#define EncoderRSW_Pin GPIO_PIN_6
#define EncoderRSW_GPIO_Port GPIOC
#define EncoderRB_Pin GPIO_PIN_7
#define EncoderRB_GPIO_Port GPIOC
#define EncoderRA_Pin GPIO_PIN_8
#define EncoderRA_GPIO_Port GPIOC
#define M0_Pin GPIO_PIN_9
#define M0_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_8
#define M1_GPIO_Port GPIOA
#define AUX_Pin GPIO_PIN_11
#define AUX_GPIO_Port GPIOA
#define SW6_Pin GPIO_PIN_12
#define SW6_GPIO_Port GPIOA
#define SW7_Pin GPIO_PIN_15
#define SW7_GPIO_Port GPIOA
#define SW5_Pin GPIO_PIN_10
#define SW5_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_11
#define SW4_GPIO_Port GPIOC
#define SW11_Pin GPIO_PIN_12
#define SW11_GPIO_Port GPIOC
#define SW10_Pin GPIO_PIN_2
#define SW10_GPIO_Port GPIOD
#define Toggle4B_Pin GPIO_PIN_5
#define Toggle4B_GPIO_Port GPIOB
#define Toggle4A_Pin GPIO_PIN_6
#define Toggle4A_GPIO_Port GPIOB
#define Toggle3B_Pin GPIO_PIN_7
#define Toggle3B_GPIO_Port GPIOB
#define Toggle3A_Pin GPIO_PIN_9
#define Toggle3A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
