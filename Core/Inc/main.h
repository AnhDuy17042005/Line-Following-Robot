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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define AN1_Pin GPIO_PIN_0
#define AN1_GPIO_Port GPIOA
#define AN2_Pin GPIO_PIN_1
#define AN2_GPIO_Port GPIOA
#define AN3_Pin GPIO_PIN_2
#define AN3_GPIO_Port GPIOA
#define AN4_Pin GPIO_PIN_3
#define AN4_GPIO_Port GPIOA
#define AN5_Pin GPIO_PIN_4
#define AN5_GPIO_Port GPIOA
#define R_CH2_Pin GPIO_PIN_6
#define R_CH2_GPIO_Port GPIOA
#define R_CH1_Pin GPIO_PIN_7
#define R_CH1_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_1
#define STBY_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOB
#define RX_Pin GPIO_PIN_11
#define RX_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_12
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_13
#define BIN2_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_14
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_15
#define AIN1_GPIO_Port GPIOB
#define L_CH1_Pin GPIO_PIN_8
#define L_CH1_GPIO_Port GPIOA
#define L_CH2_Pin GPIO_PIN_9
#define L_CH2_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define PWM_R_Pin GPIO_PIN_8
#define PWM_R_GPIO_Port GPIOB
#define PWM_L_Pin GPIO_PIN_9
#define PWM_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
