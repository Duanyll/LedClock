/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_ONBOARD_Pin GPIO_PIN_13
#define LED_ONBOARD_GPIO_Port GPIOC
#define BTN_1_Pin GPIO_PIN_3
#define BTN_1_GPIO_Port GPIOA
#define BTN_2_Pin GPIO_PIN_4
#define BTN_2_GPIO_Port GPIOA
#define BTN_3_Pin GPIO_PIN_5
#define BTN_3_GPIO_Port GPIOA
#define BTN_4_Pin GPIO_PIN_6
#define BTN_4_GPIO_Port GPIOA
#define LED_IN_1_Pin GPIO_PIN_0
#define LED_IN_1_GPIO_Port GPIOB
#define LED_IN_2_Pin GPIO_PIN_1
#define LED_IN_2_GPIO_Port GPIOB
#define LED_IN_3_Pin GPIO_PIN_10
#define LED_IN_3_GPIO_Port GPIOB
#define LED_IN_4_Pin GPIO_PIN_11
#define LED_IN_4_GPIO_Port GPIOB
#define LED_OUT_12_Pin GPIO_PIN_12
#define LED_OUT_12_GPIO_Port GPIOB
#define LED_OUT_9_Pin GPIO_PIN_13
#define LED_OUT_9_GPIO_Port GPIOB
#define LED_OUT_8_Pin GPIO_PIN_14
#define LED_OUT_8_GPIO_Port GPIOB
#define LED_OUT_6_Pin GPIO_PIN_15
#define LED_OUT_6_GPIO_Port GPIOB
#define LED_IN_11_Pin GPIO_PIN_3
#define LED_IN_11_GPIO_Port GPIOB
#define LED_IN_10_Pin GPIO_PIN_4
#define LED_IN_10_GPIO_Port GPIOB
#define LED_IN_7_Pin GPIO_PIN_5
#define LED_IN_7_GPIO_Port GPIOB
#define LED_IN_5_Pin GPIO_PIN_6
#define LED_IN_5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
