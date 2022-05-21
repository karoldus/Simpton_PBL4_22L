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
#include "stm32l0xx_hal.h"

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
#define GPIO_BU_POUT_Pin GPIO_PIN_0
#define GPIO_BU_POUT_GPIO_Port GPIOA
#define GPIO_BU_POUT_EXTI_IRQn EXTI0_1_IRQn
#define GPIO_BU_TOUT_Pin GPIO_PIN_1
#define GPIO_BU_TOUT_GPIO_Port GPIOA
#define GPIO_BU_TOUT_EXTI_IRQn EXTI0_1_IRQn
#define GPIO_LED_G_Pin GPIO_PIN_4
#define GPIO_LED_G_GPIO_Port GPIOA
#define GPIO_LED_R_Pin GPIO_PIN_5
#define GPIO_LED_R_GPIO_Port GPIOA
#define GPIO_LED_B_Pin GPIO_PIN_6
#define GPIO_LED_B_GPIO_Port GPIOA
#define GPIO_ZAS_ALRT_Pin GPIO_PIN_8
#define GPIO_ZAS_ALRT_GPIO_Port GPIOA
#define GPIO_ZAS_ALRT_EXTI_IRQn EXTI4_15_IRQn
#define ZAS_SCL_Pin GPIO_PIN_9
#define ZAS_SCL_GPIO_Port GPIOA
#define ZAS_SDA_Pin GPIO_PIN_10
#define ZAS_SDA_GPIO_Port GPIOA
#define GPIO_BLE_TX_IND_Pin GPIO_PIN_3
#define GPIO_BLE_TX_IND_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
