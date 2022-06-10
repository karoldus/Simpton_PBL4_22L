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

#define RFID_TIMEOUT	10000 // 10 s
#define BLE_TIMEOUT		10000 // 10 s

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

enum ProgramState {
	SLEEP_STATE,
	WAKING_UP_STATE,
	WAITING_FOR_RFID_STATE,
	RFID_FOUND_STATE,
	RFID_TIMEOUT_STATE,
	//PREPARE_TO_SLEEP_STATE
};

struct StateMachine {
	enum ProgramState programState;
	uint32_t RFIDStartTime;
	uint32_t BLEStartTime;
};


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void waking_up();
void rfid_found();
void rfid_not_found();
void ble_found();
void ble_not_found();
void prepare_to_sleep();



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_BU_POUT_Pin GPIO_PIN_0
#define GPIO_BU_POUT_GPIO_Port GPIOA
#define GPIO_BU_POUT_EXTI_IRQn EXTI0_1_IRQn
#define GPIO_BU_TOUT_Pin GPIO_PIN_1
#define GPIO_BU_TOUT_GPIO_Port GPIOA
#define GPIO_LED_B_Pin GPIO_PIN_4
#define GPIO_LED_B_GPIO_Port GPIOA
#define GPIO_LED_R_Pin GPIO_PIN_5
#define GPIO_LED_R_GPIO_Port GPIOA
#define GPIO_LED_G_Pin GPIO_PIN_6
#define GPIO_LED_G_GPIO_Port GPIOA
#define GPIO_RFID_MODU_Pin GPIO_PIN_7
#define GPIO_RFID_MODU_GPIO_Port GPIOA
#define GPIO_RFID_DATA_Pin GPIO_PIN_0
#define GPIO_RFID_DATA_GPIO_Port GPIOB
#define GPIO_RFID_CLK_Pin GPIO_PIN_1
#define GPIO_RFID_CLK_GPIO_Port GPIOB
#define GPIO_RFID_CLK_EXTI_IRQn EXTI0_1_IRQn
#define GPIO_BLE_TX_IND_Pin GPIO_PIN_3
#define GPIO_BLE_TX_IND_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
