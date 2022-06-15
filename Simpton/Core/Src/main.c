/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ble_driver.h"
#include "mlx90109cdc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

struct StateMachine stateMachine;

BLE ble_device;

RFID_Data reader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_BU_POUT_Pin && stateMachine.programState == SLEEP_STATE)
  {
	  // button interrupt

	  // SLEEP COMMENT IMPORTANT KAROL
	  SystemClock_Config ();
	  HAL_ResumeTick();
	  HAL_PWR_DisableSleepOnExit();

	  // WAKE UP

	  stateMachine.programState = WAKING_UP_STATE;
  }
  else if (GPIO_Pin == GPIO_RFID_CLK_Pin && stateMachine.programState == WAITING_FOR_RFID_STATE)
  {
	  if(HAL_GetTick() - stateMachine.RFIDStartTime >= RFID_TIMEOUT)
		  HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, 1);
	  // RFID CLK interrupt
	  // read rfid
	  Read_RFID(&reader,&htim21);
  }
}


//------ printf - for debuging

//int __io_putchar(int ch)
//{
//    if (ch == '\n') {
//        uint8_t ch2 = '\r';
//        HAL_UART_Transmit(&hlpuart1, &ch2, 1, HAL_MAX_DELAY);
//    }
//
//    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
//    return 1;
//}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	stateMachine.programState = SLEEP_STATE;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  Init_RFID(&reader, GPIO_RFID_DATA_GPIO_Port, GPIO_RFID_CLK_GPIO_Port, GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_DATA_Pin, GPIO_RFID_CLK_Pin, GPIO_RFID_MODU_Pin);
  HAL_TIM_Base_Start(&htim21);
  BLE_Initialise( &ble_device, &huart2, GPIO_BLE_TX_IND_GPIO_Port, GPIO_BLE_TX_IND_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, 1);

  while (1)
  {

	  switch(stateMachine.programState)
	  {
	  	  case WAKING_UP_STATE:
	  		  waking_up();
	  		  break;

	  	  case WAITING_FOR_RFID_STATE:
	  		  if(reader.tag != 0) // got tag - condition to do
			  {
	  			//HAL_Delay(500); // tests
	  			  rfid_found();
	  			  BLE_Send(&ble_device, "OK\n\r");

			  }
			  else if(HAL_GetTick() - stateMachine.RFIDStartTime >= RFID_TIMEOUT)
			  {
				  // RFID Timeout
				  rfid_not_found();
			  }

	  		  break;

	  	  case RFID_FOUND_STATE:
	  		  //if(BLE_is_connected(&ble_device) == HAL_OK && ble_device.connection == 1) // got connection - condition to do
	  		  if(1) // FAKE BLE TEST
			  {
	  			HAL_Delay(500); // tests
	  			ble_found();
			  }
			  else if(HAL_GetTick() - stateMachine.RFIDStartTime >= BLE_TIMEOUT)
			  {
				  // BLE Timeout
				  ble_not_found();
			  }

	  		  break;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_LED_B_Pin|GPIO_LED_R_Pin|GPIO_LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_BLE_TX_IND_GPIO_Port, GPIO_BLE_TX_IND_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : GPIO_BU_POUT_Pin */
  GPIO_InitStruct.Pin = GPIO_BU_POUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_BU_POUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_BU_TOUT_Pin */
  GPIO_InitStruct.Pin = GPIO_BU_TOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_BU_TOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED_B_Pin GPIO_LED_R_Pin GPIO_LED_G_Pin GPIO_RFID_MODU_Pin */
  GPIO_InitStruct.Pin = GPIO_LED_B_Pin|GPIO_LED_R_Pin|GPIO_LED_G_Pin|GPIO_RFID_MODU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RFID_DATA_Pin */
  GPIO_InitStruct.Pin = GPIO_RFID_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_RFID_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RFID_CLK_Pin */
  GPIO_InitStruct.Pin = GPIO_RFID_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_RFID_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_BLE_TX_IND_Pin */
  GPIO_InitStruct.Pin = GPIO_BLE_TX_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_BLE_TX_IND_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

void waking_up()
{
	//BLE_PowerOn(&ble_device);


	HAL_GPIO_WritePin(GPIO_LED_B_GPIO_Port, GPIO_LED_B_Pin, GPIO_PIN_SET);
	HAL_Delay(300);
	// do sth
	HAL_GPIO_WritePin(GPIO_LED_B_GPIO_Port, GPIO_LED_B_Pin, GPIO_PIN_RESET);
	// do sth

	//HAL_Delay(2000);

	// go to next state
	stateMachine.RFIDStartTime = HAL_GetTick();
	stateMachine.programState = WAITING_FOR_RFID_STATE;

	// turn on RFID
	reader.tag = 0;
	HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, 0);
}

void rfid_found()
{
	HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_RESET);

	stateMachine.BLEStartTime = HAL_GetTick();
	stateMachine.programState = RFID_FOUND_STATE;
}


void rfid_not_found()
{
	HAL_GPIO_WritePin(GPIO_RFID_MODU_GPIO_Port, GPIO_RFID_MODU_Pin, 1);

	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_RESET);

	stateMachine.BLEStartTime = HAL_GetTick();
	stateMachine.programState = RFID_TIMEOUT_STATE;

	prepare_to_sleep();
}

void ble_found()
{
	HAL_Delay(1000);
	char to_send[20] = {'\0'};
	sprintf(to_send, "%x", (reader.tag >> 32));
	BLE_Send(&ble_device, to_send);
	//HAL_UART_Abort(ble_device.uartHandle);
	//HAL_Delay(1000);


	char to_send2[20] = {'\0'};
	uint64_t v1 = reader.tag / (1<<32);
	v1 = (v1 << 32);
	uint32_t v2 = reader.tag - v1;
	sprintf(to_send2, "%x\r\n", (v2));
	BLE_Send(&ble_device, to_send2);

	// blink green led x2
	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_G_GPIO_Port, GPIO_LED_G_Pin, GPIO_PIN_RESET);

	// send tag to BLE


	// check battery level and send it to BLE

	prepare_to_sleep();
}

void ble_not_found()
{
	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIO_LED_R_GPIO_Port, GPIO_LED_R_Pin, GPIO_PIN_RESET);


	prepare_to_sleep();
}

void prepare_to_sleep()
{
	//stateMachine.programState = PREPARE_TO_SLEEP_STATE;

	//BLE_PowerOff(&ble_device);

	stateMachine.programState = SLEEP_STATE;

	// SLEEP COMMENT IMPORTANT KAROL
	HAL_PWR_EnableSleepOnExit();
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// sleep
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
