/*
 * ble_driver.c
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#include "ble_driver.h"


// ---------------------------- INITIALIZATION --------------------------------------

uint8_t BLE_Initialise( BLE *dev, UART_HandleTypeDef *uartHandle, GPIO_TypeDef *powerGPIOPort, uint16_t powerGPIOPin, char *name )
{
	dev->uartHandle 	= uartHandle;
	dev->powerGPIOPort 	= powerGPIOPort;
	dev->powerGPIOPin 	= powerGPIOPin;

	dev->connection 	= 0;
	dev->power 			= 1;

	//RN4870_SetName(uartHandle, name);
	//RN4870_Reboot(uartHandle);

	return 1;
}


// ---------------------------- POWER FUNCTIONS --------------------------------------

HAL_StatusTypeDef BLE_PowerOff( BLE *dev )
{
	RN4870_EnterCMD(dev->uartHandle);
	HAL_Delay(50);
	RN4870_WriteCommand(dev->uartHandle, SET_DORMANT_MODE);
	HAL_Delay(500);
	dev->power = 0;


//	status = BLE_Send(&ble_device, "$$$");
//		  HAL_Delay(50);
//		  status = BLE_Send(&ble_device, "O,0");
//		  status = BLE_Send(&ble_device, "\r");
//		  HAL_Delay(500);
}


HAL_StatusTypeDef BLE_PowerOn( BLE *dev )
{
	HAL_GPIO_WritePin(dev->powerGPIOPort, dev->powerGPIOPin, 0);
	HAL_Delay(1200);
	HAL_GPIO_WritePin(dev->powerGPIOPort, dev->powerGPIOPin, 1);
	dev->power = 1;
}


// ---------------------------- TRANSMIT FUNCTIONS --------------------------------------

HAL_StatusTypeDef BLE_Send( BLE *dev, char *mess )
{
	return RN4870_Write(dev->uartHandle, mess);
}


// ---------------------------- CONNECTION FUNCTIONS --------------------------------------

/*
 * Check if BLE module is connected.
 * returns HAL_OK or HAL_TIMEOUT
 * changes dev->connection value
 */
HAL_StatusTypeDef BLE_is_connected( BLE *dev )
{
	RN4870_EnterCMD(dev->uartHandle);
	HAL_Delay(50);


	RN4870_ClearRXBuffer(dev->uartHandle);
	//RN4870_WriteCommand(dev->uartHandle, GET_CONNECTION_STATUS);
	RN4870_Write(dev->uartHandle, "GK\r");
	HAL_Delay(50);
//	RN4870_WriteCommand(dev->uartHandle, GET_CONNECTION_STATUS);
//	HAL_Delay(50);

	char response[LINE_MAX_LENGTH + 1] = {0};

//	if (RN4870_GetResponse( dev->uartHandle, response ) == HAL_OK)
//	{
//		if(strstr(response, NONE_RESP) != NULL)
//			dev->connection = 0;
//		else
//			dev->connection = 1;
//
//		RN4870_ExitCMD(dev->uartHandle);
//		HAL_Delay(100);
//		return HAL_OK;
//	}
//	else
//	{
//		RN4870_ExitCMD(dev->uartHandle);
//		HAL_Delay(100);
//		return HAL_TIMEOUT;
//	}

	char line_buffer[LINE_MAX_LENGTH + 1] = {0};
	uint8_t line_length = 0;

	unsigned long previous = HAL_GetTick();

	while(HAL_GetTick() - previous < 2000) //DEFAULT_CMD_TIMEOUT
	{
		uint8_t value;

		HAL_StatusTypeDef status = HAL_UART_Receive(dev->uartHandle, &value, 1, 100);

		if ( status == HAL_OK)
		{
			if(line_length < LINE_MAX_LENGTH)
			{
				line_buffer[line_length++] = value;
			}
		}
	}

	unsigned long newtime = HAL_GetTick();

	if(strlen(line_buffer) == 0)
	{
		RN4870_ExitCMD(dev->uartHandle);
		HAL_Delay(100);
		return HAL_TIMEOUT;
	}
	else
	{
		if(strstr(response, NONE_RESP) != NULL)
			dev->connection = 0;
		else
			dev->connection = 1;

		RN4870_ExitCMD(dev->uartHandle);
		HAL_Delay(100);
		return HAL_OK;
	}
}
