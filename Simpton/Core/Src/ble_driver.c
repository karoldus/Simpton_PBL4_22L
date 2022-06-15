/*
 * ble_driver.c
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#include "ble_driver.h"


// ---------------------------- INITIALIZATION --------------------------------------

uint8_t BLE_Initialise( BLE *dev, UART_HandleTypeDef *uartHandle, GPIO_TypeDef *powerGPIOPort, uint16_t powerGPIOPin)
{
	dev->uartHandle 	= uartHandle;
	dev->powerGPIOPort 	= powerGPIOPort;
	dev->powerGPIOPin 	= powerGPIOPin;

	dev->connection 	= 0;
	dev->power 			= 1;

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
	HAL_Delay(100);

	uint8_t response[LINE_MAX_LENGTH + 1] = {0};

	RN4870_ClearRXBuffer(dev->uartHandle);
	HAL_UART_Abort(dev->uartHandle);
	RN4870_WriteCommand(dev->uartHandle, GET_CONNECTION_STATUS);
	HAL_StatusTypeDef state = HAL_UART_Receive(dev->uartHandle, response, 4, 1000);
	HAL_UART_Abort(dev->uartHandle);

	if( state == HAL_OK)
	{
		if(strstr(response, NONE_RESP) != NULL || strstr(response, ERR_RESP) != NULL)
		{
			// "none" response
			dev->connection = 0;
		}
		else
		{
			dev->connection = 1;
		}
		RN4870_ExitCMD(dev->uartHandle);
		return HAL_OK;
	}
	else // sth went wrong
	{
		dev->connection = 0;
		RN4870_ExitCMD(dev->uartHandle);
		return HAL_TIMEOUT;
	}
}
