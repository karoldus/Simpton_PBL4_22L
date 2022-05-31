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
	RN4870_Write(dev->uartHandle, SET_DORMANT_MODE);
	dev->power = 0;
}


HAL_StatusTypeDef BLE_PowerOn( BLE *dev )
{
	HAL_GPIO_WritePin(dev->powerGPIOPort, dev->powerGPIOPin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(dev->powerGPIOPort, dev->powerGPIOPin, 1);
	dev->power = 1;
}


// ---------------------------- TRANSMIT FUNCTIONS --------------------------------------

HAL_StatusTypeDef BLE_Send( BLE *dev, char *mess )
{
	return RN4870_Write(dev->uartHandle, mess);
}
