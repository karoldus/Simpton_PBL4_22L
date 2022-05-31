/*
 * ble_driver.h
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#ifndef INC_BLE_DRIVER_H_
#define INC_BLE_DRIVER_H_

/*
 *  INCLUDES
 */

#include "stm32l0xx_hal.h"
#include "rn4870.h"


/*
 *  DEFINES
 */


/*
 *  MODULE STRUCT
 */

typedef struct {

	/* UART handle */
	UART_HandleTypeDef *uartHandle;

	/* Power Pin */
	GPIO_TypeDef *powerGPIOPort;
	uint16_t powerGPIOPin;

	/* Connection status */
	uint8_t connection;

	/* Power status */
	uint8_t power;

} BLE;


/*
 *  INITIALIZATION
 */

uint8_t BLE_Initialise( BLE *dev, UART_HandleTypeDef *uartHandle, GPIO_TypeDef *powerGPIOPort, uint16_t powerGPIOPin, char *name );


/*
 *  POWER FUNCTIONS
 */

HAL_StatusTypeDef BLE_PowerOff( BLE *dev );
HAL_StatusTypeDef BLE_PowerOn( BLE *dev );


/*
 *  TRANSMIT FUNCTIONS
 */

HAL_StatusTypeDef BLE_Send( BLE *dev, char *mess );


#endif /* INC_BLE_DRIVER_H_ */
