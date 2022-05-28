/*
 * rn4870.h
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#ifndef INC_RN4870_H_
#define INC_RN4870_H_

/*
 *  INCLUDES
 */

#include "stm32l0xx_hal.h"
#include "rn4870_const.h"



/*
 *  FUNCTIONS
 */

uint8_t expectResponse(const char* expectedResponse, uint16_t timeout);

HAL_StatusTypeDef RN4870_EnterCMD( UART_HandleTypeDef *uartHandle );
HAL_StatusTypeDef RN4870_ExitCMD( UART_HandleTypeDef *uartHandle );
HAL_StatusTypeDef RN4870_WriteCommand( UART_HandleTypeDef *uartHandle, const char *command );


#endif /* INC_RN4870_H_ */
