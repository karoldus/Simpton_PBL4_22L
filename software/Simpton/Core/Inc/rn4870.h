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
#include <string.h>



/*
 *  FUNCTIONS
 */

uint8_t expectResponse( const char* expectedResponse, uint16_t timeout, UART_HandleTypeDef *uartHandle );

HAL_StatusTypeDef RN4870_EnterCMD( UART_HandleTypeDef *uartHandle );
HAL_StatusTypeDef RN4870_ExitCMD( UART_HandleTypeDef *uartHandle );
HAL_StatusTypeDef RN4870_Write( UART_HandleTypeDef *uartHandle, const char *command );

HAL_StatusTypeDef RN4870_WriteCommand( UART_HandleTypeDef *uartHandle, const char *command );

HAL_StatusTypeDef RN4870_Reboot( UART_HandleTypeDef *uartHandle );

void RN4870_ClearRXBuffer( UART_HandleTypeDef *uartHandle );

HAL_StatusTypeDef RN4870_GetResponse( UART_HandleTypeDef *uartHandle, char *response );


#endif /* INC_RN4870_H_ */
