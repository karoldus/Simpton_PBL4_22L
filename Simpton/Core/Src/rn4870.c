/*
 * rn4870.c
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#include "rn4870.h"

//uint8_t expectResponse(const char* expectedResponse, uint16_t timeout, UART_HandleTypeDef *uartHandle ) // add uart handle
//{
//	static char line_buffer[LINE_MAX_LENGTH + 1];
//	static uint32_t line_length;
//
//	unsigned long previous = HAL_GetTick();
//
//	while(HAL_GetTick() - previous < timeout)
//	{
//		uint8_t value;
//		if (HAL_UART_Receive(uartHandle, &value, 1, 0) == HAL_OK)
//		{
//			if (value == '\r' || value == '\n') {
//				if (line_length > 0) {
//					line_buffer[line_length] = '\0';
//
//					if (strstr(line_buffer, expectedResponse) != NULL)
//					{
//						return 1; // ok
//					}
//					return 0; // error
//				}
//			}
//			else {
//				if (line_length >= LINE_MAX_LENGTH)
//					line_length = 0;
//
//				line_buffer[line_length++] = value;
//			}
//		}
//	}
//
//	if (strstr(line_buffer, expectedResponse) != NULL)
//	{
//		return 1; // ok
//	}
//	return 0; // error
//}


HAL_StatusTypeDef RN4870_EnterCMD( UART_HandleTypeDef *uartHandle )
{
	HAL_Delay(DELAY_BEFORE_CMD);
	HAL_StatusTypeDef status = RN4870_Write(uartHandle, ENTER_CMD);

//	if(expectResponse(PROMPT ,DEFAULT_CMD_TIMEOUT, uartHandle))
//		return HAL_OK;
//	return HAL_ERROR;

	return status;
}


HAL_StatusTypeDef RN4870_ExitCMD( UART_HandleTypeDef *uartHandle )
{
	HAL_StatusTypeDef status = RN4870_Write(uartHandle, EXIT_CMD);

//	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT, uartHandle))
//		return HAL_OK;
//	return HAL_ERROR;

	return status;
}

HAL_StatusTypeDef RN4870_Write( UART_HandleTypeDef *uartHandle, const char *command )
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(uartHandle, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
	return status;
}

/*
 *  Adds \r at the end
 */
HAL_StatusTypeDef RN4870_WriteCommand( UART_HandleTypeDef *uartHandle, const char *command )
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(uartHandle, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
	//if(status == HAL_OK)
	status = HAL_UART_Transmit(uartHandle, DATA_LAST_CHAR, strlen(DATA_LAST_CHAR), HAL_MAX_DELAY);
	return status;
}


HAL_StatusTypeDef RN4870_SetName( UART_HandleTypeDef *uartHandle, char *name )
{
	RN4870_EnterCMD(uartHandle);
	uint8_t comm_len = strlen(SET_SERIALIZED_NAME) ;
	uint8_t len = strlen(name);

	char to_send [comm_len+len+1];
	strcpy(to_send, SET_SERIALIZED_NAME);
	strcpy(to_send, name);

	HAL_StatusTypeDef status = RN4870_WriteCommand(uartHandle, to_send);

//	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT, uartHandle))
//		return HAL_OK;
//	return HAL_ERROR;

	RN4870_ExitCMD(uartHandle);

	return status;
}


HAL_StatusTypeDef RN4870_Reboot( UART_HandleTypeDef *uartHandle )
{
	RN4870_EnterCMD(uartHandle);

	HAL_StatusTypeDef status = RN4870_WriteCommand(uartHandle, REBOOT);

	HAL_Delay(DELAY_BEFORE_CMD);

//	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT, uartHandle))
//		return HAL_OK;
//	return HAL_ERROR;

	return status;
}
