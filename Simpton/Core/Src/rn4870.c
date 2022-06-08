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
	HAL_Delay(50);

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


//HAL_StatusTypeDef RN4870_SetName( UART_HandleTypeDef *uartHandle, char *name )
//{
//	RN4870_EnterCMD(uartHandle);
//	HAL_Delay(50);
//	uint8_t comm_len = strlen(SET_SERIALIZED_NAME);
//	uint8_t len = strlen(name);
//
//	char to_send [comm_len+len+1];
//	strcpy(to_send, SET_SERIALIZED_NAME);
//	strcpy(to_send, name);
//
//	HAL_StatusTypeDef status = RN4870_WriteCommand(uartHandle, to_send);
//
////	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT, uartHandle))
////		return HAL_OK;
////	return HAL_ERROR;
//
//	HAL_Delay(50);
//
//	RN4870_ExitCMD(uartHandle);
//
//	HAL_Delay(50);
//
//	return status;
//}


HAL_StatusTypeDef RN4870_Reboot( UART_HandleTypeDef *uartHandle )
{
	RN4870_EnterCMD(uartHandle);

	HAL_Delay(50);

	HAL_StatusTypeDef status = RN4870_WriteCommand(uartHandle, REBOOT);

	HAL_Delay(1200);

//	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT, uartHandle))
//		return HAL_OK;
//	return HAL_ERROR;

	return status;
}

void RN4870_ClearRXBuffer( UART_HandleTypeDef *uartHandle )
{
	uint8_t value;
	while(HAL_UART_Receive(uartHandle, &value, 1, 0) == HAL_OK) {HAL_UART_Abort(uartHandle);}
}

/*
 * Get response from UART from BLE RN4870 module.
 * char *response  min. length is (LINE_MAX_LENGTH + 1)
 * returns HAL_OK or HAL_TIMEOUT
 */
HAL_StatusTypeDef RN4870_GetResponse( UART_HandleTypeDef *uartHandle, char *response )
{
	char line_buffer[LINE_MAX_LENGTH + 1] = {0};
	uint8_t line_length = 0;

	unsigned long previous = HAL_GetTick();

	while(HAL_GetTick() - previous < 2000) //DEFAULT_CMD_TIMEOUT
	{
		uint8_t value;
		if (HAL_UART_Receive(uartHandle, &value, 1, 0) == HAL_OK)
		{
			if(line_length < LINE_MAX_LENGTH)
			{
				line_buffer[line_length++] = value;
			}
		}
	}

	strcpy(response, line_buffer);

	if(strlen(line_buffer) == 0)
		return HAL_TIMEOUT;

//	strcpy(response, line_buffer);
	return HAL_OK;
}
