/*
 * rn4870.c
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#include "rn4870.h"

uint8_t expectResponse(const char* expectedResponse, uint16_t timeout)
{
	static char line_buffer[LINE_MAX_LENGTH + 1];
	static uint32_t line_length;

	unsigned long previous = HAL_GetTick();

	while(HAL_GetTick() - previous < timeout)
	{
		uint8_t value;
		if (HAL_UART_Receive(&huart2, &value, 1, 0) == HAL_OK)
		{
			if (value == '\r' || value == '\n') {
				if (line_length > 0) {
					line_buffer[line_length] = '\0';

					if (strstr(line_buffer, expectedResponse) != NULL)
					{
						return 1; // ok
					}
					return 0; // error
				}
			}
			else {
				if (line_length >= LINE_MAX_LENGTH)
					line_length = 0;

				line_buffer[line_length++] = value;
			}
		}
	}

	if (strstr(line_buffer, expectedResponse) != NULL)
	{
		return 1; // ok
	}
	return 0; // error
}


HAL_StatusTypeDef RN4870_EnterCMD( UART_HandleTypeDef *uartHandle )
{
	HAL_Delay(DELAY_BEFORE_CMD);
	HAL_UART_Transmit(&uartHandle, ENTER_CMD, strlen(ENTER_CMD), HAL_MAX_DELAY);

	if(expectResponse(PROMPT ,DEFAULT_CMD_TIMEOUT))
		return HAL_OK;
	return HAL_ERROR;
}


HAL_StatusTypeDef RN4870_ExitCMD( UART_HandleTypeDef *uartHandle )
{
	HAL_UART_Transmit(&uartHandle, EXIT_CMD, strlen(EXIT_CMD), HAL_MAX_DELAY);
	if(expectResponse(PROMPT_END ,DEFAULT_CMD_TIMEOUT))
		return HAL_OK;
	return HAL_ERROR;
}
