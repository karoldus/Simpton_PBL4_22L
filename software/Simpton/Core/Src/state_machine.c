/*
 * state_machine.c
 *
 *  Created on: Jun 15, 2022
 *      Author: karol
 */

#include "state_machine.h"


uint8_t Machine_Initialise(StateMachine *stateMachine, BLE *ble_device, RFID_Data *reader, GPIO_TypeDef *ledRGPIOPort, uint16_t ledRGPIOPin, GPIO_TypeDef *ledGGPIOPort, uint16_t ledGGPIOPin, GPIO_TypeDef *ledBGPIOPort, uint16_t ledBGPIOPin, GPIO_TypeDef *moduGPIOPort, uint16_t moduGPIOPin)
{
	stateMachine->programState = SLEEP_STATE;
	stateMachine->BLEStartTime = 0;
	stateMachine->RFIDStartTime = 0;

	stateMachine->ble_device = ble_device;
	stateMachine->reader = reader;

	stateMachine->ledRGPIOPort = ledRGPIOPort;
	stateMachine->ledRGPIOPin = ledRGPIOPin;
	stateMachine->ledGGPIOPort = ledGGPIOPort;
	stateMachine->ledGGPIOPin = ledGGPIOPin;
	stateMachine->ledBGPIOPort = ledBGPIOPort;
	stateMachine->ledBGPIOPin = ledBGPIOPin;

	stateMachine->moduGPIOPort = moduGPIOPort;
	stateMachine->moduGPIOPin = moduGPIOPin;

	return 1;
}



void waking_up(StateMachine *stateMachine)
{
	//BLE_PowerOn(stateMachine->ble_device);


	HAL_GPIO_WritePin(stateMachine->ledBGPIOPort, stateMachine->ledBGPIOPin, GPIO_PIN_SET);
	HAL_Delay(300);
	// do sth
	HAL_GPIO_WritePin(stateMachine->ledBGPIOPort, stateMachine->ledBGPIOPin, GPIO_PIN_RESET);
	// do sth

	//HAL_Delay(2000);

	// go to next state
	stateMachine->RFIDStartTime = HAL_GetTick();
	stateMachine->programState = WAITING_FOR_RFID_STATE;

	// turn on RFID
	stateMachine->reader->tag = 0;
	HAL_GPIO_WritePin(stateMachine->moduGPIOPort, stateMachine->moduGPIOPin, 0);
}

void rfid_found(StateMachine *stateMachine)
{
	HAL_GPIO_WritePin(stateMachine->moduGPIOPort, stateMachine->moduGPIOPin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_RESET);

	stateMachine->BLEStartTime = HAL_GetTick();
	stateMachine->programState = RFID_FOUND_STATE;
}


void rfid_not_found(StateMachine *stateMachine)
{
	HAL_GPIO_WritePin(stateMachine->moduGPIOPort, stateMachine->moduGPIOPin, 1);

	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_RESET);

	stateMachine->BLEStartTime = HAL_GetTick();
	stateMachine->programState = RFID_TIMEOUT_STATE;

	prepare_to_sleep(stateMachine);
}

void ble_found(StateMachine *stateMachine)
{
	HAL_Delay(1000);
	char to_send[20] = {'\0'};
	sprintf(to_send, "%x", (stateMachine->reader->tag >> 32));
	BLE_Send(stateMachine->ble_device, to_send);
	//HAL_UART_Abort(ble_device.uartHandle);
	//HAL_Delay(1000);


	char to_send2[20] = {'\0'};
	uint64_t v1 = stateMachine->reader->tag / (1<<32);
	v1 = (v1 << 32);
	uint32_t v2 = stateMachine->reader->tag - v1;
	sprintf(to_send2, "%x\r\n", (v2));
	BLE_Send(stateMachine->ble_device, to_send2);

	// blink green led x2
	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledGGPIOPort, stateMachine->ledGGPIOPin, GPIO_PIN_RESET);

	// send tag to BLE


	// check battery level and send it to BLE

	prepare_to_sleep(stateMachine);
}

void ble_not_found(StateMachine *stateMachine)
{
	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stateMachine->ledRGPIOPort, stateMachine->ledRGPIOPin, GPIO_PIN_RESET);


	prepare_to_sleep(stateMachine);
}

void prepare_to_sleep(StateMachine *stateMachine)
{
	//stateMachine.programState = PREPARE_TO_SLEEP_STATE;

	//BLE_PowerOff(stateMachine->ble_device);

	stateMachine->programState = SLEEP_STATE;

	HAL_PWR_EnableSleepOnExit();
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// sleep
}

