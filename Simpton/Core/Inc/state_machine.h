/*
 * state_machine.h
 *
 *  Created on: Jun 15, 2022
 *      Author: karol
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "stm32l0xx_hal.h"
#include "ble_driver.h"
#include "mlx90109cdc.h"
#include "gauge_driver.h"

#define RFID_TIMEOUT	20000 // 10 s
#define BLE_TIMEOUT		10000 // 10 s



enum ProgramState {
	SLEEP_STATE,
	WAKING_UP_STATE,
	WAITING_FOR_RFID_STATE,
	RFID_FOUND_STATE,
	RFID_TIMEOUT_STATE,
	//PREPARE_TO_SLEEP_STATE
};

typedef struct  {
	enum ProgramState programState;
	uint32_t RFIDStartTime;
	uint32_t BLEStartTime;

	BLE *ble_device;
	RFID_Data *reader;
	GPIO_TypeDef *ledRGPIOPort;
	uint16_t ledRGPIOPin;
	GPIO_TypeDef *ledGGPIOPort;
	uint16_t ledGGPIOPin;
	GPIO_TypeDef *ledBGPIOPort;
	uint16_t ledBGPIOPin;
	GPIO_TypeDef *moduGPIOPort;
	uint16_t moduGPIOPin;
} StateMachine;

uint8_t Machine_Initialise(StateMachine *stateMachine, BLE *ble_device, RFID_Data *reader, GPIO_TypeDef *ledRGPIOPort, uint16_t ledRGPIOPin, GPIO_TypeDef *ledGGPIOPort, uint16_t ledGGPIOPin, GPIO_TypeDef *ledBGPIOPort, uint16_t ledBGPIOPin, GPIO_TypeDef *moduGPIOPort, uint16_t moduGPIOPin);

void waking_up(StateMachine *stateMachine);
void rfid_found(StateMachine *stateMachine);
void rfid_not_found(StateMachine *stateMachine);
void ble_found(StateMachine *stateMachine);
void ble_not_found(StateMachine *stateMachine);
void prepare_to_sleep(StateMachine *stateMachine);


#endif /* INC_STATE_MACHINE_H_ */
