/*
 * mlx90109cdc.h
 *
 *  Created on: 11 maj 2022
 *      Author: michal
 */

#ifndef INC_MLX90109CDC_H_
#define INC_MLX90109CDC_H_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32l0xx_hal.h"
#define RFID_BUFFER_SIZE				64

typedef struct
{
	GPIO_TypeDef *rfid_data_port;
	GPIO_TypeDef *rfid_clk_port;
	GPIO_TypeDef *rfid_modu_port;

	uint16_t rfid_data_pin;
	uint16_t rfid_clk_pin;
	uint16_t rfid_modu_pin;

	uint32_t high_time;
	uint32_t low_time;
	GPIO_PinState buff[RFID_BUFFER_SIZE];
	uint16_t iterator;
	uint16_t goodClkCounter;
	uint8_t firstBitFrameIndex;

	uint64_t tag;



}RFID_Data;

void Init_RFID(RFID_Data *reader, GPIO_TypeDef *rfid_data_port, GPIO_TypeDef *rfid_clk_port, GPIO_TypeDef *rfid_modu_port, uint16_t rfid_data_pin, uint16_t rfid_clk_pin, uint16_t rfid_modu_pin);

void Read_RFID(RFID_Data *reader,TIM_HandleTypeDef *htim2);

uint64_t AnalyzeBuff(RFID_Data* reader);

void StartTim(RFID_Data* reader, TIM_HandleTypeDef* htim21);

void StartTim(RFID_Data* reader, TIM_HandleTypeDef* htim21);

#endif /* INC_MLX90109CDC_H_ */
