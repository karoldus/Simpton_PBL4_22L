/*
 * rfid_driver.c
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#include "rfid_driver.h"

void RFID_Initialise(RFID_Data *reader, GPIO_TypeDef *rfid_data_port, GPIO_TypeDef *rfid_clk_port, GPIO_TypeDef *rfid_modu_port, uint16_t rfid_data_pin, uint16_t rfid_clk_pin, uint16_t rfid_modu_pin)
{
	Init_RFID(reader, rfid_data_port, rfid_clk_port, rfid_modu_port, rfid_data_pin, rfid_clk_pin, rfid_modu_pin);
}

void RFID_Interrupt_handler(RFID_Data *reader,TIM_HandleTypeDef *timer)
{
	Read_RFID(reader, timer);
}
