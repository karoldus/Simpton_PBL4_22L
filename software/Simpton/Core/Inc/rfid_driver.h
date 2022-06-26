/*
 * rfid_driver.h
 *
 *  Created on: 11 maj 2022
 *      Author: karol
 */

#ifndef INC_RFID_DRIVER_H_
#define INC_RFID_DRIVER_H_

#include "mlx90109cdc.h"

void RFID_Initialise(RFID_Data *reader, GPIO_TypeDef *rfid_data_port, GPIO_TypeDef *rfid_clk_port, GPIO_TypeDef *rfid_modu_port, uint16_t rfid_data_pin, uint16_t rfid_clk_pin, uint16_t rfid_modu_pin);

void RFID_Interrupt_handler(RFID_Data *reader,TIM_HandleTypeDef *timer);

#endif /* INC_RFID_DRIVER_H_ */
