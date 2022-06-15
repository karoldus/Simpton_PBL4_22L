/*
 * gauge_driver.h
 *
 *  Created on: 11 maj 2022
 *      Author: kamil
 */

#ifndef INC_GAUGE_DRIVER_H_
#define INC_GAUGE_DRIVER_H_

#include "stm32l0xx_hal.h" //included for I2C


#define MAX1720X_ADDR (0x36 << 1) // MAX1720X device addresses shifted 1

#define MAX1720X_STATUS_ADDR 0x00 // Contains alert status and chip status
#define MAX1720X_VCELL_ADDR 0x09 // Lowest cell voltage of a pack, or the cell voltage for a single cell
#define MAX1720X_REPSOC_ADDR 0x06 // Reported state of charge
#define MAX1720X_REPCAP_ADDR 0x05 // Reported remaining capacity
#define MAX1720X_TEMP_ADDR 0x08 // Temperature
#define MAX1720X_CURENT_ADDR 0x0A // Battery current
#define MAX1720X_AVGCURENT_ADDR 0x0B // Average battery current
#define MAX1720X_TTE_ADDR 0x11 // Time to empty
#define MAX1720X_TTF_ADDR 0x20 // Time to full
#define MAX1720X_CAPACITY_ADDR 0x10 // Full capacity estimation
#define MAX1720X_VBAT_ADDR 0xDA// Battery pack voltage
#define MAX1720X_AVCELL_ADDR 0x17 // Battery cycles
#define MAX1720X_COMMAND_ADDR 0x60 // Command register
#define MAX1720X_CONFIG2_ADDR 0xbb // Command register

#define Res 0x01F4

typedef struct {																							//example gauge structure

	I2C_HandleTypeDef *i2cHandle;

	float current;

	float capacity;

} MAX17201;

uint8_t MAX17201_Initialise( MAX17201 *dev, I2C_HandleTypeDef *i2cHandle );									//initialize function

HAL_StatusTypeDef MAX17201_read_reg( MAX17201 *dev, uint8_t reg, uint8_t *data ); 								//read one byte
HAL_StatusTypeDef MAX17201_read_reg1( MAX17201 *dev, uint8_t reg, uint8_t *data ); 								//read one byte


HAL_StatusTypeDef MAX17201_read_regs( MAX17201 *dev, uint8_t reg, uint8_t *data, uint8_t length );			    //read multiple bytes

HAL_StatusTypeDef MAX17201_write_reg( MAX17201 *dev, uint8_t reg, uint8_t *data ); 							//write one byte
HAL_StatusTypeDef MAX17201_write_regs( MAX17201 *dev, uint8_t reg, uint16_t *data ); 							//write one byte


HAL_StatusTypeDef MAX17201_read_current( MAX17201 *dev );   													//read current function
HAL_StatusTypeDef MAX17201_read_capacity( MAX17201 *dev );														//read capacity function


#endif /* INC_GAUGE_DRIVER_H_ */
