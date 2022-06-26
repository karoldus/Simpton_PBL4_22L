/*
 * gauge_driver.h
 *
 *  Created on: 11 maj 2022
 *      Author: kamil
 */

#ifndef INC_GAUGE_DRIVER_H_
#define INC_GAUGE_DRIVER_H_

#include "max17201.h"

typedef struct {																							//example gauge structure

	I2C_HandleTypeDef *i2cHandle;

	uint16_t main_voltage;

	uint16_t batt_voltage;

	uint16_t temperatue;

	uint16_t capacity;




} GAUGE;


uint8_t Gauge_initialise( GAUGE *dev, I2C_HandleTypeDef *i2cHandle );									//initialize function


HAL_StatusTypeDef Gauge_read_batt_voltage( GAUGE *dev );
HAL_StatusTypeDef Gauge_read_main_voltage( GAUGE *dev );
HAL_StatusTypeDef Gauge_read_gauge_temp( GAUGE *dev );
HAL_StatusTypeDef Gauge_read_avg_current( GAUGE *dev );
HAL_StatusTypeDef Gauge_read_capacity( GAUGE *dev );


#endif /* INC_GAUGE_DRIVER_H_ */
