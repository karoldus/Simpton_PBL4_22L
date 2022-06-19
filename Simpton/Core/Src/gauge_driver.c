/*
 * gauge_driver.c
 *
 *  Created on: 11 maj 2022
 *      Author: kamil
 */


#include "gauge_driver.h"

/*Initialization*/

uint8_t Gauge_initialise( GAUGE *dev, I2C_HandleTypeDef *i2cHandle )
{
	dev->i2cHandle    = i2cHandle;

	dev->main_voltage     = 0;
	dev->batt_voltage     = 0;
	dev->temperatue      = 0;
	dev->capacity     = 0;

	return 0;
}


/*Read functions*/


HAL_StatusTypeDef Gauge_read_batt_voltage( GAUGE *dev ){

	uint16_t data4 = 0;

	HAL_StatusTypeDef status = MAX17201_read_regs(&gauge, MAX1720X_VCELL_ADDR, &data4, 2);

	dev->batt_voltage = data4;

	return status;
}



HAL_StatusTypeDef Gauge_read_main_voltage( GAUGE *dev ){

	uint16_t data5 = 0;

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_VBAT_ADDR, regData);

	dev->main_voltage = data5;

	return status;
}

HAL_StatusTypeDef Gauge_read_gauge_temp( GAUGE *dev ){

	uint16_t data6 = 0;

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_TEMP_ADDR, regData);

	dev->temperatue = data6;

	return status;
}


HAL_StatusTypeDef Gauge_read_capacity( GAUGE *dev ){


	uint16_t data7 = 0;

	HAL_StatusTypeDef status = MAX17201_read_regs(dev->i2cHandle, MAX1720X_REPSOC_ADDR, regData, 2);

	dev->capacity = data7;

	return status;
}
