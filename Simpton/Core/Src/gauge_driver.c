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

	dev->current      = 0.0f;
	dev->capacity     = 0.0f;

	return 0;
}


/*Read functions*/


HAL_StatusTypeDef Gauge_read_batt_voltage( GAUGE *dev ){

	uint8_t regData[2];

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_VCELL_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}



HAL_StatusTypeDef Gauge_read_main_voltage( GAUGE *dev ){

	uint8_t regData[2];

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_VBAT_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}

HAL_StatusTypeDef Gauge_read_gauge_temp( GAUGE *dev ){

	uint8_t regData[2];

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_TEMP_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}


HAL_StatusTypeDef Gauge_read_avg_current( GAUGE *dev ){

	uint8_t regData[2];

	HAL_StatusTypeDef status = MAX17201_read_reg(dev->i2cHandle, MAX1720X_AVGCURENT_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}

HAL_StatusTypeDef Gauge_read_capacity( GAUGE *dev ){

	uint16_t regData = 0;

	HAL_StatusTypeDef status = MAX17201_read_regs(dev->i2cHandle, MAX1720X_REPSOC_ADDR, regData, 2);

	dev->capacity = (regData / 256);

	return status;
}
