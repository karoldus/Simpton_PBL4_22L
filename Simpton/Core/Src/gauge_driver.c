/*
 * gauge_driver.c
 *
 *  Created on: 11 maj 2022
 *      Author: kamil
 */


#include "gauge_driver.h"

/*Initialization*/
uint8_t MAX17201_Initialise( MAX17201 *dev, I2C_HandleTypeDef *i2cHandle )
{
	dev->i2cHandle    = i2cHandle;

	dev->current      = 0.0f;
	dev->capacity     = 0.0f;

	return 0;
}
//
//
//	uint8_t errNum = 0;
//	HAL_StatusTypeDef status;
//
//	uint8_t regData;

//	status = gauge_read_reg( dev,  , &regData);    		//reading if registers are available/correct
//	errNum += (status != HAL_OK);
//
//	if ( regData !=  )
//	{
//
//		return 255;
//
//	}


//  status = gauge_write_reg( dev, , &regData);           //setting registers to different modes
//	errNum += (status != HAL_OK);
//
//	return errNum;
//}
/*Read functions*/



/*Low level functions definitions*/
HAL_StatusTypeDef MAX17201_read_reg(MAX17201 *dev, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Read( dev->i2cHandle, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_read_reg1(MAX17201 *dev, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Read( dev->i2cHandle, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_read_regs(MAX17201 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{

	return HAL_I2C_Mem_Read( dev->i2cHandle, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_write_reg(MAX17201 *dev, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Write( dev->i2cHandle, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_write_regs(MAX17201 *dev, uint8_t reg, uint16_t *data)
{

	return HAL_I2C_Mem_Write( dev->i2cHandle, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_read_batt_voltage( MAX17201 *dev ){

	uint8_t regData[2];

	//HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPCAP_ADDR, regData, 2);
	HAL_StatusTypeDef status = MAX17201_read_reg(dev, MAX1720X_VCELL_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}



HAL_StatusTypeDef MAX17201_read_main_voltage( MAX17201 *dev ){

	uint8_t regData[2];

	//HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPCAP_ADDR, regData, 2);
	HAL_StatusTypeDef status = MAX17201_read_reg(dev, MAX1720X_VBAT_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}

HAL_StatusTypeDef MAX17201_read_gauge_temp( MAX17201 *dev ){

	uint8_t regData[2];

	//HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPCAP_ADDR, regData, 2);
	HAL_StatusTypeDef status = MAX17201_read_reg(dev, MAX1720X_TEMP_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}


HAL_StatusTypeDef MAX17201_read_avg_current( MAX17201 *dev ){

	uint8_t regData[2];

	//HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPCAP_ADDR, regData, 2);
	HAL_StatusTypeDef status = MAX17201_read_reg(dev, MAX1720X_AVGCURENT_ADDR, regData);

	uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (raw_data * 0.078125);

	return status;
}

HAL_StatusTypeDef MAX17201_read_capacity( MAX17201 *dev ){

	//uint8_t regData[2];
	uint16_t regData = 0;

	//HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPCAP_ADDR, regData, 2);
	HAL_StatusTypeDef status = MAX17201_read_regs(dev, MAX1720X_REPSOC_ADDR, regData, 2);

	//uint16_t raw_data = ( regData[0] | regData[1]);

	dev->capacity = (regData / 256);

	return status;
}





///////////////////////////////////// FOR TESTS (OLD) ///////////////////////////////////////////////
//int16_t gauge_read_current(uint8_t reg, I2C_HandleTypeDef i2c)
//{
//	int16_t value = 0;
//	int8_t v1 = 0;
//	int8_t v2 = 0;
//
//	//HAL_I2C_Mem_Read(&i2c, MAX1720X_ADDR<<1, reg, 1, &value, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&i2c, MAX1720X_ADDR<<1, reg, 1, &v1, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&i2c, MAX1720X_ADDR<<1, reg+1, 1, &v2, 1, HAL_MAX_DELAY);
//
//	value = v1 | v2<<8;
//
//	return value;
//}



//int16_t gauge_read_capacity(uint8_t reg, I2C_HandleTypeDef i2c)
//{
//	int16_t value = 0;
//
//	HAL_I2C_Mem_Read(&i2c, MAX1720X_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
//
//	return value;

////////////////////////////////////////////////////////////////////////////////////////////
