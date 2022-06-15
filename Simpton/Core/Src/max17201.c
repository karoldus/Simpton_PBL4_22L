/*
 * max17201.c
 *
 *  Created on: 11 maj 2022
 *      Author: kamil
 */

#include "max17201.h"

/*Low level functions definitions*/
HAL_StatusTypeDef MAX17201_read_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Read( i2c, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_read_reg1(I2C_HandleTypeDef *i2c, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Read( i2c, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_read_regs(I2C_HandleTypeDef *i2c, uint8_t reg, uint8_t *data, uint8_t length)
{

	return HAL_I2C_Mem_Read( i2c, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_write_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint8_t *data)
{

	return HAL_I2C_Mem_Write( i2c, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MAX17201_write_regs(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t *data)
{

	return HAL_I2C_Mem_Write( i2c, MAX1720X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );

}

