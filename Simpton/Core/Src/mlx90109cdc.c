/*
 * mlx90109cdc.c
 *
 *  Created on: 11 maj 2022
 *      Author: michal
 */
#include "mlx90109cdc.h"
void Init_RFID(RFID_Data *reader, GPIO_TypeDef *rfid_data_port, GPIO_TypeDef *rfid_clk_port, GPIO_TypeDef *rfid_modu_port, uint16_t rfid_data_pin, uint16_t rfid_clk_pin, uint16_t rfid_modu_pin)
{
	reader->rfid_data_pin = rfid_data_pin;
	reader->rfid_clk_pin = rfid_clk_pin;
	reader->rfid_modu_pin = rfid_modu_pin;
	reader->rfid_data_port = rfid_data_port;
	reader->rfid_clk_port = rfid_clk_port;
	reader->rfid_modu_port = rfid_modu_port;
	reader->high_time=0;
	reader->low_time=0;
	for(int i=0;i<RFID_BUFFER_SIZE;i++)
	{
		reader->buff[i]=0;
	}
	reader->iterator=0;
	reader->goodClkCounter=0;
	reader->firstBitFrameIndex=0;
	//HAL_GPIO_TogglePin(rfid_modu_port, rfid_modu_pin);
}

void Read_RFID(RFID_Data *reader, TIM_HandleTypeDef* htim2)
{
	if(HAL_GPIO_ReadPin(reader->rfid_clk_port, reader->rfid_clk_pin)==0)
	{
		HAL_TIM_Base_Stop(htim2);
		reader->high_time = TIM21->CNT;
		TIM21->CNT = 0;
		HAL_TIM_Base_Start(htim2);
	}
	else if(HAL_GPIO_ReadPin(reader->rfid_clk_port, reader->rfid_clk_pin)==1)
	{
		reader->buff[reader->iterator++] = HAL_GPIO_ReadPin(reader->rfid_data_port, reader->rfid_data_pin);
		if(reader->iterator==RFID_BUFFER_SIZE)
		{
			reader->iterator = 0;
		}
		HAL_TIM_Base_Stop(htim2);
		reader->low_time = TIM21->CNT;
		TIM21->CNT = 0;
		HAL_TIM_Base_Start(htim2);
		if(abs(reader->low_time-reader->high_time)<1500)
		{
			reader->goodClkCounter++;
		}
		else
		{
			reader->goodClkCounter = 0;
			reader->iterator = 0;
		}
		if(reader->goodClkCounter==RFID_BUFFER_SIZE-1)
		{
			//HAL_GPIO_TogglePin(reader->rfid_led_port, reader->rfid_led_pin);
			uint64_t a = AnalyzeBuff(reader);

			if(a != 0)
			{
				// turn off RFID
				HAL_GPIO_WritePin(reader->rfid_modu_port, reader->rfid_modu_pin, 1);
				reader->tag = a;
			}

			reader->goodClkCounter=0;
		}
		else if(reader->goodClkCounter == 1)
		{
			if(reader->iterator<2)
			{
				reader->firstBitFrameIndex = reader->iterator + RFID_BUFFER_SIZE-2;
			}
			else
			{
				reader->firstBitFrameIndex = reader->iterator - 2;
			}
		}
	}
	else
	{
		printf("ERROR while reading pin state (NOT 0 or 1)\r\n");
	}

}
uint64_t AnalyzeBuff(RFID_Data *reader)
{
	GPIO_PinState data[RFID_BUFFER_SIZE-16];
	for(int z=0;z<RFID_BUFFER_SIZE-16;z++)
		{
			data[z]=0;
		}
	int iterator = 0;
	int counter = -1;
	for(int i =0;i<2*(RFID_BUFFER_SIZE);i++)
	{
		if(iterator==64)
			iterator=0;

		if(reader->buff[iterator]==GPIO_PIN_RESET)
		{

		}
		if((reader->buff[iterator]==GPIO_PIN_SET && counter>0) || (reader->buff[iterator]==GPIO_PIN_RESET && counter==0))
		{
			counter++;

			if(counter==17)
			{
				iterator++;
				for(int j = 0; j<RFID_BUFFER_SIZE-16;j++)
				{
					if(iterator == 64)
						iterator = 0;
					data[j]=reader->buff[iterator];
					iterator++;
				}
				uint64_t tagid = 0;
				for(int k = 0; k<RFID_BUFFER_SIZE-16;k++)
				{
					if(data[k]== GPIO_PIN_RESET)
					{
						continue;
					}
					else if(data[k]==GPIO_PIN_SET)
					{
						tagid = tagid + pow(2,k);
					}
					else
					{
						printf("ERROR Wrong Data!!!\r\n");
					}
				}

				return data;



			}
		}
		else
			counter = 0;

		iterator++;
	}
	return 0;

	//return data;
}
void StartTim(RFID_Data* reader, TIM_HandleTypeDef* htim21)
{

	HAL_TIM_Base_Stop(htim21);
	reader->low_time=TIM2->CNT;
	HAL_TIM_Base_Start(htim21);
}
