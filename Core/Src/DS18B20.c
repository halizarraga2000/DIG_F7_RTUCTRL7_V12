/**
  ******************************************************************************
  *  DS18B20.c
  *  Created : 30/09/2021 14:10:00
  *  Author  : Hugo Lizarraga
  *
  *  -------- DS18B20 ----------------
  *
  ******************************************************************************
  */

#include "DS18B20.h"

//==========================================================================================
//  VARIABLES del Main /* USER CODE BEGIN PV */  DS18B20 (Temperatura)
//==========================================================================================
//uint8_t Scratchpad [5];  //Scratchpad [TempL, TempH, Th, Tl, Config]
//uint16_t temp = 0;
//float temperatura = 0;
//uint8_t presence = 0;

extern void delay (uint16_t time);

//==========================================================================================
//  FUNCIONES del Main /* USER CODE BEGIN 2/ */  DS18B20 (Temperatura)
//==========================================================================================

  /***** DS18B20 TEMPERATURA - Configuracion/Grabar los datos hacia el SCRATCHPAD ********/
/* 		  presence = DS18B20_Start ();
  		  HAL_Delay (1);
  		  DS18B20_Write (0xCC);  // skip ROM
  		  DS18B20_Write (0x4E);  // convert t
  		  //----------------------------------
  		  DS18B20_Write (0x00);  // Th
  		  DS18B20_Write (0x00);  // Tl

  		  //DS18B20_Write (0x7F);  // Configuracion x 12 Bits
  		  //DS18B20_Write (0x5F);  // Configuracion x 11 Bits
  		  //DS18B20_Write (0x3F);  // Configuracion x 10 Bits
  		  DS18B20_Write (0x1F);  // Configuracion x 9 Bits

  		  HAL_Delay (100);    //Grabando  SCRATCHPAD
*/
  /****************************************************************************************/

//==========================================================================================
//  MAIN /* USER CODE BEGIN 3/ */  DS18B20 (Temperatura)
//==========================================================================================
/*
		// ***** DS18B20 TEMPERATURA - Leer los datos desde el SCRATCHPAD *******
		  presence = DS18B20_Start ();
		  HAL_Delay (1);
		  DS18B20_Write (0xCC);  // skip ROM
		  DS18B20_Write (0x44);  // convert t
		  HAL_Delay (100);  // Resolucion p/ 9bit(100), 10bit(200), 11bit(400), 12bit(800)
		//-----------------------------------------------------------------------
		  presence = DS18B20_Start ();
		  HAL_Delay(1);
		  DS18B20_Write (0xCC);  // skip ROM
		  DS18B20_Write (0xBE);  // Read Scratch-pad
		  for (int i=0; i<5; i++)
		  {
			  Scratchpad [i] = DS18B20_Read(); // Scratchpad [TempL, TempH, Th, Tl, Config]
		  }
		  temp = (Scratchpad [1]<<8) | Scratchpad [0];
		  temperatura = (float)temp/16;
		//----------------------------------------------------------------------
*/

//==========================================================================================
//  DS18B20 (Temperatura)
//==========================================================================================

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;    // PULLUP -> NOPULL
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_Start (void)
{
	uint8_t response = 0;
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin low
	delay (480);   // delay according to datasheet
	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_GPIO_Port, DS18B20_Pin))) response = 1;  //if the pin is low i.e the presence pulse is detected
	else response = -1;

	delay (400); // 480 us delay totally.

	return response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output

	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output
			HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
			delay (50);  // wait for 60 us
		}
		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);
			HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set as output

		HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_GPIO_Port, DS18B20_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}

