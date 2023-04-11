/**
  ******************************************************************************
  *  DS18B20.h
  *  Created : 30/09/2021 14:10:00
  *  Author  : Hugo Lizarraga
   *
  *  -------- DS18B20 ----------------
  *
  ******************************************************************************
  */

#ifndef __DS18B20_H
#define __DS18B20_H

#include "main.h"   //


void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

uint8_t DS18B20_Start (void);

void DS18B20_Write (uint8_t data);

uint8_t DS18B20_Read (void);


#endif /* __DS18B20_H */
