/**
  ******************************************************************************
  *  CRC_dnp-modbus.h
  *  Created : 04/03/2021 15:10:00
  *  Author  : Hugo Lizarraga
  *            Digicom S.R.L.
  *
  ******************************************************************************
  */

#ifndef __CRC_dnp_modbus_H
#define __CRC_dnp_modbus_H

#include "stdio.h"   //


void computeCRC(uint16_t *crcAccum, unsigned char dataOctet);

void cmpt_crcMODBUS(uint16_t *crcAccum, unsigned char dataOctet);


#endif /* __CRC_dnp_modbus_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
