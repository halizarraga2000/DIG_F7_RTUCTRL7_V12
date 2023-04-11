/**
  ******************************************************************************
  *  CRC_dnp-modbus.c
  *  Created : 04/03/2021 15:10:00
  *  Author  : Hugo Lizarraga
  *            Digicom S.R.L.
  *
  *  -------- DNP ----------------
  *  Polynomial: X16+X13+X12+X11+X10+X8+X6+X5+X2+X0
  *  InitValor: 0x0000
  *  Semilla  -> Normal: 0x3D65   -> Reversed: 0xA6BC
  *
  *  -------- MODBUS -------------
  *  Polynomial: X16+X15+X2+X0
  *  InitValor: 0xFFFF
  *  Semilla  -> Normal: 0x8005   -> Revers  ed: 0xA001
  *
  ******************************************************************************
  */

#include <CRC_dnp_modbus.h>

void computeCRC(uint16_t *crcAccum, unsigned char dataOctet)
{
//iniciar crc=0x0000 antes de llamar a esta función
//y negar crc después de computar todos los datos (crc=~crc;).

	unsigned char i;
	//unsigned int temp;
	uint16_t temp;
	for(i=0; i<8; i++)
	{
		temp=(*crcAccum ^ dataOctet) & 1;
		*crcAccum>>=1;
		dataOctet>>=1;
		if(temp)*crcAccum ^= 0xA6BC;  //Semilla Reversed
	}
}

void cmpt_crcMODBUS(uint16_t *crcAccum, unsigned char dataOctet)
{
	//iniciar crcMODBUS=0xFFFF antes de llamar a esta función
	//y NO negar crcMODBUS después de computar los datos.
		unsigned char i;
		uint16_t temp;
		for(i=0;i<8;i++)
		{
			temp=(*crcAccum ^ dataOctet)&1;
			*crcAccum>>=1;
			dataOctet>>=1;
			if(temp)*crcAccum ^= 0xA001;  //Semilla Reversed
		}

}
