/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "derivative.h"

typedef struct 
{
	uint8_t tag[4];
	uint32_t crcStartAddress;
	uint32_t crcByteCount;
	uint32_t crcExpectedValue;
	uint8_t enabledPeripherals;
	uint8_t i2cSlaveAddress;
	uint16_t peripheralDetectionTimeout;
	uint8_t reserved_2[2];
	uint8_t reserved_3[2];
	uint8_t reserved_4[4];
	uint8_t reserved_5;
	uint8_t reserved_6;
	uint8_t bootFlags;
	uint8_t reserved_7;
	uint8_t reserved_8[4];
	uint8_t reserved_9[4];
	uint8_t reserved_10;
	uint8_t reserved_11[7];
	uint8_t reserved_12[4];
	uint8_t reserved_13[12];
}Bootloader_Cofiguration_Area_TYPE;

#pragma define_section reserved_FCF "reserved_FCF.text"  RX
#pragma section reserved_FCF begin
static const uint8_t _flash_config_field[] = {
		
        /* NV_BACKKEY3: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY2: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY1: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY0: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY7: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY6: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY5: KEY=0xFF */ \
        0xFFU, \
        /* NV_BACKKEY4: KEY=0xFF */ \
        0xFFU, \
        /* NV_FPROT3: PROT=0xFF */ \
        0xFFU, \
        /* NV_FPROT2: PROT=0xFF */ \
        0xFFU, \
        /* NV_FPROT1: PROT=0xFF */ \
        0xFFU, \
        /* NV_FPROT0: PROT=0xFF */ \
        0xFFU, \
        /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */ \
        0x7EU, \
        /* NV_FOPT: FOPT[7:6] = 11b means boot from ROM, other value means boot from Flash */ \
        0x3FU, \
        /* Reserved */ \
        0xFFU, \
        /* Reserved */ \
        0xFFU
};
#pragma section reserved_FCF end

#pragma define_section reserved_BCA "reserved_BCA.text"  RX
#pragma section reserved_BCA begin
static const Bootloader_Cofiguration_Area_TYPE BCA_config = {
		
		/* tag */ \
		{'k', 'c', 'f', 'g'}, \
		/* Start address for image CRC check */ \
		0xffffffffU, \
		/* Byte count for image CRC check */ \
		0xffffffffU,
		/* Expected CRC value for image CRC check */ \
		0xffffffffU,
		/* Enabled Peripherals */ \
		0xffU, \
		/* i2c slave address */ \
		0xffU, \
		/* peripheral detection timeout, in milliseconds */ \
		6000, \
		/* reserved */ \
		{0xffU,0xffU}, \
		{0xffU,0xffU}, \
		{0xffU,0xffU,0xffU,0xffU}, \
		0xffU, \
		0xffU, \
		/* direct boot flag, 0xfe represents direct boot */
		0xffU, \
		/* reserved */
		0xffU, \
		{0xffU,0xffU,0xffU,0xffU}, \
		{0xffU,0xffU,0xffU,0xffU}, \
		/* reserved */
		0xffU, \
		/* reserved */ \
		{0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU}, \
		/* reserved */ \
		{0xffU,0xffU,0xffU,0xffU}, \
		/* reserved */ \
		{0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU}
};
		     
#pragma section reserved_BCA end

