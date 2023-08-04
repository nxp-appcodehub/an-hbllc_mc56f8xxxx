/*
 * Flash_config.c
 * Copyright 2019 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 *  Created on: Jan 25, 2019
 *      Author: nxa22573
 */
#include "derivative.h"

typedef struct 
{
	uint8_t tag[4];
	uint8_t reserved_1[12];
	uint8_t enabledPeripherals;
	uint8_t i2cSlaveAddress;
	uint16_t peripheralDetectionTimeout;
	uint8_t reserved_2[2];
	uint8_t reserved_3[2];
	uint8_t reserved_4[4];
	uint8_t reserved_5;
	uint8_t reserved_6;
	uint8_t reserved_7;
	uint8_t reserved_8;
	uint8_t reserved_9[4];
	uint8_t reserved_10[4];
	uint8_t reserved_11;
	uint8_t canConfig1;
	uint16_t canConfig2;
	uint16_t canTxId;
	uint16_t canRxId;
	uint8_t reserved_12[4];
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
        /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 (Note: set SEC to 00b for secure) */ \
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
		/* reserved */ \
		{0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU,0xffU}, \
		/* Enabled Peripherals */ \
		0xffU, \
		/* i2c slave address */ \
		0xffU, \
		/* peripheral detection timeout, in milliseconds */ \
		2000, \
		/* reserved */ \
		{0xffU,0xffU}, \
		{0xffU,0xffU}, \
		{0xffU,0xffU,0xffU,0xffU}, \
		0xffU, \
		0xffU, \
		0xffU, \
		0xffU, \
		{0xffU,0xffU,0xffU,0xffU}, \
		{0xffU,0xffU,0xffU,0xffU}, \
		/* reserved */
		0xffU, \
		/* can config1 */ \
		0xffU, \
		/* can config2 */ \
		0xffffU, \
		/* can TX Id */ \
		0xffffU, \
		/* can RX Id */
		0xffffU, \
		/* reserved */ \
		{0xffU,0xffU,0xffU,0xffU}	
};
		     
#pragma section reserved_BCA end
