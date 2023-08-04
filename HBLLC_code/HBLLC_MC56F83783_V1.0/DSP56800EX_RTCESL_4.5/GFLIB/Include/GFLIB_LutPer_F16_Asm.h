/*******************************************************************************
*
 * Copyright 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
* 
*
****************************************************************************//*!
*
* @brief  Look-up table algorithm implemented in assembler 
* 
*******************************************************************************/
#ifndef _GFLIB_LUTPER_F16_ASM_H_
#define _GFLIB_LUTPER_F16_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_LutPer1D_F16_Asm(f16X, pf16Table, u16TableSize) GFLIB_LutPer1D_F16_FAsm(f16X, pf16Table, u16TableSize)
#define GFLIB_LutPer2D_F16_Asm(f16X, f16Y, psParam) GFLIB_LutPer2D_F16_FAsm(f16X, f16Y, psParam)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	uint16_t  u16ShamOffset1;
	uint16_t  u16ShamOffset2;
	frac16_t *pf16Table;
} GFLIB_LUTPER2D_T_F16;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Look-up table algorithm using linear interpolation
*
* @param  ptr   		frac16_t *pf16Table
*						  - Pointer to the table values
* @param  in    		frac16_t f16X
*                         - Argument in [-1;1] in frac16_t
*						uint16_t u16TableSize
*						  - Size of the look-up table in bit shifts, 3 for 8 values
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function calculates the number from the table using interpolation
*			of two values in the table.
*			The table size must be based on 2^x, i.e. 256 values contains
*			the size 8.
*
*			SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern asm frac16_t GFLIB_LutPer1D_F16_FAsm(frac16_t f16X, const frac16_t *pf16Table, uint16_t u16TableSize);


extern asm frac16_t GFLIB_LutPer2D_F16_FAsm(frac16_t f16X, frac16_t f16Y, const GFLIB_LUTPER2D_T_F16 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_LUTPER_F16_ASM_H_ */
