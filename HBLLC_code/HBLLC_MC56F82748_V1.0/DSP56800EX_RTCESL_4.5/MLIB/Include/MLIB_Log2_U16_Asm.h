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
* @brief  Binary logarithm function in assembler
* 
*******************************************************************************/
#ifndef MLIB_LOG2ASM_H_
#define MLIB_LOG2ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Log2_U16_Asmi(u16Val) MLIB_Log2_U16_FAsmi(u16Val)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Binary logarithm of 16-bit input
*
* @param  ptr			
* 
* @param  in    		uint16_t u16Val
*                         - Argument in uint16_t
*
* @return This function returns
*     - uint16_t value
*		
* @remarks 	This function returns the 16-bit integer part of binary logarithm of the input. 
* 			Returns 0 for input u16Val == 0.  
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline uint16_t MLIB_Log2_U16_FAsmi(register uint16_t u16Val)
{
	register uint32_t u32Temp;
	register uint16_t u16Shifts = 14;  
	
	asm(.optimize_iasm on);

	asm(move.w #-1,Y1);		
		
	asm(clb u16Val,u32Temp);	/* Number of leading bits of ui16In => u32Temp */
				
	asm(tst.w u16Val);			/* Compares ui16In to 0 */

	asm(tle	u16Shifts,u32Temp);	/* If (u16Val <= 0), then u16Shifts => u32Temp */
		
	asm(sub u32Temp,u16Shifts);	/* u16Shifts = u16Shifts - u32Temp */
				
	asm(move.w	u16Shifts, u16Val);			
		
	asm(.optimize_iasm off);
		
	return u16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_LOG2ASM_H_ */
