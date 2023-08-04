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
* @brief  Leading bits count functions with 16-bit fractional input in assembler 
* 
*******************************************************************************/
#ifndef _MLIB_CLB_F16_ASM_H_
#define _MLIB_CLB_F16_ASM_H_

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
#define MLIB_Clb_U16s_Asmi(f16Val) MLIB_Clb_U16s_FAsmi(f16Val)

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
* @brief  This function returns the number of left shifts needed to normalize the 16-bit input.
*
* @param  in    		frac16_t f16Num
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - uint16_t value
*		
* @remarks 	Depending on the sign of the input value the function counts and returns the 
* 			number of the left shift needed to get an equality between input value and 
* 			the maximum fractional values "1" or "-1". For the input "0" returns "0".
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline uint16_t MLIB_Clb_U16s_FAsmi(register frac16_t f16Val)
{
	register uint16_t ui16Out;
	
	asm(.optimize_iasm on);
		
	asm(clb f16Val, ui16Out);		/* Number of leading bits of f16Val => A */
		
	asm(and.w	#15, ui16Out);		/* Mask for input f16Val = 0 */
	
	asm(.optimize_iasm off);
		
	return ui16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_CLB_F16_ASM_H_ */
