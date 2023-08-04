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
* @brief  Leading bits count functions with 32-bit fractional input in assembler 
* 
*******************************************************************************/
#ifndef _MLIB_CLB_F32_ASM_H_
#define _MLIB_CLB_F32_ASM_H_

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
#define MLIB_Clb_U16l_Asmi(f32Val) MLIB_Clb_U16l_FAsmi(f32Val)

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
* @brief  This function returns the number of left shifts needed to normalize the 32-bit input.
*
* @param  in    		frac32_t f32Num
*                         - Argument in [-1;1] in frac32_t
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
extern inline uint16_t MLIB_Clb_U16l_FAsmi(register frac32_t f32Val)
{
	register uint16_t ui16Out;

	
	asm(.optimize_iasm on);
		
	asm(clb f32Val, ui16Out);				/* Number of leading bits of f16Val => A */
		
	asm(.optimize_iasm off);
		
	return ui16Out;
}


#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_CLB_F32_ASM_H_ */
