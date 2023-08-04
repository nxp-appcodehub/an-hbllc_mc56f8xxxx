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
* @brief  Subtraction functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SUB_F16_ASM_H_
#define _MLIB_SUB_F16_ASM_H_

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
#define MLIB_Sub_F16_Asmi(f16Min, f16Sub) MLIB_Sub_F16_FAsmi(f16Min, f16Sub)
#define MLIB_SubSat_F16_Asmi(f16Min, f16Sub) MLIB_SubSat_F16_FAsmi(f16Min, f16Sub)

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
* @brief  16-bit subtraction function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Min
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub
*                         - Argument in [-1;1] in frac16_t*                         
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of two inputs:
*			result = f16Min - f16Sub
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_Sub_F16_FAsmi(register frac16_t f16Min, register frac16_t f16Sub)
{
	asm(.optimize_iasm on);
		
	asm(sub f16Sub,f16Min);
		
	asm(.optimize_iasm off);
		
	return f16Min;
}

/***************************************************************************//*!
*
* @brief  16-bit subtraction saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Min
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Sub
*                         - Argument in [-1;1] in frac16_t*                         
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of two inputs:
*			result = f16Min - f16Sub
*			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_SubSat_F16_FAsmi(register frac16_t f16Min, register frac16_t f16Sub)
{
	register frac32_t f32Val;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);
		
	asm(move.w	f16Min,f32Val);	
		
	asm(sub f16Sub,f32Val);

	asm(sat f32Val,f16Out);
	
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SUB_F16_ASM_H_ */
