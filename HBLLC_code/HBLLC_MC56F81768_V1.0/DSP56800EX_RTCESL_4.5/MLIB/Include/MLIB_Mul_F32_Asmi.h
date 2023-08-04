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
* @brief  Multiply functions with 32-bit fractional output in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F32_ASMI_H_
#define _MLIB_MUL_F32_ASMI_H_

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
* @brief  32-bit x 32-bit multiplier inputs, 32-output multiply function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f32Mult1 * f32Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac32_t MLIB_Mul_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;	
	
	asm(.optimize_iasm on);
	
	asm(mpy64 f32Mult1,f32Mult2,f32Result);		/* f32Result = f32Mult1 * f32Mult2 (without rounding) */

	asm(nop);
		
	asm(.optimize_iasm off);
	
	return f32Result;	
}

/***************************************************************************//*!
*
* @brief 32-bit x 32-bit multiplier inputs, 32-output multiply saturated function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f32Mult1 * f32Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac32_t MLIB_MulSat_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;	
	
	asm(.optimize_iasm on);
	
	asm(mpy64 f32Mult1,f32Mult2,f32Result);		/* f32Result = f32Mult1 * f32Mult2 (without rounding) */
	
	asm(nop);
	
	asm(sat f32Result);							/* Result saturation */
	
	asm(.optimize_iasm off);
	
	return f32Result;	
}

/***************************************************************************//*!
*
* @brief 32-bit x 32-bit inputs 32-output multiply negate function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result:
* 			result = -f32Mult1 * f32Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac32_t MLIB_MulNeg_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;	
	
	asm(.optimize_iasm on);
	
	asm(mpy64 f32Mult1,f32Mult2,f32Result);		/* f32Result = f32Mult1 * f32Mult2 (without rounding) */
	
	asm(nop);
	
	asm(neg f32Result);							/* Negates result */
	
	asm(.optimize_iasm off);
	
	return f32Result;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MUL_F32_ASMI_H_ */
