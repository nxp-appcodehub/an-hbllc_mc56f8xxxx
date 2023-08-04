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
* @brief  Multiply functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F16_ASM_H_
#define _MLIB_MUL_F16_ASM_H_

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
#define MLIB_Mul_F16_Asmi(f16Mult1, f16Mult2) MLIB_Mul_F16_FAsmi(f16Mult1, f16Mult2)
#define MLIB_MulSat_F16_Asmi(f16Mult1, f16Mult2) MLIB_MulSat_F16_FAsmi(f16Mult1, f16Mult2)
#define MLIB_MulNeg_F16_Asmi(f16Mult1, f16Mult2) MLIB_MulNeg_F16_FAsmi(f16Mult1, f16Mult2)

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
* @brief  16-bit inputs 16-bit output multiply function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_Mul_F16_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
	register frac16_t f16Out;		
		
	asm(.optimize_iasm on);
		
	asm(mpy f16Mult1,f16Mult2,f32Value);	/* f16Mult1 * f16Mult2 -> f32Value */

	asm(nop);
		
	asm(move.w f32Value.1,f16Out);			/* f32Value -> f16Out */
		
	asm(.optimize_iasm off);
		
	return f16Out;		
}

/***************************************************************************//*!
*
* @brief  16-bit input 16-bit output multiply saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16Mult1 * f16Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_MulSat_F16_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);
		
	asm(mpy f16Mult1,f16Mult2,f32Value);	/* f16Mult1 * f16Mult2 -> f32Value */
		
	asm(nop);
		
	asm(sat f32Value,f16Out);				/* Saturation of result*/
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

/***************************************************************************//*!
*
* @brief 16-bit inputs 16-output multiply negate function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result:
* 			result = -f16Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_MulNeg_F16_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
	register frac16_t f16Out;
		
	asm(.optimize_iasm on);
		
	asm(mpy -f16Mult1,f16Mult2,f32Value);	/* - f16Mult1 * f16Mult2 -> f32Value */
		
	asm(nop);	
	
	asm(move.w f32Value.1,f16Out);			/* f32Value -> f16Out */
		
	asm(.optimize_iasm off);
	
	return f16Out;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MUL_F16_ASM_H_ */
