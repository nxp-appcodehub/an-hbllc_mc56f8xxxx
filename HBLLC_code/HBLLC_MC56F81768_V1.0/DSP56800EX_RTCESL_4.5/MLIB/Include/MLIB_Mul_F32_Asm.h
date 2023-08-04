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
* @brief  Multiply functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F32_ASM_H_
#define _MLIB_MUL_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Mul_F32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Mul_F32ss_Asmi(f16Mult1, f16Mult2) MLIB_Mul_F32ss_FAsmi(f16Mult1, f16Mult2)
#define MLIB_MulSat_F32ss_Asmi(f16Mult1, f16Mult2) MLIB_MulSat_F32ss_FAsmi(f16Mult1, f16Mult2)
#define MLIB_MulNeg_F32ss_Asmi(f16Mult1, f16Mult2) MLIB_MulNeg_F32ss_FAsmi(f16Mult1, f16Mult2)

#define MLIB_Mul_F32_Asmi(f32Mult1, f32Mult2) MLIB_Mul_F32_FAsmi(f32Mult1, f32Mult2)
#define MLIB_MulSat_F32_Asmi(f32Mult1, f32Mult2) MLIB_MulSat_F32_FAsmi(f32Mult1, f32Mult2)
#define MLIB_MulNeg_F32_Asmi(f32Mult1, f32Mult2) MLIB_MulNeg_F32_FAsmi(f32Mult1, f32Mult2)

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
* @brief  16-bit multiplier inputs, 32-bit output multiply function
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
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Mul_F32ss_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
	
	asm(.optimize_iasm on);
		
	asm(mpy f16Mult1,f16Mult2,f32Value);
		
	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Value;
}

/***************************************************************************//*!
*
* @brief  16-bit multipliers input, 32-bit output multiply saturated function
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
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f16Mult1 * f16Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_MulSat_F32ss_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
		
	asm(.optimize_iasm on);
		
	asm(mpy f16Mult1,f16Mult2,f32Value);

	asm(nop);
		
	asm(sat f32Value);			/* Saturates result*/
		
	asm(.optimize_iasm off);
		
	return f32Value;
}

/***************************************************************************//*!
*
* @brief  16-bit multiplier inputs, 32-output multiply negate function
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
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result:
* 			result = -f16Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_MulNeg_F32ss_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Value;
	
	asm(.optimize_iasm on);
		
	asm(mpy -f16Mult1,f16Mult2,f32Value);
		
	asm(.optimize_iasm off);

	asm(nop);
		
	return f32Value;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MUL_F32_ASM_H_ */
