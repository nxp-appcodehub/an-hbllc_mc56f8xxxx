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
* @brief  Multiply accumulate functions with 32-bit rounded fractional output 
* 		  in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MACRND_F32_ASMI_H_
#define _MLIB_MACRND_F32_ASMI_H_

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
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  multiply accumulate function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and adds them to
* 			the accumulator:
* 			result = f32Accum + f32Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!	
*		    
****************************************************************************/
extern inline frac32_t MLIB_MacRnd_F32lls_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult2,f32In2);
		
	asm(mac32 f32Mult1,f32In2,f32Accum);	/* round(f32Accum + f32Mult1 * f16Mult2) */

	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  multiply accumulate saturated function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x16) and adds them to
* 			the accumulator:
* 			result = f32Accum + f32Mult1 * f16Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline frac32_t MLIB_MacRndSat_F32lls_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult2,f32In2);
		
	asm(mac32 f32Mult1,f32In2,f32Accum);	/* round(f32Accum + f32Mult1 * f16Mult2) */
		
	asm(nop);

	asm(sat f32Accum);						/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output multiply accumulate function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator and rounds to the upper 32 bits:
* 			result = f32Accum + f32Mult1 * f32Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!				
*
****************************************************************************/
extern inline frac32_t MLIB_MacRnd_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	asm(.optimize_iasm on);
	
	asm(mac32 f32Mult1,f32Mult2,f32Accum);	/* round(f32Accum + f32Mult1 * f32Mult2) */
	
	asm(nop);
	
	asm(.optimize_iasm off);
	
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output multiply accumulate saturated function
* 		  with result rounding
*
* @param  ptr			
* 
* @param  in    		frac16_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and adds them to
* 			the accumulator and rounds to the upper 32 bits:
* 			result = f32Accum + f32Mult1 * f32Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!
*
****************************************************************************/
extern inline frac32_t MLIB_MacRndSat_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	asm(.optimize_iasm on);
		
	asm(mac32 f32Mult1,f32Mult2,f32Accum);	/* round(f32Accum + f32Mult1 * f32Mult2) */
		
	asm(nop);
		
	asm(sat f32Accum);						/* Saturation*/
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MACRND_F32_ASMI_H_ */
