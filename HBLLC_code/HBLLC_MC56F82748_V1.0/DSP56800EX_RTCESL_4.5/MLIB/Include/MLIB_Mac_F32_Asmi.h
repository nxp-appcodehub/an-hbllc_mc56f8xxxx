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
* @brief  Multiply accumulate functions with 32-bit fractional output 
* 		  in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MAC_F32_ASMI_H_
#define _MLIB_MAC_F32_ASMI_H_

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
* @brief  32-bit accumulator input, 32x32-bit multipliers input, 32-output
* 		  multiply accumulate function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Accum + f32Mult1 * f32Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline frac32_t MLIB_Mac_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{	
	register frac32_t f32Mul;

	asm(.optimize_iasm on);
	
	asm(mpy64 f32Mult1,f32Mult2,f32Mul);	/* f32Mul = f32Mult1 * f32Mult2 */
	
	asm(nop);
	
	asm(add f32Mul,f32Accum);				/* f32Accum = f32Mul + f32Accum */
	
	asm(.optimize_iasm off);
	
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 32x16-bit multipliers input, 32-output
* 		  multiply accumulate saturated function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Mult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (32x32) and adds them to
* 			the accumulator:
* 			result = f32Accum + f32Mult1 * f32Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!			
*
****************************************************************************/
extern inline frac32_t MLIB_MacSat_F32_FAsmi(register frac32_t f32Accum, register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Mul;
	
	asm(.optimize_iasm on);
	
	asm(mpy64 f32Mult1,f32Mult2,f32Mul);	/* f32Mul = f32Mult1 * f32Mult2 */
	
	asm(nop);
	
	asm(add f32Mul,f32Accum);				/* f32Accum = f32Mul + f32Accum */

	asm(sat f32Accum);						/* Saturation */
		
	asm(.optimize_iasm off);
	
	return f32Accum;	
}


#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MAC_F32_ASMI_H_ */

