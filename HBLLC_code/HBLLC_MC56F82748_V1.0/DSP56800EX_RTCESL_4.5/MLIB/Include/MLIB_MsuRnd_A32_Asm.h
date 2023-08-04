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
* @brief  Multiply subtract functions with 32-bit rounded accumulator output 
* 		  in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MSURND_A32_ASM_H_
#define _MLIB_MSURND_A32_ASM_H_

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
#define MLIB_MsuRnd_A32ass_Asmi(a32Accum, f16Mult1, f16Mult2) MLIB_MsuRnd_A32ass_FAsmi(a32Accum, f16Mult1, f16Mult2)

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
* @brief  32-bit accumulator input, 16x16-bit multipliers input, 32-bit output
* 		  multiply subtract function with result rounding
*
* @param  ptr			
* 
* @param  in    		acc32_t a32Accum
*                         - Argument in [-1;1] in acc32_t, accumulator
*                       frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - acc32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs (16x16) and subtracts them from
* 			the accumulator:
* 			result = a32Accum - f16Mult1 * f16Mult2.
* 			The function does not saturate the output.
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline acc32_t MLIB_MsuRnd_A32ass_FAsmi(register acc32_t a32Accum, register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32Mul;
	
	asm(.optimize_iasm on);

	asm(mpyr f16Mult1, f16Mult2, f32Mul);	/* f16Mult1 * f16Mult2 */
	
	asm(nop);
	
	asm(asr16 f32Mul);						/* (f16Mult1 * f16Mult2) >> 16 */
	
	asm(sub f32Mul, a32Accum);				/* a32Accum - (f16Mult1 * f16Mult2) >> 16 */

	asm(.optimize_iasm off);

	return a32Accum;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSURND_A32_ASM_H_ */
