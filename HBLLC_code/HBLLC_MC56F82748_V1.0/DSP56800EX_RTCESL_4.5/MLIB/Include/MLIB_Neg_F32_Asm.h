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
* @brief  Negative value functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_NEG_F32_ASM_H_
#define _MLIB_NEG_F32_ASM_H_

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
#define MLIB_Neg_F32_Asmi(f32Val) MLIB_Neg_F32_FAsmi(f32Val)
#define MLIB_NegSat_F32_Asmi(f32Val) MLIB_NegSat_F32_FAsmi(f32Val)

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
* @brief  32-bit negative value function
*
* @param  ptr			
* 
* @param  in    		frac16_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the negative value of the input. The function
* 			does not saturate the output, i.e. if input is 0x8000 0000 the output
* 			will be 0x8000 0000 too if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline frac32_t MLIB_Neg_F32_FAsmi(register frac32_t f32Val)
{
	asm(.optimize_iasm on);
		
	asm(neg f32Val);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit negative value saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f32Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the negative value of the input. The function
* 			saturates the output to 0x7FFF FFFF if input is 0x8000 0000.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_NegSat_F32_FAsmi(register frac32_t f32Val)
{
	asm(.optimize_iasm on);
		
	asm(neg f32Val);

	asm(sat f32Val);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_NEG_F32_ASM_H_ */
