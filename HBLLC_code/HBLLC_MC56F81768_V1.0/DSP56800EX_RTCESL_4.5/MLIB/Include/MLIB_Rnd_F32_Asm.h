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
* @brief  Round functions with 32-bit fractional input in assembler
* 
*******************************************************************************/
#ifndef _MLIB_RND_F32_ASM_H_
#define _MLIB_RND_F32_ASM_H_

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
#define MLIB_Rnd_F16l_Asmi(f32Val) MLIB_Rnd_F16l_FAsmi(f32Val)
#define MLIB_RndSat_F16l_Asmi(f32Val) MLIB_RndSat_F16l_FAsmi(f32Val)

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
* @brief  16-bit round function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline frac16_t MLIB_Rnd_F16l_FAsmi(register frac32_t f32Val)
{
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);
		
	asm(rnd f32Val);
		
	asm(move.w f32Val.1,f16Out);
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit round function with saturation
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function rounds the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_RndSat_F16l_FAsmi(register frac32_t f32Val)
{
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);
		
	asm(rnd f32Val);
		
	asm(sat f32Val,f16Out);
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_RND_F32_ASM_H_ */
