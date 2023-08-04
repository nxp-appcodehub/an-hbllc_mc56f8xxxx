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
* @brief  Absolute value functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ABS_F16_ASM_H_
#define _MLIB_ABS_F16_ASM_H_

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
#define MLIB_Abs_F16_Asmi(f16Val) MLIB_Abs_F16_FAsmi(f16Val)
#define MLIB_AbsSat_F16_Asmi(f16Val) MLIB_AbsSat_F16_FAsmi(f16Val)

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
* @brief  16-bit absolute value function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the absolute value of the input. The function
* 			does not saturate the output, i.e. if input is -32768 the output
* 			will be -32768 too if the saturation mode is turned off.
*
*			SATURATION required for correct functionality!
*
****************************************************************************/
extern inline frac16_t MLIB_Abs_F16_FAsmi(register frac16_t f16Val)
{
		asm(.optimize_iasm on);
		
		asm(abs f16Val);
		
		asm(.optimize_iasm off);
		
		return f16Val;
}

/***************************************************************************//*!
*
* @brief  16-bit absolute value saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [0;1]
*		
* @remarks 	This function returns the absolute value of the input. The function
* 			saturates the output to 32767 if input is -32768.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_AbsSat_F16_FAsmi(register frac16_t f16Val)
{
		register frac32_t f32Val;
		register frac16_t f16Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w	f16Val,f32Val);	
		
		asm(abs f32Val);

		asm(sat f32Val,f16Out);
		
		asm(.optimize_iasm off);
		
		return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ABS_F16_ASM_H_ */

