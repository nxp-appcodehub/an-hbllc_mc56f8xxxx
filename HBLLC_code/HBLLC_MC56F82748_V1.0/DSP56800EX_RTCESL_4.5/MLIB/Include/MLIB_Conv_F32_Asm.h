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
* @brief  Convert functions without scale factor and with 32-bit fractional 
* 		  input or output in assembler
* 
*******************************************************************************/
#ifndef MLIB_CONV_F32_ASM_H_
#define MLIB_CONV_F32_ASM_H_

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
#define MLIB_Conv_F16l_Asmi(f32Val) MLIB_Conv_F16l_FAsmi(f32Val)
#define MLIB_Conv_F32s_Asmi(f16Val) MLIB_Conv_F32s_FAsmi(f16Val)

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
* @brief  32-bit fractional to 16-bit fractional convert function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function convert the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function does not saturate the output 
* 			if the saturation mode is turned off.
*
*			SATURATION independent!
*
****************************************************************************/
extern inline frac16_t MLIB_Conv_F16l_FAsmi(register frac32_t f32Val)
{
		register frac16_t f16Out;	
		
		asm(.optimize_iasm on);	
		
		asm(move.w f32Val.1,f16Out);		/* f16Out = f32Val.1 */
		
		asm(.optimize_iasm off);	
		
		return f16Out;
}

/***************************************************************************//*!
*
* @brief  16-bit fractional to 32-bit fractional convert function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function convert the lower 16-bits of the 32-bit input and
* 			returns the upper 16-bit. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION independent!
*
****************************************************************************/
extern inline frac32_t MLIB_Conv_F32s_FAsmi(register frac16_t f16Val)
{
		register frac32_t f32Out;
	
		asm(.optimize_iasm on);
		
		asm(move.w f16Val,f32Out);		/* f32Val = f16Val << 16 */
		
		asm(.optimize_iasm off);
		
		return f32Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_CONV_F32_ASM_H_ */
