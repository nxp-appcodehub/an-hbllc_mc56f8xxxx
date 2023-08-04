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
* @brief  Multiply functions with 32-bit rounded fractional output in assembler
* 		  for V3
* 
*******************************************************************************/
#ifndef _MLIB_MULRND_F32_ASMI_H_
#define _MLIB_MULRND_F32_ASMI_H_

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
* @brief  32-bit x 16-bit multiplier inputs, 32-output multiply function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f32Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used! 	
*
****************************************************************************/
extern inline frac32_t MLIB_MulRnd_F32ls_FAsmi(register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
	register frac32_t f32Result;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Mult2,f32In2);

	asm(mpy32 f32Mult1,f32In2,f32Result);	/* f32Result = f32Mult1 * f16Mult2 (with rounding) */
		
	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief 32-bit x 16-bit multiplier inputs, 32-output multiply saturated function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs:
* 			result = f32Mult1 * f16Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used! 				
*
****************************************************************************/
extern inline frac32_t MLIB_MulRndSat_F32ls_FAsmi(register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
	register frac32_t f32Result;
	
	asm(.optimize_iasm on);
	
	asm(move.w f16Mult2,f32In2);

	asm(mpy32 f32Mult1,f32In2,f32Result);	/* f32Result = f32Mult1 * f16Mult2 (with rounding) */
		
	asm(nop);
		
	asm(sat f32Result);						/* Result saturation */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit x 16-bit multiplier inputs, 32-output multiply negate function with result rounding
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Mult1
*                         - Argument in [-1;1] in frac32_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and negates the result:
* 			result = -f32Mult1 * f16Mult2.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used! 	
*
****************************************************************************/
extern inline frac32_t MLIB_MulNegRnd_F32ls_FAsmi(register frac32_t f32Mult1, register frac16_t f16Mult2)
{
	register frac32_t f32In2;
	register frac32_t f32Result;
		
	asm(.optimize_iasm on);
	
	asm(move.w f16Mult2,f32In2);

	asm(mpy32 f32Mult1,f32In2,f32Result);	/* f32Result = f32Mult1 * f16Mult2 (with rounding) */
		
	asm(nop);

	asm(neg f32Result);						/* Negates result */
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output multiply function with result rounding
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
extern inline frac32_t MLIB_MulRnd_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;
		
	asm(.optimize_iasm on);
		
	asm(mpy32 f32Mult1,f32Mult2,f32Result);		/* f32Result = f32Mult1 * f32Mult2 (with rounding) */
	
	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output multiply saturated function with result rounding
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
* @remarks 	This function multiplies two fractional inputs and rounds to the upper 32 bits:
* 			result = f32Mult1 * f32Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*		    The V3 core instructions used!	
*
****************************************************************************/
extern inline frac32_t MLIB_MulRndSat_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;
			
	asm(.optimize_iasm on);
		
	asm(mpy32 f32Mult1,f32Mult2,f32Result);		/* f32Result = f32Mult1 * f32Mult2 (with rounding) */
	
	asm(nop);
			
	asm(sat f32Result);							/* Result saturation */
			
	asm(.optimize_iasm off);
			
	return f32Result;
}

/***************************************************************************//*!
*
* @brief  32-bit inputs 32-output multiply negate function with result rounding
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
* @remarks 	This function multiplies two fractional inputs and negates the result
*  			and rounds to the upper 16 bits:
* 			result = -f32Mult1 * f32Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*		    The V3 core instructions used!	
*			
****************************************************************************/
extern inline frac32_t MLIB_MulNegRnd_F32_FAsmi(register frac32_t f32Mult1, register frac32_t f32Mult2)
{
	register frac32_t f32Result;
			
	asm(.optimize_iasm on);
			
	asm(mpy32 f32Mult1,f32Mult2,f32Result); 		/* f32Result = f32Mult1 * f32Mult2 (with rounding) */
	
	asm(nop);
			
	asm(neg f32Result);								/* Negates result */
			
	asm(.optimize_iasm off);
			
	return f32Result;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MULRND_F32_ASMI_H_ */
