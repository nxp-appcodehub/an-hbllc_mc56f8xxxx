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
* @brief  Multiply subtract functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MSU_F32_ASM_H_
#define _MLIB_MSU_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Msu_F32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_Msu_F32lss_Asmi(f32Accum, f16Mult1, f16Mult2) MLIB_Msu_F32lss_FAsmi(f32Accum, f16Mult1, f16Mult2)
#define MLIB_MsuSat_F32lss_Asmi(f32Accum, f16Mult1, f16Mult2) MLIB_MsuSat_F32lss_FAsmi(f32Accum, f16Mult1, f16Mult2)

#define MLIB_Msu_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_Msu_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)
#define MLIB_MsuSat_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_MsuSat_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)

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
* @brief  32-bit accumulator input, 16-bit multipliers input,
* 		  32-output multiply subtract function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f32Accum - f16Mult1 * f16Mult2.
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu_F32lss_FAsmi(register frac32_t f32Accum, register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	asm(.optimize_iasm on);
		
	asm(mac -f16Mult1,f16Mult2,f32Accum);

	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

/***************************************************************************//*!
*
* @brief  32-bit accumulator input, 16-bit multipliers input,
* 		  32-output multiply subtract saturated function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Accum
*                         - Argument in [-1;1] in frac32_t, accumulator
*                       frac16_t f16Mult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Mult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function multiplies two fractional inputs and subtracts them from
* 			the accumulator:
* 			result = f32Accum - f16Mult1 * f16Mult2.
* 			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_MsuSat_F32lss_FAsmi(register frac32_t f32Accum, register frac16_t f16Mult1, register frac16_t f16Mult2)
{
	asm(.optimize_iasm on);
		
	asm(mac -f16Mult1,f16Mult2,f32Accum);

	asm(nop);
		
	asm(sat f32Accum);
		
	asm(.optimize_iasm off);
		
	return f32Accum;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSU_F32_ASM_H_ */
