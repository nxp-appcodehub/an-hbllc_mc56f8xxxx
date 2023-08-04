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
* @brief  Subtraction functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SUB_F32_ASM_H_
#define _MLIB_SUB_F32_ASM_H_

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
#define MLIB_Sub_F32_Asmi(f32Min, f32Sub) MLIB_Sub_F32_FAsmi(f32Min, f32Sub)
#define MLIB_SubSat_F32_Asmi(f32Min, f32Sub) MLIB_SubSat_F32_FAsmi(f32Min, f32Sub)

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
* @brief  32-bit subtraction function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Min
*                         - Argument in [-1;1] in frac16_t
*						frac32_t f32Sub
*                         - Argument in [-1;1] in frac16_t                         
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of two inputs:
*			result = f32Min - f32Sub
* 			The function does not saturate the output if the saturation mode
* 			is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Sub_F32_FAsmi(register frac32_t f32Min, register frac32_t f32Sub)
{
	asm(.optimize_iasm on);
		
	asm(sub f32Sub,f32Min);
		
	asm(.optimize_iasm off);
		
	return f32Min;
}

/***************************************************************************//*!
*
* @brief  32-bit subtraction saturated function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Min
*                         - Argument in [-1;1] in frac16_t
*						frac32_t f32Sub
*                         - Argument in [-1;1] in frac16_t                         
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the subtraction of two inputs:
*			result = f32Min - f32Sub
*			The function saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_SubSat_F32_FAsmi(register frac32_t f32Min, register frac32_t f32Sub)
{
	asm(.optimize_iasm on);
		
	asm(sub f32Sub,f32Min);

	asm(sat f32Min);
		
	asm(.optimize_iasm off);
		
	return f32Min;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SUB_F32_ASM_H_ */
