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
* @brief  Addition functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ADD_F32_ASM_H_
#define _MLIB_ADD_F32_ASM_H_

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
#define MLIB_Add_F32_Asmi(f32Add1, f32Add2) MLIB_Add_F32_FAsmi(f32Add1, f32Add2)
#define MLIB_AddSat_F32_Asmi(f32Add1, f32Add2) MLIB_AddSat_F32_FAsmi(f32Add1, f32Add2)

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
* @brief  32-bit addition function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Add_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2)
{
	asm(.optimize_iasm on);
		
	asm(add f32Add2,f32Add1);	/* f32Add1 + f32Add2 */
		
	asm(.optimize_iasm off);
		
	return f32Add1;
}

/***************************************************************************//*!
*
* @brief  32-bit addition saturated function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_AddSat_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2)
{
	asm(.optimize_iasm on);
		
	asm(add f32Add2,f32Add1);		/* f32Add1 + f32Add2 */

	asm(sat f32Add1);				/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f32Add1;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ADD_F32_ASM_H_ */
