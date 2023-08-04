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
* @brief  Addition functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ADD_F16_ASM_H_
#define _MLIB_ADD_F16_ASM_H_

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
#define MLIB_Add_F16_Asmi(f16Add1, f16Add2) MLIB_Add_F16_FAsmi(f16Add1, f16Add2)
#define MLIB_AddSat_F16_Asmi(f16Add1, f16Add2) MLIB_AddSat_F16_FAsmi(f16Add1, f16Add2)

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
* @brief  16-bit addition function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Add1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2
*                         - Argument in [-1;1] in frac16_t                         
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac16_t MLIB_Add_F16_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2)
{
	asm(.optimize_iasm on);
		
	asm(add f16Add2,f16Add1);		/* f16Add1 + f16Add2 */
		
	asm(.optimize_iasm off);
		
	return f16Add1;
}

/***************************************************************************//*!
*
* @brief  16-bit addition saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16Add1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16Add2
*                         - Argument in [-1;1] in frac16_t                         
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the addition of two inputs. The function
* 			saturates the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_AddSat_F16_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2)
{
	register frac32_t f32Val;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);
		
	asm(move.w	f16Add1,f32Val);	
		
	asm(add f16Add2,f32Val);		/* f16Add1 + f16Add2 */

	asm(sat f32Val,f16Out);			/* Saturation */
		
	asm(.optimize_iasm off);
		
	return f16Out;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ADD_F16_ASM_H_ */
