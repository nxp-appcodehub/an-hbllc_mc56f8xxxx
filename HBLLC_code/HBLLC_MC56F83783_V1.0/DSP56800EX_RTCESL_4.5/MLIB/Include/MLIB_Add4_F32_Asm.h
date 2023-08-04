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
* @brief  Sum of 4 values functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_ADD4_F32_ASM_H_
#define _MLIB_ADD4_F32_ASM_H_

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
#define MLIB_Add4_F32_Asmi(f32Add1, f32Add2, f32Add3, f32Add4) MLIB_Add4_F32_FAsmi(f32Add1, f32Add2, f32Add3, f32Add4)
#define MLIB_Add4Sat_F32_Asmi(f32Add1, f32Add2, f32Add3, f32Add4) MLIB_Add4Sat_F32_FAsmi(f32Add1, f32Add2, f32Add3, f32Add4)

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
* @brief  32-bit addition function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add3
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add4
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of four inputs. The function
* 			does not saturate the output if the saturation mode is turned off.
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Add4_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2, register frac32_t f32Add3, register frac32_t f32Add4)
{
	asm(.optimize_iasm on);
	
	asm(add f32Add2,f32Add1);	/* f32Add1 + f32Add1 */
	
	asm(add f32Add3,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 */
	
	asm(add f32Add4,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 + f32Add4 */
	
	asm(.optimize_iasm off);
	
	return f32Add1;
}

/***************************************************************************//*!
*
* @brief  32-bit addition saturated function of four arguments
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Add1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add2
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32Add3
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32Add4
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the addition of four inputs. The function
* 			saturates the output.
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Add4Sat_F32_FAsmi(register frac32_t f32Add1, register frac32_t f32Add2, register frac32_t f32Add3, register frac32_t f32Add4)
{
	asm(.optimize_iasm on);
	
	asm(add f32Add2,f32Add1);	/* f32Add1 + f32Add2 */
	
	asm(add f32Add3,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 */
	
	asm(add f32Add4,f32Add1);	/* f32Add1 + f32Add2 + f32Add3 + f32Add4 */
	
	asm(sat f32Add1);			/* Saturation */
	
	asm(.optimize_iasm off);
	
	return f32Add1;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_ADD4_F32_ASM_H_ */
