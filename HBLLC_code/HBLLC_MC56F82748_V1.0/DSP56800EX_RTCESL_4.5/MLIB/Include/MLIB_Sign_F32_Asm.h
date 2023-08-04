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
* @brief  sign functions with 32-bit fractional input in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SIGN_F32_ASM_H_
#define _MLIB_SIGN_F32_ASM_H_

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
#define MLIB_Sign_F32_Asmi(f32Val) MLIB_Sign_F32_FAsmi(f32Val)

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
* @brief  Sign function
*
* @param  ptr
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the sign of the argument:
*			f32Val = 0: returns 0
*			f32Val > 0: returns 2147483647 (fractional 1.0)
*			f32Val < 0: returns -2147483648 (fractional -1.0)
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Sign_F32_FAsmi(register frac32_t f32Val)
{
	register frac32_t f32Out;
		
	asm(.optimize_iasm on);
		
	asm(tfr f32Val,f32Out);			/* f32Out = f32Val */
		
	asm(bfchg #0x8000,f32Out.1);	/* Changes the MSB f32Out */
		
	asm(sat	f32Out);				/* Saturates f32Out */
		
	asm(tst f32Val);				/* Tests f32Val */
		
	asm(teq f32Val,f32Out);			/* If f32Val = 0, then f32Out = 0 */
		
	asm(.optimize_iasm off);
		
	return f32Out;		
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SIGNASM_H_ */
