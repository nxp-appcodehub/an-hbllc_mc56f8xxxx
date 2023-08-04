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
* @brief  sign functions with 16-bit fractional input in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SIGN_F16_ASM_H_
#define _MLIB_SIGN_F16_ASM_H_

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
#define MLIB_Sign_F16_Asmi(f16Val) MLIB_Sign_F16_FAsmi(f16Val)

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
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the sign of the argument:
*			f16Val = 0: returns 0
*			f16Val > 0: returns 32767 (fractional 1.0)
*			f16Val < 0: returns -32768 (fractional -1.0)
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t MLIB_Sign_F16_FAsmi(register frac16_t f16Val)
{
	register frac32_t f32Val;
		
	asm(.optimize_iasm on);
		
	asm(move.w f16Val,f32Val);		/* f32Out = f16Val */
		
	asm(bfchg #0x8000,f32Val.1);	/* Changes the MSB f32Val */
		
	asm(tst.w f16Val);				/* Tests f16Val */
		
	asm(teq f16Val,f32Val);			/* If f16Val = 0, then f32Val = 0 */
		
	asm(move.w f32Val,f16Val);		/* f16Val = f32Val.Low with saturation*/
		
	asm(.optimize_iasm off);
		
	return f16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SIGN_F16_ASM_H_ */

