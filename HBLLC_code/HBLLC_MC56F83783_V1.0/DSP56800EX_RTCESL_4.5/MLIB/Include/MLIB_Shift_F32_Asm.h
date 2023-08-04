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
* @brief  Shift functions with 32-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_SHIFT_F32_ASM_H_
#define _MLIB_SHIFT_F32_ASM_H_

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
#define MLIB_Sh1L_F32_Asmi(f32Val) MLIB_Sh1L_F32_FAsmi(f32Val)
#define MLIB_Sh1LSat_F32_Asmi(f32Val) MLIB_Sh1LSat_F32_FAsmi(f32Val)
#define MLIB_Sh1R_F32_Asmi(f32Val) MLIB_Sh1R_F32_FAsmi(f32Val)

#define MLIB_ShL_F32_Asmi(f32Val, u16Sh) MLIB_ShL_F32_FAsmi(f32Val, u16Sh)
#define MLIB_ShLSat_F32_Asmi(f32Val, u16Sh) MLIB_ShLSat_F32_FAsmi(f32Val, u16Sh)
#define MLIB_ShR_F32_Asmi(f32Val, u16Sh) MLIB_ShR_F32_FAsmi(f32Val, u16Sh)

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
* @brief  32-bit one arithmetic shift left function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the left. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Sh1L_F32_FAsmi(register frac32_t f32Val)
{
	asm(.optimize_iasm on);
		
	asm(asl f32Val);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit one arithmetic shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the left and
* 			saturates the result if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Sh1LSat_F32_FAsmi(register frac32_t f32Val)
{
	asm(.optimize_iasm on);
		
	asm(asl f32Val);
		
	asm(sat f32Val);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit one arithmetic shift right function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the input shifted one bit to the right. The function
* 			does not saturate the output.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Sh1R_F32_FAsmi(register frac32_t f32Val)
{
	asm(.optimize_iasm on);
		
	asm(asr f32Val);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift left function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f32Val input shifted by the number of u16Sh to the left. 
* 			The function does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_ShL_F32_FAsmi(register frac32_t f32Val, register uint16_t u16Sh)
{
	asm(.optimize_iasm on);
		
	asm(asll.l u16Sh,f32Val);
		
	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift left function with saturation
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the right  
*                         
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the 32In input shifted by the number of u16Sh to the left. 
* 			The function saturates if necessary.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_ShLSat_F32_FAsmi(register frac32_t f32Val, register uint16_t u16Sh)
{
	register frac32_t f32Value1;
	register int16_t i16Clb;
	register frac16_t f16Out;
	
	asm(.optimize_iasm on);

	asm(tfr f32Val,f32Value1);		/* Makes a copy */

	asm(clb f32Val,i16Clb);    		/* Acquire number of leading ones/zeros minus one of f32Value0 */ 

	asm(asll.l u16Sh,f32Val);		/* Shifts the input to the left */

	asm(bfchg #0x8000,f32Value1.1);	/* Changes the MSB of f32Value1 */

	asm(cmp u16Sh,i16Clb);      	/* i16Clb - u16Sh -> comparing the number of leading ones of f32Value and u16Sh  */
		
	asm(tlt	f32Value1,f32Val);		/* If no space to shift, uses the limit value */

	asm(sat f32Val);				/* Saturates the result */
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

/***************************************************************************//*!
*
* @brief  32-bit arithmetic multi-bit shift right function
*
* @param  ptr			
* 
* @param  in    		frac32_t f32Val
*                         - Argument in [-1;1] in frac32_t
*                       uint16_t u16Sh
*                         - Number of shifts to the left; negative: shifts to the left  
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function returns the f32Val input shifted by the number of u16Sh to the right. 
* 			The function does not saturate the output. 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_ShR_F32_FAsmi(register frac32_t f32Val, register uint16_t u16Sh)
{
	asm(.optimize_iasm on);
		
	asm(asrr.l u16Sh,f32Val);
		
	asm(nop);
		
	asm(.optimize_iasm off);
		
	return f32Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_SHIFT_F32_ASM_H_ */

