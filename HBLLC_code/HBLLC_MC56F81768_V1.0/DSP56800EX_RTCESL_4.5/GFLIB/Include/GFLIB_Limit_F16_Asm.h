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
* @brief  Limit functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_LIMIT_F16_ASM_H_
#define _GFLIB_LIMIT_F16_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_Limit_F16_Asmi(f16Val, f16LLim, f16ULim) GFLIB_Limit_F16_FAsmi(f16Val, f16LLim, f16ULim)
#define GFLIB_UpperLimit_F16_Asmi(f16Val, f16ULim) GFLIB_UpperLimit_F16_FAsmi(f16Val, f16ULim)
#define GFLIB_LowerLimit_F16_Asmi(f16Val, f16LLim) GFLIB_LowerLimit_F16_FAsmi(f16Val, f16LLim)

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
* @brief  Limit function
*
* @param  ptr			

* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16LLim  
*                         - LowerLimit in [-1;1] in frac16_t
*                       frac16_t f16ULim
*						  - UpperLimit in [-1;1] in frac16_t
*						  
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper f16ULim and 
* 			lower f16LLim limits. The upper limit must >= lower limit.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_Limit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16LLim, register frac16_t f16ULim)
{
	register frac32_t f32Val;
	
	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);				/* Copies argument to f32Val */
	asm(cmp.w	f16ULim,f32Val);			/* Compares f32Val to upper limit */
	asm(tgt		f16ULim,f32Val);			/* Upper limit to f32Val if f32Val > upper limit */
	asm(cmp.w	f16LLim,f32Val);			/* Compares new f32Val to lower limit */
	asm(tlt		f16LLim,f32Val);			/* Lower limit to f32Val if f32Val < lower limit */
	asm(move.w 	f32Val.1,f16Val);

	asm(.optimize_iasm off);
	
	return f16Val;
}

/***************************************************************************//*!
*
* @brief  Upper limit function
*
* @param  ptr	
* 		
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16ULim
*						  - Max output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the upper 
*			limit in the f16ULim variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_UpperLimit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16ULim)
{
	register frac32_t f32Val;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);			/* Copies argument to f32Out */
	asm(cmp.w	f16ULim,f16Val);		/* Compares f16Val to upper limit */
	asm(tgt		f16ULim,f32Val);		/* Upper limit to f32Out if f16Val > upper limit */
	asm(move.w	f32Val.1,f16Val);		/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Val;
}


/***************************************************************************//*!
*
* @brief  Lower limit function
*
* @param  ptr	
* 		
* @param  in    		frac16_t f16Val
*                         - Argument in [-1;1] in frac16_t
*						frac16_t fwLowerLimit
*						  - Min output trim in [-1;1] in frac16_t
*
* @return This function returns
*     - frac16_t value [-1;1]
*		
* @remarks 	This function trims the argument according to the lower 
*			limit in the fwLowerLimit variable.
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_LowerLimit_F16_FAsmi(register frac16_t f16Val, register frac16_t f16LLim)
{
	register frac32_t f32Val;

	asm(.optimize_iasm on);
	
	asm(move.w	f16Val,f32Val);		/* Copies argument to f32Val */
	asm(cmp.w	f16LLim,f16Val);	/* Compares f16Val to upper limit */
	asm(tlt		f16LLim,f32Val);	/* Lower limit to f32Val if f32Val < lower limit */
	asm(move.w	f32Val.1,f16Val);	/* Copies the result */
	
	asm(.optimize_iasm off);
	
	return f16Val;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_LIMIT_F16_ASM_H_ */
