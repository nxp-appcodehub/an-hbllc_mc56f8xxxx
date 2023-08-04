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
* @brief  Subtraction of two products functions with 32-bit rounded fractional 
* 		  output in assembler for V3
* 
*******************************************************************************/
#ifndef _MLIB_MSU4RND_F32_ASMI_H_
#define _MLIB_MSU4RND_F32_ASMI_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Mul_F64_Asm.h"

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
* @brief  	32-bit inputs 32-output vector multiply and subtract function
* 			with result rounding
*
* @param  	ptr			
* 
* @param  	in    		frac32_t f32MinMult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32MinMult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The output saturation is not implemented in this function. 
* 			The output of the function is defined by the following equation:
* 			f32MinMult1 * f32MinMult2 - f32SubMult1 * f32SubMult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4Rnd_F32_FAsmi(register frac32_t f32MinMult1, register frac32_t f32MinMult2, register frac32_t f32SubMult1, register frac32_t f32SubMult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32MinMult1,f32MinMult2,f32MinMult1);		/* f32MinMult1:Y = f32MinMult1 * f32MinMult2 */
	
	asm(tfr	Y,f32Val2);									/* f32Val2 = Y */
	
	asm(mpy64 f32SubMult1,f32SubMult2,f32SubMult1);		/* f32SubMult1:Y = f32SubMult1 * f32SubMult2 */
	
	asm(sub f32SubMult1,f32MinMult1);					/* f32MinMult1 = f32MinMult1 - f32SubMult1 */
	
	asm(tfr Y,f32Val1);									/* f32Val1 = Y */  
	
	asm(tfr f32Val2,Y);									/* Y = f32Val2 */
	
	asm(sub f32Val1,Y);									/* Y = Y - f32Val1 */

	asm(nop);
	
	asm(lsrr.l #31,Y);									/* Y = highest bit of result lower 32 bits; for rounding */	

	asm(nop);
	
	asm(sbc Y,f32MinMult1);								/* f32MinMult1 = f32MinMult1 - Y */
	
//	asm(tfr f32MinMult1,f32Val2);
	
	asm(.optimize_iasm off);

	return f32MinMult1;	
}

/***************************************************************************//*!
*
* @brief  	32-bit inputs 32-output vector multiply and subtract saturated function
* 			with result rounding
*
* @param  	ptr			
* 
* @param  	in    		frac32_t f32MinMult1
*                         - Argument in [-1;1] in frac32_t
*                       frac32_t f32MinMult2
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult1
*                         - Argument in [-1;1] in frac32_t
*						frac32_t f32SubMult2
*                         - Argument in [-1;1] in frac32_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The input values as well as output value is considered as 32-bit fractional values. 
* 			The function saturates the output. 
* 			The output of the function is defined by the following equation:
* 			f32MinMult1 * f32MinMult2 - f32SubMult1 * f32SubMult2
*
*			SATURATION must be turned off!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4RndSat_F32_FAsmi(register frac32_t f32MinMult1, register frac32_t f32MinMult2, register frac32_t f32SubMult1, register frac32_t f32SubMult2)
{
	register frac32_t f32Val1, f32Val2;
	
	asm(.optimize_iasm on);

	asm(mpy64 f32MinMult1,f32MinMult2,f32MinMult1);		/* f32MinMult1:Y = f32MinMult1 * f32MinMult2 */
	
	asm(tfr	Y,f32Val2);									/* f32Val2 = Y */
	
	asm(mpy64 f32SubMult1,f32SubMult2,f32SubMult1);		/* f32SubMult1:Y = f32SubMult1 * f32SubMult2 */
	
	asm(sub f32SubMult1,f32MinMult1);					/* f32MinMult1 = f32MinMult1 - f32SubMult1 */
	
	asm(tfr Y,f32Val1);									/* f32Val1 = Y */  
	
	asm(tfr f32Val2,Y);									/* Y = f32Val2 */
	
	asm(sub f32Val1,Y);									/* Y = Y - f32Val1 */

	asm(nop);
	
	asm(lsrr.l #31,Y);									/* Y = highest bit of result lower 32 bits; for rounding */	

	asm(nop);
	
	asm(sbc Y,f32MinMult1);								/* f32MinMult1 = f32MinMult1 - Y */
	
	asm(sat f32MinMult1);								/* Saturation */
	
	asm(.optimize_iasm off);

	return f32MinMult1;	
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSU4RND_F32_ASMI_H_ */
