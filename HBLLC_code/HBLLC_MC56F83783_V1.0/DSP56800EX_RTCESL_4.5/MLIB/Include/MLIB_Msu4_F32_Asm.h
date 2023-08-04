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
* @brief  Subtraction of two products functions with 32-bit fractional output 
* 		  in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MSU4_F32_ASM_H_
#define _MLIB_MSU4_F32_ASM_H_

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
#define MLIB_Msu4_F32ssss_Asmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2) \
	MLIB_Msu4_F32ssss_FAsmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2)
#define MLIB_Msu4Sat_F32ssss_Asmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2) \
	MLIB_Msu4Sat_F32ssss_FAsmi(f16MinMult1, f16MinMult2, f16SubMult1, f16SubMult2)

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
* @brief  16-bit inputs 32-output vector multiply and subtract function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16MinMult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16MinMult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The output saturation is not implemented in this function. 
* 			
* 			The output of the function is defined by the following equation:
* 			f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2
*
*			SATURATION required if saturation desirable!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4_F32ssss_FAsmi(register frac16_t f16MinMult1, register frac16_t f16MinMult2, register frac16_t f16SubMult1, register frac16_t f16SubMult2)
{
		register frac32_t f32Acc;
		
		asm(.optimize_iasm on);
		
		asm(mpy f16MinMult1,f16MinMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 */
				
		asm(mac -f16SubMult1,f16SubMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2 */
		
		asm(nop);
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

/***************************************************************************//*!
*
* @brief  16-bit inputs 32-output vector multiply and subtract saturated function
*
* @param  ptr			
* 
* @param  in    		frac16_t f16MinMult1
*                         - Argument in [-1;1] in frac16_t
*                       frac16_t f16MinMult2
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult1
*                         - Argument in [-1;1] in frac16_t
*						frac16_t f16SubMult2
*                         - Argument in [-1;1] in frac16_t
*                       
*
* @return This function returns
*     - frac32_t value [-1;1]
*		
* @remarks 	This function returns the vector multiply and subtract of input values. 
* 			The function saturates the output. 
* 			
* 			The output of the function is defined by the following equation:
* 			f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac32_t MLIB_Msu4Sat_F32ssss_FAsmi(register frac16_t f16MinMult1, register frac16_t f16MinMult2, register frac16_t f16SubMult1, register frac16_t f16SubMult2)
{
		register frac32_t f32Acc;
		
		asm(.optimize_iasm on);
		
		asm(mpy f16MinMult1,f16MinMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 */
		
		asm(mac -f16SubMult1,f16SubMult2,f32Acc);	/* f16MinMult1 * f16MinMult2 - f16SubMult1 * f16SubMult2 */
		
		asm(nop);
		
		asm(sat f32Acc);							/* Saturation */
		
		asm(.optimize_iasm off);
		
		return f32Acc;
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MSU4_F32_ASM_H_ */
