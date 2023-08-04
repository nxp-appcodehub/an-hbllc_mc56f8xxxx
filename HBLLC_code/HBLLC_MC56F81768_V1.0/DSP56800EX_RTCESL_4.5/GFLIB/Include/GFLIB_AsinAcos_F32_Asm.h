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
* @brief  Arcus sine and arcus cosine functions 
* 
*******************************************************************************/
#ifndef _GFLIB_ASINACOS_F32_ASM_H_
#define _GFLIB_ASINACOS_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib_types.h"
#include "GFLIB_Sqrt_F32_AsmDef.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t  f32A[5];  /*!< Array of five 32-bit elements for storing coefficients of the piece-wise polynomial. */
} GFLIB_ASIN_COEF_T_F32;

typedef struct
{
	GFLIB_ASIN_COEF_T_F32  sAsinSector[3];
} GFLIB_ASIN_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/*****************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t GFLIB_Asin_F16_FAsm(frac16_t f16Val, const GFLIB_ASIN_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief    The GFLIB_Acos function computes the Acos(pi*x) using    
*		    piece-wise polynomial approximation.
*
* @param  ptr   		*psParam
*                           Pointer to the table
*						*psParam
*							Pointer to the SQRT function table
*
* @param  in    		f16Val
*                           The input data value is in the range of [-1,1), which
*                           corresponds to the angle in the range of [-pi,pi).   
*
* @return The function returns Acos(pi*x).
*     
*		
* @remarks 	The GFLIB_Acos function computes the Acos(pi*x) using     
*		  	Asin(pi*x) piece-wise polynomial approximation. All Asin values
*			falling beyond [-1, 1), are truncated to -1 and 1      
*			respectively.calculates Asin of the input fractional
*			argument                                               
*
*
*           Acos(f16Val) = -1 * (- 0.5 + Asin(f16Val) ) 
*
*			SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline frac16_t GFLIB_Acos_F16_FAsmi(register frac16_t f16Val, register const GFLIB_ASIN_T_F32 *psParam)
{
	register frac16_t f16Out;
	register frac32_t f32Out;
	
	asm(.optimize_iasm on);
	
	f16Out = GFLIB_Asin_F16_FAsm(f16Val,psParam);		/* f16Out = Asin(f16Val) */
	
	asm(move.l #$C0000000,f32Out);						/* f32Out = -0.5 */
	
	asm(add f16Out,f32Out);								/* f16Out = Asin(f16Val) - 0.5 */
	
	asm(neg f32Out);									/* f16Out = 0.5 - Asin(f16Val) */
	
	asm(move.w f32Out,f16Out);							/* f16Val = f32Out (with saturation) */
	
	asm(.optimize_iasm off);
	
	return(f16Out);
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_ASINACOS_F32_ASM_H_ */
