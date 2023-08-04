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
* @brief  Digital IIR Filter, 4th order assembler implementation
* 
*******************************************************************************/
#ifndef _GDFLIB_FILTERIIR4_F32_ASM_H_
#define _GDFLIB_FILTERIIR4_F32_ASM_H_

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
#define GDFLIB_FilterIIR4Init_F16_Asmi(psParam) GDFLIB_FilterIIR4Init_F16_FAsmi(psParam)
#define GDFLIB_FilterIIR4_F16_Asm(f16InX, psParam) GDFLIB_FilterIIR4_F16_FAsm(f16InX, psParam)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t			f32B0;
	frac32_t			f32B1;
	frac32_t			f32B2;
	frac32_t			f32B3;
	frac32_t			f32B4;
	frac32_t			f32A1;
	frac32_t			f32A2;
	frac32_t			f32A3;
	frac32_t			f32A4;
} GDFLIB_FILTER_IIR4_COEFF_T_F32;

typedef struct 
{
    GDFLIB_FILTER_IIR4_COEFF_T_F32	sFltCoeff;
    frac32_t 						f32FltBfrY[4];
    frac16_t 						f16FltBfrX[4];
} GDFLIB_FILTER_IIR4_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern asm frac16_t GDFLIB_FilterIIR4_F16_FAsm(frac16_t f16In, GDFLIB_FILTER_IIR4_T_F32 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function clears the buffer of 4th order IIR filter
*
* @param  ptr			GDFLIB_FILTER_IIR4_T *psParam
* 						- pointer to the structure of the filter
*
*                         GDFLIB_FILTER_IIR4_T_F32 data structure:
*                           GDFLIB_FILTER_IIR4_COEFF_T_F32	sFltCoeff   
*                               - filter coefficients in frac32_t divided by 8
*                                 in the following range
*                                 -1.0 <= f16Gain < 1.0
*                               - B coefficients:
*                               	frac32_t			f32B0
*									frac32_t			f32B1
*									frac32_t			f32B2
*									frac32_t			f32B3
*									frac32_t			f32B4
*								- -A coefficient, the sign is negated
*									frac32_t			f32A1;
*									frac32_t			f32A2;
*									frac32_t			f32A3;
*									frac32_t			f32A4;								  
*                           frac32_t f32FltBfrY[4];
*                               - y history buffer in frac32_t
*							frac16_t f16FltBfrX[4];
*								- x history buffer in frac16_t
*
* @param  in    		
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GDFLIB_FilterIIR4Init_F16_FAsmi(register GDFLIB_FILTER_IIR4_T_F32 *psParam)
{
	register frac32_t f32Temp = 0;
	register frac32_t *pf32Temp = psParam->f32FltBfrY;
	
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.l f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp)+);
	asm(move.w f32Temp,X:(pf32Temp));
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GDFLIB_FILTERIIR4_F32_ASM_H_ */
