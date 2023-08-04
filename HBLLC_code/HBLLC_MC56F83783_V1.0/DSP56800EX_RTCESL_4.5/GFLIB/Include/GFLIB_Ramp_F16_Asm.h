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
* @brief  Ramp functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_RAMP_F16_ASM_H_
#define _GFLIB_RAMP_F16_ASM_H_

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
#define GFLIB_RampInit_F16_Asm(f16InitVal, psParam) GFLIB_RampInit_F16_FAsm(f16InitVal, psParam)

#define GFLIB_Ramp_F16_Asm(f16Target, psParam) GFLIB_Ramp_F16_FAsm(f16Target, psParam)

/******************************************************************************
* Types
******************************************************************************/
/* Ramp structure */
typedef struct
{
    frac16_t f16RampUp;
    frac16_t f16RampDown;
    frac16_t f16State;
} GFLIB_RAMP_T_F16;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t GFLIB_Ramp_F16_FAsm(frac16_t f16Target, GFLIB_RAMP_T_F16 *psParam);

/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  The function initializes the actual value of Ramp_F16.
*
* @param  ptr   		GFLIB_RAMP_T_F16 *psParam
*						  - f16RampUp: Ramp-up increment
*						  - f16RampDown: Ramp-down increment
*						  - f16State: Previous ramp value
*
* @param  in    		frac16_t f16InitVal
* 							- Initial value
*
* @return N/A
*		
* @remarks 
*
****************************************************************************/
extern inline void GFLIB_RampInit_F16_FAsm(register frac16_t f16InitVal, register GFLIB_RAMP_T_F16 *psParam)
{
	asm(move.w f16InitVal,X:(psParam+2));
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_RAMP_F16_ASM_H_ */
