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
* @brief  Flex ramp functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_FLEXRAMP_A32_ASM_H_
#define _GFLIB_FLEXRAMP_A32_ASM_H_

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
#define GFLIB_FlexRampInit_F16_Asm(f16InitVal, psParam) \
			GFLIB_FlexRampInit_F16_FAsm(f16InitVal, psParam)

#define GFLIB_FlexRampCalcIncr_F16_Asm(f16Target, a32Duration, psParam) \
			GFLIB_FlexRampCalcIncr_F16_FAsm(f16Target, a32Duration, psParam)

#define GFLIB_FlexRamp_F16_Asm(psParam) \
			GFLIB_FlexRamp_F16_FAsm(psParam)


/******************************************************************************
* Types
******************************************************************************/
/* Flex dynamic ramp structure */
typedef struct
{
	frac32_t f32State; 
	frac32_t f32Incr; 
	frac32_t f32Target; 
	frac32_t f32Ts;
	frac32_t f32IncrMax; 
	bool_t bReachFlag; 
} GFLIB_FLEXRAMP_T_F32;


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/


extern asm void GFLIB_FlexRampCalcIncr_F16_FAsm(frac16_t f16Target,
											acc32_t a32Duration,
									  		GFLIB_FLEXRAMP_T_F32 *psParam);

extern asm frac16_t GFLIB_FlexRamp_F16_FAsm(GFLIB_FLEXRAMP_T_F32 *psParam);


/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Flex ramp initialization
*
* @param  in    		frac16_t f16InitVal
*                         - Measured instant value in [-1;1] in frac16_t
*
* @param  ptr			GFLIB_FLEXRAMP_T_F32 *psParam
*						  - f32State: State variable keeping the last state
*						  - f32Incr: Increment
*						  - f32Target: Target value
*						  - f32Ts: Sample time [s]
*						  - f32IncrMax: Maximum increment
*						  - bReachFlag: Flag is set to TRUE if the desired value is achieved
*
* @return void
*		
* @remarks 	The init value is stored into the f32State variable and the
* 			bReachFlag flag is cleared.
*
*			SATURATION MODE INDEPENDENT!
*
****************************************************************************/
extern inline void GFLIB_FlexRampInit_F16_FAsm(register frac16_t f16InitVal, register GFLIB_FLEXRAMP_T_F32 *psParam)
{
	register frac32_t f32State;
	
	asm(move.w f16InitVal,f32State);	/* Extension to 32 bits */
	asm(move.l f32State,X:(psParam));	/* Saves the init value to the structure */

	asm(move.w #0,X:(psParam+10));		/* Clears bReachFlag */
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_FLEXRAMP_A32_ASM_H_ */
