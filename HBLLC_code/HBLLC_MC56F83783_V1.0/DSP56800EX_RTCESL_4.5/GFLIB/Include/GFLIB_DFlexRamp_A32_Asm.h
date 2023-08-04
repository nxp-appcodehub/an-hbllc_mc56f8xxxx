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
* @brief  Dynamic flex ramp functions with 16-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _GFLIB_DFLEXRAMP_A32_ASM_H_
#define _GFLIB_DFLEXRAMP_A32_ASM_H_

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
#define GFLIB_DFlexRampInit_F16_Asm(f16InitVal, psParam) \
			GFLIB_DFlexRampInit_F16_FAsm(f16InitVal, psParam)

#define GFLIB_DFlexRampCalcIncr_F16_Asm(f16Target, a32Duration, f32IncrSatMot, f32IncrSatGen, psParam) \
			GFLIB_DFlexRampCalcIncr_F16_FAsm(f16Target, a32Duration, f32IncrSatMot, f32IncrSatGen, psParam)

#define GFLIB_DFlexRamp_F16_Asm(f16Instant, pbStopFlagMot, pbStopFlagGen, psParam) \
			GFLIB_DFlexRamp_F16_FAsm(f16Instant, pbStopFlagMot, pbStopFlagGen, psParam)


/******************************************************************************
* Types
******************************************************************************/
/* Flex dynamic ramp structure */
typedef struct
{
	frac32_t f32State; 
	frac32_t f32Incr; 
	frac32_t f32IncrSatMot;
	frac32_t f32IncrSatGen;
	frac32_t f32Target; 
	frac32_t f32Ts;
	frac32_t f32IncrMax; 
	bool_t bReachFlag; 
} GFLIB_DFLEXRAMP_T_F32;


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/


extern void GFLIB_DFlexRampCalcIncr_F16_FAsm(frac16_t f16Target,
											 acc32_t a32Duration,
											 frac32_t f32IncrSatMot,
											 frac32_t f32IncrSatGen,
									  		 GFLIB_DFLEXRAMP_T_F32 *psParam);

extern asm frac16_t GFLIB_DFlexRamp_F16_FAsm(frac16_t f16Instant,
											 const bool_t *pbStopFlagMot,
											 const bool_t *pbStopFlagGen,
									  		 GFLIB_DFLEXRAMP_T_F32 *psParam);


/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Dynamic flex ramp initialization
*
* @param  in    		frac16_t f16InitVal
*                         - Measured instant value in [-1;1] in frac16_t
*
* @param  ptr			GFLIB_FLEXRAMP_T_F32 *psParam
*						  - f32State: State variable keeping the last state
*						  - f32Incr: Increment
*						  - f32IncrSatMot: Motor mode saturation mode increment
*						  - f32IncrSatGen: Generator mode saturation mode increment 
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
extern inline void GFLIB_DFlexRampInit_F16_FAsm(register frac16_t f16InitVal, register GFLIB_DFLEXRAMP_T_F32 *psParam)
{
	register frac32_t f32State;
	
	asm(move.w f16InitVal,f32State);	/* Extension to 32 bits */
	asm(move.l f32State,X:(psParam));	/* Saves the init value to the structure */

	asm(move.w #0,X:(psParam+14));		/* Clears bReachFlag */
}

#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_DFLEXRAMP_A32_ASM_H_ */
