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
* @brief  Accumulator subtraction from product functions with 32-bit rounded fractional 
* 		  output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MNACRND_F32_ASM_H_
#define _MLIB_MNACRND_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_MnacRnd_F32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_MnacRnd_F32lls_Asmi(f32Accum, f32Mult1, f16Mult2) MLIB_MnacRnd_F32lls_FAsmi(f32Accum, f32Mult1, f16Mult2)
#define MLIB_MnacRndSat_F32lls_Asmi(f32Accum, f32Mult1, f16Mult2) MLIB_MnacRndSat_F32lls_FAsmi(f32Accum, f32Mult1, f16Mult2)

#define MLIB_MnacRnd_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_MnacRnd_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)
#define MLIB_MnacRndSat_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_MnacRndSat_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)

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
#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_MNACRND_F32_ASM_H_ */
