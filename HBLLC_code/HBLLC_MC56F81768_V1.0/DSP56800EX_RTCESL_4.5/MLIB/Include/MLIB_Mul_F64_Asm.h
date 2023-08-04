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
* @brief  Multiply functions with 64-bit fractional output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F64_ASM_H_
#define _MLIB_MUL_F64_ASM_H_

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
#define MLIB_Mul_F64ll_Asm(f32Mult1, f32Mult2) MLIB_Mul_F64ll_FAsm(f32Mult1, f32Mult2)
#define MLIB_MulSat_F64ll_Asm(f32Mult1, f32Mult2) MLIB_MulSat_F64ll_FAsm(f32Mult1, f32Mult2)
#define MLIB_MulNeg_F64ll_Asm(f32Mult1, f32Mult2) MLIB_MulNeg_F64ll_FAsm(f32Mult1, f32Mult2)

#define MLIB_Mul_F64_Asm(f64Mult1, f64Mult2) MLIB_Mul_F64_FAsm(f64Mult1, f64Mult2)
#define MLIB_MulSat_F64_Asm(f64Mult1, f64Mult2) MLIB_MulSat_F64_FAsm(f64Mult1, f64Mult2)
#define MLIB_MulNeg_F64_Asm(f64Mult1, f64Mult2) MLIB_MulNeg_F64_FAsm(f64Mult1, f64Mult2)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern frac64_t MLIB_Mul_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);
extern frac64_t MLIB_MulSat_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);
extern frac64_t MLIB_MulNeg_F64ll_FAsm(frac32_t f32Mult1, frac32_t f32Mult2);

extern frac64_t MLIB_Mul_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);
extern frac64_t MLIB_MulSat_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);
extern frac64_t MLIB_MulNeg_F64_FAsm(frac64_t f64Mult1, frac64_t f64Mult2);

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _MLIB_MUL_F64_ASM_H_ */
