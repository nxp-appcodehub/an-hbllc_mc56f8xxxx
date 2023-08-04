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
* @brief  Decoupling calculation algorithm
* 
*******************************************************************************/
#ifndef _GMCLIB_DECOUPLING_A32_ASM_H_
#define _GMCLIB_DECOUPLING_A32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gmclib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GMCLIB_DecouplingPMSM_F16_Asm(psUDQ, psIDQ, f16SpeedEl, psParam, psUDQDec) \
			GMCLIB_DecouplingPMSM_F16_FAsm(psUDQ, psIDQ, f16SpeedEl, psParam, psUDQDec)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    acc32_t  a32KdGain;
    acc32_t  a32KqGain;
} GMCLIB_DECOUPLINGPMSM_T_A32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

extern void GMCLIB_DecouplingPMSM_F16_FAsm(const GMCLIB_2COOR_DQ_T_F16 *psUDQ, \
							 			   const GMCLIB_2COOR_DQ_T_F16 *psIDQ, \
							 			   frac16_t f16SpeedEl, \
							 			   const GMCLIB_DECOUPLINGPMSM_T_A32 *psParam, \
							 			   GMCLIB_2COOR_DQ_T_F16 *psUDQDec); 

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* _GMCLIB_DECOUPLING_A32_ASM_H_ */

