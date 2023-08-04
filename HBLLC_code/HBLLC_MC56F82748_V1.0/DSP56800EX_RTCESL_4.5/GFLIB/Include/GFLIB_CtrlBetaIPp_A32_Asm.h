/*******************************************************************************
*
 * Copyright 2013 - 2017, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
* 
*
****************************************************************************//*!
*
* @brief  Parallel beta IP controller with and without anti-wind-up protection
* 
*******************************************************************************/
#ifndef _GFLIB_CTRLBETAIPP_A32_H_
#define _GFLIB_CTRLBETAIPP_A32_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib_types.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define GFLIB_CtrlBetaIPpAW_F16_Asm(f16InReq, f16In, pbStopIntegFlag, psParam)    \
        GFLIB_CtrlBetaIPpAW_F16_FAsm(f16InReq, f16In, pbStopIntegFlag, psParam)    

#define GFLIB_CtrlBetaIPpAWInit_F16_Asmi(f16InitVal, psParam)                     \
        GFLIB_CtrlBetaIPpAWInit_F16_FAsmi(f16InitVal, psParam)
/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    acc32_t   a32PGain;        /* Proportional Gain */
    acc32_t   a32IGain;        /* Integral Gain */  
    frac32_t  f32IAccK_1;      /* State variable output at step k-1 */
    frac16_t  f16InErrK_1;     /* State variable input error at step k-1 */
    frac16_t  f16UpperLim;     /* Upper Limit of the controller */
    frac16_t  f16LowerLim;     /* Lower Limit of the controller */
    frac16_t  f16BetaGain;     /* Beta Gain for restricting the overshot */     
    bool_t    bLimFlag;        /* Limitation flag, if set to 1, the controller 
                                  output has reached either the UpperLimit or LowerLimit */
}  GFLIB_CTRL_BETA_IP_P_AW_T_A32;




/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
extern frac16_t GFLIB_CtrlBetaIPpAW_F16_FAsm(frac16_t f16InReq, frac16_t f16In,
                                             const bool_t *pbStopIntegFlag, 
                                             GFLIB_CTRL_BETA_IP_P_AW_T_A32 *psParam);


/******************************************************************************
* Inline functions
******************************************************************************/

/***************************************************************************//*!
* @brief    The function initializes the actual values of CtrlBetaIPpAWInit controller.
*
* @params   in   frac16_t f16InitVal - Initial state value for Integrating part
*           ptr  GFLIB_CTRL_BETA_IP_P_AW_T_A32 *psParam - Pointer to controller structure
*
* @return   N/A
* 
*******************************************************************************/

extern inline void GFLIB_CtrlBetaIPpAWInit_F16_FAsmi(register frac16_t f16InitVal, register GFLIB_CTRL_BETA_IP_P_AW_T_A32 *psParam)
{
	register frac32_t f32Integral;
	
	asm(move.w f16InitVal,f32Integral);			/* f32Integral = f16InitVal << 16 */
	asm(move.l f32Integral,x:(psParam + 4));	/* Stores the integral portion */

	psParam->f16InErrK_1 = 0;					/* Resets the previous step error */
}


#if defined(__cplusplus) 
} 
#endif 

#endif /* _GFLIB_CTRLBETAIPP_A32_H_ */
