/*******************************************************************************
*
 * Copyright 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
* 
*
****************************************************************************/
/*!
*
* @file       PCLIB_CtrlPI_F16_Asm.h
*
* @author    B37570
* 
* @version   2.0.00.0
* 
* @date      November 2014
* 
* @brief     PI controller algorithm implemented in assembler
*
*******************************************************************************
*
* PI controller algorithm implemented in assembler.
*
******************************************************************************/

#ifndef _PCLIB_CTRLPI_F16_ASM_H_
#define _PCLIB_CTRLPI_F16_ASM_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "mlib.h"
#include "pclib_types.h"

/****************************************************************************
* Defines and macros            (scope: module-local)
****************************************************************************/
#define PCLIB_CtrlPIInit_F16_Asmi(psParam) PCLIB_CtrlPIInit_F16_FAsmi(psParam)
#define PCLIB_CtrlPI_F16_Asm(f16InErr, psParam) PCLIB_CtrlPI_F16_FAsm(f16InErr, psParam)
  
/****************************************************************************
* Implementation variant: 16-bit fractional
****************************************************************************/
  
#define PCLIB_CTRL_PI_DEFAULT_F16 {(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0}     /*!< Default value for PCLIB_CTRL_PI_T_F16. */

/************************************************************************/
/*!
@struct PCLIB_CTRL_PI_T_F16 "\PCLIB_CtrlPI_F16.h"

@brief  Structure containing parameters and states of the PI controller.
*//**********************************************************************/

  typedef struct{
	frac16_t f16PreviousIntegralOutput;	/*!< Previous Integral output (n-1) */ 
	frac16_t f16Ki;       		/*!< Integral Gain. */
	frac16_t f16IntegralUpperLimit;   	/*!< Integral Upper limit */
	frac16_t f16IntegralLowerLimit;	/*!< Integral Lower limit */
   	frac16_t f16Kp; 			/* Proportional Gain*/      
	frac16_t f16UpperLimit;  		/*!< Control Loop Output Upper Limit */
	frac16_t f16LowerLimit;		/*!< Control Loop Output Lower Limit */
  }PCLIB_CTRL_PI_T_F16;

/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************/
/*!
@brief        This function initialize the PI control algorithm structure.

@param[in,out]     psParam    Pointer to the structure of PI controller parameters.

@return       Function returns no value.

@details      This function clears the internal buffers of a PI controller. It
              shall be called before the filter parameter initialization and whenever
              the filter initialization is required.

@note         This function shall not be called together with PCLIB_CtrlPI_F16_Asm
              unless periodic clearing of buffers is required.

@par Re-entrancy
              The function is re-entrant.
*		SATURATION INDEPENDENT!
*
****************************************************************************/
extern inline void PCLIB_CtrlPIInit_F16_FAsmi(register PCLIB_CTRL_PI_T_F16 *psParam)
{
	psParam->f16PreviousIntegralOutput = (frac16_t)0;
}

/****************************************************************************
* Exported function prototypes
****************************************************************************/
   extern asm frac16_t PCLIB_CtrlPI_F16_FAsm(frac16_t f16InErr, PCLIB_CTRL_PI_T_F16 *psParam);

#if defined(__cplusplus)
}
#endif

#endif /* _PCLIB_CTRLPI_F16_ASM_H_ */
