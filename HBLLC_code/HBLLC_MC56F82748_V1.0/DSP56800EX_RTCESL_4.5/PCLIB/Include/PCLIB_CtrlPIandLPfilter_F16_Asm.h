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
* @file       PCLIB_CtrlPIandLPFilter_F16_Asm.h
*
* @author    B37570
* 
* @version   2.0.0.0
* 
* @date      November 2014
* 
* @brief     PI AND Low Pass FILTER Control algorithm implemented in assembler
*
*******************************************************************************
*
* PI AND Low Pass FILTER algorithm implemented in assembler.
*
******************************************************************************/

#ifndef _PCLIB_CTRLPILP_F16_ASM_H_
#define _PCLIB_CTRLPILP_F16_ASM_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib.h"
#include "pclib_types.h"

/****************************************************************************
* Defines and macros            (scope: module-local)
****************************************************************************/
#define PCLIB_CtrlPIandLPInit_F16_Asmi(psParam) PCLIB_CtrlPIandLPInit_F16_FAsmi(psParam)
#define PCLIB_CtrlPIandLP_F16_Asm(f16InErr, psParam) PCLIB_CtrlPIandLP_F16_FAsm(f16InErr, psParam)

/****************************************************************************
* Implementation variant: 16-bit fractional
****************************************************************************/
  
    #define PCLIB_CTRL_PI_LP_DEFAULT_F16 {(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0,(frac16_t)0}     /*!< Default value for PCLIB_CTRL_PI_LP_T_F16. */

  /************************************************************************/
  /*!
  @struct PCLIB_CTRL_PI_LP_T_F16 "\PCLIB_CtrlPIandLPFilter_F16.h"

  @brief  Structure containing parameters and states of the PI with low pass filter controller.
  *//**********************************************************************/

  typedef struct{
    frac16_t f16CoeffB0;         /*!< Controller coefficient for present error. */
    frac16_t f16CoeffB1;         /*!< Controller coefficient for past error. */
    frac16_t f16CoeffB2;         /*!< Controller coefficient for past to past error. */
    frac16_t f16CoeffA1;         /*!< Controller coefficient for past result. */
    frac16_t f16CoeffA2;         /*!< Controller coefficient for past to past result. */
    frac16_t f16DelayX1;         /*!< Controller delay parameter, past error. */
    frac16_t f16DelayX2;         /*!< Controller delay parameter, past to past error. */
    frac16_t f16DelayY1;         /*!< Controller delay parameter, past result. */
    frac16_t f16DelayY2;         /*!< Controller delay parameter, past to past result. */
  }PCLIB_CTRL_PI_LP_T_F16;


/******************************************************************************
* Global functions
******************************************************************************/

/*!
@brief        This function initialize the PI control algorithm structure.

@param[in,out]     psParam    Pointer to the structure of PI controller parameters.

@return       Function returns no value.

@details      This function clears the internal buffers of a PI controller. It
              shall be called before the filter parameter initialization and whenever
              the filter initialization is required.

@note         This function shall not be called together with PCLIB_CtrlPI_F16
              unless periodic clearing of buffers is required.

@par Re-entrancy
              The function is re-entrant.
****************************************************************************/
extern inline void PCLIB_CtrlPIandLPInit_F16_FAsmi(register PCLIB_CTRL_PI_LP_T_F16 *psParam)
{
    psParam->f16DelayX1 = (frac16_t)0;
    psParam->f16DelayY1 = (frac16_t)0;
    psParam->f16DelayX2 = (frac16_t)0;
    psParam->f16DelayY2 = (frac16_t)0;
}
/****************************************************************************
* Exported function prototypes
****************************************************************************/

  extern asm frac16_t PCLIB_CtrlPIandLP_F16_FAsm(frac16_t f16InErr, PCLIB_CTRL_PI_LP_T_F16 *psParam);


#if defined(__cplusplus)
}
#endif

#endif /* _PCLIB_CTRLPILP_F16_ASM_H_ */
