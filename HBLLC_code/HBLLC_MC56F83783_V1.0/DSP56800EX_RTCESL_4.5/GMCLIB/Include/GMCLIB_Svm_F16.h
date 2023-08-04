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
* @brief Space vector modulation 
* 
*******************************************************************************/
#ifndef _GMCLIB_SVM_F16_H_
#define _GMCLIB_SVM_F16_H_

#if defined(__cplusplus) 
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/  
#include "mlib.h"
#include "gmclib_types.h"
  
/*******************************************************************************
* Macros 
*******************************************************************************/ 
#define GMCLIB_SvmDpwm_F16_C(psIn, psOut)            GMCLIB_SvmDpwm_F16_FC(psIn, psOut) 
#define GMCLIB_SvmExDpwm_F16_C(psIn, psAngle, psOut) GMCLIB_SvmExDpwm_F16_FC(psIn, psAngle,psOut)        
    
/*******************************************************************************
* Exported function prototypes
*******************************************************************************/   
extern uint16_t GMCLIB_SvmDpwm_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn, 
                                      GMCLIB_3COOR_T_F16 *psOut);
extern uint16_t GMCLIB_SvmExDpwm_F16_FC(const GMCLIB_2COOR_ALBE_T_F16 *psIn, 
                                        const GMCLIB_2COOR_SINCOS_T_F16 *psAngle, 
                                        GMCLIB_3COOR_T_F16 *psOut);

#if defined(__cplusplus)  
}
#endif

#endif  /* _GMCLIB_SVM_F16_H_ */
