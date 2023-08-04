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
* @brief ACIM flux MTPA algotitm   
* 
*******************************************************************************/
#ifndef _AMCLIB_ACIMFLUXMTPA_F16_H_
#define _AMCLIB_ACIMFLUXMTPA_F16_H_

#if defined(__cplusplus) 
extern "C" {
#endif
  
/****************************************************************************
* Includes
****************************************************************************/   
#include "amclib_types.h" 
#include "gdflib.h"  
#include "gflib.h"     
#include "gmclib.h"     
#include "mlib.h"

/******************************************************************************
* Macros 
******************************************************************************/      
#define AMCLIB_ACIMCtrlMTPAInit_F16_Ci( f16IDMin, f16IDMax, psCtrl)           \
        AMCLIB_ACIMCtrlMTPAInit_F16_FCi(f16IDMin, f16IDMax, psCtrl)                            
#define AMCLIB_ACIMCtrlMTPA_F16_C( f16Iq, psCtrl)                             \
        AMCLIB_ACIMCtrlMTPA_F16_FC(f16Iq, psCtrl)                              
            
/******************************************************************************
* Types 
******************************************************************************/     
typedef struct
{   
	GDFLIB_FILTER_EXP_T_F32 sIdExpParam;   	  /* Exponential filter structure for Id measured */ 
    frac16_t f16LowerLim;                     /* Min value of Id required */
    frac16_t f16UpperLim;                     /* Max value of Id required */ 
} AMCLIB_ACIM_CTRL_MTPA_T_F32;   
   
/****************************************************************************
* Exported function prototypes
****************************************************************************/   
extern frac16_t AMCLIB_ACIMCtrlMTPA_F16_FC(frac16_t f16Iq,
                                          AMCLIB_ACIM_CTRL_MTPA_T_F32 *psCtrl); 
                                        
/****************************************************************************
* Inline functions 
****************************************************************************/                                 

/***************************************************************************//*!
* @brief  The function initializes the actual values of float AMCLIB_ACIMCtrlMTPA_F32 function.
*
* @params:      ptr  AMCLIB_ACIM_CTRL_MTPA_T_F16 *psParam - Pointer to ACIM flux MTPA 
*                                                           function structure                
*
* @return       N/A
* 
*******************************************************************************/
extern inline void AMCLIB_ACIMCtrlMTPAInit_F16_FCi(frac16_t f16IDMin, frac16_t f16IDMax, 
												   AMCLIB_ACIM_CTRL_MTPA_T_F32 *psCtrl)
{
    GDFLIB_FilterExpInit_F16(0, &psCtrl->sIdExpParam);
    psCtrl->f16LowerLim = f16IDMin;
    psCtrl->f16UpperLim = f16IDMax;    
}
   
#if defined(__cplusplus) 
}
#endif

#endif /* _AMCLIB_ACIMFLUXMTPA_F16_H_ */
