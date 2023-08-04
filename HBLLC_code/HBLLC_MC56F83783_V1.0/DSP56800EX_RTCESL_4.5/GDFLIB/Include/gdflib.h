/*******************************************************************************
*
 * Copyright 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
* 
****************************************************************************//*!
*
* @brief  Main GDFLIB header file
* 
*******************************************************************************/
#ifndef _GDFLIB_H_
#define _GDFLIB_H_

#if defined(__cplusplus)
extern "C" {
#endif
  
/*******************************************************************************
* Includes
*******************************************************************************/
#include "GDFLIB_FilterExp_F32.h"
#include "GDFLIB_FilterIIR1_F32_Asm.h"
#include "GDFLIB_FilterIIR2_F32_Asm.h"
#include "GDFLIB_FilterIIR3_F32_Asm.h"
#include "GDFLIB_FilterIIR4_F32_Asm.h"
#include "GDFLIB_FilterMA_A32_Asm.h"
   
/*******************************************************************************
* Macros 
*******************************************************************************/
#define GDFLIB_FilterExpInit_F16(f16InX, psParam)                              \
        GDFLIB_FilterExpInit_F16_Ci(f16InX, psParam)             
#define GDFLIB_FilterExp_F16(f16InX, psParam)                                  \
        GDFLIB_FilterExp_F16_Ci(f16InX, psParam)    

#define GDFLIB_FilterIIR1Init_F16(psParam)                                     \
        GDFLIB_FilterIIR1Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR1_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR1_F16_Asmi(f16InX, psParam) 

#define GDFLIB_FilterIIR2Init_F16(psParam)                                     \
        GDFLIB_FilterIIR2Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR2_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR2_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterIIR3Init_F16(psParam)                                     \
        GDFLIB_FilterIIR3Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR3_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR3_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterIIR4Init_F16(psParam)                                     \
        GDFLIB_FilterIIR4Init_F16_Asmi(psParam)
#define GDFLIB_FilterIIR4_F16(f16InX, psParam)                                 \
        GDFLIB_FilterIIR4_F16_Asm(f16InX, psParam) 

#define GDFLIB_FilterMAInit_F16(f16InitVal, psParam)                           \
        GDFLIB_FilterMAInit_F16_Asmi(f16InitVal, psParam)
#define GDFLIB_FilterMA_F16(f16InX, psParam)                                   \
        GDFLIB_FilterMA_F16_Asmi(f16InX, psParam)

#if defined(__cplusplus)
}
#endif

#endif /* _GDFLIB_H_ */

