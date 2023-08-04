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
* @brief Main AMCLIB header file
* 
*******************************************************************************/
#ifndef _AMCLIB_H_
#define _AMCLIB_H_

#if defined(__cplusplus) 
extern "C" {
#endif  
  
/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib.h"    
#include "AMCLIB_ACIMCtrlMTPA_F32.h"
#include "AMCLIB_ACIMRotFluxObsrv_A32.h"
#include "AMCLIB_ACIMSpeedMRAS_F32.h"
#include "AMCLIB_AngleTrackObsrv_F32_Asm.h"
#include "AMCLIB_CtrlFluxWkng_A32.h"  
#include "AMCLIB_PMSMBemfObsrvAB_A32_Asm.h"
#include "AMCLIB_PMSMBemfObsrvDQ_A32_Asm.h"
#include "AMCLIB_TrackObsrv_F32_Asm.h"

/*******************************************************************************
* Macros 
*******************************************************************************/      
#define AMCLIB_AngleTrackObsrvInit_F16(f16ThetaInit, psCtrl)                   \
	    AMCLIB_AngleTrackObsrvInit_F16_Asm(f16ThetaInit, psCtrl)
#define AMCLIB_AngleTrackObsrv_F16(psSinCos, psCtrl)                           \
	    AMCLIB_AngleTrackObsrv_F16_Asm(psSinCos, psCtrl)
#define AMCLIB_CtrlFluxWkngInit_F16(f16InitVal, psCtrl)                        \
        AMCLIB_CtrlFluxWkngInit_F16_Ci(f16InitVal, psCtrl)            
#define AMCLIB_CtrlFluxWkng_F16(f16IQErr, f16UQReq, f16UQLim, psCtrl)          \
        AMCLIB_CtrlFluxWkng_F16_C(f16IQErr, f16UQReq, f16UQLim, psCtrl)     
#define AMCLIB_TrackObsrvInit_F16(f16ThetaInit, psCtrl)                        \
        AMCLIB_TrackObsrvInit_F16_Asmi(f16ThetaInit, psCtrl)
#define AMCLIB_TrackObsrv_F16(f16Error, psCtrl)                                \
        AMCLIB_TrackObsrv_F16_Asm(f16Error, psCtrl)
#define AMCLIB_PMSMBemfObsrvABInit_F16(psCtrl)                                 \
        AMCLIB_PMSMBemfObsrvABInit_F16_Asmi(psCtrl)
#define AMCLIB_PMSMBemfObsrvAB_F16(psIAlBe, psUAlBe, f16Speed, psCtrl)         \
        AMCLIB_PMSMBemfObsrvAB_F16_Asm(psIAlBe, psUAlBe, f16Speed, psCtrl)
#define AMCLIB_PMSMBemfObsrvDQInit_F16(psCtrl)                                 \
        AMCLIB_PMSMBemfObsrvDQInit_F16_Asmi(psCtrl)
#define AMCLIB_PMSMBemfObsrvDQ_F16(psIDQ, psUDQ, f16Speed, psCtrl)             \
        AMCLIB_PMSMBemfObsrvDQ_F16_Asm(psIDQ, psUDQ, f16Speed, psCtrl)
#define AMCLIB_CtrlFluxWkng_F16(f16IQErr, f16UQReq, f16UQLim, psCtrl)          \
        AMCLIB_CtrlFluxWkng_F16_C(f16IQErr, f16UQReq, f16UQLim, psCtrl)                        
#define AMCLIB_CtrlFluxWkngInit_F16(f16InitVal, psCtrl)                        \
        AMCLIB_CtrlFluxWkngInit_F16_Ci(f16InitVal, psCtrl)   
#define AMCLIB_ACIMCtrlMTPAInit_F16(f16IDMin, f16IDMax, psCtrl)                \
        AMCLIB_ACIMCtrlMTPAInit_F16_Ci(f16IDMin, f16IDMax, psCtrl)                            
#define AMCLIB_ACIMCtrlMTPA_F16(f16Iq, psCtrl)                                 \
        AMCLIB_ACIMCtrlMTPA_F16_C(f16Iq, psCtrl)  
#define AMCLIB_ACIMRotFluxObsrvInit_F16(psCtrl)                                \
        AMCLIB_ACIMRotFluxObsrvInit_F16_Ci(psCtrl)                              
#define AMCLIB_ACIMRotFluxObsrv_F16(psIsAlBe, sUSAlBe, psCtrl)                 \
        AMCLIB_ACIMRotFluxObsrv_F16_C(psIsAlBe, sUSAlBe, psCtrl)
#define AMCLIB_ACIMSpeedMRASInit_F16(psCtrl)                                   \
        AMCLIB_ACIMSpeedMRASInit_F16_Ci(psCtrl)                                                          
#define AMCLIB_ACIMSpeedMRAS_F16(psIsAlBe, psPsiRAlBe, f16RotPos, psCtrl)      \
        AMCLIB_ACIMSpeedMRAS_F16_C(psIsAlBe, psPsiRAlBe, f16RotPos,  psCtrl) 

#if defined(__cplusplus) 
}
#endif

#endif /* _AMCLIB_H_ */ 
