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
* @brief  Angle tracking observer
* 
*******************************************************************************/
#ifndef _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_
#define _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib.h"
#include "gflib.h"
#include "gmclib.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define AMCLIB_AngleTrackObsrvInit_F16_Asm(f16ThetaInit, psCtrl) AMCLIB_AngleTrackObsrvInit_F16_FAsm(f16ThetaInit, psCtrl)
#define AMCLIB_AngleTrackObsrv_F16_Asm(psSinCos, psCtrl) AMCLIB_AngleTrackObsrv_F16_FAsm(psSinCos, psCtrl)

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	frac32_t	f32Speed;       	//0
	frac32_t	f32A2;          	//2
    frac16_t  	f16Theta;       	//4
	frac16_t	f16SinEstim;    	//5
	frac16_t	f16CosEstim;    	//6
	frac16_t	f16K1Gain;    		//7
	int16_t		i16K1GainSh;        //8
	frac16_t	f16K2Gain;    		//9
	int16_t		i16K2GainSh;        //10
	frac16_t	f16A2Gain;    		//11
	int16_t		i16A2GainSh;        //12
		
} AMCLIB_ANGLE_TRACK_OBSRV_T_F32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

/***************************************************************************//*!
*
* @brief  Tracks the angle of the system and calculates the speed.
*
* @param  ptr   		GMCLIB_2COORD_SINCOS_T_F16 *psSinCos
*                         - angle's sine and cosine components
*
*                       AMCLIB_ANGLE_TRACK_OBSRV_T_F32 * psCtrl
*                         - parameters of the observer
*
* @param  in    		None
*
* @return This function returns the calculated angle
*		
* @remarks SATURATION MUST BE TURNED OFF! 	
*
****************************************************************************/
extern asm frac16_t AMCLIB_AngleTrackObsrv_F16_FAsm
(
    const GMCLIB_2COOR_SINCOS_T_F16 *psSinCos,
    AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl
);

extern asm void AMCLIB_AngleTrackObsrvInit_F16_FAsm(frac16_t f16ThetaInit, AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl);

/******************************************************************************
* Inline functions
******************************************************************************/

#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_ANGLE_TRACK_OBSRV_F32_ASM_H_ */
