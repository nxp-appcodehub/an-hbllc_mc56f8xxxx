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
* @brief  Algorithm of back electromotive force observer in rotating reference 
* 		  frame
* 
*******************************************************************************/
#ifndef _AMCLIB_PMSM_BEMF_OBSRV_DQ_A32_ASM_H_
#define _AMCLIB_PMSM_BEMF_OBSRV_DQ_A32_ASM_H_

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
#define AMCLIB_PMSMBemfObsrvDQ_F16_Asm(psIDQ, psUDQ, f16Speed, psCtrl) \
	AMCLIB_PMSMBemfObsrvDQ_F16_FAsm(psIDQ, psUDQ, f16Speed, psCtrl)

#define AMCLIB_PMSMBemfObsrvDQInit_F16_Asmi(psCtrl) \
		AMCLIB_PMSMBemfObsrvDQInit_F16_FAsmi(psCtrl)


/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	//Extended BEMF - d/q
	GMCLIB_2COOR_DQ_T_F32 sEObsrv;			//0
	
	//Accumulators - d/q
	GMCLIB_2COOR_DQ_T_F32 sIObsrv;			//4
	
	//Observer parameters for controllers
	struct
	{
		frac32_t	f32ID_1;//d-accumulator		//8
		frac32_t	f32IQ_1;//q-accumulator		//10
		acc32_t  	a32PGain;					//12
		acc32_t  	a32IGain;					//14
	} sCtrl;

	//Configuration parameters 
	acc32_t		a32IGain; 	//current			//16
	acc32_t		a32UGain; 	//voltage			//18
	acc32_t		a32WIGain;	//decoupling		//20
	acc32_t		a32EGain;	//extended BEMF		//22

	//misalignment error of reference frame
	frac16_t	f16Error;						//24
	
} AMCLIB_BEMF_OBSRV_DQ_T_A32;

/******************************************************************************
* Global variables
******************************************************************************/
   
/******************************************************************************
* Global functions
******************************************************************************/

extern asm frac16_t AMCLIB_PMSMBemfObsrvDQ_F16_FAsm
(
    const GMCLIB_2COOR_DQ_T_F16 *psIDQ,
    const GMCLIB_2COOR_DQ_T_F16 *psUDQ,
    frac16_t f16Speed,
    AMCLIB_BEMF_OBSRV_DQ_T_A32 *psCtrl
);

/******************************************************************************
* Inline functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief  			BEMF observer in DQ initialization
*
* @param  ptr   		AMCLIB_BEMF_OBSRV_DQ_T_A32 *psCtrl
*                         GMCLIB_2COOR_DQ_T_F32 sEObsrv
*                         	- Estimated BEMF in DQ <-1;1)
*                         GMCLIB_2COOR_DQ_T_F32 sIObsrv
*                         	- Estimated current in DQ <-1;1)
*                         frac32_t sCtrl.f32ID_1
*                         	- D-comp. integral part from the PI controller <-1;1)
*                         frac32_t sCtrl.f32IQ_1
*                         	- Q-comp. integral part from the PI controller <-1;1)
*                         acc32_t sCtrl.a32PGain
*                         	- Proportional gain of the PI controller <0;65536.0)
*                         acc32_t sCtrl.a32IGain
*                         	- Integral gain of the PI controller <0;65536.0)
*					 	  acc32_t a32IGain
*					 	  	- Current coefficient <0;65536.0)
*					 	  acc32_t a32UGain
*					 	  	- Voltage coefficient <0;65536.0)
*					 	  acc32_t a32WIGain
*					 	  	- Decoupling coefficient <0;65536.0)
*					 	  acc32_t a32EGain
*					 	  	- Extended BEMF coefficient <0;65536.0)
*                         frac16_t f16Error
*                         	- Calculated Integ. constant to get speed from error <0;1)
*
* @return 			None
*                         
* @remarks	Initializes the structure of the BEMF observer
* 			
* 			sEObsrv.f32D = 0
* 			sEObsrv.f32Q = 0
*
* 			sIObsrv.f32D = 0
* 			sIObsrv.f32Q = 0 			
*
*			sCtrl.f32ID_1 = 0
*			sCtrl.f32IQ_1 = 0
*			
*			f16Error = 0
*			 			
* 			
*	 			THE SATURATION MUST BE TURNED OFF!
*
****************************************************************************/
extern inline void AMCLIB_PMSMBemfObsrvDQInit_F16_FAsmi(register AMCLIB_BEMF_OBSRV_DQ_T_A32 *psCtrl)
{
	register frac32_t f32Init;
	
	asm(move.l #0,f32Init);
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	asm(move.l f32Init,x:(psCtrl)+);			/* Stores the initial value */
	
	asm(move.w f32Init,x:(psCtrl+12));			/* Stores the initial value */

}



#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_PMSM_BEMF_OBSRV_DQ_A32_ASM_H_ */	
