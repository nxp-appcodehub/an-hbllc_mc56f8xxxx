/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * HBLLC_structure.h
 *
 *  Created on: Nov 12, 2021
 *      Author: nxa16823
 */

#ifndef HBLLC_STRUCTURE_H_
#define HBLLC_STRUCTURE_H_

#include "Cpu.h"
#include "pclib.h"
#include "gflib.h"
#include "gdflib.h"
#include "mlib.h"

/******************************************************************************
* Types
******************************************************************************/
typedef struct 
{
	GFLIB_RAMP_T_F16       sRamp;      
	frac16_t               f16InitVal; /* ramp initial value */
	frac16_t               f16Target;  /* required value (ramp output) */
}LLC_SOFTSTART_RAMP_T;

typedef struct
{
	frac16_t               f16HalfTs;      /* applied 1/2 switching period */
	frac16_t               f16CtrlFreqRatioLim1; /* min switching period when sampling frequency is twice of control frequency */
	frac16_t               f16CtrlFreqRatioLim2; /* min switching period when sampling frequency equals control frequency */
	frac16_t               f16PerHlim; /* switching period up limit */
	frac16_t               f16PerLlim; /* switching period low limit */
}LLC_PERIOD_CTRL_T;

typedef struct
{
	frac16_t               f16Duty;      /* applied duty cycle */
	bool_t                 bDutyunder;   /* flag indicate whether calculated duty is less than the burst off limit, if yes, turn off all switches*/
	bool_t                 bPWMmode;     /* flag indicate current PWM mode, when it's 1, PWM mode, when it's 0, PFM mode */
	bool_t                 bPWMmodeLast;
	frac16_t               f16BurstonDuty;  /* when calculated duty cycle is less than this hysteresis in burst on, switch to burst off */  
	frac16_t               f16BurstoffDuty; /* when calculated duty cycle is greater than this hysteresis in burst off, switch to burst on */  
	frac16_t               f16DutyMax;   /* maximum duty cycle */
}LLC_Duty_Ctrl_T;

typedef struct
{
	GFLIB_CTRL_PI_P_AW_T_A32  sPiParams;/* PI controller parameters */
	bool_t                 bstopFlag;   /* PI controller integration stop flag */ 
	frac16_t               f16outError; /* controlled value error */
	frac16_t               f16out;      /* sampled value */
	frac16_t               f16outFilt;  /* filtered sample value */
	frac16_t               f16outRef;   /* required value */
	frac16_t               f16outMax;   /* MAX applicable value */
	frac16_t               f16PIResult; /* limited PI controller output */
} LLC_PI_T;

typedef struct
{
	LLC_PI_T               sIoutCtrl;      /* output current control parameters */
	frac16_t               f16IoutOverload20per;/* IOUT 20% overload current */
	frac16_t               f16IoutOverload50per;/* IOUT 50% overload current */
	LLC_PI_T               sVoutCtrl;      /* output voltage control parameters */
	PCLIB_CTRL_2P2Z_T_F16  sVoutCtrlParams2P2Z; /* 2p2z controller parameters for vout control in dual loop mode */
	LLC_PI_T               sIprimCtrl;     /* primary current control parameters */
	LLC_SOFTSTART_RAMP_T   sRampDuty;      /* duty cycle ramp parameters at soft-start stage */ 
	LLC_SOFTSTART_RAMP_T   sRampPeriod;    /* switching period ramp parameters at soft-start stage */
	LLC_SOFTSTART_RAMP_T   sRampVout;      /* output voltage ramp parameters at soft-start stage */ 
	frac16_t               f16VoutSoftend; /* soft_start loop to normal run close loop output voltage */
	LLC_PERIOD_CTRL_T      sPeriodCtrl;
	LLC_Duty_Ctrl_T        sDutyCtrl;
	frac16_t               f16LoadoffVol;  /* when VOUT drop to this voltage, disconnect load */
	uint16_t               f16FaultId;     /* Fault identification */ 
	uint16_t               f16FaultIdPending;/* Fault identification pending */
} LLC_STRUC_T;



#endif /* HBLLC_STRUCTURE_H_ */
