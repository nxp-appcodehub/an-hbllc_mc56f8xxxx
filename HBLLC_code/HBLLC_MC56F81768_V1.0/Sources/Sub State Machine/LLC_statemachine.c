/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * LLC_statemachine.c
 *
 *  Created on: Aug 23, 2019
 *      Author: nxa16823
 */

#include "LLC_statemachine.h"


/******************************************************************************
* Global variables
******************************************************************************/
LLC_RUN_SUBSTATE_T   eLLC_Runsub;
LLC_STRUC_T          gsLLC_Drive;
uint16_t    u16PIT_Count;  /* comparison basis for time interval measurement */
bool_t      bLLC_Run = 1; /* LLC run/stop command, when it's 1, LLC will start to work automatically, when it's 0,
                          LLC will wait for an external start command. When connecting the front-level PFC circuit,
                          it should be 0 and wait the DC bus voltage ready. */ 

/******************************************************************************
* Local variables
******************************************************************************/
frac16_t   f16DutyStart[8], f16VolStart[7]; /* different initial duty cycle corresponding to different output capacitor voltage */
bool_t     bLoadon; /* flag that indicate whether the load is connected */
uint16_t   u16IoutOver20Count, u16IoutOver50Count,u16VoutUnderCount; /* time interval measurement count for iout over and vout under */
uint16_t   LLC_CtrlModeCmd = DUAL_OUTER_LOOP, LLC_CtrllModeUsed; 
uint16_t   u16NoFaultCount; /* time interval measurement count to determine when restart from fault state*/

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void LLC_StateFault(void);
static void LLC_StateInit(void);
static void LLC_StateStop(void);
static void LLC_StateRun(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void LLC_TransFaultInit(void);
static void LLC_TransInitFault(void);
static void LLC_TransInitStop(void);
static void LLC_TransStopFault(void);
static void LLC_TransStopRun(void);
static void LLC_TransRunFault(void);
static void LLC_TransRunStop(void);

/* State machine functions field (in pmem) */
__pmem static const SM_APP_STATE_FCN_T msSTATE = {LLC_StateFault, LLC_StateInit, LLC_StateStop, LLC_StateRun};


/* State-transition functions field (in pmem) */
__pmem static const SM_APP_TRANS_FCN_T msTRANS = {LLC_TransFaultInit, LLC_TransInitFault, LLC_TransInitStop, LLC_TransStopFault, LLC_TransStopRun, LLC_TransRunFault, LLC_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsLLC_Ctrl = 
{
	/* gsM1_Ctrl.psState, User state functions  */
	&msSTATE,
 	
 	/* gsM1_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsM1_Ctrl.uiCtrl, Default no control command */
  	SM_CTRL_NONE,
  	
  	/* gsM1_Ctrl.eState, Default state after reset */
  	INIT 	
};

/*------------------------------------
 * User run sub-state machine functions
 * ----------------------------------*/
static void LLC_StateRun_Softstart(void);
static void LLC_StateRun_Normal(void);
static void LLC_StateRun_Lightload(void);

/*------------------------------------
 * User run sub-state-transition functions
 * ----------------------------------*/
static void LLC_TransRun_Softstart_Normal(void);
static void LLC_TransRun_Normal_Lightload(void);
static void LLC_TransRun_Lightload_Normal(void);

/* Sub-state machine functions field (in pmem)*/ 
__pmem const PFCN_VOID_VOID mLLC_STATE_RUN_TABLE[3] = 
{
	LLC_StateRun_Softstart, LLC_StateRun_Normal, LLC_StateRun_Lightload	
};

void FaultDetection(void);
bool_t TimeDelay(uint16_t count, uint16_t Delaytime);

inline void LLC_PWM_UPDATE(frac16_t f16HalfTs,frac16_t f16Dutycycle)
{
	uint16_t u16Deadtime;
	uint32_t u32Deadtime;
	uint16_t u16HalfTs,u16QuaterTs,u16FracVal;
	frac16_t f16OffDuty;
	
	u16HalfTs = MLIB_Mul_F16(f16HalfTs, CTRLOUT_TO_PEIOD_GAIN); /* controller output to PWM VAL transfer */
	u16QuaterTs = u16HalfTs>>1;
	f16OffDuty = FRAC16(0.5) - f16Dutycycle;
	
/* PWM mode is implemented by changing the dead time. In PFM mode, the min dead time is used.*/	
#if PWM_NORMAL == 1
	u16Deadtime = MLIB_Mul_F16((u16HalfTs<<1),f16OffDuty);
	if(u16Deadtime <= DT_LIMIT)  u16Deadtime = DT_LIMIT;
	
    PWMA->SM[2].VAL1 = (u16HalfTs<<1)-1;
	PWMA->SM[2].VAL2 = u16QuaterTs+u16Deadtime;
	PWMA->SM[2].VAL3 = u16HalfTs+u16QuaterTs;
	if(u16Deadtime > u16QuaterTs) PWMA->SM[2].VAL4 = u16Deadtime-u16QuaterTs;
	else PWMA->SM[2].VAL4 =  u16HalfTs+u16QuaterTs+u16Deadtime;
	PWMA->SM[2].VAL5 =  u16QuaterTs;
#endif	
	
#if PWM_HIGH_RES ==1 
	u32Deadtime = MLIB_MulRnd_F32ls((u16HalfTs<<6), f16OffDuty);
	u16Deadtime = (uint16_t)(u32Deadtime>>5);
	if(u16Deadtime <= DT_LIMIT)  u16Deadtime = DT_LIMIT;
	
	u16FracVal = (int16_t)u32Deadtime & 0x1F;
	
	PWM->SM[2].VAL1 = (u16HalfTs<<1)-1;
	PWM->SM[2].VAL2 = u16QuaterTs+u16Deadtime;
	PWM->SM[2].FRACVAL2 = (u16FracVal<<11);
	PWM->SM[2].VAL3 = u16HalfTs+u16QuaterTs;
	if(u16Deadtime > u16QuaterTs) PWM->SM[2].VAL4 = u16Deadtime-u16QuaterTs;
	else  PWM->SM[2].VAL4 = u16HalfTs+u16QuaterTs+u16Deadtime;
	PWM->SM[2].FRACVAL4 = (u16FracVal<<11);
	PWM->SM[2].VAL5 = u16QuaterTs;
#endif
	
/* SR driver = PWM_SM1_OUTPUT & hardware signal
 * PWM_SM1_OUTPUT configuration is: turn on at the same time with main power tube, and the turn off delay a predefined time */ 
#if SR_ENABLE
	PWM->SM[1].VAL1 = (u16HalfTs<<1)-1;
	PWM->SM[1].VAL2 = u16QuaterTs+u16Deadtime;
	PWM->SM[1].VAL3 = u16HalfTs+u16QuaterTs+SR_OFFDELAY;
	if(u16Deadtime > u16QuaterTs) PWM->SM[1].VAL4 = u16Deadtime-u16QuaterTs;
	else   PWM->SM[1].VAL4 = u16HalfTs+u16QuaterTs+u16Deadtime;
	PWM->SM[1].VAL5 = u16QuaterTs+SR_OFFDELAY;
#endif

#if CTRL_FRE_CHANGE_ENABLE	
	if(f16HalfTs <= (gsLLC_Drive.sPeriodCtrl.f16CtrlFreqRatioLim1))
	{
		PWM->SM[0].VAL1 = (u16HalfTs*6)-1; // the switching frequency is three times the control time
	}
	else if(f16HalfTs <= gsLLC_Drive.sPeriodCtrl.f16CtrlFreqRatioLim2)
	{
	    PWM->SM[0].VAL1 = (u16HalfTs<<2)-1; // the switching frequency is twice the control frequency
	}
	else
	{
	    PWM->SM[0].VAL1 = (u16HalfTs<<1)-1; // the switching frequency equals the control frequency
	}
#endif 
	/* load new values to PWM registers */
	PWM->MCTRL |= PWM_MCTRL_LDOK(1); /* LDOK=1,load enable */ 
}

void FaultDetection(void)
{
	FAULT_ALL_CLEAR(gsLLC_Drive.f16FaultIdPending);
	
	/* Fault: output voltage over/under limit */
	if(gsLLC_Drive.sVoutCtrl.f16outFilt >= gsLLC_Drive.sVoutCtrl.f16outMax)
	{
		FAULT_SET(gsLLC_Drive.f16FaultIdPending, VOUT_OVER);
	}
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt <= FRAC16(VOUT_LOW_LIMIT/VOUT_SCALE) && bLoadon == 1)
	{
		if(TimeDelay(u16VoutUnderCount,T_5MS))
		{
			LOAD_DISABLE;
			bLoadon = 0;
			FAULT_SET(gsLLC_Drive.f16FaultIdPending, VOUT_LOW);
		}
	}
	else
	{
		u16VoutUnderCount = u16PIT_Count;
	}
		
	/*Fault: output current over limit */
	if(gsLLC_Drive.sIoutCtrl.f16outFilt >= gsLLC_Drive.sIoutCtrl.f16outMax)
	{
		FAULT_SET(gsLLC_Drive.f16FaultIdPending, IOUT_OVER);
	}
	else if (gsLLC_Drive.sIoutCtrl.f16outFilt >= gsLLC_Drive.f16IoutOverload50per)
	{
		if(TimeDelay(u16IoutOver50Count,T_5MS))   FAULT_SET(gsLLC_Drive.f16FaultIdPending, IOUT_OVER50);
	}
	else if (gsLLC_Drive.sIoutCtrl.f16outFilt >= gsLLC_Drive.f16IoutOverload20per)
	{
		if(TimeDelay(u16IoutOver20Count,T_20MS)) FAULT_SET(gsLLC_Drive.f16FaultIdPending, IOUT_OVER20);
	}
	else
	{
		u16IoutOver50Count = u16PIT_Count;
		u16IoutOver20Count = u16PIT_Count;
	}
		
	/*Fault: primary side current over limit */
	if(gsLLC_Drive.sIprimCtrl.f16outFilt >= gsLLC_Drive.sIprimCtrl.f16outMax)
	{
		FAULT_SET(gsLLC_Drive.f16FaultIdPending, IPRIM_OVER);
	}
	
	/*Fault: primary side current hardware protection.
	 * hardware fault flag is not cleared here, so the system won't restart after a HW fault */
	if(HW_IPRIM_OVER()) 
	{
		FAULT_SET(gsLLC_Drive.f16FaultIdPending, HW_IPRIMOVER);
		LLC_BOARD_LED3_ON; //LLC board Red LED(D3) on, indicate the HW protection
	}
		
	if(gsLLC_Drive.f16FaultIdPending > 0)
	{
		gsLLC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
		u16NoFaultCount = u16PIT_Count;
		LLC_BOARD_LED4_ON; //LLC board green LED(D4) on, indicate there is a fault
	}
		
	gsLLC_Drive.f16FaultId |= gsLLC_Drive.f16FaultIdPending;
}

bool_t TimeDelay(uint16_t count, uint16_t Delaytime)
{
	uint16_t delta_t;
	if(count > u16PIT_Count)
	{
		delta_t = u16PIT_Count + 65536 -count;
	}
	else
	{
		delta_t = u16PIT_Count - count;
	}
	if(delta_t >= Delaytime)   return true;
	else                       return false;
}


/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_StateFault(void)
{
	DISABLE_PWM_OUTPUT;
	DISABLE_SR;
	FaultDetection();
	
	if(gsLLC_Drive.sVoutCtrl.f16outFilt < gsLLC_Drive.f16LoadoffVol)
	{
		LOAD_DISABLE;
		bLoadon = 0;
	}
	if(TimeDelay(u16NoFaultCount,T_5S))   
	{
		PIT_STOP;
		LLC_BOARD_LED4_OFF; // no fault
		gsLLC_Ctrl.uiCtrl = SM_CTRL_FAULT_CLEAR; // clear fault command, system restart
	}
}

/***************************************************************************//*!
*
* @brief   INIT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_StateInit(void)
{
	FAULT_ALL_CLEAR(gsLLC_Drive.f16FaultId); //clear all fault flag 
	FAULT_ALL_CLEAR(gsLLC_Drive.f16FaultIdPending); //clear all fault flag 
	HW_FAULT_FLAG_CLEAR(); //clear hardware protection flag

	u16NoFaultCount = u16PIT_Count;
	u16IoutOver20Count = u16PIT_Count;
	u16IoutOver50Count = u16PIT_Count;
	u16VoutUnderCount = u16PIT_Count;

	bLoadon = 0;
	gsLLC_Drive.f16LoadoffVol = FRAC16(LOAD_OFF_VOUT/VOUT_SCALE);

	/*------------------------- fault thresholds --------------------------*/
	gsLLC_Drive.sVoutCtrl.f16outMax = FRAC16(VOUT_UP_LIMIT/VOUT_SCALE);
	gsLLC_Drive.sIoutCtrl.f16outMax = FRAC16(IOUT_LIMIT/IOUT_SCALE);
	gsLLC_Drive.f16IoutOverload20per = FRAC16(IOUT_OVER_20_PERCENT/IOUT_SCALE);
	gsLLC_Drive.f16IoutOverload50per = FRAC16(IOUT_OVER_50_PERCENT/IOUT_SCALE);
	gsLLC_Drive.sIprimCtrl.f16outMax = FRAC16(IPRIM_LIMIT/IPRIM_SCALE);
	
	/*------------------------- period and duty limit --------------------------*/
	gsLLC_Drive.sPeriodCtrl.f16CtrlFreqRatioLim1 = FRAC16(100/(2*FREQ_LIM1));
	gsLLC_Drive.sPeriodCtrl.f16CtrlFreqRatioLim2 = FRAC16(100/(2*FREQ_LIM2));
	gsLLC_Drive.sPeriodCtrl.f16PerLlim = FRAC16(100/(2*FREQ_MAX_SOFTSTART));
	gsLLC_Drive.sPeriodCtrl.f16PerHlim = FRAC16(100/(2*FREQ_MIN));
		
	gsLLC_Drive.sDutyCtrl.f16BurstoffDuty = FRAC16(DUTY_CYCLE_MIN);
	gsLLC_Drive.sDutyCtrl.f16BurstonDuty = FRAC16(BURST_ON_DUTY_CYCLE_HYS);
	gsLLC_Drive.sDutyCtrl.f16DutyMax = FRAC16(DUTY_CYCLE_MAX);
	
	gsLLC_Drive.sDutyCtrl.bDutyunder = false;
	gsLLC_Drive.sDutyCtrl.bPWMmode = false;
	gsLLC_Drive.sDutyCtrl.bPWMmodeLast = false;
		
	/******************************** Soft start params ****************************/
    /* different output capacitor voltage with different start duty cycle to avoid large current surge */ 	
	f16DutyStart[0] = FRAC16(0.3);
	f16DutyStart[1] = FRAC16(0.31);
	f16DutyStart[2] = FRAC16(0.32);
	f16DutyStart[3] = FRAC16(0.33);
	f16DutyStart[4] = FRAC16(0.34);
	f16DutyStart[5] = FRAC16(0.35);
	f16DutyStart[6] = FRAC16(0.4);
	f16DutyStart[7] = FRAC16(0.5);
	
	f16VolStart[0] = FRAC16(2.5/VOUT_SCALE);
	f16VolStart[1] = FRAC16(3/VOUT_SCALE);
	f16VolStart[2] = FRAC16(3.5/VOUT_SCALE);
	f16VolStart[3] = FRAC16(4/VOUT_SCALE);
	f16VolStart[4] = FRAC16(4.5/VOUT_SCALE);
	f16VolStart[5] = FRAC16(6/VOUT_SCALE);
	f16VolStart[6] = FRAC16(8/VOUT_SCALE);
		
	gsLLC_Drive.sRampDuty.sRamp.f16RampUp = FRAC16(0.0008);
	gsLLC_Drive.sRampDuty.sRamp.f16RampDown = FRAC16(0.0008);
	gsLLC_Drive.sRampDuty.f16Target = gsLLC_Drive.sDutyCtrl.f16DutyMax;
		
	//gsLLC_Drive.sRampPeriod.f16InitVal = FRAC16(100/(2*FREQ_START));
	gsLLC_Drive.sRampPeriod.sRamp.f16RampUp = FRAC16(0.0005);
	gsLLC_Drive.sRampPeriod.sRamp.f16RampDown = FRAC16(0.0005);
	//GFLIB_RampInit_F16(gsLLC_Drive.sRampPeriod.f16InitVal, &gsLLC_Drive.sRampPeriod.sRamp);
		
	/******************************** VOUT parameters *****************************/
	gsLLC_Drive.f16VoutSoftend = FRAC16(VOUT_SOFT_END/VOUT_SCALE); /* soft start end output voltage */
	gsLLC_Drive.sVoutCtrl.f16outRef = gsLLC_Drive.f16VoutSoftend; /* output reference ramp up from soft start end voltage */
	/* VOUT reference ramp params */
	gsLLC_Drive.sRampVout.f16Target = FRAC16(VOUT_REF/VOUT_SCALE);
	gsLLC_Drive.sRampVout.sRamp.f16RampUp = FRAC16(0.0004);
	
	/******************************* IOUT parameters *******************************/
	gsLLC_Drive.sIoutCtrl.f16outRef = FRAC16(IOUT_REF/IOUT_SCALE);
	/* PI controller parameters */
	gsLLC_Drive.sIoutCtrl.sPiParams.a32PGain = ACC32(IOUT_KP);
	gsLLC_Drive.sIoutCtrl.sPiParams.a32IGain = ACC32(IOUT_KI);
	gsLLC_Drive.sIoutCtrl.sPiParams.f16LowerLim = FRAC16(IOUT_LOW_LIMIT);
	gsLLC_Drive.sIoutCtrl.sPiParams.f16UpperLim = FRAC16(IOUT_UP_LIMIT);
	gsLLC_Drive.sIoutCtrl.bstopFlag = false;
	GFLIB_CtrlPIpAWInit_F16(0, &gsLLC_Drive.sIoutCtrl.sPiParams);
	
	/******************************* IPRIM parameters *******************************/
	/* PI controller parameters */
	gsLLC_Drive.sIprimCtrl.sPiParams.a32PGain = ACC32(IPRIM_KP);
	gsLLC_Drive.sIprimCtrl.sPiParams.a32IGain = ACC32(IPRIM_KI);
	//gsLLC_Drive.sIprimCtrl.sPiParams.f16PreviousIntegralOutput = gsLLC_Drive.sRampPeriod.f16InitVal;
	gsLLC_Drive.sIprimCtrl.sPiParams.f16LowerLim = IPRIM_LOW_LIMIT();
	gsLLC_Drive.sIprimCtrl.sPiParams.f16UpperLim = IPRIM_UP_LIMIT();
	gsLLC_Drive.sIprimCtrl.bstopFlag = false;
	
	/* software protection */
	FaultDetection();	
	/* INIT_DONE command */
	PIT_RUN;
	gsLLC_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}
/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_StateStop(void)
{
	/* software protection */
	FaultDetection();
	if(gsLLC_Drive.sVoutCtrl.f16outFilt < gsLLC_Drive.f16LoadoffVol)
	{
		LOAD_DISABLE;
		bLoadon = 0;
	}
	
	/* if working with PFC, communicate with primary side to determine the LLC work time */
    if(bLLC_Run == 1) gsLLC_Ctrl.uiCtrl |= SM_CTRL_START;
}
/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_StateRun(void)
{
	FaultDetection();
	
	if(bLLC_Run == 0)  gsLLC_Ctrl.uiCtrl |= SM_CTRL_STOP;  
}
/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransFaultInit(void)
{
	
}
/***************************************************************************//*!
*
* @brief   INIT to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransInitFault(void)
{
	
}
/***************************************************************************//*!
*
* @brief   INIT to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransInitStop(void)
{
	
}
/***************************************************************************//*!
*
* @brief   STOP to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransStopFault(void)
{
	
}
/***************************************************************************//*!
*
* @brief   STOP to RUN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransStopRun(void)
{
	/* determine the initial duty cycle according to the initial output voltage */
	if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[0])        gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[0];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[1])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[1];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[2])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[2];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[3])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[3];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[4])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[4];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[5])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[5];
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt < f16VolStart[6])   gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[6];
	else                                                       gsLLC_Drive.sRampDuty.f16InitVal = f16DutyStart[7];
	GFLIB_RampInit_F16(gsLLC_Drive.sRampDuty.f16InitVal, &gsLLC_Drive.sRampDuty.sRamp);
	
	gsLLC_Drive.sRampPeriod.f16InitVal = FRAC16(100/(2*FREQ_START));
	GFLIB_RampInit_F16(gsLLC_Drive.sRampPeriod.f16InitVal, &gsLLC_Drive.sRampPeriod.sRamp);
	
	gsLLC_Drive.sDutyCtrl.f16Duty = gsLLC_Drive.sRampDuty.f16InitVal;
	gsLLC_Drive.sPeriodCtrl.f16HalfTs = gsLLC_Drive.sRampPeriod.f16InitVal;
	
	LLC_CtrllModeUsed = LLC_CtrlModeCmd;  //put the llc mode command to active 
	if (LLC_CtrllModeUsed == OPEN_LOOP)
	{
		gsLLC_Drive.sRampPeriod.f16Target = FRAC16(100/(2*FREQ_OPEN_LOOP));
	}
   else
	{
	   gsLLC_Drive.sRampPeriod.f16Target = gsLLC_Drive.sPeriodCtrl.f16PerHlim;
	}
	if(LLC_CtrllModeUsed == SINGLE_VOLTAGE_LOOP)
	{
		/* PI controller parameters */
		gsLLC_Drive.sVoutCtrl.sPiParams.a32PGain = ACC32(SINGLE_LOOP_VOUT_KP);
		gsLLC_Drive.sVoutCtrl.sPiParams.a32IGain = ACC32(SINGLE_LOOP_VOUT_KI);
		gsLLC_Drive.sVoutCtrl.sPiParams.f16LowerLim = SINGLE_LOOP_VOUT_LOW_LIMIT();
		gsLLC_Drive.sVoutCtrl.sPiParams.f16UpperLim = SINGLE_LOOP_VOUT_UP_LIMIT();
		gsLLC_Drive.sVoutCtrl.bstopFlag = false; 	
	}
	else if((LLC_CtrllModeUsed == DUAL_VOLTAGE_LOOP) || (LLC_CtrllModeUsed == DUAL_OUTER_LOOP))
	{
		/* 2p2z controller parameters */
		gsLLC_Drive.sVoutCtrlParams2P2Z.f16CoeffA1 = FRAC16(DUAL_LOOP_VOUT_COEFF_A1);
		gsLLC_Drive.sVoutCtrlParams2P2Z.f16CoeffA2 = FRAC16(DUAL_LOOP_VOUT_COEFF_A2);
		gsLLC_Drive.sVoutCtrlParams2P2Z.f16CoeffB0 = FRAC16(DUAL_LOOP_VOUT_COEFF_B0);
		gsLLC_Drive.sVoutCtrlParams2P2Z.f16CoeffB1 = FRAC16(DUAL_LOOP_VOUT_COEFF_B1);
		gsLLC_Drive.sVoutCtrlParams2P2Z.f16CoeffB2 = FRAC16(DUAL_LOOP_VOUT_COEFF_B2);
		PCLIB_Ctrl2P2ZInit_F16(&gsLLC_Drive.sVoutCtrlParams2P2Z);
	}
	
	LLC_PWM_UPDATE(FRAC16(100.0/(2*280)), FRAC16(0.28));
	/* Enable PWM output */
	ENABLE_PWM_OUTPUT;
	#if SR_ENABLE == 1
		ENABLE_SR;
	#endif
	/* Init sub-state when transition to RUN */
	eLLC_Runsub = SOFTSTART;
	gsLLC_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}
/***************************************************************************//*!
*
* @brief   RUN to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransRunFault(void)
{
	DISABLE_PWM_OUTPUT;
	DISABLE_SR;
}
/***************************************************************************//*!
*
* @brief   RUN to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransRunStop(void)
{
	gsLLC_Ctrl.eState = STOP;
	DISABLE_PWM_OUTPUT;
	DISABLE_SR;
	eLLC_Runsub = SOFTSTART;
	gsLLC_Drive.sPeriodCtrl.f16PerLlim = FRAC16(100/(2*FREQ_MAX_SOFTSTART));
	gsLLC_Drive.sDutyCtrl.bDutyunder = false;
	gsLLC_Drive.sDutyCtrl.bPWMmode = false;
	gsLLC_Drive.sDutyCtrl.bPWMmodeLast = false;
	gsLLC_Drive.sDutyCtrl.f16Duty = FRAC16(0.23);
	gsLLC_Drive.sPeriodCtrl.f16HalfTs = FRAC16(100/(2*FREQ_START));
	LLC_PWM_UPDATE(gsLLC_Drive.sPeriodCtrl.f16HalfTs, gsLLC_Drive.sDutyCtrl.f16Duty);
	
	gsLLC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}
/***************************************************************************//*!
*
* @brief   RUN Soft start sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/

#pragma section CODES_IN_RAM begin
static void LLC_StateRun_Softstart(void)
{
    if(LLC_CtrllModeUsed == OPEN_LOOP)
    {
    	LOAD_ENABLE;
    	
    	/* increase the duty cycle to 50% first, then decrease frequency to open loop frequency */
	    if(gsLLC_Drive.sDutyCtrl.f16Duty != gsLLC_Drive.sRampDuty.f16Target)
	    {
		    gsLLC_Drive.sDutyCtrl.f16Duty = GFLIB_Ramp_F16(gsLLC_Drive.sRampDuty.f16Target, &gsLLC_Drive.sRampDuty.sRamp);
	    }
	     else if(gsLLC_Drive.sPeriodCtrl.f16HalfTs != gsLLC_Drive.sRampPeriod.f16Target) 
	    { 
		    gsLLC_Drive.sPeriodCtrl.f16HalfTs = GFLIB_Ramp_F16(gsLLC_Drive.sRampPeriod.f16Target, &gsLLC_Drive.sRampPeriod.sRamp);
	    }
	    /* PWM update */
	    LLC_PWM_UPDATE(gsLLC_Drive.sPeriodCtrl.f16HalfTs, gsLLC_Drive.sDutyCtrl.f16Duty);
    }

    else
    {
    	/* if the output voltage reach the soft start end voltage, switch to RUN NORMAL sub-state */
	    if(gsLLC_Drive.sVoutCtrl.f16outFilt < gsLLC_Drive.f16VoutSoftend)
	    {
		    if(gsLLC_Drive.sDutyCtrl.f16Duty < gsLLC_Drive.sRampDuty.f16Target)
		    {
			    gsLLC_Drive.sDutyCtrl.f16Duty = GFLIB_Ramp_F16(gsLLC_Drive.sRampDuty.f16Target, &gsLLC_Drive.sRampDuty.sRamp);
		    }
		    else 
		    {
			    gsLLC_Drive.sPeriodCtrl.f16HalfTs = GFLIB_Ramp_F16(gsLLC_Drive.sRampPeriod.f16Target, &gsLLC_Drive.sRampPeriod.sRamp);
		    }
		    /* PWM update*/
		    LLC_PWM_UPDATE(gsLLC_Drive.sPeriodCtrl.f16HalfTs, gsLLC_Drive.sDutyCtrl.f16Duty);	
	    }
	    else
	    {
		    /* Transition to RUN NORMAL sub-state */
		    LLC_TransRun_Softstart_Normal();
	    }
    }
}
#pragma section CODES_IN_RAM end

/***************************************************************************//*!
*
* @brief   RUN Normal sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void LLC_StateRun_Normal(void)
{
	if(gsLLC_Drive.sVoutCtrl.f16outRef < gsLLC_Drive.sRampVout.f16Target)
	{
		gsLLC_Drive.sVoutCtrl.f16outRef = GFLIB_Ramp_F16(gsLLC_Drive.sRampVout.f16Target, &gsLLC_Drive.sRampVout.sRamp);
	}
	else if(gsLLC_Drive.sVoutCtrl.f16outFilt >= (gsLLC_Drive.sVoutCtrl.f16outRef-LOAD_ON_ACCEPT_ERR))
	{
		LOAD_ENABLE;
		gsLLC_Drive.sPeriodCtrl.f16PerLlim = FRAC16(100/(2*FREQ_MAX_NORMALRUN));
		bLoadon = 1;
	}
	
    if(LLC_CtrllModeUsed == SINGLE_VOLTAGE_LOOP)
    {
    	gsLLC_Drive.sVoutCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sVoutCtrl.f16outRef, gsLLC_Drive.sVoutCtrl.f16outFilt);
    	gsLLC_Drive.sVoutCtrl.f16PIResult = GFLIB_CtrlPIpAW_F16(gsLLC_Drive.sVoutCtrl.f16outError,&gsLLC_Drive.sVoutCtrl.bstopFlag, &gsLLC_Drive.sVoutCtrl.sPiParams);
    	gsLLC_Drive.sPeriodCtrl.f16HalfTs = GFLIB_LowerLimit_F16(gsLLC_Drive.sVoutCtrl.f16PIResult, gsLLC_Drive.sPeriodCtrl.f16PerLlim);
	
	    if(bLoadon == 0)
	    {
		    gsLLC_Drive.sDutyCtrl.f16Duty = gsLLC_Drive.sDutyCtrl.f16DutyMax;
	    }
	    else
	    {
	    	/* f16PerLlim=0.25 in RUN NORMAL sub-state, use the following conversion relationship to 
	    	 * smoothly switch between PFM and PWM. */
		    if(gsLLC_Drive.sVoutCtrl.f16PIResult >= gsLLC_Drive.sPeriodCtrl.f16PerLlim)
			{gsLLC_Drive.sDutyCtrl.f16Duty = gsLLC_Drive.sDutyCtrl.f16DutyMax;}
		    else
			{gsLLC_Drive.sDutyCtrl.f16Duty = (gsLLC_Drive.sVoutCtrl.f16PIResult<<1);}
	    }
    }
    else 
    {
	    if(!gsLLC_Drive.sDutyCtrl.bPWMmode)  //PFM mode
	    {
		    gsLLC_Drive.sVoutCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sVoutCtrl.f16outRef, gsLLC_Drive.sVoutCtrl.f16outFilt);
		    gsLLC_Drive.sVoutCtrl.f16PIResult = PCLIB_Ctrl2P2Z_F16(gsLLC_Drive.sVoutCtrl.f16outError, &gsLLC_Drive.sVoutCtrlParams2P2Z);
		    gsLLC_Drive.sVoutCtrl.f16PIResult = GFLIB_Limit_F16(gsLLC_Drive.sVoutCtrl.f16PIResult<<1,0,IPRIM_REF_LIMIT);
		    gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY1 = gsLLC_Drive.sVoutCtrl.f16PIResult;
            if(LLC_CtrllModeUsed == DUAL_OUTER_LOOP)
            {
    	        gsLLC_Drive.sIoutCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sIoutCtrl.f16outRef, gsLLC_Drive.sIoutCtrl.f16outFilt);
    	        gsLLC_Drive.sIoutCtrl.f16PIResult = GFLIB_CtrlPIpAW_F16(gsLLC_Drive.sIoutCtrl.f16outError,&gsLLC_Drive.sIoutCtrl.bstopFlag, &gsLLC_Drive.sIoutCtrl.sPiParams);
    	        gsLLC_Drive.sIprimCtrl.f16outRef = MLIB_Add_F16(gsLLC_Drive.sVoutCtrl.f16PIResult, gsLLC_Drive.sIoutCtrl.f16PIResult);
            }
            else
            {
    	        gsLLC_Drive.sIprimCtrl.f16outRef = gsLLC_Drive.sVoutCtrl.f16PIResult;
            }

            gsLLC_Drive.sIprimCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sIprimCtrl.f16outRef, gsLLC_Drive.sIprimCtrl.f16outFilt);
            gsLLC_Drive.sIprimCtrl.f16PIResult = GFLIB_CtrlPIpAW_F16(gsLLC_Drive.sIprimCtrl.f16outError,&gsLLC_Drive.sIprimCtrl.bstopFlag, &gsLLC_Drive.sIprimCtrl.sPiParams);
        }
        else //PWM mode
        {
        	/* disable inner Iprim loop in PWM mode */
   	        if(gsLLC_Drive.sIoutCtrl.f16outFilt < FRAC16(0.35))
	        {
   	    	    gsLLC_Drive.sVoutCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sVoutCtrl.f16outRef, gsLLC_Drive.sVoutCtrl.f16outFilt);
   	    	    gsLLC_Drive.sVoutCtrl.f16PIResult = PCLIB_Ctrl2P2Z_F16(gsLLC_Drive.sVoutCtrl.f16outError, &gsLLC_Drive.sVoutCtrlParams2P2Z);
   	    	    gsLLC_Drive.sVoutCtrl.f16PIResult = GFLIB_Limit_F16(gsLLC_Drive.sVoutCtrl.f16PIResult<<1,0,gsLLC_Drive.sIprimCtrl.sPiParams.f16UpperLim);
   	    	    gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY1 = gsLLC_Drive.sVoutCtrl.f16PIResult;
   	    	    gsLLC_Drive.sIprimCtrl.f16PIResult = gsLLC_Drive.sVoutCtrl.f16PIResult;
    	    }
	        else
	        {
	    	    gsLLC_Drive.sIoutCtrl.f16outError = MLIB_Sub_F16(gsLLC_Drive.sIoutCtrl.f16outRef, gsLLC_Drive.sIoutCtrl.f16outFilt);
	    	    gsLLC_Drive.sIprimCtrl.f16PIResult = GFLIB_CtrlPIpAW_F16(gsLLC_Drive.sIoutCtrl.f16outError,&gsLLC_Drive.sIprimCtrl.bstopFlag,&gsLLC_Drive.sIprimCtrl.sPiParams);
	        } 
        }
	    
	    gsLLC_Drive.sPeriodCtrl.f16HalfTs = GFLIB_LowerLimit_F16(gsLLC_Drive.sIprimCtrl.f16PIResult, gsLLC_Drive.sPeriodCtrl.f16PerLlim);
	    gsLLC_Drive.sDutyCtrl.bPWMmodeLast = gsLLC_Drive.sDutyCtrl.bPWMmode;
	    if(bLoadon == 0) // when load is not connected, no PWM mode
	    {
		    gsLLC_Drive.sDutyCtrl.f16Duty = gsLLC_Drive.sDutyCtrl.f16DutyMax;
		    gsLLC_Drive.sDutyCtrl.bPWMmode = false;
	    }
	    else
	    {
	    	/* f16PerLlim=0.25 in RUN NORMAL sub-state, use the following conversion relationship to 
	         * smoothly switch between PFM and PWM. */
	    	if(gsLLC_Drive.sIprimCtrl.f16PIResult >= gsLLC_Drive.sPeriodCtrl.f16PerLlim)
		    {
		    	gsLLC_Drive.sDutyCtrl.f16Duty = gsLLC_Drive.sDutyCtrl.f16DutyMax;
		        gsLLC_Drive.sDutyCtrl.bPWMmode = false; 
		    }
		    else
		    {
		    	gsLLC_Drive.sDutyCtrl.f16Duty = (gsLLC_Drive.sIprimCtrl.f16PIResult<<1);
		        gsLLC_Drive.sDutyCtrl.bPWMmode = true; 
		    }
	    }
	    	
        if(!gsLLC_Drive.sDutyCtrl.bPWMmode)  //PFM mode
        {
    	    if(gsLLC_Drive.sDutyCtrl.bPWMmodeLast) //last state: PWM mode
    	    {
    		    if(gsLLC_Drive.sIoutCtrl.f16outFilt < FRAC16(0.35))
    		    {
    			    /*smoothly transfer: single voltage loop switch to dual loop, transmit single 
    			     *voltage loop halfTs value to Iprim controller past result  */
    		    	gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY1 = gsLLC_Drive.sIprimCtrl.f16outFilt;
    			    gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY2 = gsLLC_Drive.sIprimCtrl.f16outFilt;
    			    gsLLC_Drive.sIprimCtrl.sPiParams.f32IAccK_1 = (frac32_t)gsLLC_Drive.sPeriodCtrl.f16HalfTs<<16;
    		    }
    		    else
    		    {
    		    	/*smoothly transfer: single output current loop switch to dual loop, reset Iprim controller
    		    	 *parameters to its own value, and transmit current current controller output to IOUT controller past result */
    			    gsLLC_Drive.sIprimCtrl.sPiParams.a32PGain = ACC32(IPRIM_KP);
    			    gsLLC_Drive.sIprimCtrl.sPiParams.a32IGain = ACC32(IPRIM_KI);
    			    gsLLC_Drive.sIoutCtrl.sPiParams.f32IAccK_1 = MLIB_Sub_F32((frac32_t)gsLLC_Drive.sIprimCtrl.f16outFilt<<16,(frac32_t)gsLLC_Drive.sVoutCtrl.f16PIResult<<16);
    		    }
    	    }
        }
        else //PWM mode
        {
    	    if(!gsLLC_Drive.sDutyCtrl.bPWMmodeLast)//last state: PFM mode
    	    {
    		    if(gsLLC_Drive.sIoutCtrl.f16outFilt < FRAC16(0.35))
    		    {
    		    	/*smoothly transfer:dual loop switch to single voltage loop , transmit dual 
    			     *loop halfTs value to VOUT controller past result  */ 
    			    gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY1 = gsLLC_Drive.sPeriodCtrl.f16HalfTs;
    			    gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY2 = gsLLC_Drive.sPeriodCtrl.f16HalfTs;
    		    }
    		    else
    		    {
    		    	/*smoothly transfer:dual loop switch to single current loop , assign Iprim controller 
    		    	 *parameters to IOUT controller parameters, use Iprim controller for later calculation,
    		    	 *no past value transfer is needed */ 
    			    gsLLC_Drive.sIprimCtrl.sPiParams.a32PGain = ACC32(IOUT_KP);
    			    gsLLC_Drive.sIprimCtrl.sPiParams.a32IGain = ACC32(IOUT_KI);
    		    }
    	    }
        }
    }
    
    /* burst on/off control, when duty cycle is less than the burstoff duty, disable pwm output,
     * when duty cycle is greater than burston duty, enable pwm output */
	if(gsLLC_Drive.sDutyCtrl.f16Duty >= gsLLC_Drive.sDutyCtrl.f16BurstonDuty)
	{
		gsLLC_Drive.sDutyCtrl.bDutyunder = false;
	}
	else if(gsLLC_Drive.sDutyCtrl.f16Duty <= gsLLC_Drive.sDutyCtrl.f16BurstoffDuty) 
	{
		gsLLC_Drive.sDutyCtrl.bDutyunder = true;
	}
	
	if(!gsLLC_Drive.sDutyCtrl.bDutyunder)
	{
		ENABLE_PWM_OUTPUT;
		ENABLE_SR;
		LLC_PWM_UPDATE(gsLLC_Drive.sPeriodCtrl.f16HalfTs, gsLLC_Drive.sDutyCtrl.f16Duty);
	}
	else
	{
		DISABLE_PWM_OUTPUT;
		DISABLE_SR;
	}	
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   RUN light-load sub-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void LLC_StateRun_Lightload(void)
{
	
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   RUN SOFTSTART to NORMAL transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void LLC_TransRun_Softstart_Normal(void)
{
    if((LLC_CtrllModeUsed == DUAL_VOLTAGE_LOOP) || (LLC_CtrllModeUsed == DUAL_OUTER_LOOP))
    {
    	/* smoothly switch: open loop control switch to close loop control, load is not connected yet,
    	 * transmit current halfTs to Iprim controller last result, and current measured Iprim to Vout controller last result. */
    	gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY1 = gsLLC_Drive.sIprimCtrl.f16outFilt;
    	gsLLC_Drive.sVoutCtrlParams2P2Z.f16DelayY2 = gsLLC_Drive.sIprimCtrl.f16outFilt;
    	GFLIB_CtrlPIpAWInit_F16(gsLLC_Drive.sPeriodCtrl.f16HalfTs, &gsLLC_Drive.sIprimCtrl.sPiParams);
    }
    
    if(LLC_CtrllModeUsed == SINGLE_VOLTAGE_LOOP)
    {
    	/* smoothly switch: open loop control switch to close loop control, 
    	 * transmit current halfTs to Vout controller last result. */
    	GFLIB_CtrlPIpAWInit_F16(gsLLC_Drive.sPeriodCtrl.f16HalfTs, &gsLLC_Drive.sVoutCtrl.sPiParams);
    }
    
    gsLLC_Drive.sVoutCtrl.f16outRef = GFLIB_UpperLimit_F16(gsLLC_Drive.sVoutCtrl.f16outFilt, gsLLC_Drive.sRampVout.f16Target);
    gsLLC_Drive.sRampVout.f16InitVal = gsLLC_Drive.sVoutCtrl.f16outRef;
    GFLIB_RampInit_F16(gsLLC_Drive.sRampVout.f16InitVal, &gsLLC_Drive.sRampVout.sRamp);
    /*  sub-state Run NORMAL */
    eLLC_Runsub = NORMAL;
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   RUN normal to light-load transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransRun_Normal_Lightload(void)
{
	
}
/***************************************************************************//*!
*
* @brief   RUN light-load to normal transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LLC_TransRun_Lightload_Normal(void)
{
	
}


