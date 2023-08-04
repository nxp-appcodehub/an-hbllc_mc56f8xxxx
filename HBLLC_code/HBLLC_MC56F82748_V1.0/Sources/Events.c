/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/* ###################################################################
**     Filename    : Events.c
**     Project     : LLC_Stru_82748
**     Processor   : MC56F82748VLH
**     Component   : Events
**     Version     : Driver 01.03
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2015-09-08, 17:55, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file Events.c
** @version 01.03
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Init_Config.h"
#include "Cpu.h"
#include "Events.h"
#include "LLC_statemachine.h"

/* User includes (#include below this line is not maintained by Processor Expert) */

frac16_t  f16VoutF[4] = {0}, f16IoutF[4] = {0}, f16IprimF[4] = {0};
int m = 0;

#pragma section CODES_IN_RAM begin
#pragma interrupt saveall
void PWM_Trigger_ISR() // frequency changes according to switching frequency
{
	TP30_UP;
	/* read ADC results, make sure the results are ready */
	while(!ADC_RDY_08) { }
	gsLLC_Drive.sVoutCtrl.f16out = ADC_RSLT8;
	gsLLC_Drive.sIoutCtrl.f16out = ADC_RSLT0;
	
	/* moving average filter */
	gsLLC_Drive.sVoutCtrl.f16outFilt -= f16VoutF[m];
	gsLLC_Drive.sIoutCtrl.f16outFilt -= f16IoutF[m];
	f16VoutF[m] = gsLLC_Drive.sVoutCtrl.f16out>>2;
	f16IoutF[m] = gsLLC_Drive.sIoutCtrl.f16out>>2;
	gsLLC_Drive.sVoutCtrl.f16outFilt += f16VoutF[m];
	gsLLC_Drive.sIoutCtrl.f16outFilt += f16IoutF[m];
		
	while(!ADC_RDY_19) { }
	gsLLC_Drive.sIprimCtrl.f16out = ADC_RSLT1;
		
	gsLLC_Drive.sIprimCtrl.f16outFilt -= f16IprimF[m];
	f16IprimF[m] = gsLLC_Drive.sIprimCtrl.f16out>>2;
	gsLLC_Drive.sIprimCtrl.f16outFilt += f16IprimF[m];

	m++;
	if (m > 3)  m = 0;
		
	/* RUN sub-state machine call */
	if (gsLLC_Ctrl.eState == RUN)
	{
		mLLC_STATE_RUN_TABLE[eLLC_Runsub]();
	}
	
	/* clear interrupt flag */
	PWMA_SM0STS |= PWM_STS_CMPF(1);
		
	FMSTR_Recorder();
	TP30_DOWN;
}
#pragma interrupt off
#pragma section CODES_IN_RAM end


#pragma interrupt on
void PIT0_ISR() //1kHz
{
	//TP31_UP;
	u16LLCReceiveMsgDelay++;
	if(u16LLCReceiveMsgDelay > LLC_RECEIVE_MESSAGE_RESTART_DURATION) 
	{
		u16LLCReceiveMsgDelay = LLC_RECEIVE_MESSAGE_RESTART_DURATION;
	}
	
	u16LLCMsgTransDelay++;
	
	u16PIT_Count++;
	
	/* clear interrupt flag */
	PIT0_CTRL &= ~PIT_CTRL_PRF_MASK;
	//TP31_DOWN;
}
#pragma interrupt off

/* END Events */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale 56800 series of microcontrollers.
**
** ###################################################################
*/
