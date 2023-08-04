/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * ISR.c
 *
 *  Created on: Aug 21, 2019
 *      Author: nxa16823
 */
#include "isr.h"
#include "Cpu.h"
#include "LLC_statemachine.h"
#include "freemaster.h"

frac16_t  f16VoutF[4] = {0}, f16IoutF[4] = {0}, f16IprimF[4] = {0}, f16VprimF[4] = {0};
int m = 0;

#pragma section CODES_IN_RAM begin
#pragma interrupt saveall
void PWMA0_trig0_isr()
{
	TP30_UP;
	/* read ADC results, make sure the results are ready */
	while(!ADC_RDY_08) { }
	while(!ADC_RDY_08) { }
	gsLLC_Drive.sVoutCtrl.f16out = ADC->RSLT[8];
	gsLLC_Drive.sIoutCtrl.f16out = ADC->RSLT[0];
		
	/* moving average filter */
	gsLLC_Drive.sVoutCtrl.f16outFilt -= f16VoutF[m];
	gsLLC_Drive.sIoutCtrl.f16outFilt -= f16IoutF[m];
	f16VoutF[m] = gsLLC_Drive.sVoutCtrl.f16out>>2;
	f16IoutF[m] = gsLLC_Drive.sIoutCtrl.f16out>>2;
	gsLLC_Drive.sVoutCtrl.f16outFilt += f16VoutF[m];
	gsLLC_Drive.sIoutCtrl.f16outFilt += f16IoutF[m];
			
	while(!ADC_RDY_19) { }
	gsLLC_Drive.sIprimCtrl.f16out = ADC->RSLT[1];
			
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
	PWM->SM[0].STS |= PWM_STS_CMPF(1);
	
	FMSTR_Recorder();
	TP30_DOWN;
}
#pragma interrupt off
#pragma section CODES_IN_RAM end

#pragma interrupt on
void PIT0_isr() //1kHz
{
	TP31_UP;
	u16LLCReceiveMsgDelay++;
	if(u16LLCReceiveMsgDelay > LLC_RECEIVE_MESSAGE_RESTART_DURATION) 
	{
		u16LLCReceiveMsgDelay = LLC_RECEIVE_MESSAGE_RESTART_DURATION;
	}
		
	u16LLCMsgTransDelay++;
		
	u16PIT_Count++;
	
	/* clear interrupt flag */
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;
	TP31_DOWN;
}
#pragma interrupt off
