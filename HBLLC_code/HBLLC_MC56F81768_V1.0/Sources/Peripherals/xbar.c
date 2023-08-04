/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/
#include "xbar.h"

/*
 *  Use the MACRO in xbar.h to configure XBARA: XBAR_INIT(in,out)
 *  
 *  "in" is the input signal to XBARA, choose from the MACROs in "XBAR input signals"
 *  "out" is the output signal of XBARA, choose from the MACROs in "XBAR output signals"
 *  
 *  E.g.
 *  	XBAR_INIT(in_AN0_LIMIT_or_TIMERA0_OUT, out_ADCA_TRIG);
 *  
 *  This is to use AN0_LIMIT or TIMERA0_OUT (decided by SIM_ADC_TMR_SEL) to trigger ADCA. 	
 *  
 * */
