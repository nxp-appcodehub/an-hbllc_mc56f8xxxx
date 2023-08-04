/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * LLC_statemachine.h
 *
 *  Created on: Aug 23, 2019
 *      Author: nxa16823
 */

#ifndef LLC_STATEMACHINE_H_
#define LLC_STATEMACHINE_H_

#include "state_machine.h"
#include "HBLLC_structure.h"
#include "HBLLC_ctrl.h"

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
	SOFTSTART       = 0,
	NORMAL          = 1,
	LIGHTLOAD       = 2
} LLC_RUN_SUBSTATE_T;     /* Application Run sub-state identification enum */

/******************************************************************************
* Global variables
******************************************************************************/
extern SM_APP_CTRL_T        gsLLC_Ctrl;
extern LLC_STRUC_T          gsLLC_Drive;
extern LLC_RUN_SUBSTATE_T   eLLC_Runsub;
__pmem extern const PFCN_VOID_VOID mLLC_STATE_RUN_TABLE[3];

extern uint16_t  u16LLCReceiveMsgDelay, u16LLCMsgTransDelay;
extern uint16_t  u16PIT_Count;  
extern bool_t    bLLC_Run;

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* section definition
******************************************************************************/
#pragma define_section CODES_IN_RAM "codesInRam.text"  RX

#endif /* LLC_STATEMACHINE_H_ */
