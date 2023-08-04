/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/



/*  freescale sample code  */



#include <stdio.h>
#include <stdlib.h>
#include "cpu.h"

#include "derivative.h" /* include peripheral declarations */
#include "freemaster.h"
#include "LLC_statemachine.h"


//-------------------- Macros definitions ----------------------


//-------------------- Global variables ------------------------
extern uint16_t interrupt_base_addr_high, interrupt_base_addr_low;
uint16_t u16LLCReceiveMsgDelay = 0, u16LLCMsgTransDelay = 0;

//-------------------- Local variables ------------------------
// communication related
uint8_t  RxMsg_num=0,RxMsg_head,RxMsg_1,RxMsg_2,RxMsg_3,RxMsg_checksum, Vprim_flag=0; 
uint16_t uw16Vprim; /* received primary voltage */
uint8_t	 TxMsg_seq=0,TxMsg_checksum,FaultId_H, FaultId_L, LLC_State, LLC_VOUT_H, LLC_VOUT_L, LLC_IOUT_H, LLC_IOUT_L;
acc16_t  LLC_VOUT, LLC_IOUT;

//-------------------- Function declarations ----------------------


void main(void)
 {	
	INTC->VBA = (uint16_t)(((uint32_t)(&interrupt_base_addr_high)<<16+(uint32_t)(&interrupt_base_addr_low))>>8);
	
	Peripheral_init();
	FMSTR_Init();
	
	__EI(0); // enable interrupts of all priorities at CPU core
	
    PWMA_RUN;
	while(1)
	{
		TP31_UP;
		FMSTR_Poll();
		TP31_DOWN;

#if COMM_WITH_PFC_ENABLE
	  if(u16LLCReceiveMsgDelay >= LLC_RECEIVE_MESSAGE_RESTART_DURATION)   RxMsg_num = 0;	  
	  
	  /* message received from front end, two kind of message can be identified:  
	   * Bus voltage (Vprim) inform message: head=0x2
	   * LLC run/stop control message: head=0x1
	   * */
	  if(QSCI0->STAT & QSCI_STAT_RDRF_MASK)
	  {
	      if(RxMsg_num == 0)
	      {
	    	  RxMsg_head = QSCI0->DATA;
	  	      if(RxMsg_head == 0x01 || RxMsg_head == 0x02) { RxMsg_num ++;  u16LLCReceiveMsgDelay = 0;}
	      }
	      else if(RxMsg_num == 1)
	      {
	    	  RxMsg_1 = QSCI0->DATA;
	    	  RxMsg_checksum = RxMsg_head ^ RxMsg_1;
	          RxMsg_num++; 
	    	  u16LLCReceiveMsgDelay = 0;
	      }
	      else if(RxMsg_num == 2)
	      {
	    	  RxMsg_2 = QSCI0->DATA;
	    	  if(RxMsg_head == 0x01)
	    	  {
	    		  RxMsg_num = 0;
	    	  	  if(RxMsg_checksum == RxMsg_2)   
	    	  	  { 
	    	  		  if(RxMsg_1 == 0x55 && Vprim_flag > 0) {bLLC_Run = 1;Vprim_flag = 2;}
	    	  		  else if (RxMsg_1 == 0x33) {bLLC_Run = 0;Vprim_flag = 2;}
	    	  	  }
	    	  }
	    	  else {RxMsg_num++; u16LLCReceiveMsgDelay = 0; RxMsg_checksum ^= RxMsg_2; } 	  
	      }
	      else
	      {
	    	  RxMsg_3 = QSCI0->DATA; 
	          RxMsg_num = 0; 
	          if(RxMsg_checksum == RxMsg_3)
	          {
	          	uw16Vprim = (uint16_t)RxMsg_1<<8 | RxMsg_2;
	          	/* LLC starts immediately when the input voltage meets the conditions */
	          	if(uw16Vprim >= VPRIM_LOW_LIMIT && uw16Vprim <= VPRIM_UP_LIMIT)  { bLLC_Run = 1; Vprim_flag = 2;} 
	          }
	      }
	  }
	  
	  /* LLC transmit message to front end, two kind of message are transferred: 
	   * the LLC state, faultId, VOUT and IOUT are transmitted about every 200ms: head=0x15 
	   * received control command confirmation: head=0x12
	   * */
	  if(u16LLCMsgTransDelay >= LLC_MESSAGE_TRANS_DURATION) 
	  {
		  if(QSCI0->STAT & QSCI_STAT_TDRE_MASK)
		  {
			  if(TxMsg_seq == 0)
			  {
		  	      QSCI0->DATA = 0x15; // the head of tx message is 0x15
		  	      LLC_State = gsLLC_Ctrl.eState;
		  	      QSCI0->DATA = LLC_State;
		  	      FaultId_H = gsLLC_Drive.f16FaultId >> 8;
		  	      FaultId_L = gsLLC_Drive.f16FaultId;
		  	      QSCI0->DATA = FaultId_H;
		  	      QSCI0->DATA = FaultId_L;
		  	      TxMsg_checksum = 0x15 ^ gsLLC_Ctrl.eState ^ FaultId_H ^ FaultId_L;
		  	      TxMsg_seq = 1;
		      }
			  else if(TxMsg_seq == 1) 
			  {
				  LLC_VOUT = MLIB_Mul_F16(ACC16(VOUT_SCALE),gsLLC_Drive.sVoutCtrl.f16outFilt);
				  LLC_IOUT = MLIB_Mul_F16(ACC16(IOUT_SCALE),gsLLC_Drive.sIoutCtrl.f16outFilt);
				  LLC_VOUT_H = LLC_VOUT>>8;
				  LLC_VOUT_L = LLC_VOUT;
				  QSCI0->DATA = LLC_VOUT_H;
				  QSCI0->DATA = LLC_VOUT_L;
				  LLC_IOUT_H = LLC_IOUT>>8;
				  LLC_IOUT_L = LLC_IOUT;
				  QSCI0->DATA = LLC_IOUT_H;
				  QSCI0->DATA = LLC_IOUT_L;
				  TxMsg_checksum = LLC_VOUT_H ^ LLC_VOUT_L ^ LLC_IOUT_H ^ LLC_IOUT_L ^ TxMsg_checksum;
				  TxMsg_seq = 2;
			  }
			  else
			  {
				  QSCI0->DATA = TxMsg_checksum;
				  TxMsg_seq = 0;
				  u16LLCMsgTransDelay = 0;
			  }
	      }
	  }
	  else if(Vprim_flag == 2) /* feedback confirmation message to front end the control command received by LLC */
	  {
	  	  if(QSCI0->STAT & QSCI_STAT_TDRE_MASK)
	  	  {
	  		  if(bLLC_Run == 1)
	  		  {
	  	         QSCI0->DATA = 0x12;
	  	         QSCI0->DATA = 0x01; /* LLC run command is received */
	  	         QSCI0->DATA = 0x13;
	  	         Vprim_flag = 1; /* no repeat confirmation message when no new command is received */
	  		  }
	  		  else
	  		  {
	  			 QSCI0->DATA = 0x12;
	  		     QSCI0->DATA = 0x03; // LLC stop command is received
	  		     QSCI0->DATA = 0x11;
	  		     Vprim_flag = 1; /* no repeat confirmation message when no new command is received */
	  		  }
	  	  }
	  }
#endif
	      	 	
	  /* Call state-machine */
	  SM_StateMachine(&gsLLC_Ctrl);
	}
}




