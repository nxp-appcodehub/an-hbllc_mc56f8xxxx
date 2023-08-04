/* 
* Copyright 2019 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/
    
#include "Cpu.h"
#include "ISR.h"

#ifndef _lint

extern void _EntryPoint(void);         /* Startup routine */


volatile asm void _vect(void);
#pragma define_section interrupt_vectors "interrupt_vectors.text"  RX
#pragma section interrupt_vectors begin
volatile asm void _vect(void) {
  JMP  _EntryPoint                     /* Interrupt no. 0 (Used)    */
  JMP  _EntryPoint                     /* Interrupt no. 1 (Used)    */
  JSR  ivINT_ILLEGAL_OP                /* Interrupt no. 2       */
  JSR  ivINT_SWI3                      /* Interrupt no. 3    	*/
  JSR  ivINT_OVERFLOW                  /* Interrupt no. 4    	*/
  JSR  ivINT_MISALIGNED                /* Interrupt no. 5    	*/
  JSR  ivINT_STPCNT                    /* Interrupt no. 6    	*/
  JSR  ivINT_BKPT                      /* Interrupt no. 7    	*/
  JSR  ivINT_TRBUF                     /* Interrupt no. 8    	*/
  JSR  ivINT_TX_REG                    /* Interrupt no. 9    	*/
  JSR  ivINT_RX_REG                    /* Interrupt no. 10   	*/
  JSR  ivINT_BUS_ERR                   /* Interrupt no. 11   	*/
  JSR  ivINT_PLAT_3                    /* Interrupt no. 12   	*/
  JSR  ivINT_PLAT_2                    /* Interrupt no. 13   	*/
  JSR  ivINT_PLAT_1                    /* Interrupt no. 14   	*/
  JSR  ivINT_SWI2                      /* Interrupt no. 15   	*/
  JSR  ivINT_SWI1                      /* Interrupt no. 16   	*/
  JSR  ivINT_SWI0                      /* Interrupt no. 17   	*/
  JSR  ivINT_XBARA                     /* Interrupt no. 18   	*/
  JSR  ivINT_LVI1                      /* Interrupt no. 19      */
  JSR  ivINT_OCCS                      /* Interrupt no. 20      */
  JSR  ivINT_TMRB_3                    /* Interrupt no. 21      */
  JSR  ivINT_TMRB_2                    /* Interrupt no. 22      */
  JSR  ivINT_TMRB_1                    /* Interrupt no. 23      */
  JSR  ivINT_TMRB_0                    /* Interrupt no. 24      */
  JSR  ivINT_TMRA_3                    /* Interrupt no. 25      */
  JSR  ivINT_TMRA_2                    /* Interrupt no. 26      */
  JSR  ivINT_TMRA_1                    /* Interrupt no. 27      */
  JSR  ivINT_TMRA_0                    /* Interrupt no. 28      */
  JSR  ivINT_ADC12_CC1                 /* Interrupt no. 29      */
  JSR  ivINT_ADC12_CC0                 /* Interrupt no. 30      */
  JSR  ivINT_ADC12_ERR                 /* Interrupt no. 31      */
  JSR  ivINT_DMA_ERR                   /* Interrupt no. 32      */
  JSR  ivINT_DMA3                      /* Interrupt no. 33      */
  JSR  ivINT_DMA2                      /* Interrupt no. 34      */
  JSR  ivINT_DMA1                      /* Interrupt no. 35      */
  JSR  ivINT_DMA0                      /* Interrupt no. 36      */
  JSR  ivINT_CAN_MB_OR                 /* Interrupt no. 37      */
  JSR  ivINT_CAN_BUS_OFF               /* Interrupt no. 38      */
  JSR  ivINT_CAN_ERROR                 /* Interrupt no. 39      */
  JSR  ivINT_CAN_TX_WARN               /* Interrupt no. 40      */
  JSR  ivINT_CAN_RX_WARN               /* Interrupt no. 41      */
  JSR  ivINT_CAN_WAKEUP                /* Interrupt no. 42      */
  JSR  ivINT_QSCI2_RERR                /* Interrupt no. 43      */
  JSR  ivINT_QSCI2_RCV                 /* Interrupt no. 44      */
  JSR  ivINT_QSCI2_TIDLE               /* Interrupt no. 45      */
  JSR  ivINT_QSCI2_TDRE                /* Interrupt no. 46      */
  JSR  ivINT_QSCI1_RERR                /* Interrupt no. 47      */
  JSR  ivINT_QSCI1_RCV                 /* Interrupt no. 48      */
  JSR  ivINT_QSCI1_TIDLE               /* Interrupt no. 49      */
  JSR  ivINT_QSCI1_TDRE                /* Interrupt no. 50      */
  JSR  ivINT_QSCI0_RERR                /* Interrupt no. 51      */
  JSR  ivINT_QSCI0_RCV                 /* Interrupt no. 52      */
  JSR  ivINT_QSCI0_TIDLE               /* Interrupt no. 53      */
  JSR  ivINT_QSCI0_TDRE                /* Interrupt no. 54      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 55      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 56      */
  JSR  ivINT_QSPI1_XMIT                /* Interrupt no. 57      */
  JSR  ivINT_QSPI1_RCV                 /* Interrupt no. 58      */
  JSR  ivINT_QSPI0_XMIT                /* Interrupt no. 59      */
  JSR  ivINT_QSPI0_RCV                 /* Interrupt no. 60      */
  JSR  ivINT_IIC1                      /* Interrupt no. 61      */
  JSR  ivINT_IIC0                      /* Interrupt no. 62      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 63      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 64      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 65      */
  JSR  ivINT_eFlexPWMB_FAULT           /* Interrupt no. 66      */
  JSR  ivINT_eFlexPWMB_RERR            /* Interrupt no. 67      */
  JSR  ivINT_eFlexPWMB_CAP             /* Interrupt no. 68      */
  JSR  ivINT_eFlexPWMB_RELOAD3         /* Interrupt no. 69      */
  JSR  ivINT_eFlexPWMB_CMP3            /* Interrupt no. 70      */
  JSR  ivINT_eFlexPWMB_RELOAD2         /* Interrupt no. 71      */
  JSR  ivINT_eFlexPWMB_CMP2            /* Interrupt no. 72      */
  JSR  ivINT_eFlexPWMB_RELOAD1         /* Interrupt no. 73      */
  JSR  ivINT_eFlexPWMB_CMP1            /* Interrupt no. 74      */
  JSR  ivINT_eFlexPWMB_RELOAD0         /* Interrupt no. 75      */
  JSR  ivINT_eFlexPWMB_CMP0            /* Interrupt no. 76      */
  JSR  ivINT_eFlexPWMA_FAULT           /* Interrupt no. 77      */
  JSR  ivINT_eFlexPWMA_RERR            /* Interrupt no. 78      */
  JSR  ivINT_eFlexPWMA_CAP             /* Interrupt no. 79      */
  JSR  ivINT_eFlexPWMA_RELOAD3         /* Interrupt no. 80      */
  JSR  ivINT_eFlexPWMA_CMP3            /* Interrupt no. 81      */
  JSR  ivINT_eFlexPWMA_RELOAD2         /* Interrupt no. 82      */
  JSR  ivINT_eFlexPWMA_CMP2            /* Interrupt no. 83      */
  JSR  ivINT_eFlexPWMA_RELOAD1         /* Interrupt no. 84      */
  JSR  ivINT_eFlexPWMA_CMP1            /* Interrupt no. 85      */
  JSR  ivINT_eFlexPWMA_RELOAD0         /* Interrupt no. 86      */
  JSR  ivINT_eFlexPWMA_CMP0            /* Interrupt no. 87      */
  JSR  ivINT_FTFE_RDCOL                /* Interrupt no. 88      */
  JSR  ivINT_FTFE_CC                   /* Interrupt no. 89      */
  JSR  ivINT_CMPD                      /* Interrupt no. 90      */
  JSR  ivINT_CMPC                      /* Interrupt no. 91      */
  JSR  ivINT_CMPB                      /* Interrupt no. 92      */
  JSR  ivINT_CMPA                      /* Interrupt no. 93      */
  JSR  ivINT_PIT1_ROLLOVR              /* Interrupt no. 94      */
  JSR  ivINT_PIT0_ROLLOVR              /* Interrupt no. 95      */
  JSR  ivINT_FTFE_DFD                  /* Interrupt no. 96      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 97      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 98      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 99      */
  JSR  Cpu_Interrupt                   /* Interrupt no. 100     */
  JSR  ivINT_GPIOG                     /* Interrupt no. 101     */
  JSR  ivINT_GPIOF                     /* Interrupt no. 102     */
  JSR  ivINT_GPIOE                     /* Interrupt no. 103     */
  JSR  ivINT_GPIOD                     /* Interrupt no. 104     */
  JSR  ivINT_GPIOC                     /* Interrupt no. 105     */
  JSR  ivINT_GPIOB                     /* Interrupt no. 106     */
  JSR  ivINT_GPIOA                     /* Interrupt no. 107     */
  JSR  ivINT_COP_INT                   /* Interrupt no. 108     */
  JSR  ivINT_EWM_INT                   /* Interrupt no. 109     */
  JSR  ivINT_SWILP                     /* Interrupt no. 110     */
  JSR  ivINT_USB                       /* Interrupt no. 111     */
}
#pragma section interrupt_vectors end

#endif _lint

/* Disable MISRA rules for interurupt routines */
/*lint -esym(765,Cpu_Interrupt) Disable MISRA rule (8.10) checking for symbols (Cpu_Interrupt). */

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
