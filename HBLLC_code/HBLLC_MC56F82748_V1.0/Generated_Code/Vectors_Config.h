/** ###################################################################
**      Filename    : Vectors.h
**      Processor   : MC56F82748VLH
**      Version     : 1.0
** 
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file Vectors.h                                                  
** @version 01.00
*/         
/*!
**  @addtogroup Vectors_module Vectors module documentation
**  @{
*/         

#ifndef __Vectors_Config_H
#define __Vectors_Config_H

/* MODULE Vectors_Config.h */

#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
#include "FMSTR1.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CPU_INT_VECTOR_ROM                                 0x01U     /* Interrupt vector table placed in ROM */ 
#define CPU_BOOT_ISR_INIT                                  0x00U     /* Boot vector not used */ 

/* 
   Interrupt vector table service routine names table 
   Format: CPU_INDIRECT_ISR_ADDR_<N> = Service routine name for normally handled interrupt vector <N>
           CPU_DIRECT_ISR_ADDR_<N> = Service routine name for interrupt vector <N> handled as fast interrupt
*/
  
#define CPU_DIRECT_ISR_ADDR_0      _EntryPoint           /* Interrupt no. 0 (Used)   - ivINT_HW_RESET */
#define CPU_DIRECT_ISR_ADDR_1      _EntryPoint           /* Interrupt no. 1 (Used)   - ivINT_COP_RESET */
#define CPU_INDIRECT_ISR_ADDR_2    CpuUnhandledInterrupt /* Interrupt no. 2 (Unused) - ivINT_ILLEGAL_OP */
#define CPU_INDIRECT_ISR_ADDR_3    CpuUnhandledInterrupt /* Interrupt no. 3 (Unused) - ivINT_SWI3 */
#define CPU_INDIRECT_ISR_ADDR_4    CpuUnhandledInterrupt /* Interrupt no. 4 (Unused) - ivINT_OVERFLOW */
#define CPU_INDIRECT_ISR_ADDR_5    CpuUnhandledInterrupt /* Interrupt no. 5 (Unused) - ivINT_MISALIGNED */
#define CPU_INDIRECT_ISR_ADDR_6    CpuUnhandledInterrupt /* Interrupt no. 6 (Unused) - ivINT_STPCNT */
#define CPU_INDIRECT_ISR_ADDR_7    CpuUnhandledInterrupt /* Interrupt no. 7 (Unused) - ivINT_BKPT */
#define CPU_INDIRECT_ISR_ADDR_8    CpuUnhandledInterrupt /* Interrupt no. 8 (Unused) - ivINT_TRBUF */
#define CPU_INDIRECT_ISR_ADDR_9    CpuUnhandledInterrupt /* Interrupt no. 9 (Unused) - ivINT_TX_REG */
#define CPU_INDIRECT_ISR_ADDR_10   CpuUnhandledInterrupt /* Interrupt no. 10 (Unused) - ivINT_RX_REG */
#define CPU_INDIRECT_ISR_ADDR_11   CpuUnhandledInterrupt /* Interrupt no. 11 (Unused) - ivINT_BUS_ERR */
#define CPU_INDIRECT_ISR_ADDR_12   CpuUnhandledInterrupt /* Interrupt no. 12 (Unused) - ivINT_reserved12 */
#define CPU_INDIRECT_ISR_ADDR_13   CpuUnhandledInterrupt /* Interrupt no. 13 (Unused) - ivINT_reserved13 */
#define CPU_INDIRECT_ISR_ADDR_14   CpuUnhandledInterrupt /* Interrupt no. 14 (Unused) - ivINT_reserved14 */
#define CPU_INDIRECT_ISR_ADDR_15   CpuUnhandledInterrupt /* Interrupt no. 15 (Unused) - ivINT_SWI2 */
#define CPU_INDIRECT_ISR_ADDR_16   CpuUnhandledInterrupt /* Interrupt no. 16 (Unused) - ivINT_SWI1 */
#define CPU_INDIRECT_ISR_ADDR_17   CpuUnhandledInterrupt /* Interrupt no. 17 (Unused) - ivINT_SWI0 */
#define CPU_INDIRECT_ISR_ADDR_18   CpuUnhandledInterrupt /* Interrupt no. 18 (Unused) - ivINT_XBARA */
#define CPU_INDIRECT_ISR_ADDR_19   CpuUnhandledInterrupt /* Interrupt no. 19 (Unused) - ivINT_LVI1 */
#define CPU_INDIRECT_ISR_ADDR_20   CpuUnhandledInterrupt /* Interrupt no. 20 (Unused) - ivINT_OCCS */
#define CPU_INDIRECT_ISR_ADDR_21   CpuUnhandledInterrupt /* Interrupt no. 21 (Unused) - ivINT_reserved21 */
#define CPU_INDIRECT_ISR_ADDR_22   CpuUnhandledInterrupt /* Interrupt no. 22 (Unused) - ivINT_reserved22 */
#define CPU_INDIRECT_ISR_ADDR_23   CpuUnhandledInterrupt /* Interrupt no. 23 (Unused) - ivINT_reserved23 */
#define CPU_INDIRECT_ISR_ADDR_24   CpuUnhandledInterrupt /* Interrupt no. 24 (Unused) - ivINT_reserved24 */
#define CPU_INDIRECT_ISR_ADDR_25   CpuUnhandledInterrupt /* Interrupt no. 25 (Unused) - ivINT_TMRA_3 */
#define CPU_INDIRECT_ISR_ADDR_26   CpuUnhandledInterrupt /* Interrupt no. 26 (Unused) - ivINT_TMRA_2 */
#define CPU_INDIRECT_ISR_ADDR_27   CpuUnhandledInterrupt /* Interrupt no. 27 (Unused) - ivINT_TMRA_1 */
#define CPU_INDIRECT_ISR_ADDR_28   CpuUnhandledInterrupt /* Interrupt no. 28 (Unused) - ivINT_TMRA_0 */
#define CPU_INDIRECT_ISR_ADDR_29   CpuUnhandledInterrupt /* Interrupt no. 29 (Unused) - ivINT_ADC_CC1 */
#define CPU_INDIRECT_ISR_ADDR_30   CpuUnhandledInterrupt /* Interrupt no. 30 (Unused) - ivINT_ADC_CC0 */
#define CPU_INDIRECT_ISR_ADDR_31   CpuUnhandledInterrupt /* Interrupt no. 31 (Unused) - ivINT_ADC_ERR */
#define CPU_INDIRECT_ISR_ADDR_32   CpuUnhandledInterrupt /* Interrupt no. 32 (Unused) - ivINT_reserved32 */
#define CPU_INDIRECT_ISR_ADDR_33   CpuUnhandledInterrupt /* Interrupt no. 33 (Unused) - ivINT_DMA3 */
#define CPU_INDIRECT_ISR_ADDR_34   CpuUnhandledInterrupt /* Interrupt no. 34 (Unused) - ivINT_DMA2 */
#define CPU_INDIRECT_ISR_ADDR_35   CpuUnhandledInterrupt /* Interrupt no. 35 (Unused) - ivINT_DMA1 */
#define CPU_INDIRECT_ISR_ADDR_36   CpuUnhandledInterrupt /* Interrupt no. 36 (Unused) - ivINT_DMA0 */
#define CPU_INDIRECT_ISR_ADDR_37   CpuUnhandledInterrupt /* Interrupt no. 37 (Unused) - ivINT_reserved37 */
#define CPU_INDIRECT_ISR_ADDR_38   CpuUnhandledInterrupt /* Interrupt no. 38 (Unused) - ivINT_reserved38 */
#define CPU_INDIRECT_ISR_ADDR_39   CpuUnhandledInterrupt /* Interrupt no. 39 (Unused) - ivINT_ERROR */
#define CPU_INDIRECT_ISR_ADDR_40   CpuUnhandledInterrupt /* Interrupt no. 40 (Unused) - ivINT_TX_WARN */
#define CPU_INDIRECT_ISR_ADDR_41   CpuUnhandledInterrupt /* Interrupt no. 41 (Unused) - ivINT_RX_WARN */
#define CPU_INDIRECT_ISR_ADDR_42   CpuUnhandledInterrupt /* Interrupt no. 42 (Unused) - ivINT_WAKEUP */
#define CPU_INDIRECT_ISR_ADDR_43   CpuUnhandledInterrupt /* Interrupt no. 43 (Unused) - ivINT_reserved43 */
#define CPU_INDIRECT_ISR_ADDR_44   CpuUnhandledInterrupt /* Interrupt no. 44 (Unused) - ivINT_reserved44 */
#define CPU_INDIRECT_ISR_ADDR_45   CpuUnhandledInterrupt /* Interrupt no. 45 (Unused) - ivINT_reserved45 */
#define CPU_INDIRECT_ISR_ADDR_46   CpuUnhandledInterrupt /* Interrupt no. 46 (Unused) - ivINT_reserved46 */
#define CPU_INDIRECT_ISR_ADDR_47   CpuUnhandledInterrupt /* Interrupt no. 47 (Unused) - ivINT_QSCI1_RERR */
#define CPU_INDIRECT_ISR_ADDR_48   FMSTR1_Isr            /* Interrupt no. 48 (Used)   - ivINT_QSCI1_RCV */
#define CPU_INDIRECT_ISR_ADDR_49   CpuUnhandledInterrupt /* Interrupt no. 49 (Unused) - ivINT_QSCI1_TIDLE */
#define CPU_INDIRECT_ISR_ADDR_50   FMSTR1_Isr            /* Interrupt no. 50 (Used)   - ivINT_QSCI1_TDRE */
#define CPU_INDIRECT_ISR_ADDR_51   CpuUnhandledInterrupt /* Interrupt no. 51 (Unused) - ivINT_QSCI0_RERR */
#define CPU_INDIRECT_ISR_ADDR_52   CpuUnhandledInterrupt /* Interrupt no. 52 (Unused) - ivINT_QSCI0_RCV */
#define CPU_INDIRECT_ISR_ADDR_53   CpuUnhandledInterrupt /* Interrupt no. 53 (Unused) - ivINT_QSCI0_TIDLE */
#define CPU_INDIRECT_ISR_ADDR_54   CpuUnhandledInterrupt /* Interrupt no. 54 (Unused) - ivINT_QSCI0_TDRE */
#define CPU_INDIRECT_ISR_ADDR_55   CpuUnhandledInterrupt /* Interrupt no. 55 (Unused) - ivINT_reserved55 */
#define CPU_INDIRECT_ISR_ADDR_56   CpuUnhandledInterrupt /* Interrupt no. 56 (Unused) - ivINT_reserved56 */
#define CPU_INDIRECT_ISR_ADDR_57   CpuUnhandledInterrupt /* Interrupt no. 57 (Unused) - ivINT_QSPI1_XMIT */
#define CPU_INDIRECT_ISR_ADDR_58   CpuUnhandledInterrupt /* Interrupt no. 58 (Unused) - ivINT_QSPI1_RCV */
#define CPU_INDIRECT_ISR_ADDR_59   CpuUnhandledInterrupt /* Interrupt no. 59 (Unused) - ivINT_QSPI0_XMIT */
#define CPU_INDIRECT_ISR_ADDR_60   CpuUnhandledInterrupt /* Interrupt no. 60 (Unused) - ivINT_QSPI0_RCV */
#define CPU_INDIRECT_ISR_ADDR_61   CpuUnhandledInterrupt /* Interrupt no. 61 (Unused) - ivINT_reserved61 */
#define CPU_INDIRECT_ISR_ADDR_62   CpuUnhandledInterrupt /* Interrupt no. 62 (Unused) - ivINT_IIC0 */
#define CPU_INDIRECT_ISR_ADDR_63   CpuUnhandledInterrupt /* Interrupt no. 63 (Unused) - ivINT_reserved63 */
#define CPU_INDIRECT_ISR_ADDR_64   CpuUnhandledInterrupt /* Interrupt no. 64 (Unused) - ivINT_reserved64 */
#define CPU_INDIRECT_ISR_ADDR_65   CpuUnhandledInterrupt /* Interrupt no. 65 (Unused) - ivINT_reserved65 */
#define CPU_INDIRECT_ISR_ADDR_66   CpuUnhandledInterrupt /* Interrupt no. 66 (Unused) - ivINT_reserved66 */
#define CPU_INDIRECT_ISR_ADDR_67   CpuUnhandledInterrupt /* Interrupt no. 67 (Unused) - ivINT_reserved67 */
#define CPU_INDIRECT_ISR_ADDR_68   CpuUnhandledInterrupt /* Interrupt no. 68 (Unused) - ivINT_reserved68 */
#define CPU_INDIRECT_ISR_ADDR_69   CpuUnhandledInterrupt /* Interrupt no. 69 (Unused) - ivINT_reserved69 */
#define CPU_INDIRECT_ISR_ADDR_70   CpuUnhandledInterrupt /* Interrupt no. 70 (Unused) - ivINT_reserved70 */
#define CPU_INDIRECT_ISR_ADDR_71   CpuUnhandledInterrupt /* Interrupt no. 71 (Unused) - ivINT_reserved71 */
#define CPU_INDIRECT_ISR_ADDR_72   CpuUnhandledInterrupt /* Interrupt no. 72 (Unused) - ivINT_reserved72 */
#define CPU_INDIRECT_ISR_ADDR_73   CpuUnhandledInterrupt /* Interrupt no. 73 (Unused) - ivINT_reserved73 */
#define CPU_INDIRECT_ISR_ADDR_74   CpuUnhandledInterrupt /* Interrupt no. 74 (Unused) - ivINT_reserved74 */
#define CPU_INDIRECT_ISR_ADDR_75   CpuUnhandledInterrupt /* Interrupt no. 75 (Unused) - ivINT_reserved75 */
#define CPU_INDIRECT_ISR_ADDR_76   CpuUnhandledInterrupt /* Interrupt no. 76 (Unused) - ivINT_reserved76 */
#define CPU_INDIRECT_ISR_ADDR_77   CpuUnhandledInterrupt /* Interrupt no. 77 (Unused) - ivINT_eFlexPWMA_FAULT */
#define CPU_INDIRECT_ISR_ADDR_78   CpuUnhandledInterrupt /* Interrupt no. 78 (Unused) - ivINT_eFlexPWMA_RERR */
#define CPU_INDIRECT_ISR_ADDR_79   CpuUnhandledInterrupt /* Interrupt no. 79 (Unused) - ivINT_eFlexPWMA_RELOAD3 */
#define CPU_INDIRECT_ISR_ADDR_80   CpuUnhandledInterrupt /* Interrupt no. 80 (Unused) - ivINT_eFlexPWMA_CAP */
#define CPU_INDIRECT_ISR_ADDR_81   CpuUnhandledInterrupt /* Interrupt no. 81 (Unused) - ivINT_eFlexPWMA_CMP3 */
#define CPU_INDIRECT_ISR_ADDR_82   CpuUnhandledInterrupt /* Interrupt no. 82 (Unused) - ivINT_eFlexPWMA_RELOAD2 */
#define CPU_INDIRECT_ISR_ADDR_83   CpuUnhandledInterrupt /* Interrupt no. 83 (Unused) - ivINT_eFlexPWMA_CMP2 */
#define CPU_INDIRECT_ISR_ADDR_84   CpuUnhandledInterrupt /* Interrupt no. 84 (Unused) - ivINT_eFlexPWMA_RELOAD1 */
#define CPU_INDIRECT_ISR_ADDR_85   CpuUnhandledInterrupt /* Interrupt no. 85 (Unused) - ivINT_eFlexPWMA_CMP1 */
#define CPU_INDIRECT_ISR_ADDR_86   CpuUnhandledInterrupt /* Interrupt no. 86 (Unused) - ivINT_eFlexPWMA_RELOAD0 */
#define CPU_INDIRECT_ISR_ADDR_87   PWM_Trigger_ISR       /* Interrupt no. 87 (Used)   - ivINT_eFlexPWMA_CMP0 */
#define CPU_INDIRECT_ISR_ADDR_88   CpuUnhandledInterrupt /* Interrupt no. 88 (Unused) - ivINT_FTFA_RDCOL */
#define CPU_INDIRECT_ISR_ADDR_89   CpuUnhandledInterrupt /* Interrupt no. 89 (Unused) - ivINT_FTFA_CC */
#define CPU_INDIRECT_ISR_ADDR_90   CpuUnhandledInterrupt /* Interrupt no. 90 (Unused) - ivINT_CMPD */
#define CPU_INDIRECT_ISR_ADDR_91   CpuUnhandledInterrupt /* Interrupt no. 91 (Unused) - ivINT_CMPC */
#define CPU_INDIRECT_ISR_ADDR_92   CpuUnhandledInterrupt /* Interrupt no. 92 (Unused) - ivINT_CMPB */
#define CPU_INDIRECT_ISR_ADDR_93   CpuUnhandledInterrupt /* Interrupt no. 93 (Unused) - ivINT_CMPA */
#define CPU_INDIRECT_ISR_ADDR_94   CpuUnhandledInterrupt /* Interrupt no. 94 (Unused) - ivINT_PIT1_ROLLOVR */
#define CPU_INDIRECT_ISR_ADDR_95   PIT0_ISR              /* Interrupt no. 95 (Used)   - ivINT_PIT0_ROLLOVR */
#define CPU_INDIRECT_ISR_ADDR_96   CpuUnhandledInterrupt /* Interrupt no. 96 (Unused) - ivINT_reserved96 */
#define CPU_INDIRECT_ISR_ADDR_97   CpuUnhandledInterrupt /* Interrupt no. 97 (Unused) - ivINT_reserved97 */
#define CPU_INDIRECT_ISR_ADDR_98   CpuUnhandledInterrupt /* Interrupt no. 98 (Unused) - ivINT_reserved98 */
#define CPU_INDIRECT_ISR_ADDR_99   CpuUnhandledInterrupt /* Interrupt no. 99 (Unused) - ivINT_reserved99 */
#define CPU_INDIRECT_ISR_ADDR_100  CpuUnhandledInterrupt /* Interrupt no. 100 (Unused) - ivINT_reserved100 */
#define CPU_INDIRECT_ISR_ADDR_101  CpuUnhandledInterrupt /* Interrupt no. 101 (Unused) - ivINT_reserved101 */
#define CPU_INDIRECT_ISR_ADDR_102  CpuUnhandledInterrupt /* Interrupt no. 102 (Unused) - ivINT_GPIOF */
#define CPU_INDIRECT_ISR_ADDR_103  CpuUnhandledInterrupt /* Interrupt no. 103 (Unused) - ivINT_GPIOE */
#define CPU_INDIRECT_ISR_ADDR_104  CpuUnhandledInterrupt /* Interrupt no. 104 (Unused) - ivINT_GPIOD */
#define CPU_INDIRECT_ISR_ADDR_105  CpuUnhandledInterrupt /* Interrupt no. 105 (Unused) - ivINT_GPIOC */
#define CPU_INDIRECT_ISR_ADDR_106  CpuUnhandledInterrupt /* Interrupt no. 106 (Unused) - ivINT_GPIOB */
#define CPU_INDIRECT_ISR_ADDR_107  CpuUnhandledInterrupt /* Interrupt no. 107 (Unused) - ivINT_GPIOA */
#define CPU_INDIRECT_ISR_ADDR_108  CpuUnhandledInterrupt /* Interrupt no. 108 (Unused) - ivINT_COP_INT */
#define CPU_INDIRECT_ISR_ADDR_109  CpuUnhandledInterrupt /* Interrupt no. 109 (Unused) - ivINT_EWM_INT */
#define CPU_INDIRECT_ISR_ADDR_110  CpuUnhandledInterrupt /* Interrupt no. 110 (Unused) - ivINT_SWILP */


#ifdef __cplusplus
}
#endif

/* END Vectors_Config.h */

#endif /* __Vectors_Config_H */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file is a part of Processor Expert static initialization
**     library for the Freescale 56800 series of microcontrollers.
**
** ###################################################################
*/
