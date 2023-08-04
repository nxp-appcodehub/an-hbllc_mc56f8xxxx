/*
* Copyright 2019 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * ISR.h
 *
 *  Created on: Feb 21, 2019
 *      Author: nxa22573
 */

#ifndef ISR_H_
#define ISR_H_

/*	
 * 	Put ISR prototype declaration here
 * */
extern void PWMA0_trig0_isr(void);
extern void PIT0_isr(void);
/*
 *  Vector ISR redefinition. Change the desired vector to your own ISR. 
 * */
#define  ivINT_ILLEGAL_OP       Cpu_Interrupt         /* Interrupt no. 2       */
#define  ivINT_SWI3             Cpu_Interrupt         /* Interrupt no. 3       */
#define  ivINT_OVERFLOW         Cpu_Interrupt         /* Interrupt no. 4       */
#define  ivINT_MISALIGNED       Cpu_Interrupt         /* Interrupt no. 5       */
#define  ivINT_STPCNT           Cpu_Interrupt         /* Interrupt no. 6       */
#define  ivINT_BKPT             Cpu_Interrupt         /* Interrupt no. 7       */
#define  ivINT_TRBUF            Cpu_Interrupt         /* Interrupt no. 8       */
#define  ivINT_TX_REG           Cpu_Interrupt         /* Interrupt no. 9       */
#define  ivINT_RX_REG           Cpu_Interrupt         /* Interrupt no. 10      */
#define  ivINT_BUS_ERR          Cpu_Interrupt         /* Interrupt no. 11      */
#define  ivINT_PLAT_3           Cpu_Interrupt         /* Interrupt no. 12      */
#define  ivINT_PLAT_2           Cpu_Interrupt         /* Interrupt no. 13      */
#define  ivINT_PLAT_1           Cpu_Interrupt         /* Interrupt no. 14      */
#define  ivINT_SWI2             Cpu_Interrupt         /* Interrupt no. 15      */
#define  ivINT_SWI1             Cpu_Interrupt         /* Interrupt no. 16      */
#define  ivINT_SWI0             Cpu_Interrupt         /* Interrupt no. 17      */
#define  ivINT_XBARA            Cpu_Interrupt         /* Interrupt no. 18      */
#define  ivINT_LVI1             Cpu_Interrupt         /* Interrupt no. 19      */
#define  ivINT_OCCS             Cpu_Interrupt         /* Interrupt no. 20      */
#define  ivINT_TMRB_3           Cpu_Interrupt         /* Interrupt no. 21      */
#define  ivINT_TMRB_2           Cpu_Interrupt         /* Interrupt no. 22      */
#define  ivINT_TMRB_1           Cpu_Interrupt         /* Interrupt no. 23      */
#define  ivINT_TMRB_0           Cpu_Interrupt         /* Interrupt no. 24      */
#define  ivINT_TMRA_3           Cpu_Interrupt         /* Interrupt no. 25      */
#define  ivINT_TMRA_2           Cpu_Interrupt         /* Interrupt no. 26      */
#define  ivINT_TMRA_1           Cpu_Interrupt         /* Interrupt no. 27      */
#define  ivINT_TMRA_0           Cpu_Interrupt         /* Interrupt no. 28      */
#define  ivINT_ADC12_CC1        Cpu_Interrupt         /* Interrupt no. 29      */
#define  ivINT_ADC12_CC0        Cpu_Interrupt         /* Interrupt no. 30      */
#define  ivINT_ADC12_ERR        Cpu_Interrupt         /* Interrupt no. 31      */
#define  ivINT_DMA_ERR          Cpu_Interrupt         /* Interrupt no. 32      */
#define  ivINT_DMA3             Cpu_Interrupt         /* Interrupt no. 33      */
#define  ivINT_DMA2             Cpu_Interrupt         /* Interrupt no. 34      */
#define  ivINT_DMA1             Cpu_Interrupt         /* Interrupt no. 35      */
#define  ivINT_DMA0             Cpu_Interrupt         /* Interrupt no. 36      */
#define  ivINT_CAN_MB_OR        Cpu_Interrupt         /* Interrupt no. 37      */
#define  ivINT_CAN_BUS_OFF      Cpu_Interrupt         /* Interrupt no. 38      */
#define  ivINT_CAN_ERROR        Cpu_Interrupt         /* Interrupt no. 39      */
#define  ivINT_CAN_TX_WARN      Cpu_Interrupt         /* Interrupt no. 40      */
#define  ivINT_CAN_RX_WARN      Cpu_Interrupt         /* Interrupt no. 41      */
#define  ivINT_CAN_WAKEUP       Cpu_Interrupt         /* Interrupt no. 42      */
#define  ivINT_QSCI2_RERR       Cpu_Interrupt         /* Interrupt no. 43      */
#define  ivINT_QSCI2_RCV        Cpu_Interrupt         /* Interrupt no. 44      */
#define  ivINT_QSCI2_TIDLE      Cpu_Interrupt         /* Interrupt no. 45      */
#define  ivINT_QSCI2_TDRE       Cpu_Interrupt         /* Interrupt no. 46      */
#define  ivINT_QSCI1_RERR       Cpu_Interrupt         /* Interrupt no. 47      */
#define  ivINT_QSCI1_RCV        Cpu_Interrupt         /* Interrupt no. 48      */
#define  ivINT_QSCI1_TIDLE      Cpu_Interrupt         /* Interrupt no. 49      */
#define  ivINT_QSCI1_TDRE       Cpu_Interrupt         /* Interrupt no. 50      */
#define  ivINT_QSCI0_RERR       Cpu_Interrupt         /* Interrupt no. 51      */
#define  ivINT_QSCI0_RCV        Cpu_Interrupt         /* Interrupt no. 52      */
#define  ivINT_QSCI0_TIDLE      Cpu_Interrupt         /* Interrupt no. 53      */
#define  ivINT_QSCI0_TDRE       Cpu_Interrupt         /* Interrupt no. 54      */
#define  ivINT_QSPI1_XMIT       Cpu_Interrupt         /* Interrupt no. 57      */
#define  ivINT_QSPI1_RCV        Cpu_Interrupt         /* Interrupt no. 58      */
#define  ivINT_QSPI0_XMIT       Cpu_Interrupt         /* Interrupt no. 59      */
#define  ivINT_QSPI0_RCV        Cpu_Interrupt         /* Interrupt no. 60      */
#define  ivINT_IIC1             Cpu_Interrupt         /* Interrupt no. 61      */
#define  ivINT_IIC0             Cpu_Interrupt         /* Interrupt no. 62      */
#define  ivINT_eFlexPWMB_FAULT  	Cpu_Interrupt     /* Interrupt no. 66      */
#define  ivINT_eFlexPWMB_RERR       Cpu_Interrupt     /* Interrupt no. 67      */
#define  ivINT_eFlexPWMB_CAP        Cpu_Interrupt     /* Interrupt no. 68      */
#define  ivINT_eFlexPWMB_RELOAD3    Cpu_Interrupt     /* Interrupt no. 69      */
#define  ivINT_eFlexPWMB_CMP3       Cpu_Interrupt     /* Interrupt no. 70      */
#define  ivINT_eFlexPWMB_RELOAD2    Cpu_Interrupt     /* Interrupt no. 71      */
#define  ivINT_eFlexPWMB_CMP2       Cpu_Interrupt     /* Interrupt no. 72      */
#define  ivINT_eFlexPWMB_RELOAD1    Cpu_Interrupt     /* Interrupt no. 73      */
#define  ivINT_eFlexPWMB_CMP1       Cpu_Interrupt     /* Interrupt no. 74      */
#define  ivINT_eFlexPWMB_RELOAD0    Cpu_Interrupt     /* Interrupt no. 75      */
#define  ivINT_eFlexPWMB_CMP0       Cpu_Interrupt     /* Interrupt no. 76      */
#define  ivINT_eFlexPWMA_FAULT      Cpu_Interrupt     /* Interrupt no. 77      */
#define  ivINT_eFlexPWMA_RERR       Cpu_Interrupt     /* Interrupt no. 78      */
#define  ivINT_eFlexPWMA_CAP        Cpu_Interrupt     /* Interrupt no. 79      */
#define  ivINT_eFlexPWMA_RELOAD3    Cpu_Interrupt     /* Interrupt no. 80      */
#define  ivINT_eFlexPWMA_CMP3       Cpu_Interrupt     /* Interrupt no. 81      */
#define  ivINT_eFlexPWMA_RELOAD2    Cpu_Interrupt     /* Interrupt no. 82      */
#define  ivINT_eFlexPWMA_CMP2       Cpu_Interrupt     /* Interrupt no. 83      */
#define  ivINT_eFlexPWMA_RELOAD1    Cpu_Interrupt     /* Interrupt no. 84      */
#define  ivINT_eFlexPWMA_CMP1       Cpu_Interrupt     /* Interrupt no. 85      */
#define  ivINT_eFlexPWMA_RELOAD0    Cpu_Interrupt     /* Interrupt no. 86      */
#define  ivINT_eFlexPWMA_CMP0       PWMA0_trig0_isr     /* Interrupt no. 87      */
#define  ivINT_FTFE_RDCOL           Cpu_Interrupt     /* Interrupt no. 88      */
#define  ivINT_FTFE_CC              Cpu_Interrupt     /* Interrupt no. 89      */
#define  ivINT_CMPD                 Cpu_Interrupt     /* Interrupt no. 90      */
#define  ivINT_CMPC                 Cpu_Interrupt     /* Interrupt no. 91      */
#define  ivINT_CMPB                 Cpu_Interrupt     /* Interrupt no. 92      */
#define  ivINT_CMPA                 Cpu_Interrupt     /* Interrupt no. 93      */
#define  ivINT_PIT1_ROLLOVR         Cpu_Interrupt     /* Interrupt no. 94      */
#define  ivINT_PIT0_ROLLOVR         PIT0_isr     /* Interrupt no. 95      */
#define  ivINT_FTFE_DFD             Cpu_Interrupt     /* Interrupt no. 96      */
#define  ivINT_GPIOG                Cpu_Interrupt     /* Interrupt no. 101     */
#define  ivINT_GPIOF                Cpu_Interrupt     /* Interrupt no. 102     */
#define  ivINT_GPIOE                Cpu_Interrupt     /* Interrupt no. 103     */
#define  ivINT_GPIOD                Cpu_Interrupt     /* Interrupt no. 104     */
#define  ivINT_GPIOC                Cpu_Interrupt     /* Interrupt no. 105     */
#define  ivINT_GPIOB                Cpu_Interrupt     /* Interrupt no. 106     */
#define  ivINT_GPIOA                Cpu_Interrupt     /* Interrupt no. 107     */
#define  ivINT_COP_INT              Cpu_Interrupt     /* Interrupt no. 108     */
#define  ivINT_EWM_INT              Cpu_Interrupt     /* Interrupt no. 109     */
#define  ivINT_SWILP                Cpu_Interrupt     /* Interrupt no. 110     */
#define  ivINT_USB                  Cpu_Interrupt     /* Interrupt no. 111     */

#endif /* ISR_H_ */
