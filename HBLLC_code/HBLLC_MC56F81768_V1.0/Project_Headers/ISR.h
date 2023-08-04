/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef ISR_H_
#define ISR_H_

/*
 *  Put ISR declaration here. 
 * */
void PWMA0_trig0_isr();
void PIT0_isr();

/*
 *  Vector ISR redefinition. Change the desired vector to your own ISR. 
 * */
#define  ivINT_ILLEGAL_OP               Cpu_Interrupt         /* Interrupt no. 2       */
#define  ivINT_SWI3                     Cpu_Interrupt         /* Interrupt no. 3       */
#define  ivINT_OVERFLOW                 Cpu_Interrupt         /* Interrupt no. 4       */
#define  ivINT_MISALIGNED               Cpu_Interrupt         /* Interrupt no. 5       */
#define  ivINT_STPCNT                   Cpu_Interrupt         /* Interrupt no. 6       */
#define  ivINT_BKPT                     Cpu_Interrupt         /* Interrupt no. 7       */
#define  ivINT_TRBUF                    Cpu_Interrupt         /* Interrupt no. 8       */
#define  ivINT_TX_REG                   Cpu_Interrupt         /* Interrupt no. 9       */
#define  ivINT_RX_REG                   Cpu_Interrupt         /* Interrupt no. 10      */
#define  ivINT_BUS_ERR                  Cpu_Interrupt         /* Interrupt no. 11      */
#define  ivINT_PLAT_3                   Cpu_Interrupt         /* Interrupt no. 12      */
#define  ivINT_PLAT_2                   Cpu_Interrupt         /* Interrupt no. 13      */
#define  ivINT_PLAT_1                   Cpu_Interrupt         /* Interrupt no. 14      */
#define  ivINT_SWI2                     Cpu_Interrupt         /* Interrupt no. 15      */
#define  ivINT_SWI1                     Cpu_Interrupt         /* Interrupt no. 16      */
#define  ivINT_SWI0                     Cpu_Interrupt         /* Interrupt no. 17      */
#define  ivINT_XBARA                    Cpu_Interrupt         /* Interrupt no. 18      */
#define  ivINT_LVI1                     Cpu_Interrupt         /* Interrupt no. 19      */
#define  ivINT_OCCS                     Cpu_Interrupt         /* Interrupt no. 20      */
#define  ivINT_TMRA_3                   Cpu_Interrupt         /* Interrupt no. 25      */
#define  ivINT_TMRA_2                   Cpu_Interrupt         /* Interrupt no. 26      */
#define  ivINT_TMRA_1                   Cpu_Interrupt         /* Interrupt no. 27      */
#define  ivINT_TMRA_0                   Cpu_Interrupt         /* Interrupt no. 28      */
#define  ivINT_ADC_CC1                  Cpu_Interrupt         /* Interrupt no. 29      */
#define  ivINT_ADC_CC0                  Cpu_Interrupt         /* Interrupt no. 30      */
#define  ivINT_ADC_ERR                  Cpu_Interrupt         /* Interrupt no. 31      */
#define  ivINT_DMA_ERR                  Cpu_Interrupt         /* Interrupt no. 32      */
#define  ivINT_DMACH3                   Cpu_Interrupt         /* Interrupt no. 33      */
#define  ivINT_DMACH2                   Cpu_Interrupt         /* Interrupt no. 34      */
#define  ivINT_DMACH1                   Cpu_Interrupt         /* Interrupt no. 35      */
#define  ivINT_DMACH0                   Cpu_Interrupt         /* Interrupt no. 36      */
#define  ivINT_QSCI1_RERR               Cpu_Interrupt         /* Interrupt no. 47      */
#define  ivINT_QSCI1_RDFD_OR_RIEF       Cpu_Interrupt         /* Interrupt no. 48      */
#define  ivINT_QSCI1_TIDLE_RIDLE        Cpu_Interrupt         /* Interrupt no. 49      */
#define  ivINT_QSCI1_TDRE               Cpu_Interrupt         /* Interrupt no. 50      */
#define  ivINT_QSCI0_RERR               Cpu_Interrupt         /* Interrupt no. 51      */
#define  ivINT_QSCI0_RDFD_OR_RIEF       Cpu_Interrupt         /* Interrupt no. 52      */
#define  ivINT_QSCI0_TIDLE_RIDLE        Cpu_Interrupt         /* Interrupt no. 53      */
#define  ivINT_QSCI0_TDRE               Cpu_Interrupt         /* Interrupt no. 54      */
#define  ivINT_QSPI0_XMIT               Cpu_Interrupt         /* Interrupt no. 59      */
#define  ivINT_QSPI0_RCV                Cpu_Interrupt         /* Interrupt no. 60      */
#define  ivINT_LPI2C1                   Cpu_Interrupt         /* Interrupt no. 61      */
#define  ivINT_LPI2C0                   Cpu_Interrupt         /* Interrupt no. 62      */
#define  ivINT_eFlexPWMA_FAULT          Cpu_Interrupt         /* Interrupt no. 77      */
#define  ivINT_eFlexPWMA_RERR           Cpu_Interrupt         /* Interrupt no. 78      */
#define  ivINT_eFlexPWMA_CAP            Cpu_Interrupt         /* Interrupt no. 79      */
#define  ivINT_eFlexPWMA_RELOAD3        Cpu_Interrupt         /* Interrupt no. 80      */
#define  ivINT_eFlexPWMA_CMP3           Cpu_Interrupt         /* Interrupt no. 81      */
#define  ivINT_eFlexPWMA_RELOAD2        Cpu_Interrupt         /* Interrupt no. 82      */
#define  ivINT_eFlexPWMA_CMP2           Cpu_Interrupt         /* Interrupt no. 83      */
#define  ivINT_eFlexPWMA_RELOAD1        Cpu_Interrupt         /* Interrupt no. 84      */
#define  ivINT_eFlexPWMA_CMP1           Cpu_Interrupt         /* Interrupt no. 85      */
#define  ivINT_eFlexPWMA_RELOAD0        Cpu_Interrupt         /* Interrupt no. 86      */
#define  ivINT_eFlexPWMA_CMP0           PWMA0_trig0_isr         /* Interrupt no. 87      */
#define  ivINT_FTFA_RDCOL               Cpu_Interrupt         /* Interrupt no. 88      */
#define  ivINT_FTFA_CC                  Cpu_Interrupt         /* Interrupt no. 89      */
#define  ivINT_CMPD                     Cpu_Interrupt         /* Interrupt no. 90      */
#define  ivINT_CMPC                     Cpu_Interrupt         /* Interrupt no. 91      */
#define  ivINT_CMPB_OPAMPB              Cpu_Interrupt         /* Interrupt no. 92      */
#define  ivINT_CMPA_OPAMPA              Cpu_Interrupt         /* Interrupt no. 93      */
#define  ivINT_PIT1_ROLLOVR             Cpu_Interrupt         /* Interrupt no. 94      */
#define  ivINT_PIT0_ROLLOVR             PIT0_isr         /* Interrupt no. 95      */
#define  ivINT_QDC_INDEX_RO_RU_CMP      Cpu_Interrupt         /* Interrupt no. 99      */
#define  ivINT_QDC_HOME_WDG_SAB         Cpu_Interrupt         /* Interrupt no. 100     */
#define  ivINT_GPIOF                    Cpu_Interrupt         /* Interrupt no. 102     */
#define  ivINT_GPIOE                    Cpu_Interrupt         /* Interrupt no. 103     */
#define  ivINT_GPIOD                    Cpu_Interrupt         /* Interrupt no. 104     */
#define  ivINT_GPIOC                    Cpu_Interrupt         /* Interrupt no. 105     */
#define  ivINT_GPIOB                    Cpu_Interrupt         /* Interrupt no. 106     */
#define  ivINT_GPIOA                    Cpu_Interrupt         /* Interrupt no. 107     */
#define  ivINT_COP_INT                  Cpu_Interrupt         /* Interrupt no. 108     */
#define  ivINT_EWM_INT                  Cpu_Interrupt         /* Interrupt no. 109     */
#define  ivINT_SWILP                    Cpu_Interrupt         /* Interrupt no. 110     */
#define  ivINT_ECDSA_TRNG               Cpu_Interrupt         /* Interrupt no. 112     */

#endif /* ISR_H_ */
