/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : XBARA_Config.h
**     Project     : HBLLC_MC56F82748_V1.0
**     Processor   : MC56F82748VLH
**     Component   : Init_XBAR
**     Version     : Component 01.000, Driver 01.00, CPU db: 3.50.001
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2022-01-27, 11:59, # CodeGen: 0
**     Abstract    :
**          This file implements the XBAR (XBARA) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
**     Settings    :
**          Component name                                 : XBARA
**          Device                                         : XBARA
**          Settings                                       : 
**            Output DMA_REQ0                              : VSS
**            Output DMA_REQ1                              : VSS
**            Output DMA_REQ2                              : VSS
**            Output DMA_REQ3                              : VSS
**            Output XB_OUT4                               : PWM0_TRG0
**            Output XB_OUT5                               : VSS
**            Output XB_OUT6                               : AND_OR_INVERT_3
**            Output XB_OUT7                               : AND_OR_INVERT_2
**            Output XB_OUT8                               : AND_OR_INVERT_0
**            Output XB_OUT9                               : AND_OR_INVERT_1
**            Output XB_OUT10                              : VSS
**            Output XB_OUT11                              : VSS
**            Output ADCA_TRIG                             : PWM0_TRG0
**            Output ADCB_TRIG                             : VSS
**            Output DACB_12B_SYNC                         : VSS
**            Output DACA_12B_SYNC                         : VSS
**            Output CMPA                                  : VSS
**            Output CMPB                                  : VSS
**            Output CMPC                                  : VSS
**            Output CMPD                                  : VSS
**            Output PWM0_EXTA                             : VSS
**            Output PWM1_EXTA                             : VSS
**            Output PWM2_EXTA                             : VSS
**            Output PWM3_EXTA                             : VSS
**            Output PWM0_EXT_SYNC                         : VSS
**            Output PWM1_EXT_SYNC                         : VSS
**            Output PWM2_EXT_SYNC                         : VSS
**            Output PWM3_EXT_SYNC                         : VSS
**            Output PWM_EXT_CLK                           : VSS
**            Output PWM_FAULT0                            : CMPA_OUT
**            Output PWM_FAULT1                            : VSS
**            Output PWM_FAULT2                            : VSS
**            Output PWM_FAULT3                            : VSS
**            Output PWM_FORCE                             : VSS
**            Output TA0_IN                                : VSS
**            Output TA1_IN                                : VSS
**            Output TA2_IN                                : VSS
**            Output TA3_IN                                : VSS
**            Output SCI0_RXD                              : VSS
**            Output SCI1_RXD                              : VSS
**            Output EWM_IN                                : VSS
**            Output DMA_REQ0 interrupt assert             : Neither edge
**            Output DMA_REQ1 interrupt assert             : Neither edge
**            Output DMA_REQ2 interrupt assert             : Neither edge
**            Output DMA_REQ3 interrupt assert             : Neither edge
**          Interrupts                                     : 
**            Output interrupt                             : 
**              Interrupt                                  : INT_XBARA
**              Interrupt priority                         : disabled
**              ISR Name                                   : 
**              Output 0 interrupt enable                  : no
**              Output 0 DMA enable                        : no
**              Output 1 interrupt enable                  : no
**              Output 1 DMA enable                        : no
**              Output 2 interrupt enable                  : no
**              Output 2 DMA enable                        : no
**              Output 3 interrupt enable                  : no
**              Output 3 DMA enable                        : no
**          Initialization                                 : 
**            Call Init method                             : yes
**            Utilize after reset values                   : default
**     Contents    :
**         Init - void XBARA_Init(void);
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
** @file XBARA_Config.h                                                  
** @version 01.00
** @brief
**          This file implements the XBAR (XBARA) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
*/         
/*!
**  @addtogroup XBARA_Config_module XBARA_Config module documentation
**  @{
*/         

#ifndef XBARA_Config_H_
#define XBARA_Config_H_

/* MODULE XBARA. */

/* XBARA_SEL0: SEL1=0,SEL0=0 */
#define XBARA_SEL0_VALUE     0x00
#define XBARA_SEL0_MASK      0x1F1F
/* XBARA_SEL1: SEL3=0,SEL2=0 */
#define XBARA_SEL1_VALUE     0x00
#define XBARA_SEL1_MASK      0x1F1F
/* XBARA_SEL2: SEL5=0,SEL4=0x12 */
#define XBARA_SEL2_VALUE     0x12
#define XBARA_SEL2_MASK      0x1F1F
/* XBARA_SEL3: SEL7=0x1E,SEL6=0x1F */
#define XBARA_SEL3_VALUE     0x1E1F
#define XBARA_SEL3_MASK      0x1F1F
/* XBARA_SEL4: SEL9=0x1D,SEL8=0x1C */
#define XBARA_SEL4_VALUE     0x1D1C
#define XBARA_SEL4_MASK      0x1F1F
/* XBARA_SEL5: SEL11=0,SEL10=0 */
#define XBARA_SEL5_VALUE     0x00
#define XBARA_SEL5_MASK      0x1F1F
/* XBARA_SEL6: SEL13=0,SEL12=0x12 */
#define XBARA_SEL6_VALUE     0x12
#define XBARA_SEL6_MASK      0x1F1F
/* XBARA_SEL7: SEL15=0,SEL14=0 */
#define XBARA_SEL7_VALUE     0x00
#define XBARA_SEL7_MASK      0x1F1F
/* XBARA_SEL8: SEL17=0,SEL16=0 */
#define XBARA_SEL8_VALUE     0x00
#define XBARA_SEL8_MASK      0x1F1F
/* XBARA_SEL9: SEL19=0,SEL18=0 */
#define XBARA_SEL9_VALUE     0x00
#define XBARA_SEL9_MASK      0x1F1F
/* XBARA_SEL10: SEL21=0,SEL20=0 */
#define XBARA_SEL10_VALUE    0x00
#define XBARA_SEL10_MASK     0x1F1F
/* XBARA_SEL11: SEL23=0,SEL22=0 */
#define XBARA_SEL11_VALUE    0x00
#define XBARA_SEL11_MASK     0x1F1F
/* XBARA_SEL12: SEL25=0,SEL24=0 */
#define XBARA_SEL12_VALUE    0x00
#define XBARA_SEL12_MASK     0x1F1F
/* XBARA_SEL13: SEL27=0,SEL26=0 */
#define XBARA_SEL13_VALUE    0x00
#define XBARA_SEL13_MASK     0x1F1F
/* XBARA_SEL14: SEL29=0x0A,SEL28=0 */
#define XBARA_SEL14_VALUE    0x0A00
#define XBARA_SEL14_MASK     0x1F1F
/* XBARA_SEL15: SEL31=0,SEL30=0 */
#define XBARA_SEL15_VALUE    0x00
#define XBARA_SEL15_MASK     0x1F1F
/* XBARA_SEL16: SEL33=0,SEL32=0 */
#define XBARA_SEL16_VALUE    0x00
#define XBARA_SEL16_MASK     0x1F1F
/* XBARA_SEL17: SEL35=0,SEL34=0 */
#define XBARA_SEL17_VALUE    0x00
#define XBARA_SEL17_MASK     0x1F1F
/* XBARA_SEL18: SEL37=0,SEL36=0 */
#define XBARA_SEL18_VALUE    0x00
#define XBARA_SEL18_MASK     0x1F1F
/* XBARA_SEL19: SEL39=0,SEL38=0 */
#define XBARA_SEL19_VALUE    0x00
#define XBARA_SEL19_MASK     0x1F1F
/* XBARA_SEL20: SEL40=0 */
#define XBARA_SEL20_VALUE    0x00
#define XBARA_SEL20_MASK     0x1F
/* XBARA_CTRL0: STS1=1,EDGE1=0,IEN1=0,DEN1=0,STS0=1,EDGE0=0,IEN0=0,DEN0=0 */
#define XBARA_CTRL0_VALUE    0x1010
#define XBARA_CTRL0_MASK     0x1F1F
/* XBARA_CTRL1: STS3=1,EDGE3=0,IEN3=0,DEN3=0,STS2=1,EDGE2=0,IEN2=0,DEN2=0 */
#define XBARA_CTRL1_VALUE    0x1010
#define XBARA_CTRL1_MASK     0x1F1F

#define XBARA_AUTOINIT

/* END XBARA. */
#endif /* #ifndef __XBARA_Config_H_ */
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
