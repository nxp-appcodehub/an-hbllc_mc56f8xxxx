/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : XBARB_Config.h
**     Project     : HBLLC_MC56F82748_V1.0
**     Processor   : MC56F82748VLH
**     Component   : Init_XBAR
**     Version     : Component 01.000, Driver 01.00, CPU db: 3.50.001
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2022-01-27, 11:59, # CodeGen: 0
**     Abstract    :
**          This file implements the XBAR (XBARB) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
**     Settings    :
**          Component name                                 : XBARB
**          Device                                         : XBARB
**          Settings                                       : 
**            Output AOI_Input0A                           : PWM2_TRG1
**            Output AOI_Input0B                           : CMPD_OUT
**            Output AOI_Input0C                           : PWM1_TRG1
**            Output AOI_Input0D                           : CMPA_OUT
**            Output AOI_Input1A                           : PWM2_TRG0
**            Output AOI_Input1B                           : CMPB_OUT
**            Output AOI_Input1C                           : PWM1_TRG0
**            Output AOI_Input1D                           : CMPA_OUT
**            Output AOI_Input2A                           : PWM2_TRG0
**            Output AOI_Input2B                           : SCI0_TXD
**            Output AOI_Input2C                           : CMPA_OUT
**            Output AOI_Input2D                           : SCI0_TXD
**            Output AOI_Input3A                           : PWM2_TRG1
**            Output AOI_Input3B                           : SCI0_TXD
**            Output AOI_Input3C                           : CMPA_OUT
**            Output AOI_Input3D                           : SCI0_TXD
**          Interrupts                                     : 
**          Initialization                                 : 
**            Call Init method                             : yes
**            Utilize after reset values                   : default
**     Contents    :
**         Init - void XBARB_Init(void);
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
** @file XBARB_Config.h                                                  
** @version 01.00
** @brief
**          This file implements the XBAR (XBARB) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
*/         
/*!
**  @addtogroup XBARB_Config_module XBARB_Config module documentation
**  @{
*/         

#ifndef XBARB_Config_H_
#define XBARB_Config_H_

/* MODULE XBARB. */

/* XBARB_SEL0: SEL1=0x0D,SEL0=0x17 */
#define XBARB_SEL0_VALUE     0x0D17
#define XBARB_SEL0_MASK      0x1F1F
/* XBARB_SEL1: SEL3=0x0A,SEL2=0x15 */
#define XBARB_SEL1_VALUE     0x0A15
#define XBARB_SEL1_MASK      0x1F1F
/* XBARB_SEL2: SEL5=0x0B,SEL4=0x16 */
#define XBARB_SEL2_VALUE     0x0B16
#define XBARB_SEL2_MASK      0x1F1F
/* XBARB_SEL3: SEL7=0x0A,SEL6=0x14 */
#define XBARB_SEL3_VALUE     0x0A14
#define XBARB_SEL3_MASK      0x1F1F
/* XBARB_SEL4: SEL9=0,SEL8=0x16 */
#define XBARB_SEL4_VALUE     0x16
#define XBARB_SEL4_MASK      0x1F1F
/* XBARB_SEL5: SEL11=0,SEL10=0x0A */
#define XBARB_SEL5_VALUE     0x0A
#define XBARB_SEL5_MASK      0x1F1F
/* XBARB_SEL6: SEL13=0,SEL12=0x17 */
#define XBARB_SEL6_VALUE     0x17
#define XBARB_SEL6_MASK      0x1F1F
/* XBARB_SEL7: SEL15=0,SEL14=0x0A */
#define XBARB_SEL7_VALUE     0x0A
#define XBARB_SEL7_MASK      0x1F1F

#define XBARB_AUTOINIT

/* END XBARB. */
#endif /* #ifndef __XBARB_Config_H_ */
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
