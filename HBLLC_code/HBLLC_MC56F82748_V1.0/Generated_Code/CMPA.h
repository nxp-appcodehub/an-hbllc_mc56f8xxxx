/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : CMPA.h
**     Project     : HBLLC_MC56F82748_V1.0
**     Processor   : MC56F82748VLH
**     Component   : Init_HSCMP
**     Version     : Component 01.009, Driver 01.09, CPU db: 3.50.001
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2022-01-27, 11:59, # CodeGen: 0
**     Abstract    :
**          This file implements the HSCMP (CMPA) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
**     Settings    :
**          Component name                                 : CMPA
**          Device                                         : CMPA
**          Settings                                       : 
**            Sample                                       : Disabled
**            Windowing                                    : Disabled
**            Filter sample count                          : 3
**            Filter sample period                         : 1
**            Sampling period                              : 0.020 us
**            Invert control                               : Do NOT invert
**            Comparator output select                     : Filtered
**            Power mode                                   : High speed
**            Hysteresis control                           : Level 0
**            DAC module                                   : Enabled
**              DAC device                                 : CMPA_DACCR
**              Reference selection                        : Internal
**              Output voltage select                      : 52
**          Pins/Signals                                   : 
**            Positive input pins                          : Enabled
**              Positive input pin0                        : Disabled
**              Positive input pin1                        : Disabled
**              Positive input pin2                        : Disabled
**              Positive input pin3                        : Disabled
**              Positive input pin4                        : Disabled
**              Positive input pin5                        : Disabled
**              Positive input pin6                        : Disabled
**              Positive input pin7                        : Enabled
**                Positive input pin7                      : DAC6bA_Output
**                Positive input pin7 signal               : 
**            Negative input pins                          : Enabled
**              Negative input pin0                        : Disabled
**              Negative input pin1                        : Disabled
**              Negative input pin2                        : Disabled
**              Negative input pin3                        : Enabled
**                Negative input pin3                      : GPIOA0/ANA0/CMPA_IN3/CMPC_O
**                Negative input pin3 signal               : 
**              Negative input pin4                        : Disabled
**              Negative input pin5                        : Disabled
**              Negative input pin6                        : Disabled
**              Negative input pin7                        : Disabled
**            Window/Sample input                          : Disabled
**            Output pin                                   : Disabled
**          Interrupts/DMA                                 : 
**            Rising or Falling compare                    : 
**              Interrupt                                  : INT_CMPA
**              Interrupt priority                         : disabled
**              ISR name                                   : 
**              Rising compare interrupt                   : Disabled
**              Falling compare interrupt                  : Disabled
**          Initialization                                 : 
**            Comparator module                            : Enabled
**            Initial positive input                       : IN7
**            Initial negative input                       : IN3
**            Enable peripheral clock                      : yes
**            Call Init method                             : yes
**            Utilize after reset values                   : default
**     Contents    :
**         Init - void CMPA_Init(void);
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
** @file CMPA.h
** @version 01.09
** @brief
**          This file implements the HSCMP (CMPA) module initialization
**          according to the Peripheral Initialization settings, and
**          defines interrupt service routines prototypes.
*/         
/*!
**  @addtogroup CMPA_module CMPA module documentation
**  @{
*/         

#ifndef CMPA_H_
#define CMPA_H_

/* MODULE CMPA. */

#include "CMPA_Init.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialization method user name */
#define CMPA_Init CMPA_Init
/* PDD macros peripheral base address parameter */
#define CMPA_DEVICE CMPA_BASE_PTR

#ifdef __cplusplus
}
#endif

/* END CMPA. */
#endif /* #ifndef __CMPA_H_ */
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
