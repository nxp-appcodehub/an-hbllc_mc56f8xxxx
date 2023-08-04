/** ###################################################################
**      Filename    : Peripherals_Init.c
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
** @file Peripherals_Init.c                                                  
** @version 01.00
*/         
/*!
**  @addtogroup Peripherals_Init_module Peripherals_Init module documentation
**  @{
*/         

/* MODULE Peripherals_Init.c */

#include "Peripherals_Init.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void Peripherals_Init(void)
{
#ifdef INTC_AUTOINIT
  INTC_Init();
#endif /* INTC_AUTOINIT */
#ifdef SIM_AUTOINIT
  SIM_Init();
#endif /* SIM_AUTOINIT */
#ifdef XBARA_AUTOINIT
  XBARA_Init();
#endif /* XBARA_AUTOINIT */
#ifdef XBARB_AUTOINIT
  XBARB_Init();
#endif /* XBARB_AUTOINIT */

#ifdef GPIOA_AUTOINIT
  GPIOA_Init();
#endif /* GPIOA_AUTOINIT */
#ifdef GPIOB_AUTOINIT
  GPIOB_Init();
#endif /* GPIOB_AUTOINIT */
#ifdef GPIOC_AUTOINIT
  GPIOC_Init();
#endif /* GPIOC_AUTOINIT */
#ifdef GPIOD_AUTOINIT
  GPIOD_Init();
#endif /* GPIOD_AUTOINIT */
#ifdef GPIOE_AUTOINIT
  GPIOE_Init();
#endif /* GPIOE_AUTOINIT */
#ifdef GPIOF_AUTOINIT
  GPIOF_Init();
#endif /* GPIOF_AUTOINIT */
#ifdef MCM_AUTOINIT
  MCM_Init();
#endif /* MCM_AUTOINIT */
#ifdef PMC_AUTOINIT
  PMC_Init();
#endif /* PMC_AUTOINIT */

#ifdef ADC_AUTOINIT
  ADC_Init();
#endif /* ADC_AUTOINIT */
#ifdef AOI_AUTOINIT
  AOI_Init();
#endif /* AOI_AUTOINIT */
#ifdef CAN_AUTOINIT
  CAN_Init();
#endif /* CAN_AUTOINIT */
#ifdef CMPA_AUTOINIT
  CMPA_Init();
#endif /* CMPA_AUTOINIT */
#ifdef CMPB_AUTOINIT
  CMPB_Init();
#endif /* CMPB_AUTOINIT */
#ifdef CMPC_AUTOINIT
  CMPC_Init();
#endif /* CMPC_AUTOINIT */
#ifdef CMPD_AUTOINIT
  CMPD_Init();
#endif /* CMPD_AUTOINIT */
#ifdef COP_AUTOINIT
  COP_Init();
#endif /* COP_AUTOINIT */
#ifdef CRC_AUTOINIT
  CRC_Init();
#endif /* CRC_AUTOINIT */
#ifdef DACA_AUTOINIT
  DACA_Init();
#endif /* DACA_AUTOINIT */
#ifdef DACB_AUTOINIT
  DACB_Init();
#endif /* DACB_AUTOINIT */
#ifdef DMA_AUTOINIT
  DMA_Init();
#endif /* DMA_AUTOINIT */
#ifdef EWM_AUTOINIT
  EWM_Init();
#endif /* EWM_AUTOINIT */
#ifdef FMC_AUTOINIT
  FMC_Init();
#endif /* FMC_AUTOINIT */
#ifdef FTFA_AUTOINIT
  FTFA_Init();
#endif /* FTFA_AUTOINIT */
#ifdef I2C_AUTOINIT
  I2C_Init();
#endif /* I2C_AUTOINIT */
#ifdef PIT0_AUTOINIT
  PIT0_Init();
#endif /* PIT0_AUTOINIT */
#ifdef PIT1_AUTOINIT
  PIT1_Init();
#endif /* PIT1_AUTOINIT */
#ifdef PWMA_AUTOINIT
  PWMA_Init();
#endif /* PWMA_AUTOINIT */
#ifdef QSCI0_AUTOINIT
  QSCI0_Init();
#endif /* QSCI0_AUTOINIT */
#ifdef QSCI1_AUTOINIT
  QSCI1_Init();
#endif /* QSCI1_AUTOINIT */
#ifdef QSPI0_AUTOINIT
  QSPI0_Init();
#endif /* QSPI0_AUTOINIT */
#ifdef QSPI1_AUTOINIT
  QSPI1_Init();
#endif /* QSPI1_AUTOINIT */
#ifdef TMR_AUTOINIT
  TMR_Init();
#endif /* TMR_AUTOINIT */
}

#ifdef __cplusplus
}
#endif

/* END Peripherals_Init.h */

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
