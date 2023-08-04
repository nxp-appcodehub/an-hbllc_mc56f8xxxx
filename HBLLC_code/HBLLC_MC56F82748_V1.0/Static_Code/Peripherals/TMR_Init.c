/** ###################################################################
 **     Filename    : TMR_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the TMR module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void TMR_Init(void);
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
 * @file TMR_Init.c
 * @brief This file implements the TMR module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE TMR_Init. */

#include "TMR_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void TMR_Init(void) {

  /* Register 'TMR_0_CTRL' initialization */
  #ifdef TMR_0_CTRL_VALUE_1
  TMR_0_CTRL = TMR_0_CTRL_VALUE_1;
  #endif

  /* Register 'TMR_1_CTRL' initialization */
  #ifdef TMR_1_CTRL_VALUE_1
  TMR_1_CTRL = TMR_1_CTRL_VALUE_1;
  #endif

  /* Register 'TMR_2_CTRL' initialization */
  #ifdef TMR_2_CTRL_VALUE_1
  TMR_2_CTRL = TMR_2_CTRL_VALUE_1;
  #endif

  /* Register 'TMR_3_CTRL' initialization */
  #ifdef TMR_3_CTRL_VALUE_1
  TMR_3_CTRL = TMR_3_CTRL_VALUE_1;
  #endif

  TMR_0_ENBL = 0;

  /* Register 'TMR_0_SCTRL' initialization */
  #ifdef TMR_0_SCTRL_VALUE_1
  TMR_0_SCTRL = TMR_0_SCTRL_VALUE_1;
  #endif

  /* Register 'TMR_1_SCTRL' initialization */
  #ifdef TMR_1_SCTRL_VALUE_1
  TMR_1_SCTRL = TMR_1_SCTRL_VALUE_1;
  #endif

  /* Register 'TMR_2_SCTRL' initialization */
  #ifdef TMR_2_SCTRL_VALUE_1
  TMR_2_SCTRL = TMR_2_SCTRL_VALUE_1;
  #endif

  /* Register 'TMR_3_SCTRL' initialization */
  #ifdef TMR_3_SCTRL_VALUE_1
  TMR_3_SCTRL = TMR_3_SCTRL_VALUE_1;
  #endif

  /* Register 'TMR_0_CNTR' initialization */
  #ifdef TMR_0_CNTR_VALUE
  TMR_0_CNTR = TMR_0_CNTR_VALUE;
  #endif

  /* Register 'TMR_1_CNTR' initialization */
  #ifdef TMR_1_CNTR_VALUE
  TMR_1_CNTR = TMR_1_CNTR_VALUE;
  #endif

  /* Register 'TMR_2_CNTR' initialization */
  #ifdef TMR_2_CNTR_VALUE
  TMR_2_CNTR = TMR_2_CNTR_VALUE;
  #endif

  /* Register 'TMR_3_CNTR' initialization */
  #ifdef TMR_3_CNTR_VALUE
  TMR_3_CNTR = TMR_3_CNTR_VALUE;
  #endif

  /* Register 'TMR_0_HOLD' initialization */
  #ifdef TMR_0_HOLD_VALUE
  TMR_0_HOLD = TMR_0_HOLD_VALUE;
  #endif

  /* Register 'TMR_1_HOLD' initialization */
  #ifdef TMR_1_HOLD_VALUE
  TMR_1_HOLD = TMR_1_HOLD_VALUE;
  #endif

  /* Register 'TMR_2_HOLD' initialization */
  #ifdef TMR_2_HOLD_VALUE
  TMR_2_HOLD = TMR_2_HOLD_VALUE;
  #endif

  /* Register 'TMR_3_HOLD' initialization */
  #ifdef TMR_3_HOLD_VALUE
  TMR_3_HOLD = TMR_3_HOLD_VALUE;
  #endif

  /* Register 'TMR_0_LOAD' initialization */
  #ifdef TMR_0_LOAD_VALUE
  TMR_0_LOAD = TMR_0_LOAD_VALUE;
  #endif

  /* Register 'TMR_1_LOAD' initialization */
  #ifdef TMR_1_LOAD_VALUE
  TMR_1_LOAD = TMR_1_LOAD_VALUE;
  #endif

  /* Register 'TMR_2_LOAD' initialization */
  #ifdef TMR_2_LOAD_VALUE
  TMR_2_LOAD = TMR_2_LOAD_VALUE;
  #endif

  /* Register 'TMR_3_LOAD' initialization */
  #ifdef TMR_3_LOAD_VALUE
  TMR_3_LOAD = TMR_3_LOAD_VALUE;
  #endif

  /* Register 'TMR_0_COMP1' initialization */
  #ifdef TMR_0_COMP1_VALUE
  TMR_0_COMP1 = TMR_0_COMP1_VALUE;
  #endif

  /* Register 'TMR_1_COMP1' initialization */
  #ifdef TMR_1_COMP1_VALUE
  TMR_1_COMP1 = TMR_1_COMP1_VALUE;
  #endif

  /* Register 'TMR_2_COMP1' initialization */
  #ifdef TMR_2_COMP1_VALUE
  TMR_2_COMP1 = TMR_2_COMP1_VALUE;
  #endif

  /* Register 'TMR_3_COMP1' initialization */
  #ifdef TMR_3_COMP1_VALUE
  TMR_3_COMP1 = TMR_3_COMP1_VALUE;
  #endif

  /* Register 'TMR_0_CMPLD1' initialization */
  #ifdef TMR_0_CMPLD1_VALUE
  TMR_0_CMPLD1 = TMR_0_CMPLD1_VALUE;
  #endif

  /* Register 'TMR_1_CMPLD1' initialization */
  #ifdef TMR_1_CMPLD1_VALUE
  TMR_1_CMPLD1 = TMR_1_CMPLD1_VALUE;
  #endif

  /* Register 'TMR_2_CMPLD1' initialization */
  #ifdef TMR_2_CMPLD1_VALUE
  TMR_2_CMPLD1 = TMR_2_CMPLD1_VALUE;
  #endif

  /* Register 'TMR_3_CMPLD1' initialization */
  #ifdef TMR_3_CMPLD1_VALUE
  TMR_3_CMPLD1 = TMR_3_CMPLD1_VALUE;
  #endif

  /* Register 'TMR_0_COMP2' initialization */
  #ifdef TMR_0_COMP2_VALUE
  TMR_0_COMP2 = TMR_0_COMP2_VALUE;
  #endif

  /* Register 'TMR_1_COMP2' initialization */
  #ifdef TMR_1_COMP2_VALUE
  TMR_1_COMP2 = TMR_1_COMP2_VALUE;
  #endif

  /* Register 'TMR_2_COMP2' initialization */
  #ifdef TMR_2_COMP2_VALUE
  TMR_2_COMP2 = TMR_2_COMP2_VALUE;
  #endif

  /* Register 'TMR_3_COMP2' initialization */
  #ifdef TMR_3_COMP2_VALUE
  TMR_3_COMP2 = TMR_3_COMP2_VALUE;
  #endif

  /* Register 'TMR_0_CMPLD2' initialization */
  #ifdef TMR_0_CMPLD2_VALUE
  TMR_0_CMPLD2 = TMR_0_CMPLD2_VALUE;
  #endif

  /* Register 'TMR_1_CMPLD2' initialization */
  #ifdef TMR_1_CMPLD2_VALUE
  TMR_1_CMPLD2 = TMR_1_CMPLD2_VALUE;
  #endif

  /* Register 'TMR_2_CMPLD2' initialization */
  #ifdef TMR_2_CMPLD2_VALUE
  TMR_2_CMPLD2 = TMR_2_CMPLD2_VALUE;
  #endif

  /* Register 'TMR_3_CMPLD2' initialization */
  #ifdef TMR_3_CMPLD2_VALUE
  TMR_3_CMPLD2 = TMR_3_CMPLD2_VALUE;
  #endif

  /* Register 'TMR_0_CAPT' initialization */
  #ifdef TMR_0_CAPT_VALUE
  TMR_0_CAPT = TMR_0_CAPT_VALUE;
  #endif

  /* Register 'TMR_1_CAPT' initialization */
  #ifdef TMR_1_CAPT_VALUE
  TMR_1_CAPT = TMR_1_CAPT_VALUE;
  #endif

  /* Register 'TMR_2_CAPT' initialization */
  #ifdef TMR_2_CAPT_VALUE
  TMR_2_CAPT = TMR_2_CAPT_VALUE;
  #endif

  /* Register 'TMR_3_CAPT' initialization */
  #ifdef TMR_3_CAPT_VALUE
  TMR_3_CAPT = TMR_3_CAPT_VALUE;
  #endif

  /* Register 'TMR_0_SCTRL' initialization */
  #ifdef TMR_0_SCTRL_VALUE_2
  TMR_0_SCTRL = TMR_0_SCTRL_VALUE_2;
  #endif

  /* Register 'TMR_1_SCTRL' initialization */
  #ifdef TMR_1_SCTRL_VALUE_2
  TMR_1_SCTRL = TMR_1_SCTRL_VALUE_2;
  #endif

  /* Register 'TMR_2_SCTRL' initialization */
  #ifdef TMR_2_SCTRL_VALUE_2
  TMR_2_SCTRL = TMR_2_SCTRL_VALUE_2;
  #endif

  /* Register 'TMR_3_SCTRL' initialization */
  #ifdef TMR_3_SCTRL_VALUE_2
  TMR_3_SCTRL = TMR_3_SCTRL_VALUE_2;
  #endif

  /* Register 'TMR_0_CSCTRL' initialization */
  #ifdef TMR_0_CSCTRL_VALUE
  TMR_0_CSCTRL = TMR_0_CSCTRL_VALUE;
  #endif

  /* Register 'TMR_1_CSCTRL' initialization */
  #ifdef TMR_1_CSCTRL_VALUE
  TMR_1_CSCTRL = TMR_1_CSCTRL_VALUE;
  #endif

  /* Register 'TMR_2_CSCTRL' initialization */
  #ifdef TMR_2_CSCTRL_VALUE
  TMR_2_CSCTRL = TMR_2_CSCTRL_VALUE;
  #endif

  /* Register 'TMR_3_CSCTRL' initialization */
  #ifdef TMR_3_CSCTRL_VALUE
  TMR_3_CSCTRL = TMR_3_CSCTRL_VALUE;
  #endif

  /* Register 'TMR_0_FILT' initialization */
  #ifdef TMR_0_FILT_VALUE
  TMR_0_FILT = TMR_0_FILT_VALUE;
  #endif

  /* Register 'TMR_1_FILT' initialization */
  #ifdef TMR_1_FILT_VALUE
  TMR_1_FILT = TMR_1_FILT_VALUE;
  #endif

  /* Register 'TMR_2_FILT' initialization */
  #ifdef TMR_2_FILT_VALUE
  TMR_2_FILT = TMR_2_FILT_VALUE;
  #endif

  /* Register 'TMR_3_FILT' initialization */
  #ifdef TMR_3_FILT_VALUE
  TMR_3_FILT = TMR_3_FILT_VALUE;
  #endif

  /* Register 'TMR_0_DMA' initialization */
  #ifdef TMR_0_DMA_VALUE
  TMR_0_DMA = TMR_0_DMA_VALUE;
  #endif

  /* Register 'TMR_1_DMA' initialization */
  #ifdef TMR_1_DMA_VALUE
  TMR_1_DMA = TMR_1_DMA_VALUE;
  #endif

  /* Register 'TMR_2_DMA' initialization */
  #ifdef TMR_2_DMA_VALUE
  TMR_2_DMA = TMR_2_DMA_VALUE;
  #endif

  /* Register 'TMR_3_DMA' initialization */
  #ifdef TMR_3_DMA_VALUE
  TMR_3_DMA = TMR_3_DMA_VALUE;
  #endif

  /* Register 'TMR_0_CTRL' initialization */
  #ifdef TMR_0_CTRL_VALUE_2
  TMR_0_CTRL = TMR_0_CTRL_VALUE_2;
  #endif

  /* Register 'TMR_1_CTRL' initialization */
  #ifdef TMR_1_CTRL_VALUE_2
  TMR_1_CTRL = TMR_1_CTRL_VALUE_2;
  #endif

  /* Register 'TMR_2_CTRL' initialization */
  #ifdef TMR_2_CTRL_VALUE_2
  TMR_2_CTRL = TMR_2_CTRL_VALUE_2;
  #endif

  /* Register 'TMR_3_CTRL' initialization */
  #ifdef TMR_3_CTRL_VALUE_2
  TMR_3_CTRL = TMR_3_CTRL_VALUE_2;
  #endif

  /* Register 'TMR_0_ENBL' initialization */
  #ifdef TMR_0_ENBL_VALUE
  TMR_0_ENBL = TMR_0_ENBL_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END TMR_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
