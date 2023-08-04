/** ###################################################################
 **     Filename    : CMPA_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the CMPA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMPA_Init(void);
 **
 **     Copyright : 1997 - 2014 Freescale Semiconductor, Inc.
 **     
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
 * @file CMPA_Init.c
 * @brief This file implements the CMPA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMPA_Init. */

#include "CMPA_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMPA_Init(void) {

  CMPA_CR1 = 0;

  CMPA_CR0 = 0;

  CMPA_MUXCR = 0;

  /* Register 'CMPA_FPR' initialization */
  #if CMPA_FPR_MASK
    #if CMPA_FPR_MASK == 0xFFFF
  CMPA_FPR = CMPA_FPR_VALUE;
    #elif CMPA_FPR_MASK == CMPA_FPR_VALUE
  CMPA_FPR |= CMPA_FPR_VALUE;
    #elif CMPA_FPR_VALUE == 0
  CMPA_FPR &= ~CMPA_FPR_MASK;
    #else
  CMPA_FPR = (CMPA_FPR & (~CMPA_FPR_MASK)) | CMPA_FPR_VALUE;
    #endif
  #elif defined(CMPA_FPR_VALUE)
  CMPA_FPR = CMPA_FPR_VALUE;
  #endif

  /* Register 'CMPA_CR1' initialization */
  #ifdef CMPA_CR1_VALUE_1
  CMPA_CR1 = CMPA_CR1_VALUE_1;
  #endif

  /* Register 'CMPA_SCR' initialization */
  #ifdef CMPA_SCR_VALUE
  CMPA_SCR = CMPA_SCR_VALUE;
  #endif

  /* Register 'CMPA_CR0' initialization */
  #ifdef CMPA_CR0_VALUE
  CMPA_CR0 = CMPA_CR0_VALUE;
  #endif

  /* Register 'CMPA_DACCR' initialization */
  #if CMPA_DACCR_MASK
    #if CMPA_DACCR_MASK == 0xFFFF
  CMPA_DACCR = CMPA_DACCR_VALUE;
    #elif CMPA_DACCR_MASK == CMPA_DACCR_VALUE
  CMPA_DACCR |= CMPA_DACCR_VALUE;
    #elif CMPA_DACCR_VALUE == 0
  CMPA_DACCR &= ~CMPA_DACCR_MASK;
    #else
  CMPA_DACCR = (CMPA_DACCR & (~CMPA_DACCR_MASK)) | CMPA_DACCR_VALUE;
    #endif
  #elif defined(CMPA_DACCR_VALUE)
  CMPA_DACCR = CMPA_DACCR_VALUE;
  #endif

  /* Register 'CMPA_MUXCR' initialization */
  #ifdef CMPA_MUXCR_VALUE
  CMPA_MUXCR = CMPA_MUXCR_VALUE;
  #endif

  /* Register 'CMPA_CR1' initialization */
  #if CMPA_CR1_MASK_2
    #if CMPA_CR1_MASK_2 == 0xFFFF
  CMPA_CR1 = CMPA_CR1_VALUE_2;
    #elif CMPA_CR1_MASK_2 == CMPA_CR1_VALUE_2
  CMPA_CR1 |= CMPA_CR1_VALUE_2;
    #elif CMPA_CR1_VALUE_2 == 0
  CMPA_CR1 &= ~CMPA_CR1_MASK_2;
    #else
  CMPA_CR1 = (CMPA_CR1 & (~CMPA_CR1_MASK_2)) | CMPA_CR1_VALUE_2;
    #endif
  #elif defined(CMPA_CR1_VALUE_2)
  CMPA_CR1 = CMPA_CR1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMPA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
