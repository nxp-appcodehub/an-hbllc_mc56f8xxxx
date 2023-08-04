/** ###################################################################
 **     Filename    : CMPB_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the CMPB module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMPB_Init(void);
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
 * @file CMPB_Init.c
 * @brief This file implements the CMPB module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMPB_Init. */

#include "CMPB_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMPB_Init(void) {

  CMPB_CR1 = 0;

  CMPB_CR0 = 0;

  CMPB_MUXCR = 0;

  /* Register 'CMPB_FPR' initialization */
  #if CMPB_FPR_MASK
    #if CMPB_FPR_MASK == 0xFFFF
  CMPB_FPR = CMPB_FPR_VALUE;
    #elif CMPB_FPR_MASK == CMPB_FPR_VALUE
  CMPB_FPR |= CMPB_FPR_VALUE;
    #elif CMPB_FPR_VALUE == 0
  CMPB_FPR &= ~CMPB_FPR_MASK;
    #else
  CMPB_FPR = (CMPB_FPR & (~CMPB_FPR_MASK)) | CMPB_FPR_VALUE;
    #endif
  #elif defined(CMPB_FPR_VALUE)
  CMPB_FPR = CMPB_FPR_VALUE;
  #endif

  /* Register 'CMPB_CR1' initialization */
  #ifdef CMPB_CR1_VALUE_1
  CMPB_CR1 = CMPB_CR1_VALUE_1;
  #endif

  /* Register 'CMPB_SCR' initialization */
  #ifdef CMPB_SCR_VALUE
  CMPB_SCR = CMPB_SCR_VALUE;
  #endif

  /* Register 'CMPB_CR0' initialization */
  #ifdef CMPB_CR0_VALUE
  CMPB_CR0 = CMPB_CR0_VALUE;
  #endif

  /* Register 'CMPB_DACCR' initialization */
  #if CMPB_DACCR_MASK
    #if CMPB_DACCR_MASK == 0xFFFF
  CMPB_DACCR = CMPB_DACCR_VALUE;
    #elif CMPB_DACCR_MASK == CMPB_DACCR_VALUE
  CMPB_DACCR |= CMPB_DACCR_VALUE;
    #elif CMPB_DACCR_VALUE == 0
  CMPB_DACCR &= ~CMPB_DACCR_MASK;
    #else
  CMPB_DACCR = (CMPB_DACCR & (~CMPB_DACCR_MASK)) | CMPB_DACCR_VALUE;
    #endif
  #elif defined(CMPB_DACCR_VALUE)
  CMPB_DACCR = CMPB_DACCR_VALUE;
  #endif

  /* Register 'CMPB_MUXCR' initialization */
  #ifdef CMPB_MUXCR_VALUE
  CMPB_MUXCR = CMPB_MUXCR_VALUE;
  #endif

  /* Register 'CMPB_CR1' initialization */
  #if CMPB_CR1_MASK_2
    #if CMPB_CR1_MASK_2 == 0xFFFF
  CMPB_CR1 = CMPB_CR1_VALUE_2;
    #elif CMPB_CR1_MASK_2 == CMPB_CR1_VALUE_2
  CMPB_CR1 |= CMPB_CR1_VALUE_2;
    #elif CMPB_CR1_VALUE_2 == 0
  CMPB_CR1 &= ~CMPB_CR1_MASK_2;
    #else
  CMPB_CR1 = (CMPB_CR1 & (~CMPB_CR1_MASK_2)) | CMPB_CR1_VALUE_2;
    #endif
  #elif defined(CMPB_CR1_VALUE_2)
  CMPB_CR1 = CMPB_CR1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMPB_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
