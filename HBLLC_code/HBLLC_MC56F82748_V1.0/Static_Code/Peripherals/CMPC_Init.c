/** ###################################################################
 **     Filename    : CMPC_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the CMPC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMPC_Init(void);
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
 * @file CMPC_Init.c
 * @brief This file implements the CMPC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMPC_Init. */

#include "CMPC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMPC_Init(void) {

  CMPC_CR1 = 0;

  CMPC_CR0 = 0;

  CMPC_MUXCR = 0;

  /* Register 'CMPC_FPR' initialization */
  #if CMPC_FPR_MASK
    #if CMPC_FPR_MASK == 0xFFFF
  CMPC_FPR = CMPC_FPR_VALUE;
    #elif CMPC_FPR_MASK == CMPC_FPR_VALUE
  CMPC_FPR |= CMPC_FPR_VALUE;
    #elif CMPC_FPR_VALUE == 0
  CMPC_FPR &= ~CMPC_FPR_MASK;
    #else
  CMPC_FPR = (CMPC_FPR & (~CMPC_FPR_MASK)) | CMPC_FPR_VALUE;
    #endif
  #elif defined(CMPC_FPR_VALUE)
  CMPC_FPR = CMPC_FPR_VALUE;
  #endif

  /* Register 'CMPC_CR1' initialization */
  #ifdef CMPC_CR1_VALUE_1
  CMPC_CR1 = CMPC_CR1_VALUE_1;
  #endif

  /* Register 'CMPC_SCR' initialization */
  #ifdef CMPC_SCR_VALUE
  CMPC_SCR = CMPC_SCR_VALUE;
  #endif

  /* Register 'CMPC_CR0' initialization */
  #ifdef CMPC_CR0_VALUE
  CMPC_CR0 = CMPC_CR0_VALUE;
  #endif

  /* Register 'CMPC_DACCR' initialization */
  #if CMPC_DACCR_MASK
    #if CMPC_DACCR_MASK == 0xFFFF
  CMPC_DACCR = CMPC_DACCR_VALUE;
    #elif CMPC_DACCR_MASK == CMPC_DACCR_VALUE
  CMPC_DACCR |= CMPC_DACCR_VALUE;
    #elif CMPC_DACCR_VALUE == 0
  CMPC_DACCR &= ~CMPC_DACCR_MASK;
    #else
  CMPC_DACCR = (CMPC_DACCR & (~CMPC_DACCR_MASK)) | CMPC_DACCR_VALUE;
    #endif
  #elif defined(CMPC_DACCR_VALUE)
  CMPC_DACCR = CMPC_DACCR_VALUE;
  #endif

  /* Register 'CMPC_MUXCR' initialization */
  #ifdef CMPC_MUXCR_VALUE
  CMPC_MUXCR = CMPC_MUXCR_VALUE;
  #endif

  /* Register 'CMPC_CR1' initialization */
  #if CMPC_CR1_MASK_2
    #if CMPC_CR1_MASK_2 == 0xFFFF
  CMPC_CR1 = CMPC_CR1_VALUE_2;
    #elif CMPC_CR1_MASK_2 == CMPC_CR1_VALUE_2
  CMPC_CR1 |= CMPC_CR1_VALUE_2;
    #elif CMPC_CR1_VALUE_2 == 0
  CMPC_CR1 &= ~CMPC_CR1_MASK_2;
    #else
  CMPC_CR1 = (CMPC_CR1 & (~CMPC_CR1_MASK_2)) | CMPC_CR1_VALUE_2;
    #endif
  #elif defined(CMPC_CR1_VALUE_2)
  CMPC_CR1 = CMPC_CR1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMPC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
