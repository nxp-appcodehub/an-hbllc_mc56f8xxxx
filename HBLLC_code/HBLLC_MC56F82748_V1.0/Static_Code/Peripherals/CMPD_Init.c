/** ###################################################################
 **     Filename    : CMPD_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the CMPD module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CMPD_Init(void);
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
 * @file CMPD_Init.c
 * @brief This file implements the CMPD module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CMPD_Init. */

#include "CMPD_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CMPD_Init(void) {

  CMPD_CR1 = 0;

  CMPD_CR0 = 0;

  CMPD_MUXCR = 0;

  /* Register 'CMPD_FPR' initialization */
  #if CMPD_FPR_MASK
    #if CMPD_FPR_MASK == 0xFFFF
  CMPD_FPR = CMPD_FPR_VALUE;
    #elif CMPD_FPR_MASK == CMPD_FPR_VALUE
  CMPD_FPR |= CMPD_FPR_VALUE;
    #elif CMPD_FPR_VALUE == 0
  CMPD_FPR &= ~CMPD_FPR_MASK;
    #else
  CMPD_FPR = (CMPD_FPR & (~CMPD_FPR_MASK)) | CMPD_FPR_VALUE;
    #endif
  #elif defined(CMPD_FPR_VALUE)
  CMPD_FPR = CMPD_FPR_VALUE;
  #endif

  /* Register 'CMPD_CR1' initialization */
  #ifdef CMPD_CR1_VALUE_1
  CMPD_CR1 = CMPD_CR1_VALUE_1;
  #endif

  /* Register 'CMPD_SCR' initialization */
  #ifdef CMPD_SCR_VALUE
  CMPD_SCR = CMPD_SCR_VALUE;
  #endif

  /* Register 'CMPD_CR0' initialization */
  #ifdef CMPD_CR0_VALUE
  CMPD_CR0 = CMPD_CR0_VALUE;
  #endif

  /* Register 'CMPD_DACCR' initialization */
  #if CMPD_DACCR_MASK
    #if CMPD_DACCR_MASK == 0xFFFF
  CMPD_DACCR = CMPD_DACCR_VALUE;
    #elif CMPD_DACCR_MASK == CMPD_DACCR_VALUE
  CMPD_DACCR |= CMPD_DACCR_VALUE;
    #elif CMPD_DACCR_VALUE == 0
  CMPD_DACCR &= ~CMPD_DACCR_MASK;
    #else
  CMPD_DACCR = (CMPD_DACCR & (~CMPD_DACCR_MASK)) | CMPD_DACCR_VALUE;
    #endif
  #elif defined(CMPD_DACCR_VALUE)
  CMPD_DACCR = CMPD_DACCR_VALUE;
  #endif

  /* Register 'CMPD_MUXCR' initialization */
  #ifdef CMPD_MUXCR_VALUE
  CMPD_MUXCR = CMPD_MUXCR_VALUE;
  #endif

  /* Register 'CMPD_CR1' initialization */
  #if CMPD_CR1_MASK_2
    #if CMPD_CR1_MASK_2 == 0xFFFF
  CMPD_CR1 = CMPD_CR1_VALUE_2;
    #elif CMPD_CR1_MASK_2 == CMPD_CR1_VALUE_2
  CMPD_CR1 |= CMPD_CR1_VALUE_2;
    #elif CMPD_CR1_VALUE_2 == 0
  CMPD_CR1 &= ~CMPD_CR1_MASK_2;
    #else
  CMPD_CR1 = (CMPD_CR1 & (~CMPD_CR1_MASK_2)) | CMPD_CR1_VALUE_2;
    #endif
  #elif defined(CMPD_CR1_VALUE_2)
  CMPD_CR1 = CMPD_CR1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CMPD_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
