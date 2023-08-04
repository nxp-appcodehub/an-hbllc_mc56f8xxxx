/** ###################################################################
 **     Filename    : QSPI1_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the QSPI1 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void QSPI1_Init(void);
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
 * @file QSPI1_Init.c
 * @brief This file implements the QSPI1 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE QSPI1_Init. */

#include "QSPI1_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void QSPI1_Init(void) {

  QSPI1_SPSCR &= ~0x20;

  /* Register 'QSPI1_SPDSR' initialization */
  #if QSPI1_SPDSR_MASK_1
    #if QSPI1_SPDSR_MASK_1 == 0xFFFF
  QSPI1_SPDSR = QSPI1_SPDSR_VALUE_1;
    #elif QSPI1_SPDSR_MASK_1 == QSPI1_SPDSR_VALUE_1
  QSPI1_SPDSR |= QSPI1_SPDSR_VALUE_1;
    #elif QSPI1_SPDSR_VALUE_1 == 0
  QSPI1_SPDSR &= ~QSPI1_SPDSR_MASK_1;
    #else
  QSPI1_SPDSR = (QSPI1_SPDSR & (~QSPI1_SPDSR_MASK_1)) | QSPI1_SPDSR_VALUE_1;
    #endif
  #elif defined(QSPI1_SPDSR_VALUE_1)
  QSPI1_SPDSR = QSPI1_SPDSR_VALUE_1;
  #endif

  /* Register 'QSPI1_SPWAIT' initialization */
  #if QSPI1_SPWAIT_MASK
    #if QSPI1_SPWAIT_MASK == 0xFFFF
  QSPI1_SPWAIT = QSPI1_SPWAIT_VALUE;
    #elif QSPI1_SPWAIT_MASK == QSPI1_SPWAIT_VALUE
  QSPI1_SPWAIT |= QSPI1_SPWAIT_VALUE;
    #elif QSPI1_SPWAIT_VALUE == 0
  QSPI1_SPWAIT &= ~QSPI1_SPWAIT_MASK;
    #else
  QSPI1_SPWAIT = (QSPI1_SPWAIT & (~QSPI1_SPWAIT_MASK)) | QSPI1_SPWAIT_VALUE;
    #endif
  #elif defined(QSPI1_SPWAIT_VALUE)
  QSPI1_SPWAIT = QSPI1_SPWAIT_VALUE;
  #endif

  /* Register 'QSPI1_SPFIFO' initialization */
  #if QSPI1_SPFIFO_MASK
    #if QSPI1_SPFIFO_MASK == 0xFFFF
  QSPI1_SPFIFO = QSPI1_SPFIFO_VALUE;
    #elif QSPI1_SPFIFO_MASK == QSPI1_SPFIFO_VALUE
  QSPI1_SPFIFO |= QSPI1_SPFIFO_VALUE;
    #elif QSPI1_SPFIFO_VALUE == 0
  QSPI1_SPFIFO &= ~QSPI1_SPFIFO_MASK;
    #else
  QSPI1_SPFIFO = (QSPI1_SPFIFO & (~QSPI1_SPFIFO_MASK)) | QSPI1_SPFIFO_VALUE;
    #endif
  #elif defined(QSPI1_SPFIFO_VALUE)
  QSPI1_SPFIFO = QSPI1_SPFIFO_VALUE;
  #endif

  /* Register 'QSPI1_SPCTL2' initialization */
  #if QSPI1_SPCTL2_MASK
    #if QSPI1_SPCTL2_MASK == 0xFFFF
  QSPI1_SPCTL2 = QSPI1_SPCTL2_VALUE;
    #elif QSPI1_SPCTL2_MASK == QSPI1_SPCTL2_VALUE
  QSPI1_SPCTL2 |= QSPI1_SPCTL2_VALUE;
    #elif QSPI1_SPCTL2_VALUE == 0
  QSPI1_SPCTL2 &= ~QSPI1_SPCTL2_MASK;
    #else
  QSPI1_SPCTL2 = (QSPI1_SPCTL2 & (~QSPI1_SPCTL2_MASK)) | QSPI1_SPCTL2_VALUE;
    #endif
  #elif defined(QSPI1_SPCTL2_VALUE)
  QSPI1_SPCTL2 = QSPI1_SPCTL2_VALUE;
  #endif

  /* Register 'QSPI1_SPSCR' initialization */
  #if QSPI1_SPSCR_MASK_1
    #if QSPI1_SPSCR_MASK_1 == 0xFFFF
  QSPI1_SPSCR = QSPI1_SPSCR_VALUE_1;
    #elif QSPI1_SPSCR_MASK_1 == QSPI1_SPSCR_VALUE_1
  QSPI1_SPSCR |= QSPI1_SPSCR_VALUE_1;
    #elif QSPI1_SPSCR_VALUE_1 == 0
  QSPI1_SPSCR &= ~QSPI1_SPSCR_MASK_1;
    #else
  QSPI1_SPSCR = (QSPI1_SPSCR & (~QSPI1_SPSCR_MASK_1)) | QSPI1_SPSCR_VALUE_1;
    #endif
  #elif defined(QSPI1_SPSCR_VALUE_1)
  QSPI1_SPSCR = QSPI1_SPSCR_VALUE_1;
  #endif

  /* Register 'QSPI1_SPDSR' initialization */
  #if QSPI1_SPDSR_MASK_2
    #if QSPI1_SPDSR_MASK_2 == 0xFFFF
  QSPI1_SPDSR = QSPI1_SPDSR_VALUE_2;
    #elif QSPI1_SPDSR_MASK_2 == QSPI1_SPDSR_VALUE_2
  QSPI1_SPDSR |= QSPI1_SPDSR_VALUE_2;
    #elif QSPI1_SPDSR_VALUE_2 == 0
  QSPI1_SPDSR &= ~QSPI1_SPDSR_MASK_2;
    #else
  QSPI1_SPDSR = (QSPI1_SPDSR & (~QSPI1_SPDSR_MASK_2)) | QSPI1_SPDSR_VALUE_2;
    #endif
  #elif defined(QSPI1_SPDSR_VALUE_2)
  QSPI1_SPDSR = QSPI1_SPDSR_VALUE_2;
  #endif
  /* Clear flags */
  #ifdef QSPI1_CLEAR_FLAGS
  QSPI1_SPSCR |= 0x02;
  #endif

  /* Register 'QSPI1_SPSCR' initialization */
  #if QSPI1_SPSCR_MASK_3
    #if QSPI1_SPSCR_MASK_3 == 0xFFFF
  QSPI1_SPSCR = QSPI1_SPSCR_VALUE_3;
    #elif QSPI1_SPSCR_MASK_3 == QSPI1_SPSCR_VALUE_3
  QSPI1_SPSCR |= QSPI1_SPSCR_VALUE_3;
    #elif QSPI1_SPSCR_VALUE_3 == 0
  QSPI1_SPSCR &= ~QSPI1_SPSCR_MASK_3;
    #else
  QSPI1_SPSCR = (QSPI1_SPSCR & (~QSPI1_SPSCR_MASK_3)) | QSPI1_SPSCR_VALUE_3;
    #endif
  #elif defined(QSPI1_SPSCR_VALUE_3)
  QSPI1_SPSCR = QSPI1_SPSCR_VALUE_3;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END QSPI1_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
