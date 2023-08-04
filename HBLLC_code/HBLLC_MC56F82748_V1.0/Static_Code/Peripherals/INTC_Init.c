/** ###################################################################
 **     Filename    : INTC_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the INTC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void INTC_Init(void);
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
 * @file INTC_Init.c
 * @brief This file implements the INTC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE INTC_Init. */

#include "INTC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void INTC_Init(void) {

  /* Register 'INTC_VBA' initialization */
  #if INTC_VBA_MASK
    #if INTC_VBA_MASK == 0xFFFF
  INTC_VBA = INTC_VBA_VALUE;
    #elif INTC_VBA_MASK == INTC_VBA_VALUE
  INTC_VBA |= INTC_VBA_VALUE;
    #elif INTC_VBA_VALUE == 0
  INTC_VBA &= ~INTC_VBA_MASK;
    #else
  INTC_VBA = (INTC_VBA & (~INTC_VBA_MASK)) | INTC_VBA_VALUE;
    #endif
  #elif defined(INTC_VBA_VALUE)
  INTC_VBA = INTC_VBA_VALUE;
  #endif

  /* Register 'INTC_IPR2' initialization */
  #if INTC_IPR2_MASK
    #if INTC_IPR2_MASK == 0xFFFF
  INTC_IPR2 = INTC_IPR2_VALUE;
    #elif INTC_IPR2_MASK == INTC_IPR2_VALUE
  INTC_IPR2 |= INTC_IPR2_VALUE;
    #elif INTC_IPR2_VALUE == 0
  INTC_IPR2 &= ~INTC_IPR2_MASK;
    #else
  INTC_IPR2 = (INTC_IPR2 & (~INTC_IPR2_MASK)) | INTC_IPR2_VALUE;
    #endif
  #elif defined(INTC_IPR2_VALUE)
  INTC_IPR2 = INTC_IPR2_VALUE;
  #endif

  /* Register 'INTC_IPR3' initialization */
  #if INTC_IPR3_MASK
    #if INTC_IPR3_MASK == 0xFFFF
  INTC_IPR3 = INTC_IPR3_VALUE;
    #elif INTC_IPR3_MASK == INTC_IPR3_VALUE
  INTC_IPR3 |= INTC_IPR3_VALUE;
    #elif INTC_IPR3_VALUE == 0
  INTC_IPR3 &= ~INTC_IPR3_MASK;
    #else
  INTC_IPR3 = (INTC_IPR3 & (~INTC_IPR3_MASK)) | INTC_IPR3_VALUE;
    #endif
  #elif defined(INTC_IPR3_VALUE)
  INTC_IPR3 = INTC_IPR3_VALUE;
  #endif

  /* Register 'INTC_IPR4' initialization */
  #if INTC_IPR4_MASK
    #if INTC_IPR4_MASK == 0xFFFF
  INTC_IPR4 = INTC_IPR4_VALUE;
    #elif INTC_IPR4_MASK == INTC_IPR4_VALUE
  INTC_IPR4 |= INTC_IPR4_VALUE;
    #elif INTC_IPR4_VALUE == 0
  INTC_IPR4 &= ~INTC_IPR4_MASK;
    #else
  INTC_IPR4 = (INTC_IPR4 & (~INTC_IPR4_MASK)) | INTC_IPR4_VALUE;
    #endif
  #elif defined(INTC_IPR4_VALUE)
  INTC_IPR4 = INTC_IPR4_VALUE;
  #endif

  /* Register 'INTC_IPR5' initialization */
  #if INTC_IPR5_MASK
    #if INTC_IPR5_MASK == 0xFFFF
  INTC_IPR5 = INTC_IPR5_VALUE;
    #elif INTC_IPR5_MASK == INTC_IPR5_VALUE
  INTC_IPR5 |= INTC_IPR5_VALUE;
    #elif INTC_IPR5_VALUE == 0
  INTC_IPR5 &= ~INTC_IPR5_MASK;
    #else
  INTC_IPR5 = (INTC_IPR5 & (~INTC_IPR5_MASK)) | INTC_IPR5_VALUE;
    #endif
  #elif defined(INTC_IPR5_VALUE)
  INTC_IPR5 = INTC_IPR5_VALUE;
  #endif

  /* Register 'INTC_IPR6' initialization */
  #if INTC_IPR6_MASK
    #if INTC_IPR6_MASK == 0xFFFF
  INTC_IPR6 = INTC_IPR6_VALUE;
    #elif INTC_IPR6_MASK == INTC_IPR6_VALUE
  INTC_IPR6 |= INTC_IPR6_VALUE;
    #elif INTC_IPR6_VALUE == 0
  INTC_IPR6 &= ~INTC_IPR6_MASK;
    #else
  INTC_IPR6 = (INTC_IPR6 & (~INTC_IPR6_MASK)) | INTC_IPR6_VALUE;
    #endif
  #elif defined(INTC_IPR6_VALUE)
  INTC_IPR6 = INTC_IPR6_VALUE;
  #endif

  /* Register 'INTC_IPR8' initialization */
  #if INTC_IPR8_MASK
    #if INTC_IPR8_MASK == 0xFFFF
  INTC_IPR8 = INTC_IPR8_VALUE;
    #elif INTC_IPR8_MASK == INTC_IPR8_VALUE
  INTC_IPR8 |= INTC_IPR8_VALUE;
    #elif INTC_IPR8_VALUE == 0
  INTC_IPR8 &= ~INTC_IPR8_MASK;
    #else
  INTC_IPR8 = (INTC_IPR8 & (~INTC_IPR8_MASK)) | INTC_IPR8_VALUE;
    #endif
  #elif defined(INTC_IPR8_VALUE)
  INTC_IPR8 = INTC_IPR8_VALUE;
  #endif

  /* Register 'INTC_IPR9' initialization */
  #ifdef INTC_IPR9_VALUE
  INTC_IPR9 = INTC_IPR9_VALUE;
  #endif

  /* Register 'INTC_IPR10' initialization */
  #if INTC_IPR10_MASK
    #if INTC_IPR10_MASK == 0xFFFF
  INTC_IPR10 = INTC_IPR10_VALUE;
    #elif INTC_IPR10_MASK == INTC_IPR10_VALUE
  INTC_IPR10 |= INTC_IPR10_VALUE;
    #elif INTC_IPR10_VALUE == 0
  INTC_IPR10 &= ~INTC_IPR10_MASK;
    #else
  INTC_IPR10 = (INTC_IPR10 & (~INTC_IPR10_MASK)) | INTC_IPR10_VALUE;
    #endif
  #elif defined(INTC_IPR10_VALUE)
  INTC_IPR10 = INTC_IPR10_VALUE;
  #endif

  /* Register 'INTC_IPR11' initialization */
  #if INTC_IPR11_MASK
    #if INTC_IPR11_MASK == 0xFFFF
  INTC_IPR11 = INTC_IPR11_VALUE;
    #elif INTC_IPR11_MASK == INTC_IPR11_VALUE
  INTC_IPR11 |= INTC_IPR11_VALUE;
    #elif INTC_IPR11_VALUE == 0
  INTC_IPR11 &= ~INTC_IPR11_MASK;
    #else
  INTC_IPR11 = (INTC_IPR11 & (~INTC_IPR11_MASK)) | INTC_IPR11_VALUE;
    #endif
  #elif defined(INTC_IPR11_VALUE)
  INTC_IPR11 = INTC_IPR11_VALUE;
  #endif

  /* Register 'INTC_IPR12' initialization */
  #if INTC_IPR12_MASK
    #if INTC_IPR12_MASK == 0xFFFF
  INTC_IPR12 = INTC_IPR12_VALUE;
    #elif INTC_IPR12_MASK == INTC_IPR12_VALUE
  INTC_IPR12 |= INTC_IPR12_VALUE;
    #elif INTC_IPR12_VALUE == 0
  INTC_IPR12 &= ~INTC_IPR12_MASK;
    #else
  INTC_IPR12 = (INTC_IPR12 & (~INTC_IPR12_MASK)) | INTC_IPR12_VALUE;
    #endif
  #elif defined(INTC_IPR12_VALUE)
  INTC_IPR12 = INTC_IPR12_VALUE;
  #endif

  /* Register 'INTC_IPR0' initialization */
  #if INTC_IPR0_MASK
    #if INTC_IPR0_MASK == 0xFFFF
  INTC_IPR0 = INTC_IPR0_VALUE;
    #elif INTC_IPR0_MASK == INTC_IPR0_VALUE
  INTC_IPR0 |= INTC_IPR0_VALUE;
    #elif INTC_IPR0_VALUE == 0
  INTC_IPR0 &= ~INTC_IPR0_MASK;
    #else
  INTC_IPR0 = (INTC_IPR0 & (~INTC_IPR0_MASK)) | INTC_IPR0_VALUE;
    #endif
  #elif defined(INTC_IPR0_VALUE)
  INTC_IPR0 = INTC_IPR0_VALUE;
  #endif

  /* Register 'INTC_IPR1' initialization */
  #if INTC_IPR1_MASK
    #if INTC_IPR1_MASK == 0xFFFF
  INTC_IPR1 = INTC_IPR1_VALUE;
    #elif INTC_IPR1_MASK == INTC_IPR1_VALUE
  INTC_IPR1 |= INTC_IPR1_VALUE;
    #elif INTC_IPR1_VALUE == 0
  INTC_IPR1 &= ~INTC_IPR1_MASK;
    #else
  INTC_IPR1 = (INTC_IPR1 & (~INTC_IPR1_MASK)) | INTC_IPR1_VALUE;
    #endif
  #elif defined(INTC_IPR1_VALUE)
  INTC_IPR1 = INTC_IPR1_VALUE;
  #endif
  /* Register 'INTC_FIVAL0' initialization */
  #ifdef INTC_FIVAL0_VALUE
  INTC_FIVAL0 = INTC_FIVAL0_VALUE;
  #endif
  /* Register 'INTC_FIVAH0' initialization */
  #ifdef INTC_FIVAH0_VALUE
  INTC_FIVAH0 = INTC_FIVAH0_VALUE;
  #endif
  /* Register 'INTC_FIVAL1' initialization */
  #ifdef INTC_FIVAL1_VALUE
  INTC_FIVAL1 = INTC_FIVAL1_VALUE;
  #endif
  /* Register 'INTC_FIVAH1' initialization */
  #ifdef INTC_FIVAH1_VALUE
  INTC_FIVAH1 = INTC_FIVAH1_VALUE;
  #endif

  /* Register 'INTC_FIM0' initialization */
  #if INTC_FIM0_MASK
    #if INTC_FIM0_MASK == 0xFFFF
  INTC_FIM0 = INTC_FIM0_VALUE;
    #elif INTC_FIM0_MASK == INTC_FIM0_VALUE
  INTC_FIM0 |= INTC_FIM0_VALUE;
    #elif INTC_FIM0_VALUE == 0
  INTC_FIM0 &= ~INTC_FIM0_MASK;
    #else
  INTC_FIM0 = (INTC_FIM0 & (~INTC_FIM0_MASK)) | INTC_FIM0_VALUE;
    #endif
  #elif defined(INTC_FIM0_VALUE)
  INTC_FIM0 = INTC_FIM0_VALUE;
  #endif

  /* Register 'INTC_FIM1' initialization */
  #if INTC_FIM1_MASK
    #if INTC_FIM1_MASK == 0xFFFF
  INTC_FIM1 = INTC_FIM1_VALUE;
    #elif INTC_FIM1_MASK == INTC_FIM1_VALUE
  INTC_FIM1 |= INTC_FIM1_VALUE;
    #elif INTC_FIM1_VALUE == 0
  INTC_FIM1 &= ~INTC_FIM1_MASK;
    #else
  INTC_FIM1 = (INTC_FIM1 & (~INTC_FIM1_MASK)) | INTC_FIM1_VALUE;
    #endif
  #elif defined(INTC_FIM1_VALUE)
  INTC_FIM1 = INTC_FIM1_VALUE;
  #endif

  /* Register 'INTC_CTRL' initialization */
  #if INTC_CTRL_MASK
    #if INTC_CTRL_MASK == 0xFFFF
  INTC_CTRL = INTC_CTRL_VALUE;
    #elif INTC_CTRL_MASK == INTC_CTRL_VALUE
  INTC_CTRL |= INTC_CTRL_VALUE;
    #elif INTC_CTRL_VALUE == 0
  INTC_CTRL &= ~INTC_CTRL_MASK;
    #else
  INTC_CTRL = (INTC_CTRL & (~INTC_CTRL_MASK)) | INTC_CTRL_VALUE;
    #endif
  #elif defined(INTC_CTRL_VALUE)
  INTC_CTRL = INTC_CTRL_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END INTC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
