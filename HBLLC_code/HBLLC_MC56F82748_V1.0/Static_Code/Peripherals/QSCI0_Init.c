/** ###################################################################
 **     Filename    : QSCI0_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the QSCI0 module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void QSCI0_Init(void);
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
 * @file QSCI0_Init.c
 * @brief This file implements the QSCI0 module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE QSCI0_Init. */

#include "QSCI0_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void QSCI0_Init(void) {

  QSCI0_CTRL1 &= ~0xC;
  /* Clear flags */
  #ifdef QSCI0_CLEAR_FLAGS
  QSCI0_STAT |= 0x10;
  #endif

  /* Register 'QSCI0_RATE' initialization */
  #ifdef QSCI0_RATE_VALUE
  QSCI0_RATE = QSCI0_RATE_VALUE;
  #endif

  /* Register 'QSCI0_CTRL3' initialization */
  #if QSCI0_CTRL3_MASK_1
    #if QSCI0_CTRL3_MASK_1 == 0xFFFF
  QSCI0_CTRL3 = QSCI0_CTRL3_VALUE_1;
    #elif QSCI0_CTRL3_MASK_1 == QSCI0_CTRL3_VALUE_1
  QSCI0_CTRL3 |= QSCI0_CTRL3_VALUE_1;
    #elif QSCI0_CTRL3_VALUE_1 == 0
  QSCI0_CTRL3 &= ~QSCI0_CTRL3_MASK_1;
    #else
  QSCI0_CTRL3 = (QSCI0_CTRL3 & (~QSCI0_CTRL3_MASK_1)) | QSCI0_CTRL3_VALUE_1;
    #endif
  #elif defined(QSCI0_CTRL3_VALUE_1)
  QSCI0_CTRL3 = QSCI0_CTRL3_VALUE_1;
  #endif

  /* Register 'QSCI0_CTRL1' initialization */
  #if QSCI0_CTRL1_MASK_1
    #if QSCI0_CTRL1_MASK_1 == 0xFFFF
  QSCI0_CTRL1 = QSCI0_CTRL1_VALUE_1;
    #elif QSCI0_CTRL1_MASK_1 == QSCI0_CTRL1_VALUE_1
  QSCI0_CTRL1 |= QSCI0_CTRL1_VALUE_1;
    #elif QSCI0_CTRL1_VALUE_1 == 0
  QSCI0_CTRL1 &= ~QSCI0_CTRL1_MASK_1;
    #else
  QSCI0_CTRL1 = (QSCI0_CTRL1 & (~QSCI0_CTRL1_MASK_1)) | QSCI0_CTRL1_VALUE_1;
    #endif
  #elif defined(QSCI0_CTRL1_VALUE_1)
  QSCI0_CTRL1 = QSCI0_CTRL1_VALUE_1;
  #endif

  /* Register 'QSCI0_CTRL2' initialization */
  #if QSCI0_CTRL2_MASK
    #if QSCI0_CTRL2_MASK == 0xFFFF
  QSCI0_CTRL2 = QSCI0_CTRL2_VALUE;
    #elif QSCI0_CTRL2_MASK == QSCI0_CTRL2_VALUE
  QSCI0_CTRL2 |= QSCI0_CTRL2_VALUE;
    #elif QSCI0_CTRL2_VALUE == 0
  QSCI0_CTRL2 &= ~QSCI0_CTRL2_MASK;
    #else
  QSCI0_CTRL2 = (QSCI0_CTRL2 & (~QSCI0_CTRL2_MASK)) | QSCI0_CTRL2_VALUE;
    #endif
  #elif defined(QSCI0_CTRL2_VALUE)
  QSCI0_CTRL2 = QSCI0_CTRL2_VALUE;
  #endif

  /* Register 'QSCI0_CTRL3' initialization */
  #if QSCI0_CTRL3_MASK_2
    #if QSCI0_CTRL3_MASK_2 == 0xFFFF
  QSCI0_CTRL3 = QSCI0_CTRL3_VALUE_2;
    #elif QSCI0_CTRL3_MASK_2 == QSCI0_CTRL3_VALUE_2
  QSCI0_CTRL3 |= QSCI0_CTRL3_VALUE_2;
    #elif QSCI0_CTRL3_VALUE_2 == 0
  QSCI0_CTRL3 &= ~QSCI0_CTRL3_MASK_2;
    #else
  QSCI0_CTRL3 = (QSCI0_CTRL3 & (~QSCI0_CTRL3_MASK_2)) | QSCI0_CTRL3_VALUE_2;
    #endif
  #elif defined(QSCI0_CTRL3_VALUE_2)
  QSCI0_CTRL3 = QSCI0_CTRL3_VALUE_2;
  #endif

  /* Register 'QSCI0_CTRL1' initialization */
  #if QSCI0_CTRL1_MASK_2
    #if QSCI0_CTRL1_MASK_2 == 0xFFFF
  QSCI0_CTRL1 = QSCI0_CTRL1_VALUE_2;
    #elif QSCI0_CTRL1_MASK_2 == QSCI0_CTRL1_VALUE_2
  QSCI0_CTRL1 |= QSCI0_CTRL1_VALUE_2;
    #elif QSCI0_CTRL1_VALUE_2 == 0
  QSCI0_CTRL1 &= ~QSCI0_CTRL1_MASK_2;
    #else
  QSCI0_CTRL1 = (QSCI0_CTRL1 & (~QSCI0_CTRL1_MASK_2)) | QSCI0_CTRL1_VALUE_2;
    #endif
  #elif defined(QSCI0_CTRL1_VALUE_2)
  QSCI0_CTRL1 = QSCI0_CTRL1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END QSCI0_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
