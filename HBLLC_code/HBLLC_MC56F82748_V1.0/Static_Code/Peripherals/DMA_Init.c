/** ###################################################################
 **     Filename    : DMA_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the DMA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void DMA_Init(void);
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
 * @file DMA_Init.c
 * @brief This file implements the DMA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE DMA_Init. */

#include "DMA_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void DMA_Init(void) {

  /* Register 'DMA_DSR_BCR0' initialization */
  #if DMA_DSR_BCR0_MASK_1
    #if DMA_DSR_BCR0_MASK_1 == 0xFFFFFFFF
  DMA_DSR_BCR0 = DMA_DSR_BCR0_VALUE_1;
    #elif DMA_DSR_BCR0_MASK_1 == DMA_DSR_BCR0_VALUE_1
  DMA_DSR_BCR0 |= DMA_DSR_BCR0_VALUE_1;
    #elif DMA_DSR_BCR0_VALUE_1 == 0
  DMA_DSR_BCR0 &= ~DMA_DSR_BCR0_MASK_1;
    #else
  DMA_DSR_BCR0 = (DMA_DSR_BCR0 & (~DMA_DSR_BCR0_MASK_1)) | DMA_DSR_BCR0_VALUE_1;
    #endif
  #elif defined(DMA_DSR_BCR0_VALUE_1)
  DMA_DSR_BCR0 = DMA_DSR_BCR0_VALUE_1;
  #endif

  /* Register 'DMA_DSR_BCR1' initialization */
  #if DMA_DSR_BCR1_MASK_1
    #if DMA_DSR_BCR1_MASK_1 == 0xFFFFFFFF
  DMA_DSR_BCR1 = DMA_DSR_BCR1_VALUE_1;
    #elif DMA_DSR_BCR1_MASK_1 == DMA_DSR_BCR1_VALUE_1
  DMA_DSR_BCR1 |= DMA_DSR_BCR1_VALUE_1;
    #elif DMA_DSR_BCR1_VALUE_1 == 0
  DMA_DSR_BCR1 &= ~DMA_DSR_BCR1_MASK_1;
    #else
  DMA_DSR_BCR1 = (DMA_DSR_BCR1 & (~DMA_DSR_BCR1_MASK_1)) | DMA_DSR_BCR1_VALUE_1;
    #endif
  #elif defined(DMA_DSR_BCR1_VALUE_1)
  DMA_DSR_BCR1 = DMA_DSR_BCR1_VALUE_1;
  #endif

  /* Register 'DMA_DSR_BCR2' initialization */
  #if DMA_DSR_BCR2_MASK_1
    #if DMA_DSR_BCR2_MASK_1 == 0xFFFFFFFF
  DMA_DSR_BCR2 = DMA_DSR_BCR2_VALUE_1;
    #elif DMA_DSR_BCR2_MASK_1 == DMA_DSR_BCR2_VALUE_1
  DMA_DSR_BCR2 |= DMA_DSR_BCR2_VALUE_1;
    #elif DMA_DSR_BCR2_VALUE_1 == 0
  DMA_DSR_BCR2 &= ~DMA_DSR_BCR2_MASK_1;
    #else
  DMA_DSR_BCR2 = (DMA_DSR_BCR2 & (~DMA_DSR_BCR2_MASK_1)) | DMA_DSR_BCR2_VALUE_1;
    #endif
  #elif defined(DMA_DSR_BCR2_VALUE_1)
  DMA_DSR_BCR2 = DMA_DSR_BCR2_VALUE_1;
  #endif

  /* Register 'DMA_DSR_BCR3' initialization */
  #if DMA_DSR_BCR3_MASK_1
    #if DMA_DSR_BCR3_MASK_1 == 0xFFFFFFFF
  DMA_DSR_BCR3 = DMA_DSR_BCR3_VALUE_1;
    #elif DMA_DSR_BCR3_MASK_1 == DMA_DSR_BCR3_VALUE_1
  DMA_DSR_BCR3 |= DMA_DSR_BCR3_VALUE_1;
    #elif DMA_DSR_BCR3_VALUE_1 == 0
  DMA_DSR_BCR3 &= ~DMA_DSR_BCR3_MASK_1;
    #else
  DMA_DSR_BCR3 = (DMA_DSR_BCR3 & (~DMA_DSR_BCR3_MASK_1)) | DMA_DSR_BCR3_VALUE_1;
    #endif
  #elif defined(DMA_DSR_BCR3_VALUE_1)
  DMA_DSR_BCR3 = DMA_DSR_BCR3_VALUE_1;
  #endif

  /* Register 'DMA_REQC' initialization */
  #if DMA_REQC_MASK_1
    #if DMA_REQC_MASK_1 == 0xFFFFFFFF
  DMA_REQC = DMA_REQC_VALUE_1;
    #elif DMA_REQC_MASK_1 == DMA_REQC_VALUE_1
  DMA_REQC |= DMA_REQC_VALUE_1;
    #elif DMA_REQC_VALUE_1 == 0
  DMA_REQC &= ~DMA_REQC_MASK_1;
    #else
  DMA_REQC = (DMA_REQC & (~DMA_REQC_MASK_1)) | DMA_REQC_VALUE_1;
    #endif
  #elif defined(DMA_REQC_VALUE_1)
  DMA_REQC = DMA_REQC_VALUE_1;
  #endif

  /* Register 'DMA_REQC' initialization */
  #if DMA_REQC_MASK_2
    #if DMA_REQC_MASK_2 == 0xFFFFFFFF
  DMA_REQC = DMA_REQC_VALUE_2;
    #elif DMA_REQC_MASK_2 == DMA_REQC_VALUE_2
  DMA_REQC |= DMA_REQC_VALUE_2;
    #elif DMA_REQC_VALUE_2 == 0
  DMA_REQC &= ~DMA_REQC_MASK_2;
    #else
  DMA_REQC = (DMA_REQC & (~DMA_REQC_MASK_2)) | DMA_REQC_VALUE_2;
    #endif
  #elif defined(DMA_REQC_VALUE_2)
  DMA_REQC = DMA_REQC_VALUE_2;
  #endif
  #ifdef DMA_SAR0_VALUE
  DMA_SAR0 = DMA_SAR0_VALUE;
  #endif
  #ifdef DMA_SAR1_VALUE
  DMA_SAR1 = DMA_SAR1_VALUE;
  #endif
  #ifdef DMA_SAR2_VALUE
  DMA_SAR2 = DMA_SAR2_VALUE;
  #endif
  #ifdef DMA_SAR3_VALUE
  DMA_SAR3 = DMA_SAR3_VALUE;
  #endif
  #ifdef DMA_DAR0_VALUE
  DMA_DAR0 = DMA_DAR0_VALUE;
  #endif
  #ifdef DMA_DAR1_VALUE
  DMA_DAR1 = DMA_DAR1_VALUE;
  #endif
  #ifdef DMA_DAR2_VALUE
  DMA_DAR2 = DMA_DAR2_VALUE;
  #endif
  #ifdef DMA_DAR3_VALUE
  DMA_DAR3 = DMA_DAR3_VALUE;
  #endif

  /* Register 'DMA_DSR_BCR0' initialization */
  #ifdef DMA_DSR_BCR0_VALUE_2
  DMA_DSR_BCR0 = DMA_DSR_BCR0_VALUE_2;
  #endif

  /* Register 'DMA_DSR_BCR1' initialization */
  #ifdef DMA_DSR_BCR1_VALUE_2
  DMA_DSR_BCR1 = DMA_DSR_BCR1_VALUE_2;
  #endif

  /* Register 'DMA_DSR_BCR2' initialization */
  #ifdef DMA_DSR_BCR2_VALUE_2
  DMA_DSR_BCR2 = DMA_DSR_BCR2_VALUE_2;
  #endif

  /* Register 'DMA_DSR_BCR3' initialization */
  #ifdef DMA_DSR_BCR3_VALUE_2
  DMA_DSR_BCR3 = DMA_DSR_BCR3_VALUE_2;
  #endif

  /* Register 'DMA_DCR0' initialization */
  #ifdef DMA_DCR0_VALUE
  DMA_DCR0 = DMA_DCR0_VALUE;
  #endif

  /* Register 'DMA_DCR1' initialization */
  #ifdef DMA_DCR1_VALUE
  DMA_DCR1 = DMA_DCR1_VALUE;
  #endif

  /* Register 'DMA_DCR2' initialization */
  #ifdef DMA_DCR2_VALUE
  DMA_DCR2 = DMA_DCR2_VALUE;
  #endif

  /* Register 'DMA_DCR3' initialization */
  #ifdef DMA_DCR3_VALUE
  DMA_DCR3 = DMA_DCR3_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END DMA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
