/** ###################################################################
 **     Filename    : GPIOC_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOC_Init(void);
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
 * @file GPIOC_Init.c
 * @brief This file implements the GPIOC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOC_Init. */

#include "GPIOC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOC_Init(void) {

  /* Register 'GPIOC_PUS' initialization */
  #if GPIOC_PUS_MASK
    #if GPIOC_PUS_MASK == 0xFFFF
  GPIOC_PUS = GPIOC_PUS_VALUE;
    #elif GPIOC_PUS_MASK == GPIOC_PUS_VALUE
  GPIOC_PUS |= GPIOC_PUS_VALUE;
    #elif GPIOC_PUS_VALUE == 0
  GPIOC_PUS &= ~GPIOC_PUS_MASK;
    #else
  GPIOC_PUS = (GPIOC_PUS & (~GPIOC_PUS_MASK)) | GPIOC_PUS_VALUE;
    #endif
  #elif defined(GPIOC_PUS_VALUE)
  GPIOC_PUS = GPIOC_PUS_VALUE;
  #endif

  /* Register 'GPIOC_PUR' initialization */
  #if GPIOC_PUR_MASK
    #if GPIOC_PUR_MASK == 0xFFFF
  GPIOC_PUR = GPIOC_PUR_VALUE;
    #elif GPIOC_PUR_MASK == GPIOC_PUR_VALUE
  GPIOC_PUR |= GPIOC_PUR_VALUE;
    #elif GPIOC_PUR_VALUE == 0
  GPIOC_PUR &= ~GPIOC_PUR_MASK;
    #else
  GPIOC_PUR = (GPIOC_PUR & (~GPIOC_PUR_MASK)) | GPIOC_PUR_VALUE;
    #endif
  #elif defined(GPIOC_PUR_VALUE)
  GPIOC_PUR = GPIOC_PUR_VALUE;
  #endif

  /* Register 'GPIOC_DRIVE' initialization */
  #if GPIOC_DRIVE_MASK
    #if GPIOC_DRIVE_MASK == 0xFFFF
  GPIOC_DRIVE = GPIOC_DRIVE_VALUE;
    #elif GPIOC_DRIVE_MASK == GPIOC_DRIVE_VALUE
  GPIOC_DRIVE |= GPIOC_DRIVE_VALUE;
    #elif GPIOC_DRIVE_VALUE == 0
  GPIOC_DRIVE &= ~GPIOC_DRIVE_MASK;
    #else
  GPIOC_DRIVE = (GPIOC_DRIVE & (~GPIOC_DRIVE_MASK)) | GPIOC_DRIVE_VALUE;
    #endif
  #elif defined(GPIOC_DRIVE_VALUE)
  GPIOC_DRIVE = GPIOC_DRIVE_VALUE;
  #endif

  /* Register 'GPIOC_SRE' initialization */
  #if GPIOC_SRE_MASK
    #if GPIOC_SRE_MASK == 0xFFFF
  GPIOC_SRE = GPIOC_SRE_VALUE;
    #elif GPIOC_SRE_MASK == GPIOC_SRE_VALUE
  GPIOC_SRE |= GPIOC_SRE_VALUE;
    #elif GPIOC_SRE_VALUE == 0
  GPIOC_SRE &= ~GPIOC_SRE_MASK;
    #else
  GPIOC_SRE = (GPIOC_SRE & (~GPIOC_SRE_MASK)) | GPIOC_SRE_VALUE;
    #endif
  #elif defined(GPIOC_SRE_VALUE)
  GPIOC_SRE = GPIOC_SRE_VALUE;
  #endif

  /* Register 'GPIOC_PPMODE' initialization */
  #if GPIOC_PPMODE_MASK
    #if GPIOC_PPMODE_MASK == 0xFFFF
  GPIOC_PPMODE = GPIOC_PPMODE_VALUE;
    #elif GPIOC_PPMODE_MASK == GPIOC_PPMODE_VALUE
  GPIOC_PPMODE |= GPIOC_PPMODE_VALUE;
    #elif GPIOC_PPMODE_VALUE == 0
  GPIOC_PPMODE &= ~GPIOC_PPMODE_MASK;
    #else
  GPIOC_PPMODE = (GPIOC_PPMODE & (~GPIOC_PPMODE_MASK)) | GPIOC_PPMODE_VALUE;
    #endif
  #elif defined(GPIOC_PPMODE_VALUE)
  GPIOC_PPMODE = GPIOC_PPMODE_VALUE;
  #endif

  /* Register 'GPIOC_DR' initialization */
  #if GPIOC_DR_MASK
    #if GPIOC_DR_MASK == 0xFFFF
  GPIOC_DR = GPIOC_DR_VALUE;
    #elif GPIOC_DR_MASK == GPIOC_DR_VALUE
  GPIOC_DR |= GPIOC_DR_VALUE;
    #elif GPIOC_DR_VALUE == 0
  GPIOC_DR &= ~GPIOC_DR_MASK;
    #else
  GPIOC_DR = (GPIOC_DR & (~GPIOC_DR_MASK)) | GPIOC_DR_VALUE;
    #endif
  #elif defined(GPIOC_DR_VALUE)
  GPIOC_DR = GPIOC_DR_VALUE;
  #endif

  /* Register 'GPIOC_DDR' initialization */
  #if GPIOC_DDR_MASK
    #if GPIOC_DDR_MASK == 0xFFFF
  GPIOC_DDR = GPIOC_DDR_VALUE;
    #elif GPIOC_DDR_MASK == GPIOC_DDR_VALUE
  GPIOC_DDR |= GPIOC_DDR_VALUE;
    #elif GPIOC_DDR_VALUE == 0
  GPIOC_DDR &= ~GPIOC_DDR_MASK;
    #else
  GPIOC_DDR = (GPIOC_DDR & (~GPIOC_DDR_MASK)) | GPIOC_DDR_VALUE;
    #endif
  #elif defined(GPIOC_DDR_VALUE)
  GPIOC_DDR = GPIOC_DDR_VALUE;
  #endif

  /* Register 'GPIOC_IPOLR' initialization */
  #if GPIOC_IPOLR_MASK
    #if GPIOC_IPOLR_MASK == 0xFFFF
  GPIOC_IPOLR = GPIOC_IPOLR_VALUE;
    #elif GPIOC_IPOLR_MASK == GPIOC_IPOLR_VALUE
  GPIOC_IPOLR |= GPIOC_IPOLR_VALUE;
    #elif GPIOC_IPOLR_VALUE == 0
  GPIOC_IPOLR &= ~GPIOC_IPOLR_MASK;
    #else
  GPIOC_IPOLR = (GPIOC_IPOLR & (~GPIOC_IPOLR_MASK)) | GPIOC_IPOLR_VALUE;
    #endif
  #elif defined(GPIOC_IPOLR_VALUE)
  GPIOC_IPOLR = GPIOC_IPOLR_VALUE;
  #endif

  /* Register 'GPIOC_PER' initialization */
  #if GPIOC_PER_MASK
    #if GPIOC_PER_MASK == 0xFFFF
  GPIOC_PER = GPIOC_PER_VALUE;
    #elif GPIOC_PER_MASK == GPIOC_PER_VALUE
  GPIOC_PER |= GPIOC_PER_VALUE;
    #elif GPIOC_PER_VALUE == 0
  GPIOC_PER &= ~GPIOC_PER_MASK;
    #else
  GPIOC_PER = (GPIOC_PER & (~GPIOC_PER_MASK)) | GPIOC_PER_VALUE;
    #endif
  #elif defined(GPIOC_PER_VALUE)
  GPIOC_PER = GPIOC_PER_VALUE;
  #endif

  /* Register 'GPIOC_IESR' initialization */
  #if GPIOC_IESR_MASK
    #if GPIOC_IESR_MASK == 0xFFFF
  GPIOC_IESR = GPIOC_IESR_VALUE;
    #elif GPIOC_IESR_MASK == GPIOC_IESR_VALUE
  GPIOC_IESR |= GPIOC_IESR_VALUE;
    #elif GPIOC_IESR_VALUE == 0
  GPIOC_IESR &= ~GPIOC_IESR_MASK;
    #else
  GPIOC_IESR = (GPIOC_IESR & (~GPIOC_IESR_MASK)) | GPIOC_IESR_VALUE;
    #endif
  #elif defined(GPIOC_IESR_VALUE)
  GPIOC_IESR = GPIOC_IESR_VALUE;
  #endif

  /* Register 'GPIOC_IENR' initialization */
  #if GPIOC_IENR_MASK
    #if GPIOC_IENR_MASK == 0xFFFF
  GPIOC_IENR = GPIOC_IENR_VALUE;
    #elif GPIOC_IENR_MASK == GPIOC_IENR_VALUE
  GPIOC_IENR |= GPIOC_IENR_VALUE;
    #elif GPIOC_IENR_VALUE == 0
  GPIOC_IENR &= ~GPIOC_IENR_MASK;
    #else
  GPIOC_IENR = (GPIOC_IENR & (~GPIOC_IENR_MASK)) | GPIOC_IENR_VALUE;
    #endif
  #elif defined(GPIOC_IENR_VALUE)
  GPIOC_IENR = GPIOC_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
