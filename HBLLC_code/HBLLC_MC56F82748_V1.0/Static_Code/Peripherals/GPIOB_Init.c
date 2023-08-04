/** ###################################################################
 **     Filename    : GPIOB_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOB module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOB_Init(void);
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
 * @file GPIOB_Init.c
 * @brief This file implements the GPIOB module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOB_Init. */

#include "GPIOB_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOB_Init(void) {

  /* Register 'GPIOB_PUS' initialization */
  #if GPIOB_PUS_MASK
    #if GPIOB_PUS_MASK == 0xFFFF
  GPIOB_PUS = GPIOB_PUS_VALUE;
    #elif GPIOB_PUS_MASK == GPIOB_PUS_VALUE
  GPIOB_PUS |= GPIOB_PUS_VALUE;
    #elif GPIOB_PUS_VALUE == 0
  GPIOB_PUS &= ~GPIOB_PUS_MASK;
    #else
  GPIOB_PUS = (GPIOB_PUS & (~GPIOB_PUS_MASK)) | GPIOB_PUS_VALUE;
    #endif
  #elif defined(GPIOB_PUS_VALUE)
  GPIOB_PUS = GPIOB_PUS_VALUE;
  #endif

  /* Register 'GPIOB_PUR' initialization */
  #if GPIOB_PUR_MASK
    #if GPIOB_PUR_MASK == 0xFFFF
  GPIOB_PUR = GPIOB_PUR_VALUE;
    #elif GPIOB_PUR_MASK == GPIOB_PUR_VALUE
  GPIOB_PUR |= GPIOB_PUR_VALUE;
    #elif GPIOB_PUR_VALUE == 0
  GPIOB_PUR &= ~GPIOB_PUR_MASK;
    #else
  GPIOB_PUR = (GPIOB_PUR & (~GPIOB_PUR_MASK)) | GPIOB_PUR_VALUE;
    #endif
  #elif defined(GPIOB_PUR_VALUE)
  GPIOB_PUR = GPIOB_PUR_VALUE;
  #endif

  /* Register 'GPIOB_DRIVE' initialization */
  #if GPIOB_DRIVE_MASK
    #if GPIOB_DRIVE_MASK == 0xFFFF
  GPIOB_DRIVE = GPIOB_DRIVE_VALUE;
    #elif GPIOB_DRIVE_MASK == GPIOB_DRIVE_VALUE
  GPIOB_DRIVE |= GPIOB_DRIVE_VALUE;
    #elif GPIOB_DRIVE_VALUE == 0
  GPIOB_DRIVE &= ~GPIOB_DRIVE_MASK;
    #else
  GPIOB_DRIVE = (GPIOB_DRIVE & (~GPIOB_DRIVE_MASK)) | GPIOB_DRIVE_VALUE;
    #endif
  #elif defined(GPIOB_DRIVE_VALUE)
  GPIOB_DRIVE = GPIOB_DRIVE_VALUE;
  #endif

  /* Register 'GPIOB_SRE' initialization */
  #if GPIOB_SRE_MASK
    #if GPIOB_SRE_MASK == 0xFFFF
  GPIOB_SRE = GPIOB_SRE_VALUE;
    #elif GPIOB_SRE_MASK == GPIOB_SRE_VALUE
  GPIOB_SRE |= GPIOB_SRE_VALUE;
    #elif GPIOB_SRE_VALUE == 0
  GPIOB_SRE &= ~GPIOB_SRE_MASK;
    #else
  GPIOB_SRE = (GPIOB_SRE & (~GPIOB_SRE_MASK)) | GPIOB_SRE_VALUE;
    #endif
  #elif defined(GPIOB_SRE_VALUE)
  GPIOB_SRE = GPIOB_SRE_VALUE;
  #endif

  /* Register 'GPIOB_PPMODE' initialization */
  #if GPIOB_PPMODE_MASK
    #if GPIOB_PPMODE_MASK == 0xFFFF
  GPIOB_PPMODE = GPIOB_PPMODE_VALUE;
    #elif GPIOB_PPMODE_MASK == GPIOB_PPMODE_VALUE
  GPIOB_PPMODE |= GPIOB_PPMODE_VALUE;
    #elif GPIOB_PPMODE_VALUE == 0
  GPIOB_PPMODE &= ~GPIOB_PPMODE_MASK;
    #else
  GPIOB_PPMODE = (GPIOB_PPMODE & (~GPIOB_PPMODE_MASK)) | GPIOB_PPMODE_VALUE;
    #endif
  #elif defined(GPIOB_PPMODE_VALUE)
  GPIOB_PPMODE = GPIOB_PPMODE_VALUE;
  #endif

  /* Register 'GPIOB_DR' initialization */
  #if GPIOB_DR_MASK
    #if GPIOB_DR_MASK == 0xFFFF
  GPIOB_DR = GPIOB_DR_VALUE;
    #elif GPIOB_DR_MASK == GPIOB_DR_VALUE
  GPIOB_DR |= GPIOB_DR_VALUE;
    #elif GPIOB_DR_VALUE == 0
  GPIOB_DR &= ~GPIOB_DR_MASK;
    #else
  GPIOB_DR = (GPIOB_DR & (~GPIOB_DR_MASK)) | GPIOB_DR_VALUE;
    #endif
  #elif defined(GPIOB_DR_VALUE)
  GPIOB_DR = GPIOB_DR_VALUE;
  #endif

  /* Register 'GPIOB_DDR' initialization */
  #if GPIOB_DDR_MASK
    #if GPIOB_DDR_MASK == 0xFFFF
  GPIOB_DDR = GPIOB_DDR_VALUE;
    #elif GPIOB_DDR_MASK == GPIOB_DDR_VALUE
  GPIOB_DDR |= GPIOB_DDR_VALUE;
    #elif GPIOB_DDR_VALUE == 0
  GPIOB_DDR &= ~GPIOB_DDR_MASK;
    #else
  GPIOB_DDR = (GPIOB_DDR & (~GPIOB_DDR_MASK)) | GPIOB_DDR_VALUE;
    #endif
  #elif defined(GPIOB_DDR_VALUE)
  GPIOB_DDR = GPIOB_DDR_VALUE;
  #endif

  /* Register 'GPIOB_IPOLR' initialization */
  #if GPIOB_IPOLR_MASK
    #if GPIOB_IPOLR_MASK == 0xFFFF
  GPIOB_IPOLR = GPIOB_IPOLR_VALUE;
    #elif GPIOB_IPOLR_MASK == GPIOB_IPOLR_VALUE
  GPIOB_IPOLR |= GPIOB_IPOLR_VALUE;
    #elif GPIOB_IPOLR_VALUE == 0
  GPIOB_IPOLR &= ~GPIOB_IPOLR_MASK;
    #else
  GPIOB_IPOLR = (GPIOB_IPOLR & (~GPIOB_IPOLR_MASK)) | GPIOB_IPOLR_VALUE;
    #endif
  #elif defined(GPIOB_IPOLR_VALUE)
  GPIOB_IPOLR = GPIOB_IPOLR_VALUE;
  #endif

  /* Register 'GPIOB_PER' initialization */
  #if GPIOB_PER_MASK
    #if GPIOB_PER_MASK == 0xFFFF
  GPIOB_PER = GPIOB_PER_VALUE;
    #elif GPIOB_PER_MASK == GPIOB_PER_VALUE
  GPIOB_PER |= GPIOB_PER_VALUE;
    #elif GPIOB_PER_VALUE == 0
  GPIOB_PER &= ~GPIOB_PER_MASK;
    #else
  GPIOB_PER = (GPIOB_PER & (~GPIOB_PER_MASK)) | GPIOB_PER_VALUE;
    #endif
  #elif defined(GPIOB_PER_VALUE)
  GPIOB_PER = GPIOB_PER_VALUE;
  #endif

  /* Register 'GPIOB_IESR' initialization */
  #if GPIOB_IESR_MASK
    #if GPIOB_IESR_MASK == 0xFFFF
  GPIOB_IESR = GPIOB_IESR_VALUE;
    #elif GPIOB_IESR_MASK == GPIOB_IESR_VALUE
  GPIOB_IESR |= GPIOB_IESR_VALUE;
    #elif GPIOB_IESR_VALUE == 0
  GPIOB_IESR &= ~GPIOB_IESR_MASK;
    #else
  GPIOB_IESR = (GPIOB_IESR & (~GPIOB_IESR_MASK)) | GPIOB_IESR_VALUE;
    #endif
  #elif defined(GPIOB_IESR_VALUE)
  GPIOB_IESR = GPIOB_IESR_VALUE;
  #endif

  /* Register 'GPIOB_IENR' initialization */
  #if GPIOB_IENR_MASK
    #if GPIOB_IENR_MASK == 0xFFFF
  GPIOB_IENR = GPIOB_IENR_VALUE;
    #elif GPIOB_IENR_MASK == GPIOB_IENR_VALUE
  GPIOB_IENR |= GPIOB_IENR_VALUE;
    #elif GPIOB_IENR_VALUE == 0
  GPIOB_IENR &= ~GPIOB_IENR_MASK;
    #else
  GPIOB_IENR = (GPIOB_IENR & (~GPIOB_IENR_MASK)) | GPIOB_IENR_VALUE;
    #endif
  #elif defined(GPIOB_IENR_VALUE)
  GPIOB_IENR = GPIOB_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOB_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
