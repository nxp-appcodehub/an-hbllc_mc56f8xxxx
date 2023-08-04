/** ###################################################################
 **     Filename    : GPIOA_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOA_Init(void);
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
 * @file GPIOA_Init.c
 * @brief This file implements the GPIOA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOA_Init. */

#include "GPIOA_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOA_Init(void) {

  /* Register 'GPIOA_PUS' initialization */
  #if GPIOA_PUS_MASK
    #if GPIOA_PUS_MASK == 0xFFFF
  GPIOA_PUS = GPIOA_PUS_VALUE;
    #elif GPIOA_PUS_MASK == GPIOA_PUS_VALUE
  GPIOA_PUS |= GPIOA_PUS_VALUE;
    #elif GPIOA_PUS_VALUE == 0
  GPIOA_PUS &= ~GPIOA_PUS_MASK;
    #else
  GPIOA_PUS = (GPIOA_PUS & (~GPIOA_PUS_MASK)) | GPIOA_PUS_VALUE;
    #endif
  #elif defined(GPIOA_PUS_VALUE)
  GPIOA_PUS = GPIOA_PUS_VALUE;
  #endif

  /* Register 'GPIOA_PUR' initialization */
  #if GPIOA_PUR_MASK
    #if GPIOA_PUR_MASK == 0xFFFF
  GPIOA_PUR = GPIOA_PUR_VALUE;
    #elif GPIOA_PUR_MASK == GPIOA_PUR_VALUE
  GPIOA_PUR |= GPIOA_PUR_VALUE;
    #elif GPIOA_PUR_VALUE == 0
  GPIOA_PUR &= ~GPIOA_PUR_MASK;
    #else
  GPIOA_PUR = (GPIOA_PUR & (~GPIOA_PUR_MASK)) | GPIOA_PUR_VALUE;
    #endif
  #elif defined(GPIOA_PUR_VALUE)
  GPIOA_PUR = GPIOA_PUR_VALUE;
  #endif

  /* Register 'GPIOA_DRIVE' initialization */
  #if GPIOA_DRIVE_MASK
    #if GPIOA_DRIVE_MASK == 0xFFFF
  GPIOA_DRIVE = GPIOA_DRIVE_VALUE;
    #elif GPIOA_DRIVE_MASK == GPIOA_DRIVE_VALUE
  GPIOA_DRIVE |= GPIOA_DRIVE_VALUE;
    #elif GPIOA_DRIVE_VALUE == 0
  GPIOA_DRIVE &= ~GPIOA_DRIVE_MASK;
    #else
  GPIOA_DRIVE = (GPIOA_DRIVE & (~GPIOA_DRIVE_MASK)) | GPIOA_DRIVE_VALUE;
    #endif
  #elif defined(GPIOA_DRIVE_VALUE)
  GPIOA_DRIVE = GPIOA_DRIVE_VALUE;
  #endif

  /* Register 'GPIOA_SRE' initialization */
  #if GPIOA_SRE_MASK
    #if GPIOA_SRE_MASK == 0xFFFF
  GPIOA_SRE = GPIOA_SRE_VALUE;
    #elif GPIOA_SRE_MASK == GPIOA_SRE_VALUE
  GPIOA_SRE |= GPIOA_SRE_VALUE;
    #elif GPIOA_SRE_VALUE == 0
  GPIOA_SRE &= ~GPIOA_SRE_MASK;
    #else
  GPIOA_SRE = (GPIOA_SRE & (~GPIOA_SRE_MASK)) | GPIOA_SRE_VALUE;
    #endif
  #elif defined(GPIOA_SRE_VALUE)
  GPIOA_SRE = GPIOA_SRE_VALUE;
  #endif

  /* Register 'GPIOA_PPMODE' initialization */
  #if GPIOA_PPMODE_MASK
    #if GPIOA_PPMODE_MASK == 0xFFFF
  GPIOA_PPMODE = GPIOA_PPMODE_VALUE;
    #elif GPIOA_PPMODE_MASK == GPIOA_PPMODE_VALUE
  GPIOA_PPMODE |= GPIOA_PPMODE_VALUE;
    #elif GPIOA_PPMODE_VALUE == 0
  GPIOA_PPMODE &= ~GPIOA_PPMODE_MASK;
    #else
  GPIOA_PPMODE = (GPIOA_PPMODE & (~GPIOA_PPMODE_MASK)) | GPIOA_PPMODE_VALUE;
    #endif
  #elif defined(GPIOA_PPMODE_VALUE)
  GPIOA_PPMODE = GPIOA_PPMODE_VALUE;
  #endif

  /* Register 'GPIOA_DR' initialization */
  #if GPIOA_DR_MASK
    #if GPIOA_DR_MASK == 0xFFFF
  GPIOA_DR = GPIOA_DR_VALUE;
    #elif GPIOA_DR_MASK == GPIOA_DR_VALUE
  GPIOA_DR |= GPIOA_DR_VALUE;
    #elif GPIOA_DR_VALUE == 0
  GPIOA_DR &= ~GPIOA_DR_MASK;
    #else
  GPIOA_DR = (GPIOA_DR & (~GPIOA_DR_MASK)) | GPIOA_DR_VALUE;
    #endif
  #elif defined(GPIOA_DR_VALUE)
  GPIOA_DR = GPIOA_DR_VALUE;
  #endif

  /* Register 'GPIOA_DDR' initialization */
  #if GPIOA_DDR_MASK
    #if GPIOA_DDR_MASK == 0xFFFF
  GPIOA_DDR = GPIOA_DDR_VALUE;
    #elif GPIOA_DDR_MASK == GPIOA_DDR_VALUE
  GPIOA_DDR |= GPIOA_DDR_VALUE;
    #elif GPIOA_DDR_VALUE == 0
  GPIOA_DDR &= ~GPIOA_DDR_MASK;
    #else
  GPIOA_DDR = (GPIOA_DDR & (~GPIOA_DDR_MASK)) | GPIOA_DDR_VALUE;
    #endif
  #elif defined(GPIOA_DDR_VALUE)
  GPIOA_DDR = GPIOA_DDR_VALUE;
  #endif

  /* Register 'GPIOA_IPOLR' initialization */
  #if GPIOA_IPOLR_MASK
    #if GPIOA_IPOLR_MASK == 0xFFFF
  GPIOA_IPOLR = GPIOA_IPOLR_VALUE;
    #elif GPIOA_IPOLR_MASK == GPIOA_IPOLR_VALUE
  GPIOA_IPOLR |= GPIOA_IPOLR_VALUE;
    #elif GPIOA_IPOLR_VALUE == 0
  GPIOA_IPOLR &= ~GPIOA_IPOLR_MASK;
    #else
  GPIOA_IPOLR = (GPIOA_IPOLR & (~GPIOA_IPOLR_MASK)) | GPIOA_IPOLR_VALUE;
    #endif
  #elif defined(GPIOA_IPOLR_VALUE)
  GPIOA_IPOLR = GPIOA_IPOLR_VALUE;
  #endif

  /* Register 'GPIOA_PER' initialization */
  #if GPIOA_PER_MASK
    #if GPIOA_PER_MASK == 0xFFFF
  GPIOA_PER = GPIOA_PER_VALUE;
    #elif GPIOA_PER_MASK == GPIOA_PER_VALUE
  GPIOA_PER |= GPIOA_PER_VALUE;
    #elif GPIOA_PER_VALUE == 0
  GPIOA_PER &= ~GPIOA_PER_MASK;
    #else
  GPIOA_PER = (GPIOA_PER & (~GPIOA_PER_MASK)) | GPIOA_PER_VALUE;
    #endif
  #elif defined(GPIOA_PER_VALUE)
  GPIOA_PER = GPIOA_PER_VALUE;
  #endif

  /* Register 'GPIOA_IESR' initialization */
  #if GPIOA_IESR_MASK
    #if GPIOA_IESR_MASK == 0xFFFF
  GPIOA_IESR = GPIOA_IESR_VALUE;
    #elif GPIOA_IESR_MASK == GPIOA_IESR_VALUE
  GPIOA_IESR |= GPIOA_IESR_VALUE;
    #elif GPIOA_IESR_VALUE == 0
  GPIOA_IESR &= ~GPIOA_IESR_MASK;
    #else
  GPIOA_IESR = (GPIOA_IESR & (~GPIOA_IESR_MASK)) | GPIOA_IESR_VALUE;
    #endif
  #elif defined(GPIOA_IESR_VALUE)
  GPIOA_IESR = GPIOA_IESR_VALUE;
  #endif

  /* Register 'GPIOA_IENR' initialization */
  #if GPIOA_IENR_MASK
    #if GPIOA_IENR_MASK == 0xFFFF
  GPIOA_IENR = GPIOA_IENR_VALUE;
    #elif GPIOA_IENR_MASK == GPIOA_IENR_VALUE
  GPIOA_IENR |= GPIOA_IENR_VALUE;
    #elif GPIOA_IENR_VALUE == 0
  GPIOA_IENR &= ~GPIOA_IENR_MASK;
    #else
  GPIOA_IENR = (GPIOA_IENR & (~GPIOA_IENR_MASK)) | GPIOA_IENR_VALUE;
    #endif
  #elif defined(GPIOA_IENR_VALUE)
  GPIOA_IENR = GPIOA_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
