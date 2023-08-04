/** ###################################################################
 **     Filename    : GPIOF_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOF module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOF_Init(void);
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
 * @file GPIOF_Init.c
 * @brief This file implements the GPIOF module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOF_Init. */

#include "GPIOF_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOF_Init(void) {

  /* Register 'GPIOF_PUS' initialization */
  #if GPIOF_PUS_MASK
    #if GPIOF_PUS_MASK == 0xFFFF
  GPIOF_PUS = GPIOF_PUS_VALUE;
    #elif GPIOF_PUS_MASK == GPIOF_PUS_VALUE
  GPIOF_PUS |= GPIOF_PUS_VALUE;
    #elif GPIOF_PUS_VALUE == 0
  GPIOF_PUS &= ~GPIOF_PUS_MASK;
    #else
  GPIOF_PUS = (GPIOF_PUS & (~GPIOF_PUS_MASK)) | GPIOF_PUS_VALUE;
    #endif
  #elif defined(GPIOF_PUS_VALUE)
  GPIOF_PUS = GPIOF_PUS_VALUE;
  #endif

  /* Register 'GPIOF_PUR' initialization */
  #if GPIOF_PUR_MASK
    #if GPIOF_PUR_MASK == 0xFFFF
  GPIOF_PUR = GPIOF_PUR_VALUE;
    #elif GPIOF_PUR_MASK == GPIOF_PUR_VALUE
  GPIOF_PUR |= GPIOF_PUR_VALUE;
    #elif GPIOF_PUR_VALUE == 0
  GPIOF_PUR &= ~GPIOF_PUR_MASK;
    #else
  GPIOF_PUR = (GPIOF_PUR & (~GPIOF_PUR_MASK)) | GPIOF_PUR_VALUE;
    #endif
  #elif defined(GPIOF_PUR_VALUE)
  GPIOF_PUR = GPIOF_PUR_VALUE;
  #endif

  /* Register 'GPIOF_DRIVE' initialization */
  #if GPIOF_DRIVE_MASK
    #if GPIOF_DRIVE_MASK == 0xFFFF
  GPIOF_DRIVE = GPIOF_DRIVE_VALUE;
    #elif GPIOF_DRIVE_MASK == GPIOF_DRIVE_VALUE
  GPIOF_DRIVE |= GPIOF_DRIVE_VALUE;
    #elif GPIOF_DRIVE_VALUE == 0
  GPIOF_DRIVE &= ~GPIOF_DRIVE_MASK;
    #else
  GPIOF_DRIVE = (GPIOF_DRIVE & (~GPIOF_DRIVE_MASK)) | GPIOF_DRIVE_VALUE;
    #endif
  #elif defined(GPIOF_DRIVE_VALUE)
  GPIOF_DRIVE = GPIOF_DRIVE_VALUE;
  #endif

  /* Register 'GPIOF_SRE' initialization */
  #if GPIOF_SRE_MASK
    #if GPIOF_SRE_MASK == 0xFFFF
  GPIOF_SRE = GPIOF_SRE_VALUE;
    #elif GPIOF_SRE_MASK == GPIOF_SRE_VALUE
  GPIOF_SRE |= GPIOF_SRE_VALUE;
    #elif GPIOF_SRE_VALUE == 0
  GPIOF_SRE &= ~GPIOF_SRE_MASK;
    #else
  GPIOF_SRE = (GPIOF_SRE & (~GPIOF_SRE_MASK)) | GPIOF_SRE_VALUE;
    #endif
  #elif defined(GPIOF_SRE_VALUE)
  GPIOF_SRE = GPIOF_SRE_VALUE;
  #endif

  /* Register 'GPIOF_PPMODE' initialization */
  #if GPIOF_PPMODE_MASK
    #if GPIOF_PPMODE_MASK == 0xFFFF
  GPIOF_PPMODE = GPIOF_PPMODE_VALUE;
    #elif GPIOF_PPMODE_MASK == GPIOF_PPMODE_VALUE
  GPIOF_PPMODE |= GPIOF_PPMODE_VALUE;
    #elif GPIOF_PPMODE_VALUE == 0
  GPIOF_PPMODE &= ~GPIOF_PPMODE_MASK;
    #else
  GPIOF_PPMODE = (GPIOF_PPMODE & (~GPIOF_PPMODE_MASK)) | GPIOF_PPMODE_VALUE;
    #endif
  #elif defined(GPIOF_PPMODE_VALUE)
  GPIOF_PPMODE = GPIOF_PPMODE_VALUE;
  #endif

  /* Register 'GPIOF_DR' initialization */
  #if GPIOF_DR_MASK
    #if GPIOF_DR_MASK == 0xFFFF
  GPIOF_DR = GPIOF_DR_VALUE;
    #elif GPIOF_DR_MASK == GPIOF_DR_VALUE
  GPIOF_DR |= GPIOF_DR_VALUE;
    #elif GPIOF_DR_VALUE == 0
  GPIOF_DR &= ~GPIOF_DR_MASK;
    #else
  GPIOF_DR = (GPIOF_DR & (~GPIOF_DR_MASK)) | GPIOF_DR_VALUE;
    #endif
  #elif defined(GPIOF_DR_VALUE)
  GPIOF_DR = GPIOF_DR_VALUE;
  #endif

  /* Register 'GPIOF_DDR' initialization */
  #if GPIOF_DDR_MASK
    #if GPIOF_DDR_MASK == 0xFFFF
  GPIOF_DDR = GPIOF_DDR_VALUE;
    #elif GPIOF_DDR_MASK == GPIOF_DDR_VALUE
  GPIOF_DDR |= GPIOF_DDR_VALUE;
    #elif GPIOF_DDR_VALUE == 0
  GPIOF_DDR &= ~GPIOF_DDR_MASK;
    #else
  GPIOF_DDR = (GPIOF_DDR & (~GPIOF_DDR_MASK)) | GPIOF_DDR_VALUE;
    #endif
  #elif defined(GPIOF_DDR_VALUE)
  GPIOF_DDR = GPIOF_DDR_VALUE;
  #endif

  /* Register 'GPIOF_IPOLR' initialization */
  #if GPIOF_IPOLR_MASK
    #if GPIOF_IPOLR_MASK == 0xFFFF
  GPIOF_IPOLR = GPIOF_IPOLR_VALUE;
    #elif GPIOF_IPOLR_MASK == GPIOF_IPOLR_VALUE
  GPIOF_IPOLR |= GPIOF_IPOLR_VALUE;
    #elif GPIOF_IPOLR_VALUE == 0
  GPIOF_IPOLR &= ~GPIOF_IPOLR_MASK;
    #else
  GPIOF_IPOLR = (GPIOF_IPOLR & (~GPIOF_IPOLR_MASK)) | GPIOF_IPOLR_VALUE;
    #endif
  #elif defined(GPIOF_IPOLR_VALUE)
  GPIOF_IPOLR = GPIOF_IPOLR_VALUE;
  #endif

  /* Register 'GPIOF_PER' initialization */
  #if GPIOF_PER_MASK
    #if GPIOF_PER_MASK == 0xFFFF
  GPIOF_PER = GPIOF_PER_VALUE;
    #elif GPIOF_PER_MASK == GPIOF_PER_VALUE
  GPIOF_PER |= GPIOF_PER_VALUE;
    #elif GPIOF_PER_VALUE == 0
  GPIOF_PER &= ~GPIOF_PER_MASK;
    #else
  GPIOF_PER = (GPIOF_PER & (~GPIOF_PER_MASK)) | GPIOF_PER_VALUE;
    #endif
  #elif defined(GPIOF_PER_VALUE)
  GPIOF_PER = GPIOF_PER_VALUE;
  #endif

  /* Register 'GPIOF_IESR' initialization */
  #if GPIOF_IESR_MASK
    #if GPIOF_IESR_MASK == 0xFFFF
  GPIOF_IESR = GPIOF_IESR_VALUE;
    #elif GPIOF_IESR_MASK == GPIOF_IESR_VALUE
  GPIOF_IESR |= GPIOF_IESR_VALUE;
    #elif GPIOF_IESR_VALUE == 0
  GPIOF_IESR &= ~GPIOF_IESR_MASK;
    #else
  GPIOF_IESR = (GPIOF_IESR & (~GPIOF_IESR_MASK)) | GPIOF_IESR_VALUE;
    #endif
  #elif defined(GPIOF_IESR_VALUE)
  GPIOF_IESR = GPIOF_IESR_VALUE;
  #endif

  /* Register 'GPIOF_IENR' initialization */
  #if GPIOF_IENR_MASK
    #if GPIOF_IENR_MASK == 0xFFFF
  GPIOF_IENR = GPIOF_IENR_VALUE;
    #elif GPIOF_IENR_MASK == GPIOF_IENR_VALUE
  GPIOF_IENR |= GPIOF_IENR_VALUE;
    #elif GPIOF_IENR_VALUE == 0
  GPIOF_IENR &= ~GPIOF_IENR_MASK;
    #else
  GPIOF_IENR = (GPIOF_IENR & (~GPIOF_IENR_MASK)) | GPIOF_IENR_VALUE;
    #endif
  #elif defined(GPIOF_IENR_VALUE)
  GPIOF_IENR = GPIOF_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOF_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
