/** ###################################################################
 **     Filename    : GPIOD_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOD module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOD_Init(void);
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
 * @file GPIOD_Init.c
 * @brief This file implements the GPIOD module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOD_Init. */

#include "GPIOD_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOD_Init(void) {

  /* Register 'GPIOD_PUS' initialization */
  #if GPIOD_PUS_MASK
    #if GPIOD_PUS_MASK == 0xFFFF
  GPIOD_PUS = GPIOD_PUS_VALUE;
    #elif GPIOD_PUS_MASK == GPIOD_PUS_VALUE
  GPIOD_PUS |= GPIOD_PUS_VALUE;
    #elif GPIOD_PUS_VALUE == 0
  GPIOD_PUS &= ~GPIOD_PUS_MASK;
    #else
  GPIOD_PUS = (GPIOD_PUS & (~GPIOD_PUS_MASK)) | GPIOD_PUS_VALUE;
    #endif
  #elif defined(GPIOD_PUS_VALUE)
  GPIOD_PUS = GPIOD_PUS_VALUE;
  #endif

  /* Register 'GPIOD_PUR' initialization */
  #if GPIOD_PUR_MASK
    #if GPIOD_PUR_MASK == 0xFFFF
  GPIOD_PUR = GPIOD_PUR_VALUE;
    #elif GPIOD_PUR_MASK == GPIOD_PUR_VALUE
  GPIOD_PUR |= GPIOD_PUR_VALUE;
    #elif GPIOD_PUR_VALUE == 0
  GPIOD_PUR &= ~GPIOD_PUR_MASK;
    #else
  GPIOD_PUR = (GPIOD_PUR & (~GPIOD_PUR_MASK)) | GPIOD_PUR_VALUE;
    #endif
  #elif defined(GPIOD_PUR_VALUE)
  GPIOD_PUR = GPIOD_PUR_VALUE;
  #endif

  /* Register 'GPIOD_DRIVE' initialization */
  #if GPIOD_DRIVE_MASK
    #if GPIOD_DRIVE_MASK == 0xFFFF
  GPIOD_DRIVE = GPIOD_DRIVE_VALUE;
    #elif GPIOD_DRIVE_MASK == GPIOD_DRIVE_VALUE
  GPIOD_DRIVE |= GPIOD_DRIVE_VALUE;
    #elif GPIOD_DRIVE_VALUE == 0
  GPIOD_DRIVE &= ~GPIOD_DRIVE_MASK;
    #else
  GPIOD_DRIVE = (GPIOD_DRIVE & (~GPIOD_DRIVE_MASK)) | GPIOD_DRIVE_VALUE;
    #endif
  #elif defined(GPIOD_DRIVE_VALUE)
  GPIOD_DRIVE = GPIOD_DRIVE_VALUE;
  #endif

  /* Register 'GPIOD_SRE' initialization */
  #if GPIOD_SRE_MASK
    #if GPIOD_SRE_MASK == 0xFFFF
  GPIOD_SRE = GPIOD_SRE_VALUE;
    #elif GPIOD_SRE_MASK == GPIOD_SRE_VALUE
  GPIOD_SRE |= GPIOD_SRE_VALUE;
    #elif GPIOD_SRE_VALUE == 0
  GPIOD_SRE &= ~GPIOD_SRE_MASK;
    #else
  GPIOD_SRE = (GPIOD_SRE & (~GPIOD_SRE_MASK)) | GPIOD_SRE_VALUE;
    #endif
  #elif defined(GPIOD_SRE_VALUE)
  GPIOD_SRE = GPIOD_SRE_VALUE;
  #endif

  /* Register 'GPIOD_PPMODE' initialization */
  #if GPIOD_PPMODE_MASK
    #if GPIOD_PPMODE_MASK == 0xFFFF
  GPIOD_PPMODE = GPIOD_PPMODE_VALUE;
    #elif GPIOD_PPMODE_MASK == GPIOD_PPMODE_VALUE
  GPIOD_PPMODE |= GPIOD_PPMODE_VALUE;
    #elif GPIOD_PPMODE_VALUE == 0
  GPIOD_PPMODE &= ~GPIOD_PPMODE_MASK;
    #else
  GPIOD_PPMODE = (GPIOD_PPMODE & (~GPIOD_PPMODE_MASK)) | GPIOD_PPMODE_VALUE;
    #endif
  #elif defined(GPIOD_PPMODE_VALUE)
  GPIOD_PPMODE = GPIOD_PPMODE_VALUE;
  #endif

  /* Register 'GPIOD_DR' initialization */
  #if GPIOD_DR_MASK
    #if GPIOD_DR_MASK == 0xFFFF
  GPIOD_DR = GPIOD_DR_VALUE;
    #elif GPIOD_DR_MASK == GPIOD_DR_VALUE
  GPIOD_DR |= GPIOD_DR_VALUE;
    #elif GPIOD_DR_VALUE == 0
  GPIOD_DR &= ~GPIOD_DR_MASK;
    #else
  GPIOD_DR = (GPIOD_DR & (~GPIOD_DR_MASK)) | GPIOD_DR_VALUE;
    #endif
  #elif defined(GPIOD_DR_VALUE)
  GPIOD_DR = GPIOD_DR_VALUE;
  #endif

  /* Register 'GPIOD_DDR' initialization */
  #if GPIOD_DDR_MASK
    #if GPIOD_DDR_MASK == 0xFFFF
  GPIOD_DDR = GPIOD_DDR_VALUE;
    #elif GPIOD_DDR_MASK == GPIOD_DDR_VALUE
  GPIOD_DDR |= GPIOD_DDR_VALUE;
    #elif GPIOD_DDR_VALUE == 0
  GPIOD_DDR &= ~GPIOD_DDR_MASK;
    #else
  GPIOD_DDR = (GPIOD_DDR & (~GPIOD_DDR_MASK)) | GPIOD_DDR_VALUE;
    #endif
  #elif defined(GPIOD_DDR_VALUE)
  GPIOD_DDR = GPIOD_DDR_VALUE;
  #endif

  /* Register 'GPIOD_IPOLR' initialization */
  #if GPIOD_IPOLR_MASK
    #if GPIOD_IPOLR_MASK == 0xFFFF
  GPIOD_IPOLR = GPIOD_IPOLR_VALUE;
    #elif GPIOD_IPOLR_MASK == GPIOD_IPOLR_VALUE
  GPIOD_IPOLR |= GPIOD_IPOLR_VALUE;
    #elif GPIOD_IPOLR_VALUE == 0
  GPIOD_IPOLR &= ~GPIOD_IPOLR_MASK;
    #else
  GPIOD_IPOLR = (GPIOD_IPOLR & (~GPIOD_IPOLR_MASK)) | GPIOD_IPOLR_VALUE;
    #endif
  #elif defined(GPIOD_IPOLR_VALUE)
  GPIOD_IPOLR = GPIOD_IPOLR_VALUE;
  #endif

  /* Register 'GPIOD_PER' initialization */
  #if GPIOD_PER_MASK
    #if GPIOD_PER_MASK == 0xFFFF
  GPIOD_PER = GPIOD_PER_VALUE;
    #elif GPIOD_PER_MASK == GPIOD_PER_VALUE
  GPIOD_PER |= GPIOD_PER_VALUE;
    #elif GPIOD_PER_VALUE == 0
  GPIOD_PER &= ~GPIOD_PER_MASK;
    #else
  GPIOD_PER = (GPIOD_PER & (~GPIOD_PER_MASK)) | GPIOD_PER_VALUE;
    #endif
  #elif defined(GPIOD_PER_VALUE)
  GPIOD_PER = GPIOD_PER_VALUE;
  #endif

  /* Register 'GPIOD_IESR' initialization */
  #if GPIOD_IESR_MASK
    #if GPIOD_IESR_MASK == 0xFFFF
  GPIOD_IESR = GPIOD_IESR_VALUE;
    #elif GPIOD_IESR_MASK == GPIOD_IESR_VALUE
  GPIOD_IESR |= GPIOD_IESR_VALUE;
    #elif GPIOD_IESR_VALUE == 0
  GPIOD_IESR &= ~GPIOD_IESR_MASK;
    #else
  GPIOD_IESR = (GPIOD_IESR & (~GPIOD_IESR_MASK)) | GPIOD_IESR_VALUE;
    #endif
  #elif defined(GPIOD_IESR_VALUE)
  GPIOD_IESR = GPIOD_IESR_VALUE;
  #endif

  /* Register 'GPIOD_IENR' initialization */
  #if GPIOD_IENR_MASK
    #if GPIOD_IENR_MASK == 0xFFFF
  GPIOD_IENR = GPIOD_IENR_VALUE;
    #elif GPIOD_IENR_MASK == GPIOD_IENR_VALUE
  GPIOD_IENR |= GPIOD_IENR_VALUE;
    #elif GPIOD_IENR_VALUE == 0
  GPIOD_IENR &= ~GPIOD_IENR_MASK;
    #else
  GPIOD_IENR = (GPIOD_IENR & (~GPIOD_IENR_MASK)) | GPIOD_IENR_VALUE;
    #endif
  #elif defined(GPIOD_IENR_VALUE)
  GPIOD_IENR = GPIOD_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOD_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
