/** ###################################################################
 **     Filename    : GPIOE_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the GPIOE module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void GPIOE_Init(void);
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
 * @file GPIOE_Init.c
 * @brief This file implements the GPIOE module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE GPIOE_Init. */

#include "GPIOE_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIOE_Init(void) {

  /* Register 'GPIOE_PUS' initialization */
  #if GPIOE_PUS_MASK
    #if GPIOE_PUS_MASK == 0xFFFF
  GPIOE_PUS = GPIOE_PUS_VALUE;
    #elif GPIOE_PUS_MASK == GPIOE_PUS_VALUE
  GPIOE_PUS |= GPIOE_PUS_VALUE;
    #elif GPIOE_PUS_VALUE == 0
  GPIOE_PUS &= ~GPIOE_PUS_MASK;
    #else
  GPIOE_PUS = (GPIOE_PUS & (~GPIOE_PUS_MASK)) | GPIOE_PUS_VALUE;
    #endif
  #elif defined(GPIOE_PUS_VALUE)
  GPIOE_PUS = GPIOE_PUS_VALUE;
  #endif

  /* Register 'GPIOE_PUR' initialization */
  #if GPIOE_PUR_MASK
    #if GPIOE_PUR_MASK == 0xFFFF
  GPIOE_PUR = GPIOE_PUR_VALUE;
    #elif GPIOE_PUR_MASK == GPIOE_PUR_VALUE
  GPIOE_PUR |= GPIOE_PUR_VALUE;
    #elif GPIOE_PUR_VALUE == 0
  GPIOE_PUR &= ~GPIOE_PUR_MASK;
    #else
  GPIOE_PUR = (GPIOE_PUR & (~GPIOE_PUR_MASK)) | GPIOE_PUR_VALUE;
    #endif
  #elif defined(GPIOE_PUR_VALUE)
  GPIOE_PUR = GPIOE_PUR_VALUE;
  #endif

  /* Register 'GPIOE_DRIVE' initialization */
  #if GPIOE_DRIVE_MASK
    #if GPIOE_DRIVE_MASK == 0xFFFF
  GPIOE_DRIVE = GPIOE_DRIVE_VALUE;
    #elif GPIOE_DRIVE_MASK == GPIOE_DRIVE_VALUE
  GPIOE_DRIVE |= GPIOE_DRIVE_VALUE;
    #elif GPIOE_DRIVE_VALUE == 0
  GPIOE_DRIVE &= ~GPIOE_DRIVE_MASK;
    #else
  GPIOE_DRIVE = (GPIOE_DRIVE & (~GPIOE_DRIVE_MASK)) | GPIOE_DRIVE_VALUE;
    #endif
  #elif defined(GPIOE_DRIVE_VALUE)
  GPIOE_DRIVE = GPIOE_DRIVE_VALUE;
  #endif

  /* Register 'GPIOE_SRE' initialization */
  #if GPIOE_SRE_MASK
    #if GPIOE_SRE_MASK == 0xFFFF
  GPIOE_SRE = GPIOE_SRE_VALUE;
    #elif GPIOE_SRE_MASK == GPIOE_SRE_VALUE
  GPIOE_SRE |= GPIOE_SRE_VALUE;
    #elif GPIOE_SRE_VALUE == 0
  GPIOE_SRE &= ~GPIOE_SRE_MASK;
    #else
  GPIOE_SRE = (GPIOE_SRE & (~GPIOE_SRE_MASK)) | GPIOE_SRE_VALUE;
    #endif
  #elif defined(GPIOE_SRE_VALUE)
  GPIOE_SRE = GPIOE_SRE_VALUE;
  #endif

  /* Register 'GPIOE_PPMODE' initialization */
  #if GPIOE_PPMODE_MASK
    #if GPIOE_PPMODE_MASK == 0xFFFF
  GPIOE_PPMODE = GPIOE_PPMODE_VALUE;
    #elif GPIOE_PPMODE_MASK == GPIOE_PPMODE_VALUE
  GPIOE_PPMODE |= GPIOE_PPMODE_VALUE;
    #elif GPIOE_PPMODE_VALUE == 0
  GPIOE_PPMODE &= ~GPIOE_PPMODE_MASK;
    #else
  GPIOE_PPMODE = (GPIOE_PPMODE & (~GPIOE_PPMODE_MASK)) | GPIOE_PPMODE_VALUE;
    #endif
  #elif defined(GPIOE_PPMODE_VALUE)
  GPIOE_PPMODE = GPIOE_PPMODE_VALUE;
  #endif

  /* Register 'GPIOE_DR' initialization */
  #if GPIOE_DR_MASK
    #if GPIOE_DR_MASK == 0xFFFF
  GPIOE_DR = GPIOE_DR_VALUE;
    #elif GPIOE_DR_MASK == GPIOE_DR_VALUE
  GPIOE_DR |= GPIOE_DR_VALUE;
    #elif GPIOE_DR_VALUE == 0
  GPIOE_DR &= ~GPIOE_DR_MASK;
    #else
  GPIOE_DR = (GPIOE_DR & (~GPIOE_DR_MASK)) | GPIOE_DR_VALUE;
    #endif
  #elif defined(GPIOE_DR_VALUE)
  GPIOE_DR = GPIOE_DR_VALUE;
  #endif

  /* Register 'GPIOE_DDR' initialization */
  #if GPIOE_DDR_MASK
    #if GPIOE_DDR_MASK == 0xFFFF
  GPIOE_DDR = GPIOE_DDR_VALUE;
    #elif GPIOE_DDR_MASK == GPIOE_DDR_VALUE
  GPIOE_DDR |= GPIOE_DDR_VALUE;
    #elif GPIOE_DDR_VALUE == 0
  GPIOE_DDR &= ~GPIOE_DDR_MASK;
    #else
  GPIOE_DDR = (GPIOE_DDR & (~GPIOE_DDR_MASK)) | GPIOE_DDR_VALUE;
    #endif
  #elif defined(GPIOE_DDR_VALUE)
  GPIOE_DDR = GPIOE_DDR_VALUE;
  #endif

  /* Register 'GPIOE_IPOLR' initialization */
  #if GPIOE_IPOLR_MASK
    #if GPIOE_IPOLR_MASK == 0xFFFF
  GPIOE_IPOLR = GPIOE_IPOLR_VALUE;
    #elif GPIOE_IPOLR_MASK == GPIOE_IPOLR_VALUE
  GPIOE_IPOLR |= GPIOE_IPOLR_VALUE;
    #elif GPIOE_IPOLR_VALUE == 0
  GPIOE_IPOLR &= ~GPIOE_IPOLR_MASK;
    #else
  GPIOE_IPOLR = (GPIOE_IPOLR & (~GPIOE_IPOLR_MASK)) | GPIOE_IPOLR_VALUE;
    #endif
  #elif defined(GPIOE_IPOLR_VALUE)
  GPIOE_IPOLR = GPIOE_IPOLR_VALUE;
  #endif

  /* Register 'GPIOE_PER' initialization */
  #if GPIOE_PER_MASK
    #if GPIOE_PER_MASK == 0xFFFF
  GPIOE_PER = GPIOE_PER_VALUE;
    #elif GPIOE_PER_MASK == GPIOE_PER_VALUE
  GPIOE_PER |= GPIOE_PER_VALUE;
    #elif GPIOE_PER_VALUE == 0
  GPIOE_PER &= ~GPIOE_PER_MASK;
    #else
  GPIOE_PER = (GPIOE_PER & (~GPIOE_PER_MASK)) | GPIOE_PER_VALUE;
    #endif
  #elif defined(GPIOE_PER_VALUE)
  GPIOE_PER = GPIOE_PER_VALUE;
  #endif

  /* Register 'GPIOE_IESR' initialization */
  #if GPIOE_IESR_MASK
    #if GPIOE_IESR_MASK == 0xFFFF
  GPIOE_IESR = GPIOE_IESR_VALUE;
    #elif GPIOE_IESR_MASK == GPIOE_IESR_VALUE
  GPIOE_IESR |= GPIOE_IESR_VALUE;
    #elif GPIOE_IESR_VALUE == 0
  GPIOE_IESR &= ~GPIOE_IESR_MASK;
    #else
  GPIOE_IESR = (GPIOE_IESR & (~GPIOE_IESR_MASK)) | GPIOE_IESR_VALUE;
    #endif
  #elif defined(GPIOE_IESR_VALUE)
  GPIOE_IESR = GPIOE_IESR_VALUE;
  #endif

  /* Register 'GPIOE_IENR' initialization */
  #if GPIOE_IENR_MASK
    #if GPIOE_IENR_MASK == 0xFFFF
  GPIOE_IENR = GPIOE_IENR_VALUE;
    #elif GPIOE_IENR_MASK == GPIOE_IENR_VALUE
  GPIOE_IENR |= GPIOE_IENR_VALUE;
    #elif GPIOE_IENR_VALUE == 0
  GPIOE_IENR &= ~GPIOE_IENR_MASK;
    #else
  GPIOE_IENR = (GPIOE_IENR & (~GPIOE_IENR_MASK)) | GPIOE_IENR_VALUE;
    #endif
  #elif defined(GPIOE_IENR_VALUE)
  GPIOE_IENR = GPIOE_IENR_VALUE;
  #endif

}

#ifdef __cplusplus
}
#endif

/* END GPIOE_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
