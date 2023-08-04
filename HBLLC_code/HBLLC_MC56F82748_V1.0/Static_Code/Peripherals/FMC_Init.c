/** ###################################################################
 **     Filename    : FMC_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the FMC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void FMC_Init(void);
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
 * @file FMC_Init.c
 * @brief This file implements the FMC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE FMC_Init. */

#include "FMC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void FMC_Init(void) {

  /* Register 'FMC_PFAPR' initialization */
  #if FMC_PFAPR_MASK
    #if FMC_PFAPR_MASK == 0xFFFFFFFF
  FMC_PFAPR = FMC_PFAPR_VALUE;
    #elif FMC_PFAPR_MASK == FMC_PFAPR_VALUE
  FMC_PFAPR |= FMC_PFAPR_VALUE;
    #elif FMC_PFAPR_VALUE == 0
  FMC_PFAPR &= ~FMC_PFAPR_MASK;
    #else
  FMC_PFAPR = (FMC_PFAPR & (~FMC_PFAPR_MASK)) | FMC_PFAPR_VALUE;
    #endif
  #elif defined(FMC_PFAPR_VALUE)
  FMC_PFAPR = FMC_PFAPR_VALUE;
  #endif

  /* Register 'FMC_PFB0CR' initialization */
  #if FMC_PFB0CR_MASK
    #if FMC_PFB0CR_MASK == 0xFFFFFFFF
  FMC_PFB0CR = FMC_PFB0CR_VALUE;
    #elif FMC_PFB0CR_MASK == FMC_PFB0CR_VALUE
  FMC_PFB0CR |= FMC_PFB0CR_VALUE;
    #elif FMC_PFB0CR_VALUE == 0
  FMC_PFB0CR &= ~FMC_PFB0CR_MASK;
    #else
  FMC_PFB0CR = (FMC_PFB0CR & (~FMC_PFB0CR_MASK)) | FMC_PFB0CR_VALUE;
    #endif
  #elif defined(FMC_PFB0CR_VALUE)
  FMC_PFB0CR = FMC_PFB0CR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END FMC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
