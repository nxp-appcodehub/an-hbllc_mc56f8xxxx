/** ###################################################################
 **     Filename    : COP_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the COP module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void COP_Init(void);
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
 * @file COP_Init.c
 * @brief This file implements the COP module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE COP_Init. */

#include "COP_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void COP_Init(void) {

  /* Register 'COP_CTRL' initialization */
  #ifdef COP_CTRL_VALUE_1
  COP_CTRL = COP_CTRL_VALUE_1;
  #endif

  /* Register 'COP_TOUT' initialization */
  #ifdef COP_TOUT_VALUE
  COP_TOUT = COP_TOUT_VALUE;
  #endif

  /* Register 'COP_INTVAL' initialization */
  #ifdef COP_INTVAL_VALUE
  COP_INTVAL = COP_INTVAL_VALUE;
  #endif

  /* Register 'COP_WINDOW' initialization */
  #ifdef COP_WINDOW_VALUE
  COP_WINDOW = COP_WINDOW_VALUE;
  #endif

  /* Register 'COP_CTRL' initialization */
  #if COP_CTRL_MASK_2
    #if COP_CTRL_MASK_2 == 0xFFFF
  COP_CTRL = COP_CTRL_VALUE_2;
    #elif COP_CTRL_MASK_2 == COP_CTRL_VALUE_2
  COP_CTRL |= COP_CTRL_VALUE_2;
    #elif COP_CTRL_VALUE_2 == 0
  COP_CTRL &= ~COP_CTRL_MASK_2;
    #else
  COP_CTRL = (COP_CTRL & (~COP_CTRL_MASK_2)) | COP_CTRL_VALUE_2;
    #endif
  #elif defined(COP_CTRL_VALUE_2)
  COP_CTRL = COP_CTRL_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END COP_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
