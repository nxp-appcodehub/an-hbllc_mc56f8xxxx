/** ###################################################################
 **     Filename    : MCM_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the MCM module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void MCM_Init(void);
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
 * @file MCM_Init.c
 * @brief This file implements the MCM module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE MCM_Init. */

#include "MCM_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void MCM_Init(void) {


  /* Register 'MCM_UPRAMBAR' initialization */
  #ifdef MCM_UPRAMBAR_VALUE
  MCM_UPRAMBAR = MCM_UPRAMBAR_VALUE;
  #endif

  /* Register 'MCM_UFLASHBAR' initialization */
  #ifdef MCM_UFLASHBAR_VALUE
  MCM_UFLASHBAR = MCM_UFLASHBAR_VALUE;
  #endif

  /* Register 'MCM_CPCR' initialization */
  #ifdef MCM_CPCR_VALUE
  MCM_CPCR = MCM_CPCR_VALUE;
  #endif

  /* Register 'MCM_CFISR' initialization */
  #ifdef MCM_CFISR_VALUE
  MCM_CFISR = MCM_CFISR_VALUE;
  #endif

  /* Register 'MCM_CFIER' initialization */
  #ifdef MCM_CFIER_VALUE
  MCM_CFIER = MCM_CFIER_VALUE;
  #endif
            /* Register 'MCM_SRPOSP' initialization */
  #ifdef MCM_SRPOSP_VALUE
  MCM_SRPOSP = MCM_SRPOSP_VALUE;
  #endif

  /* Register 'MCM_RPCR' initialization */
  #ifdef MCM_RPCR_VALUE
  MCM_RPCR = MCM_RPCR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END MCM_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
