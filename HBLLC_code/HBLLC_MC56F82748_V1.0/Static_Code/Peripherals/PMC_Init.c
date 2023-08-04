/** ###################################################################
 **     Filename    : PMC_Init.c
 **     Processor   : MC56F82748VLH
 **     Version     : 1.0
 **     Abstract    :
 **          This file implements the PMC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PMC_Init(void);
 **
 **     Copyright : 1997 - 2013 Freescale Semiconductor, Inc. 
 **     SOURCE DISTRIBUTION PERMISSIBLE as directed in End User License Agreement.
 **     
 **     SPDX-License-Identifier: BSD-3-Clause
 **
 **     http      : www.freescale.com
 **     mail      : support@freescale.com
 ** ################################################################### */

/*!
 * @file PMC_Init.c
 * @version 1.0
 * @brief This file implements the PMC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PMC_Init. */

#include "PMC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PMC_Init(void) {

  /* Register 'PMC_STS' initialization */
  #ifdef PMC_STS_VALUE
  PMC_STS = PMC_STS_VALUE;
  #endif

  /* Register 'PMC_CTRL' initialization */
  #if PMC_CTRL_MASK
    #if PMC_CTRL_MASK == 0xFFFF
  PMC_CTRL = PMC_CTRL_VALUE;
    #elif PMC_CTRL_MASK == PMC_CTRL_VALUE
  PMC_CTRL |= PMC_CTRL_VALUE;
    #elif PMC_CTRL_VALUE == 0
  PMC_CTRL &= ~PMC_CTRL_MASK;
    #else
  PMC_CTRL = (PMC_CTRL & (~PMC_CTRL_MASK)) | PMC_CTRL_VALUE;
    #endif
  #elif defined(PMC_CTRL_VALUE)
  PMC_CTRL = PMC_CTRL_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PMC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for the Freescale 56800 series of microcontrollers.
 **
 ** ################################################################### */
