/** ###################################################################
 **     Filename    : XBARB_Init.c
 **     Processor   : MC56F82748VLH
 **     Version     : 1.0
 **     Abstract    :
 **          This file implements the XBARB module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void XBARB_Init(void);
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
 * @file XBARB_Init.c
 * @version 1.0
 * @brief This file implements the XBARB module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE XBARB_Init. */

#include "XBARB_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void XBARB_Init(void) {

  /* Register 'XBARB_SEL0' initialization */
  #if XBARB_SEL0_MASK
    #if XBARB_SEL0_MASK == 0xFFFF
  XBARB_SEL0 = XBARB_SEL0_VALUE;
    #elif XBARB_SEL0_MASK == XBARB_SEL0_VALUE
  XBARB_SEL0 |= XBARB_SEL0_VALUE;
    #elif XBARB_SEL0_VALUE == 0
  XBARB_SEL0 &= ~XBARB_SEL0_MASK;
    #else
  XBARB_SEL0 = (XBARB_SEL0 & (~XBARB_SEL0_MASK)) | XBARB_SEL0_VALUE;
    #endif
  #elif defined(XBARB_SEL0_VALUE)
  XBARB_SEL0 = XBARB_SEL0_VALUE;
  #endif

  /* Register 'XBARB_SEL1' initialization */
  #if XBARB_SEL1_MASK
    #if XBARB_SEL1_MASK == 0xFFFF
  XBARB_SEL1 = XBARB_SEL1_VALUE;
    #elif XBARB_SEL1_MASK == XBARB_SEL1_VALUE
  XBARB_SEL1 |= XBARB_SEL1_VALUE;
    #elif XBARB_SEL1_VALUE == 0
  XBARB_SEL1 &= ~XBARB_SEL1_MASK;
    #else
  XBARB_SEL1 = (XBARB_SEL1 & (~XBARB_SEL1_MASK)) | XBARB_SEL1_VALUE;
    #endif
  #elif defined(XBARB_SEL1_VALUE)
  XBARB_SEL1 = XBARB_SEL1_VALUE;
  #endif

  /* Register 'XBARB_SEL2' initialization */
  #if XBARB_SEL2_MASK
    #if XBARB_SEL2_MASK == 0xFFFF
  XBARB_SEL2 = XBARB_SEL2_VALUE;
    #elif XBARB_SEL2_MASK == XBARB_SEL2_VALUE
  XBARB_SEL2 |= XBARB_SEL2_VALUE;
    #elif XBARB_SEL2_VALUE == 0
  XBARB_SEL2 &= ~XBARB_SEL2_MASK;
    #else
  XBARB_SEL2 = (XBARB_SEL2 & (~XBARB_SEL2_MASK)) | XBARB_SEL2_VALUE;
    #endif
  #elif defined(XBARB_SEL2_VALUE)
  XBARB_SEL2 = XBARB_SEL2_VALUE;
  #endif

  /* Register 'XBARB_SEL3' initialization */
  #if XBARB_SEL3_MASK
    #if XBARB_SEL3_MASK == 0xFFFF
  XBARB_SEL3 = XBARB_SEL3_VALUE;
    #elif XBARB_SEL3_MASK == XBARB_SEL3_VALUE
  XBARB_SEL3 |= XBARB_SEL3_VALUE;
    #elif XBARB_SEL3_VALUE == 0
  XBARB_SEL3 &= ~XBARB_SEL3_MASK;
    #else
  XBARB_SEL3 = (XBARB_SEL3 & (~XBARB_SEL3_MASK)) | XBARB_SEL3_VALUE;
    #endif
  #elif defined(XBARB_SEL3_VALUE)
  XBARB_SEL3 = XBARB_SEL3_VALUE;
  #endif

  /* Register 'XBARB_SEL4' initialization */
  #if XBARB_SEL4_MASK
    #if XBARB_SEL4_MASK == 0xFFFF
  XBARB_SEL4 = XBARB_SEL4_VALUE;
    #elif XBARB_SEL4_MASK == XBARB_SEL4_VALUE
  XBARB_SEL4 |= XBARB_SEL4_VALUE;
    #elif XBARB_SEL4_VALUE == 0
  XBARB_SEL4 &= ~XBARB_SEL4_MASK;
    #else
  XBARB_SEL4 = (XBARB_SEL4 & (~XBARB_SEL4_MASK)) | XBARB_SEL4_VALUE;
    #endif
  #elif defined(XBARB_SEL4_VALUE)
  XBARB_SEL4 = XBARB_SEL4_VALUE;
  #endif

  /* Register 'XBARB_SEL5' initialization */
  #if XBARB_SEL5_MASK
    #if XBARB_SEL5_MASK == 0xFFFF
  XBARB_SEL5 = XBARB_SEL5_VALUE;
    #elif XBARB_SEL5_MASK == XBARB_SEL5_VALUE
  XBARB_SEL5 |= XBARB_SEL5_VALUE;
    #elif XBARB_SEL5_VALUE == 0
  XBARB_SEL5 &= ~XBARB_SEL5_MASK;
    #else
  XBARB_SEL5 = (XBARB_SEL5 & (~XBARB_SEL5_MASK)) | XBARB_SEL5_VALUE;
    #endif
  #elif defined(XBARB_SEL5_VALUE)
  XBARB_SEL5 = XBARB_SEL5_VALUE;
  #endif

  /* Register 'XBARB_SEL6' initialization */
  #if XBARB_SEL6_MASK
    #if XBARB_SEL6_MASK == 0xFFFF
  XBARB_SEL6 = XBARB_SEL6_VALUE;
    #elif XBARB_SEL6_MASK == XBARB_SEL6_VALUE
  XBARB_SEL6 |= XBARB_SEL6_VALUE;
    #elif XBARB_SEL6_VALUE == 0
  XBARB_SEL6 &= ~XBARB_SEL6_MASK;
    #else
  XBARB_SEL6 = (XBARB_SEL6 & (~XBARB_SEL6_MASK)) | XBARB_SEL6_VALUE;
    #endif
  #elif defined(XBARB_SEL6_VALUE)
  XBARB_SEL6 = XBARB_SEL6_VALUE;
  #endif

  /* Register 'XBARB_SEL7' initialization */
  #if XBARB_SEL7_MASK
    #if XBARB_SEL7_MASK == 0xFFFF
  XBARB_SEL7 = XBARB_SEL7_VALUE;
    #elif XBARB_SEL7_MASK == XBARB_SEL7_VALUE
  XBARB_SEL7 |= XBARB_SEL7_VALUE;
    #elif XBARB_SEL7_VALUE == 0
  XBARB_SEL7 &= ~XBARB_SEL7_MASK;
    #else
  XBARB_SEL7 = (XBARB_SEL7 & (~XBARB_SEL7_MASK)) | XBARB_SEL7_VALUE;
    #endif
  #elif defined(XBARB_SEL7_VALUE)
  XBARB_SEL7 = XBARB_SEL7_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END XBARB_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for the Freescale 56800 series of microcontrollers.
 **
 ** ################################################################### */
