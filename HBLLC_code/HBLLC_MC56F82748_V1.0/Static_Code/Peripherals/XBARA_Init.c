/** ###################################################################
 **     Filename    : XBARA_Init.c
 **     Processor   : MC56F82748VLH
 **     Version     : 1.0
 **     Abstract    :
 **          This file implements the XBARA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void XBARA_Init(void);
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
 * @file XBARA_Init.c
 * @version 1.0
 * @brief This file implements the XBARA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE XBARA_Init. */

#include "XBARA_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void XBARA_Init(void) {

  /* Register 'XBARA_SEL0' initialization */
  #if XBARA_SEL0_MASK
    #if XBARA_SEL0_MASK == 0xFFFF
  XBARA_SEL0 = XBARA_SEL0_VALUE;
    #elif XBARA_SEL0_MASK == XBARA_SEL0_VALUE
  XBARA_SEL0 |= XBARA_SEL0_VALUE;
    #elif XBARA_SEL0_VALUE == 0
  XBARA_SEL0 &= ~XBARA_SEL0_MASK;
    #else
  XBARA_SEL0 = (XBARA_SEL0 & (~XBARA_SEL0_MASK)) | XBARA_SEL0_VALUE;
    #endif
  #elif defined(XBARA_SEL0_VALUE)
  XBARA_SEL0 = XBARA_SEL0_VALUE;
  #endif

  /* Register 'XBARA_SEL1' initialization */
  #if XBARA_SEL1_MASK
    #if XBARA_SEL1_MASK == 0xFFFF
  XBARA_SEL1 = XBARA_SEL1_VALUE;
    #elif XBARA_SEL1_MASK == XBARA_SEL1_VALUE
  XBARA_SEL1 |= XBARA_SEL1_VALUE;
    #elif XBARA_SEL1_VALUE == 0
  XBARA_SEL1 &= ~XBARA_SEL1_MASK;
    #else
  XBARA_SEL1 = (XBARA_SEL1 & (~XBARA_SEL1_MASK)) | XBARA_SEL1_VALUE;
    #endif
  #elif defined(XBARA_SEL1_VALUE)
  XBARA_SEL1 = XBARA_SEL1_VALUE;
  #endif

  /* Register 'XBARA_SEL2' initialization */
  #if XBARA_SEL2_MASK
    #if XBARA_SEL2_MASK == 0xFFFF
  XBARA_SEL2 = XBARA_SEL2_VALUE;
    #elif XBARA_SEL2_MASK == XBARA_SEL2_VALUE
  XBARA_SEL2 |= XBARA_SEL2_VALUE;
    #elif XBARA_SEL2_VALUE == 0
  XBARA_SEL2 &= ~XBARA_SEL2_MASK;
    #else
  XBARA_SEL2 = (XBARA_SEL2 & (~XBARA_SEL2_MASK)) | XBARA_SEL2_VALUE;
    #endif
  #elif defined(XBARA_SEL2_VALUE)
  XBARA_SEL2 = XBARA_SEL2_VALUE;
  #endif

  /* Register 'XBARA_SEL3' initialization */
  #if XBARA_SEL3_MASK
    #if XBARA_SEL3_MASK == 0xFFFF
  XBARA_SEL3 = XBARA_SEL3_VALUE;
    #elif XBARA_SEL3_MASK == XBARA_SEL3_VALUE
  XBARA_SEL3 |= XBARA_SEL3_VALUE;
    #elif XBARA_SEL3_VALUE == 0
  XBARA_SEL3 &= ~XBARA_SEL3_MASK;
    #else
  XBARA_SEL3 = (XBARA_SEL3 & (~XBARA_SEL3_MASK)) | XBARA_SEL3_VALUE;
    #endif
  #elif defined(XBARA_SEL3_VALUE)
  XBARA_SEL3 = XBARA_SEL3_VALUE;
  #endif

  /* Register 'XBARA_SEL4' initialization */
  #if XBARA_SEL4_MASK
    #if XBARA_SEL4_MASK == 0xFFFF
  XBARA_SEL4 = XBARA_SEL4_VALUE;
    #elif XBARA_SEL4_MASK == XBARA_SEL4_VALUE
  XBARA_SEL4 |= XBARA_SEL4_VALUE;
    #elif XBARA_SEL4_VALUE == 0
  XBARA_SEL4 &= ~XBARA_SEL4_MASK;
    #else
  XBARA_SEL4 = (XBARA_SEL4 & (~XBARA_SEL4_MASK)) | XBARA_SEL4_VALUE;
    #endif
  #elif defined(XBARA_SEL4_VALUE)
  XBARA_SEL4 = XBARA_SEL4_VALUE;
  #endif

  /* Register 'XBARA_SEL5' initialization */
  #if XBARA_SEL5_MASK
    #if XBARA_SEL5_MASK == 0xFFFF
  XBARA_SEL5 = XBARA_SEL5_VALUE;
    #elif XBARA_SEL5_MASK == XBARA_SEL5_VALUE
  XBARA_SEL5 |= XBARA_SEL5_VALUE;
    #elif XBARA_SEL5_VALUE == 0
  XBARA_SEL5 &= ~XBARA_SEL5_MASK;
    #else
  XBARA_SEL5 = (XBARA_SEL5 & (~XBARA_SEL5_MASK)) | XBARA_SEL5_VALUE;
    #endif
  #elif defined(XBARA_SEL5_VALUE)
  XBARA_SEL5 = XBARA_SEL5_VALUE;
  #endif

  /* Register 'XBARA_SEL6' initialization */
  #if XBARA_SEL6_MASK
    #if XBARA_SEL6_MASK == 0xFFFF
  XBARA_SEL6 = XBARA_SEL6_VALUE;
    #elif XBARA_SEL6_MASK == XBARA_SEL6_VALUE
  XBARA_SEL6 |= XBARA_SEL6_VALUE;
    #elif XBARA_SEL6_VALUE == 0
  XBARA_SEL6 &= ~XBARA_SEL6_MASK;
    #else
  XBARA_SEL6 = (XBARA_SEL6 & (~XBARA_SEL6_MASK)) | XBARA_SEL6_VALUE;
    #endif
  #elif defined(XBARA_SEL6_VALUE)
  XBARA_SEL6 = XBARA_SEL6_VALUE;
  #endif

  /* Register 'XBARA_SEL7' initialization */
  #if XBARA_SEL7_MASK
    #if XBARA_SEL7_MASK == 0xFFFF
  XBARA_SEL7 = XBARA_SEL7_VALUE;
    #elif XBARA_SEL7_MASK == XBARA_SEL7_VALUE
  XBARA_SEL7 |= XBARA_SEL7_VALUE;
    #elif XBARA_SEL7_VALUE == 0
  XBARA_SEL7 &= ~XBARA_SEL7_MASK;
    #else
  XBARA_SEL7 = (XBARA_SEL7 & (~XBARA_SEL7_MASK)) | XBARA_SEL7_VALUE;
    #endif
  #elif defined(XBARA_SEL7_VALUE)
  XBARA_SEL7 = XBARA_SEL7_VALUE;
  #endif

  /* Register 'XBARA_SEL8' initialization */
  #if XBARA_SEL8_MASK
    #if XBARA_SEL8_MASK == 0xFFFF
  XBARA_SEL8 = XBARA_SEL8_VALUE;
    #elif XBARA_SEL8_MASK == XBARA_SEL8_VALUE
  XBARA_SEL8 |= XBARA_SEL8_VALUE;
    #elif XBARA_SEL8_VALUE == 0
  XBARA_SEL8 &= ~XBARA_SEL8_MASK;
    #else
  XBARA_SEL8 = (XBARA_SEL8 & (~XBARA_SEL8_MASK)) | XBARA_SEL8_VALUE;
    #endif
  #elif defined(XBARA_SEL8_VALUE)
  XBARA_SEL8 = XBARA_SEL8_VALUE;
  #endif

  /* Register 'XBARA_SEL9' initialization */
  #if XBARA_SEL9_MASK
    #if XBARA_SEL9_MASK == 0xFFFF
  XBARA_SEL9 = XBARA_SEL9_VALUE;
    #elif XBARA_SEL9_MASK == XBARA_SEL9_VALUE
  XBARA_SEL9 |= XBARA_SEL9_VALUE;
    #elif XBARA_SEL9_VALUE == 0
  XBARA_SEL9 &= ~XBARA_SEL9_MASK;
    #else
  XBARA_SEL9 = (XBARA_SEL9 & (~XBARA_SEL9_MASK)) | XBARA_SEL9_VALUE;
    #endif
  #elif defined(XBARA_SEL9_VALUE)
  XBARA_SEL9 = XBARA_SEL9_VALUE;
  #endif

  /* Register 'XBARA_SEL10' initialization */
  #if XBARA_SEL10_MASK
    #if XBARA_SEL10_MASK == 0xFFFF
  XBARA_SEL10 = XBARA_SEL10_VALUE;
    #elif XBARA_SEL10_MASK == XBARA_SEL10_VALUE
  XBARA_SEL10 |= XBARA_SEL10_VALUE;
    #elif XBARA_SEL10_VALUE == 0
  XBARA_SEL10 &= ~XBARA_SEL10_MASK;
    #else
  XBARA_SEL10 = (XBARA_SEL10 & (~XBARA_SEL10_MASK)) | XBARA_SEL10_VALUE;
    #endif
  #elif defined(XBARA_SEL10_VALUE)
  XBARA_SEL10 = XBARA_SEL10_VALUE;
  #endif

  /* Register 'XBARA_SEL11' initialization */
  #if XBARA_SEL11_MASK
    #if XBARA_SEL11_MASK == 0xFFFF
  XBARA_SEL11 = XBARA_SEL11_VALUE;
    #elif XBARA_SEL11_MASK == XBARA_SEL11_VALUE
  XBARA_SEL11 |= XBARA_SEL11_VALUE;
    #elif XBARA_SEL11_VALUE == 0
  XBARA_SEL11 &= ~XBARA_SEL11_MASK;
    #else
  XBARA_SEL11 = (XBARA_SEL11 & (~XBARA_SEL11_MASK)) | XBARA_SEL11_VALUE;
    #endif
  #elif defined(XBARA_SEL11_VALUE)
  XBARA_SEL11 = XBARA_SEL11_VALUE;
  #endif

  /* Register 'XBARA_SEL12' initialization */
  #if XBARA_SEL12_MASK
    #if XBARA_SEL12_MASK == 0xFFFF
  XBARA_SEL12 = XBARA_SEL12_VALUE;
    #elif XBARA_SEL12_MASK == XBARA_SEL12_VALUE
  XBARA_SEL12 |= XBARA_SEL12_VALUE;
    #elif XBARA_SEL12_VALUE == 0
  XBARA_SEL12 &= ~XBARA_SEL12_MASK;
    #else
  XBARA_SEL12 = (XBARA_SEL12 & (~XBARA_SEL12_MASK)) | XBARA_SEL12_VALUE;
    #endif
  #elif defined(XBARA_SEL12_VALUE)
  XBARA_SEL12 = XBARA_SEL12_VALUE;
  #endif

  /* Register 'XBARA_SEL13' initialization */
  #if XBARA_SEL13_MASK
    #if XBARA_SEL13_MASK == 0xFFFF
  XBARA_SEL13 = XBARA_SEL13_VALUE;
    #elif XBARA_SEL13_MASK == XBARA_SEL13_VALUE
  XBARA_SEL13 |= XBARA_SEL13_VALUE;
    #elif XBARA_SEL13_VALUE == 0
  XBARA_SEL13 &= ~XBARA_SEL13_MASK;
    #else
  XBARA_SEL13 = (XBARA_SEL13 & (~XBARA_SEL13_MASK)) | XBARA_SEL13_VALUE;
    #endif
  #elif defined(XBARA_SEL13_VALUE)
  XBARA_SEL13 = XBARA_SEL13_VALUE;
  #endif

  /* Register 'XBARA_SEL14' initialization */
  #if XBARA_SEL14_MASK
    #if XBARA_SEL14_MASK == 0xFFFF
  XBARA_SEL14 = XBARA_SEL14_VALUE;
    #elif XBARA_SEL14_MASK == XBARA_SEL14_VALUE
  XBARA_SEL14 |= XBARA_SEL14_VALUE;
    #elif XBARA_SEL14_VALUE == 0
  XBARA_SEL14 &= ~XBARA_SEL14_MASK;
    #else
  XBARA_SEL14 = (XBARA_SEL14 & (~XBARA_SEL14_MASK)) | XBARA_SEL14_VALUE;
    #endif
  #elif defined(XBARA_SEL14_VALUE)
  XBARA_SEL14 = XBARA_SEL14_VALUE;
  #endif

  /* Register 'XBARA_SEL15' initialization */
  #if XBARA_SEL15_MASK
    #if XBARA_SEL15_MASK == 0xFFFF
  XBARA_SEL15 = XBARA_SEL15_VALUE;
    #elif XBARA_SEL15_MASK == XBARA_SEL15_VALUE
  XBARA_SEL15 |= XBARA_SEL15_VALUE;
    #elif XBARA_SEL15_VALUE == 0
  XBARA_SEL15 &= ~XBARA_SEL15_MASK;
    #else
  XBARA_SEL15 = (XBARA_SEL15 & (~XBARA_SEL15_MASK)) | XBARA_SEL15_VALUE;
    #endif
  #elif defined(XBARA_SEL15_VALUE)
  XBARA_SEL15 = XBARA_SEL15_VALUE;
  #endif

  /* Register 'XBARA_SEL16' initialization */
  #if XBARA_SEL16_MASK
    #if XBARA_SEL16_MASK == 0xFFFF
  XBARA_SEL16 = XBARA_SEL16_VALUE;
    #elif XBARA_SEL16_MASK == XBARA_SEL16_VALUE
  XBARA_SEL16 |= XBARA_SEL16_VALUE;
    #elif XBARA_SEL16_VALUE == 0
  XBARA_SEL16 &= ~XBARA_SEL16_MASK;
    #else
  XBARA_SEL16 = (XBARA_SEL16 & (~XBARA_SEL16_MASK)) | XBARA_SEL16_VALUE;
    #endif
  #elif defined(XBARA_SEL16_VALUE)
  XBARA_SEL16 = XBARA_SEL16_VALUE;
  #endif

  /* Register 'XBARA_SEL17' initialization */
  #if XBARA_SEL17_MASK
    #if XBARA_SEL17_MASK == 0xFFFF
  XBARA_SEL17 = XBARA_SEL17_VALUE;
    #elif XBARA_SEL17_MASK == XBARA_SEL17_VALUE
  XBARA_SEL17 |= XBARA_SEL17_VALUE;
    #elif XBARA_SEL17_VALUE == 0
  XBARA_SEL17 &= ~XBARA_SEL17_MASK;
    #else
  XBARA_SEL17 = (XBARA_SEL17 & (~XBARA_SEL17_MASK)) | XBARA_SEL17_VALUE;
    #endif
  #elif defined(XBARA_SEL17_VALUE)
  XBARA_SEL17 = XBARA_SEL17_VALUE;
  #endif

  /* Register 'XBARA_SEL18' initialization */
  #if XBARA_SEL18_MASK
    #if XBARA_SEL18_MASK == 0xFFFF
  XBARA_SEL18 = XBARA_SEL18_VALUE;
    #elif XBARA_SEL18_MASK == XBARA_SEL18_VALUE
  XBARA_SEL18 |= XBARA_SEL18_VALUE;
    #elif XBARA_SEL18_VALUE == 0
  XBARA_SEL18 &= ~XBARA_SEL18_MASK;
    #else
  XBARA_SEL18 = (XBARA_SEL18 & (~XBARA_SEL18_MASK)) | XBARA_SEL18_VALUE;
    #endif
  #elif defined(XBARA_SEL18_VALUE)
  XBARA_SEL18 = XBARA_SEL18_VALUE;
  #endif

  /* Register 'XBARA_SEL19' initialization */
  #if XBARA_SEL19_MASK
    #if XBARA_SEL19_MASK == 0xFFFF
  XBARA_SEL19 = XBARA_SEL19_VALUE;
    #elif XBARA_SEL19_MASK == XBARA_SEL19_VALUE
  XBARA_SEL19 |= XBARA_SEL19_VALUE;
    #elif XBARA_SEL19_VALUE == 0
  XBARA_SEL19 &= ~XBARA_SEL19_MASK;
    #else
  XBARA_SEL19 = (XBARA_SEL19 & (~XBARA_SEL19_MASK)) | XBARA_SEL19_VALUE;
    #endif
  #elif defined(XBARA_SEL19_VALUE)
  XBARA_SEL19 = XBARA_SEL19_VALUE;
  #endif

  /* Register 'XBARA_SEL20' initialization */
  #if XBARA_SEL20_MASK
    #if XBARA_SEL20_MASK == 0xFFFF
  XBARA_SEL20 = XBARA_SEL20_VALUE;
    #elif XBARA_SEL20_MASK == XBARA_SEL20_VALUE
  XBARA_SEL20 |= XBARA_SEL20_VALUE;
    #elif XBARA_SEL20_VALUE == 0
  XBARA_SEL20 &= ~XBARA_SEL20_MASK;
    #else
  XBARA_SEL20 = (XBARA_SEL20 & (~XBARA_SEL20_MASK)) | XBARA_SEL20_VALUE;
    #endif
  #elif defined(XBARA_SEL20_VALUE)
  XBARA_SEL20 = XBARA_SEL20_VALUE;
  #endif

  /* Register 'XBARA_CTRL0' initialization */
  #if XBARA_CTRL0_MASK
    #if XBARA_CTRL0_MASK == 0xFFFF
  XBARA_CTRL0 = XBARA_CTRL0_VALUE;
    #elif XBARA_CTRL0_MASK == XBARA_CTRL0_VALUE
  XBARA_CTRL0 |= XBARA_CTRL0_VALUE;
    #elif XBARA_CTRL0_VALUE == 0
  XBARA_CTRL0 &= ~XBARA_CTRL0_MASK;
    #else
  XBARA_CTRL0 = (XBARA_CTRL0 & (~XBARA_CTRL0_MASK)) | XBARA_CTRL0_VALUE;
    #endif
  #elif defined(XBARA_CTRL0_VALUE)
  XBARA_CTRL0 = XBARA_CTRL0_VALUE;
  #endif

  /* Register 'XBARA_CTRL1' initialization */
  #if XBARA_CTRL1_MASK
    #if XBARA_CTRL1_MASK == 0xFFFF
  XBARA_CTRL1 = XBARA_CTRL1_VALUE;
    #elif XBARA_CTRL1_MASK == XBARA_CTRL1_VALUE
  XBARA_CTRL1 |= XBARA_CTRL1_VALUE;
    #elif XBARA_CTRL1_VALUE == 0
  XBARA_CTRL1 &= ~XBARA_CTRL1_MASK;
    #else
  XBARA_CTRL1 = (XBARA_CTRL1 & (~XBARA_CTRL1_MASK)) | XBARA_CTRL1_VALUE;
    #endif
  #elif defined(XBARA_CTRL1_VALUE)
  XBARA_CTRL1 = XBARA_CTRL1_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END XBARA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for the Freescale 56800 series of microcontrollers.
 **
 ** ################################################################### */
