/** ###################################################################
 **     Filename    : SIM_Init.c
 **     Processor   : MC56F82748VLH
 **     Version     : 1.0
 **     Abstract    :
 **          This file implements the SIM module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void SIM_Init(void);
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
 * @file SIM_Init.c
 * @version 1.0
 * @brief This file implements the SIM module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE SIM_Init. */

#include "SIM_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void SIM_Init(void) {

  /* Register 'SIM_CTRL' initialization */
  #if SIM_CTRL_MASK
    #if SIM_CTRL_MASK == 0xFFFF
  SIM_CTRL = SIM_CTRL_VALUE;
    #elif SIM_CTRL_MASK == SIM_CTRL_VALUE
  SIM_CTRL |= SIM_CTRL_VALUE;
    #elif SIM_CTRL_VALUE == 0
  SIM_CTRL &= ~SIM_CTRL_MASK;
    #else
  SIM_CTRL = (SIM_CTRL & (~SIM_CTRL_MASK)) | SIM_CTRL_VALUE;
    #endif
  #elif defined(SIM_CTRL_VALUE)
  SIM_CTRL = SIM_CTRL_VALUE;
  #endif

  /* Register 'SIM_MISC0' initialization */
  #if SIM_MISC0_MASK
    #if SIM_MISC0_MASK == 0xFFFF
  SIM_MISC0 = SIM_MISC0_VALUE;
    #elif SIM_MISC0_MASK == SIM_MISC0_VALUE
  SIM_MISC0 |= SIM_MISC0_VALUE;
    #elif SIM_MISC0_VALUE == 0
  SIM_MISC0 &= ~SIM_MISC0_MASK;
    #else
  SIM_MISC0 = (SIM_MISC0 & (~SIM_MISC0_MASK)) | SIM_MISC0_VALUE;
    #endif
  #elif defined(SIM_MISC0_VALUE)
  SIM_MISC0 = SIM_MISC0_VALUE;
  #endif

  /* Register 'SIM_PWR' initialization */
  #if SIM_PWR_MASK
    #if SIM_PWR_MASK == 0xFFFF
  SIM_PWR = SIM_PWR_VALUE;
    #elif SIM_PWR_MASK == SIM_PWR_VALUE
  SIM_PWR |= SIM_PWR_VALUE;
    #elif SIM_PWR_VALUE == 0
  SIM_PWR &= ~SIM_PWR_MASK;
    #else
  SIM_PWR = (SIM_PWR & (~SIM_PWR_MASK)) | SIM_PWR_VALUE;
    #endif
  #elif defined(SIM_PWR_VALUE)
  SIM_PWR = SIM_PWR_VALUE;
  #endif

  /* Register 'SIM_GPSAL' initialization */
  #if SIM_GPSAL_MASK
    #if SIM_GPSAL_MASK == 0xFFFF
  SIM_GPSAL = SIM_GPSAL_VALUE;
    #elif SIM_GPSAL_MASK == SIM_GPSAL_VALUE
  SIM_GPSAL |= SIM_GPSAL_VALUE;
    #elif SIM_GPSAL_VALUE == 0
  SIM_GPSAL &= ~SIM_GPSAL_MASK;
    #else
  SIM_GPSAL = (SIM_GPSAL & (~SIM_GPSAL_MASK)) | SIM_GPSAL_VALUE;
    #endif
  #elif defined(SIM_GPSAL_VALUE)
  SIM_GPSAL = SIM_GPSAL_VALUE;
  #endif

  /* Register 'SIM_GPSBL' initialization */
  #if SIM_GPSBL_MASK
    #if SIM_GPSBL_MASK == 0xFFFF
  SIM_GPSBL = SIM_GPSBL_VALUE;
    #elif SIM_GPSBL_MASK == SIM_GPSBL_VALUE
  SIM_GPSBL |= SIM_GPSBL_VALUE;
    #elif SIM_GPSBL_VALUE == 0
  SIM_GPSBL &= ~SIM_GPSBL_MASK;
    #else
  SIM_GPSBL = (SIM_GPSBL & (~SIM_GPSBL_MASK)) | SIM_GPSBL_VALUE;
    #endif
  #elif defined(SIM_GPSBL_VALUE)
  SIM_GPSBL = SIM_GPSBL_VALUE;
  #endif

  /* Register 'SIM_GPSCL' initialization */
  #if SIM_GPSCL_MASK
    #if SIM_GPSCL_MASK == 0xFFFF
  SIM_GPSCL = SIM_GPSCL_VALUE;
    #elif SIM_GPSCL_MASK == SIM_GPSCL_VALUE
  SIM_GPSCL |= SIM_GPSCL_VALUE;
    #elif SIM_GPSCL_VALUE == 0
  SIM_GPSCL &= ~SIM_GPSCL_MASK;
    #else
  SIM_GPSCL = (SIM_GPSCL & (~SIM_GPSCL_MASK)) | SIM_GPSCL_VALUE;
    #endif
  #elif defined(SIM_GPSCL_VALUE)
  SIM_GPSCL = SIM_GPSCL_VALUE;
  #endif

  /* Register 'SIM_GPSCH' initialization */
  #ifdef SIM_GPSCH_VALUE
  SIM_GPSCH = SIM_GPSCH_VALUE;
  #endif

  /* Register 'SIM_GPSEL' initialization */
  #if SIM_GPSEL_MASK
    #if SIM_GPSEL_MASK == 0xFFFF
  SIM_GPSEL = SIM_GPSEL_VALUE;
    #elif SIM_GPSEL_MASK == SIM_GPSEL_VALUE
  SIM_GPSEL |= SIM_GPSEL_VALUE;
    #elif SIM_GPSEL_VALUE == 0
  SIM_GPSEL &= ~SIM_GPSEL_MASK;
    #else
  SIM_GPSEL = (SIM_GPSEL & (~SIM_GPSEL_MASK)) | SIM_GPSEL_VALUE;
    #endif
  #elif defined(SIM_GPSEL_VALUE)
  SIM_GPSEL = SIM_GPSEL_VALUE;
  #endif

  /* Register 'SIM_GPSFL' initialization */
  #ifdef SIM_GPSFL_VALUE
  SIM_GPSFL = SIM_GPSFL_VALUE;
  #endif

  /* Register 'SIM_GPSFH' initialization */
  #if SIM_GPSFH_MASK
    #if SIM_GPSFH_MASK == 0xFFFF
  SIM_GPSFH = SIM_GPSFH_VALUE;
    #elif SIM_GPSFH_MASK == SIM_GPSFH_VALUE
  SIM_GPSFH |= SIM_GPSFH_VALUE;
    #elif SIM_GPSFH_VALUE == 0
  SIM_GPSFH &= ~SIM_GPSFH_MASK;
    #else
  SIM_GPSFH = (SIM_GPSFH & (~SIM_GPSFH_MASK)) | SIM_GPSFH_VALUE;
    #endif
  #elif defined(SIM_GPSFH_VALUE)
  SIM_GPSFH = SIM_GPSFH_VALUE;
  #endif

  /* Register 'SIM_IPSn' initialization */
  #if SIM_IPSn_MASK
    #if SIM_IPSn_MASK == 0xFFFF
  SIM_IPSn = SIM_IPSn_VALUE;
    #elif SIM_IPSn_MASK == SIM_IPSn_VALUE
  SIM_IPSn |= SIM_IPSn_VALUE;
    #elif SIM_IPSn_VALUE == 0
  SIM_IPSn &= ~SIM_IPSn_MASK;
    #else
  SIM_IPSn = (SIM_IPSn & (~SIM_IPSn_MASK)) | SIM_IPSn_VALUE;
    #endif
  #elif defined(SIM_IPSn_VALUE)
  SIM_IPSn = SIM_IPSn_VALUE;
  #endif

  /* Register 'SIM_PCR' initialization */
  #if SIM_PCR_MASK
    #if SIM_PCR_MASK == 0xFFFF
  SIM_PCR = SIM_PCR_VALUE;
    #elif SIM_PCR_MASK == SIM_PCR_VALUE
  SIM_PCR |= SIM_PCR_VALUE;
    #elif SIM_PCR_VALUE == 0
  SIM_PCR &= ~SIM_PCR_MASK;
    #else
  SIM_PCR = (SIM_PCR & (~SIM_PCR_MASK)) | SIM_PCR_VALUE;
    #endif
  #elif defined(SIM_PCR_VALUE)
  SIM_PCR = SIM_PCR_VALUE;
  #endif

  /* Register 'SIM_CLKOUT' initialization */
  #if SIM_CLKOUT_MASK
    #if SIM_CLKOUT_MASK == 0xFFFF
  SIM_CLKOUT = SIM_CLKOUT_VALUE;
    #elif SIM_CLKOUT_MASK == SIM_CLKOUT_VALUE
  SIM_CLKOUT |= SIM_CLKOUT_VALUE;
    #elif SIM_CLKOUT_VALUE == 0
  SIM_CLKOUT &= ~SIM_CLKOUT_MASK;
    #else
  SIM_CLKOUT = (SIM_CLKOUT & (~SIM_CLKOUT_MASK)) | SIM_CLKOUT_VALUE;
    #endif
  #elif defined(SIM_CLKOUT_VALUE)
  SIM_CLKOUT = SIM_CLKOUT_VALUE;
  #endif

  /* Register 'SIM_PCE0' initialization */
  #if SIM_PCE0_MASK
    #if SIM_PCE0_MASK == 0xFFFF
  SIM_PCE0 = SIM_PCE0_VALUE;
    #elif SIM_PCE0_MASK == SIM_PCE0_VALUE
  SIM_PCE0 |= SIM_PCE0_VALUE;
    #elif SIM_PCE0_VALUE == 0
  SIM_PCE0 &= ~SIM_PCE0_MASK;
    #else
  SIM_PCE0 = (SIM_PCE0 & (~SIM_PCE0_MASK)) | SIM_PCE0_VALUE;
    #endif
  #elif defined(SIM_PCE0_VALUE)
  SIM_PCE0 = SIM_PCE0_VALUE;
  #endif

  /* Register 'SIM_PCE1' initialization */
  #if SIM_PCE1_MASK
    #if SIM_PCE1_MASK == 0xFFFF
  SIM_PCE1 = SIM_PCE1_VALUE;
    #elif SIM_PCE1_MASK == SIM_PCE1_VALUE
  SIM_PCE1 |= SIM_PCE1_VALUE;
    #elif SIM_PCE1_VALUE == 0
  SIM_PCE1 &= ~SIM_PCE1_MASK;
    #else
  SIM_PCE1 = (SIM_PCE1 & (~SIM_PCE1_MASK)) | SIM_PCE1_VALUE;
    #endif
  #elif defined(SIM_PCE1_VALUE)
  SIM_PCE1 = SIM_PCE1_VALUE;
  #endif

  /* Register 'SIM_PCE2' initialization */
  #if SIM_PCE2_MASK
    #if SIM_PCE2_MASK == 0xFFFF
  SIM_PCE2 = SIM_PCE2_VALUE;
    #elif SIM_PCE2_MASK == SIM_PCE2_VALUE
  SIM_PCE2 |= SIM_PCE2_VALUE;
    #elif SIM_PCE2_VALUE == 0
  SIM_PCE2 &= ~SIM_PCE2_MASK;
    #else
  SIM_PCE2 = (SIM_PCE2 & (~SIM_PCE2_MASK)) | SIM_PCE2_VALUE;
    #endif
  #elif defined(SIM_PCE2_VALUE)
  SIM_PCE2 = SIM_PCE2_VALUE;
  #endif

  /* Register 'SIM_PCE3' initialization */
  #if SIM_PCE3_MASK
    #if SIM_PCE3_MASK == 0xFFFF
  SIM_PCE3 = SIM_PCE3_VALUE;
    #elif SIM_PCE3_MASK == SIM_PCE3_VALUE
  SIM_PCE3 |= SIM_PCE3_VALUE;
    #elif SIM_PCE3_VALUE == 0
  SIM_PCE3 &= ~SIM_PCE3_MASK;
    #else
  SIM_PCE3 = (SIM_PCE3 & (~SIM_PCE3_MASK)) | SIM_PCE3_VALUE;
    #endif
  #elif defined(SIM_PCE3_VALUE)
  SIM_PCE3 = SIM_PCE3_VALUE;
  #endif

  /* Register 'SIM_SD0' initialization */
  #if SIM_SD0_MASK
    #if SIM_SD0_MASK == 0xFFFF
  SIM_SD0 = SIM_SD0_VALUE;
    #elif SIM_SD0_MASK == SIM_SD0_VALUE
  SIM_SD0 |= SIM_SD0_VALUE;
    #elif SIM_SD0_VALUE == 0
  SIM_SD0 &= ~SIM_SD0_MASK;
    #else
  SIM_SD0 = (SIM_SD0 & (~SIM_SD0_MASK)) | SIM_SD0_VALUE;
    #endif
  #elif defined(SIM_SD0_VALUE)
  SIM_SD0 = SIM_SD0_VALUE;
  #endif

  /* Register 'SIM_SD1' initialization */
  #if SIM_SD1_MASK
    #if SIM_SD1_MASK == 0xFFFF
  SIM_SD1 = SIM_SD1_VALUE;
    #elif SIM_SD1_MASK == SIM_SD1_VALUE
  SIM_SD1 |= SIM_SD1_VALUE;
    #elif SIM_SD1_VALUE == 0
  SIM_SD1 &= ~SIM_SD1_MASK;
    #else
  SIM_SD1 = (SIM_SD1 & (~SIM_SD1_MASK)) | SIM_SD1_VALUE;
    #endif
  #elif defined(SIM_SD1_VALUE)
  SIM_SD1 = SIM_SD1_VALUE;
  #endif

  /* Register 'SIM_SD2' initialization */
  #if SIM_SD2_MASK
    #if SIM_SD2_MASK == 0xFFFF
  SIM_SD2 = SIM_SD2_VALUE;
    #elif SIM_SD2_MASK == SIM_SD2_VALUE
  SIM_SD2 |= SIM_SD2_VALUE;
    #elif SIM_SD2_VALUE == 0
  SIM_SD2 &= ~SIM_SD2_MASK;
    #else
  SIM_SD2 = (SIM_SD2 & (~SIM_SD2_MASK)) | SIM_SD2_VALUE;
    #endif
  #elif defined(SIM_SD2_VALUE)
  SIM_SD2 = SIM_SD2_VALUE;
  #endif

  /* Register 'SIM_SD3' initialization */
  #if SIM_SD3_MASK
    #if SIM_SD3_MASK == 0xFFFF
  SIM_SD3 = SIM_SD3_VALUE;
    #elif SIM_SD3_MASK == SIM_SD3_VALUE
  SIM_SD3 |= SIM_SD3_VALUE;
    #elif SIM_SD3_VALUE == 0
  SIM_SD3 &= ~SIM_SD3_MASK;
    #else
  SIM_SD3 = (SIM_SD3 & (~SIM_SD3_MASK)) | SIM_SD3_VALUE;
    #endif
  #elif defined(SIM_SD3_VALUE)
  SIM_SD3 = SIM_SD3_VALUE;
  #endif

  /* Register 'SIM_PWRMODE' initialization */
  #ifdef SIM_PWRMODE_VALUE
  SIM_PWRMODE = SIM_PWRMODE_VALUE;
  #endif

  /* Register 'SIM_PROT' initialization */
  #if SIM_PROT_MASK
    #if SIM_PROT_MASK == 0xFFFF
  SIM_PROT = SIM_PROT_VALUE;
    #elif SIM_PROT_MASK == SIM_PROT_VALUE
  SIM_PROT |= SIM_PROT_VALUE;
    #elif SIM_PROT_VALUE == 0
  SIM_PROT &= ~SIM_PROT_MASK;
    #else
  SIM_PROT = (SIM_PROT & (~SIM_PROT_MASK)) | SIM_PROT_VALUE;
    #endif
  #elif defined(SIM_PROT_VALUE)
  SIM_PROT = SIM_PROT_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END SIM_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
