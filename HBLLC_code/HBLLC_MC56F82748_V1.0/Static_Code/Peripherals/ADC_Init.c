/** ###################################################################
 **     Filename    : ADC_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the ADC module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void ADC_Init(void);
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
 * @file ADC_Init.c
 * @brief This file implements the ADC module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE ADC_Init. */

#include "ADC_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADC_Init(void) {

  /* Register 'ADC_CLIST1' initialization */
  #ifdef ADC_CLIST1_VALUE
  ADC_CLIST1 = ADC_CLIST1_VALUE;
  #endif

  /* Register 'ADC_CLIST2' initialization */
  #ifdef ADC_CLIST2_VALUE
  ADC_CLIST2 = ADC_CLIST2_VALUE;
  #endif

  /* Register 'ADC_CLIST3' initialization */
  #ifdef ADC_CLIST3_VALUE
  ADC_CLIST3 = ADC_CLIST3_VALUE;
  #endif

  /* Register 'ADC_CLIST4' initialization */
  #ifdef ADC_CLIST4_VALUE
  ADC_CLIST4 = ADC_CLIST4_VALUE;
  #endif

  /* Register 'ADC_CLIST5' initialization */
  #ifdef ADC_CLIST5_VALUE
  ADC_CLIST5 = ADC_CLIST5_VALUE;
  #endif

  /* Register 'ADC_ZXCTRL1' initialization */
  #ifdef ADC_ZXCTRL1_VALUE
  ADC_ZXCTRL1 = ADC_ZXCTRL1_VALUE;
  #endif

  /* Register 'ADC_ZXCTRL2' initialization */
  #ifdef ADC_ZXCTRL2_VALUE
  ADC_ZXCTRL2 = ADC_ZXCTRL2_VALUE;
  #endif

  /* Register 'ADC_ZXCTRL3' initialization */
  #ifdef ADC_ZXCTRL3_VALUE
  ADC_ZXCTRL3 = ADC_ZXCTRL3_VALUE;
  #endif

  /* Register 'ADC_LOLIM0' initialization */
  #ifdef ADC_LOLIM0_VALUE
  ADC_LOLIM0 = ADC_LOLIM0_VALUE;
  #endif

  /* Register 'ADC_LOLIM1' initialization */
  #ifdef ADC_LOLIM1_VALUE
  ADC_LOLIM1 = ADC_LOLIM1_VALUE;
  #endif

  /* Register 'ADC_LOLIM2' initialization */
  #ifdef ADC_LOLIM2_VALUE
  ADC_LOLIM2 = ADC_LOLIM2_VALUE;
  #endif

  /* Register 'ADC_LOLIM3' initialization */
  #ifdef ADC_LOLIM3_VALUE
  ADC_LOLIM3 = ADC_LOLIM3_VALUE;
  #endif

  /* Register 'ADC_LOLIM4' initialization */
  #ifdef ADC_LOLIM4_VALUE
  ADC_LOLIM4 = ADC_LOLIM4_VALUE;
  #endif

  /* Register 'ADC_LOLIM5' initialization */
  #ifdef ADC_LOLIM5_VALUE
  ADC_LOLIM5 = ADC_LOLIM5_VALUE;
  #endif

  /* Register 'ADC_LOLIM6' initialization */
  #ifdef ADC_LOLIM6_VALUE
  ADC_LOLIM6 = ADC_LOLIM6_VALUE;
  #endif

  /* Register 'ADC_LOLIM7' initialization */
  #ifdef ADC_LOLIM7_VALUE
  ADC_LOLIM7 = ADC_LOLIM7_VALUE;
  #endif

  /* Register 'ADC_LOLIM8' initialization */
  #ifdef ADC_LOLIM8_VALUE
  ADC_LOLIM8 = ADC_LOLIM8_VALUE;
  #endif

  /* Register 'ADC_LOLIM9' initialization */
  #ifdef ADC_LOLIM9_VALUE
  ADC_LOLIM9 = ADC_LOLIM9_VALUE;
  #endif

  /* Register 'ADC_LOLIM10' initialization */
  #ifdef ADC_LOLIM10_VALUE
  ADC_LOLIM10 = ADC_LOLIM10_VALUE;
  #endif

  /* Register 'ADC_LOLIM11' initialization */
  #ifdef ADC_LOLIM11_VALUE
  ADC_LOLIM11 = ADC_LOLIM11_VALUE;
  #endif

  /* Register 'ADC_LOLIM12' initialization */
  #ifdef ADC_LOLIM12_VALUE
  ADC_LOLIM12 = ADC_LOLIM12_VALUE;
  #endif

  /* Register 'ADC_LOLIM13' initialization */
  #ifdef ADC_LOLIM13_VALUE
  ADC_LOLIM13 = ADC_LOLIM13_VALUE;
  #endif

  /* Register 'ADC_LOLIM14' initialization */
  #ifdef ADC_LOLIM14_VALUE
  ADC_LOLIM14 = ADC_LOLIM14_VALUE;
  #endif

  /* Register 'ADC_LOLIM15' initialization */
  #ifdef ADC_LOLIM15_VALUE
  ADC_LOLIM15 = ADC_LOLIM15_VALUE;
  #endif

  /* Register 'ADC_LOLIM216' initialization */
  #ifdef ADC_LOLIM216_VALUE
  ADC_LOLIM216 = ADC_LOLIM216_VALUE;
  #endif

  /* Register 'ADC_LOLIM217' initialization */
  #ifdef ADC_LOLIM217_VALUE
  ADC_LOLIM217 = ADC_LOLIM217_VALUE;
  #endif

  /* Register 'ADC_LOLIM218' initialization */
  #ifdef ADC_LOLIM218_VALUE
  ADC_LOLIM218 = ADC_LOLIM218_VALUE;
  #endif

  /* Register 'ADC_LOLIM219' initialization */
  #ifdef ADC_LOLIM219_VALUE
  ADC_LOLIM219 = ADC_LOLIM219_VALUE;
  #endif

  /* Register 'ADC_HILIM0' initialization */
  #ifdef ADC_HILIM0_VALUE
  ADC_HILIM0 = ADC_HILIM0_VALUE;
  #endif

  /* Register 'ADC_HILIM1' initialization */
  #ifdef ADC_HILIM1_VALUE
  ADC_HILIM1 = ADC_HILIM1_VALUE;
  #endif

  /* Register 'ADC_HILIM2' initialization */
  #ifdef ADC_HILIM2_VALUE
  ADC_HILIM2 = ADC_HILIM2_VALUE;
  #endif

  /* Register 'ADC_HILIM3' initialization */
  #ifdef ADC_HILIM3_VALUE
  ADC_HILIM3 = ADC_HILIM3_VALUE;
  #endif

  /* Register 'ADC_HILIM4' initialization */
  #ifdef ADC_HILIM4_VALUE
  ADC_HILIM4 = ADC_HILIM4_VALUE;
  #endif

  /* Register 'ADC_HILIM5' initialization */
  #ifdef ADC_HILIM5_VALUE
  ADC_HILIM5 = ADC_HILIM5_VALUE;
  #endif

  /* Register 'ADC_HILIM6' initialization */
  #ifdef ADC_HILIM6_VALUE
  ADC_HILIM6 = ADC_HILIM6_VALUE;
  #endif

  /* Register 'ADC_HILIM7' initialization */
  #ifdef ADC_HILIM7_VALUE
  ADC_HILIM7 = ADC_HILIM7_VALUE;
  #endif

  /* Register 'ADC_HILIM8' initialization */
  #ifdef ADC_HILIM8_VALUE
  ADC_HILIM8 = ADC_HILIM8_VALUE;
  #endif

  /* Register 'ADC_HILIM9' initialization */
  #ifdef ADC_HILIM9_VALUE
  ADC_HILIM9 = ADC_HILIM9_VALUE;
  #endif

  /* Register 'ADC_HILIM10' initialization */
  #ifdef ADC_HILIM10_VALUE
  ADC_HILIM10 = ADC_HILIM10_VALUE;
  #endif

  /* Register 'ADC_HILIM11' initialization */
  #ifdef ADC_HILIM11_VALUE
  ADC_HILIM11 = ADC_HILIM11_VALUE;
  #endif

  /* Register 'ADC_HILIM12' initialization */
  #ifdef ADC_HILIM12_VALUE
  ADC_HILIM12 = ADC_HILIM12_VALUE;
  #endif

  /* Register 'ADC_HILIM13' initialization */
  #ifdef ADC_HILIM13_VALUE
  ADC_HILIM13 = ADC_HILIM13_VALUE;
  #endif

  /* Register 'ADC_HILIM14' initialization */
  #ifdef ADC_HILIM14_VALUE
  ADC_HILIM14 = ADC_HILIM14_VALUE;
  #endif

  /* Register 'ADC_HILIM15' initialization */
  #ifdef ADC_HILIM15_VALUE
  ADC_HILIM15 = ADC_HILIM15_VALUE;
  #endif

  /* Register 'ADC_HILIM216' initialization */
  #ifdef ADC_HILIM216_VALUE
  ADC_HILIM216 = ADC_HILIM216_VALUE;
  #endif

  /* Register 'ADC_HILIM217' initialization */
  #ifdef ADC_HILIM217_VALUE
  ADC_HILIM217 = ADC_HILIM217_VALUE;
  #endif

  /* Register 'ADC_HILIM218' initialization */
  #ifdef ADC_HILIM218_VALUE
  ADC_HILIM218 = ADC_HILIM218_VALUE;
  #endif

  /* Register 'ADC_HILIM219' initialization */
  #ifdef ADC_HILIM219_VALUE
  ADC_HILIM219 = ADC_HILIM219_VALUE;
  #endif

  /* Register 'ADC_OFFST0' initialization */
  #ifdef ADC_OFFST0_VALUE
  ADC_OFFST0 = ADC_OFFST0_VALUE;
  #endif

  /* Register 'ADC_OFFST1' initialization */
  #ifdef ADC_OFFST1_VALUE
  ADC_OFFST1 = ADC_OFFST1_VALUE;
  #endif

  /* Register 'ADC_OFFST2' initialization */
  #ifdef ADC_OFFST2_VALUE
  ADC_OFFST2 = ADC_OFFST2_VALUE;
  #endif

  /* Register 'ADC_OFFST3' initialization */
  #ifdef ADC_OFFST3_VALUE
  ADC_OFFST3 = ADC_OFFST3_VALUE;
  #endif

  /* Register 'ADC_OFFST4' initialization */
  #ifdef ADC_OFFST4_VALUE
  ADC_OFFST4 = ADC_OFFST4_VALUE;
  #endif

  /* Register 'ADC_OFFST5' initialization */
  #ifdef ADC_OFFST5_VALUE
  ADC_OFFST5 = ADC_OFFST5_VALUE;
  #endif

  /* Register 'ADC_OFFST6' initialization */
  #ifdef ADC_OFFST6_VALUE
  ADC_OFFST6 = ADC_OFFST6_VALUE;
  #endif

  /* Register 'ADC_OFFST7' initialization */
  #ifdef ADC_OFFST7_VALUE
  ADC_OFFST7 = ADC_OFFST7_VALUE;
  #endif

  /* Register 'ADC_OFFST8' initialization */
  #ifdef ADC_OFFST8_VALUE
  ADC_OFFST8 = ADC_OFFST8_VALUE;
  #endif

  /* Register 'ADC_OFFST9' initialization */
  #ifdef ADC_OFFST9_VALUE
  ADC_OFFST9 = ADC_OFFST9_VALUE;
  #endif

  /* Register 'ADC_OFFST10' initialization */
  #ifdef ADC_OFFST10_VALUE
  ADC_OFFST10 = ADC_OFFST10_VALUE;
  #endif

  /* Register 'ADC_OFFST11' initialization */
  #ifdef ADC_OFFST11_VALUE
  ADC_OFFST11 = ADC_OFFST11_VALUE;
  #endif

  /* Register 'ADC_OFFST12' initialization */
  #ifdef ADC_OFFST12_VALUE
  ADC_OFFST12 = ADC_OFFST12_VALUE;
  #endif

  /* Register 'ADC_OFFST13' initialization */
  #ifdef ADC_OFFST13_VALUE
  ADC_OFFST13 = ADC_OFFST13_VALUE;
  #endif

  /* Register 'ADC_OFFST14' initialization */
  #ifdef ADC_OFFST14_VALUE
  ADC_OFFST14 = ADC_OFFST14_VALUE;
  #endif

  /* Register 'ADC_OFFST15' initialization */
  #ifdef ADC_OFFST15_VALUE
  ADC_OFFST15 = ADC_OFFST15_VALUE;
  #endif

  /* Register 'ADC_OFFST216' initialization */
  #ifdef ADC_OFFST216_VALUE
  ADC_OFFST216 = ADC_OFFST216_VALUE;
  #endif

  /* Register 'ADC_OFFST217' initialization */
  #ifdef ADC_OFFST217_VALUE
  ADC_OFFST217 = ADC_OFFST217_VALUE;
  #endif

  /* Register 'ADC_OFFST218' initialization */
  #ifdef ADC_OFFST218_VALUE
  ADC_OFFST218 = ADC_OFFST218_VALUE;
  #endif

  /* Register 'ADC_OFFST219' initialization */
  #ifdef ADC_OFFST219_VALUE
  ADC_OFFST219 = ADC_OFFST219_VALUE;
  #endif

  /* Register 'ADC_GC1' initialization */
  #ifdef ADC_GC1_VALUE
  ADC_GC1 = ADC_GC1_VALUE;
  #endif

  /* Register 'ADC_GC2' initialization */
  #ifdef ADC_GC2_VALUE
  ADC_GC2 = ADC_GC2_VALUE;
  #endif

  /* Register 'ADC_GC3' initialization */
  #ifdef ADC_GC3_VALUE
  ADC_GC3 = ADC_GC3_VALUE;
  #endif

  /* Register 'ADC_SCHLTEN' initialization */
  #ifdef ADC_SCHLTEN_VALUE
  ADC_SCHLTEN = ADC_SCHLTEN_VALUE;
  #endif

  /* Register 'ADC_SCHLTEN2' initialization */
  #ifdef ADC_SCHLTEN2_VALUE
  ADC_SCHLTEN2 = ADC_SCHLTEN2_VALUE;
  #endif

  /* Register 'ADC_SCTRL' initialization */
  #ifdef ADC_SCTRL_VALUE
  ADC_SCTRL = ADC_SCTRL_VALUE;
  #endif

  /* Register 'ADC_SCTRL2' initialization */
  #ifdef ADC_SCTRL2_VALUE
  ADC_SCTRL2 = ADC_SCTRL2_VALUE;
  #endif

  /* Register 'ADC_CAL' initialization */
  #ifdef ADC_CAL_VALUE
  ADC_CAL = ADC_CAL_VALUE;
  #endif

  /* Register 'ADC_SDIS' initialization */
  #ifdef ADC_SDIS_VALUE
  ADC_SDIS = ADC_SDIS_VALUE;
  #endif

  /* Register 'ADC_SDIS2' initialization */
  #ifdef ADC_SDIS2_VALUE
  ADC_SDIS2 = ADC_SDIS2_VALUE;
  #endif

  /* Register 'ADC_PWR2' initialization */
  #ifdef ADC_PWR2_VALUE
  ADC_PWR2 = ADC_PWR2_VALUE;
  #endif

  /* Register 'ADC_CTRL3' initialization */
  #ifdef ADC_CTRL3_VALUE
  ADC_CTRL3 = ADC_CTRL3_VALUE;
  #endif

  ADC_LOLIMSTAT = 0xFFFF;

  ADC_LOLIMSTAT2 = 0xFFFF;

  ADC_HILIMSTAT = 0xFFFF;

  ADC_HILIMSTAT2 = 0xFFFF;

  ADC_ZXSTAT = 0xFFFF;

  ADC_ZXSTAT2 = 0xFFFF;

  ADC_STAT = 0x1800;

  /* Register 'ADC_CTRL2' initialization */
  #ifdef ADC_CTRL2_VALUE
  ADC_CTRL2 = ADC_CTRL2_VALUE;
  #endif

  /* Register 'ADC_CTRL1' initialization */
  #ifdef ADC_CTRL1_VALUE
  ADC_CTRL1 = ADC_CTRL1_VALUE;
  #endif

  /* Register 'ADC_PWR' initialization */
  #ifdef ADC_PWR_VALUE
  ADC_PWR = ADC_PWR_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END ADC_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
