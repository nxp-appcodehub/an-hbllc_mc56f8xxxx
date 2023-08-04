/** ###################################################################
 **     Filename    : PWMA_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the PWMA module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void PWMA_Init(void);
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
 * @file PWMA_Init.c
 * @brief This file implements the PWMA module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE PWMA_Init. */

#include "PWMA_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PWMA_Init(void) {

  /* Register 'PWMA_MCTRL' initialization */
  #if PWMA_MCTRL_MASK_1
    #if PWMA_MCTRL_MASK_1 == 0xFFFF
  PWMA_MCTRL = PWMA_MCTRL_VALUE_1;
    #elif PWMA_MCTRL_MASK_1 == PWMA_MCTRL_VALUE_1
  PWMA_MCTRL |= PWMA_MCTRL_VALUE_1;
    #elif PWMA_MCTRL_VALUE_1 == 0
  PWMA_MCTRL &= ~PWMA_MCTRL_MASK_1;
    #else
  PWMA_MCTRL = (PWMA_MCTRL & (~PWMA_MCTRL_MASK_1)) | PWMA_MCTRL_VALUE_1;
    #endif
  #elif defined(PWMA_MCTRL_VALUE_1)
  PWMA_MCTRL = PWMA_MCTRL_VALUE_1;
  #endif

  /* Register 'PWMA_SM0INTEN' initialization */
  #ifdef PWMA_SM0INTEN_VALUE_1
  PWMA_SM0INTEN = PWMA_SM0INTEN_VALUE_1;
  #endif

  /* Register 'PWMA_SM1INTEN' initialization */
  #ifdef PWMA_SM1INTEN_VALUE_1
  PWMA_SM1INTEN = PWMA_SM1INTEN_VALUE_1;
  #endif

  /* Register 'PWMA_SM2INTEN' initialization */
  #ifdef PWMA_SM2INTEN_VALUE_1
  PWMA_SM2INTEN = PWMA_SM2INTEN_VALUE_1;
  #endif

  /* Register 'PWMA_SM3INTEN' initialization */
  #ifdef PWMA_SM3INTEN_VALUE_1
  PWMA_SM3INTEN = PWMA_SM3INTEN_VALUE_1;
  #endif

  /* Register 'PWMA_FCTRL0' initialization */
  #ifdef PWMA_FCTRL0_VALUE_1
  PWMA_FCTRL0 = PWMA_FCTRL0_VALUE_1;
  #endif

  /* Register 'PWMA_FCTRL1' initialization */
  #ifdef PWMA_FCTRL1_VALUE_1
  PWMA_FCTRL1 = PWMA_FCTRL1_VALUE_1;
  #endif

  /* Register 'PWMA_SM0OCTRL' initialization */
  #ifdef PWMA_SM0OCTRL_VALUE
  PWMA_SM0OCTRL = PWMA_SM0OCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM1OCTRL' initialization */
  #ifdef PWMA_SM1OCTRL_VALUE
  PWMA_SM1OCTRL = PWMA_SM1OCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM2OCTRL' initialization */
  #ifdef PWMA_SM2OCTRL_VALUE
  PWMA_SM2OCTRL = PWMA_SM2OCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM3OCTRL' initialization */
  #ifdef PWMA_SM3OCTRL_VALUE
  PWMA_SM3OCTRL = PWMA_SM3OCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM0INIT' initialization */
  #ifdef PWMA_SM0INIT_VALUE
  PWMA_SM0INIT = PWMA_SM0INIT_VALUE;
  #endif

  /* Register 'PWMA_SM1INIT' initialization */
  #ifdef PWMA_SM1INIT_VALUE
  PWMA_SM1INIT = PWMA_SM1INIT_VALUE;
  #endif

  /* Register 'PWMA_SM2INIT' initialization */
  #ifdef PWMA_SM2INIT_VALUE
  PWMA_SM2INIT = PWMA_SM2INIT_VALUE;
  #endif

  /* Register 'PWMA_SM3INIT' initialization */
  #ifdef PWMA_SM3INIT_VALUE
  PWMA_SM3INIT = PWMA_SM3INIT_VALUE;
  #endif

  /* Register 'PWMA_SM0CTRL2' initialization */
  #ifdef PWMA_SM0CTRL2_VALUE
  PWMA_SM0CTRL2 = PWMA_SM0CTRL2_VALUE;
  #endif

  /* Register 'PWMA_SM1CTRL2' initialization */
  #ifdef PWMA_SM1CTRL2_VALUE
  PWMA_SM1CTRL2 = PWMA_SM1CTRL2_VALUE;
  #endif

  /* Register 'PWMA_SM2CTRL2' initialization */
  #ifdef PWMA_SM2CTRL2_VALUE
  PWMA_SM2CTRL2 = PWMA_SM2CTRL2_VALUE;
  #endif

  /* Register 'PWMA_SM3CTRL2' initialization */
  #ifdef PWMA_SM3CTRL2_VALUE
  PWMA_SM3CTRL2 = PWMA_SM3CTRL2_VALUE;
  #endif

  /* Register 'PWMA_SM0CTRL' initialization */
  #ifdef PWMA_SM0CTRL_VALUE
  PWMA_SM0CTRL = PWMA_SM0CTRL_VALUE;
  #endif

  /* Register 'PWMA_SM1CTRL' initialization */
  #ifdef PWMA_SM1CTRL_VALUE
  PWMA_SM1CTRL = PWMA_SM1CTRL_VALUE;
  #endif

  /* Register 'PWMA_SM2CTRL' initialization */
  #ifdef PWMA_SM2CTRL_VALUE
  PWMA_SM2CTRL = PWMA_SM2CTRL_VALUE;
  #endif

  /* Register 'PWMA_SM3CTRL' initialization */
  #ifdef PWMA_SM3CTRL_VALUE
  PWMA_SM3CTRL = PWMA_SM3CTRL_VALUE;
  #endif

  /* Register 'PWMA_SM0TCTRL' initialization */
  #ifdef PWMA_SM0TCTRL_VALUE
  PWMA_SM0TCTRL = PWMA_SM0TCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM1TCTRL' initialization */
  #ifdef PWMA_SM1TCTRL_VALUE
  PWMA_SM1TCTRL = PWMA_SM1TCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM2TCTRL' initialization */
  #ifdef PWMA_SM2TCTRL_VALUE
  PWMA_SM2TCTRL = PWMA_SM2TCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM3TCTRL' initialization */
  #ifdef PWMA_SM3TCTRL_VALUE
  PWMA_SM3TCTRL = PWMA_SM3TCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL0' initialization */
  #ifdef PWMA_SM0VAL0_VALUE
  PWMA_SM0VAL0 = PWMA_SM0VAL0_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL0' initialization */
  #ifdef PWMA_SM1VAL0_VALUE
  PWMA_SM1VAL0 = PWMA_SM1VAL0_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL0' initialization */
  #ifdef PWMA_SM2VAL0_VALUE
  PWMA_SM2VAL0 = PWMA_SM2VAL0_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL0' initialization */
  #ifdef PWMA_SM3VAL0_VALUE
  PWMA_SM3VAL0 = PWMA_SM3VAL0_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL1' initialization */
  #ifdef PWMA_SM0VAL1_VALUE
  PWMA_SM0VAL1 = PWMA_SM0VAL1_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL1' initialization */
  #ifdef PWMA_SM1VAL1_VALUE
  PWMA_SM1VAL1 = PWMA_SM1VAL1_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL1' initialization */
  #ifdef PWMA_SM2VAL1_VALUE
  PWMA_SM2VAL1 = PWMA_SM2VAL1_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL1' initialization */
  #ifdef PWMA_SM3VAL1_VALUE
  PWMA_SM3VAL1 = PWMA_SM3VAL1_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL2' initialization */
  #ifdef PWMA_SM0VAL2_VALUE
  PWMA_SM0VAL2 = PWMA_SM0VAL2_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL2' initialization */
  #ifdef PWMA_SM1VAL2_VALUE
  PWMA_SM1VAL2 = PWMA_SM1VAL2_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL2' initialization */
  #ifdef PWMA_SM2VAL2_VALUE
  PWMA_SM2VAL2 = PWMA_SM2VAL2_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL2' initialization */
  #ifdef PWMA_SM3VAL2_VALUE
  PWMA_SM3VAL2 = PWMA_SM3VAL2_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL3' initialization */
  #ifdef PWMA_SM0VAL3_VALUE
  PWMA_SM0VAL3 = PWMA_SM0VAL3_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL3' initialization */
  #ifdef PWMA_SM1VAL3_VALUE
  PWMA_SM1VAL3 = PWMA_SM1VAL3_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL3' initialization */
  #ifdef PWMA_SM2VAL3_VALUE
  PWMA_SM2VAL3 = PWMA_SM2VAL3_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL3' initialization */
  #ifdef PWMA_SM3VAL3_VALUE
  PWMA_SM3VAL3 = PWMA_SM3VAL3_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL4' initialization */
  #ifdef PWMA_SM0VAL4_VALUE
  PWMA_SM0VAL4 = PWMA_SM0VAL4_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL4' initialization */
  #ifdef PWMA_SM1VAL4_VALUE
  PWMA_SM1VAL4 = PWMA_SM1VAL4_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL4' initialization */
  #ifdef PWMA_SM2VAL4_VALUE
  PWMA_SM2VAL4 = PWMA_SM2VAL4_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL4' initialization */
  #ifdef PWMA_SM3VAL4_VALUE
  PWMA_SM3VAL4 = PWMA_SM3VAL4_VALUE;
  #endif

  /* Register 'PWMA_SM0VAL5' initialization */
  #ifdef PWMA_SM0VAL5_VALUE
  PWMA_SM0VAL5 = PWMA_SM0VAL5_VALUE;
  #endif

  /* Register 'PWMA_SM1VAL5' initialization */
  #ifdef PWMA_SM1VAL5_VALUE
  PWMA_SM1VAL5 = PWMA_SM1VAL5_VALUE;
  #endif

  /* Register 'PWMA_SM2VAL5' initialization */
  #ifdef PWMA_SM2VAL5_VALUE
  PWMA_SM2VAL5 = PWMA_SM2VAL5_VALUE;
  #endif

  /* Register 'PWMA_SM3VAL5' initialization */
  #ifdef PWMA_SM3VAL5_VALUE
  PWMA_SM3VAL5 = PWMA_SM3VAL5_VALUE;
  #endif

  /* Register 'PWMA_SM0FRACVAL1' initialization */
  #ifdef PWMA_SM0FRACVAL1_VALUE
  PWMA_SM0FRACVAL1 = PWMA_SM0FRACVAL1_VALUE;
  #endif

  /* Register 'PWMA_SM1FRACVAL1' initialization */
  #ifdef PWMA_SM1FRACVAL1_VALUE
  PWMA_SM1FRACVAL1 = PWMA_SM1FRACVAL1_VALUE;
  #endif

  /* Register 'PWMA_SM2FRACVAL1' initialization */
  #ifdef PWMA_SM2FRACVAL1_VALUE
  PWMA_SM2FRACVAL1 = PWMA_SM2FRACVAL1_VALUE;
  #endif

  /* Register 'PWMA_SM3FRACVAL1' initialization */
  #ifdef PWMA_SM3FRACVAL1_VALUE
  PWMA_SM3FRACVAL1 = PWMA_SM3FRACVAL1_VALUE;
  #endif

  /* Register 'PWMA_SM0FRACVAL2' initialization */
  #ifdef PWMA_SM0FRACVAL2_VALUE
  PWMA_SM0FRACVAL2 = PWMA_SM0FRACVAL2_VALUE;
  #endif

  /* Register 'PWMA_SM1FRACVAL2' initialization */
  #ifdef PWMA_SM1FRACVAL2_VALUE
  PWMA_SM1FRACVAL2 = PWMA_SM1FRACVAL2_VALUE;
  #endif

  /* Register 'PWMA_SM2FRACVAL2' initialization */
  #ifdef PWMA_SM2FRACVAL2_VALUE
  PWMA_SM2FRACVAL2 = PWMA_SM2FRACVAL2_VALUE;
  #endif

  /* Register 'PWMA_SM3FRACVAL2' initialization */
  #ifdef PWMA_SM3FRACVAL2_VALUE
  PWMA_SM3FRACVAL2 = PWMA_SM3FRACVAL2_VALUE;
  #endif

  /* Register 'PWMA_SM0FRACVAL3' initialization */
  #ifdef PWMA_SM0FRACVAL3_VALUE
  PWMA_SM0FRACVAL3 = PWMA_SM0FRACVAL3_VALUE;
  #endif

  /* Register 'PWMA_SM1FRACVAL3' initialization */
  #ifdef PWMA_SM1FRACVAL3_VALUE
  PWMA_SM1FRACVAL3 = PWMA_SM1FRACVAL3_VALUE;
  #endif

  /* Register 'PWMA_SM2FRACVAL3' initialization */
  #ifdef PWMA_SM2FRACVAL3_VALUE
  PWMA_SM2FRACVAL3 = PWMA_SM2FRACVAL3_VALUE;
  #endif

  /* Register 'PWMA_SM3FRACVAL3' initialization */
  #ifdef PWMA_SM3FRACVAL3_VALUE
  PWMA_SM3FRACVAL3 = PWMA_SM3FRACVAL3_VALUE;
  #endif

  /* Register 'PWMA_SM0FRACVAL4' initialization */
  #ifdef PWMA_SM0FRACVAL4_VALUE
  PWMA_SM0FRACVAL4 = PWMA_SM0FRACVAL4_VALUE;
  #endif

  /* Register 'PWMA_SM1FRACVAL4' initialization */
  #ifdef PWMA_SM1FRACVAL4_VALUE
  PWMA_SM1FRACVAL4 = PWMA_SM1FRACVAL4_VALUE;
  #endif

  /* Register 'PWMA_SM2FRACVAL4' initialization */
  #ifdef PWMA_SM2FRACVAL4_VALUE
  PWMA_SM2FRACVAL4 = PWMA_SM2FRACVAL4_VALUE;
  #endif

  /* Register 'PWMA_SM3FRACVAL4' initialization */
  #ifdef PWMA_SM3FRACVAL4_VALUE
  PWMA_SM3FRACVAL4 = PWMA_SM3FRACVAL4_VALUE;
  #endif

  /* Register 'PWMA_SM0FRACVAL5' initialization */
  #ifdef PWMA_SM0FRACVAL5_VALUE
  PWMA_SM0FRACVAL5 = PWMA_SM0FRACVAL5_VALUE;
  #endif

  /* Register 'PWMA_SM1FRACVAL5' initialization */
  #ifdef PWMA_SM1FRACVAL5_VALUE
  PWMA_SM1FRACVAL5 = PWMA_SM1FRACVAL5_VALUE;
  #endif

  /* Register 'PWMA_SM2FRACVAL5' initialization */
  #ifdef PWMA_SM2FRACVAL5_VALUE
  PWMA_SM2FRACVAL5 = PWMA_SM2FRACVAL5_VALUE;
  #endif

  /* Register 'PWMA_SM3FRACVAL5' initialization */
  #ifdef PWMA_SM3FRACVAL5_VALUE
  PWMA_SM3FRACVAL5 = PWMA_SM3FRACVAL5_VALUE;
  #endif

  /* Register 'PWMA_SM0FRCTRL' initialization */
  #ifdef PWMA_SM0FRCTRL_VALUE
  PWMA_SM0FRCTRL = PWMA_SM0FRCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM1FRCTRL' initialization */
  #ifdef PWMA_SM1FRCTRL_VALUE
  PWMA_SM1FRCTRL = PWMA_SM1FRCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM2FRCTRL' initialization */
  #ifdef PWMA_SM2FRCTRL_VALUE
  PWMA_SM2FRCTRL = PWMA_SM2FRCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM3FRCTRL' initialization */
  #ifdef PWMA_SM3FRCTRL_VALUE
  PWMA_SM3FRCTRL = PWMA_SM3FRCTRL_VALUE;
  #endif

  /* Register 'PWMA_SM0DISMAP0' initialization */
  #ifdef PWMA_SM0DISMAP0_VALUE
  PWMA_SM0DISMAP0 = PWMA_SM0DISMAP0_VALUE;
  #endif

  /* Register 'PWMA_SM1DISMAP0' initialization */
  #ifdef PWMA_SM1DISMAP0_VALUE
  PWMA_SM1DISMAP0 = PWMA_SM1DISMAP0_VALUE;
  #endif

  /* Register 'PWMA_SM2DISMAP0' initialization */
  #ifdef PWMA_SM2DISMAP0_VALUE
  PWMA_SM2DISMAP0 = PWMA_SM2DISMAP0_VALUE;
  #endif

  /* Register 'PWMA_SM3DISMAP0' initialization */
  #ifdef PWMA_SM3DISMAP0_VALUE
  PWMA_SM3DISMAP0 = PWMA_SM3DISMAP0_VALUE;
  #endif

  /* Register 'PWMA_SM0DISMAP1' initialization */
  #ifdef PWMA_SM0DISMAP1_VALUE
  PWMA_SM0DISMAP1 = PWMA_SM0DISMAP1_VALUE;
  #endif

  /* Register 'PWMA_SM1DISMAP1' initialization */
  #ifdef PWMA_SM1DISMAP1_VALUE
  PWMA_SM1DISMAP1 = PWMA_SM1DISMAP1_VALUE;
  #endif

  /* Register 'PWMA_SM2DISMAP1' initialization */
  #ifdef PWMA_SM2DISMAP1_VALUE
  PWMA_SM2DISMAP1 = PWMA_SM2DISMAP1_VALUE;
  #endif

  /* Register 'PWMA_SM3DISMAP1' initialization */
  #ifdef PWMA_SM3DISMAP1_VALUE
  PWMA_SM3DISMAP1 = PWMA_SM3DISMAP1_VALUE;
  #endif

  /* Register 'PWMA_SM0DTCNT0' initialization */
  #ifdef PWMA_SM0DTCNT0_VALUE
  PWMA_SM0DTCNT0 = PWMA_SM0DTCNT0_VALUE;
  #endif

  /* Register 'PWMA_SM1DTCNT0' initialization */
  #ifdef PWMA_SM1DTCNT0_VALUE
  PWMA_SM1DTCNT0 = PWMA_SM1DTCNT0_VALUE;
  #endif

  /* Register 'PWMA_SM2DTCNT0' initialization */
  #ifdef PWMA_SM2DTCNT0_VALUE
  PWMA_SM2DTCNT0 = PWMA_SM2DTCNT0_VALUE;
  #endif

  /* Register 'PWMA_SM3DTCNT0' initialization */
  #ifdef PWMA_SM3DTCNT0_VALUE
  PWMA_SM3DTCNT0 = PWMA_SM3DTCNT0_VALUE;
  #endif

  /* Register 'PWMA_SM0DTCNT1' initialization */
  #ifdef PWMA_SM0DTCNT1_VALUE
  PWMA_SM0DTCNT1 = PWMA_SM0DTCNT1_VALUE;
  #endif

  /* Register 'PWMA_SM1DTCNT1' initialization */
  #ifdef PWMA_SM1DTCNT1_VALUE
  PWMA_SM1DTCNT1 = PWMA_SM1DTCNT1_VALUE;
  #endif

  /* Register 'PWMA_SM2DTCNT1' initialization */
  #ifdef PWMA_SM2DTCNT1_VALUE
  PWMA_SM2DTCNT1 = PWMA_SM2DTCNT1_VALUE;
  #endif

  /* Register 'PWMA_SM3DTCNT1' initialization */
  #ifdef PWMA_SM3DTCNT1_VALUE
  PWMA_SM3DTCNT1 = PWMA_SM3DTCNT1_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCOMPA' initialization */
  #ifdef PWMA_SM0CAPTCOMPA_VALUE
  PWMA_SM0CAPTCOMPA = PWMA_SM0CAPTCOMPA_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCOMPA' initialization */
  #ifdef PWMA_SM1CAPTCOMPA_VALUE
  PWMA_SM1CAPTCOMPA = PWMA_SM1CAPTCOMPA_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCOMPA' initialization */
  #ifdef PWMA_SM2CAPTCOMPA_VALUE
  PWMA_SM2CAPTCOMPA = PWMA_SM2CAPTCOMPA_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCOMPA' initialization */
  #ifdef PWMA_SM3CAPTCOMPA_VALUE
  PWMA_SM3CAPTCOMPA = PWMA_SM3CAPTCOMPA_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCOMPB' initialization */
  #ifdef PWMA_SM0CAPTCOMPB_VALUE
  PWMA_SM0CAPTCOMPB = PWMA_SM0CAPTCOMPB_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCOMPB' initialization */
  #ifdef PWMA_SM1CAPTCOMPB_VALUE
  PWMA_SM1CAPTCOMPB = PWMA_SM1CAPTCOMPB_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCOMPB' initialization */
  #ifdef PWMA_SM2CAPTCOMPB_VALUE
  PWMA_SM2CAPTCOMPB = PWMA_SM2CAPTCOMPB_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCOMPB' initialization */
  #ifdef PWMA_SM3CAPTCOMPB_VALUE
  PWMA_SM3CAPTCOMPB = PWMA_SM3CAPTCOMPB_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCOMPX' initialization */
  #ifdef PWMA_SM0CAPTCOMPX_VALUE
  PWMA_SM0CAPTCOMPX = PWMA_SM0CAPTCOMPX_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCOMPX' initialization */
  #ifdef PWMA_SM1CAPTCOMPX_VALUE
  PWMA_SM1CAPTCOMPX = PWMA_SM1CAPTCOMPX_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCOMPX' initialization */
  #ifdef PWMA_SM2CAPTCOMPX_VALUE
  PWMA_SM2CAPTCOMPX = PWMA_SM2CAPTCOMPX_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCOMPX' initialization */
  #ifdef PWMA_SM3CAPTCOMPX_VALUE
  PWMA_SM3CAPTCOMPX = PWMA_SM3CAPTCOMPX_VALUE;
  #endif

  /* Register 'PWMA_FFILT0' initialization */
  #if PWMA_FFILT0_MASK
    #if PWMA_FFILT0_MASK == 0xFFFF
  PWMA_FFILT0 = PWMA_FFILT0_VALUE;
    #elif PWMA_FFILT0_MASK == PWMA_FFILT0_VALUE
  PWMA_FFILT0 |= PWMA_FFILT0_VALUE;
    #elif PWMA_FFILT0_VALUE == 0
  PWMA_FFILT0 &= ~PWMA_FFILT0_MASK;
    #else
  PWMA_FFILT0 = (PWMA_FFILT0 & (~PWMA_FFILT0_MASK)) | PWMA_FFILT0_VALUE;
    #endif
  #elif defined(PWMA_FFILT0_VALUE)
  PWMA_FFILT0 = PWMA_FFILT0_VALUE;
  #endif

  /* Register 'PWMA_FFILT1' initialization */
  #if PWMA_FFILT1_MASK
    #if PWMA_FFILT1_MASK == 0xFFFF
  PWMA_FFILT1 = PWMA_FFILT1_VALUE;
    #elif PWMA_FFILT1_MASK == PWMA_FFILT1_VALUE
  PWMA_FFILT1 |= PWMA_FFILT1_VALUE;
    #elif PWMA_FFILT1_VALUE == 0
  PWMA_FFILT1 &= ~PWMA_FFILT1_MASK;
    #else
  PWMA_FFILT1 = (PWMA_FFILT1 & (~PWMA_FFILT1_MASK)) | PWMA_FFILT1_VALUE;
    #endif
  #elif defined(PWMA_FFILT1_VALUE)
  PWMA_FFILT1 = PWMA_FFILT1_VALUE;
  #endif

  /* Register 'PWMA_MASK' initialization */
  #if PWMA_MASK_MASK
    #if PWMA_MASK_MASK == 0xFFFF
  PWMA_MASK = PWMA_MASK_VALUE;
    #elif PWMA_MASK_MASK == PWMA_MASK_VALUE
  PWMA_MASK |= PWMA_MASK_VALUE;
    #elif PWMA_MASK_VALUE == 0
  PWMA_MASK &= ~PWMA_MASK_MASK;
    #else
  PWMA_MASK = (PWMA_MASK & (~PWMA_MASK_MASK)) | PWMA_MASK_VALUE;
    #endif
  #elif defined(PWMA_MASK_VALUE)
  PWMA_MASK = PWMA_MASK_VALUE;
  #endif

  /* Register 'PWMA_SWCOUT' initialization */
  #if PWMA_SWCOUT_MASK
    #if PWMA_SWCOUT_MASK == 0xFFFF
  PWMA_SWCOUT = PWMA_SWCOUT_VALUE;
    #elif PWMA_SWCOUT_MASK == PWMA_SWCOUT_VALUE
  PWMA_SWCOUT |= PWMA_SWCOUT_VALUE;
    #elif PWMA_SWCOUT_VALUE == 0
  PWMA_SWCOUT &= ~PWMA_SWCOUT_MASK;
    #else
  PWMA_SWCOUT = (PWMA_SWCOUT & (~PWMA_SWCOUT_MASK)) | PWMA_SWCOUT_VALUE;
    #endif
  #elif defined(PWMA_SWCOUT_VALUE)
  PWMA_SWCOUT = PWMA_SWCOUT_VALUE;
  #endif

  /* Register 'PWMA_DTSRCSEL' initialization */
  #if PWMA_DTSRCSEL_MASK
    #if PWMA_DTSRCSEL_MASK == 0xFFFF
  PWMA_DTSRCSEL = PWMA_DTSRCSEL_VALUE;
    #elif PWMA_DTSRCSEL_MASK == PWMA_DTSRCSEL_VALUE
  PWMA_DTSRCSEL |= PWMA_DTSRCSEL_VALUE;
    #elif PWMA_DTSRCSEL_VALUE == 0
  PWMA_DTSRCSEL &= ~PWMA_DTSRCSEL_MASK;
    #else
  PWMA_DTSRCSEL = (PWMA_DTSRCSEL & (~PWMA_DTSRCSEL_MASK)) | PWMA_DTSRCSEL_VALUE;
    #endif
  #elif defined(PWMA_DTSRCSEL_VALUE)
  PWMA_DTSRCSEL = PWMA_DTSRCSEL_VALUE;
  #endif

  /* Register 'PWMA_MCTRL2' initialization */
  #ifdef PWMA_MCTRL2_VALUE
  PWMA_MCTRL2 = PWMA_MCTRL2_VALUE;
  #endif

  /* Register 'PWMA_OUTEN' initialization */
  #if PWMA_OUTEN_MASK
    #if PWMA_OUTEN_MASK == 0xFFFF
  PWMA_OUTEN = PWMA_OUTEN_VALUE;
    #elif PWMA_OUTEN_MASK == PWMA_OUTEN_VALUE
  PWMA_OUTEN |= PWMA_OUTEN_VALUE;
    #elif PWMA_OUTEN_VALUE == 0
  PWMA_OUTEN &= ~PWMA_OUTEN_MASK;
    #else
  PWMA_OUTEN = (PWMA_OUTEN & (~PWMA_OUTEN_MASK)) | PWMA_OUTEN_VALUE;
    #endif
  #elif defined(PWMA_OUTEN_VALUE)
  PWMA_OUTEN = PWMA_OUTEN_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCTRLA' initialization */
  #ifdef PWMA_SM0CAPTCTRLA_VALUE
  PWMA_SM0CAPTCTRLA = PWMA_SM0CAPTCTRLA_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCTRLA' initialization */
  #ifdef PWMA_SM1CAPTCTRLA_VALUE
  PWMA_SM1CAPTCTRLA = PWMA_SM1CAPTCTRLA_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCTRLA' initialization */
  #ifdef PWMA_SM2CAPTCTRLA_VALUE
  PWMA_SM2CAPTCTRLA = PWMA_SM2CAPTCTRLA_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCTRLA' initialization */
  #ifdef PWMA_SM3CAPTCTRLA_VALUE
  PWMA_SM3CAPTCTRLA = PWMA_SM3CAPTCTRLA_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCTRLB' initialization */
  #ifdef PWMA_SM0CAPTCTRLB_VALUE
  PWMA_SM0CAPTCTRLB = PWMA_SM0CAPTCTRLB_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCTRLB' initialization */
  #ifdef PWMA_SM1CAPTCTRLB_VALUE
  PWMA_SM1CAPTCTRLB = PWMA_SM1CAPTCTRLB_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCTRLB' initialization */
  #ifdef PWMA_SM2CAPTCTRLB_VALUE
  PWMA_SM2CAPTCTRLB = PWMA_SM2CAPTCTRLB_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCTRLB' initialization */
  #ifdef PWMA_SM3CAPTCTRLB_VALUE
  PWMA_SM3CAPTCTRLB = PWMA_SM3CAPTCTRLB_VALUE;
  #endif

  /* Register 'PWMA_SM0CAPTCTRLX' initialization */
  #ifdef PWMA_SM0CAPTCTRLX_VALUE
  PWMA_SM0CAPTCTRLX = PWMA_SM0CAPTCTRLX_VALUE;
  #endif

  /* Register 'PWMA_SM1CAPTCTRLX' initialization */
  #ifdef PWMA_SM1CAPTCTRLX_VALUE
  PWMA_SM1CAPTCTRLX = PWMA_SM1CAPTCTRLX_VALUE;
  #endif

  /* Register 'PWMA_SM2CAPTCTRLX' initialization */
  #ifdef PWMA_SM2CAPTCTRLX_VALUE
  PWMA_SM2CAPTCTRLX = PWMA_SM2CAPTCTRLX_VALUE;
  #endif

  /* Register 'PWMA_SM3CAPTCTRLX' initialization */
  #ifdef PWMA_SM3CAPTCTRLX_VALUE
  PWMA_SM3CAPTCTRLX = PWMA_SM3CAPTCTRLX_VALUE;
  #endif

  /* Register 'PWMA_MCTRL' initialization */
  #if PWMA_MCTRL_MASK_2
    #if PWMA_MCTRL_MASK_2 == 0xFFFF
  PWMA_MCTRL = PWMA_MCTRL_VALUE_2;
    #elif PWMA_MCTRL_MASK_2 == PWMA_MCTRL_VALUE_2
  PWMA_MCTRL |= PWMA_MCTRL_VALUE_2;
    #elif PWMA_MCTRL_VALUE_2 == 0
  PWMA_MCTRL &= ~PWMA_MCTRL_MASK_2;
    #else
  PWMA_MCTRL = (PWMA_MCTRL & (~PWMA_MCTRL_MASK_2)) | PWMA_MCTRL_VALUE_2;
    #endif
  #elif defined(PWMA_MCTRL_VALUE_2)
  PWMA_MCTRL = PWMA_MCTRL_VALUE_2;
  #endif

  /* Register 'PWMA_FSTS1' initialization */
  #ifdef PWMA_FSTS1_VALUE
  PWMA_FSTS1 = PWMA_FSTS1_VALUE;
  #endif

  /* Register 'PWMA_FSTS0' initialization */
  #ifdef PWMA_FSTS0_VALUE
  PWMA_FSTS0 = PWMA_FSTS0_VALUE;
  #endif

  /* Register 'PWMA_FCTRL0' initialization */
  #ifdef PWMA_FCTRL0_VALUE_2
  PWMA_FCTRL0 = PWMA_FCTRL0_VALUE_2;
  #endif

  /* Register 'PWMA_FCTRL1' initialization */
  #ifdef PWMA_FCTRL1_VALUE_2
  PWMA_FCTRL1 = PWMA_FCTRL1_VALUE_2;
  #endif

  /* Register 'PWMA_MCTRL' initialization */
  #if PWMA_MCTRL_MASK_3
    #if PWMA_MCTRL_MASK_3 == 0xFFFF
  PWMA_MCTRL = PWMA_MCTRL_VALUE_3;
    #elif PWMA_MCTRL_MASK_3 == PWMA_MCTRL_VALUE_3
  PWMA_MCTRL |= PWMA_MCTRL_VALUE_3;
    #elif PWMA_MCTRL_VALUE_3 == 0
  PWMA_MCTRL &= ~PWMA_MCTRL_MASK_3;
    #else
  PWMA_MCTRL = (PWMA_MCTRL & (~PWMA_MCTRL_MASK_3)) | PWMA_MCTRL_VALUE_3;
    #endif
  #elif defined(PWMA_MCTRL_VALUE_3)
  PWMA_MCTRL = PWMA_MCTRL_VALUE_3;
  #endif

  /* Register 'PWMA_SM0STS' initialization */
  #ifdef PWMA_SM0STS_VALUE
  PWMA_SM0STS = PWMA_SM0STS_VALUE;
  #endif

  /* Register 'PWMA_SM1STS' initialization */
  #ifdef PWMA_SM1STS_VALUE
  PWMA_SM1STS = PWMA_SM1STS_VALUE;
  #endif

  /* Register 'PWMA_SM2STS' initialization */
  #ifdef PWMA_SM2STS_VALUE
  PWMA_SM2STS = PWMA_SM2STS_VALUE;
  #endif

  /* Register 'PWMA_SM3STS' initialization */
  #ifdef PWMA_SM3STS_VALUE
  PWMA_SM3STS = PWMA_SM3STS_VALUE;
  #endif

  /* Register 'PWMA_SM0INTEN' initialization */
  #ifdef PWMA_SM0INTEN_VALUE_2
  PWMA_SM0INTEN = PWMA_SM0INTEN_VALUE_2;
  #endif

  /* Register 'PWMA_SM1INTEN' initialization */
  #ifdef PWMA_SM1INTEN_VALUE_2
  PWMA_SM1INTEN = PWMA_SM1INTEN_VALUE_2;
  #endif

  /* Register 'PWMA_SM2INTEN' initialization */
  #ifdef PWMA_SM2INTEN_VALUE_2
  PWMA_SM2INTEN = PWMA_SM2INTEN_VALUE_2;
  #endif

  /* Register 'PWMA_SM3INTEN' initialization */
  #ifdef PWMA_SM3INTEN_VALUE_2
  PWMA_SM3INTEN = PWMA_SM3INTEN_VALUE_2;
  #endif

  /* Register 'PWMA_SM0DMAEN' initialization */
  #ifdef PWMA_SM0DMAEN_VALUE
  PWMA_SM0DMAEN = PWMA_SM0DMAEN_VALUE;
  #endif

  /* Register 'PWMA_SM1DMAEN' initialization */
  #ifdef PWMA_SM1DMAEN_VALUE
  PWMA_SM1DMAEN = PWMA_SM1DMAEN_VALUE;
  #endif

  /* Register 'PWMA_SM2DMAEN' initialization */
  #ifdef PWMA_SM2DMAEN_VALUE
  PWMA_SM2DMAEN = PWMA_SM2DMAEN_VALUE;
  #endif

  /* Register 'PWMA_SM3DMAEN' initialization */
  #ifdef PWMA_SM3DMAEN_VALUE
  PWMA_SM3DMAEN = PWMA_SM3DMAEN_VALUE;
  #endif

  /* Register 'PWMA_FTST0' initialization */
  #ifdef PWMA_FTST0_VALUE
  PWMA_FTST0 = PWMA_FTST0_VALUE;
  #endif

  /* Register 'PWMA_FTST1' initialization */
  #ifdef PWMA_FTST1_VALUE
  PWMA_FTST1 = PWMA_FTST1_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END PWMA_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
