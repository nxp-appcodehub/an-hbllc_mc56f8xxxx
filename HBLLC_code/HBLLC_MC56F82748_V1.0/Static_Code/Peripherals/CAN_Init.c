/** ###################################################################
 **     Filename    : CAN_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the CAN module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void CAN_Init(void);
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
 * @file CAN_Init.c
 * @brief This file implements the CAN module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE CAN_Init. */

#include "CAN_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN_Init(void) {

  CAN_CTL0 |= 0x1;
  while((CAN_CTL1 & CAN_CTL1_INITAK_MASK) == 0U) {}

  /* Register 'CAN_IDAR3' initialization */
  #if CAN_IDAR3_MASK
    #if CAN_IDAR3_MASK == 0xFFFF
  CAN_IDAR3 = CAN_IDAR3_VALUE;
    #elif CAN_IDAR3_MASK == CAN_IDAR3_VALUE
  CAN_IDAR3 |= CAN_IDAR3_VALUE;
    #elif CAN_IDAR3_VALUE == 0
  CAN_IDAR3 &= ~CAN_IDAR3_MASK;
    #else
  CAN_IDAR3 = (CAN_IDAR3 & (~CAN_IDAR3_MASK)) | CAN_IDAR3_VALUE;
    #endif
  #elif defined(CAN_IDAR3_VALUE)
  CAN_IDAR3 = CAN_IDAR3_VALUE;
  #endif

  /* Register 'CAN_IDAR2' initialization */
  #if CAN_IDAR2_MASK
    #if CAN_IDAR2_MASK == 0xFFFF
  CAN_IDAR2 = CAN_IDAR2_VALUE;
    #elif CAN_IDAR2_MASK == CAN_IDAR2_VALUE
  CAN_IDAR2 |= CAN_IDAR2_VALUE;
    #elif CAN_IDAR2_VALUE == 0
  CAN_IDAR2 &= ~CAN_IDAR2_MASK;
    #else
  CAN_IDAR2 = (CAN_IDAR2 & (~CAN_IDAR2_MASK)) | CAN_IDAR2_VALUE;
    #endif
  #elif defined(CAN_IDAR2_VALUE)
  CAN_IDAR2 = CAN_IDAR2_VALUE;
  #endif

  /* Register 'CAN_IDAR1' initialization */
  #if CAN_IDAR1_MASK
    #if CAN_IDAR1_MASK == 0xFFFF
  CAN_IDAR1 = CAN_IDAR1_VALUE;
    #elif CAN_IDAR1_MASK == CAN_IDAR1_VALUE
  CAN_IDAR1 |= CAN_IDAR1_VALUE;
    #elif CAN_IDAR1_VALUE == 0
  CAN_IDAR1 &= ~CAN_IDAR1_MASK;
    #else
  CAN_IDAR1 = (CAN_IDAR1 & (~CAN_IDAR1_MASK)) | CAN_IDAR1_VALUE;
    #endif
  #elif defined(CAN_IDAR1_VALUE)
  CAN_IDAR1 = CAN_IDAR1_VALUE;
  #endif

  /* Register 'CAN_IDAR0' initialization */
  #if CAN_IDAR0_MASK
    #if CAN_IDAR0_MASK == 0xFFFF
  CAN_IDAR0 = CAN_IDAR0_VALUE;
    #elif CAN_IDAR0_MASK == CAN_IDAR0_VALUE
  CAN_IDAR0 |= CAN_IDAR0_VALUE;
    #elif CAN_IDAR0_VALUE == 0
  CAN_IDAR0 &= ~CAN_IDAR0_MASK;
    #else
  CAN_IDAR0 = (CAN_IDAR0 & (~CAN_IDAR0_MASK)) | CAN_IDAR0_VALUE;
    #endif
  #elif defined(CAN_IDAR0_VALUE)
  CAN_IDAR0 = CAN_IDAR0_VALUE;
  #endif

  /* Register 'CAN_IDAR7' initialization */
  #if CAN_IDAR7_MASK
    #if CAN_IDAR7_MASK == 0xFFFF
  CAN_IDAR7 = CAN_IDAR7_VALUE;
    #elif CAN_IDAR7_MASK == CAN_IDAR7_VALUE
  CAN_IDAR7 |= CAN_IDAR7_VALUE;
    #elif CAN_IDAR7_VALUE == 0
  CAN_IDAR7 &= ~CAN_IDAR7_MASK;
    #else
  CAN_IDAR7 = (CAN_IDAR7 & (~CAN_IDAR7_MASK)) | CAN_IDAR7_VALUE;
    #endif
  #elif defined(CAN_IDAR7_VALUE)
  CAN_IDAR7 = CAN_IDAR7_VALUE;
  #endif

  /* Register 'CAN_IDAR6' initialization */
  #if CAN_IDAR6_MASK
    #if CAN_IDAR6_MASK == 0xFFFF
  CAN_IDAR6 = CAN_IDAR6_VALUE;
    #elif CAN_IDAR6_MASK == CAN_IDAR6_VALUE
  CAN_IDAR6 |= CAN_IDAR6_VALUE;
    #elif CAN_IDAR6_VALUE == 0
  CAN_IDAR6 &= ~CAN_IDAR6_MASK;
    #else
  CAN_IDAR6 = (CAN_IDAR6 & (~CAN_IDAR6_MASK)) | CAN_IDAR6_VALUE;
    #endif
  #elif defined(CAN_IDAR6_VALUE)
  CAN_IDAR6 = CAN_IDAR6_VALUE;
  #endif

  /* Register 'CAN_IDAR5' initialization */
  #if CAN_IDAR5_MASK
    #if CAN_IDAR5_MASK == 0xFFFF
  CAN_IDAR5 = CAN_IDAR5_VALUE;
    #elif CAN_IDAR5_MASK == CAN_IDAR5_VALUE
  CAN_IDAR5 |= CAN_IDAR5_VALUE;
    #elif CAN_IDAR5_VALUE == 0
  CAN_IDAR5 &= ~CAN_IDAR5_MASK;
    #else
  CAN_IDAR5 = (CAN_IDAR5 & (~CAN_IDAR5_MASK)) | CAN_IDAR5_VALUE;
    #endif
  #elif defined(CAN_IDAR5_VALUE)
  CAN_IDAR5 = CAN_IDAR5_VALUE;
  #endif

  /* Register 'CAN_IDAR4' initialization */
  #if CAN_IDAR4_MASK
    #if CAN_IDAR4_MASK == 0xFFFF
  CAN_IDAR4 = CAN_IDAR4_VALUE;
    #elif CAN_IDAR4_MASK == CAN_IDAR4_VALUE
  CAN_IDAR4 |= CAN_IDAR4_VALUE;
    #elif CAN_IDAR4_VALUE == 0
  CAN_IDAR4 &= ~CAN_IDAR4_MASK;
    #else
  CAN_IDAR4 = (CAN_IDAR4 & (~CAN_IDAR4_MASK)) | CAN_IDAR4_VALUE;
    #endif
  #elif defined(CAN_IDAR4_VALUE)
  CAN_IDAR4 = CAN_IDAR4_VALUE;
  #endif

  /* Register 'CAN_IDMR3' initialization */
  #if CAN_IDMR3_MASK
    #if CAN_IDMR3_MASK == 0xFFFF
  CAN_IDMR3 = CAN_IDMR3_VALUE;
    #elif CAN_IDMR3_MASK == CAN_IDMR3_VALUE
  CAN_IDMR3 |= CAN_IDMR3_VALUE;
    #elif CAN_IDMR3_VALUE == 0
  CAN_IDMR3 &= ~CAN_IDMR3_MASK;
    #else
  CAN_IDMR3 = (CAN_IDMR3 & (~CAN_IDMR3_MASK)) | CAN_IDMR3_VALUE;
    #endif
  #elif defined(CAN_IDMR3_VALUE)
  CAN_IDMR3 = CAN_IDMR3_VALUE;
  #endif

  /* Register 'CAN_IDMR2' initialization */
  #if CAN_IDMR2_MASK
    #if CAN_IDMR2_MASK == 0xFFFF
  CAN_IDMR2 = CAN_IDMR2_VALUE;
    #elif CAN_IDMR2_MASK == CAN_IDMR2_VALUE
  CAN_IDMR2 |= CAN_IDMR2_VALUE;
    #elif CAN_IDMR2_VALUE == 0
  CAN_IDMR2 &= ~CAN_IDMR2_MASK;
    #else
  CAN_IDMR2 = (CAN_IDMR2 & (~CAN_IDMR2_MASK)) | CAN_IDMR2_VALUE;
    #endif
  #elif defined(CAN_IDMR2_VALUE)
  CAN_IDMR2 = CAN_IDMR2_VALUE;
  #endif

  /* Register 'CAN_IDMR1' initialization */
  #if CAN_IDMR1_MASK
    #if CAN_IDMR1_MASK == 0xFFFF
  CAN_IDMR1 = CAN_IDMR1_VALUE;
    #elif CAN_IDMR1_MASK == CAN_IDMR1_VALUE
  CAN_IDMR1 |= CAN_IDMR1_VALUE;
    #elif CAN_IDMR1_VALUE == 0
  CAN_IDMR1 &= ~CAN_IDMR1_MASK;
    #else
  CAN_IDMR1 = (CAN_IDMR1 & (~CAN_IDMR1_MASK)) | CAN_IDMR1_VALUE;
    #endif
  #elif defined(CAN_IDMR1_VALUE)
  CAN_IDMR1 = CAN_IDMR1_VALUE;
  #endif

  /* Register 'CAN_IDMR0' initialization */
  #if CAN_IDMR0_MASK
    #if CAN_IDMR0_MASK == 0xFFFF
  CAN_IDMR0 = CAN_IDMR0_VALUE;
    #elif CAN_IDMR0_MASK == CAN_IDMR0_VALUE
  CAN_IDMR0 |= CAN_IDMR0_VALUE;
    #elif CAN_IDMR0_VALUE == 0
  CAN_IDMR0 &= ~CAN_IDMR0_MASK;
    #else
  CAN_IDMR0 = (CAN_IDMR0 & (~CAN_IDMR0_MASK)) | CAN_IDMR0_VALUE;
    #endif
  #elif defined(CAN_IDMR0_VALUE)
  CAN_IDMR0 = CAN_IDMR0_VALUE;
  #endif

  /* Register 'CAN_IDMR7' initialization */
  #if CAN_IDMR7_MASK
    #if CAN_IDMR7_MASK == 0xFFFF
  CAN_IDMR7 = CAN_IDMR7_VALUE;
    #elif CAN_IDMR7_MASK == CAN_IDMR7_VALUE
  CAN_IDMR7 |= CAN_IDMR7_VALUE;
    #elif CAN_IDMR7_VALUE == 0
  CAN_IDMR7 &= ~CAN_IDMR7_MASK;
    #else
  CAN_IDMR7 = (CAN_IDMR7 & (~CAN_IDMR7_MASK)) | CAN_IDMR7_VALUE;
    #endif
  #elif defined(CAN_IDMR7_VALUE)
  CAN_IDMR7 = CAN_IDMR7_VALUE;
  #endif

  /* Register 'CAN_IDMR6' initialization */
  #if CAN_IDMR6_MASK
    #if CAN_IDMR6_MASK == 0xFFFF
  CAN_IDMR6 = CAN_IDMR6_VALUE;
    #elif CAN_IDMR6_MASK == CAN_IDMR6_VALUE
  CAN_IDMR6 |= CAN_IDMR6_VALUE;
    #elif CAN_IDMR6_VALUE == 0
  CAN_IDMR6 &= ~CAN_IDMR6_MASK;
    #else
  CAN_IDMR6 = (CAN_IDMR6 & (~CAN_IDMR6_MASK)) | CAN_IDMR6_VALUE;
    #endif
  #elif defined(CAN_IDMR6_VALUE)
  CAN_IDMR6 = CAN_IDMR6_VALUE;
  #endif

  /* Register 'CAN_IDMR5' initialization */
  #if CAN_IDMR5_MASK
    #if CAN_IDMR5_MASK == 0xFFFF
  CAN_IDMR5 = CAN_IDMR5_VALUE;
    #elif CAN_IDMR5_MASK == CAN_IDMR5_VALUE
  CAN_IDMR5 |= CAN_IDMR5_VALUE;
    #elif CAN_IDMR5_VALUE == 0
  CAN_IDMR5 &= ~CAN_IDMR5_MASK;
    #else
  CAN_IDMR5 = (CAN_IDMR5 & (~CAN_IDMR5_MASK)) | CAN_IDMR5_VALUE;
    #endif
  #elif defined(CAN_IDMR5_VALUE)
  CAN_IDMR5 = CAN_IDMR5_VALUE;
  #endif

  /* Register 'CAN_IDMR4' initialization */
  #if CAN_IDMR4_MASK
    #if CAN_IDMR4_MASK == 0xFFFF
  CAN_IDMR4 = CAN_IDMR4_VALUE;
    #elif CAN_IDMR4_MASK == CAN_IDMR4_VALUE
  CAN_IDMR4 |= CAN_IDMR4_VALUE;
    #elif CAN_IDMR4_VALUE == 0
  CAN_IDMR4 &= ~CAN_IDMR4_MASK;
    #else
  CAN_IDMR4 = (CAN_IDMR4 & (~CAN_IDMR4_MASK)) | CAN_IDMR4_VALUE;
    #endif
  #elif defined(CAN_IDMR4_VALUE)
  CAN_IDMR4 = CAN_IDMR4_VALUE;
  #endif

  /* Register 'CAN_IDAC' initialization */
  #if CAN_IDAC_MASK
    #if CAN_IDAC_MASK == 0xFFFF
  CAN_IDAC = CAN_IDAC_VALUE;
    #elif CAN_IDAC_MASK == CAN_IDAC_VALUE
  CAN_IDAC |= CAN_IDAC_VALUE;
    #elif CAN_IDAC_VALUE == 0
  CAN_IDAC &= ~CAN_IDAC_MASK;
    #else
  CAN_IDAC = (CAN_IDAC & (~CAN_IDAC_MASK)) | CAN_IDAC_VALUE;
    #endif
  #elif defined(CAN_IDAC_VALUE)
  CAN_IDAC = CAN_IDAC_VALUE;
  #endif

  /* Register 'CAN_BTR0' initialization */
  #if CAN_BTR0_MASK
    #if CAN_BTR0_MASK == 0xFFFF
  CAN_BTR0 = CAN_BTR0_VALUE;
    #elif CAN_BTR0_MASK == CAN_BTR0_VALUE
  CAN_BTR0 |= CAN_BTR0_VALUE;
    #elif CAN_BTR0_VALUE == 0
  CAN_BTR0 &= ~CAN_BTR0_MASK;
    #else
  CAN_BTR0 = (CAN_BTR0 & (~CAN_BTR0_MASK)) | CAN_BTR0_VALUE;
    #endif
  #elif defined(CAN_BTR0_VALUE)
  CAN_BTR0 = CAN_BTR0_VALUE;
  #endif

  /* Register 'CAN_BTR1' initialization */
  #if CAN_BTR1_MASK
    #if CAN_BTR1_MASK == 0xFFFF
  CAN_BTR1 = CAN_BTR1_VALUE;
    #elif CAN_BTR1_MASK == CAN_BTR1_VALUE
  CAN_BTR1 |= CAN_BTR1_VALUE;
    #elif CAN_BTR1_VALUE == 0
  CAN_BTR1 &= ~CAN_BTR1_MASK;
    #else
  CAN_BTR1 = (CAN_BTR1 & (~CAN_BTR1_MASK)) | CAN_BTR1_VALUE;
    #endif
  #elif defined(CAN_BTR1_VALUE)
  CAN_BTR1 = CAN_BTR1_VALUE;
  #endif

  /* Register 'CAN_CTL1' initialization */
  #if CAN_CTL1_MASK_2
    #if CAN_CTL1_MASK_2 == 0xFFFF
  CAN_CTL1 = CAN_CTL1_VALUE_2;
    #elif CAN_CTL1_MASK_2 == CAN_CTL1_VALUE_2
  CAN_CTL1 |= CAN_CTL1_VALUE_2;
    #elif CAN_CTL1_VALUE_2 == 0
  CAN_CTL1 &= ~CAN_CTL1_MASK_2;
    #else
  CAN_CTL1 = (CAN_CTL1 & (~CAN_CTL1_MASK_2)) | CAN_CTL1_VALUE_2;
    #endif
  #elif defined(CAN_CTL1_VALUE_2)
  CAN_CTL1 = CAN_CTL1_VALUE_2;
  #endif

  CAN_CTL0 &= ~0x1;
  while((CAN_CTL1 & CAN_CTL1_INITAK_MASK) != 0U) {}

  /* Register 'CAN_RIER' initialization */
  #if CAN_RIER_MASK
    #if CAN_RIER_MASK == 0xFFFF
  CAN_RIER = CAN_RIER_VALUE;
    #elif CAN_RIER_MASK == CAN_RIER_VALUE
  CAN_RIER |= CAN_RIER_VALUE;
    #elif CAN_RIER_VALUE == 0
  CAN_RIER &= ~CAN_RIER_MASK;
    #else
  CAN_RIER = (CAN_RIER & (~CAN_RIER_MASK)) | CAN_RIER_VALUE;
    #endif
  #elif defined(CAN_RIER_VALUE)
  CAN_RIER = CAN_RIER_VALUE;
  #endif

  /* Register 'CAN_TIER' initialization */
  #if CAN_TIER_MASK
    #if CAN_TIER_MASK == 0xFFFF
  CAN_TIER = CAN_TIER_VALUE;
    #elif CAN_TIER_MASK == CAN_TIER_VALUE
  CAN_TIER |= CAN_TIER_VALUE;
    #elif CAN_TIER_VALUE == 0
  CAN_TIER &= ~CAN_TIER_MASK;
    #else
  CAN_TIER = (CAN_TIER & (~CAN_TIER_MASK)) | CAN_TIER_VALUE;
    #endif
  #elif defined(CAN_TIER_VALUE)
  CAN_TIER = CAN_TIER_VALUE;
  #endif

  /* Register 'CAN_CTL0' initialization */
  #if CAN_CTL0_MASK
    #if CAN_CTL0_MASK == 0xFFFF
  CAN_CTL0 = CAN_CTL0_VALUE;
    #elif CAN_CTL0_MASK == CAN_CTL0_VALUE
  CAN_CTL0 |= CAN_CTL0_VALUE;
    #elif CAN_CTL0_VALUE == 0
  CAN_CTL0 &= ~CAN_CTL0_MASK;
    #else
  CAN_CTL0 = (CAN_CTL0 & (~CAN_CTL0_MASK)) | CAN_CTL0_VALUE;
    #endif
  #elif defined(CAN_CTL0_VALUE)
  CAN_CTL0 = CAN_CTL0_VALUE;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END CAN_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
