/** ###################################################################
 **     Filename    : I2C_Init.c
 **     Processor   : MC56F82748VLH
 **     Abstract    :
 **          This file implements the I2C module initialization
 **          according to the Peripheral Initialization settings, and
 **          defines interrupt service routines prototypes.
 **
 **     Contents    :
 **         Int   - void I2C_Init(void);
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
 * @file I2C_Init.c
 * @brief This file implements the I2C module initialization according to the
 *        Peripheral Initialization settings, and defines interrupt service
 *        routines prototypes.
 */


/* MODULE I2C_Init. */

#include "I2C_Init.h"
#include "MC56F82748.h"
#include "Init_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

void I2C_Init(void) {

  /* Register 'I2C_FLT' initialization */
  #if I2C_FLT_MASK
    #if I2C_FLT_MASK == 0xFFFF
  I2C_FLT = I2C_FLT_VALUE;
    #elif I2C_FLT_MASK == I2C_FLT_VALUE
  I2C_FLT |= I2C_FLT_VALUE;
    #elif I2C_FLT_VALUE == 0
  I2C_FLT &= ~I2C_FLT_MASK;
    #else
  I2C_FLT = (I2C_FLT & (~I2C_FLT_MASK)) | I2C_FLT_VALUE;
    #endif
  #elif defined(I2C_FLT_VALUE)
  I2C_FLT = I2C_FLT_VALUE;
  #endif

  /* Register 'I2C_A1' initialization */
  #if I2C_A1_MASK
    #if I2C_A1_MASK == 0xFFFF
  I2C_A1 = I2C_A1_VALUE;
    #elif I2C_A1_MASK == I2C_A1_VALUE
  I2C_A1 |= I2C_A1_VALUE;
    #elif I2C_A1_VALUE == 0
  I2C_A1 &= ~I2C_A1_MASK;
    #else
  I2C_A1 = (I2C_A1 & (~I2C_A1_MASK)) | I2C_A1_VALUE;
    #endif
  #elif defined(I2C_A1_VALUE)
  I2C_A1 = I2C_A1_VALUE;
  #endif

  /* Register 'I2C_C2' initialization */
  #if I2C_C2_MASK
    #if I2C_C2_MASK == 0xFFFF
  I2C_C2 = I2C_C2_VALUE;
    #elif I2C_C2_MASK == I2C_C2_VALUE
  I2C_C2 |= I2C_C2_VALUE;
    #elif I2C_C2_VALUE == 0
  I2C_C2 &= ~I2C_C2_MASK;
    #else
  I2C_C2 = (I2C_C2 & (~I2C_C2_MASK)) | I2C_C2_VALUE;
    #endif
  #elif defined(I2C_C2_VALUE)
  I2C_C2 = I2C_C2_VALUE;
  #endif

  /* Register 'I2C_RA' initialization */
  #if I2C_RA_MASK
    #if I2C_RA_MASK == 0xFFFF
  I2C_RA = I2C_RA_VALUE;
    #elif I2C_RA_MASK == I2C_RA_VALUE
  I2C_RA |= I2C_RA_VALUE;
    #elif I2C_RA_VALUE == 0
  I2C_RA &= ~I2C_RA_MASK;
    #else
  I2C_RA = (I2C_RA & (~I2C_RA_MASK)) | I2C_RA_VALUE;
    #endif
  #elif defined(I2C_RA_VALUE)
  I2C_RA = I2C_RA_VALUE;
  #endif

  /* Register 'I2C_F' initialization */
  #if I2C_F_MASK
    #if I2C_F_MASK == 0xFFFF
  I2C_F = I2C_F_VALUE;
    #elif I2C_F_MASK == I2C_F_VALUE
  I2C_F |= I2C_F_VALUE;
    #elif I2C_F_VALUE == 0
  I2C_F &= ~I2C_F_MASK;
    #else
  I2C_F = (I2C_F & (~I2C_F_MASK)) | I2C_F_VALUE;
    #endif
  #elif defined(I2C_F_VALUE)
  I2C_F = I2C_F_VALUE;
  #endif

  /* Register 'I2C_A2' initialization */
  #if I2C_A2_MASK
    #if I2C_A2_MASK == 0xFFFF
  I2C_A2 = I2C_A2_VALUE;
    #elif I2C_A2_MASK == I2C_A2_VALUE
  I2C_A2 |= I2C_A2_VALUE;
    #elif I2C_A2_VALUE == 0
  I2C_A2 &= ~I2C_A2_MASK;
    #else
  I2C_A2 = (I2C_A2 & (~I2C_A2_MASK)) | I2C_A2_VALUE;
    #endif
  #elif defined(I2C_A2_VALUE)
  I2C_A2 = I2C_A2_VALUE;
  #endif

  /* Register 'I2C_SMB' initialization */
  #if I2C_SMB_MASK
    #if I2C_SMB_MASK == 0xFFFF
  I2C_SMB = I2C_SMB_VALUE;
    #elif I2C_SMB_MASK == I2C_SMB_VALUE
  I2C_SMB |= I2C_SMB_VALUE;
    #elif I2C_SMB_VALUE == 0
  I2C_SMB &= ~I2C_SMB_MASK;
    #else
  I2C_SMB = (I2C_SMB & (~I2C_SMB_MASK)) | I2C_SMB_VALUE;
    #endif
  #elif defined(I2C_SMB_VALUE)
  I2C_SMB = I2C_SMB_VALUE;
  #endif

  /* Register 'I2C_SLTL' initialization */
  #if I2C_SLTL_MASK
    #if I2C_SLTL_MASK == 0xFFFF
  I2C_SLTL = I2C_SLTL_VALUE;
    #elif I2C_SLTL_MASK == I2C_SLTL_VALUE
  I2C_SLTL |= I2C_SLTL_VALUE;
    #elif I2C_SLTL_VALUE == 0
  I2C_SLTL &= ~I2C_SLTL_MASK;
    #else
  I2C_SLTL = (I2C_SLTL & (~I2C_SLTL_MASK)) | I2C_SLTL_VALUE;
    #endif
  #elif defined(I2C_SLTL_VALUE)
  I2C_SLTL = I2C_SLTL_VALUE;
  #endif

  /* Register 'I2C_SLTH' initialization */
  #if I2C_SLTH_MASK
    #if I2C_SLTH_MASK == 0xFFFF
  I2C_SLTH = I2C_SLTH_VALUE;
    #elif I2C_SLTH_MASK == I2C_SLTH_VALUE
  I2C_SLTH |= I2C_SLTH_VALUE;
    #elif I2C_SLTH_VALUE == 0
  I2C_SLTH &= ~I2C_SLTH_MASK;
    #else
  I2C_SLTH = (I2C_SLTH & (~I2C_SLTH_MASK)) | I2C_SLTH_VALUE;
    #endif
  #elif defined(I2C_SLTH_VALUE)
  I2C_SLTH = I2C_SLTH_VALUE;
  #endif

  /* Register 'I2C_S' initialization */
  #if I2C_S_MASK
    #if I2C_S_MASK == 0xFFFF
  I2C_S = I2C_S_VALUE;
    #elif I2C_S_MASK == I2C_S_VALUE
  I2C_S |= I2C_S_VALUE;
    #elif I2C_S_VALUE == 0
  I2C_S &= ~I2C_S_MASK;
    #else
  I2C_S = (I2C_S & (~I2C_S_MASK)) | I2C_S_VALUE;
    #endif
  #elif defined(I2C_S_VALUE)
  I2C_S = I2C_S_VALUE;
  #endif

  /* Register 'I2C_C1' initialization */
  #if I2C_C1_MASK_1
    #if I2C_C1_MASK_1 == 0xFFFF
  I2C_C1 = I2C_C1_VALUE_1;
    #elif I2C_C1_MASK_1 == I2C_C1_VALUE_1
  I2C_C1 |= I2C_C1_VALUE_1;
    #elif I2C_C1_VALUE_1 == 0
  I2C_C1 &= ~I2C_C1_MASK_1;
    #else
  I2C_C1 = (I2C_C1 & (~I2C_C1_MASK_1)) | I2C_C1_VALUE_1;
    #endif
  #elif defined(I2C_C1_VALUE_1)
  I2C_C1 = I2C_C1_VALUE_1;
  #endif

  /* Register 'I2C_C1' initialization */
  #if I2C_C1_MASK_2
    #if I2C_C1_MASK_2 == 0xFFFF
  I2C_C1 = I2C_C1_VALUE_2;
    #elif I2C_C1_MASK_2 == I2C_C1_VALUE_2
  I2C_C1 |= I2C_C1_VALUE_2;
    #elif I2C_C1_VALUE_2 == 0
  I2C_C1 &= ~I2C_C1_MASK_2;
    #else
  I2C_C1 = (I2C_C1 & (~I2C_C1_MASK_2)) | I2C_C1_VALUE_2;
    #endif
  #elif defined(I2C_C1_VALUE_2)
  I2C_C1 = I2C_C1_VALUE_2;
  #endif
}

#ifdef __cplusplus
}
#endif

/* END I2C_Init. */

/** ###################################################################
 **
 **     This file is a part of Processor Expert static initialization
 **     library for Freescale microcontrollers.
 **
 ** ################################################################### */
