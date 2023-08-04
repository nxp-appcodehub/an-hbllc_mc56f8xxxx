/** ###################################################################
**      Filename    : Vectors.c
**      Processor   : MC56F82748VLH
**      Version     : 1.0
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
** @file Vectors.c                                                  
** @version 01.00
*/         
/*!
**  @addtogroup Vectors_module Vectors module documentation
**  @{
*/         

#include "Init_Config.h"
#include "Vectors_Config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
** ===================================================================
**     Method      :  Cpu_Interrupt (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#pragma interrupt alignsp
static void CpuUnhandledInterrupt(void)
{
  asm(DEBUGHLT);                       /* Halt the core and placing it in the debug processing state */
}

/*
** ===================================================================
**     Method      :  Cpu_Interrupt0 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_0
#pragma interrupt alignsp
static void CpuUnhandledInterrupt0(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_HW_RESET" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_0 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt1 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_1
#pragma interrupt alignsp
static void CpuUnhandledInterrupt1(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_COP_RESET" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_1 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt2 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_2
#pragma interrupt alignsp
static void CpuUnhandledInterrupt2(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_ILLEGAL_OP" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_2 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt3 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_3
#pragma interrupt alignsp
static void CpuUnhandledInterrupt3(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_SWI3" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_3 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt4 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_4
#pragma interrupt alignsp
static void CpuUnhandledInterrupt4(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_OVERFLOW" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_4 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt5 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_5
#pragma interrupt alignsp
static void CpuUnhandledInterrupt5(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_MISALIGNED" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_5 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt6 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_6
#pragma interrupt alignsp
static void CpuUnhandledInterrupt6(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_STPCNT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_6 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt7 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_7
#pragma interrupt alignsp
static void CpuUnhandledInterrupt7(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_BKPT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_7 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt8 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_8
#pragma interrupt alignsp
static void CpuUnhandledInterrupt8(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TRBUF" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_8 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt9 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_9
#pragma interrupt alignsp
static void CpuUnhandledInterrupt9(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TX_REG" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_9 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt10 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_10
#pragma interrupt alignsp
static void CpuUnhandledInterrupt10(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_RX_REG" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_10 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt11 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_11
#pragma interrupt alignsp
static void CpuUnhandledInterrupt11(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_BUS_ERR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_11 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt12 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_12
#pragma interrupt alignsp
static void CpuUnhandledInterrupt12(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved12" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_12 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt13 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_13
#pragma interrupt alignsp
static void CpuUnhandledInterrupt13(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved13" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_13 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt14 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_14
#pragma interrupt alignsp
static void CpuUnhandledInterrupt14(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved14" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_14 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt15 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_15
#pragma interrupt alignsp
static void CpuUnhandledInterrupt15(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_SWI2" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_15 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt16 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_16
#pragma interrupt alignsp
static void CpuUnhandledInterrupt16(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_SWI1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_16 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt17 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_17
#pragma interrupt alignsp
static void CpuUnhandledInterrupt17(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_SWI0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_17 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt18 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_18
#pragma interrupt alignsp
static void CpuUnhandledInterrupt18(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_XBARA" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_18 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt19 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_19
#pragma interrupt alignsp
static void CpuUnhandledInterrupt19(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_LVI1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_19 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt20 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_20
#pragma interrupt alignsp
static void CpuUnhandledInterrupt20(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_OCCS" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_20 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt21 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_21
#pragma interrupt alignsp
static void CpuUnhandledInterrupt21(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved21" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_21 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt22 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_22
#pragma interrupt alignsp
static void CpuUnhandledInterrupt22(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved22" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_22 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt23 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_23
#pragma interrupt alignsp
static void CpuUnhandledInterrupt23(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved23" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_23 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt24 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_24
#pragma interrupt alignsp
static void CpuUnhandledInterrupt24(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved24" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_24 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt25 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_25
#pragma interrupt alignsp
static void CpuUnhandledInterrupt25(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TMRA_3" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_25 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt26 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_26
#pragma interrupt alignsp
static void CpuUnhandledInterrupt26(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TMRA_2" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_26 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt27 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_27
#pragma interrupt alignsp
static void CpuUnhandledInterrupt27(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TMRA_1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_27 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt28 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_28
#pragma interrupt alignsp
static void CpuUnhandledInterrupt28(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TMRA_0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_28 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt29 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_29
#pragma interrupt alignsp
static void CpuUnhandledInterrupt29(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_ADC_CC1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_29 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt30 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_30
#pragma interrupt alignsp
static void CpuUnhandledInterrupt30(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_ADC_CC0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_30 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt31 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_31
#pragma interrupt alignsp
static void CpuUnhandledInterrupt31(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_ADC_ERR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_31 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt32 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_32
#pragma interrupt alignsp
static void CpuUnhandledInterrupt32(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved32" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_32 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt33 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_33
#pragma interrupt alignsp
static void CpuUnhandledInterrupt33(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_DMA3" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_33 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt34 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_34
#pragma interrupt alignsp
static void CpuUnhandledInterrupt34(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_DMA2" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_34 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt35 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_35
#pragma interrupt alignsp
static void CpuUnhandledInterrupt35(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_DMA1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_35 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt36 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_36
#pragma interrupt alignsp
static void CpuUnhandledInterrupt36(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_DMA0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_36 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt37 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_37
#pragma interrupt alignsp
static void CpuUnhandledInterrupt37(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved37" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_37 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt38 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_38
#pragma interrupt alignsp
static void CpuUnhandledInterrupt38(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved38" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_38 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt39 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_39
#pragma interrupt alignsp
static void CpuUnhandledInterrupt39(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_ERROR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_39 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt40 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_40
#pragma interrupt alignsp
static void CpuUnhandledInterrupt40(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_TX_WARN" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_40 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt41 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_41
#pragma interrupt alignsp
static void CpuUnhandledInterrupt41(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_RX_WARN" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_41 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt42 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_42
#pragma interrupt alignsp
static void CpuUnhandledInterrupt42(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_WAKEUP" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_42 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt43 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_43
#pragma interrupt alignsp
static void CpuUnhandledInterrupt43(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved43" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_43 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt44 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_44
#pragma interrupt alignsp
static void CpuUnhandledInterrupt44(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved44" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_44 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt45 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_45
#pragma interrupt alignsp
static void CpuUnhandledInterrupt45(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved45" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_45 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt46 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_46
#pragma interrupt alignsp
static void CpuUnhandledInterrupt46(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved46" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_46 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt47 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_47
#pragma interrupt alignsp
static void CpuUnhandledInterrupt47(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI1_RERR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_47 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt48 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_48
#pragma interrupt alignsp
static void CpuUnhandledInterrupt48(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI1_RCV" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_48 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt49 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_49
#pragma interrupt alignsp
static void CpuUnhandledInterrupt49(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI1_TIDLE" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_49 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt50 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_50
#pragma interrupt alignsp
static void CpuUnhandledInterrupt50(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI1_TDRE" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_50 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt51 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_51
#pragma interrupt alignsp
static void CpuUnhandledInterrupt51(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI0_RERR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_51 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt52 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_52
#pragma interrupt alignsp
static void CpuUnhandledInterrupt52(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI0_RCV" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_52 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt53 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_53
#pragma interrupt alignsp
static void CpuUnhandledInterrupt53(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI0_TIDLE" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_53 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt54 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_54
#pragma interrupt alignsp
static void CpuUnhandledInterrupt54(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSCI0_TDRE" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_54 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt55 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_55
#pragma interrupt alignsp
static void CpuUnhandledInterrupt55(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved55" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_55 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt56 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_56
#pragma interrupt alignsp
static void CpuUnhandledInterrupt56(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved56" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_56 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt57 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_57
#pragma interrupt alignsp
static void CpuUnhandledInterrupt57(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSPI1_XMIT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_57 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt58 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_58
#pragma interrupt alignsp
static void CpuUnhandledInterrupt58(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSPI1_RCV" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_58 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt59 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_59
#pragma interrupt alignsp
static void CpuUnhandledInterrupt59(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSPI0_XMIT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_59 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt60 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_60
#pragma interrupt alignsp
static void CpuUnhandledInterrupt60(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_QSPI0_RCV" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_60 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt61 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_61
#pragma interrupt alignsp
static void CpuUnhandledInterrupt61(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved61" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_61 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt62 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_62
#pragma interrupt alignsp
static void CpuUnhandledInterrupt62(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_IIC0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_62 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt63 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_63
#pragma interrupt alignsp
static void CpuUnhandledInterrupt63(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved63" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_63 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt64 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_64
#pragma interrupt alignsp
static void CpuUnhandledInterrupt64(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved64" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_64 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt65 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_65
#pragma interrupt alignsp
static void CpuUnhandledInterrupt65(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved65" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_65 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt66 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_66
#pragma interrupt alignsp
static void CpuUnhandledInterrupt66(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved66" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_66 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt67 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_67
#pragma interrupt alignsp
static void CpuUnhandledInterrupt67(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved67" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_67 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt68 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_68
#pragma interrupt alignsp
static void CpuUnhandledInterrupt68(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved68" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_68 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt69 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_69
#pragma interrupt alignsp
static void CpuUnhandledInterrupt69(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved69" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_69 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt70 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_70
#pragma interrupt alignsp
static void CpuUnhandledInterrupt70(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved70" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_70 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt71 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_71
#pragma interrupt alignsp
static void CpuUnhandledInterrupt71(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved71" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_71 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt72 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_72
#pragma interrupt alignsp
static void CpuUnhandledInterrupt72(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved72" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_72 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt73 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_73
#pragma interrupt alignsp
static void CpuUnhandledInterrupt73(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved73" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_73 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt74 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_74
#pragma interrupt alignsp
static void CpuUnhandledInterrupt74(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved74" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_74 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt75 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_75
#pragma interrupt alignsp
static void CpuUnhandledInterrupt75(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved75" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_75 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt76 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_76
#pragma interrupt alignsp
static void CpuUnhandledInterrupt76(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved76" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_76 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt77 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_77
#pragma interrupt alignsp
static void CpuUnhandledInterrupt77(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_FAULT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_77 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt78 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_78
#pragma interrupt alignsp
static void CpuUnhandledInterrupt78(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_RERR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_78 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt79 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_79
#pragma interrupt alignsp
static void CpuUnhandledInterrupt79(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_RELOAD3" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_79 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt80 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_80
#pragma interrupt alignsp
static void CpuUnhandledInterrupt80(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_CAP" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_80 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt81 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_81
#pragma interrupt alignsp
static void CpuUnhandledInterrupt81(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_CMP3" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_81 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt82 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_82
#pragma interrupt alignsp
static void CpuUnhandledInterrupt82(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_RELOAD2" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_82 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt83 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_83
#pragma interrupt alignsp
static void CpuUnhandledInterrupt83(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_CMP2" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_83 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt84 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_84
#pragma interrupt alignsp
static void CpuUnhandledInterrupt84(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_RELOAD1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_84 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt85 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_85
#pragma interrupt alignsp
static void CpuUnhandledInterrupt85(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_CMP1" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_85 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt86 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_86
#pragma interrupt alignsp
static void CpuUnhandledInterrupt86(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_RELOAD0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_86 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt87 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_87
#pragma interrupt alignsp
static void CpuUnhandledInterrupt87(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_eFlexPWMA_CMP0" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_87 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt88 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_88
#pragma interrupt alignsp
static void CpuUnhandledInterrupt88(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_FTFA_RDCOL" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_88 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt89 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_89
#pragma interrupt alignsp
static void CpuUnhandledInterrupt89(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_FTFA_CC" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_89 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt90 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_90
#pragma interrupt alignsp
static void CpuUnhandledInterrupt90(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_CMPD" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_90 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt91 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_91
#pragma interrupt alignsp
static void CpuUnhandledInterrupt91(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_CMPC" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_91 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt92 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_92
#pragma interrupt alignsp
static void CpuUnhandledInterrupt92(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_CMPB" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_92 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt93 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_93
#pragma interrupt alignsp
static void CpuUnhandledInterrupt93(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_CMPA" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_93 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt94 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_94
#pragma interrupt alignsp
static void CpuUnhandledInterrupt94(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_PIT1_ROLLOVR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_94 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt95 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_95
#pragma interrupt alignsp
static void CpuUnhandledInterrupt95(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_PIT0_ROLLOVR" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_95 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt96 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_96
#pragma interrupt alignsp
static void CpuUnhandledInterrupt96(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved96" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_96 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt97 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_97
#pragma interrupt alignsp
static void CpuUnhandledInterrupt97(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved97" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_97 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt98 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_98
#pragma interrupt alignsp
static void CpuUnhandledInterrupt98(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved98" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_98 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt99 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_99
#pragma interrupt alignsp
static void CpuUnhandledInterrupt99(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved99" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_99 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt100 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_100
#pragma interrupt alignsp
static void CpuUnhandledInterrupt100(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved100" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_100 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt101 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_101
#pragma interrupt alignsp
static void CpuUnhandledInterrupt101(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_reserved101" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_101 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt102 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_102
#pragma interrupt alignsp
static void CpuUnhandledInterrupt102(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOF" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_102 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt103 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_103
#pragma interrupt alignsp
static void CpuUnhandledInterrupt103(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOE" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_103 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt104 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_104
#pragma interrupt alignsp
static void CpuUnhandledInterrupt104(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOD" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_104 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt105 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_105
#pragma interrupt alignsp
static void CpuUnhandledInterrupt105(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOC" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_105 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt106 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_106
#pragma interrupt alignsp
static void CpuUnhandledInterrupt106(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOB" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_106 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt107 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_107
#pragma interrupt alignsp
static void CpuUnhandledInterrupt107(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_GPIOA" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_107 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt108 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_108
#pragma interrupt alignsp
static void CpuUnhandledInterrupt108(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_COP_INT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_108 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt109 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_109
#pragma interrupt alignsp
static void CpuUnhandledInterrupt109(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_EWM_INT" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_109 */

/*
** ===================================================================
**     Method      :  Cpu_Interrupt110 (component MC56F82748VLH)
**
**     Description :
**         This method services unhandled interrupt vectors.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#ifdef CPU_UNHANDLED_INTERRUPT_110
#pragma interrupt alignsp
static void CpuUnhandledInterrupt110(void)
{
  asm(DEBUGHLT);                       /* Interrupt "ivINT_SWILP" - Halt the core and placing it in the debug processing state */
}
#endif /* CPU_UNHANDLED_INTERRUPT_110 */


#ifndef _lint

#if CPU_INT_VECTOR_ROM
extern void _EntryPoint(void);         /* Startup routine */
#if 0 
extern void OSIntCtxSw(void);          /* OS special function */
#endif

#if CPU_BOOT_ISR_INIT
volatile asm void _vectboot(void);
#pragma define_section interrupt_vectorsboot "interrupt_vectorsboot.text"  RX
#pragma section interrupt_vectorsboot begin
volatile asm void _vectboot(void) {
  JMP CPU_BOOT_ISR_ADDR_0
  JMP CPU_BOOT_ISR_ADDR_1
}
#pragma section interrupt_vectorsboot end
#endif /* CPU_BOOT_ISR_INIT */

volatile asm void _vect(void);
#pragma define_section interrupt_vectors "interrupt_vectors.text"  RX
#pragma section interrupt_vectors begin
volatile asm void _vect(void) {
#if defined(CPU_DIRECT_ISR_ADDR_0)
  JMP CPU_DIRECT_ISR_ADDR_0            /* Interrupt no. 0 (Used)   - ivINT_HW_RESET - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_0)
  JSR CPU_INDIRECT_ISR_ADDR_0          /* Interrupt no. 0 (Used)   - ivINT_HW_RESET  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 0 (Unused) - ivINT_HW_RESET  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_1)
  JMP CPU_DIRECT_ISR_ADDR_1            /* Interrupt no. 1 (Used)   - ivINT_COP_RESET - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_1)
  JSR CPU_INDIRECT_ISR_ADDR_1          /* Interrupt no. 1 (Used)   - ivINT_COP_RESET  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 1 (Unused) - ivINT_COP_RESET  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_2)
  JMP CPU_DIRECT_ISR_ADDR_2            /* Interrupt no. 2 (Used)   - ivINT_ILLEGAL_OP - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_2)
  JSR CPU_INDIRECT_ISR_ADDR_2          /* Interrupt no. 2 (Used)   - ivINT_ILLEGAL_OP  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 2 (Unused) - ivINT_ILLEGAL_OP  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_3)
  JMP CPU_DIRECT_ISR_ADDR_3            /* Interrupt no. 3 (Used)   - ivINT_SWI3 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_3)
  JSR CPU_INDIRECT_ISR_ADDR_3          /* Interrupt no. 3 (Used)   - ivINT_SWI3  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 3 (Unused) - ivINT_SWI3  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_4)
  JMP CPU_DIRECT_ISR_ADDR_4            /* Interrupt no. 4 (Used)   - ivINT_OVERFLOW - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_4)
  JSR CPU_INDIRECT_ISR_ADDR_4          /* Interrupt no. 4 (Used)   - ivINT_OVERFLOW  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 4 (Unused) - ivINT_OVERFLOW  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_5)
  JMP CPU_DIRECT_ISR_ADDR_5            /* Interrupt no. 5 (Used)   - ivINT_MISALIGNED - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_5)
  JSR CPU_INDIRECT_ISR_ADDR_5          /* Interrupt no. 5 (Used)   - ivINT_MISALIGNED  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 5 (Unused) - ivINT_MISALIGNED  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_6)
  JMP CPU_DIRECT_ISR_ADDR_6            /* Interrupt no. 6 (Used)   - ivINT_STPCNT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_6)
  JSR CPU_INDIRECT_ISR_ADDR_6          /* Interrupt no. 6 (Used)   - ivINT_STPCNT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 6 (Unused) - ivINT_STPCNT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_7)
  JMP CPU_DIRECT_ISR_ADDR_7            /* Interrupt no. 7 (Used)   - ivINT_BKPT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_7)
  JSR CPU_INDIRECT_ISR_ADDR_7          /* Interrupt no. 7 (Used)   - ivINT_BKPT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 7 (Unused) - ivINT_BKPT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_8)
  JMP CPU_DIRECT_ISR_ADDR_8            /* Interrupt no. 8 (Used)   - ivINT_TRBUF - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_8)
  JSR CPU_INDIRECT_ISR_ADDR_8          /* Interrupt no. 8 (Used)   - ivINT_TRBUF  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 8 (Unused) - ivINT_TRBUF  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_9)
  JMP CPU_DIRECT_ISR_ADDR_9            /* Interrupt no. 9 (Used)   - ivINT_TX_REG - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_9)
  JSR CPU_INDIRECT_ISR_ADDR_9          /* Interrupt no. 9 (Used)   - ivINT_TX_REG  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 9 (Unused) - ivINT_TX_REG  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_10)
  JMP CPU_DIRECT_ISR_ADDR_10           /* Interrupt no. 10 (Used)   - ivINT_RX_REG - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_10)
  JSR CPU_INDIRECT_ISR_ADDR_10         /* Interrupt no. 10 (Used)   - ivINT_RX_REG  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 10 (Unused) - ivINT_RX_REG  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_11)
  JMP CPU_DIRECT_ISR_ADDR_11           /* Interrupt no. 11 (Used)   - ivINT_BUS_ERR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_11)
  JSR CPU_INDIRECT_ISR_ADDR_11         /* Interrupt no. 11 (Used)   - ivINT_BUS_ERR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 11 (Unused) - ivINT_BUS_ERR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_12)
  JMP CPU_DIRECT_ISR_ADDR_12           /* Interrupt no. 12 (Used)   - ivINT_reserved12 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_12)
  JSR CPU_INDIRECT_ISR_ADDR_12         /* Interrupt no. 12 (Used)   - ivINT_reserved12  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 12 (Unused) - ivINT_reserved12  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_13)
  JMP CPU_DIRECT_ISR_ADDR_13           /* Interrupt no. 13 (Used)   - ivINT_reserved13 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_13)
  JSR CPU_INDIRECT_ISR_ADDR_13         /* Interrupt no. 13 (Used)   - ivINT_reserved13  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 13 (Unused) - ivINT_reserved13  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_14)
  JMP CPU_DIRECT_ISR_ADDR_14           /* Interrupt no. 14 (Used)   - ivINT_reserved14 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_14)
  JSR CPU_INDIRECT_ISR_ADDR_14         /* Interrupt no. 14 (Used)   - ivINT_reserved14  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 14 (Unused) - ivINT_reserved14  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_15)
  JMP CPU_DIRECT_ISR_ADDR_15           /* Interrupt no. 15 (Used)   - ivINT_SWI2 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_15)
  JSR CPU_INDIRECT_ISR_ADDR_15         /* Interrupt no. 15 (Used)   - ivINT_SWI2  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 15 (Unused) - ivINT_SWI2  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_16)
  JMP CPU_DIRECT_ISR_ADDR_16           /* Interrupt no. 16 (Used)   - ivINT_SWI1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_16)
  JSR CPU_INDIRECT_ISR_ADDR_16         /* Interrupt no. 16 (Used)   - ivINT_SWI1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 16 (Unused) - ivINT_SWI1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_17)
  JMP CPU_DIRECT_ISR_ADDR_17           /* Interrupt no. 17 (Used)   - ivINT_SWI0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_17)
  JSR CPU_INDIRECT_ISR_ADDR_17         /* Interrupt no. 17 (Used)   - ivINT_SWI0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 17 (Unused) - ivINT_SWI0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_18)
  JMP CPU_DIRECT_ISR_ADDR_18           /* Interrupt no. 18 (Used)   - ivINT_XBARA - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_18)
  JSR CPU_INDIRECT_ISR_ADDR_18         /* Interrupt no. 18 (Used)   - ivINT_XBARA  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 18 (Unused) - ivINT_XBARA  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_19)
  JMP CPU_DIRECT_ISR_ADDR_19           /* Interrupt no. 19 (Used)   - ivINT_LVI1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_19)
  JSR CPU_INDIRECT_ISR_ADDR_19         /* Interrupt no. 19 (Used)   - ivINT_LVI1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 19 (Unused) - ivINT_LVI1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_20)
  JMP CPU_DIRECT_ISR_ADDR_20           /* Interrupt no. 20 (Used)   - ivINT_OCCS - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_20)
  JSR CPU_INDIRECT_ISR_ADDR_20         /* Interrupt no. 20 (Used)   - ivINT_OCCS  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 20 (Unused) - ivINT_OCCS  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_21)
  JMP CPU_DIRECT_ISR_ADDR_21           /* Interrupt no. 21 (Used)   - ivINT_reserved21 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_21)
  JSR CPU_INDIRECT_ISR_ADDR_21         /* Interrupt no. 21 (Used)   - ivINT_reserved21  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 21 (Unused) - ivINT_reserved21  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_22)
  JMP CPU_DIRECT_ISR_ADDR_22           /* Interrupt no. 22 (Used)   - ivINT_reserved22 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_22)
  JSR CPU_INDIRECT_ISR_ADDR_22         /* Interrupt no. 22 (Used)   - ivINT_reserved22  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 22 (Unused) - ivINT_reserved22  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_23)
  JMP CPU_DIRECT_ISR_ADDR_23           /* Interrupt no. 23 (Used)   - ivINT_reserved23 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_23)
  JSR CPU_INDIRECT_ISR_ADDR_23         /* Interrupt no. 23 (Used)   - ivINT_reserved23  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 23 (Unused) - ivINT_reserved23  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_24)
  JMP CPU_DIRECT_ISR_ADDR_24           /* Interrupt no. 24 (Used)   - ivINT_reserved24 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_24)
  JSR CPU_INDIRECT_ISR_ADDR_24         /* Interrupt no. 24 (Used)   - ivINT_reserved24  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 24 (Unused) - ivINT_reserved24  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_25)
  JMP CPU_DIRECT_ISR_ADDR_25           /* Interrupt no. 25 (Used)   - ivINT_TMRA_3 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_25)
  JSR CPU_INDIRECT_ISR_ADDR_25         /* Interrupt no. 25 (Used)   - ivINT_TMRA_3  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 25 (Unused) - ivINT_TMRA_3  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_26)
  JMP CPU_DIRECT_ISR_ADDR_26           /* Interrupt no. 26 (Used)   - ivINT_TMRA_2 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_26)
  JSR CPU_INDIRECT_ISR_ADDR_26         /* Interrupt no. 26 (Used)   - ivINT_TMRA_2  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 26 (Unused) - ivINT_TMRA_2  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_27)
  JMP CPU_DIRECT_ISR_ADDR_27           /* Interrupt no. 27 (Used)   - ivINT_TMRA_1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_27)
  JSR CPU_INDIRECT_ISR_ADDR_27         /* Interrupt no. 27 (Used)   - ivINT_TMRA_1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 27 (Unused) - ivINT_TMRA_1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_28)
  JMP CPU_DIRECT_ISR_ADDR_28           /* Interrupt no. 28 (Used)   - ivINT_TMRA_0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_28)
  JSR CPU_INDIRECT_ISR_ADDR_28         /* Interrupt no. 28 (Used)   - ivINT_TMRA_0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 28 (Unused) - ivINT_TMRA_0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_29)
  JMP CPU_DIRECT_ISR_ADDR_29           /* Interrupt no. 29 (Used)   - ivINT_ADC_CC1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_29)
  JSR CPU_INDIRECT_ISR_ADDR_29         /* Interrupt no. 29 (Used)   - ivINT_ADC_CC1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 29 (Unused) - ivINT_ADC_CC1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_30)
  JMP CPU_DIRECT_ISR_ADDR_30           /* Interrupt no. 30 (Used)   - ivINT_ADC_CC0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_30)
  JSR CPU_INDIRECT_ISR_ADDR_30         /* Interrupt no. 30 (Used)   - ivINT_ADC_CC0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 30 (Unused) - ivINT_ADC_CC0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_31)
  JMP CPU_DIRECT_ISR_ADDR_31           /* Interrupt no. 31 (Used)   - ivINT_ADC_ERR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_31)
  JSR CPU_INDIRECT_ISR_ADDR_31         /* Interrupt no. 31 (Used)   - ivINT_ADC_ERR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 31 (Unused) - ivINT_ADC_ERR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_32)
  JMP CPU_DIRECT_ISR_ADDR_32           /* Interrupt no. 32 (Used)   - ivINT_reserved32 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_32)
  JSR CPU_INDIRECT_ISR_ADDR_32         /* Interrupt no. 32 (Used)   - ivINT_reserved32  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 32 (Unused) - ivINT_reserved32  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_33)
  JMP CPU_DIRECT_ISR_ADDR_33           /* Interrupt no. 33 (Used)   - ivINT_DMA3 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_33)
  JSR CPU_INDIRECT_ISR_ADDR_33         /* Interrupt no. 33 (Used)   - ivINT_DMA3  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 33 (Unused) - ivINT_DMA3  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_34)
  JMP CPU_DIRECT_ISR_ADDR_34           /* Interrupt no. 34 (Used)   - ivINT_DMA2 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_34)
  JSR CPU_INDIRECT_ISR_ADDR_34         /* Interrupt no. 34 (Used)   - ivINT_DMA2  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 34 (Unused) - ivINT_DMA2  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_35)
  JMP CPU_DIRECT_ISR_ADDR_35           /* Interrupt no. 35 (Used)   - ivINT_DMA1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_35)
  JSR CPU_INDIRECT_ISR_ADDR_35         /* Interrupt no. 35 (Used)   - ivINT_DMA1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 35 (Unused) - ivINT_DMA1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_36)
  JMP CPU_DIRECT_ISR_ADDR_36           /* Interrupt no. 36 (Used)   - ivINT_DMA0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_36)
  JSR CPU_INDIRECT_ISR_ADDR_36         /* Interrupt no. 36 (Used)   - ivINT_DMA0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 36 (Unused) - ivINT_DMA0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_37)
  JMP CPU_DIRECT_ISR_ADDR_37           /* Interrupt no. 37 (Used)   - ivINT_reserved37 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_37)
  JSR CPU_INDIRECT_ISR_ADDR_37         /* Interrupt no. 37 (Used)   - ivINT_reserved37  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 37 (Unused) - ivINT_reserved37  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_38)
  JMP CPU_DIRECT_ISR_ADDR_38           /* Interrupt no. 38 (Used)   - ivINT_reserved38 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_38)
  JSR CPU_INDIRECT_ISR_ADDR_38         /* Interrupt no. 38 (Used)   - ivINT_reserved38  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 38 (Unused) - ivINT_reserved38  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_39)
  JMP CPU_DIRECT_ISR_ADDR_39           /* Interrupt no. 39 (Used)   - ivINT_ERROR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_39)
  JSR CPU_INDIRECT_ISR_ADDR_39         /* Interrupt no. 39 (Used)   - ivINT_ERROR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 39 (Unused) - ivINT_ERROR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_40)
  JMP CPU_DIRECT_ISR_ADDR_40           /* Interrupt no. 40 (Used)   - ivINT_TX_WARN - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_40)
  JSR CPU_INDIRECT_ISR_ADDR_40         /* Interrupt no. 40 (Used)   - ivINT_TX_WARN  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 40 (Unused) - ivINT_TX_WARN  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_41)
  JMP CPU_DIRECT_ISR_ADDR_41           /* Interrupt no. 41 (Used)   - ivINT_RX_WARN - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_41)
  JSR CPU_INDIRECT_ISR_ADDR_41         /* Interrupt no. 41 (Used)   - ivINT_RX_WARN  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 41 (Unused) - ivINT_RX_WARN  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_42)
  JMP CPU_DIRECT_ISR_ADDR_42           /* Interrupt no. 42 (Used)   - ivINT_WAKEUP - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_42)
  JSR CPU_INDIRECT_ISR_ADDR_42         /* Interrupt no. 42 (Used)   - ivINT_WAKEUP  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 42 (Unused) - ivINT_WAKEUP  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_43)
  JMP CPU_DIRECT_ISR_ADDR_43           /* Interrupt no. 43 (Used)   - ivINT_reserved43 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_43)
  JSR CPU_INDIRECT_ISR_ADDR_43         /* Interrupt no. 43 (Used)   - ivINT_reserved43  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 43 (Unused) - ivINT_reserved43  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_44)
  JMP CPU_DIRECT_ISR_ADDR_44           /* Interrupt no. 44 (Used)   - ivINT_reserved44 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_44)
  JSR CPU_INDIRECT_ISR_ADDR_44         /* Interrupt no. 44 (Used)   - ivINT_reserved44  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 44 (Unused) - ivINT_reserved44  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_45)
  JMP CPU_DIRECT_ISR_ADDR_45           /* Interrupt no. 45 (Used)   - ivINT_reserved45 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_45)
  JSR CPU_INDIRECT_ISR_ADDR_45         /* Interrupt no. 45 (Used)   - ivINT_reserved45  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 45 (Unused) - ivINT_reserved45  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_46)
  JMP CPU_DIRECT_ISR_ADDR_46           /* Interrupt no. 46 (Used)   - ivINT_reserved46 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_46)
  JSR CPU_INDIRECT_ISR_ADDR_46         /* Interrupt no. 46 (Used)   - ivINT_reserved46  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 46 (Unused) - ivINT_reserved46  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_47)
  JMP CPU_DIRECT_ISR_ADDR_47           /* Interrupt no. 47 (Used)   - ivINT_QSCI1_RERR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_47)
  JSR CPU_INDIRECT_ISR_ADDR_47         /* Interrupt no. 47 (Used)   - ivINT_QSCI1_RERR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 47 (Unused) - ivINT_QSCI1_RERR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_48)
  JMP CPU_DIRECT_ISR_ADDR_48           /* Interrupt no. 48 (Used)   - ivINT_QSCI1_RCV - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_48)
  JSR CPU_INDIRECT_ISR_ADDR_48         /* Interrupt no. 48 (Used)   - ivINT_QSCI1_RCV  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 48 (Unused) - ivINT_QSCI1_RCV  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_49)
  JMP CPU_DIRECT_ISR_ADDR_49           /* Interrupt no. 49 (Used)   - ivINT_QSCI1_TIDLE - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_49)
  JSR CPU_INDIRECT_ISR_ADDR_49         /* Interrupt no. 49 (Used)   - ivINT_QSCI1_TIDLE  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 49 (Unused) - ivINT_QSCI1_TIDLE  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_50)
  JMP CPU_DIRECT_ISR_ADDR_50           /* Interrupt no. 50 (Used)   - ivINT_QSCI1_TDRE - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_50)
  JSR CPU_INDIRECT_ISR_ADDR_50         /* Interrupt no. 50 (Used)   - ivINT_QSCI1_TDRE  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 50 (Unused) - ivINT_QSCI1_TDRE  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_51)
  JMP CPU_DIRECT_ISR_ADDR_51           /* Interrupt no. 51 (Used)   - ivINT_QSCI0_RERR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_51)
  JSR CPU_INDIRECT_ISR_ADDR_51         /* Interrupt no. 51 (Used)   - ivINT_QSCI0_RERR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 51 (Unused) - ivINT_QSCI0_RERR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_52)
  JMP CPU_DIRECT_ISR_ADDR_52           /* Interrupt no. 52 (Used)   - ivINT_QSCI0_RCV - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_52)
  JSR CPU_INDIRECT_ISR_ADDR_52         /* Interrupt no. 52 (Used)   - ivINT_QSCI0_RCV  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 52 (Unused) - ivINT_QSCI0_RCV  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_53)
  JMP CPU_DIRECT_ISR_ADDR_53           /* Interrupt no. 53 (Used)   - ivINT_QSCI0_TIDLE - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_53)
  JSR CPU_INDIRECT_ISR_ADDR_53         /* Interrupt no. 53 (Used)   - ivINT_QSCI0_TIDLE  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 53 (Unused) - ivINT_QSCI0_TIDLE  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_54)
  JMP CPU_DIRECT_ISR_ADDR_54           /* Interrupt no. 54 (Used)   - ivINT_QSCI0_TDRE - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_54)
  JSR CPU_INDIRECT_ISR_ADDR_54         /* Interrupt no. 54 (Used)   - ivINT_QSCI0_TDRE  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 54 (Unused) - ivINT_QSCI0_TDRE  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_55)
  JMP CPU_DIRECT_ISR_ADDR_55           /* Interrupt no. 55 (Used)   - ivINT_reserved55 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_55)
  JSR CPU_INDIRECT_ISR_ADDR_55         /* Interrupt no. 55 (Used)   - ivINT_reserved55  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 55 (Unused) - ivINT_reserved55  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_56)
  JMP CPU_DIRECT_ISR_ADDR_56           /* Interrupt no. 56 (Used)   - ivINT_reserved56 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_56)
  JSR CPU_INDIRECT_ISR_ADDR_56         /* Interrupt no. 56 (Used)   - ivINT_reserved56  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 56 (Unused) - ivINT_reserved56  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_57)
  JMP CPU_DIRECT_ISR_ADDR_57           /* Interrupt no. 57 (Used)   - ivINT_QSPI1_XMIT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_57)
  JSR CPU_INDIRECT_ISR_ADDR_57         /* Interrupt no. 57 (Used)   - ivINT_QSPI1_XMIT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 57 (Unused) - ivINT_QSPI1_XMIT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_58)
  JMP CPU_DIRECT_ISR_ADDR_58           /* Interrupt no. 58 (Used)   - ivINT_QSPI1_RCV - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_58)
  JSR CPU_INDIRECT_ISR_ADDR_58         /* Interrupt no. 58 (Used)   - ivINT_QSPI1_RCV  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 58 (Unused) - ivINT_QSPI1_RCV  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_59)
  JMP CPU_DIRECT_ISR_ADDR_59           /* Interrupt no. 59 (Used)   - ivINT_QSPI0_XMIT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_59)
  JSR CPU_INDIRECT_ISR_ADDR_59         /* Interrupt no. 59 (Used)   - ivINT_QSPI0_XMIT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 59 (Unused) - ivINT_QSPI0_XMIT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_60)
  JMP CPU_DIRECT_ISR_ADDR_60           /* Interrupt no. 60 (Used)   - ivINT_QSPI0_RCV - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_60)
  JSR CPU_INDIRECT_ISR_ADDR_60         /* Interrupt no. 60 (Used)   - ivINT_QSPI0_RCV  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 60 (Unused) - ivINT_QSPI0_RCV  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_61)
  JMP CPU_DIRECT_ISR_ADDR_61           /* Interrupt no. 61 (Used)   - ivINT_reserved61 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_61)
  JSR CPU_INDIRECT_ISR_ADDR_61         /* Interrupt no. 61 (Used)   - ivINT_reserved61  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 61 (Unused) - ivINT_reserved61  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_62)
  JMP CPU_DIRECT_ISR_ADDR_62           /* Interrupt no. 62 (Used)   - ivINT_IIC0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_62)
  JSR CPU_INDIRECT_ISR_ADDR_62         /* Interrupt no. 62 (Used)   - ivINT_IIC0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 62 (Unused) - ivINT_IIC0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_63)
  JMP CPU_DIRECT_ISR_ADDR_63           /* Interrupt no. 63 (Used)   - ivINT_reserved63 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_63)
  JSR CPU_INDIRECT_ISR_ADDR_63         /* Interrupt no. 63 (Used)   - ivINT_reserved63  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 63 (Unused) - ivINT_reserved63  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_64)
  JMP CPU_DIRECT_ISR_ADDR_64           /* Interrupt no. 64 (Used)   - ivINT_reserved64 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_64)
  JSR CPU_INDIRECT_ISR_ADDR_64         /* Interrupt no. 64 (Used)   - ivINT_reserved64  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 64 (Unused) - ivINT_reserved64  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_65)
  JMP CPU_DIRECT_ISR_ADDR_65           /* Interrupt no. 65 (Used)   - ivINT_reserved65 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_65)
  JSR CPU_INDIRECT_ISR_ADDR_65         /* Interrupt no. 65 (Used)   - ivINT_reserved65  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 65 (Unused) - ivINT_reserved65  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_66)
  JMP CPU_DIRECT_ISR_ADDR_66           /* Interrupt no. 66 (Used)   - ivINT_reserved66 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_66)
  JSR CPU_INDIRECT_ISR_ADDR_66         /* Interrupt no. 66 (Used)   - ivINT_reserved66  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 66 (Unused) - ivINT_reserved66  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_67)
  JMP CPU_DIRECT_ISR_ADDR_67           /* Interrupt no. 67 (Used)   - ivINT_reserved67 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_67)
  JSR CPU_INDIRECT_ISR_ADDR_67         /* Interrupt no. 67 (Used)   - ivINT_reserved67  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 67 (Unused) - ivINT_reserved67  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_68)
  JMP CPU_DIRECT_ISR_ADDR_68           /* Interrupt no. 68 (Used)   - ivINT_reserved68 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_68)
  JSR CPU_INDIRECT_ISR_ADDR_68         /* Interrupt no. 68 (Used)   - ivINT_reserved68  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 68 (Unused) - ivINT_reserved68  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_69)
  JMP CPU_DIRECT_ISR_ADDR_69           /* Interrupt no. 69 (Used)   - ivINT_reserved69 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_69)
  JSR CPU_INDIRECT_ISR_ADDR_69         /* Interrupt no. 69 (Used)   - ivINT_reserved69  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 69 (Unused) - ivINT_reserved69  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_70)
  JMP CPU_DIRECT_ISR_ADDR_70           /* Interrupt no. 70 (Used)   - ivINT_reserved70 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_70)
  JSR CPU_INDIRECT_ISR_ADDR_70         /* Interrupt no. 70 (Used)   - ivINT_reserved70  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 70 (Unused) - ivINT_reserved70  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_71)
  JMP CPU_DIRECT_ISR_ADDR_71           /* Interrupt no. 71 (Used)   - ivINT_reserved71 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_71)
  JSR CPU_INDIRECT_ISR_ADDR_71         /* Interrupt no. 71 (Used)   - ivINT_reserved71  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 71 (Unused) - ivINT_reserved71  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_72)
  JMP CPU_DIRECT_ISR_ADDR_72           /* Interrupt no. 72 (Used)   - ivINT_reserved72 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_72)
  JSR CPU_INDIRECT_ISR_ADDR_72         /* Interrupt no. 72 (Used)   - ivINT_reserved72  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 72 (Unused) - ivINT_reserved72  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_73)
  JMP CPU_DIRECT_ISR_ADDR_73           /* Interrupt no. 73 (Used)   - ivINT_reserved73 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_73)
  JSR CPU_INDIRECT_ISR_ADDR_73         /* Interrupt no. 73 (Used)   - ivINT_reserved73  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 73 (Unused) - ivINT_reserved73  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_74)
  JMP CPU_DIRECT_ISR_ADDR_74           /* Interrupt no. 74 (Used)   - ivINT_reserved74 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_74)
  JSR CPU_INDIRECT_ISR_ADDR_74         /* Interrupt no. 74 (Used)   - ivINT_reserved74  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 74 (Unused) - ivINT_reserved74  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_75)
  JMP CPU_DIRECT_ISR_ADDR_75           /* Interrupt no. 75 (Used)   - ivINT_reserved75 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_75)
  JSR CPU_INDIRECT_ISR_ADDR_75         /* Interrupt no. 75 (Used)   - ivINT_reserved75  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 75 (Unused) - ivINT_reserved75  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_76)
  JMP CPU_DIRECT_ISR_ADDR_76           /* Interrupt no. 76 (Used)   - ivINT_reserved76 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_76)
  JSR CPU_INDIRECT_ISR_ADDR_76         /* Interrupt no. 76 (Used)   - ivINT_reserved76  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 76 (Unused) - ivINT_reserved76  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_77)
  JMP CPU_DIRECT_ISR_ADDR_77           /* Interrupt no. 77 (Used)   - ivINT_eFlexPWMA_FAULT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_77)
  JSR CPU_INDIRECT_ISR_ADDR_77         /* Interrupt no. 77 (Used)   - ivINT_eFlexPWMA_FAULT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 77 (Unused) - ivINT_eFlexPWMA_FAULT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_78)
  JMP CPU_DIRECT_ISR_ADDR_78           /* Interrupt no. 78 (Used)   - ivINT_eFlexPWMA_RERR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_78)
  JSR CPU_INDIRECT_ISR_ADDR_78         /* Interrupt no. 78 (Used)   - ivINT_eFlexPWMA_RERR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 78 (Unused) - ivINT_eFlexPWMA_RERR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_79)
  JMP CPU_DIRECT_ISR_ADDR_79           /* Interrupt no. 79 (Used)   - ivINT_eFlexPWMA_RELOAD3 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_79)
  JSR CPU_INDIRECT_ISR_ADDR_79         /* Interrupt no. 79 (Used)   - ivINT_eFlexPWMA_RELOAD3  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 79 (Unused) - ivINT_eFlexPWMA_RELOAD3  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_80)
  JMP CPU_DIRECT_ISR_ADDR_80           /* Interrupt no. 80 (Used)   - ivINT_eFlexPWMA_CAP - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_80)
  JSR CPU_INDIRECT_ISR_ADDR_80         /* Interrupt no. 80 (Used)   - ivINT_eFlexPWMA_CAP  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 80 (Unused) - ivINT_eFlexPWMA_CAP  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_81)
  JMP CPU_DIRECT_ISR_ADDR_81           /* Interrupt no. 81 (Used)   - ivINT_eFlexPWMA_CMP3 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_81)
  JSR CPU_INDIRECT_ISR_ADDR_81         /* Interrupt no. 81 (Used)   - ivINT_eFlexPWMA_CMP3  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 81 (Unused) - ivINT_eFlexPWMA_CMP3  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_82)
  JMP CPU_DIRECT_ISR_ADDR_82           /* Interrupt no. 82 (Used)   - ivINT_eFlexPWMA_RELOAD2 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_82)
  JSR CPU_INDIRECT_ISR_ADDR_82         /* Interrupt no. 82 (Used)   - ivINT_eFlexPWMA_RELOAD2  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 82 (Unused) - ivINT_eFlexPWMA_RELOAD2  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_83)
  JMP CPU_DIRECT_ISR_ADDR_83           /* Interrupt no. 83 (Used)   - ivINT_eFlexPWMA_CMP2 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_83)
  JSR CPU_INDIRECT_ISR_ADDR_83         /* Interrupt no. 83 (Used)   - ivINT_eFlexPWMA_CMP2  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 83 (Unused) - ivINT_eFlexPWMA_CMP2  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_84)
  JMP CPU_DIRECT_ISR_ADDR_84           /* Interrupt no. 84 (Used)   - ivINT_eFlexPWMA_RELOAD1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_84)
  JSR CPU_INDIRECT_ISR_ADDR_84         /* Interrupt no. 84 (Used)   - ivINT_eFlexPWMA_RELOAD1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 84 (Unused) - ivINT_eFlexPWMA_RELOAD1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_85)
  JMP CPU_DIRECT_ISR_ADDR_85           /* Interrupt no. 85 (Used)   - ivINT_eFlexPWMA_CMP1 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_85)
  JSR CPU_INDIRECT_ISR_ADDR_85         /* Interrupt no. 85 (Used)   - ivINT_eFlexPWMA_CMP1  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 85 (Unused) - ivINT_eFlexPWMA_CMP1  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_86)
  JMP CPU_DIRECT_ISR_ADDR_86           /* Interrupt no. 86 (Used)   - ivINT_eFlexPWMA_RELOAD0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_86)
  JSR CPU_INDIRECT_ISR_ADDR_86         /* Interrupt no. 86 (Used)   - ivINT_eFlexPWMA_RELOAD0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 86 (Unused) - ivINT_eFlexPWMA_RELOAD0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_87)
  JMP CPU_DIRECT_ISR_ADDR_87           /* Interrupt no. 87 (Used)   - ivINT_eFlexPWMA_CMP0 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_87)
  JSR CPU_INDIRECT_ISR_ADDR_87         /* Interrupt no. 87 (Used)   - ivINT_eFlexPWMA_CMP0  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 87 (Unused) - ivINT_eFlexPWMA_CMP0  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_88)
  JMP CPU_DIRECT_ISR_ADDR_88           /* Interrupt no. 88 (Used)   - ivINT_FTFA_RDCOL - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_88)
  JSR CPU_INDIRECT_ISR_ADDR_88         /* Interrupt no. 88 (Used)   - ivINT_FTFA_RDCOL  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 88 (Unused) - ivINT_FTFA_RDCOL  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_89)
  JMP CPU_DIRECT_ISR_ADDR_89           /* Interrupt no. 89 (Used)   - ivINT_FTFA_CC - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_89)
  JSR CPU_INDIRECT_ISR_ADDR_89         /* Interrupt no. 89 (Used)   - ivINT_FTFA_CC  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 89 (Unused) - ivINT_FTFA_CC  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_90)
  JMP CPU_DIRECT_ISR_ADDR_90           /* Interrupt no. 90 (Used)   - ivINT_CMPD - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_90)
  JSR CPU_INDIRECT_ISR_ADDR_90         /* Interrupt no. 90 (Used)   - ivINT_CMPD  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 90 (Unused) - ivINT_CMPD  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_91)
  JMP CPU_DIRECT_ISR_ADDR_91           /* Interrupt no. 91 (Used)   - ivINT_CMPC - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_91)
  JSR CPU_INDIRECT_ISR_ADDR_91         /* Interrupt no. 91 (Used)   - ivINT_CMPC  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 91 (Unused) - ivINT_CMPC  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_92)
  JMP CPU_DIRECT_ISR_ADDR_92           /* Interrupt no. 92 (Used)   - ivINT_CMPB - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_92)
  JSR CPU_INDIRECT_ISR_ADDR_92         /* Interrupt no. 92 (Used)   - ivINT_CMPB  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 92 (Unused) - ivINT_CMPB  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_93)
  JMP CPU_DIRECT_ISR_ADDR_93           /* Interrupt no. 93 (Used)   - ivINT_CMPA - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_93)
  JSR CPU_INDIRECT_ISR_ADDR_93         /* Interrupt no. 93 (Used)   - ivINT_CMPA  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 93 (Unused) - ivINT_CMPA  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_94)
  JMP CPU_DIRECT_ISR_ADDR_94           /* Interrupt no. 94 (Used)   - ivINT_PIT1_ROLLOVR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_94)
  JSR CPU_INDIRECT_ISR_ADDR_94         /* Interrupt no. 94 (Used)   - ivINT_PIT1_ROLLOVR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 94 (Unused) - ivINT_PIT1_ROLLOVR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_95)
  JMP CPU_DIRECT_ISR_ADDR_95           /* Interrupt no. 95 (Used)   - ivINT_PIT0_ROLLOVR - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_95)
  JSR CPU_INDIRECT_ISR_ADDR_95         /* Interrupt no. 95 (Used)   - ivINT_PIT0_ROLLOVR  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 95 (Unused) - ivINT_PIT0_ROLLOVR  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_96)
  JMP CPU_DIRECT_ISR_ADDR_96           /* Interrupt no. 96 (Used)   - ivINT_reserved96 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_96)
  JSR CPU_INDIRECT_ISR_ADDR_96         /* Interrupt no. 96 (Used)   - ivINT_reserved96  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 96 (Unused) - ivINT_reserved96  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_97)
  JMP CPU_DIRECT_ISR_ADDR_97           /* Interrupt no. 97 (Used)   - ivINT_reserved97 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_97)
  JSR CPU_INDIRECT_ISR_ADDR_97         /* Interrupt no. 97 (Used)   - ivINT_reserved97  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 97 (Unused) - ivINT_reserved97  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_98)
  JMP CPU_DIRECT_ISR_ADDR_98           /* Interrupt no. 98 (Used)   - ivINT_reserved98 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_98)
  JSR CPU_INDIRECT_ISR_ADDR_98         /* Interrupt no. 98 (Used)   - ivINT_reserved98  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 98 (Unused) - ivINT_reserved98  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_99)
  JMP CPU_DIRECT_ISR_ADDR_99           /* Interrupt no. 99 (Used)   - ivINT_reserved99 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_99)
  JSR CPU_INDIRECT_ISR_ADDR_99         /* Interrupt no. 99 (Used)   - ivINT_reserved99  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 99 (Unused) - ivINT_reserved99  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_100)
  JMP CPU_DIRECT_ISR_ADDR_100          /* Interrupt no. 100 (Used)   - ivINT_reserved100 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_100)
  JSR CPU_INDIRECT_ISR_ADDR_100        /* Interrupt no. 100 (Used)   - ivINT_reserved100  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 100 (Unused) - ivINT_reserved100  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_101)
  JMP CPU_DIRECT_ISR_ADDR_101          /* Interrupt no. 101 (Used)   - ivINT_reserved101 - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_101)
  JSR CPU_INDIRECT_ISR_ADDR_101        /* Interrupt no. 101 (Used)   - ivINT_reserved101  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 101 (Unused) - ivINT_reserved101  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_102)
  JMP CPU_DIRECT_ISR_ADDR_102          /* Interrupt no. 102 (Used)   - ivINT_GPIOF - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_102)
  JSR CPU_INDIRECT_ISR_ADDR_102        /* Interrupt no. 102 (Used)   - ivINT_GPIOF  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 102 (Unused) - ivINT_GPIOF  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_103)
  JMP CPU_DIRECT_ISR_ADDR_103          /* Interrupt no. 103 (Used)   - ivINT_GPIOE - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_103)
  JSR CPU_INDIRECT_ISR_ADDR_103        /* Interrupt no. 103 (Used)   - ivINT_GPIOE  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 103 (Unused) - ivINT_GPIOE  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_104)
  JMP CPU_DIRECT_ISR_ADDR_104          /* Interrupt no. 104 (Used)   - ivINT_GPIOD - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_104)
  JSR CPU_INDIRECT_ISR_ADDR_104        /* Interrupt no. 104 (Used)   - ivINT_GPIOD  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 104 (Unused) - ivINT_GPIOD  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_105)
  JMP CPU_DIRECT_ISR_ADDR_105          /* Interrupt no. 105 (Used)   - ivINT_GPIOC - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_105)
  JSR CPU_INDIRECT_ISR_ADDR_105        /* Interrupt no. 105 (Used)   - ivINT_GPIOC  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 105 (Unused) - ivINT_GPIOC  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_106)
  JMP CPU_DIRECT_ISR_ADDR_106          /* Interrupt no. 106 (Used)   - ivINT_GPIOB - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_106)
  JSR CPU_INDIRECT_ISR_ADDR_106        /* Interrupt no. 106 (Used)   - ivINT_GPIOB  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 106 (Unused) - ivINT_GPIOB  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_107)
  JMP CPU_DIRECT_ISR_ADDR_107          /* Interrupt no. 107 (Used)   - ivINT_GPIOA - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_107)
  JSR CPU_INDIRECT_ISR_ADDR_107        /* Interrupt no. 107 (Used)   - ivINT_GPIOA  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 107 (Unused) - ivINT_GPIOA  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_108)
  JMP CPU_DIRECT_ISR_ADDR_108          /* Interrupt no. 108 (Used)   - ivINT_COP_INT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_108)
  JSR CPU_INDIRECT_ISR_ADDR_108        /* Interrupt no. 108 (Used)   - ivINT_COP_INT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 108 (Unused) - ivINT_COP_INT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_109)
  JMP CPU_DIRECT_ISR_ADDR_109          /* Interrupt no. 109 (Used)   - ivINT_EWM_INT - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_109)
  JSR CPU_INDIRECT_ISR_ADDR_109        /* Interrupt no. 109 (Used)   - ivINT_EWM_INT  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 109 (Unused) - ivINT_EWM_INT  */
#endif
#if defined(CPU_DIRECT_ISR_ADDR_110)
  JMP CPU_DIRECT_ISR_ADDR_110          /* Interrupt no. 110 (Used)   - ivINT_SWILP - configured as a fast interrupt */
#elif defined(CPU_INDIRECT_ISR_ADDR_110)
  JSR CPU_INDIRECT_ISR_ADDR_110        /* Interrupt no. 110 (Used)   - ivINT_SWILP  */
#else
  JSR CpuUnhandledInterrupt            /* Interrupt no. 110 (Unused) - ivINT_SWILP  */
#endif
}
#pragma section interrupt_vectors end
#endif /* CPU_INT_VECTOR_ROM */

#endif _lint

#ifdef __cplusplus
}
#endif

/*!
** @}
*/
/*
** ###################################################################
**
**     This file is a part of Processor Expert static initialization
**     library for the Freescale 56800 series of microcontrollers.
**
** ###################################################################
*/
