/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : Cpu.h
**     Project     : HBLLC_MC56F82748_V1.0
**     Processor   : MC56F82748VLH
**     Component   : MC56F82748VLH
**     Version     : Component 01.055, Driver 01.00, CPU db: 3.50.001
**     Datasheet   : MC56F827XXRM Rev.1 Draft A 02/2012
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2022-01-27, 11:59, # CodeGen: 0
**     Abstract    :
**
**     Settings    :
**
**     Contents    :
**         EnableInt   - void Cpu_EnableInt(void);
**         DisableInt  - void Cpu_DisableInt(void);
**         SetWaitMode - void Cpu_SetWaitMode(void);
**         SetStopMode - void Cpu_SetStopMode(void);
**
**     (c) Freescale Semiconductor, Inc.
**     2004 All Rights Reserved
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
** @file Cpu.h
** @version 01.00
** @brief
**
*/         
/*!
**  @addtogroup Cpu_module Cpu module documentation
**  @{
*/         

#ifndef __Cpu_H
#define __Cpu_H

/* MODULE Cpu. */

/*Include shared modules, which are used for whole project*/
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Init_Config.h"
#include "PDD_Includes.h"
#include "assert.h"
#include "CPU_Init.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Active configuration define symbol */
#define PEcfg_MC56F82748VLH_Internal_PFlash_SDM 1U

/* Clock configuration settings. Generated information about clock configuration frequencies defined in the CPU component. 
   Initialization of clock configuration description structure is done in PE_LDD.c module. */

#define CPU_BUS_CLK_HZ                  50000000UL /* Initial value of the bus clock frequency in Hz */
#define CPU_CLOCK_CONFIG_NUMBER         0x01U /* Specifies number of defined clock configurations */

/* Enabled clock configuration system bus frequencies */
#define CPU_BUS_CLK_HZ_CLOCK_CONFIG0    50000000UL /* Value of the bus clock frequency in the clock configuration 0 in Hz */

/* Enabled clock sources frequencies */
#define CPU_INT_FAST_CLK_HZ             8000000UL /* Value of the fast internal oscillator clock frequency in Hz  */

/* CPU frequencies in clock configuration 0 */
#define CPU_CLOCK_CONFIG_0              0x00U /* Clock configuration 0 identifier */
#define CPU_CORE_CLK_HZ_CONFIG_0        100000000UL /* Core clock frequency in clock configuration 0 */
#define CPU_SYSTEM_CLK_HZ_CONFIG_0      100000000UL /* System clock frequency in clock configuration 0 */
#define CPU_BUS_CLK_HZ_CONFIG_0         50000000UL /* Bus clock frequency in clock configuration 0 */
#define CPU_PLL_CLK_HZ_CONFIG_0         400000000UL /* PLL clock frequency in clock configuration 0 */
#define CPU_XTAL_CLK_HZ_CONFIG_0        0UL /* External reference clock frequency in clock configuration 0 */

/* Clock configuration description structure */  
typedef struct  {
  uint32_t cpu_core_clk_hz;            /* Core clock frequency in clock configuration */
  uint32_t cpu_system_clk_hz;          /* System clock frequency in clock configuration */
  uint32_t cpu_bus_clk_hz;             /* Bus clock frequency in clock configuration */
  uint32_t cpu_pll_clk_hz;             /* PLL clock frequency in clock configuration */
  uint32_t cpu_xtal_clk_hz;            /* System OSC external reference clock frequency in clock configuration */
} TCpuClockConfiguration;

/* The array of clock frequencies in configured clock configurations */
extern const TCpuClockConfiguration PE_CpuClockConfigurations[CPU_CLOCK_CONFIG_NUMBER];

/* MCU derivative constants */

#define CPU_FAMILY_56800               /* Specification of the core type of the selected CPU */
#define CPU_DERIVATIVE_MC56F82748VLH   /* Name of the selected CPU derivative */
#define CPU_PARTNUM_MC56F82748VLH      /* Part number of the selected CPU */
#define CPU_LITTLE_ENDIAN              /* The selected CPU uses little endian */

/*
** ===================================================================
**     Method      :  Common_Init (component MC56F82748VLH)
**     Description :
**         Initialization of registers for that there is no 
**         initialization component.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_COMMON_INIT
void Common_Init(void);
#endif /* CPU_COMMON_INIT */

/*
** ===================================================================
**     Method      :  Components_Init (component MC56F82748VLH)
**     Description :
**         Initialization of components (with exception for Peripheral
**         Initialization Components which are initialized in 
**         Peripherals_Init() method).
**         For example, if automatic initialization feature 
**         is enabled in LDD component then its Init method call 
**         is executed in Components_Init() method.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_COMPONENTS_INIT
void Components_Init(void);
#endif /* CPU_COMPONENTS_INIT */

/* Method synonyms. Following constants maps static CPU methods with enabled user methods of which names are derived from the CPU component name */

#define Cpu_EnableInt() CPU_EnableInt()

#define Cpu_DisableInt() CPU_DisableInt()

#define Cpu_SetWaitMode() CPU_SetWaitMode()

#define Cpu_SetStopMode() CPU_SetStopMode()


#ifdef __cplusplus
}
#endif

/* END Cpu. */

#endif
/* __Cpu_H */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale 56800 series of microcontrollers.
**
** ###################################################################
*/

