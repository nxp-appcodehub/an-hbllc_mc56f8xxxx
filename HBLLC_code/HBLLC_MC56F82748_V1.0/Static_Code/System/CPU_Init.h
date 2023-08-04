/** ###################################################################
**      Filename    : CPU_Init.h
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
** @file CPU_Init.h                                                  
** @version 01.00
*/         
/*!
**  @addtogroup CPU_Init_module CPU_Init module documentation
**  @{
*/         

#ifndef __CPU_Init_H
#define __CPU_Init_H

/* MODULE CPU_Init.h */

#include "PE_LDD.h"
#include "CPU_Config.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifndef __BWUserType_TShadowRegs
#define __BWUserType_TShadowRegs
  typedef struct {                     /* Shadow register type. */
    int32_t N;                         /* Shadow register N. */
    int16_t N3;                        /* Shadow register N3. */
    int16_t M01;                       /* Shadow register M01. */
    uint16_t* R0;                      /* Shadow register R0. */
    uint16_t* R1;                      /* Shadow register R1. */
    uint16_t* R3;                      /* Shadow register R3. */
    uint16_t* R4;                      /* Shadow register R4. */
    uint16_t* R5;                      /* Shadow register R5. */
    uint16_t* R2;                      /* Shadow register R2. In InitShadowRegs() this register is used to index through TShadowRegs so is placed as last struct item */
  } TShadowRegs;
#endif

/* Global variables */
extern volatile uint16_t SR_lock;      /* Lock */
extern volatile uint16_t SR_reg;       /* Current value of the SR register */


/*
** ===================================================================
**     Method      :  _EntryPoint (component MC56F82748VLH)
**
**     Description :
**         Initializes the whole system like timing and so on. At the end 
**         of this function, the C startup is invoked to initialize stack,
**         memory areas and so on.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void _EntryPoint(void);

/*
** ===================================================================
**     Method      :  PE_low_level_init (component MC56F82748VLH)
**
**     Description :
**         Initializes components and provides common register 
**         initialization. The method is called automatically as a part 
**         of the application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void PE_low_level_init(void);

/*
** ===================================================================
**     Method      :  UnusedPins_Init (component MC56F82748VLH)
**
**     Description :
**         This method initializes I/O pins which are not used by any 
**         component in the project.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
#if CPU_UNUSED_PINS_INIT
void UnusedPins_Init(void);
#endif /* CPU_UNUSED_PINS_INIT */

/*
** ===================================================================
**     Method      :  Cpu_SetClockConfiguration (component MC56F82748VLH)
**     Description :
**         Calling of this method will cause the clock configuration
**         change and reconfiguration of all components according to
**         the requested clock configuration setting.
**     Parameters  :
**         NAME            - DESCRIPTION
**         ModeID          - Clock configuration identifier
**     Returns     :
**         ---             - ERR_OK - OK.
**                           ERR_RANGE - Mode parameter out of range
** ===================================================================
*/
#if CPU_SET_CLOCK_CONFIGURATION
LDD_TError CPU_SetClockConfiguration(LDD_TClockConfiguration ModeID);
#endif /* CPU_SET_CLOCK_CONFIGURATION */

/*
** ===================================================================
**     Method      :  Cpu_GetClockConfiguration (component MC56F82748VLH)
**     Description :
**         Returns the active clock configuration identifier. The
**         method is automatically enabled if more than one clock
**         configuration is enabled in the component.
**     Parameters  : None
**     Returns     :
**         ---             - Active clock configuration identifier
** ===================================================================
*/
#if CPU_GET_CLOCK_CONFIGURATION
uint8_t CPU_GetClockConfiguration(void);
#endif /* CPU_GET_CLOCK_CONFIGURATION */

/*
** ===================================================================
**     Method      :  Cpu_SetOperationMode (component MC56F82748VLH)
**     Description :
**         This method requests to change the component's operation
**         mode (RUN, WAIT, SLEEP, STOP). The target operation mode
**         will be entered immediately. 
**         See <Operation mode settings> for further details of the
**         operation modes mapping to low power modes of the cpu.
**     Parameters  :
**         NAME            - DESCRIPTION
**         OperationMode   - Requested driver
**                           operation mode
**         ModeChangeCallback - Callback to
**                           notify the upper layer once a mode has been
**                           changed. Parameter is ignored, only for
**                           compatibility of API with other components.
**       * ModeChangeCallbackParamPtr 
**                           - Pointer to callback parameter to notify
**                           the upper layer once a mode has been
**                           changed. Parameter is ignored, only for
**                           compatibility of API with other components.
**     Returns     :
**         ---             - Error code
**                           ERR_OK - OK
**                           ERR_PARAM_MODE - Invalid operation mode
** ===================================================================
*/
#if CPU_SET_OPERATION_MODE
LDD_TError CPU_SetOperationMode(LDD_TDriverOperationMode OperationMode, LDD_TCallback ModeChangeCallback, LDD_TCallbackParam *ModeChangeCallbackParamPtr);
#endif /* CPU_SET_OPERATION_MODE */

/*
** ===================================================================
**     Method      :  Cpu_EnableInt (component MC56F82748VLH)
**     Description :
**         Enables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_ENABLE_INT
#define CPU_EnableInt()  __EI(0)       /* Enable interrupts */
#endif /* CPU_ENABLE_INT */

/*
** ===================================================================
**     Method      :  Cpu_DisableInt (component MC56F82748VLH)
**     Description :
**         Disables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_DISABLE_INT
#define CPU_DisableInt()  __DI()       /* Disable interrupts */
#endif /* CPU_DISABLE_INT */

/*
** ===================================================================
**     Method      :  Cpu_GetResetSource (component MC56F82748VLH)
**     Description :
**         Gets low byte of the reset status register.
**     Parameters  : None
**     Returns     :
**         ---             - Content of the reset status register.
**                           You can use predefined constants listed
**                           bellow to find out a reset cause.
**                           Reset source (predefined constants):
**                           RSTSRC_POR - Power-on reset
**                           RSTSRC_PIN - External reset
**                           RSTSRC_COP - COP reset
**                           RSTSRC_SWR - Software reset
** ===================================================================
*/
#if CPU_GET_RESET_SOURCE
#define CPU_GetResetSource() (byte)getReg(SIM_RSTAT)
#endif /* CPU_GET_RESET_SOURCE */

/*
** ===================================================================
**     Method      :  Cpu_SetWaitMode (component MC56F82748VLH)
**     Description :
**         Sets low power mode - Wait mode.
**         For more information about the wait mode see this CPU
**         documentation.
**         Release from wait mode: Reset or interrupt
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_SET_WAIT_MODE
#define CPU_SetWaitMode()  asm(WAIT)   /* Set WAIT mode */
#endif /* CPU_SET_WAIT_MODE */

/*
** ===================================================================
**     Method      :  Cpu_SetStopMode (component MC56F82748VLH)
**     Description :
**         Sets low power mode - Stop mode.
**         For more information about the stop mode see this CPU
**         documentation.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_SET_STOP_MODE
#define CPU_SetStopMode()  asm(STOP)   /* Set STOP mode */
#endif /* CPU_SET_STOP_MODE */

/*
** ===================================================================
**     Method      :  Cpu_GetBusFreqHz (component MC56F82748VLH)
**     Description :
**         Gets the bus frequency [Hz] of the actual clock 
**         configuration.
**     Parameters  : None
**     Returns     :
**         ---             - Bus frequency of the actual clock 
**         ---               configuration.
** ===================================================================
*/
#if CPU_GET_BUS_FREQ_HZ
dword CPU_GetBusFreqHz(void);
#endif /* CPU_GET_BUS_FREQ_HZ */

/*
** ===================================================================
**     Method      :  Cpu_OpenBackDoor (component MC56F82748VLH)
**     Description :
**         Makes the flash to be not secured until the next reset. This
**         method is only available when the flash security and
**         protection properties are enabled.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * key             - A pointer to the back door key. The key
**                           has to be composed from four parcial keys
**                           of the 16 bit size each. The pointer points
**                           to the parcial key 0.
**     Returns     :
**         ---             - Error code, possible codes: 
**                           - ERR_OK - OK 
**                           - ERR_BUSY - Flash device is busy 
**                           - ERR_VALUE - Backdoor keys do not match,
**                           security is not released
** ===================================================================
*/
#if CPU_OPEN_BACKDOOR
LDD_TError CPU_OpenBackDoor(LDD_TBackDoorKey *KeyPtr);
#endif /* CPU_OPEN_BACKDOOR */

/*
** ===================================================================
**     Method      :  Cpu_SetFlexRAMFunction (component MC56F82748VLH)
**     Description :
**         Changes the function of the FlexRAM as RAM or EEPROM. If is
**         FlexRAM switched to RAM function, flash device write a
**         background of ones to all FlexRAM locations. If is FlexRAM
**         switched to EEPROM function, flash device Copy-down existing
**         EEPROM data to FlexRAM. This method is available only if CPU
**         contains FlexNVM memory area and EEPROM data size value is
**         set to greater than 0. EEPROM size is set by <EEE data size>
**         property in CPU component.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Function        - Specify FlexRAM function: 
**                           - FlexRAM_AS_RAM - FlexRAM area is
**                           available for traditional RAM operation. 
**                           - FlexRAM_AS_EEPROM - FlexRAM is available
**                           for EEPROM operations.
**     Returns     :
**         ---             - Error code, possible codes: 
**                           - ERR_OK - OK 
**                           - ERR_BUSY - Flash device is busy 
**                           - ERR_NOTAVAIL - Required FlexRAM function
**                           is not defined 
**                           - ERR_FAILED - FlexRAM function is not
**                           available now, operation failed
** ===================================================================
*/
#if CPU_SET_FLEXRAM_FUNCTION
LDD_TError CPU_SetFlexRAMFunction(LDD_TFlexRAMFunction Function);
#endif /* CPU_SET_FLEXRAM_FUNCTION */

/*
** ===================================================================
**     Method      :  Cpu_SetFlexNVMPartition (component MC56F82748VLH)
**     Description :
**         Sets the FlexNVM block for use as data flash, EEPROM backup,
**         or a combination of both and initializes the FlexRAM. This
**         method is possible use only once in application life or
**         after flash memory mass erase. This method is not be
**         launched from data flash memory, because erases the contents
**         of the FlexNVM memory. This method is available only if CPU
**         contains FlexNVM memory area and EEPROM size is set to
**         greater than 0. EEPROM size is set by <EEE data size>
**         property in CPU component.
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes: 
**                           - ERR_OK - OK 
**                           - ERR_BUSY - Flash device is busy 
**                           - ERR_FAILED - Partition operation is not
**                           available in current mode/security
** ===================================================================
*/
#if CPU_SET_FLEXNVM_PARTITION
LDD_TError CPU_SetFlexNVMPartition(void);
#endif /* CPU_SET_FLEXNVM_PARTITION */

/*
** ===================================================================
**     Method      :  Cpu_InitShadowRegs (component MC56F82748VLH)
**     Description :
**         Initialize shadow registers.
**     Parameters  :
**         NAME            - DESCRIPTION
**       * Shadow          - A pointer to shadow registers
**                           content.
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_INIT_SHADOW_REGS
/*lint -save  -e586 Disable MISRA rule (2.1) checking. Method is implemented using assembler. */
asm void CPU_InitShadowRegs(TShadowRegs *Shadow);
/*lint -restore Enable MISRA rule (2.1) checking. */
#endif /* CPU_INIT_SHADOW_REGS */

/*
** ===================================================================
**     Method      :  Cpu_Delay100US (component MC56F82748VLH)
**     Description :
**         This method realizes software delay. The length of delay
**         is at least 100 microsecond multiply input parameter
**         [us100]. As the delay implementation is not based on real
**         clock, the delay time may be increased by interrupt
**         service routines processed during the delay. Adding read
**         wait states for external program memory can cause delay
**         extension as well. The method is independent on selected
**         clock configuration.
**     Parameters  :
**         NAME            - DESCRIPTION
**         us100           - Number of 100 us delay repetitions.
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_DELAY_100US
/*lint -save  -e586 Disable MISRA rule (2.1) checking. Method is implemented using assembler. */
asm void CPU_Delay100US(word us100);
/*lint -restore Enable MISRA rule (2.1) checking. */
#endif /* CPU_DELAY_100US */

/*
** ===================================================================
**     Method      :  Cpu_EnablePLL (component MC56F82748VLH)
**     Description :
**         Initializes and enables the PLL as configured in the bean
**         inspector for the clock configuration 0. This method is
**         available only if the property 'Enable PLL after reset'
**         is set to 'no'. User is responsible for enabling the PLL
**         using this method. See also hint of the property 'Enable
**         PLL after reset' for details and possible problems.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#if CPU_ENABLE_PLL
void CPU_EnablePLL(void);
#endif /* CPU_ENABLE_PLL */

#ifdef __cplusplus
}
#endif

/* END CPU_Init.h */

#endif /* __CPU_Init_H */

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
