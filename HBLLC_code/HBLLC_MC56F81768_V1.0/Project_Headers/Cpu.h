/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/* ###################################################################
**     Filename    : Cpu.h
**     Processor   : MC56F81xxx
**     Component   : MC56F81xxx
**     Version     : 0.1
**     Datasheet   : MC56F81xxx Reference Manual
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2020-05-19, 16:33
*/

#ifndef __Cpu_H
#define __Cpu_H


#include <stdint.h>

#pragma define_section CODES_IN_RAM "codesInRam.text"  RX
#pragma define_section NVM_DATA "nvm.data"  RW


#define __EI0() do { asm(bfclr  #0x0300,SR); } while(0) /* Enable interrupts of level 0,1,2,3 */
#define __EI1() do { asm(bfset  #0x0100,SR); asm(bfclr  #0x0200,SR); } while(0) /* Enable interrupts of level 1,2,3 */
#define __EI2() do { asm(bfset  #0x0200,SR); asm(bfclr  #0x0100,SR); } while(0) /* Enable interrupts of level 2,3 */
#define __EI3() do { asm(bfset  #0x0300,SR); } while(0) /* Enable interrupts of level 3 */
/*lint -save  -esym(960,14.3) Disable MISRA rule (14.3) checking. */
#define __EI(level) __EI##level()      /* Enable interrupts of the given level */
/*lint -restore  +esym(960,14.3) Enable MISRA rule (14.3) checking. */
#define __DI()  __EI3()                /* Disable interrupts, only level 3 is allowed */

#define EnableWatchdog COP->CTRL |= COP_CTRL_CEN_MASK;
#define DisableWatchdog COP->CTRL &= ~COP_CTRL_CEN_MASK;


/*lint -esym(765,SR_reg,SR_lock,_EntryPoint,_vect,_vectboot) Disable MISRA rule (8.10) checking for symbols (SR_reg,SR_lock,_EntryPoint,_vect,_vectboot). */
/* Global variables */
#ifdef __cplusplus
extern "C" {
#endif
extern volatile short int SR_lock;          /* Lock */
extern volatile short int SR_reg;           /* Current value of the SR register */
#ifdef __cplusplus
}
#endif

#define   Cpu_SetStopMode()  asm(STOP) /* Set STOP mode */
/*
** ===================================================================
**     Method      :  Cpu_SetStopMode
**     Description :
**         Sets low power mode - Stop mode.
**         For more information about the stop mode see this CPU
**         documentation.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define   Cpu_SetWaitMode()  asm(WAIT) /* Set WAIT mode */
/*
** ===================================================================
**     Method      :  Cpu_SetWaitMode 
**     Description :
**         Sets low power mode - Wait mode.
**         For more information about the wait mode see this CPU
**         documentation.
**         Release from wait mode: Reset or interrupt
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/



#define   Cpu_DisableInt()  __DI()     /* Disable interrupts */
/*
** ===================================================================
**     Method      :  Cpu_DisableInt 
**     Description :
**         Disables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define   Cpu_EnableInt()  __EI(0)     /* Enable interrupts */
/*
** ===================================================================
**     Method      :  Cpu_EnableInt 
**     Description :
**         Enables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void Cpu_Interrupt(void);
/*
** ===================================================================
**     Method      :  Cpu_Interrupt 
**
**     Description :
**         The method services unhandled interrupt vectors.
** ===================================================================
*/


void _EntryPoint(void);
/*
** ===================================================================
**     Method      :  _EntryPoint 
**
**     Description :
**         Initializes the whole system like timing and so on. At the end 
**         of this function, the C startup is invoked to initialize stack,
**         memory areas and so on.
** ===================================================================
*/

/* END Cpu. */

#endif
/* __Cpu_H */
