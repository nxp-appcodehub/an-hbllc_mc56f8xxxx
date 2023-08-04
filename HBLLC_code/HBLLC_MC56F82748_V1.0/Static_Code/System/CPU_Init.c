/** ###################################################################
**      Filename    : CPU_Init.c
**      Processor   : MC56F82748VLH
**      Version     : 1.02
**      Abstract    :
**         This file implements the MCU initialization and CPU component runtime methods.
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
** @file CPU_Init.c                                                  
** @version 01.00
** @brief
**         This file implements the MCU initialization and CPU component runtime methods.
*/         
/*!
**  @addtogroup CPU_Init_module CPU_Init module documentation
**  @{
*/         

/* MODULE CPU_Init.c */

#include "PE_Types.h"
#include "MC56F82748.h"
#include "Init_Config.h"
#include "Peripherals_Init.h"
#include "CPU_Init.h"
#include "Events.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CPU_FLASH_CONFIG_FIELD
/* Flash configuration field */
/* Pragma to place the flash configuration field on correct location defined in linkler file. */
#pragma define_section reserved_FCF "reserved_FCF.text"  RX
#pragma section reserved_FCF begin
static const uint8_t _cfm[] = {CPU_FLASH_CONFIG_FIELD};
#pragma section reserved_FCF end
#endif    

#if CPU_OPEN_BACKDOOR
#define FTFA_SAFE_ROUTINE_SIZE 17U     /* Size of the safe launch and wait routine */
/* Define FTFA safe routine buffer for flash operation */ 
typedef void (* FTFA_TSafeRoutinePtr)(void); /* Safe routine pointer type */

typedef struct{
  uint16_t FTFA_SafeRoutine[FTFA_SAFE_ROUTINE_SIZE]; /* Safe routine buffer */
} FTFA_TSafeRoutine;                   /* Safe routine buffer type */

const static uint16_t FTFA_SafeRoutine[FTFA_SAFE_ROUTINE_SIZE] = {
  0xE418U, 0xC780U, 0x0001U,  /* move.l      #0x1c780,R0*/
  0x8740U, 0x0080U,           /* move.w      #128,A */
  0xD0B4U,                    /* move.bp     A1,X:(R0) */
  0xE418U, 0xC780U, 0x0001U,  /* move.l      #0x1c780,R0 */
  0xF8B4U,                    /* moveu.bp    X:(R0),A */
  0x7C02U,                    /* zxt.b       A,A */
  0x8950U, 0x0080U,           /* bftstl      #0x80,A1 */
  0xA178U,                    /* bcs         *-7 */
  0xE700U,                    /* nop */
  0xE700U,                    /* nop */
  0xE708U                     /* rts */
};

#endif

/* Global variables */
volatile uint16_t SR_lock = 0U;            /* Lock */
volatile uint16_t SR_reg;                  /* Current value of the SR register */
static uint32_t Lck0ChckCnt;               /* LCK0 check counter */
LDD_TClockConfiguration ClockConfigurationID = CPU_CLOCK_CONFIGURATION_0; /* Active clock configuration */

extern void init_56800_(void);         /* Forward declaration of external startup function declared in startup file */
extern void Common_Init(void);         /* Forward declaration of generated common initialization in generated CPU module */
extern void Components_Init(void);            /* Forward declaration of generated LDD components initialization in generated CPU module */
extern void LDD_SetClockConfiguration(LDD_TClockConfiguration ClockConfiguration);

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
void _EntryPoint(void)
{
  /* Disable constant to array reallocation compiler optimization before RAM is filled in startup */
  #pragma constarray off
  
  #if STARTUP_FAST_MODE
  if ((SIM_MISC0 & SIM_MISC0_MODE_STAT_MASK) == 0U) { /* Do not switch system to Fast mode if it is already running in it */
    /* SIM_MISC0: FAST_MODE=1 */
    PeriphSetBitMask16(SIM_MISC0, SIM_MISC0_FAST_MODE_MASK); /* Prepare fast mode */
    /* SIM_CTRL: SWRst=1 */
    PeriphSetBitMask16(SIM_CTRL, SIM_CTRL_SWRst_MASK); /* Switch to fast mode via software reset */
  }
  #else
  if ((SIM_MISC0 & SIM_MISC0_MODE_STAT_MASK) != 0U) { /* Do not switch system to Normal mode if it is already running in it */
    /* SIM_MISC0: FAST_MODE=0 */
    PeriphClearBitMask16(SIM_MISC0, SIM_MISC0_FAST_MODE_MASK); /* Prepare normal mode */
    /* SIM_CTRL: SWRst=1 */
    PeriphSetBitMask16(SIM_CTRL, SIM_CTRL_SWRst_MASK); /* Switch to normal mode via software reset */
  }
  #endif /* STARTUP_FAST_MODE */

  #if STARTUP_WDOG
  PeriphWriteReg16(COP_CTRL, STARTUP_COP_CTRL_VALUE); 
  #endif /* STARTUP_WDOG */
  
  /* Fast interrupts initialization */
  #if STARTUP_FAST_INT_0
  PeriphWriteReg16(INTC_FIM0, STARTUP_INTC_FIM0_VALUE); /* Vector number for fast interrupt 0 */ 
  PeriphWriteReg16(INTC_FIVAL0, STARTUP_INTC_FIVAL0_VALUE); /* Low address of interrupt routine */ 
  PeriphWriteReg16(INTC_FIVAH0, STARTUP_INTC_FIVAH0_VALUE); /* High address of interrupt routine */ 
  #endif /* STARTUP_FAST_INT_0 */
  #if STARTUP_FAST_INT_1
  PeriphWriteReg16(INTC_FIM1, STARTUP_INTC_FIM1_VALUE); /* Vector number for fast interrupt 1 */ 
  PeriphWriteReg16(INTC_FIVAL1, STARTUP_INTC_FIVAL1_VALUE); /* Low address of interrupt routine */ 
  PeriphWriteReg16(INTC_FIVAH1, STARTUP_INTC_FIVAH1_VALUE); /* High address of interrupt routine */ 
  #endif /* STARTUP_FAST_INT_1 */
  
  /* PLL interrupts initialization */

  #if STARTUP_PLL_INT
  PeriphSetBits16(INTC_IPR1, INTC_IPR1_OCCS_MASK, STARTUP_INTC_IPR1_VALUE & INTC_IPR1_OCCS_MASK); /* PLL interrupt priority initialization */ 
  #endif /* STARTUP_PLL_INT */
  
  /* External reference clock initialization */
  
  #if STARTUP_CLOCK_EXTERNAL
  PeriphSetBitMask16(SIM_PCE0, STARTUP_SIM_PCE0_VALUE & SIM_PCE0_GPIOC_MASK); /* External reference pins clock gate enable */ 
  
  #if (STARTUP_CLOCK_EXTERNAL_SELECT == 0)
  /* XTAL and EXTAL pin routing */
  /* GPIOC_PER: PE|=3 */
  PeriphSetBitMask16(GPIOC_PER, GPIO_PER_PE(0x03));
  /* SIM_GPSCL: C0=0 */
  PeriphClearBitMask16(SIM_GPSCL, SIM_GPSCL_C0_MASK);
  /* GPIOC_PUR: PU&=~3 */
  PeriphClearBitMask16(GPIOC_PUR, GPIO_PUR_PU(0x03)); /* EXTAL/CLKIN and XTAL pin pull-up disable */
  /* OCCS_OSCTL1: COHL=1,CLK_MODE=0,EXT_SEL=0 */
  PeriphSetBits16(OCCS_OSCTL1, \
    OCCS_OSCTL1_CLK_MODE_MASK | \
    OCCS_OSCTL1_EXT_SEL_MASK,  \
    OCCS_OSCTL1_COHL_MASK \
  );                                   /* Enable and select crystal oscillator in Loop Controlled Pierce mode as external clock source */
  /* OCCS_OSCTL2: COPD=0 */
  PeriphClearBitMask16(OCCS_OSCTL2, OCCS_OSCTL2_COPD_MASK); /* Power-up crystal oscillator */
  /* Wait about 10ms for external oscillator stabilization */
/*lint -save  -e586 Disable MISRA rule (2.1) checking. Functionality is implemented using assembler. */
  asm {
    /*
     * DelayClockConfiguration0
     *   - requested                  : 10 ms @ 4MHz,
     *   - possible                   : 40000 c, 10000000 ns
     */
    move.w #0x4E1B, A                  /* (2 c: 500 ns) number of iterations */
    do A, label0                       /* (8 c: 2000 ns) repeat 19995x nop */
      nop                              /* (1 c: 250 ns) wait for 1 c */
    label0:
  }
/*lint -restore Enable MISRA rule (2.1) checking. */
  
  #elif (STARTUP_CLOCK_EXTERNAL_SELECT == 1)
  /* CLKIN0 pin routing */
  /* GPIOC_PER: PE|=1 */
  PeriphSetBitMask16(GPIOC_PER, GPIO_PER_PE(0x01));
  /* SIM_GPSCL: C0=1 */
  PeriphSetBitMask16(SIM_GPSCL, SIM_GPSCL_C0_MASK);
  /* SIM_MISC0: CLKINSEL=0 */
  PeriphClearBitMask16(SIM_MISC0, SIM_MISC0_CLKINSEL_MASK);
  /* GPIOC_PUR: PU&=~1 */
  PeriphClearBitMask16(GPIOC_PUR, GPIO_PUR_PU(0x01)); /* CLKIN pin pull-up disable */
  /* OCCS_OSCTL1: CLK_MODE=1,EXT_SEL=1 */
  PeriphSetBitMask16(OCCS_OSCTL1, \
    OCCS_OSCTL1_CLK_MODE_MASK | \
    OCCS_OSCTL1_EXT_SEL_MASK \
  );                                   /* Select CLKIN as external clock source in bypass mode */
  
  #elif (STARTUP_CLOCK_EXTERNAL_SELECT == 2)
  /* CLKIN1 pin routing */
  /* GPIOC_PER: PE|=8 */
  PeriphSetBitMask16(GPIOC_PER, GPIO_PER_PE(0x08));
  /* SIM_GPSCL: C3=3 */
  PeriphSetBitMask16(SIM_GPSCL, SIM_GPSCL_C3(0x03));
  /* SIM_MISC0: CLKINSEL=1 */
  PeriphSetBitMask16(SIM_MISC0, SIM_MISC0_CLKINSEL_MASK);
  /* GPIOC_PUR: PU&=~8 */
  PeriphClearBitMask16(GPIOC_PUR, GPIO_PUR_PU(0x08)); /* CLKIN pin pull-up disable */
  /* OCCS_OSCTL1: CLK_MODE=1,EXT_SEL=1 */
  PeriphSetBitMask16(OCCS_OSCTL1, \
    OCCS_OSCTL1_CLK_MODE_MASK | \
    OCCS_OSCTL1_EXT_SEL_MASK \
  );                                   /* Select CLKIN as external clock source in bypass mode */
  
  #else
    #error Unsupported external clock source selected.
  #endif /* STARTUP_CLOCK_EXTERNAL_SELECT */

  #endif /* STARTUP_CLOCK_EXTERNAL */
  
  /* Slow internal reference clock initialization */

  #if STARTUP_CLOCK_INTERNAL_SLOW
  #if STARTUP_CLOCK_INTERNAL_SLOW_FREQ_TRIM
  PeriphSetBits16(OCCS_OSCTL2, OCCS_OSCTL2_FREQ_TRIM200K_MASK, STARTUP_OCCS_OSCTL2_VALUE & OCCS_OSCTL2_FREQ_TRIM200K_MASK); /* Trim slow internal clock source */ 
  #endif /* STARTUP_CLOCK_INTERNAL_SLOW_FREQ_TRIM */
  /* OCCS_OSCTL2: ROPD200K=0 */
  PeriphClearBitMask16(OCCS_OSCTL2, OCCS_OSCTL2_ROPD200K_MASK); /* Enable slow internal clock source */
  #endif /* STARTUP_CLOCK_INTERNAL_SLOW */

  /* Fast internal reference clock initialization */

  #if STARTUP_CLOCK_INTERNAL_FAST
  #if STARTUP_CLOCK_INTERNAL_FAST_FREQ_TRIM
  PeriphSetBits16(OCCS_OSCTL1, OCCS_OSCTL1_FREQ_TRIM8M_MASK, STARTUP_OCCS_OSCTL1_VALUE & OCCS_OSCTL1_FREQ_TRIM8M_MASK); /* Frequency trim of the 8MHz internal clock source */ 
  #endif /* STARTUP_CLOCK_INTERNAL_FAST_FREQ_TRIM */
  #if STARTUP_CLOCK_INTERNAL_FAST_TEMP_TRIM
  PeriphSetBits16(OCCS_OSCTL2, OCCS_OSCTL2_TEMP_TRIM8M_MASK, STARTUP_OCCS_OSCTL2_VALUE & OCCS_OSCTL2_TEMP_TRIM8M_MASK); /* Temperature trim of the 8MHz internal clock source */ 
  #endif /* STARTUP_CLOCK_INTERNAL_FAST_TEMP_TRIM */
  /* OCCS_OSCTL1: ROPD=0 */
  PeriphClearBitMask16(OCCS_OSCTL1, OCCS_OSCTL1_ROPD_MASK); /* Enable 8MHz internal relaxation oscillator */
  #endif /* STARTUP_CLOCK_INTERNAL_FAST */

  /* Clock selection */

  #if (STARTUP_CLOCK_SOURCE_SELECT == 0)
  /* OCCS_CTRL: PRECS=0 */
  PeriphClearBitMask16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03)); /* Select an internal 8MHz clock source for the CPU core */
  
  #elif (STARTUP_CLOCK_SOURCE_SELECT == 1)
  /* OCCS_CTRL: PRECS=2 */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03), OCCS_CTRL_PRECS(0x02)); /* Select an internal slow clock source for the CPU core */
  
  #elif (STARTUP_CLOCK_SOURCE_SELECT == 2)
  /* OCCS_CTRL: PRECS=1 */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03), OCCS_CTRL_PRECS(0x01));
  /* Wait 6 NOP’s for the synchronizing circuit to change clocks */
  asm(nop);
  asm(nop);
  asm(nop);
  asm(nop);
  asm(nop);
  asm(nop);
  
  #else
    #error Unsupported clock source selected.
  #endif /* STARTUP_CLOCK_SOURCE_SELECT */

  /* PLL initialization */
  
  #if STARTUP_PLL
  /* OCCS_CTRL: LCKON=1,PLLPD=0 */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLPD_MASK, OCCS_CTRL_LCKON_MASK); /* Enable PLL and lock detector */
  PeriphSetBits16(OCCS_DIVBY, OCCS_DIVBY_COD_MASK | OCCS_DIVBY_PLLDB_MASK | OCCS_DIVBY_PWM_DIV2_MASK, STARTUP_OCCS_DIVBY_VALUE & (OCCS_DIVBY_COD_MASK | OCCS_DIVBY_PLLDB_MASK | OCCS_DIVBY_PWM_DIV2_MASK)); /* Set the clock prescaler register */ 
  Lck0ChckCnt = 0;                     /* Init. PLL lock counter */
  while (Lck0ChckCnt < NUMBER_OF_PLL_CHECKS) { /* Wait for number of PLL locks */
    while(!getRegBit(OCCS_STAT, LCK0)){ /* Wait for PLL lock */
      Lck0ChckCnt = 0;                 /* PLL not locked */
    }
    Lck0ChckCnt++;                     /* PLL lock counter increment */
  }
  /* OCCS_CTRL: ZSRC=1 */
  PeriphSetBitMask16(OCCS_CTRL, OCCS_CTRL_ZSRC_MASK); /* Select clock source from PLL */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK, STARTUP_OCCS_CTRL_VALUE & (OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK)); /* Set PLL interrupts */ 
  #else /* STARTUP_PLL */
  PeriphSetBits16(OCCS_DIVBY, OCCS_DIVBY_COD_MASK, STARTUP_OCCS_DIVBY_VALUE & OCCS_DIVBY_COD_MASK); /* Set the clock postscaler */ 
  #endif /* STARTUP_PLL */
  
  PeriphWriteReg16(SIM_PCR, STARTUP_SIM_PCR_VALUE); /* Set the peripheral clock rate register */ 
  PeriphWriteReg16(OCCS_PROT, STARTUP_OCCS_PROT_VALUE); /* Set the peripheral clock protection register */ 
  
  asm(JMP init_56800_);                /* Jump to C startup code */
}

extern void PE_SystemTask(void *data);
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
void PE_low_level_init(void)
{
  /* Vector base address initialization */
  
  #if CPU_VECTOR_BASE_ADDRESS
  PeriphWriteReg16(INTC_VBA, CPU_INTC_VBA_VALUE); /* Set the vector base address register */ 
  #endif /* CPU_VECTOR_BASE_ADDRESS */

  /* Unused pins initialization */
  
  #if CPU_UNUSED_PINS_INIT
  UnusedPins_Init();
  #endif /* CPU_UNUSED_PINS_INIT */

  /* Saturation initialization */
  
  #if CPU_SATURATION
/*lint -save  -e586 Disable MISRA rule (2.1) checking. Functionality is implemented using assembler. */
  /* Set two's complement rounding and enable saturation */
  asm {
    bfset #0x10,OMR
    bfset #0x20,OMR
    bfclr #0x40,SR
  }
/*lint -restore Enable MISRA rule (2.1) checking. */
  #endif
  
  /* Shadow registers initialization */
  
  #if CPU_SHADOW_REGISTERS
/*lint -save  -e586 Disable MISRA rule (2.1) checking. Functionality is implemented using assembler. */
  /* Shadow registers initialization */
  asm {
    swap shadows
    nop
    nop
    move.l #CPU_SHADOW_R0_VALUE, R0
    move.l #CPU_SHADOW_R1_VALUE, R1
    move.l #CPU_SHADOW_R2_VALUE, R2
    move.l #CPU_SHADOW_R3_VALUE, R3
    move.l #CPU_SHADOW_R4_VALUE, R4
    move.l #CPU_SHADOW_R5_VALUE, R5
    move.l #CPU_SHADOW_N_VALUE, N
    moveu.w #CPU_SHADOW_N3_VALUE, N3
    moveu.w #CPU_SHADOW_M01_VALUE, M01
    nop
    nop
    swap shadows
  }
/*lint -restore Enable MISRA rule (2.1) checking. */
  #endif
  
  /* Bandgap trim inititilaiztion */
  
  #if CPU_BANDGAP_TRIM
  PeriphWriteReg16(PMC_CTRL, PMC_CTRL_TRIM_MASK & CPU_BANDGAP_TRIM_VALUE); /* Initialize bandgap trim */ 
  #endif

  /* Reset pin initialization */
  
  #if CPU_RESET_PIN
  /* Reset pin routing */
  /* GPIOD_PER: PE|=0x10 */
  PeriphSetBitMask16(GPIOD_PER, GPIO_PER_PE(0x10));
  #endif /* CPU_RESET_PIN */
  
  /* JTAG pins initialization */

  #if CPU_TDI_PIN
  /* Test data in pin routing */
  /* GPIOD_PER: PE|=1 */
  PeriphSetBitMask16(GPIOD_PER, GPIO_PER_PE(0x01));
  #endif /* CPU_TDI_PIN */
  #if CPU_TDO_PIN
  /* Test data out pin routing */
  /* GPIOD_PER: PE|=2 */
  PeriphSetBitMask16(GPIOD_PER, GPIO_PER_PE(0x02));
  #endif /* CPU_TDO_PIN */
  #if CPU_TCK_PIN
  /* Test clock pin routing */
  /* GPIOD_PER: PE|=4 */
  PeriphSetBitMask16(GPIOD_PER, GPIO_PER_PE(0x04));
  #endif /* CPU_TCK_PIN */
  #if CPU_TMS_PIN
  /* Test mode select pin routing */
  /* GPIOD_PER: PE|=8 */
  PeriphSetBitMask16(GPIOD_PER, GPIO_PER_PE(0x08));
  #endif /* CPU_TMS_PIN */
  
  /* OnReset event */
  
  #ifdef CPU_ON_RESET_EVENT_NAME
  CPU_ON_RESET_EVENT_NAME((uint8_t)getReg(SIM_RSTAT)); /* Invoke an user event */
  #endif /* CPU_ON_RESET_EVENT_NAME */

  /* Common initialization */
  
  #if CPU_COMMON_INIT
  Common_Init();
  #endif /* CPU_COMMON_INIT */
  
  /* Peripheral initialization components initialization */

  #if CPU_PERIPHERALS_INIT
  Peripherals_Init();
  #endif /* CPU_PERIPHERALS_INIT */
  
  /* Rest of components initialization */
  
  #if CPU_COMPONENTS_INIT
  Components_Init();
  #endif /* CPU_COMPONENTS_INIT */
  
  /* Interrupts priority level initialization */
  
  #if CPU_INT_PRIORITY == 0
  __EI(0);                             /* Enable interrupts of priority level 0 and higher */
  #elif CPU_INT_PRIORITY == 1
  __EI(1);                             /* Enable interrupts of priority level 1 and higher */
  #elif CPU_INT_PRIORITY == 2
  __EI(2);                             /* Enable interrupts of priority level 2 and higher */
  #elif CPU_INT_PRIORITY == 3
  __EI(3);                             /* Enable interrupts of priority level 3 and higher - all maskable interrupts disabled */
  #else
    #error Unsupported interrupt priority level
  #endif
  
}

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
void UnusedPins_Init(void)
{
  /* 
     GPIOA unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOA
    /* Set GPIOA pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOA_PER_MASK /* CPU_UNUSED_PINS_GPIOA_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOA_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_PER_MASK != 0 */
    PeriphWriteReg16(GPIOA_PER, CPU_UNUSED_PINS_GPIOA_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOA_PER_MASK == CPU_UNUSED_PINS_GPIOA_PER_VALUE /* CPU_UNUSED_PINS_GPIOA_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOA_PER, CPU_UNUSED_PINS_GPIOA_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOA_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOA_PER, CPU_UNUSED_PINS_GPIOA_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOA_PER_MASK != 0*/
    PeriphSetBits16(GPIOA_PER, CPU_UNUSED_PINS_GPIOA_PER_MASK, CPU_UNUSED_PINS_GPIOA_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOA_PER_VALUE)
    PeriphWriteReg16(GPIOA_PER, CPU_UNUSED_PINS_GPIOA_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOA_PER_MASK */
    /* Set GPIOA pins direction */
    #if CPU_UNUSED_PINS_GPIOA_DDR_MASK /* CPU_UNUSED_PINS_GPIOA_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOA_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOA_DDR, CPU_UNUSED_PINS_GPIOA_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOA_DDR_MASK == CPU_UNUSED_PINS_GPIOA_DDR_VALUE /* CPU_UNUSED_PINS_GPIOA_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOA_DDR, CPU_UNUSED_PINS_GPIOA_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOA_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOA_DDR, CPU_UNUSED_PINS_GPIOA_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOA_DDR_MASK != 0*/
    PeriphSetBits16(GPIOA_DDR, CPU_UNUSED_PINS_GPIOA_DDR_MASK, CPU_UNUSED_PINS_GPIOA_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOA_DDR_VALUE)
    PeriphWriteReg16(GPIOA_DDR, CPU_UNUSED_PINS_GPIOA_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOA_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOA pins pull-up */
      #if CPU_UNUSED_PINS_GPIOA_PUR_MASK /* CPU_UNUSED_PINS_GPIOA_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOA_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOA_PUR, CPU_UNUSED_PINS_GPIOA_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PUR_MASK == CPU_UNUSED_PINS_GPIOA_PUR_VALUE /* CPU_UNUSED_PINS_GPIOA_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOA_PUR, CPU_UNUSED_PINS_GPIOA_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOA_PUR, CPU_UNUSED_PINS_GPIOA_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOA_PUR_MASK != 0*/
      PeriphSetBits16(GPIOA_PUR, CPU_UNUSED_PINS_GPIOA_PUR_MASK, CPU_UNUSED_PINS_GPIOA_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOA_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOA_PUR_VALUE)
      PeriphWriteReg16(GPIOA_PUR, CPU_UNUSED_PINS_GPIOA_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_PUR_MASK */
      /* Select GPIOA pins pull-up */
      #if CPU_UNUSED_PINS_GPIOA_PUS_MASK /* CPU_UNUSED_PINS_GPIOA_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOA_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOA_PUS, CPU_UNUSED_PINS_GPIOA_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PUS_MASK == CPU_UNUSED_PINS_GPIOA_PUS_VALUE /* CPU_UNUSED_PINS_GPIOA_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOA_PUS, CPU_UNUSED_PINS_GPIOA_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOA_PUS, CPU_UNUSED_PINS_GPIOA_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOA_PUS_MASK != 0*/
      PeriphSetBits16(GPIOA_PUS, CPU_UNUSED_PINS_GPIOA_PUS_MASK, CPU_UNUSED_PINS_GPIOA_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOA_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOA_PUS_VALUE)
      PeriphWriteReg16(GPIOA_PUS, CPU_UNUSED_PINS_GPIOA_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOA pins output value */
      #if CPU_UNUSED_PINS_GPIOA_DR_MASK /* CPU_UNUSED_PINS_GPIOA_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOA_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_DR_MASK != 0 */
      PeriphWriteReg16(GPIOA_DR, CPU_UNUSED_PINS_GPIOA_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_DR_MASK == CPU_UNUSED_PINS_GPIOA_DR_VALUE /* CPU_UNUSED_PINS_GPIOA_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOA_DR, CPU_UNUSED_PINS_GPIOA_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOA_DR, CPU_UNUSED_PINS_GPIOA_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOA_DR_MASK != 0*/
      PeriphSetBits16(GPIOA_DR, CPU_UNUSED_PINS_GPIOA_DR_MASK, CPU_UNUSED_PINS_GPIOA_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOA_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOA_DR_VALUE)
      PeriphWriteReg16(GPIOA_DR, CPU_UNUSED_PINS_GPIOA_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_DR_MASK */
      /* Set GPIOA pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOA_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOA_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOA_PPMODE, CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PPMODE_MASK == CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOA_PPMODE, CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOA_PPMODE, CPU_UNUSED_PINS_GPIOA_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOA_PPMODE, CPU_UNUSED_PINS_GPIOA_PPMODE_MASK, CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE)
      PeriphWriteReg16(GPIOA_PPMODE, CPU_UNUSED_PINS_GPIOA_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOA_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOA unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOA */
  
  /* 
     GPIOB unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOB
    /* Set GPIOB pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOB_PER_MASK /* CPU_UNUSED_PINS_GPIOB_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOB_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_PER_MASK != 0 */
    PeriphWriteReg16(GPIOB_PER, CPU_UNUSED_PINS_GPIOB_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOB_PER_MASK == CPU_UNUSED_PINS_GPIOB_PER_VALUE /* CPU_UNUSED_PINS_GPIOB_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOB_PER, CPU_UNUSED_PINS_GPIOB_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOB_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOB_PER, CPU_UNUSED_PINS_GPIOB_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOB_PER_MASK != 0*/
    PeriphSetBits16(GPIOB_PER, CPU_UNUSED_PINS_GPIOB_PER_MASK, CPU_UNUSED_PINS_GPIOB_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOB_PER_VALUE)
    PeriphWriteReg16(GPIOB_PER, CPU_UNUSED_PINS_GPIOB_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOB_PER_MASK */
    /* Set GPIOB pins direction */
    #if CPU_UNUSED_PINS_GPIOB_DDR_MASK /* CPU_UNUSED_PINS_GPIOB_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOB_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOB_DDR, CPU_UNUSED_PINS_GPIOB_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOB_DDR_MASK == CPU_UNUSED_PINS_GPIOB_DDR_VALUE /* CPU_UNUSED_PINS_GPIOB_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOB_DDR, CPU_UNUSED_PINS_GPIOB_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOB_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOB_DDR, CPU_UNUSED_PINS_GPIOB_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOB_DDR_MASK != 0*/
    PeriphSetBits16(GPIOB_DDR, CPU_UNUSED_PINS_GPIOB_DDR_MASK, CPU_UNUSED_PINS_GPIOB_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOB_DDR_VALUE)
    PeriphWriteReg16(GPIOB_DDR, CPU_UNUSED_PINS_GPIOB_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOB_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOB pins pull-up */
      #if CPU_UNUSED_PINS_GPIOB_PUR_MASK /* CPU_UNUSED_PINS_GPIOB_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOB_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOB_PUR, CPU_UNUSED_PINS_GPIOB_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PUR_MASK == CPU_UNUSED_PINS_GPIOB_PUR_VALUE /* CPU_UNUSED_PINS_GPIOB_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOB_PUR, CPU_UNUSED_PINS_GPIOB_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOB_PUR, CPU_UNUSED_PINS_GPIOB_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOB_PUR_MASK != 0*/
      PeriphSetBits16(GPIOB_PUR, CPU_UNUSED_PINS_GPIOB_PUR_MASK, CPU_UNUSED_PINS_GPIOB_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOB_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOB_PUR_VALUE)
      PeriphWriteReg16(GPIOB_PUR, CPU_UNUSED_PINS_GPIOB_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_PUR_MASK */
      /* Select GPIOB pins pull-up */
      #if CPU_UNUSED_PINS_GPIOB_PUS_MASK /* CPU_UNUSED_PINS_GPIOB_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOB_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOB_PUS, CPU_UNUSED_PINS_GPIOB_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PUS_MASK == CPU_UNUSED_PINS_GPIOB_PUS_VALUE /* CPU_UNUSED_PINS_GPIOB_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOB_PUS, CPU_UNUSED_PINS_GPIOB_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOB_PUS, CPU_UNUSED_PINS_GPIOB_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOB_PUS_MASK != 0*/
      PeriphSetBits16(GPIOB_PUS, CPU_UNUSED_PINS_GPIOB_PUS_MASK, CPU_UNUSED_PINS_GPIOB_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOB_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOB_PUS_VALUE)
      PeriphWriteReg16(GPIOB_PUS, CPU_UNUSED_PINS_GPIOB_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOB pins output value */
      #if CPU_UNUSED_PINS_GPIOB_DR_MASK /* CPU_UNUSED_PINS_GPIOB_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOB_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_DR_MASK != 0 */
      PeriphWriteReg16(GPIOB_DR, CPU_UNUSED_PINS_GPIOB_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_DR_MASK == CPU_UNUSED_PINS_GPIOB_DR_VALUE /* CPU_UNUSED_PINS_GPIOB_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOB_DR, CPU_UNUSED_PINS_GPIOB_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOB_DR, CPU_UNUSED_PINS_GPIOB_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOB_DR_MASK != 0*/
      PeriphSetBits16(GPIOB_DR, CPU_UNUSED_PINS_GPIOB_DR_MASK, CPU_UNUSED_PINS_GPIOB_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOB_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOB_DR_VALUE)
      PeriphWriteReg16(GPIOB_DR, CPU_UNUSED_PINS_GPIOB_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_DR_MASK */
      /* Set GPIOB pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOB_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOB_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOB_PPMODE, CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PPMODE_MASK == CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOB_PPMODE, CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOB_PPMODE, CPU_UNUSED_PINS_GPIOB_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOB_PPMODE, CPU_UNUSED_PINS_GPIOB_PPMODE_MASK, CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE)
      PeriphWriteReg16(GPIOB_PPMODE, CPU_UNUSED_PINS_GPIOB_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOB_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOB unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOB */
  
  /* 
     GPIOC unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOC
    /* Set GPIOC pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOC_PER_MASK /* CPU_UNUSED_PINS_GPIOC_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOC_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_PER_MASK != 0 */
    PeriphWriteReg16(GPIOC_PER, CPU_UNUSED_PINS_GPIOC_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOC_PER_MASK == CPU_UNUSED_PINS_GPIOC_PER_VALUE /* CPU_UNUSED_PINS_GPIOC_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOC_PER, CPU_UNUSED_PINS_GPIOC_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOC_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOC_PER, CPU_UNUSED_PINS_GPIOC_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOC_PER_MASK != 0*/
    PeriphSetBits16(GPIOC_PER, CPU_UNUSED_PINS_GPIOC_PER_MASK, CPU_UNUSED_PINS_GPIOC_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOC_PER_VALUE)
    PeriphWriteReg16(GPIOC_PER, CPU_UNUSED_PINS_GPIOC_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOC_PER_MASK */
    /* Set GPIOC pins direction */
    #if CPU_UNUSED_PINS_GPIOC_DDR_MASK /* CPU_UNUSED_PINS_GPIOC_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOC_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOC_DDR, CPU_UNUSED_PINS_GPIOC_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOC_DDR_MASK == CPU_UNUSED_PINS_GPIOC_DDR_VALUE /* CPU_UNUSED_PINS_GPIOC_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOC_DDR, CPU_UNUSED_PINS_GPIOC_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOC_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOC_DDR, CPU_UNUSED_PINS_GPIOC_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOC_DDR_MASK != 0*/
    PeriphSetBits16(GPIOC_DDR, CPU_UNUSED_PINS_GPIOC_DDR_MASK, CPU_UNUSED_PINS_GPIOC_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOC_DDR_VALUE)
    PeriphWriteReg16(GPIOC_DDR, CPU_UNUSED_PINS_GPIOC_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOC_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOC pins pull-up */
      #if CPU_UNUSED_PINS_GPIOC_PUR_MASK /* CPU_UNUSED_PINS_GPIOC_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOC_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOC_PUR, CPU_UNUSED_PINS_GPIOC_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PUR_MASK == CPU_UNUSED_PINS_GPIOC_PUR_VALUE /* CPU_UNUSED_PINS_GPIOC_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOC_PUR, CPU_UNUSED_PINS_GPIOC_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOC_PUR, CPU_UNUSED_PINS_GPIOC_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOC_PUR_MASK != 0*/
      PeriphSetBits16(GPIOC_PUR, CPU_UNUSED_PINS_GPIOC_PUR_MASK, CPU_UNUSED_PINS_GPIOC_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOC_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOC_PUR_VALUE)
      PeriphWriteReg16(GPIOC_PUR, CPU_UNUSED_PINS_GPIOC_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_PUR_MASK */
      /* Select GPIOC pins pull-up */
      #if CPU_UNUSED_PINS_GPIOC_PUS_MASK /* CPU_UNUSED_PINS_GPIOC_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOC_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOC_PUS, CPU_UNUSED_PINS_GPIOC_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PUS_MASK == CPU_UNUSED_PINS_GPIOC_PUS_VALUE /* CPU_UNUSED_PINS_GPIOC_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOC_PUS, CPU_UNUSED_PINS_GPIOC_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOC_PUS, CPU_UNUSED_PINS_GPIOC_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOC_PUS_MASK != 0*/
      PeriphSetBits16(GPIOC_PUS, CPU_UNUSED_PINS_GPIOC_PUS_MASK, CPU_UNUSED_PINS_GPIOC_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOC_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOC_PUS_VALUE)
      PeriphWriteReg16(GPIOC_PUS, CPU_UNUSED_PINS_GPIOC_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOC pins output value */
      #if CPU_UNUSED_PINS_GPIOC_DR_MASK /* CPU_UNUSED_PINS_GPIOC_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOC_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_DR_MASK != 0 */
      PeriphWriteReg16(GPIOC_DR, CPU_UNUSED_PINS_GPIOC_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_DR_MASK == CPU_UNUSED_PINS_GPIOC_DR_VALUE /* CPU_UNUSED_PINS_GPIOC_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOC_DR, CPU_UNUSED_PINS_GPIOC_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOC_DR, CPU_UNUSED_PINS_GPIOC_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOC_DR_MASK != 0*/
      PeriphSetBits16(GPIOC_DR, CPU_UNUSED_PINS_GPIOC_DR_MASK, CPU_UNUSED_PINS_GPIOC_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOC_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOC_DR_VALUE)
      PeriphWriteReg16(GPIOC_DR, CPU_UNUSED_PINS_GPIOC_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_DR_MASK */
      /* Set GPIOC pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOC_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOC_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOC_PPMODE, CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PPMODE_MASK == CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOC_PPMODE, CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOC_PPMODE, CPU_UNUSED_PINS_GPIOC_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOC_PPMODE, CPU_UNUSED_PINS_GPIOC_PPMODE_MASK, CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE)
      PeriphWriteReg16(GPIOC_PPMODE, CPU_UNUSED_PINS_GPIOC_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOC_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOC unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOC */
  
  /* 
     GPIOD unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOD
    /* Set GPIOD pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOD_PER_MASK /* CPU_UNUSED_PINS_GPIOD_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOD_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_PER_MASK != 0 */
    PeriphWriteReg16(GPIOD_PER, CPU_UNUSED_PINS_GPIOD_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOD_PER_MASK == CPU_UNUSED_PINS_GPIOD_PER_VALUE /* CPU_UNUSED_PINS_GPIOD_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOD_PER, CPU_UNUSED_PINS_GPIOD_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOD_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOD_PER, CPU_UNUSED_PINS_GPIOD_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOD_PER_MASK != 0*/
    PeriphSetBits16(GPIOD_PER, CPU_UNUSED_PINS_GPIOD_PER_MASK, CPU_UNUSED_PINS_GPIOD_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOD_PER_VALUE)
    PeriphWriteReg16(GPIOD_PER, CPU_UNUSED_PINS_GPIOD_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOD_PER_MASK */
    /* Set GPIOD pins direction */
    #if CPU_UNUSED_PINS_GPIOD_DDR_MASK /* CPU_UNUSED_PINS_GPIOD_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOD_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOD_DDR, CPU_UNUSED_PINS_GPIOD_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOD_DDR_MASK == CPU_UNUSED_PINS_GPIOD_DDR_VALUE /* CPU_UNUSED_PINS_GPIOD_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOD_DDR, CPU_UNUSED_PINS_GPIOD_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOD_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOD_DDR, CPU_UNUSED_PINS_GPIOD_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOD_DDR_MASK != 0*/
    PeriphSetBits16(GPIOD_DDR, CPU_UNUSED_PINS_GPIOD_DDR_MASK, CPU_UNUSED_PINS_GPIOD_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOD_DDR_VALUE)
    PeriphWriteReg16(GPIOD_DDR, CPU_UNUSED_PINS_GPIOD_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOD_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOD pins pull-up */
      #if CPU_UNUSED_PINS_GPIOD_PUR_MASK /* CPU_UNUSED_PINS_GPIOD_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOD_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOD_PUR, CPU_UNUSED_PINS_GPIOD_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PUR_MASK == CPU_UNUSED_PINS_GPIOD_PUR_VALUE /* CPU_UNUSED_PINS_GPIOD_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOD_PUR, CPU_UNUSED_PINS_GPIOD_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOD_PUR, CPU_UNUSED_PINS_GPIOD_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOD_PUR_MASK != 0*/
      PeriphSetBits16(GPIOD_PUR, CPU_UNUSED_PINS_GPIOD_PUR_MASK, CPU_UNUSED_PINS_GPIOD_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOD_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOD_PUR_VALUE)
      PeriphWriteReg16(GPIOD_PUR, CPU_UNUSED_PINS_GPIOD_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_PUR_MASK */
      /* Select GPIOD pins pull-up */
      #if CPU_UNUSED_PINS_GPIOD_PUS_MASK /* CPU_UNUSED_PINS_GPIOD_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOD_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOD_PUS, CPU_UNUSED_PINS_GPIOD_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PUS_MASK == CPU_UNUSED_PINS_GPIOD_PUS_VALUE /* CPU_UNUSED_PINS_GPIOD_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOD_PUS, CPU_UNUSED_PINS_GPIOD_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOD_PUS, CPU_UNUSED_PINS_GPIOD_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOD_PUS_MASK != 0*/
      PeriphSetBits16(GPIOD_PUS, CPU_UNUSED_PINS_GPIOD_PUS_MASK, CPU_UNUSED_PINS_GPIOD_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOD_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOD_PUS_VALUE)
      PeriphWriteReg16(GPIOD_PUS, CPU_UNUSED_PINS_GPIOD_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOD pins output value */
      #if CPU_UNUSED_PINS_GPIOD_DR_MASK /* CPU_UNUSED_PINS_GPIOD_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOD_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_DR_MASK != 0 */
      PeriphWriteReg16(GPIOD_DR, CPU_UNUSED_PINS_GPIOD_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_DR_MASK == CPU_UNUSED_PINS_GPIOD_DR_VALUE /* CPU_UNUSED_PINS_GPIOD_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOD_DR, CPU_UNUSED_PINS_GPIOD_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOD_DR, CPU_UNUSED_PINS_GPIOD_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOD_DR_MASK != 0*/
      PeriphSetBits16(GPIOD_DR, CPU_UNUSED_PINS_GPIOD_DR_MASK, CPU_UNUSED_PINS_GPIOD_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOD_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOD_DR_VALUE)
      PeriphWriteReg16(GPIOD_DR, CPU_UNUSED_PINS_GPIOD_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_DR_MASK */
      /* Set GPIOD pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOD_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOD_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOD_PPMODE, CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PPMODE_MASK == CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOD_PPMODE, CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOD_PPMODE, CPU_UNUSED_PINS_GPIOD_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOD_PPMODE, CPU_UNUSED_PINS_GPIOD_PPMODE_MASK, CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE)
      PeriphWriteReg16(GPIOD_PPMODE, CPU_UNUSED_PINS_GPIOD_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOD_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOD unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOD */
  
  /* 
     GPIOE unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOE
    /* Set GPIOE pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOE_PER_MASK /* CPU_UNUSED_PINS_GPIOE_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOE_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_PER_MASK != 0 */
    PeriphWriteReg16(GPIOE_PER, CPU_UNUSED_PINS_GPIOE_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOE_PER_MASK == CPU_UNUSED_PINS_GPIOE_PER_VALUE /* CPU_UNUSED_PINS_GPIOE_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOE_PER, CPU_UNUSED_PINS_GPIOE_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOE_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOE_PER, CPU_UNUSED_PINS_GPIOE_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOE_PER_MASK != 0*/
    PeriphSetBits16(GPIOE_PER, CPU_UNUSED_PINS_GPIOE_PER_MASK, CPU_UNUSED_PINS_GPIOE_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOE_PER_VALUE)
    PeriphWriteReg16(GPIOE_PER, CPU_UNUSED_PINS_GPIOE_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOE_PER_MASK */
    /* Set GPIOE pins direction */
    #if CPU_UNUSED_PINS_GPIOE_DDR_MASK /* CPU_UNUSED_PINS_GPIOE_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOE_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOE_DDR, CPU_UNUSED_PINS_GPIOE_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOE_DDR_MASK == CPU_UNUSED_PINS_GPIOE_DDR_VALUE /* CPU_UNUSED_PINS_GPIOE_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOE_DDR, CPU_UNUSED_PINS_GPIOE_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOE_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOE_DDR, CPU_UNUSED_PINS_GPIOE_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOE_DDR_MASK != 0*/
    PeriphSetBits16(GPIOE_DDR, CPU_UNUSED_PINS_GPIOE_DDR_MASK, CPU_UNUSED_PINS_GPIOE_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOE_DDR_VALUE)
    PeriphWriteReg16(GPIOE_DDR, CPU_UNUSED_PINS_GPIOE_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOE_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOE pins pull-up */
      #if CPU_UNUSED_PINS_GPIOE_PUR_MASK /* CPU_UNUSED_PINS_GPIOE_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOE_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOE_PUR, CPU_UNUSED_PINS_GPIOE_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PUR_MASK == CPU_UNUSED_PINS_GPIOE_PUR_VALUE /* CPU_UNUSED_PINS_GPIOE_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOE_PUR, CPU_UNUSED_PINS_GPIOE_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOE_PUR, CPU_UNUSED_PINS_GPIOE_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOE_PUR_MASK != 0*/
      PeriphSetBits16(GPIOE_PUR, CPU_UNUSED_PINS_GPIOE_PUR_MASK, CPU_UNUSED_PINS_GPIOE_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOE_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOE_PUR_VALUE)
      PeriphWriteReg16(GPIOE_PUR, CPU_UNUSED_PINS_GPIOE_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_PUR_MASK */
      /* Select GPIOE pins pull-up */
      #if CPU_UNUSED_PINS_GPIOE_PUS_MASK /* CPU_UNUSED_PINS_GPIOE_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOE_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOE_PUS, CPU_UNUSED_PINS_GPIOE_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PUS_MASK == CPU_UNUSED_PINS_GPIOE_PUS_VALUE /* CPU_UNUSED_PINS_GPIOE_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOE_PUS, CPU_UNUSED_PINS_GPIOE_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOE_PUS, CPU_UNUSED_PINS_GPIOE_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOE_PUS_MASK != 0*/
      PeriphSetBits16(GPIOE_PUS, CPU_UNUSED_PINS_GPIOE_PUS_MASK, CPU_UNUSED_PINS_GPIOE_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOE_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOE_PUS_VALUE)
      PeriphWriteReg16(GPIOE_PUS, CPU_UNUSED_PINS_GPIOE_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOE pins output value */
      #if CPU_UNUSED_PINS_GPIOE_DR_MASK /* CPU_UNUSED_PINS_GPIOE_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOE_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_DR_MASK != 0 */
      PeriphWriteReg16(GPIOE_DR, CPU_UNUSED_PINS_GPIOE_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_DR_MASK == CPU_UNUSED_PINS_GPIOE_DR_VALUE /* CPU_UNUSED_PINS_GPIOE_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOE_DR, CPU_UNUSED_PINS_GPIOE_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOE_DR, CPU_UNUSED_PINS_GPIOE_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOE_DR_MASK != 0*/
      PeriphSetBits16(GPIOE_DR, CPU_UNUSED_PINS_GPIOE_DR_MASK, CPU_UNUSED_PINS_GPIOE_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOE_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOE_DR_VALUE)
      PeriphWriteReg16(GPIOE_DR, CPU_UNUSED_PINS_GPIOE_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_DR_MASK */
      /* Set GPIOE pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOE_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOE_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOE_PPMODE, CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PPMODE_MASK == CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOE_PPMODE, CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOE_PPMODE, CPU_UNUSED_PINS_GPIOE_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOE_PPMODE, CPU_UNUSED_PINS_GPIOE_PPMODE_MASK, CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE)
      PeriphWriteReg16(GPIOE_PPMODE, CPU_UNUSED_PINS_GPIOE_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOE_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOE unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOE */
  
  /* 
     GPIOF unused pins initialization 
   */
   
  #if CPU_UNUSED_PINS_GPIOF
    /* Set GPIOF pins to GPIO mode */
    #if CPU_UNUSED_PINS_GPIOF_PER_MASK /* CPU_UNUSED_PINS_GPIOF_PER_MASK */
      #if CPU_UNUSED_PINS_GPIOF_PER_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_PER_MASK != 0 */
    PeriphWriteReg16(GPIOF_PER, CPU_UNUSED_PINS_GPIOF_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOF_PER_MASK == CPU_UNUSED_PINS_GPIOF_PER_VALUE /* CPU_UNUSED_PINS_GPIOF_PER_MASK != 0 */
    PeriphSetBitMask16(GPIOF_PER, CPU_UNUSED_PINS_GPIOF_PER_VALUE);
      #elif CPU_UNUSED_PINS_GPIOF_PER_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_PER_MASK != 0 */
    PeriphClearBitMask16(GPIOF_PER, CPU_UNUSED_PINS_GPIOF_PER_MASK);
      #else /* CPU_UNUSED_PINS_GPIOF_PER_MASK != 0*/
    PeriphSetBits16(GPIOF_PER, CPU_UNUSED_PINS_GPIOF_PER_MASK, CPU_UNUSED_PINS_GPIOF_PER_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_PER_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOF_PER_VALUE)
    PeriphWriteReg16(GPIOF_PER, CPU_UNUSED_PINS_GPIOF_PER_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOF_PER_MASK */
    /* Set GPIOF pins direction */
    #if CPU_UNUSED_PINS_GPIOF_DDR_MASK /* CPU_UNUSED_PINS_GPIOF_DDR_MASK */
      #if CPU_UNUSED_PINS_GPIOF_DDR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_DDR_MASK != 0 */
    PeriphWriteReg16(GPIOF_DDR, CPU_UNUSED_PINS_GPIOF_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOF_DDR_MASK == CPU_UNUSED_PINS_GPIOF_DDR_VALUE /* CPU_UNUSED_PINS_GPIOF_DDR_MASK != 0 */
    PeriphSetBitMask16(GPIOF_DDR, CPU_UNUSED_PINS_GPIOF_DDR_VALUE);
      #elif CPU_UNUSED_PINS_GPIOF_DDR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_DDR_MASK != 0 */
    PeriphClearBitMask16(GPIOF_DDR, CPU_UNUSED_PINS_GPIOF_DDR_MASK);
      #else /* CPU_UNUSED_PINS_GPIOF_DDR_MASK != 0*/
    PeriphSetBits16(GPIOF_DDR, CPU_UNUSED_PINS_GPIOF_DDR_MASK, CPU_UNUSED_PINS_GPIOF_DDR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_DDR_MASK != 0*/
    #elif defined(CPU_UNUSED_PINS_GPIOF_DDR_VALUE)
    PeriphWriteReg16(GPIOF_DDR, CPU_UNUSED_PINS_GPIOF_DDR_VALUE);
    #endif /* CPU_UNUSED_PINS_GPIOF_DDR_MASK */
  
    #if (CPU_UNUSED_PINS_DIRECTION == 0) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused input pins features */
      /* Enable GPIOF pins pull-up */
      #if CPU_UNUSED_PINS_GPIOF_PUR_MASK /* CPU_UNUSED_PINS_GPIOF_PUR_MASK */
        #if CPU_UNUSED_PINS_GPIOF_PUR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_PUR_MASK != 0 */
      PeriphWriteReg16(GPIOF_PUR, CPU_UNUSED_PINS_GPIOF_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PUR_MASK == CPU_UNUSED_PINS_GPIOF_PUR_VALUE /* CPU_UNUSED_PINS_GPIOF_PUR_MASK != 0 */
      PeriphSetBitMask16(GPIOF_PUR, CPU_UNUSED_PINS_GPIOF_PUR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PUR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_PUR_MASK != 0 */
      PeriphClearBitMask16(GPIOF_PUR, CPU_UNUSED_PINS_GPIOF_PUR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOF_PUR_MASK != 0*/
      PeriphSetBits16(GPIOF_PUR, CPU_UNUSED_PINS_GPIOF_PUR_MASK, CPU_UNUSED_PINS_GPIOF_PUR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOF_PUR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOF_PUR_VALUE)
      PeriphWriteReg16(GPIOF_PUR, CPU_UNUSED_PINS_GPIOF_PUR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_PUR_MASK */
      /* Select GPIOF pins pull-up */
      #if CPU_UNUSED_PINS_GPIOF_PUS_MASK /* CPU_UNUSED_PINS_GPIOF_PUS_MASK */
        #if CPU_UNUSED_PINS_GPIOF_PUS_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_PUS_MASK != 0 */
      PeriphWriteReg16(GPIOF_PUS, CPU_UNUSED_PINS_GPIOF_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PUS_MASK == CPU_UNUSED_PINS_GPIOF_PUS_VALUE /* CPU_UNUSED_PINS_GPIOF_PUS_MASK != 0 */
      PeriphSetBitMask16(GPIOF_PUS, CPU_UNUSED_PINS_GPIOF_PUS_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PUS_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_PUS_MASK != 0 */
      PeriphClearBitMask16(GPIOF_PUS, CPU_UNUSED_PINS_GPIOF_PUS_MASK);
        #else /* CPU_UNUSED_PINS_GPIOF_PUS_MASK != 0*/
      PeriphSetBits16(GPIOF_PUS, CPU_UNUSED_PINS_GPIOF_PUS_MASK, CPU_UNUSED_PINS_GPIOF_PUS_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOF_PUS_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOF_PUS_VALUE)
      PeriphWriteReg16(GPIOF_PUS, CPU_UNUSED_PINS_GPIOF_PUS_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_PUS_MASK */
  
    #elif (CPU_UNUSED_PINS_DIRECTION == 1) /* CPU_UNUSED_PINS_DIRECTION */
      /* Initialize unused output pins features */
      /* Set GPIOF pins output value */
      #if CPU_UNUSED_PINS_GPIOF_DR_MASK /* CPU_UNUSED_PINS_GPIOF_DR_MASK */
        #if CPU_UNUSED_PINS_GPIOF_DR_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_DR_MASK != 0 */
      PeriphWriteReg16(GPIOF_DR, CPU_UNUSED_PINS_GPIOF_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_DR_MASK == CPU_UNUSED_PINS_GPIOF_DR_VALUE /* CPU_UNUSED_PINS_GPIOF_DR_MASK != 0 */
      PeriphSetBitMask16(GPIOF_DR, CPU_UNUSED_PINS_GPIOF_DR_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_DR_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_DR_MASK != 0 */
      PeriphClearBitMask16(GPIOF_DR, CPU_UNUSED_PINS_GPIOF_DR_MASK);
        #else /* CPU_UNUSED_PINS_GPIOF_DR_MASK != 0*/
      PeriphSetBits16(GPIOF_DR, CPU_UNUSED_PINS_GPIOF_DR_MASK, CPU_UNUSED_PINS_GPIOF_DR_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOF_DR_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOF_DR_VALUE)
      PeriphWriteReg16(GPIOF_DR, CPU_UNUSED_PINS_GPIOF_DR_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_DR_MASK */
      /* Set GPIOF pins push-pull/open drain mode */
      #if CPU_UNUSED_PINS_GPIOF_PPMODE_MASK /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK */
        #if CPU_UNUSED_PINS_GPIOF_PPMODE_MASK == 0xFFFF /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK != 0 */
      PeriphWriteReg16(GPIOF_PPMODE, CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PPMODE_MASK == CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK != 0 */
      PeriphSetBitMask16(GPIOF_PPMODE, CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE);
        #elif CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE == 0 /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK != 0 */
      PeriphClearBitMask16(GPIOF_PPMODE, CPU_UNUSED_PINS_GPIOF_PPMODE_MASK);
        #else /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK != 0*/
      PeriphSetBits16(GPIOF_PPMODE, CPU_UNUSED_PINS_GPIOF_PPMODE_MASK, CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE);
        #endif /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK != 0*/
      #elif defined(CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE)
      PeriphWriteReg16(GPIOF_PPMODE, CPU_UNUSED_PINS_GPIOF_PPMODE_VALUE);
      #endif /* CPU_UNUSED_PINS_GPIOF_PPMODE_MASK */
      
    #else /* CPU_UNUSED_PINS_DIRECTION */
      #error Unsupported direction of GPIOF unused pins.
    #endif /* CPU_UNUSED_PINS_DIRECTION */
  #endif /* CPU_UNUSED_PINS_GPIOF */
  
}
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
LDD_TError CPU_SetClockConfiguration(LDD_TClockConfiguration ModeID)
{
  if (ModeID > CPU_CLOCK_CONFIGURATIONS_NUMBER) {
    return ERR_RANGE;
  }

  switch (CpuClockSources[ModeID]) {
    case CPU_CLOCK_SOURCE_EXTERNAL:
      /* OCCS_CTRL: PRECS=1 */
      PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03), OCCS_CTRL_PRECS(0x01)); /* Select an external clock source for the CPU core */
      /* Wait 6 NOP’s for the synchronizing circuit to change clocks */
      asm(nop);
      asm(nop);
      asm(nop);
      asm(nop);
      asm(nop);
      asm(nop);
      break;
    case CPU_CLOCK_SOURCE_INTERNAL_FAST:
      /* OCCS_CTRL: PRECS=0 */
      PeriphClearBitMask16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03)); /* Select an internal 8MHz clock source for the CPU core */
      break;
  case CPU_CLOCK_SOURCE_INTERNAL_SLOW:
      /* OCCS_CTRL: PRECS=2 */
      PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PRECS(0x03), OCCS_CTRL_PRECS(0x02)); /* Select an internal slow clock source for the CPU core */
      break;
    default:
      return ERR_PARAM_MODE;
  }
  if (CpuPllEnabled[ClockConfigurationID] != 0U) {
    /* OCCS_CTRL: PLLIE1=0,PLLIE0=0,LOCIE=0 */
    PeriphClearBitMask16(OCCS_CTRL, \
      OCCS_CTRL_PLLIE1(0x03) | \
      OCCS_CTRL_PLLIE0(0x03) | \
      OCCS_CTRL_LOCIE_MASK \
    );                                 /* Disable PLL interrupts */
  }
  if (CpuPllEnabled[ModeID] != 0U) {
    /* OCCS_CTRL: LCKON=1,PLLPD=0 */
    PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLPD_MASK, OCCS_CTRL_LCKON_MASK); /* Enable PLL and lock detector */
    PeriphSetBits16(OCCS_DIVBY, OCCS_DIVBY_COD_MASK | OCCS_DIVBY_PLLDB_MASK | OCCS_DIVBY_PWM_DIV2_MASK, CpuClockDividers[ModeID]); /* Set the clock prescaler register */ 
    Lck0ChckCnt = 0;                   /* Init. PLL lock counter */
    while (Lck0ChckCnt < CpuNumberOfPllChecks[ModeID]) { /* Wait for number of PLL locks */
      while(!getRegBit(OCCS_STAT, LCK0)){ /* Wait for PLL lock */
        Lck0ChckCnt = 0;               /* PLL not locked */
      }
      Lck0ChckCnt++;                   /* PLL lock counter increment */
    }
    /* OCCS_CTRL: ZSRC=1 */
    PeriphSetBitMask16(OCCS_CTRL, OCCS_CTRL_ZSRC_MASK); /* Select clock source from PLL */
    PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK, STARTUP_OCCS_CTRL_VALUE & (OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK)); /* Set PLL interrupts */ 
  }  
  else {
    /* OCCS_CTRL: PLLPD=1,ZSRC=0 */
    PeriphSetBits16(OCCS_CTRL, \
      OCCS_CTRL_PLLPD_MASK | \
      OCCS_CTRL_ZSRC_MASK,  \
      OCCS_CTRL_PLLPD_MASK \
    );                                 /* Disable PLL and select clock source from MSTR_OSC */
    PeriphSetBits16(OCCS_DIVBY, OCCS_DIVBY_COD_MASK, CpuClockDividers[ModeID]); /* Set the clock postscaler */ 
  }
  PeriphWriteReg16(SIM_PCR, CpuClockRates[ModeID]); /* Set the peripheral clock rate register */ 
  LDD_SetClockConfiguration(ModeID);   /* Call all LDD components to update the clock configuration */
  ClockConfigurationID = ModeID;       /* Store clock configuration identifier */
  return ERR_OK;
}
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
LDD_TClockConfiguration CPU_GetClockConfiguration(void)
{
#if CPU_CLOCK_CONFIGURATIONS_NUMBER > 1
  return ClockConfigurationID;         /* Return the actual clock configuration identifier */
#else
  return CPU_CLOCK_CONFIGURATION_0;    /* Return the actual clock configuration identifier */
#endif
}
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
LDD_TError CPU_SetOperationMode(LDD_TDriverOperationMode OperationMode, LDD_TCallback ModeChangeCallback, LDD_TCallbackParam *ModeChangeCallbackParamPtr)
{
  (void) ModeChangeCallback;           /* Parameter is not used, suppress unused argument warning */
  (void) ModeChangeCallbackParamPtr;   /* Parameter is not used, suppress unused argument warning */
  switch (OperationMode) {
    case DOM_RUN:
      break;
    case DOM_WAIT:
    case DOM_SLEEP:
      asm(WAIT);                       /* Enter the WAIT processing state */
      break;
    case DOM_STOP:
      asm(STOP);                       /* Enter the STOP processing state */
      break;
    default:
      return ERR_PARAM_MODE;
  }
  return ERR_OK;
}
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
/*
void CPU_EnableInt(void)

**      This method is implemented as macro in the header module. **
*/

/*
** ===================================================================
**     Method      :  Cpu_DisableInt (component MC56F82748VLH)
**     Description :
**         Disables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void CPU_DisableInt(void)

**      This method is implemented as macro in the header module. **
*/

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
/*
uint8_t CPU_GetResetSource(void)

**      This method is implemented as macro in the header module. **
*/

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
/*
void CPU_SetStopMode(void)

**      This method is implemented as macro in the header module. **
*/

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
/*
void CPU_SetWaitMode(void)

**      This method is implemented as macro in the header module. **
*/

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
uint32_t CPU_GetBusFreqHz(void)
{
#if CPU_CLOCK_CONFIGURATIONS_NUMBER > 1
  return (uint32_t)CpuBusFreqHz[ClockConfigurationID];
#else /* CPU_CLOCK_CONFIGURATIONS_NUMBER > 1 */
  (uint32_t)CpuBusFreqHz[0];
#endif /* CPU_CLOCK_CONFIGURATIONS_NUMBER > 1 */
}
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
LDD_TError CPU_OpenBackDoor(LDD_TBackDoorKey *KeyPtr)
{
  FTFA_TSafeRoutine SaveRoutineStackSpace;
  FTFA_TSafeRoutinePtr SafeRoutineInDataPtr, SafeRoutineInCodePtr;
  uint8_t SaveInterruptConfig;
  register uint8_t *TmpKey = (uint8_t *)KeyPtr;

  if (getRegBit(FTFA_FSTAT, CCIF) == 0U) { /* Is FTFA device in running? */
    return ERR_BUSY;                   /* If yes, return error code */
  }
  SaveInterruptConfig = (byte)(getReg8(FTFA_FCNFG) & (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Save interrupt settings */
  PeriphClearBitMask8(FTFA_FCNFG, (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Disable FTFA interrupts */
  PeriphWriteReg8(FTFA_FCCOB0, 0x45U); /* Write backdoor access key command to command register */
  FTFA_FCCOB4 = TmpKey[0];             /* Set key 0 value */  
  FTFA_FCCOB5 = TmpKey[1];             /* Set key 1 value */  
  FTFA_FCCOB6 = TmpKey[2];             /* Set key 2 value */  
  FTFA_FCCOB7 = TmpKey[3];             /* Set key 3 value */  
  FTFA_FCCOB8 = TmpKey[4];             /* Set key 4 value */  
  FTFA_FCCOB9 = TmpKey[5];             /* Set key 5 value */  
  FTFA_FCCOBA = TmpKey[6];             /* Set key 6 value */  
  FTFA_FCCOBB = TmpKey[7];             /* Set key 7 value */  
  SafeRoutineInDataPtr = (FTFA_TSafeRoutinePtr)&SaveRoutineStackSpace;
  *(FTFA_TSafeRoutine *)(void *)SafeRoutineInDataPtr = *(FTFA_TSafeRoutine *)(void *)&FTFA_SafeRoutine; /* Copy the safe routine's code to a buffer on the stack */
  SafeRoutineInCodePtr = (FTFA_TSafeRoutinePtr)((uint32_t)SafeRoutineInDataPtr + 0xF000UL);
  EnterCritical();                     /* Disable all low level interrupts */                                        
  SafeRoutineInCodePtr();              /* Process routine in RAM */
  ExitCritical();                      /* Enable all low level interrupts */
  if ((getReg(FTFA_FSTAT) & FTFA_FSTAT_ACCERR_MASK) != 0U){ /* Is error detected during execution command? */
    PeriphWriteReg8(FTFA_FSTAT, FTFA_FSTAT_ACCERR_MASK); /* Clear error flag */
    PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
    return ERR_VALUE;                  /* If yes then return the error */
  }
  PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
  return ERR_OK;   
}
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
#if CPU_SET_FLEX_RAM_FUNCTION
LDD_TError CPU_SetFlexRAMFunction(LDD_TFlexRAMFunction Function)
{
  uint8_t SaveInterruptConfig;

  if (getRegBit(FTFA_FSTAT, CCIF) == 0U) { /* Is FTFA device in running? */
    return ERR_BUSY;                   /* If yes, return error code */
  }
  SaveInterruptConfig = (byte)(getReg8(FTFA_FCNFG) & (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Save interrupt settings */
  PeriphClearBitMask8(FTFA_FCNFG, (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Disable FTFA interrupts */
  PeriphWriteReg8(FTFA_FCCOB0, 0x81U); /* Write FlexRAM function command to the command register */
  switch (Function) {                  /* Decode FlexRAM required function */
    case LDD_FlexRAM_AS_RAM:
      PeriphWriteReg8(FTFA_FCCOB1, 0xFFU); /* Set command parameter - FlexRAM available as RAM */
      break;
    case LDD_FlexRAM_AS_EEPROM:
      PeriphWriteReg8(FTFA_FCCOB1, 0x00U); /* Set command parameter - FlexRAM available for EEPROM */
      break;
    default:
      PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
      return ERR_NOTAVAIL;
  }  
  PeriphSetBitMask8(FTFA_FSTAT, FTFA_FSTAT_CCIF_MASK); /* Launch command */
  while (getRegBit(FTFA_FSTAT, CCIF) == 0U) {}; /* Wait till the end of the command */  
  if ((getReg(FTFA_FSTAT) & FTFA_FSTAT_ACCERR_MASK) != 0U){ /* Is error detected during execution command? */
    PeriphWriteReg8(FTFA_FSTAT, FTFA_FSTAT_ACCERR_MASK); /* Clear error flag */
    PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
    return ERR_FAILED;                 /* If yes then return the error */
  }
  PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
  return ERR_OK;     
}
#endif /* CPU_SET_FLEX_RAM_FUNCTION */

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
#if CPU_SET_FLEX_NVM_PARTITION
LDD_TError CPU_SetFlexNVMPartition(void)
{
  uint8_t SaveInterruptConfig;

  if (getRegBit(FTFA_FSTAT, CCIF) == 0U) { /* Is FTFA device in running? */
    return ERR_BUSY;                   /* If yes, return error code */
  }
  SaveInterruptConfig = (byte)(getReg8(FTFA_FCNFG) & (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Save interrupt settings */
  PeriphClearBitMask8(FTFA_FCNFG, (FTFA_FCNFG_CCIE_MASK | FTFA_FCNFG_RDCOLLIE_MASK | FTFA_FCNFG_ERSSUSP_MASK)); /* Disable FTFA interrupts */
  PeriphWriteReg8(FTFA_FCCOB0, 0x80U); /* Write program partition command to command register */
  PeriphWriteReg8(FTFA_FCCOB4, 0x37U); /* Set command parameter - EEPROM size  */
  PeriphWriteReg8(FTFA_FCCOB5, 0x01U); /* Set command parameter - data flash size */
  PeriphSetBitMask8(FTFA_FSTAT, FTFA_FSTAT_CCIF_MASK); /* Launch command */
  while (getRegBit(FTFA_FSTAT, CCIF) == 0U) {}; /* Wait till the end of the command */  
  if ((getReg(FTFA_FSTAT) & FTFA_FSTAT_ACCERR_MASK) != 0U){ /* Is error detected during execution command? */
    PeriphWriteReg8(FTFA_FSTAT, FTFA_FSTAT_ACCERR_MASK); /* Clear error flag */
    PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
    return ERR_FAILED;                 /* If yes then return the error */
  }
  PeriphWriteReg8(FTFA_FCNFG, SaveInterruptConfig); /* Restore interrupt settings */
  return ERR_OK;    
}
#endif /* CPU_SET_FLEX_NVM_PARTITION */

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
asm void CPU_InitShadowRegs(TShadowRegs *Shadow)
{
  /* Load fast interrupt registers */
  move.l     x:(R2)+,N
  moveu.w    x:(R2)+,N3
  moveu.w    x:(R2)+,M01
  #if __m56800E_lmm__         /* large data memory model */
    move.l   x:(R2)+,R0
    move.l   x:(R2)+,R1
    move.l   x:(R2)+,R3
    move.l   x:(R2)+,R4
    move.l   x:(R2)+,R5
    move.l   x:(R2),R2
  #else                       /* small data memory model */
    moveu.w  x:(R2)+,R0
    moveu.w  x:(R2)+,R1
    moveu.w  x:(R2)+,R3
    moveu.w  x:(R2)+,R4
    moveu.w  x:(R2)+,R5
    moveu.w  x:(R2),R2
  #endif
  /* Init shadow registers */
  nop
  nop
  swap shadows
  /* Reset the m01 register for linear addressing */
  moveu.w #65535,m01
  rts
}
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
asm void CPU_Delay100US(word us100)
{
  /* Total irremovable overhead consists of:
     move.w:  2 instruction cycles overhead (load parameter into register)
     jsr:     5 instruction cycles overhead (jump to subroutine)
     dec.w:   1 instruction cycle overhead (decrement us100 parameter)
     jne:     5/4 instruction cycles overhead (jump to start/continue)
     nop:     1 instruction cycle overhead (aditional nop)
     rts:     8 instruction cycles overhead (return from subroutine) */
     
  /* Total control algorithm overhead consists of:
     adda     1 instruction cycle overhead (move SP forward)
     move.l   1 instruction cycle overhead (push A)
     moveu.bp 2 instruction cycles (get current clock configuration)
     cmp.b    1 instruction cycle (test current clock configuration)
     bne      5/3 instruction cycles (go to next section/continue)
     nop:     2 instruction cycles overhead (aditional nops)
     move.w   2 instruction cycles overhead (move loop counter value to register if used)
     do:      8/6 instruction cycles (start HW do loop; counter value in register/constant)
     bra      5 instruction cycles (go to end; only if multiple clock configurations are used)
     move.l   1 instruction cycle overhead (pop A)
     suba     1 instruction cycle overhead (move SP backward) */
     
  Delay100US_loop:
  /* 100 us delay block begin */
  adda #2, SP                          /* (1 instruction cycle) move SP forward */
  move.l A10, X:(SP)                   /* (1 instruction cycles) push A */
  moveu.bp ClockConfigurationID, A     /* (2 instruction cycles) get ClockConfigurationID */
  
  /* Clock configuration 0 */
  cmp.b #CPU_CLOCK_CONFIGURATION_0, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_0 */
#if defined(CPU_CLOCK_CONFIGURATION_1)
  bne Delay100US_ClockConfig1          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig0_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig0_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_2, Delay100US_ClockConfig0_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig0_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_0_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_0_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_0_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_0_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_1)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_1) */

  /* Clock configuration 1 */
  Delay100US_ClockConfig1:             /* Clock configuration 1 block */
#if defined(CPU_CLOCK_CONFIGURATION_1)
  cmp.b #CPU_CLOCK_CONFIGURATION_1, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_1 */
#if defined(CPU_CLOCK_CONFIGURATION_2)
  bne Delay100US_ClockConfig2          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig1_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig1_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_2, Delay100US_ClockConfig1_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig1_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_1_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_1_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_1_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_1_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_2)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_2) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_1) */

  /* Clock configuration 2 */
  Delay100US_ClockConfig2:             /* Clock configuration 2 block */
#if defined(CPU_CLOCK_CONFIGURATION_2)
  cmp.b #CPU_CLOCK_CONFIGURATION_2, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_2 */
#if defined(CPU_CLOCK_CONFIGURATION_3)
  bne Delay100US_ClockConfig3          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig2_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig2_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_2, Delay100US_ClockConfig2_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig2_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_2_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_2_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_2_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_2_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_3)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_3) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_2) */

  /* Clock configuration 3 */
  Delay100US_ClockConfig3:             /* Clock configuration 3 block */
#if defined(CPU_CLOCK_CONFIGURATION_3)
  cmp.b #CPU_CLOCK_CONFIGURATION_3, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_3 */
#if defined(CPU_CLOCK_CONFIGURATION_4)
  bne Delay100US_ClockConfig4          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig3_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig3_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_2, Delay100US_ClockConfig3_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig3_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_3_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_3_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_3_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_3_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_4)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_4) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_3) */

  /* Clock configuration 4 */
  Delay100US_ClockConfig4:             /* Clock configuration 4 block */
#if defined(CPU_CLOCK_CONFIGURATION_4)
  cmp.b #CPU_CLOCK_CONFIGURATION_4, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_4 */
#if defined(CPU_CLOCK_CONFIGURATION_5)
  bne Delay100US_ClockConfig5          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig4_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig4_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_2, Delay100US_ClockConfig4_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig4_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_4_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_4_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_4_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_4_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_5)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_5) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_4) */

  /* Clock configuration 5 */
  Delay100US_ClockConfig5:             /* Clock configuration 5 block */
#if defined(CPU_CLOCK_CONFIGURATION_5)
  cmp.b #CPU_CLOCK_CONFIGURATION_5, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_5 */
#if defined(CPU_CLOCK_CONFIGURATION_6)
  bne Delay100US_ClockConfig6          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig5_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig5_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_2, Delay100US_ClockConfig5_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig5_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_5_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_5_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_5_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_5_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_6)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_6) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_5) */

  /* Clock configuration 6 */
  Delay100US_ClockConfig6:             /* Clock configuration 6 block */
#if defined(CPU_CLOCK_CONFIGURATION_6)
  cmp.b #CPU_CLOCK_CONFIGURATION_6, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_6 */
#if defined(CPU_CLOCK_CONFIGURATION_7)
  bne Delay100US_ClockConfig7          /* (5/3 instruction cycles) not equal? goto next section */
#else
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
#endif
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig6_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig6_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_2, Delay100US_ClockConfig6_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig6_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_6_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_6_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_6_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_6_NOPS) */
#if defined(CPU_CLOCK_CONFIGURATION_7)
  bra Delay100US_end_of_loop           /* (5 instruction cycles) finishing delay, goto end */
#endif /* defined(CPU_CLOCK_CONFIGURATION_7) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_6) */

  /* Clock configuration 7 */
  Delay100US_ClockConfig7:             /* Clock configuration 7 block */
#if defined(CPU_CLOCK_CONFIGURATION_7)
  cmp.b #CPU_CLOCK_CONFIGURATION_7, A  /* (1 instruction cycle) compare it to CPU_CLOCK_CONFIG_7 */
  bne Delay100US_end_of_loop           /* (5/3 instruction cycles) not equal? goto next section */
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts (2 NOPs required - second follows) */
  /* Delay loops */
#if CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1
  /* Delay loop uses counter stored in register */
  move.w #CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1, A /* (2 instruction cycles) number of iterations */
  do A, Delay100US_ClockConfig7_loop1  /* (8 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig7_loop1:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1 */
#if CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_2
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1) */
  /* Delay loop uses counter stored in constant */
  do #CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_2, Delay100US_ClockConfig7_loop2 /* (6 instruction cycles) repeat nop */
    nop
  Delay100US_ClockConfig7_loop2:
#endif /* CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_2 */
#if defined(CPU_DELAY_100US_CLOCK_CONFIG_7_NOPS)
  /* Fine tuning using additional NOP instructions */
  CPU_DELAY_100US_CLOCK_CONFIG_7_NOPS
#endif /* defined(CPU_DELAY_100US_CLOCK_CONFIG_7_NOPS) */
#if !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_NOPS)
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
#endif /* !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_1) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_LOOP_2) && !defined(CPU_DELAY_100US_CLOCK_CONFIG_7_NOPS) */
#endif /* defined(CPU_CLOCK_CONFIGURATION_7) */

  Delay100US_end_of_loop:              /* End of delays */
  move.l X:(SP), A                     /* (1 instruction cycle) pop A */
  suba #2, SP                          /* (1 instruction cycle) move SP back */
  /* 100 us delay block end */
  dec.w Y0                             /* (1 instruction cycle) us100 parameter is passed via Y0 register */
  jne Delay100US_loop                  /* (5/4 instruction cycles) next loop */
  nop                                  /* (1 instruction cycle) avoid pipeline conflicts */
  rts                                  /* (8 instruction cycles) return from subroutine */
}
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
void CPU_EnablePLL(void)
{
  /* OCCS_CTRL: LCKON=1,PLLPD=0 */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLPD_MASK, OCCS_CTRL_LCKON_MASK); /* Enable PLL and lock detector */
  PeriphSetBits16(OCCS_DIVBY, OCCS_DIVBY_COD_MASK | OCCS_DIVBY_PLLDB_MASK | OCCS_DIVBY_PWM_DIV2_MASK, STARTUP_OCCS_DIVBY_VALUE & (OCCS_DIVBY_COD_MASK | OCCS_DIVBY_PLLDB_MASK | OCCS_DIVBY_PWM_DIV2_MASK)); /* Set the clock prescaler register */ 
  Lck0ChckCnt = 0;                     /* Init. PLL lock counter */
  while (Lck0ChckCnt < CpuNumberOfPllChecks[ClockConfigurationID]) { /* Wait for number of PLL locks */
    while(!getRegBit(OCCS_STAT, LCK0)){ /* Wait for PLL lock */
      Lck0ChckCnt = 0;                 /* PLL not locked */
    }
    Lck0ChckCnt++;                     /* PLL lock counter increment */
  }
  /* OCCS_CTRL: ZSRC=1 */
  PeriphSetBitMask16(OCCS_CTRL, OCCS_CTRL_ZSRC_MASK); /* Select clock source from PLL */
  PeriphSetBits16(OCCS_CTRL, OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK, STARTUP_OCCS_CTRL_VALUE & (OCCS_CTRL_PLLIE1_MASK | OCCS_CTRL_PLLIE0_MASK | OCCS_CTRL_LOCIE_MASK)); /* Set PLL interrupts */ 
}
#endif /* CPU_ENABLE_PLL */


#ifdef __cplusplus
}
#endif

/* END CPU_Init.c */

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
