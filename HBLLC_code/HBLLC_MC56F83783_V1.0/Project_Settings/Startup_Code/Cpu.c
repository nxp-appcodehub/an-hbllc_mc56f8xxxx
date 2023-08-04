/* 
* Copyright 2019 NXP
* SPDX_License-Identifier: BSD-3-Clause
*/

#include "derivative.h"
#include "Cpu.h"

#define CRYSTAL_PLL 0x01
#define RC8M_PLL    0x02
#define RC48M		0x03
#define RC8M		0x04
#define CLOCK_IN	0x05
#define RC8M_PLL_CRYSTAL_FOR_FLEXCAN 0x06

#define CLOCK_CONFIG RC8M_PLL

/* Global variables */
volatile int16_t SR_lock = 0U;            /* Lock */
volatile int16_t SR_reg;                  /* Current value of the SR register */
/*
** ===================================================================
**    
**
**     Description :
**         The function services unhandled interrupt vectors.
**         
** ===================================================================
*/
#pragma interrupt alignsp
void Cpu_Interrupt(void)
{
  /* This code can be changed using the CPU bean property "Build Options / Unhandled int code" */
  asm(DEBUGHLT);                       /* Halt the core and placing it in the debug processing state. */
  /* Here should be two NOPs if DEBUGHLT is removed.
  asm(nop);
  asm(nop); */
}


extern void init_56800_(void);         /* Forward declaration of external startup function declared in startup file */

/*** !!! Here you can place your own code using property "User data declarations" on the build options tab. !!! ***/

void _EntryPoint(void)
{
  #pragma constarray off
	
  /* Disable COP */
  COP->CTRL &= ~COP_CTRL_CEN_MASK;  // Disable watchdog

  /* System clock initialization */
#if CLOCK_CONFIG == CRYSTAL_PLL
  SIM->PCE0 |= SIM_PCE0_GPIOC_MASK;
  GPIOC->PER |= GPIO_PER_PE(0x0001); // Set GPIOC0 as peripheral
  GPIOC->PER |= GPIO_PER_PE(0x0002); // Set GPIOC1 as peripheral
  SIM->GPSCL &= ~SIM_GPSCL_C0_MASK;  // GPIOC0 is EXTAL
  
  
  OCCS->OSCTL1 &= ~(OCCS_OSCTL1_ROPD_MASK|OCCS_OSCTL1_ROSB_MASK); // Enable 8MHz RC
  
  OCCS->OSCTL2 &= ~OCCS_OSCTL2_COPD_MASK;     // Power up crystal oscillator
  OCCS->OSCTL1 |=  OCCS_OSCTL1_COHL_MASK;	  // Low power mode 
  OCCS->OSCTL1 &= ~OCCS_OSCTL1_EXT_SEL_MASK;  // Select crystal oscillator as external input
  OCCS->OSCTL1 &= ~OCCS_OSCTL1_CLK_MODE_MASK; // Enable crystal oscillator
  
  while((OCCS->STAT & OCCS_STAT_OSC_OK_MASK) == 0);
  
 
  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK; 
  OCCS->CTRL |= OCCS_CTRL_PRECS(1); // PRECS=01 to select crystal 8MHz clock source
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  OCCS->CTRL &= ~OCCS_CTRL_PLLPD_MASK; // Power up PLL
  OCCS->CTRL |= OCCS_CTRL_LCKON_MASK; // Enable LCKON
  OCCS->DIVBY = 0x2031U; // Output of PLL is 400MHz, no postscaler
  while((OCCS->STAT & OCCS_STAT_LCK0_MASK) == 0);
  while((OCCS->STAT & OCCS_STAT_LCK1_MASK) == 0);
  OCCS->CTRL |= OCCS_CTRL_ZSRC_MASK; // Select PLL/2 as sys_clk_x2 to SIM 
  OCCS->PROT = 0x00; // No protection
  
#elif CLOCK_CONFIG == RC8M_PLL
  /* 8M clock can be from 8M IRC or 48M IRC depending on IFR */
  OCCS->OSCTL1 &= ~(OCCS_OSCTL1_ROPD_MASK|OCCS_OSCTL1_ROSB_MASK); // Enable 8MHz RC
  SIM->MISC0 |= SIM_MISC0_IRC48M_EN_MASK; // Enable 48MHz RC
  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK; 
  OCCS->CTRL |= OCCS_CTRL_PRECS(0); // PRECS=00 to select internal 8MHz clock source
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  OCCS->DIVBY = 0x2031U; // Output of PLL is 400MHz, no postscaler
  OCCS->CTRL &= ~OCCS_CTRL_PLLPD_MASK; // Power up PLL
  OCCS->CTRL |= OCCS_CTRL_LCKON_MASK; // Enable LCKON
  while((OCCS->STAT & OCCS_STAT_LCK0_MASK) == 0);
  while((OCCS->STAT & OCCS_STAT_LCK1_MASK) == 0);
  OCCS->CTRL |= OCCS_CTRL_ZSRC_MASK; // Select PLL/2 as sys_clk_x2 to SIM
  OCCS->PROT = 0x00; // No protection
#elif CLOCK_CONFIG == RC8M_PLL_CRYSTAL_FOR_FLEXCAN
  SIM->PCE0 |= SIM_PCE0_GPIOC_MASK;
  GPIOC->PER |= GPIO_PER_PE(0x0001); // Set GPIOC0 as peripheral
  GPIOC->PER |= GPIO_PER_PE(0x0002); // Set GPIOC1 as peripheral
  SIM->GPSCL &= ~SIM_GPSCL_C0_MASK;  // GPIOC0 is EXTAL
  SIM->PCE3 |= SIM_PCE3_USB_MASK;
  
  OCCS->OSCTL1 &= ~(OCCS_OSCTL1_ROPD_MASK|OCCS_OSCTL1_ROSB_MASK); // Enable 8MHz RC
  
  OCCS->OSCTL2 &= ~OCCS_OSCTL2_COPD_MASK;     // Power up crystal oscillator
  OCCS->OSCTL1 |=  OCCS_OSCTL1_COHL_MASK;	  // Low power mode 
  OCCS->OSCTL1 &= ~OCCS_OSCTL1_EXT_SEL_MASK;  // Select crystal oscillator as external input
  OCCS->OSCTL1 &= ~OCCS_OSCTL1_CLK_MODE_MASK; // Enable crystal oscillator
  
  while((OCCS->STAT & OCCS_STAT_OSC_OK_MASK) == 0);
  
 
  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK; 
  OCCS->CTRL |= OCCS_CTRL_PRECS(0); // PRECS=00 to select internal 8MHz clock source
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  OCCS->CTRL &= ~OCCS_CTRL_PLLPD_MASK; // Power up PLL
  OCCS->CTRL |= OCCS_CTRL_LCKON_MASK; // Enable LCKON
  OCCS->DIVBY = 0x2031U; // Output of PLL is 400MHz, no postscaler
  while((OCCS->STAT & OCCS_STAT_LCK0_MASK) == 0);
  while((OCCS->STAT & OCCS_STAT_LCK1_MASK) == 0);
  OCCS->CTRL |= OCCS_CTRL_ZSRC_MASK; // Select PLL/2 as sys_clk_x2 to SIM 
  OCCS->PROT = 0x00; // No protection
#endif
  
  asm {
    swap shadows
    nop
    nop
    move.l #0, R0
    move.l #0, R1
    move.l #0, R2
    move.l #0, R3
    move.l #0, R4
    move.l #0, R5
    move.l #0, N
    moveu.w #65535, N3
    moveu.w #65535, M01
    nop
    nop
    swap shadows
  }
  SIM->PCE3 |= SIM_PCE3_USB_MASK; // To avoid a debugger issue
  asm(JMP init_56800_);                /* Jump to C startup code */
}



/* END Cpu. */

