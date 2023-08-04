/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "derivative.h"
#include "Cpu.h"

#define CRYSTAL_PLL 0x01 // 8MHz crystal with PLL, system clock is 100MHz
#define RC8M_PLL    0x02 // 8MHz internal clock with PLL, system clock is 100MHz
#define RC8M        0x03 // 8MHz internal clock without PLL, system clock is 4MHz 

#define CLOCK_CONFIG        RC8M_PLL
#define STARTUP_FAST_MODE   0x1 //enable this function when you want to enter fast operation mode
#define ENTER_BOOTLOADER    0x0 //enable this function when you want to run ROM bootloader and FOPT[7:6] is not 11b 

#define BOOTLOADER_ENTRY_POINT 0x78147

/* Global variables */
volatile int16_t SR_lock = 0U; /* Lock */
volatile int16_t SR_reg; /* Current value of the SR register */
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
void Cpu_Interrupt(void) {

  asm(DEBUGHLT);
  /* Halt the core and placing it in the debug processing state. */
  /* Here should be two NOPs if DEBUGHLT is removed.
  asm(nop);
  asm(nop); */
}

extern void init_56800_(void); /* Forward declaration of external startup function declared in startup file */

/*** !!! Here you can place your own code using property "User data declarations" on the build options tab. !!! ***/

void _EntryPoint(void) 
{
/* Disable constant to array reallocation compiler optimization before RAM is filled in startup */
#pragma constarray off
	
#if ENTER_BOOTLOADER
  /*if((SIM->SCR0 & 0x1) == 0) // flag to identify if it's the first time to enter ROM
  {
	if((SIM->RSTAT&SIM_RSTAT_BOOT_MODE_STATUS_MASK) != SIM_RSTAT_BOOT_MODE_STATUS_MASK) // don't run back ROM again if it just come out from ROM
	{
		SIM->BOOT_MODE_OVERRIDE |= SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK; //enable boot from ROM
		SIM->SCR0 = 0x1; 
		/* SIM_CTRL: SWRst=1 */
		//SIM->CTRL |= SIM_CTRL_SWRST_MASK; /* run ROM bootloader via software reset */
	/*}
  }
  else
  {
	SIM->BOOT_MODE_OVERRIDE &= ~SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK; //disable boot from ROM
	SIM->SCR0 = 0;
  }*/
	if((SIM->SCR0 & 0x1) == 0) // flag to identify if it's the first time to enter ROM
	{
		SIM->SCR0 = 0x1;
		asm(jmp BOOTLOADER_ENTRY_POINT);
	}
	else
	{
		SIM->SCR0 = 0;
	}
	
#endif
  
#if STARTUP_FAST_MODE
	if((SIM->MISC0 & SIM_MISC0_MODE_STAT_MASK) == 0)/* Do not switch system to Fast mode if it is already running in it */
	{
		/* SIM_MISC0: FAST_MODE=1 */
		SIM->MISC0 |= SIM_MISC0_FAST_MODE_MASK; /* Prepare fast mode */
		SIM->SCR0 = 0x1; // flag to indicate a operation mode change reset
		/* SIM_CTRL: SWRst=1 */
		SIM->CTRL |= SIM_CTRL_SWRST_MASK; /* Switch to fast mode via software reset */
	}
	else SIM->SCR0 = 0;
#else
	if ((SIM->MISC0 & SIM_MISC0_MODE_STAT_MASK) != 0U)  /* Do not switch system to Normal mode if it is already running in it */
	{
		/* SIM_MISC0: FAST_MODE=0 */
	    SIM->MISC0 &= ~SIM_MISC0_FAST_MODE_MASK; /* Prepare normal mode */
	    SIM->SCR0 = 0x1; // flag to indicate a operation mode change reset
	    /* SIM_CTRL: SWRst=1 */
	    SIM->CTRL |= SIM_CTRL_SWRST_MASK; /* Switch to normal mode via software reset */
	}
	else SIM->SCR0 = 0;
#endif

  /* Disable COP */
  COP->CTRL &= ~COP_CTRL_CEN_MASK;  // Disable watchdog
  __DI(); // Disable interrupts of level 0,1,2

  /* System clock initialization */
#if CLOCK_CONFIG == CRYSTAL_PLL
  /* use external crystal clock for PLL */ 
  SIM->PCE0 |= SIM_PCE0_GPIOC_MASK; // enable GPIOC 
  GPIOC->PER |= GPIO_PER_PE(0x1); // Set GPIOC0 as peripheral
  GPIOC->PER |= GPIO_PER_PE(0x2); // Set GPIOC1 as peripheral
  SIM->GPSCL &= ~SIM_GPSCL_C0_MASK;  // GPIOC0 is EXTAL

  OCCS->OSCTL1 &= ~OCCS_OSCTL1_EXT_SEL_MASK;  // Select crystal oscillator output as external input clock
  OCCS->OSCTL1 &= ~OCCS_OSCTL1_CLK_MODE_MASK; // Enable the output of crystal oscillator
  OCCS->OSCTL1 |= OCCS_OSCTL1_COHL_MASK;      // LCP mode 
  OCCS->OSCTL2 &= ~OCCS_OSCTL2_COPD_MASK;     // Power up crystal oscillator

  // Wait for crystal oscillator to get stable
  while ((OCCS->STAT & OCCS_STAT_OSC_OK_MASK)== 0) {}

  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK;
  OCCS->CTRL |= OCCS_CTRL_PRECS(1); // PRECS=01 to select crystal 8MHz clock source for PLL
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  if((SIM->MISC0 & SIM_MISC0_MODE_STAT_MASK) == 0)  OCCS->DIVBY = 0x2171U; // Output of PLL is 400MHz, SYS_CLK = 50MHz in normal mode
  else OCCS->DIVBY = 0x2071U; // Output of PLL is 400MHz, SYS_CLK = 100MHz in fast mode
  OCCS->CTRL &= ~OCCS_CTRL_PLLPD_MASK; // Power up PLL
  OCCS->CTRL |= OCCS_CTRL_LCKON_MASK;  // Enable LCKON

  while ((OCCS ->STAT & OCCS_STAT_LCK0_MASK)== 0) {}
  while ((OCCS ->STAT & OCCS_STAT_LCK1_MASK)== 0) {}
  OCCS->CTRL |= OCCS_CTRL_ZSRC_MASK; // Select PLL/2 as sys_clk_x2 to SIM 
  OCCS->PROT = 0x00; // No protection

#elif CLOCK_CONFIG == RC8M_PLL
  /* use internal 8M clock for PLL */
  OCCS->OSCTL1 &= ~(OCCS_OSCTL1_ROPD_MASK|OCCS_OSCTL1_ROSB_MASK); // enable 8M RC
  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK;
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  if((SIM->MISC0 & SIM_MISC0_MODE_STAT_MASK) == 0)  OCCS->DIVBY = 0x2171U; // Output of PLL is 400MHz, SYS_CLK = 50MHz in normal mode
  else OCCS->DIVBY = 0x2071U; // Output of PLL is 400MHz, SYS_CLK = 100MHz in fast mode
  OCCS->CTRL &= ~OCCS_CTRL_PLLPD_MASK; // Power up PLL
  OCCS->CTRL |= OCCS_CTRL_LCKON_MASK;  // Enable LCKON
  while ((OCCS->STAT & OCCS_STAT_LCK0_MASK) == 0) {}
  while ((OCCS->STAT & OCCS_STAT_LCK1_MASK) == 0) {}
  OCCS->CTRL |= OCCS_CTRL_ZSRC_MASK; // Select PLL/2 as sys_clk_x2 to SIM
  OCCS->PROT = 0x00; // No protection

#elif CLOCK_CONFIG == RC8M
  /* use RC8M for SIM, no PLL, it's the reset configuration  */
  OCCS->CTRL &= ~OCCS_CTRL_PRECS_MASK;
  OCCS->CTRL |= OCCS_CTRL_PRECS(0); // PRECS=00b to select 8MHz clock source 
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  asm(NOP);
  OCCS->CTRL &= ~OCCS_CTRL_ZSRC_MASK; // Select MSTR_OSC as sys_clk_x2 to SIM
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

  asm(JMP init_56800_);
  /* Jump to C startup code */
}

/* END Cpu. */

