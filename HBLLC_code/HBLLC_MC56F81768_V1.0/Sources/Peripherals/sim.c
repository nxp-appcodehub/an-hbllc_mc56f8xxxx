/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * 	Clock gate configuration in run mode
 * 
 *  Use the MACRO in sim.h to enable peripheral clock gate: CLOCKGATE_ENABLE(module_name) 
 *  Use the MACRO in sim.h to disable peripheral clock gate: CLOCKGATE_DISABLE(module_name) 
 *  
 *  "module_name" is the peripheral whose clock gate needs to be enabled or disabled in run mode, choose from the MACROs in "Module names"
 *   
 *  E.g.
 *  	CLOCKGATE_ENABLE(GPIO_B);
 *  
 *  This is to enable the clock gate of GPIOB in run mode. 	
 *  
 * */


/*
 * 	Clock gate configuration in stop mode
 * 
 *  Use the MACRO in sim.h to enable peripheral clock gate: CLOCKGATE_STOPMODE_ENABLE(module_name) 
 *  Use the MACRO in sim.h to disable peripheral clock gate: CLOCKGATE_STOPMODE_DISABLE(module_name) 
 *  
 *  "module_name" is the peripheral whose clock gate needs to be enabled or disabled in run mode, choose from the MACROs in "Module names"
 *   
 *  E.g.
 *  	CLOCKGATE_STOPMODE_ENABLE(GPIO_B);
 *  
 *  This is to enable the clock gate of GPIOB in stop mode. 	
 *  
 * */

/*
 * 	Generate SW reset
 * 
 *  Use the MACRO in sim.h to generate SW reset
 *   
 *  E.g.
 *  	SW_RESET(); 	
 *  
 * */

/*
 * 	Force to boot from ROM after SW reset
 * 
 *  Use the MACRO in sim.h to force the chip to boot from ROM after SW reset 
 *   
 *  E.g.
 *  	BOOT_FROM_ROM_AFTER_SW_RESET(); 	
 *  
 * */


/*
 * 	DMA enable/disable configuration in various power modes
 * 
 *  Use the inline functions in sim.h to enable/disable DMA.
 *  Note that DMA is enabled in all power modes by default after reset.
 *   
 *  E.g.
 *  	DMA_enable_in_run(); 	
 *  	
 *  This is to enable DMA in run mode only.	
 *  
 * */


/*
 * 	ADC and QTimerA mux selection configuration
 * 
 *  Use the MACROS in sim.h to select one from the multiplexed ADC and QTimerA for XBARA input.
 *  
 *   
 *  E.g.
 *  	AN0_LIMIT_or_TIMERA0_OUT_ADC_EN(); 	
 *  	
 *  This is to choose AN0_LIMIT signal for XBARA input.	
 *  
 * */
