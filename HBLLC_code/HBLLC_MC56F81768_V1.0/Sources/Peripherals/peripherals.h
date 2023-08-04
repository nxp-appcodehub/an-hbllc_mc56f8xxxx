/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef PERIPHEARLS_H_
#define PERIPHEARLS_H_

#include "derivative.h"
#include "sim.h"
//#include "intc.h"
#include "xbar.h"
/******************************************
 * Definitions
 ******************************************/
#define MASK_BIT15 0x8000
#define MASK_BIT14 0x4000
#define MASK_BIT13 0x2000
#define MASK_BIT12 0x1000
#define MASK_BIT11 0x0800
#define MASK_BIT10 0x0400
#define MASK_BIT9 0x0200
#define MASK_BIT8 0x0100
#define MASK_BIT7 0x0080
#define MASK_BIT6 0x0040
#define MASK_BIT5 0x0020
#define MASK_BIT4 0x0010
#define MASK_BIT3 0x0008
#define MASK_BIT2 0x0004
#define MASK_BIT1 0x0002
#define MASK_BIT0 0x0001

#define LOAD_ENABLE       GPIOC->DR &= ~MASK_BIT4
#define LOAD_DISABLE      GPIOC->DR |= MASK_BIT4

#define ADC_RDY_08        ADC->RDY & 0x101
#define ADC_RDY_19        ADC->RDY & 0x202

#define PWMA_RUN          PWM->MCTRL |= PWM_MCTRL_RUN(7)
#define PWMA_STOP         PWM->MCTRL &= ~PWM_MCTRL_RUN(7)

#define HW_IPRIM_OVER()            PWM->FAULT[0].FSTS & PWM_FSTS_FFLAG(1)
#define HW_FAULT_FLAG_CLEAR()    PWM->FAULT[0].FSTS |= PWM_FSTS_FFLAG(1)

#define PIT_RUN           PIT0->CTRL |= PIT_CTRL_CNT_EN_MASK
#define PIT_STOP          PIT0->CTRL &= ~PIT_CTRL_CNT_EN_MASK

#define ENABLE_PWM_OUTPUT       { EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
	                              EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}
#define DISABLE_PWM_OUTPUT      { EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
	                              EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 &= ~EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK;}
#define ENABLE_SR     { EVTG->EVTG_INST[0].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
	                    EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;}      
#define DISABLE_SR    { EVTG->EVTG_INST[0].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;\
	                    EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 &= ~EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK;} 

#define TP30_UP      GPIOE->DR |= 0x40
#define TP30_DOWN    GPIOE->DR &= ~0x40
#define TP31_UP      GPIOE->DR |= 0x80
#define TP31_DOWN    GPIOE->DR &= ~0x80

#define LLC_BOARD_LED3_ON   GPIOC->DR |= 0x01
#define LLC_BOARD_LED3_OFF  GPIOC->DR &= ~0x01
#define LLC_BOARD_LED4_ON   GPIOF->DR |= 0x40
#define LLC_BOARD_LED4_OFF  GPIOF->DR &= ~0x40
/******************************************
 * API
 ******************************************/
void Pin_init(void);
void XBAR_init(void);
void eFlexPWMA_init(void);
void sci0_init(void);
void sci1_init(void);
void ADC_init(void);
void CMP_init(void);
void PIT0_init(void);
extern void Peripherals_init(void);

#endif /* PERIPHEARLS_H_ */
