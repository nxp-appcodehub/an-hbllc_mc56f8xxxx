/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * peripheral.c
 *
 *  Created on: Aug 21, 2019
 *      Author: nxa16823
 */
#include "peripheral.h"

void Pin_init(void)
{
	/* pin clock enable */
	SIM->PCE0 |= (SIM_PCE0_GPIOA_MASK | SIM_PCE0_GPIOB_MASK | SIM_PCE0_GPIOC_MASK |
				SIM_PCE0_GPIOE_MASK | SIM_PCE0_GPIOF_MASK);
	
	/* enable GPIOC1 for USER_LED_2, HVP D1 */
	GPIOC->PER &= ~MASK_BIT1;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT1;   // Set to output mode
	GPIOC->DR &= ~MASK_BIT1;   // Output 0
	/* enable GPIOC0 for User_LED_1, D3 */
	GPIOC->PER &= ~MASK_BIT0;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT0;   // Set to output mode
	GPIOC->DR &= ~MASK_BIT0;   // Output 0
	/* enable GPIOF6 for User_LED_2, D4 */
	GPIOF->PER &= ~MASK_BIT6;  // Set to GPIO mode
	GPIOF->DDR |= MASK_BIT6;   // Set to output mode
	GPIOF->DR &= ~MASK_BIT6;   // Output 0
	
	/* enable GPIOA4/A5 for Iout and Iprim sampling */
	GPIOA->PER |= MASK_BIT4|MASK_BIT1|MASK_BIT5;  // set to peripheral mode	
	/* enable GPIOB1/2 for Vprim and Vout sampling */
	GPIOB->PER |= MASK_BIT1|MASK_BIT2;  // set to peripheral mode
	/*enable GPIOA0 for I_prim_protect */
	GPIOA->PER |= MASK_BIT0;
	/*enable GPIOA7/b7 for transformer voltage transition detect to control off timing of SR */
	GPIOA->PER |= MASK_BIT7;
	GPIOB->PER |= MASK_BIT7;
	
	/* enable GPIOF2 for XBOUT6(PWM_L), GPIOF3 for XBOUT7(PWM_H) */
	GPIOF->PER |= MASK_BIT2|MASK_BIT3;  // set to peripheral mode
	SIM->GPSFL = (SIM->GPSFL & ~SIM_GPSFL_F2_MASK)|SIM_GPSFL_F2(1); // Set as XBOUT6
	SIM->GPSFL = (SIM->GPSFL & ~SIM_GPSFL_F3_MASK)|SIM_GPSFL_F3(1); // Set as XBOUT7
	/* enable GPIOF4 for XBOUT8(PWM_SR_1), GPIOF5 for XBOUT9(PWM_SR_2) */
	GPIOF->PER |= MASK_BIT4|MASK_BIT5;  // set to peripheral mode
	SIM->GPSFL = (SIM->GPSFL & ~SIM_GPSFL_F4_MASK)|SIM_GPSFL_F4(1); // Set as XBOUT8
	SIM->GPSFL = (SIM->GPSFL & ~SIM_GPSFL_F5_MASK)|SIM_GPSFL_F5(1); // Set as XBOUT9
	
	/* enable GPIOC4 for load connection control */
	GPIOC->PER &= ~MASK_BIT4;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT4;   // Set to output mode
	LOAD_DISABLE;   // Output 0
	
	/* Configure SCI1 pins for freeMaster communication */
	GPIOC->PER |= GPIO_PER_PE(MASK_BIT11|MASK_BIT12); // Set to peripheral mode
	SIM->GPSCH = (SIM->GPSCH & ~SIM_GPSCH_C11_MASK)|SIM_GPSCH_C11(2); // GPIOC11 Set as TXD1
	SIM->GPSCH = (SIM->GPSCH & ~SIM_GPSCH_C12_MASK)|SIM_GPSCH_C12(2); // GPIOC12 Set as RXD1
	#if COMM_WITH_PFC_ENABLE
	/* Configure SCI0 pins for power board communication */
	GPIOC->PER |= GPIO_PER_PE(MASK_BIT2|MASK_BIT3); // Set to peripheral mode
	SIM->GPSCL = (SIM->GPSCL & ~SIM_GPSCL_C2_MASK)|SIM_GPSCL_C2(0); // GPIOC2 Set as RXD0
	SIM->GPSCL = (SIM->GPSCL & ~SIM_GPSCL_C3_MASK)|SIM_GPSCL_C3(2); // GPIOC3 Set as TXD0
	#endif
	
	/*************** for test ****************/
	/* TP30, GPIOE6 */
	GPIOE->PER &= ~MASK_BIT6;  // Set to GPIO mode
	GPIOE->DDR |= MASK_BIT6;   // Set to output mode
	TP30_DOWN;   // Output 0
	/* TP31, GPIOE7 */
	GPIOE->PER &= ~MASK_BIT7;  // Set to GPIO mode
	GPIOE->DDR |= MASK_BIT7;   // Set to output mode
	TP31_DOWN;   // Output 0
	/* enable GPIOE0/1 for PWM output */
	//GPIOE->PER |= GPIO_PER_PE(MASK_BIT4|MASK_BIT5); // Set to peripheral mode
}

void XBAR_init(void)
{
	/**************** ADC trigger ********************/
	SIM->PWM_SEL |= SIM_PWM_SEL_XBAR_IN28_MASK; // select pwma0 trigger0 as xbar_in28
	XBARA->SEL6 = (XBARA->SEL6 & ~XBARA_SEL6_SEL12_MASK) | XBARA_SEL6_SEL12(0x1C); // pwma0 trigger0 as adc trigger

	/**************** PWM fault **********************/
	XBARA->SEL14 = (XBARA->SEL14 & ~XBARA_SEL14_SEL29_MASK) | XBARA_SEL14_SEL29(0xC); // CMPA_OUT as pwma fault0
	SIM->IPS0 |= SIM_IPS0_PWMAF0_MASK; // pwm fault0 comes form xbar  
	
    /**************** SR pwm output ******************/
	SIM->PWM_SEL &= ~(SIM_PWM_SEL_XBAR_IN22_MASK | SIM_PWM_SEL_XBAR_IN23_MASK); // Select PWMA1 trigger as xbar_in22/23
	XBARA->SEL23 = (XBARA->SEL23 & ~XBARA_SEL23_SEL47_MASK) | XBARA_SEL23_SEL47(0xF); // CMPD_OUT as EVTG0_A
	XBARA->SEL24 = 0xC17; // PWM1B as EVTG0_B, CMPA_OUT as EVTG0_C
	/* EVTG0_A: PWM_SR1 = CMPD_OUT & PWM1B & CMPA_OUT */
	EVTG->EVTG_INST[0].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(1)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1); 
	EVTG->EVTG_INST[0].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[0].EVTG_AOI0_FILT = 0; // disable filter 
	
	XBARA->SEL25 = (XBARA->SEL25 & ~XBARA_SEL25_SEL51_MASK) | XBARA_SEL25_SEL51(0xD); // CMPB_OUT as EVTG1_A
	XBARA->SEL26 = 0xC16; // PWM1A as EVTG1_B, CMPA_OUT as EVTG1_C
	/* EVTG1_A: PWM_SR2 = CMPB_OUT & PWM1A & CMPA_OUT */
	EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(1)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1);
	EVTG->EVTG_INST[1].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[1].EVTG_AOI0_FILT = 0; // disable filter
	XBARA->SEL4 = 0x2928; // EVTG0_A as XBOUT8, EVTG1_A as XBOUT9
	
	/***************** HB pwm output *******************/
	SIM->PWM_SEL &= ~(SIM_PWM_SEL_XBAR_IN24_MASK | SIM_PWM_SEL_XBAR_IN25_MASK); // Select PWMA2 trigger as xbar_in24/25
	XBARA->SEL27 = (XBARA->SEL27 & ~XBARA_SEL27_SEL55_MASK) | XBARA_SEL27_SEL55(0x18); //PWM2A as EVTG2_A
	XBARA->SEL28 = 0XC19; // PWM2B as EVTG2_B, CMPA_OUT as EVTG2_C
	/* EVTG2_A: PWM_L = PWM2B & CMPA_OUT */
	EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(3)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1);
    /* EVTG2_B: PWM_H = PWM2A & CMPA_OUT */
	EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_AC(1)|EVTG_EVTG_AOI1_BFT01_PT0_BC(3)|EVTG_EVTG_AOI1_BFT01_PT0_CC(1);
	EVTG->EVTG_INST[2].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[2].EVTG_AOI0_FILT = 0; // disable filter
    XBARA->SEL3 = 0x302A; // EVTG2_A as XBOUT6, EVTG2_B as XBOUT7
}

void eFlexPWMA_init(void)
{
	/* enable PWMA sm0&1&2 clock */
	SIM->PCE3 |= SIM_PCE3_PWMACH0_MASK|SIM_PCE3_PWMACH1_MASK|SIM_PCE3_PWMACH2_MASK;
	
	/*********** configure sm0 to generate ADC trigger and control interrupt ***********/
	PWMA->SM0CTRL |= PWM_SM0CTRL_FULL_MASK; // full cycle reload
	PWMA->SM0CTRL &= ~PWM_SM0CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWMA->SM0CTRL &= ~PWM_SM0CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWMA->SM0CTRL2 = (PWMA->SM0CTRL2 & ~PWM_SM0CTRL2_CLK_SEL_MASK)|PWM_SM0CTRL2_CLK_SEL(0); // use IPBus clock
	PWMA->SM0CTRL2 = (PWMA->SM0CTRL2 & ~PWM_SM0CTRL2_INIT_SEL_MASK)|PWM_SM0CTRL2_INIT_SEL(0); // use local sync for SM0
	PWMA->SM0CTRL2 &= ~PWM_SM0CTRL2_RELOAD_SEL_MASK; // use local reload signal

	/* trigger configuration */
	PWMA->SM0TCTRL &= ~PWM_SM0TCTRL_PWAOT0_MASK; // pwm out_trig is routed to the PWM_TRIG0 port
	PWMA->SM0TCTRL = (PWMA->SM0TCTRL & ~PWM_SM0TCTRL_OUT_TRIG_EN_MASK)| PWM_SM0TCTRL_OUT_TRIG_EN(MASK_BIT0); // val0 trigger is used for ADC trigger
		
	PWMA->SM0INIT  = 0; 
	PWMA->SM0VAL1  = 1199; 
	PWMA->SM0VAL0 = 0;
	
	/*********** configure sm1 to generate lengthened pwm driver for SR ***********/
	PWMA->SM1CTRL |= PWM_SM1CTRL_FULL_MASK; // full cycle reload
	PWMA->SM1CTRL &= ~PWM_SM1CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWMA->SM1CTRL &= ~PWM_SM1CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWMA->SM1CTRL2 = (PWMA->SM1CTRL2 & ~PWM_SM1CTRL2_CLK_SEL_MASK)|PWM_SM1CTRL2_CLK_SEL(2); // SM0's clock is used
	PWMA->SM1CTRL2 = (PWMA->SM1CTRL2 & ~PWM_SM1CTRL2_INIT_SEL_MASK)|PWM_SM1CTRL2_INIT_SEL(0); // use local sync
	PWMA->SM1CTRL2 |= PWM_SM1CTRL2_RELOAD_SEL_MASK; // reload source: master reload
	PWMA->SM1CTRL2 |= PWM_SM1CTRL2_INDEP_MASK; // independent operation
	PWMA->SM1CTRL2 |= PWM_SM1CTRL2_FRCEN_MASK; // enable force
	PWMA->SM1CTRL2 = (PWMA->SM1CTRL2 & ~ PWM_SM1CTRL2_FORCE_SEL_MASK) | PWM_SM1CTRL2_FORCE_SEL(3); //master reload is the force source
	PWMA->SM1CTRL2 |= PWM_SM1CTRL2_PWM45_INIT_MASK; // PWMB init high
	
	/* trigger configuration */
	PWMA->SM1TCTRL |= (PWM_SM1TCTRL_PWAOT0_MASK | PWM_SM1TCTRL_PWBOT1_MASK); // pwm output is routed to the PWM_TRIGx port
	PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(2)|PWM_OUTEN_PWMB_EN(2)); // when route to XBAR, output must be enabled in nevis3
	
	/* output configuration */
	PWMA->SM1OCTRL &= ~0x73F; // fault state: 0 and not invert
	PWMA->SM1DISMAP0 = (PWM_SM1DISMAP0_DIS0A(1) | PWM_SM1DISMAP0_DIS0B(1));
	PWMA->SM1DISMAP1 = 0;
	DISABLE_SR;
	
	PWMA->SM1INIT  = 0; 
	PWMA->SM1VAL1  = 1199; 
	PWMA->SM1VAL2  = 0;
	PWMA->SM1VAL3  = 0;
	PWMA->SM1VAL4  = 0;
	PWMA->SM1VAL5  = 0;
		
	/*********** configure sm2 to generate pwm driver for HB ***********/
	PWMA->SM2CTRL |= PWM_SM2CTRL_FULL_MASK; // full cycle reload
	PWMA->SM2CTRL &= ~PWM_SM2CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWMA->SM2CTRL &= ~PWM_SM2CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWMA->SM2CTRL2 = (PWMA->SM2CTRL2 & ~PWM_SM2CTRL2_CLK_SEL_MASK)|PWM_SM2CTRL2_CLK_SEL(2); // SM0's clock is used
	PWMA->SM2CTRL2 = (PWMA->SM2CTRL2 & ~PWM_SM2CTRL2_INIT_SEL_MASK)|PWM_SM2CTRL2_INIT_SEL(0); // use local sync
	PWMA->SM2CTRL2 |= PWM_SM2CTRL2_RELOAD_SEL_MASK; // reload source: master reload
	PWMA->SM2CTRL2 |= PWM_SM2CTRL2_INDEP_MASK; // independent operation
	PWMA->SM2CTRL2 |= PWM_SM2CTRL2_FRCEN_MASK; // enable force
	PWMA->SM2CTRL2 = (PWMA->SM2CTRL2 & ~ PWM_SM2CTRL2_FORCE_SEL_MASK) | PWM_SM2CTRL2_FORCE_SEL(3); //master reload is the force source
	PWMA->SM2CTRL2 |= PWM_SM2CTRL2_PWM45_INIT_MASK; // PWMB init high
	
	/* trigger configuration */
	PWMA->SM2TCTRL |= (PWM_SM2TCTRL_PWAOT0_MASK | PWM_SM2TCTRL_PWBOT1_MASK); // pwm output is routed to the PWM_TRIGx port
	PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(4)|PWM_OUTEN_PWMB_EN(4)); // when route to XBAR, output must be enabled in nevis3
	
	/* output configuration */
	PWMA->SM2OCTRL &= ~0x73F; // fault state: 0 and not invert
	PWMA->SM2DISMAP0 = (PWM_SM2DISMAP0_DIS0A(1) | PWM_SM2DISMAP0_DIS0B(1));
	PWMA->SM2DISMAP1 = 0;
    DISABLE_PWM_OUTPUT;
	
	PWMA->SM2INIT  = 0; 
	PWMA->SM2VAL1  = 1199; 
	PWMA->SM2VAL2  = 0;
	PWMA->SM2VAL3  = 0;
	PWMA->SM2VAL4  = 0;
	PWMA->SM2VAL5  = 0;
	PWMA->SM2FRCTRL |= PWM_SM0FRCTRL_FRAC_PU_MASK;  //power up fractional delay circuit
	PWMA->SM2FRCTRL |= PWM_SM1FRCTRL_FRAC23_EN_MASK|PWM_SM1FRCTRL_FRAC45_EN_MASK;  //fractional cycle placement enable
	PWMA->SM2FRACVAL2 = 0;
	PWMA->SM2FRACVAL3 = 0;
	PWMA->SM2FRACVAL4 = 0;
	PWMA->SM2FRACVAL5 = 0;
	PWMA->MCTRL2 = (PWMA->MCTRL2 & ~PWM_MCTRL2_MONPLL_MASK)|PWM_MCTRL2_MONPLL(1); // monitor PLL operation
	
	/*********** fault configuration **************/
	PWMA->FCTRL0 &= ~PWM_FCTRL0_FLVL_MASK; // A logic 0 on fault0 indicates a fault condition
	PWMA->FCTRL0 = (PWMA->FCTRL0 & ~PWM_FCTRL0_FAUTO_MASK)|PWM_FCTRL0_FAUTO(0); // Manual clearing for fault0
	PWMA->FCTRL0 = (PWMA->FCTRL0 & ~PWM_FCTRL0_FSAFE_MASK)|PWM_FCTRL0_FSAFE(1); // Safe mode for fault0
	PWMA->FSTS0 = (PWMA->FSTS0 & ~PWM_FSTS0_FFULL_MASK)|PWM_FSTS0_FFULL(1); // Full cycle recovery for fault0
	PWMA->FCTRL20 = (PWMA->FCTRL20 & ~PWM_FCTRL20_NOCOMB_MASK)|PWM_FCTRL20_NOCOMB(0); // enable combinational path for fault
	PWMA->FFILT0 = PWM_FFILT0_FILT_CNT(1)|PWM_FFILT0_FILT_PER(1);
		
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(7); // clear LDOK of sm0-sm2
	PWMA->MCTRL |= PWM_MCTRL_LDOK(7); // set LDOK of sm0-sm2
	
	/* enable sm0 val0 compare interrupt for control interrupt */
	PWMA->SM0STS |= PWM_SM0STS_CMPF(1); // clear sm0 val0 compare flag
	PWMA->SM0INTEN |= PWM_SM0INTEN_CMPIE(1); // enable sm0 val0 compare interrupt
	INTC->IPR9 = (INTC->IPR9 & ~INTC_IPR9_PWMA_CMP0_MASK) | INTC_IPR9_PWMA_CMP0(2);  // set PWMA_CMP0 irq priority level 1
	/* enable PWM fault interrupt */
	/*PWMA->FSTS0 |= PWM_FSTS0_FFLAG_MASK; // Clear fault flags of fault0~3
	PWMA->FCTRL0 |= PWM_FCTRL0_FIE(1);  // enable iprim fault interrupt
	INTC->IPR8 = (INTC->IPR8 & ~INTC_IPR8_PWMA_FAULT_MASK) | INTC_IPR8_PWMA_FAULT(3);*/  // set PWMA_FAULT irq priority level 2
}

void sci_init(void)
{
	/* sci1 clock enable, for freemaster communication */
	SIM->PCE1 |= SIM_PCE1_SCI1_MASK; // SCI default clock rate = 100MHz
	QSCI1->CTRL1 |= (QSCI_CTRL1_TE_MASK|QSCI_CTRL1_RE_MASK);
	QSCI1->RATE = (((uint16_t)(324)<<3)|4); // 19200 baudrate
	QSCI1->CTRL3 = 0;
		
	#if COMM_WITH_PFC_ENABLE
	/* sci0 clock enable, for communication with PFC stage */
	SIM->PCE1 |= SIM_PCE1_SCI0_MASK;
	QSCI0->CTRL2 |= QSCI_CTRL2_FIFO_EN_MASK; // enable 4-word-deep FIFOs
	QSCI0->CTRL1 |= (QSCI_CTRL1_TE_MASK|QSCI_CTRL1_RE_MASK);
	QSCI0->RATE = (((uint16_t)(2604)<<3)|1); // 2400 baudrate
	QSCI0->CTRL3 = 0;
	#endif
}

void ADC_init(void)
{
	/* enable cyclic ADC clock */
	SIM->PCE2 |= SIM_PCE2_CYCADC_MASK;
		
	ADC->CTRL2 = (ADC->CTRL2 & ~ADC_CTRL2_DIV0_MASK) | ADC_CTRL2_DIV0(3); // ADC CLK = sys_clk/4 = 25MHz
	ADC->CTRL1 = (ADC->CTRL1 & ~ADC_CTRL1_SMODE_MASK) | ADC_CTRL1_SMODE(5); // Triggered parallel
	ADC->CTRL2 |= ADC_CTRL2_SIMULT_MASK; // Simultaneous mode
	/* sample list configuration */
	ADC->SDIS = 0xFCFC; // four channel to be scanned
	ADC->CLIST1 = (ADC->CLIST1 & ~ADC_CLIST1_SAMPLE0_MASK)|ADC_CLIST1_SAMPLE0(0x4); // ANA4->Sample0, Iout 
	ADC->CLIST1 = (ADC->CLIST1 & ~ADC_CLIST1_SAMPLE1_MASK)|ADC_CLIST1_SAMPLE1(0x5); // ANA5->Sample1, Iprim
	ADC->CLIST3 = (ADC->CLIST3 & ~ADC_CLIST3_SAMPLE8_MASK)|ADC_CLIST3_SAMPLE8(0xA); // ANB2->Sample8, Vout
	ADC->CLIST3 = (ADC->CLIST3 & ~ADC_CLIST3_SAMPLE9_MASK)|ADC_CLIST3_SAMPLE9(0x9); // ANB1->Sample9, Vprim
	
	ADC->CTRL1 |= ADC_CTRL1_SYNC0_MASK;  // ADC0 can be triggered by SYNC0 input pulse
	ADC->CTRL1 &= ~ADC_CTRL1_STOP0_MASK; // Normal operation for ADC0
	
	ADC->PWR &= ~(ADC_PWR_PD0_MASK|ADC_PWR_PD1_MASK); // Power up ADC0&1
}

void CMP_init(void)
{
	/* enable both CMP and 8-bit DAC clock of CMPA/B/D */
	SIM->PCE2 |= (SIM_PCE2_CMPD_MASK | SIM_PCE2_CMPB_MASK | SIM_PCE2_CMPA_MASK);
	
	/*CMPA configuration for resonant tank over-current protection */
	CMPA->CR1 |= CMP_CR1_PMODE_MASK; // high speed
	CMPA->DACCR = CMP_DACCR_VOSEL(235); 
	CMPA->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPA->MUXCR = (CMPA->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACA
	CMPA->MUXCR = (CMPA->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(3); // minus input: IN3, ANA0
		
	CMPA->CR0 = (CMPA->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 3 consecutive samples must agree
	CMPA->FPR = (CMPA->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(2); // filter sample period = 2
	CMPA->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
	
	/*CMPB configuration for SR2 off control */
	CMPB->CR1 |= CMP_CR1_PMODE_MASK; // high speed		
	CMPB->DACCR = CMP_DACCR_VOSEL(80); 
	CMPB->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPB->MUXCR = (CMPB->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACB
	CMPB->MUXCR = (CMPB->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(2); // minus input: IN2, ANB7
		
	CMPB->CR0 = (CMPB->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 3 consecutive samples must agree
	CMPB->FPR = (CMPB->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(1); // filter sample period = 1
	CMPB->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
	
	/*CMPD configuration for SR1 off control */
	CMPD->CR1 |= CMP_CR1_PMODE_MASK; // high speed			
	CMPD->DACCR = CMP_DACCR_VOSEL(80); 
	CMPD->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPD->MUXCR = (CMPD->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACD
	CMPD->MUXCR = (CMPD->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(3); // minus input: IN3, ANA7
			
	CMPD->CR0 = (CMPD->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 7 consecutive samples must agree
	CMPD->FPR = (CMPD->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(1); // filter sample period = 3
	CMPD->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
}

void PIT_init(void)
{
	/* enable PIT0 clock */
	SIM->PCE2 |= SIM_PCE2_PIT0_MASK;
		
	PIT0->CTRL &= ~PIT_CTRL_CLKSEL_MASK;  // bus clock
	PIT0->CTRL = (PIT0->CTRL & ~PIT_CTRL_PRESCALER_MASK) | PIT_CTRL_PRESCALER(1); // clock = bus clock/2 = 50M
	PIT0->MOD = 50000;  // interrupt frequency = 1kHz
		
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;
	PIT0->CTRL |= PIT_CTRL_PRIE_MASK;  // enable PIT roll-ver interrupt
	INTC->IPR10 = (INTC->IPR10 & ~INTC_IPR10_PIT0_ROLLOVR_MASK) | INTC_IPR10_PIT0_ROLLOVR(1);  // priority level 0
}

void Peripheral_init(void)
{
	Pin_init();
	CMP_init();
	eFlexPWMA_init();
	sci_init();
	ADC_init();	
	PIT_init();
	XBAR_init();	
}
