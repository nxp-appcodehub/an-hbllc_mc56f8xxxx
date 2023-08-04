/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "peripherals.h"
#include "LLC_statemachine.h"

/*!
 * @brief Initializes the pin functions.
 */
void Pin_init(void)
{
	CLOCKGATE_ENABLE(GPIO_A);
	CLOCKGATE_ENABLE(GPIO_B);
	CLOCKGATE_ENABLE(GPIO_C);
	CLOCKGATE_ENABLE(GPIO_E);
	CLOCKGATE_ENABLE(GPIO_F);
	
	/*************************LED pins configuration************************/
	/* enable GPIOC1 for USER_LED_2, HVP D1 */
	GPIOC->PER &= ~MASK_BIT1;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT1;   // Set to output mode
	GPIOC->DR &= ~MASK_BIT1;   // Output 0
	/* enable GPIOC0 for User_LED_1, D3 */
	GPIOC->PER &= ~MASK_BIT0;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT0;   // Set to output mode
	GPIOC->DR &= ~MASK_BIT0;   // Output 0
	/* enable GPIOF0 for User_LED_2, D4 */
	GPIOF->PER &= ~MASK_BIT0;  // Set to GPIO mode
	GPIOF->DDR |= MASK_BIT0;   // Set to output mode
	GPIOF->DR &= ~MASK_BIT0;   // Output 0
	
	/*************************ADC pins configuration*************************/
	/* enable GPIOA0/A5 for Iout and Iprim sampling */
	GPIOA->PER |= MASK_BIT0|MASK_BIT5;  // set to peripheral mode	
	/* enable GPIOB1/2 for Vprim and Vout sampling */
	GPIOB->PER |= MASK_BIT1|MASK_BIT2;  // set to peripheral mode
	/*enable GPIOB7 for I_prim_protect */
	GPIOB->PER |= MASK_BIT7;
	/*enable GPIOA3/A4 for transformer voltage transition detect to control off timing of SR */
	GPIOA->PER |= MASK_BIT3|MASK_BIT4;
	
	/************************PWM pins configuration**************************/
	/* enable GPIOE1 for XBOUT5(PWM_L), GPIOE0 for XBOUT4(PWM_H) */
	GPIOE->PER |= MASK_BIT0|MASK_BIT1;  // set to peripheral mode
	SIM->GPSEL = (SIM->GPSEL & ~SIM_GPSEL_E0_MASK)|SIM_GPSEL_E0(3); // Set as XBOUT4
	SIM->GPSEL = (SIM->GPSEL & ~SIM_GPSEL_E1_MASK)|SIM_GPSEL_E1(3); // Set as XBOUT5
	/* enable GPIOE3 for XBOUT7(PWM_SR_1), GPIOE2 for XBOUT6(PWM_SR_2) */
	GPIOE->PER |= MASK_BIT2|MASK_BIT3;  // set to peripheral mode
	SIM->GPSEL = (SIM->GPSEL & ~SIM_GPSEL_E2_MASK)|SIM_GPSEL_E2(3); // Set as XBOUT6
	SIM->GPSEL = (SIM->GPSEL & ~SIM_GPSEL_E3_MASK)|SIM_GPSEL_E3(3); // Set as XBOUT7
	
	/*************************load connection pin configuration***************/
	/* enable GPIOC4 for load connection control */
	GPIOC->PER &= ~MASK_BIT4;  // Set to GPIO mode
	GPIOC->DDR |= MASK_BIT4;   // Set to output mode
	LOAD_DISABLE;   // Output 0
	
	/*************************SCI configuration*******************************/
	/*SCI1 pins, HVP cp2102*/
	GPIOC->PER |= GPIO_PER_PE(MASK_BIT11|MASK_BIT12); // Set GPIOC11&12 to peripheral mode
	SIM->GPSCH = (SIM->GPSCH & ~SIM_GPSCH_C11_MASK)|SIM_GPSCH_C11(2); // Set GPIOC11 as TXD1
	SIM->GPSCH = (SIM->GPSCH & ~SIM_GPSCH_C12_MASK)|SIM_GPSCH_C12(2); // Set GPIOC12 as RXD1
	/*SCI0 pins, communicate with primary side*/
#if SCI0_ENABLE
	GPIOC->PER |= GPIO_PER_PE(MASK_BIT2|MASK_BIT3); // Set to peripheral mode
	SIM->GPSCL = (SIM->GPSCL & ~SIM_GPSCL_C2_MASK)|SIM_GPSCL_C2(0); // GPIOC2 Set as TXD0
	SIM->GPSCL = (SIM->GPSCL & ~SIM_GPSCL_C3_MASK)|SIM_GPSCL_C3(2); // GPIOC3 Set as RXD0
#endif
	
	/*************************for test**************************************/
	/* power board TP30, GPIOE6 */
	GPIOE->PER &= ~MASK_BIT6;  // Set to GPIO mode
	GPIOE->DDR |= MASK_BIT6;   // Set to output mode
	TP30_DOWN;   // Output 0
	/* power board TP31, GPIOE7 */
	GPIOE->PER &= ~MASK_BIT7;  // Set to GPIO mode
	GPIOE->DDR |= MASK_BIT7;   // Set to output mode
	TP31_DOWN;   // Output 0
}

/*!
 * @brief Initializes internal signal connection through XBAR.
 */
void XBAR_init(void)
{	
	/**************** ADC trigger ********************/
	XBAR_INIT(in_PWMA0_OUT_TRIG0,out_ADCA_TRIG); // pwma0 trigger0 as adc trigger
	
	/**************** PWM fault **********************/
	XBAR_INIT(in_CMPB_OUT,out_PWMA_FAULT0);	// CMPB_OUT as pwma fault0
	
	/**************** SR pwm output ******************/
	XBAR_INIT(in_CMPA_OUT,out_EVTG0_INA); // CMPA_OUT as EVTG0_A
	XBAR_INIT(in_PWMA1_MUX_TRIG1,out_EVTG0_INB); // PWM1B as EVTG0_B
	XBAR_INIT(in_CMPB_OUT,out_EVTG0_INC); // CMPB_OUT as EVTG0_C
	/* EVTG0_A: PWM_SR1 = CMPA_OUT & PWM1B & CMPB_OUT */
	EVTG->EVTG_INST[0].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(1)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1); 
	EVTG->EVTG_INST[0].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[0].EVTG_AOI0_FILT = 0; // disable filter
	
	XBAR_INIT(in_CMPD_OUT,out_EVTG1_INA); // CMPD_OUT as EVTG1_A
	XBAR_INIT(in_PWMA1_MUX_TRIG0,out_EVTG1_INB); // PWM1A as EVTG1_B
	XBAR_INIT(in_CMPB_OUT,out_EVTG1_INC); // CMPB_OUT as EVTG1_C
	/* EVTG1_A: PWM_SR2 = CMPD_OUT & PWM1A & CMPB_OUT */
	EVTG->EVTG_INST[1].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(1)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1);
	EVTG->EVTG_INST[1].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[1].EVTG_AOI0_FILT = 0; // disable filter
	
	XBAR_INIT(in_EVTG0_OUTA,out_XB_OUT7); // EVTG0_A as PWM_SR_1(XBOUT7)
	XBAR_INIT(in_EVTG1_OUTA,out_XB_OUT6); // EVTG1_A as PWM_SR_2(XBOUT6)
	
	/***************** HB pwm output *******************/
	XBAR_INIT(in_PWMA2_MUX_TRIG0,out_EVTG2_INA); // PWM2A as EVTG2_A
	XBAR_INIT(in_PWMA2_MUX_TRIG1,out_EVTG2_INB); // PWM2B as EVTG2_B
	XBAR_INIT(in_CMPB_OUT,out_EVTG2_INC); // CMPB_OUT as EVTG2_C
	/* EVTG2_A: PWM_L = PWM2B & CMPB_OUT */
	EVTG->EVTG_INST[2].EVTG_AOI0_BFT01 |= EVTG_EVTG_AOI0_BFT01_PT0_AC(3)|EVTG_EVTG_AOI0_BFT01_PT0_BC(1)|EVTG_EVTG_AOI0_BFT01_PT0_CC(1);
	/* EVTG2_B: PWM_H = PWM2A & CMPB_OUT */
	EVTG->EVTG_INST[2].EVTG_AOI1_BFT01 |= EVTG_EVTG_AOI1_BFT01_PT0_AC(1)|EVTG_EVTG_AOI1_BFT01_PT0_BC(3)|EVTG_EVTG_AOI1_BFT01_PT0_CC(1);
	EVTG->EVTG_INST[2].EVTG_CTRL &= ~(EVTG_EVTG_CTRL_SYNC_CTRL_MASK | EVTG_EVTG_CTRL_MODE_SEL_MASK); //bypass mode and disable input sync to avoid trigger loss
	EVTG->EVTG_INST[2].EVTG_AOI0_FILT = 0; // disable filter
	XBAR_INIT(in_EVTG2_OUTA,out_XB_OUT5); // EVTG2_A as PWM_L(XBOUT5)
	XBAR_INIT(in_EVTG2_OUTB,out_XB_OUT4); // EVTG2_B as PWM_H(XBOUT4)
}

/*!
 * @brief Initializes PIT0 to generate 1kHz periodic interrupt.
 */
void PIT0_init(void)
{
	CLOCKGATE_ENABLE(PIT_0);
	
	PIT0->CTRL &= ~PIT_CTRL_CNT_EN_MASK; // reset PIT counter, PIT0 is not in the reset value after ROM bootloader
	PIT0->CTRL &= ~PIT_CTRL_CLKSEL_MASK; // PIT clock = IPBus clock 
	PIT0->CTRL |= PIT_CTRL_PRESCALER(0);
	PIT0->CTRL |= PIT_CTRL_CLKSEL(0);
			
	PIT0->MOD_L = 50000; // 1KHz
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;
	PIT0->CTRL |= PIT_CTRL_PRIE_MASK; //enable PIT0 rollover interrupt			
	INTC->IPR10 &= ~INTC_IPR10_PIT0_ROLLOVR_MASK; // Level 0 interrupt for pit0 roll-over
	INTC->IPR10 |= INTC_IPR10_PIT0_ROLLOVR(1);
	
	//PIT0->CTRL |= PIT_CTRL_CNT_EN_MASK; //enable PIT0 count
}

/*!
 * @brief Initializes PWMA SM0/1/2.
 * SM0 is configured to generate ADC trigger and control interrupt.
 * SM1 is configured to generate lengthened pwm driver for SR.
 * SM2 is configured to generate pwm driver for HB.
 */
void eFlexPWMA_init(void)
{
	CLOCKGATE_ENABLE(PWMA_SM0);
	CLOCKGATE_ENABLE(PWMA_SM1);
	CLOCKGATE_ENABLE(PWMA_SM2);
	
	/*********** configure sm0 to generate ADC trigger and control interrupt ***********/
	PWM->SM[0].CTRL |= PWM_CTRL_FULL_MASK; // full cycle reload
	PWM->SM[0].CTRL &= ~PWM_CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWM->SM[0].CTRL &= ~PWM_CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWM->SM[0].CTRL2 = (PWM->SM[0].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK)|PWM_CTRL2_CLK_SEL(0); // use IPBus clock
	PWM->SM[0].CTRL2 = (PWM->SM[0].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK)|PWM_CTRL2_INIT_SEL(0); // use local sync for SM0
	PWM->SM[0].CTRL2 &= ~PWM_CTRL2_RELOAD_SEL_MASK; // use local reload signal
	
	/* trigger configuration */
	PWM->SM[0].TCTRL &= ~PWM_TCTRL_PWAOT0_MASK; // pwm out_trig is routed to the PWM_TRIG0 port
	PWM->SM[0].TCTRL = (PWM->SM[0].TCTRL & ~PWM_TCTRL_OUT_TRIG_EN_MASK)| PWM_TCTRL_OUT_TRIG_EN(MASK_BIT0); // val0 trigger is used for ADC trigger
			
	PWM->SM[0].INIT  = 0; 
	PWM->SM[0].VAL1  = 1199; 
	PWM->SM[0].VAL0 = 0;
	
	/*********** configure sm1 to generate lengthened pwm driver for SR ***********/
	PWM->SM[1].CTRL |= PWM_CTRL_FULL_MASK; // full cycle reload
	PWM->SM[1].CTRL &= ~PWM_CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWM->SM[1].CTRL &= ~PWM_CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWM->SM[1].CTRL2 = (PWM->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK)|PWM_CTRL2_CLK_SEL(2); // SM0's clock is used
	PWM->SM[1].CTRL2 = (PWM->SM[1].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK)|PWM_CTRL2_INIT_SEL(0); // use local sync
	PWM->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK; // reload source: master reload
	PWM->SM[1].CTRL2 |= PWM_CTRL2_INDEP_MASK; // independent operation
	PWM->SM[1].CTRL2 |= PWM_CTRL2_FRCEN_MASK; // enable force
	PWM->SM[1].CTRL2 = (PWM->SM[1].CTRL2 & ~ PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FORCE_SEL(3); //master reload is the force source
	PWM->SM[1].CTRL2 |= PWM_CTRL2_PWM45_INIT_MASK; // PWMB init high
		
	/* trigger configuration */
	PWM->SM[1].TCTRL |= (PWM_TCTRL_PWAOT0_MASK | PWM_TCTRL_PWBOT1_MASK); // pwm output is routed to the PWM_TRIGx port
	PWM->OUTEN |= (PWM_OUTEN_PWMA_EN(2)|PWM_OUTEN_PWMB_EN(2)); // when route to XBAR, output must be enabled 
		
	/* output configuration */
	PWM->SM[1].OCTRL &= ~0x73F; // fault state: 0 and not invert
	PWM->SM[1].DISMAP[0] = (PWM_DISMAP_DIS0A(1) | PWM_DISMAP_DIS0B(1));
	PWM->SM[1].DISMAP[1] = 0;
	DISABLE_SR;
		
	PWM->SM[1].INIT  = 0; 
	PWM->SM[1].VAL1  = 1199; 
	PWM->SM[1].VAL2  = 0;
	PWM->SM[1].VAL3  = 0;
	PWM->SM[1].VAL4  = 0;
	PWM->SM[1].VAL5  = 0;
	
	/*********** configure sm2 to generate pwm driver for HB ***********/
	PWM->SM[2].CTRL |= PWM_CTRL_FULL_MASK; // full cycle reload
	PWM->SM[2].CTRL &= ~PWM_CTRL_COMPMODE_MASK; // Edge is generated on counter "equal to" value register
	PWM->SM[2].CTRL &= ~PWM_CTRL_LDMOD_MASK; // buffered registers take effect at PWM reload signal when LDOK is set
	PWM->SM[2].CTRL2 = (PWM->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK)|PWM_CTRL2_CLK_SEL(2); // SM0's clock is used
	PWM->SM[2].CTRL2 = (PWM->SM[2].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK)|PWM_CTRL2_INIT_SEL(0); // use local sync
	PWM->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK; // reload source: master reload
	PWM->SM[2].CTRL2 |= PWM_CTRL2_INDEP_MASK; // independent operation
	PWM->SM[2].CTRL2 |= PWM_CTRL2_FRCEN_MASK; // enable force
	PWM->SM[2].CTRL2 = (PWM->SM[2].CTRL2 & ~ PWM_CTRL2_FORCE_SEL_MASK) | PWM_CTRL2_FORCE_SEL(3); //master reload is the force source
	PWM->SM[2].CTRL2 |= PWM_CTRL2_PWM45_INIT_MASK; // PWMB init high
		
	/* trigger configuration */
	PWM->SM[2].TCTRL |= (PWM_TCTRL_PWAOT0_MASK | PWM_TCTRL_PWBOT1_MASK); // pwm output is routed to the PWM_TRIGx port
	PWM->OUTEN |= (PWM_OUTEN_PWMA_EN(4)|PWM_OUTEN_PWMB_EN(4)); // when route to XBAR, output must be enabled in nevis3
		
	/* output configuration */
	PWM->SM[2].OCTRL &= ~0x73F; // fault state: 0 and not invert
	PWM->SM[2].DISMAP[0] = (PWM_DISMAP_DIS0A(1) | PWM_DISMAP_DIS0B(1));
	PWM->SM[2].DISMAP[1] = 0;
	DISABLE_PWM_OUTPUT;
		
	PWM->SM[2].INIT  = 0; 
	PWM->SM[2].VAL1  = 1199; 
	PWM->SM[2].VAL2  = 0;
	PWM->SM[2].VAL3  = 0;
	PWM->SM[2].VAL4  = 0;
	PWM->SM[2].VAL5  = 0;
	PWM->SM[2].FRCTRL |= PWM_FRCTRL_FRAC_PU_MASK;  //power up fractional delay circuit
	PWM->SM[2].FRCTRL |= PWM_FRCTRL_FRAC23_EN_MASK|PWM_FRCTRL_FRAC45_EN_MASK;  //fractional cycle placement enable
	PWM->SM[2].FRACVAL2 = 0;
	PWM->SM[2].FRACVAL3 = 0;
	PWM->SM[2].FRACVAL4 = 0;
	PWM->SM[2].FRACVAL5 = 0;
	PWM->MCTRL2 = (PWM->MCTRL2 & ~PWM_MCTRL2_MONPLL_MASK)|PWM_MCTRL2_MONPLL(1); // monitor PLL operation
		
	/*********** fault configuration **************/
	PWM->FAULT[0].FCTRL &= ~PWM_FCTRL_FLVL_MASK; // A logic 0 on fault0 indicates a fault condition
	PWM->FAULT[0].FCTRL = (PWM->FAULT[0].FCTRL & ~PWM_FCTRL_FAUTO_MASK)|PWM_FCTRL_FAUTO(0); // Manual clearing for fault0
	PWM->FAULT[0].FCTRL = (PWM->FAULT[0].FCTRL & ~PWM_FCTRL_FSAFE_MASK)|PWM_FCTRL_FSAFE(1); // Safe mode for fault0
	PWM->FAULT[0].FSTS = (PWM->FAULT[0].FSTS & ~PWM_FSTS_FFULL_MASK)|PWM_FSTS_FFULL(1); // Full cycle recovery for fault0
	PWM->FAULT[0].FCTRL2 = (PWM->FAULT[0].FCTRL2 & ~PWM_FCTRL2_NOCOMB_MASK)|PWM_FCTRL2_NOCOMB(0); // enable combinational path for fault
	PWM->FAULT[0].FFILT = PWM_FFILT_FILT_CNT(1)|PWM_FFILT_FILT_PER(1);
			
	PWM->MCTRL |= PWM_MCTRL_CLDOK(7); // clear LDOK of sm0-sm2
	PWM->MCTRL |= PWM_MCTRL_LDOK(7); // set LDOK of sm0-sm2
		
	/* enable sm0 val0 compare interrupt for control interrupt */
	PWM->SM[0].STS |= PWM_STS_CMPF(1); // clear sm0 val0 compare flag
	PWM->SM[0].INTEN |= PWM_INTEN_CMPIE(1); // enable sm0 val0 compare interrupt
	INTC->IPR9 = (INTC->IPR9 & ~INTC_IPR9_PWMA_CMP0_MASK) | INTC_IPR9_PWMA_CMP0(2);  // set PWMA_CMP0 irq priority level 1
	/* enable PWM fault interrupt */
	/*PWM->FAULT[0].FSTS |= PWM_FSTS_FFLAG_MASK; // Clear fault flags of fault0~3
	PWM->FAULT[0].FCTRL |= PWM_FCTRL_FIE(1);  // enable iprim fault interrupt
	INTC->IPR8 = (INTC->IPR8 & ~INTC_IPR8_PWMA_FAULT_MASK) | INTC_IPR8_PWMA_FAULT(3); */ // set PWMA_FAULT irq priority level 2
}

/*!
 * @brief Initializes ADC module .
 */
void ADC_init(void)
{
	CLOCKGATE_ENABLE(CYC_ADC);
	
	ADC->CTRL2 &= ~ADC_CTRL2_DIV0_MASK;
	ADC->CTRL2 |= ADC_CTRL2_DIV0(4); // ADC CLK = sys_clk/5 = 10MHz
	ADC->CTRL1 = (ADC->CTRL1 & ~ADC_CTRL1_SMODE_MASK) | ADC_CTRL1_SMODE(5); // Triggered parallel
	ADC->CTRL2 |= ADC_CTRL2_SIMULT_MASK; // Simultaneous mode
	/* sample list configuration */
	ADC->SDIS = 0xFCFC; // four channel to be scanned
	ADC->CLIST1 = (ADC->CLIST1 & ~ADC_CLIST1_SAMPLE0_MASK)|ADC_CLIST1_SAMPLE0(0x0); // ANA0->Sample0, Iout 
	ADC->CLIST1 = (ADC->CLIST1 & ~ADC_CLIST1_SAMPLE1_MASK)|ADC_CLIST1_SAMPLE1(0x5); // ANA5->Sample1, Iprim
	ADC->CLIST3 = (ADC->CLIST3 & ~ADC_CLIST3_SAMPLE8_MASK)|ADC_CLIST3_SAMPLE8(0xA); // ANB2->Sample8, Vout
	ADC->CLIST3 = (ADC->CLIST3 & ~ADC_CLIST3_SAMPLE9_MASK)|ADC_CLIST3_SAMPLE9(0x9); // ANB1->Sample9, Vprim
		
	ADC->CTRL1 |= ADC_CTRL1_SYNC0_MASK;  // ADC0 can be triggered by SYNC0 input pulse
	ADC->CTRL1 &= ~ADC_CTRL1_STOP0_MASK; // Normal operation for ADC0

	ADC->PWR &= ~(ADC_PWR_PD0_MASK|ADC_PWR_PD1_MASK); // Power up ADC0&1
}

/*!
 * @brief Initializes compartors.
 * CMPB for resonant tank over-current protection: Postive input: DACB, negative input: ANB7
 * CMPA for SR1 off control: Postive input: DACA, negative input: ANA3
 * CMPD for SR2 off control: Postive input: DACD, negative input: ANA4
 */
void CMP_init(void)
{
	/*CMPB configuration for resonant tank over-current protection */
	CLOCKGATE_ENABLE(CMP_B);
	CMPB->CR1 |= CMP_CR1_PMODE_MASK; // high speed
	CMPB->DACCR = CMP_DACCR_VOSEL(235); 
	CMPB->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPB->MUXCR = (CMPB->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACB
	CMPB->MUXCR = (CMPB->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(2); // minus input: IN2, ANB7
			
	CMPB->CR0 = (CMPB->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 3 consecutive samples must agree
	CMPB->FPR = (CMPB->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(2); // filter sample period = 2
	CMPB->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
		
	/*CMPD configuration for SR2 off control */
	CLOCKGATE_ENABLE(CMP_D);
	CMPD->CR1 |= CMP_CR1_PMODE_MASK; // high speed		
	CMPD->DACCR = CMP_DACCR_VOSEL(80); 
	CMPD->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPD->MUXCR = (CMPD->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACD
	CMPD->MUXCR = (CMPD->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(0); // minus input: IN0, ANA4
			
	CMPD->CR0 = (CMPD->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 3 consecutive samples must agree
	CMPD->FPR = (CMPD->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(1); // filter sample period = 1
	CMPD->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
		
	/*CMPB configuration for SR1 off control */
	CLOCKGATE_ENABLE(CMP_A);
	CMPA->CR1 |= CMP_CR1_PMODE_MASK; // high speed			
	CMPA->DACCR = CMP_DACCR_VOSEL(80); 
	CMPA->MUXCR |= CMP_MUXCR_DACEN_MASK; // enable DAC 
	CMPA->MUXCR = (CMPA->MUXCR & ~CMP_MUXCR_PSEL_MASK)| CMP_MUXCR_PSEL(7); // plus input: IN7, DACA
	CMPA->MUXCR = (CMPA->MUXCR & ~CMP_MUXCR_MSEL_MASK)| CMP_MUXCR_MSEL(2); // minus input: IN2, ANA3
				
	CMPA->CR0 = (CMPA->CR0 & ~CMP_CR0_FILTER_CNT_MASK)| CMP_CR0_FILTER_CNT(3); // filter count, 7 consecutive samples must agree
	CMPA->FPR = (CMPA->FPR & ~CMP_FPR_FILT_PER_MASK)| CMP_FPR_FILT_PER(1); // filter sample period = 3
	CMPA->CR1 |= CMP_CR1_EN_MASK; // enable comparator module
}

/*!
 * @brief Initializes SCI0 for FreeMASTER communiction.
 */
void sci0_init(void)
{
	CLOCKGATE_ENABLE(SCI_0);
	QSCI0->RATE = (((uint16_t)(27)<<3)|1); // 115200
	QSCI0->CTRL2 |= QSCI_CTRL2_FIFO_EN_MASK; // enable 4-word-deep FIFOs
	QSCI0->CTRL3 = 0;
	QSCI0->CTRL1 |= (QSCI_CTRL1_TE_MASK|QSCI_CTRL1_RE_MASK);//enable TX and RX
}

/*!
 * @brief Initializes SCI0 for FreeMASTER communiction.
 */
void sci1_init(void)
{
	CLOCKGATE_ENABLE(SCI_1);
	QSCI1->RATE = (((uint16_t)(27)<<3)|1); // 115200
	QSCI1->CTRL3 = 0;
	QSCI1->CTRL1 |= (QSCI_CTRL1_TE_MASK|QSCI_CTRL1_RE_MASK);//enable TX and RX
}

/*!
 * @brief Initializes all used module.
 */
void Peripherals_init(void)
{
	Pin_init();
#if SCI0_ENABLE
	sci0_init();
#endif
	sci1_init();
	eFlexPWMA_init();
	ADC_init();
	PIT0_init();
	CMP_init();
	XBAR_init();
}
