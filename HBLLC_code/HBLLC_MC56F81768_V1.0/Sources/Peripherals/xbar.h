/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef XBAR_H_
#define XBAR_H_

/************************************ XBAR related MACRO ******************************************/
#define XBAR_INIT(in,out) XBAR_INIT_1(in,out##_REG,out)

// XBAR output signals
#define out_DMA_REQ0 					0
#define out_DMA_REQ1 					1
#define out_DMA_REQ2 					2
#define out_DMA_REQ3 					3
#define out_XB_OUT4  					4
#define out_XB_OUT5  					5
#define out_XB_OUT6  					6
#define out_XB_OUT7  					7
#define out_XB_OUT8  					8
#define out_XB_OUT9  					9
#define out_XB_OUT10  				10
#define out_XB_OUT11  				11
#define out_ADCA_TRIG 				12
#define out_ADCB_TRIG 				13
#define out_DAC_12B_SYNC 			15
#define out_CMPA 						16
#define out_CMPB 						17
#define out_CMPC 						18
#define out_CMPD 						19
#define out_PWMA_SM0_EXTA_INPUT  20
#define out_PWMA_SM1_EXTA_INPUT  21
#define out_PWMA_SM2_EXTA_INPUT  22
#define out_PWMA_SM3_EXTA_INPUT  23
#define out_PWMA_SM0_EXT_SYNC			24
#define out_PWMA_SM1_EXT_SYNC			25
#define out_PWMA_SM2_EXT_SYNC			26
#define out_PWMA_SM3_EXT_SYNC			27
#define out_PWMA_EXT_CLK			28
#define out_PWMA_FAULT0			29
#define out_PWMA_FAULT1			30
#define out_PWMA_FAULT2			31
#define out_PWMA_FAULT3			32
#define out_PWMA_FORCE				33
#define out_OPAMPA_MUX_WIN				34
#define out_OPAMPB_MUX_WIN				35
#define out_SCI0_RXD				36
#define out_SCI1_RXD				37
#define out_TIMERA_IN0				38
#define out_TIMERA_IN1				39
#define out_TIMERA_IN2				40
#define out_TIMERA_IN3				41
#define out_QDC_PHA			42
#define out_QDC_PHB			43
#define out_QDC_INDEX			44
#define out_QDC_HOME			45
#define out_QDC_TRIG				46
#define out_EVTG0_INA					47
#define out_EVTG0_INB					48
#define out_EVTG0_INC					49
#define out_EVTG0_IND					50
#define out_EVTG1_INA					51
#define out_EVTG1_INB					52
#define out_EVTG1_INC					53
#define out_EVTG1_IND					54
#define out_EVTG2_INA					55
#define out_EVTG2_INB					56
#define out_EVTG2_INC					57
#define out_EVTG2_IND					58
#define out_EVTG3_INA					59
#define out_EVTG3_INB					60
#define out_EVTG3_INC					61
#define out_EVTG3_IND					62
#define out_EWN_IN					63
#define out_LPI2C0_IN_TRIG_HREQ		64
#define out_LPI2C1_IN_TRIG_HREQ		65

// XBAR input signals
#define in_VSS						0
#define in_VDD						1
#define in_XB_IN2					2
#define in_XB_IN3					3
#define in_XB_IN4					4
#define in_XB_IN5					5
#define in_XB_IN6					6
#define in_XB_IN7					7
#define in_XB_IN8					8
#define in_XB_IN9					9
#define in_LPI2C0_M_S_OUT_TRIG			10
#define in_EMW_OUT_B					11
#define in_CMPA_OUT					12
#define in_CMPB_OUT					13
#define in_CMPC_OUT					14
#define in_CMPD_OUT					15
#define in_AN8_LIMIT		16
#define in_AN9_LIMIT		17
#define in_AN10_LIMIT		18
#define in_AN11_LIMIT		19
#define in_PWMA0_MUX_TRIG0  20
#define in_PWMA0_MUX_TRIG1  21
#define in_PWMA1_MUX_TRIG0  22
#define in_PWMA1_MUX_TRIG1  23
#define in_PWMA2_MUX_TRIG0  24
#define in_PWMA2_MUX_TRIG1  25
#define in_PWMA3_MUX_TRIG0  26
#define in_PWMA3_MUX_TRIG1  27
#define in_PWMA0_OUT_TRIG0  28
#define in_PWMA0_OUT_TRIG1 29
#define in_PWMA1_OUT_TRIG0 30
#define in_PWMA1_OUT_TRIG1 31
#define in_PWMA2_OUT_TRIG0 32
#define in_PWMA2_OUT_TRIG1 33
#define in_PWMA3_OUT_TRIG0 34
#define in_PWMA3_OUT_TRIG1 35
#define in_AN0_LIMIT_or_TIMERA0_OUT		36
#define in_AN1_LIMIT_or_TIMERA1_OUT		37
#define in_AN2_LIMIT_or_TIMERA2_OUT		38
#define in_AN3_LIMIT_or_TIMERA3_OUT		39
#define in_EVTG0_OUTA					40
#define in_EVTG0_OUTB					46
#define in_EVTG1_OUTA					41
#define in_EVTG1_OUTB					47
#define in_EVTG2_OUTA					42
#define in_EVTG2_OUTB					48
#define in_EVTG3_OUTA					43
#define in_EVTG3_OUTB					49
#define in_PIT0_SYNC_OUT				44
#define in_PIT1_SYNC_OUT				45
#define in_QDC_POSMATCH					50
#define in_PWMA_ALL_TRIG				51
#define in_SCI0_RXD_FULL				52
#define in_LPI2C1_M_S_OUT_TRIG			53

// XBAR register number
#define out_DMA_REQ0_REG 					0
#define out_DMA_REQ1_REG 					0
#define out_DMA_REQ2_REG 					1
#define out_DMA_REQ3_REG 					1
#define out_XB_OUT4_REG  					2
#define out_XB_OUT5_REG  					2
#define out_XB_OUT6_REG  					3
#define out_XB_OUT7_REG  					3
#define out_XB_OUT8_REG  					4
#define out_XB_OUT9_REG  					4
#define out_XB_OUT10_REG  				5
#define out_XB_OUT11_REG  				5
#define out_ADCA_TRIG_REG 				6
#define out_ADCB_TRIG_REG 				6

#define out_DAC_12B_SYNC_REG 			7
#define out_CMPA_REG 						8
#define out_CMPB_REG 						8
#define out_CMPC_REG 						9
#define out_CMPD_REG 						9
#define out_PWMA_SM0_EXTA_INPUT_REG  10
#define out_PWMA_SM1_EXTA_INPUT_REG  10
#define out_PWMA_SM2_EXTA_INPUT_REG  11
#define out_PWMA_SM3_EXTA_INPUT_REG  11
#define out_PWMA_SM0_EXT_SYNC_REG			12
#define out_PWMA_SM1_EXT_SYNC_REG			12
#define out_PWMA_SM2_EXT_SYNC_REG			13
#define out_PWMA_SM3_EXT_SYNC_REG			13
#define out_PWMA_EXT_CLK_REG			14
#define out_PWMA_FAULT0_REG			14
#define out_PWMA_FAULT1_REG			15
#define out_PWMA_FAULT2_REG			15
#define out_PWMA_FAULT3_REG			16
#define out_PWMA_FORCE_REG				16
#define out_OPAMPA_MUX_WIN_REG				17
#define out_OPAMPB_MUX_WIN_REG				17
#define out_SCI0_RXD_REG				18
#define out_SCI1_RXD_REG				18
#define out_TIMERA_IN0_REG				19
#define out_TIMERA_IN1_REG				19
#define out_TIMERA_IN2_REG				20
#define out_TIMERA_IN3_REG				20
#define out_QDC_PHA_REG			21
#define out_QDC_PHB_REG			21
#define out_QDC_INDEX_REG			22
#define out_QDC_HOME_REG			22
#define out_QDC_TRIG_REG				23
#define out_EVTG0_INA_REG					23
#define out_EVTG0_INB_REG					24
#define out_EVTG0_INC_REG					24
#define out_EVTG0_IND_REG					25
#define out_EVTG1_INA_REG					25
#define out_EVTG1_INB_REG					26
#define out_EVTG1_INC_REG					26
#define out_EVTG1_IND_REG					27
#define out_EVTG2_INA_REG					27
#define out_EVTG2_INB_REG					28
#define out_EVTG2_INC_REG					28
#define out_EVTG2_IND_REG					29
#define out_EVTG3_INA_REG					29
#define out_EVTG3_INB_REG					30
#define out_EVTG3_INC_REG					30
#define out_EVTG3_IND_REG					31
#define out_EWN_IN_REG					31
#define out_LPI2C0_IN_TRIG_HREQ_reg		32
#define out_LPI2C1_IN_TRIG_HREQ_reg		32


#define XBAR_INIT_1(in,outreg,out) XBAR_INIT_2(in,outreg,out)
#define XBAR_INIT_2(in,outreg,out) XBARA->SEL##outreg = (XBARA->SEL##outreg &= ~XBARA_SEL##outreg##_SEL##out##_MASK)|XBARA_SEL##outreg##_SEL##out(in) 

#endif /* XBAR_H_ */
