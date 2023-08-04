/*
* Copyright 2020 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef SIM_H_
#define SIM_H_

/************************************ Enable clock gate related MACRO ******************************************/

#define CLOCKGATE_ENABLE(module_name) CLOCKGATE_ENABLE1(module_name)
#define CLOCKGATE_ENABLE1(module_name) CLOCKGATE_ENABLE2(module_name##_pce_no,module_name##_MASK)
#define CLOCKGATE_STOPMODE_ENABLE(module_name) CLOCKGATE_STOPMODE_ENABLE1(module_name)
#define CLOCKGATE_STOPMODE_ENABLE1(module_name) CLOCKGATE_STOPMODE_ENABLE2(module_name##_pce_no,module_name##_MASK)
#define CLOCKGATE_DISABLE(module_name) CLOCKGATE_DISABLE1(module_name)
#define CLOCKGATE_DISABLE1(module_name) CLOCKGATE_DISABLE2(module_name##_pce_no,module_name##_MASK)
#define CLOCKGATE_STOPMODE_DISABLE(module_name) CLOCKGATE_STOPMODE_DISABLE1(module_name)
#define CLOCKGATE_STOPMODE_DISABLE1(module_name) CLOCKGATE_STOPMODE_DISABLE2(module_name##_pce_no,module_name##_MASK)
// Module names
#define QTimer_A0  QTimer_A0
#define QTimer_A1  QTimer_A1			
#define QTimer_A2  QTimer_A2		
#define QTimer_A3  QTimer_A3				
#define GPIO_A	GPIO_A			
#define GPIO_B	GPIO_B			
#define GPIO_C	GPIO_C			
#define GPIO_D	GPIO_D			
#define GPIO_E	GPIO_E			
#define GPIO_F	GPIO_F			
#define DAC_0	DAC_0				
#define SCI_0	SCI_0			
#define SCI_1	SCI_1					
#define QSPI_0	QSPI_0					
#define LPI2C_0	LPI2C_0			
#define LPI2C_1	LPI2C_1						
#define CMP_A	CMP_A			
#define CMP_B	CMP_B
#define CMP_C	CMP_C
#define CMP_D	CMP_D
#define CYC_ADC CYC_ADC
#define CRC_0	CRC_0
#define PIT_0	PIT_0
#define PIT_1	PIT_1
#define QDC_0	QDC_0
#define ROM	    ROM
#define PWMA_SM0 PWMA_SM0
#define PWMA_SM1 PWMA_SM1
#define PWMA_SM2 PWMA_SM2
#define PWMA_SM3 PWMA_SM3
#define DSASS    DSASS
#define OPAMP_A  OPAMP_A
#define OPAMP_B  OPAMP_B

// SIM_PCE register number
#define QTimer_A0_pce_no 	0
#define QTimer_A1_pce_no 	0			
#define QTimer_A2_pce_no 	0			
#define QTimer_A3_pce_no 	0						
#define GPIO_A_pce_no 	0				
#define GPIO_B_pce_no 	0				
#define GPIO_C_pce_no 	0				
#define GPIO_D_pce_no 	0				
#define GPIO_E_pce_no 	0				
#define GPIO_F_pce_no 	0								
#define DAC_0_pce_no	1				
#define SCI_0_pce_no	1				
#define SCI_1_pce_no	1								
#define QSPI_0_pce_no	1								
#define LPI2C_0_pce_no	1				
#define LPI2C_1_pce_no	1				
#define CMP_A_pce_no	2				
#define CMP_B_pce_no	2		
#define CMP_C_pce_no	2	
#define CMP_D_pce_no	2	
#define CYC_ADC_pce_no	2	
#define CRC_0_pce_no	2	
#define QDC_0_pce_no	2	
#define PIT_0_pce_no	2	
#define PIT_1_pce_no	2
#define ROM_pce_no		3
#define DSASS_pce_no		3
#define PWMA_SM0_pce_no		3
#define PWMA_SM1_pce_no		3
#define PWMA_SM2_pce_no		3
#define PWMA_SM3_pce_no		3
#define OPAMP_A_pce_no		3
#define OPAMP_B_pce_no		3


#define QTimer_A0_MASK		TA0_MASK
#define QTimer_A1_MASK		TA1_MASK			
#define QTimer_A2_MASK		TA2_MASK			
#define QTimer_A3_MASK		TA3_MASK			
#define GPIO_A_MASK			GPIOA_MASK				
#define GPIO_B_MASK			GPIOB_MASK				
#define GPIO_C_MASK			GPIOC_MASK				
#define GPIO_D_MASK			GPIOD_MASK				
#define GPIO_E_MASK			GPIOE_MASK				
#define GPIO_F_MASK			GPIOF_MASK				
#define DAC_0_MASK			DAC_MASK							
#define SCI_0_MASK			SCI0_MASK				
#define SCI_1_MASK			SCI1_MASK		
#define QSPI_0_MASK			QSPI0_MASK	
#define LPI2C_0_MASK		LPI2C0_MASK
#define LPI2C_1_MASK		LPI2C1_MASK		
#define CMP_A_MASK			CMPA_MASK	
#define CMP_B_MASK			CMPB_MASK
#define CMP_C_MASK			CMPC_MASK
#define CMP_D_MASK			CMPD_MASK
#define CYC_ADC_MASK		CYCADC_MASK
#define CRC_0_MASK   		CRC_MASK
#define QDC_0_MASK   		QDC_MASK
#define PIT_0_MASK			PIT0_MASK
#define PIT_1_MASK			PIT1_MASK
#define ROM_MASK			ROM_MASK
#define PWMA_SM0_MASK		PWMACH0_MASK
#define PWMA_SM1_MASK		PWMACH1_MASK
#define PWMA_SM2_MASK		PWMACH2_MASK
#define PWMA_SM3_MASK		PWMACH3_MASK
#define DSASS_MASK			DSASS_MASK
#define OPAMP_A_MASK		OPAMPA_MASK
#define OPAMP_B_MASK		OPAMPB_MASK

#define CLOCKGATE_ENABLE2(pce_no,mask_name) CLOCKGATE_ENABLE3(pce_no,mask_name) 
#define CLOCKGATE_ENABLE3(pce_no,mask_name) SIM->PCE##pce_no |= SIM_PCE##pce_no##_##mask_name
#define CLOCKGATE_STOPMODE_ENABLE2(pce_no,mask_name) CLOCKGATE_STOPMODE_ENABLE3(pce_no,mask_name) 
#define CLOCKGATE_STOPMODE_ENABLE3(pce_no,mask_name) SIM->SD##pce_no |= SIM_SD##pce_no##_##mask_name
#define CLOCKGATE_DISABLE2(pce_no,mask_name) CLOCKGATE_DISABLE3(pce_no,mask_name) 
#define CLOCKGATE_DISABLE3(pce_no,mask_name) SIM->PCE##pce_no &= ~SIM_PCE##pce_no##_##mask_name
#define CLOCKGATE_STOPMODE_DISABLE2(pce_no,mask_name) CLOCKGATE_STOPMODE_DISABLE3(pce_no,mask_name) 
#define CLOCKGATE_STOPMODE_DISABLE3(pce_no,mask_name) SIM->SD##pce_no &= ~SIM_SD##pce_no##_##mask_name

/* 
 * 	Generate SW reset
 * 
 *  */
#define SW_RESET() SIM->CTRL |= SIM_CTRL_SWRST_MASK

/* 
 * 	Disable DMA
 * 
 *  */
inline void DMA_disable(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
}

/* 
 * 	Enable DMA in run mode only
 * 
 *  */
inline void DMA_enable_in_run(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(1);
}

/* 
 * 	Enable DMA in run and wait mode 
 * 
 *  */
inline void DMA_enable_in_run_wait(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(2);
}

/* 
 * 	Enable DMA in all power modes. Can't switch to STOP mode in this configuration. This is default configuration.
 * 
 *  */
inline void DMA_enable(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(3);
}

/* 
 * 	Disable DMA and write protect till next reset
 * 
 *  */
inline void DMA_disable_wp(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(4);
}

/* 
 * 	Enable DMA in run mode only and write protect till next reset
 * 
 *  */
inline void DMA_enable_in_run_wp(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(5);
}

/* 
 * 	Enable DMA in run and wait mode and write protect till next reset
 * 
 *  */
inline void DMA_enable_in_run_wait_wp(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(6);
}

/* 
 * 	Enable DMA in all power modes and write protect till next reset. Can't switch to STOP mode in this configuration. 
 * 
 *  */
inline void DMA_enable_wp(void)
{
	SIM->CTRL &= ~SIM_CTRL_DMAEBL_MASK;
	SIM->CTRL |= SIM_CTRL_DMAEBL(7);
}



/* 
 * 	Choose ADC and QTimerA/B output signals for XBARA inputs
 * 
 *  */

#define AN0_LIMIT_or_TIMERA0_OUT_ADC_EN() SIM->ADC_TMR_SEL |= SIM_ADC_TMR_SEL_XBAR_IN36_MASK
#define AN0_LIMIT_or_TIMERA0_OUT_TMR_EN() SIM->ADC_TMR_SEL &= ~SIM_ADC_TMR_SEL_XBAR_IN36_MASK
#define AN1_LIMIT_or_TIMERA1_OUT_ADC_EN() SIM->ADC_TMR_SEL |= SIM_ADC_TMR_SEL_XBAR_IN37_MASK
#define AN1_LIMIT_or_TIMERA1_OUT_TMR_EN() SIM->ADC_TMR_SEL &= ~SIM_ADC_TMR_SEL_XBAR_IN37_MASK
#define AN2_LIMIT_or_TIMERA2_OUT_ADC_EN() SIM->ADC_TMR_SEL |= SIM_ADC_TMR_SEL_XBAR_IN38_MASK
#define AN2_LIMIT_or_TIMERA2_OUT_TMR_EN() SIM->ADC_TMR_SEL &= ~SIM_ADC_TMR_SEL_XBAR_IN38_MASK
#define AN3_LIMIT_or_TIMERA3_OUT_ADC_EN() SIM->ADC_TMR_SEL |= SIM_ADC_TMR_SEL_XBAR_IN39_MASK
#define AN3_LIMIT_or_TIMERA3_OUT_TMR_EN() SIM->ADC_TMR_SEL &= ~SIM_ADC_TMR_SEL_XBAR_IN39_MASK

/*
 *  Force to boot from Flash after SW reset 
 * 
 * */
#define BOOT_FROM_ROM_AFTER_SW_RESET() SIM->BOOT_MODE_OVERRIDE |= SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK 



#endif /* SIM_H_ */
