/*
  PDD layer implementation for peripheral type SIM
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(SIM_PDD_H_)
#define SIM_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error SIM PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* SIM */ && \
      !defined(MCU_MC56F82316) /* SIM */ && \
      !defined(MCU_MC56F82723) /* SIM */ && \
      !defined(MCU_MC56F82726) /* SIM */ && \
      !defined(MCU_MC56F82728) /* SIM */ && \
      !defined(MCU_MC56F82733) /* SIM */ && \
      !defined(MCU_MC56F82736) /* SIM */ && \
      !defined(MCU_MC56F82738) /* SIM */ && \
      !defined(MCU_MC56F82743) /* SIM */ && \
      !defined(MCU_MC56F82746) /* SIM */ && \
      !defined(MCU_MC56F82748) /* SIM */
  // Unsupported MCU is active
  #error SIM PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* DMA configuration constants. */
#define SIM_PDD_DMA_DISABLE                   0U /**< DMA module is disabled. */
#define SIM_PDD_DMA_ENABLE_RUN                0x40U /**< DMA module is enabled in run mode only. */
#define SIM_PDD_DMA_ENABLE_RUN_WAIT           0x80U /**< DMA module is enabled in run and wait modes only. */
#define SIM_PDD_DMA_ENABLE_ALL                0xC0U /**< DMA module is enabled in all power modes. */
#define SIM_PDD_DMA_DISABLE_PROTECTED         0x100U /**< DMA module is disabled and this state is write protected until the next reset. */
#define SIM_PDD_DMA_ENABLE_RUN_PROTECTED      0x140U /**< DMA module is enabled in run mode only and protected until the next reset. */
#define SIM_PDD_DMA_ENABLE_RUN_WAIT_PROTECTED 0x180U /**< DMA module is enabled in run and wait modes only and this state is write protected until the next reset. */
#define SIM_PDD_DMA_ENABLE_ALL_PROTECTED      0x1C0U /**< DMA module is enabled in all low power modes and this state is write protected until the next reset. */

/* Reset status constants. */
#define SIM_PDD_SW_RESET_FLAG                         SIM_RSTAT_SWR_MASK /**< Software reset request flag. */
#define SIM_PDD_WATCHDOG_WINDOW_RESET_FLAG            SIM_RSTAT_COP_WNDOW_MASK /**< COP window time-out reset flag. */
#define SIM_PDD_WATCHDOG_CPU_RESET_FLAG               SIM_RSTAT_COP_CPU_MASK /**< COP CPU time-out reset flag. */
#define SIM_PDD_WATCHDOG_LOSS_OF_REFERENCE_RESET_FLAG SIM_RSTAT_COP_LOR_MASK /**< COP loss of reference reset flag. */
#define SIM_PDD_EXTERNAL_RESET_FLAG                   SIM_RSTAT_EXTR_MASK /**< External reference reset flag. */
#define SIM_PDD_POWER_ON_RESET_FLAG                   SIM_RSTAT_POR_MASK /**< Power-on reset flag. */

/* CLKOUT divide factor constants. */
#define SIM_PDD_DIVIDE_1   0U                    /**< Divide by 1. */
#define SIM_PDD_DIVIDE_2   0x2000U               /**< Divide by 2. */
#define SIM_PDD_DIVIDE_4   0x4000U               /**< Divide by 4. */
#define SIM_PDD_DIVIDE_8   0x6000U               /**< Divide by 8. */
#define SIM_PDD_DIVIDE_16  0x8000U               /**< Divide by 16. */
#define SIM_PDD_DIVIDE_32  0xA000U               /**< Divide by 32. */
#define SIM_PDD_DIVIDE_64  0xC000U               /**< Divide by 64. */
#define SIM_PDD_DIVIDE_128 0xE000U               /**< Divide by 128. */

/* TMR peripherals clock gate mask constants. Used to specify TMR peripherals of
   which clock gate will be enbaled or disabled. */
#define SIM_PDD_CLOCK_GATE_TMRA0_MASK SIM_PCE0_TA0_MASK /**< TMRA0 clock gate. */
#define SIM_PDD_CLOCK_GATE_TMRA1_MASK SIM_PCE0_TA1_MASK /**< TMRA1 clock gate. */
#define SIM_PDD_CLOCK_GATE_TMRA2_MASK SIM_PCE0_TA2_MASK /**< TMRA2 clock gate. */
#define SIM_PDD_CLOCK_GATE_TMRA3_MASK SIM_PCE0_TA3_MASK /**< TMRA3 clock gate. */

/* GPIO peripherals clock gate mask constants. Used to specify GPIO peripherals
   of which clock gate will be enbaled or disabled. */
#define SIM_PDD_CLOCK_GATE_GPIOA_MASK SIM_PCE0_GPIOA_MASK /**< GPIOA clock gate. */
#define SIM_PDD_CLOCK_GATE_GPIOB_MASK SIM_PCE0_GPIOB_MASK /**< GPIOB clock gate. */
#define SIM_PDD_CLOCK_GATE_GPIOC_MASK SIM_PCE0_GPIOC_MASK /**< GPIOC clock gate. */
#define SIM_PDD_CLOCK_GATE_GPIOD_MASK SIM_PCE0_GPIOD_MASK /**< GPIOD clock gate. */
#define SIM_PDD_CLOCK_GATE_GPIOE_MASK SIM_PCE0_GPIOE_MASK /**< GPIOE clock gate. */
#define SIM_PDD_CLOCK_GATE_GPIOF_MASK SIM_PCE0_GPIOF_MASK /**< GPIOF clock gate. */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/* QSCI peripherals clock gate mask constants. Used to specify QSCI peripherals
   of which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_QSCI0_MASK SIM_PCE1_SCI0_MASK /**< QSCI0 clock gate. */

#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/* QSCI peripherals clock gate mask constants. Used to specify QSCI peripherals
   of which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_QSCI0_MASK SIM_PCE1_SCI0_MASK /**< QSCI0 clock gate. */
  #define SIM_PDD_CLOCK_GATE_QSCI1_MASK SIM_PCE1_SCI1_MASK /**< QSCI1 clock gate. */

#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/* QSPI peripherals clock gate mask constants. Used to specify QSPI peripherals
   of which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_QSPI0_MASK SIM_PCE1_QSPI0_MASK /**< QSPI0 clock gate. */
  #define SIM_PDD_CLOCK_GATE_QSPI1_MASK SIM_PCE1_QSPI1_MASK /**< QSPI1 clock gate. */

#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/* QSPI peripherals clock gate mask constants. Used to specify QSPI peripherals
   of which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_QSPI0_MASK SIM_PCE1_QSPI0_MASK /**< QSPI0 clock gate. */

#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
#if (defined(MCU_MC56F82313))
/* CMP peripherals clock gate mask constants. Used to specify CMP peripherals of
   which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_CMPA_MASK SIM_PCE2_CMPA_MASK /**< CMPA clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPB_MASK SIM_PCE2_CMPB_MASK /**< CMPB clock gate. */

#elif ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/* CMP peripherals clock gate mask constants. Used to specify CMP peripherals of
   which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_CMPA_MASK SIM_PCE2_CMPA_MASK /**< CMPA clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPB_MASK SIM_PCE2_CMPB_MASK /**< CMPB clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPC_MASK SIM_PCE2_CMPC_MASK /**< CMPC clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPD_MASK SIM_PCE2_CMPD_MASK /**< CMPD clock gate. */

#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/* CMP peripherals clock gate mask constants. Used to specify CMP peripherals of
   which clock gate will be enbaled or disabled. */
  #define SIM_PDD_CLOCK_GATE_CMPA_MASK SIM_PCE2_CMPA_MASK /**< CMPA clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPB_MASK SIM_PCE2_CMPB_MASK /**< CMPB clock gate. */
  #define SIM_PDD_CLOCK_GATE_CMPC_MASK SIM_PCE2_CMPC_MASK /**< CMPC clock gate. */

#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/* PIT peripherals clock gate mask constants. Used to specify PIT peripherals of
   which clock gate will be enbaled or disabled. */
#define SIM_PDD_CLOCK_GATE_PIT0_MASK SIM_PCE2_PIT0_MASK /**< PIT0 clock gate. */
#define SIM_PDD_CLOCK_GATE_PIT1_MASK SIM_PCE2_PIT1_MASK /**< PIT1 clock gate. */

/* PWM peripherals clock gate mask constants. Used to specify PWM peripherals of
   which clock gate will be enbaled or disabled. */
#define SIM_PDD_CLOCK_GATE_PWMACH0_MASK SIM_PCE3_PWMACH0_MASK /**< PWMA channel 0 clock gate. */
#define SIM_PDD_CLOCK_GATE_PWMACH1_MASK SIM_PCE3_PWMACH1_MASK /**< PWMA channel 1 clock gate. */
#define SIM_PDD_CLOCK_GATE_PWMACH2_MASK SIM_PCE3_PWMACH2_MASK /**< PWMA channel 2 clock gate. */
#define SIM_PDD_CLOCK_GATE_PWMACH3_MASK SIM_PCE3_PWMACH3_MASK /**< PWMA channel 3 clock gate. */

/* GPIO A0 peripheral function constants. */
#define SIM_PDD_GPIOA0_ANA0_CMPA3 0U             /**< GPIOA0_ANA0_CMPA3 function. */
#define SIM_PDD_GPIOA0_CMPC_O     0x1U           /**< GPIOA0_CMPC_O function. */

/* GPIO B1 peripheral function constants. */
#define SIM_PDD_GPIOB1_ANB1_CMPB_IN0 0U          /**< GPIOB1_ANB1_CMPB_IN0 function. */
#define SIM_PDD_GPIOB1_DACB          0x4U        /**< GPIOB1_DACB function. */

/* GPIO C0 peripheral function constants. */
#define SIM_PDD_GPIOC0_EXTAL 0U                  /**< GPIOC0_EXTAL function. */
#define SIM_PDD_GPIOC0_CLKIN 0x1U                /**< GPIOC0_CLKIN function. */

/* GPIO C2 peripheral function constants. */
#define SIM_PDD_GPIOC2_TXD0     0U               /**< GPIOC2_TXD0 function. */
#define SIM_PDD_GPIOC2_XBOUT_11 0x10U            /**< GPIOC2_XBOUT_11 function. */
#define SIM_PDD_GPIOC2_XB_IN2   0x20U            /**< GPIOC2_XB_IN2 function. */
#define SIM_PDD_GPIOC2_CLKOUT0  0x30U            /**< GPIOC2_CLKOUT0 function. */

/* GPIO C3 peripheral function constants. */
#define SIM_PDD_GPIOC3_TA0    0U                 /**< GPIOC3_TA0 function. */
#define SIM_PDD_GPIOC3_CMPA_O 0x40U              /**< GPIOC3_CMPA_O function. */
#define SIM_PDD_GPIOC3_RXD0   0x80U              /**< GPIOC3_RXD0 function. */
#define SIM_PDD_GPIOC3_CLKIN1 0xC0U              /**< GPIOC3_CLKIN1 function. */

/* GPIO C4 peripheral function constants. */
#define SIM_PDD_GPIOC4_TA1       0U              /**< GPIOC4_TA1 function. */
#define SIM_PDD_GPIOC4_CMPB_O    0x100U          /**< GPIOC4_CMPB_O function. */
#define SIM_PDD_GPIOC4_XB_IN6    0x200U          /**< GPIOC4_XB_IN6 function. */
#define SIM_PDD_GPIOC4_EWM_OUT_B 0x300U          /**< GPIOC4_EWM_OUT_B function. */

/* GPIO C5 peripheral function constants. */
#define SIM_PDD_GPIOC5_DACA   0U                 /**< GPIOC5_DACA function. */
#define SIM_PDD_GPIOC5_XB_IN7 0x400U             /**< GPIOC5_XB_IN7 function. */

/* GPIO C6 peripheral function constants. */
#define SIM_PDD_GPIOC6_TA2    0U                 /**< GPIOC6_TA2 function. */
#define SIM_PDD_GPIOC6_XB_IN3 0x1000U            /**< GPIOC6_XB_IN3 function. */
#define SIM_PDD_GPIOC6_CMPREF 0x2000U            /**< GPIOC6_CMPREF function. */
#define SIM_PDD_GPIOC6_SS0_B  0x3000U            /**< GPIOC6_SS0_B function. */

/* GPIO C7 peripheral function constants. */
#define SIM_PDD_GPIOC7_SS0_B  0U                 /**< GPIOC7_SS0_B function. */
#define SIM_PDD_GPIOC7_TXD0   0x4000U            /**< GPIOC7_TXD0 function. */
#define SIM_PDD_GPIOC7_XBIN_8 0x8000U            /**< GPIOC7_XBIN_8 function. */

/* GPIO C8 peripheral function constants. */
#define SIM_PDD_GPIOC8_MISO0   0U                /**< GPIOC8_MISO0 function. */
#define SIM_PDD_GPIOC8_RXD0    0x1U              /**< GPIOC8_RXD0 function. */
#define SIM_PDD_GPIOC8_XB_IN9  0x2U              /**< GPIOC8_XB_IN9 function. */
#define SIM_PDD_GPIOC8_XB_OUT6 0x3U              /**< GPIOC8_XB_OUT6 function. */

/* GPIO C9 peripheral function constants. */
#define SIM_PDD_GPIOC9_SCLK0   0U                /**< GPIOC9_SCLK0 function. */
#define SIM_PDD_GPIOC9_XB_IN4  0x4U              /**< GPIOC9_XB_IN4 function. */
#define SIM_PDD_GPIOC9_TXD0    0x8U              /**< GPIOC9_TXD0 function. */
#define SIM_PDD_GPIOC9_XB_OUT8 0xCU              /**< GPIOC9_XB_OUT8 function. */

/* GPIO C10 peripheral function constants. */
#define SIM_PDD_GPIOC10_MOSI0   0U               /**< GPIOC10_MOSI0 function. */
#define SIM_PDD_GPIOC10_XB_IN5  0x10U            /**< GPIOC10_XB_IN5 function. */
#define SIM_PDD_GPIOC10_MISO0   0x20U            /**< GPIOC10_MISO0 function. */
#define SIM_PDD_GPIOC10_XB_OUT9 0x30U            /**< GPIOC10_XB_OUT9 function. */

/* GPIO C11 peripheral function constants. */
#define SIM_PDD_GPIOC11_CANTX; 0U                /**< GPIOC11_CANTX; function. */
#define SIM_PDD_GPIOC11_SCL0   0x40U             /**< GPIOC11_SCL0 function. */
#define SIM_PDD_GPIOC11_TXD1   0x80U             /**< GPIOC11_TXD1 function. */

/* GPIO C12 peripheral function constants. */
#define SIM_PDD_GPIOC12_CANRX 0U                 /**< GPIOC12_CANRX function. */
#define SIM_PDD_GPIOC12_SDA0  0x100U             /**< GPIOC12_SDA0 function. */
#define SIM_PDD_GPIOC12_RXD1  0x200U             /**< GPIOC12_RXD1 function. */

/* GPIO C13 peripheral function constants. */
#define SIM_PDD_GPIOC13_TA3       0U             /**< GPIOC13_TA3 function. */
#define SIM_PDD_GPIOC13_XB_IN6    0x400U         /**< GPIOC13_XB_IN6 function. */
#define SIM_PDD_GPIOC13_EWM_OUT_B 0x800U         /**< GPIOC13_EWM_OUT_B function. */

/* GPIO C14 peripheral function constants. */
#define SIM_PDD_GPIOC14_SDA0       0U            /**< GPIOC14_SDA0 function. */
#define SIM_PDD_GPIOC14_XB_OUT4    0x1000U       /**< GPIOC14_XB_OUT4 function. */
#define SIM_PDD_GPIOC14_PWM_FAULT4 0x2000U       /**< GPIOC14_PWM_FAULT4 function. */

/* GPIO C15 peripheral function constants. */
#define SIM_PDD_GPIOC15_SCL0       0U            /**< GPIOC15_SCL0 function. */
#define SIM_PDD_GPIOC15_XB_OUT5    0x4000U       /**< GPIOC15_XB_OUT5 function. */
#define SIM_PDD_GPIOC15_PWM_FAULT5 0x8000U       /**< GPIOC15_PWM_FAULT5 function. */

/* GPIO E4 peripheral function constants. */
#define SIM_PDD_GPIOE4_PWMA_2B 0U                /**< GPIOE4_PWMA_2B function. */
#define SIM_PDD_GPIOE4_XB_IN2  0x100U            /**< GPIOE4_XB_IN2 function. */

/* GPIO E5 peripheral function constants. */
#define SIM_PDD_GPIOE5_PWMA_2A 0U                /**< GPIOE5_PWMA_2A function. */
#define SIM_PDD_GPIOE5_XB_IN3  0x400U            /**< GPIOE5_XB_IN3 function. */

/* GPIO E6 peripheral function constants. */
#define SIM_PDD_GPIOE6_PWMA_3B 0U                /**< GPIOE6_PWMA_3B function. */
#define SIM_PDD_GPIOE6_XB_IN4  0x1000U           /**< GPIOE6_XB_IN4 function. */

/* GPIO E7 peripheral function constants. */
#define SIM_PDD_GPIOE7_PWMA_3A 0U                /**< GPIOE7_PWMA_3A function. */
#define SIM_PDD_GPIOE7_XB_IN5  0x4000U           /**< GPIOE7_XB_IN5 function. */

/* GPIO F0 peripheral function constants. */
#define SIM_PDD_GPIOF0_XB_IN6 0U                 /**< GPIOF0_XB_IN6 function. */
#define SIM_PDD_GPIOF0_SCLK1  0x2U               /**< GPIOF0_SCLK1 function. */

/* GPIO F1 peripheral function constants. */
#define SIM_PDD_GPIOF1_CLKOUT1 0U                /**< GPIOF1_CLKOUT1 function. */
#define SIM_PDD_GPIOF1_XB_IN7  0x4U              /**< GPIOF1_XB_IN7 function. */
#define SIM_PDD_GPIOF1_CMPD_O  0x8U              /**< GPIOF1_CMPD_O function. */

/* GPIO F2 peripheral function constants. */
#define SIM_PDD_GPIOF2_SCL0    0U                /**< GPIOF2_SCL0 function. */
#define SIM_PDD_GPIOF2_XB_OUT6 0x10U             /**< GPIOF2_XB_OUT6 function. */
#define SIM_PDD_GPIOF2_MISO1   0x20U             /**< GPIOF2_MISO1 function. */

/* GPIO F3 peripheral function constants. */
#define SIM_PDD_GPIOF3_SDA0    0U                /**< GPIOF3_SDA0 function. */
#define SIM_PDD_GPIOF3_XB_OUT7 0x40U             /**< GPIOF3_XB_OUT7 function. */
#define SIM_PDD_GPIOF3_MOSI1   0x80U             /**< GPIOF3_MOSI1 function. */

/* GPIO F4 peripheral function constants. */
#define SIM_PDD_GPIOF4_TXD1        0U            /**< GPIOF4_TXD1 function. */
#define SIM_PDD_GPIOF4_XB_OUT8     0x100U        /**< GPIOF4_XB_OUT8 function. */
#define SIM_PDD_GPIOF4_PWMA_0X     0x200U        /**< GPIOF4_PWMA_0X function. */
#define SIM_PDD_GPIOF4_PWMA_FAULT6 0x300U        /**< GPIOF4_PWMA_FAULT6 function. */

/* GPIO F5 peripheral function constants. */
#define SIM_PDD_GPIOF5_RXD1        0U            /**< GPIOF5_RXD1 function. */
#define SIM_PDD_GPIOF5_XB_OUT9     0x400U        /**< GPIOF5_XB_OUT9 function. */
#define SIM_PDD_GPIOF5_PWMA_1x     0x800U        /**< GPIOF5_PWMA_1x function. */
#define SIM_PDD_GPIOF5_PWMA_FAULT7 0xC00U        /**< GPIOF5_PWMA_FAULT7 function. */

/* GPIO F6 peripheral function constants. */
#define SIM_PDD_GPIOF6_PWMA_3X 0x1000U           /**< GPIOF6_PWMA_3X function. */
#define SIM_PDD_GPIOF6_XB_IN2  0x3000U           /**< GPIOF6_XB_IN2 function. */

/* GPIO F7 peripheral function constants. */
#define SIM_PDD_GPIOF7_CMPC_O 0x4000U            /**< GPIOF7_CMPC_O function. */
#define SIM_PDD_GPIOF7_SS1_B  0x8000U            /**< GPIOF7_SS1_B function. */
#define SIM_PDD_GPIOF7_XB_IN3 0xC000U            /**< GPIOF7_XB_IN3 function. */

/* GPIO F8 peripheral function constants. */
#define SIM_PDD_GPIOF8_RXD0     0U               /**< GPIOA0_RXD0 function. */
#define SIM_PDD_GPIOA0_XB_OUT10 0x1U             /**< GPIOA0_XB_OUT10 function. */
#define SIM_PDD_GPIOA0_CMPD_O   0x2U             /**< GPIOA0_CMPD_O function. */
#define SIM_PDD_GPIOA0_PWMA_2X  0x3U             /**< GPIOA0_PWMA_2x function. */

/* TMR internal peripheral function channel mask constants. Used to select TMR
   channel. */
#define SIM_PDD_PERIPHERAL_FUNCTION_TMRA0 SIM_IPSn_TA0_MASK /**< TMRA channel 0. */
#define SIM_PDD_PERIPHERAL_FUNCTION_TMRA1 SIM_IPSn_TA1_MASK /**< TMRA channel 1. */
#define SIM_PDD_PERIPHERAL_FUNCTION_TMRA2 SIM_IPSn_TA2_MASK /**< TMRA channel 2. */
#define SIM_PDD_PERIPHERAL_FUNCTION_TMRA3 SIM_IPSn_TA3_MASK /**< TMRA channel 3. */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/* SCI internal peripheral function channel mask constants. Used to select SCI
   channel. */
  #define SIM_PDD_PERIPHERAL_FUNCTION_SCI0_RXD SIM_IPSn_SCI0_MASK /**< SCI0 peripheral. */

#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/* SCI internal peripheral function channel mask constants. Used to select SCI
   channel. */
  #define SIM_PDD_PERIPHERAL_FUNCTION_SCI0_RXD SIM_IPSn_SCI0_MASK /**< SCI0 peripheral. */
  #define SIM_PDD_PERIPHERAL_FUNCTION_SCI1_RXD SIM_IPSn_SCI1_MASK /**< SCI1 peripheral. */

#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/* OCCS CLKIN source pin constants. */
#define SIM_PDD_OCCS_CLKIN0 0U                   /**< CLKIN0 pin. */
#define SIM_PDD_OCCS_CLKIN1 SIM_MISC0_CLKINSEL_MASK /**< CLKIN1 pin. */

/* Master PIT. */
#define SIM_PDD_MASTER_PIT0 0U                   /**< PIT0 as master and PIT1 as slave. */
#define SIM_PDD_MASTER_PIT1 SIM_MISC0_PIT_MSTR_MASK /**< PIT1 as master and PIT0 as slave. */

/* TMR peripheral software reset mask constant. */
#define SIM_PDD_RESET_TMRA SIM_PSWR0_TA_MASK     /**< TMRA reset. */

/* GPIO peripheral software reset mask constant. */
#define SIM_PDD_RESET_GPIO SIM_PSWR0_GPIO_MASK   /**< GPIO reset. */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/* QSCI peripherals software reset mask constants. Used to specify which QSCI
   peripheral is (in) reset. */
  #define SIM_PDD_RESET_QSCI0 SIM_PSWR1_SCI0_MASK  /**< QSCI0 reset. */

#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/* QSCI peripherals software reset mask constants. Used to specify which QSCI
   peripheral is (in) reset. */
  #define SIM_PDD_RESET_QSCI0 SIM_PSWR1_SCI0_MASK  /**< QSCI0 reset. */
  #define SIM_PDD_RESET_QSCI1 SIM_PSWR1_SCI1_MASK  /**< QSCI1 reset. */

#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/* QSPI peripherals software reset mask constants. Used to specify which QSPI
   peripheral is (in) reset. */
  #define SIM_PDD_RESET_QSPI0 SIM_PSWR1_QSPI0_MASK /**< QSPI0 reset. */
  #define SIM_PDD_RESET_QSPI1 SIM_PSWR1_QSPI1_MASK /**< QSPI1 reset. */

#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/* QSPI peripherals software reset mask constants. Used to specify which QSPI
   peripheral is (in) reset. */
  #define SIM_PDD_RESET_QSPI0 SIM_PSWR1_QSPI0_MASK /**< QSPI0 reset. */

#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/* IIC peripheral software reset mask constant. */
#define SIM_PDD_RESET_IIC0 SIM_PSWR1_IIC0_MASK   /**< IIC0 reset. */

/* EWM peripheral software reset mask constant. */
#define SIM_PDD_RESET_EWM SIM_PSWR2_EWM_MASK     /**< EWM reset. */

/* CMP peripheral software reset mask constant. */
#define SIM_PDD_RESET_CMP SIM_PSWR2_CMP_MASK     /**< CMP reset. */

/* Cyclic 12-bit ADC peripheral software reset mask constant. */
#define SIM_PDD_RESET_ADC12 SIM_PSWR2_CYCADC_MASK /**< Cyclic 12-bit ADC reset. */

/* CRC peripheral software reset mask constant. */
#define SIM_PDD_RESET_CRC SIM_PSWR2_CRC_MASK     /**< CRC reset. */

/* PIT peripherals software reset mask constants. Used to specify which PIT
   peripheral is (in) reset. */
#define SIM_PDD_RESET_PIT0 SIM_PSWR2_PIT0_MASK   /**< PIT0 reset. */
#define SIM_PDD_RESET_PIT1 SIM_PSWR2_PIT1_MASK   /**< PIT1 reset. */

/* PWM peripheral software reset mask constant. */
#define SIM_PDD_RESET_PWMA SIM_PSWR3_PWMA_MASK   /**< PWMA reset. */

/* Power modes mask constants. */
#define SIM_PDD_LOW_POWER_MODE      SIM_PWRMODE_LPMS_MASK /**< Low power mode. */
#define SIM_PDD_VERY_LOW_POWER_MODE SIM_PWRMODE_VLPMS_MASK /**< Very low power mode. */

/* DAC peripherals clock gate mask constants. Used to specify DAC peripherals of
   which clock gate will be enbaled or disabled. */
#define SIM_PDD_CLOCK_GATE_DACA_MASK SIM_PCE1_DACA_MASK /**< DACA clock gate. */
#define SIM_PDD_CLOCK_GATE_DACB_MASK SIM_PCE1_DACB_MASK /**< DACB clock gate. */

/* DAC peripherals software reset mask constants. Used to specify which DAC
   peripheral is (in) reset. */
#define SIM_PDD_RESET_DACA SIM_PSWR1_DACA_MASK   /**< DACA reset. */
#define SIM_PDD_RESET_DACB SIM_PSWR1_DACB_MASK   /**< DACB reset. */

/* MSCAN peripheral software reset mask constant. */
#define SIM_PDD_RESET_MSCAN SIM_PSWR1_MSCAN_MASK /**< MSCAN reset. */

/* STOP and WAIT instructions enable constants. */
#define SIM_PDD_INSTRUCTION_ENABLE            0U /**< Stop mode is entered when the DSC core executes a STOP or WAIT instruction. */
#define SIM_PDD_INSTRUCTION_DISABLE           0x1U /**< The DSC core STOP or WAIT instruction does not cause entry into stop mode. */
#define SIM_PDD_INSTRUCTION_ENABLE_PROTECTED  0x2U /**< Stop mode is entered when the DSC core executes a STOP or WAIT instruction and this state is write protected until the next reset. */
#define SIM_PDD_INSTRUCTION_DISABLE_PROTECTED 0x3U /**< The DSC core STOP or WAIT instruction does not cause entry into stop mode and this state is write protected until the next reset. */

/* Power supply control constants. */
#define SIM_PDD_POWER_ENABLE            0U       /**< Regulator supply placed in normal mode. */
#define SIM_PDD_POWER_DISABLE           0x1U     /**< Regulator supply placed in standby mode. */
#define SIM_PDD_POWER_ENABLE_PROTECTED  0x2U     /**< Regulator supply placed in normal mode and this state is write protected until chip reset. */
#define SIM_PDD_POWER_DISABLE_PROTECTED 0x3U     /**< Regulator supply placed in standby mode and this state is write protected until chip reset. */

/* Clock output source constants. */
#define SIM_PDD_SYS_CLK      0U                  /**< SYS_CLK continuous after reset. */
#define SIM_PDD_MSTR_2X_CLK  0x1U                /**< MSTR_2X_CLK continuous after reset. */
#define SIM_PDD_DIV2_BUS_CLK 0x2U                /**< DIV2_BUS_CLK continuous after reset. */
#define SIM_PDD_MSTR_OSC     0x3U                /**< MSTR_OSC (master clock) continuous after reset. */
#define SIM_PDD_ROSC_8M      0x4U                /**< ROSC_8M (8 MHz / 400 kHz relaxation oscillator clock). */
#define SIM_PDD_ROSC_200K    0x5U                /**< ROSC_200K (200 kHz relaxation oscillator clock). */

/* Clock rate speed constants. */
#define SIM_PDD_CLOCK_RATE_BUS_CLK    0U         /**< Clock rate equals bus clock rate. */
#define SIM_PDD_CLOCK_RATE_2X_BUS_CLK 0x1U       /**< Clock rate equals 2X bus clock rate. */

/* Write protection constants. */
#define SIM_PDD_PROTECTION_DISABLE        0U     /**< Write protection off. */
#define SIM_PDD_PROTECTION_ENABLE         0x1U   /**< Write protection on. */
#define SIM_PDD_PROTECTION_DISABLE_LOCKED 0x2U   /**< Write protection off and locked until chip reset. */
#define SIM_PDD_PROTECTION_ENABLE_LOCKED  0x3U   /**< Write protection on and locked until chip reset. */

/* System speed mode constants. */
#define SIM_PDD_NORMAL_MODE 0U                   /**< Device in normal operating mode. Core:bus frequency ratio is 1:1. */
#define SIM_PDD_FAST_MODE   0x1U                 /**< Device in fast mode. Core:bus frequency ratio is 2:1. */


/* ----------------------------------------------------------------------------
   -- EnableExternalResetPadcellFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable external reset padcell input filter.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if external reset padcell input filter is
 *        enabled or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableExternalResetPadcellFilter(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableExternalResetPadcellFilter(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_RST_FILT_MASK)) : ( \
      PeriphSetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_RST_FILT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalResetPadcellFilterEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns external reset padcell input filter enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetExternalResetPadcellFilterEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetExternalResetPadcellFilterEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_RST_FILT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetDmaEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets whether DMA is enabled in RUN, WAIT, RUN and WAIT, or all modes.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying DMA configuration. Use constants from group
 *        "DMA configuration constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetDmaEnable(deviceID, SIM_PDD_DMA_DISABLE);
 *      @endcode
 */
#define SIM_PDD_SetDmaEnable(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_CTRL_REG(PeripheralBase), \
     SIM_CTRL_DMAEbl_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether DMA is enabled in RUN, WAIT, RUN and WAIT, or all
 * modes.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DMA configuration constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetDmaEnable(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetDmaEnable(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_DMAEbl_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOnce
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable OnCE.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if OnCE is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableOnce(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableOnce(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_OnceEbl_MASK)) : ( \
      PeriphSetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_OnceEbl_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOnceEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns OnCE enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetOnceEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetOnceEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_OnceEbl_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetStopInstructionEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets STOP instruction enable mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying STOP instruction enable mode. This
 *        parameter is of "STOP and WAIT instructions enable constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetStopInstructionEnable(deviceID, SIM_PDD_INSTRUCTION_ENABLE);
 *      @endcode
 */
#define SIM_PDD_SetStopInstructionEnable(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_CTRL_REG(PeripheralBase), \
     SIM_CTRL_STOP_disable_MASK, \
     (uint16_t)((uint16_t)(State) << SIM_CTRL_STOP_disable_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetStopInstructionEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns STOP instruction enable mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetStopInstructionEnable(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetStopInstructionEnable(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_STOP_disable_MASK)) >> ( \
     SIM_CTRL_STOP_disable_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetWaitInstructionEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets WAIT instruction enable mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying WAIT instruction enable mode. This
 *        parameter is of "STOP and WAIT instructions enable constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetWaitInstructionEnable(deviceID, SIM_PDD_INSTRUCTION_ENABLE);
 *      @endcode
 */
#define SIM_PDD_SetWaitInstructionEnable(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_CTRL_REG(PeripheralBase), \
     SIM_CTRL_WAIT_disable_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetWaitInstructionEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns WAIT instruction enable mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetWaitInstructionEnable(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetWaitInstructionEnable(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_WAIT_disable_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetResetStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns reset status flags.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Reset status constants." for processing
 *         return value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetResetStatusFlags(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetResetStatusFlags(PeripheralBase) ( \
    PeriphReadReg16(SIM_RSTAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSoftwareResetFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns SW reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetSoftwareResetFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetSoftwareResetFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_SWR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetWatchdogWindowTimeoutResetFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns COP window time-out reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetWatchdogWindowTimeoutResetFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetWatchdogWindowTimeoutResetFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_COP_WNDOW_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetWatchdogCpuTimeoutResetFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns COP CPU time-out reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetWatchdogCpuTimeoutResetFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetWatchdogCpuTimeoutResetFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_COP_CPU_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetWatchdogLossOfReferenceFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns COP loss of reference reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetWatchdogLossOfReferenceFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetWatchdogLossOfReferenceFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_COP_LOR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalResetFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns external reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetExternalResetFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetExternalResetFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_EXTR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerOnResetFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns power-on reset flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPowerOnResetFlag(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPowerOnResetFlag(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_RSTAT_REG(PeripheralBase), SIM_RSTAT_POR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetJtagIdHigh
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns most significant half of JTAG ID.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_MSHID.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetJtagIdHigh(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetJtagIdHigh(PeripheralBase) ( \
    PeriphReadReg16(SIM_MSHID_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetJtagIdLow
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns least significant half of JTAG ID.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_LSHID.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetJtagIdLow(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetJtagIdLow(PeripheralBase) ( \
    PeriphReadReg16(SIM_LSHID_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- Set12VRegulatorStanbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets small regulator 1.2 V supply standby control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying small regulator 1.2 V supply standby
 *        control mode. This parameter is of "Power supply control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      SIM_PDD_Set12VRegulatorStanbyControl(deviceID, SIM_PDD_POWER_ENABLE);
 *      @endcode
 */
#define SIM_PDD_Set12VRegulatorStanbyControl(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_PWR_REG(PeripheralBase), \
     SIM_PWR_SR12STDBY_MASK, \
     (uint16_t)((uint16_t)(State) << SIM_PWR_SR12STDBY_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- Get12VRegulatorStanbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns small regulator 1.2 V supply standby control mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_Get12VRegulatorStanbyControl(deviceID);
 *      @endcode
 */
#define SIM_PDD_Get12VRegulatorStanbyControl(PeripheralBase) ( \
    (uint8_t)(PeriphGetBitMask16(SIM_PWR_REG(PeripheralBase), SIM_PWR_SR12STDBY_MASK) >> 12U) \
  )

/* ----------------------------------------------------------------------------
   -- Set27VRegulatorPowerdownControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets small regulator 2.7 V supply powerdown control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying small regulator 2.7 V supply powerdown
 *        control mode. This parameter is of "Power supply control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      SIM_PDD_Set27VRegulatorPowerdownControl(deviceID, SIM_PDD_POWER_ENABLE);
 *      @endcode
 */
#define SIM_PDD_Set27VRegulatorPowerdownControl(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_PWR_REG(PeripheralBase), \
     SIM_PWR_SR27PDN_MASK, \
     (uint16_t)((uint16_t)(State) << SIM_PWR_SR27PDN_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- Get27VRegulatorPowerdownControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns small regulator 2.7 V supply powerdown control mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_Get27VRegulatorPowerdownControl(deviceID);
 *      @endcode
 */
#define SIM_PDD_Get27VRegulatorPowerdownControl(PeripheralBase) ( \
    (uint8_t)(PeriphGetBitMask16(SIM_PWR_REG(PeripheralBase), SIM_PWR_SR27PDN_MASK) >> 8U) \
  )

/* ----------------------------------------------------------------------------
   -- Set27VRegulatorStandbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets small regulator 2.7 V supply standby control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying small regulator 2.7 V supply standby
 *        control mode. This parameter is of "Power supply control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      SIM_PDD_Set27VRegulatorStandbyControl(deviceID, SIM_PDD_POWER_ENABLE);
 *      @endcode
 */
#define SIM_PDD_Set27VRegulatorStandbyControl(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_PWR_REG(PeripheralBase), \
     SIM_PWR_SR27STDBY_MASK, \
     (uint16_t)((uint16_t)(State) << SIM_PWR_SR27STDBY_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- Get27VRegulatorStandbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns small regulator 2.7 V supply standby control mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_Get27VRegulatorStandbyControl(deviceID);
 *      @endcode
 */
#define SIM_PDD_Get27VRegulatorStandbyControl(PeripheralBase) ( \
    (uint8_t)(PeriphGetBitMask16(SIM_PWR_REG(PeripheralBase), SIM_PWR_SR27STDBY_MASK) >> 4U) \
  )

/* ----------------------------------------------------------------------------
   -- SetLargeRegulatorStandbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets large regulator standby control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying large regulator standby control mode. This
 *        parameter is of "Power supply control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetLargeRegulatorStandbyControl(deviceID, SIM_PDD_POWER_ENABLE);
 *      @endcode
 */
#define SIM_PDD_SetLargeRegulatorStandbyControl(PeripheralBase, State) ( \
    PeriphSetBits16( \
     SIM_PWR_REG(PeripheralBase), \
     SIM_PWR_LRSTDBY_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetLargeRegulatorStandbyControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns large regulator standby control mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetLargeRegulatorStandbyControl(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetLargeRegulatorStandbyControl(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(SIM_PWR_REG(PeripheralBase), SIM_PWR_LRSTDBY_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockOutputDivider
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets CLKOUT divide factor.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying CLKOUT divide factor. Use constants from
 *        group "CLKOUT divide factor constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockOutputDivider(deviceID, SIM_PDD_DIVIDE_1);
 *      @endcode
 */
#define SIM_PDD_SetClockOutputDivider(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     SIM_CLKOUT_REG(PeripheralBase), \
     SIM_CLKOUT_CLKODIV_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutputDivider
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns CLKOUT divide factor.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CLKOUT divide factor constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetClockOutputDivider(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockOutputDivider(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKODIV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableClockOutput1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable clock output 1.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock output 1 is enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableClockOutput1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableClockOutput1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS1_MASK)) : ( \
      PeriphClearBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutput1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns clock output 1 enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockOutput1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockOutput1Enabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS1_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockOutputSource1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock source for the CLKOUT1 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying clock source for the CLKOUT1 pin. This
 *        parameter is of "Clock output source constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockOutputSource1(deviceID, SIM_PDD_SYS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockOutputSource1(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     SIM_CLKOUT_REG(PeripheralBase), \
     SIM_CLKOUT_CLKOSEL1_MASK, \
     (uint16_t)((uint16_t)(Source) << SIM_CLKOUT_CLKOSEL1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutputSource1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns clock source for the CLKOUT1 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockOutputSource1(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockOutputSource1(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKOSEL1_MASK)) >> ( \
     SIM_CLKOUT_CLKOSEL1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableClockOutput0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable clock output 0.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock output 0 is enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableClockOutput0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableClockOutput0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS0_MASK)) : ( \
      PeriphClearBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutput0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns clock output 0 enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockOutput0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockOutput0Enabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKDIS0_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockOutputSource0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock source for the CLKOUT0 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying clock source for the CLKOUT0 pin. This
 *        parameter is of "Clock output source constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockOutputSource0(deviceID, SIM_PDD_SYS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockOutputSource0(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     SIM_CLKOUT_REG(PeripheralBase), \
     SIM_CLKOUT_CLKOSEL0_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutputSource0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns clock source for the CLKOUT0 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockOutputSource0(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockOutputSource0(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(SIM_CLKOUT_REG(PeripheralBase), SIM_CLKOUT_CLKOSEL0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockRateSci0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock speed for the SCI0 module.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock speed for the SCI0 module. This
 *        parameter is of "Clock rate speed constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockRateSci0(deviceID, SIM_PDD_CLOCK_RATE_BUS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockRateSci0(PeripheralBase, Source) ( \
    ((Source) == SIM_PDD_CLOCK_RATE_BUS_CLK) ? ( \
      PeriphClearBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI0_CR_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI0_CR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockRateSci0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock speed for the SCI0 module.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock rate speed constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockRateSci0(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockRateSci0(PeripheralBase) ( \
    (PeriphGetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI0_CR_MASK) == 0U) ? ( \
      SIM_PDD_CLOCK_RATE_BUS_CLK) : ( \
      SIM_PDD_CLOCK_RATE_2X_BUS_CLK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockRateTmr
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock speed for the TMR module.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock speed for the TMR module. This
 *        parameter is of "Clock rate speed constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockRateTmr(deviceID, SIM_PDD_CLOCK_RATE_BUS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockRateTmr(PeripheralBase, Source) ( \
    ((Source) == SIM_PDD_CLOCK_RATE_BUS_CLK) ? ( \
      PeriphClearBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_TMR_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_TMR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockRateTmr
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock speed for the TMR module.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock rate speed constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockRateTmr(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockRateTmr(PeripheralBase) ( \
    (PeriphGetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_TMR_MASK) == 0U) ? ( \
      SIM_PDD_CLOCK_RATE_BUS_CLK) : ( \
      SIM_PDD_CLOCK_RATE_2X_BUS_CLK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockRatePwm
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock speed for the PWM module.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock speed for the PWM module. This
 *        parameter is of "Clock rate speed constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockRatePwm(deviceID, SIM_PDD_CLOCK_RATE_BUS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockRatePwm(PeripheralBase, Source) ( \
    ((Source) == SIM_PDD_CLOCK_RATE_BUS_CLK) ? ( \
      PeriphClearBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_PWM_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_PWM_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockRatePwm
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock speed for the PWM module.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock rate speed constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockRatePwm(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockRatePwm(PeripheralBase) ( \
    (PeriphGetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_PWM_MASK) == 0U) ? ( \
      SIM_PDD_CLOCK_RATE_BUS_CLK) : ( \
      SIM_PDD_CLOCK_RATE_2X_BUS_CLK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockRateIicFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock speed for the IIC_FILTER module.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock speed for the IIC_FILTER module.
 *        This parameter is of "Clock rate speed constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockRateIicFilter(deviceID, SIM_PDD_CLOCK_RATE_BUS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockRateIicFilter(PeripheralBase, Source) ( \
    ((Source) == SIM_PDD_CLOCK_RATE_BUS_CLK) ? ( \
      PeriphClearBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_IIC_FILT_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_IIC_FILT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockRateIicFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock speed for the IIC_FILTER module.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock rate speed constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockRateIicFilter(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockRateIicFilter(PeripheralBase) ( \
    (PeriphGetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_IIC_FILT_MASK) == 0U) ? ( \
      SIM_PDD_CLOCK_RATE_BUS_CLK) : ( \
      SIM_PDD_CLOCK_RATE_2X_BUS_CLK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockTmrMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected TMR peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of TMR peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "TMR peripherals clock gate mask constants.
 *        Used to specify TMR peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of TMR peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "TMR peripherals clock gate mask constants. Used
 *        to specify TMR peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockTmrMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_TMRA0_MASK, SIM_PDD_CLOCK_GATE_TMRA0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockTmrMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR peripherals clock gate mask constants.
 *         Used to specify TMR peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE0_TA0_MASK) | (( \
      SIM_PCE0_TA1_MASK) | (( \
      SIM_PCE0_TA2_MASK) | ( \
      SIM_PCE0_TA3_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR peripherals clock gate mask constants.
 *         Used to specify TMR peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE0_TA0_MASK) | (( \
      SIM_PCE0_TA1_MASK) | (( \
      SIM_PCE0_TA2_MASK) | ( \
      SIM_PCE0_TA3_MASK)))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockTmrA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of TMRA0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockTmrA0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockTmrA0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrA0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of TMRA0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrA0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrA0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockTmrA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of TMRA1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockTmrA1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockTmrA1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA1_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrA1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of TMRA1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrA1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrA1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockTmrA2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of TMRA2 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockTmrA2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockTmrA2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA2_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA2_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrA2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of TMRA2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrA2Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrA2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockTmrA3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of TMRA3 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockTmrA3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockTmrA3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA3_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA3_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockTmrA3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of TMRA3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockTmrA3Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockTmrA3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_TA3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected GPIO peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of GPIO peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "GPIO peripherals clock gate mask constants.
 *        Used to specify GPIO peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of GPIO peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "GPIO peripherals clock gate mask constants.
 *        Used to specify GPIO peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_GPIOA_MASK, SIM_PDD_CLOCK_GATE_GPIOA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of GPIO peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO peripherals clock gate mask constants.
 *         Used to specify GPIO peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE0_GPIOA_MASK) | (( \
      SIM_PCE0_GPIOB_MASK) | (( \
      SIM_PCE0_GPIOC_MASK) | (( \
      SIM_PCE0_GPIOD_MASK) | (( \
      SIM_PCE0_GPIOE_MASK) | ( \
      SIM_PCE0_GPIOF_MASK)))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of GPIO peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO peripherals clock gate mask constants.
 *         Used to specify GPIO peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_PCE0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE0_GPIOA_MASK) | (( \
      SIM_PCE0_GPIOB_MASK) | (( \
      SIM_PCE0_GPIOC_MASK) | (( \
      SIM_PCE0_GPIOD_MASK) | (( \
      SIM_PCE0_GPIOE_MASK) | ( \
      SIM_PCE0_GPIOF_MASK)))))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOA_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOB_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioC
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioC(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioC(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOC_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioCEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioCEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioCEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioD
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOD peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioD(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioD(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOD_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioDEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioDEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioDEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioE
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOE peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioE(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioE(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOE_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioEEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOE.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioEEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioEEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockGpioF
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of GPIOF peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockGpioF(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockGpioF(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOF_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOF_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockGpioFEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of GPIOF.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockGpioFEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockGpioFEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE0_REG(PeripheralBase), SIM_PCE0_GPIOF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQsciMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected QSCI peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of QSCI peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSCI peripherals clock gate mask constants.
 *        Used to specify QSCI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of QSCI peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSCI peripherals clock gate mask constants.
 *        Used to specify QSCI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQsciMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_QSCI0_MASK, SIM_PDD_CLOCK_GATE_QSCI0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQsciMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQsciEnabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of QSCI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsciEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQsciEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI0_MASK) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of QSCI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsciEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQsciEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PCE1_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE1_SCI0_MASK | SIM_PCE1_SCI1_MASK) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQsciDisabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of QSCI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsciDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQsciDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI0_MASK)) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of QSCI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsciDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQsciDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_PCE1_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE1_SCI0_MASK | SIM_PCE1_SCI1_MASK) \
      )) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQsci0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of QSCI0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQsci0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQsci0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQsci0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of QSCI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsci0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockQsci0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQspiMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected QSPI peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of QSPI peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSPI peripherals clock gate mask constants.
 *        Used to specify QSPI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of QSPI peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSPI peripherals clock gate mask constants.
 *        Used to specify QSPI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQspiMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_QSPI0_MASK, SIM_PDD_CLOCK_GATE_QSPI0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQspiMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQspiEnabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of QSPI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspiEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQspiEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PCE1_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE1_QSPI0_MASK | SIM_PCE1_QSPI1_MASK) \
      ) \
    )
#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of QSPI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspiEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQspiEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI0_MASK) \
    )
#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQspiDisabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of QSPI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspiDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQspiDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_PCE1_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE1_QSPI0_MASK | SIM_PCE1_QSPI1_MASK) \
      )) \
    )
#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of QSPI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspiDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockQspiDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI0_MASK)) \
    )
#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQspi0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of QSPI0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQspi0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQspi0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQspi0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of QSPI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspi0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockQspi0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockIic0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of IIC0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockIic0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockIic0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_IIC0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_IIC0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockIic0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of selected IIC0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockIic0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockIic0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_IIC0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCmpMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected CMP peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of CMP peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "CMP peripherals clock gate mask constants.
 *        Used to specify CMP peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of CMP peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "CMP peripherals clock gate mask constants. Used
 *        to specify CMP peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCmpMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_CMPA_MASK, SIM_PDD_CLOCK_GATE_CMPA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCmpMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE2_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpEnabledMask
   ---------------------------------------------------------------------------- */

#if (defined(MCU_MC56F82313))
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE2_CMPA_MASK | SIM_PCE2_CMPB_MASK) \
      ) \
    )
#elif ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(( \
        SIM_PCE2_CMPA_MASK) | (( \
        SIM_PCE2_CMPB_MASK) | (( \
        SIM_PCE2_CMPC_MASK) | ( \
        SIM_PCE2_CMPD_MASK)))) \
      ) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE2_CMPA_MASK | (SIM_PCE2_CMPB_MASK | SIM_PCE2_CMPC_MASK)) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpDisabledMask
   ---------------------------------------------------------------------------- */

#if (defined(MCU_MC56F82313))
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE2_CMPA_MASK | SIM_PCE2_CMPB_MASK) \
      )) \
    )
#elif ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(( \
        SIM_PCE2_CMPA_MASK) | (( \
        SIM_PCE2_CMPB_MASK) | (( \
        SIM_PCE2_CMPC_MASK) | ( \
        SIM_PCE2_CMPD_MASK)))) \
      )) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_PCE2_REG(PeripheralBase), \
       (uint16_t)(SIM_PCE2_CMPA_MASK | (SIM_PCE2_CMPB_MASK | SIM_PCE2_CMPC_MASK)) \
      )) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCmpA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of CMPA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCmpA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCmpA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPA_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of CMPA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockCmpAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCmpB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of CMPB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCmpB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCmpB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPB_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of CMPB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockCmpBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockAdc12
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of 12-bit (cyclic) ADC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockAdc12(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockAdc12(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CYCADC_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CYCADC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockAdc12Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of 12-bit (cyclic) ADC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockAdc12Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockAdc12Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CYCADC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCrc
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of CRC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCrc(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCrc(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CRC_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CRC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCrcEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of CRC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCrcEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockCrcEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CRC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPitMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected PIT peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of PIT peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "PIT peripherals clock gate mask constants.
 *        Used to specify PIT peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of PIT peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "PIT peripherals clock gate mask constants. Used
 *        to specify PIT peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPitMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_PIT0_MASK, SIM_PDD_CLOCK_GATE_PIT0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPitMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE2_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPitEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PIT peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PIT peripherals clock gate mask constants.
 *         Used to specify PIT peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPitEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPitEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PCE2_REG(PeripheralBase), \
     (uint16_t)(SIM_PCE2_PIT0_MASK | SIM_PCE2_PIT1_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPitDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PIT peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PIT peripherals clock gate mask constants.
 *         Used to specify PIT peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPitDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPitDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_PCE2_REG(PeripheralBase), \
     (uint16_t)(SIM_PCE2_PIT0_MASK | SIM_PCE2_PIT1_MASK) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPit0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PIT0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPit0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPit0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPit0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PIT0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPit0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPit0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPit1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PIT1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPit1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPit1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT1_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPit1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PIT1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPit1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPit1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_PIT1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPwmMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected PWM peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of PWM peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "PWM peripherals clock gate mask constants.
 *        Used to specify PWM peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of PWM peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "PWM peripherals clock gate mask constants. Used
 *        to specify PWM peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPwmMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_PWMACH0_MASK, SIM_PDD_CLOCK_GATE_PWMACH0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPwmMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE3_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PWM peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PWM peripherals clock gate mask constants.
 *         Used to specify PWM peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PCE3_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE3_PWMACH0_MASK) | (( \
      SIM_PCE3_PWMACH1_MASK) | (( \
      SIM_PCE3_PWMACH2_MASK) | ( \
      SIM_PCE3_PWMACH3_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PWM peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PWM peripherals clock gate mask constants.
 *         Used to specify PWM peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_PCE3_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_PCE3_PWMACH0_MASK) | (( \
      SIM_PCE3_PWMACH1_MASK) | (( \
      SIM_PCE3_PWMACH2_MASK) | ( \
      SIM_PCE3_PWMACH3_MASK)))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPwmA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PWMA channel 0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPwmA0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPwmA0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH0_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmA0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PWMA channel 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmA0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmA0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPwmA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PWMA channel 1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPwmA1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPwmA1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH1_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmA1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PWMA channel 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmA1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmA1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPwmA2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PWMA channel 2 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPwmA2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPwmA2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH2_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH2_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmA2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PWMA channel 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmA2Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmA2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockPwmA3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of PWMA channel 3 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockPwmA3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockPwmA3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH3_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH3_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockPwmA3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of PWMA channel 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockPwmA3Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockPwmA3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE3_REG(PeripheralBase), SIM_PCE3_PWMACH3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockTmrMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected TMR peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of TMR peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "TMR peripherals clock gate mask constants.
 *        Used to specify TMR peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of TMR peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "TMR peripherals clock gate mask constants. Used
 *        to specify TMR peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockTmrMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_TMRA0_MASK, SIM_PDD_CLOCK_GATE_TMRA0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockTmrMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR peripherals clock gate mask constants.
 *         Used to specify TMR peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockTmrEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(SIM_SD0_TA0_MASK | (SIM_SD0_TA1_MASK | (SIM_SD0_TA2_MASK | SIM_SD0_TA3_MASK))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR peripherals clock gate mask constants.
 *         Used to specify TMR peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockTmrDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(SIM_SD0_TA0_MASK | (SIM_SD0_TA1_MASK | (SIM_SD0_TA2_MASK | SIM_SD0_TA3_MASK))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockTmrA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of TMRA0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockTmrA0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockTmrA0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrA0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of TMRA0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockTmrA0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrA0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockTmrA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of TMRA1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockTmrA1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockTmrA1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA1_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrA1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of TMRA1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockTmrA1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrA1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockTmrA2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of TMRA2 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockTmrA2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockTmrA2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA2_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA2_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrA2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of TMRA2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockTmrA2Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrA2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockTmrA3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of TMRA3 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockTmrA3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockTmrA3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA3_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA3_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockTmrA3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of TMRA3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockTmrA3Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockTmrA3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_TA3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected GPIO peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of GPIO peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "GPIO peripherals clock gate mask constants.
 *        Used to specify GPIO peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of GPIO peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "GPIO peripherals clock gate mask constants.
 *        Used to specify GPIO peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_GPIOA_MASK, SIM_PDD_CLOCK_GATE_GPIOA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of GPIO peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO peripherals clock gate mask constants.
 *         Used to specify GPIO peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockGpioEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_SD0_GPIOA_MASK) | (( \
      SIM_SD0_GPIOB_MASK) | (( \
      SIM_SD0_GPIOC_MASK) | (( \
      SIM_SD0_GPIOD_MASK) | (( \
      SIM_SD0_GPIOE_MASK) | ( \
      SIM_SD0_GPIOF_MASK)))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of GPIO peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO peripherals clock gate mask constants.
 *         Used to specify GPIO peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockGpioDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_SD0_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_SD0_GPIOA_MASK) | (( \
      SIM_SD0_GPIOB_MASK) | (( \
      SIM_SD0_GPIOC_MASK) | (( \
      SIM_SD0_GPIOD_MASK) | (( \
      SIM_SD0_GPIOE_MASK) | ( \
      SIM_SD0_GPIOF_MASK)))))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOA_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOB_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioC
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioC(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioC(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOC_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioCEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioCEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioCEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioD
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOD peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioD(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioD(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOD_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioDEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioDEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioDEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioE
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOE peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioE(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioE(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOE_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioEEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOE.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioEEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioEEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockGpioF
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of GPIOF peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockGpioF(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockGpioF(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOF_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOF_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockGpioFEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of GPIOF.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockGpioFEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockGpioFEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD0_REG(PeripheralBase), SIM_SD0_GPIOF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQsciMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected QSCI peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of QSCI peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSCI peripherals clock gate mask constants.
 *        Used to specify QSCI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of QSCI peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSCI peripherals clock gate mask constants.
 *        Used to specify QSCI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQsciMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_QSCI0_MASK, SIM_PDD_CLOCK_GATE_QSCI0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQsciMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQsciEnabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of QSCI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQsciEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQsciEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI0_MASK) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of QSCI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQsciEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQsciEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_SD1_REG(PeripheralBase), \
       (uint16_t)(SIM_SD1_SCI0_MASK | SIM_SD1_SCI1_MASK) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQsciDisabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of QSCI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQsciDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQsciDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI0_MASK)) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of QSCI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals clock gate mask constants.
 *         Used to specify QSCI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQsciDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQsciDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_SD1_REG(PeripheralBase), \
       (uint16_t)(SIM_SD1_SCI0_MASK | SIM_SD1_SCI1_MASK) \
      )) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQsci0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of QSCI0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQsci0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQsci0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQsci0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of QSCI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockQsci0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockQsci0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQspiMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected QSPI peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of QSPI peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSPI peripherals clock gate mask constants.
 *        Used to specify QSPI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of QSPI peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used.
 *        Use constants from group "QSPI peripherals clock gate mask constants.
 *        Used to specify QSPI peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQspiMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_QSPI0_MASK, SIM_PDD_CLOCK_GATE_QSPI0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQspiMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQspiEnabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of QSPI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQspiEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQspiEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_SD1_REG(PeripheralBase), \
       (uint16_t)(SIM_SD1_QSPI0_MASK | SIM_SD1_QSPI1_MASK) \
      ) \
    )
#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of QSPI peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQspiEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQspiEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI0_MASK) \
    )
#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQspiDisabledMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of QSPI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQspiDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQspiDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_SD1_REG(PeripheralBase), \
       (uint16_t)(SIM_SD1_QSPI0_MASK | SIM_SD1_QSPI1_MASK) \
      )) \
    )
#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of QSPI peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals clock gate mask constants.
 *         Used to specify QSPI peripherals of which clock gate will be enbaled
 *         or disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockQspiDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockQspiDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI0_MASK)) \
    )
#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQspi0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of QSPI0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQspi0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQspi0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQspi0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of QSPI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockQspi0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockQspi0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockIic0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of IIC0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockIic0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockIic0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_IIC0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_IIC0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockIic0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of selected IIC0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockIic0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockIic0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_IIC0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCmpMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected CMP peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of CMP peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "CMP peripherals clock gate mask constants.
 *        Used to specify CMP peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of CMP peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "CMP peripherals clock gate mask constants. Used
 *        to specify CMP peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCmpMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_CMPA_MASK, SIM_PDD_CLOCK_GATE_CMPA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCmpMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD2_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpEnabledMask
   ---------------------------------------------------------------------------- */

#if (defined(MCU_MC56F82313))
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(SIM_SD2_CMPA_MASK | SIM_SD2_CMPB_MASK) \
      ) \
    )
#elif ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(( \
        SIM_SD2_CMPA_MASK) | (( \
        SIM_SD2_CMPB_MASK) | (( \
        SIM_SD2_CMPC_MASK) | ( \
        SIM_SD2_CMPD_MASK)))) \
      ) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of CMP peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpEnabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpEnabledMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(SIM_SD2_CMPA_MASK | (SIM_SD2_CMPB_MASK | SIM_SD2_CMPC_MASK)) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpDisabledMask
   ---------------------------------------------------------------------------- */

#if (defined(MCU_MC56F82313))
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(SIM_SD2_CMPA_MASK | SIM_SD2_CMPB_MASK) \
      )) \
    )
#elif ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(( \
        SIM_SD2_CMPA_MASK) | (( \
        SIM_SD2_CMPB_MASK) | (( \
        SIM_SD2_CMPC_MASK) | ( \
        SIM_SD2_CMPD_MASK)))) \
      )) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of CMP peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "CMP peripherals clock gate mask constants.
 *         Used to specify CMP peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockCmpDisabledMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetPeripheralStopClockCmpDisabledMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_SD2_REG(PeripheralBase), \
       (uint16_t)(SIM_SD2_CMPA_MASK | (SIM_SD2_CMPB_MASK | SIM_SD2_CMPC_MASK)) \
      )) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCmpA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of CMPA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCmpA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCmpA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPA_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of CMPA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockCmpAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockCmpAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCmpB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of CMPB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCmpB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCmpB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPB_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of CMPB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockCmpBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockCmpBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockAdc12
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of 12-bit (cyclic) ADC
 * peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockAdc12(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockAdc12(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CYCADC_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CYCADC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockAdc12Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of 12-bit (cyclic) ADC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockAdc12Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockAdc12Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CYCADC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCrc
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of CRC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCrc(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCrc(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CRC_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CRC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCrcEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of CRC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockCrcEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockCrcEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CRC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPitMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected PIT peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of PIT peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "PIT peripherals clock gate mask constants.
 *        Used to specify PIT peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of PIT peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "PIT peripherals clock gate mask constants. Used
 *        to specify PIT peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPitMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_PIT0_MASK, SIM_PDD_CLOCK_GATE_PIT0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPitMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD2_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPitEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PIT peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PIT peripherals clock gate mask constants.
 *         Used to specify PIT peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockPitEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPitEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_SD2_REG(PeripheralBase), \
     (uint16_t)(SIM_SD2_PIT0_MASK | SIM_SD2_PIT1_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPitDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PIT peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PIT peripherals clock gate mask constants.
 *         Used to specify PIT peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockPitDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPitDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_SD2_REG(PeripheralBase), \
     (uint16_t)(SIM_SD2_PIT0_MASK | SIM_SD2_PIT1_MASK) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPit0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PIT0 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPit0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPit0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPit0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PIT0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPit0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPit0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPit1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PIT1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPit1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPit1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT1_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPit1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PIT1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPit1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPit1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_PIT1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPwmMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected PWM peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of PWM peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "PWM peripherals clock gate mask constants.
 *        Used to specify PWM peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of PWM peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "PWM peripherals clock gate mask constants. Used
 *        to specify PWM peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPwmMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_PWMACH0_MASK, SIM_PDD_CLOCK_GATE_PWMACH0_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPwmMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD3_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PWM peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PWM peripherals clock gate mask constants.
 *         Used to specify PWM peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockPwmEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_SD3_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_SD3_PWMACH0_MASK) | (( \
      SIM_SD3_PWMACH1_MASK) | (( \
      SIM_SD3_PWMACH2_MASK) | ( \
      SIM_SD3_PWMACH3_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PWM peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PWM peripherals clock gate mask constants.
 *         Used to specify PWM peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockPwmDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_SD3_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_SD3_PWMACH0_MASK) | (( \
      SIM_SD3_PWMACH1_MASK) | (( \
      SIM_SD3_PWMACH2_MASK) | ( \
      SIM_SD3_PWMACH3_MASK)))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPwmA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PWMA channel 0
 * peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPwmA0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPwmA0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH0_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmA0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PWMA channel 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPwmA0Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmA0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPwmA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PWMA channel 1
 * peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPwmA1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPwmA1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH1_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmA1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PWMA channel 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPwmA1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmA1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPwmA2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PWMA channel 2
 * peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPwmA2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPwmA2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH2_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH2_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmA2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PWMA channel 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPwmA2Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmA2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockPwmA3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of PWMA channel 3
 * peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockPwmA3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockPwmA3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH3_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH3_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockPwmA3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of PWMA channel 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockPwmA3Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockPwmA3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD3_REG(PeripheralBase), SIM_SD3_PWMACH3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetIoShortAddressLocationHigh
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the I/O short address location high portion.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the I/O short address location high portion
 *        value. This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IOSAHI.
 * @par Example:
 *      @code
 *      SIM_PDD_SetIoShortAddressLocationHigh(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_SetIoShortAddressLocationHigh(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     SIM_IOSAHI_REG(PeripheralBase), \
     SIM_IOSAHI_ISAL_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetIoShortAddressLocationHigh
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the I/O short address location high portion value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_IOSAHI.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetIoShortAddressLocationHigh(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetIoShortAddressLocationHigh(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(SIM_IOSAHI_REG(PeripheralBase), SIM_IOSAHI_ISAL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetIoShortAddressLocationLow
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the I/O short address location low portion.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the I/O short address location low portion
 *        value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IOSALO.
 * @par Example:
 *      @code
 *      SIM_PDD_SetIoShortAddressLocationLow(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_SetIoShortAddressLocationLow(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_IOSALO_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIoShortAddressLocationLow
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the I/O short address location low portion value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_IOSALO.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetIoShortAddressLocationLow(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetIoShortAddressLocationLow(PeripheralBase) ( \
    PeriphReadReg16(SIM_IOSALO_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetIoShortAddressLocation
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the I/O short address location.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the I/O short address location value. This
 *        parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IOSAHI, SIM_IOSALO
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      SIM_PDD_SetIoShortAddressLocation(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_SetIoShortAddressLocation(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     SIM_IOSAHI_REG(PeripheralBase), \
     SIM_IOSAHI_ISAL_MASK, \
     (uint16_t)((uint32_t)(Value) >> 16U) \
    ), \
    PeriphWriteReg16(SIM_IOSALO_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIoShortAddressLocation
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the I/O short address locatiion value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: SIM_IOSALO, SIM_IOSAHI
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result = SIM_PDD_GetIoShortAddressLocation(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetIoShortAddressLocation(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(SIM_IOSALO_REG(PeripheralBase))) | ( \
     (uint32_t)(( \
      (uint32_t)PeriphGetBitMask16(SIM_IOSAHI_REG(PeripheralBase), SIM_IOSAHI_ISAL_MASK)) << ( \
      16U)))) \
  )

/* ----------------------------------------------------------------------------
   -- SetPowerModeControlProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of the PWRMODE register.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the PWRMODE
 *        register. This parameter is of "Write protection constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPowerModeControlProtection(deviceID,
 *      SIM_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define SIM_PDD_SetPowerModeControlProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     SIM_PROT_REG(PeripheralBase), \
     SIM_PROT_PMODE_MASK, \
     (uint16_t)((uint16_t)(Protection) << SIM_PROT_PMODE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerModeControlProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the PWRMODE register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetPowerModeControlProtection(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPowerModeControlProtection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(SIM_PROT_REG(PeripheralBase), SIM_PROT_PMODE_MASK)) >> ( \
     SIM_PROT_PMODE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortDProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of the GPIO port D GPIO_D_PER, GPIO_D_PPMODE,
 * and GPIO_D_DRIVE registers and also GPSDL register and CTRL[RST_FILT] bit.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the GPIO port
 *        D registers. This parameter is of "Write protection constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPortDProtection(deviceID, SIM_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define SIM_PDD_SetPortDProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     SIM_PROT_REG(PeripheralBase), \
     SIM_PROT_GDP_MASK, \
     (uint16_t)((uint16_t)(Protection) << SIM_PROT_GDP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPortDProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the GPIO port D registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetPortDProtection(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPortDProtection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(SIM_PROT_REG(PeripheralBase), SIM_PROT_GDP_MASK)) >> ( \
     SIM_PROT_GDP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralClockEnableProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of all fields in the PCEn, SDn, PSWRn, and PCR
 * registers.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the peripheral
 *        clock enable registers. This parameter is of "Write protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralClockEnableProtection(deviceID,
 *      SIM_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralClockEnableProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     SIM_PROT_REG(PeripheralBase), \
     SIM_PROT_PCEP_MASK, \
     (uint16_t)((uint16_t)(Protection) << SIM_PROT_PCEP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockEnableProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the peripheral clock enable registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetPeripheralClockEnableProtection(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockEnableProtection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(SIM_PROT_REG(PeripheralBase), SIM_PROT_PCEP_MASK)) >> ( \
     SIM_PROT_PCEP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetGpioInternalPeripheralSelectProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of GPSn and IPSn registers. Also write protect
 * registers in all XBAR, AOI, GPIOn_PER, GPIOn_PPMODE, and GPIOn_DRIVE registers.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the GPIO and
 *        IPS registers. This parameter is of "Write protection constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      SIM_PDD_SetGpioInternalPeripheralSelectProtection(deviceID,
 *      SIM_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define SIM_PDD_SetGpioInternalPeripheralSelectProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     SIM_PROT_REG(PeripheralBase), \
     SIM_PROT_GIPSP_MASK, \
     (uint16_t)(Protection) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetGpioInternalPeripheralSelectProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the GPIO and IPS registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      SIM_PDD_GetGpioInternalPeripheralSelectProtection(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetGpioInternalPeripheralSelectProtection(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(SIM_PROT_REG(PeripheralBase), SIM_PROT_GIPSP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO A0 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO A0 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSAL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioA0(deviceID,
 *      SIM_PDD_GPIOA0_ANA0_CMPA3);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioA0(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSAL_REG(PeripheralBase), \
     SIM_GPSAL_A0_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO A0 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO A0 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSAL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioA0(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioA0(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSAL_REG(PeripheralBase), SIM_GPSAL_A0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioB1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO B1 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO B1 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSBL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioB1(deviceID,
 *      SIM_PDD_GPIOB1_ANB1_CMPB_IN0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioB1(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSBL_REG(PeripheralBase), \
     SIM_GPSBL_B1_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioB1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO B1 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO B1 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSBL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioB1(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioB1(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSBL_REG(PeripheralBase), SIM_GPSBL_B1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C0 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C0 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC0(deviceID, SIM_PDD_GPIOC0_EXTAL);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC0(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C0_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C0 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C0 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC0(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC0(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC2
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C2 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C2 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC2(deviceID, SIM_PDD_GPIOC2_TXD0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC2(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C2_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C2 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C2 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC2(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC2(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC3
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C3 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C3 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC3(deviceID, SIM_PDD_GPIOC3_TA0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC3(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C3_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C3 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C3 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC3(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC3(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC4
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C4 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C4 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC4(deviceID, SIM_PDD_GPIOC4_TA1);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC4(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C4_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C4 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C4 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC4(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC4(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C4_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC5
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C5 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C5 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC5(deviceID, SIM_PDD_GPIOC5_DACA);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC5(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C5_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C5 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C5 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC5(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC5(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C5_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC6
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C6 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C6 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC6(deviceID, SIM_PDD_GPIOC6_TA2);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC6(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C6_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C6 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C6 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC6(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC6(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C6_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC7
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C7 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C7 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC7(deviceID, SIM_PDD_GPIOC7_SS0_B);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC7(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCL_REG(PeripheralBase), \
     SIM_GPSCL_C7_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC7
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C7 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C7 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC7(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC7(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCL_REG(PeripheralBase), SIM_GPSCL_C7_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC8
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C8 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C8 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC8(deviceID, SIM_PDD_GPIOC8_MISO0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC8(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C8_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C8 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C8 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC8(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC8(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C8_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC9
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C9 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C9 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC9(deviceID, SIM_PDD_GPIOC9_SCLK0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC9(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C9_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC9
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C9 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C9 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC9(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC9(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C9_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC10
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C10 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C10 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC10(deviceID, SIM_PDD_GPIOC10_MOSI0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC10(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C10_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC10
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C10 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C10 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC10(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC10(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C10_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC11
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C11 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C11 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC11(deviceID, SIM_PDD_GPIOC11_CANTX;);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC11(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C11_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC11
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C11 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C11 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC11(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC11(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C11_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC12
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C12 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C12 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC12(deviceID, SIM_PDD_GPIOC12_CANRX);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC12(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C12_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC12
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C12 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C12 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC12(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC12(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C12_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC13
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C13 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C13 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC13(deviceID, SIM_PDD_GPIOC13_TA3);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC13(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C13_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC13
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C13 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C13 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC13(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC13(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C13_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC14
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C14 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C14 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC14(deviceID, SIM_PDD_GPIOC14_SDA0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC14(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C14_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC14
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C14 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C14 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC14(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC14(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C14_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioC15
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO C15 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO C15 peripheral function constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioC15(deviceID, SIM_PDD_GPIOC15_SCL0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioC15(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSCH_REG(PeripheralBase), \
     SIM_GPSCH_C15_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioC15
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO C15 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO C15 peripheral function constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioC15(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioC15(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSCH_REG(PeripheralBase), SIM_GPSCH_C15_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioE4
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO E4 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO E4 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioE4(deviceID, SIM_PDD_GPIOE4_PWMA_2B);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioE4(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSEL_REG(PeripheralBase), \
     SIM_GPSEL_E4_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioE4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO E4 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO E4 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioE4(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioE4(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSEL_REG(PeripheralBase), SIM_GPSEL_E4_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioE5
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO E5 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO E5 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioE5(deviceID, SIM_PDD_GPIOE5_PWMA_2A);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioE5(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSEL_REG(PeripheralBase), \
     SIM_GPSEL_E5_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioE5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO E5 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO E5 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioE5(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioE5(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSEL_REG(PeripheralBase), SIM_GPSEL_E5_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioE6
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO E6 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO E6 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioE6(deviceID, SIM_PDD_GPIOE6_PWMA_3B);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioE6(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSEL_REG(PeripheralBase), \
     SIM_GPSEL_E6_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioE6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO E6 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO E6 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioE6(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioE6(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSEL_REG(PeripheralBase), SIM_GPSEL_E6_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioE7
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO E7 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO E7 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioE7(deviceID, SIM_PDD_GPIOE7_PWMA_3A);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioE7(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSEL_REG(PeripheralBase), \
     SIM_GPSEL_E7_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioE7
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO E7 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO E7 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioE7(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioE7(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSEL_REG(PeripheralBase), SIM_GPSEL_E7_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F0 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F0 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF0(deviceID, SIM_PDD_GPIOF0_XB_IN6);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF0(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F0_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F0 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F0 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF0(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF0(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F1 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F1 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF1(deviceID, SIM_PDD_GPIOF1_CLKOUT1);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF1(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F1_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F1 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F1 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF1(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF1(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF2
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F2 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F2 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF2(deviceID, SIM_PDD_GPIOF2_SCL0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF2(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F2_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F2 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F2 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF2(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF2(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF3
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F3 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F3 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF3(deviceID, SIM_PDD_GPIOF3_SDA0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF3(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F3_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F3 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F3 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF3(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF3(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF4
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F4 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F4 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF4(deviceID, SIM_PDD_GPIOF4_TXD1);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF4(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F4_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F4 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F4 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF4(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF4(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F4_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF5
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F5 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F5 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF5(deviceID, SIM_PDD_GPIOF5_RXD1);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF5(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F5_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F5 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F5 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF5(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF5(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F5_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF6
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F6 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F6 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF6(deviceID, SIM_PDD_GPIOF6_PWMA_3X);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF6(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F6_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F6 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F6 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF6(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF6(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F6_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF7
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F7 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F7 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF7(deviceID, SIM_PDD_GPIOF7_CMPC_O);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF7(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFL_REG(PeripheralBase), \
     SIM_GPSFL_F7_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF7
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F7 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F7 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF7(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF7(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFL_REG(PeripheralBase), SIM_GPSFL_F7_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPeripheralFunctionGpioF8
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral function of the GPIO F8 pin.
 * @param PeripheralBase Peripheral base address.
 * @param Function Parameter specifying pin function. Use constants from group
 *        "GPIO F8 peripheral function constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFH.
 * @par Example:
 *      @code
 *      SIM_PDD_SetPeripheralFunctionGpioF8(deviceID, SIM_PDD_GPIOF8_RXD0);
 *      @endcode
 */
#define SIM_PDD_SetPeripheralFunctionGpioF8(PeripheralBase, Function) ( \
    PeriphSetBits16( \
     SIM_GPSFH_REG(PeripheralBase), \
     SIM_GPSFH_F8_MASK, \
     (uint16_t)(Function) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralFunctionGpioF8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral function of the GPIO F8 pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "GPIO F8 peripheral function constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_GPSFH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralFunctionGpioF8(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralFunctionGpioF8(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_GPSFH_REG(PeripheralBase), SIM_GPSFH_F8_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetInternalPeripheralFunctionTmrMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets input peripheral function of the selected TMR peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param GpioFunctionMask Mask of TMR channels with GPIO function on input.
 *        Logical or of associated mask constants can be used. Use constants from
 *        group "TMR internal peripheral function channel mask constants. Used to
 *        select TMR channel.". This parameter is 16 bits wide.
 * @param XbarFunctionMask Mask of TMR channels with XBAR function on input.
 *        Logical or of associated mask constants can be used. Use constants from
 *        group "TMR internal peripheral function channel mask constants. Used to
 *        select TMR channel.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      SIM_PDD_SetInternalPeripheralFunctionTmrMask(deviceID,
 *      SIM_PDD_PERIPHERAL_FUNCTION_TMRA0, SIM_PDD_PERIPHERAL_FUNCTION_TMRA0);
 *      @endcode
 */
#define SIM_PDD_SetInternalPeripheralFunctionTmrMask(PeripheralBase, GpioFunctionMask, XbarFunctionMask) ( \
    PeriphSetBits16( \
     SIM_IPSn_REG(PeripheralBase), \
     (uint16_t)(GpioFunctionMask), \
     (uint16_t)(XbarFunctionMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInternalPeripheralFunctionGpioTmrMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR channels with GPIO internal peripheral function on
 * their input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR internal peripheral function channel
 *         mask constants. Used to select TMR channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetInternalPeripheralFunctionGpioTmrMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetInternalPeripheralFunctionGpioTmrMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_IPSn_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_IPSn_TA0_MASK) | (( \
      SIM_IPSn_TA1_MASK) | (( \
      SIM_IPSn_TA2_MASK) | ( \
      SIM_IPSn_TA3_MASK)))) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- GetInternalPeripheralFunctionXbarTmrMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of TMR channels with XBAR internal peripheral function on
 * their input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "TMR internal peripheral function channel
 *         mask constants. Used to select TMR channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetInternalPeripheralFunctionXbarTmrMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetInternalPeripheralFunctionXbarTmrMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_IPSn_REG(PeripheralBase), \
     (uint16_t)(( \
      SIM_IPSn_TA0_MASK) | (( \
      SIM_IPSn_TA1_MASK) | (( \
      SIM_IPSn_TA2_MASK) | ( \
      SIM_IPSn_TA3_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetInternalPeripheralFunctionSciMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets input peripheral function of the selected SCI peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param GpioFunctionMask Mask of SCI peripherals with GPIO function on input.
 *        Logical or of associated mask constants can be used. Use constants from
 *        group "SCI internal peripheral function channel mask constants. Used
 *        to select SCI channel.". This parameter is 16 bits wide.
 * @param XbarFunctionMask Mask of SCI peripherals with XBAR function on input.
 *        Logical or of associated mask constants can be used. Use constants from
 *        group "SCI internal peripheral function channel mask constants. Used
 *        to select SCI channel.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      SIM_PDD_SetInternalPeripheralFunctionSciMask(deviceID,
 *      SIM_PDD_PERIPHERAL_FUNCTION_SCI0_RXD, SIM_PDD_PERIPHERAL_FUNCTION_SCI0_RXD);
 *      @endcode
 */
#define SIM_PDD_SetInternalPeripheralFunctionSciMask(PeripheralBase, GpioFunctionMask, XbarFunctionMask) ( \
    PeriphSetBits16( \
     SIM_IPSn_REG(PeripheralBase), \
     (uint16_t)(GpioFunctionMask), \
     (uint16_t)(XbarFunctionMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetGpioInternalPeripheralFunctionSciRxdMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of SCI peripherals with GPIO internal peripheral function
 * on their RXD input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "SCI internal peripheral function channel
 *         mask constants. Used to select SCI channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetGpioInternalPeripheralFunctionSciRxdMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetGpioInternalPeripheralFunctionSciRxdMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16(SIM_IPSn_REG(PeripheralBase), SIM_IPSn_SCI0_MASK)) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of SCI peripherals with GPIO internal peripheral function
 * on their RXD input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "SCI internal peripheral function channel
 *         mask constants. Used to select SCI channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetGpioInternalPeripheralFunctionSciRxdMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetGpioInternalPeripheralFunctionSciRxdMask(PeripheralBase) ( \
      (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
       SIM_IPSn_REG(PeripheralBase), \
       (uint16_t)(SIM_IPSn_SCI0_MASK | SIM_IPSn_SCI1_MASK) \
      )) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- GetXbarInternalPeripheralFunctionSciRxdMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of SCI peripherals with XBAR internal peripheral function
 * on their RXD input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "SCI internal peripheral function channel
 *         mask constants. Used to select SCI channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetXbarInternalPeripheralFunctionSciRxdMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetXbarInternalPeripheralFunctionSciRxdMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_IPSn_REG(PeripheralBase), SIM_IPSn_SCI0_MASK) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of SCI peripherals with XBAR internal peripheral function
 * on their RXD input.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "SCI internal peripheral function channel
 *         mask constants. Used to select SCI channel." for processing return value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetXbarInternalPeripheralFunctionSciRxdMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetXbarInternalPeripheralFunctionSciRxdMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_IPSn_REG(PeripheralBase), \
       (uint16_t)(SIM_IPSn_SCI0_MASK | SIM_IPSn_SCI1_MASK) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- EnableAdcScanControlReordering
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable the re-ordering of scan control bits of Cyclic ADC
 * for test channels.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the re-ordering of scan control bits of
 *        Cyclic ADC for test channels is enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableAdcScanControlReordering(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableAdcScanControlReordering(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_ADC_SCTRL_MASK)) : ( \
      PeriphSetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_ADC_SCTRL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAdcScanControlReorderingEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the re-ordering of scan control bits of Cyclic ADC for test
 * channels enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetAdcScanControlReorderingEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetAdcScanControlReorderingEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_ADC_SCTRL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetOccsClockInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets OCCS CLKIN source pin.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the CLKIN source pin. Use constants from
 *        group "OCCS CLKIN source pin constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      SIM_PDD_SetOccsClockInput(deviceID, SIM_PDD_OCCS_CLKIN0);
 *      @endcode
 */
#define SIM_PDD_SetOccsClockInput(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     SIM_MISC0_REG(PeripheralBase), \
     SIM_MISC0_CLKINSEL_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOccsClockInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns selected OCCS CLKIN source pin.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "OCCS CLKIN source pin constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetOccsClockInput(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetOccsClockInput(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_CLKINSEL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetMasterPit
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets master PIT.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the master PIT. Use constants from group
 *        "Master PIT.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      SIM_PDD_SetMasterPit(deviceID, SIM_PDD_MASTER_PIT0);
 *      @endcode
 */
#define SIM_PDD_SetMasterPit(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     SIM_MISC0_REG(PeripheralBase), \
     SIM_MISC0_PIT_MSTR_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetMasterPit
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns selected master PIT.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Master PIT." for processing return value.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetMasterPit(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetMasterPit(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_PIT_MSTR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetTmrA
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the TMRA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetTmrA(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetTmrA(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR0_REG(PeripheralBase), SIM_PSWR0_TA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetTmrAReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns TMRA software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetTmrAReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetTmrAReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR0_REG(PeripheralBase), SIM_PSWR0_TA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetGpio
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the GPIO.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetGpio(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetGpio(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR0_REG(PeripheralBase), SIM_PSWR0_GPIO_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetGpioReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns GPIO software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetGpioReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetGpioReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR0_REG(PeripheralBase), SIM_PSWR0_GPIO_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetQsciMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of specified QSCI peripherals. Peripherals are
 * specified by mask constant. Bits associated with peripherals not specified in
 * input mask stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of QSCI peripherals defining which peripherals will be
 *        reset. Logical or of associated mask constants can be used. Use constants
 *        from group "QSCI peripherals software reset mask constants. Used to
 *        specify which QSCI peripheral is (in) reset.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQsciMask(deviceID, SIM_PDD_RESET_QSCI0);
 *      @endcode
 */
#define SIM_PDD_ResetQsciMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetQsciResetMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82313)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82743)))
/**
 * @brief Returns mask of QSCI peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals software reset mask
 *         constants. Used to specify which QSCI peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQsciResetMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetQsciResetMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_SCI0_MASK) \
    )
#else /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */
/**
 * @brief Returns mask of QSCI peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSCI peripherals software reset mask
 *         constants. Used to specify which QSCI peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQsciResetMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetQsciResetMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PSWR1_REG(PeripheralBase), \
       (uint16_t)(SIM_PSWR1_SCI0_MASK | SIM_PSWR1_SCI1_MASK) \
      ) \
    )
#endif /* (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82728)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82746)) || (defined(MCU_MC56F82748)) */

/* ----------------------------------------------------------------------------
   -- ResetQsci0
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the QSCI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQsci0(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetQsci0(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_SCI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetQsci0Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns QSCI0 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQsci0Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetQsci0Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_SCI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetQspiMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of specified QSPI peripherals. Peripherals are
 * specified by mask constant. Bits associated with peripherals not specified in
 * input mask stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of QSPI peripherals defining which peripherals will be
 *        reset. Logical or of associated mask constants can be used. Use constants
 *        from group "QSPI peripherals software reset mask constants. Used to
 *        specify which QSPI peripheral is (in) reset.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQspiMask(deviceID, SIM_PDD_RESET_QSPI0);
 *      @endcode
 */
#define SIM_PDD_ResetQspiMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetQspiResetMask
   ---------------------------------------------------------------------------- */

#if ((defined(MCU_MC56F82728)) || (defined(MCU_MC56F82738)) || (defined(MCU_MC56F82748)))
/**
 * @brief Returns mask of QSPI peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals software reset mask
 *         constants. Used to specify which QSPI peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQspiResetMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetQspiResetMask(PeripheralBase) ( \
      PeriphGetBitMask16( \
       SIM_PSWR1_REG(PeripheralBase), \
       (uint16_t)(SIM_PSWR1_QSPI0_MASK | SIM_PSWR1_QSPI1_MASK) \
      ) \
    )
#else /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */
/**
 * @brief Returns mask of QSPI peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "QSPI peripherals software reset mask
 *         constants. Used to specify which QSPI peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQspiResetMask(deviceID);
 *      @endcode
 */
  #define SIM_PDD_GetQspiResetMask(PeripheralBase) ( \
      PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_QSPI0_MASK) \
    )
#endif /* (defined(MCU_MC56F82313)) || (defined(MCU_MC56F82316)) || (defined(MCU_MC56F82723)) || (defined(MCU_MC56F82726)) || (defined(MCU_MC56F82733)) || (defined(MCU_MC56F82736)) || (defined(MCU_MC56F82743)) || (defined(MCU_MC56F82746)) */

/* ----------------------------------------------------------------------------
   -- ResetQspi0
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the QSPI0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQspi0(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetQspi0(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_QSPI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetQspi0Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns QSPI0 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQspi0Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetQspi0Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_QSPI0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetIic0
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the IIC0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetIic0(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetIic0(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_IIC0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetIic0Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns IIC0 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetIic0Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetIic0Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_IIC0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetEwm
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the EWM.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetEwm(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetEwm(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_EWM_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetEwmReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns EWM software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetEwmReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetEwmReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_EWM_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetCmp
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the CMP.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetCmp(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetCmp(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CMP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCmpReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns CMP software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetCmpReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetCmpReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CMP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetAdc12
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the cyclic 12-bit ADC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetAdc12(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetAdc12(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CYCADC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAdc12Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns cyclic 12-bit ADC software reset state. Peripheral is in reset
 * if non-zero value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetAdc12Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetAdc12Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CYCADC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetCrc
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the CRC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetCrc(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetCrc(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CRC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCrcReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns CRC software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetCrcReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetCrcReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_CRC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetPitMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of specified PIT peripherals. Peripherals are
 * specified by mask constant. Bits associated with peripherals not specified in
 * input mask stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of PIT peripherals defining which peripherals will be reset.
 *        Logical or of associated mask constants can be used. Use constants
 *        from group "PIT peripherals software reset mask constants. Used to specify
 *        which PIT peripheral is (in) reset.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetPitMask(deviceID, SIM_PDD_RESET_PIT0);
 *      @endcode
 */
#define SIM_PDD_ResetPitMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPitResetMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of PIT peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PIT peripherals software reset mask
 *         constants. Used to specify which PIT peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPitResetMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPitResetMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PSWR2_REG(PeripheralBase), \
     (uint16_t)(SIM_PSWR2_PIT0_MASK | SIM_PSWR2_PIT1_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ResetPit0
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the PIT0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetPit0(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetPit0(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_PIT0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPit0Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns PIT0 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPit0Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPit0Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_PIT0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetPit1
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the PIT1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetPit1(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetPit1(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_PIT1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPit1Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns PIT1 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPit1Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPit1Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR2_REG(PeripheralBase), SIM_PSWR2_PIT1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetPwmA
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the PWMA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR3.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetPwmA(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetPwmA(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR3_REG(PeripheralBase), SIM_PSWR3_PWMA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPwmAReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns PWMA software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPwmAReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPwmAReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR3_REG(PeripheralBase), SIM_PSWR3_PWMA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerModeStatusMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns power mode status mask.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Power modes mask constants." for processing
 *         return value.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPowerModeStatusMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPowerModeStatusMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PWRMODE_REG(PeripheralBase), \
     (uint16_t)(SIM_PWRMODE_LPMS_MASK | SIM_PWRMODE_VLPMS_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVeryLowPowerModeStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns very low power mode status. Chip is in very low power mode if
 * non-zero value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetVeryLowPowerModeStatus(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetVeryLowPowerModeStatus(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_VLPMS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetLowPowerModeStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns low power mode status. Chip is in low power mode if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetLowPowerModeStatus(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetLowPowerModeStatus(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_LPMS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLowPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables (enters) or disables (exits) the low power mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the low power mode is entered or exited.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableLowPowerMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableLowPowerMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_LPMODE_MASK)) : ( \
      PeriphSetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_LPMODE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetLowPowerModeEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the low power mode enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetLowPowerModeEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetLowPowerModeEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_LPMODE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVeryLowPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables (enters) or disables (exits) the very low power mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the very low power mode is entered or
 *        exited. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      SIM_PDD_EnableVeryLowPowerMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnableVeryLowPowerMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_VLPMODE_MASK)) : ( \
      PeriphSetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_VLPMODE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetVeryLowPowerModeEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the very low power mode enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetVeryLowPowerModeEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetVeryLowPowerModeEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PWRMODE_REG(PeripheralBase), SIM_PWRMODE_VLPMODE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastRoscTemperatureTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator temperature trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_NVMOPT2H.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetFastRoscTemperatureTrim(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetFastRoscTemperatureTrim(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      SIM_NVMOPT2H_REG(PeripheralBase), \
      SIM_NVMOPT2H_ROSC_8M_TTRIM_MASK \
     )) >> ( \
     SIM_NVMOPT2H_ROSC_8M_TTRIM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastRoscFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator frequency trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 10-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: SIM_NVMOPT2H.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetFastRoscFrequencyTrim(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetFastRoscFrequencyTrim(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     SIM_NVMOPT2H_REG(PeripheralBase), \
     SIM_NVMOPT2H_ROSC_8M_FTRIM_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerControllerBandgapTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the power management controller bandgap trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_NVMOPT2L.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetPowerControllerBandgapTrim(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPowerControllerBandgapTrim(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      SIM_NVMOPT2L_REG(PeripheralBase), \
      SIM_NVMOPT2L_PMC_BGTRIM_MASK \
     )) >> ( \
     SIM_NVMOPT2L_PMC_BGTRIM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSlowRoscFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 200 kHz relaxation oscillator frequency trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 9-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: SIM_NVMOPT2L.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetSlowRoscFrequencyTrim(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetSlowRoscFrequencyTrim(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     SIM_NVMOPT2L_REG(PeripheralBase), \
     SIM_NVMOPT2L_ROSC_200K_FTRIM_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR0.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl0Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl0Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl0Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR1.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl1Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl1Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR2.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl2Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl2Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR3.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl3Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl3Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl3Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 4.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR4.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl4Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl4Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR4_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR4.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl4Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl4Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 5.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR5.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl5Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl5Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR5_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR5.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl5Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl5Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 6.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR6.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl6Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl6Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR6_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR6.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl6Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl6Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR6_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControl7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the software control register 7.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SCR7.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteSoftwareControl7Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteSoftwareControl7Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SCR7_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControl7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the software control register 7.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SCR7.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadSoftwareControl7Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadSoftwareControl7Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SCR7_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadResetStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the reset status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_RSTAT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadResetStatusReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadResetStatusReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_RSTAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadJtagIdHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the most significant half of JTAG ID register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_MSHID.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadJtagIdHighReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadJtagIdHighReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_MSHID_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadJtagIdLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the least significant half of JTAG ID register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_LSHID.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadJtagIdLowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadJtagIdLowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_LSHID_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePowerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the power control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePowerControlReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePowerControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PWR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPowerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the power control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PWR.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPowerControlReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPowerControlReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PWR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteClockOutputSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the clock output select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteClockOutputSelectReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteClockOutputSelectReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_CLKOUT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadClockOutputSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the clock output select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_CLKOUT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadClockOutputSelectReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadClockOutputSelectReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_CLKOUT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralClockRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock rate register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralClockRateReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralClockRateReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PCR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralClockRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock rate register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralClockRateReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralClockRateReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PCR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralClockEnable0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralClockEnable0Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralClockEnable0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PCE0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralClockEnable0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralClockEnable0Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralClockEnable0Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PCE0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralClockEnable1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralClockEnable1Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralClockEnable1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PCE1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralClockEnable1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralClockEnable1Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralClockEnable1Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PCE1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralClockEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralClockEnable2Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralClockEnable2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PCE2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralClockEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralClockEnable2Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralClockEnable2Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PCE2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralClockEnable3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralClockEnable3Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralClockEnable3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PCE3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralClockEnable3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralClockEnable3Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralClockEnable3Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PCE3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralStopClockEnable0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable in STOP mode register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralStopClockEnable0Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralStopClockEnable0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SD0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralStopClockEnable0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable in STOP mode
 * register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralStopClockEnable0Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralStopClockEnable0Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SD0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralStopClockEnable1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable in STOP mode register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralStopClockEnable1Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralStopClockEnable1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SD1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralStopClockEnable1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable in STOP mode
 * register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralStopClockEnable1Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralStopClockEnable1Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SD1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralStopClockEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable in STOP mode register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralStopClockEnable2Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralStopClockEnable2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SD2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralStopClockEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable in STOP mode
 * register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralStopClockEnable2Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralStopClockEnable2Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SD2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralStopClockEnable3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral clock enable in STOP mode register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralStopClockEnable3Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralStopClockEnable3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_SD3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralStopClockEnable3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral clock enable in STOP mode
 * register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralStopClockEnable3Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralStopClockEnable3Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_SD3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIoShortAddressLocationHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the I/O short address location high register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IOSAHI.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteIoShortAddressLocationHighReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteIoShortAddressLocationHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_IOSAHI_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIoShortAddressLocationHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the I/O short address location high register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_IOSAHI.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadIoShortAddressLocationHighReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadIoShortAddressLocationHighReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_IOSAHI_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIoShortAddressLocationLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the I/O short address location low register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IOSALO.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteIoShortAddressLocationLowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteIoShortAddressLocationLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_IOSALO_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIoShortAddressLocationLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the I/O short address location low register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_IOSALO.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadIoShortAddressLocationLowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadIoShortAddressLocationLowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_IOSALO_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteProtectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the protection register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteProtectionReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteProtectionReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PROT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadProtectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the protection register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PROT.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadProtectionReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadProtectionReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PROT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioALowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOA LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSAL.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioALowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioALowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioALowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOA LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSAL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioALowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioALowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioBLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOB LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSBL.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioBLowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioBLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSBL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioBLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOB LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSBL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioBLowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioBLowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSBL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioCLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOC LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioCLowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioCLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSCL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioCLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOC LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSCL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioCLowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioCLowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSCL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioCHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOC MSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioCHighReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioCHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSCH_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioCHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOC MSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSCH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioCHighReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioCHighReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSCH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioELowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOE LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioELowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioELowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSEL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioELowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOE LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSEL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioELowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioELowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSEL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioFLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOF LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioFLowReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioFLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSFL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioFLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOF LSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSFL.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioFLowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioFLowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSFL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSelectGpioFHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the GPIOF MSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_GPSFH.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSelectGpioFHighReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSelectGpioFHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_GPSFH_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSelectGpioFHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the GPIOF MSB peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_GPSFH.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSelectGpioFHighReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSelectGpioFHighReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_GPSFH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInternalPeripheralSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the internal peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteInternalPeripheralSelectReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteInternalPeripheralSelectReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_IPSn_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInternalPeripheralSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the internal peripheral select register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_IPSn.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadInternalPeripheralSelectReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadInternalPeripheralSelectReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_IPSn_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMiscellaneous0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the miscellaneous register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      SIM_PDD_WriteMiscellaneous0Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WriteMiscellaneous0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_MISC0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMiscellaneous0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the miscellaneous register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadMiscellaneous0Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadMiscellaneous0Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_MISC0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSoftwareReset0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral software reset register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSoftwareReset0Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSoftwareReset0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PSWR0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSoftwareReset0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral software reset register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR0.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSoftwareReset0Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSoftwareReset0Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PSWR0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSoftwareReset1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral software reset register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSoftwareReset1Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSoftwareReset1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PSWR1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSoftwareReset1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral software reset register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSoftwareReset1Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSoftwareReset1Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PSWR1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSoftwareReset2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral software reset register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSoftwareReset2Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSoftwareReset2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PSWR2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSoftwareReset2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral software reset register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSoftwareReset2Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSoftwareReset2Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PSWR2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralSoftwareReset3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the peripheral software reset register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR3.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePeripheralSoftwareReset3Reg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePeripheralSoftwareReset3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PSWR3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralSoftwareReset3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the peripheral software reset register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR3.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPeripheralSoftwareReset3Reg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPeripheralSoftwareReset3Reg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PSWR3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePowerModeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the power mode register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      SIM_PDD_WritePowerModeReg(deviceID, 1);
 *      @endcode
 */
#define SIM_PDD_WritePowerModeReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(SIM_PWRMODE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPowerModeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the power mode register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PWRMODE.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadPowerModeReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadPowerModeReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_PWRMODE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadNonVolatileOption2HighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the non-volatile memory option register 2 high.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_NVMOPT2H.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadNonVolatileOption2HighReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadNonVolatileOption2HighReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_NVMOPT2H_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadNonVolatileOption2LowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the non-volatile memory option register 2 low.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_NVMOPT2L.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_ReadNonVolatileOption2LowReg(deviceID);
 *      @endcode
 */
#define SIM_PDD_ReadNonVolatileOption2LowReg(PeripheralBase) ( \
    PeriphReadReg16(SIM_NVMOPT2L_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SoftwareReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_CTRL.
 * @par Example:
 *      @code
 *      SIM_PDD_SoftwareReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_SoftwareReset(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_CTRL_REG(PeripheralBase), SIM_CTRL_SWRst_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockRateSci1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock speed for the SCI1 module.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock speed for the SCI1 module. This
 *        parameter is of "Clock rate speed constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      SIM_PDD_SetClockRateSci1(deviceID, SIM_PDD_CLOCK_RATE_BUS_CLK);
 *      @endcode
 */
#define SIM_PDD_SetClockRateSci1(PeripheralBase, Source) ( \
    ((Source) == SIM_PDD_CLOCK_RATE_BUS_CLK) ? ( \
      PeriphClearBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI1_CR_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI1_CR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockRateSci1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock speed for the SCI1 module.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock rate speed constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_PCR.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetClockRateSci1(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetClockRateSci1(PeripheralBase) ( \
    (PeriphGetBitMask16(SIM_PCR_REG(PeripheralBase), SIM_PCR_SCI1_CR_MASK) == 0U) ? ( \
      SIM_PDD_CLOCK_RATE_BUS_CLK) : ( \
      SIM_PDD_CLOCK_RATE_2X_BUS_CLK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockDacMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral clock of selected DAC peripherals. Peripherals are
 * specified by clock gate mask constants which can be bitwise OR-ed together to
 * enable or disable more clcok gates at once. Bits associated with peripherals not
 * specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of DAC peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "DAC peripherals clock gate mask constants.
 *        Used to specify DAC peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of DAC peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "DAC peripherals clock gate mask constants. Used
 *        to specify DAC peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockDacMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_DACA_MASK, SIM_PDD_CLOCK_GATE_DACA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockDacMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_PCE1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockDacEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of DAC peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DAC peripherals clock gate mask constants.
 *         Used to specify DAC peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockDacEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockDacEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PCE1_REG(PeripheralBase), \
     (uint16_t)(SIM_PCE1_DACA_MASK | SIM_PCE1_DACB_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockDacDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of DAC peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DAC peripherals clock gate mask constants.
 *         Used to specify DAC peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockDacDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockDacDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_PCE1_REG(PeripheralBase), \
     (uint16_t)(SIM_PCE1_DACA_MASK | SIM_PCE1_DACB_MASK) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockDacA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of DACA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockDacA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockDacA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACA_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockDacAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of DACA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockDacAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockDacAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockDacB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of DACB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockDacB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockDacB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACB_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockDacBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of DACB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockDacBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockDacBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_DACB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQsci1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of QSCI1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQsci1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQsci1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI1_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQsci1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of QSCI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQsci1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockQsci1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_SCI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCmpC
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of CMPC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCmpC(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCmpC(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPC_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpCEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of CMPC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpCEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockCmpCEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockDacMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets peripheral STOP mode clock of selected DAC peripherals.
 * Peripherals are specified by clock gate mask constants which can be bitwise OR-ed
 * together to enable or disable more clcok gates at once. Bits associated with
 * peripherals not specified in input masks stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param ClockDisableMask Mask of DAC peripherals defining which clock gate
 *        should be disabled. Logical or of associated mask constants can be used.
 *        Use constants from group "DAC peripherals clock gate mask constants.
 *        Used to specify DAC peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @param ClockEnableMask Mask of DAC peripherals defining which clock gate
 *        should be enabled. Logical or of associated mask constants can be used. Use
 *        constants from group "DAC peripherals clock gate mask constants. Used
 *        to specify DAC peripherals of which clock gate will be enbaled or
 *        disabled.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockDacMask(deviceID,
 *      SIM_PDD_CLOCK_GATE_DACA_MASK, SIM_PDD_CLOCK_GATE_DACA_MASK);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockDacMask(PeripheralBase, ClockDisableMask, ClockEnableMask) ( \
    PeriphSetBits16( \
     SIM_SD1_REG(PeripheralBase), \
     (uint16_t)(ClockDisableMask), \
     (uint16_t)(ClockEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockDacEnabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of DAC peripherals with enabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DAC peripherals clock gate mask constants.
 *         Used to specify DAC peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockDacEnabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockDacEnabledMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_SD1_REG(PeripheralBase), \
     (uint16_t)(SIM_SD1_DACA_MASK | SIM_SD1_DACB_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockDacDisabledMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of DAC peripherals with disabled clock gate.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DAC peripherals clock gate mask constants.
 *         Used to specify DAC peripherals of which clock gate will be enbaled or
 *         disabled." for processing return value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      SIM_PDD_GetPeripheralStopClockDacDisabledMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockDacDisabledMask(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)PeriphGetBitMask16( \
     SIM_SD1_REG(PeripheralBase), \
     (uint16_t)(SIM_SD1_DACA_MASK | SIM_SD1_DACB_MASK) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockDacA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of DACA peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockDacA(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockDacA(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACA_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockDacAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of DACA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockDacAEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockDacAEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockDacB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of DACB peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockDacB(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockDacB(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACB_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockDacBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of DACB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockDacBEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockDacBEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_DACB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQsci1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of QSCI1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQsci1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQsci1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI1_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQsci1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of QSCI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockQsci1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockQsci1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_SCI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCmpC
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of CMPC peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCmpC(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCmpC(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPC_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpCEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of CMPC.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockCmpCEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockCmpCEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetDacMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of specified DAC peripherals. Peripherals are
 * specified by mask constant. Bits associated with peripherals not specified in
 * input mask stays untouched.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of DAC peripherals defining which peripherals will be reset.
 *        Logical or of associated mask constants can be used. Use constants
 *        from group "DAC peripherals software reset mask constants. Used to specify
 *        which DAC peripheral is (in) reset.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetDacMask(deviceID, SIM_PDD_RESET_DACA);
 *      @endcode
 */
#define SIM_PDD_ResetDacMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDacResetMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of DAC peripherals in software reset state.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "DAC peripherals software reset mask
 *         constants. Used to specify which DAC peripheral is (in) reset." for
 *         processing return value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetDacResetMask(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetDacResetMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     SIM_PSWR1_REG(PeripheralBase), \
     (uint16_t)(SIM_PSWR1_DACA_MASK | SIM_PSWR1_DACB_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ResetDacA
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the DACA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetDacA(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetDacA(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_DACA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDacAReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DACA software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetDacAReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetDacAReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_DACA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetDacB
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the DACB.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetDacB(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetDacB(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_DACB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDacBReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DACB software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetDacBReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetDacBReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_DACB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetQsci1
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the QSCI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQsci1(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetQsci1(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_SCI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetQsci1Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns QSCI1 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQsci1Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetQsci1Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_SCI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetSpeedModeStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns system speed mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "System speed mode constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetSpeedModeStatus(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetSpeedModeStatus(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_MODE_STAT_MASK)) == ( \
      0U)) ? ( \
      SIM_PDD_NORMAL_MODE) : ( \
      SIM_PDD_FAST_MODE) \
  )

/* ----------------------------------------------------------------------------
   -- SetSpeedMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets system speed mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Parameter specifying the system speed mode. This parameter is of
 *        "System speed mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      SIM_PDD_SetSpeedMode(deviceID, SIM_PDD_NORMAL_MODE);
 *      @endcode
 */
#define SIM_PDD_SetSpeedMode(PeripheralBase, Mode) ( \
    ((Mode) == SIM_PDD_NORMAL_MODE) ? ( \
      PeriphClearBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_FAST_MODE_MASK)) : ( \
      PeriphSetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_FAST_MODE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSpeedMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns system speed mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "System speed mode constants." type. The value is
 *         cast to "uint8_t".
 * @remarks The macro accesses the following registers: SIM_MISC0.
 * @par Example:
 *      @code
 *      uint8_t result = SIM_PDD_GetSpeedMode(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetSpeedMode(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(SIM_MISC0_REG(PeripheralBase), SIM_MISC0_FAST_MODE_MASK)) == ( \
      0U)) ? ( \
      SIM_PDD_NORMAL_MODE) : ( \
      SIM_PDD_FAST_MODE) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockMscan
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of MSCAN peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockMscan(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockMscan(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_MSCAN_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_MSCAN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockMscanEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of selected MSCAN.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockMscanEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockMscanEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_MSCAN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockMscan
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of MSCAN peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockMscan(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockMscan(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_MSCAN_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_MSCAN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockMscanEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of selected MSCAN.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockMscanEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockMscanEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_MSCAN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetMscan
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the MSCAN.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetMscan(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetMscan(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_MSCAN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetMscanReset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns MSCAN software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetMscanReset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetMscanReset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_MSCAN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockQspi1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of QSPI1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockQspi1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockQspi1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI1_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockQspi1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of QSPI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockQspi1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockQspi1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE1_REG(PeripheralBase), SIM_PCE1_QSPI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralClockCmpD
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral clock of CMPD peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralClockCmpD(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralClockCmpD(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPD_MASK)) : ( \
      PeriphSetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralClockCmpDEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral clock enable state of CMPD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PCE2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralClockCmpDEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralClockCmpDEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PCE2_REG(PeripheralBase), SIM_PCE2_CMPD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockQspi1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of QSPI1 peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockQspi1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockQspi1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI1_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockQspi1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of QSPI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockQspi1Enabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockQspi1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD1_REG(PeripheralBase), SIM_SD1_QSPI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralStopClockCmpD
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable peripheral STOP mode clock of CMPD peripheral.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if clock gate is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      SIM_PDD_EnablePeripheralStopClockCmpD(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define SIM_PDD_EnablePeripheralStopClockCmpD(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPD_MASK)) : ( \
      PeriphSetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralStopClockCmpDEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral STOP mode clock enable state of CMPD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_SD2.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetPeripheralStopClockCmpDEnabled(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetPeripheralStopClockCmpDEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_SD2_REG(PeripheralBase), SIM_SD2_CMPD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ResetQspi1
   ---------------------------------------------------------------------------- */

/**
 * @brief Performs software reset of the QSPI1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      SIM_PDD_ResetQspi1(deviceID);
 *      @endcode
 */
#define SIM_PDD_ResetQspi1(PeripheralBase) ( \
    PeriphSetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_QSPI1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetQspi1Reset
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns QSPI1 software reset state. Peripheral is in reset if non-zero
 * value is returned.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SIM_PSWR1.
 * @par Example:
 *      @code
 *      uint16_t result = SIM_PDD_GetQspi1Reset(deviceID);
 *      @endcode
 */
#define SIM_PDD_GetQspi1Reset(PeripheralBase) ( \
    PeriphGetBitMask16(SIM_PSWR1_REG(PeripheralBase), SIM_PSWR1_QSPI1_MASK) \
  )
#endif  /* #if defined(SIM_PDD_H_) */

/* SIM_PDD.h, eof. */
