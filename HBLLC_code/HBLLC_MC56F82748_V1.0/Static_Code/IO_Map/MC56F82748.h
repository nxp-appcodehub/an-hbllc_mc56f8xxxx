/*
** ###################################################################
**     Processor:           MC56F82748VLH
**     Compiler:            CodeWarrior C/C++ for DSP M56800E
**     Reference manual:    MC56F827XXRM Rev.1 Draft A 02/2012
**     Version:             rev. 0.2, 2013-04-05
**
**     Abstract:
**         This header file implements peripheral memory map for MC56F82748
**         processor.
**
**     Copyright: 1997 - 2013 Freescale, Inc.
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 0.1 (2012-05-11)
**         Initial version
**     - rev. 0.2 (2013-04-05)
**         Changed start of doxygen comment.
**
** ###################################################################
*/

/*!
 * @file MC56F82748.h
 * @version 0.2
 * @date 2013-04-05
 * @brief Peripheral memory map for MC56F82748
 *
 * This header file implements peripheral memory map for MC56F82748 processor.
 */


/* ----------------------------------------------------------------------------
   -- MCU activation
   ---------------------------------------------------------------------------- */

/* Prevention from multiple including the same memory map */
#if !defined(MCU_MC56F82748)  /* Check if memory map has not been already included */
#define MCU_MC56F82748

/* Check if another memory map has not been also included */
#if (defined(MCU_ACTIVE))
  #error MC56F82748 memory map: There is already included another memory map. Only one memory map can be included.
#endif /* (defined(MCU_ACTIVE)) */
#define MCU_ACTIVE

#include <stdint.h>

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0000u
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0002u


/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
typedef enum {
  INT_HW_RESET                 = 0,                /**< Reserved for Reset Overlay */
  INT_COP_RESET                = 1,                /**< Reserved for COP Reset Overlay */
  INT_ILLEGAL_OP               = 2,                /**< Illegal Instruction */
  INT_SWI3                     = 3,                /**< SW Interrupt 3 */
  INT_OVERFLOW                 = 4,                /**< Hardware Stack Overflow */
  INT_MISALIGNED               = 5,                /**< Misaligned Data Access */
  INT_STPCNT                   = 6,                /**< EOnCE Step Counter Interrupt */
  INT_BKPT                     = 7,                /**< EOnCE Breakpoint Unit */
  INT_TRBUF                    = 8,                /**< EOnCE Trace Buffer Interrupt */
  INT_TX_REG                   = 9,                /**< EOnCE Transmit Register Empty */
  INT_RX_REG                   = 10,               /**< EOnCE Receive Register Full */
  INT_BUS_ERR                  = 11,               /**< Bus Error Interrupt */
  INT_reserved12               = 12,               /**< Reserved interrupt */
  INT_reserved13               = 13,               /**< Reserved interrupt */
  INT_reserved14               = 14,               /**< Reserved interrupt */
  INT_SWI2                     = 15,               /**< SW Interrupt 2 */
  INT_SWI1                     = 16,               /**< SW Interrupt 1 */
  INT_SWI0                     = 17,               /**< SW Interrupt 0 */
  INT_XBARA                    = 18,               /**< Crossbar Interrupt */
  INT_LVI1                     = 19,               /**< Low Voltage Interrupt */
  INT_OCCS                     = 20,               /**< PLL Loss of Lock 0; PLL Loss of Lock 1; PLL Loss of Reference Clock */
  INT_reserved21               = 21,               /**< Reserved interrupt */
  INT_reserved22               = 22,               /**< Reserved interrupt */
  INT_reserved23               = 23,               /**< Reserved interrupt */
  INT_reserved24               = 24,               /**< Reserved interrupt */
  INT_TMRA_3                   = 25,               /**< Timer Compare; Timer Overflow; Input Edge Flag;Timer Compare Flag 1; Timer Compare Flag 2 */
  INT_TMRA_2                   = 26,               /**< Timer Compare; Timer Overflow; Input Edge Flag;Timer Compare Flag 1; Timer Compare Flag 2 */
  INT_TMRA_1                   = 27,               /**< Timer Compare; Timer Overflow; Input Edge Flag;Timer Compare Flag 1; Timer Compare Flag 2 */
  INT_TMRA_0                   = 28,               /**< Timer Compare; Timer Overflow; Input Edge Flag;Timer Compare Flag 1; Timer Compare Flag 2 */
  INT_ADC_CC1                  = 29,               /**< ADC Conversion Complete; Converter B in non-simultaneous parallel scan mode */
  INT_ADC_CC0                  = 30,               /**< ADC Conversion Complete; any scan type except Converter B in non_simultaneous parallel scan mode */
  INT_ADC_ERR                  = 31,               /**< ADC zero crossing; low limit; and high limit */
  INT_reserved32               = 32,               /**< Reserved interrupt */
  INT_DMA3                     = 33,               /**< DMA 3 Service Req */
  INT_DMA2                     = 34,               /**< DMA 2 Service Req */
  INT_DMA1                     = 35,               /**< DMA 1 Service Req */
  INT_DMA0                     = 36,               /**< DMA 0 Service Req */
  INT_reserved37               = 37,               /**< Reserved interrupt */
  INT_reserved38               = 38,               /**< Reserved interrupt */
  INT_ERROR                    = 39,               /**< Error */
  INT_TX_WARN                  = 40,               /**< Transmit Warning */
  INT_RX_WARN                  = 41,               /**< Receive Warning */
  INT_WAKEUP                   = 42,               /**< Wakeup */
  INT_reserved43               = 43,               /**< Reserved interrupt */
  INT_reserved44               = 44,               /**< Reserved interrupt */
  INT_reserved45               = 45,               /**< Reserved interrupt */
  INT_reserved46               = 46,               /**< Reserved interrupt */
  INT_QSCI1_RERR               = 47,               /**< Receiver Error */
  INT_QSCI1_RCV                = 48,               /**< Receive Data Register Full */
  INT_QSCI1_TIDLE              = 49,               /**< Transmitter Idle */
  INT_QSCI1_TDRE               = 50,               /**< Transmit Data Register Empty */
  INT_QSCI0_RERR               = 51,               /**< Receiver Error */
  INT_QSCI0_RCV                = 52,               /**< Receive Data Register Full */
  INT_QSCI0_TIDLE              = 53,               /**< Transmitter Idle */
  INT_QSCI0_TDRE               = 54,               /**< Transmit Data Register Empty */
  INT_reserved55               = 55,               /**< Reserved interrupt */
  INT_reserved56               = 56,               /**< Reserved interrupt */
  INT_QSPI1_XMIT               = 57,               /**< Transmitter Empty */
  INT_QSPI1_RCV                = 58,               /**< Receiver Full */
  INT_QSPI0_XMIT               = 59,               /**< Transmitter Empty */
  INT_QSPI0_RCV                = 60,               /**< Receiver Full */
  INT_reserved61               = 61,               /**< Reserved interrupt */
  INT_IIC0                     = 62,               /**< Complete 1-byte transfer (TCF); Match of received calling address (IAAS); Arbitration Lost (ARBL); SMBus Timeout (SLTF) Interrupts. Stop detect interrupt for which local enable is CR1[IICIE] and FLT[STOPIE]. */
  INT_reserved63               = 63,               /**< Reserved interrupt */
  INT_reserved64               = 64,               /**< Reserved interrupt */
  INT_reserved65               = 65,               /**< Reserved interrupt */
  INT_reserved66               = 66,               /**< Reserved interrupt */
  INT_reserved67               = 67,               /**< Reserved interrupt */
  INT_reserved68               = 68,               /**< Reserved interrupt */
  INT_reserved69               = 69,               /**< Reserved interrupt */
  INT_reserved70               = 70,               /**< Reserved interrupt */
  INT_reserved71               = 71,               /**< Reserved interrupt */
  INT_reserved72               = 72,               /**< Reserved interrupt */
  INT_reserved73               = 73,               /**< Reserved interrupt */
  INT_reserved74               = 74,               /**< Reserved interrupt */
  INT_reserved75               = 75,               /**< Reserved interrupt */
  INT_reserved76               = 76,               /**< Reserved interrupt */
  INT_eFlexPWMA_FAULT          = 77,               /**< Fault Condition */
  INT_eFlexPWMA_RERR           = 78,               /**< Reload Error */
  INT_eFlexPWMA_RELOAD3        = 79,               /**< Submodule 3 Reload */
  INT_eFlexPWMA_CAP            = 80,               /**< Logic Or'ed all Submodule 0-3 Input Captures */
  INT_eFlexPWMA_CMP3           = 81,               /**< Submodule 3 Compare */
  INT_eFlexPWMA_RELOAD2        = 82,               /**< Submodule 2 Reload */
  INT_eFlexPWMA_CMP2           = 83,               /**< Submodule 2 Compare */
  INT_eFlexPWMA_RELOAD1        = 84,               /**< Submodule 1 Reload */
  INT_eFlexPWMA_CMP1           = 85,               /**< Submodule 1 Compare */
  INT_eFlexPWMA_RELOAD0        = 86,               /**< Submodule 0 Reload */
  INT_eFlexPWMA_CMP0           = 87,               /**< Submodule 0 Compare */
  INT_FTFA_RDCOL               = 88,               /**< Access Error */
  INT_FTFA_CC                  = 89,               /**< Command Complete */
  INT_CMPD                     = 90,               /**< Rising Edge; Falling Edge */
  INT_CMPC                     = 91,               /**< Rising Edge; Falling Edge */
  INT_CMPB                     = 92,               /**< Rising Edge; Falling Edge */
  INT_CMPA                     = 93,               /**< Rising Edge; Falling Edge */
  INT_PIT1_ROLLOVR             = 94,               /**< Roll Over */
  INT_PIT0_ROLLOVR             = 95,               /**< Roll Over */
  INT_reserved96               = 96,               /**< Reserved interrupt */
  INT_reserved97               = 97,               /**< Reserved interrupt */
  INT_reserved98               = 98,               /**< Reserved interrupt */
  INT_reserved99               = 99,               /**< Reserved interrupt */
  INT_reserved100              = 100,              /**< Reserved interrupt */
  INT_reserved101              = 101,              /**< Reserved interrupt */
  INT_GPIOF                    = 102,              /**< GPIO */
  INT_GPIOE                    = 103,              /**< GPIO */
  INT_GPIOD                    = 104,              /**< GPIO */
  INT_GPIOC                    = 105,              /**< GPIO */
  INT_GPIOB                    = 106,              /**< GPIO */
  INT_GPIOA                    = 107,              /**< GPIO */
  INT_COP_INT                  = 108,              /**< COP_Warning */
  INT_EWM_INT                  = 109,              /**< EWM_Indicate */
  INT_SWILP                    = 110               /**< SWILP */
} IRQInterruptIndex;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Peripheral type defines
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_defines Peripheral type defines
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__m56800E__)
  /* anonymous unions are enabled by default */
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral ADC
 * @{
 */

/** ADC - Peripheral register structure */
typedef struct ADC_MemMap {
  uint16_t CTRL1;                                  /**< ADC Control Register 1, offset: 0x0 */
  uint16_t CTRL2;                                  /**< ADC Control Register 2, offset: 0x1 */
  uint16_t ZXCTRL1;                                /**< ADC Zero Crossing Control 1 Register, offset: 0x2 */
  uint16_t ZXCTRL2;                                /**< ADC Zero Crossing Control 2 Register, offset: 0x3 */
  uint16_t CLIST1;                                 /**< ADC Channel List Register 1, offset: 0x4 */
  uint16_t CLIST2;                                 /**< ADC Channel List Register 2, offset: 0x5 */
  uint16_t CLIST3;                                 /**< ADC Channel List Register 3, offset: 0x6 */
  uint16_t CLIST4;                                 /**< ADC Channel List Register 4, offset: 0x7 */
  uint16_t SDIS;                                   /**< ADC Sample Disable Register, offset: 0x8 */
  uint16_t STAT;                                   /**< ADC Status Register, offset: 0x9 */
  uint16_t RDY;                                    /**< ADC Ready Register, offset: 0xA */
  uint16_t LOLIMSTAT;                              /**< ADC Low Limit Status Register, offset: 0xB */
  uint16_t HILIMSTAT;                              /**< ADC High Limit Status Register, offset: 0xC */
  uint16_t ZXSTAT;                                 /**< ADC Zero Crossing Status Register, offset: 0xD */
  uint16_t RSLT[16];                               /**< ADC Result Registers with sign extension, array offset: 0xE, array step: 0x1 */
  uint16_t LOLIM[16];                              /**< ADC Low Limit Registers, array offset: 0x1E, array step: 0x1 */
  uint16_t HILIM[16];                              /**< ADC High Limit Registers, array offset: 0x2E, array step: 0x1 */
  uint16_t OFFST[16];                              /**< ADC Offset Registers, array offset: 0x3E, array step: 0x1 */
  uint16_t PWR;                                    /**< ADC Power Control Register, offset: 0x4E */
  uint16_t CAL;                                    /**< ADC Calibration Register, offset: 0x4F */
  uint16_t GC1;                                    /**< Gain Control 1 Register, offset: 0x50 */
  uint16_t GC2;                                    /**< Gain Control 2 Register, offset: 0x51 */
  uint16_t SCTRL;                                  /**< ADC Scan Control Register, offset: 0x52 */
  uint16_t PWR2;                                   /**< ADC Power Control Register 2, offset: 0x53 */
  uint16_t CTRL3;                                  /**< ADC Control Register 3, offset: 0x54 */
  uint16_t SCHLTEN;                                /**< ADC Scan Halted Interrupt Enable Register, offset: 0x55 */
  uint16_t RESERVED_0[2];
  uint16_t ZXCTRL3;                                /**< ADC Zero Crossing Control 3 Register, offset: 0x58 */
  uint16_t CLIST5;                                 /**< ADC Channel List Register 5, offset: 0x59 */
  uint16_t SDIS2;                                  /**< ADC Sample Disable Register 2, offset: 0x5A */
  uint16_t RDY2;                                   /**< ADC Ready Register 2, offset: 0x5B */
  uint16_t LOLIMSTAT2;                             /**< ADC Low Limit Status Register 2, offset: 0x5C */
  uint16_t HILIMSTAT2;                             /**< ADC High Limit Status Register 2, offset: 0x5D */
  uint16_t ZXSTAT2;                                /**< ADC Zero Crossing Status Register 2, offset: 0x5E */
  uint16_t RSLT_2[4];                              /**< ADC Result Registers 2 with sign extension, array offset: 0x5F, array step: 0x1 */
  uint16_t LOLIM_2[4];                             /**< ADC Low Limit Registers 2, array offset: 0x63, array step: 0x1 */
  uint16_t HILIM_2[4];                             /**< ADC High Limit Registers 2, array offset: 0x67, array step: 0x1 */
  uint16_t OFFST_2[4];                             /**< ADC Offset Registers 2, array offset: 0x6B, array step: 0x1 */
  uint16_t GC3;                                    /**< Gain Control 3 Register, offset: 0x6F */
  uint16_t SCTRL2;                                 /**< ADC Scan Control Register 2, offset: 0x70 */
  uint16_t SCHLTEN2;                               /**< ADC Scan Halted Interrupt Enable Register 2, offset: 0x71 */
} volatile *ADC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register accessors */
#define ADC_CTRL1_REG(base)                      ((base)->CTRL1)
#define ADC_CTRL2_REG(base)                      ((base)->CTRL2)
#define ADC_ZXCTRL1_REG(base)                    ((base)->ZXCTRL1)
#define ADC_ZXCTRL2_REG(base)                    ((base)->ZXCTRL2)
#define ADC_CLIST1_REG(base)                     ((base)->CLIST1)
#define ADC_CLIST2_REG(base)                     ((base)->CLIST2)
#define ADC_CLIST3_REG(base)                     ((base)->CLIST3)
#define ADC_CLIST4_REG(base)                     ((base)->CLIST4)
#define ADC_SDIS_REG(base)                       ((base)->SDIS)
#define ADC_STAT_REG(base)                       ((base)->STAT)
#define ADC_RDY_REG(base)                        ((base)->RDY)
#define ADC_LOLIMSTAT_REG(base)                  ((base)->LOLIMSTAT)
#define ADC_HILIMSTAT_REG(base)                  ((base)->HILIMSTAT)
#define ADC_ZXSTAT_REG(base)                     ((base)->ZXSTAT)
#define ADC_RSLT_REG(base,index)                 ((base)->RSLT[index])
#define ADC_LOLIM_REG(base,index)                ((base)->LOLIM[index])
#define ADC_HILIM_REG(base,index)                ((base)->HILIM[index])
#define ADC_OFFST_REG(base,index)                ((base)->OFFST[index])
#define ADC_PWR_REG(base)                        ((base)->PWR)
#define ADC_CAL_REG(base)                        ((base)->CAL)
#define ADC_GC1_REG(base)                        ((base)->GC1)
#define ADC_GC2_REG(base)                        ((base)->GC2)
#define ADC_SCTRL_REG(base)                      ((base)->SCTRL)
#define ADC_PWR2_REG(base)                       ((base)->PWR2)
#define ADC_CTRL3_REG(base)                      ((base)->CTRL3)
#define ADC_SCHLTEN_REG(base)                    ((base)->SCHLTEN)
#define ADC_ZXCTRL3_REG(base)                    ((base)->ZXCTRL3)
#define ADC_CLIST5_REG(base)                     ((base)->CLIST5)
#define ADC_SDIS2_REG(base)                      ((base)->SDIS2)
#define ADC_RDY2_REG(base)                       ((base)->RDY2)
#define ADC_LOLIMSTAT2_REG(base)                 ((base)->LOLIMSTAT2)
#define ADC_HILIMSTAT2_REG(base)                 ((base)->HILIMSTAT2)
#define ADC_ZXSTAT2_REG(base)                    ((base)->ZXSTAT2)
#define ADC_RSLT_2_REG(base,index)               ((base)->RSLT_2[index])
#define ADC_LOLIM_2_REG(base,index)              ((base)->LOLIM_2[index])
#define ADC_HILIM_2_REG(base,index)              ((base)->HILIM_2[index])
#define ADC_OFFST_2_REG(base,index)              ((base)->OFFST_2[index])
#define ADC_GC3_REG(base)                        ((base)->GC3)
#define ADC_SCTRL2_REG(base)                     ((base)->SCTRL2)
#define ADC_SCHLTEN2_REG(base)                   ((base)->SCHLTEN2)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/* CTRL1 Bit Fields */
#define ADC_CTRL1_SMODE_MASK                     0x7u
#define ADC_CTRL1_SMODE_SHIFT                    0
#define ADC_CTRL1_SMODE(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_SMODE_SHIFT))&ADC_CTRL1_SMODE_MASK)
#define ADC_CTRL1_CHNCFG_L_MASK                  0xF0u
#define ADC_CTRL1_CHNCFG_L_SHIFT                 4
#define ADC_CTRL1_CHNCFG_L(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CTRL1_CHNCFG_L_SHIFT))&ADC_CTRL1_CHNCFG_L_MASK)
#define ADC_CTRL1_HLMTIE_MASK                    0x100u
#define ADC_CTRL1_HLMTIE_SHIFT                   8
#define ADC_CTRL1_LLMTIE_MASK                    0x200u
#define ADC_CTRL1_LLMTIE_SHIFT                   9
#define ADC_CTRL1_ZCIE_MASK                      0x400u
#define ADC_CTRL1_ZCIE_SHIFT                     10
#define ADC_CTRL1_EOSIE0_MASK                    0x800u
#define ADC_CTRL1_EOSIE0_SHIFT                   11
#define ADC_CTRL1_SYNC0_MASK                     0x1000u
#define ADC_CTRL1_SYNC0_SHIFT                    12
#define ADC_CTRL1_START0_MASK                    0x2000u
#define ADC_CTRL1_START0_SHIFT                   13
#define ADC_CTRL1_STOP0_MASK                     0x4000u
#define ADC_CTRL1_STOP0_SHIFT                    14
#define ADC_CTRL1_DMAEN0_MASK                    0x8000u
#define ADC_CTRL1_DMAEN0_SHIFT                   15
/* CTRL2 Bit Fields */
#define ADC_CTRL2_DIV0_MASK                      0x3Fu
#define ADC_CTRL2_DIV0_SHIFT                     0
#define ADC_CTRL2_DIV0(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_DIV0_SHIFT))&ADC_CTRL2_DIV0_MASK)
#define ADC_CTRL2_SIMULT_MASK                    0x40u
#define ADC_CTRL2_SIMULT_SHIFT                   6
#define ADC_CTRL2_CHNCFG_H_MASK                  0x780u
#define ADC_CTRL2_CHNCFG_H_SHIFT                 7
#define ADC_CTRL2_CHNCFG_H(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CTRL2_CHNCFG_H_SHIFT))&ADC_CTRL2_CHNCFG_H_MASK)
#define ADC_CTRL2_EOSIE1_MASK                    0x800u
#define ADC_CTRL2_EOSIE1_SHIFT                   11
#define ADC_CTRL2_SYNC1_MASK                     0x1000u
#define ADC_CTRL2_SYNC1_SHIFT                    12
#define ADC_CTRL2_START1_MASK                    0x2000u
#define ADC_CTRL2_START1_SHIFT                   13
#define ADC_CTRL2_STOP1_MASK                     0x4000u
#define ADC_CTRL2_STOP1_SHIFT                    14
#define ADC_CTRL2_DMAEN1_MASK                    0x8000u
#define ADC_CTRL2_DMAEN1_SHIFT                   15
/* ZXCTRL1 Bit Fields */
#define ADC_ZXCTRL1_ZCE0_MASK                    0x3u
#define ADC_ZXCTRL1_ZCE0_SHIFT                   0
#define ADC_ZXCTRL1_ZCE0(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE0_SHIFT))&ADC_ZXCTRL1_ZCE0_MASK)
#define ADC_ZXCTRL1_ZCE1_MASK                    0xCu
#define ADC_ZXCTRL1_ZCE1_SHIFT                   2
#define ADC_ZXCTRL1_ZCE1(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE1_SHIFT))&ADC_ZXCTRL1_ZCE1_MASK)
#define ADC_ZXCTRL1_ZCE2_MASK                    0x30u
#define ADC_ZXCTRL1_ZCE2_SHIFT                   4
#define ADC_ZXCTRL1_ZCE2(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE2_SHIFT))&ADC_ZXCTRL1_ZCE2_MASK)
#define ADC_ZXCTRL1_ZCE3_MASK                    0xC0u
#define ADC_ZXCTRL1_ZCE3_SHIFT                   6
#define ADC_ZXCTRL1_ZCE3(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE3_SHIFT))&ADC_ZXCTRL1_ZCE3_MASK)
#define ADC_ZXCTRL1_ZCE4_MASK                    0x300u
#define ADC_ZXCTRL1_ZCE4_SHIFT                   8
#define ADC_ZXCTRL1_ZCE4(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE4_SHIFT))&ADC_ZXCTRL1_ZCE4_MASK)
#define ADC_ZXCTRL1_ZCE5_MASK                    0xC00u
#define ADC_ZXCTRL1_ZCE5_SHIFT                   10
#define ADC_ZXCTRL1_ZCE5(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE5_SHIFT))&ADC_ZXCTRL1_ZCE5_MASK)
#define ADC_ZXCTRL1_ZCE6_MASK                    0x3000u
#define ADC_ZXCTRL1_ZCE6_SHIFT                   12
#define ADC_ZXCTRL1_ZCE6(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE6_SHIFT))&ADC_ZXCTRL1_ZCE6_MASK)
#define ADC_ZXCTRL1_ZCE7_MASK                    0xC000u
#define ADC_ZXCTRL1_ZCE7_SHIFT                   14
#define ADC_ZXCTRL1_ZCE7(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL1_ZCE7_SHIFT))&ADC_ZXCTRL1_ZCE7_MASK)
/* ZXCTRL2 Bit Fields */
#define ADC_ZXCTRL2_ZCE8_MASK                    0x3u
#define ADC_ZXCTRL2_ZCE8_SHIFT                   0
#define ADC_ZXCTRL2_ZCE8(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE8_SHIFT))&ADC_ZXCTRL2_ZCE8_MASK)
#define ADC_ZXCTRL2_ZCE9_MASK                    0xCu
#define ADC_ZXCTRL2_ZCE9_SHIFT                   2
#define ADC_ZXCTRL2_ZCE9(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE9_SHIFT))&ADC_ZXCTRL2_ZCE9_MASK)
#define ADC_ZXCTRL2_ZCE10_MASK                   0x30u
#define ADC_ZXCTRL2_ZCE10_SHIFT                  4
#define ADC_ZXCTRL2_ZCE10(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE10_SHIFT))&ADC_ZXCTRL2_ZCE10_MASK)
#define ADC_ZXCTRL2_ZCE11_MASK                   0xC0u
#define ADC_ZXCTRL2_ZCE11_SHIFT                  6
#define ADC_ZXCTRL2_ZCE11(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE11_SHIFT))&ADC_ZXCTRL2_ZCE11_MASK)
#define ADC_ZXCTRL2_ZCE12_MASK                   0x300u
#define ADC_ZXCTRL2_ZCE12_SHIFT                  8
#define ADC_ZXCTRL2_ZCE12(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE12_SHIFT))&ADC_ZXCTRL2_ZCE12_MASK)
#define ADC_ZXCTRL2_ZCE13_MASK                   0xC00u
#define ADC_ZXCTRL2_ZCE13_SHIFT                  10
#define ADC_ZXCTRL2_ZCE13(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE13_SHIFT))&ADC_ZXCTRL2_ZCE13_MASK)
#define ADC_ZXCTRL2_ZCE14_MASK                   0x3000u
#define ADC_ZXCTRL2_ZCE14_SHIFT                  12
#define ADC_ZXCTRL2_ZCE14(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE14_SHIFT))&ADC_ZXCTRL2_ZCE14_MASK)
#define ADC_ZXCTRL2_ZCE15_MASK                   0xC000u
#define ADC_ZXCTRL2_ZCE15_SHIFT                  14
#define ADC_ZXCTRL2_ZCE15(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL2_ZCE15_SHIFT))&ADC_ZXCTRL2_ZCE15_MASK)
/* CLIST1 Bit Fields */
#define ADC_CLIST1_SAMPLE0_MASK                  0xFu
#define ADC_CLIST1_SAMPLE0_SHIFT                 0
#define ADC_CLIST1_SAMPLE0(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE0_SHIFT))&ADC_CLIST1_SAMPLE0_MASK)
#define ADC_CLIST1_SAMPLE1_MASK                  0xF0u
#define ADC_CLIST1_SAMPLE1_SHIFT                 4
#define ADC_CLIST1_SAMPLE1(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE1_SHIFT))&ADC_CLIST1_SAMPLE1_MASK)
#define ADC_CLIST1_SAMPLE2_MASK                  0xF00u
#define ADC_CLIST1_SAMPLE2_SHIFT                 8
#define ADC_CLIST1_SAMPLE2(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE2_SHIFT))&ADC_CLIST1_SAMPLE2_MASK)
#define ADC_CLIST1_SAMPLE3_MASK                  0xF000u
#define ADC_CLIST1_SAMPLE3_SHIFT                 12
#define ADC_CLIST1_SAMPLE3(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST1_SAMPLE3_SHIFT))&ADC_CLIST1_SAMPLE3_MASK)
/* CLIST2 Bit Fields */
#define ADC_CLIST2_SAMPLE4_MASK                  0xFu
#define ADC_CLIST2_SAMPLE4_SHIFT                 0
#define ADC_CLIST2_SAMPLE4(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE4_SHIFT))&ADC_CLIST2_SAMPLE4_MASK)
#define ADC_CLIST2_SAMPLE5_MASK                  0xF0u
#define ADC_CLIST2_SAMPLE5_SHIFT                 4
#define ADC_CLIST2_SAMPLE5(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE5_SHIFT))&ADC_CLIST2_SAMPLE5_MASK)
#define ADC_CLIST2_SAMPLE6_MASK                  0xF00u
#define ADC_CLIST2_SAMPLE6_SHIFT                 8
#define ADC_CLIST2_SAMPLE6(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE6_SHIFT))&ADC_CLIST2_SAMPLE6_MASK)
#define ADC_CLIST2_SAMPLE7_MASK                  0xF000u
#define ADC_CLIST2_SAMPLE7_SHIFT                 12
#define ADC_CLIST2_SAMPLE7(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST2_SAMPLE7_SHIFT))&ADC_CLIST2_SAMPLE7_MASK)
/* CLIST3 Bit Fields */
#define ADC_CLIST3_SAMPLE8_MASK                  0xFu
#define ADC_CLIST3_SAMPLE8_SHIFT                 0
#define ADC_CLIST3_SAMPLE8(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE8_SHIFT))&ADC_CLIST3_SAMPLE8_MASK)
#define ADC_CLIST3_SAMPLE9_MASK                  0xF0u
#define ADC_CLIST3_SAMPLE9_SHIFT                 4
#define ADC_CLIST3_SAMPLE9(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE9_SHIFT))&ADC_CLIST3_SAMPLE9_MASK)
#define ADC_CLIST3_SAMPLE10_MASK                 0xF00u
#define ADC_CLIST3_SAMPLE10_SHIFT                8
#define ADC_CLIST3_SAMPLE10(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE10_SHIFT))&ADC_CLIST3_SAMPLE10_MASK)
#define ADC_CLIST3_SAMPLE11_MASK                 0xF000u
#define ADC_CLIST3_SAMPLE11_SHIFT                12
#define ADC_CLIST3_SAMPLE11(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST3_SAMPLE11_SHIFT))&ADC_CLIST3_SAMPLE11_MASK)
/* CLIST4 Bit Fields */
#define ADC_CLIST4_SAMPLE12_MASK                 0xFu
#define ADC_CLIST4_SAMPLE12_SHIFT                0
#define ADC_CLIST4_SAMPLE12(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE12_SHIFT))&ADC_CLIST4_SAMPLE12_MASK)
#define ADC_CLIST4_SAMPLE13_MASK                 0xF0u
#define ADC_CLIST4_SAMPLE13_SHIFT                4
#define ADC_CLIST4_SAMPLE13(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE13_SHIFT))&ADC_CLIST4_SAMPLE13_MASK)
#define ADC_CLIST4_SAMPLE14_MASK                 0xF00u
#define ADC_CLIST4_SAMPLE14_SHIFT                8
#define ADC_CLIST4_SAMPLE14(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE14_SHIFT))&ADC_CLIST4_SAMPLE14_MASK)
#define ADC_CLIST4_SAMPLE15_MASK                 0xF000u
#define ADC_CLIST4_SAMPLE15_SHIFT                12
#define ADC_CLIST4_SAMPLE15(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST4_SAMPLE15_SHIFT))&ADC_CLIST4_SAMPLE15_MASK)
/* SDIS Bit Fields */
#define ADC_SDIS_DS_MASK                         0xFFFFu
#define ADC_SDIS_DS_SHIFT                        0
#define ADC_SDIS_DS(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_SDIS_DS_SHIFT))&ADC_SDIS_DS_MASK)
/* STAT Bit Fields */
#define ADC_STAT_UNDEFINED_MASK                  0xFFu
#define ADC_STAT_UNDEFINED_SHIFT                 0
#define ADC_STAT_UNDEFINED(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_STAT_UNDEFINED_SHIFT))&ADC_STAT_UNDEFINED_MASK)
#define ADC_STAT_HLMTI_MASK                      0x100u
#define ADC_STAT_HLMTI_SHIFT                     8
#define ADC_STAT_LLMTI_MASK                      0x200u
#define ADC_STAT_LLMTI_SHIFT                     9
#define ADC_STAT_ZCI_MASK                        0x400u
#define ADC_STAT_ZCI_SHIFT                       10
#define ADC_STAT_EOSI0_MASK                      0x800u
#define ADC_STAT_EOSI0_SHIFT                     11
#define ADC_STAT_EOSI1_MASK                      0x1000u
#define ADC_STAT_EOSI1_SHIFT                     12
#define ADC_STAT_CIP1_MASK                       0x4000u
#define ADC_STAT_CIP1_SHIFT                      14
#define ADC_STAT_CIP0_MASK                       0x8000u
#define ADC_STAT_CIP0_SHIFT                      15
/* RDY Bit Fields */
#define ADC_RDY_RDY_MASK                         0xFFFFu
#define ADC_RDY_RDY_SHIFT                        0
#define ADC_RDY_RDY(x)                           (((uint16_t)(((uint16_t)(x))<<ADC_RDY_RDY_SHIFT))&ADC_RDY_RDY_MASK)
/* LOLIMSTAT Bit Fields */
#define ADC_LOLIMSTAT_LLS_MASK                   0xFFFFu
#define ADC_LOLIMSTAT_LLS_SHIFT                  0
#define ADC_LOLIMSTAT_LLS(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_LOLIMSTAT_LLS_SHIFT))&ADC_LOLIMSTAT_LLS_MASK)
/* HILIMSTAT Bit Fields */
#define ADC_HILIMSTAT_HLS_MASK                   0xFFFFu
#define ADC_HILIMSTAT_HLS_SHIFT                  0
#define ADC_HILIMSTAT_HLS(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_HILIMSTAT_HLS_SHIFT))&ADC_HILIMSTAT_HLS_MASK)
/* ZXSTAT Bit Fields */
#define ADC_ZXSTAT_ZCS_MASK                      0xFFFFu
#define ADC_ZXSTAT_ZCS_SHIFT                     0
#define ADC_ZXSTAT_ZCS(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_ZXSTAT_ZCS_SHIFT))&ADC_ZXSTAT_ZCS_MASK)
/* RSLT Bit Fields */
#define ADC_RSLT_RSLT_MASK                       0x7FF8u
#define ADC_RSLT_RSLT_SHIFT                      3
#define ADC_RSLT_RSLT(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_RSLT_RSLT_SHIFT))&ADC_RSLT_RSLT_MASK)
#define ADC_RSLT_SEXT_MASK                       0x8000u
#define ADC_RSLT_SEXT_SHIFT                      15
/* LOLIM Bit Fields */
#define ADC_LOLIM_LLMT_MASK                      0x7FF8u
#define ADC_LOLIM_LLMT_SHIFT                     3
#define ADC_LOLIM_LLMT(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_LOLIM_LLMT_SHIFT))&ADC_LOLIM_LLMT_MASK)
/* HILIM Bit Fields */
#define ADC_HILIM_HLMT_MASK                      0x7FF8u
#define ADC_HILIM_HLMT_SHIFT                     3
#define ADC_HILIM_HLMT(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_HILIM_HLMT_SHIFT))&ADC_HILIM_HLMT_MASK)
/* OFFST Bit Fields */
#define ADC_OFFST_OFFSET_MASK                    0x7FF8u
#define ADC_OFFST_OFFSET_SHIFT                   3
#define ADC_OFFST_OFFSET(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_OFFST_OFFSET_SHIFT))&ADC_OFFST_OFFSET_MASK)
/* PWR Bit Fields */
#define ADC_PWR_PD0_MASK                         0x1u
#define ADC_PWR_PD0_SHIFT                        0
#define ADC_PWR_PD1_MASK                         0x2u
#define ADC_PWR_PD1_SHIFT                        1
#define ADC_PWR_APD_MASK                         0x8u
#define ADC_PWR_APD_SHIFT                        3
#define ADC_PWR_PUDELAY_MASK                     0x3F0u
#define ADC_PWR_PUDELAY_SHIFT                    4
#define ADC_PWR_PUDELAY(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_PWR_PUDELAY_SHIFT))&ADC_PWR_PUDELAY_MASK)
#define ADC_PWR_PSTS0_MASK                       0x400u
#define ADC_PWR_PSTS0_SHIFT                      10
#define ADC_PWR_PSTS1_MASK                       0x800u
#define ADC_PWR_PSTS1_SHIFT                      11
#define ADC_PWR_ASB_MASK                         0x8000u
#define ADC_PWR_ASB_SHIFT                        15
/* CAL Bit Fields */
#define ADC_CAL_SEL_VREFL_A_MASK                 0x1000u
#define ADC_CAL_SEL_VREFL_A_SHIFT                12
#define ADC_CAL_SEL_VREFH_A_MASK                 0x2000u
#define ADC_CAL_SEL_VREFH_A_SHIFT                13
#define ADC_CAL_SEL_VREFL_B_MASK                 0x4000u
#define ADC_CAL_SEL_VREFL_B_SHIFT                14
#define ADC_CAL_SEL_VREFH_B_MASK                 0x8000u
#define ADC_CAL_SEL_VREFH_B_SHIFT                15
/* GC1 Bit Fields */
#define ADC_GC1_GAIN0_MASK                       0x3u
#define ADC_GC1_GAIN0_SHIFT                      0
#define ADC_GC1_GAIN0(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN0_SHIFT))&ADC_GC1_GAIN0_MASK)
#define ADC_GC1_GAIN1_MASK                       0xCu
#define ADC_GC1_GAIN1_SHIFT                      2
#define ADC_GC1_GAIN1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN1_SHIFT))&ADC_GC1_GAIN1_MASK)
#define ADC_GC1_GAIN2_MASK                       0x30u
#define ADC_GC1_GAIN2_SHIFT                      4
#define ADC_GC1_GAIN2(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN2_SHIFT))&ADC_GC1_GAIN2_MASK)
#define ADC_GC1_GAIN3_MASK                       0xC0u
#define ADC_GC1_GAIN3_SHIFT                      6
#define ADC_GC1_GAIN3(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN3_SHIFT))&ADC_GC1_GAIN3_MASK)
#define ADC_GC1_GAIN4_MASK                       0x300u
#define ADC_GC1_GAIN4_SHIFT                      8
#define ADC_GC1_GAIN4(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN4_SHIFT))&ADC_GC1_GAIN4_MASK)
#define ADC_GC1_GAIN5_MASK                       0xC00u
#define ADC_GC1_GAIN5_SHIFT                      10
#define ADC_GC1_GAIN5(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN5_SHIFT))&ADC_GC1_GAIN5_MASK)
#define ADC_GC1_GAIN6_MASK                       0x3000u
#define ADC_GC1_GAIN6_SHIFT                      12
#define ADC_GC1_GAIN6(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN6_SHIFT))&ADC_GC1_GAIN6_MASK)
#define ADC_GC1_GAIN7_MASK                       0xC000u
#define ADC_GC1_GAIN7_SHIFT                      14
#define ADC_GC1_GAIN7(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC1_GAIN7_SHIFT))&ADC_GC1_GAIN7_MASK)
/* GC2 Bit Fields */
#define ADC_GC2_GAIN8_MASK                       0x3u
#define ADC_GC2_GAIN8_SHIFT                      0
#define ADC_GC2_GAIN8(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN8_SHIFT))&ADC_GC2_GAIN8_MASK)
#define ADC_GC2_GAIN9_MASK                       0xCu
#define ADC_GC2_GAIN9_SHIFT                      2
#define ADC_GC2_GAIN9(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN9_SHIFT))&ADC_GC2_GAIN9_MASK)
#define ADC_GC2_GAIN10_MASK                      0x30u
#define ADC_GC2_GAIN10_SHIFT                     4
#define ADC_GC2_GAIN10(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN10_SHIFT))&ADC_GC2_GAIN10_MASK)
#define ADC_GC2_GAIN11_MASK                      0xC0u
#define ADC_GC2_GAIN11_SHIFT                     6
#define ADC_GC2_GAIN11(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN11_SHIFT))&ADC_GC2_GAIN11_MASK)
#define ADC_GC2_GAIN12_MASK                      0x300u
#define ADC_GC2_GAIN12_SHIFT                     8
#define ADC_GC2_GAIN12(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN12_SHIFT))&ADC_GC2_GAIN12_MASK)
#define ADC_GC2_GAIN13_MASK                      0xC00u
#define ADC_GC2_GAIN13_SHIFT                     10
#define ADC_GC2_GAIN13(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN13_SHIFT))&ADC_GC2_GAIN13_MASK)
#define ADC_GC2_GAIN14_MASK                      0x3000u
#define ADC_GC2_GAIN14_SHIFT                     12
#define ADC_GC2_GAIN14(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN14_SHIFT))&ADC_GC2_GAIN14_MASK)
#define ADC_GC2_GAIN15_MASK                      0xC000u
#define ADC_GC2_GAIN15_SHIFT                     14
#define ADC_GC2_GAIN15(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC2_GAIN15_SHIFT))&ADC_GC2_GAIN15_MASK)
/* SCTRL Bit Fields */
#define ADC_SCTRL_SC_MASK                        0xFFFFu
#define ADC_SCTRL_SC_SHIFT                       0
#define ADC_SCTRL_SC(x)                          (((uint16_t)(((uint16_t)(x))<<ADC_SCTRL_SC_SHIFT))&ADC_SCTRL_SC_MASK)
/* PWR2 Bit Fields */
#define ADC_PWR2_DIV1_MASK                       0x3F00u
#define ADC_PWR2_DIV1_SHIFT                      8
#define ADC_PWR2_DIV1(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_PWR2_DIV1_SHIFT))&ADC_PWR2_DIV1_MASK)
/* CTRL3 Bit Fields */
#define ADC_CTRL3_DMASRC_MASK                    0x40u
#define ADC_CTRL3_DMASRC_SHIFT                   6
#define ADC_CTRL3_UPDEN_L_MASK                   0xF00u
#define ADC_CTRL3_UPDEN_L_SHIFT                  8
#define ADC_CTRL3_UPDEN_L(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_CTRL3_UPDEN_L_SHIFT))&ADC_CTRL3_UPDEN_L_MASK)
#define ADC_CTRL3_UPDEN_H_MASK                   0xF000u
#define ADC_CTRL3_UPDEN_H_SHIFT                  12
#define ADC_CTRL3_UPDEN_H(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_CTRL3_UPDEN_H_SHIFT))&ADC_CTRL3_UPDEN_H_MASK)
/* SCHLTEN Bit Fields */
#define ADC_SCHLTEN_SCHLTEN_MASK                 0xFFFFu
#define ADC_SCHLTEN_SCHLTEN_SHIFT                0
#define ADC_SCHLTEN_SCHLTEN(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_SCHLTEN_SCHLTEN_SHIFT))&ADC_SCHLTEN_SCHLTEN_MASK)
/* ZXCTRL3 Bit Fields */
#define ADC_ZXCTRL3_ZCE16_MASK                   0x3u
#define ADC_ZXCTRL3_ZCE16_SHIFT                  0
#define ADC_ZXCTRL3_ZCE16(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL3_ZCE16_SHIFT))&ADC_ZXCTRL3_ZCE16_MASK)
#define ADC_ZXCTRL3_ZCE17_MASK                   0xCu
#define ADC_ZXCTRL3_ZCE17_SHIFT                  2
#define ADC_ZXCTRL3_ZCE17(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL3_ZCE17_SHIFT))&ADC_ZXCTRL3_ZCE17_MASK)
#define ADC_ZXCTRL3_ZCE18_MASK                   0x30u
#define ADC_ZXCTRL3_ZCE18_SHIFT                  4
#define ADC_ZXCTRL3_ZCE18(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL3_ZCE18_SHIFT))&ADC_ZXCTRL3_ZCE18_MASK)
#define ADC_ZXCTRL3_ZCE19_MASK                   0xC0u
#define ADC_ZXCTRL3_ZCE19_SHIFT                  6
#define ADC_ZXCTRL3_ZCE19(x)                     (((uint16_t)(((uint16_t)(x))<<ADC_ZXCTRL3_ZCE19_SHIFT))&ADC_ZXCTRL3_ZCE19_MASK)
/* CLIST5 Bit Fields */
#define ADC_CLIST5_SAMPLE16_MASK                 0x3u
#define ADC_CLIST5_SAMPLE16_SHIFT                0
#define ADC_CLIST5_SAMPLE16(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST5_SAMPLE16_SHIFT))&ADC_CLIST5_SAMPLE16_MASK)
#define ADC_CLIST5_SAMPLE17_MASK                 0xCu
#define ADC_CLIST5_SAMPLE17_SHIFT                2
#define ADC_CLIST5_SAMPLE17(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST5_SAMPLE17_SHIFT))&ADC_CLIST5_SAMPLE17_MASK)
#define ADC_CLIST5_SAMPLE18_MASK                 0x30u
#define ADC_CLIST5_SAMPLE18_SHIFT                4
#define ADC_CLIST5_SAMPLE18(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST5_SAMPLE18_SHIFT))&ADC_CLIST5_SAMPLE18_MASK)
#define ADC_CLIST5_SAMPLE19_MASK                 0xC0u
#define ADC_CLIST5_SAMPLE19_SHIFT                6
#define ADC_CLIST5_SAMPLE19(x)                   (((uint16_t)(((uint16_t)(x))<<ADC_CLIST5_SAMPLE19_SHIFT))&ADC_CLIST5_SAMPLE19_MASK)
#define ADC_CLIST5_SEL_TEMP_0_MASK               0x100u
#define ADC_CLIST5_SEL_TEMP_0_SHIFT              8
#define ADC_CLIST5_SEL_INTERNAL_0_MASK           0x200u
#define ADC_CLIST5_SEL_INTERNAL_0_SHIFT          9
#define ADC_CLIST5_SEL_TEMP_1_MASK               0x400u
#define ADC_CLIST5_SEL_TEMP_1_SHIFT              10
#define ADC_CLIST5_SEL_INTERNAL_1_MASK           0x800u
#define ADC_CLIST5_SEL_INTERNAL_1_SHIFT          11
/* SDIS2 Bit Fields */
#define ADC_SDIS2_DS_MASK                        0xFu
#define ADC_SDIS2_DS_SHIFT                       0
#define ADC_SDIS2_DS(x)                          (((uint16_t)(((uint16_t)(x))<<ADC_SDIS2_DS_SHIFT))&ADC_SDIS2_DS_MASK)
/* RDY2 Bit Fields */
#define ADC_RDY2_RDY_MASK                        0xFu
#define ADC_RDY2_RDY_SHIFT                       0
#define ADC_RDY2_RDY(x)                          (((uint16_t)(((uint16_t)(x))<<ADC_RDY2_RDY_SHIFT))&ADC_RDY2_RDY_MASK)
/* LOLIMSTAT2 Bit Fields */
#define ADC_LOLIMSTAT2_LLS_MASK                  0xFu
#define ADC_LOLIMSTAT2_LLS_SHIFT                 0
#define ADC_LOLIMSTAT2_LLS(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_LOLIMSTAT2_LLS_SHIFT))&ADC_LOLIMSTAT2_LLS_MASK)
/* HILIMSTAT2 Bit Fields */
#define ADC_HILIMSTAT2_HLS_MASK                  0xFu
#define ADC_HILIMSTAT2_HLS_SHIFT                 0
#define ADC_HILIMSTAT2_HLS(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_HILIMSTAT2_HLS_SHIFT))&ADC_HILIMSTAT2_HLS_MASK)
/* ZXSTAT2 Bit Fields */
#define ADC_ZXSTAT2_ZCS_MASK                     0xFu
#define ADC_ZXSTAT2_ZCS_SHIFT                    0
#define ADC_ZXSTAT2_ZCS(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_ZXSTAT2_ZCS_SHIFT))&ADC_ZXSTAT2_ZCS_MASK)
/* RSLT_2 Bit Fields */
#define ADC_RSLT_2_RSLT_MASK                     0x7FF8u
#define ADC_RSLT_2_RSLT_SHIFT                    3
#define ADC_RSLT_2_RSLT(x)                       (((uint16_t)(((uint16_t)(x))<<ADC_RSLT_2_RSLT_SHIFT))&ADC_RSLT_2_RSLT_MASK)
#define ADC_RSLT_2_SEXT_MASK                     0x8000u
#define ADC_RSLT_2_SEXT_SHIFT                    15
/* LOLIM_2 Bit Fields */
#define ADC_LOLIM_2_LLMT_MASK                    0x7FF8u
#define ADC_LOLIM_2_LLMT_SHIFT                   3
#define ADC_LOLIM_2_LLMT(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_LOLIM_2_LLMT_SHIFT))&ADC_LOLIM_2_LLMT_MASK)
/* HILIM_2 Bit Fields */
#define ADC_HILIM_2_HLMT_MASK                    0x7FF8u
#define ADC_HILIM_2_HLMT_SHIFT                   3
#define ADC_HILIM_2_HLMT(x)                      (((uint16_t)(((uint16_t)(x))<<ADC_HILIM_2_HLMT_SHIFT))&ADC_HILIM_2_HLMT_MASK)
/* OFFST_2 Bit Fields */
#define ADC_OFFST_2_OFFSET_MASK                  0x7FF8u
#define ADC_OFFST_2_OFFSET_SHIFT                 3
#define ADC_OFFST_2_OFFSET(x)                    (((uint16_t)(((uint16_t)(x))<<ADC_OFFST_2_OFFSET_SHIFT))&ADC_OFFST_2_OFFSET_MASK)
/* GC3 Bit Fields */
#define ADC_GC3_GAIN16_MASK                      0x3u
#define ADC_GC3_GAIN16_SHIFT                     0
#define ADC_GC3_GAIN16(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC3_GAIN16_SHIFT))&ADC_GC3_GAIN16_MASK)
#define ADC_GC3_GAIN17_MASK                      0xCu
#define ADC_GC3_GAIN17_SHIFT                     2
#define ADC_GC3_GAIN17(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC3_GAIN17_SHIFT))&ADC_GC3_GAIN17_MASK)
#define ADC_GC3_GAIN18_MASK                      0x30u
#define ADC_GC3_GAIN18_SHIFT                     4
#define ADC_GC3_GAIN18(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC3_GAIN18_SHIFT))&ADC_GC3_GAIN18_MASK)
#define ADC_GC3_GAIN19_MASK                      0xC0u
#define ADC_GC3_GAIN19_SHIFT                     6
#define ADC_GC3_GAIN19(x)                        (((uint16_t)(((uint16_t)(x))<<ADC_GC3_GAIN19_SHIFT))&ADC_GC3_GAIN19_MASK)
/* SCTRL2 Bit Fields */
#define ADC_SCTRL2_SC_MASK                       0xFu
#define ADC_SCTRL2_SC_SHIFT                      0
#define ADC_SCTRL2_SC(x)                         (((uint16_t)(((uint16_t)(x))<<ADC_SCTRL2_SC_SHIFT))&ADC_SCTRL2_SC_MASK)
/* SCHLTEN2 Bit Fields */
#define ADC_SCHLTEN2_SCHLTEN_MASK                0xFu
#define ADC_SCHLTEN2_SCHLTEN_SHIFT               0
#define ADC_SCHLTEN2_SCHLTEN(x)                  (((uint16_t)(((uint16_t)(x))<<ADC_SCHLTEN2_SCHLTEN_SHIFT))&ADC_SCHLTEN2_SCHLTEN_MASK)

/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC base pointer */
#define ADC_BASE_PTR                             ((ADC_MemMapPtr)0xE500u)
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register instance definitions */
/* ADC */
#define ADC_CTRL1                                ADC_CTRL1_REG(ADC_BASE_PTR)
#define ADC_CTRL2                                ADC_CTRL2_REG(ADC_BASE_PTR)
#define ADC_ZXCTRL1                              ADC_ZXCTRL1_REG(ADC_BASE_PTR)
#define ADC_ZXCTRL2                              ADC_ZXCTRL2_REG(ADC_BASE_PTR)
#define ADC_CLIST1                               ADC_CLIST1_REG(ADC_BASE_PTR)
#define ADC_CLIST2                               ADC_CLIST2_REG(ADC_BASE_PTR)
#define ADC_CLIST3                               ADC_CLIST3_REG(ADC_BASE_PTR)
#define ADC_CLIST4                               ADC_CLIST4_REG(ADC_BASE_PTR)
#define ADC_SDIS                                 ADC_SDIS_REG(ADC_BASE_PTR)
#define ADC_STAT                                 ADC_STAT_REG(ADC_BASE_PTR)
#define ADC_RDY                                  ADC_RDY_REG(ADC_BASE_PTR)
#define ADC_LOLIMSTAT                            ADC_LOLIMSTAT_REG(ADC_BASE_PTR)
#define ADC_HILIMSTAT                            ADC_HILIMSTAT_REG(ADC_BASE_PTR)
#define ADC_ZXSTAT                               ADC_ZXSTAT_REG(ADC_BASE_PTR)
#define ADC_RSLT0                                ADC_RSLT_REG(ADC_BASE_PTR,0)
#define ADC_RSLT1                                ADC_RSLT_REG(ADC_BASE_PTR,1)
#define ADC_RSLT2                                ADC_RSLT_REG(ADC_BASE_PTR,2)
#define ADC_RSLT3                                ADC_RSLT_REG(ADC_BASE_PTR,3)
#define ADC_RSLT4                                ADC_RSLT_REG(ADC_BASE_PTR,4)
#define ADC_RSLT5                                ADC_RSLT_REG(ADC_BASE_PTR,5)
#define ADC_RSLT6                                ADC_RSLT_REG(ADC_BASE_PTR,6)
#define ADC_RSLT7                                ADC_RSLT_REG(ADC_BASE_PTR,7)
#define ADC_RSLT8                                ADC_RSLT_REG(ADC_BASE_PTR,8)
#define ADC_RSLT9                                ADC_RSLT_REG(ADC_BASE_PTR,9)
#define ADC_RSLT10                               ADC_RSLT_REG(ADC_BASE_PTR,10)
#define ADC_RSLT11                               ADC_RSLT_REG(ADC_BASE_PTR,11)
#define ADC_RSLT12                               ADC_RSLT_REG(ADC_BASE_PTR,12)
#define ADC_RSLT13                               ADC_RSLT_REG(ADC_BASE_PTR,13)
#define ADC_RSLT14                               ADC_RSLT_REG(ADC_BASE_PTR,14)
#define ADC_RSLT15                               ADC_RSLT_REG(ADC_BASE_PTR,15)
#define ADC_LOLIM0                               ADC_LOLIM_REG(ADC_BASE_PTR,0)
#define ADC_LOLIM1                               ADC_LOLIM_REG(ADC_BASE_PTR,1)
#define ADC_LOLIM2                               ADC_LOLIM_REG(ADC_BASE_PTR,2)
#define ADC_LOLIM3                               ADC_LOLIM_REG(ADC_BASE_PTR,3)
#define ADC_LOLIM4                               ADC_LOLIM_REG(ADC_BASE_PTR,4)
#define ADC_LOLIM5                               ADC_LOLIM_REG(ADC_BASE_PTR,5)
#define ADC_LOLIM6                               ADC_LOLIM_REG(ADC_BASE_PTR,6)
#define ADC_LOLIM7                               ADC_LOLIM_REG(ADC_BASE_PTR,7)
#define ADC_LOLIM8                               ADC_LOLIM_REG(ADC_BASE_PTR,8)
#define ADC_LOLIM9                               ADC_LOLIM_REG(ADC_BASE_PTR,9)
#define ADC_LOLIM10                              ADC_LOLIM_REG(ADC_BASE_PTR,10)
#define ADC_LOLIM11                              ADC_LOLIM_REG(ADC_BASE_PTR,11)
#define ADC_LOLIM12                              ADC_LOLIM_REG(ADC_BASE_PTR,12)
#define ADC_LOLIM13                              ADC_LOLIM_REG(ADC_BASE_PTR,13)
#define ADC_LOLIM14                              ADC_LOLIM_REG(ADC_BASE_PTR,14)
#define ADC_LOLIM15                              ADC_LOLIM_REG(ADC_BASE_PTR,15)
#define ADC_HILIM0                               ADC_HILIM_REG(ADC_BASE_PTR,0)
#define ADC_HILIM1                               ADC_HILIM_REG(ADC_BASE_PTR,1)
#define ADC_HILIM2                               ADC_HILIM_REG(ADC_BASE_PTR,2)
#define ADC_HILIM3                               ADC_HILIM_REG(ADC_BASE_PTR,3)
#define ADC_HILIM4                               ADC_HILIM_REG(ADC_BASE_PTR,4)
#define ADC_HILIM5                               ADC_HILIM_REG(ADC_BASE_PTR,5)
#define ADC_HILIM6                               ADC_HILIM_REG(ADC_BASE_PTR,6)
#define ADC_HILIM7                               ADC_HILIM_REG(ADC_BASE_PTR,7)
#define ADC_HILIM8                               ADC_HILIM_REG(ADC_BASE_PTR,8)
#define ADC_HILIM9                               ADC_HILIM_REG(ADC_BASE_PTR,9)
#define ADC_HILIM10                              ADC_HILIM_REG(ADC_BASE_PTR,10)
#define ADC_HILIM11                              ADC_HILIM_REG(ADC_BASE_PTR,11)
#define ADC_HILIM12                              ADC_HILIM_REG(ADC_BASE_PTR,12)
#define ADC_HILIM13                              ADC_HILIM_REG(ADC_BASE_PTR,13)
#define ADC_HILIM14                              ADC_HILIM_REG(ADC_BASE_PTR,14)
#define ADC_HILIM15                              ADC_HILIM_REG(ADC_BASE_PTR,15)
#define ADC_OFFST0                               ADC_OFFST_REG(ADC_BASE_PTR,0)
#define ADC_OFFST1                               ADC_OFFST_REG(ADC_BASE_PTR,1)
#define ADC_OFFST2                               ADC_OFFST_REG(ADC_BASE_PTR,2)
#define ADC_OFFST3                               ADC_OFFST_REG(ADC_BASE_PTR,3)
#define ADC_OFFST4                               ADC_OFFST_REG(ADC_BASE_PTR,4)
#define ADC_OFFST5                               ADC_OFFST_REG(ADC_BASE_PTR,5)
#define ADC_OFFST6                               ADC_OFFST_REG(ADC_BASE_PTR,6)
#define ADC_OFFST7                               ADC_OFFST_REG(ADC_BASE_PTR,7)
#define ADC_OFFST8                               ADC_OFFST_REG(ADC_BASE_PTR,8)
#define ADC_OFFST9                               ADC_OFFST_REG(ADC_BASE_PTR,9)
#define ADC_OFFST10                              ADC_OFFST_REG(ADC_BASE_PTR,10)
#define ADC_OFFST11                              ADC_OFFST_REG(ADC_BASE_PTR,11)
#define ADC_OFFST12                              ADC_OFFST_REG(ADC_BASE_PTR,12)
#define ADC_OFFST13                              ADC_OFFST_REG(ADC_BASE_PTR,13)
#define ADC_OFFST14                              ADC_OFFST_REG(ADC_BASE_PTR,14)
#define ADC_OFFST15                              ADC_OFFST_REG(ADC_BASE_PTR,15)
#define ADC_PWR                                  ADC_PWR_REG(ADC_BASE_PTR)
#define ADC_CAL                                  ADC_CAL_REG(ADC_BASE_PTR)
#define ADC_GC1                                  ADC_GC1_REG(ADC_BASE_PTR)
#define ADC_GC2                                  ADC_GC2_REG(ADC_BASE_PTR)
#define ADC_SCTRL                                ADC_SCTRL_REG(ADC_BASE_PTR)
#define ADC_PWR2                                 ADC_PWR2_REG(ADC_BASE_PTR)
#define ADC_CTRL3                                ADC_CTRL3_REG(ADC_BASE_PTR)
#define ADC_SCHLTEN                              ADC_SCHLTEN_REG(ADC_BASE_PTR)
#define ADC_ZXCTRL3                              ADC_ZXCTRL3_REG(ADC_BASE_PTR)
#define ADC_CLIST5                               ADC_CLIST5_REG(ADC_BASE_PTR)
#define ADC_SDIS2                                ADC_SDIS2_REG(ADC_BASE_PTR)
#define ADC_RDY2                                 ADC_RDY2_REG(ADC_BASE_PTR)
#define ADC_LOLIMSTAT2                           ADC_LOLIMSTAT2_REG(ADC_BASE_PTR)
#define ADC_HILIMSTAT2                           ADC_HILIMSTAT2_REG(ADC_BASE_PTR)
#define ADC_ZXSTAT2                              ADC_ZXSTAT2_REG(ADC_BASE_PTR)
#define ADC_RSLT216                              ADC_RSLT_2_REG(ADC_BASE_PTR,0)
#define ADC_RSLT217                              ADC_RSLT_2_REG(ADC_BASE_PTR,1)
#define ADC_RSLT218                              ADC_RSLT_2_REG(ADC_BASE_PTR,2)
#define ADC_RSLT219                              ADC_RSLT_2_REG(ADC_BASE_PTR,3)
#define ADC_LOLIM216                             ADC_LOLIM_2_REG(ADC_BASE_PTR,0)
#define ADC_LOLIM217                             ADC_LOLIM_2_REG(ADC_BASE_PTR,1)
#define ADC_LOLIM218                             ADC_LOLIM_2_REG(ADC_BASE_PTR,2)
#define ADC_LOLIM219                             ADC_LOLIM_2_REG(ADC_BASE_PTR,3)
#define ADC_HILIM216                             ADC_HILIM_2_REG(ADC_BASE_PTR,0)
#define ADC_HILIM217                             ADC_HILIM_2_REG(ADC_BASE_PTR,1)
#define ADC_HILIM218                             ADC_HILIM_2_REG(ADC_BASE_PTR,2)
#define ADC_HILIM219                             ADC_HILIM_2_REG(ADC_BASE_PTR,3)
#define ADC_OFFST216                             ADC_OFFST_2_REG(ADC_BASE_PTR,0)
#define ADC_OFFST217                             ADC_OFFST_2_REG(ADC_BASE_PTR,1)
#define ADC_OFFST218                             ADC_OFFST_2_REG(ADC_BASE_PTR,2)
#define ADC_OFFST219                             ADC_OFFST_2_REG(ADC_BASE_PTR,3)
#define ADC_GC3                                  ADC_GC3_REG(ADC_BASE_PTR)
#define ADC_SCTRL2                               ADC_SCTRL2_REG(ADC_BASE_PTR)
#define ADC_SCHLTEN2                             ADC_SCHLTEN2_REG(ADC_BASE_PTR)

/* ADC - Register array accessors */
#define ADC_RSLT(index)                          ADC_RSLT_REG(ADC_BASE_PTR,index)
#define ADC_LOLIM(index)                         ADC_LOLIM_REG(ADC_BASE_PTR,index)
#define ADC_HILIM(index)                         ADC_HILIM_REG(ADC_BASE_PTR,index)
#define ADC_OFFST(index)                         ADC_OFFST_REG(ADC_BASE_PTR,index)
#define ADC_RSLT_2(index)                        ADC_RSLT_2_REG(ADC_BASE_PTR,index)
#define ADC_LOLIM_2(index)                       ADC_LOLIM_2_REG(ADC_BASE_PTR,index)
#define ADC_HILIM_2(index)                       ADC_HILIM_2_REG(ADC_BASE_PTR,index)
#define ADC_OFFST_2(index)                       ADC_OFFST_2_REG(ADC_BASE_PTR,index)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group ADC_Peripheral */


/* ----------------------------------------------------------------------------
   -- AOI
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Peripheral AOI
 * @{
 */

/** AOI - Peripheral register structure */
typedef struct AOI_MemMap {
  struct {                                         /* offset: 0x0, array step: 0x2 */
    uint16_t BFCRT01;                                /**< Boolean Function Term 0 and 1 Configuration Register for EVENTn, array offset: 0x0, array step: 0x2 */
    uint16_t BFCRT23;                                /**< Boolean Function Term 2 and 3 Configuration Register for EVENTn, array offset: 0x1, array step: 0x2 */
  } BFCRT[4];
} volatile *AOI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- AOI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Accessor_Macros AOI - Register accessor macros
 * @{
 */


/* AOI - Register accessors */
#define AOI_BFCRT01_REG(base,index)              ((base)->BFCRT[index].BFCRT01)
#define AOI_BFCRT23_REG(base,index)              ((base)->BFCRT[index].BFCRT23)

/*!
 * @}
 */ /* end of group AOI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- AOI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Masks AOI Register Masks
 * @{
 */

/* BFCRT01 Bit Fields */
#define AOI_BFCRT01_PT1_DC_MASK                  0x3u
#define AOI_BFCRT01_PT1_DC_SHIFT                 0
#define AOI_BFCRT01_PT1_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_DC_SHIFT))&AOI_BFCRT01_PT1_DC_MASK)
#define AOI_BFCRT01_PT1_CC_MASK                  0xCu
#define AOI_BFCRT01_PT1_CC_SHIFT                 2
#define AOI_BFCRT01_PT1_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_CC_SHIFT))&AOI_BFCRT01_PT1_CC_MASK)
#define AOI_BFCRT01_PT1_BC_MASK                  0x30u
#define AOI_BFCRT01_PT1_BC_SHIFT                 4
#define AOI_BFCRT01_PT1_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_BC_SHIFT))&AOI_BFCRT01_PT1_BC_MASK)
#define AOI_BFCRT01_PT1_AC_MASK                  0xC0u
#define AOI_BFCRT01_PT1_AC_SHIFT                 6
#define AOI_BFCRT01_PT1_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT1_AC_SHIFT))&AOI_BFCRT01_PT1_AC_MASK)
#define AOI_BFCRT01_PT0_DC_MASK                  0x300u
#define AOI_BFCRT01_PT0_DC_SHIFT                 8
#define AOI_BFCRT01_PT0_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_DC_SHIFT))&AOI_BFCRT01_PT0_DC_MASK)
#define AOI_BFCRT01_PT0_CC_MASK                  0xC00u
#define AOI_BFCRT01_PT0_CC_SHIFT                 10
#define AOI_BFCRT01_PT0_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_CC_SHIFT))&AOI_BFCRT01_PT0_CC_MASK)
#define AOI_BFCRT01_PT0_BC_MASK                  0x3000u
#define AOI_BFCRT01_PT0_BC_SHIFT                 12
#define AOI_BFCRT01_PT0_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_BC_SHIFT))&AOI_BFCRT01_PT0_BC_MASK)
#define AOI_BFCRT01_PT0_AC_MASK                  0xC000u
#define AOI_BFCRT01_PT0_AC_SHIFT                 14
#define AOI_BFCRT01_PT0_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT01_PT0_AC_SHIFT))&AOI_BFCRT01_PT0_AC_MASK)
/* BFCRT23 Bit Fields */
#define AOI_BFCRT23_PT3_DC_MASK                  0x3u
#define AOI_BFCRT23_PT3_DC_SHIFT                 0
#define AOI_BFCRT23_PT3_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_DC_SHIFT))&AOI_BFCRT23_PT3_DC_MASK)
#define AOI_BFCRT23_PT3_CC_MASK                  0xCu
#define AOI_BFCRT23_PT3_CC_SHIFT                 2
#define AOI_BFCRT23_PT3_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_CC_SHIFT))&AOI_BFCRT23_PT3_CC_MASK)
#define AOI_BFCRT23_PT3_BC_MASK                  0x30u
#define AOI_BFCRT23_PT3_BC_SHIFT                 4
#define AOI_BFCRT23_PT3_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_BC_SHIFT))&AOI_BFCRT23_PT3_BC_MASK)
#define AOI_BFCRT23_PT3_AC_MASK                  0xC0u
#define AOI_BFCRT23_PT3_AC_SHIFT                 6
#define AOI_BFCRT23_PT3_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT3_AC_SHIFT))&AOI_BFCRT23_PT3_AC_MASK)
#define AOI_BFCRT23_PT2_DC_MASK                  0x300u
#define AOI_BFCRT23_PT2_DC_SHIFT                 8
#define AOI_BFCRT23_PT2_DC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_DC_SHIFT))&AOI_BFCRT23_PT2_DC_MASK)
#define AOI_BFCRT23_PT2_CC_MASK                  0xC00u
#define AOI_BFCRT23_PT2_CC_SHIFT                 10
#define AOI_BFCRT23_PT2_CC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_CC_SHIFT))&AOI_BFCRT23_PT2_CC_MASK)
#define AOI_BFCRT23_PT2_BC_MASK                  0x3000u
#define AOI_BFCRT23_PT2_BC_SHIFT                 12
#define AOI_BFCRT23_PT2_BC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_BC_SHIFT))&AOI_BFCRT23_PT2_BC_MASK)
#define AOI_BFCRT23_PT2_AC_MASK                  0xC000u
#define AOI_BFCRT23_PT2_AC_SHIFT                 14
#define AOI_BFCRT23_PT2_AC(x)                    (((uint16_t)(((uint16_t)(x))<<AOI_BFCRT23_PT2_AC_SHIFT))&AOI_BFCRT23_PT2_AC_MASK)

/*!
 * @}
 */ /* end of group AOI_Register_Masks */


/* AOI - Peripheral instance base addresses */
/** Peripheral AOI base pointer */
#define AOI_BASE_PTR                             ((AOI_MemMapPtr)0xE380u)
/** Array initializer of AOI peripheral base pointers */
#define AOI_BASE_PTRS                            { AOI_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- AOI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AOI_Register_Accessor_Macros AOI - Register accessor macros
 * @{
 */


/* AOI - Register instance definitions */
/* AOI */
#define AOI_BFCRT010                             AOI_BFCRT01_REG(AOI_BASE_PTR,0)
#define AOI_BFCRT230                             AOI_BFCRT23_REG(AOI_BASE_PTR,0)
#define AOI_BFCRT011                             AOI_BFCRT01_REG(AOI_BASE_PTR,1)
#define AOI_BFCRT231                             AOI_BFCRT23_REG(AOI_BASE_PTR,1)
#define AOI_BFCRT012                             AOI_BFCRT01_REG(AOI_BASE_PTR,2)
#define AOI_BFCRT232                             AOI_BFCRT23_REG(AOI_BASE_PTR,2)
#define AOI_BFCRT013                             AOI_BFCRT01_REG(AOI_BASE_PTR,3)
#define AOI_BFCRT233                             AOI_BFCRT23_REG(AOI_BASE_PTR,3)

/* AOI - Register array accessors */
#define AOI_BFCRT01(index)                       AOI_BFCRT01_REG(AOI_BASE_PTR,index)
#define AOI_BFCRT23(index)                       AOI_BFCRT23_REG(AOI_BASE_PTR,index)

/*!
 * @}
 */ /* end of group AOI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group AOI_Peripheral */


/* ----------------------------------------------------------------------------
   -- CAN
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral CAN
 * @{
 */

/** CAN - Peripheral register structure */
typedef struct CAN_MemMap {
  uint16_t CTL0;                                   /**< MSCAN Control Register 0, offset: 0x0 */
  uint16_t CTL1;                                   /**< MSCAN Control Register 1, offset: 0x1 */
  uint16_t BTR0;                                   /**< MSCAN Bus Timing Register 0, offset: 0x2 */
  uint16_t BTR1;                                   /**< MSCAN Bus Timing Register 1, offset: 0x3 */
  uint16_t RFLG;                                   /**< MSCAN Receiver Flag Register, offset: 0x4 */
  uint16_t RIER;                                   /**< MSCAN Receiver Interrupt Enable Register, offset: 0x5 */
  uint16_t TFLG;                                   /**< MSCAN Transmitter Flag Register, offset: 0x6 */
  uint16_t TIER;                                   /**< MSCAN Transmitter Interrupt Enable Register, offset: 0x7 */
  uint16_t TARQ;                                   /**< MSCAN Transmitter Message Abort Request Register, offset: 0x8 */
  uint16_t TAAK;                                   /**< MSCAN Transmitter Message Abort Acknowledge Register, offset: 0x9 */
  uint16_t TBSEL;                                  /**< MSCAN Transmit Buffer Selection Register, offset: 0xA */
  uint16_t IDAC;                                   /**< MSCAN Identifier Acceptance Control Register, offset: 0xB */
  uint16_t RESERVED_0[1];
  uint16_t MISC;                                   /**< MSCAN Miscellaneous Register, offset: 0xD */
  uint16_t RXERR;                                  /**< MSCAN Receive Error Counter Register, offset: 0xE */
  uint16_t TXERR;                                  /**< MSCAN Transmit Error Counter Register, offset: 0xF */
  uint16_t IDAR_BANK_1[4];                         /**< MSCAN Identifier Acceptance Registers (First Bank), array offset: 0x10, array step: 0x1 */
  uint16_t IDMR_BANK_1[4];                         /**< MSCAN Identifier Mask Registers (First Bank), array offset: 0x14, array step: 0x1 */
  uint16_t IDAR_BANK_2[4];                         /**< MSCAN Identifier Acceptance Registers (Second Bank), array offset: 0x18, array step: 0x1 */
  uint16_t IDMR_BANK_2[4];                         /**< MSCAN Identifier Mask Registers (Second Bank), array offset: 0x1C, array step: 0x1 */
  union {                                          /* offset: 0x20 */
    uint16_t RXFG_IDR0_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 0 - Extended Identifer Mapping, offset: 0x20 */
    uint16_t RXFG_IDR0_STD;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 0 - Standard Identifer Mapping, offset: 0x20 */
  };
  union {                                          /* offset: 0x21 */
    uint16_t RXFG_IDR1_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 1 - Extended Identifer Mapping, offset: 0x21 */
    uint16_t RXFG_IDR1_STD;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 1 - Standard Identifier Mapping, offset: 0x21 */
  };
  uint16_t RXFG_IDR2_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 2 - Extended Identifer Mapping, offset: 0x22 */
  uint16_t RXFG_IDR3_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 3 - Extended Identifier Mapping, offset: 0x23 */
  uint16_t RXFG_DSR[8];                            /**< Receive Buffer Data Segment Registers, array offset: 0x24, array step: 0x1 */
  uint16_t RXFG_DLR;                               /**< MSCAN Receive Buffer Data Length Register, offset: 0x2C */
  uint16_t RESERVED_1[1];
  uint16_t RXFG_TSRH;                              /**< Receive Buffer Time Stamp Register - High Byte, offset: 0x2E */
  uint16_t RXFG_TSRL;                              /**< Receive Buffer Time Stamp Register - Low Byte, offset: 0x2F */
  union {                                          /* offset: 0x30 */
    uint16_t TXFG_IDR0_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 0 - Extended Identifer Mapping, offset: 0x30 */
    uint16_t TXFG_IDR0_STD;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 0 - Standard Identifer Mapping, offset: 0x30 */
  };
  union {                                          /* offset: 0x31 */
    uint16_t TXFG_IDR1_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 1 - Extended Identifer Mapping, offset: 0x31 */
    uint16_t TXFG_IDR1_STD;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 1 - Standard Identifier Mapping, offset: 0x31 */
  };
  uint16_t TXFG_IDR2_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 2 - Extended Identifer Mapping, offset: 0x32 */
  uint16_t TXFG_IDR3_EXT;                          /**< MSCAN Receive and Transmit Buffer Identifier Register 3 - Extended Identifier Mapping, offset: 0x33 */
  uint16_t TXFG_DSR[8];                            /**< Transmit Buffer Data Segment Registers, array offset: 0x34, array step: 0x1 */
  uint16_t TXFG_DLR;                               /**< MSCAN Transmit Buffer Data Length Register, offset: 0x3C */
  uint16_t TXFG_TBPR;                              /**< MSCAN Transmit Buffer Priority Register, offset: 0x3D */
  uint16_t TXFG_TSRH;                              /**< Transmit Buffer Time Stamp Register - High Byte, offset: 0x3E */
  uint16_t TXFG_TSRL;                              /**< Transmit Buffer Time Stamp Register - Low Byte, offset: 0x3F */
} volatile *CAN_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register accessors */
#define CAN_CTL0_REG(base)                       ((base)->CTL0)
#define CAN_CTL1_REG(base)                       ((base)->CTL1)
#define CAN_BTR0_REG(base)                       ((base)->BTR0)
#define CAN_BTR1_REG(base)                       ((base)->BTR1)
#define CAN_RFLG_REG(base)                       ((base)->RFLG)
#define CAN_RIER_REG(base)                       ((base)->RIER)
#define CAN_TFLG_REG(base)                       ((base)->TFLG)
#define CAN_TIER_REG(base)                       ((base)->TIER)
#define CAN_TARQ_REG(base)                       ((base)->TARQ)
#define CAN_TAAK_REG(base)                       ((base)->TAAK)
#define CAN_TBSEL_REG(base)                      ((base)->TBSEL)
#define CAN_IDAC_REG(base)                       ((base)->IDAC)
#define CAN_MISC_REG(base)                       ((base)->MISC)
#define CAN_RXERR_REG(base)                      ((base)->RXERR)
#define CAN_TXERR_REG(base)                      ((base)->TXERR)
#define CAN_IDAR_BANK_1_REG(base,index)          ((base)->IDAR_BANK_1[index])
#define CAN_IDMR_BANK_1_REG(base,index)          ((base)->IDMR_BANK_1[index])
#define CAN_IDAR_BANK_2_REG(base,index)          ((base)->IDAR_BANK_2[index])
#define CAN_IDMR_BANK_2_REG(base,index)          ((base)->IDMR_BANK_2[index])
#define CAN_RXFG_IDR0_EXT_REG(base)              ((base)->RXFG_IDR0_EXT)
#define CAN_RXFG_IDR0_STD_REG(base)              ((base)->RXFG_IDR0_STD)
#define CAN_RXFG_IDR1_EXT_REG(base)              ((base)->RXFG_IDR1_EXT)
#define CAN_RXFG_IDR1_STD_REG(base)              ((base)->RXFG_IDR1_STD)
#define CAN_RXFG_IDR2_EXT_REG(base)              ((base)->RXFG_IDR2_EXT)
#define CAN_RXFG_IDR3_EXT_REG(base)              ((base)->RXFG_IDR3_EXT)
#define CAN_RXFG_DSR_REG(base,index)             ((base)->RXFG_DSR[index])
#define CAN_RXFG_DLR_REG(base)                   ((base)->RXFG_DLR)
#define CAN_RXFG_TSRH_REG(base)                  ((base)->RXFG_TSRH)
#define CAN_RXFG_TSRL_REG(base)                  ((base)->RXFG_TSRL)
#define CAN_TXFG_IDR0_EXT_REG(base)              ((base)->TXFG_IDR0_EXT)
#define CAN_TXFG_IDR0_STD_REG(base)              ((base)->TXFG_IDR0_STD)
#define CAN_TXFG_IDR1_EXT_REG(base)              ((base)->TXFG_IDR1_EXT)
#define CAN_TXFG_IDR1_STD_REG(base)              ((base)->TXFG_IDR1_STD)
#define CAN_TXFG_IDR2_EXT_REG(base)              ((base)->TXFG_IDR2_EXT)
#define CAN_TXFG_IDR3_EXT_REG(base)              ((base)->TXFG_IDR3_EXT)
#define CAN_TXFG_DSR_REG(base,index)             ((base)->TXFG_DSR[index])
#define CAN_TXFG_DLR_REG(base)                   ((base)->TXFG_DLR)
#define CAN_TXFG_TBPR_REG(base)                  ((base)->TXFG_TBPR)
#define CAN_TXFG_TSRH_REG(base)                  ((base)->TXFG_TSRH)
#define CAN_TXFG_TSRL_REG(base)                  ((base)->TXFG_TSRL)

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/* CTL0 Bit Fields */
#define CAN_CTL0_INITRQ_MASK                     0x1u
#define CAN_CTL0_INITRQ_SHIFT                    0
#define CAN_CTL0_SLPRQ_MASK                      0x2u
#define CAN_CTL0_SLPRQ_SHIFT                     1
#define CAN_CTL0_WUPE_MASK                       0x4u
#define CAN_CTL0_WUPE_SHIFT                      2
#define CAN_CTL0_TIME_MASK                       0x8u
#define CAN_CTL0_TIME_SHIFT                      3
#define CAN_CTL0_SYNCH_MASK                      0x10u
#define CAN_CTL0_SYNCH_SHIFT                     4
#define CAN_CTL0_CSWAI_MASK                      0x20u
#define CAN_CTL0_CSWAI_SHIFT                     5
#define CAN_CTL0_RXACT_MASK                      0x40u
#define CAN_CTL0_RXACT_SHIFT                     6
#define CAN_CTL0_RXFRM_MASK                      0x80u
#define CAN_CTL0_RXFRM_SHIFT                     7
/* CTL1 Bit Fields */
#define CAN_CTL1_INITAK_MASK                     0x1u
#define CAN_CTL1_INITAK_SHIFT                    0
#define CAN_CTL1_SLPAK_MASK                      0x2u
#define CAN_CTL1_SLPAK_SHIFT                     1
#define CAN_CTL1_WUPM_MASK                       0x4u
#define CAN_CTL1_WUPM_SHIFT                      2
#define CAN_CTL1_BORM_MASK                       0x8u
#define CAN_CTL1_BORM_SHIFT                      3
#define CAN_CTL1_LISTEN_MASK                     0x10u
#define CAN_CTL1_LISTEN_SHIFT                    4
#define CAN_CTL1_LOOPB_MASK                      0x20u
#define CAN_CTL1_LOOPB_SHIFT                     5
#define CAN_CTL1_CLKSRC_MASK                     0x40u
#define CAN_CTL1_CLKSRC_SHIFT                    6
#define CAN_CTL1_CANE_MASK                       0x80u
#define CAN_CTL1_CANE_SHIFT                      7
/* BTR0 Bit Fields */
#define CAN_BTR0_BRP_MASK                        0x3Fu
#define CAN_BTR0_BRP_SHIFT                       0
#define CAN_BTR0_BRP(x)                          (((uint16_t)(((uint16_t)(x))<<CAN_BTR0_BRP_SHIFT))&CAN_BTR0_BRP_MASK)
#define CAN_BTR0_SJW_MASK                        0xC0u
#define CAN_BTR0_SJW_SHIFT                       6
#define CAN_BTR0_SJW(x)                          (((uint16_t)(((uint16_t)(x))<<CAN_BTR0_SJW_SHIFT))&CAN_BTR0_SJW_MASK)
/* BTR1 Bit Fields */
#define CAN_BTR1_TSEG1_MASK                      0xFu
#define CAN_BTR1_TSEG1_SHIFT                     0
#define CAN_BTR1_TSEG1(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_BTR1_TSEG1_SHIFT))&CAN_BTR1_TSEG1_MASK)
#define CAN_BTR1_TSEG2_MASK                      0x70u
#define CAN_BTR1_TSEG2_SHIFT                     4
#define CAN_BTR1_TSEG2(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_BTR1_TSEG2_SHIFT))&CAN_BTR1_TSEG2_MASK)
#define CAN_BTR1_SAMP_MASK                       0x80u
#define CAN_BTR1_SAMP_SHIFT                      7
/* RFLG Bit Fields */
#define CAN_RFLG_RXF_MASK                        0x1u
#define CAN_RFLG_RXF_SHIFT                       0
#define CAN_RFLG_OVRIF_MASK                      0x2u
#define CAN_RFLG_OVRIF_SHIFT                     1
#define CAN_RFLG_TSTAT_MASK                      0xCu
#define CAN_RFLG_TSTAT_SHIFT                     2
#define CAN_RFLG_TSTAT(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_RFLG_TSTAT_SHIFT))&CAN_RFLG_TSTAT_MASK)
#define CAN_RFLG_RSTAT_MASK                      0x30u
#define CAN_RFLG_RSTAT_SHIFT                     4
#define CAN_RFLG_RSTAT(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_RFLG_RSTAT_SHIFT))&CAN_RFLG_RSTAT_MASK)
#define CAN_RFLG_CSCIF_MASK                      0x40u
#define CAN_RFLG_CSCIF_SHIFT                     6
#define CAN_RFLG_WUPIF_MASK                      0x80u
#define CAN_RFLG_WUPIF_SHIFT                     7
/* RIER Bit Fields */
#define CAN_RIER_RXFIE_MASK                      0x1u
#define CAN_RIER_RXFIE_SHIFT                     0
#define CAN_RIER_OVRIE_MASK                      0x2u
#define CAN_RIER_OVRIE_SHIFT                     1
#define CAN_RIER_TSTATE_MASK                     0xCu
#define CAN_RIER_TSTATE_SHIFT                    2
#define CAN_RIER_TSTATE(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_RIER_TSTATE_SHIFT))&CAN_RIER_TSTATE_MASK)
#define CAN_RIER_RSTATE_MASK                     0x30u
#define CAN_RIER_RSTATE_SHIFT                    4
#define CAN_RIER_RSTATE(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_RIER_RSTATE_SHIFT))&CAN_RIER_RSTATE_MASK)
#define CAN_RIER_CSCIE_MASK                      0x40u
#define CAN_RIER_CSCIE_SHIFT                     6
#define CAN_RIER_WUPIE_MASK                      0x80u
#define CAN_RIER_WUPIE_SHIFT                     7
/* TFLG Bit Fields */
#define CAN_TFLG_TXE_MASK                        0x7u
#define CAN_TFLG_TXE_SHIFT                       0
#define CAN_TFLG_TXE(x)                          (((uint16_t)(((uint16_t)(x))<<CAN_TFLG_TXE_SHIFT))&CAN_TFLG_TXE_MASK)
/* TIER Bit Fields */
#define CAN_TIER_TXEIE_MASK                      0x7u
#define CAN_TIER_TXEIE_SHIFT                     0
#define CAN_TIER_TXEIE(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_TIER_TXEIE_SHIFT))&CAN_TIER_TXEIE_MASK)
/* TARQ Bit Fields */
#define CAN_TARQ_ABTRQ_MASK                      0x7u
#define CAN_TARQ_ABTRQ_SHIFT                     0
#define CAN_TARQ_ABTRQ(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_TARQ_ABTRQ_SHIFT))&CAN_TARQ_ABTRQ_MASK)
/* TAAK Bit Fields */
#define CAN_TAAK_ABTAK_MASK                      0x7u
#define CAN_TAAK_ABTAK_SHIFT                     0
#define CAN_TAAK_ABTAK(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_TAAK_ABTAK_SHIFT))&CAN_TAAK_ABTAK_MASK)
/* TBSEL Bit Fields */
#define CAN_TBSEL_TX_MASK                        0x7u
#define CAN_TBSEL_TX_SHIFT                       0
#define CAN_TBSEL_TX(x)                          (((uint16_t)(((uint16_t)(x))<<CAN_TBSEL_TX_SHIFT))&CAN_TBSEL_TX_MASK)
/* IDAC Bit Fields */
#define CAN_IDAC_IDHIT_MASK                      0x7u
#define CAN_IDAC_IDHIT_SHIFT                     0
#define CAN_IDAC_IDHIT(x)                        (((uint16_t)(((uint16_t)(x))<<CAN_IDAC_IDHIT_SHIFT))&CAN_IDAC_IDHIT_MASK)
#define CAN_IDAC_IDAM_MASK                       0x30u
#define CAN_IDAC_IDAM_SHIFT                      4
#define CAN_IDAC_IDAM(x)                         (((uint16_t)(((uint16_t)(x))<<CAN_IDAC_IDAM_SHIFT))&CAN_IDAC_IDAM_MASK)
/* MISC Bit Fields */
#define CAN_MISC_BOHOLD_MASK                     0x1u
#define CAN_MISC_BOHOLD_SHIFT                    0
/* RXERR Bit Fields */
#define CAN_RXERR_RXERR_MASK                     0xFFu
#define CAN_RXERR_RXERR_SHIFT                    0
#define CAN_RXERR_RXERR(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_RXERR_RXERR_SHIFT))&CAN_RXERR_RXERR_MASK)
/* TXERR Bit Fields */
#define CAN_TXERR_TXERR_MASK                     0xFFu
#define CAN_TXERR_TXERR_SHIFT                    0
#define CAN_TXERR_TXERR(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_TXERR_TXERR_SHIFT))&CAN_TXERR_TXERR_MASK)
/* IDAR_BANK_1 Bit Fields */
#define CAN_IDAR_BANK_1_AC_MASK                  0xFFu
#define CAN_IDAR_BANK_1_AC_SHIFT                 0
#define CAN_IDAR_BANK_1_AC(x)                    (((uint16_t)(((uint16_t)(x))<<CAN_IDAR_BANK_1_AC_SHIFT))&CAN_IDAR_BANK_1_AC_MASK)
/* IDMR_BANK_1 Bit Fields */
#define CAN_IDMR_BANK_1_AM_MASK                  0xFFu
#define CAN_IDMR_BANK_1_AM_SHIFT                 0
#define CAN_IDMR_BANK_1_AM(x)                    (((uint16_t)(((uint16_t)(x))<<CAN_IDMR_BANK_1_AM_SHIFT))&CAN_IDMR_BANK_1_AM_MASK)
/* IDAR_BANK_2 Bit Fields */
#define CAN_IDAR_BANK_2_AC_MASK                  0xFFu
#define CAN_IDAR_BANK_2_AC_SHIFT                 0
#define CAN_IDAR_BANK_2_AC(x)                    (((uint16_t)(((uint16_t)(x))<<CAN_IDAR_BANK_2_AC_SHIFT))&CAN_IDAR_BANK_2_AC_MASK)
/* IDMR_BANK_2 Bit Fields */
#define CAN_IDMR_BANK_2_AM_MASK                  0xFFu
#define CAN_IDMR_BANK_2_AM_SHIFT                 0
#define CAN_IDMR_BANK_2_AM(x)                    (((uint16_t)(((uint16_t)(x))<<CAN_IDMR_BANK_2_AM_SHIFT))&CAN_IDMR_BANK_2_AM_MASK)
/* RXFG_IDR0_EXT Bit Fields */
#define CAN_RXFG_IDR0_EXT_ID_MASK                0xFFu
#define CAN_RXFG_IDR0_EXT_ID_SHIFT               0
#define CAN_RXFG_IDR0_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR0_EXT_ID_SHIFT))&CAN_RXFG_IDR0_EXT_ID_MASK)
/* RXFG_IDR0_STD Bit Fields */
#define CAN_RXFG_IDR0_STD_ID_MASK                0xFFu
#define CAN_RXFG_IDR0_STD_ID_SHIFT               0
#define CAN_RXFG_IDR0_STD_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR0_STD_ID_SHIFT))&CAN_RXFG_IDR0_STD_ID_MASK)
/* RXFG_IDR1_EXT Bit Fields */
#define CAN_RXFG_IDR1_EXT_ID_17_15_MASK          0x7u
#define CAN_RXFG_IDR1_EXT_ID_17_15_SHIFT         0
#define CAN_RXFG_IDR1_EXT_ID_17_15(x)            (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR1_EXT_ID_17_15_SHIFT))&CAN_RXFG_IDR1_EXT_ID_17_15_MASK)
#define CAN_RXFG_IDR1_EXT_IDE_MASK               0x8u
#define CAN_RXFG_IDR1_EXT_IDE_SHIFT              3
#define CAN_RXFG_IDR1_EXT_SRR_MASK               0x10u
#define CAN_RXFG_IDR1_EXT_SRR_SHIFT              4
#define CAN_RXFG_IDR1_EXT_ID_20_18_MASK          0xE0u
#define CAN_RXFG_IDR1_EXT_ID_20_18_SHIFT         5
#define CAN_RXFG_IDR1_EXT_ID_20_18(x)            (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR1_EXT_ID_20_18_SHIFT))&CAN_RXFG_IDR1_EXT_ID_20_18_MASK)
/* RXFG_IDR1_STD Bit Fields */
#define CAN_RXFG_IDR1_STD_Unimplemented_MASK     0x7u
#define CAN_RXFG_IDR1_STD_Unimplemented_SHIFT    0
#define CAN_RXFG_IDR1_STD_Unimplemented(x)       (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR1_STD_Unimplemented_SHIFT))&CAN_RXFG_IDR1_STD_Unimplemented_MASK)
#define CAN_RXFG_IDR1_STD_IDE_MASK               0x8u
#define CAN_RXFG_IDR1_STD_IDE_SHIFT              3
#define CAN_RXFG_IDR1_STD_RTR_MASK               0x10u
#define CAN_RXFG_IDR1_STD_RTR_SHIFT              4
#define CAN_RXFG_IDR1_STD_ID_MASK                0xE0u
#define CAN_RXFG_IDR1_STD_ID_SHIFT               5
#define CAN_RXFG_IDR1_STD_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR1_STD_ID_SHIFT))&CAN_RXFG_IDR1_STD_ID_MASK)
/* RXFG_IDR2_EXT Bit Fields */
#define CAN_RXFG_IDR2_EXT_ID_MASK                0xFFu
#define CAN_RXFG_IDR2_EXT_ID_SHIFT               0
#define CAN_RXFG_IDR2_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR2_EXT_ID_SHIFT))&CAN_RXFG_IDR2_EXT_ID_MASK)
/* RXFG_IDR3_EXT Bit Fields */
#define CAN_RXFG_IDR3_EXT_RTR_MASK               0x1u
#define CAN_RXFG_IDR3_EXT_RTR_SHIFT              0
#define CAN_RXFG_IDR3_EXT_ID_MASK                0xFEu
#define CAN_RXFG_IDR3_EXT_ID_SHIFT               1
#define CAN_RXFG_IDR3_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_IDR3_EXT_ID_SHIFT))&CAN_RXFG_IDR3_EXT_ID_MASK)
/* RXFG_DSR Bit Fields */
#define CAN_RXFG_DSR_DB_MASK                     0xFFu
#define CAN_RXFG_DSR_DB_SHIFT                    0
#define CAN_RXFG_DSR_DB(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_DSR_DB_SHIFT))&CAN_RXFG_DSR_DB_MASK)
/* RXFG_DLR Bit Fields */
#define CAN_RXFG_DLR_DLC_MASK                    0xFu
#define CAN_RXFG_DLR_DLC_SHIFT                   0
#define CAN_RXFG_DLR_DLC(x)                      (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_DLR_DLC_SHIFT))&CAN_RXFG_DLR_DLC_MASK)
#define CAN_RXFG_DLR_Unimplemented_MASK          0xF0u
#define CAN_RXFG_DLR_Unimplemented_SHIFT         4
#define CAN_RXFG_DLR_Unimplemented(x)            (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_DLR_Unimplemented_SHIFT))&CAN_RXFG_DLR_Unimplemented_MASK)
/* RXFG_TSRH Bit Fields */
#define CAN_RXFG_TSRH_TSR_MASK                   0xFFu
#define CAN_RXFG_TSRH_TSR_SHIFT                  0
#define CAN_RXFG_TSRH_TSR(x)                     (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_TSRH_TSR_SHIFT))&CAN_RXFG_TSRH_TSR_MASK)
/* RXFG_TSRL Bit Fields */
#define CAN_RXFG_TSRL_TSR_MASK                   0xFFu
#define CAN_RXFG_TSRL_TSR_SHIFT                  0
#define CAN_RXFG_TSRL_TSR(x)                     (((uint16_t)(((uint16_t)(x))<<CAN_RXFG_TSRL_TSR_SHIFT))&CAN_RXFG_TSRL_TSR_MASK)
/* TXFG_IDR0_EXT Bit Fields */
#define CAN_TXFG_IDR0_EXT_ID_MASK                0xFFu
#define CAN_TXFG_IDR0_EXT_ID_SHIFT               0
#define CAN_TXFG_IDR0_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR0_EXT_ID_SHIFT))&CAN_TXFG_IDR0_EXT_ID_MASK)
/* TXFG_IDR0_STD Bit Fields */
#define CAN_TXFG_IDR0_STD_ID_MASK                0xFFu
#define CAN_TXFG_IDR0_STD_ID_SHIFT               0
#define CAN_TXFG_IDR0_STD_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR0_STD_ID_SHIFT))&CAN_TXFG_IDR0_STD_ID_MASK)
/* TXFG_IDR1_EXT Bit Fields */
#define CAN_TXFG_IDR1_EXT_ID_17_15_MASK          0x7u
#define CAN_TXFG_IDR1_EXT_ID_17_15_SHIFT         0
#define CAN_TXFG_IDR1_EXT_ID_17_15(x)            (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR1_EXT_ID_17_15_SHIFT))&CAN_TXFG_IDR1_EXT_ID_17_15_MASK)
#define CAN_TXFG_IDR1_EXT_IDE_MASK               0x8u
#define CAN_TXFG_IDR1_EXT_IDE_SHIFT              3
#define CAN_TXFG_IDR1_EXT_SRR_MASK               0x10u
#define CAN_TXFG_IDR1_EXT_SRR_SHIFT              4
#define CAN_TXFG_IDR1_EXT_ID_20_18_MASK          0xE0u
#define CAN_TXFG_IDR1_EXT_ID_20_18_SHIFT         5
#define CAN_TXFG_IDR1_EXT_ID_20_18(x)            (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR1_EXT_ID_20_18_SHIFT))&CAN_TXFG_IDR1_EXT_ID_20_18_MASK)
/* TXFG_IDR1_STD Bit Fields */
#define CAN_TXFG_IDR1_STD_Unimplemented_MASK     0x7u
#define CAN_TXFG_IDR1_STD_Unimplemented_SHIFT    0
#define CAN_TXFG_IDR1_STD_Unimplemented(x)       (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR1_STD_Unimplemented_SHIFT))&CAN_TXFG_IDR1_STD_Unimplemented_MASK)
#define CAN_TXFG_IDR1_STD_IDE_MASK               0x8u
#define CAN_TXFG_IDR1_STD_IDE_SHIFT              3
#define CAN_TXFG_IDR1_STD_RTR_MASK               0x10u
#define CAN_TXFG_IDR1_STD_RTR_SHIFT              4
#define CAN_TXFG_IDR1_STD_ID_MASK                0xE0u
#define CAN_TXFG_IDR1_STD_ID_SHIFT               5
#define CAN_TXFG_IDR1_STD_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR1_STD_ID_SHIFT))&CAN_TXFG_IDR1_STD_ID_MASK)
/* TXFG_IDR2_EXT Bit Fields */
#define CAN_TXFG_IDR2_EXT_ID_MASK                0xFFu
#define CAN_TXFG_IDR2_EXT_ID_SHIFT               0
#define CAN_TXFG_IDR2_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR2_EXT_ID_SHIFT))&CAN_TXFG_IDR2_EXT_ID_MASK)
/* TXFG_IDR3_EXT Bit Fields */
#define CAN_TXFG_IDR3_EXT_RTR_MASK               0x1u
#define CAN_TXFG_IDR3_EXT_RTR_SHIFT              0
#define CAN_TXFG_IDR3_EXT_ID_MASK                0xFEu
#define CAN_TXFG_IDR3_EXT_ID_SHIFT               1
#define CAN_TXFG_IDR3_EXT_ID(x)                  (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_IDR3_EXT_ID_SHIFT))&CAN_TXFG_IDR3_EXT_ID_MASK)
/* TXFG_DSR Bit Fields */
#define CAN_TXFG_DSR_DB_MASK                     0xFFu
#define CAN_TXFG_DSR_DB_SHIFT                    0
#define CAN_TXFG_DSR_DB(x)                       (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_DSR_DB_SHIFT))&CAN_TXFG_DSR_DB_MASK)
/* TXFG_DLR Bit Fields */
#define CAN_TXFG_DLR_DLC_MASK                    0xFu
#define CAN_TXFG_DLR_DLC_SHIFT                   0
#define CAN_TXFG_DLR_DLC(x)                      (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_DLR_DLC_SHIFT))&CAN_TXFG_DLR_DLC_MASK)
#define CAN_TXFG_DLR_Unimplemented_MASK          0xF0u
#define CAN_TXFG_DLR_Unimplemented_SHIFT         4
#define CAN_TXFG_DLR_Unimplemented(x)            (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_DLR_Unimplemented_SHIFT))&CAN_TXFG_DLR_Unimplemented_MASK)
/* TXFG_TBPR Bit Fields */
#define CAN_TXFG_TBPR_PRIO_MASK                  0xFFu
#define CAN_TXFG_TBPR_PRIO_SHIFT                 0
#define CAN_TXFG_TBPR_PRIO(x)                    (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_TBPR_PRIO_SHIFT))&CAN_TXFG_TBPR_PRIO_MASK)
/* TXFG_TSRH Bit Fields */
#define CAN_TXFG_TSRH_TSR_MASK                   0xFFu
#define CAN_TXFG_TSRH_TSR_SHIFT                  0
#define CAN_TXFG_TSRH_TSR(x)                     (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_TSRH_TSR_SHIFT))&CAN_TXFG_TSRH_TSR_MASK)
/* TXFG_TSRL Bit Fields */
#define CAN_TXFG_TSRL_TSR_MASK                   0xFFu
#define CAN_TXFG_TSRL_TSR_SHIFT                  0
#define CAN_TXFG_TSRL_TSR(x)                     (((uint16_t)(((uint16_t)(x))<<CAN_TXFG_TSRL_TSR_SHIFT))&CAN_TXFG_TSRL_TSR_MASK)

/*!
 * @}
 */ /* end of group CAN_Register_Masks */


/* CAN - Peripheral instance base addresses */
/** Peripheral CAN base pointer */
#define CAN_BASE_PTR                             ((CAN_MemMapPtr)0xE700u)
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { CAN_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register instance definitions */
/* CAN */
#define CAN_CTL0                                 CAN_CTL0_REG(CAN_BASE_PTR)
#define CAN_CTL1                                 CAN_CTL1_REG(CAN_BASE_PTR)
#define CAN_BTR0                                 CAN_BTR0_REG(CAN_BASE_PTR)
#define CAN_BTR1                                 CAN_BTR1_REG(CAN_BASE_PTR)
#define CAN_RFLG                                 CAN_RFLG_REG(CAN_BASE_PTR)
#define CAN_RIER                                 CAN_RIER_REG(CAN_BASE_PTR)
#define CAN_TFLG                                 CAN_TFLG_REG(CAN_BASE_PTR)
#define CAN_TIER                                 CAN_TIER_REG(CAN_BASE_PTR)
#define CAN_TARQ                                 CAN_TARQ_REG(CAN_BASE_PTR)
#define CAN_TAAK                                 CAN_TAAK_REG(CAN_BASE_PTR)
#define CAN_TBSEL                                CAN_TBSEL_REG(CAN_BASE_PTR)
#define CAN_IDAC                                 CAN_IDAC_REG(CAN_BASE_PTR)
#define CAN_MISC                                 CAN_MISC_REG(CAN_BASE_PTR)
#define CAN_RXERR                                CAN_RXERR_REG(CAN_BASE_PTR)
#define CAN_TXERR                                CAN_TXERR_REG(CAN_BASE_PTR)
#define CAN_IDAR0                                CAN_IDAR_BANK_1_REG(CAN_BASE_PTR,0)
#define CAN_IDAR1                                CAN_IDAR_BANK_1_REG(CAN_BASE_PTR,1)
#define CAN_IDAR2                                CAN_IDAR_BANK_1_REG(CAN_BASE_PTR,2)
#define CAN_IDAR3                                CAN_IDAR_BANK_1_REG(CAN_BASE_PTR,3)
#define CAN_IDMR0                                CAN_IDMR_BANK_1_REG(CAN_BASE_PTR,0)
#define CAN_IDMR1                                CAN_IDMR_BANK_1_REG(CAN_BASE_PTR,1)
#define CAN_IDMR2                                CAN_IDMR_BANK_1_REG(CAN_BASE_PTR,2)
#define CAN_IDMR3                                CAN_IDMR_BANK_1_REG(CAN_BASE_PTR,3)
#define CAN_IDAR4                                CAN_IDAR_BANK_2_REG(CAN_BASE_PTR,0)
#define CAN_IDAR5                                CAN_IDAR_BANK_2_REG(CAN_BASE_PTR,1)
#define CAN_IDAR6                                CAN_IDAR_BANK_2_REG(CAN_BASE_PTR,2)
#define CAN_IDAR7                                CAN_IDAR_BANK_2_REG(CAN_BASE_PTR,3)
#define CAN_IDMR4                                CAN_IDMR_BANK_2_REG(CAN_BASE_PTR,0)
#define CAN_IDMR5                                CAN_IDMR_BANK_2_REG(CAN_BASE_PTR,1)
#define CAN_IDMR6                                CAN_IDMR_BANK_2_REG(CAN_BASE_PTR,2)
#define CAN_IDMR7                                CAN_IDMR_BANK_2_REG(CAN_BASE_PTR,3)
#define CAN_RXFG_IDR0_EXT                        CAN_RXFG_IDR0_EXT_REG(CAN_BASE_PTR)
#define CAN_RXFG_IDR0_STD                        CAN_RXFG_IDR0_STD_REG(CAN_BASE_PTR)
#define CAN_RXFG_IDR1_EXT                        CAN_RXFG_IDR1_EXT_REG(CAN_BASE_PTR)
#define CAN_RXFG_IDR1_STD                        CAN_RXFG_IDR1_STD_REG(CAN_BASE_PTR)
#define CAN_RXFG_IDR2_EXT                        CAN_RXFG_IDR2_EXT_REG(CAN_BASE_PTR)
#define CAN_RXFG_IDR3_EXT                        CAN_RXFG_IDR3_EXT_REG(CAN_BASE_PTR)
#define CAN_RXFG_DSR0                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,0)
#define CAN_RXFG_DSR1                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,1)
#define CAN_RXFG_DSR2                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,2)
#define CAN_RXFG_DSR3                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,3)
#define CAN_RXFG_DSR4                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,4)
#define CAN_RXFG_DSR5                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,5)
#define CAN_RXFG_DSR6                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,6)
#define CAN_RXFG_DSR7                            CAN_RXFG_DSR_REG(CAN_BASE_PTR,7)
#define CAN_RXFG_DLR                             CAN_RXFG_DLR_REG(CAN_BASE_PTR)
#define CAN_RXFG_TSRH                            CAN_RXFG_TSRH_REG(CAN_BASE_PTR)
#define CAN_RXFG_TSRL                            CAN_RXFG_TSRL_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR0_EXT                        CAN_TXFG_IDR0_EXT_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR0_STD                        CAN_TXFG_IDR0_STD_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR1_EXT                        CAN_TXFG_IDR1_EXT_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR1_STD                        CAN_TXFG_IDR1_STD_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR2_EXT                        CAN_TXFG_IDR2_EXT_REG(CAN_BASE_PTR)
#define CAN_TXFG_IDR3_EXT                        CAN_TXFG_IDR3_EXT_REG(CAN_BASE_PTR)
#define CAN_TXFG_DSR0                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,0)
#define CAN_TXFG_DSR1                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,1)
#define CAN_TXFG_DSR2                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,2)
#define CAN_TXFG_DSR3                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,3)
#define CAN_TXFG_DSR4                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,4)
#define CAN_TXFG_DSR5                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,5)
#define CAN_TXFG_DSR6                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,6)
#define CAN_TXFG_DSR7                            CAN_TXFG_DSR_REG(CAN_BASE_PTR,7)
#define CAN_TXFG_DLR                             CAN_TXFG_DLR_REG(CAN_BASE_PTR)
#define CAN_TXFG_TBPR                            CAN_TXFG_TBPR_REG(CAN_BASE_PTR)
#define CAN_TXFG_TSRH                            CAN_TXFG_TSRH_REG(CAN_BASE_PTR)
#define CAN_TXFG_TSRL                            CAN_TXFG_TSRL_REG(CAN_BASE_PTR)

/* CAN - Register array accessors */
#define CAN_IDAR_BANK_1(index)                   CAN_IDAR_BANK_1_REG(CAN_BASE_PTR,index)
#define CAN_IDMR_BANK_1(index)                   CAN_IDMR_BANK_1_REG(CAN_BASE_PTR,index)
#define CAN_IDAR_BANK_2(index)                   CAN_IDAR_BANK_2_REG(CAN_BASE_PTR,index)
#define CAN_IDMR_BANK_2(index)                   CAN_IDMR_BANK_2_REG(CAN_BASE_PTR,index)
#define CAN_RXFG_DSR(index)                      CAN_RXFG_DSR_REG(CAN_BASE_PTR,index)
#define CAN_TXFG_DSR(index)                      CAN_TXFG_DSR_REG(CAN_BASE_PTR,index)

/*!
 * @}
 */ /* end of group CAN_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CAN_Peripheral */


/* ----------------------------------------------------------------------------
   -- CMP
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral CMP
 * @{
 */

/** CMP - Peripheral register structure */
typedef struct CMP_MemMap {
  uint16_t CR0;                                    /**< CMP Control Register 0, offset: 0x0 */
  uint16_t CR1;                                    /**< CMP Control Register 1, offset: 0x1 */
  uint16_t FPR;                                    /**< CMP Filter Period Register, offset: 0x2 */
  uint16_t SCR;                                    /**< CMP Status and Control Register, offset: 0x3 */
  uint16_t DACCR;                                  /**< DAC Control Register, offset: 0x4 */
  uint16_t MUXCR;                                  /**< MUX Control Register, offset: 0x5 */
} volatile *CMP_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register accessors */
#define CMP_CR0_REG(base)                        ((base)->CR0)
#define CMP_CR1_REG(base)                        ((base)->CR1)
#define CMP_FPR_REG(base)                        ((base)->FPR)
#define CMP_SCR_REG(base)                        ((base)->SCR)
#define CMP_DACCR_REG(base)                      ((base)->DACCR)
#define CMP_MUXCR_REG(base)                      ((base)->MUXCR)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/* CR0 Bit Fields */
#define CMP_CR0_HYSTCTR_MASK                     0x3u
#define CMP_CR0_HYSTCTR_SHIFT                    0
#define CMP_CR0_HYSTCTR(x)                       (((uint16_t)(((uint16_t)(x))<<CMP_CR0_HYSTCTR_SHIFT))&CMP_CR0_HYSTCTR_MASK)
#define CMP_CR0_FILTER_CNT_MASK                  0x70u
#define CMP_CR0_FILTER_CNT_SHIFT                 4
#define CMP_CR0_FILTER_CNT(x)                    (((uint16_t)(((uint16_t)(x))<<CMP_CR0_FILTER_CNT_SHIFT))&CMP_CR0_FILTER_CNT_MASK)
/* CR1 Bit Fields */
#define CMP_CR1_EN_MASK                          0x1u
#define CMP_CR1_EN_SHIFT                         0
#define CMP_CR1_OPE_MASK                         0x2u
#define CMP_CR1_OPE_SHIFT                        1
#define CMP_CR1_COS_MASK                         0x4u
#define CMP_CR1_COS_SHIFT                        2
#define CMP_CR1_INV_MASK                         0x8u
#define CMP_CR1_INV_SHIFT                        3
#define CMP_CR1_PMODE_MASK                       0x10u
#define CMP_CR1_PMODE_SHIFT                      4
#define CMP_CR1_WE_MASK                          0x40u
#define CMP_CR1_WE_SHIFT                         6
#define CMP_CR1_SE_MASK                          0x80u
#define CMP_CR1_SE_SHIFT                         7
/* FPR Bit Fields */
#define CMP_FPR_FILT_PER_MASK                    0xFFu
#define CMP_FPR_FILT_PER_SHIFT                   0
#define CMP_FPR_FILT_PER(x)                      (((uint16_t)(((uint16_t)(x))<<CMP_FPR_FILT_PER_SHIFT))&CMP_FPR_FILT_PER_MASK)
/* SCR Bit Fields */
#define CMP_SCR_COUT_MASK                        0x1u
#define CMP_SCR_COUT_SHIFT                       0
#define CMP_SCR_CFF_MASK                         0x2u
#define CMP_SCR_CFF_SHIFT                        1
#define CMP_SCR_CFR_MASK                         0x4u
#define CMP_SCR_CFR_SHIFT                        2
#define CMP_SCR_IEF_MASK                         0x8u
#define CMP_SCR_IEF_SHIFT                        3
#define CMP_SCR_IER_MASK                         0x10u
#define CMP_SCR_IER_SHIFT                        4
/* DACCR Bit Fields */
#define CMP_DACCR_VOSEL_MASK                     0x3Fu
#define CMP_DACCR_VOSEL_SHIFT                    0
#define CMP_DACCR_VOSEL(x)                       (((uint16_t)(((uint16_t)(x))<<CMP_DACCR_VOSEL_SHIFT))&CMP_DACCR_VOSEL_MASK)
#define CMP_DACCR_VRSEL_MASK                     0x40u
#define CMP_DACCR_VRSEL_SHIFT                    6
#define CMP_DACCR_DACEN_MASK                     0x80u
#define CMP_DACCR_DACEN_SHIFT                    7
/* MUXCR Bit Fields */
#define CMP_MUXCR_MSEL_MASK                      0x7u
#define CMP_MUXCR_MSEL_SHIFT                     0
#define CMP_MUXCR_MSEL(x)                        (((uint16_t)(((uint16_t)(x))<<CMP_MUXCR_MSEL_SHIFT))&CMP_MUXCR_MSEL_MASK)
#define CMP_MUXCR_PSEL_MASK                      0x38u
#define CMP_MUXCR_PSEL_SHIFT                     3
#define CMP_MUXCR_PSEL(x)                        (((uint16_t)(((uint16_t)(x))<<CMP_MUXCR_PSEL_SHIFT))&CMP_MUXCR_PSEL_MASK)

/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMPA base pointer */
#define CMPA_BASE_PTR                            ((CMP_MemMapPtr)0xE020u)
/** Peripheral CMPB base pointer */
#define CMPB_BASE_PTR                            ((CMP_MemMapPtr)0xE028u)
/** Peripheral CMPC base pointer */
#define CMPC_BASE_PTR                            ((CMP_MemMapPtr)0xE030u)
/** Peripheral CMPD base pointer */
#define CMPD_BASE_PTR                            ((CMP_MemMapPtr)0xE038u)
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMPA_BASE_PTR, CMPB_BASE_PTR, CMPC_BASE_PTR, CMPD_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register instance definitions */
/* CMPA */
#define CMPA_CR0                                 CMP_CR0_REG(CMPA_BASE_PTR)
#define CMPA_CR1                                 CMP_CR1_REG(CMPA_BASE_PTR)
#define CMPA_FPR                                 CMP_FPR_REG(CMPA_BASE_PTR)
#define CMPA_SCR                                 CMP_SCR_REG(CMPA_BASE_PTR)
#define CMPA_DACCR                               CMP_DACCR_REG(CMPA_BASE_PTR)
#define CMPA_MUXCR                               CMP_MUXCR_REG(CMPA_BASE_PTR)
/* CMPB */
#define CMPB_CR0                                 CMP_CR0_REG(CMPB_BASE_PTR)
#define CMPB_CR1                                 CMP_CR1_REG(CMPB_BASE_PTR)
#define CMPB_FPR                                 CMP_FPR_REG(CMPB_BASE_PTR)
#define CMPB_SCR                                 CMP_SCR_REG(CMPB_BASE_PTR)
#define CMPB_DACCR                               CMP_DACCR_REG(CMPB_BASE_PTR)
#define CMPB_MUXCR                               CMP_MUXCR_REG(CMPB_BASE_PTR)
/* CMPC */
#define CMPC_CR0                                 CMP_CR0_REG(CMPC_BASE_PTR)
#define CMPC_CR1                                 CMP_CR1_REG(CMPC_BASE_PTR)
#define CMPC_FPR                                 CMP_FPR_REG(CMPC_BASE_PTR)
#define CMPC_SCR                                 CMP_SCR_REG(CMPC_BASE_PTR)
#define CMPC_DACCR                               CMP_DACCR_REG(CMPC_BASE_PTR)
#define CMPC_MUXCR                               CMP_MUXCR_REG(CMPC_BASE_PTR)
/* CMPD */
#define CMPD_CR0                                 CMP_CR0_REG(CMPD_BASE_PTR)
#define CMPD_CR1                                 CMP_CR1_REG(CMPD_BASE_PTR)
#define CMPD_FPR                                 CMP_FPR_REG(CMPD_BASE_PTR)
#define CMPD_SCR                                 CMP_SCR_REG(CMPD_BASE_PTR)
#define CMPD_DACCR                               CMP_DACCR_REG(CMPD_BASE_PTR)
#define CMPD_MUXCR                               CMP_MUXCR_REG(CMPD_BASE_PTR)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CMP_Peripheral */


/* ----------------------------------------------------------------------------
   -- COP
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Peripheral COP
 * @{
 */

/** COP - Peripheral register structure */
typedef struct COP_MemMap {
  uint16_t CTRL;                                   /**< COP Control Register, offset: 0x0 */
  uint16_t TOUT;                                   /**< COP Timeout Register, offset: 0x1 */
  uint16_t CNTR;                                   /**< COP Counter Register, offset: 0x2 */
  uint16_t INTVAL;                                 /**< COP Interrupt Value Register, offset: 0x3 */
  uint16_t WINDOW;                                 /**< COP Window Timeout Register, offset: 0x4 */
} volatile *COP_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- COP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Register_Accessor_Macros COP - Register accessor macros
 * @{
 */


/* COP - Register accessors */
#define COP_CTRL_REG(base)                       ((base)->CTRL)
#define COP_TOUT_REG(base)                       ((base)->TOUT)
#define COP_CNTR_REG(base)                       ((base)->CNTR)
#define COP_INTVAL_REG(base)                     ((base)->INTVAL)
#define COP_WINDOW_REG(base)                     ((base)->WINDOW)

/*!
 * @}
 */ /* end of group COP_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- COP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Register_Masks COP Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define COP_CTRL_CWP_MASK                        0x1u
#define COP_CTRL_CWP_SHIFT                       0
#define COP_CTRL_CEN_MASK                        0x2u
#define COP_CTRL_CEN_SHIFT                       1
#define COP_CTRL_CWEN_MASK                       0x4u
#define COP_CTRL_CWEN_SHIFT                      2
#define COP_CTRL_CSEN_MASK                       0x8u
#define COP_CTRL_CSEN_SHIFT                      3
#define COP_CTRL_CLOREN_MASK                     0x10u
#define COP_CTRL_CLOREN_SHIFT                    4
#define COP_CTRL_CLKSEL_MASK                     0x60u
#define COP_CTRL_CLKSEL_SHIFT                    5
#define COP_CTRL_CLKSEL(x)                       (((uint16_t)(((uint16_t)(x))<<COP_CTRL_CLKSEL_SHIFT))&COP_CTRL_CLKSEL_MASK)
#define COP_CTRL_INTEN_MASK                      0x80u
#define COP_CTRL_INTEN_SHIFT                     7
#define COP_CTRL_PSS_MASK                        0x300u
#define COP_CTRL_PSS_SHIFT                       8
#define COP_CTRL_PSS(x)                          (((uint16_t)(((uint16_t)(x))<<COP_CTRL_PSS_SHIFT))&COP_CTRL_PSS_MASK)
/* TOUT Bit Fields */
#define COP_TOUT_TIMEOUT_MASK                    0xFFFFu
#define COP_TOUT_TIMEOUT_SHIFT                   0
#define COP_TOUT_TIMEOUT(x)                      (((uint16_t)(((uint16_t)(x))<<COP_TOUT_TIMEOUT_SHIFT))&COP_TOUT_TIMEOUT_MASK)
/* CNTR Bit Fields */
#define COP_CNTR_COUNT_SERVICE_MASK              0xFFFFu
#define COP_CNTR_COUNT_SERVICE_SHIFT             0
#define COP_CNTR_COUNT_SERVICE(x)                (((uint16_t)(((uint16_t)(x))<<COP_CNTR_COUNT_SERVICE_SHIFT))&COP_CNTR_COUNT_SERVICE_MASK)
/* INTVAL Bit Fields */
#define COP_INTVAL_INTERRUPT_VALUE_MASK          0xFFFFu
#define COP_INTVAL_INTERRUPT_VALUE_SHIFT         0
#define COP_INTVAL_INTERRUPT_VALUE(x)            (((uint16_t)(((uint16_t)(x))<<COP_INTVAL_INTERRUPT_VALUE_SHIFT))&COP_INTVAL_INTERRUPT_VALUE_MASK)
/* WINDOW Bit Fields */
#define COP_WINDOW_WINDOW_VALUE_MASK             0xFFFFu
#define COP_WINDOW_WINDOW_VALUE_SHIFT            0
#define COP_WINDOW_WINDOW_VALUE(x)               (((uint16_t)(((uint16_t)(x))<<COP_WINDOW_WINDOW_VALUE_SHIFT))&COP_WINDOW_WINDOW_VALUE_MASK)

/*!
 * @}
 */ /* end of group COP_Register_Masks */


/* COP - Peripheral instance base addresses */
/** Peripheral COP base pointer */
#define COP_BASE_PTR                             ((COP_MemMapPtr)0xE320u)
/** Array initializer of COP peripheral base pointers */
#define COP_BASE_PTRS                            { COP_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- COP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Register_Accessor_Macros COP - Register accessor macros
 * @{
 */


/* COP - Register instance definitions */
/* COP */
#define COP_CTRL                                 COP_CTRL_REG(COP_BASE_PTR)
#define COP_TOUT                                 COP_TOUT_REG(COP_BASE_PTR)
#define COP_CNTR                                 COP_CNTR_REG(COP_BASE_PTR)
#define COP_INTVAL                               COP_INTVAL_REG(COP_BASE_PTR)
#define COP_WINDOW                               COP_WINDOW_REG(COP_BASE_PTR)

/*!
 * @}
 */ /* end of group COP_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group COP_Peripheral */


/* ----------------------------------------------------------------------------
   -- CRC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral CRC
 * @{
 */

/** CRC - Peripheral register structure */
typedef struct CRC_MemMap {
  uint16_t CRCH;                                   /**< CRC High Register, offset: 0x0 */
  uint16_t CRCL;                                   /**< CRC Low Register, offset: 0x1 */
  uint16_t TRANSPOSE;                              /**< CRC Transpose Register, offset: 0x2 */
} volatile *CRC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register accessors */
#define CRC_CRCH_REG(base)                       ((base)->CRCH)
#define CRC_CRCL_REG(base)                       ((base)->CRCL)
#define CRC_TRANSPOSE_REG(base)                  ((base)->TRANSPOSE)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/* CRCH Bit Fields */
#define CRC_CRCH_CRCH_MASK                       0xFFu
#define CRC_CRCH_CRCH_SHIFT                      0
#define CRC_CRCH_CRCH(x)                         (((uint16_t)(((uint16_t)(x))<<CRC_CRCH_CRCH_SHIFT))&CRC_CRCH_CRCH_MASK)
/* CRCL Bit Fields */
#define CRC_CRCL_CRCL_MASK                       0xFFu
#define CRC_CRCL_CRCL_SHIFT                      0
#define CRC_CRCL_CRCL(x)                         (((uint16_t)(((uint16_t)(x))<<CRC_CRCL_CRCL_SHIFT))&CRC_CRCL_CRCL_MASK)
/* TRANSPOSE Bit Fields */
#define CRC_TRANSPOSE_TRANSPOSE_MASK             0xFFu
#define CRC_TRANSPOSE_TRANSPOSE_SHIFT            0
#define CRC_TRANSPOSE_TRANSPOSE(x)               (((uint16_t)(((uint16_t)(x))<<CRC_TRANSPOSE_TRANSPOSE_SHIFT))&CRC_TRANSPOSE_TRANSPOSE_MASK)

/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base pointer */
#define CRC_BASE_PTR                             ((CRC_MemMapPtr)0xE1C0u)
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register instance definitions */
/* CRC */
#define CRC_CRCH                                 CRC_CRCH_REG(CRC_BASE_PTR)
#define CRC_CRCL                                 CRC_CRCL_REG(CRC_BASE_PTR)
#define CRC_TRANSPOSE                            CRC_TRANSPOSE_REG(CRC_BASE_PTR)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CRC_Peripheral */


/* ----------------------------------------------------------------------------
   -- DAC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral DAC
 * @{
 */

/** DAC - Peripheral register structure */
typedef struct DAC_MemMap {
  uint16_t CTRL0;                                  /**< Control Register 0, offset: 0x0 */
  union {                                          /* offset: 0x1 */
    uint16_t DATAREG;                                /**< Buffered Data Register, offset: 0x1 */
    uint16_t DATAREG_FMT1;                           /**< Buffered Data Register, offset: 0x1 */
  };
  union {                                          /* offset: 0x2 */
    uint16_t STEPVAL;                                /**< Step Size Register, offset: 0x2 */
    uint16_t STEPVAL_FMT1;                           /**< Step Size Register, offset: 0x2 */
  };
  union {                                          /* offset: 0x3 */
    uint16_t MINVAL;                                 /**< Minimum Value Register, offset: 0x3 */
    uint16_t MINVAL_FMT1;                            /**< Minimum Value Register, offset: 0x3 */
  };
  union {                                          /* offset: 0x4 */
    uint16_t MAXVAL;                                 /**< Maximum Value Register, offset: 0x4 */
    uint16_t MAXVAL_FMT1;                            /**< Maximum Value Register, offset: 0x4 */
  };
  uint16_t STATUS;                                 /**< Status Register, offset: 0x5 */
  uint16_t CTRL1;                                  /**< Control Register 1, offset: 0x6 */
} volatile *DAC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register accessors */
#define DAC_CTRL0_REG(base)                      ((base)->CTRL0)
#define DAC_DATAREG_REG(base)                    ((base)->DATAREG)
#define DAC_DATAREG_FMT1_REG(base)               ((base)->DATAREG_FMT1)
#define DAC_STEPVAL_REG(base)                    ((base)->STEPVAL)
#define DAC_STEPVAL_FMT1_REG(base)               ((base)->STEPVAL_FMT1)
#define DAC_MINVAL_REG(base)                     ((base)->MINVAL)
#define DAC_MINVAL_FMT1_REG(base)                ((base)->MINVAL_FMT1)
#define DAC_MAXVAL_REG(base)                     ((base)->MAXVAL)
#define DAC_MAXVAL_FMT1_REG(base)                ((base)->MAXVAL_FMT1)
#define DAC_STATUS_REG(base)                     ((base)->STATUS)
#define DAC_CTRL1_REG(base)                      ((base)->CTRL1)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/* CTRL0 Bit Fields */
#define DAC_CTRL0_PDN_MASK                       0x1u
#define DAC_CTRL0_PDN_SHIFT                      0
#define DAC_CTRL0_FORMAT_MASK                    0x2u
#define DAC_CTRL0_FORMAT_SHIFT                   1
#define DAC_CTRL0_SYNC_EN_MASK                   0x4u
#define DAC_CTRL0_SYNC_EN_SHIFT                  2
#define DAC_CTRL0_AUTO_MASK                      0x8u
#define DAC_CTRL0_AUTO_SHIFT                     3
#define DAC_CTRL0_DOWN_MASK                      0x10u
#define DAC_CTRL0_DOWN_SHIFT                     4
#define DAC_CTRL0_UP_MASK                        0x20u
#define DAC_CTRL0_UP_SHIFT                       5
#define DAC_CTRL0_HSLS_MASK                      0x40u
#define DAC_CTRL0_HSLS_SHIFT                     6
#define DAC_CTRL0_DMA_EN_MASK                    0x80u
#define DAC_CTRL0_DMA_EN_SHIFT                   7
#define DAC_CTRL0_WTMK_LVL_MASK                  0x300u
#define DAC_CTRL0_WTMK_LVL_SHIFT                 8
#define DAC_CTRL0_WTMK_LVL(x)                    (((uint16_t)(((uint16_t)(x))<<DAC_CTRL0_WTMK_LVL_SHIFT))&DAC_CTRL0_WTMK_LVL_MASK)
#define DAC_CTRL0_FILT_EN_MASK                   0x1000u
#define DAC_CTRL0_FILT_EN_SHIFT                  12
/* DATAREG Bit Fields */
#define DAC_DATAREG_DATA_MASK                    0xFFFu
#define DAC_DATAREG_DATA_SHIFT                   0
#define DAC_DATAREG_DATA(x)                      (((uint16_t)(((uint16_t)(x))<<DAC_DATAREG_DATA_SHIFT))&DAC_DATAREG_DATA_MASK)
/* DATAREG_FMT1 Bit Fields */
#define DAC_DATAREG_FMT1_DATA_MASK               0xFFF0u
#define DAC_DATAREG_FMT1_DATA_SHIFT              4
#define DAC_DATAREG_FMT1_DATA(x)                 (((uint16_t)(((uint16_t)(x))<<DAC_DATAREG_FMT1_DATA_SHIFT))&DAC_DATAREG_FMT1_DATA_MASK)
/* STEPVAL Bit Fields */
#define DAC_STEPVAL_STEP_MASK                    0xFFFu
#define DAC_STEPVAL_STEP_SHIFT                   0
#define DAC_STEPVAL_STEP(x)                      (((uint16_t)(((uint16_t)(x))<<DAC_STEPVAL_STEP_SHIFT))&DAC_STEPVAL_STEP_MASK)
/* STEPVAL_FMT1 Bit Fields */
#define DAC_STEPVAL_FMT1_STEP_MASK               0xFFF0u
#define DAC_STEPVAL_FMT1_STEP_SHIFT              4
#define DAC_STEPVAL_FMT1_STEP(x)                 (((uint16_t)(((uint16_t)(x))<<DAC_STEPVAL_FMT1_STEP_SHIFT))&DAC_STEPVAL_FMT1_STEP_MASK)
/* MINVAL Bit Fields */
#define DAC_MINVAL_MINVAL_MASK                   0xFFFu
#define DAC_MINVAL_MINVAL_SHIFT                  0
#define DAC_MINVAL_MINVAL(x)                     (((uint16_t)(((uint16_t)(x))<<DAC_MINVAL_MINVAL_SHIFT))&DAC_MINVAL_MINVAL_MASK)
/* MINVAL_FMT1 Bit Fields */
#define DAC_MINVAL_FMT1_MINVAL_MASK              0xFFF0u
#define DAC_MINVAL_FMT1_MINVAL_SHIFT             4
#define DAC_MINVAL_FMT1_MINVAL(x)                (((uint16_t)(((uint16_t)(x))<<DAC_MINVAL_FMT1_MINVAL_SHIFT))&DAC_MINVAL_FMT1_MINVAL_MASK)
/* MAXVAL Bit Fields */
#define DAC_MAXVAL_MAXVAL_MASK                   0xFFFu
#define DAC_MAXVAL_MAXVAL_SHIFT                  0
#define DAC_MAXVAL_MAXVAL(x)                     (((uint16_t)(((uint16_t)(x))<<DAC_MAXVAL_MAXVAL_SHIFT))&DAC_MAXVAL_MAXVAL_MASK)
/* MAXVAL_FMT1 Bit Fields */
#define DAC_MAXVAL_FMT1_MAXVAL_MASK              0xFFF0u
#define DAC_MAXVAL_FMT1_MAXVAL_SHIFT             4
#define DAC_MAXVAL_FMT1_MAXVAL(x)                (((uint16_t)(((uint16_t)(x))<<DAC_MAXVAL_FMT1_MAXVAL_SHIFT))&DAC_MAXVAL_FMT1_MAXVAL_MASK)
/* STATUS Bit Fields */
#define DAC_STATUS_EMPTY_MASK                    0x1u
#define DAC_STATUS_EMPTY_SHIFT                   0
#define DAC_STATUS_FULL_MASK                     0x2u
#define DAC_STATUS_FULL_SHIFT                    1
/* CTRL1 Bit Fields */
#define DAC_CTRL1_FILT_CNT_MASK                  0x3Fu
#define DAC_CTRL1_FILT_CNT_SHIFT                 0
#define DAC_CTRL1_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x))<<DAC_CTRL1_FILT_CNT_SHIFT))&DAC_CTRL1_FILT_CNT_MASK)

/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DACA base pointer */
#define DACA_BASE_PTR                            ((DAC_MemMapPtr)0xE000u)
/** Peripheral DACB base pointer */
#define DACB_BASE_PTR                            ((DAC_MemMapPtr)0xE010u)
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DACA_BASE_PTR, DACB_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register instance definitions */
/* DACA */
#define DACA_CTRL0                               DAC_CTRL0_REG(DACA_BASE_PTR)
#define DACA_DATAREG_FMT0                        DAC_DATAREG_REG(DACA_BASE_PTR)
#define DACA_DATAREG_FMT1                        DAC_DATAREG_FMT1_REG(DACA_BASE_PTR)
#define DACA_STEPVAL_FMT0                        DAC_STEPVAL_REG(DACA_BASE_PTR)
#define DACA_STEPVAL_FMT1                        DAC_STEPVAL_FMT1_REG(DACA_BASE_PTR)
#define DACA_MINVAL_FMT0                         DAC_MINVAL_REG(DACA_BASE_PTR)
#define DACA_MINVAL_FMT1                         DAC_MINVAL_FMT1_REG(DACA_BASE_PTR)
#define DACA_MAXVAL_FMT0                         DAC_MAXVAL_REG(DACA_BASE_PTR)
#define DACA_MAXVAL_FMT1                         DAC_MAXVAL_FMT1_REG(DACA_BASE_PTR)
#define DACA_STATUS                              DAC_STATUS_REG(DACA_BASE_PTR)
#define DACA_CTRL1                               DAC_CTRL1_REG(DACA_BASE_PTR)
/* DACB */
#define DACB_CTRL0                               DAC_CTRL0_REG(DACB_BASE_PTR)
#define DACB_DATAREG_FMT0                        DAC_DATAREG_REG(DACB_BASE_PTR)
#define DACB_DATAREG_FMT1                        DAC_DATAREG_FMT1_REG(DACB_BASE_PTR)
#define DACB_STEPVAL_FMT0                        DAC_STEPVAL_REG(DACB_BASE_PTR)
#define DACB_STEPVAL_FMT1                        DAC_STEPVAL_FMT1_REG(DACB_BASE_PTR)
#define DACB_MINVAL_FMT0                         DAC_MINVAL_REG(DACB_BASE_PTR)
#define DACB_MINVAL_FMT1                         DAC_MINVAL_FMT1_REG(DACB_BASE_PTR)
#define DACB_MAXVAL_FMT0                         DAC_MAXVAL_REG(DACB_BASE_PTR)
#define DACB_MAXVAL_FMT1                         DAC_MAXVAL_FMT1_REG(DACB_BASE_PTR)
#define DACB_STATUS                              DAC_STATUS_REG(DACB_BASE_PTR)
#define DACB_CTRL1                               DAC_CTRL1_REG(DACB_BASE_PTR)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DAC_Peripheral */


/* ----------------------------------------------------------------------------
   -- DMA
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral DMA
 * @{
 */

/** DMA - Peripheral register structure */
typedef struct DMA_MemMap {
  union {                                          /* offset: 0x0 */
    uint32_t REQC;                                   /**< DMA Request Control Register, offset: 0x0 */
  };
  uint16_t RESERVED_0[126];
  struct {                                         /* offset: 0x80, array step: 0x8 */
    uint32_t SAR;                                    /**< Source Address Register, array offset: 0x80, array step: 0x8 */
    uint32_t DAR;                                    /**< Destination Address Register, array offset: 0x82, array step: 0x8 */
    union {                                          /* offset: 0x84, array step: 0x8 */
      uint32_t DSR_BCR;                                /**< DMA Status Register / Byte Count Register, array offset: 0x84, array step: 0x8 */
    };
    uint32_t DCR;                                    /**< DMA Control Register, array offset: 0x86, array step: 0x8 */
  } DMA[4];
} volatile *DMA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register accessors */
#define DMA_REQC_REG(base)                       ((base)->REQC)
#define DMA_SAR_REG(base,index)                  ((base)->DMA[index].SAR)
#define DMA_DAR_REG(base,index)                  ((base)->DMA[index].DAR)
#define DMA_DSR_BCR_REG(base,index)              ((base)->DMA[index].DSR_BCR)
#define DMA_DCR_REG(base,index)                  ((base)->DMA[index].DCR)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/* REQC Bit Fields */
#define DMA_REQC_DMAC3_MASK                      0xFu
#define DMA_REQC_DMAC3_SHIFT                     0
#define DMA_REQC_DMAC3(x)                        (((uint32_t)(((uint32_t)(x))<<DMA_REQC_DMAC3_SHIFT))&DMA_REQC_DMAC3_MASK)
#define DMA_REQC_CFSM3_MASK                      0x80u
#define DMA_REQC_CFSM3_SHIFT                     7
#define DMA_REQC_DMAC2_MASK                      0xF00u
#define DMA_REQC_DMAC2_SHIFT                     8
#define DMA_REQC_DMAC2(x)                        (((uint32_t)(((uint32_t)(x))<<DMA_REQC_DMAC2_SHIFT))&DMA_REQC_DMAC2_MASK)
#define DMA_REQC_CFSM2_MASK                      0x8000u
#define DMA_REQC_CFSM2_SHIFT                     15
#define DMA_REQC_DMAC1_MASK                      0xF0000u
#define DMA_REQC_DMAC1_SHIFT                     16
#define DMA_REQC_DMAC1(x)                        (((uint32_t)(((uint32_t)(x))<<DMA_REQC_DMAC1_SHIFT))&DMA_REQC_DMAC1_MASK)
#define DMA_REQC_CFSM1_MASK                      0x800000u
#define DMA_REQC_CFSM1_SHIFT                     23
#define DMA_REQC_DMAC0_MASK                      0xF000000u
#define DMA_REQC_DMAC0_SHIFT                     24
#define DMA_REQC_DMAC0(x)                        (((uint32_t)(((uint32_t)(x))<<DMA_REQC_DMAC0_SHIFT))&DMA_REQC_DMAC0_MASK)
#define DMA_REQC_CFSM0_MASK                      0x80000000u
#define DMA_REQC_CFSM0_SHIFT                     31
/* SAR Bit Fields */
#define DMA_SAR_SAR_MASK                         0xFFFFFFFFu
#define DMA_SAR_SAR_SHIFT                        0
#define DMA_SAR_SAR(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_SAR_SAR_SHIFT))&DMA_SAR_SAR_MASK)
/* DAR Bit Fields */
#define DMA_DAR_DAR_MASK                         0xFFFFFFFFu
#define DMA_DAR_DAR_SHIFT                        0
#define DMA_DAR_DAR(x)                           (((uint32_t)(((uint32_t)(x))<<DMA_DAR_DAR_SHIFT))&DMA_DAR_DAR_MASK)
/* DSR_BCR Bit Fields */
#define DMA_DSR_BCR_BCR_MASK                     0xFFFFFFu
#define DMA_DSR_BCR_BCR_SHIFT                    0
#define DMA_DSR_BCR_BCR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_DSR_BCR_BCR_SHIFT))&DMA_DSR_BCR_BCR_MASK)
#define DMA_DSR_BCR_DONE_MASK                    0x1000000u
#define DMA_DSR_BCR_DONE_SHIFT                   24
#define DMA_DSR_BCR_BSY_MASK                     0x2000000u
#define DMA_DSR_BCR_BSY_SHIFT                    25
#define DMA_DSR_BCR_REQ_MASK                     0x4000000u
#define DMA_DSR_BCR_REQ_SHIFT                    26
#define DMA_DSR_BCR_BED_MASK                     0x10000000u
#define DMA_DSR_BCR_BED_SHIFT                    28
#define DMA_DSR_BCR_BES_MASK                     0x20000000u
#define DMA_DSR_BCR_BES_SHIFT                    29
#define DMA_DSR_BCR_CE_MASK                      0x40000000u
#define DMA_DSR_BCR_CE_SHIFT                     30
/* DCR Bit Fields */
#define DMA_DCR_LCH2_MASK                        0x3u
#define DMA_DCR_LCH2_SHIFT                       0
#define DMA_DCR_LCH2(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_DCR_LCH2_SHIFT))&DMA_DCR_LCH2_MASK)
#define DMA_DCR_LCH1_MASK                        0xCu
#define DMA_DCR_LCH1_SHIFT                       2
#define DMA_DCR_LCH1(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_DCR_LCH1_SHIFT))&DMA_DCR_LCH1_MASK)
#define DMA_DCR_LINKCC_MASK                      0x30u
#define DMA_DCR_LINKCC_SHIFT                     4
#define DMA_DCR_LINKCC(x)                        (((uint32_t)(((uint32_t)(x))<<DMA_DCR_LINKCC_SHIFT))&DMA_DCR_LINKCC_MASK)
#define DMA_DCR_D_REQ_MASK                       0x80u
#define DMA_DCR_D_REQ_SHIFT                      7
#define DMA_DCR_DMOD_MASK                        0xF00u
#define DMA_DCR_DMOD_SHIFT                       8
#define DMA_DCR_DMOD(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_DCR_DMOD_SHIFT))&DMA_DCR_DMOD_MASK)
#define DMA_DCR_SMOD_MASK                        0xF000u
#define DMA_DCR_SMOD_SHIFT                       12
#define DMA_DCR_SMOD(x)                          (((uint32_t)(((uint32_t)(x))<<DMA_DCR_SMOD_SHIFT))&DMA_DCR_SMOD_MASK)
#define DMA_DCR_START_MASK                       0x10000u
#define DMA_DCR_START_SHIFT                      16
#define DMA_DCR_DSIZE_MASK                       0x60000u
#define DMA_DCR_DSIZE_SHIFT                      17
#define DMA_DCR_DSIZE(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_DCR_DSIZE_SHIFT))&DMA_DCR_DSIZE_MASK)
#define DMA_DCR_DINC_MASK                        0x80000u
#define DMA_DCR_DINC_SHIFT                       19
#define DMA_DCR_SSIZE_MASK                       0x300000u
#define DMA_DCR_SSIZE_SHIFT                      20
#define DMA_DCR_SSIZE(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_DCR_SSIZE_SHIFT))&DMA_DCR_SSIZE_MASK)
#define DMA_DCR_SINC_MASK                        0x400000u
#define DMA_DCR_SINC_SHIFT                       22
#define DMA_DCR_AA_MASK                          0x10000000u
#define DMA_DCR_AA_SHIFT                         28
#define DMA_DCR_CS_MASK                          0x20000000u
#define DMA_DCR_CS_SHIFT                         29
#define DMA_DCR_ERQ_MASK                         0x40000000u
#define DMA_DCR_ERQ_SHIFT                        30
#define DMA_DCR_EINT_MASK                        0x80000000u
#define DMA_DCR_EINT_SHIFT                       31

/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base pointer */
#define DMA_BASE_PTR                             ((DMA_MemMapPtr)0xC800u)
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register instance definitions */
/* DMA */
#define DMA_REQC                                 DMA_REQC_REG(DMA_BASE_PTR)
#define DMA_SAR0                                 DMA_SAR_REG(DMA_BASE_PTR,0)
#define DMA_DAR0                                 DMA_DAR_REG(DMA_BASE_PTR,0)
#define DMA_DSR_BCR0                             DMA_DSR_BCR_REG(DMA_BASE_PTR,0)
#define DMA_DCR0                                 DMA_DCR_REG(DMA_BASE_PTR,0)
#define DMA_SAR1                                 DMA_SAR_REG(DMA_BASE_PTR,1)
#define DMA_DAR1                                 DMA_DAR_REG(DMA_BASE_PTR,1)
#define DMA_DSR_BCR1                             DMA_DSR_BCR_REG(DMA_BASE_PTR,1)
#define DMA_DCR1                                 DMA_DCR_REG(DMA_BASE_PTR,1)
#define DMA_SAR2                                 DMA_SAR_REG(DMA_BASE_PTR,2)
#define DMA_DAR2                                 DMA_DAR_REG(DMA_BASE_PTR,2)
#define DMA_DSR_BCR2                             DMA_DSR_BCR_REG(DMA_BASE_PTR,2)
#define DMA_DCR2                                 DMA_DCR_REG(DMA_BASE_PTR,2)
#define DMA_SAR3                                 DMA_SAR_REG(DMA_BASE_PTR,3)
#define DMA_DAR3                                 DMA_DAR_REG(DMA_BASE_PTR,3)
#define DMA_DSR_BCR3                             DMA_DSR_BCR_REG(DMA_BASE_PTR,3)
#define DMA_DCR3                                 DMA_DCR_REG(DMA_BASE_PTR,3)

/* DMA - Register array accessors */
#define DMA_SAR(index)                           DMA_SAR_REG(DMA_BASE_PTR,index)
#define DMA_DAR(index)                           DMA_DAR_REG(DMA_BASE_PTR,index)
#define DMA_DSR_BCR(index)                       DMA_DSR_BCR_REG(DMA_BASE_PTR,index)
#define DMA_DCR(index)                           DMA_DCR_REG(DMA_BASE_PTR,index)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMA_Peripheral */


/* ----------------------------------------------------------------------------
   -- EWM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral EWM
 * @{
 */

/** EWM - Peripheral register structure */
typedef struct EWM_MemMap {
  uint16_t CTRL;                                   /**< Control Register, offset: 0x0 */
  uint16_t SERV;                                   /**< Service Register, offset: 0x1 */
  uint16_t CMPL;                                   /**< Compare Low Register, offset: 0x2 */
  uint16_t CMPH;                                   /**< Compare High Register, offset: 0x3 */
  uint16_t CLKCTRL;                                /**< Clock Control Register, offset: 0x4 */
  uint16_t CLKPRESCALER;                           /**< Clock Prescaler Register, offset: 0x5 */
} volatile *EWM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register accessors */
#define EWM_CTRL_REG(base)                       ((base)->CTRL)
#define EWM_SERV_REG(base)                       ((base)->SERV)
#define EWM_CMPL_REG(base)                       ((base)->CMPL)
#define EWM_CMPH_REG(base)                       ((base)->CMPH)
#define EWM_CLKCTRL_REG(base)                    ((base)->CLKCTRL)
#define EWM_CLKPRESCALER_REG(base)               ((base)->CLKPRESCALER)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define EWM_CTRL_EWMEN_MASK                      0x1u
#define EWM_CTRL_EWMEN_SHIFT                     0
#define EWM_CTRL_ASSIN_MASK                      0x2u
#define EWM_CTRL_ASSIN_SHIFT                     1
#define EWM_CTRL_INEN_MASK                       0x4u
#define EWM_CTRL_INEN_SHIFT                      2
#define EWM_CTRL_INTEN_MASK                      0x8u
#define EWM_CTRL_INTEN_SHIFT                     3
/* SERV Bit Fields */
#define EWM_SERV_SERVICE_MASK                    0xFFu
#define EWM_SERV_SERVICE_SHIFT                   0
#define EWM_SERV_SERVICE(x)                      (((uint16_t)(((uint16_t)(x))<<EWM_SERV_SERVICE_SHIFT))&EWM_SERV_SERVICE_MASK)
/* CMPL Bit Fields */
#define EWM_CMPL_COMPAREL_MASK                   0xFFu
#define EWM_CMPL_COMPAREL_SHIFT                  0
#define EWM_CMPL_COMPAREL(x)                     (((uint16_t)(((uint16_t)(x))<<EWM_CMPL_COMPAREL_SHIFT))&EWM_CMPL_COMPAREL_MASK)
/* CMPH Bit Fields */
#define EWM_CMPH_COMPAREH_MASK                   0xFFu
#define EWM_CMPH_COMPAREH_SHIFT                  0
#define EWM_CMPH_COMPAREH(x)                     (((uint16_t)(((uint16_t)(x))<<EWM_CMPH_COMPAREH_SHIFT))&EWM_CMPH_COMPAREH_MASK)
/* CLKCTRL Bit Fields */
#define EWM_CLKCTRL_CLKSEL_MASK                  0x3u
#define EWM_CLKCTRL_CLKSEL_SHIFT                 0
#define EWM_CLKCTRL_CLKSEL(x)                    (((uint16_t)(((uint16_t)(x))<<EWM_CLKCTRL_CLKSEL_SHIFT))&EWM_CLKCTRL_CLKSEL_MASK)
/* CLKPRESCALER Bit Fields */
#define EWM_CLKPRESCALER_CLK_DIV_MASK            0xFFu
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT           0
#define EWM_CLKPRESCALER_CLK_DIV(x)              (((uint16_t)(((uint16_t)(x))<<EWM_CLKPRESCALER_CLK_DIV_SHIFT))&EWM_CLKPRESCALER_CLK_DIV_MASK)

/*!
 * @}
 */ /* end of group EWM_Register_Masks */


/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base pointer */
#define EWM_BASE_PTR                             ((EWM_MemMapPtr)0xE330u)
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register instance definitions */
/* EWM */
#define EWM_CTRL                                 EWM_CTRL_REG(EWM_BASE_PTR)
#define EWM_SERV                                 EWM_SERV_REG(EWM_BASE_PTR)
#define EWM_CMPL                                 EWM_CMPL_REG(EWM_BASE_PTR)
#define EWM_CMPH                                 EWM_CMPH_REG(EWM_BASE_PTR)
#define EWM_CLKCTRL                              EWM_CLKCTRL_REG(EWM_BASE_PTR)
#define EWM_CLKPRESCALER                         EWM_CLKPRESCALER_REG(EWM_BASE_PTR)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group EWM_Peripheral */


/* ----------------------------------------------------------------------------
   -- FMC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Peripheral FMC
 * @{
 */

/** FMC - Peripheral register structure */
typedef struct FMC_MemMap {
  uint32_t PFAPR;                                  /**< Flash Access Protection Register, offset: 0x0 */
  uint32_t PFB0CR;                                 /**< Flash Control Register, offset: 0x2 */
  uint16_t RESERVED_0[124];
  uint32_t TAGVD[4][4];                            /**< Cache Tag Storage, array offset: 0x80, array step: index*0x8, index2*0x2 */
  uint16_t RESERVED_1[96];
  uint32_t DATA[4][4];                             /**< Cache Data Storage, array offset: 0x100, array step: index*0x8, index2*0x2 */
} volatile *FMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register accessors */
#define FMC_PFAPR_REG(base)                      ((base)->PFAPR)
#define FMC_PFB0CR_REG(base)                     ((base)->PFB0CR)
#define FMC_TAGVD_REG(base,index,index2)         ((base)->TAGVD[index][index2])
#define FMC_DATA_REG(base,index,index2)          ((base)->DATA[index][index2])

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Masks FMC Register Masks
 * @{
 */

/* PFAPR Bit Fields */
#define FMC_PFAPR_M0AP_MASK                      0x3u
#define FMC_PFAPR_M0AP_SHIFT                     0
#define FMC_PFAPR_M0AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M0AP_SHIFT))&FMC_PFAPR_M0AP_MASK)
#define FMC_PFAPR_M1AP_MASK                      0xCu
#define FMC_PFAPR_M1AP_SHIFT                     2
#define FMC_PFAPR_M1AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M1AP_SHIFT))&FMC_PFAPR_M1AP_MASK)
#define FMC_PFAPR_M2AP_MASK                      0x30u
#define FMC_PFAPR_M2AP_SHIFT                     4
#define FMC_PFAPR_M2AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M2AP_SHIFT))&FMC_PFAPR_M2AP_MASK)
#define FMC_PFAPR_M3AP_MASK                      0xC0u
#define FMC_PFAPR_M3AP_SHIFT                     6
#define FMC_PFAPR_M3AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M3AP_SHIFT))&FMC_PFAPR_M3AP_MASK)
#define FMC_PFAPR_M0PFD_MASK                     0x10000u
#define FMC_PFAPR_M0PFD_SHIFT                    16
#define FMC_PFAPR_M1PFD_MASK                     0x20000u
#define FMC_PFAPR_M1PFD_SHIFT                    17
#define FMC_PFAPR_M2PFD_MASK                     0x40000u
#define FMC_PFAPR_M2PFD_SHIFT                    18
#define FMC_PFAPR_M3PFD_MASK                     0x80000u
#define FMC_PFAPR_M3PFD_SHIFT                    19
/* PFB0CR Bit Fields */
#define FMC_PFB0CR_B0SEBE_MASK                   0x1u
#define FMC_PFB0CR_B0SEBE_SHIFT                  0
#define FMC_PFB0CR_B0IPE_MASK                    0x2u
#define FMC_PFB0CR_B0IPE_SHIFT                   1
#define FMC_PFB0CR_B0DPE_MASK                    0x4u
#define FMC_PFB0CR_B0DPE_SHIFT                   2
#define FMC_PFB0CR_B0ICE_MASK                    0x8u
#define FMC_PFB0CR_B0ICE_SHIFT                   3
#define FMC_PFB0CR_B0DCE_MASK                    0x10u
#define FMC_PFB0CR_B0DCE_SHIFT                   4
#define FMC_PFB0CR_CRC_MASK                      0xE0u
#define FMC_PFB0CR_CRC_SHIFT                     5
#define FMC_PFB0CR_CRC(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CRC_SHIFT))&FMC_PFB0CR_CRC_MASK)
#define FMC_PFB0CR_B0MW_MASK                     0x60000u
#define FMC_PFB0CR_B0MW_SHIFT                    17
#define FMC_PFB0CR_B0MW(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0MW_SHIFT))&FMC_PFB0CR_B0MW_MASK)
#define FMC_PFB0CR_S_B_INV_MASK                  0x80000u
#define FMC_PFB0CR_S_B_INV_SHIFT                 19
#define FMC_PFB0CR_CINV_WAY_MASK                 0xF00000u
#define FMC_PFB0CR_CINV_WAY_SHIFT                20
#define FMC_PFB0CR_CINV_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CINV_WAY_SHIFT))&FMC_PFB0CR_CINV_WAY_MASK)
#define FMC_PFB0CR_CLCK_WAY_MASK                 0xF000000u
#define FMC_PFB0CR_CLCK_WAY_SHIFT                24
#define FMC_PFB0CR_CLCK_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CLCK_WAY_SHIFT))&FMC_PFB0CR_CLCK_WAY_MASK)
#define FMC_PFB0CR_B0RWSC_MASK                   0xF0000000u
#define FMC_PFB0CR_B0RWSC_SHIFT                  28
#define FMC_PFB0CR_B0RWSC(x)                     (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0RWSC_SHIFT))&FMC_PFB0CR_B0RWSC_MASK)
/* TAGVD Bit Fields */
#define FMC_TAGVD_valid_MASK                     0x1u
#define FMC_TAGVD_valid_SHIFT                    0
#define FMC_TAGVD_tag_MASK                       0xFFFF0u
#define FMC_TAGVD_tag_SHIFT                      4
#define FMC_TAGVD_tag(x)                         (((uint32_t)(((uint32_t)(x))<<FMC_TAGVD_tag_SHIFT))&FMC_TAGVD_tag_MASK)
/* DATA Bit Fields */
#define FMC_DATA_data_MASK                       0xFFFFFFFFu
#define FMC_DATA_data_SHIFT                      0
#define FMC_DATA_data(x)                         (((uint32_t)(((uint32_t)(x))<<FMC_DATA_data_SHIFT))&FMC_DATA_data_MASK)

/*!
 * @}
 */ /* end of group FMC_Register_Masks */


/* FMC - Peripheral instance base addresses */
/** Peripheral FMC base pointer */
#define FMC_BASE_PTR                             ((FMC_MemMapPtr)0xDE00u)
/** Array initializer of FMC peripheral base pointers */
#define FMC_BASE_PTRS                            { FMC_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register instance definitions */
/* FMC */
#define FMC_PFAPR                                FMC_PFAPR_REG(FMC_BASE_PTR)
#define FMC_PFB0CR                               FMC_PFB0CR_REG(FMC_BASE_PTR)
#define FMC_TAGVDW0S0                            FMC_TAGVD_REG(FMC_BASE_PTR,0,0)
#define FMC_TAGVDW0S1                            FMC_TAGVD_REG(FMC_BASE_PTR,0,1)
#define FMC_TAGVDW0S2                            FMC_TAGVD_REG(FMC_BASE_PTR,0,2)
#define FMC_TAGVDW0S3                            FMC_TAGVD_REG(FMC_BASE_PTR,0,3)
#define FMC_TAGVDW1S0                            FMC_TAGVD_REG(FMC_BASE_PTR,1,0)
#define FMC_TAGVDW1S1                            FMC_TAGVD_REG(FMC_BASE_PTR,1,1)
#define FMC_TAGVDW1S2                            FMC_TAGVD_REG(FMC_BASE_PTR,1,2)
#define FMC_TAGVDW1S3                            FMC_TAGVD_REG(FMC_BASE_PTR,1,3)
#define FMC_TAGVDW2S0                            FMC_TAGVD_REG(FMC_BASE_PTR,2,0)
#define FMC_TAGVDW2S1                            FMC_TAGVD_REG(FMC_BASE_PTR,2,1)
#define FMC_TAGVDW2S2                            FMC_TAGVD_REG(FMC_BASE_PTR,2,2)
#define FMC_TAGVDW2S3                            FMC_TAGVD_REG(FMC_BASE_PTR,2,3)
#define FMC_TAGVDW3S0                            FMC_TAGVD_REG(FMC_BASE_PTR,3,0)
#define FMC_TAGVDW3S1                            FMC_TAGVD_REG(FMC_BASE_PTR,3,1)
#define FMC_TAGVDW3S2                            FMC_TAGVD_REG(FMC_BASE_PTR,3,2)
#define FMC_TAGVDW3S3                            FMC_TAGVD_REG(FMC_BASE_PTR,3,3)
#define FMC_DATAW0S0                             FMC_DATA_REG(FMC_BASE_PTR,0,0)
#define FMC_DATAW0S1                             FMC_DATA_REG(FMC_BASE_PTR,0,1)
#define FMC_DATAW0S2                             FMC_DATA_REG(FMC_BASE_PTR,0,2)
#define FMC_DATAW0S3                             FMC_DATA_REG(FMC_BASE_PTR,0,3)
#define FMC_DATAW1S0                             FMC_DATA_REG(FMC_BASE_PTR,1,0)
#define FMC_DATAW1S1                             FMC_DATA_REG(FMC_BASE_PTR,1,1)
#define FMC_DATAW1S2                             FMC_DATA_REG(FMC_BASE_PTR,1,2)
#define FMC_DATAW1S3                             FMC_DATA_REG(FMC_BASE_PTR,1,3)
#define FMC_DATAW2S0                             FMC_DATA_REG(FMC_BASE_PTR,2,0)
#define FMC_DATAW2S1                             FMC_DATA_REG(FMC_BASE_PTR,2,1)
#define FMC_DATAW2S2                             FMC_DATA_REG(FMC_BASE_PTR,2,2)
#define FMC_DATAW2S3                             FMC_DATA_REG(FMC_BASE_PTR,2,3)
#define FMC_DATAW3S0                             FMC_DATA_REG(FMC_BASE_PTR,3,0)
#define FMC_DATAW3S1                             FMC_DATA_REG(FMC_BASE_PTR,3,1)
#define FMC_DATAW3S2                             FMC_DATA_REG(FMC_BASE_PTR,3,2)
#define FMC_DATAW3S3                             FMC_DATA_REG(FMC_BASE_PTR,3,3)

/* FMC - Register array accessors */
#define FMC_TAGVD(index,index2)                  FMC_TAGVD_REG(FMC_BASE_PTR,index,index2)
#define FMC_DATA(index,index2)                   FMC_DATA_REG(FMC_BASE_PTR,index,index2)

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FMC_Peripheral */


/* ----------------------------------------------------------------------------
   -- FTFA
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Peripheral FTFA
 * @{
 */

/** FTFA - Peripheral register structure */
typedef struct FTFA_MemMap {
  uint8_t FSTAT;                                   /**< Flash Status Register, offset: 0x0 */
  uint8_t FCNFG;                                   /**< Flash Configuration Register, offset: 0x1 */
  uint8_t FSEC;                                    /**< Flash Security Register, offset: 0x2 */
  uint8_t FOPT;                                    /**< Flash Option Register, offset: 0x3 */
  uint8_t FCCOB3;                                  /**< Flash Common Command Object Registers, offset: 0x4 */
  uint8_t FCCOB2;                                  /**< Flash Common Command Object Registers, offset: 0x5 */
  uint8_t FCCOB1;                                  /**< Flash Common Command Object Registers, offset: 0x6 */
  uint8_t FCCOB0;                                  /**< Flash Common Command Object Registers, offset: 0x7 */
  uint8_t FCCOB7;                                  /**< Flash Common Command Object Registers, offset: 0x8 */
  uint8_t FCCOB6;                                  /**< Flash Common Command Object Registers, offset: 0x9 */
  uint8_t FCCOB5;                                  /**< Flash Common Command Object Registers, offset: 0xA */
  uint8_t FCCOB4;                                  /**< Flash Common Command Object Registers, offset: 0xB */
  uint8_t FCCOBB;                                  /**< Flash Common Command Object Registers, offset: 0xC */
  uint8_t FCCOBA;                                  /**< Flash Common Command Object Registers, offset: 0xD */
  uint8_t FCCOB9;                                  /**< Flash Common Command Object Registers, offset: 0xE */
  uint8_t FCCOB8;                                  /**< Flash Common Command Object Registers, offset: 0xF */
  uint8_t FPROT3;                                  /**< Program Flash Protection Registers, offset: 0x10 */
  uint8_t FPROT2;                                  /**< Program Flash Protection Registers, offset: 0x11 */
  uint8_t FPROT1;                                  /**< Program Flash Protection Registers, offset: 0x12 */
  uint8_t FPROT0;                                  /**< Program Flash Protection Registers, offset: 0x13 */
} volatile *FTFA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register accessors */
#define FTFA_FSTAT_REG(base)                     ((base)->FSTAT)
#define FTFA_FCNFG_REG(base)                     ((base)->FCNFG)
#define FTFA_FSEC_REG(base)                      ((base)->FSEC)
#define FTFA_FOPT_REG(base)                      ((base)->FOPT)
#define FTFA_FCCOB3_REG(base)                    ((base)->FCCOB3)
#define FTFA_FCCOB2_REG(base)                    ((base)->FCCOB2)
#define FTFA_FCCOB1_REG(base)                    ((base)->FCCOB1)
#define FTFA_FCCOB0_REG(base)                    ((base)->FCCOB0)
#define FTFA_FCCOB7_REG(base)                    ((base)->FCCOB7)
#define FTFA_FCCOB6_REG(base)                    ((base)->FCCOB6)
#define FTFA_FCCOB5_REG(base)                    ((base)->FCCOB5)
#define FTFA_FCCOB4_REG(base)                    ((base)->FCCOB4)
#define FTFA_FCCOBB_REG(base)                    ((base)->FCCOBB)
#define FTFA_FCCOBA_REG(base)                    ((base)->FCCOBA)
#define FTFA_FCCOB9_REG(base)                    ((base)->FCCOB9)
#define FTFA_FCCOB8_REG(base)                    ((base)->FCCOB8)
#define FTFA_FPROT3_REG(base)                    ((base)->FPROT3)
#define FTFA_FPROT2_REG(base)                    ((base)->FPROT2)
#define FTFA_FPROT1_REG(base)                    ((base)->FPROT1)
#define FTFA_FPROT0_REG(base)                    ((base)->FPROT0)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTFA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Masks FTFA Register Masks
 * @{
 */

/* FSTAT Bit Fields */
#define FTFA_FSTAT_MGSTAT0_MASK                  0x1u
#define FTFA_FSTAT_MGSTAT0_SHIFT                 0
#define FTFA_FSTAT_FPVIOL_MASK                   0x10u
#define FTFA_FSTAT_FPVIOL_SHIFT                  4
#define FTFA_FSTAT_ACCERR_MASK                   0x20u
#define FTFA_FSTAT_ACCERR_SHIFT                  5
#define FTFA_FSTAT_RDCOLERR_MASK                 0x40u
#define FTFA_FSTAT_RDCOLERR_SHIFT                6
#define FTFA_FSTAT_CCIF_MASK                     0x80u
#define FTFA_FSTAT_CCIF_SHIFT                    7
/* FCNFG Bit Fields */
#define FTFA_FCNFG_ERSSUSP_MASK                  0x10u
#define FTFA_FCNFG_ERSSUSP_SHIFT                 4
#define FTFA_FCNFG_ERSAREQ_MASK                  0x20u
#define FTFA_FCNFG_ERSAREQ_SHIFT                 5
#define FTFA_FCNFG_RDCOLLIE_MASK                 0x40u
#define FTFA_FCNFG_RDCOLLIE_SHIFT                6
#define FTFA_FCNFG_CCIE_MASK                     0x80u
#define FTFA_FCNFG_CCIE_SHIFT                    7
/* FSEC Bit Fields */
#define FTFA_FSEC_SEC_MASK                       0x3u
#define FTFA_FSEC_SEC_SHIFT                      0
#define FTFA_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_SEC_SHIFT))&FTFA_FSEC_SEC_MASK)
#define FTFA_FSEC_FSLACC_MASK                    0xCu
#define FTFA_FSEC_FSLACC_SHIFT                   2
#define FTFA_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_FSLACC_SHIFT))&FTFA_FSEC_FSLACC_MASK)
#define FTFA_FSEC_MEEN_MASK                      0x30u
#define FTFA_FSEC_MEEN_SHIFT                     4
#define FTFA_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_MEEN_SHIFT))&FTFA_FSEC_MEEN_MASK)
#define FTFA_FSEC_KEYEN_MASK                     0xC0u
#define FTFA_FSEC_KEYEN_SHIFT                    6
#define FTFA_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_KEYEN_SHIFT))&FTFA_FSEC_KEYEN_MASK)
/* FOPT Bit Fields */
#define FTFA_FOPT_OPT_MASK                       0xFFu
#define FTFA_FOPT_OPT_SHIFT                      0
#define FTFA_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FOPT_OPT_SHIFT))&FTFA_FOPT_OPT_MASK)
/* FCCOB3 Bit Fields */
#define FTFA_FCCOB3_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB3_CCOBn_SHIFT                  0
#define FTFA_FCCOB3_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB3_CCOBn_SHIFT))&FTFA_FCCOB3_CCOBn_MASK)
/* FCCOB2 Bit Fields */
#define FTFA_FCCOB2_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB2_CCOBn_SHIFT                  0
#define FTFA_FCCOB2_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB2_CCOBn_SHIFT))&FTFA_FCCOB2_CCOBn_MASK)
/* FCCOB1 Bit Fields */
#define FTFA_FCCOB1_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB1_CCOBn_SHIFT                  0
#define FTFA_FCCOB1_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB1_CCOBn_SHIFT))&FTFA_FCCOB1_CCOBn_MASK)
/* FCCOB0 Bit Fields */
#define FTFA_FCCOB0_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB0_CCOBn_SHIFT                  0
#define FTFA_FCCOB0_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB0_CCOBn_SHIFT))&FTFA_FCCOB0_CCOBn_MASK)
/* FCCOB7 Bit Fields */
#define FTFA_FCCOB7_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB7_CCOBn_SHIFT                  0
#define FTFA_FCCOB7_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB7_CCOBn_SHIFT))&FTFA_FCCOB7_CCOBn_MASK)
/* FCCOB6 Bit Fields */
#define FTFA_FCCOB6_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB6_CCOBn_SHIFT                  0
#define FTFA_FCCOB6_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB6_CCOBn_SHIFT))&FTFA_FCCOB6_CCOBn_MASK)
/* FCCOB5 Bit Fields */
#define FTFA_FCCOB5_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB5_CCOBn_SHIFT                  0
#define FTFA_FCCOB5_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB5_CCOBn_SHIFT))&FTFA_FCCOB5_CCOBn_MASK)
/* FCCOB4 Bit Fields */
#define FTFA_FCCOB4_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB4_CCOBn_SHIFT                  0
#define FTFA_FCCOB4_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB4_CCOBn_SHIFT))&FTFA_FCCOB4_CCOBn_MASK)
/* FCCOBB Bit Fields */
#define FTFA_FCCOBB_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBB_CCOBn_SHIFT                  0
#define FTFA_FCCOBB_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBB_CCOBn_SHIFT))&FTFA_FCCOBB_CCOBn_MASK)
/* FCCOBA Bit Fields */
#define FTFA_FCCOBA_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBA_CCOBn_SHIFT                  0
#define FTFA_FCCOBA_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBA_CCOBn_SHIFT))&FTFA_FCCOBA_CCOBn_MASK)
/* FCCOB9 Bit Fields */
#define FTFA_FCCOB9_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB9_CCOBn_SHIFT                  0
#define FTFA_FCCOB9_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB9_CCOBn_SHIFT))&FTFA_FCCOB9_CCOBn_MASK)
/* FCCOB8 Bit Fields */
#define FTFA_FCCOB8_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB8_CCOBn_SHIFT                  0
#define FTFA_FCCOB8_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB8_CCOBn_SHIFT))&FTFA_FCCOB8_CCOBn_MASK)
/* FPROT3 Bit Fields */
#define FTFA_FPROT3_PROT_MASK                    0xFFu
#define FTFA_FPROT3_PROT_SHIFT                   0
#define FTFA_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT3_PROT_SHIFT))&FTFA_FPROT3_PROT_MASK)
/* FPROT2 Bit Fields */
#define FTFA_FPROT2_PROT_MASK                    0xFFu
#define FTFA_FPROT2_PROT_SHIFT                   0
#define FTFA_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT2_PROT_SHIFT))&FTFA_FPROT2_PROT_MASK)
/* FPROT1 Bit Fields */
#define FTFA_FPROT1_PROT_MASK                    0xFFu
#define FTFA_FPROT1_PROT_SHIFT                   0
#define FTFA_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT1_PROT_SHIFT))&FTFA_FPROT1_PROT_MASK)
/* FPROT0 Bit Fields */
#define FTFA_FPROT0_PROT_MASK                    0xFFu
#define FTFA_FPROT0_PROT_SHIFT                   0
#define FTFA_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT0_PROT_SHIFT))&FTFA_FPROT0_PROT_MASK)

/*!
 * @}
 */ /* end of group FTFA_Register_Masks */


/* FTFA - Peripheral instance base addresses */
/** Peripheral FTFA base pointer */
#define FTFA_BASE_PTR                            ((FTFA_MemMapPtr)0xE3C0u)
/** Array initializer of FTFA peripheral base pointers */
#define FTFA_BASE_PTRS                           { FTFA_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register instance definitions */
/* FTFA */
#define FTFA_FSTAT                               FTFA_FSTAT_REG(FTFA_BASE_PTR)
#define FTFA_FCNFG                               FTFA_FCNFG_REG(FTFA_BASE_PTR)
#define FTFA_FSEC                                FTFA_FSEC_REG(FTFA_BASE_PTR)
#define FTFA_FOPT                                FTFA_FOPT_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB3                              FTFA_FCCOB3_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB2                              FTFA_FCCOB2_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB1                              FTFA_FCCOB1_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB0                              FTFA_FCCOB0_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB7                              FTFA_FCCOB7_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB6                              FTFA_FCCOB6_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB5                              FTFA_FCCOB5_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB4                              FTFA_FCCOB4_REG(FTFA_BASE_PTR)
#define FTFA_FCCOBB                              FTFA_FCCOBB_REG(FTFA_BASE_PTR)
#define FTFA_FCCOBA                              FTFA_FCCOBA_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB9                              FTFA_FCCOB9_REG(FTFA_BASE_PTR)
#define FTFA_FCCOB8                              FTFA_FCCOB8_REG(FTFA_BASE_PTR)
#define FTFA_FPROT3                              FTFA_FPROT3_REG(FTFA_BASE_PTR)
#define FTFA_FPROT2                              FTFA_FPROT2_REG(FTFA_BASE_PTR)
#define FTFA_FPROT1                              FTFA_FPROT1_REG(FTFA_BASE_PTR)
#define FTFA_FPROT0                              FTFA_FPROT0_REG(FTFA_BASE_PTR)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FTFA_Peripheral */


/* ----------------------------------------------------------------------------
   -- GPIO
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral GPIO
 * @{
 */

/** GPIO - Peripheral register structure */
typedef struct GPIO_MemMap {
  uint16_t PUR;                                    /**< GPIO Pull Resistor Enable Register, offset: 0x0 */
  uint16_t DR;                                     /**< GPIO Data Register, offset: 0x1 */
  uint16_t DDR;                                    /**< GPIO Data Direction Register, offset: 0x2 */
  uint16_t PER;                                    /**< GPIO Peripheral Enable Register, offset: 0x3 */
  uint16_t IAR;                                    /**< GPIO Interrupt Assert Register, offset: 0x4 */
  uint16_t IENR;                                   /**< GPIO Interrupt Enable Register, offset: 0x5 */
  uint16_t IPOLR;                                  /**< GPIO Interrupt Polarity Register, offset: 0x6 */
  uint16_t IPR;                                    /**< GPIO Interrupt Pending Register, offset: 0x7 */
  uint16_t IESR;                                   /**< GPIO Interrupt Edge Sensitive Register, offset: 0x8 */
  uint16_t PPMODE;                                 /**< GPIO Push-Pull Mode Register, offset: 0x9 */
  uint16_t RAWDATA;                                /**< GPIO Raw Data Register, offset: 0xA */
  uint16_t DRIVE;                                  /**< GPIO Drive Strength Control Register, offset: 0xB */
  uint16_t PUS;                                    /**< GPIO Pull Resistor Type Select, offset: 0xC */
  uint16_t SRE;                                    /**< Slew Rate Control Register, offset: 0xD */
} volatile *GPIO_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register accessors */
#define GPIO_PUR_REG(base)                       ((base)->PUR)
#define GPIO_DR_REG(base)                        ((base)->DR)
#define GPIO_DDR_REG(base)                       ((base)->DDR)
#define GPIO_PER_REG(base)                       ((base)->PER)
#define GPIO_IAR_REG(base)                       ((base)->IAR)
#define GPIO_IENR_REG(base)                      ((base)->IENR)
#define GPIO_IPOLR_REG(base)                     ((base)->IPOLR)
#define GPIO_IPR_REG(base)                       ((base)->IPR)
#define GPIO_IESR_REG(base)                      ((base)->IESR)
#define GPIO_PPMODE_REG(base)                    ((base)->PPMODE)
#define GPIO_RAWDATA_REG(base)                   ((base)->RAWDATA)
#define GPIO_DRIVE_REG(base)                     ((base)->DRIVE)
#define GPIO_PUS_REG(base)                       ((base)->PUS)
#define GPIO_SRE_REG(base)                       ((base)->SRE)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/* PUR Bit Fields */
#define GPIO_PUR_PU_MASK                         0xFFFFu
#define GPIO_PUR_PU_SHIFT                        0
#define GPIO_PUR_PU(x)                           (((uint16_t)(((uint16_t)(x))<<GPIO_PUR_PU_SHIFT))&GPIO_PUR_PU_MASK)
/* DR Bit Fields */
#define GPIO_DR_D_MASK                           0xFFFFu
#define GPIO_DR_D_SHIFT                          0
#define GPIO_DR_D(x)                             (((uint16_t)(((uint16_t)(x))<<GPIO_DR_D_SHIFT))&GPIO_DR_D_MASK)
/* DDR Bit Fields */
#define GPIO_DDR_DD_MASK                         0xFFFFu
#define GPIO_DDR_DD_SHIFT                        0
#define GPIO_DDR_DD(x)                           (((uint16_t)(((uint16_t)(x))<<GPIO_DDR_DD_SHIFT))&GPIO_DDR_DD_MASK)
/* PER Bit Fields */
#define GPIO_PER_PE_MASK                         0xFFFFu
#define GPIO_PER_PE_SHIFT                        0
#define GPIO_PER_PE(x)                           (((uint16_t)(((uint16_t)(x))<<GPIO_PER_PE_SHIFT))&GPIO_PER_PE_MASK)
/* IAR Bit Fields */
#define GPIO_IAR_IA_MASK                         0xFFFFu
#define GPIO_IAR_IA_SHIFT                        0
#define GPIO_IAR_IA(x)                           (((uint16_t)(((uint16_t)(x))<<GPIO_IAR_IA_SHIFT))&GPIO_IAR_IA_MASK)
/* IENR Bit Fields */
#define GPIO_IENR_IEN_MASK                       0xFFFFu
#define GPIO_IENR_IEN_SHIFT                      0
#define GPIO_IENR_IEN(x)                         (((uint16_t)(((uint16_t)(x))<<GPIO_IENR_IEN_SHIFT))&GPIO_IENR_IEN_MASK)
/* IPOLR Bit Fields */
#define GPIO_IPOLR_IPOL_MASK                     0xFFFFu
#define GPIO_IPOLR_IPOL_SHIFT                    0
#define GPIO_IPOLR_IPOL(x)                       (((uint16_t)(((uint16_t)(x))<<GPIO_IPOLR_IPOL_SHIFT))&GPIO_IPOLR_IPOL_MASK)
/* IPR Bit Fields */
#define GPIO_IPR_IP_MASK                         0xFFFFu
#define GPIO_IPR_IP_SHIFT                        0
#define GPIO_IPR_IP(x)                           (((uint16_t)(((uint16_t)(x))<<GPIO_IPR_IP_SHIFT))&GPIO_IPR_IP_MASK)
/* IESR Bit Fields */
#define GPIO_IESR_IES_MASK                       0xFFFFu
#define GPIO_IESR_IES_SHIFT                      0
#define GPIO_IESR_IES(x)                         (((uint16_t)(((uint16_t)(x))<<GPIO_IESR_IES_SHIFT))&GPIO_IESR_IES_MASK)
/* PPMODE Bit Fields */
#define GPIO_PPMODE_PPMODE_MASK                  0xFFFFu
#define GPIO_PPMODE_PPMODE_SHIFT                 0
#define GPIO_PPMODE_PPMODE(x)                    (((uint16_t)(((uint16_t)(x))<<GPIO_PPMODE_PPMODE_SHIFT))&GPIO_PPMODE_PPMODE_MASK)
/* RAWDATA Bit Fields */
#define GPIO_RAWDATA_RAW_DATA_MASK               0xFFFFu
#define GPIO_RAWDATA_RAW_DATA_SHIFT              0
#define GPIO_RAWDATA_RAW_DATA(x)                 (((uint16_t)(((uint16_t)(x))<<GPIO_RAWDATA_RAW_DATA_SHIFT))&GPIO_RAWDATA_RAW_DATA_MASK)
/* DRIVE Bit Fields */
#define GPIO_DRIVE_DRIVE_MASK                    0xFFFFu
#define GPIO_DRIVE_DRIVE_SHIFT                   0
#define GPIO_DRIVE_DRIVE(x)                      (((uint16_t)(((uint16_t)(x))<<GPIO_DRIVE_DRIVE_SHIFT))&GPIO_DRIVE_DRIVE_MASK)
/* PUS Bit Fields */
#define GPIO_PUS_PUS_MASK                        0xFFFFu
#define GPIO_PUS_PUS_SHIFT                       0
#define GPIO_PUS_PUS(x)                          (((uint16_t)(((uint16_t)(x))<<GPIO_PUS_PUS_SHIFT))&GPIO_PUS_PUS_MASK)
/* SRE Bit Fields */
#define GPIO_SRE_SRE_MASK                        0xFFFFu
#define GPIO_SRE_SRE_SHIFT                       0
#define GPIO_SRE_SRE(x)                          (((uint16_t)(((uint16_t)(x))<<GPIO_SRE_SRE_SHIFT))&GPIO_SRE_SRE_MASK)

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIOA base pointer */
#define GPIOA_BASE_PTR                           ((GPIO_MemMapPtr)0xE200u)
/** Peripheral GPIOB base pointer */
#define GPIOB_BASE_PTR                           ((GPIO_MemMapPtr)0xE210u)
/** Peripheral GPIOC base pointer */
#define GPIOC_BASE_PTR                           ((GPIO_MemMapPtr)0xE220u)
/** Peripheral GPIOD base pointer */
#define GPIOD_BASE_PTR                           ((GPIO_MemMapPtr)0xE230u)
/** Peripheral GPIOE base pointer */
#define GPIOE_BASE_PTR                           ((GPIO_MemMapPtr)0xE240u)
/** Peripheral GPIOF base pointer */
#define GPIOF_BASE_PTR                           ((GPIO_MemMapPtr)0xE250u)
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIOA_BASE_PTR, GPIOB_BASE_PTR, GPIOC_BASE_PTR, GPIOD_BASE_PTR, GPIOE_BASE_PTR, GPIOF_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register instance definitions */
/* GPIOA */
#define GPIOA_PUR                                GPIO_PUR_REG(GPIOA_BASE_PTR)
#define GPIOA_DR                                 GPIO_DR_REG(GPIOA_BASE_PTR)
#define GPIOA_DDR                                GPIO_DDR_REG(GPIOA_BASE_PTR)
#define GPIOA_PER                                GPIO_PER_REG(GPIOA_BASE_PTR)
#define GPIOA_IAR                                GPIO_IAR_REG(GPIOA_BASE_PTR)
#define GPIOA_IENR                               GPIO_IENR_REG(GPIOA_BASE_PTR)
#define GPIOA_IPOLR                              GPIO_IPOLR_REG(GPIOA_BASE_PTR)
#define GPIOA_IPR                                GPIO_IPR_REG(GPIOA_BASE_PTR)
#define GPIOA_IESR                               GPIO_IESR_REG(GPIOA_BASE_PTR)
#define GPIOA_PPMODE                             GPIO_PPMODE_REG(GPIOA_BASE_PTR)
#define GPIOA_RAWDATA                            GPIO_RAWDATA_REG(GPIOA_BASE_PTR)
#define GPIOA_DRIVE                              GPIO_DRIVE_REG(GPIOA_BASE_PTR)
#define GPIOA_PUS                                GPIO_PUS_REG(GPIOA_BASE_PTR)
#define GPIOA_SRE                                GPIO_SRE_REG(GPIOA_BASE_PTR)
/* GPIOB */
#define GPIOB_PUR                                GPIO_PUR_REG(GPIOB_BASE_PTR)
#define GPIOB_DR                                 GPIO_DR_REG(GPIOB_BASE_PTR)
#define GPIOB_DDR                                GPIO_DDR_REG(GPIOB_BASE_PTR)
#define GPIOB_PER                                GPIO_PER_REG(GPIOB_BASE_PTR)
#define GPIOB_IAR                                GPIO_IAR_REG(GPIOB_BASE_PTR)
#define GPIOB_IENR                               GPIO_IENR_REG(GPIOB_BASE_PTR)
#define GPIOB_IPOLR                              GPIO_IPOLR_REG(GPIOB_BASE_PTR)
#define GPIOB_IPR                                GPIO_IPR_REG(GPIOB_BASE_PTR)
#define GPIOB_IESR                               GPIO_IESR_REG(GPIOB_BASE_PTR)
#define GPIOB_PPMODE                             GPIO_PPMODE_REG(GPIOB_BASE_PTR)
#define GPIOB_RAWDATA                            GPIO_RAWDATA_REG(GPIOB_BASE_PTR)
#define GPIOB_DRIVE                              GPIO_DRIVE_REG(GPIOB_BASE_PTR)
#define GPIOB_PUS                                GPIO_PUS_REG(GPIOB_BASE_PTR)
#define GPIOB_SRE                                GPIO_SRE_REG(GPIOB_BASE_PTR)
/* GPIOC */
#define GPIOC_PUR                                GPIO_PUR_REG(GPIOC_BASE_PTR)
#define GPIOC_DR                                 GPIO_DR_REG(GPIOC_BASE_PTR)
#define GPIOC_DDR                                GPIO_DDR_REG(GPIOC_BASE_PTR)
#define GPIOC_PER                                GPIO_PER_REG(GPIOC_BASE_PTR)
#define GPIOC_IAR                                GPIO_IAR_REG(GPIOC_BASE_PTR)
#define GPIOC_IENR                               GPIO_IENR_REG(GPIOC_BASE_PTR)
#define GPIOC_IPOLR                              GPIO_IPOLR_REG(GPIOC_BASE_PTR)
#define GPIOC_IPR                                GPIO_IPR_REG(GPIOC_BASE_PTR)
#define GPIOC_IESR                               GPIO_IESR_REG(GPIOC_BASE_PTR)
#define GPIOC_PPMODE                             GPIO_PPMODE_REG(GPIOC_BASE_PTR)
#define GPIOC_RAWDATA                            GPIO_RAWDATA_REG(GPIOC_BASE_PTR)
#define GPIOC_DRIVE                              GPIO_DRIVE_REG(GPIOC_BASE_PTR)
#define GPIOC_PUS                                GPIO_PUS_REG(GPIOC_BASE_PTR)
#define GPIOC_SRE                                GPIO_SRE_REG(GPIOC_BASE_PTR)
/* GPIOD */
#define GPIOD_PUR                                GPIO_PUR_REG(GPIOD_BASE_PTR)
#define GPIOD_DR                                 GPIO_DR_REG(GPIOD_BASE_PTR)
#define GPIOD_DDR                                GPIO_DDR_REG(GPIOD_BASE_PTR)
#define GPIOD_PER                                GPIO_PER_REG(GPIOD_BASE_PTR)
#define GPIOD_IAR                                GPIO_IAR_REG(GPIOD_BASE_PTR)
#define GPIOD_IENR                               GPIO_IENR_REG(GPIOD_BASE_PTR)
#define GPIOD_IPOLR                              GPIO_IPOLR_REG(GPIOD_BASE_PTR)
#define GPIOD_IPR                                GPIO_IPR_REG(GPIOD_BASE_PTR)
#define GPIOD_IESR                               GPIO_IESR_REG(GPIOD_BASE_PTR)
#define GPIOD_PPMODE                             GPIO_PPMODE_REG(GPIOD_BASE_PTR)
#define GPIOD_RAWDATA                            GPIO_RAWDATA_REG(GPIOD_BASE_PTR)
#define GPIOD_DRIVE                              GPIO_DRIVE_REG(GPIOD_BASE_PTR)
#define GPIOD_PUS                                GPIO_PUS_REG(GPIOD_BASE_PTR)
#define GPIOD_SRE                                GPIO_SRE_REG(GPIOD_BASE_PTR)
/* GPIOE */
#define GPIOE_PUR                                GPIO_PUR_REG(GPIOE_BASE_PTR)
#define GPIOE_DR                                 GPIO_DR_REG(GPIOE_BASE_PTR)
#define GPIOE_DDR                                GPIO_DDR_REG(GPIOE_BASE_PTR)
#define GPIOE_PER                                GPIO_PER_REG(GPIOE_BASE_PTR)
#define GPIOE_IAR                                GPIO_IAR_REG(GPIOE_BASE_PTR)
#define GPIOE_IENR                               GPIO_IENR_REG(GPIOE_BASE_PTR)
#define GPIOE_IPOLR                              GPIO_IPOLR_REG(GPIOE_BASE_PTR)
#define GPIOE_IPR                                GPIO_IPR_REG(GPIOE_BASE_PTR)
#define GPIOE_IESR                               GPIO_IESR_REG(GPIOE_BASE_PTR)
#define GPIOE_PPMODE                             GPIO_PPMODE_REG(GPIOE_BASE_PTR)
#define GPIOE_RAWDATA                            GPIO_RAWDATA_REG(GPIOE_BASE_PTR)
#define GPIOE_DRIVE                              GPIO_DRIVE_REG(GPIOE_BASE_PTR)
#define GPIOE_PUS                                GPIO_PUS_REG(GPIOE_BASE_PTR)
#define GPIOE_SRE                                GPIO_SRE_REG(GPIOE_BASE_PTR)
/* GPIOF */
#define GPIOF_PUR                                GPIO_PUR_REG(GPIOF_BASE_PTR)
#define GPIOF_DR                                 GPIO_DR_REG(GPIOF_BASE_PTR)
#define GPIOF_DDR                                GPIO_DDR_REG(GPIOF_BASE_PTR)
#define GPIOF_PER                                GPIO_PER_REG(GPIOF_BASE_PTR)
#define GPIOF_IAR                                GPIO_IAR_REG(GPIOF_BASE_PTR)
#define GPIOF_IENR                               GPIO_IENR_REG(GPIOF_BASE_PTR)
#define GPIOF_IPOLR                              GPIO_IPOLR_REG(GPIOF_BASE_PTR)
#define GPIOF_IPR                                GPIO_IPR_REG(GPIOF_BASE_PTR)
#define GPIOF_IESR                               GPIO_IESR_REG(GPIOF_BASE_PTR)
#define GPIOF_PPMODE                             GPIO_PPMODE_REG(GPIOF_BASE_PTR)
#define GPIOF_RAWDATA                            GPIO_RAWDATA_REG(GPIOF_BASE_PTR)
#define GPIOF_DRIVE                              GPIO_DRIVE_REG(GPIOF_BASE_PTR)
#define GPIOF_PUS                                GPIO_PUS_REG(GPIOF_BASE_PTR)
#define GPIOF_SRE                                GPIO_SRE_REG(GPIOF_BASE_PTR)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group GPIO_Peripheral */


/* ----------------------------------------------------------------------------
   -- I2C
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral I2C
 * @{
 */

/** I2C - Peripheral register structure */
typedef struct I2C_MemMap {
  uint16_t A1;                                     /**< I2C Address Register 1, offset: 0x0 */
  uint16_t F;                                      /**< I2C Frequency Divider register, offset: 0x1 */
  uint16_t C1;                                     /**< I2C Control Register 1, offset: 0x2 */
  uint16_t S;                                      /**< I2C Status register, offset: 0x3 */
  uint16_t D;                                      /**< I2C Data I/O register, offset: 0x4 */
  uint16_t C2;                                     /**< I2C Control Register 2, offset: 0x5 */
  uint16_t FLT;                                    /**< I2C Programmable Input Glitch Filter register, offset: 0x6 */
  uint16_t RA;                                     /**< I2C Range Address register, offset: 0x7 */
  uint16_t SMB;                                    /**< I2C SMBus Control and Status register, offset: 0x8 */
  uint16_t A2;                                     /**< I2C Address Register 2, offset: 0x9 */
  uint16_t SLTH;                                   /**< I2C SCL Low Timeout Register High, offset: 0xA */
  uint16_t SLTL;                                   /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} volatile *I2C_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register accessors */
#define I2C_A1_REG(base)                         ((base)->A1)
#define I2C_F_REG(base)                          ((base)->F)
#define I2C_C1_REG(base)                         ((base)->C1)
#define I2C_S_REG(base)                          ((base)->S)
#define I2C_D_REG(base)                          ((base)->D)
#define I2C_C2_REG(base)                         ((base)->C2)
#define I2C_FLT_REG(base)                        ((base)->FLT)
#define I2C_RA_REG(base)                         ((base)->RA)
#define I2C_SMB_REG(base)                        ((base)->SMB)
#define I2C_A2_REG(base)                         ((base)->A2)
#define I2C_SLTH_REG(base)                       ((base)->SLTH)
#define I2C_SLTL_REG(base)                       ((base)->SLTL)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/* A1 Bit Fields */
#define I2C_A1_AD_MASK                           0xFEu
#define I2C_A1_AD_SHIFT                          1
#define I2C_A1_AD(x)                             (((uint16_t)(((uint16_t)(x))<<I2C_A1_AD_SHIFT))&I2C_A1_AD_MASK)
/* F Bit Fields */
#define I2C_F_ICR_MASK                           0x3Fu
#define I2C_F_ICR_SHIFT                          0
#define I2C_F_ICR(x)                             (((uint16_t)(((uint16_t)(x))<<I2C_F_ICR_SHIFT))&I2C_F_ICR_MASK)
#define I2C_F_MULT_MASK                          0xC0u
#define I2C_F_MULT_SHIFT                         6
#define I2C_F_MULT(x)                            (((uint16_t)(((uint16_t)(x))<<I2C_F_MULT_SHIFT))&I2C_F_MULT_MASK)
/* C1 Bit Fields */
#define I2C_C1_DMAEN_MASK                        0x1u
#define I2C_C1_DMAEN_SHIFT                       0
#define I2C_C1_WUEN_MASK                         0x2u
#define I2C_C1_WUEN_SHIFT                        1
#define I2C_C1_RSTA_MASK                         0x4u
#define I2C_C1_RSTA_SHIFT                        2
#define I2C_C1_TXAK_MASK                         0x8u
#define I2C_C1_TXAK_SHIFT                        3
#define I2C_C1_TX_MASK                           0x10u
#define I2C_C1_TX_SHIFT                          4
#define I2C_C1_MST_MASK                          0x20u
#define I2C_C1_MST_SHIFT                         5
#define I2C_C1_IICIE_MASK                        0x40u
#define I2C_C1_IICIE_SHIFT                       6
#define I2C_C1_IICEN_MASK                        0x80u
#define I2C_C1_IICEN_SHIFT                       7
/* S Bit Fields */
#define I2C_S_RXAK_MASK                          0x1u
#define I2C_S_RXAK_SHIFT                         0
#define I2C_S_IICIF_MASK                         0x2u
#define I2C_S_IICIF_SHIFT                        1
#define I2C_S_SRW_MASK                           0x4u
#define I2C_S_SRW_SHIFT                          2
#define I2C_S_RAM_MASK                           0x8u
#define I2C_S_RAM_SHIFT                          3
#define I2C_S_ARBL_MASK                          0x10u
#define I2C_S_ARBL_SHIFT                         4
#define I2C_S_BUSY_MASK                          0x20u
#define I2C_S_BUSY_SHIFT                         5
#define I2C_S_IAAS_MASK                          0x40u
#define I2C_S_IAAS_SHIFT                         6
#define I2C_S_TCF_MASK                           0x80u
#define I2C_S_TCF_SHIFT                          7
/* D Bit Fields */
#define I2C_D_DATA_MASK                          0xFFu
#define I2C_D_DATA_SHIFT                         0
#define I2C_D_DATA(x)                            (((uint16_t)(((uint16_t)(x))<<I2C_D_DATA_SHIFT))&I2C_D_DATA_MASK)
/* C2 Bit Fields */
#define I2C_C2_AD_MASK                           0x7u
#define I2C_C2_AD_SHIFT                          0
#define I2C_C2_AD(x)                             (((uint16_t)(((uint16_t)(x))<<I2C_C2_AD_SHIFT))&I2C_C2_AD_MASK)
#define I2C_C2_RMEN_MASK                         0x8u
#define I2C_C2_RMEN_SHIFT                        3
#define I2C_C2_SBRC_MASK                         0x10u
#define I2C_C2_SBRC_SHIFT                        4
#define I2C_C2_HDRS_MASK                         0x20u
#define I2C_C2_HDRS_SHIFT                        5
#define I2C_C2_ADEXT_MASK                        0x40u
#define I2C_C2_ADEXT_SHIFT                       6
#define I2C_C2_GCAEN_MASK                        0x80u
#define I2C_C2_GCAEN_SHIFT                       7
/* FLT Bit Fields */
#define I2C_FLT_FLT_MASK                         0xFu
#define I2C_FLT_FLT_SHIFT                        0
#define I2C_FLT_FLT(x)                           (((uint16_t)(((uint16_t)(x))<<I2C_FLT_FLT_SHIFT))&I2C_FLT_FLT_MASK)
#define I2C_FLT_STARTF_MASK                      0x10u
#define I2C_FLT_STARTF_SHIFT                     4
#define I2C_FLT_SSIE_MASK                        0x20u
#define I2C_FLT_SSIE_SHIFT                       5
#define I2C_FLT_STOPF_MASK                       0x40u
#define I2C_FLT_STOPF_SHIFT                      6
#define I2C_FLT_SHEN_MASK                        0x80u
#define I2C_FLT_SHEN_SHIFT                       7
/* RA Bit Fields */
#define I2C_RA_RAD_MASK                          0xFEu
#define I2C_RA_RAD_SHIFT                         1
#define I2C_RA_RAD(x)                            (((uint16_t)(((uint16_t)(x))<<I2C_RA_RAD_SHIFT))&I2C_RA_RAD_MASK)
/* SMB Bit Fields */
#define I2C_SMB_SHTF2IE_MASK                     0x1u
#define I2C_SMB_SHTF2IE_SHIFT                    0
#define I2C_SMB_SHTF2_MASK                       0x2u
#define I2C_SMB_SHTF2_SHIFT                      1
#define I2C_SMB_SHTF1_MASK                       0x4u
#define I2C_SMB_SHTF1_SHIFT                      2
#define I2C_SMB_SLTF_MASK                        0x8u
#define I2C_SMB_SLTF_SHIFT                       3
#define I2C_SMB_TCKSEL_MASK                      0x10u
#define I2C_SMB_TCKSEL_SHIFT                     4
#define I2C_SMB_SIICAEN_MASK                     0x20u
#define I2C_SMB_SIICAEN_SHIFT                    5
#define I2C_SMB_ALERTEN_MASK                     0x40u
#define I2C_SMB_ALERTEN_SHIFT                    6
#define I2C_SMB_FACK_MASK                        0x80u
#define I2C_SMB_FACK_SHIFT                       7
/* A2 Bit Fields */
#define I2C_A2_SAD_MASK                          0xFEu
#define I2C_A2_SAD_SHIFT                         1
#define I2C_A2_SAD(x)                            (((uint16_t)(((uint16_t)(x))<<I2C_A2_SAD_SHIFT))&I2C_A2_SAD_MASK)
/* SLTH Bit Fields */
#define I2C_SLTH_SSLT_MASK                       0xFFu
#define I2C_SLTH_SSLT_SHIFT                      0
#define I2C_SLTH_SSLT(x)                         (((uint16_t)(((uint16_t)(x))<<I2C_SLTH_SSLT_SHIFT))&I2C_SLTH_SSLT_MASK)
/* SLTL Bit Fields */
#define I2C_SLTL_SSLT_MASK                       0xFFu
#define I2C_SLTL_SSLT_SHIFT                      0
#define I2C_SLTL_SSLT(x)                         (((uint16_t)(((uint16_t)(x))<<I2C_SLTL_SSLT_SHIFT))&I2C_SLTL_SSLT_MASK)

/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C base pointer */
#define I2C_BASE_PTR                             ((I2C_MemMapPtr)0xE0E0u)
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register instance definitions */
/* I2C */
#define I2C_A1                                   I2C_A1_REG(I2C_BASE_PTR)
#define I2C_F                                    I2C_F_REG(I2C_BASE_PTR)
#define I2C_C1                                   I2C_C1_REG(I2C_BASE_PTR)
#define I2C_S                                    I2C_S_REG(I2C_BASE_PTR)
#define I2C_D                                    I2C_D_REG(I2C_BASE_PTR)
#define I2C_C2                                   I2C_C2_REG(I2C_BASE_PTR)
#define I2C_FLT                                  I2C_FLT_REG(I2C_BASE_PTR)
#define I2C_RA                                   I2C_RA_REG(I2C_BASE_PTR)
#define I2C_SMB                                  I2C_SMB_REG(I2C_BASE_PTR)
#define I2C_A2                                   I2C_A2_REG(I2C_BASE_PTR)
#define I2C_SLTH                                 I2C_SLTH_REG(I2C_BASE_PTR)
#define I2C_SLTL                                 I2C_SLTL_REG(I2C_BASE_PTR)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group I2C_Peripheral */


/* ----------------------------------------------------------------------------
   -- INTC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Peripheral INTC
 * @{
 */

/** INTC - Peripheral register structure */
typedef struct INTC_MemMap {
  uint16_t IPR0;                                   /**< Interrupt Priority Register 0, offset: 0x0 */
  uint16_t IPR1;                                   /**< Interrupt Priority Register 1, offset: 0x1 */
  uint16_t IPR2;                                   /**< Interrupt Priority Register 2, offset: 0x2 */
  uint16_t IPR3;                                   /**< Interrupt Priority Register 3, offset: 0x3 */
  uint16_t IPR4;                                   /**< Interrupt Priority Register 4, offset: 0x4 */
  uint16_t IPR5;                                   /**< Interrupt Priority Register 5, offset: 0x5 */
  uint16_t IPR6;                                   /**< Interrupt Priority Register 6, offset: 0x6 */
  uint16_t RESERVED_0[1];
  uint16_t IPR8;                                   /**< Interrupt Priority Register 8, offset: 0x8 */
  uint16_t IPR9;                                   /**< Interrupt Priority Register 9, offset: 0x9 */
  uint16_t IPR10;                                  /**< Interrupt Priority Register 10, offset: 0xA */
  uint16_t IPR11;                                  /**< Interrupt Priority Register 11, offset: 0xB */
  uint16_t IPR12;                                  /**< Interrupt Priority Register 12, offset: 0xC */
  uint16_t VBA;                                    /**< Vector Base Address Register, offset: 0xD */
  uint16_t FIM0;                                   /**< Fast Interrupt 0 Match Register, offset: 0xE */
  uint16_t FIVAL0;                                 /**< Fast Interrupt 0 Vector Address Low Register, offset: 0xF */
  uint16_t FIVAH0;                                 /**< Fast Interrupt 0 Vector Address High Register, offset: 0x10 */
  uint16_t FIM1;                                   /**< Fast Interrupt 1 Match Register, offset: 0x11 */
  uint16_t FIVAL1;                                 /**< Fast Interrupt 1 Vector Address Low Register, offset: 0x12 */
  uint16_t FIVAH1;                                 /**< Fast Interrupt 1 Vector Address High Register, offset: 0x13 */
  uint16_t IRQP0;                                  /**< IRQ Pending Register 0, offset: 0x14 */
  uint16_t IRQP1;                                  /**< IRQ Pending Register 1, offset: 0x15 */
  uint16_t IRQP2;                                  /**< IRQ Pending Register 2, offset: 0x16 */
  uint16_t IRQP3;                                  /**< IRQ Pending Register 3, offset: 0x17 */
  uint16_t IRQP4;                                  /**< IRQ Pending Register 4, offset: 0x18 */
  uint16_t IRQP5;                                  /**< IRQ Pending Register 5, offset: 0x19 */
  uint16_t IRQP6;                                  /**< IRQ Pending Register 6, offset: 0x1A */
  uint16_t CTRL;                                   /**< Control Register, offset: 0x1B */
} volatile *INTC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- INTC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Register_Accessor_Macros INTC - Register accessor macros
 * @{
 */


/* INTC - Register accessors */
#define INTC_IPR0_REG(base)                      ((base)->IPR0)
#define INTC_IPR1_REG(base)                      ((base)->IPR1)
#define INTC_IPR2_REG(base)                      ((base)->IPR2)
#define INTC_IPR3_REG(base)                      ((base)->IPR3)
#define INTC_IPR4_REG(base)                      ((base)->IPR4)
#define INTC_IPR5_REG(base)                      ((base)->IPR5)
#define INTC_IPR6_REG(base)                      ((base)->IPR6)
#define INTC_IPR8_REG(base)                      ((base)->IPR8)
#define INTC_IPR9_REG(base)                      ((base)->IPR9)
#define INTC_IPR10_REG(base)                     ((base)->IPR10)
#define INTC_IPR11_REG(base)                     ((base)->IPR11)
#define INTC_IPR12_REG(base)                     ((base)->IPR12)
#define INTC_VBA_REG(base)                       ((base)->VBA)
#define INTC_FIM0_REG(base)                      ((base)->FIM0)
#define INTC_FIVAL0_REG(base)                    ((base)->FIVAL0)
#define INTC_FIVAH0_REG(base)                    ((base)->FIVAH0)
#define INTC_FIM1_REG(base)                      ((base)->FIM1)
#define INTC_FIVAL1_REG(base)                    ((base)->FIVAL1)
#define INTC_FIVAH1_REG(base)                    ((base)->FIVAH1)
#define INTC_IRQP0_REG(base)                     ((base)->IRQP0)
#define INTC_IRQP1_REG(base)                     ((base)->IRQP1)
#define INTC_IRQP2_REG(base)                     ((base)->IRQP2)
#define INTC_IRQP3_REG(base)                     ((base)->IRQP3)
#define INTC_IRQP4_REG(base)                     ((base)->IRQP4)
#define INTC_IRQP5_REG(base)                     ((base)->IRQP5)
#define INTC_IRQP6_REG(base)                     ((base)->IRQP6)
#define INTC_CTRL_REG(base)                      ((base)->CTRL)

/*!
 * @}
 */ /* end of group INTC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- INTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Register_Masks INTC Register Masks
 * @{
 */

/* IPR0 Bit Fields */
#define INTC_IPR0_STPCNT_MASK                    0x3u
#define INTC_IPR0_STPCNT_SHIFT                   0
#define INTC_IPR0_STPCNT(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_STPCNT_SHIFT))&INTC_IPR0_STPCNT_MASK)
#define INTC_IPR0_BKPT_MASK                      0xCu
#define INTC_IPR0_BKPT_SHIFT                     2
#define INTC_IPR0_BKPT(x)                        (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_BKPT_SHIFT))&INTC_IPR0_BKPT_MASK)
#define INTC_IPR0_TRBUF_MASK                     0x30u
#define INTC_IPR0_TRBUF_SHIFT                    4
#define INTC_IPR0_TRBUF(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_TRBUF_SHIFT))&INTC_IPR0_TRBUF_MASK)
#define INTC_IPR0_TX_REG_MASK                    0xC0u
#define INTC_IPR0_TX_REG_SHIFT                   6
#define INTC_IPR0_TX_REG(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_TX_REG_SHIFT))&INTC_IPR0_TX_REG_MASK)
#define INTC_IPR0_RX_REG_MASK                    0x300u
#define INTC_IPR0_RX_REG_SHIFT                   8
#define INTC_IPR0_RX_REG(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_RX_REG_SHIFT))&INTC_IPR0_RX_REG_MASK)
#define INTC_IPR0_BUS_ERR_MASK                   0xC00u
#define INTC_IPR0_BUS_ERR_SHIFT                  10
#define INTC_IPR0_BUS_ERR(x)                     (((uint16_t)(((uint16_t)(x))<<INTC_IPR0_BUS_ERR_SHIFT))&INTC_IPR0_BUS_ERR_MASK)
/* IPR1 Bit Fields */
#define INTC_IPR1_XBARA_MASK                     0xCu
#define INTC_IPR1_XBARA_SHIFT                    2
#define INTC_IPR1_XBARA(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR1_XBARA_SHIFT))&INTC_IPR1_XBARA_MASK)
#define INTC_IPR1_LVI1_MASK                      0x30u
#define INTC_IPR1_LVI1_SHIFT                     4
#define INTC_IPR1_LVI1(x)                        (((uint16_t)(((uint16_t)(x))<<INTC_IPR1_LVI1_SHIFT))&INTC_IPR1_LVI1_MASK)
#define INTC_IPR1_OCCS_MASK                      0xC0u
#define INTC_IPR1_OCCS_SHIFT                     6
#define INTC_IPR1_OCCS(x)                        (((uint16_t)(((uint16_t)(x))<<INTC_IPR1_OCCS_SHIFT))&INTC_IPR1_OCCS_MASK)
/* IPR2 Bit Fields */
#define INTC_IPR2_TMRA_3_MASK                    0x3u
#define INTC_IPR2_TMRA_3_SHIFT                   0
#define INTC_IPR2_TMRA_3(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_TMRA_3_SHIFT))&INTC_IPR2_TMRA_3_MASK)
#define INTC_IPR2_TMRA_2_MASK                    0xCu
#define INTC_IPR2_TMRA_2_SHIFT                   2
#define INTC_IPR2_TMRA_2(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_TMRA_2_SHIFT))&INTC_IPR2_TMRA_2_MASK)
#define INTC_IPR2_TMRA_1_MASK                    0x30u
#define INTC_IPR2_TMRA_1_SHIFT                   4
#define INTC_IPR2_TMRA_1(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_TMRA_1_SHIFT))&INTC_IPR2_TMRA_1_MASK)
#define INTC_IPR2_TMRA_0_MASK                    0xC0u
#define INTC_IPR2_TMRA_0_SHIFT                   6
#define INTC_IPR2_TMRA_0(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_TMRA_0_SHIFT))&INTC_IPR2_TMRA_0_MASK)
#define INTC_IPR2_ADC_CC1_MASK                   0x300u
#define INTC_IPR2_ADC_CC1_SHIFT                  8
#define INTC_IPR2_ADC_CC1(x)                     (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_ADC_CC1_SHIFT))&INTC_IPR2_ADC_CC1_MASK)
#define INTC_IPR2_ADC_CC0_MASK                   0xC00u
#define INTC_IPR2_ADC_CC0_SHIFT                  10
#define INTC_IPR2_ADC_CC0(x)                     (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_ADC_CC0_SHIFT))&INTC_IPR2_ADC_CC0_MASK)
#define INTC_IPR2_ADC_ERR_MASK                   0x3000u
#define INTC_IPR2_ADC_ERR_SHIFT                  12
#define INTC_IPR2_ADC_ERR(x)                     (((uint16_t)(((uint16_t)(x))<<INTC_IPR2_ADC_ERR_SHIFT))&INTC_IPR2_ADC_ERR_MASK)
/* IPR3 Bit Fields */
#define INTC_IPR3_DMACH3_MASK                    0x3u
#define INTC_IPR3_DMACH3_SHIFT                   0
#define INTC_IPR3_DMACH3(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_DMACH3_SHIFT))&INTC_IPR3_DMACH3_MASK)
#define INTC_IPR3_DMACH2_MASK                    0xCu
#define INTC_IPR3_DMACH2_SHIFT                   2
#define INTC_IPR3_DMACH2(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_DMACH2_SHIFT))&INTC_IPR3_DMACH2_MASK)
#define INTC_IPR3_DMACH1_MASK                    0x30u
#define INTC_IPR3_DMACH1_SHIFT                   4
#define INTC_IPR3_DMACH1(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_DMACH1_SHIFT))&INTC_IPR3_DMACH1_MASK)
#define INTC_IPR3_DMACH0_MASK                    0xC0u
#define INTC_IPR3_DMACH0_SHIFT                   6
#define INTC_IPR3_DMACH0(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_DMACH0_SHIFT))&INTC_IPR3_DMACH0_MASK)
#define INTC_IPR3_CAN_ERROR_MASK                 0x3000u
#define INTC_IPR3_CAN_ERROR_SHIFT                12
#define INTC_IPR3_CAN_ERROR(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_CAN_ERROR_SHIFT))&INTC_IPR3_CAN_ERROR_MASK)
#define INTC_IPR3_CAN_TX_WARN_MASK               0xC000u
#define INTC_IPR3_CAN_TX_WARN_SHIFT              14
#define INTC_IPR3_CAN_TX_WARN(x)                 (((uint16_t)(((uint16_t)(x))<<INTC_IPR3_CAN_TX_WARN_SHIFT))&INTC_IPR3_CAN_TX_WARN_MASK)
/* IPR4 Bit Fields */
#define INTC_IPR4_CAN_RX_WARN_MASK               0x3u
#define INTC_IPR4_CAN_RX_WARN_SHIFT              0
#define INTC_IPR4_CAN_RX_WARN(x)                 (((uint16_t)(((uint16_t)(x))<<INTC_IPR4_CAN_RX_WARN_SHIFT))&INTC_IPR4_CAN_RX_WARN_MASK)
#define INTC_IPR4_CAN_WAKEUP_MASK                0xCu
#define INTC_IPR4_CAN_WAKEUP_SHIFT               2
#define INTC_IPR4_CAN_WAKEUP(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR4_CAN_WAKEUP_SHIFT))&INTC_IPR4_CAN_WAKEUP_MASK)
#define INTC_IPR4_QSCI1_RERR_MASK                0x3000u
#define INTC_IPR4_QSCI1_RERR_SHIFT               12
#define INTC_IPR4_QSCI1_RERR(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR4_QSCI1_RERR_SHIFT))&INTC_IPR4_QSCI1_RERR_MASK)
#define INTC_IPR4_QSCI1_RCV_MASK                 0xC000u
#define INTC_IPR4_QSCI1_RCV_SHIFT                14
#define INTC_IPR4_QSCI1_RCV(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR4_QSCI1_RCV_SHIFT))&INTC_IPR4_QSCI1_RCV_MASK)
/* IPR5 Bit Fields */
#define INTC_IPR5_QSCI1_TIDLE_MASK               0x3u
#define INTC_IPR5_QSCI1_TIDLE_SHIFT              0
#define INTC_IPR5_QSCI1_TIDLE(x)                 (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI1_TIDLE_SHIFT))&INTC_IPR5_QSCI1_TIDLE_MASK)
#define INTC_IPR5_QSCI1_TDRE_MASK                0xCu
#define INTC_IPR5_QSCI1_TDRE_SHIFT               2
#define INTC_IPR5_QSCI1_TDRE(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI1_TDRE_SHIFT))&INTC_IPR5_QSCI1_TDRE_MASK)
#define INTC_IPR5_QSCI0_RERR_MASK                0x30u
#define INTC_IPR5_QSCI0_RERR_SHIFT               4
#define INTC_IPR5_QSCI0_RERR(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI0_RERR_SHIFT))&INTC_IPR5_QSCI0_RERR_MASK)
#define INTC_IPR5_QSCI0_RCV_MASK                 0xC0u
#define INTC_IPR5_QSCI0_RCV_SHIFT                6
#define INTC_IPR5_QSCI0_RCV(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI0_RCV_SHIFT))&INTC_IPR5_QSCI0_RCV_MASK)
#define INTC_IPR5_QSCI0_TIDLE_MASK               0x300u
#define INTC_IPR5_QSCI0_TIDLE_SHIFT              8
#define INTC_IPR5_QSCI0_TIDLE(x)                 (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI0_TIDLE_SHIFT))&INTC_IPR5_QSCI0_TIDLE_MASK)
#define INTC_IPR5_QSCI0_TDRE_MASK                0xC00u
#define INTC_IPR5_QSCI0_TDRE_SHIFT               10
#define INTC_IPR5_QSCI0_TDRE(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR5_QSCI0_TDRE_SHIFT))&INTC_IPR5_QSCI0_TDRE_MASK)
/* IPR6 Bit Fields */
#define INTC_IPR6_QSPI1_XMIT_MASK                0x3u
#define INTC_IPR6_QSPI1_XMIT_SHIFT               0
#define INTC_IPR6_QSPI1_XMIT(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR6_QSPI1_XMIT_SHIFT))&INTC_IPR6_QSPI1_XMIT_MASK)
#define INTC_IPR6_QSPI1_RCV_MASK                 0xCu
#define INTC_IPR6_QSPI1_RCV_SHIFT                2
#define INTC_IPR6_QSPI1_RCV(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR6_QSPI1_RCV_SHIFT))&INTC_IPR6_QSPI1_RCV_MASK)
#define INTC_IPR6_QSPI0_XMIT_MASK                0x30u
#define INTC_IPR6_QSPI0_XMIT_SHIFT               4
#define INTC_IPR6_QSPI0_XMIT(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR6_QSPI0_XMIT_SHIFT))&INTC_IPR6_QSPI0_XMIT_MASK)
#define INTC_IPR6_QSPI0_RCV_MASK                 0xC0u
#define INTC_IPR6_QSPI0_RCV_SHIFT                6
#define INTC_IPR6_QSPI0_RCV(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR6_QSPI0_RCV_SHIFT))&INTC_IPR6_QSPI0_RCV_MASK)
#define INTC_IPR6_IIC0_MASK                      0xC00u
#define INTC_IPR6_IIC0_SHIFT                     10
#define INTC_IPR6_IIC0(x)                        (((uint16_t)(((uint16_t)(x))<<INTC_IPR6_IIC0_SHIFT))&INTC_IPR6_IIC0_MASK)
/* IPR8 Bit Fields */
#define INTC_IPR8_PWMA_FAULT_MASK                0x300u
#define INTC_IPR8_PWMA_FAULT_SHIFT               8
#define INTC_IPR8_PWMA_FAULT(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR8_PWMA_FAULT_SHIFT))&INTC_IPR8_PWMA_FAULT_MASK)
#define INTC_IPR8_PWMA_RERR_MASK                 0xC00u
#define INTC_IPR8_PWMA_RERR_SHIFT                10
#define INTC_IPR8_PWMA_RERR(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR8_PWMA_RERR_SHIFT))&INTC_IPR8_PWMA_RERR_MASK)
#define INTC_IPR8_PWMA_RELOAD3_MASK              0x3000u
#define INTC_IPR8_PWMA_RELOAD3_SHIFT             12
#define INTC_IPR8_PWMA_RELOAD3(x)                (((uint16_t)(((uint16_t)(x))<<INTC_IPR8_PWMA_RELOAD3_SHIFT))&INTC_IPR8_PWMA_RELOAD3_MASK)
#define INTC_IPR8_PWMA_CAP_MASK                  0xC000u
#define INTC_IPR8_PWMA_CAP_SHIFT                 14
#define INTC_IPR8_PWMA_CAP(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IPR8_PWMA_CAP_SHIFT))&INTC_IPR8_PWMA_CAP_MASK)
/* IPR9 Bit Fields */
#define INTC_IPR9_PWMA_CMP3_MASK                 0x3u
#define INTC_IPR9_PWMA_CMP3_SHIFT                0
#define INTC_IPR9_PWMA_CMP3(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_CMP3_SHIFT))&INTC_IPR9_PWMA_CMP3_MASK)
#define INTC_IPR9_PWMA_RELOAD2_MASK              0xCu
#define INTC_IPR9_PWMA_RELOAD2_SHIFT             2
#define INTC_IPR9_PWMA_RELOAD2(x)                (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_RELOAD2_SHIFT))&INTC_IPR9_PWMA_RELOAD2_MASK)
#define INTC_IPR9_PWMA_CMP2_MASK                 0x30u
#define INTC_IPR9_PWMA_CMP2_SHIFT                4
#define INTC_IPR9_PWMA_CMP2(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_CMP2_SHIFT))&INTC_IPR9_PWMA_CMP2_MASK)
#define INTC_IPR9_PWMA_RELOAD1_MASK              0xC0u
#define INTC_IPR9_PWMA_RELOAD1_SHIFT             6
#define INTC_IPR9_PWMA_RELOAD1(x)                (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_RELOAD1_SHIFT))&INTC_IPR9_PWMA_RELOAD1_MASK)
#define INTC_IPR9_PWMA_CMP1_MASK                 0x300u
#define INTC_IPR9_PWMA_CMP1_SHIFT                8
#define INTC_IPR9_PWMA_CMP1(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_CMP1_SHIFT))&INTC_IPR9_PWMA_CMP1_MASK)
#define INTC_IPR9_PWMA_RELOAD0_MASK              0xC00u
#define INTC_IPR9_PWMA_RELOAD0_SHIFT             10
#define INTC_IPR9_PWMA_RELOAD0(x)                (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_RELOAD0_SHIFT))&INTC_IPR9_PWMA_RELOAD0_MASK)
#define INTC_IPR9_PWMA_CMP0_MASK                 0x3000u
#define INTC_IPR9_PWMA_CMP0_SHIFT                12
#define INTC_IPR9_PWMA_CMP0(x)                   (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_PWMA_CMP0_SHIFT))&INTC_IPR9_PWMA_CMP0_MASK)
#define INTC_IPR9_FTFA_RDCOL_MASK                0xC000u
#define INTC_IPR9_FTFA_RDCOL_SHIFT               14
#define INTC_IPR9_FTFA_RDCOL(x)                  (((uint16_t)(((uint16_t)(x))<<INTC_IPR9_FTFA_RDCOL_SHIFT))&INTC_IPR9_FTFA_RDCOL_MASK)
/* IPR10 Bit Fields */
#define INTC_IPR10_FTFA_CC_MASK                  0x3u
#define INTC_IPR10_FTFA_CC_SHIFT                 0
#define INTC_IPR10_FTFA_CC(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_FTFA_CC_SHIFT))&INTC_IPR10_FTFA_CC_MASK)
#define INTC_IPR10_CMPD_MASK                     0xCu
#define INTC_IPR10_CMPD_SHIFT                    2
#define INTC_IPR10_CMPD(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_CMPD_SHIFT))&INTC_IPR10_CMPD_MASK)
#define INTC_IPR10_CMPC_MASK                     0x30u
#define INTC_IPR10_CMPC_SHIFT                    4
#define INTC_IPR10_CMPC(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_CMPC_SHIFT))&INTC_IPR10_CMPC_MASK)
#define INTC_IPR10_CMPB_MASK                     0xC0u
#define INTC_IPR10_CMPB_SHIFT                    6
#define INTC_IPR10_CMPB(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_CMPB_SHIFT))&INTC_IPR10_CMPB_MASK)
#define INTC_IPR10_CMPA_MASK                     0x300u
#define INTC_IPR10_CMPA_SHIFT                    8
#define INTC_IPR10_CMPA(x)                       (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_CMPA_SHIFT))&INTC_IPR10_CMPA_MASK)
#define INTC_IPR10_PIT1_ROLLOVR_MASK             0xC00u
#define INTC_IPR10_PIT1_ROLLOVR_SHIFT            10
#define INTC_IPR10_PIT1_ROLLOVR(x)               (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_PIT1_ROLLOVR_SHIFT))&INTC_IPR10_PIT1_ROLLOVR_MASK)
#define INTC_IPR10_PIT0_ROLLOVR_MASK             0x3000u
#define INTC_IPR10_PIT0_ROLLOVR_SHIFT            12
#define INTC_IPR10_PIT0_ROLLOVR(x)               (((uint16_t)(((uint16_t)(x))<<INTC_IPR10_PIT0_ROLLOVR_SHIFT))&INTC_IPR10_PIT0_ROLLOVR_MASK)
/* IPR11 Bit Fields */
#define INTC_IPR11_GPIOF_MASK                    0xC00u
#define INTC_IPR11_GPIOF_SHIFT                   10
#define INTC_IPR11_GPIOF(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR11_GPIOF_SHIFT))&INTC_IPR11_GPIOF_MASK)
#define INTC_IPR11_GPIOE_MASK                    0x3000u
#define INTC_IPR11_GPIOE_SHIFT                   12
#define INTC_IPR11_GPIOE(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR11_GPIOE_SHIFT))&INTC_IPR11_GPIOE_MASK)
#define INTC_IPR11_GPIOD_MASK                    0xC000u
#define INTC_IPR11_GPIOD_SHIFT                   14
#define INTC_IPR11_GPIOD(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR11_GPIOD_SHIFT))&INTC_IPR11_GPIOD_MASK)
/* IPR12 Bit Fields */
#define INTC_IPR12_GPIOC_MASK                    0x3u
#define INTC_IPR12_GPIOC_SHIFT                   0
#define INTC_IPR12_GPIOC(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR12_GPIOC_SHIFT))&INTC_IPR12_GPIOC_MASK)
#define INTC_IPR12_GPIOB_MASK                    0xCu
#define INTC_IPR12_GPIOB_SHIFT                   2
#define INTC_IPR12_GPIOB(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR12_GPIOB_SHIFT))&INTC_IPR12_GPIOB_MASK)
#define INTC_IPR12_GPIOA_MASK                    0x30u
#define INTC_IPR12_GPIOA_SHIFT                   4
#define INTC_IPR12_GPIOA(x)                      (((uint16_t)(((uint16_t)(x))<<INTC_IPR12_GPIOA_SHIFT))&INTC_IPR12_GPIOA_MASK)
#define INTC_IPR12_COP_INT_MASK                  0xC0u
#define INTC_IPR12_COP_INT_SHIFT                 6
#define INTC_IPR12_COP_INT(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IPR12_COP_INT_SHIFT))&INTC_IPR12_COP_INT_MASK)
#define INTC_IPR12_EWM_INT_MASK                  0x300u
#define INTC_IPR12_EWM_INT_SHIFT                 8
#define INTC_IPR12_EWM_INT(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IPR12_EWM_INT_SHIFT))&INTC_IPR12_EWM_INT_MASK)
/* VBA Bit Fields */
#define INTC_VBA_VECTOR_BASE_ADDRESS_MASK        0x1FFFu
#define INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT       0
#define INTC_VBA_VECTOR_BASE_ADDRESS(x)          (((uint16_t)(((uint16_t)(x))<<INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT))&INTC_VBA_VECTOR_BASE_ADDRESS_MASK)
/* FIM0 Bit Fields */
#define INTC_FIM0_FAST_INTERRUPT_0_MASK          0x7Fu
#define INTC_FIM0_FAST_INTERRUPT_0_SHIFT         0
#define INTC_FIM0_FAST_INTERRUPT_0(x)            (((uint16_t)(((uint16_t)(x))<<INTC_FIM0_FAST_INTERRUPT_0_SHIFT))&INTC_FIM0_FAST_INTERRUPT_0_MASK)
/* FIVAL0 Bit Fields */
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK 0xFFFFu
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT 0
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW(x)   (((uint16_t)(((uint16_t)(x))<<INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT))&INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK)
/* FIVAH0 Bit Fields */
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK 0x1Fu
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT 0
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH(x)  (((uint16_t)(((uint16_t)(x))<<INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT))&INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK)
/* FIM1 Bit Fields */
#define INTC_FIM1_FAST_INTERRUPT_1_MASK          0x7Fu
#define INTC_FIM1_FAST_INTERRUPT_1_SHIFT         0
#define INTC_FIM1_FAST_INTERRUPT_1(x)            (((uint16_t)(((uint16_t)(x))<<INTC_FIM1_FAST_INTERRUPT_1_SHIFT))&INTC_FIM1_FAST_INTERRUPT_1_MASK)
/* FIVAL1 Bit Fields */
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK 0xFFFFu
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT 0
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW(x)   (((uint16_t)(((uint16_t)(x))<<INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT))&INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK)
/* FIVAH1 Bit Fields */
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK 0x1Fu
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT 0
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH(x)  (((uint16_t)(((uint16_t)(x))<<INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT))&INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK)
/* IRQP0 Bit Fields */
#define INTC_IRQP0_PENDING_MASK                  0xFFFEu
#define INTC_IRQP0_PENDING_SHIFT                 1
#define INTC_IRQP0_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP0_PENDING_SHIFT))&INTC_IRQP0_PENDING_MASK)
/* IRQP1 Bit Fields */
#define INTC_IRQP1_PENDING_MASK                  0xFFFFu
#define INTC_IRQP1_PENDING_SHIFT                 0
#define INTC_IRQP1_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP1_PENDING_SHIFT))&INTC_IRQP1_PENDING_MASK)
/* IRQP2 Bit Fields */
#define INTC_IRQP2_PENDING_MASK                  0xFFFFu
#define INTC_IRQP2_PENDING_SHIFT                 0
#define INTC_IRQP2_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP2_PENDING_SHIFT))&INTC_IRQP2_PENDING_MASK)
/* IRQP3 Bit Fields */
#define INTC_IRQP3_PENDING_MASK                  0xFFFFu
#define INTC_IRQP3_PENDING_SHIFT                 0
#define INTC_IRQP3_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP3_PENDING_SHIFT))&INTC_IRQP3_PENDING_MASK)
/* IRQP4 Bit Fields */
#define INTC_IRQP4_PENDING_MASK                  0xFFFFu
#define INTC_IRQP4_PENDING_SHIFT                 0
#define INTC_IRQP4_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP4_PENDING_SHIFT))&INTC_IRQP4_PENDING_MASK)
/* IRQP5 Bit Fields */
#define INTC_IRQP5_PENDING_MASK                  0xFFFFu
#define INTC_IRQP5_PENDING_SHIFT                 0
#define INTC_IRQP5_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP5_PENDING_SHIFT))&INTC_IRQP5_PENDING_MASK)
/* IRQP6 Bit Fields */
#define INTC_IRQP6_PENDING_MASK                  0x3FFFu
#define INTC_IRQP6_PENDING_SHIFT                 0
#define INTC_IRQP6_PENDING(x)                    (((uint16_t)(((uint16_t)(x))<<INTC_IRQP6_PENDING_SHIFT))&INTC_IRQP6_PENDING_MASK)
/* CTRL Bit Fields */
#define INTC_CTRL_INT_DIS_MASK                   0x20u
#define INTC_CTRL_INT_DIS_SHIFT                  5
#define INTC_CTRL_VAB_MASK                       0x1FC0u
#define INTC_CTRL_VAB_SHIFT                      6
#define INTC_CTRL_VAB(x)                         (((uint16_t)(((uint16_t)(x))<<INTC_CTRL_VAB_SHIFT))&INTC_CTRL_VAB_MASK)
#define INTC_CTRL_IPIC_MASK                      0x6000u
#define INTC_CTRL_IPIC_SHIFT                     13
#define INTC_CTRL_IPIC(x)                        (((uint16_t)(((uint16_t)(x))<<INTC_CTRL_IPIC_SHIFT))&INTC_CTRL_IPIC_MASK)
#define INTC_CTRL_INT_MASK                       0x8000u
#define INTC_CTRL_INT_SHIFT                      15

/*!
 * @}
 */ /* end of group INTC_Register_Masks */


/* INTC - Peripheral instance base addresses */
/** Peripheral INTC base pointer */
#define INTC_BASE_PTR                            ((INTC_MemMapPtr)0xE300u)
/** Array initializer of INTC peripheral base pointers */
#define INTC_BASE_PTRS                           { INTC_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- INTC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Register_Accessor_Macros INTC - Register accessor macros
 * @{
 */


/* INTC - Register instance definitions */
/* INTC */
#define INTC_IPR0                                INTC_IPR0_REG(INTC_BASE_PTR)
#define INTC_IPR1                                INTC_IPR1_REG(INTC_BASE_PTR)
#define INTC_IPR2                                INTC_IPR2_REG(INTC_BASE_PTR)
#define INTC_IPR3                                INTC_IPR3_REG(INTC_BASE_PTR)
#define INTC_IPR4                                INTC_IPR4_REG(INTC_BASE_PTR)
#define INTC_IPR5                                INTC_IPR5_REG(INTC_BASE_PTR)
#define INTC_IPR6                                INTC_IPR6_REG(INTC_BASE_PTR)
#define INTC_IPR8                                INTC_IPR8_REG(INTC_BASE_PTR)
#define INTC_IPR9                                INTC_IPR9_REG(INTC_BASE_PTR)
#define INTC_IPR10                               INTC_IPR10_REG(INTC_BASE_PTR)
#define INTC_IPR11                               INTC_IPR11_REG(INTC_BASE_PTR)
#define INTC_IPR12                               INTC_IPR12_REG(INTC_BASE_PTR)
#define INTC_VBA                                 INTC_VBA_REG(INTC_BASE_PTR)
#define INTC_FIM0                                INTC_FIM0_REG(INTC_BASE_PTR)
#define INTC_FIVAL0                              INTC_FIVAL0_REG(INTC_BASE_PTR)
#define INTC_FIVAH0                              INTC_FIVAH0_REG(INTC_BASE_PTR)
#define INTC_FIM1                                INTC_FIM1_REG(INTC_BASE_PTR)
#define INTC_FIVAL1                              INTC_FIVAL1_REG(INTC_BASE_PTR)
#define INTC_FIVAH1                              INTC_FIVAH1_REG(INTC_BASE_PTR)
#define INTC_IRQP0                               INTC_IRQP0_REG(INTC_BASE_PTR)
#define INTC_IRQP1                               INTC_IRQP1_REG(INTC_BASE_PTR)
#define INTC_IRQP2                               INTC_IRQP2_REG(INTC_BASE_PTR)
#define INTC_IRQP3                               INTC_IRQP3_REG(INTC_BASE_PTR)
#define INTC_IRQP4                               INTC_IRQP4_REG(INTC_BASE_PTR)
#define INTC_IRQP5                               INTC_IRQP5_REG(INTC_BASE_PTR)
#define INTC_IRQP6                               INTC_IRQP6_REG(INTC_BASE_PTR)
#define INTC_CTRL                                INTC_CTRL_REG(INTC_BASE_PTR)

/*!
 * @}
 */ /* end of group INTC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group INTC_Peripheral */


/* ----------------------------------------------------------------------------
   -- MCM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral MCM
 * @{
 */

/** MCM - Peripheral register structure */
typedef struct MCM_MemMap {
  uint8_t RESERVED_0[8];
  uint16_t PLASC;                                  /**< Crossbar switch (AXBS) slave configuration, offset: 0x8 */
  uint16_t PLAMC;                                  /**< Crossbar switch (AXBS) master configuration, offset: 0xA */
  uint32_t CPCR;                                   /**< Core control register, offset: 0xC */
  uint32_t CFADR;                                  /**< Core fault address register, offset: 0x10 */
  uint8_t CFATR;                                   /**< Core fault attributes register, offset: 0x14 */
  uint8_t CFLOC;                                   /**< Core fault location register, offset: 0x15 */
  uint8_t CFIER;                                   /**< Core fault interrupt enable register, offset: 0x16 */
  uint8_t CFISR;                                   /**< MCM interrupt status register, offset: 0x17 */
  uint32_t CFDTR;                                  /**< Core fault data register, offset: 0x18 */
  uint8_t RESERVED_1[4];
  uint32_t RPCR;                                   /**< Resource Protection Control Register, offset: 0x20 */
  uint32_t UFLASHBAR;                              /**< User Flash Base Address Register, offset: 0x24 */
  uint32_t UPRAMBAR;                               /**< User Program RAM Base Address Register, offset: 0x28 */
  uint8_t RESERVED_2[4];
  uint32_t SRPOSP;                                 /**< Resource Protection Other Stack Pointer, offset: 0x30 */
  uint32_t SRPIPC;                                 /**< Memory Protection Illegal PC, offset: 0x34 */
  uint32_t SRPMPC;                                 /**< Resource Protection Misaligned PC, offset: 0x38 */
} volatile *MCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register accessors */
#define MCM_PLASC_REG(base)                      ((base)->PLASC)
#define MCM_PLAMC_REG(base)                      ((base)->PLAMC)
#define MCM_CPCR_REG(base)                       ((base)->CPCR)
#define MCM_CFADR_REG(base)                      ((base)->CFADR)
#define MCM_CFATR_REG(base)                      ((base)->CFATR)
#define MCM_CFLOC_REG(base)                      ((base)->CFLOC)
#define MCM_CFIER_REG(base)                      ((base)->CFIER)
#define MCM_CFISR_REG(base)                      ((base)->CFISR)
#define MCM_CFDTR_REG(base)                      ((base)->CFDTR)
#define MCM_RPCR_REG(base)                       ((base)->RPCR)
#define MCM_UFLASHBAR_REG(base)                  ((base)->UFLASHBAR)
#define MCM_UPRAMBAR_REG(base)                   ((base)->UPRAMBAR)
#define MCM_SRPOSP_REG(base)                     ((base)->SRPOSP)
#define MCM_SRPIPC_REG(base)                     ((base)->SRPIPC)
#define MCM_SRPMPC_REG(base)                     ((base)->SRPMPC)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/* PLASC Bit Fields */
#define MCM_PLASC_ASC_MASK                       0xFFu
#define MCM_PLASC_ASC_SHIFT                      0
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLASC_ASC_SHIFT))&MCM_PLASC_ASC_MASK)
#define MCM_PLASC_DP64_MASK                      0x8000u
#define MCM_PLASC_DP64_SHIFT                     15
/* PLAMC Bit Fields */
#define MCM_PLAMC_AMC_MASK                       0xFFu
#define MCM_PLAMC_AMC_SHIFT                      0
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLAMC_AMC_SHIFT))&MCM_PLAMC_AMC_MASK)
/* CPCR Bit Fields */
#define MCM_CPCR_INSDIS_MASK                     0x10000u
#define MCM_CPCR_INSDIS_SHIFT                    16
#define MCM_CPCR_RCDIS_MASK                      0x20000u
#define MCM_CPCR_RCDIS_SHIFT                     17
#define MCM_CPCR_SRDIS_MASK                      0x40000u
#define MCM_CPCR_SRDIS_SHIFT                     18
#define MCM_CPCR_IBDIS_MASK                      0x80000u
#define MCM_CPCR_IBDIS_SHIFT                     19
#define MCM_CPCR_FCSDIS_MASK                     0x100000u
#define MCM_CPCR_FCSDIS_SHIFT                    20
/* CFADR Bit Fields */
#define MCM_CFADR_ADDR_MASK                      0xFFFFFFFFu
#define MCM_CFADR_ADDR_SHIFT                     0
#define MCM_CFADR_ADDR(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CFADR_ADDR_SHIFT))&MCM_CFADR_ADDR_MASK)
/* CFATR Bit Fields */
#define MCM_CFATR_TYPE_MASK                      0x1u
#define MCM_CFATR_TYPE_SHIFT                     0
#define MCM_CFATR_BUFFER_MASK                    0x4u
#define MCM_CFATR_BUFFER_SHIFT                   2
#define MCM_CFATR_SIZE_MASK                      0x70u
#define MCM_CFATR_SIZE_SHIFT                     4
#define MCM_CFATR_SIZE(x)                        (((uint8_t)(((uint8_t)(x))<<MCM_CFATR_SIZE_SHIFT))&MCM_CFATR_SIZE_MASK)
#define MCM_CFATR_DIR_MASK                       0x80u
#define MCM_CFATR_DIR_SHIFT                      7
/* CFLOC Bit Fields */
#define MCM_CFLOC_LOC_MASK                       0xC0u
#define MCM_CFLOC_LOC_SHIFT                      6
#define MCM_CFLOC_LOC(x)                         (((uint8_t)(((uint8_t)(x))<<MCM_CFLOC_LOC_SHIFT))&MCM_CFLOC_LOC_MASK)
/* CFIER Bit Fields */
#define MCM_CFIER_ECFEI_MASK                     0x80u
#define MCM_CFIER_ECFEI_SHIFT                    7
/* CFISR Bit Fields */
#define MCM_CFISR_CFEI_MASK                      0x80u
#define MCM_CFISR_CFEI_SHIFT                     7
/* CFDTR Bit Fields */
#define MCM_CFDTR_DATA_MASK                      0xFFFFFFFFu
#define MCM_CFDTR_DATA_SHIFT                     0
#define MCM_CFDTR_DATA(x)                        (((uint32_t)(((uint32_t)(x))<<MCM_CFDTR_DATA_SHIFT))&MCM_CFDTR_DATA_MASK)
/* RPCR Bit Fields */
#define MCM_RPCR_RPE_MASK                        0x1u
#define MCM_RPCR_RPE_SHIFT                       0
#define MCM_RPCR_RL_MASK                         0x2u
#define MCM_RPCR_RL_SHIFT                        1
/* UFLASHBAR Bit Fields */
#define MCM_UFLASHBAR_FBA_MASK                   0x3F000u
#define MCM_UFLASHBAR_FBA_SHIFT                  12
#define MCM_UFLASHBAR_FBA(x)                     (((uint32_t)(((uint32_t)(x))<<MCM_UFLASHBAR_FBA_SHIFT))&MCM_UFLASHBAR_FBA_MASK)
/* UPRAMBAR Bit Fields */
#define MCM_UPRAMBAR_RBA_MASK                    0x7F00u
#define MCM_UPRAMBAR_RBA_SHIFT                   8
#define MCM_UPRAMBAR_RBA(x)                      (((uint32_t)(((uint32_t)(x))<<MCM_UPRAMBAR_RBA_SHIFT))&MCM_UPRAMBAR_RBA_MASK)
/* SRPOSP Bit Fields */
#define MCM_SRPOSP_SRPOSP_MASK                   0xFFFFFFu
#define MCM_SRPOSP_SRPOSP_SHIFT                  0
#define MCM_SRPOSP_SRPOSP(x)                     (((uint32_t)(((uint32_t)(x))<<MCM_SRPOSP_SRPOSP_SHIFT))&MCM_SRPOSP_SRPOSP_MASK)
/* SRPIPC Bit Fields */
#define MCM_SRPIPC_SRPIFPC_MASK                  0x1FFFFFu
#define MCM_SRPIPC_SRPIFPC_SHIFT                 0
#define MCM_SRPIPC_SRPIFPC(x)                    (((uint32_t)(((uint32_t)(x))<<MCM_SRPIPC_SRPIFPC_SHIFT))&MCM_SRPIPC_SRPIFPC_MASK)
#define MCM_SRPIPC_SRPIFV_MASK                   0x80000000u
#define MCM_SRPIPC_SRPIFV_SHIFT                  31
/* SRPMPC Bit Fields */
#define MCM_SRPMPC_SRPMFPC_MASK                  0x1FFFFFu
#define MCM_SRPMPC_SRPMFPC_SHIFT                 0
#define MCM_SRPMPC_SRPMFPC(x)                    (((uint32_t)(((uint32_t)(x))<<MCM_SRPMPC_SRPMFPC_SHIFT))&MCM_SRPMPC_SRPMFPC_MASK)
#define MCM_SRPMPC_SRPMFV_MASK                   0x80000000u
#define MCM_SRPMPC_SRPMFV_SHIFT                  31

/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base pointer */
#define MCM_BASE_PTR                             ((MCM_MemMapPtr)0xC000u)
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register instance definitions */
/* MCM */
#define MCM_PLASC                                MCM_PLASC_REG(MCM_BASE_PTR)
#define MCM_PLAMC                                MCM_PLAMC_REG(MCM_BASE_PTR)
#define MCM_CPCR                                 MCM_CPCR_REG(MCM_BASE_PTR)
#define MCM_CFADR                                MCM_CFADR_REG(MCM_BASE_PTR)
#define MCM_CFATR                                MCM_CFATR_REG(MCM_BASE_PTR)
#define MCM_CFLOC                                MCM_CFLOC_REG(MCM_BASE_PTR)
#define MCM_CFIER                                MCM_CFIER_REG(MCM_BASE_PTR)
#define MCM_CFISR                                MCM_CFISR_REG(MCM_BASE_PTR)
#define MCM_CFDTR                                MCM_CFDTR_REG(MCM_BASE_PTR)
#define MCM_RPCR                                 MCM_RPCR_REG(MCM_BASE_PTR)
#define MCM_UFLASHBAR                            MCM_UFLASHBAR_REG(MCM_BASE_PTR)
#define MCM_UPRAMBAR                             MCM_UPRAMBAR_REG(MCM_BASE_PTR)
#define MCM_SRPOSP                               MCM_SRPOSP_REG(MCM_BASE_PTR)
#define MCM_SRPIPC                               MCM_SRPIPC_REG(MCM_BASE_PTR)
#define MCM_SRPMPC                               MCM_SRPMPC_REG(MCM_BASE_PTR)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MCM_Peripheral */


/* ----------------------------------------------------------------------------
   -- OCCS
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Peripheral OCCS
 * @{
 */

/** OCCS - Peripheral register structure */
typedef struct OCCS_MemMap {
  uint16_t CTRL;                                   /**< PLL Control Register, offset: 0x0 */
  uint16_t DIVBY;                                  /**< PLL Divide-By Register, offset: 0x1 */
  uint16_t STAT;                                   /**< OCCS Status Register, offset: 0x2 */
  uint16_t RESERVED_0[1];
  uint16_t OSCTL1;                                 /**< Oscillator Control Register 1, offset: 0x4 */
  uint16_t OSCTL2;                                 /**< Oscillator Control Register 2, offset: 0x5 */
  uint16_t CLKCHKR;                                /**< External Clock Check Reference, offset: 0x6 */
  uint16_t CLKCHKT;                                /**< External Clock Check Target, offset: 0x7 */
  uint16_t PROT;                                   /**< Protection Register, offset: 0x8 */
} volatile *OCCS_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- OCCS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Register_Accessor_Macros OCCS - Register accessor macros
 * @{
 */


/* OCCS - Register accessors */
#define OCCS_CTRL_REG(base)                      ((base)->CTRL)
#define OCCS_DIVBY_REG(base)                     ((base)->DIVBY)
#define OCCS_STAT_REG(base)                      ((base)->STAT)
#define OCCS_OSCTL1_REG(base)                    ((base)->OSCTL1)
#define OCCS_OSCTL2_REG(base)                    ((base)->OSCTL2)
#define OCCS_CLKCHKR_REG(base)                   ((base)->CLKCHKR)
#define OCCS_CLKCHKT_REG(base)                   ((base)->CLKCHKT)
#define OCCS_PROT_REG(base)                      ((base)->PROT)

/*!
 * @}
 */ /* end of group OCCS_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- OCCS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Register_Masks OCCS Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define OCCS_CTRL_ZSRC_MASK                      0x1u
#define OCCS_CTRL_ZSRC_SHIFT                     0
#define OCCS_CTRL_PRECS_MASK                     0xCu
#define OCCS_CTRL_PRECS_SHIFT                    2
#define OCCS_CTRL_PRECS(x)                       (((uint16_t)(((uint16_t)(x))<<OCCS_CTRL_PRECS_SHIFT))&OCCS_CTRL_PRECS_MASK)
#define OCCS_CTRL_PLLPD_MASK                     0x10u
#define OCCS_CTRL_PLLPD_SHIFT                    4
#define OCCS_CTRL_LCKON_MASK                     0x80u
#define OCCS_CTRL_LCKON_SHIFT                    7
#define OCCS_CTRL_LOCIE_MASK                     0x800u
#define OCCS_CTRL_LOCIE_SHIFT                    11
#define OCCS_CTRL_PLLIE0_MASK                    0x3000u
#define OCCS_CTRL_PLLIE0_SHIFT                   12
#define OCCS_CTRL_PLLIE0(x)                      (((uint16_t)(((uint16_t)(x))<<OCCS_CTRL_PLLIE0_SHIFT))&OCCS_CTRL_PLLIE0_MASK)
#define OCCS_CTRL_PLLIE1_MASK                    0xC000u
#define OCCS_CTRL_PLLIE1_SHIFT                   14
#define OCCS_CTRL_PLLIE1(x)                      (((uint16_t)(((uint16_t)(x))<<OCCS_CTRL_PLLIE1_SHIFT))&OCCS_CTRL_PLLIE1_MASK)
/* DIVBY Bit Fields */
#define OCCS_DIVBY_PLLDB_MASK                    0x3Fu
#define OCCS_DIVBY_PLLDB_SHIFT                   0
#define OCCS_DIVBY_PLLDB(x)                      (((uint16_t)(((uint16_t)(x))<<OCCS_DIVBY_PLLDB_SHIFT))&OCCS_DIVBY_PLLDB_MASK)
#define OCCS_DIVBY_PWM_DIV2_MASK                 0x40u
#define OCCS_DIVBY_PWM_DIV2_SHIFT                6
#define OCCS_DIVBY_COD_MASK                      0xF00u
#define OCCS_DIVBY_COD_SHIFT                     8
#define OCCS_DIVBY_COD(x)                        (((uint16_t)(((uint16_t)(x))<<OCCS_DIVBY_COD_SHIFT))&OCCS_DIVBY_COD_MASK)
#define OCCS_DIVBY_LORTP_MASK                    0xF000u
#define OCCS_DIVBY_LORTP_SHIFT                   12
#define OCCS_DIVBY_LORTP(x)                      (((uint16_t)(((uint16_t)(x))<<OCCS_DIVBY_LORTP_SHIFT))&OCCS_DIVBY_LORTP_MASK)
/* STAT Bit Fields */
#define OCCS_STAT_ZSRCS_MASK                     0x3u
#define OCCS_STAT_ZSRCS_SHIFT                    0
#define OCCS_STAT_ZSRCS(x)                       (((uint16_t)(((uint16_t)(x))<<OCCS_STAT_ZSRCS_SHIFT))&OCCS_STAT_ZSRCS_MASK)
#define OCCS_STAT_PLLPDN_MASK                    0x10u
#define OCCS_STAT_PLLPDN_SHIFT                   4
#define OCCS_STAT_LCK0_MASK                      0x20u
#define OCCS_STAT_LCK0_SHIFT                     5
#define OCCS_STAT_LCK1_MASK                      0x40u
#define OCCS_STAT_LCK1_SHIFT                     6
#define OCCS_STAT_OSC_OK_MASK                    0x80u
#define OCCS_STAT_OSC_OK_SHIFT                   7
#define OCCS_STAT_MON_FAILURE_MASK               0x100u
#define OCCS_STAT_MON_FAILURE_SHIFT              8
#define OCCS_STAT_LOCI_MASK                      0x2000u
#define OCCS_STAT_LOCI_SHIFT                     13
#define OCCS_STAT_LOLI0_MASK                     0x4000u
#define OCCS_STAT_LOLI0_SHIFT                    14
#define OCCS_STAT_LOLI1_MASK                     0x8000u
#define OCCS_STAT_LOLI1_SHIFT                    15
/* OSCTL1 Bit Fields */
#define OCCS_OSCTL1_FREQ_TRIM8M_MASK             0x3FFu
#define OCCS_OSCTL1_FREQ_TRIM8M_SHIFT            0
#define OCCS_OSCTL1_FREQ_TRIM8M(x)               (((uint16_t)(((uint16_t)(x))<<OCCS_OSCTL1_FREQ_TRIM8M_SHIFT))&OCCS_OSCTL1_FREQ_TRIM8M_MASK)
#define OCCS_OSCTL1_EXT_SEL_MASK                 0x400u
#define OCCS_OSCTL1_EXT_SEL_SHIFT                10
#define OCCS_OSCTL1_CLK_MODE_MASK                0x1000u
#define OCCS_OSCTL1_CLK_MODE_SHIFT               12
#define OCCS_OSCTL1_COHL_MASK                    0x2000u
#define OCCS_OSCTL1_COHL_SHIFT                   13
#define OCCS_OSCTL1_ROSB_MASK                    0x4000u
#define OCCS_OSCTL1_ROSB_SHIFT                   14
#define OCCS_OSCTL1_ROPD_MASK                    0x8000u
#define OCCS_OSCTL1_ROPD_SHIFT                   15
/* OSCTL2 Bit Fields */
#define OCCS_OSCTL2_FREQ_TRIM200K_MASK           0x1FFu
#define OCCS_OSCTL2_FREQ_TRIM200K_SHIFT          0
#define OCCS_OSCTL2_FREQ_TRIM200K(x)             (((uint16_t)(((uint16_t)(x))<<OCCS_OSCTL2_FREQ_TRIM200K_SHIFT))&OCCS_OSCTL2_FREQ_TRIM200K_MASK)
#define OCCS_OSCTL2_MON_ENABLE_MASK              0x200u
#define OCCS_OSCTL2_MON_ENABLE_SHIFT             9
#define OCCS_OSCTL2_TEMP_TRIM8M_MASK             0x3C00u
#define OCCS_OSCTL2_TEMP_TRIM8M_SHIFT            10
#define OCCS_OSCTL2_TEMP_TRIM8M(x)               (((uint16_t)(((uint16_t)(x))<<OCCS_OSCTL2_TEMP_TRIM8M_SHIFT))&OCCS_OSCTL2_TEMP_TRIM8M_MASK)
#define OCCS_OSCTL2_COPD_MASK                    0x4000u
#define OCCS_OSCTL2_COPD_SHIFT                   14
#define OCCS_OSCTL2_ROPD200K_MASK                0x8000u
#define OCCS_OSCTL2_ROPD200K_SHIFT               15
/* CLKCHKR Bit Fields */
#define OCCS_CLKCHKR_REF_CNT_MASK                0x7Fu
#define OCCS_CLKCHKR_REF_CNT_SHIFT               0
#define OCCS_CLKCHKR_REF_CNT(x)                  (((uint16_t)(((uint16_t)(x))<<OCCS_CLKCHKR_REF_CNT_SHIFT))&OCCS_CLKCHKR_REF_CNT_MASK)
#define OCCS_CLKCHKR_CHK_ENA_MASK                0x8000u
#define OCCS_CLKCHKR_CHK_ENA_SHIFT               15
/* CLKCHKT Bit Fields */
#define OCCS_CLKCHKT_TARGET_CNT_MASK             0x7FFu
#define OCCS_CLKCHKT_TARGET_CNT_SHIFT            0
#define OCCS_CLKCHKT_TARGET_CNT(x)               (((uint16_t)(((uint16_t)(x))<<OCCS_CLKCHKT_TARGET_CNT_SHIFT))&OCCS_CLKCHKT_TARGET_CNT_MASK)
/* PROT Bit Fields */
#define OCCS_PROT_PLLEP_MASK                     0x3u
#define OCCS_PROT_PLLEP_SHIFT                    0
#define OCCS_PROT_PLLEP(x)                       (((uint16_t)(((uint16_t)(x))<<OCCS_PROT_PLLEP_SHIFT))&OCCS_PROT_PLLEP_MASK)
#define OCCS_PROT_OSCEP_MASK                     0xCu
#define OCCS_PROT_OSCEP_SHIFT                    2
#define OCCS_PROT_OSCEP(x)                       (((uint16_t)(((uint16_t)(x))<<OCCS_PROT_OSCEP_SHIFT))&OCCS_PROT_OSCEP_MASK)
#define OCCS_PROT_FRQEP_MASK                     0x30u
#define OCCS_PROT_FRQEP_SHIFT                    4
#define OCCS_PROT_FRQEP(x)                       (((uint16_t)(((uint16_t)(x))<<OCCS_PROT_FRQEP_SHIFT))&OCCS_PROT_FRQEP_MASK)

/*!
 * @}
 */ /* end of group OCCS_Register_Masks */


/* OCCS - Peripheral instance base addresses */
/** Peripheral OCCS base pointer */
#define OCCS_BASE_PTR                            ((OCCS_MemMapPtr)0xE2B0u)
/** Array initializer of OCCS peripheral base pointers */
#define OCCS_BASE_PTRS                           { OCCS_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- OCCS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Register_Accessor_Macros OCCS - Register accessor macros
 * @{
 */


/* OCCS - Register instance definitions */
/* OCCS */
#define OCCS_CTRL                                OCCS_CTRL_REG(OCCS_BASE_PTR)
#define OCCS_DIVBY                               OCCS_DIVBY_REG(OCCS_BASE_PTR)
#define OCCS_STAT                                OCCS_STAT_REG(OCCS_BASE_PTR)
#define OCCS_OSCTL1                              OCCS_OSCTL1_REG(OCCS_BASE_PTR)
#define OCCS_OSCTL2                              OCCS_OSCTL2_REG(OCCS_BASE_PTR)
#define OCCS_CLKCHKR                             OCCS_CLKCHKR_REG(OCCS_BASE_PTR)
#define OCCS_CLKCHKT                             OCCS_CLKCHKT_REG(OCCS_BASE_PTR)
#define OCCS_PROT                                OCCS_PROT_REG(OCCS_BASE_PTR)

/*!
 * @}
 */ /* end of group OCCS_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group OCCS_Peripheral */


/* ----------------------------------------------------------------------------
   -- PIT
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral PIT
 * @{
 */

/** PIT - Peripheral register structure */
typedef struct PIT_MemMap {
  uint16_t CTRL;                                   /**< PIT Control Register, offset: 0x0 */
  uint16_t MOD;                                    /**< PIT Modulo Register, offset: 0x1 */
  uint16_t CNTR;                                   /**< PIT Counter Register, offset: 0x2 */
} volatile *PIT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register accessors */
#define PIT_CTRL_REG(base)                       ((base)->CTRL)
#define PIT_MOD_REG(base)                        ((base)->MOD)
#define PIT_CNTR_REG(base)                       ((base)->CNTR)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define PIT_CTRL_CNT_EN_MASK                     0x1u
#define PIT_CTRL_CNT_EN_SHIFT                    0
#define PIT_CTRL_PRIE_MASK                       0x2u
#define PIT_CTRL_PRIE_SHIFT                      1
#define PIT_CTRL_PRF_MASK                        0x4u
#define PIT_CTRL_PRF_SHIFT                       2
#define PIT_CTRL_PRESCALER_MASK                  0x78u
#define PIT_CTRL_PRESCALER_SHIFT                 3
#define PIT_CTRL_PRESCALER(x)                    (((uint16_t)(((uint16_t)(x))<<PIT_CTRL_PRESCALER_SHIFT))&PIT_CTRL_PRESCALER_MASK)
#define PIT_CTRL_CLKSEL_MASK                     0x300u
#define PIT_CTRL_CLKSEL_SHIFT                    8
#define PIT_CTRL_CLKSEL(x)                       (((uint16_t)(((uint16_t)(x))<<PIT_CTRL_CLKSEL_SHIFT))&PIT_CTRL_CLKSEL_MASK)
#define PIT_CTRL_SLAVE_MASK                      0x8000u
#define PIT_CTRL_SLAVE_SHIFT                     15
/* MOD Bit Fields */
#define PIT_MOD_MODULO_VALUE_MASK                0xFFFFu
#define PIT_MOD_MODULO_VALUE_SHIFT               0
#define PIT_MOD_MODULO_VALUE(x)                  (((uint16_t)(((uint16_t)(x))<<PIT_MOD_MODULO_VALUE_SHIFT))&PIT_MOD_MODULO_VALUE_MASK)
/* CNTR Bit Fields */
#define PIT_CNTR_COUNTER_VALUE_MASK              0xFFFFu
#define PIT_CNTR_COUNTER_VALUE_SHIFT             0
#define PIT_CNTR_COUNTER_VALUE(x)                (((uint16_t)(((uint16_t)(x))<<PIT_CNTR_COUNTER_VALUE_SHIFT))&PIT_CNTR_COUNTER_VALUE_MASK)

/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT0 base pointer */
#define PIT0_BASE_PTR                            ((PIT_MemMapPtr)0xE100u)
/** Peripheral PIT1 base pointer */
#define PIT1_BASE_PTR                            ((PIT_MemMapPtr)0xE110u)
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT0_BASE_PTR, PIT1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register instance definitions */
/* PIT0 */
#define PIT0_CTRL                                PIT_CTRL_REG(PIT0_BASE_PTR)
#define PIT0_MOD                                 PIT_MOD_REG(PIT0_BASE_PTR)
#define PIT0_CNTR                                PIT_CNTR_REG(PIT0_BASE_PTR)
/* PIT1 */
#define PIT1_CTRL                                PIT_CTRL_REG(PIT1_BASE_PTR)
#define PIT1_MOD                                 PIT_MOD_REG(PIT1_BASE_PTR)
#define PIT1_CNTR                                PIT_CNTR_REG(PIT1_BASE_PTR)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PIT_Peripheral */


/* ----------------------------------------------------------------------------
   -- PMC
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral PMC
 * @{
 */

/** PMC - Peripheral register structure */
typedef struct PMC_MemMap {
  uint16_t CTRL;                                   /**< Control Register, offset: 0x0 */
  uint16_t STS;                                    /**< Status Register, offset: 0x1 */
} volatile *PMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register accessors */
#define PMC_CTRL_REG(base)                       ((base)->CTRL)
#define PMC_STS_REG(base)                        ((base)->STS)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define PMC_CTRL_LV22IE_MASK                     0x1u
#define PMC_CTRL_LV22IE_SHIFT                    0
#define PMC_CTRL_LV27IE_MASK                     0x2u
#define PMC_CTRL_LV27IE_SHIFT                    1
#define PMC_CTRL_HV22IE_MASK                     0x4u
#define PMC_CTRL_HV22IE_SHIFT                    2
#define PMC_CTRL_HV27IE_MASK                     0x8u
#define PMC_CTRL_HV27IE_SHIFT                    3
#define PMC_CTRL_VRBEN_MASK                      0x80u
#define PMC_CTRL_VRBEN_SHIFT                     7
#define PMC_CTRL_TRIM_MASK                       0xF000u
#define PMC_CTRL_TRIM_SHIFT                      12
#define PMC_CTRL_TRIM(x)                         (((uint16_t)(((uint16_t)(x))<<PMC_CTRL_TRIM_SHIFT))&PMC_CTRL_TRIM_MASK)
/* STS Bit Fields */
#define PMC_STS_LV22F_MASK                       0x1u
#define PMC_STS_LV22F_SHIFT                      0
#define PMC_STS_LV27F_MASK                       0x2u
#define PMC_STS_LV27F_SHIFT                      1
#define PMC_STS_SLV22F_MASK                      0x4u
#define PMC_STS_SLV22F_SHIFT                     2
#define PMC_STS_SLV27F_MASK                      0x8u
#define PMC_STS_SLV27F_SHIFT                     3
#define PMC_STS_LVI_MASK                         0x10u
#define PMC_STS_LVI_SHIFT                        4
#define PMC_STS_SR27_MASK                        0x20u
#define PMC_STS_SR27_SHIFT                       5

/*!
 * @}
 */ /* end of group PMC_Register_Masks */


/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base pointer */
#define PMC_BASE_PTR                             ((PMC_MemMapPtr)0xE2A0u)
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register instance definitions */
/* PMC */
#define PMC_CTRL                                 PMC_CTRL_REG(PMC_BASE_PTR)
#define PMC_STS                                  PMC_STS_REG(PMC_BASE_PTR)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PMC_Peripheral */


/* ----------------------------------------------------------------------------
   -- PWM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Peripheral PWM
 * @{
 */

/** PWM - Peripheral register structure */
typedef struct PWM_MemMap {
  struct {                                         /* offset: 0x0, array step: 0x30 */
    uint16_t CNT;                                    /**< Counter Register, array offset: 0x0, array step: 0x30 */
    uint16_t INIT;                                   /**< Initial Count Register, array offset: 0x1, array step: 0x30 */
    uint16_t CTRL2;                                  /**< Control 2 Register, array offset: 0x2, array step: 0x30 */
    uint16_t CTRL;                                   /**< Control Register, array offset: 0x3, array step: 0x30 */
    uint16_t RESERVED_0[1];
    uint16_t VAL0;                                   /**< Value Register 0, array offset: 0x5, array step: 0x30 */
    uint16_t FRACVAL1;                               /**< Fractional Value Register 1, array offset: 0x6, array step: 0x30 */
    uint16_t VAL1;                                   /**< Value Register 1, array offset: 0x7, array step: 0x30 */
    uint16_t FRACVAL2;                               /**< Fractional Value Register 2, array offset: 0x8, array step: 0x30 */
    uint16_t VAL2;                                   /**< Value Register 2, array offset: 0x9, array step: 0x30 */
    uint16_t FRACVAL3;                               /**< Fractional Value Register 3, array offset: 0xA, array step: 0x30 */
    uint16_t VAL3;                                   /**< Value Register 3, array offset: 0xB, array step: 0x30 */
    uint16_t FRACVAL4;                               /**< Fractional Value Register 4, array offset: 0xC, array step: 0x30 */
    uint16_t VAL4;                                   /**< Value Register 4, array offset: 0xD, array step: 0x30 */
    uint16_t FRACVAL5;                               /**< Fractional Value Register 5, array offset: 0xE, array step: 0x30 */
    uint16_t VAL5;                                   /**< Value Register 5, array offset: 0xF, array step: 0x30 */
    uint16_t FRCTRL;                                 /**< Fractional Control Register, array offset: 0x10, array step: 0x30 */
    uint16_t OCTRL;                                  /**< Output Control Register, array offset: 0x11, array step: 0x30 */
    uint16_t STS;                                    /**< Status Register, array offset: 0x12, array step: 0x30 */
    uint16_t INTEN;                                  /**< Interrupt Enable Register, array offset: 0x13, array step: 0x30 */
    uint16_t DMAEN;                                  /**< DMA Enable Register, array offset: 0x14, array step: 0x30 */
    uint16_t TCTRL;                                  /**< Output Trigger Control Register, array offset: 0x15, array step: 0x30 */
    uint16_t DISMAP[2];                              /**< Fault Disable Mapping Register 0..Fault Disable Mapping Register 1, array offset: 0x16, array step: index*0x30, index2*0x1 */
    uint16_t DTCNT0;                                 /**< Deadtime Count Register 0, array offset: 0x18, array step: 0x30 */
    uint16_t DTCNT1;                                 /**< Deadtime Count Register 1, array offset: 0x19, array step: 0x30 */
    uint16_t CAPTCTRLA;                              /**< Capture Control A Register, array offset: 0x1A, array step: 0x30 */
    uint16_t CAPTCOMPA;                              /**< Capture Compare A Register, array offset: 0x1B, array step: 0x30 */
    uint16_t CAPTCTRLB;                              /**< Capture Control B Register, array offset: 0x1C, array step: 0x30 */
    uint16_t CAPTCOMPB;                              /**< Capture Compare B Register, array offset: 0x1D, array step: 0x30 */
    uint16_t CAPTCTRLX;                              /**< Capture Control X Register, array offset: 0x1E, array step: 0x30 */
    uint16_t CAPTCOMPX;                              /**< Capture Compare X Register, array offset: 0x1F, array step: 0x30 */
    uint16_t CVAL0;                                  /**< Capture Value 0 Register, array offset: 0x20, array step: 0x30 */
    uint16_t CVAL0CYC;                               /**< Capture Value 0 Cycle Register, array offset: 0x21, array step: 0x30 */
    uint16_t CVAL1;                                  /**< Capture Value 1 Register, array offset: 0x22, array step: 0x30 */
    uint16_t CVAL1CYC;                               /**< Capture Value 1 Cycle Register, array offset: 0x23, array step: 0x30 */
    uint16_t CVAL2;                                  /**< Capture Value 2 Register, array offset: 0x24, array step: 0x30 */
    uint16_t CVAL2CYC;                               /**< Capture Value 2 Cycle Register, array offset: 0x25, array step: 0x30 */
    uint16_t CVAL3;                                  /**< Capture Value 3 Register, array offset: 0x26, array step: 0x30 */
    uint16_t CVAL3CYC;                               /**< Capture Value 3 Cycle Register, array offset: 0x27, array step: 0x30 */
    uint16_t CVAL4;                                  /**< Capture Value 4 Register, array offset: 0x28, array step: 0x30 */
    uint16_t CVAL4CYC;                               /**< Capture Value 4 Cycle Register, array offset: 0x29, array step: 0x30 */
    uint16_t CVAL5;                                  /**< Capture Value 5 Register, array offset: 0x2A, array step: 0x30 */
    uint16_t CVAL5CYC;                               /**< Capture Value 5 Cycle Register, array offset: 0x2B, array step: 0x30 */
    uint16_t RESERVED_1[4];
  } SM[4];
  uint16_t OUTEN;                                  /**< Output Enable Register, offset: 0xC0 */
  uint16_t MASK;                                   /**< Mask Register, offset: 0xC1 */
  uint16_t SWCOUT;                                 /**< Software Controlled Output Register, offset: 0xC2 */
  uint16_t DTSRCSEL;                               /**< PWM Source Select Register, offset: 0xC3 */
  uint16_t MCTRL;                                  /**< Master Control Register, offset: 0xC4 */
  uint16_t MCTRL2;                                 /**< Master Control 2 Register, offset: 0xC5 */
  struct {                                         /* offset: 0xC6, array step: 0x4 */
    uint16_t FCTRL;                                  /**< Fault Control Register, array offset: 0xC6, array step: 0x4 */
    uint16_t FSTS;                                   /**< Fault Status Register, array offset: 0xC7, array step: 0x4 */
    uint16_t FFILT;                                  /**< Fault Filter Register, array offset: 0xC8, array step: 0x4 */
    uint16_t FTST;                                   /**< Fault Test Register, array offset: 0xC9, array step: 0x4 */
  } FAULT[2];
} volatile *PWM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Accessor_Macros PWM - Register accessor macros
 * @{
 */


/* PWM - Register accessors */
#define PWM_CNT_REG(base,index)                  ((base)->SM[index].CNT)
#define PWM_INIT_REG(base,index)                 ((base)->SM[index].INIT)
#define PWM_CTRL2_REG(base,index)                ((base)->SM[index].CTRL2)
#define PWM_CTRL_REG(base,index)                 ((base)->SM[index].CTRL)
#define PWM_VAL0_REG(base,index)                 ((base)->SM[index].VAL0)
#define PWM_FRACVAL1_REG(base,index)             ((base)->SM[index].FRACVAL1)
#define PWM_VAL1_REG(base,index)                 ((base)->SM[index].VAL1)
#define PWM_FRACVAL2_REG(base,index)             ((base)->SM[index].FRACVAL2)
#define PWM_VAL2_REG(base,index)                 ((base)->SM[index].VAL2)
#define PWM_FRACVAL3_REG(base,index)             ((base)->SM[index].FRACVAL3)
#define PWM_VAL3_REG(base,index)                 ((base)->SM[index].VAL3)
#define PWM_FRACVAL4_REG(base,index)             ((base)->SM[index].FRACVAL4)
#define PWM_VAL4_REG(base,index)                 ((base)->SM[index].VAL4)
#define PWM_FRACVAL5_REG(base,index)             ((base)->SM[index].FRACVAL5)
#define PWM_VAL5_REG(base,index)                 ((base)->SM[index].VAL5)
#define PWM_FRCTRL_REG(base,index)               ((base)->SM[index].FRCTRL)
#define PWM_OCTRL_REG(base,index)                ((base)->SM[index].OCTRL)
#define PWM_STS_REG(base,index)                  ((base)->SM[index].STS)
#define PWM_INTEN_REG(base,index)                ((base)->SM[index].INTEN)
#define PWM_DMAEN_REG(base,index)                ((base)->SM[index].DMAEN)
#define PWM_TCTRL_REG(base,index)                ((base)->SM[index].TCTRL)
#define PWM_DISMAP_REG(base,index,index2)        ((base)->SM[index].DISMAP[index2])
#define PWM_DTCNT0_REG(base,index)               ((base)->SM[index].DTCNT0)
#define PWM_DTCNT1_REG(base,index)               ((base)->SM[index].DTCNT1)
#define PWM_CAPTCTRLA_REG(base,index)            ((base)->SM[index].CAPTCTRLA)
#define PWM_CAPTCOMPA_REG(base,index)            ((base)->SM[index].CAPTCOMPA)
#define PWM_CAPTCTRLB_REG(base,index)            ((base)->SM[index].CAPTCTRLB)
#define PWM_CAPTCOMPB_REG(base,index)            ((base)->SM[index].CAPTCOMPB)
#define PWM_CAPTCTRLX_REG(base,index)            ((base)->SM[index].CAPTCTRLX)
#define PWM_CAPTCOMPX_REG(base,index)            ((base)->SM[index].CAPTCOMPX)
#define PWM_CVAL0_REG(base,index)                ((base)->SM[index].CVAL0)
#define PWM_CVAL0CYC_REG(base,index)             ((base)->SM[index].CVAL0CYC)
#define PWM_CVAL1_REG(base,index)                ((base)->SM[index].CVAL1)
#define PWM_CVAL1CYC_REG(base,index)             ((base)->SM[index].CVAL1CYC)
#define PWM_CVAL2_REG(base,index)                ((base)->SM[index].CVAL2)
#define PWM_CVAL2CYC_REG(base,index)             ((base)->SM[index].CVAL2CYC)
#define PWM_CVAL3_REG(base,index)                ((base)->SM[index].CVAL3)
#define PWM_CVAL3CYC_REG(base,index)             ((base)->SM[index].CVAL3CYC)
#define PWM_CVAL4_REG(base,index)                ((base)->SM[index].CVAL4)
#define PWM_CVAL4CYC_REG(base,index)             ((base)->SM[index].CVAL4CYC)
#define PWM_CVAL5_REG(base,index)                ((base)->SM[index].CVAL5)
#define PWM_CVAL5CYC_REG(base,index)             ((base)->SM[index].CVAL5CYC)
#define PWM_OUTEN_REG(base)                      ((base)->OUTEN)
#define PWM_MASK_REG(base)                       ((base)->MASK)
#define PWM_SWCOUT_REG(base)                     ((base)->SWCOUT)
#define PWM_DTSRCSEL_REG(base)                   ((base)->DTSRCSEL)
#define PWM_MCTRL_REG(base)                      ((base)->MCTRL)
#define PWM_MCTRL2_REG(base)                     ((base)->MCTRL2)
#define PWM_FCTRL_REG(base,index)                ((base)->FAULT[index].FCTRL)
#define PWM_FSTS_REG(base,index)                 ((base)->FAULT[index].FSTS)
#define PWM_FFILT_REG(base,index)                ((base)->FAULT[index].FFILT)
#define PWM_FTST_REG(base,index)                 ((base)->FAULT[index].FTST)

/*!
 * @}
 */ /* end of group PWM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Masks PWM Register Masks
 * @{
 */

/* CNT Bit Fields */
#define PWM_CNT_CNT_MASK                         0xFFFFu
#define PWM_CNT_CNT_SHIFT                        0
#define PWM_CNT_CNT(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_CNT_CNT_SHIFT))&PWM_CNT_CNT_MASK)
/* INIT Bit Fields */
#define PWM_INIT_INIT_MASK                       0xFFFFu
#define PWM_INIT_INIT_SHIFT                      0
#define PWM_INIT_INIT(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_INIT_INIT_SHIFT))&PWM_INIT_INIT_MASK)
/* CTRL2 Bit Fields */
#define PWM_CTRL2_CLK_SEL_MASK                   0x3u
#define PWM_CTRL2_CLK_SEL_SHIFT                  0
#define PWM_CTRL2_CLK_SEL(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_CLK_SEL_SHIFT))&PWM_CTRL2_CLK_SEL_MASK)
#define PWM_CTRL2_RELOAD_SEL_MASK                0x4u
#define PWM_CTRL2_RELOAD_SEL_SHIFT               2
#define PWM_CTRL2_FORCE_SEL_MASK                 0x38u
#define PWM_CTRL2_FORCE_SEL_SHIFT                3
#define PWM_CTRL2_FORCE_SEL(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_FORCE_SEL_SHIFT))&PWM_CTRL2_FORCE_SEL_MASK)
#define PWM_CTRL2_FORCE_MASK                     0x40u
#define PWM_CTRL2_FORCE_SHIFT                    6
#define PWM_CTRL2_FRCEN_MASK                     0x80u
#define PWM_CTRL2_FRCEN_SHIFT                    7
#define PWM_CTRL2_INIT_SEL_MASK                  0x300u
#define PWM_CTRL2_INIT_SEL_SHIFT                 8
#define PWM_CTRL2_INIT_SEL(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CTRL2_INIT_SEL_SHIFT))&PWM_CTRL2_INIT_SEL_MASK)
#define PWM_CTRL2_PWMX_INIT_MASK                 0x400u
#define PWM_CTRL2_PWMX_INIT_SHIFT                10
#define PWM_CTRL2_PWM45_INIT_MASK                0x800u
#define PWM_CTRL2_PWM45_INIT_SHIFT               11
#define PWM_CTRL2_PWM23_INIT_MASK                0x1000u
#define PWM_CTRL2_PWM23_INIT_SHIFT               12
#define PWM_CTRL2_INDEP_MASK                     0x2000u
#define PWM_CTRL2_INDEP_SHIFT                    13
#define PWM_CTRL2_WAITEN_MASK                    0x4000u
#define PWM_CTRL2_WAITEN_SHIFT                   14
#define PWM_CTRL2_DBGEN_MASK                     0x8000u
#define PWM_CTRL2_DBGEN_SHIFT                    15
/* CTRL Bit Fields */
#define PWM_CTRL_DBLEN_MASK                      0x1u
#define PWM_CTRL_DBLEN_SHIFT                     0
#define PWM_CTRL_DBLX_MASK                       0x2u
#define PWM_CTRL_DBLX_SHIFT                      1
#define PWM_CTRL_LDMOD_MASK                      0x4u
#define PWM_CTRL_LDMOD_SHIFT                     2
#define PWM_CTRL_SPLIT_MASK                      0x8u
#define PWM_CTRL_SPLIT_SHIFT                     3
#define PWM_CTRL_PRSC_MASK                       0x70u
#define PWM_CTRL_PRSC_SHIFT                      4
#define PWM_CTRL_PRSC(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_PRSC_SHIFT))&PWM_CTRL_PRSC_MASK)
#define PWM_CTRL_DT_MASK                         0x300u
#define PWM_CTRL_DT_SHIFT                        8
#define PWM_CTRL_DT(x)                           (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_DT_SHIFT))&PWM_CTRL_DT_MASK)
#define PWM_CTRL_FULL_MASK                       0x400u
#define PWM_CTRL_FULL_SHIFT                      10
#define PWM_CTRL_HALF_MASK                       0x800u
#define PWM_CTRL_HALF_SHIFT                      11
#define PWM_CTRL_LDFQ_MASK                       0xF000u
#define PWM_CTRL_LDFQ_SHIFT                      12
#define PWM_CTRL_LDFQ(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_CTRL_LDFQ_SHIFT))&PWM_CTRL_LDFQ_MASK)
/* VAL0 Bit Fields */
#define PWM_VAL0_VAL0_MASK                       0xFFFFu
#define PWM_VAL0_VAL0_SHIFT                      0
#define PWM_VAL0_VAL0(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL0_VAL0_SHIFT))&PWM_VAL0_VAL0_MASK)
/* FRACVAL1 Bit Fields */
#define PWM_FRACVAL1_FRACVAL1_MASK               0xF800u
#define PWM_FRACVAL1_FRACVAL1_SHIFT              11
#define PWM_FRACVAL1_FRACVAL1(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL1_FRACVAL1_SHIFT))&PWM_FRACVAL1_FRACVAL1_MASK)
/* VAL1 Bit Fields */
#define PWM_VAL1_VAL1_MASK                       0xFFFFu
#define PWM_VAL1_VAL1_SHIFT                      0
#define PWM_VAL1_VAL1(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL1_VAL1_SHIFT))&PWM_VAL1_VAL1_MASK)
/* FRACVAL2 Bit Fields */
#define PWM_FRACVAL2_FRACVAL2_MASK               0xF800u
#define PWM_FRACVAL2_FRACVAL2_SHIFT              11
#define PWM_FRACVAL2_FRACVAL2(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL2_FRACVAL2_SHIFT))&PWM_FRACVAL2_FRACVAL2_MASK)
/* VAL2 Bit Fields */
#define PWM_VAL2_VAL2_MASK                       0xFFFFu
#define PWM_VAL2_VAL2_SHIFT                      0
#define PWM_VAL2_VAL2(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL2_VAL2_SHIFT))&PWM_VAL2_VAL2_MASK)
/* FRACVAL3 Bit Fields */
#define PWM_FRACVAL3_FRACVAL3_MASK               0xF800u
#define PWM_FRACVAL3_FRACVAL3_SHIFT              11
#define PWM_FRACVAL3_FRACVAL3(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL3_FRACVAL3_SHIFT))&PWM_FRACVAL3_FRACVAL3_MASK)
/* VAL3 Bit Fields */
#define PWM_VAL3_VAL3_MASK                       0xFFFFu
#define PWM_VAL3_VAL3_SHIFT                      0
#define PWM_VAL3_VAL3(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL3_VAL3_SHIFT))&PWM_VAL3_VAL3_MASK)
/* FRACVAL4 Bit Fields */
#define PWM_FRACVAL4_FRACVAL4_MASK               0xF800u
#define PWM_FRACVAL4_FRACVAL4_SHIFT              11
#define PWM_FRACVAL4_FRACVAL4(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL4_FRACVAL4_SHIFT))&PWM_FRACVAL4_FRACVAL4_MASK)
/* VAL4 Bit Fields */
#define PWM_VAL4_VAL4_MASK                       0xFFFFu
#define PWM_VAL4_VAL4_SHIFT                      0
#define PWM_VAL4_VAL4(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL4_VAL4_SHIFT))&PWM_VAL4_VAL4_MASK)
/* FRACVAL5 Bit Fields */
#define PWM_FRACVAL5_FRACVAL5_MASK               0xF800u
#define PWM_FRACVAL5_FRACVAL5_SHIFT              11
#define PWM_FRACVAL5_FRACVAL5(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_FRACVAL5_FRACVAL5_SHIFT))&PWM_FRACVAL5_FRACVAL5_MASK)
/* VAL5 Bit Fields */
#define PWM_VAL5_VAL5_MASK                       0xFFFFu
#define PWM_VAL5_VAL5_SHIFT                      0
#define PWM_VAL5_VAL5(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_VAL5_VAL5_SHIFT))&PWM_VAL5_VAL5_MASK)
/* FRCTRL Bit Fields */
#define PWM_FRCTRL_FRAC1_EN_MASK                 0x2u
#define PWM_FRCTRL_FRAC1_EN_SHIFT                1
#define PWM_FRCTRL_FRAC23_EN_MASK                0x4u
#define PWM_FRCTRL_FRAC23_EN_SHIFT               2
#define PWM_FRCTRL_FRAC45_EN_MASK                0x10u
#define PWM_FRCTRL_FRAC45_EN_SHIFT               4
#define PWM_FRCTRL_FRAC_PU_MASK                  0x100u
#define PWM_FRCTRL_FRAC_PU_SHIFT                 8
#define PWM_FRCTRL_TEST_MASK                     0x8000u
#define PWM_FRCTRL_TEST_SHIFT                    15
/* OCTRL Bit Fields */
#define PWM_OCTRL_PWMXFS_MASK                    0x3u
#define PWM_OCTRL_PWMXFS_SHIFT                   0
#define PWM_OCTRL_PWMXFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMXFS_SHIFT))&PWM_OCTRL_PWMXFS_MASK)
#define PWM_OCTRL_PWMBFS_MASK                    0xCu
#define PWM_OCTRL_PWMBFS_SHIFT                   2
#define PWM_OCTRL_PWMBFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMBFS_SHIFT))&PWM_OCTRL_PWMBFS_MASK)
#define PWM_OCTRL_PWMAFS_MASK                    0x30u
#define PWM_OCTRL_PWMAFS_SHIFT                   4
#define PWM_OCTRL_PWMAFS(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_OCTRL_PWMAFS_SHIFT))&PWM_OCTRL_PWMAFS_MASK)
#define PWM_OCTRL_POLX_MASK                      0x100u
#define PWM_OCTRL_POLX_SHIFT                     8
#define PWM_OCTRL_POLB_MASK                      0x200u
#define PWM_OCTRL_POLB_SHIFT                     9
#define PWM_OCTRL_POLA_MASK                      0x400u
#define PWM_OCTRL_POLA_SHIFT                     10
#define PWM_OCTRL_PWMX_IN_MASK                   0x2000u
#define PWM_OCTRL_PWMX_IN_SHIFT                  13
#define PWM_OCTRL_PWMB_IN_MASK                   0x4000u
#define PWM_OCTRL_PWMB_IN_SHIFT                  14
#define PWM_OCTRL_PWMA_IN_MASK                   0x8000u
#define PWM_OCTRL_PWMA_IN_SHIFT                  15
/* STS Bit Fields */
#define PWM_STS_CMPF_MASK                        0x3Fu
#define PWM_STS_CMPF_SHIFT                       0
#define PWM_STS_CMPF(x)                          (((uint16_t)(((uint16_t)(x))<<PWM_STS_CMPF_SHIFT))&PWM_STS_CMPF_MASK)
#define PWM_STS_CFX0_MASK                        0x40u
#define PWM_STS_CFX0_SHIFT                       6
#define PWM_STS_CFX1_MASK                        0x80u
#define PWM_STS_CFX1_SHIFT                       7
#define PWM_STS_CFB0_MASK                        0x100u
#define PWM_STS_CFB0_SHIFT                       8
#define PWM_STS_CFB1_MASK                        0x200u
#define PWM_STS_CFB1_SHIFT                       9
#define PWM_STS_CFA0_MASK                        0x400u
#define PWM_STS_CFA0_SHIFT                       10
#define PWM_STS_CFA1_MASK                        0x800u
#define PWM_STS_CFA1_SHIFT                       11
#define PWM_STS_RF_MASK                          0x1000u
#define PWM_STS_RF_SHIFT                         12
#define PWM_STS_REF_MASK                         0x2000u
#define PWM_STS_REF_SHIFT                        13
#define PWM_STS_RUF_MASK                         0x4000u
#define PWM_STS_RUF_SHIFT                        14
/* INTEN Bit Fields */
#define PWM_INTEN_CMPIE_MASK                     0x3Fu
#define PWM_INTEN_CMPIE_SHIFT                    0
#define PWM_INTEN_CMPIE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_INTEN_CMPIE_SHIFT))&PWM_INTEN_CMPIE_MASK)
#define PWM_INTEN_CX0IE_MASK                     0x40u
#define PWM_INTEN_CX0IE_SHIFT                    6
#define PWM_INTEN_CX1IE_MASK                     0x80u
#define PWM_INTEN_CX1IE_SHIFT                    7
#define PWM_INTEN_CB0IE_MASK                     0x100u
#define PWM_INTEN_CB0IE_SHIFT                    8
#define PWM_INTEN_CB1IE_MASK                     0x200u
#define PWM_INTEN_CB1IE_SHIFT                    9
#define PWM_INTEN_CA0IE_MASK                     0x400u
#define PWM_INTEN_CA0IE_SHIFT                    10
#define PWM_INTEN_CA1IE_MASK                     0x800u
#define PWM_INTEN_CA1IE_SHIFT                    11
#define PWM_INTEN_RIE_MASK                       0x1000u
#define PWM_INTEN_RIE_SHIFT                      12
#define PWM_INTEN_REIE_MASK                      0x2000u
#define PWM_INTEN_REIE_SHIFT                     13
/* DMAEN Bit Fields */
#define PWM_DMAEN_CX0DE_MASK                     0x1u
#define PWM_DMAEN_CX0DE_SHIFT                    0
#define PWM_DMAEN_CX1DE_MASK                     0x2u
#define PWM_DMAEN_CX1DE_SHIFT                    1
#define PWM_DMAEN_CB0DE_MASK                     0x4u
#define PWM_DMAEN_CB0DE_SHIFT                    2
#define PWM_DMAEN_CB1DE_MASK                     0x8u
#define PWM_DMAEN_CB1DE_SHIFT                    3
#define PWM_DMAEN_CA0DE_MASK                     0x10u
#define PWM_DMAEN_CA0DE_SHIFT                    4
#define PWM_DMAEN_CA1DE_MASK                     0x20u
#define PWM_DMAEN_CA1DE_SHIFT                    5
#define PWM_DMAEN_CAPTDE_MASK                    0xC0u
#define PWM_DMAEN_CAPTDE_SHIFT                   6
#define PWM_DMAEN_CAPTDE(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DMAEN_CAPTDE_SHIFT))&PWM_DMAEN_CAPTDE_MASK)
#define PWM_DMAEN_FAND_MASK                      0x100u
#define PWM_DMAEN_FAND_SHIFT                     8
#define PWM_DMAEN_VALDE_MASK                     0x200u
#define PWM_DMAEN_VALDE_SHIFT                    9
/* TCTRL Bit Fields */
#define PWM_TCTRL_OUT_TRIG_EN_MASK               0x3Fu
#define PWM_TCTRL_OUT_TRIG_EN_SHIFT              0
#define PWM_TCTRL_OUT_TRIG_EN(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_TCTRL_OUT_TRIG_EN_SHIFT))&PWM_TCTRL_OUT_TRIG_EN_MASK)
#define PWM_TCTRL_PWBOT1_MASK                    0x4000u
#define PWM_TCTRL_PWBOT1_SHIFT                   14
#define PWM_TCTRL_PWAOT0_MASK                    0x8000u
#define PWM_TCTRL_PWAOT0_SHIFT                   15
/* DISMAP Bit Fields */
#define PWM_DISMAP_DIS0A_MASK                    0xFu
#define PWM_DISMAP_DIS0A_SHIFT                   0
#define PWM_DISMAP_DIS0A(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0A_SHIFT))&PWM_DISMAP_DIS0A_MASK)
#define PWM_DISMAP_DIS1A_MASK                    0xFu
#define PWM_DISMAP_DIS1A_SHIFT                   0
#define PWM_DISMAP_DIS1A(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1A_SHIFT))&PWM_DISMAP_DIS1A_MASK)
#define PWM_DISMAP_DIS0B_MASK                    0xF0u
#define PWM_DISMAP_DIS0B_SHIFT                   4
#define PWM_DISMAP_DIS0B(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0B_SHIFT))&PWM_DISMAP_DIS0B_MASK)
#define PWM_DISMAP_DIS1B_MASK                    0xF0u
#define PWM_DISMAP_DIS1B_SHIFT                   4
#define PWM_DISMAP_DIS1B(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1B_SHIFT))&PWM_DISMAP_DIS1B_MASK)
#define PWM_DISMAP_DIS1X_MASK                    0xF00u
#define PWM_DISMAP_DIS1X_SHIFT                   8
#define PWM_DISMAP_DIS1X(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS1X_SHIFT))&PWM_DISMAP_DIS1X_MASK)
#define PWM_DISMAP_DIS0X_MASK                    0xF00u
#define PWM_DISMAP_DIS0X_SHIFT                   8
#define PWM_DISMAP_DIS0X(x)                      (((uint16_t)(((uint16_t)(x))<<PWM_DISMAP_DIS0X_SHIFT))&PWM_DISMAP_DIS0X_MASK)
/* DTCNT0 Bit Fields */
#define PWM_DTCNT0_DTCNT0_MASK                   0x7FFu
#define PWM_DTCNT0_DTCNT0_SHIFT                  0
#define PWM_DTCNT0_DTCNT0(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_DTCNT0_DTCNT0_SHIFT))&PWM_DTCNT0_DTCNT0_MASK)
/* DTCNT1 Bit Fields */
#define PWM_DTCNT1_DTCNT1_MASK                   0x7FFu
#define PWM_DTCNT1_DTCNT1_SHIFT                  0
#define PWM_DTCNT1_DTCNT1(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_DTCNT1_DTCNT1_SHIFT))&PWM_DTCNT1_DTCNT1_MASK)
/* CAPTCTRLA Bit Fields */
#define PWM_CAPTCTRLA_ARMA_MASK                  0x1u
#define PWM_CAPTCTRLA_ARMA_SHIFT                 0
#define PWM_CAPTCTRLA_ONESHOTA_MASK              0x2u
#define PWM_CAPTCTRLA_ONESHOTA_SHIFT             1
#define PWM_CAPTCTRLA_EDGA0_MASK                 0xCu
#define PWM_CAPTCTRLA_EDGA0_SHIFT                2
#define PWM_CAPTCTRLA_EDGA0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_EDGA0_SHIFT))&PWM_CAPTCTRLA_EDGA0_MASK)
#define PWM_CAPTCTRLA_EDGA1_MASK                 0x30u
#define PWM_CAPTCTRLA_EDGA1_SHIFT                4
#define PWM_CAPTCTRLA_EDGA1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_EDGA1_SHIFT))&PWM_CAPTCTRLA_EDGA1_MASK)
#define PWM_CAPTCTRLA_INP_SELA_MASK              0x40u
#define PWM_CAPTCTRLA_INP_SELA_SHIFT             6
#define PWM_CAPTCTRLA_EDGCNTA_EN_MASK            0x80u
#define PWM_CAPTCTRLA_EDGCNTA_EN_SHIFT           7
#define PWM_CAPTCTRLA_CFAWM_MASK                 0x300u
#define PWM_CAPTCTRLA_CFAWM_SHIFT                8
#define PWM_CAPTCTRLA_CFAWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CFAWM_SHIFT))&PWM_CAPTCTRLA_CFAWM_MASK)
#define PWM_CAPTCTRLA_CA0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLA_CA0CNT_SHIFT               10
#define PWM_CAPTCTRLA_CA0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CA0CNT_SHIFT))&PWM_CAPTCTRLA_CA0CNT_MASK)
#define PWM_CAPTCTRLA_CA1CNT_MASK                0xE000u
#define PWM_CAPTCTRLA_CA1CNT_SHIFT               13
#define PWM_CAPTCTRLA_CA1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLA_CA1CNT_SHIFT))&PWM_CAPTCTRLA_CA1CNT_MASK)
/* CAPTCOMPA Bit Fields */
#define PWM_CAPTCOMPA_EDGCMPA_MASK               0xFFu
#define PWM_CAPTCOMPA_EDGCMPA_SHIFT              0
#define PWM_CAPTCOMPA_EDGCMPA(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPA_EDGCMPA_SHIFT))&PWM_CAPTCOMPA_EDGCMPA_MASK)
#define PWM_CAPTCOMPA_EDGCNTA_MASK               0xFF00u
#define PWM_CAPTCOMPA_EDGCNTA_SHIFT              8
#define PWM_CAPTCOMPA_EDGCNTA(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPA_EDGCNTA_SHIFT))&PWM_CAPTCOMPA_EDGCNTA_MASK)
/* CAPTCTRLB Bit Fields */
#define PWM_CAPTCTRLB_ARMB_MASK                  0x1u
#define PWM_CAPTCTRLB_ARMB_SHIFT                 0
#define PWM_CAPTCTRLB_ONESHOTB_MASK              0x2u
#define PWM_CAPTCTRLB_ONESHOTB_SHIFT             1
#define PWM_CAPTCTRLB_EDGB0_MASK                 0xCu
#define PWM_CAPTCTRLB_EDGB0_SHIFT                2
#define PWM_CAPTCTRLB_EDGB0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_EDGB0_SHIFT))&PWM_CAPTCTRLB_EDGB0_MASK)
#define PWM_CAPTCTRLB_EDGB1_MASK                 0x30u
#define PWM_CAPTCTRLB_EDGB1_SHIFT                4
#define PWM_CAPTCTRLB_EDGB1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_EDGB1_SHIFT))&PWM_CAPTCTRLB_EDGB1_MASK)
#define PWM_CAPTCTRLB_INP_SELB_MASK              0x40u
#define PWM_CAPTCTRLB_INP_SELB_SHIFT             6
#define PWM_CAPTCTRLB_EDGCNTB_EN_MASK            0x80u
#define PWM_CAPTCTRLB_EDGCNTB_EN_SHIFT           7
#define PWM_CAPTCTRLB_CFBWM_MASK                 0x300u
#define PWM_CAPTCTRLB_CFBWM_SHIFT                8
#define PWM_CAPTCTRLB_CFBWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CFBWM_SHIFT))&PWM_CAPTCTRLB_CFBWM_MASK)
#define PWM_CAPTCTRLB_CB0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLB_CB0CNT_SHIFT               10
#define PWM_CAPTCTRLB_CB0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CB0CNT_SHIFT))&PWM_CAPTCTRLB_CB0CNT_MASK)
#define PWM_CAPTCTRLB_CB1CNT_MASK                0xE000u
#define PWM_CAPTCTRLB_CB1CNT_SHIFT               13
#define PWM_CAPTCTRLB_CB1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLB_CB1CNT_SHIFT))&PWM_CAPTCTRLB_CB1CNT_MASK)
/* CAPTCOMPB Bit Fields */
#define PWM_CAPTCOMPB_EDGCMPB_MASK               0xFFu
#define PWM_CAPTCOMPB_EDGCMPB_SHIFT              0
#define PWM_CAPTCOMPB_EDGCMPB(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPB_EDGCMPB_SHIFT))&PWM_CAPTCOMPB_EDGCMPB_MASK)
#define PWM_CAPTCOMPB_EDGCNTB_MASK               0xFF00u
#define PWM_CAPTCOMPB_EDGCNTB_SHIFT              8
#define PWM_CAPTCOMPB_EDGCNTB(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPB_EDGCNTB_SHIFT))&PWM_CAPTCOMPB_EDGCNTB_MASK)
/* CAPTCTRLX Bit Fields */
#define PWM_CAPTCTRLX_ARMX_MASK                  0x1u
#define PWM_CAPTCTRLX_ARMX_SHIFT                 0
#define PWM_CAPTCTRLX_ONESHOTX_MASK              0x2u
#define PWM_CAPTCTRLX_ONESHOTX_SHIFT             1
#define PWM_CAPTCTRLX_EDGX0_MASK                 0xCu
#define PWM_CAPTCTRLX_EDGX0_SHIFT                2
#define PWM_CAPTCTRLX_EDGX0(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_EDGX0_SHIFT))&PWM_CAPTCTRLX_EDGX0_MASK)
#define PWM_CAPTCTRLX_EDGX1_MASK                 0x30u
#define PWM_CAPTCTRLX_EDGX1_SHIFT                4
#define PWM_CAPTCTRLX_EDGX1(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_EDGX1_SHIFT))&PWM_CAPTCTRLX_EDGX1_MASK)
#define PWM_CAPTCTRLX_INP_SELX_MASK              0x40u
#define PWM_CAPTCTRLX_INP_SELX_SHIFT             6
#define PWM_CAPTCTRLX_EDGCNTX_EN_MASK            0x80u
#define PWM_CAPTCTRLX_EDGCNTX_EN_SHIFT           7
#define PWM_CAPTCTRLX_CFXWM_MASK                 0x300u
#define PWM_CAPTCTRLX_CFXWM_SHIFT                8
#define PWM_CAPTCTRLX_CFXWM(x)                   (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CFXWM_SHIFT))&PWM_CAPTCTRLX_CFXWM_MASK)
#define PWM_CAPTCTRLX_CX0CNT_MASK                0x1C00u
#define PWM_CAPTCTRLX_CX0CNT_SHIFT               10
#define PWM_CAPTCTRLX_CX0CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CX0CNT_SHIFT))&PWM_CAPTCTRLX_CX0CNT_MASK)
#define PWM_CAPTCTRLX_CX1CNT_MASK                0xE000u
#define PWM_CAPTCTRLX_CX1CNT_SHIFT               13
#define PWM_CAPTCTRLX_CX1CNT(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCTRLX_CX1CNT_SHIFT))&PWM_CAPTCTRLX_CX1CNT_MASK)
/* CAPTCOMPX Bit Fields */
#define PWM_CAPTCOMPX_EDGCMPX_MASK               0xFFu
#define PWM_CAPTCOMPX_EDGCMPX_SHIFT              0
#define PWM_CAPTCOMPX_EDGCMPX(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPX_EDGCMPX_SHIFT))&PWM_CAPTCOMPX_EDGCMPX_MASK)
#define PWM_CAPTCOMPX_EDGCNTX_MASK               0xFF00u
#define PWM_CAPTCOMPX_EDGCNTX_SHIFT              8
#define PWM_CAPTCOMPX_EDGCNTX(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CAPTCOMPX_EDGCNTX_SHIFT))&PWM_CAPTCOMPX_EDGCNTX_MASK)
/* CVAL0 Bit Fields */
#define PWM_CVAL0_CAPTVAL0_MASK                  0xFFFFu
#define PWM_CVAL0_CAPTVAL0_SHIFT                 0
#define PWM_CVAL0_CAPTVAL0(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL0_CAPTVAL0_SHIFT))&PWM_CVAL0_CAPTVAL0_MASK)
/* CVAL0CYC Bit Fields */
#define PWM_CVAL0CYC_CVAL0CYC_MASK               0xFu
#define PWM_CVAL0CYC_CVAL0CYC_SHIFT              0
#define PWM_CVAL0CYC_CVAL0CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL0CYC_CVAL0CYC_SHIFT))&PWM_CVAL0CYC_CVAL0CYC_MASK)
/* CVAL1 Bit Fields */
#define PWM_CVAL1_CAPTVAL1_MASK                  0xFFFFu
#define PWM_CVAL1_CAPTVAL1_SHIFT                 0
#define PWM_CVAL1_CAPTVAL1(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL1_CAPTVAL1_SHIFT))&PWM_CVAL1_CAPTVAL1_MASK)
/* CVAL1CYC Bit Fields */
#define PWM_CVAL1CYC_CVAL1CYC_MASK               0xFu
#define PWM_CVAL1CYC_CVAL1CYC_SHIFT              0
#define PWM_CVAL1CYC_CVAL1CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL1CYC_CVAL1CYC_SHIFT))&PWM_CVAL1CYC_CVAL1CYC_MASK)
/* CVAL2 Bit Fields */
#define PWM_CVAL2_CAPTVAL2_MASK                  0xFFFFu
#define PWM_CVAL2_CAPTVAL2_SHIFT                 0
#define PWM_CVAL2_CAPTVAL2(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL2_CAPTVAL2_SHIFT))&PWM_CVAL2_CAPTVAL2_MASK)
/* CVAL2CYC Bit Fields */
#define PWM_CVAL2CYC_CVAL2CYC_MASK               0xFu
#define PWM_CVAL2CYC_CVAL2CYC_SHIFT              0
#define PWM_CVAL2CYC_CVAL2CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL2CYC_CVAL2CYC_SHIFT))&PWM_CVAL2CYC_CVAL2CYC_MASK)
/* CVAL3 Bit Fields */
#define PWM_CVAL3_CAPTVAL3_MASK                  0xFFFFu
#define PWM_CVAL3_CAPTVAL3_SHIFT                 0
#define PWM_CVAL3_CAPTVAL3(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL3_CAPTVAL3_SHIFT))&PWM_CVAL3_CAPTVAL3_MASK)
/* CVAL3CYC Bit Fields */
#define PWM_CVAL3CYC_CVAL3CYC_MASK               0xFu
#define PWM_CVAL3CYC_CVAL3CYC_SHIFT              0
#define PWM_CVAL3CYC_CVAL3CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL3CYC_CVAL3CYC_SHIFT))&PWM_CVAL3CYC_CVAL3CYC_MASK)
/* CVAL4 Bit Fields */
#define PWM_CVAL4_CAPTVAL4_MASK                  0xFFFFu
#define PWM_CVAL4_CAPTVAL4_SHIFT                 0
#define PWM_CVAL4_CAPTVAL4(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL4_CAPTVAL4_SHIFT))&PWM_CVAL4_CAPTVAL4_MASK)
/* CVAL4CYC Bit Fields */
#define PWM_CVAL4CYC_CVAL4CYC_MASK               0xFu
#define PWM_CVAL4CYC_CVAL4CYC_SHIFT              0
#define PWM_CVAL4CYC_CVAL4CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL4CYC_CVAL4CYC_SHIFT))&PWM_CVAL4CYC_CVAL4CYC_MASK)
/* CVAL5 Bit Fields */
#define PWM_CVAL5_CAPTVAL5_MASK                  0xFFFFu
#define PWM_CVAL5_CAPTVAL5_SHIFT                 0
#define PWM_CVAL5_CAPTVAL5(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_CVAL5_CAPTVAL5_SHIFT))&PWM_CVAL5_CAPTVAL5_MASK)
/* CVAL5CYC Bit Fields */
#define PWM_CVAL5CYC_CVAL5CYC_MASK               0xFu
#define PWM_CVAL5CYC_CVAL5CYC_SHIFT              0
#define PWM_CVAL5CYC_CVAL5CYC(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_CVAL5CYC_CVAL5CYC_SHIFT))&PWM_CVAL5CYC_CVAL5CYC_MASK)
/* OUTEN Bit Fields */
#define PWM_OUTEN_PWMX_EN_MASK                   0xFu
#define PWM_OUTEN_PWMX_EN_SHIFT                  0
#define PWM_OUTEN_PWMX_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMX_EN_SHIFT))&PWM_OUTEN_PWMX_EN_MASK)
#define PWM_OUTEN_PWMB_EN_MASK                   0xF0u
#define PWM_OUTEN_PWMB_EN_SHIFT                  4
#define PWM_OUTEN_PWMB_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMB_EN_SHIFT))&PWM_OUTEN_PWMB_EN_MASK)
#define PWM_OUTEN_PWMA_EN_MASK                   0xF00u
#define PWM_OUTEN_PWMA_EN_SHIFT                  8
#define PWM_OUTEN_PWMA_EN(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_OUTEN_PWMA_EN_SHIFT))&PWM_OUTEN_PWMA_EN_MASK)
/* MASK Bit Fields */
#define PWM_MASK_MASKX_MASK                      0xFu
#define PWM_MASK_MASKX_SHIFT                     0
#define PWM_MASK_MASKX(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKX_SHIFT))&PWM_MASK_MASKX_MASK)
#define PWM_MASK_MASKB_MASK                      0xF0u
#define PWM_MASK_MASKB_SHIFT                     4
#define PWM_MASK_MASKB(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKB_SHIFT))&PWM_MASK_MASKB_MASK)
#define PWM_MASK_MASKA_MASK                      0xF00u
#define PWM_MASK_MASKA_SHIFT                     8
#define PWM_MASK_MASKA(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MASK_MASKA_SHIFT))&PWM_MASK_MASKA_MASK)
#define PWM_MASK_UPDATE_MASK_MASK                0xF000u
#define PWM_MASK_UPDATE_MASK_SHIFT               12
#define PWM_MASK_UPDATE_MASK(x)                  (((uint16_t)(((uint16_t)(x))<<PWM_MASK_UPDATE_MASK_SHIFT))&PWM_MASK_UPDATE_MASK_MASK)
/* SWCOUT Bit Fields */
#define PWM_SWCOUT_SM0OUT45_MASK                 0x1u
#define PWM_SWCOUT_SM0OUT45_SHIFT                0
#define PWM_SWCOUT_SM0OUT23_MASK                 0x2u
#define PWM_SWCOUT_SM0OUT23_SHIFT                1
#define PWM_SWCOUT_SM1OUT45_MASK                 0x4u
#define PWM_SWCOUT_SM1OUT45_SHIFT                2
#define PWM_SWCOUT_SM1OUT23_MASK                 0x8u
#define PWM_SWCOUT_SM1OUT23_SHIFT                3
#define PWM_SWCOUT_SM2OUT45_MASK                 0x10u
#define PWM_SWCOUT_SM2OUT45_SHIFT                4
#define PWM_SWCOUT_SM2OUT23_MASK                 0x20u
#define PWM_SWCOUT_SM2OUT23_SHIFT                5
#define PWM_SWCOUT_SM3OUT45_MASK                 0x40u
#define PWM_SWCOUT_SM3OUT45_SHIFT                6
#define PWM_SWCOUT_SM3OUT23_MASK                 0x80u
#define PWM_SWCOUT_SM3OUT23_SHIFT                7
/* DTSRCSEL Bit Fields */
#define PWM_DTSRCSEL_SM0SEL45_MASK               0x3u
#define PWM_DTSRCSEL_SM0SEL45_SHIFT              0
#define PWM_DTSRCSEL_SM0SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM0SEL45_SHIFT))&PWM_DTSRCSEL_SM0SEL45_MASK)
#define PWM_DTSRCSEL_SM0SEL23_MASK               0xCu
#define PWM_DTSRCSEL_SM0SEL23_SHIFT              2
#define PWM_DTSRCSEL_SM0SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM0SEL23_SHIFT))&PWM_DTSRCSEL_SM0SEL23_MASK)
#define PWM_DTSRCSEL_SM1SEL45_MASK               0x30u
#define PWM_DTSRCSEL_SM1SEL45_SHIFT              4
#define PWM_DTSRCSEL_SM1SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM1SEL45_SHIFT))&PWM_DTSRCSEL_SM1SEL45_MASK)
#define PWM_DTSRCSEL_SM1SEL23_MASK               0xC0u
#define PWM_DTSRCSEL_SM1SEL23_SHIFT              6
#define PWM_DTSRCSEL_SM1SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM1SEL23_SHIFT))&PWM_DTSRCSEL_SM1SEL23_MASK)
#define PWM_DTSRCSEL_SM2SEL45_MASK               0x300u
#define PWM_DTSRCSEL_SM2SEL45_SHIFT              8
#define PWM_DTSRCSEL_SM2SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM2SEL45_SHIFT))&PWM_DTSRCSEL_SM2SEL45_MASK)
#define PWM_DTSRCSEL_SM2SEL23_MASK               0xC00u
#define PWM_DTSRCSEL_SM2SEL23_SHIFT              10
#define PWM_DTSRCSEL_SM2SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM2SEL23_SHIFT))&PWM_DTSRCSEL_SM2SEL23_MASK)
#define PWM_DTSRCSEL_SM3SEL45_MASK               0x3000u
#define PWM_DTSRCSEL_SM3SEL45_SHIFT              12
#define PWM_DTSRCSEL_SM3SEL45(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM3SEL45_SHIFT))&PWM_DTSRCSEL_SM3SEL45_MASK)
#define PWM_DTSRCSEL_SM3SEL23_MASK               0xC000u
#define PWM_DTSRCSEL_SM3SEL23_SHIFT              14
#define PWM_DTSRCSEL_SM3SEL23(x)                 (((uint16_t)(((uint16_t)(x))<<PWM_DTSRCSEL_SM3SEL23_SHIFT))&PWM_DTSRCSEL_SM3SEL23_MASK)
/* MCTRL Bit Fields */
#define PWM_MCTRL_LDOK_MASK                      0xFu
#define PWM_MCTRL_LDOK_SHIFT                     0
#define PWM_MCTRL_LDOK(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_LDOK_SHIFT))&PWM_MCTRL_LDOK_MASK)
#define PWM_MCTRL_CLDOK_MASK                     0xF0u
#define PWM_MCTRL_CLDOK_SHIFT                    4
#define PWM_MCTRL_CLDOK(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_CLDOK_SHIFT))&PWM_MCTRL_CLDOK_MASK)
#define PWM_MCTRL_RUN_MASK                       0xF00u
#define PWM_MCTRL_RUN_SHIFT                      8
#define PWM_MCTRL_RUN(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_RUN_SHIFT))&PWM_MCTRL_RUN_MASK)
#define PWM_MCTRL_IPOL_MASK                      0xF000u
#define PWM_MCTRL_IPOL_SHIFT                     12
#define PWM_MCTRL_IPOL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL_IPOL_SHIFT))&PWM_MCTRL_IPOL_MASK)
/* MCTRL2 Bit Fields */
#define PWM_MCTRL2_MONPLL_MASK                   0x3u
#define PWM_MCTRL2_MONPLL_SHIFT                  0
#define PWM_MCTRL2_MONPLL(x)                     (((uint16_t)(((uint16_t)(x))<<PWM_MCTRL2_MONPLL_SHIFT))&PWM_MCTRL2_MONPLL_MASK)
/* FCTRL Bit Fields */
#define PWM_FCTRL_FIE_MASK                       0xFu
#define PWM_FCTRL_FIE_SHIFT                      0
#define PWM_FCTRL_FIE(x)                         (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FIE_SHIFT))&PWM_FCTRL_FIE_MASK)
#define PWM_FCTRL_FSAFE_MASK                     0xF0u
#define PWM_FCTRL_FSAFE_SHIFT                    4
#define PWM_FCTRL_FSAFE(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FSAFE_SHIFT))&PWM_FCTRL_FSAFE_MASK)
#define PWM_FCTRL_FAUTO_MASK                     0xF00u
#define PWM_FCTRL_FAUTO_SHIFT                    8
#define PWM_FCTRL_FAUTO(x)                       (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FAUTO_SHIFT))&PWM_FCTRL_FAUTO_MASK)
#define PWM_FCTRL_FLVL_MASK                      0xF000u
#define PWM_FCTRL_FLVL_SHIFT                     12
#define PWM_FCTRL_FLVL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FCTRL_FLVL_SHIFT))&PWM_FCTRL_FLVL_MASK)
/* FSTS Bit Fields */
#define PWM_FSTS_FFLAG_MASK                      0xFu
#define PWM_FSTS_FFLAG_SHIFT                     0
#define PWM_FSTS_FFLAG(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFLAG_SHIFT))&PWM_FSTS_FFLAG_MASK)
#define PWM_FSTS_FFULL_MASK                      0xF0u
#define PWM_FSTS_FFULL_SHIFT                     4
#define PWM_FSTS_FFULL(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFULL_SHIFT))&PWM_FSTS_FFULL_MASK)
#define PWM_FSTS_FFPIN_MASK                      0xF00u
#define PWM_FSTS_FFPIN_SHIFT                     8
#define PWM_FSTS_FFPIN(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FFPIN_SHIFT))&PWM_FSTS_FFPIN_MASK)
#define PWM_FSTS_FHALF_MASK                      0xF000u
#define PWM_FSTS_FHALF_SHIFT                     12
#define PWM_FSTS_FHALF(x)                        (((uint16_t)(((uint16_t)(x))<<PWM_FSTS_FHALF_SHIFT))&PWM_FSTS_FHALF_MASK)
/* FFILT Bit Fields */
#define PWM_FFILT_FILT_PER_MASK                  0xFFu
#define PWM_FFILT_FILT_PER_SHIFT                 0
#define PWM_FFILT_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_FFILT_FILT_PER_SHIFT))&PWM_FFILT_FILT_PER_MASK)
#define PWM_FFILT_FILT_CNT_MASK                  0x700u
#define PWM_FFILT_FILT_CNT_SHIFT                 8
#define PWM_FFILT_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x))<<PWM_FFILT_FILT_CNT_SHIFT))&PWM_FFILT_FILT_CNT_MASK)
#define PWM_FFILT_GSTR_MASK                      0x8000u
#define PWM_FFILT_GSTR_SHIFT                     15
/* FTST Bit Fields */
#define PWM_FTST_FTEST_MASK                      0x1u
#define PWM_FTST_FTEST_SHIFT                     0

/*!
 * @}
 */ /* end of group PWM_Register_Masks */


/* PWM - Peripheral instance base addresses */
/** Peripheral PWMA base pointer */
#define PWMA_BASE_PTR                            ((PWM_MemMapPtr)0xE600u)
/** Array initializer of PWM peripheral base pointers */
#define PWM_BASE_PTRS                            { PWMA_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- PWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Accessor_Macros PWM - Register accessor macros
 * @{
 */


/* PWM - Register instance definitions */
/* PWMA */
#define PWMA_SM0CNT                              PWM_CNT_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0INIT                             PWM_INIT_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CTRL2                            PWM_CTRL2_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CTRL                             PWM_CTRL_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL0                             PWM_VAL0_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRACVAL1                         PWM_FRACVAL1_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL1                             PWM_VAL1_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRACVAL2                         PWM_FRACVAL2_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL2                             PWM_VAL2_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRACVAL3                         PWM_FRACVAL3_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL3                             PWM_VAL3_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRACVAL4                         PWM_FRACVAL4_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL4                             PWM_VAL4_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRACVAL5                         PWM_FRACVAL5_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0VAL5                             PWM_VAL5_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0FRCTRL                           PWM_FRCTRL_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0OCTRL                            PWM_OCTRL_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0STS                              PWM_STS_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0INTEN                            PWM_INTEN_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0DMAEN                            PWM_DMAEN_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0TCTRL                            PWM_TCTRL_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0DISMAP0                          PWM_DISMAP_REG(PWMA_BASE_PTR,0,0)
#define PWMA_SM0DISMAP1                          PWM_DISMAP_REG(PWMA_BASE_PTR,0,1)
#define PWMA_SM0DTCNT0                           PWM_DTCNT0_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0DTCNT1                           PWM_DTCNT1_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL0                            PWM_CVAL0_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL1                            PWM_CVAL1_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL2                            PWM_CVAL2_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL3                            PWM_CVAL3_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL4                            PWM_CVAL4_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL5                            PWM_CVAL5_REG(PWMA_BASE_PTR,0)
#define PWMA_SM0CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA_BASE_PTR,0)
#define PWMA_SM1CNT                              PWM_CNT_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1INIT                             PWM_INIT_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CTRL2                            PWM_CTRL2_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CTRL                             PWM_CTRL_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL0                             PWM_VAL0_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRACVAL1                         PWM_FRACVAL1_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL1                             PWM_VAL1_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRACVAL2                         PWM_FRACVAL2_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL2                             PWM_VAL2_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRACVAL3                         PWM_FRACVAL3_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL3                             PWM_VAL3_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRACVAL4                         PWM_FRACVAL4_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL4                             PWM_VAL4_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRACVAL5                         PWM_FRACVAL5_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1VAL5                             PWM_VAL5_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1FRCTRL                           PWM_FRCTRL_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1OCTRL                            PWM_OCTRL_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1STS                              PWM_STS_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1INTEN                            PWM_INTEN_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1DMAEN                            PWM_DMAEN_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1TCTRL                            PWM_TCTRL_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1DISMAP0                          PWM_DISMAP_REG(PWMA_BASE_PTR,1,0)
#define PWMA_SM1DISMAP1                          PWM_DISMAP_REG(PWMA_BASE_PTR,1,1)
#define PWMA_SM1DTCNT0                           PWM_DTCNT0_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1DTCNT1                           PWM_DTCNT1_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL0                            PWM_CVAL0_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL1                            PWM_CVAL1_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL2                            PWM_CVAL2_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL3                            PWM_CVAL3_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL4                            PWM_CVAL4_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL5                            PWM_CVAL5_REG(PWMA_BASE_PTR,1)
#define PWMA_SM1CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA_BASE_PTR,1)
#define PWMA_SM2CNT                              PWM_CNT_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2INIT                             PWM_INIT_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CTRL2                            PWM_CTRL2_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CTRL                             PWM_CTRL_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL0                             PWM_VAL0_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRACVAL1                         PWM_FRACVAL1_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL1                             PWM_VAL1_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRACVAL2                         PWM_FRACVAL2_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL2                             PWM_VAL2_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRACVAL3                         PWM_FRACVAL3_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL3                             PWM_VAL3_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRACVAL4                         PWM_FRACVAL4_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL4                             PWM_VAL4_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRACVAL5                         PWM_FRACVAL5_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2VAL5                             PWM_VAL5_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2FRCTRL                           PWM_FRCTRL_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2OCTRL                            PWM_OCTRL_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2STS                              PWM_STS_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2INTEN                            PWM_INTEN_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2DMAEN                            PWM_DMAEN_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2TCTRL                            PWM_TCTRL_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2DISMAP0                          PWM_DISMAP_REG(PWMA_BASE_PTR,2,0)
#define PWMA_SM2DISMAP1                          PWM_DISMAP_REG(PWMA_BASE_PTR,2,1)
#define PWMA_SM2DTCNT0                           PWM_DTCNT0_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2DTCNT1                           PWM_DTCNT1_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL0                            PWM_CVAL0_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL1                            PWM_CVAL1_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL2                            PWM_CVAL2_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL3                            PWM_CVAL3_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL4                            PWM_CVAL4_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL5                            PWM_CVAL5_REG(PWMA_BASE_PTR,2)
#define PWMA_SM2CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA_BASE_PTR,2)
#define PWMA_SM3CNT                              PWM_CNT_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3INIT                             PWM_INIT_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CTRL2                            PWM_CTRL2_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CTRL                             PWM_CTRL_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL0                             PWM_VAL0_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRACVAL1                         PWM_FRACVAL1_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL1                             PWM_VAL1_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRACVAL2                         PWM_FRACVAL2_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL2                             PWM_VAL2_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRACVAL3                         PWM_FRACVAL3_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL3                             PWM_VAL3_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRACVAL4                         PWM_FRACVAL4_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL4                             PWM_VAL4_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRACVAL5                         PWM_FRACVAL5_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3VAL5                             PWM_VAL5_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3FRCTRL                           PWM_FRCTRL_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3OCTRL                            PWM_OCTRL_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3STS                              PWM_STS_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3INTEN                            PWM_INTEN_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3DMAEN                            PWM_DMAEN_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3TCTRL                            PWM_TCTRL_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3DISMAP0                          PWM_DISMAP_REG(PWMA_BASE_PTR,3,0)
#define PWMA_SM3DISMAP1                          PWM_DISMAP_REG(PWMA_BASE_PTR,3,1)
#define PWMA_SM3DTCNT0                           PWM_DTCNT0_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3DTCNT1                           PWM_DTCNT1_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCTRLA                        PWM_CAPTCTRLA_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCOMPA                        PWM_CAPTCOMPA_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCTRLB                        PWM_CAPTCTRLB_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCOMPB                        PWM_CAPTCOMPB_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCTRLX                        PWM_CAPTCTRLX_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CAPTCOMPX                        PWM_CAPTCOMPX_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL0                            PWM_CVAL0_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL0CYC                         PWM_CVAL0CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL1                            PWM_CVAL1_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL1CYC                         PWM_CVAL1CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL2                            PWM_CVAL2_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL2CYC                         PWM_CVAL2CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL3                            PWM_CVAL3_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL3CYC                         PWM_CVAL3CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL4                            PWM_CVAL4_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL4CYC                         PWM_CVAL4CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL5                            PWM_CVAL5_REG(PWMA_BASE_PTR,3)
#define PWMA_SM3CVAL5CYC                         PWM_CVAL5CYC_REG(PWMA_BASE_PTR,3)
#define PWMA_OUTEN                               PWM_OUTEN_REG(PWMA_BASE_PTR)
#define PWMA_MASK                                PWM_MASK_REG(PWMA_BASE_PTR)
#define PWMA_SWCOUT                              PWM_SWCOUT_REG(PWMA_BASE_PTR)
#define PWMA_DTSRCSEL                            PWM_DTSRCSEL_REG(PWMA_BASE_PTR)
#define PWMA_MCTRL                               PWM_MCTRL_REG(PWMA_BASE_PTR)
#define PWMA_MCTRL2                              PWM_MCTRL2_REG(PWMA_BASE_PTR)
#define PWMA_FCTRL0                              PWM_FCTRL_REG(PWMA_BASE_PTR,0)
#define PWMA_FSTS0                               PWM_FSTS_REG(PWMA_BASE_PTR,0)
#define PWMA_FFILT0                              PWM_FFILT_REG(PWMA_BASE_PTR,0)
#define PWMA_FTST0                               PWM_FTST_REG(PWMA_BASE_PTR,0)
#define PWMA_FCTRL1                              PWM_FCTRL_REG(PWMA_BASE_PTR,1)
#define PWMA_FSTS1                               PWM_FSTS_REG(PWMA_BASE_PTR,1)
#define PWMA_FFILT1                              PWM_FFILT_REG(PWMA_BASE_PTR,1)
#define PWMA_FTST1                               PWM_FTST_REG(PWMA_BASE_PTR,1)

/* PWM - Register array accessors */
#define PWMA_CNT(index)                          PWM_CNT_REG(PWMA_BASE_PTR,index)
#define PWMA_INIT(index)                         PWM_INIT_REG(PWMA_BASE_PTR,index)
#define PWMA_CTRL2(index)                        PWM_CTRL2_REG(PWMA_BASE_PTR,index)
#define PWMA_CTRL(index)                         PWM_CTRL_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL0(index)                         PWM_VAL0_REG(PWMA_BASE_PTR,index)
#define PWMA_FRACVAL1(index)                     PWM_FRACVAL1_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL1(index)                         PWM_VAL1_REG(PWMA_BASE_PTR,index)
#define PWMA_FRACVAL2(index)                     PWM_FRACVAL2_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL2(index)                         PWM_VAL2_REG(PWMA_BASE_PTR,index)
#define PWMA_FRACVAL3(index)                     PWM_FRACVAL3_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL3(index)                         PWM_VAL3_REG(PWMA_BASE_PTR,index)
#define PWMA_FRACVAL4(index)                     PWM_FRACVAL4_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL4(index)                         PWM_VAL4_REG(PWMA_BASE_PTR,index)
#define PWMA_FRACVAL5(index)                     PWM_FRACVAL5_REG(PWMA_BASE_PTR,index)
#define PWMA_VAL5(index)                         PWM_VAL5_REG(PWMA_BASE_PTR,index)
#define PWMA_FRCTRL(index)                       PWM_FRCTRL_REG(PWMA_BASE_PTR,index)
#define PWMA_OCTRL(index)                        PWM_OCTRL_REG(PWMA_BASE_PTR,index)
#define PWMA_STS(index)                          PWM_STS_REG(PWMA_BASE_PTR,index)
#define PWMA_INTEN(index)                        PWM_INTEN_REG(PWMA_BASE_PTR,index)
#define PWMA_DMAEN(index)                        PWM_DMAEN_REG(PWMA_BASE_PTR,index)
#define PWMA_TCTRL(index)                        PWM_TCTRL_REG(PWMA_BASE_PTR,index)
#define PWMA_DISMAP(index,index2)                PWM_DISMAP_REG(PWMA_BASE_PTR,index,index2)
#define PWMA_DTCNT0(index)                       PWM_DTCNT0_REG(PWMA_BASE_PTR,index)
#define PWMA_DTCNT1(index)                       PWM_DTCNT1_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCTRLA(index)                    PWM_CAPTCTRLA_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCOMPA(index)                    PWM_CAPTCOMPA_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCTRLB(index)                    PWM_CAPTCTRLB_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCOMPB(index)                    PWM_CAPTCOMPB_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCTRLX(index)                    PWM_CAPTCTRLX_REG(PWMA_BASE_PTR,index)
#define PWMA_CAPTCOMPX(index)                    PWM_CAPTCOMPX_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL0(index)                        PWM_CVAL0_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL0CYC(index)                     PWM_CVAL0CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL1(index)                        PWM_CVAL1_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL1CYC(index)                     PWM_CVAL1CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL2(index)                        PWM_CVAL2_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL2CYC(index)                     PWM_CVAL2CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL3(index)                        PWM_CVAL3_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL3CYC(index)                     PWM_CVAL3CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL4(index)                        PWM_CVAL4_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL4CYC(index)                     PWM_CVAL4CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL5(index)                        PWM_CVAL5_REG(PWMA_BASE_PTR,index)
#define PWMA_CVAL5CYC(index)                     PWM_CVAL5CYC_REG(PWMA_BASE_PTR,index)
#define PWMA_FCTRL(index)                        PWM_FCTRL_REG(PWMA_BASE_PTR,index)
#define PWMA_FSTS(index)                         PWM_FSTS_REG(PWMA_BASE_PTR,index)
#define PWMA_FFILT(index)                        PWM_FFILT_REG(PWMA_BASE_PTR,index)
#define PWMA_FTST(index)                         PWM_FTST_REG(PWMA_BASE_PTR,index)

/*!
 * @}
 */ /* end of group PWM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PWM_Peripheral */


/* ----------------------------------------------------------------------------
   -- QSCI
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Peripheral QSCI
 * @{
 */

/** QSCI - Peripheral register structure */
typedef struct QSCI_MemMap {
  uint16_t RATE;                                   /**< QSCI Baud Rate Register, offset: 0x0 */
  uint16_t CTRL1;                                  /**< QSCI Control Register 1, offset: 0x1 */
  uint16_t CTRL2;                                  /**< QSCI Control Register 2, offset: 0x2 */
  uint16_t STAT;                                   /**< QSCI Status Register, offset: 0x3 */
  uint16_t DATA;                                   /**< QSCI Data Register, offset: 0x4 */
  uint16_t CTRL3;                                  /**< QSCI Control Register 3, offset: 0x5 */
} volatile *QSCI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- QSCI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Register_Accessor_Macros QSCI - Register accessor macros
 * @{
 */


/* QSCI - Register accessors */
#define QSCI_RATE_REG(base)                      ((base)->RATE)
#define QSCI_CTRL1_REG(base)                     ((base)->CTRL1)
#define QSCI_CTRL2_REG(base)                     ((base)->CTRL2)
#define QSCI_STAT_REG(base)                      ((base)->STAT)
#define QSCI_DATA_REG(base)                      ((base)->DATA)
#define QSCI_CTRL3_REG(base)                     ((base)->CTRL3)

/*!
 * @}
 */ /* end of group QSCI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- QSCI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Register_Masks QSCI Register Masks
 * @{
 */

/* RATE Bit Fields */
#define QSCI_RATE_FRAC_SBR_MASK                  0x7u
#define QSCI_RATE_FRAC_SBR_SHIFT                 0
#define QSCI_RATE_FRAC_SBR(x)                    (((uint16_t)(((uint16_t)(x))<<QSCI_RATE_FRAC_SBR_SHIFT))&QSCI_RATE_FRAC_SBR_MASK)
#define QSCI_RATE_SBRL_MASK                      0xFFF8u
#define QSCI_RATE_SBRL_SHIFT                     3
#define QSCI_RATE_SBRL(x)                        (((uint16_t)(((uint16_t)(x))<<QSCI_RATE_SBRL_SHIFT))&QSCI_RATE_SBRL_MASK)
/* CTRL1 Bit Fields */
#define QSCI_CTRL1_SBK_MASK                      0x1u
#define QSCI_CTRL1_SBK_SHIFT                     0
#define QSCI_CTRL1_RWU_MASK                      0x2u
#define QSCI_CTRL1_RWU_SHIFT                     1
#define QSCI_CTRL1_RE_MASK                       0x4u
#define QSCI_CTRL1_RE_SHIFT                      2
#define QSCI_CTRL1_TE_MASK                       0x8u
#define QSCI_CTRL1_TE_SHIFT                      3
#define QSCI_CTRL1_REIE_MASK                     0x10u
#define QSCI_CTRL1_REIE_SHIFT                    4
#define QSCI_CTRL1_RFIE_MASK                     0x20u
#define QSCI_CTRL1_RFIE_SHIFT                    5
#define QSCI_CTRL1_TIIE_MASK                     0x40u
#define QSCI_CTRL1_TIIE_SHIFT                    6
#define QSCI_CTRL1_TEIE_MASK                     0x80u
#define QSCI_CTRL1_TEIE_SHIFT                    7
#define QSCI_CTRL1_PT_MASK                       0x100u
#define QSCI_CTRL1_PT_SHIFT                      8
#define QSCI_CTRL1_PE_MASK                       0x200u
#define QSCI_CTRL1_PE_SHIFT                      9
#define QSCI_CTRL1_POL_MASK                      0x400u
#define QSCI_CTRL1_POL_SHIFT                     10
#define QSCI_CTRL1_WAKE_MASK                     0x800u
#define QSCI_CTRL1_WAKE_SHIFT                    11
#define QSCI_CTRL1_M_MASK                        0x1000u
#define QSCI_CTRL1_M_SHIFT                       12
#define QSCI_CTRL1_RSRC_MASK                     0x2000u
#define QSCI_CTRL1_RSRC_SHIFT                    13
#define QSCI_CTRL1_SWAI_MASK                     0x4000u
#define QSCI_CTRL1_SWAI_SHIFT                    14
#define QSCI_CTRL1_LOOP_MASK                     0x8000u
#define QSCI_CTRL1_LOOP_SHIFT                    15
/* CTRL2 Bit Fields */
#define QSCI_CTRL2_RDE_MASK                      0x1u
#define QSCI_CTRL2_RDE_SHIFT                     0
#define QSCI_CTRL2_TDE_MASK                      0x2u
#define QSCI_CTRL2_TDE_SHIFT                     1
#define QSCI_CTRL2_LINMODE_MASK                  0x8u
#define QSCI_CTRL2_LINMODE_SHIFT                 3
#define QSCI_CTRL2_RIEIE_MASK                    0x10u
#define QSCI_CTRL2_RIEIE_SHIFT                   4
#define QSCI_CTRL2_FIFO_EN_MASK                  0x20u
#define QSCI_CTRL2_FIFO_EN_SHIFT                 5
#define QSCI_CTRL2_RFWM_MASK                     0xC0u
#define QSCI_CTRL2_RFWM_SHIFT                    6
#define QSCI_CTRL2_RFWM(x)                       (((uint16_t)(((uint16_t)(x))<<QSCI_CTRL2_RFWM_SHIFT))&QSCI_CTRL2_RFWM_MASK)
#define QSCI_CTRL2_RFCNT_MASK                    0x700u
#define QSCI_CTRL2_RFCNT_SHIFT                   8
#define QSCI_CTRL2_RFCNT(x)                      (((uint16_t)(((uint16_t)(x))<<QSCI_CTRL2_RFCNT_SHIFT))&QSCI_CTRL2_RFCNT_MASK)
#define QSCI_CTRL2_TFWM_MASK                     0x1800u
#define QSCI_CTRL2_TFWM_SHIFT                    11
#define QSCI_CTRL2_TFWM(x)                       (((uint16_t)(((uint16_t)(x))<<QSCI_CTRL2_TFWM_SHIFT))&QSCI_CTRL2_TFWM_MASK)
#define QSCI_CTRL2_TFCNT_MASK                    0xE000u
#define QSCI_CTRL2_TFCNT_SHIFT                   13
#define QSCI_CTRL2_TFCNT(x)                      (((uint16_t)(((uint16_t)(x))<<QSCI_CTRL2_TFCNT_SHIFT))&QSCI_CTRL2_TFCNT_MASK)
/* STAT Bit Fields */
#define QSCI_STAT_RAF_MASK                       0x1u
#define QSCI_STAT_RAF_SHIFT                      0
#define QSCI_STAT_RDMA_MASK                      0x2u
#define QSCI_STAT_RDMA_SHIFT                     1
#define QSCI_STAT_TDMA_MASK                      0x4u
#define QSCI_STAT_TDMA_SHIFT                     2
#define QSCI_STAT_LSE_MASK                       0x8u
#define QSCI_STAT_LSE_SHIFT                      3
#define QSCI_STAT_RIEF_MASK                      0x10u
#define QSCI_STAT_RIEF_SHIFT                     4
#define QSCI_STAT_PF_MASK                        0x100u
#define QSCI_STAT_PF_SHIFT                       8
#define QSCI_STAT_FE_MASK                        0x200u
#define QSCI_STAT_FE_SHIFT                       9
#define QSCI_STAT_NF_MASK                        0x400u
#define QSCI_STAT_NF_SHIFT                       10
#define QSCI_STAT_OR_MASK                        0x800u
#define QSCI_STAT_OR_SHIFT                       11
#define QSCI_STAT_RIDLE_MASK                     0x1000u
#define QSCI_STAT_RIDLE_SHIFT                    12
#define QSCI_STAT_RDRF_MASK                      0x2000u
#define QSCI_STAT_RDRF_SHIFT                     13
#define QSCI_STAT_TIDLE_MASK                     0x4000u
#define QSCI_STAT_TIDLE_SHIFT                    14
#define QSCI_STAT_TDRE_MASK                      0x8000u
#define QSCI_STAT_TDRE_SHIFT                     15
/* DATA Bit Fields */
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK     0x1FFu
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT    0
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA(x)       (((uint16_t)(((uint16_t)(x))<<QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT))&QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK)
/* CTRL3 Bit Fields */
#define QSCI_CTRL3_SHEN_MASK                     0x1u
#define QSCI_CTRL3_SHEN_SHIFT                    0
#define QSCI_CTRL3_SBRH_MASK                     0xE000u
#define QSCI_CTRL3_SBRH_SHIFT                    13
#define QSCI_CTRL3_SBRH(x)                       (((uint16_t)(((uint16_t)(x))<<QSCI_CTRL3_SBRH_SHIFT))&QSCI_CTRL3_SBRH_MASK)

/*!
 * @}
 */ /* end of group QSCI_Register_Masks */


/* QSCI - Peripheral instance base addresses */
/** Peripheral QSCI0 base pointer */
#define QSCI0_BASE_PTR                           ((QSCI_MemMapPtr)0xE080u)
/** Peripheral QSCI1 base pointer */
#define QSCI1_BASE_PTR                           ((QSCI_MemMapPtr)0xE090u)
/** Array initializer of QSCI peripheral base pointers */
#define QSCI_BASE_PTRS                           { QSCI0_BASE_PTR, QSCI1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- QSCI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Register_Accessor_Macros QSCI - Register accessor macros
 * @{
 */


/* QSCI - Register instance definitions */
/* QSCI0 */
#define QSCI0_RATE                               QSCI_RATE_REG(QSCI0_BASE_PTR)
#define QSCI0_CTRL1                              QSCI_CTRL1_REG(QSCI0_BASE_PTR)
#define QSCI0_CTRL2                              QSCI_CTRL2_REG(QSCI0_BASE_PTR)
#define QSCI0_STAT                               QSCI_STAT_REG(QSCI0_BASE_PTR)
#define QSCI0_DATA                               QSCI_DATA_REG(QSCI0_BASE_PTR)
#define QSCI0_CTRL3                              QSCI_CTRL3_REG(QSCI0_BASE_PTR)
/* QSCI1 */
#define QSCI1_RATE                               QSCI_RATE_REG(QSCI1_BASE_PTR)
#define QSCI1_CTRL1                              QSCI_CTRL1_REG(QSCI1_BASE_PTR)
#define QSCI1_CTRL2                              QSCI_CTRL2_REG(QSCI1_BASE_PTR)
#define QSCI1_STAT                               QSCI_STAT_REG(QSCI1_BASE_PTR)
#define QSCI1_DATA                               QSCI_DATA_REG(QSCI1_BASE_PTR)
#define QSCI1_CTRL3                              QSCI_CTRL3_REG(QSCI1_BASE_PTR)

/*!
 * @}
 */ /* end of group QSCI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group QSCI_Peripheral */


/* ----------------------------------------------------------------------------
   -- QSPI
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Peripheral QSPI
 * @{
 */

/** QSPI - Peripheral register structure */
typedef struct QSPI_MemMap {
  uint16_t SPSCR;                                  /**< SPI Status and Control Register, offset: 0x0 */
  uint16_t SPDSR;                                  /**< SPI Data Size and Control Register, offset: 0x1 */
  uint16_t SPDRR;                                  /**< SPI Data Receive Register, offset: 0x2 */
  uint16_t SPDTR;                                  /**< SPI Data Transmit Register, offset: 0x3 */
  uint16_t SPFIFO;                                 /**< SPI FIFO Control Register, offset: 0x4 */
  uint16_t SPWAIT;                                 /**< SPI Word Delay Register, offset: 0x5 */
  uint16_t SPCTL2;                                 /**< SPI Control Register 2, offset: 0x6 */
} volatile *QSPI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- QSPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Register_Accessor_Macros QSPI - Register accessor macros
 * @{
 */


/* QSPI - Register accessors */
#define QSPI_SPSCR_REG(base)                     ((base)->SPSCR)
#define QSPI_SPDSR_REG(base)                     ((base)->SPDSR)
#define QSPI_SPDRR_REG(base)                     ((base)->SPDRR)
#define QSPI_SPDTR_REG(base)                     ((base)->SPDTR)
#define QSPI_SPFIFO_REG(base)                    ((base)->SPFIFO)
#define QSPI_SPWAIT_REG(base)                    ((base)->SPWAIT)
#define QSPI_SPCTL2_REG(base)                    ((base)->SPCTL2)

/*!
 * @}
 */ /* end of group QSPI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- QSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Register_Masks QSPI Register Masks
 * @{
 */

/* SPSCR Bit Fields */
#define QSPI_SPSCR_SPTE_MASK                     0x1u
#define QSPI_SPSCR_SPTE_SHIFT                    0
#define QSPI_SPSCR_MODF_MASK                     0x2u
#define QSPI_SPSCR_MODF_SHIFT                    1
#define QSPI_SPSCR_OVRF_MASK                     0x4u
#define QSPI_SPSCR_OVRF_SHIFT                    2
#define QSPI_SPSCR_SPRF_MASK                     0x8u
#define QSPI_SPSCR_SPRF_SHIFT                    3
#define QSPI_SPSCR_SPTIE_MASK                    0x10u
#define QSPI_SPSCR_SPTIE_SHIFT                   4
#define QSPI_SPSCR_SPE_MASK                      0x20u
#define QSPI_SPSCR_SPE_SHIFT                     5
#define QSPI_SPSCR_CPHA_MASK                     0x40u
#define QSPI_SPSCR_CPHA_SHIFT                    6
#define QSPI_SPSCR_CPOL_MASK                     0x80u
#define QSPI_SPSCR_CPOL_SHIFT                    7
#define QSPI_SPSCR_SPMSTR_MASK                   0x100u
#define QSPI_SPSCR_SPMSTR_SHIFT                  8
#define QSPI_SPSCR_SPRIE_MASK                    0x200u
#define QSPI_SPSCR_SPRIE_SHIFT                   9
#define QSPI_SPSCR_MODFEN_MASK                   0x400u
#define QSPI_SPSCR_MODFEN_SHIFT                  10
#define QSPI_SPSCR_ERRIE_MASK                    0x800u
#define QSPI_SPSCR_ERRIE_SHIFT                   11
#define QSPI_SPSCR_DSO_MASK                      0x1000u
#define QSPI_SPSCR_DSO_SHIFT                     12
#define QSPI_SPSCR_SPR_MASK                      0xE000u
#define QSPI_SPSCR_SPR_SHIFT                     13
#define QSPI_SPSCR_SPR(x)                        (((uint16_t)(((uint16_t)(x))<<QSPI_SPSCR_SPR_SHIFT))&QSPI_SPSCR_SPR_MASK)
/* SPDSR Bit Fields */
#define QSPI_SPDSR_DS_MASK                       0xFu
#define QSPI_SPDSR_DS_SHIFT                      0
#define QSPI_SPDSR_DS(x)                         (((uint16_t)(((uint16_t)(x))<<QSPI_SPDSR_DS_SHIFT))&QSPI_SPDSR_DS_MASK)
#define QSPI_SPDSR_SPR3_MASK                     0x10u
#define QSPI_SPDSR_SPR3_SHIFT                    4
#define QSPI_SPDSR_SSB_OVER_MASK                 0x20u
#define QSPI_SPDSR_SSB_OVER_SHIFT                5
#define QSPI_SPDSR_SSB_STRB_MASK                 0x40u
#define QSPI_SPDSR_SSB_STRB_SHIFT                6
#define QSPI_SPDSR_SSB_DDR_MASK                  0x80u
#define QSPI_SPDSR_SSB_DDR_SHIFT                 7
#define QSPI_SPDSR_SSB_AUTO_MASK                 0x100u
#define QSPI_SPDSR_SSB_AUTO_SHIFT                8
#define QSPI_SPDSR_SSB_ODM_MASK                  0x200u
#define QSPI_SPDSR_SSB_ODM_SHIFT                 9
#define QSPI_SPDSR_SSB_DATA_MASK                 0x400u
#define QSPI_SPDSR_SSB_DATA_SHIFT                10
#define QSPI_SPDSR_SSB_IN_MASK                   0x800u
#define QSPI_SPDSR_SSB_IN_SHIFT                  11
#define QSPI_SPDSR_BD2X_MASK                     0x1000u
#define QSPI_SPDSR_BD2X_SHIFT                    12
#define QSPI_SPDSR_RDMAEN_MASK                   0x2000u
#define QSPI_SPDSR_RDMAEN_SHIFT                  13
#define QSPI_SPDSR_TDMAEN_MASK                   0x4000u
#define QSPI_SPDSR_TDMAEN_SHIFT                  14
#define QSPI_SPDSR_WOM_MASK                      0x8000u
#define QSPI_SPDSR_WOM_SHIFT                     15
/* SPDRR Bit Fields */
#define QSPI_SPDRR_R0_MASK                       0x1u
#define QSPI_SPDRR_R0_SHIFT                      0
#define QSPI_SPDRR_R1_MASK                       0x2u
#define QSPI_SPDRR_R1_SHIFT                      1
#define QSPI_SPDRR_R2_MASK                       0x4u
#define QSPI_SPDRR_R2_SHIFT                      2
#define QSPI_SPDRR_R3_MASK                       0x8u
#define QSPI_SPDRR_R3_SHIFT                      3
#define QSPI_SPDRR_R4_MASK                       0x10u
#define QSPI_SPDRR_R4_SHIFT                      4
#define QSPI_SPDRR_R5_MASK                       0x20u
#define QSPI_SPDRR_R5_SHIFT                      5
#define QSPI_SPDRR_R6_MASK                       0x40u
#define QSPI_SPDRR_R6_SHIFT                      6
#define QSPI_SPDRR_R7_MASK                       0x80u
#define QSPI_SPDRR_R7_SHIFT                      7
#define QSPI_SPDRR_R8_MASK                       0x100u
#define QSPI_SPDRR_R8_SHIFT                      8
#define QSPI_SPDRR_R9_MASK                       0x200u
#define QSPI_SPDRR_R9_SHIFT                      9
#define QSPI_SPDRR_R10_MASK                      0x400u
#define QSPI_SPDRR_R10_SHIFT                     10
#define QSPI_SPDRR_R11_MASK                      0x800u
#define QSPI_SPDRR_R11_SHIFT                     11
#define QSPI_SPDRR_R12_MASK                      0x1000u
#define QSPI_SPDRR_R12_SHIFT                     12
#define QSPI_SPDRR_R13_MASK                      0x2000u
#define QSPI_SPDRR_R13_SHIFT                     13
#define QSPI_SPDRR_R14_MASK                      0x4000u
#define QSPI_SPDRR_R14_SHIFT                     14
#define QSPI_SPDRR_R15_MASK                      0x8000u
#define QSPI_SPDRR_R15_SHIFT                     15
/* SPDTR Bit Fields */
#define QSPI_SPDTR_T0_MASK                       0x1u
#define QSPI_SPDTR_T0_SHIFT                      0
#define QSPI_SPDTR_T1_MASK                       0x2u
#define QSPI_SPDTR_T1_SHIFT                      1
#define QSPI_SPDTR_T2_MASK                       0x4u
#define QSPI_SPDTR_T2_SHIFT                      2
#define QSPI_SPDTR_T3_MASK                       0x8u
#define QSPI_SPDTR_T3_SHIFT                      3
#define QSPI_SPDTR_T4_MASK                       0x10u
#define QSPI_SPDTR_T4_SHIFT                      4
#define QSPI_SPDTR_T5_MASK                       0x20u
#define QSPI_SPDTR_T5_SHIFT                      5
#define QSPI_SPDTR_T6_MASK                       0x40u
#define QSPI_SPDTR_T6_SHIFT                      6
#define QSPI_SPDTR_T7_MASK                       0x80u
#define QSPI_SPDTR_T7_SHIFT                      7
#define QSPI_SPDTR_T8_MASK                       0x100u
#define QSPI_SPDTR_T8_SHIFT                      8
#define QSPI_SPDTR_T9_MASK                       0x200u
#define QSPI_SPDTR_T9_SHIFT                      9
#define QSPI_SPDTR_T10_MASK                      0x400u
#define QSPI_SPDTR_T10_SHIFT                     10
#define QSPI_SPDTR_T11_MASK                      0x800u
#define QSPI_SPDTR_T11_SHIFT                     11
#define QSPI_SPDTR_T12_MASK                      0x1000u
#define QSPI_SPDTR_T12_SHIFT                     12
#define QSPI_SPDTR_T13_MASK                      0x2000u
#define QSPI_SPDTR_T13_SHIFT                     13
#define QSPI_SPDTR_T14_MASK                      0x4000u
#define QSPI_SPDTR_T14_SHIFT                     14
#define QSPI_SPDTR_T15_MASK                      0x8000u
#define QSPI_SPDTR_T15_SHIFT                     15
/* SPFIFO Bit Fields */
#define QSPI_SPFIFO_FIFO_ENA_MASK                0x1u
#define QSPI_SPFIFO_FIFO_ENA_SHIFT               0
#define QSPI_SPFIFO_RFWM_MASK                    0xCu
#define QSPI_SPFIFO_RFWM_SHIFT                   2
#define QSPI_SPFIFO_RFWM(x)                      (((uint16_t)(((uint16_t)(x))<<QSPI_SPFIFO_RFWM_SHIFT))&QSPI_SPFIFO_RFWM_MASK)
#define QSPI_SPFIFO_TFWM_MASK                    0x60u
#define QSPI_SPFIFO_TFWM_SHIFT                   5
#define QSPI_SPFIFO_TFWM(x)                      (((uint16_t)(((uint16_t)(x))<<QSPI_SPFIFO_TFWM_SHIFT))&QSPI_SPFIFO_TFWM_MASK)
#define QSPI_SPFIFO_RFCNT_MASK                   0x700u
#define QSPI_SPFIFO_RFCNT_SHIFT                  8
#define QSPI_SPFIFO_RFCNT(x)                     (((uint16_t)(((uint16_t)(x))<<QSPI_SPFIFO_RFCNT_SHIFT))&QSPI_SPFIFO_RFCNT_MASK)
#define QSPI_SPFIFO_TFCNT_MASK                   0x7000u
#define QSPI_SPFIFO_TFCNT_SHIFT                  12
#define QSPI_SPFIFO_TFCNT(x)                     (((uint16_t)(((uint16_t)(x))<<QSPI_SPFIFO_TFCNT_SHIFT))&QSPI_SPFIFO_TFCNT_MASK)
/* SPWAIT Bit Fields */
#define QSPI_SPWAIT_WAIT_MASK                    0x1FFFu
#define QSPI_SPWAIT_WAIT_SHIFT                   0
#define QSPI_SPWAIT_WAIT(x)                      (((uint16_t)(((uint16_t)(x))<<QSPI_SPWAIT_WAIT_SHIFT))&QSPI_SPWAIT_WAIT_MASK)
/* SPCTL2 Bit Fields */
#define QSPI_SPCTL2_SHEN_MASK                    0x1u
#define QSPI_SPCTL2_SHEN_SHIFT                   0

/*!
 * @}
 */ /* end of group QSPI_Register_Masks */


/* QSPI - Peripheral instance base addresses */
/** Peripheral QSPI0 base pointer */
#define QSPI0_BASE_PTR                           ((QSPI_MemMapPtr)0xE0B0u)
/** Peripheral QSPI1 base pointer */
#define QSPI1_BASE_PTR                           ((QSPI_MemMapPtr)0xE0C0u)
/** Array initializer of QSPI peripheral base pointers */
#define QSPI_BASE_PTRS                           { QSPI0_BASE_PTR, QSPI1_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- QSPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Register_Accessor_Macros QSPI - Register accessor macros
 * @{
 */


/* QSPI - Register instance definitions */
/* QSPI0 */
#define QSPI0_SPSCR                              QSPI_SPSCR_REG(QSPI0_BASE_PTR)
#define QSPI0_SPDSR                              QSPI_SPDSR_REG(QSPI0_BASE_PTR)
#define QSPI0_SPDRR                              QSPI_SPDRR_REG(QSPI0_BASE_PTR)
#define QSPI0_SPDTR                              QSPI_SPDTR_REG(QSPI0_BASE_PTR)
#define QSPI0_SPFIFO                             QSPI_SPFIFO_REG(QSPI0_BASE_PTR)
#define QSPI0_SPWAIT                             QSPI_SPWAIT_REG(QSPI0_BASE_PTR)
#define QSPI0_SPCTL2                             QSPI_SPCTL2_REG(QSPI0_BASE_PTR)
/* QSPI1 */
#define QSPI1_SPSCR                              QSPI_SPSCR_REG(QSPI1_BASE_PTR)
#define QSPI1_SPDSR                              QSPI_SPDSR_REG(QSPI1_BASE_PTR)
#define QSPI1_SPDRR                              QSPI_SPDRR_REG(QSPI1_BASE_PTR)
#define QSPI1_SPDTR                              QSPI_SPDTR_REG(QSPI1_BASE_PTR)
#define QSPI1_SPFIFO                             QSPI_SPFIFO_REG(QSPI1_BASE_PTR)
#define QSPI1_SPWAIT                             QSPI_SPWAIT_REG(QSPI1_BASE_PTR)
#define QSPI1_SPCTL2                             QSPI_SPCTL2_REG(QSPI1_BASE_PTR)

/*!
 * @}
 */ /* end of group QSPI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group QSPI_Peripheral */


/* ----------------------------------------------------------------------------
   -- SIM
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral SIM
 * @{
 */

/** SIM - Peripheral register structure */
typedef struct SIM_MemMap {
  uint16_t CTRL;                                   /**< Control Register, offset: 0x0 */
  uint16_t RSTAT;                                  /**< Reset Status Register, offset: 0x1 */
  uint16_t RESERVED_0[4];
  uint16_t MSHID;                                  /**< Most Significant Half of JTAG ID, offset: 0x6 */
  uint16_t LSHID;                                  /**< Least Significant Half of JTAG ID, offset: 0x7 */
  uint16_t PWR;                                    /**< Power Control Register, offset: 0x8 */
  uint16_t RESERVED_1[1];
  uint16_t CLKOUT;                                 /**< Clock Output Select Register, offset: 0xA */
  uint16_t PCR;                                    /**< Peripheral Clock Rate Register, offset: 0xB */
  uint16_t PCE0;                                   /**< Peripheral Clock Enable Register 0, offset: 0xC */
  uint16_t PCE1;                                   /**< Peripheral Clock Enable Register 1, offset: 0xD */
  uint16_t PCE2;                                   /**< Peripheral Clock Enable Register 2, offset: 0xE */
  uint16_t PCE3;                                   /**< Peripheral Clock Enable Register 3, offset: 0xF */
  uint16_t SD0;                                    /**< STOP Disable Register 0, offset: 0x10 */
  uint16_t SD1;                                    /**< Peripheral Clock STOP Disable Register 1, offset: 0x11 */
  uint16_t SD2;                                    /**< Peripheral Clock STOP Disable Register 2, offset: 0x12 */
  uint16_t SD3;                                    /**< Peripheral Clock STOP Disable Register 3, offset: 0x13 */
  uint16_t IOSAHI;                                 /**< I/O Short Address Location Register, offset: 0x14 */
  uint16_t IOSALO;                                 /**< I/O Short Address Location Register, offset: 0x15 */
  uint16_t PROT;                                   /**< Protection Register, offset: 0x16 */
  uint16_t GPSAL;                                  /**< GPIOA LSBs Peripheral Select Register, offset: 0x17 */
  uint16_t GPSBL;                                  /**< GPIOB LSBs Peripheral Select Register, offset: 0x18 */
  uint16_t GPSCL;                                  /**< GPIOC LSBs Peripheral Select Register, offset: 0x19 */
  uint16_t GPSCH;                                  /**< GPIOC MSBs Peripheral Select Register, offset: 0x1A */
  uint16_t RESERVED_2[1];
  uint16_t GPSEL;                                  /**< GPIOE LSBs Peripheral Select Register, offset: 0x1C */
  uint16_t RESERVED_3[1];
  uint16_t GPSFL;                                  /**< GPIOF LSBs Peripheral Select Register, offset: 0x1E */
  uint16_t GPSFH;                                  /**< GPIOF MSBs Peripheral Select Register, offset: 0x1F */
  uint16_t RESERVED_4[2];
  uint16_t IPSn;                                   /**< Internal Peripheral Select Register, offset: 0x22 */
  uint16_t MISC0;                                  /**< Miscellaneous Register 0, offset: 0x23 */
  uint16_t PSWR0;                                  /**< Peripheral Software Reset Register 0, offset: 0x24 */
  uint16_t PSWR1;                                  /**< Peripheral Software Reset Register 1, offset: 0x25 */
  uint16_t PSWR2;                                  /**< Peripheral Software Reset Register 2, offset: 0x26 */
  uint16_t PSWR3;                                  /**< Peripheral Software Reset Register 3, offset: 0x27 */
  uint16_t PWRMODE;                                /**< Power Mode Register, offset: 0x28 */
  uint16_t RESERVED_5[3];
  uint16_t NVMOPT2H;                               /**< Non-Volatile Memory Option Register 2 (High), offset: 0x2C */
  uint16_t NVMOPT2L;                               /**< Non-Volatile Memory Option Register 2 (Low), offset: 0x2D */
  uint16_t RESERVED_6[23];
  uint16_t SCR0;                                   /**< Software Control Register, offset: 0x45 */
  uint16_t SCR1;                                   /**< Software Control Register 1, offset: 0x46 */
  uint16_t SCR2;                                   /**< Software Control Register 2, offset: 0x47 */
  uint16_t SCR3;                                   /**< Software Control Register 3, offset: 0x48 */
  uint16_t SCR4;                                   /**< Software Control Register 4, offset: 0x49 */
  uint16_t SCR5;                                   /**< Software Control Register, offset: 0x4A */
  uint16_t SCR6;                                   /**< Software Control Register 5, offset: 0x4B */
  uint16_t SCR7;                                   /**< Software Control Register 6, offset: 0x4C */
} volatile *SIM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register accessors */
#define SIM_CTRL_REG(base)                       ((base)->CTRL)
#define SIM_RSTAT_REG(base)                      ((base)->RSTAT)
#define SIM_MSHID_REG(base)                      ((base)->MSHID)
#define SIM_LSHID_REG(base)                      ((base)->LSHID)
#define SIM_PWR_REG(base)                        ((base)->PWR)
#define SIM_CLKOUT_REG(base)                     ((base)->CLKOUT)
#define SIM_PCR_REG(base)                        ((base)->PCR)
#define SIM_PCE0_REG(base)                       ((base)->PCE0)
#define SIM_PCE1_REG(base)                       ((base)->PCE1)
#define SIM_PCE2_REG(base)                       ((base)->PCE2)
#define SIM_PCE3_REG(base)                       ((base)->PCE3)
#define SIM_SD0_REG(base)                        ((base)->SD0)
#define SIM_SD1_REG(base)                        ((base)->SD1)
#define SIM_SD2_REG(base)                        ((base)->SD2)
#define SIM_SD3_REG(base)                        ((base)->SD3)
#define SIM_IOSAHI_REG(base)                     ((base)->IOSAHI)
#define SIM_IOSALO_REG(base)                     ((base)->IOSALO)
#define SIM_PROT_REG(base)                       ((base)->PROT)
#define SIM_GPSAL_REG(base)                      ((base)->GPSAL)
#define SIM_GPSBL_REG(base)                      ((base)->GPSBL)
#define SIM_GPSCL_REG(base)                      ((base)->GPSCL)
#define SIM_GPSCH_REG(base)                      ((base)->GPSCH)
#define SIM_GPSEL_REG(base)                      ((base)->GPSEL)
#define SIM_GPSFL_REG(base)                      ((base)->GPSFL)
#define SIM_GPSFH_REG(base)                      ((base)->GPSFH)
#define SIM_IPSn_REG(base)                       ((base)->IPSn)
#define SIM_MISC0_REG(base)                      ((base)->MISC0)
#define SIM_PSWR0_REG(base)                      ((base)->PSWR0)
#define SIM_PSWR1_REG(base)                      ((base)->PSWR1)
#define SIM_PSWR2_REG(base)                      ((base)->PSWR2)
#define SIM_PSWR3_REG(base)                      ((base)->PSWR3)
#define SIM_PWRMODE_REG(base)                    ((base)->PWRMODE)
#define SIM_NVMOPT2H_REG(base)                   ((base)->NVMOPT2H)
#define SIM_NVMOPT2L_REG(base)                   ((base)->NVMOPT2L)
#define SIM_SCR0_REG(base)                       ((base)->SCR0)
#define SIM_SCR1_REG(base)                       ((base)->SCR1)
#define SIM_SCR2_REG(base)                       ((base)->SCR2)
#define SIM_SCR3_REG(base)                       ((base)->SCR3)
#define SIM_SCR4_REG(base)                       ((base)->SCR4)
#define SIM_SCR5_REG(base)                       ((base)->SCR5)
#define SIM_SCR6_REG(base)                       ((base)->SCR6)
#define SIM_SCR7_REG(base)                       ((base)->SCR7)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define SIM_CTRL_WAIT_disable_MASK               0x3u
#define SIM_CTRL_WAIT_disable_SHIFT              0
#define SIM_CTRL_WAIT_disable(x)                 (((uint16_t)(((uint16_t)(x))<<SIM_CTRL_WAIT_disable_SHIFT))&SIM_CTRL_WAIT_disable_MASK)
#define SIM_CTRL_STOP_disable_MASK               0xCu
#define SIM_CTRL_STOP_disable_SHIFT              2
#define SIM_CTRL_STOP_disable(x)                 (((uint16_t)(((uint16_t)(x))<<SIM_CTRL_STOP_disable_SHIFT))&SIM_CTRL_STOP_disable_MASK)
#define SIM_CTRL_SWRst_MASK                      0x10u
#define SIM_CTRL_SWRst_SHIFT                     4
#define SIM_CTRL_OnceEbl_MASK                    0x20u
#define SIM_CTRL_OnceEbl_SHIFT                   5
#define SIM_CTRL_DMAEbl_MASK                     0x1C0u
#define SIM_CTRL_DMAEbl_SHIFT                    6
#define SIM_CTRL_DMAEbl(x)                       (((uint16_t)(((uint16_t)(x))<<SIM_CTRL_DMAEbl_SHIFT))&SIM_CTRL_DMAEbl_MASK)
#define SIM_CTRL_RST_FILT_MASK                   0x400u
#define SIM_CTRL_RST_FILT_SHIFT                  10
/* RSTAT Bit Fields */
#define SIM_RSTAT_POR_MASK                       0x4u
#define SIM_RSTAT_POR_SHIFT                      2
#define SIM_RSTAT_EXTR_MASK                      0x8u
#define SIM_RSTAT_EXTR_SHIFT                     3
#define SIM_RSTAT_COP_LOR_MASK                   0x10u
#define SIM_RSTAT_COP_LOR_SHIFT                  4
#define SIM_RSTAT_COP_CPU_MASK                   0x20u
#define SIM_RSTAT_COP_CPU_SHIFT                  5
#define SIM_RSTAT_COP_WNDOW_MASK                 0x40u
#define SIM_RSTAT_COP_WNDOW_SHIFT                6
#define SIM_RSTAT_SWR_MASK                       0x80u
#define SIM_RSTAT_SWR_SHIFT                      7
/* PWR Bit Fields */
#define SIM_PWR_LRSTDBY_MASK                     0x3u
#define SIM_PWR_LRSTDBY_SHIFT                    0
#define SIM_PWR_LRSTDBY(x)                       (((uint16_t)(((uint16_t)(x))<<SIM_PWR_LRSTDBY_SHIFT))&SIM_PWR_LRSTDBY_MASK)
#define SIM_PWR_SR27STDBY_MASK                   0xCu
#define SIM_PWR_SR27STDBY_SHIFT                  2
#define SIM_PWR_SR27STDBY(x)                     (((uint16_t)(((uint16_t)(x))<<SIM_PWR_SR27STDBY_SHIFT))&SIM_PWR_SR27STDBY_MASK)
#define SIM_PWR_SR27PDN_MASK                     0x30u
#define SIM_PWR_SR27PDN_SHIFT                    4
#define SIM_PWR_SR27PDN(x)                       (((uint16_t)(((uint16_t)(x))<<SIM_PWR_SR27PDN_SHIFT))&SIM_PWR_SR27PDN_MASK)
#define SIM_PWR_SR12STDBY_MASK                   0xC0u
#define SIM_PWR_SR12STDBY_SHIFT                  6
#define SIM_PWR_SR12STDBY(x)                     (((uint16_t)(((uint16_t)(x))<<SIM_PWR_SR12STDBY_SHIFT))&SIM_PWR_SR12STDBY_MASK)
/* CLKOUT Bit Fields */
#define SIM_CLKOUT_CLKOSEL0_MASK                 0x7u
#define SIM_CLKOUT_CLKOSEL0_SHIFT                0
#define SIM_CLKOUT_CLKOSEL0(x)                   (((uint16_t)(((uint16_t)(x))<<SIM_CLKOUT_CLKOSEL0_SHIFT))&SIM_CLKOUT_CLKOSEL0_MASK)
#define SIM_CLKOUT_CLKDIS0_MASK                  0x20u
#define SIM_CLKOUT_CLKDIS0_SHIFT                 5
#define SIM_CLKOUT_CLKOSEL1_MASK                 0x380u
#define SIM_CLKOUT_CLKOSEL1_SHIFT                7
#define SIM_CLKOUT_CLKOSEL1(x)                   (((uint16_t)(((uint16_t)(x))<<SIM_CLKOUT_CLKOSEL1_SHIFT))&SIM_CLKOUT_CLKOSEL1_MASK)
#define SIM_CLKOUT_CLKDIS1_MASK                  0x1000u
#define SIM_CLKOUT_CLKDIS1_SHIFT                 12
#define SIM_CLKOUT_CLKODIV_MASK                  0xE000u
#define SIM_CLKOUT_CLKODIV_SHIFT                 13
#define SIM_CLKOUT_CLKODIV(x)                    (((uint16_t)(((uint16_t)(x))<<SIM_CLKOUT_CLKODIV_SHIFT))&SIM_CLKOUT_CLKODIV_MASK)
/* PCR Bit Fields */
#define SIM_PCR_IIC_FILT_MASK                    0x200u
#define SIM_PCR_IIC_FILT_SHIFT                   9
#define SIM_PCR_PWM_MASK                         0x400u
#define SIM_PCR_PWM_SHIFT                        10
#define SIM_PCR_TMR_MASK                         0x800u
#define SIM_PCR_TMR_SHIFT                        11
#define SIM_PCR_SCI1_CR_MASK                     0x1000u
#define SIM_PCR_SCI1_CR_SHIFT                    12
#define SIM_PCR_SCI0_CR_MASK                     0x2000u
#define SIM_PCR_SCI0_CR_SHIFT                    13
/* PCE0 Bit Fields */
#define SIM_PCE0_GPIOF_MASK                      0x2u
#define SIM_PCE0_GPIOF_SHIFT                     1
#define SIM_PCE0_GPIOE_MASK                      0x4u
#define SIM_PCE0_GPIOE_SHIFT                     2
#define SIM_PCE0_GPIOD_MASK                      0x8u
#define SIM_PCE0_GPIOD_SHIFT                     3
#define SIM_PCE0_GPIOC_MASK                      0x10u
#define SIM_PCE0_GPIOC_SHIFT                     4
#define SIM_PCE0_GPIOB_MASK                      0x20u
#define SIM_PCE0_GPIOB_SHIFT                     5
#define SIM_PCE0_GPIOA_MASK                      0x40u
#define SIM_PCE0_GPIOA_SHIFT                     6
#define SIM_PCE0_TA3_MASK                        0x1000u
#define SIM_PCE0_TA3_SHIFT                       12
#define SIM_PCE0_TA2_MASK                        0x2000u
#define SIM_PCE0_TA2_SHIFT                       13
#define SIM_PCE0_TA1_MASK                        0x4000u
#define SIM_PCE0_TA1_SHIFT                       14
#define SIM_PCE0_TA0_MASK                        0x8000u
#define SIM_PCE0_TA0_SHIFT                       15
/* PCE1 Bit Fields */
#define SIM_PCE1_MSCAN_MASK                      0x1u
#define SIM_PCE1_MSCAN_SHIFT                     0
#define SIM_PCE1_IIC0_MASK                       0x40u
#define SIM_PCE1_IIC0_SHIFT                      6
#define SIM_PCE1_QSPI1_MASK                      0x100u
#define SIM_PCE1_QSPI1_SHIFT                     8
#define SIM_PCE1_QSPI0_MASK                      0x200u
#define SIM_PCE1_QSPI0_SHIFT                     9
#define SIM_PCE1_SCI1_MASK                       0x800u
#define SIM_PCE1_SCI1_SHIFT                      11
#define SIM_PCE1_SCI0_MASK                       0x1000u
#define SIM_PCE1_SCI0_SHIFT                      12
#define SIM_PCE1_DACA_MASK                       0x2000u
#define SIM_PCE1_DACA_SHIFT                      13
#define SIM_PCE1_DACB_MASK                       0x4000u
#define SIM_PCE1_DACB_SHIFT                      14
/* PCE2 Bit Fields */
#define SIM_PCE2_PIT1_MASK                       0x4u
#define SIM_PCE2_PIT1_SHIFT                      2
#define SIM_PCE2_PIT0_MASK                       0x8u
#define SIM_PCE2_PIT0_SHIFT                      3
#define SIM_PCE2_CRC_MASK                        0x20u
#define SIM_PCE2_CRC_SHIFT                       5
#define SIM_PCE2_CYCADC_MASK                     0x80u
#define SIM_PCE2_CYCADC_SHIFT                    7
#define SIM_PCE2_CMPD_MASK                       0x200u
#define SIM_PCE2_CMPD_SHIFT                      9
#define SIM_PCE2_CMPC_MASK                       0x400u
#define SIM_PCE2_CMPC_SHIFT                      10
#define SIM_PCE2_CMPB_MASK                       0x800u
#define SIM_PCE2_CMPB_SHIFT                      11
#define SIM_PCE2_CMPA_MASK                       0x1000u
#define SIM_PCE2_CMPA_SHIFT                      12
/* PCE3 Bit Fields */
#define SIM_PCE3_PWMACH3_MASK                    0x10u
#define SIM_PCE3_PWMACH3_SHIFT                   4
#define SIM_PCE3_PWMACH2_MASK                    0x20u
#define SIM_PCE3_PWMACH2_SHIFT                   5
#define SIM_PCE3_PWMACH1_MASK                    0x40u
#define SIM_PCE3_PWMACH1_SHIFT                   6
#define SIM_PCE3_PWMACH0_MASK                    0x80u
#define SIM_PCE3_PWMACH0_SHIFT                   7
/* SD0 Bit Fields */
#define SIM_SD0_GPIOF_MASK                       0x2u
#define SIM_SD0_GPIOF_SHIFT                      1
#define SIM_SD0_GPIOE_MASK                       0x4u
#define SIM_SD0_GPIOE_SHIFT                      2
#define SIM_SD0_GPIOD_MASK                       0x8u
#define SIM_SD0_GPIOD_SHIFT                      3
#define SIM_SD0_GPIOC_MASK                       0x10u
#define SIM_SD0_GPIOC_SHIFT                      4
#define SIM_SD0_GPIOB_MASK                       0x20u
#define SIM_SD0_GPIOB_SHIFT                      5
#define SIM_SD0_GPIOA_MASK                       0x40u
#define SIM_SD0_GPIOA_SHIFT                      6
#define SIM_SD0_TA3_MASK                         0x1000u
#define SIM_SD0_TA3_SHIFT                        12
#define SIM_SD0_TA2_MASK                         0x2000u
#define SIM_SD0_TA2_SHIFT                        13
#define SIM_SD0_TA1_MASK                         0x4000u
#define SIM_SD0_TA1_SHIFT                        14
#define SIM_SD0_TA0_MASK                         0x8000u
#define SIM_SD0_TA0_SHIFT                        15
/* SD1 Bit Fields */
#define SIM_SD1_MSCAN_MASK                       0x1u
#define SIM_SD1_MSCAN_SHIFT                      0
#define SIM_SD1_IIC0_MASK                        0x40u
#define SIM_SD1_IIC0_SHIFT                       6
#define SIM_SD1_QSPI1_MASK                       0x100u
#define SIM_SD1_QSPI1_SHIFT                      8
#define SIM_SD1_QSPI0_MASK                       0x200u
#define SIM_SD1_QSPI0_SHIFT                      9
#define SIM_SD1_SCI1_MASK                        0x800u
#define SIM_SD1_SCI1_SHIFT                       11
#define SIM_SD1_SCI0_MASK                        0x1000u
#define SIM_SD1_SCI0_SHIFT                       12
#define SIM_SD1_DACA_MASK                        0x2000u
#define SIM_SD1_DACA_SHIFT                       13
#define SIM_SD1_DACB_MASK                        0x4000u
#define SIM_SD1_DACB_SHIFT                       14
/* SD2 Bit Fields */
#define SIM_SD2_PIT1_MASK                        0x4u
#define SIM_SD2_PIT1_SHIFT                       2
#define SIM_SD2_PIT0_MASK                        0x8u
#define SIM_SD2_PIT0_SHIFT                       3
#define SIM_SD2_CRC_MASK                         0x20u
#define SIM_SD2_CRC_SHIFT                        5
#define SIM_SD2_CYCADC_MASK                      0x80u
#define SIM_SD2_CYCADC_SHIFT                     7
#define SIM_SD2_CMPD_MASK                        0x200u
#define SIM_SD2_CMPD_SHIFT                       9
#define SIM_SD2_CMPC_MASK                        0x400u
#define SIM_SD2_CMPC_SHIFT                       10
#define SIM_SD2_CMPB_MASK                        0x800u
#define SIM_SD2_CMPB_SHIFT                       11
#define SIM_SD2_CMPA_MASK                        0x1000u
#define SIM_SD2_CMPA_SHIFT                       12
/* SD3 Bit Fields */
#define SIM_SD3_PWMACH3_MASK                     0x10u
#define SIM_SD3_PWMACH3_SHIFT                    4
#define SIM_SD3_PWMACH2_MASK                     0x20u
#define SIM_SD3_PWMACH2_SHIFT                    5
#define SIM_SD3_PWMACH1_MASK                     0x40u
#define SIM_SD3_PWMACH1_SHIFT                    6
#define SIM_SD3_PWMACH0_MASK                     0x80u
#define SIM_SD3_PWMACH0_SHIFT                    7
/* IOSAHI Bit Fields */
#define SIM_IOSAHI_ISAL_MASK                     0x3u
#define SIM_IOSAHI_ISAL_SHIFT                    0
#define SIM_IOSAHI_ISAL(x)                       (((uint16_t)(((uint16_t)(x))<<SIM_IOSAHI_ISAL_SHIFT))&SIM_IOSAHI_ISAL_MASK)
/* IOSALO Bit Fields */
#define SIM_IOSALO_ISAL_MASK                     0xFFFFu
#define SIM_IOSALO_ISAL_SHIFT                    0
#define SIM_IOSALO_ISAL(x)                       (((uint16_t)(((uint16_t)(x))<<SIM_IOSALO_ISAL_SHIFT))&SIM_IOSALO_ISAL_MASK)
/* PROT Bit Fields */
#define SIM_PROT_GIPSP_MASK                      0x3u
#define SIM_PROT_GIPSP_SHIFT                     0
#define SIM_PROT_GIPSP(x)                        (((uint16_t)(((uint16_t)(x))<<SIM_PROT_GIPSP_SHIFT))&SIM_PROT_GIPSP_MASK)
#define SIM_PROT_PCEP_MASK                       0xCu
#define SIM_PROT_PCEP_SHIFT                      2
#define SIM_PROT_PCEP(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_PROT_PCEP_SHIFT))&SIM_PROT_PCEP_MASK)
#define SIM_PROT_GDP_MASK                        0x30u
#define SIM_PROT_GDP_SHIFT                       4
#define SIM_PROT_GDP(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_PROT_GDP_SHIFT))&SIM_PROT_GDP_MASK)
#define SIM_PROT_PMODE_MASK                      0xC0u
#define SIM_PROT_PMODE_SHIFT                     6
#define SIM_PROT_PMODE(x)                        (((uint16_t)(((uint16_t)(x))<<SIM_PROT_PMODE_SHIFT))&SIM_PROT_PMODE_MASK)
/* GPSAL Bit Fields */
#define SIM_GPSAL_A0_MASK                        0x1u
#define SIM_GPSAL_A0_SHIFT                       0
/* GPSBL Bit Fields */
#define SIM_GPSBL_B1_MASK                        0x4u
#define SIM_GPSBL_B1_SHIFT                       2
/* GPSCL Bit Fields */
#define SIM_GPSCL_C0_MASK                        0x1u
#define SIM_GPSCL_C0_SHIFT                       0
#define SIM_GPSCL_C2_MASK                        0x30u
#define SIM_GPSCL_C2_SHIFT                       4
#define SIM_GPSCL_C2(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCL_C2_SHIFT))&SIM_GPSCL_C2_MASK)
#define SIM_GPSCL_C3_MASK                        0xC0u
#define SIM_GPSCL_C3_SHIFT                       6
#define SIM_GPSCL_C3(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCL_C3_SHIFT))&SIM_GPSCL_C3_MASK)
#define SIM_GPSCL_C4_MASK                        0x300u
#define SIM_GPSCL_C4_SHIFT                       8
#define SIM_GPSCL_C4(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCL_C4_SHIFT))&SIM_GPSCL_C4_MASK)
#define SIM_GPSCL_C5_MASK                        0x400u
#define SIM_GPSCL_C5_SHIFT                       10
#define SIM_GPSCL_C6_MASK                        0x3000u
#define SIM_GPSCL_C6_SHIFT                       12
#define SIM_GPSCL_C6(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCL_C6_SHIFT))&SIM_GPSCL_C6_MASK)
#define SIM_GPSCL_C7_MASK                        0xC000u
#define SIM_GPSCL_C7_SHIFT                       14
#define SIM_GPSCL_C7(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCL_C7_SHIFT))&SIM_GPSCL_C7_MASK)
/* GPSCH Bit Fields */
#define SIM_GPSCH_C8_MASK                        0x3u
#define SIM_GPSCH_C8_SHIFT                       0
#define SIM_GPSCH_C8(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C8_SHIFT))&SIM_GPSCH_C8_MASK)
#define SIM_GPSCH_C9_MASK                        0xCu
#define SIM_GPSCH_C9_SHIFT                       2
#define SIM_GPSCH_C9(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C9_SHIFT))&SIM_GPSCH_C9_MASK)
#define SIM_GPSCH_C10_MASK                       0x30u
#define SIM_GPSCH_C10_SHIFT                      4
#define SIM_GPSCH_C10(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C10_SHIFT))&SIM_GPSCH_C10_MASK)
#define SIM_GPSCH_C11_MASK                       0xC0u
#define SIM_GPSCH_C11_SHIFT                      6
#define SIM_GPSCH_C11(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C11_SHIFT))&SIM_GPSCH_C11_MASK)
#define SIM_GPSCH_C12_MASK                       0x300u
#define SIM_GPSCH_C12_SHIFT                      8
#define SIM_GPSCH_C12(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C12_SHIFT))&SIM_GPSCH_C12_MASK)
#define SIM_GPSCH_C13_MASK                       0xC00u
#define SIM_GPSCH_C13_SHIFT                      10
#define SIM_GPSCH_C13(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C13_SHIFT))&SIM_GPSCH_C13_MASK)
#define SIM_GPSCH_C14_MASK                       0x3000u
#define SIM_GPSCH_C14_SHIFT                      12
#define SIM_GPSCH_C14(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C14_SHIFT))&SIM_GPSCH_C14_MASK)
#define SIM_GPSCH_C15_MASK                       0xC000u
#define SIM_GPSCH_C15_SHIFT                      14
#define SIM_GPSCH_C15(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_GPSCH_C15_SHIFT))&SIM_GPSCH_C15_MASK)
/* GPSEL Bit Fields */
#define SIM_GPSEL_E4_MASK                        0x100u
#define SIM_GPSEL_E4_SHIFT                       8
#define SIM_GPSEL_E5_MASK                        0x400u
#define SIM_GPSEL_E5_SHIFT                       10
#define SIM_GPSEL_E6_MASK                        0x1000u
#define SIM_GPSEL_E6_SHIFT                       12
#define SIM_GPSEL_E7_MASK                        0x4000u
#define SIM_GPSEL_E7_SHIFT                       14
/* GPSFL Bit Fields */
#define SIM_GPSFL_F0_MASK                        0x3u
#define SIM_GPSFL_F0_SHIFT                       0
#define SIM_GPSFL_F0(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F0_SHIFT))&SIM_GPSFL_F0_MASK)
#define SIM_GPSFL_F1_MASK                        0xCu
#define SIM_GPSFL_F1_SHIFT                       2
#define SIM_GPSFL_F1(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F1_SHIFT))&SIM_GPSFL_F1_MASK)
#define SIM_GPSFL_F2_MASK                        0x30u
#define SIM_GPSFL_F2_SHIFT                       4
#define SIM_GPSFL_F2(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F2_SHIFT))&SIM_GPSFL_F2_MASK)
#define SIM_GPSFL_F3_MASK                        0xC0u
#define SIM_GPSFL_F3_SHIFT                       6
#define SIM_GPSFL_F3(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F3_SHIFT))&SIM_GPSFL_F3_MASK)
#define SIM_GPSFL_F4_MASK                        0x300u
#define SIM_GPSFL_F4_SHIFT                       8
#define SIM_GPSFL_F4(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F4_SHIFT))&SIM_GPSFL_F4_MASK)
#define SIM_GPSFL_F5_MASK                        0xC00u
#define SIM_GPSFL_F5_SHIFT                       10
#define SIM_GPSFL_F5(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F5_SHIFT))&SIM_GPSFL_F5_MASK)
#define SIM_GPSFL_F6_MASK                        0x3000u
#define SIM_GPSFL_F6_SHIFT                       12
#define SIM_GPSFL_F6(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F6_SHIFT))&SIM_GPSFL_F6_MASK)
#define SIM_GPSFL_F7_MASK                        0xC000u
#define SIM_GPSFL_F7_SHIFT                       14
#define SIM_GPSFL_F7(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFL_F7_SHIFT))&SIM_GPSFL_F7_MASK)
/* GPSFH Bit Fields */
#define SIM_GPSFH_F8_MASK                        0x3u
#define SIM_GPSFH_F8_SHIFT                       0
#define SIM_GPSFH_F8(x)                          (((uint16_t)(((uint16_t)(x))<<SIM_GPSFH_F8_SHIFT))&SIM_GPSFH_F8_MASK)
/* IPSn Bit Fields */
#define SIM_IPSn_SCI0_MASK                       0x1u
#define SIM_IPSn_SCI0_SHIFT                      0
#define SIM_IPSn_SCI1_MASK                       0x2u
#define SIM_IPSn_SCI1_SHIFT                      1
#define SIM_IPSn_TA0_MASK                        0x100u
#define SIM_IPSn_TA0_SHIFT                       8
#define SIM_IPSn_TA1_MASK                        0x200u
#define SIM_IPSn_TA1_SHIFT                       9
#define SIM_IPSn_TA2_MASK                        0x400u
#define SIM_IPSn_TA2_SHIFT                       10
#define SIM_IPSn_TA3_MASK                        0x800u
#define SIM_IPSn_TA3_SHIFT                       11
/* MISC0 Bit Fields */
#define SIM_MISC0_PIT_MSTR_MASK                  0x1u
#define SIM_MISC0_PIT_MSTR_SHIFT                 0
#define SIM_MISC0_CLKINSEL_MASK                  0x2u
#define SIM_MISC0_CLKINSEL_SHIFT                 1
#define SIM_MISC0_FAST_MODE_MASK                 0x4u
#define SIM_MISC0_FAST_MODE_SHIFT                2
#define SIM_MISC0_ADC_SCTRL_MASK                 0x8u
#define SIM_MISC0_ADC_SCTRL_SHIFT                3
#define SIM_MISC0_MODE_STAT_MASK                 0x100u
#define SIM_MISC0_MODE_STAT_SHIFT                8
/* PSWR0 Bit Fields */
#define SIM_PSWR0_GPIO_MASK                      0x40u
#define SIM_PSWR0_GPIO_SHIFT                     6
#define SIM_PSWR0_TA_MASK                        0x8000u
#define SIM_PSWR0_TA_SHIFT                       15
/* PSWR1 Bit Fields */
#define SIM_PSWR1_MSCAN_MASK                     0x1u
#define SIM_PSWR1_MSCAN_SHIFT                    0
#define SIM_PSWR1_IIC0_MASK                      0x40u
#define SIM_PSWR1_IIC0_SHIFT                     6
#define SIM_PSWR1_QSPI1_MASK                     0x100u
#define SIM_PSWR1_QSPI1_SHIFT                    8
#define SIM_PSWR1_QSPI0_MASK                     0x200u
#define SIM_PSWR1_QSPI0_SHIFT                    9
#define SIM_PSWR1_SCI1_MASK                      0x800u
#define SIM_PSWR1_SCI1_SHIFT                     11
#define SIM_PSWR1_SCI0_MASK                      0x1000u
#define SIM_PSWR1_SCI0_SHIFT                     12
#define SIM_PSWR1_DACA_MASK                      0x2000u
#define SIM_PSWR1_DACA_SHIFT                     13
#define SIM_PSWR1_DACB_MASK                      0x4000u
#define SIM_PSWR1_DACB_SHIFT                     14
/* PSWR2 Bit Fields */
#define SIM_PSWR2_PIT1_MASK                      0x4u
#define SIM_PSWR2_PIT1_SHIFT                     2
#define SIM_PSWR2_PIT0_MASK                      0x8u
#define SIM_PSWR2_PIT0_SHIFT                     3
#define SIM_PSWR2_CRC_MASK                       0x20u
#define SIM_PSWR2_CRC_SHIFT                      5
#define SIM_PSWR2_CYCADC_MASK                    0x80u
#define SIM_PSWR2_CYCADC_SHIFT                   7
#define SIM_PSWR2_CMP_MASK                       0x1000u
#define SIM_PSWR2_CMP_SHIFT                      12
#define SIM_PSWR2_EWM_MASK                       0x8000u
#define SIM_PSWR2_EWM_SHIFT                      15
/* PSWR3 Bit Fields */
#define SIM_PSWR3_PWMA_MASK                      0x80u
#define SIM_PSWR3_PWMA_SHIFT                     7
/* PWRMODE Bit Fields */
#define SIM_PWRMODE_VLPMODE_MASK                 0x1u
#define SIM_PWRMODE_VLPMODE_SHIFT                0
#define SIM_PWRMODE_LPMODE_MASK                  0x2u
#define SIM_PWRMODE_LPMODE_SHIFT                 1
#define SIM_PWRMODE_VLPMS_MASK                   0x100u
#define SIM_PWRMODE_VLPMS_SHIFT                  8
#define SIM_PWRMODE_LPMS_MASK                    0x200u
#define SIM_PWRMODE_LPMS_SHIFT                   9
/* NVMOPT2H Bit Fields */
#define SIM_NVMOPT2H_ROSC_8M_FTRIM_MASK          0x3FFu
#define SIM_NVMOPT2H_ROSC_8M_FTRIM_SHIFT         0
#define SIM_NVMOPT2H_ROSC_8M_FTRIM(x)            (((uint16_t)(((uint16_t)(x))<<SIM_NVMOPT2H_ROSC_8M_FTRIM_SHIFT))&SIM_NVMOPT2H_ROSC_8M_FTRIM_MASK)
#define SIM_NVMOPT2H_ROSC_8M_TTRIM_MASK          0x3C00u
#define SIM_NVMOPT2H_ROSC_8M_TTRIM_SHIFT         10
#define SIM_NVMOPT2H_ROSC_8M_TTRIM(x)            (((uint16_t)(((uint16_t)(x))<<SIM_NVMOPT2H_ROSC_8M_TTRIM_SHIFT))&SIM_NVMOPT2H_ROSC_8M_TTRIM_MASK)
/* NVMOPT2L Bit Fields */
#define SIM_NVMOPT2L_ROSC_200K_FTRIM_MASK        0x1FFu
#define SIM_NVMOPT2L_ROSC_200K_FTRIM_SHIFT       0
#define SIM_NVMOPT2L_ROSC_200K_FTRIM(x)          (((uint16_t)(((uint16_t)(x))<<SIM_NVMOPT2L_ROSC_200K_FTRIM_SHIFT))&SIM_NVMOPT2L_ROSC_200K_FTRIM_MASK)
#define SIM_NVMOPT2L_PMC_BGTRIM_MASK             0xF000u
#define SIM_NVMOPT2L_PMC_BGTRIM_SHIFT            12
#define SIM_NVMOPT2L_PMC_BGTRIM(x)               (((uint16_t)(((uint16_t)(x))<<SIM_NVMOPT2L_PMC_BGTRIM_SHIFT))&SIM_NVMOPT2L_PMC_BGTRIM_MASK)
/* SCR0 Bit Fields */
#define SIM_SCR0_SCR0_MASK                       0xFFFFu
#define SIM_SCR0_SCR0_SHIFT                      0
#define SIM_SCR0_SCR0(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR0_SCR0_SHIFT))&SIM_SCR0_SCR0_MASK)
/* SCR1 Bit Fields */
#define SIM_SCR1_SCR1_MASK                       0xFFFFu
#define SIM_SCR1_SCR1_SHIFT                      0
#define SIM_SCR1_SCR1(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR1_SCR1_SHIFT))&SIM_SCR1_SCR1_MASK)
/* SCR2 Bit Fields */
#define SIM_SCR2_SCR2_MASK                       0xFFFFu
#define SIM_SCR2_SCR2_SHIFT                      0
#define SIM_SCR2_SCR2(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR2_SCR2_SHIFT))&SIM_SCR2_SCR2_MASK)
/* SCR3 Bit Fields */
#define SIM_SCR3_SCR3_MASK                       0xFFFFu
#define SIM_SCR3_SCR3_SHIFT                      0
#define SIM_SCR3_SCR3(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR3_SCR3_SHIFT))&SIM_SCR3_SCR3_MASK)
/* SCR4 Bit Fields */
#define SIM_SCR4_SCR4_MASK                       0xFFFFu
#define SIM_SCR4_SCR4_SHIFT                      0
#define SIM_SCR4_SCR4(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR4_SCR4_SHIFT))&SIM_SCR4_SCR4_MASK)
/* SCR5 Bit Fields */
#define SIM_SCR5_SCR5_MASK                       0xFFFFu
#define SIM_SCR5_SCR5_SHIFT                      0
#define SIM_SCR5_SCR5(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR5_SCR5_SHIFT))&SIM_SCR5_SCR5_MASK)
/* SCR6 Bit Fields */
#define SIM_SCR6_SCR6_MASK                       0xFFFFu
#define SIM_SCR6_SCR6_SHIFT                      0
#define SIM_SCR6_SCR6(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR6_SCR6_SHIFT))&SIM_SCR6_SCR6_MASK)
/* SCR7 Bit Fields */
#define SIM_SCR7_SCR7_MASK                       0xFFFFu
#define SIM_SCR7_SCR7_SHIFT                      0
#define SIM_SCR7_SCR7(x)                         (((uint16_t)(((uint16_t)(x))<<SIM_SCR7_SCR7_SHIFT))&SIM_SCR7_SCR7_MASK)

/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base pointer */
#define SIM_BASE_PTR                             ((SIM_MemMapPtr)0xE400u)
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register instance definitions */
/* SIM */
#define SIM_CTRL                                 SIM_CTRL_REG(SIM_BASE_PTR)
#define SIM_RSTAT                                SIM_RSTAT_REG(SIM_BASE_PTR)
#define SIM_MSHID                                SIM_MSHID_REG(SIM_BASE_PTR)
#define SIM_LSHID                                SIM_LSHID_REG(SIM_BASE_PTR)
#define SIM_PWR                                  SIM_PWR_REG(SIM_BASE_PTR)
#define SIM_CLKOUT                               SIM_CLKOUT_REG(SIM_BASE_PTR)
#define SIM_PCR                                  SIM_PCR_REG(SIM_BASE_PTR)
#define SIM_PCE0                                 SIM_PCE0_REG(SIM_BASE_PTR)
#define SIM_PCE1                                 SIM_PCE1_REG(SIM_BASE_PTR)
#define SIM_PCE2                                 SIM_PCE2_REG(SIM_BASE_PTR)
#define SIM_PCE3                                 SIM_PCE3_REG(SIM_BASE_PTR)
#define SIM_SD0                                  SIM_SD0_REG(SIM_BASE_PTR)
#define SIM_SD1                                  SIM_SD1_REG(SIM_BASE_PTR)
#define SIM_SD2                                  SIM_SD2_REG(SIM_BASE_PTR)
#define SIM_SD3                                  SIM_SD3_REG(SIM_BASE_PTR)
#define SIM_IOSAHI                               SIM_IOSAHI_REG(SIM_BASE_PTR)
#define SIM_IOSALO                               SIM_IOSALO_REG(SIM_BASE_PTR)
#define SIM_PROT                                 SIM_PROT_REG(SIM_BASE_PTR)
#define SIM_GPSAL                                SIM_GPSAL_REG(SIM_BASE_PTR)
#define SIM_GPSBL                                SIM_GPSBL_REG(SIM_BASE_PTR)
#define SIM_GPSCL                                SIM_GPSCL_REG(SIM_BASE_PTR)
#define SIM_GPSCH                                SIM_GPSCH_REG(SIM_BASE_PTR)
#define SIM_GPSEL                                SIM_GPSEL_REG(SIM_BASE_PTR)
#define SIM_GPSFL                                SIM_GPSFL_REG(SIM_BASE_PTR)
#define SIM_GPSFH                                SIM_GPSFH_REG(SIM_BASE_PTR)
#define SIM_IPSn                                 SIM_IPSn_REG(SIM_BASE_PTR)
#define SIM_MISC0                                SIM_MISC0_REG(SIM_BASE_PTR)
#define SIM_PSWR0                                SIM_PSWR0_REG(SIM_BASE_PTR)
#define SIM_PSWR1                                SIM_PSWR1_REG(SIM_BASE_PTR)
#define SIM_PSWR2                                SIM_PSWR2_REG(SIM_BASE_PTR)
#define SIM_PSWR3                                SIM_PSWR3_REG(SIM_BASE_PTR)
#define SIM_PWRMODE                              SIM_PWRMODE_REG(SIM_BASE_PTR)
#define SIM_NVMOPT2H                             SIM_NVMOPT2H_REG(SIM_BASE_PTR)
#define SIM_NVMOPT2L                             SIM_NVMOPT2L_REG(SIM_BASE_PTR)
#define SIM_SCR0                                 SIM_SCR0_REG(SIM_BASE_PTR)
#define SIM_SCR1                                 SIM_SCR1_REG(SIM_BASE_PTR)
#define SIM_SCR2                                 SIM_SCR2_REG(SIM_BASE_PTR)
#define SIM_SCR3                                 SIM_SCR3_REG(SIM_BASE_PTR)
#define SIM_SCR4                                 SIM_SCR4_REG(SIM_BASE_PTR)
#define SIM_SCR5                                 SIM_SCR5_REG(SIM_BASE_PTR)
#define SIM_SCR6                                 SIM_SCR6_REG(SIM_BASE_PTR)
#define SIM_SCR7                                 SIM_SCR7_REG(SIM_BASE_PTR)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SIM_Peripheral */


/* ----------------------------------------------------------------------------
   -- TMR
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Peripheral TMR
 * @{
 */

/** TMR - Peripheral register structure */
typedef struct TMR_MemMap {
  struct {                                         /* offset: 0x0, array step: 0x10 */
    uint16_t COMP1;                                  /**< Timer Channel Compare Register 1, array offset: 0x0, array step: 0x10 */
    uint16_t COMP2;                                  /**< Timer Channel Compare Register 2, array offset: 0x1, array step: 0x10 */
    uint16_t CAPT;                                   /**< Timer Channel Capture Register, array offset: 0x2, array step: 0x10 */
    uint16_t LOAD;                                   /**< Timer Channel Load Register, array offset: 0x3, array step: 0x10 */
    uint16_t HOLD;                                   /**< Timer Channel Hold Register, array offset: 0x4, array step: 0x10 */
    uint16_t CNTR;                                   /**< Timer Channel Counter Register, array offset: 0x5, array step: 0x10 */
    uint16_t CTRL;                                   /**< Timer Channel Control Register, array offset: 0x6, array step: 0x10 */
    uint16_t SCTRL;                                  /**< Timer Channel Status and Control Register, array offset: 0x7, array step: 0x10 */
    uint16_t CMPLD1;                                 /**< Timer Channel Comparator Load Register 1, array offset: 0x8, array step: 0x10 */
    uint16_t CMPLD2;                                 /**< Timer Channel Comparator Load Register 2, array offset: 0x9, array step: 0x10 */
    uint16_t CSCTRL;                                 /**< Timer Channel Comparator Status and Control Register, array offset: 0xA, array step: 0x10 */
    uint16_t FILT;                                   /**< Timer Channel Input Filter Register, array offset: 0xB, array step: 0x10 */
    uint16_t DMA;                                    /**< Timer Channel DMA Enable Register, array offset: 0xC, array step: 0x10 */
    uint16_t RESERVED_0[2];
    uint16_t ENBL;                                   /**< Timer Channel Enable Register, array offset: 0xF, array step: 0x10 */
  } GROUPS[4];
} volatile *TMR_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- TMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Register_Accessor_Macros TMR - Register accessor macros
 * @{
 */


/* TMR - Register accessors */
#define TMR_COMP1_REG(base,index)                ((base)->GROUPS[index].COMP1)
#define TMR_COMP2_REG(base,index)                ((base)->GROUPS[index].COMP2)
#define TMR_CAPT_REG(base,index)                 ((base)->GROUPS[index].CAPT)
#define TMR_LOAD_REG(base,index)                 ((base)->GROUPS[index].LOAD)
#define TMR_HOLD_REG(base,index)                 ((base)->GROUPS[index].HOLD)
#define TMR_CNTR_REG(base,index)                 ((base)->GROUPS[index].CNTR)
#define TMR_CTRL_REG(base,index)                 ((base)->GROUPS[index].CTRL)
#define TMR_SCTRL_REG(base,index)                ((base)->GROUPS[index].SCTRL)
#define TMR_CMPLD1_REG(base,index)               ((base)->GROUPS[index].CMPLD1)
#define TMR_CMPLD2_REG(base,index)               ((base)->GROUPS[index].CMPLD2)
#define TMR_CSCTRL_REG(base,index)               ((base)->GROUPS[index].CSCTRL)
#define TMR_FILT_REG(base,index)                 ((base)->GROUPS[index].FILT)
#define TMR_DMA_REG(base,index)                  ((base)->GROUPS[index].DMA)
#define TMR_ENBL_REG(base,index)                 ((base)->GROUPS[index].ENBL)

/*!
 * @}
 */ /* end of group TMR_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- TMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Register_Masks TMR Register Masks
 * @{
 */

/* COMP1 Bit Fields */
#define TMR_COMP1_COMPARISON_1_MASK              0xFFFFu
#define TMR_COMP1_COMPARISON_1_SHIFT             0
#define TMR_COMP1_COMPARISON_1(x)                (((uint16_t)(((uint16_t)(x))<<TMR_COMP1_COMPARISON_1_SHIFT))&TMR_COMP1_COMPARISON_1_MASK)
/* COMP2 Bit Fields */
#define TMR_COMP2_COMPARISON_2_MASK              0xFFFFu
#define TMR_COMP2_COMPARISON_2_SHIFT             0
#define TMR_COMP2_COMPARISON_2(x)                (((uint16_t)(((uint16_t)(x))<<TMR_COMP2_COMPARISON_2_SHIFT))&TMR_COMP2_COMPARISON_2_MASK)
/* CAPT Bit Fields */
#define TMR_CAPT_CAPTURE_MASK                    0xFFFFu
#define TMR_CAPT_CAPTURE_SHIFT                   0
#define TMR_CAPT_CAPTURE(x)                      (((uint16_t)(((uint16_t)(x))<<TMR_CAPT_CAPTURE_SHIFT))&TMR_CAPT_CAPTURE_MASK)
/* LOAD Bit Fields */
#define TMR_LOAD_LOAD_MASK                       0xFFFFu
#define TMR_LOAD_LOAD_SHIFT                      0
#define TMR_LOAD_LOAD(x)                         (((uint16_t)(((uint16_t)(x))<<TMR_LOAD_LOAD_SHIFT))&TMR_LOAD_LOAD_MASK)
/* HOLD Bit Fields */
#define TMR_HOLD_HOLD_MASK                       0xFFFFu
#define TMR_HOLD_HOLD_SHIFT                      0
#define TMR_HOLD_HOLD(x)                         (((uint16_t)(((uint16_t)(x))<<TMR_HOLD_HOLD_SHIFT))&TMR_HOLD_HOLD_MASK)
/* CNTR Bit Fields */
#define TMR_CNTR_COUNTER_MASK                    0xFFFFu
#define TMR_CNTR_COUNTER_SHIFT                   0
#define TMR_CNTR_COUNTER(x)                      (((uint16_t)(((uint16_t)(x))<<TMR_CNTR_COUNTER_SHIFT))&TMR_CNTR_COUNTER_MASK)
/* CTRL Bit Fields */
#define TMR_CTRL_OUTMODE_MASK                    0x7u
#define TMR_CTRL_OUTMODE_SHIFT                   0
#define TMR_CTRL_OUTMODE(x)                      (((uint16_t)(((uint16_t)(x))<<TMR_CTRL_OUTMODE_SHIFT))&TMR_CTRL_OUTMODE_MASK)
#define TMR_CTRL_COINIT_MASK                     0x8u
#define TMR_CTRL_COINIT_SHIFT                    3
#define TMR_CTRL_DIR_MASK                        0x10u
#define TMR_CTRL_DIR_SHIFT                       4
#define TMR_CTRL_LENGTH_MASK                     0x20u
#define TMR_CTRL_LENGTH_SHIFT                    5
#define TMR_CTRL_ONCE_MASK                       0x40u
#define TMR_CTRL_ONCE_SHIFT                      6
#define TMR_CTRL_SCS_MASK                        0x180u
#define TMR_CTRL_SCS_SHIFT                       7
#define TMR_CTRL_SCS(x)                          (((uint16_t)(((uint16_t)(x))<<TMR_CTRL_SCS_SHIFT))&TMR_CTRL_SCS_MASK)
#define TMR_CTRL_PCS_MASK                        0x1E00u
#define TMR_CTRL_PCS_SHIFT                       9
#define TMR_CTRL_PCS(x)                          (((uint16_t)(((uint16_t)(x))<<TMR_CTRL_PCS_SHIFT))&TMR_CTRL_PCS_MASK)
#define TMR_CTRL_CM_MASK                         0xE000u
#define TMR_CTRL_CM_SHIFT                        13
#define TMR_CTRL_CM(x)                           (((uint16_t)(((uint16_t)(x))<<TMR_CTRL_CM_SHIFT))&TMR_CTRL_CM_MASK)
/* SCTRL Bit Fields */
#define TMR_SCTRL_OEN_MASK                       0x1u
#define TMR_SCTRL_OEN_SHIFT                      0
#define TMR_SCTRL_OPS_MASK                       0x2u
#define TMR_SCTRL_OPS_SHIFT                      1
#define TMR_SCTRL_FORCE_MASK                     0x4u
#define TMR_SCTRL_FORCE_SHIFT                    2
#define TMR_SCTRL_VAL_MASK                       0x8u
#define TMR_SCTRL_VAL_SHIFT                      3
#define TMR_SCTRL_EEOF_MASK                      0x10u
#define TMR_SCTRL_EEOF_SHIFT                     4
#define TMR_SCTRL_MSTR_MASK                      0x20u
#define TMR_SCTRL_MSTR_SHIFT                     5
#define TMR_SCTRL_CAPTURE_MODE_MASK              0xC0u
#define TMR_SCTRL_CAPTURE_MODE_SHIFT             6
#define TMR_SCTRL_CAPTURE_MODE(x)                (((uint16_t)(((uint16_t)(x))<<TMR_SCTRL_CAPTURE_MODE_SHIFT))&TMR_SCTRL_CAPTURE_MODE_MASK)
#define TMR_SCTRL_INPUT_MASK                     0x100u
#define TMR_SCTRL_INPUT_SHIFT                    8
#define TMR_SCTRL_IPS_MASK                       0x200u
#define TMR_SCTRL_IPS_SHIFT                      9
#define TMR_SCTRL_IEFIE_MASK                     0x400u
#define TMR_SCTRL_IEFIE_SHIFT                    10
#define TMR_SCTRL_IEF_MASK                       0x800u
#define TMR_SCTRL_IEF_SHIFT                      11
#define TMR_SCTRL_TOFIE_MASK                     0x1000u
#define TMR_SCTRL_TOFIE_SHIFT                    12
#define TMR_SCTRL_TOF_MASK                       0x2000u
#define TMR_SCTRL_TOF_SHIFT                      13
#define TMR_SCTRL_TCFIE_MASK                     0x4000u
#define TMR_SCTRL_TCFIE_SHIFT                    14
#define TMR_SCTRL_TCF_MASK                       0x8000u
#define TMR_SCTRL_TCF_SHIFT                      15
/* CMPLD1 Bit Fields */
#define TMR_CMPLD1_COMPARATOR_LOAD_1_MASK        0xFFFFu
#define TMR_CMPLD1_COMPARATOR_LOAD_1_SHIFT       0
#define TMR_CMPLD1_COMPARATOR_LOAD_1(x)          (((uint16_t)(((uint16_t)(x))<<TMR_CMPLD1_COMPARATOR_LOAD_1_SHIFT))&TMR_CMPLD1_COMPARATOR_LOAD_1_MASK)
/* CMPLD2 Bit Fields */
#define TMR_CMPLD2_COMPARATOR_LOAD_2_MASK        0xFFFFu
#define TMR_CMPLD2_COMPARATOR_LOAD_2_SHIFT       0
#define TMR_CMPLD2_COMPARATOR_LOAD_2(x)          (((uint16_t)(((uint16_t)(x))<<TMR_CMPLD2_COMPARATOR_LOAD_2_SHIFT))&TMR_CMPLD2_COMPARATOR_LOAD_2_MASK)
/* CSCTRL Bit Fields */
#define TMR_CSCTRL_CL1_MASK                      0x3u
#define TMR_CSCTRL_CL1_SHIFT                     0
#define TMR_CSCTRL_CL1(x)                        (((uint16_t)(((uint16_t)(x))<<TMR_CSCTRL_CL1_SHIFT))&TMR_CSCTRL_CL1_MASK)
#define TMR_CSCTRL_CL2_MASK                      0xCu
#define TMR_CSCTRL_CL2_SHIFT                     2
#define TMR_CSCTRL_CL2(x)                        (((uint16_t)(((uint16_t)(x))<<TMR_CSCTRL_CL2_SHIFT))&TMR_CSCTRL_CL2_MASK)
#define TMR_CSCTRL_TCF1_MASK                     0x10u
#define TMR_CSCTRL_TCF1_SHIFT                    4
#define TMR_CSCTRL_TCF2_MASK                     0x20u
#define TMR_CSCTRL_TCF2_SHIFT                    5
#define TMR_CSCTRL_TCF1EN_MASK                   0x40u
#define TMR_CSCTRL_TCF1EN_SHIFT                  6
#define TMR_CSCTRL_TCF2EN_MASK                   0x80u
#define TMR_CSCTRL_TCF2EN_SHIFT                  7
#define TMR_CSCTRL_UP_MASK                       0x200u
#define TMR_CSCTRL_UP_SHIFT                      9
#define TMR_CSCTRL_TCI_MASK                      0x400u
#define TMR_CSCTRL_TCI_SHIFT                     10
#define TMR_CSCTRL_ROC_MASK                      0x800u
#define TMR_CSCTRL_ROC_SHIFT                     11
#define TMR_CSCTRL_ALT_LOAD_MASK                 0x1000u
#define TMR_CSCTRL_ALT_LOAD_SHIFT                12
#define TMR_CSCTRL_FAULT_MASK                    0x2000u
#define TMR_CSCTRL_FAULT_SHIFT                   13
#define TMR_CSCTRL_DBG_EN_MASK                   0xC000u
#define TMR_CSCTRL_DBG_EN_SHIFT                  14
#define TMR_CSCTRL_DBG_EN(x)                     (((uint16_t)(((uint16_t)(x))<<TMR_CSCTRL_DBG_EN_SHIFT))&TMR_CSCTRL_DBG_EN_MASK)
/* FILT Bit Fields */
#define TMR_FILT_FILT_PER_MASK                   0xFFu
#define TMR_FILT_FILT_PER_SHIFT                  0
#define TMR_FILT_FILT_PER(x)                     (((uint16_t)(((uint16_t)(x))<<TMR_FILT_FILT_PER_SHIFT))&TMR_FILT_FILT_PER_MASK)
#define TMR_FILT_FILT_CNT_MASK                   0x700u
#define TMR_FILT_FILT_CNT_SHIFT                  8
#define TMR_FILT_FILT_CNT(x)                     (((uint16_t)(((uint16_t)(x))<<TMR_FILT_FILT_CNT_SHIFT))&TMR_FILT_FILT_CNT_MASK)
/* DMA Bit Fields */
#define TMR_DMA_IEFDE_MASK                       0x1u
#define TMR_DMA_IEFDE_SHIFT                      0
#define TMR_DMA_CMPLD1DE_MASK                    0x2u
#define TMR_DMA_CMPLD1DE_SHIFT                   1
#define TMR_DMA_CMPLD2DE_MASK                    0x4u
#define TMR_DMA_CMPLD2DE_SHIFT                   2
/* ENBL Bit Fields */
#define TMR_ENBL_ENBL_MASK                       0xFu
#define TMR_ENBL_ENBL_SHIFT                      0
#define TMR_ENBL_ENBL(x)                         (((uint16_t)(((uint16_t)(x))<<TMR_ENBL_ENBL_SHIFT))&TMR_ENBL_ENBL_MASK)

/*!
 * @}
 */ /* end of group TMR_Register_Masks */


/* TMR - Peripheral instance base addresses */
/** Peripheral TMR base pointer */
#define TMR_BASE_PTR                             ((TMR_MemMapPtr)0xE140u)
/** Array initializer of TMR peripheral base pointers */
#define TMR_BASE_PTRS                            { TMR_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- TMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Register_Accessor_Macros TMR - Register accessor macros
 * @{
 */


/* TMR - Register instance definitions */
/* TMR */
#define TMR_0_COMP1                              TMR_COMP1_REG(TMR_BASE_PTR,0)
#define TMR_0_COMP2                              TMR_COMP2_REG(TMR_BASE_PTR,0)
#define TMR_0_CAPT                               TMR_CAPT_REG(TMR_BASE_PTR,0)
#define TMR_0_LOAD                               TMR_LOAD_REG(TMR_BASE_PTR,0)
#define TMR_0_HOLD                               TMR_HOLD_REG(TMR_BASE_PTR,0)
#define TMR_0_CNTR                               TMR_CNTR_REG(TMR_BASE_PTR,0)
#define TMR_0_CTRL                               TMR_CTRL_REG(TMR_BASE_PTR,0)
#define TMR_0_SCTRL                              TMR_SCTRL_REG(TMR_BASE_PTR,0)
#define TMR_0_CMPLD1                             TMR_CMPLD1_REG(TMR_BASE_PTR,0)
#define TMR_0_CMPLD2                             TMR_CMPLD2_REG(TMR_BASE_PTR,0)
#define TMR_0_CSCTRL                             TMR_CSCTRL_REG(TMR_BASE_PTR,0)
#define TMR_0_FILT                               TMR_FILT_REG(TMR_BASE_PTR,0)
#define TMR_0_DMA                                TMR_DMA_REG(TMR_BASE_PTR,0)
#define TMR_0_ENBL                               TMR_ENBL_REG(TMR_BASE_PTR,0)
#define TMR_1_COMP1                              TMR_COMP1_REG(TMR_BASE_PTR,1)
#define TMR_1_COMP2                              TMR_COMP2_REG(TMR_BASE_PTR,1)
#define TMR_1_CAPT                               TMR_CAPT_REG(TMR_BASE_PTR,1)
#define TMR_1_LOAD                               TMR_LOAD_REG(TMR_BASE_PTR,1)
#define TMR_1_HOLD                               TMR_HOLD_REG(TMR_BASE_PTR,1)
#define TMR_1_CNTR                               TMR_CNTR_REG(TMR_BASE_PTR,1)
#define TMR_1_CTRL                               TMR_CTRL_REG(TMR_BASE_PTR,1)
#define TMR_1_SCTRL                              TMR_SCTRL_REG(TMR_BASE_PTR,1)
#define TMR_1_CMPLD1                             TMR_CMPLD1_REG(TMR_BASE_PTR,1)
#define TMR_1_CMPLD2                             TMR_CMPLD2_REG(TMR_BASE_PTR,1)
#define TMR_1_CSCTRL                             TMR_CSCTRL_REG(TMR_BASE_PTR,1)
#define TMR_1_FILT                               TMR_FILT_REG(TMR_BASE_PTR,1)
#define TMR_1_DMA                                TMR_DMA_REG(TMR_BASE_PTR,1)
#define TMR_2_COMP1                              TMR_COMP1_REG(TMR_BASE_PTR,2)
#define TMR_2_COMP2                              TMR_COMP2_REG(TMR_BASE_PTR,2)
#define TMR_2_CAPT                               TMR_CAPT_REG(TMR_BASE_PTR,2)
#define TMR_2_LOAD                               TMR_LOAD_REG(TMR_BASE_PTR,2)
#define TMR_2_HOLD                               TMR_HOLD_REG(TMR_BASE_PTR,2)
#define TMR_2_CNTR                               TMR_CNTR_REG(TMR_BASE_PTR,2)
#define TMR_2_CTRL                               TMR_CTRL_REG(TMR_BASE_PTR,2)
#define TMR_2_SCTRL                              TMR_SCTRL_REG(TMR_BASE_PTR,2)
#define TMR_2_CMPLD1                             TMR_CMPLD1_REG(TMR_BASE_PTR,2)
#define TMR_2_CMPLD2                             TMR_CMPLD2_REG(TMR_BASE_PTR,2)
#define TMR_2_CSCTRL                             TMR_CSCTRL_REG(TMR_BASE_PTR,2)
#define TMR_2_FILT                               TMR_FILT_REG(TMR_BASE_PTR,2)
#define TMR_2_DMA                                TMR_DMA_REG(TMR_BASE_PTR,2)
#define TMR_3_COMP1                              TMR_COMP1_REG(TMR_BASE_PTR,3)
#define TMR_3_COMP2                              TMR_COMP2_REG(TMR_BASE_PTR,3)
#define TMR_3_CAPT                               TMR_CAPT_REG(TMR_BASE_PTR,3)
#define TMR_3_LOAD                               TMR_LOAD_REG(TMR_BASE_PTR,3)
#define TMR_3_HOLD                               TMR_HOLD_REG(TMR_BASE_PTR,3)
#define TMR_3_CNTR                               TMR_CNTR_REG(TMR_BASE_PTR,3)
#define TMR_3_CTRL                               TMR_CTRL_REG(TMR_BASE_PTR,3)
#define TMR_3_SCTRL                              TMR_SCTRL_REG(TMR_BASE_PTR,3)
#define TMR_3_CMPLD1                             TMR_CMPLD1_REG(TMR_BASE_PTR,3)
#define TMR_3_CMPLD2                             TMR_CMPLD2_REG(TMR_BASE_PTR,3)
#define TMR_3_CSCTRL                             TMR_CSCTRL_REG(TMR_BASE_PTR,3)
#define TMR_3_FILT                               TMR_FILT_REG(TMR_BASE_PTR,3)
#define TMR_3_DMA                                TMR_DMA_REG(TMR_BASE_PTR,3)

/* TMR - Register array accessors */
#define TMR_COMP1(index)                         TMR_COMP1_REG(TMR_BASE_PTR,index)
#define TMR_COMP2(index)                         TMR_COMP2_REG(TMR_BASE_PTR,index)
#define TMR_CAPT(index)                          TMR_CAPT_REG(TMR_BASE_PTR,index)
#define TMR_LOAD(index)                          TMR_LOAD_REG(TMR_BASE_PTR,index)
#define TMR_HOLD(index)                          TMR_HOLD_REG(TMR_BASE_PTR,index)
#define TMR_CNTR(index)                          TMR_CNTR_REG(TMR_BASE_PTR,index)
#define TMR_CTRL(index)                          TMR_CTRL_REG(TMR_BASE_PTR,index)
#define TMR_SCTRL(index)                         TMR_SCTRL_REG(TMR_BASE_PTR,index)
#define TMR_CMPLD1(index)                        TMR_CMPLD1_REG(TMR_BASE_PTR,index)
#define TMR_CMPLD2(index)                        TMR_CMPLD2_REG(TMR_BASE_PTR,index)
#define TMR_CSCTRL(index)                        TMR_CSCTRL_REG(TMR_BASE_PTR,index)
#define TMR_FILT(index)                          TMR_FILT_REG(TMR_BASE_PTR,index)
#define TMR_DMA(index)                           TMR_DMA_REG(TMR_BASE_PTR,index)
#define TMR_ENBL(index)                          TMR_ENBL_REG(TMR_BASE_PTR,index)

/*!
 * @}
 */ /* end of group TMR_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group TMR_Peripheral */


/* ----------------------------------------------------------------------------
   -- XBARA
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Peripheral XBARA
 * @{
 */

/** XBARA - Peripheral register structure */
typedef struct XBARA_MemMap {
  uint16_t SEL0;                                   /**< Crossbar A Select Register 0, offset: 0x0 */
  uint16_t SEL1;                                   /**< Crossbar A Select Register 1, offset: 0x1 */
  uint16_t SEL2;                                   /**< Crossbar A Select Register 2, offset: 0x2 */
  uint16_t SEL3;                                   /**< Crossbar A Select Register 3, offset: 0x3 */
  uint16_t SEL4;                                   /**< Crossbar A Select Register 4, offset: 0x4 */
  uint16_t SEL5;                                   /**< Crossbar A Select Register 5, offset: 0x5 */
  uint16_t SEL6;                                   /**< Crossbar A Select Register 6, offset: 0x6 */
  uint16_t SEL7;                                   /**< Crossbar A Select Register 7, offset: 0x7 */
  uint16_t SEL8;                                   /**< Crossbar A Select Register 8, offset: 0x8 */
  uint16_t SEL9;                                   /**< Crossbar A Select Register 9, offset: 0x9 */
  uint16_t SEL10;                                  /**< Crossbar A Select Register 10, offset: 0xA */
  uint16_t SEL11;                                  /**< Crossbar A Select Register 11, offset: 0xB */
  uint16_t SEL12;                                  /**< Crossbar A Select Register 12, offset: 0xC */
  uint16_t SEL13;                                  /**< Crossbar A Select Register 13, offset: 0xD */
  uint16_t SEL14;                                  /**< Crossbar A Select Register 14, offset: 0xE */
  uint16_t SEL15;                                  /**< Crossbar A Select Register 15, offset: 0xF */
  uint16_t SEL16;                                  /**< Crossbar A Select Register 16, offset: 0x10 */
  uint16_t SEL17;                                  /**< Crossbar A Select Register 17, offset: 0x11 */
  uint16_t SEL18;                                  /**< Crossbar A Select Register 18, offset: 0x12 */
  uint16_t SEL19;                                  /**< Crossbar A Select Register 19, offset: 0x13 */
  uint16_t SEL20;                                  /**< Crossbar A Select Register 20, offset: 0x14 */
  uint16_t CTRL0;                                  /**< Crossbar A Control Register 0, offset: 0x15 */
  uint16_t CTRL1;                                  /**< Crossbar A Control Register 1, offset: 0x16 */
} volatile *XBARA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- XBARA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Accessor_Macros XBARA - Register accessor macros
 * @{
 */


/* XBARA - Register accessors */
#define XBARA_SEL0_REG(base)                     ((base)->SEL0)
#define XBARA_SEL1_REG(base)                     ((base)->SEL1)
#define XBARA_SEL2_REG(base)                     ((base)->SEL2)
#define XBARA_SEL3_REG(base)                     ((base)->SEL3)
#define XBARA_SEL4_REG(base)                     ((base)->SEL4)
#define XBARA_SEL5_REG(base)                     ((base)->SEL5)
#define XBARA_SEL6_REG(base)                     ((base)->SEL6)
#define XBARA_SEL7_REG(base)                     ((base)->SEL7)
#define XBARA_SEL8_REG(base)                     ((base)->SEL8)
#define XBARA_SEL9_REG(base)                     ((base)->SEL9)
#define XBARA_SEL10_REG(base)                    ((base)->SEL10)
#define XBARA_SEL11_REG(base)                    ((base)->SEL11)
#define XBARA_SEL12_REG(base)                    ((base)->SEL12)
#define XBARA_SEL13_REG(base)                    ((base)->SEL13)
#define XBARA_SEL14_REG(base)                    ((base)->SEL14)
#define XBARA_SEL15_REG(base)                    ((base)->SEL15)
#define XBARA_SEL16_REG(base)                    ((base)->SEL16)
#define XBARA_SEL17_REG(base)                    ((base)->SEL17)
#define XBARA_SEL18_REG(base)                    ((base)->SEL18)
#define XBARA_SEL19_REG(base)                    ((base)->SEL19)
#define XBARA_SEL20_REG(base)                    ((base)->SEL20)
#define XBARA_CTRL0_REG(base)                    ((base)->CTRL0)
#define XBARA_CTRL1_REG(base)                    ((base)->CTRL1)

/*!
 * @}
 */ /* end of group XBARA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- XBARA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Masks XBARA Register Masks
 * @{
 */

/* SEL0 Bit Fields */
#define XBARA_SEL0_SEL0_MASK                     0x1Fu
#define XBARA_SEL0_SEL0_SHIFT                    0
#define XBARA_SEL0_SEL0(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL0_SEL0_SHIFT))&XBARA_SEL0_SEL0_MASK)
#define XBARA_SEL0_SEL1_MASK                     0x1F00u
#define XBARA_SEL0_SEL1_SHIFT                    8
#define XBARA_SEL0_SEL1(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL0_SEL1_SHIFT))&XBARA_SEL0_SEL1_MASK)
/* SEL1 Bit Fields */
#define XBARA_SEL1_SEL2_MASK                     0x1Fu
#define XBARA_SEL1_SEL2_SHIFT                    0
#define XBARA_SEL1_SEL2(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL1_SEL2_SHIFT))&XBARA_SEL1_SEL2_MASK)
#define XBARA_SEL1_SEL3_MASK                     0x1F00u
#define XBARA_SEL1_SEL3_SHIFT                    8
#define XBARA_SEL1_SEL3(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL1_SEL3_SHIFT))&XBARA_SEL1_SEL3_MASK)
/* SEL2 Bit Fields */
#define XBARA_SEL2_SEL4_MASK                     0x1Fu
#define XBARA_SEL2_SEL4_SHIFT                    0
#define XBARA_SEL2_SEL4(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL2_SEL4_SHIFT))&XBARA_SEL2_SEL4_MASK)
#define XBARA_SEL2_SEL5_MASK                     0x1F00u
#define XBARA_SEL2_SEL5_SHIFT                    8
#define XBARA_SEL2_SEL5(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL2_SEL5_SHIFT))&XBARA_SEL2_SEL5_MASK)
/* SEL3 Bit Fields */
#define XBARA_SEL3_SEL6_MASK                     0x1Fu
#define XBARA_SEL3_SEL6_SHIFT                    0
#define XBARA_SEL3_SEL6(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL3_SEL6_SHIFT))&XBARA_SEL3_SEL6_MASK)
#define XBARA_SEL3_SEL7_MASK                     0x1F00u
#define XBARA_SEL3_SEL7_SHIFT                    8
#define XBARA_SEL3_SEL7(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL3_SEL7_SHIFT))&XBARA_SEL3_SEL7_MASK)
/* SEL4 Bit Fields */
#define XBARA_SEL4_SEL8_MASK                     0x1Fu
#define XBARA_SEL4_SEL8_SHIFT                    0
#define XBARA_SEL4_SEL8(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL4_SEL8_SHIFT))&XBARA_SEL4_SEL8_MASK)
#define XBARA_SEL4_SEL9_MASK                     0x1F00u
#define XBARA_SEL4_SEL9_SHIFT                    8
#define XBARA_SEL4_SEL9(x)                       (((uint16_t)(((uint16_t)(x))<<XBARA_SEL4_SEL9_SHIFT))&XBARA_SEL4_SEL9_MASK)
/* SEL5 Bit Fields */
#define XBARA_SEL5_SEL10_MASK                    0x1Fu
#define XBARA_SEL5_SEL10_SHIFT                   0
#define XBARA_SEL5_SEL10(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL5_SEL10_SHIFT))&XBARA_SEL5_SEL10_MASK)
#define XBARA_SEL5_SEL11_MASK                    0x1F00u
#define XBARA_SEL5_SEL11_SHIFT                   8
#define XBARA_SEL5_SEL11(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL5_SEL11_SHIFT))&XBARA_SEL5_SEL11_MASK)
/* SEL6 Bit Fields */
#define XBARA_SEL6_SEL12_MASK                    0x1Fu
#define XBARA_SEL6_SEL12_SHIFT                   0
#define XBARA_SEL6_SEL12(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL6_SEL12_SHIFT))&XBARA_SEL6_SEL12_MASK)
#define XBARA_SEL6_SEL13_MASK                    0x1F00u
#define XBARA_SEL6_SEL13_SHIFT                   8
#define XBARA_SEL6_SEL13(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL6_SEL13_SHIFT))&XBARA_SEL6_SEL13_MASK)
/* SEL7 Bit Fields */
#define XBARA_SEL7_SEL14_MASK                    0x1Fu
#define XBARA_SEL7_SEL14_SHIFT                   0
#define XBARA_SEL7_SEL14(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL7_SEL14_SHIFT))&XBARA_SEL7_SEL14_MASK)
#define XBARA_SEL7_SEL15_MASK                    0x1F00u
#define XBARA_SEL7_SEL15_SHIFT                   8
#define XBARA_SEL7_SEL15(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL7_SEL15_SHIFT))&XBARA_SEL7_SEL15_MASK)
/* SEL8 Bit Fields */
#define XBARA_SEL8_SEL16_MASK                    0x1Fu
#define XBARA_SEL8_SEL16_SHIFT                   0
#define XBARA_SEL8_SEL16(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL8_SEL16_SHIFT))&XBARA_SEL8_SEL16_MASK)
#define XBARA_SEL8_SEL17_MASK                    0x1F00u
#define XBARA_SEL8_SEL17_SHIFT                   8
#define XBARA_SEL8_SEL17(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL8_SEL17_SHIFT))&XBARA_SEL8_SEL17_MASK)
/* SEL9 Bit Fields */
#define XBARA_SEL9_SEL18_MASK                    0x1Fu
#define XBARA_SEL9_SEL18_SHIFT                   0
#define XBARA_SEL9_SEL18(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL9_SEL18_SHIFT))&XBARA_SEL9_SEL18_MASK)
#define XBARA_SEL9_SEL19_MASK                    0x1F00u
#define XBARA_SEL9_SEL19_SHIFT                   8
#define XBARA_SEL9_SEL19(x)                      (((uint16_t)(((uint16_t)(x))<<XBARA_SEL9_SEL19_SHIFT))&XBARA_SEL9_SEL19_MASK)
/* SEL10 Bit Fields */
#define XBARA_SEL10_SEL20_MASK                   0x1Fu
#define XBARA_SEL10_SEL20_SHIFT                  0
#define XBARA_SEL10_SEL20(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL10_SEL20_SHIFT))&XBARA_SEL10_SEL20_MASK)
#define XBARA_SEL10_SEL21_MASK                   0x1F00u
#define XBARA_SEL10_SEL21_SHIFT                  8
#define XBARA_SEL10_SEL21(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL10_SEL21_SHIFT))&XBARA_SEL10_SEL21_MASK)
/* SEL11 Bit Fields */
#define XBARA_SEL11_SEL22_MASK                   0x1Fu
#define XBARA_SEL11_SEL22_SHIFT                  0
#define XBARA_SEL11_SEL22(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL11_SEL22_SHIFT))&XBARA_SEL11_SEL22_MASK)
#define XBARA_SEL11_SEL23_MASK                   0x1F00u
#define XBARA_SEL11_SEL23_SHIFT                  8
#define XBARA_SEL11_SEL23(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL11_SEL23_SHIFT))&XBARA_SEL11_SEL23_MASK)
/* SEL12 Bit Fields */
#define XBARA_SEL12_SEL24_MASK                   0x1Fu
#define XBARA_SEL12_SEL24_SHIFT                  0
#define XBARA_SEL12_SEL24(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL12_SEL24_SHIFT))&XBARA_SEL12_SEL24_MASK)
#define XBARA_SEL12_SEL25_MASK                   0x1F00u
#define XBARA_SEL12_SEL25_SHIFT                  8
#define XBARA_SEL12_SEL25(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL12_SEL25_SHIFT))&XBARA_SEL12_SEL25_MASK)
/* SEL13 Bit Fields */
#define XBARA_SEL13_SEL26_MASK                   0x1Fu
#define XBARA_SEL13_SEL26_SHIFT                  0
#define XBARA_SEL13_SEL26(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL13_SEL26_SHIFT))&XBARA_SEL13_SEL26_MASK)
#define XBARA_SEL13_SEL27_MASK                   0x1F00u
#define XBARA_SEL13_SEL27_SHIFT                  8
#define XBARA_SEL13_SEL27(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL13_SEL27_SHIFT))&XBARA_SEL13_SEL27_MASK)
/* SEL14 Bit Fields */
#define XBARA_SEL14_SEL28_MASK                   0x1Fu
#define XBARA_SEL14_SEL28_SHIFT                  0
#define XBARA_SEL14_SEL28(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL14_SEL28_SHIFT))&XBARA_SEL14_SEL28_MASK)
#define XBARA_SEL14_SEL29_MASK                   0x1F00u
#define XBARA_SEL14_SEL29_SHIFT                  8
#define XBARA_SEL14_SEL29(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL14_SEL29_SHIFT))&XBARA_SEL14_SEL29_MASK)
/* SEL15 Bit Fields */
#define XBARA_SEL15_SEL30_MASK                   0x1Fu
#define XBARA_SEL15_SEL30_SHIFT                  0
#define XBARA_SEL15_SEL30(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL15_SEL30_SHIFT))&XBARA_SEL15_SEL30_MASK)
#define XBARA_SEL15_SEL31_MASK                   0x1F00u
#define XBARA_SEL15_SEL31_SHIFT                  8
#define XBARA_SEL15_SEL31(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL15_SEL31_SHIFT))&XBARA_SEL15_SEL31_MASK)
/* SEL16 Bit Fields */
#define XBARA_SEL16_SEL32_MASK                   0x1Fu
#define XBARA_SEL16_SEL32_SHIFT                  0
#define XBARA_SEL16_SEL32(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL16_SEL32_SHIFT))&XBARA_SEL16_SEL32_MASK)
#define XBARA_SEL16_SEL33_MASK                   0x1F00u
#define XBARA_SEL16_SEL33_SHIFT                  8
#define XBARA_SEL16_SEL33(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL16_SEL33_SHIFT))&XBARA_SEL16_SEL33_MASK)
/* SEL17 Bit Fields */
#define XBARA_SEL17_SEL34_MASK                   0x1Fu
#define XBARA_SEL17_SEL34_SHIFT                  0
#define XBARA_SEL17_SEL34(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL17_SEL34_SHIFT))&XBARA_SEL17_SEL34_MASK)
#define XBARA_SEL17_SEL35_MASK                   0x1F00u
#define XBARA_SEL17_SEL35_SHIFT                  8
#define XBARA_SEL17_SEL35(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL17_SEL35_SHIFT))&XBARA_SEL17_SEL35_MASK)
/* SEL18 Bit Fields */
#define XBARA_SEL18_SEL36_MASK                   0x1Fu
#define XBARA_SEL18_SEL36_SHIFT                  0
#define XBARA_SEL18_SEL36(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL18_SEL36_SHIFT))&XBARA_SEL18_SEL36_MASK)
#define XBARA_SEL18_SEL37_MASK                   0x1F00u
#define XBARA_SEL18_SEL37_SHIFT                  8
#define XBARA_SEL18_SEL37(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL18_SEL37_SHIFT))&XBARA_SEL18_SEL37_MASK)
/* SEL19 Bit Fields */
#define XBARA_SEL19_SEL38_MASK                   0x1Fu
#define XBARA_SEL19_SEL38_SHIFT                  0
#define XBARA_SEL19_SEL38(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL19_SEL38_SHIFT))&XBARA_SEL19_SEL38_MASK)
#define XBARA_SEL19_SEL39_MASK                   0x1F00u
#define XBARA_SEL19_SEL39_SHIFT                  8
#define XBARA_SEL19_SEL39(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL19_SEL39_SHIFT))&XBARA_SEL19_SEL39_MASK)
/* SEL20 Bit Fields */
#define XBARA_SEL20_SEL40_MASK                   0x1Fu
#define XBARA_SEL20_SEL40_SHIFT                  0
#define XBARA_SEL20_SEL40(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_SEL20_SEL40_SHIFT))&XBARA_SEL20_SEL40_MASK)
/* CTRL0 Bit Fields */
#define XBARA_CTRL0_DEN0_MASK                    0x1u
#define XBARA_CTRL0_DEN0_SHIFT                   0
#define XBARA_CTRL0_IEN0_MASK                    0x2u
#define XBARA_CTRL0_IEN0_SHIFT                   1
#define XBARA_CTRL0_EDGE0_MASK                   0xCu
#define XBARA_CTRL0_EDGE0_SHIFT                  2
#define XBARA_CTRL0_EDGE0(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_EDGE0_SHIFT))&XBARA_CTRL0_EDGE0_MASK)
#define XBARA_CTRL0_STS0_MASK                    0x10u
#define XBARA_CTRL0_STS0_SHIFT                   4
#define XBARA_CTRL0_DEN1_MASK                    0x100u
#define XBARA_CTRL0_DEN1_SHIFT                   8
#define XBARA_CTRL0_IEN1_MASK                    0x200u
#define XBARA_CTRL0_IEN1_SHIFT                   9
#define XBARA_CTRL0_EDGE1_MASK                   0xC00u
#define XBARA_CTRL0_EDGE1_SHIFT                  10
#define XBARA_CTRL0_EDGE1(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL0_EDGE1_SHIFT))&XBARA_CTRL0_EDGE1_MASK)
#define XBARA_CTRL0_STS1_MASK                    0x1000u
#define XBARA_CTRL0_STS1_SHIFT                   12
/* CTRL1 Bit Fields */
#define XBARA_CTRL1_DEN2_MASK                    0x1u
#define XBARA_CTRL1_DEN2_SHIFT                   0
#define XBARA_CTRL1_IEN2_MASK                    0x2u
#define XBARA_CTRL1_IEN2_SHIFT                   1
#define XBARA_CTRL1_EDGE2_MASK                   0xCu
#define XBARA_CTRL1_EDGE2_SHIFT                  2
#define XBARA_CTRL1_EDGE2(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_EDGE2_SHIFT))&XBARA_CTRL1_EDGE2_MASK)
#define XBARA_CTRL1_STS2_MASK                    0x10u
#define XBARA_CTRL1_STS2_SHIFT                   4
#define XBARA_CTRL1_DEN3_MASK                    0x100u
#define XBARA_CTRL1_DEN3_SHIFT                   8
#define XBARA_CTRL1_IEN3_MASK                    0x200u
#define XBARA_CTRL1_IEN3_SHIFT                   9
#define XBARA_CTRL1_EDGE3_MASK                   0xC00u
#define XBARA_CTRL1_EDGE3_SHIFT                  10
#define XBARA_CTRL1_EDGE3(x)                     (((uint16_t)(((uint16_t)(x))<<XBARA_CTRL1_EDGE3_SHIFT))&XBARA_CTRL1_EDGE3_MASK)
#define XBARA_CTRL1_STS3_MASK                    0x1000u
#define XBARA_CTRL1_STS3_SHIFT                   12

/*!
 * @}
 */ /* end of group XBARA_Register_Masks */


/* XBARA - Peripheral instance base addresses */
/** Peripheral XBARA base pointer */
#define XBARA_BASE_PTR                           ((XBARA_MemMapPtr)0xE340u)
/** Array initializer of XBARA peripheral base pointers */
#define XBARA_BASE_PTRS                          { XBARA_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- XBARA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Accessor_Macros XBARA - Register accessor macros
 * @{
 */


/* XBARA - Register instance definitions */
/* XBARA */
#define XBARA_SEL0                               XBARA_SEL0_REG(XBARA_BASE_PTR)
#define XBARA_SEL1                               XBARA_SEL1_REG(XBARA_BASE_PTR)
#define XBARA_SEL2                               XBARA_SEL2_REG(XBARA_BASE_PTR)
#define XBARA_SEL3                               XBARA_SEL3_REG(XBARA_BASE_PTR)
#define XBARA_SEL4                               XBARA_SEL4_REG(XBARA_BASE_PTR)
#define XBARA_SEL5                               XBARA_SEL5_REG(XBARA_BASE_PTR)
#define XBARA_SEL6                               XBARA_SEL6_REG(XBARA_BASE_PTR)
#define XBARA_SEL7                               XBARA_SEL7_REG(XBARA_BASE_PTR)
#define XBARA_SEL8                               XBARA_SEL8_REG(XBARA_BASE_PTR)
#define XBARA_SEL9                               XBARA_SEL9_REG(XBARA_BASE_PTR)
#define XBARA_SEL10                              XBARA_SEL10_REG(XBARA_BASE_PTR)
#define XBARA_SEL11                              XBARA_SEL11_REG(XBARA_BASE_PTR)
#define XBARA_SEL12                              XBARA_SEL12_REG(XBARA_BASE_PTR)
#define XBARA_SEL13                              XBARA_SEL13_REG(XBARA_BASE_PTR)
#define XBARA_SEL14                              XBARA_SEL14_REG(XBARA_BASE_PTR)
#define XBARA_SEL15                              XBARA_SEL15_REG(XBARA_BASE_PTR)
#define XBARA_SEL16                              XBARA_SEL16_REG(XBARA_BASE_PTR)
#define XBARA_SEL17                              XBARA_SEL17_REG(XBARA_BASE_PTR)
#define XBARA_SEL18                              XBARA_SEL18_REG(XBARA_BASE_PTR)
#define XBARA_SEL19                              XBARA_SEL19_REG(XBARA_BASE_PTR)
#define XBARA_SEL20                              XBARA_SEL20_REG(XBARA_BASE_PTR)
#define XBARA_CTRL0                              XBARA_CTRL0_REG(XBARA_BASE_PTR)
#define XBARA_CTRL1                              XBARA_CTRL1_REG(XBARA_BASE_PTR)

/*!
 * @}
 */ /* end of group XBARA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group XBARA_Peripheral */


/* ----------------------------------------------------------------------------
   -- XBARB
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Peripheral XBARB
 * @{
 */

/** XBARB - Peripheral register structure */
typedef struct XBARB_MemMap {
  uint16_t SEL0;                                   /**< Crossbar B Select Register 0, offset: 0x0 */
  uint16_t SEL1;                                   /**< Crossbar B Select Register 1, offset: 0x1 */
  uint16_t SEL2;                                   /**< Crossbar B Select Register 2, offset: 0x2 */
  uint16_t SEL3;                                   /**< Crossbar B Select Register 3, offset: 0x3 */
  uint16_t SEL4;                                   /**< Crossbar B Select Register 4, offset: 0x4 */
  uint16_t SEL5;                                   /**< Crossbar B Select Register 5, offset: 0x5 */
  uint16_t SEL6;                                   /**< Crossbar B Select Register 6, offset: 0x6 */
  uint16_t SEL7;                                   /**< Crossbar B Select Register 7, offset: 0x7 */
} volatile *XBARB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- XBARB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Accessor_Macros XBARB - Register accessor macros
 * @{
 */


/* XBARB - Register accessors */
#define XBARB_SEL0_REG(base)                     ((base)->SEL0)
#define XBARB_SEL1_REG(base)                     ((base)->SEL1)
#define XBARB_SEL2_REG(base)                     ((base)->SEL2)
#define XBARB_SEL3_REG(base)                     ((base)->SEL3)
#define XBARB_SEL4_REG(base)                     ((base)->SEL4)
#define XBARB_SEL5_REG(base)                     ((base)->SEL5)
#define XBARB_SEL6_REG(base)                     ((base)->SEL6)
#define XBARB_SEL7_REG(base)                     ((base)->SEL7)

/*!
 * @}
 */ /* end of group XBARB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- XBARB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Masks XBARB Register Masks
 * @{
 */

/* SEL0 Bit Fields */
#define XBARB_SEL0_SEL0_MASK                     0x1Fu
#define XBARB_SEL0_SEL0_SHIFT                    0
#define XBARB_SEL0_SEL0(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL0_SEL0_SHIFT))&XBARB_SEL0_SEL0_MASK)
#define XBARB_SEL0_SEL1_MASK                     0x1F00u
#define XBARB_SEL0_SEL1_SHIFT                    8
#define XBARB_SEL0_SEL1(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL0_SEL1_SHIFT))&XBARB_SEL0_SEL1_MASK)
/* SEL1 Bit Fields */
#define XBARB_SEL1_SEL2_MASK                     0x1Fu
#define XBARB_SEL1_SEL2_SHIFT                    0
#define XBARB_SEL1_SEL2(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL1_SEL2_SHIFT))&XBARB_SEL1_SEL2_MASK)
#define XBARB_SEL1_SEL3_MASK                     0x1F00u
#define XBARB_SEL1_SEL3_SHIFT                    8
#define XBARB_SEL1_SEL3(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL1_SEL3_SHIFT))&XBARB_SEL1_SEL3_MASK)
/* SEL2 Bit Fields */
#define XBARB_SEL2_SEL4_MASK                     0x1Fu
#define XBARB_SEL2_SEL4_SHIFT                    0
#define XBARB_SEL2_SEL4(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL2_SEL4_SHIFT))&XBARB_SEL2_SEL4_MASK)
#define XBARB_SEL2_SEL5_MASK                     0x1F00u
#define XBARB_SEL2_SEL5_SHIFT                    8
#define XBARB_SEL2_SEL5(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL2_SEL5_SHIFT))&XBARB_SEL2_SEL5_MASK)
/* SEL3 Bit Fields */
#define XBARB_SEL3_SEL6_MASK                     0x1Fu
#define XBARB_SEL3_SEL6_SHIFT                    0
#define XBARB_SEL3_SEL6(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL3_SEL6_SHIFT))&XBARB_SEL3_SEL6_MASK)
#define XBARB_SEL3_SEL7_MASK                     0x1F00u
#define XBARB_SEL3_SEL7_SHIFT                    8
#define XBARB_SEL3_SEL7(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL3_SEL7_SHIFT))&XBARB_SEL3_SEL7_MASK)
/* SEL4 Bit Fields */
#define XBARB_SEL4_SEL8_MASK                     0x1Fu
#define XBARB_SEL4_SEL8_SHIFT                    0
#define XBARB_SEL4_SEL8(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL4_SEL8_SHIFT))&XBARB_SEL4_SEL8_MASK)
#define XBARB_SEL4_SEL9_MASK                     0x1F00u
#define XBARB_SEL4_SEL9_SHIFT                    8
#define XBARB_SEL4_SEL9(x)                       (((uint16_t)(((uint16_t)(x))<<XBARB_SEL4_SEL9_SHIFT))&XBARB_SEL4_SEL9_MASK)
/* SEL5 Bit Fields */
#define XBARB_SEL5_SEL10_MASK                    0x1Fu
#define XBARB_SEL5_SEL10_SHIFT                   0
#define XBARB_SEL5_SEL10(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL5_SEL10_SHIFT))&XBARB_SEL5_SEL10_MASK)
#define XBARB_SEL5_SEL11_MASK                    0x1F00u
#define XBARB_SEL5_SEL11_SHIFT                   8
#define XBARB_SEL5_SEL11(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL5_SEL11_SHIFT))&XBARB_SEL5_SEL11_MASK)
/* SEL6 Bit Fields */
#define XBARB_SEL6_SEL12_MASK                    0x1Fu
#define XBARB_SEL6_SEL12_SHIFT                   0
#define XBARB_SEL6_SEL12(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL6_SEL12_SHIFT))&XBARB_SEL6_SEL12_MASK)
#define XBARB_SEL6_SEL13_MASK                    0x1F00u
#define XBARB_SEL6_SEL13_SHIFT                   8
#define XBARB_SEL6_SEL13(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL6_SEL13_SHIFT))&XBARB_SEL6_SEL13_MASK)
/* SEL7 Bit Fields */
#define XBARB_SEL7_SEL14_MASK                    0x1Fu
#define XBARB_SEL7_SEL14_SHIFT                   0
#define XBARB_SEL7_SEL14(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL7_SEL14_SHIFT))&XBARB_SEL7_SEL14_MASK)
#define XBARB_SEL7_SEL15_MASK                    0x1F00u
#define XBARB_SEL7_SEL15_SHIFT                   8
#define XBARB_SEL7_SEL15(x)                      (((uint16_t)(((uint16_t)(x))<<XBARB_SEL7_SEL15_SHIFT))&XBARB_SEL7_SEL15_MASK)

/*!
 * @}
 */ /* end of group XBARB_Register_Masks */


/* XBARB - Peripheral instance base addresses */
/** Peripheral XBARB base pointer */
#define XBARB_BASE_PTR                           ((XBARB_MemMapPtr)0xE360u)
/** Array initializer of XBARB peripheral base pointers */
#define XBARB_BASE_PTRS                          { XBARB_BASE_PTR }

/* ----------------------------------------------------------------------------
   -- XBARB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARB_Register_Accessor_Macros XBARB - Register accessor macros
 * @{
 */


/* XBARB - Register instance definitions */
/* XBARB */
#define XBARB_SEL0                               XBARB_SEL0_REG(XBARB_BASE_PTR)
#define XBARB_SEL1                               XBARB_SEL1_REG(XBARB_BASE_PTR)
#define XBARB_SEL2                               XBARB_SEL2_REG(XBARB_BASE_PTR)
#define XBARB_SEL3                               XBARB_SEL3_REG(XBARB_BASE_PTR)
#define XBARB_SEL4                               XBARB_SEL4_REG(XBARB_BASE_PTR)
#define XBARB_SEL5                               XBARB_SEL5_REG(XBARB_BASE_PTR)
#define XBARB_SEL6                               XBARB_SEL6_REG(XBARB_BASE_PTR)
#define XBARB_SEL7                               XBARB_SEL7_REG(XBARB_BASE_PTR)

/*!
 * @}
 */ /* end of group XBARB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group XBARB_Peripheral */


/*
** End of section using anonymous unions
*/

#if defined(__m56800E__)
  /* leave anonymous unions enabled */
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_defines */


/* ----------------------------------------------------------------------------
   -- Backward Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Backward_Compatibility_Symbols Backward Compatibility
 * @{
 */

/* No backward compatibility issues. */

/*!
 * @}
 */ /* end of group Backward_Compatibility_Symbols */


#else /* #if !defined(MCU_MC56F82748) */
  /* There is already included the same memory map. Check if it is compatible (has the same major version) */
  #if (MCU_MEM_MAP_VERSION != 0x0000u)
    #if (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING))
      #warning There are included two not compatible versions of memory maps. Please check possible differences.
    #endif /* (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING)) */
  #endif /* (MCU_MEM_MAP_VERSION != 0x0000u) */
#endif  /* #if !defined(MCU_MC56F82748) */

/* MC56F82748.h, eof. */
