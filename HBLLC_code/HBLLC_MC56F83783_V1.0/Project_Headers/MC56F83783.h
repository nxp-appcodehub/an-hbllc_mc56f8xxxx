/*
** ###################################################################
**     Processor:           MC56F83783
**     Build:               b190724
**
**     Abstract:
**         Peripheral Access Layer for MC56F83783
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2019 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**
** ###################################################################
*/

/*!
 * @file MC56F83783.h
 * @version 0.0
 * @date 0-00-00
 * @brief Peripheral Access Layer for MC56F83783
 *
 * Peripheral Access Layer for MC56F83783
 */

#ifndef _MC56F83783_H_
#define _MC56F83783_H_                           /**< Symbol preventing repeated inclusion */


#include <stdint.h>

#define __IO volatile
#define __I const volatile
#define __O volatile

#ifdef __cplusplus
  typedef unsigned char uint8_t;
  typedef unsigned short uint16_t;
  typedef unsigned int uint32_t;
#endif

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/


/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL1;                             /**< ADC Control Register 1, offset: 0x0 */
  __IO uint16_t CTRL2;                             /**< ADC Control Register 2, offset: 0x1 */
  __IO uint16_t ZXCTRL1;                           /**< ADC Zero Crossing Control 1 Register, offset: 0x2 */
  __IO uint16_t ZXCTRL2;                           /**< ADC Zero Crossing Control 2 Register, offset: 0x3 */
  __IO uint16_t CLIST1;                            /**< ADC Channel List Register 1, offset: 0x4 */
  __IO uint16_t CLIST2;                            /**< ADC Channel List Register 2, offset: 0x5 */
  __IO uint16_t CLIST3;                            /**< ADC Channel List Register 3, offset: 0x6 */
  __IO uint16_t CLIST4;                            /**< ADC Channel List Register 4, offset: 0x7 */
  __IO uint16_t SDIS;                              /**< ADC Sample Disable Register, offset: 0x8 */
  __IO uint16_t STAT;                              /**< ADC Status Register, offset: 0x9 */
  __I  uint16_t RDY;                               /**< ADC Ready Register, offset: 0xA */
  __IO uint16_t LOLIMSTAT;                         /**< ADC Low Limit Status Register, offset: 0xB */
  __IO uint16_t HILIMSTAT;                         /**< ADC High Limit Status Register, offset: 0xC */
  __IO uint16_t ZXSTAT;                            /**< ADC Zero Crossing Status Register, offset: 0xD */
  __IO uint16_t RSLT[16];                          /**< ADC Result Registers with sign extension, array offset: 0xE, array step: 0x1 */
  __IO uint16_t LOLIM[16];                         /**< ADC Low Limit Registers, array offset: 0x1E, array step: 0x1 */
  __IO uint16_t HILIM[16];                         /**< ADC High Limit Registers, array offset: 0x2E, array step: 0x1 */
  __IO uint16_t OFFST[16];                         /**< ADC Offset Registers, array offset: 0x3E, array step: 0x1 */
  __IO uint16_t PWR;                               /**< ADC Power Control Register, offset: 0x4E */
  __IO uint16_t CAL;                               /**< ADC Calibration Register, offset: 0x4F */
  __IO uint16_t GC1;                               /**< Gain Control 1 Register, offset: 0x50 */
  __IO uint16_t GC2;                               /**< Gain Control 2 Register, offset: 0x51 */
  __IO uint16_t SCTRL;                             /**< ADC Scan Control Register, offset: 0x52 */
  __IO uint16_t PWR2;                              /**< ADC Power Control Register 2, offset: 0x53 */
  __IO uint16_t CTRL3;                             /**< ADC Control Register 3, offset: 0x54 */
  __IO uint16_t SCHLTEN;                           /**< ADC Scan Interrupt Enable Register, offset: 0x55 */
       uint16_t RESERVED_0[2];
  __IO uint16_t ZXCTRL3;                           /**< ADC Zero Crossing Control 3 Register, offset: 0x58 */
  __IO uint16_t CLIST5;                            /**< ADC Channel List Register 5, offset: 0x59 */
  __IO uint16_t SDIS2;                             /**< ADC Sample Disable Register 2, offset: 0x5A */
  __I  uint16_t RDY2;                              /**< ADC Ready Register 2, offset: 0x5B */
  __IO uint16_t LOLIMSTAT2;                        /**< ADC Low Limit Status Register 2, offset: 0x5C */
  __IO uint16_t HILIMSTAT2;                        /**< ADC High Limit Status Register 2, offset: 0x5D */
  __IO uint16_t ZXSTAT2;                           /**< ADC Zero Crossing Status Register 2, offset: 0x5E */
  __IO uint16_t RSLT2[4];                          /**< ADC Result Registers 2 with sign extension, array offset: 0x5F, array step: 0x1 */
  __IO uint16_t LOLIM2[4];                         /**< ADC Low Limit Registers 2, array offset: 0x63, array step: 0x1 */
  __IO uint16_t HILIM2[4];                         /**< ADC High Limit Registers 2, array offset: 0x67, array step: 0x1 */
  __IO uint16_t OFFST2[4];                         /**< ADC Offset Registers 2, array offset: 0x6B, array step: 0x1 */
  __IO uint16_t GC3;                               /**< Gain Control 3 Register, offset: 0x6F */
  __IO uint16_t SCTRL2;                            /**< ADC Scan Control Register 2, offset: 0x70 */
  __IO uint16_t SCHLTEN2;                          /**< ADC Scan Interrupt Enable Register 2, offset: 0x71 */
} ADC_Type;

/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/*! @name CTRL1 - ADC Control Register 1 */
/*! @{ */
/*! SMODE - ADC Scan Mode Control
 *  0b000..Once (single) sequential
 *  0b001..Once parallel
 *  0b010..Loop sequential
 *  0b011..Loop parallel
 *  0b100..Triggered sequential
 *  0b101..Triggered parallel (default)
 *  0b11x..Reserved
 */
#define ADC_CTRL1_SMODE_MASK                     (0x7U)
#define ADC_CTRL1_SMODE_SHIFT                    (0U)
#define ADC_CTRL1_SMODE(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_SMODE_SHIFT)) & ADC_CTRL1_SMODE_MASK)
/*! CHNCFG_L - CHCNF (Channel Configure Low) bits
 *  0bxxx1..Inputs = ANA0-ANA1
 *  0bxxx0..Inputs = ANA0-ANA1
 *  0bxx1x..Inputs = ANA2-ANA3
 *  0bxx0x..Inputs = ANA2-ANA3
 *  0bx1xx..Inputs = ANB0-ANB1
 *  0bx0xx..Inputs = ANB0-ANB1
 *  0b1xxx..Inputs = ANB2-ANB3
 */
#define ADC_CTRL1_CHNCFG_L_MASK                  (0xF0U)
#define ADC_CTRL1_CHNCFG_L_SHIFT                 (4U)
#define ADC_CTRL1_CHNCFG_L(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_CHNCFG_L_SHIFT)) & ADC_CTRL1_CHNCFG_L_MASK)
/*! HLMTIE - High Limit Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_HLMTIE_MASK                    (0x100U)
#define ADC_CTRL1_HLMTIE_SHIFT                   (8U)
#define ADC_CTRL1_HLMTIE(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_HLMTIE_SHIFT)) & ADC_CTRL1_HLMTIE_MASK)
/*! LLMTIE - Low Limit Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_LLMTIE_MASK                    (0x200U)
#define ADC_CTRL1_LLMTIE_SHIFT                   (9U)
#define ADC_CTRL1_LLMTIE(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_LLMTIE_SHIFT)) & ADC_CTRL1_LLMTIE_MASK)
/*! ZCIE - Zero Crossing Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_ZCIE_MASK                      (0x400U)
#define ADC_CTRL1_ZCIE_SHIFT                     (10U)
#define ADC_CTRL1_ZCIE(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_ZCIE_SHIFT)) & ADC_CTRL1_ZCIE_MASK)
/*! EOSIE0 - End Of Scan Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL1_EOSIE0_MASK                    (0x800U)
#define ADC_CTRL1_EOSIE0_SHIFT                   (11U)
#define ADC_CTRL1_EOSIE0(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_EOSIE0_SHIFT)) & ADC_CTRL1_EOSIE0_MASK)
/*! SYNC0 - SYNC0 Enable
 *  0b0..Scan is initiated by a write to CTRL1[START0] only
 *  0b1..Use a SYNC0 input pulse or CTRL1[START0] to initiate a scan
 */
#define ADC_CTRL1_SYNC0_MASK                     (0x1000U)
#define ADC_CTRL1_SYNC0_SHIFT                    (12U)
#define ADC_CTRL1_SYNC0(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_SYNC0_SHIFT)) & ADC_CTRL1_SYNC0_MASK)
/*! START0 - START0 Conversion
 *  0b0..No action
 *  0b1..Start command is issued
 */
#define ADC_CTRL1_START0_MASK                    (0x2000U)
#define ADC_CTRL1_START0_SHIFT                   (13U)
#define ADC_CTRL1_START0(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_START0_SHIFT)) & ADC_CTRL1_START0_MASK)
/*! STOP0 - Stop
 *  0b0..Normal operation
 *  0b1..Stop mode
 */
#define ADC_CTRL1_STOP0_MASK                     (0x4000U)
#define ADC_CTRL1_STOP0_SHIFT                    (14U)
#define ADC_CTRL1_STOP0(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_STOP0_SHIFT)) & ADC_CTRL1_STOP0_MASK)
/*! DMAEN0 - DMA enable
 *  0b0..DMA is not enabled.
 *  0b1..DMA is enabled.
 */
#define ADC_CTRL1_DMAEN0_MASK                    (0x8000U)
#define ADC_CTRL1_DMAEN0_SHIFT                   (15U)
#define ADC_CTRL1_DMAEN0(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL1_DMAEN0_SHIFT)) & ADC_CTRL1_DMAEN0_MASK)
/*! @} */

/*! @name CTRL2 - ADC Control Register 2 */
/*! @{ */
#define ADC_CTRL2_DIV0_MASK                      (0x3FU)
#define ADC_CTRL2_DIV0_SHIFT                     (0U)
#define ADC_CTRL2_DIV0(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_DIV0_SHIFT)) & ADC_CTRL2_DIV0_MASK)
/*! SIMULT - Simultaneous mode
 *  0b0..Parallel scans done independently
 *  0b1..Parallel scans done simultaneously (default)
 */
#define ADC_CTRL2_SIMULT_MASK                    (0x40U)
#define ADC_CTRL2_SIMULT_SHIFT                   (6U)
#define ADC_CTRL2_SIMULT(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_SIMULT_SHIFT)) & ADC_CTRL2_SIMULT_MASK)
/*! CHNCFG_H - CHCNF (Channel Configure High) bits
 *  0bxxx1..Inputs = ANA4-ANA5
 *  0bxxx0..Inputs = ANA4-ANA5
 *  0bxx1x..Inputs = ANA6-ANA7
 *  0bxx0x..Inputs = ANA6-ANA7
 *  0bx1xx..Inputs = ANB4-ANB5
 *  0bx0xx..Inputs = ANB4-ANB5
 *  0b1xxx..Inputs = ANB6-ANB7
 */
#define ADC_CTRL2_CHNCFG_H_MASK                  (0x780U)
#define ADC_CTRL2_CHNCFG_H_SHIFT                 (7U)
#define ADC_CTRL2_CHNCFG_H(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_CHNCFG_H_SHIFT)) & ADC_CTRL2_CHNCFG_H_MASK)
/*! EOSIE1 - End Of Scan Interrupt Enable
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define ADC_CTRL2_EOSIE1_MASK                    (0x800U)
#define ADC_CTRL2_EOSIE1_SHIFT                   (11U)
#define ADC_CTRL2_EOSIE1(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_EOSIE1_SHIFT)) & ADC_CTRL2_EOSIE1_MASK)
/*! SYNC1 - SYNC1 Enable
 *  0b0..B converter parallel scan is initiated by a write to CTRL2[START1] bit only
 *  0b1..Use a SYNC1 input pulse or CTRL2[START1] bit to initiate a B converter parallel scan
 */
#define ADC_CTRL2_SYNC1_MASK                     (0x1000U)
#define ADC_CTRL2_SYNC1_SHIFT                    (12U)
#define ADC_CTRL2_SYNC1(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_SYNC1_SHIFT)) & ADC_CTRL2_SYNC1_MASK)
/*! START1 - START1 Conversion
 *  0b0..No action
 *  0b1..Start command is issued
 */
#define ADC_CTRL2_START1_MASK                    (0x2000U)
#define ADC_CTRL2_START1_SHIFT                   (13U)
#define ADC_CTRL2_START1(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_START1_SHIFT)) & ADC_CTRL2_START1_MASK)
/*! STOP1 - Stop
 *  0b0..Normal operation
 *  0b1..Stop mode
 */
#define ADC_CTRL2_STOP1_MASK                     (0x4000U)
#define ADC_CTRL2_STOP1_SHIFT                    (14U)
#define ADC_CTRL2_STOP1(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_STOP1_SHIFT)) & ADC_CTRL2_STOP1_MASK)
/*! DMAEN1 - DMA enable
 *  0b0..DMA is not enabled.
 *  0b1..DMA is enabled.
 */
#define ADC_CTRL2_DMAEN1_MASK                    (0x8000U)
#define ADC_CTRL2_DMAEN1_SHIFT                   (15U)
#define ADC_CTRL2_DMAEN1(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL2_DMAEN1_SHIFT)) & ADC_CTRL2_DMAEN1_MASK)
/*! @} */

/*! @name ZXCTRL1 - ADC Zero Crossing Control 1 Register */
/*! @{ */
/*! ZCE0 - Zero crossing enable 0
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE0_MASK                    (0x3U)
#define ADC_ZXCTRL1_ZCE0_SHIFT                   (0U)
#define ADC_ZXCTRL1_ZCE0(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE0_SHIFT)) & ADC_ZXCTRL1_ZCE0_MASK)
/*! ZCE1 - Zero crossing enable 1
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE1_MASK                    (0xCU)
#define ADC_ZXCTRL1_ZCE1_SHIFT                   (2U)
#define ADC_ZXCTRL1_ZCE1(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE1_SHIFT)) & ADC_ZXCTRL1_ZCE1_MASK)
/*! ZCE2 - Zero crossing enable 2
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE2_MASK                    (0x30U)
#define ADC_ZXCTRL1_ZCE2_SHIFT                   (4U)
#define ADC_ZXCTRL1_ZCE2(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE2_SHIFT)) & ADC_ZXCTRL1_ZCE2_MASK)
/*! ZCE3 - Zero crossing enable 3
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE3_MASK                    (0xC0U)
#define ADC_ZXCTRL1_ZCE3_SHIFT                   (6U)
#define ADC_ZXCTRL1_ZCE3(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE3_SHIFT)) & ADC_ZXCTRL1_ZCE3_MASK)
/*! ZCE4 - Zero crossing enable 4
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE4_MASK                    (0x300U)
#define ADC_ZXCTRL1_ZCE4_SHIFT                   (8U)
#define ADC_ZXCTRL1_ZCE4(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE4_SHIFT)) & ADC_ZXCTRL1_ZCE4_MASK)
/*! ZCE5 - Zero crossing enable 5
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE5_MASK                    (0xC00U)
#define ADC_ZXCTRL1_ZCE5_SHIFT                   (10U)
#define ADC_ZXCTRL1_ZCE5(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE5_SHIFT)) & ADC_ZXCTRL1_ZCE5_MASK)
/*! ZCE6 - Zero crossing enable 6
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE6_MASK                    (0x3000U)
#define ADC_ZXCTRL1_ZCE6_SHIFT                   (12U)
#define ADC_ZXCTRL1_ZCE6(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE6_SHIFT)) & ADC_ZXCTRL1_ZCE6_MASK)
/*! ZCE7 - Zero crossing enable 7
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL1_ZCE7_MASK                    (0xC000U)
#define ADC_ZXCTRL1_ZCE7_SHIFT                   (14U)
#define ADC_ZXCTRL1_ZCE7(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL1_ZCE7_SHIFT)) & ADC_ZXCTRL1_ZCE7_MASK)
/*! @} */

/*! @name ZXCTRL2 - ADC Zero Crossing Control 2 Register */
/*! @{ */
/*! ZCE8 - Zero crossing enable 8
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE8_MASK                    (0x3U)
#define ADC_ZXCTRL2_ZCE8_SHIFT                   (0U)
#define ADC_ZXCTRL2_ZCE8(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE8_SHIFT)) & ADC_ZXCTRL2_ZCE8_MASK)
/*! ZCE9 - Zero crossing enable 9
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE9_MASK                    (0xCU)
#define ADC_ZXCTRL2_ZCE9_SHIFT                   (2U)
#define ADC_ZXCTRL2_ZCE9(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE9_SHIFT)) & ADC_ZXCTRL2_ZCE9_MASK)
/*! ZCE10 - Zero crossing enable 10
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE10_MASK                   (0x30U)
#define ADC_ZXCTRL2_ZCE10_SHIFT                  (4U)
#define ADC_ZXCTRL2_ZCE10(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE10_SHIFT)) & ADC_ZXCTRL2_ZCE10_MASK)
/*! ZCE11 - Zero crossing enable 11
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE11_MASK                   (0xC0U)
#define ADC_ZXCTRL2_ZCE11_SHIFT                  (6U)
#define ADC_ZXCTRL2_ZCE11(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE11_SHIFT)) & ADC_ZXCTRL2_ZCE11_MASK)
/*! ZCE12 - Zero crossing enable 12
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE12_MASK                   (0x300U)
#define ADC_ZXCTRL2_ZCE12_SHIFT                  (8U)
#define ADC_ZXCTRL2_ZCE12(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE12_SHIFT)) & ADC_ZXCTRL2_ZCE12_MASK)
/*! ZCE13 - Zero crossing enable 13
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE13_MASK                   (0xC00U)
#define ADC_ZXCTRL2_ZCE13_SHIFT                  (10U)
#define ADC_ZXCTRL2_ZCE13(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE13_SHIFT)) & ADC_ZXCTRL2_ZCE13_MASK)
/*! ZCE14 - Zero crossing enable 14
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE14_MASK                   (0x3000U)
#define ADC_ZXCTRL2_ZCE14_SHIFT                  (12U)
#define ADC_ZXCTRL2_ZCE14(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE14_SHIFT)) & ADC_ZXCTRL2_ZCE14_MASK)
/*! ZCE15 - Zero crossing enable 15
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL2_ZCE15_MASK                   (0xC000U)
#define ADC_ZXCTRL2_ZCE15_SHIFT                  (14U)
#define ADC_ZXCTRL2_ZCE15(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL2_ZCE15_SHIFT)) & ADC_ZXCTRL2_ZCE15_MASK)
/*! @} */

/*! @name CLIST1 - ADC Channel List Register 1 */
/*! @{ */
/*! SAMPLE0 - Sample Field 0
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE0_MASK                  (0xFU)
#define ADC_CLIST1_SAMPLE0_SHIFT                 (0U)
#define ADC_CLIST1_SAMPLE0(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE0_SHIFT)) & ADC_CLIST1_SAMPLE0_MASK)
/*! SAMPLE1 - Sample Field 1
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE1_MASK                  (0xF0U)
#define ADC_CLIST1_SAMPLE1_SHIFT                 (4U)
#define ADC_CLIST1_SAMPLE1(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE1_SHIFT)) & ADC_CLIST1_SAMPLE1_MASK)
/*! SAMPLE2 - Sample Field 2
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE2_MASK                  (0xF00U)
#define ADC_CLIST1_SAMPLE2_SHIFT                 (8U)
#define ADC_CLIST1_SAMPLE2(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE2_SHIFT)) & ADC_CLIST1_SAMPLE2_MASK)
/*! SAMPLE3 - Sample Field 3
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST1_SAMPLE3_MASK                  (0xF000U)
#define ADC_CLIST1_SAMPLE3_SHIFT                 (12U)
#define ADC_CLIST1_SAMPLE3(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST1_SAMPLE3_SHIFT)) & ADC_CLIST1_SAMPLE3_MASK)
/*! @} */

/*! @name CLIST2 - ADC Channel List Register 2 */
/*! @{ */
/*! SAMPLE4 - Sample Field 4
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE4_MASK                  (0xFU)
#define ADC_CLIST2_SAMPLE4_SHIFT                 (0U)
#define ADC_CLIST2_SAMPLE4(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE4_SHIFT)) & ADC_CLIST2_SAMPLE4_MASK)
/*! SAMPLE5 - Sample Field 5
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE5_MASK                  (0xF0U)
#define ADC_CLIST2_SAMPLE5_SHIFT                 (4U)
#define ADC_CLIST2_SAMPLE5(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE5_SHIFT)) & ADC_CLIST2_SAMPLE5_MASK)
/*! SAMPLE6 - Sample Field 6
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE6_MASK                  (0xF00U)
#define ADC_CLIST2_SAMPLE6_SHIFT                 (8U)
#define ADC_CLIST2_SAMPLE6(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE6_SHIFT)) & ADC_CLIST2_SAMPLE6_MASK)
/*! SAMPLE7 - Sample Field 7
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST2_SAMPLE7_MASK                  (0xF000U)
#define ADC_CLIST2_SAMPLE7_SHIFT                 (12U)
#define ADC_CLIST2_SAMPLE7(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST2_SAMPLE7_SHIFT)) & ADC_CLIST2_SAMPLE7_MASK)
/*! @} */

/*! @name CLIST3 - ADC Channel List Register 3 */
/*! @{ */
/*! SAMPLE8 - Sample Field 8
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE8_MASK                  (0xFU)
#define ADC_CLIST3_SAMPLE8_SHIFT                 (0U)
#define ADC_CLIST3_SAMPLE8(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE8_SHIFT)) & ADC_CLIST3_SAMPLE8_MASK)
/*! SAMPLE9 - Sample Field 9
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE9_MASK                  (0xF0U)
#define ADC_CLIST3_SAMPLE9_SHIFT                 (4U)
#define ADC_CLIST3_SAMPLE9(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE9_SHIFT)) & ADC_CLIST3_SAMPLE9_MASK)
/*! SAMPLE10 - Sample Field 10
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE10_MASK                 (0xF00U)
#define ADC_CLIST3_SAMPLE10_SHIFT                (8U)
#define ADC_CLIST3_SAMPLE10(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE10_SHIFT)) & ADC_CLIST3_SAMPLE10_MASK)
/*! SAMPLE11 - Sample Field 11
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST3_SAMPLE11_MASK                 (0xF000U)
#define ADC_CLIST3_SAMPLE11_SHIFT                (12U)
#define ADC_CLIST3_SAMPLE11(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST3_SAMPLE11_SHIFT)) & ADC_CLIST3_SAMPLE11_MASK)
/*! @} */

/*! @name CLIST4 - ADC Channel List Register 4 */
/*! @{ */
/*! SAMPLE12 - Sample Field 12
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE12_MASK                 (0xFU)
#define ADC_CLIST4_SAMPLE12_SHIFT                (0U)
#define ADC_CLIST4_SAMPLE12(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE12_SHIFT)) & ADC_CLIST4_SAMPLE12_MASK)
/*! SAMPLE13 - Sample Field 13
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE13_MASK                 (0xF0U)
#define ADC_CLIST4_SAMPLE13_SHIFT                (4U)
#define ADC_CLIST4_SAMPLE13(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE13_SHIFT)) & ADC_CLIST4_SAMPLE13_MASK)
/*! SAMPLE14 - Sample Field 14
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE14_MASK                 (0xF00U)
#define ADC_CLIST4_SAMPLE14_SHIFT                (8U)
#define ADC_CLIST4_SAMPLE14(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE14_SHIFT)) & ADC_CLIST4_SAMPLE14_MASK)
/*! SAMPLE15 - Sample Field 15
 *  0b0000..Single Ended: ANA0, Differential: ANA0+, ANA1-
 *  0b0001..Single Ended: ANA1, Differential: ANA0+, ANA1-
 *  0b0010..Single Ended: ANA2, Differential: ANA2+, ANA3-
 *  0b0011..Single Ended: ANA3, Differential: ANA2+, ANA3-
 *  0b0100..Single Ended: ANA4, Differential: ANA4+, ANA5-
 *  0b0101..Single Ended: ANA5, Differential: ANA4+, ANA5-
 *  0b0110..Single Ended: ANA6, Differential: ANA6+, ANA7-
 *  0b0111..Single Ended: ANA7, Differential: ANA6+, ANA7-
 *  0b1000..Single Ended: ANB0, Differential: ANB0+, ANB1-
 *  0b1001..Single Ended: ANB1, Differential: ANB0+, ANB1-
 *  0b1010..Single Ended: ANB2, Differential: ANB2+, ANB3-
 *  0b1011..Single Ended: ANB3, Differential: ANB2+, ANB3-
 *  0b1100..Single Ended: ANB4, Differential: ANB4+, ANB5-
 *  0b1101..Single Ended: ANB5, Differential: ANB4+, ANB5-
 *  0b1110..Single Ended: ANB6, Differential: ANB6+, ANB7-
 *  0b1111..Single Ended: ANB7, Differential: ANB6+, ANB7-
 */
#define ADC_CLIST4_SAMPLE15_MASK                 (0xF000U)
#define ADC_CLIST4_SAMPLE15_SHIFT                (12U)
#define ADC_CLIST4_SAMPLE15(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST4_SAMPLE15_SHIFT)) & ADC_CLIST4_SAMPLE15_MASK)
/*! @} */

/*! @name SDIS - ADC Sample Disable Register */
/*! @{ */
/*! DS - Disable Sample Bits
 *  0b0000000000000000..SAMPLEx channel is enabled for ADC scan.
 *  0b0000000000000001..SAMPLEx channel is disabled for ADC scan and corresponding channels after SAMPLEx will also not occur in an ADC scan.
 */
#define ADC_SDIS_DS_MASK                         (0xFFFFU)
#define ADC_SDIS_DS_SHIFT                        (0U)
#define ADC_SDIS_DS(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_SDIS_DS_SHIFT)) & ADC_SDIS_DS_MASK)
/*! @} */

/*! @name STAT - ADC Status Register */
/*! @{ */
#define ADC_STAT_UNDEFINED_MASK                  (0xFFU)
#define ADC_STAT_UNDEFINED_SHIFT                 (0U)
#define ADC_STAT_UNDEFINED(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_STAT_UNDEFINED_SHIFT)) & ADC_STAT_UNDEFINED_MASK)
/*! HLMTI - High Limit Interrupt
 *  0b0..No high limit interrupt request
 *  0b1..High limit exceeded, IRQ pending if CTRL1[HLMTIE] is set
 */
#define ADC_STAT_HLMTI_MASK                      (0x100U)
#define ADC_STAT_HLMTI_SHIFT                     (8U)
#define ADC_STAT_HLMTI(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_STAT_HLMTI_SHIFT)) & ADC_STAT_HLMTI_MASK)
/*! LLMTI - Low Limit Interrupt
 *  0b0..No low limit interrupt request
 *  0b1..Low limit exceeded, IRQ pending if CTRL1[LLMTIE] is set
 */
#define ADC_STAT_LLMTI_MASK                      (0x200U)
#define ADC_STAT_LLMTI_SHIFT                     (9U)
#define ADC_STAT_LLMTI(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_STAT_LLMTI_SHIFT)) & ADC_STAT_LLMTI_MASK)
/*! ZCI - Zero Crossing Interrupt
 *  0b0..No zero crossing interrupt request
 *  0b1..Zero crossing encountered, IRQ pending if CTRL1[ZCIE] is set
 */
#define ADC_STAT_ZCI_MASK                        (0x400U)
#define ADC_STAT_ZCI_SHIFT                       (10U)
#define ADC_STAT_ZCI(x)                          (((uint16_t)(((uint16_t)(x)) << ADC_STAT_ZCI_SHIFT)) & ADC_STAT_ZCI_MASK)
/*! EOSI0 - End of Scan Interrupt
 *  0b0..A scan cycle has not been completed, no end of scan IRQ pending
 *  0b1..A scan cycle has been completed, end of scan IRQ pending
 */
#define ADC_STAT_EOSI0_MASK                      (0x800U)
#define ADC_STAT_EOSI0_SHIFT                     (11U)
#define ADC_STAT_EOSI0(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_STAT_EOSI0_SHIFT)) & ADC_STAT_EOSI0_MASK)
/*! EOSI1 - End of Scan Interrupt
 *  0b0..A scan cycle has not been completed, no end of scan IRQ pending
 *  0b1..A scan cycle has been completed, end of scan IRQ pending
 */
#define ADC_STAT_EOSI1_MASK                      (0x1000U)
#define ADC_STAT_EOSI1_SHIFT                     (12U)
#define ADC_STAT_EOSI1(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_STAT_EOSI1_SHIFT)) & ADC_STAT_EOSI1_MASK)
/*! CIP1 - Conversion in Progress
 *  0b0..Idle state
 *  0b1..A scan cycle is in progress. The ADC will ignore all sync pulses or start commands
 */
#define ADC_STAT_CIP1_MASK                       (0x4000U)
#define ADC_STAT_CIP1_SHIFT                      (14U)
#define ADC_STAT_CIP1(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_STAT_CIP1_SHIFT)) & ADC_STAT_CIP1_MASK)
/*! CIP0 - Conversion in Progress
 *  0b0..Idle state
 *  0b1..A scan cycle is in progress. The ADC will ignore all sync pulses or start commands
 */
#define ADC_STAT_CIP0_MASK                       (0x8000U)
#define ADC_STAT_CIP0_SHIFT                      (15U)
#define ADC_STAT_CIP0(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_STAT_CIP0_SHIFT)) & ADC_STAT_CIP0_MASK)
/*! @} */

/*! @name RDY - ADC Ready Register */
/*! @{ */
/*! RDY - Ready Sample
 *  0b0000000000000000..Sample not ready or has been read
 *  0b0000000000000001..Sample ready to be read
 */
#define ADC_RDY_RDY_MASK                         (0xFFFFU)
#define ADC_RDY_RDY_SHIFT                        (0U)
#define ADC_RDY_RDY(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_RDY_RDY_SHIFT)) & ADC_RDY_RDY_MASK)
/*! @} */

/*! @name LOLIMSTAT - ADC Low Limit Status Register */
/*! @{ */
#define ADC_LOLIMSTAT_LLS_MASK                   (0xFFFFU)
#define ADC_LOLIMSTAT_LLS_SHIFT                  (0U)
#define ADC_LOLIMSTAT_LLS(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_LOLIMSTAT_LLS_SHIFT)) & ADC_LOLIMSTAT_LLS_MASK)
/*! @} */

/*! @name HILIMSTAT - ADC High Limit Status Register */
/*! @{ */
#define ADC_HILIMSTAT_HLS_MASK                   (0xFFFFU)
#define ADC_HILIMSTAT_HLS_SHIFT                  (0U)
#define ADC_HILIMSTAT_HLS(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_HILIMSTAT_HLS_SHIFT)) & ADC_HILIMSTAT_HLS_MASK)
/*! @} */

/*! @name ZXSTAT - ADC Zero Crossing Status Register */
/*! @{ */
/*! ZCS - Zero Crossing Status
 *  0b0000000000000000..Either: A sign change did not occur in a comparison between the current channelx result
 *                      and the previous channelx result, or Zero crossing control is disabled for channelx in the
 *                      zero crossing control register, ZXCTRL
 *  0b0000000000000001..In a comparison between the current channelx result and the previous channelx result, a
 *                      sign change condition occurred as defined in the zero crossing control register (ZXCTRL)
 */
#define ADC_ZXSTAT_ZCS_MASK                      (0xFFFFU)
#define ADC_ZXSTAT_ZCS_SHIFT                     (0U)
#define ADC_ZXSTAT_ZCS(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_ZXSTAT_ZCS_SHIFT)) & ADC_ZXSTAT_ZCS_MASK)
/*! @} */

/*! @name RSLT - ADC Result Registers with sign extension */
/*! @{ */
#define ADC_RSLT_RSLT_MASK                       (0x7FF8U)
#define ADC_RSLT_RSLT_SHIFT                      (3U)
#define ADC_RSLT_RSLT(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_RSLT_RSLT_SHIFT)) & ADC_RSLT_RSLT_MASK)
#define ADC_RSLT_SEXT_MASK                       (0x8000U)
#define ADC_RSLT_SEXT_SHIFT                      (15U)
#define ADC_RSLT_SEXT(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_RSLT_SEXT_SHIFT)) & ADC_RSLT_SEXT_MASK)
/*! @} */

/* The count of ADC_RSLT */
#define ADC_RSLT_COUNT                           (16U)

/*! @name LOLIM - ADC Low Limit Registers */
/*! @{ */
#define ADC_LOLIM_LLMT_MASK                      (0x7FF8U)
#define ADC_LOLIM_LLMT_SHIFT                     (3U)
#define ADC_LOLIM_LLMT(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_LOLIM_LLMT_SHIFT)) & ADC_LOLIM_LLMT_MASK)
/*! @} */

/* The count of ADC_LOLIM */
#define ADC_LOLIM_COUNT                          (16U)

/*! @name HILIM - ADC High Limit Registers */
/*! @{ */
#define ADC_HILIM_HLMT_MASK                      (0x7FF8U)
#define ADC_HILIM_HLMT_SHIFT                     (3U)
#define ADC_HILIM_HLMT(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_HILIM_HLMT_SHIFT)) & ADC_HILIM_HLMT_MASK)
/*! @} */

/* The count of ADC_HILIM */
#define ADC_HILIM_COUNT                          (16U)

/*! @name OFFST - ADC Offset Registers */
/*! @{ */
#define ADC_OFFST_OFFSET_MASK                    (0x7FF8U)
#define ADC_OFFST_OFFSET_SHIFT                   (3U)
#define ADC_OFFST_OFFSET(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_OFFST_OFFSET_SHIFT)) & ADC_OFFST_OFFSET_MASK)
/*! @} */

/* The count of ADC_OFFST */
#define ADC_OFFST_COUNT                          (16U)

/*! @name PWR - ADC Power Control Register */
/*! @{ */
/*! PD0 - Manual Power Down for Converter A
 *  0b0..Power Up ADC converter A
 *  0b1..Power Down ADC converter A
 */
#define ADC_PWR_PD0_MASK                         (0x1U)
#define ADC_PWR_PD0_SHIFT                        (0U)
#define ADC_PWR_PD0(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PD0_SHIFT)) & ADC_PWR_PD0_MASK)
/*! PD1 - Manual Power Down for Converter B
 *  0b0..Power Up ADC converter B
 *  0b1..Power Down ADC converter B
 */
#define ADC_PWR_PD1_MASK                         (0x2U)
#define ADC_PWR_PD1_SHIFT                        (1U)
#define ADC_PWR_PD1(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PD1_SHIFT)) & ADC_PWR_PD1_MASK)
/*! APD - Auto Powerdown
 *  0b0..Auto Powerdown Mode is not active
 *  0b1..Auto Powerdown Mode is active
 */
#define ADC_PWR_APD_MASK                         (0x8U)
#define ADC_PWR_APD_SHIFT                        (3U)
#define ADC_PWR_APD(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_PWR_APD_SHIFT)) & ADC_PWR_APD_MASK)
#define ADC_PWR_PUDELAY_MASK                     (0x3F0U)
#define ADC_PWR_PUDELAY_SHIFT                    (4U)
#define ADC_PWR_PUDELAY(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PUDELAY_SHIFT)) & ADC_PWR_PUDELAY_MASK)
/*! PSTS0 - ADC Converter A Power Status
 *  0b0..ADC Converter A is currently powered up
 *  0b1..ADC Converter A is currently powered down
 */
#define ADC_PWR_PSTS0_MASK                       (0x400U)
#define ADC_PWR_PSTS0_SHIFT                      (10U)
#define ADC_PWR_PSTS0(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PSTS0_SHIFT)) & ADC_PWR_PSTS0_MASK)
/*! PSTS1 - ADC Converter B Power Status
 *  0b0..ADC Converter B is currently powered up
 *  0b1..ADC Converter B is currently powered down
 */
#define ADC_PWR_PSTS1_MASK                       (0x800U)
#define ADC_PWR_PSTS1_SHIFT                      (11U)
#define ADC_PWR_PSTS1(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_PWR_PSTS1_SHIFT)) & ADC_PWR_PSTS1_MASK)
/*! ASB - Auto Standby
 *  0b0..Auto standby mode disabled
 *  0b1..Auto standby mode enabled
 */
#define ADC_PWR_ASB_MASK                         (0x8000U)
#define ADC_PWR_ASB_SHIFT                        (15U)
#define ADC_PWR_ASB(x)                           (((uint16_t)(((uint16_t)(x)) << ADC_PWR_ASB_SHIFT)) & ADC_PWR_ASB_MASK)
/*! @} */

/*! @name CAL - ADC Calibration Register */
/*! @{ */
/*! SEL_VREFL_A - Select V REFLO Source
 *  0b0..Internal VSSA
 *  0b1..ANA3
 */
#define ADC_CAL_SEL_VREFL_A_MASK                 (0x1000U)
#define ADC_CAL_SEL_VREFL_A_SHIFT                (12U)
#define ADC_CAL_SEL_VREFL_A(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFL_A_SHIFT)) & ADC_CAL_SEL_VREFL_A_MASK)
/*! SEL_VREFH_A - Select V REFH Source
 *  0b0..Internal VDDA
 *  0b1..ANA2
 */
#define ADC_CAL_SEL_VREFH_A_MASK                 (0x2000U)
#define ADC_CAL_SEL_VREFH_A_SHIFT                (13U)
#define ADC_CAL_SEL_VREFH_A(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFH_A_SHIFT)) & ADC_CAL_SEL_VREFH_A_MASK)
/*! SEL_VREFL_B - Select V REFLO Source
 *  0b0..Internal VSSA
 *  0b1..ANB3
 */
#define ADC_CAL_SEL_VREFL_B_MASK                 (0x4000U)
#define ADC_CAL_SEL_VREFL_B_SHIFT                (14U)
#define ADC_CAL_SEL_VREFL_B(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFL_B_SHIFT)) & ADC_CAL_SEL_VREFL_B_MASK)
/*! SEL_VREFH_B - Select V REFH Source
 *  0b0..Internal VDDA
 *  0b1..ANB2
 */
#define ADC_CAL_SEL_VREFH_B_MASK                 (0x8000U)
#define ADC_CAL_SEL_VREFH_B_SHIFT                (15U)
#define ADC_CAL_SEL_VREFH_B(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CAL_SEL_VREFH_B_SHIFT)) & ADC_CAL_SEL_VREFH_B_MASK)
/*! @} */

/*! @name GC1 - Gain Control 1 Register */
/*! @{ */
/*! GAIN0 - Gain Control Bit 0
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN0_MASK                       (0x3U)
#define ADC_GC1_GAIN0_SHIFT                      (0U)
#define ADC_GC1_GAIN0(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN0_SHIFT)) & ADC_GC1_GAIN0_MASK)
/*! GAIN1 - Gain Control Bit 1
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN1_MASK                       (0xCU)
#define ADC_GC1_GAIN1_SHIFT                      (2U)
#define ADC_GC1_GAIN1(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN1_SHIFT)) & ADC_GC1_GAIN1_MASK)
/*! GAIN2 - Gain Control Bit 2
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN2_MASK                       (0x30U)
#define ADC_GC1_GAIN2_SHIFT                      (4U)
#define ADC_GC1_GAIN2(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN2_SHIFT)) & ADC_GC1_GAIN2_MASK)
/*! GAIN3 - Gain Control Bit 3
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN3_MASK                       (0xC0U)
#define ADC_GC1_GAIN3_SHIFT                      (6U)
#define ADC_GC1_GAIN3(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN3_SHIFT)) & ADC_GC1_GAIN3_MASK)
/*! GAIN4 - Gain Control Bit 4
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN4_MASK                       (0x300U)
#define ADC_GC1_GAIN4_SHIFT                      (8U)
#define ADC_GC1_GAIN4(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN4_SHIFT)) & ADC_GC1_GAIN4_MASK)
/*! GAIN5 - Gain Control Bit 5
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN5_MASK                       (0xC00U)
#define ADC_GC1_GAIN5_SHIFT                      (10U)
#define ADC_GC1_GAIN5(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN5_SHIFT)) & ADC_GC1_GAIN5_MASK)
/*! GAIN6 - Gain Control Bit 6
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN6_MASK                       (0x3000U)
#define ADC_GC1_GAIN6_SHIFT                      (12U)
#define ADC_GC1_GAIN6(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN6_SHIFT)) & ADC_GC1_GAIN6_MASK)
/*! GAIN7 - Gain Control Bit 7
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC1_GAIN7_MASK                       (0xC000U)
#define ADC_GC1_GAIN7_SHIFT                      (14U)
#define ADC_GC1_GAIN7(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC1_GAIN7_SHIFT)) & ADC_GC1_GAIN7_MASK)
/*! @} */

/*! @name GC2 - Gain Control 2 Register */
/*! @{ */
/*! GAIN8 - Gain Control Bit 8
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN8_MASK                       (0x3U)
#define ADC_GC2_GAIN8_SHIFT                      (0U)
#define ADC_GC2_GAIN8(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN8_SHIFT)) & ADC_GC2_GAIN8_MASK)
/*! GAIN9 - Gain Control Bit 9
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN9_MASK                       (0xCU)
#define ADC_GC2_GAIN9_SHIFT                      (2U)
#define ADC_GC2_GAIN9(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN9_SHIFT)) & ADC_GC2_GAIN9_MASK)
/*! GAIN10 - Gain Control Bit 10
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN10_MASK                      (0x30U)
#define ADC_GC2_GAIN10_SHIFT                     (4U)
#define ADC_GC2_GAIN10(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN10_SHIFT)) & ADC_GC2_GAIN10_MASK)
/*! GAIN11 - Gain Control Bit 11
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN11_MASK                      (0xC0U)
#define ADC_GC2_GAIN11_SHIFT                     (6U)
#define ADC_GC2_GAIN11(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN11_SHIFT)) & ADC_GC2_GAIN11_MASK)
/*! GAIN12 - Gain Control Bit 12
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN12_MASK                      (0x300U)
#define ADC_GC2_GAIN12_SHIFT                     (8U)
#define ADC_GC2_GAIN12(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN12_SHIFT)) & ADC_GC2_GAIN12_MASK)
/*! GAIN13 - Gain Control Bit 13
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN13_MASK                      (0xC00U)
#define ADC_GC2_GAIN13_SHIFT                     (10U)
#define ADC_GC2_GAIN13(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN13_SHIFT)) & ADC_GC2_GAIN13_MASK)
/*! GAIN14 - Gain Control Bit 14
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN14_MASK                      (0x3000U)
#define ADC_GC2_GAIN14_SHIFT                     (12U)
#define ADC_GC2_GAIN14(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN14_SHIFT)) & ADC_GC2_GAIN14_MASK)
/*! GAIN15 - Gain Control Bit 15
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC2_GAIN15_MASK                      (0xC000U)
#define ADC_GC2_GAIN15_SHIFT                     (14U)
#define ADC_GC2_GAIN15(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC2_GAIN15_SHIFT)) & ADC_GC2_GAIN15_MASK)
/*! @} */

/*! @name SCTRL - ADC Scan Control Register */
/*! @{ */
/*! SC - Scan Control Bits
 *  0b0000000000000000..Perform sample immediately after the completion of the current sample.
 *  0b0000000000000001..Delay sample until a new sync input occurs.
 */
#define ADC_SCTRL_SC_MASK                        (0xFFFFU)
#define ADC_SCTRL_SC_SHIFT                       (0U)
#define ADC_SCTRL_SC(x)                          (((uint16_t)(((uint16_t)(x)) << ADC_SCTRL_SC_SHIFT)) & ADC_SCTRL_SC_MASK)
/*! @} */

/*! @name PWR2 - ADC Power Control Register 2 */
/*! @{ */
#define ADC_PWR2_DIV1_MASK                       (0x3F00U)
#define ADC_PWR2_DIV1_SHIFT                      (8U)
#define ADC_PWR2_DIV1(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_PWR2_DIV1_SHIFT)) & ADC_PWR2_DIV1_MASK)
/*! @} */

/*! @name CTRL3 - ADC Control Register 3 */
/*! @{ */
/*! DMASRC - DMA Trigger Source
 *  0b0..DMA trigger source is end of scan interrupt
 *  0b1..DMA trigger source is RDY bits
 */
#define ADC_CTRL3_DMASRC_MASK                    (0x40U)
#define ADC_CTRL3_DMASRC_SHIFT                   (6U)
#define ADC_CTRL3_DMASRC(x)                      (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_DMASRC_SHIFT)) & ADC_CTRL3_DMASRC_MASK)
/*! UPDEN_L - Unipolar Differential Enable Low bits
 *  0bxxx1..Inputs = ANA0-ANA1
 *  0bxxx0..Inputs = ANA0-ANA1
 *  0bxx1x..Inputs = ANA2-ANA3
 *  0bxx0x..Inputs = ANA2-ANA3
 *  0bx1xx..Inputs = ANB0-ANB1
 *  0bx0xx..Inputs = ANB0-ANB1
 *  0b1xxx..Inputs = ANB2-ANB3
 */
#define ADC_CTRL3_UPDEN_L_MASK                   (0xF00U)
#define ADC_CTRL3_UPDEN_L_SHIFT                  (8U)
#define ADC_CTRL3_UPDEN_L(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_UPDEN_L_SHIFT)) & ADC_CTRL3_UPDEN_L_MASK)
/*! UPDEN_H - Unipolar Differential Enable High bits
 *  0bxxx1..Inputs = ANA4-ANA5
 *  0bxxx0..Inputs = ANA4-ANA5
 *  0bxx1x..Inputs = ANA6-ANA7
 *  0bxx0x..Inputs = ANA6-ANA7
 *  0bx1xx..Inputs = ANB4-ANB5
 *  0bx0xx..Inputs = ANB4-ANB5
 *  0b1xxx..Inputs = ANB6-ANB7
 */
#define ADC_CTRL3_UPDEN_H_MASK                   (0xF000U)
#define ADC_CTRL3_UPDEN_H_SHIFT                  (12U)
#define ADC_CTRL3_UPDEN_H(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_CTRL3_UPDEN_H_SHIFT)) & ADC_CTRL3_UPDEN_H_MASK)
/*! @} */

/*! @name SCHLTEN - ADC Scan Interrupt Enable Register */
/*! @{ */
/*! SCHLTEN
 *  0b0000000000000000..Scan interrupt is not enabled for this sample.
 *  0b0000000000000001..Scan interrupt is enabled for this sample.
 */
#define ADC_SCHLTEN_SCHLTEN_MASK                 (0xFFFFU)
#define ADC_SCHLTEN_SCHLTEN_SHIFT                (0U)
#define ADC_SCHLTEN_SCHLTEN(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_SCHLTEN_SCHLTEN_SHIFT)) & ADC_SCHLTEN_SCHLTEN_MASK)
/*! @} */

/*! @name ZXCTRL3 - ADC Zero Crossing Control 3 Register */
/*! @{ */
/*! ZCE16 - Zero crossing enable 16
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE16_MASK                   (0x3U)
#define ADC_ZXCTRL3_ZCE16_SHIFT                  (0U)
#define ADC_ZXCTRL3_ZCE16(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE16_SHIFT)) & ADC_ZXCTRL3_ZCE16_MASK)
/*! ZCE17 - Zero crossing enable 17
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE17_MASK                   (0xCU)
#define ADC_ZXCTRL3_ZCE17_SHIFT                  (2U)
#define ADC_ZXCTRL3_ZCE17(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE17_SHIFT)) & ADC_ZXCTRL3_ZCE17_MASK)
/*! ZCE18 - Zero crossing enable 18
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE18_MASK                   (0x30U)
#define ADC_ZXCTRL3_ZCE18_SHIFT                  (4U)
#define ADC_ZXCTRL3_ZCE18(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE18_SHIFT)) & ADC_ZXCTRL3_ZCE18_MASK)
/*! ZCE19 - Zero crossing enable 19
 *  0b00..Zero Crossing disabled
 *  0b01..Zero Crossing enabled for positive to negative sign change
 *  0b10..Zero Crossing enabled for negative to positive sign change
 *  0b11..Zero Crossing enabled for any sign change
 */
#define ADC_ZXCTRL3_ZCE19_MASK                   (0xC0U)
#define ADC_ZXCTRL3_ZCE19_SHIFT                  (6U)
#define ADC_ZXCTRL3_ZCE19(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_ZXCTRL3_ZCE19_SHIFT)) & ADC_ZXCTRL3_ZCE19_MASK)
/*! @} */

/*! @name CLIST5 - ADC Channel List Register 5 */
/*! @{ */
/*! SAMPLE16 - Sample Field 16
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE16_MASK                 (0x3U)
#define ADC_CLIST5_SAMPLE16_SHIFT                (0U)
#define ADC_CLIST5_SAMPLE16(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE16_SHIFT)) & ADC_CLIST5_SAMPLE16_MASK)
/*! SAMPLE17 - Sample Field 17
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE17_MASK                 (0xCU)
#define ADC_CLIST5_SAMPLE17_SHIFT                (2U)
#define ADC_CLIST5_SAMPLE17(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE17_SHIFT)) & ADC_CLIST5_SAMPLE17_MASK)
/*! SAMPLE18 - Sample Field 18
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADC B analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE18_MASK                 (0x30U)
#define ADC_CLIST5_SAMPLE18_SHIFT                (4U)
#define ADC_CLIST5_SAMPLE18(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE18_SHIFT)) & ADC_CLIST5_SAMPLE18_MASK)
/*! SAMPLE19 - Sample Field 19
 *  0b00..Single Ended: ADCA temperature sensor
 *  0b01..Single Ended: ADCA analog input for on-chip generated signals
 *  0b10..Single Ended: ADCB temperature sensor
 *  0b11..Single Ended: ADCB analog input for on-chip generated signals
 */
#define ADC_CLIST5_SAMPLE19_MASK                 (0xC0U)
#define ADC_CLIST5_SAMPLE19_SHIFT                (6U)
#define ADC_CLIST5_SAMPLE19(x)                   (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SAMPLE19_SHIFT)) & ADC_CLIST5_SAMPLE19_MASK)
/*! SEL_TEMP_0 - Select Temperature Sensor Alternate Source
 *  0b0..Normal Operation (ADCA6)
 *  0b1..ADCA6 input is replaced with ADCA temperature sensor
 */
#define ADC_CLIST5_SEL_TEMP_0_MASK               (0x100U)
#define ADC_CLIST5_SEL_TEMP_0_SHIFT              (8U)
#define ADC_CLIST5_SEL_TEMP_0(x)                 (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_TEMP_0_SHIFT)) & ADC_CLIST5_SEL_TEMP_0_MASK)
/*! SEL_INTERNAL_0 - Select On-Chip Analog Input Alternate Source
 *  0b0..Normal Operation (ADCA7)
 *  0b1..ADCA7 input is replaced with ADCA on-chip analog input
 */
#define ADC_CLIST5_SEL_INTERNAL_0_MASK           (0x200U)
#define ADC_CLIST5_SEL_INTERNAL_0_SHIFT          (9U)
#define ADC_CLIST5_SEL_INTERNAL_0(x)             (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_INTERNAL_0_SHIFT)) & ADC_CLIST5_SEL_INTERNAL_0_MASK)
/*! SEL_TEMP_1 - Select Temperature Sensor Alternate Source
 *  0b0..Normal Operation (ADCB6)
 *  0b1..ADCB6 input is replaced with ADCB temperature sensor
 */
#define ADC_CLIST5_SEL_TEMP_1_MASK               (0x400U)
#define ADC_CLIST5_SEL_TEMP_1_SHIFT              (10U)
#define ADC_CLIST5_SEL_TEMP_1(x)                 (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_TEMP_1_SHIFT)) & ADC_CLIST5_SEL_TEMP_1_MASK)
/*! SEL_INTERNAL_1 - Select On-Chip Analog Input Alternate Source
 *  0b0..Normal operation (ADCB7)
 *  0b1..ADCB7 input is replaced with ADCB on-chip analog input
 */
#define ADC_CLIST5_SEL_INTERNAL_1_MASK           (0x800U)
#define ADC_CLIST5_SEL_INTERNAL_1_SHIFT          (11U)
#define ADC_CLIST5_SEL_INTERNAL_1(x)             (((uint16_t)(((uint16_t)(x)) << ADC_CLIST5_SEL_INTERNAL_1_SHIFT)) & ADC_CLIST5_SEL_INTERNAL_1_MASK)
/*! @} */

/*! @name SDIS2 - ADC Sample Disable Register 2 */
/*! @{ */
/*! DS - Disable Sample Bits
 *  0b0000..SAMPLEx channel is enabled for ADC scan.
 *  0b0001..SAMPLEx channel is disabled for ADC scan and corresponding channels after SAMPLEx will also not occur
 *          in an ADC scan. Please note that enabling the four extra sample slots by themselves (ADC_SDIS=FFFF, and
 *          samples enabled in ADC_SDIS2 ) is supported only in once sequential mode. Sequential loop, and
 *          parallel (both sequential and loop) modes are not supported. It is suggested to poll the ADC_RDY2 register to
 *          check for conversion completion for this case.
 */
#define ADC_SDIS2_DS_MASK                        (0xFU)
#define ADC_SDIS2_DS_SHIFT                       (0U)
#define ADC_SDIS2_DS(x)                          (((uint16_t)(((uint16_t)(x)) << ADC_SDIS2_DS_SHIFT)) & ADC_SDIS2_DS_MASK)
/*! @} */

/*! @name RDY2 - ADC Ready Register 2 */
/*! @{ */
/*! RDY - Ready Sample
 *  0b0000..Sample not ready or has been read
 *  0b0001..Sample ready to be read
 */
#define ADC_RDY2_RDY_MASK                        (0xFU)
#define ADC_RDY2_RDY_SHIFT                       (0U)
#define ADC_RDY2_RDY(x)                          (((uint16_t)(((uint16_t)(x)) << ADC_RDY2_RDY_SHIFT)) & ADC_RDY2_RDY_MASK)
/*! @} */

/*! @name LOLIMSTAT2 - ADC Low Limit Status Register 2 */
/*! @{ */
#define ADC_LOLIMSTAT2_LLS_MASK                  (0xFU)
#define ADC_LOLIMSTAT2_LLS_SHIFT                 (0U)
#define ADC_LOLIMSTAT2_LLS(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_LOLIMSTAT2_LLS_SHIFT)) & ADC_LOLIMSTAT2_LLS_MASK)
/*! @} */

/*! @name HILIMSTAT2 - ADC High Limit Status Register 2 */
/*! @{ */
#define ADC_HILIMSTAT2_HLS_MASK                  (0xFU)
#define ADC_HILIMSTAT2_HLS_SHIFT                 (0U)
#define ADC_HILIMSTAT2_HLS(x)                    (((uint16_t)(((uint16_t)(x)) << ADC_HILIMSTAT2_HLS_SHIFT)) & ADC_HILIMSTAT2_HLS_MASK)
/*! @} */

/*! @name ZXSTAT2 - ADC Zero Crossing Status Register 2 */
/*! @{ */
/*! ZCS - Zero Crossing Status
 *  0b0000..Either: A sign change did not occur in a comparison between the current channelx result and the
 *          previous channelx result, or Zero crossing control is disabled for channelx in the zero crossing control
 *          register, ZXCTRL3
 *  0b0001..In a comparison between the current channelx result and the previous channelx result, a sign change
 *          condition occurred as defined in the zero crossing control register (ZXCTRL3)
 */
#define ADC_ZXSTAT2_ZCS_MASK                     (0xFU)
#define ADC_ZXSTAT2_ZCS_SHIFT                    (0U)
#define ADC_ZXSTAT2_ZCS(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_ZXSTAT2_ZCS_SHIFT)) & ADC_ZXSTAT2_ZCS_MASK)
/*! @} */

/*! @name RSLT2 - ADC Result Registers 2 with sign extension */
/*! @{ */
#define ADC_RSLT2_RSLT_MASK                      (0x7FF8U)
#define ADC_RSLT2_RSLT_SHIFT                     (3U)
#define ADC_RSLT2_RSLT(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_RSLT2_RSLT_SHIFT)) & ADC_RSLT2_RSLT_MASK)
#define ADC_RSLT2_SEXT_MASK                      (0x8000U)
#define ADC_RSLT2_SEXT_SHIFT                     (15U)
#define ADC_RSLT2_SEXT(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_RSLT2_SEXT_SHIFT)) & ADC_RSLT2_SEXT_MASK)
/*! @} */

/* The count of ADC_RSLT2 */
#define ADC_RSLT2_COUNT                          (4U)

/*! @name LOLIM2 - ADC Low Limit Registers 2 */
/*! @{ */
#define ADC_LOLIM2_LLMT_MASK                     (0x7FF8U)
#define ADC_LOLIM2_LLMT_SHIFT                    (3U)
#define ADC_LOLIM2_LLMT(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_LOLIM2_LLMT_SHIFT)) & ADC_LOLIM2_LLMT_MASK)
/*! @} */

/* The count of ADC_LOLIM2 */
#define ADC_LOLIM2_COUNT                         (4U)

/*! @name HILIM2 - ADC High Limit Registers 2 */
/*! @{ */
#define ADC_HILIM2_HLMT_MASK                     (0x7FF8U)
#define ADC_HILIM2_HLMT_SHIFT                    (3U)
#define ADC_HILIM2_HLMT(x)                       (((uint16_t)(((uint16_t)(x)) << ADC_HILIM2_HLMT_SHIFT)) & ADC_HILIM2_HLMT_MASK)
/*! @} */

/* The count of ADC_HILIM2 */
#define ADC_HILIM2_COUNT                         (4U)

/*! @name OFFST2 - ADC Offset Registers 2 */
/*! @{ */
#define ADC_OFFST2_OFFSET_MASK                   (0x7FF8U)
#define ADC_OFFST2_OFFSET_SHIFT                  (3U)
#define ADC_OFFST2_OFFSET(x)                     (((uint16_t)(((uint16_t)(x)) << ADC_OFFST2_OFFSET_SHIFT)) & ADC_OFFST2_OFFSET_MASK)
/*! @} */

/* The count of ADC_OFFST2 */
#define ADC_OFFST2_COUNT                         (4U)

/*! @name GC3 - Gain Control 3 Register */
/*! @{ */
/*! GAIN16 - Gain Control Bit 16
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN16_MASK                      (0x3U)
#define ADC_GC3_GAIN16_SHIFT                     (0U)
#define ADC_GC3_GAIN16(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN16_SHIFT)) & ADC_GC3_GAIN16_MASK)
/*! GAIN17 - Gain Control Bit 17
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN17_MASK                      (0xCU)
#define ADC_GC3_GAIN17_SHIFT                     (2U)
#define ADC_GC3_GAIN17(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN17_SHIFT)) & ADC_GC3_GAIN17_MASK)
/*! GAIN18 - Gain Control Bit 18
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN18_MASK                      (0x30U)
#define ADC_GC3_GAIN18_SHIFT                     (4U)
#define ADC_GC3_GAIN18(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN18_SHIFT)) & ADC_GC3_GAIN18_MASK)
/*! GAIN19 - Gain Control Bit 19
 *  0b00..x1 amplification
 *  0b01..x2 amplification
 *  0b10..x4 amplification
 *  0b11..reserved
 */
#define ADC_GC3_GAIN19_MASK                      (0xC0U)
#define ADC_GC3_GAIN19_SHIFT                     (6U)
#define ADC_GC3_GAIN19(x)                        (((uint16_t)(((uint16_t)(x)) << ADC_GC3_GAIN19_SHIFT)) & ADC_GC3_GAIN19_MASK)
/*! @} */

/*! @name SCTRL2 - ADC Scan Control Register 2 */
/*! @{ */
/*! SC - Scan Control Bits
 *  0b0000..Perform sample immediately after the completion of the current sample.
 *  0b0001..Delay sample until a new sync input occurs.
 */
#define ADC_SCTRL2_SC_MASK                       (0xFU)
#define ADC_SCTRL2_SC_SHIFT                      (0U)
#define ADC_SCTRL2_SC(x)                         (((uint16_t)(((uint16_t)(x)) << ADC_SCTRL2_SC_SHIFT)) & ADC_SCTRL2_SC_MASK)
/*! @} */

/*! @name SCHLTEN2 - ADC Scan Interrupt Enable Register 2 */
/*! @{ */
/*! SCHLTEN
 *  0b0000..Scan interrupt is not enabled for this sample.
 *  0b0001..Scan interrupt is enabled for this sample.
 */
#define ADC_SCHLTEN2_SCHLTEN_MASK                (0xFU)
#define ADC_SCHLTEN2_SCHLTEN_SHIFT               (0U)
#define ADC_SCHLTEN2_SCHLTEN(x)                  (((uint16_t)(((uint16_t)(x)) << ADC_SCHLTEN2_SCHLTEN_SHIFT)) & ADC_SCHLTEN2_SCHLTEN_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC base address */
#define ADC_BASE                                 (0xE500u)
/** Peripheral ADC base pointer */
#define ADC                                      ((ADC_Type *)ADC_BASE)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC }

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CAN Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Peripheral_Access_Layer CAN Peripheral Access Layer
 * @{
 */

/** CAN - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration register, offset: 0x0 */
  __IO uint32_t CTRL1;                             /**< Control 1 register, offset: 0x4 */
  __IO uint32_t TIMER;                             /**< Free Running Timer, offset: 0x8 */
       uint8_t RESERVED_0[4];
  __IO uint32_t RXMGMASK;                          /**< Rx Mailboxes Global Mask Register, offset: 0x10 */
  __IO uint32_t RX14MASK;                          /**< Rx 14 Mask register, offset: 0x14 */
  __IO uint32_t RX15MASK;                          /**< Rx 15 Mask register, offset: 0x18 */
  __IO uint32_t ECR;                               /**< Error Counter, offset: 0x1C */
  __IO uint32_t ESR1;                              /**< Error and Status 1 register, offset: 0x20 */
       uint8_t RESERVED_1[4];
  __IO uint32_t IMASK1;                            /**< Interrupt Masks 1 register, offset: 0x28 */
       uint8_t RESERVED_2[4];
  __IO uint32_t IFLAG1;                            /**< Interrupt Flags 1 register, offset: 0x30 */
  __IO uint32_t CTRL2;                             /**< Control 2 register, offset: 0x34 */
  __I  uint32_t ESR2;                              /**< Error and Status 2 register, offset: 0x38 */
       uint8_t RESERVED_3[8];
  __I  uint32_t CRCR;                              /**< CRC Register, offset: 0x44 */
  __IO uint32_t RXFGMASK;                          /**< Rx FIFO Global Mask register, offset: 0x48 */
  __I  uint32_t RXFIR;                             /**< Rx FIFO Information Register, offset: 0x4C */
  __IO uint32_t CBT;                               /**< CAN Bit Timing Register, offset: 0x50 */
       uint8_t RESERVED_4[44];
  union {                                          /* offset: 0x80 */
    struct {                                         /* offset: 0x80, array step: 0x18 */
      __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 20 CS Register, array offset: 0x80, array step: 0x18 */
      __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 20 ID Register, array offset: 0x84, array step: 0x18 */
      __IO uint32_t WORD[4];                           /**< Message Buffer 0 WORD_16B Register..Message Buffer 20 WORD_16B Register, array offset: 0x88, array step: index*0x18, index2*0x4 */
    } MB_16B[21];
    struct {                                         /* offset: 0x80, array step: 0x28 */
      __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 11 CS Register, array offset: 0x80, array step: 0x28 */
      __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 11 ID Register, array offset: 0x84, array step: 0x28 */
      __IO uint32_t WORD[8];                           /**< Message Buffer 0 WORD_32B Register..Message Buffer 11 WORD_32B Register, array offset: 0x88, array step: index*0x28, index2*0x4 */
    } MB_32B[12];
    struct {                                         /* offset: 0x80, array step: 0x48 */
      __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 6 CS Register, array offset: 0x80, array step: 0x48 */
      __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 6 ID Register, array offset: 0x84, array step: 0x48 */
      __IO uint32_t WORD[16];                          /**< Message Buffer 0 WORD_64B Register..Message Buffer 6 WORD_64B Register, array offset: 0x88, array step: index*0x48, index2*0x4 */
    } MB_64B[7];
    struct {                                         /* offset: 0x80, array step: 0x10 */
      __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 31 CS Register, array offset: 0x80, array step: 0x10 */
      __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 31 ID Register, array offset: 0x84, array step: 0x10 */
      __IO uint32_t WORD[2];                           /**< Message Buffer 0 WORD_8B Register..Message Buffer 31 WORD_8B Register, array offset: 0x88, array step: index*0x10, index2*0x4 */
    } MB_8B[32];
    struct {                                         /* offset: 0x80, array step: 0x10 */
      __IO uint32_t CS;                                /**< Message Buffer 0 CS Register..Message Buffer 31 CS Register, array offset: 0x80, array step: 0x10 */
      __IO uint32_t ID;                                /**< Message Buffer 0 ID Register..Message Buffer 31 ID Register, array offset: 0x84, array step: 0x10 */
      __IO uint32_t WORD0;                             /**< Message Buffer 0 WORD0 Register..Message Buffer 31 WORD0 Register, array offset: 0x88, array step: 0x10 */
      __IO uint32_t WORD1;                             /**< Message Buffer 0 WORD1 Register..Message Buffer 31 WORD1 Register, array offset: 0x8C, array step: 0x10 */
    } MB[32];
  };
       uint8_t RESERVED_5[1536];
  __IO uint32_t RXIMR[32];                         /**< Rx Individual Mask Registers, array offset: 0x880, array step: 0x4 */
       uint8_t RESERVED_6[768];
  __IO uint32_t FDCTRL;                            /**< CAN FD Control Register, offset: 0xC00 */
  __IO uint32_t FDCBT;                             /**< CAN FD Bit Timing Register, offset: 0xC04 */
  __I  uint32_t FDCRC;                             /**< CAN FD CRC Register, offset: 0xC08 */
} CAN_Type;

/* ----------------------------------------------------------------------------
   -- CAN Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Masks CAN Register Masks
 * @{
 */

/*! @name MCR - Module Configuration register */
/*! @{ */
#define CAN_MCR_MAXMB_MASK                       (0x7FU)
#define CAN_MCR_MAXMB_SHIFT                      (0U)
#define CAN_MCR_MAXMB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MAXMB_SHIFT)) & CAN_MCR_MAXMB_MASK)
/*! IDAM - ID Acceptance Mode
 *  0b00..Format A: One full ID (standard and extended) per ID filter table element.
 *  0b01..Format B: Two full standard IDs or two partial 14-bit (standard and extended) IDs per ID filter table element.
 *  0b10..Format C: Four partial 8-bit standard IDs per ID filter table element.
 *  0b11..Format D: All frames rejected.
 */
#define CAN_MCR_IDAM_MASK                        (0x300U)
#define CAN_MCR_IDAM_SHIFT                       (8U)
#define CAN_MCR_IDAM(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IDAM_SHIFT)) & CAN_MCR_IDAM_MASK)
/*! FDEN - CAN FD operation enable
 *  0b1..CAN FD is enabled. FlexCAN is able to receive and transmit messages in both CAN FD and CAN 2.0 formats.
 *  0b0..CAN FD is disabled. FlexCAN is able to receive and transmit messages in CAN 2.0 format.
 */
#define CAN_MCR_FDEN_MASK                        (0x800U)
#define CAN_MCR_FDEN_SHIFT                       (11U)
#define CAN_MCR_FDEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FDEN_SHIFT)) & CAN_MCR_FDEN_MASK)
/*! AEN - Abort Enable
 *  0b0..Abort disabled.
 *  0b1..Abort enabled.
 */
#define CAN_MCR_AEN_MASK                         (0x1000U)
#define CAN_MCR_AEN_SHIFT                        (12U)
#define CAN_MCR_AEN(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_AEN_SHIFT)) & CAN_MCR_AEN_MASK)
/*! LPRIOEN - Local Priority Enable
 *  0b0..Local Priority disabled.
 *  0b1..Local Priority enabled.
 */
#define CAN_MCR_LPRIOEN_MASK                     (0x2000U)
#define CAN_MCR_LPRIOEN_SHIFT                    (13U)
#define CAN_MCR_LPRIOEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPRIOEN_SHIFT)) & CAN_MCR_LPRIOEN_MASK)
/*! DMA - DMA Enable
 *  0b0..DMA feature for RX FIFO disabled.
 *  0b1..DMA feature for RX FIFO enabled.
 */
#define CAN_MCR_DMA_MASK                         (0x8000U)
#define CAN_MCR_DMA_SHIFT                        (15U)
#define CAN_MCR_DMA(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DMA_SHIFT)) & CAN_MCR_DMA_MASK)
/*! IRMQ - Individual Rx Masking And Queue Enable
 *  0b0..Individual Rx masking and queue feature are disabled. For backward compatibility with legacy
 *       applications, the reading of C/S word locks the MB even if it is EMPTY.
 *  0b1..Individual Rx masking and queue feature are enabled.
 */
#define CAN_MCR_IRMQ_MASK                        (0x10000U)
#define CAN_MCR_IRMQ_SHIFT                       (16U)
#define CAN_MCR_IRMQ(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_IRMQ_SHIFT)) & CAN_MCR_IRMQ_MASK)
/*! SRXDIS - Self Reception Disable
 *  0b0..Self-reception enabled.
 *  0b1..Self-reception disabled.
 */
#define CAN_MCR_SRXDIS_MASK                      (0x20000U)
#define CAN_MCR_SRXDIS_SHIFT                     (17U)
#define CAN_MCR_SRXDIS(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SRXDIS_SHIFT)) & CAN_MCR_SRXDIS_MASK)
/*! DOZE - Doze Mode Enable
 *  0b0..FlexCAN is not enabled to enter low-power mode when Doze mode is requested.
 *  0b1..FlexCAN is enabled to enter low-power mode when Doze mode is requested.
 */
#define CAN_MCR_DOZE_MASK                        (0x40000U)
#define CAN_MCR_DOZE_SHIFT                       (18U)
#define CAN_MCR_DOZE(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_DOZE_SHIFT)) & CAN_MCR_DOZE_MASK)
/*! WAKSRC - Wake Up Source
 *  0b0..FlexCAN uses the unfiltered Rx input to detect recessive to dominant edges on the CAN bus.
 *  0b1..FlexCAN uses the filtered Rx input to detect recessive to dominant edges on the CAN bus.
 */
#define CAN_MCR_WAKSRC_MASK                      (0x80000U)
#define CAN_MCR_WAKSRC_SHIFT                     (19U)
#define CAN_MCR_WAKSRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKSRC_SHIFT)) & CAN_MCR_WAKSRC_MASK)
/*! LPMACK - Low-Power Mode Acknowledge
 *  0b0..FlexCAN is not in a low-power mode.
 *  0b1..FlexCAN is in a low-power mode.
 */
#define CAN_MCR_LPMACK_MASK                      (0x100000U)
#define CAN_MCR_LPMACK_SHIFT                     (20U)
#define CAN_MCR_LPMACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_LPMACK_SHIFT)) & CAN_MCR_LPMACK_MASK)
/*! WRNEN - Warning Interrupt Enable
 *  0b0..TWRNINT and RWRNINT bits are zero, independent of the values in the error counters.
 *  0b1..TWRNINT and RWRNINT bits are set when the respective error counter transitions from less than 96 to greater than or equal to 96.
 */
#define CAN_MCR_WRNEN_MASK                       (0x200000U)
#define CAN_MCR_WRNEN_SHIFT                      (21U)
#define CAN_MCR_WRNEN(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WRNEN_SHIFT)) & CAN_MCR_WRNEN_MASK)
/*! SLFWAK - Self Wake Up
 *  0b0..FlexCAN Self Wake Up feature is disabled.
 *  0b1..FlexCAN Self Wake Up feature is enabled.
 */
#define CAN_MCR_SLFWAK_MASK                      (0x400000U)
#define CAN_MCR_SLFWAK_SHIFT                     (22U)
#define CAN_MCR_SLFWAK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SLFWAK_SHIFT)) & CAN_MCR_SLFWAK_MASK)
/*! SUPV - Supervisor Mode
 *  0b0..FlexCAN is in User mode. Affected registers allow both Supervisor and Unrestricted accesses (ips_supervisor_access signal is ignored).
 *  0b1..FlexCAN is in Supervisor mode. Affected registers allow only Supervisor access. Unrestricted access
 *       (ips_supervisor_access negated) behaves as though the access was done to an unimplemented register location
 *       (ips_xfr_error asserted in the SkyBlue Interface).
 */
#define CAN_MCR_SUPV_MASK                        (0x800000U)
#define CAN_MCR_SUPV_SHIFT                       (23U)
#define CAN_MCR_SUPV(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SUPV_SHIFT)) & CAN_MCR_SUPV_MASK)
/*! FRZACK - Freeze Mode Acknowledge
 *  0b0..FlexCAN not in Freeze mode, prescaler running.
 *  0b1..FlexCAN in Freeze mode, prescaler stopped.
 */
#define CAN_MCR_FRZACK_MASK                      (0x1000000U)
#define CAN_MCR_FRZACK_SHIFT                     (24U)
#define CAN_MCR_FRZACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZACK_SHIFT)) & CAN_MCR_FRZACK_MASK)
/*! SOFTRST - Soft Reset
 *  0b0..No reset request.
 *  0b1..Resets the registers affected by soft reset.
 */
#define CAN_MCR_SOFTRST_MASK                     (0x2000000U)
#define CAN_MCR_SOFTRST_SHIFT                    (25U)
#define CAN_MCR_SOFTRST(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_MCR_SOFTRST_SHIFT)) & CAN_MCR_SOFTRST_MASK)
/*! WAKMSK - Wake Up Interrupt Mask
 *  0b0..Wake Up interrupt is disabled.
 *  0b1..Wake Up interrupt is enabled.
 */
#define CAN_MCR_WAKMSK_MASK                      (0x4000000U)
#define CAN_MCR_WAKMSK_SHIFT                     (26U)
#define CAN_MCR_WAKMSK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_WAKMSK_SHIFT)) & CAN_MCR_WAKMSK_MASK)
/*! NOTRDY - FlexCAN Not Ready
 *  0b0..FlexCAN module is either in Normal mode, Listen-Only mode, or Loop-Back mode.
 *  0b1..FlexCAN module is either in Disable mode, Doze mode, Stop mode, or Freeze mode.
 */
#define CAN_MCR_NOTRDY_MASK                      (0x8000000U)
#define CAN_MCR_NOTRDY_SHIFT                     (27U)
#define CAN_MCR_NOTRDY(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_MCR_NOTRDY_SHIFT)) & CAN_MCR_NOTRDY_MASK)
/*! HALT - Halt FlexCAN
 *  0b0..No Freeze mode request.
 *  0b1..Enters Freeze mode if the FRZ bit is asserted.
 */
#define CAN_MCR_HALT_MASK                        (0x10000000U)
#define CAN_MCR_HALT_SHIFT                       (28U)
#define CAN_MCR_HALT(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_HALT_SHIFT)) & CAN_MCR_HALT_MASK)
/*! RFEN - Rx FIFO Enable
 *  0b0..Rx FIFO not enabled.
 *  0b1..Rx FIFO enabled.
 */
#define CAN_MCR_RFEN_MASK                        (0x20000000U)
#define CAN_MCR_RFEN_SHIFT                       (29U)
#define CAN_MCR_RFEN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_RFEN_SHIFT)) & CAN_MCR_RFEN_MASK)
/*! FRZ - Freeze Enable
 *  0b0..Not enabled to enter Freeze mode.
 *  0b1..Enabled to enter Freeze mode.
 */
#define CAN_MCR_FRZ_MASK                         (0x40000000U)
#define CAN_MCR_FRZ_SHIFT                        (30U)
#define CAN_MCR_FRZ(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_MCR_FRZ_SHIFT)) & CAN_MCR_FRZ_MASK)
/*! MDIS - Module Disable
 *  0b0..Enable the FlexCAN module.
 *  0b1..Disable the FlexCAN module.
 */
#define CAN_MCR_MDIS_MASK                        (0x80000000U)
#define CAN_MCR_MDIS_SHIFT                       (31U)
#define CAN_MCR_MDIS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_MCR_MDIS_SHIFT)) & CAN_MCR_MDIS_MASK)
/*! @} */

/*! @name CTRL1 - Control 1 register */
/*! @{ */
#define CAN_CTRL1_PROPSEG_MASK                   (0x7U)
#define CAN_CTRL1_PROPSEG_SHIFT                  (0U)
#define CAN_CTRL1_PROPSEG(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PROPSEG_SHIFT)) & CAN_CTRL1_PROPSEG_MASK)
/*! LOM - Listen-Only Mode
 *  0b0..Listen-Only mode is deactivated.
 *  0b1..FlexCAN module operates in Listen-Only mode.
 */
#define CAN_CTRL1_LOM_MASK                       (0x8U)
#define CAN_CTRL1_LOM_SHIFT                      (3U)
#define CAN_CTRL1_LOM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LOM_SHIFT)) & CAN_CTRL1_LOM_MASK)
/*! LBUF - Lowest Buffer Transmitted First
 *  0b0..Buffer with highest priority is transmitted first.
 *  0b1..Lowest number buffer is transmitted first.
 */
#define CAN_CTRL1_LBUF_MASK                      (0x10U)
#define CAN_CTRL1_LBUF_SHIFT                     (4U)
#define CAN_CTRL1_LBUF(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LBUF_SHIFT)) & CAN_CTRL1_LBUF_MASK)
/*! TSYN - Timer Sync
 *  0b0..Timer sync feature disabled
 *  0b1..Timer sync feature enabled
 */
#define CAN_CTRL1_TSYN_MASK                      (0x20U)
#define CAN_CTRL1_TSYN_SHIFT                     (5U)
#define CAN_CTRL1_TSYN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TSYN_SHIFT)) & CAN_CTRL1_TSYN_MASK)
/*! BOFFREC - Bus Off Recovery
 *  0b0..Automatic recovering from Bus Off state enabled.
 *  0b1..Automatic recovering from Bus Off state disabled.
 */
#define CAN_CTRL1_BOFFREC_MASK                   (0x40U)
#define CAN_CTRL1_BOFFREC_SHIFT                  (6U)
#define CAN_CTRL1_BOFFREC(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFREC_SHIFT)) & CAN_CTRL1_BOFFREC_MASK)
/*! SMP - CAN Bit Sampling
 *  0b0..Just one sample is used to determine the bit value.
 *  0b1..Three samples are used to determine the value of the received bit: the regular one (sample point) and two
 *       preceding samples; a majority rule is used.
 */
#define CAN_CTRL1_SMP_MASK                       (0x80U)
#define CAN_CTRL1_SMP_SHIFT                      (7U)
#define CAN_CTRL1_SMP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_SMP_SHIFT)) & CAN_CTRL1_SMP_MASK)
/*! RWRNMSK - Rx Warning Interrupt Mask
 *  0b0..Rx Warning interrupt disabled.
 *  0b1..Rx Warning interrupt enabled.
 */
#define CAN_CTRL1_RWRNMSK_MASK                   (0x400U)
#define CAN_CTRL1_RWRNMSK_SHIFT                  (10U)
#define CAN_CTRL1_RWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RWRNMSK_SHIFT)) & CAN_CTRL1_RWRNMSK_MASK)
/*! TWRNMSK - Tx Warning Interrupt Mask
 *  0b0..Tx Warning interrupt disabled.
 *  0b1..Tx Warning interrupt enabled.
 */
#define CAN_CTRL1_TWRNMSK_MASK                   (0x800U)
#define CAN_CTRL1_TWRNMSK_SHIFT                  (11U)
#define CAN_CTRL1_TWRNMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_TWRNMSK_SHIFT)) & CAN_CTRL1_TWRNMSK_MASK)
/*! LPB - Loop Back Mode
 *  0b0..Loop Back disabled.
 *  0b1..Loop Back enabled.
 */
#define CAN_CTRL1_LPB_MASK                       (0x1000U)
#define CAN_CTRL1_LPB_SHIFT                      (12U)
#define CAN_CTRL1_LPB(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_LPB_SHIFT)) & CAN_CTRL1_LPB_MASK)
/*! CLKSRC - CAN Engine Clock Source
 *  0b0..The CAN engine clock source is the oscillator clock. Under this condition, the oscillator clock frequency must be lower than the bus clock.
 *  0b1..The CAN engine clock source is the peripheral clock.
 */
#define CAN_CTRL1_CLKSRC_MASK                    (0x2000U)
#define CAN_CTRL1_CLKSRC_SHIFT                   (13U)
#define CAN_CTRL1_CLKSRC(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_CLKSRC_SHIFT)) & CAN_CTRL1_CLKSRC_MASK)
/*! ERRMSK - Error Interrupt Mask
 *  0b0..Error interrupt disabled.
 *  0b1..Error interrupt enabled.
 */
#define CAN_CTRL1_ERRMSK_MASK                    (0x4000U)
#define CAN_CTRL1_ERRMSK_SHIFT                   (14U)
#define CAN_CTRL1_ERRMSK(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_ERRMSK_SHIFT)) & CAN_CTRL1_ERRMSK_MASK)
/*! BOFFMSK - Bus Off Interrupt Mask
 *  0b0..Bus Off interrupt disabled.
 *  0b1..Bus Off interrupt enabled.
 */
#define CAN_CTRL1_BOFFMSK_MASK                   (0x8000U)
#define CAN_CTRL1_BOFFMSK_SHIFT                  (15U)
#define CAN_CTRL1_BOFFMSK(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_BOFFMSK_SHIFT)) & CAN_CTRL1_BOFFMSK_MASK)
#define CAN_CTRL1_PSEG2_MASK                     (0x70000U)
#define CAN_CTRL1_PSEG2_SHIFT                    (16U)
#define CAN_CTRL1_PSEG2(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG2_SHIFT)) & CAN_CTRL1_PSEG2_MASK)
#define CAN_CTRL1_PSEG1_MASK                     (0x380000U)
#define CAN_CTRL1_PSEG1_SHIFT                    (19U)
#define CAN_CTRL1_PSEG1(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PSEG1_SHIFT)) & CAN_CTRL1_PSEG1_MASK)
#define CAN_CTRL1_RJW_MASK                       (0xC00000U)
#define CAN_CTRL1_RJW_SHIFT                      (22U)
#define CAN_CTRL1_RJW(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_RJW_SHIFT)) & CAN_CTRL1_RJW_MASK)
#define CAN_CTRL1_PRESDIV_MASK                   (0xFF000000U)
#define CAN_CTRL1_PRESDIV_SHIFT                  (24U)
#define CAN_CTRL1_PRESDIV(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_CTRL1_PRESDIV_SHIFT)) & CAN_CTRL1_PRESDIV_MASK)
/*! @} */

/*! @name TIMER - Free Running Timer */
/*! @{ */
#define CAN_TIMER_TIMER_MASK                     (0xFFFFU)
#define CAN_TIMER_TIMER_SHIFT                    (0U)
#define CAN_TIMER_TIMER(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_TIMER_TIMER_SHIFT)) & CAN_TIMER_TIMER_MASK)
/*! @} */

/*! @name TCR - Test Configuration */
/*! @{ */
/*! DSCACK - Discard Acknowledge
 *  0b0..No such occurrence.
 *  0b1..Discard acknowledge bit.
 */
#define CAN_TCR_DSCACK_MASK                      (0x100U)
#define CAN_TCR_DSCACK_SHIFT                     (8U)
#define CAN_TCR_DSCACK(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_TCR_DSCACK_SHIFT)) & CAN_TCR_DSCACK_MASK)
/*! BIT_CLS - Output Bit Clock
 *  0b0..No such occurrence.
 *  0b1..Internal FlexCAN bit clock on ipp_do_cantx pin.
 */
#define CAN_TCR_BIT_CLS_MASK                     (0x200U)
#define CAN_TCR_BIT_CLS_SHIFT                    (9U)
#define CAN_TCR_BIT_CLS(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_TCR_BIT_CLS_SHIFT)) & CAN_TCR_BIT_CLS_MASK)
/*! TRD - Test Read
 *  0b0..Read operations behave as in Normal mode.
 *  0b1..Read access by CPU does not cause changes in registers or state variables.
 */
#define CAN_TCR_TRD_MASK                         (0x400U)
#define CAN_TCR_TRD_SHIFT                        (10U)
#define CAN_TCR_TRD(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_TCR_TRD_SHIFT)) & CAN_TCR_TRD_MASK)
/*! @} */

/*! @name RXMGMASK - Rx Mailboxes Global Mask register */
/*! @{ */
#define CAN_RXMGMASK_MG_MASK                     (0xFFFFFFFFU)
#define CAN_RXMGMASK_MG_SHIFT                    (0U)
#define CAN_RXMGMASK_MG(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXMGMASK_MG_SHIFT)) & CAN_RXMGMASK_MG_MASK)
/*! @} */

/*! @name RX14MASK - Rx 14 Mask register */
/*! @{ */
#define CAN_RX14MASK_RX14M_MASK                  (0xFFFFFFFFU)
#define CAN_RX14MASK_RX14M_SHIFT                 (0U)
#define CAN_RX14MASK_RX14M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX14MASK_RX14M_SHIFT)) & CAN_RX14MASK_RX14M_MASK)
/*! @} */

/*! @name RX15MASK - Rx 15 Mask register */
/*! @{ */
#define CAN_RX15MASK_RX15M_MASK                  (0xFFFFFFFFU)
#define CAN_RX15MASK_RX15M_SHIFT                 (0U)
#define CAN_RX15MASK_RX15M(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_RX15MASK_RX15M_SHIFT)) & CAN_RX15MASK_RX15M_MASK)
/*! @} */

/*! @name ECR - Error Counter */
/*! @{ */
#define CAN_ECR_TXERRCNT_MASK                    (0xFFU)
#define CAN_ECR_TXERRCNT_SHIFT                   (0U)
#define CAN_ECR_TXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_SHIFT)) & CAN_ECR_TXERRCNT_MASK)
#define CAN_ECR_RXERRCNT_MASK                    (0xFF00U)
#define CAN_ECR_RXERRCNT_SHIFT                   (8U)
#define CAN_ECR_RXERRCNT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_SHIFT)) & CAN_ECR_RXERRCNT_MASK)
#define CAN_ECR_TXERRCNT_FAST_MASK               (0xFF0000U)
#define CAN_ECR_TXERRCNT_FAST_SHIFT              (16U)
#define CAN_ECR_TXERRCNT_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ECR_TXERRCNT_FAST_SHIFT)) & CAN_ECR_TXERRCNT_FAST_MASK)
#define CAN_ECR_RXERRCNT_FAST_MASK               (0xFF000000U)
#define CAN_ECR_RXERRCNT_FAST_SHIFT              (24U)
#define CAN_ECR_RXERRCNT_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ECR_RXERRCNT_FAST_SHIFT)) & CAN_ECR_RXERRCNT_FAST_MASK)
/*! @} */

/*! @name ESR1 - Error and Status 1 register */
/*! @{ */
/*! WAKINT - Wake-Up Interrupt
 *  0b0..No such occurrence.
 *  0b1..Indicates a recessive to dominant transition was received on the CAN bus.
 */
#define CAN_ESR1_WAKINT_MASK                     (0x1U)
#define CAN_ESR1_WAKINT_SHIFT                    (0U)
#define CAN_ESR1_WAKINT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_WAKINT_SHIFT)) & CAN_ESR1_WAKINT_MASK)
/*! ERRINT - Error Interrupt
 *  0b0..No such occurrence.
 *  0b1..Indicates setting of any error bit in the Error and Status register.
 */
#define CAN_ESR1_ERRINT_MASK                     (0x2U)
#define CAN_ESR1_ERRINT_SHIFT                    (1U)
#define CAN_ESR1_ERRINT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_SHIFT)) & CAN_ESR1_ERRINT_MASK)
/*! BOFFINT - Bus Off Interrupt
 *  0b0..No such occurrence.
 *  0b1..FlexCAN module entered Bus Off state.
 */
#define CAN_ESR1_BOFFINT_MASK                    (0x4U)
#define CAN_ESR1_BOFFINT_SHIFT                   (2U)
#define CAN_ESR1_BOFFINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFINT_SHIFT)) & CAN_ESR1_BOFFINT_MASK)
/*! RX - FlexCAN In Reception
 *  0b0..FlexCAN is not receiving a message.
 *  0b1..FlexCAN is receiving a message.
 */
#define CAN_ESR1_RX_MASK                         (0x8U)
#define CAN_ESR1_RX_SHIFT                        (3U)
#define CAN_ESR1_RX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RX_SHIFT)) & CAN_ESR1_RX_MASK)
/*! FLTCONF - Fault Confinement State
 *  0b00..Error Active
 *  0b01..Error Passive
 *  0b1x..Bus Off
 */
#define CAN_ESR1_FLTCONF_MASK                    (0x30U)
#define CAN_ESR1_FLTCONF_SHIFT                   (4U)
#define CAN_ESR1_FLTCONF(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FLTCONF_SHIFT)) & CAN_ESR1_FLTCONF_MASK)
/*! TX - FlexCAN In Transmission
 *  0b0..FlexCAN is not transmitting a message.
 *  0b1..FlexCAN is transmitting a message.
 */
#define CAN_ESR1_TX_MASK                         (0x40U)
#define CAN_ESR1_TX_SHIFT                        (6U)
#define CAN_ESR1_TX(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TX_SHIFT)) & CAN_ESR1_TX_MASK)
/*! IDLE - IDLE
 *  0b0..No such occurrence.
 *  0b1..CAN bus is now IDLE.
 */
#define CAN_ESR1_IDLE_MASK                       (0x80U)
#define CAN_ESR1_IDLE_SHIFT                      (7U)
#define CAN_ESR1_IDLE(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_IDLE_SHIFT)) & CAN_ESR1_IDLE_MASK)
/*! RXWRN - Rx Error Warning
 *  0b0..No such occurrence.
 *  0b1..RXERRCNT is greater than or equal to 96.
 */
#define CAN_ESR1_RXWRN_MASK                      (0x100U)
#define CAN_ESR1_RXWRN_SHIFT                     (8U)
#define CAN_ESR1_RXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RXWRN_SHIFT)) & CAN_ESR1_RXWRN_MASK)
/*! TXWRN - TX Error Warning
 *  0b0..No such occurrence.
 *  0b1..TXERRCNT is greater than or equal to 96.
 */
#define CAN_ESR1_TXWRN_MASK                      (0x200U)
#define CAN_ESR1_TXWRN_SHIFT                     (9U)
#define CAN_ESR1_TXWRN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TXWRN_SHIFT)) & CAN_ESR1_TXWRN_MASK)
/*! STFERR - Stuffing Error
 *  0b0..No such occurrence.
 *  0b1..A stuffing error occurred since last read of this register.
 */
#define CAN_ESR1_STFERR_MASK                     (0x400U)
#define CAN_ESR1_STFERR_SHIFT                    (10U)
#define CAN_ESR1_STFERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_SHIFT)) & CAN_ESR1_STFERR_MASK)
/*! FRMERR - Form Error
 *  0b0..No such occurrence.
 *  0b1..A Form Error occurred since last read of this register.
 */
#define CAN_ESR1_FRMERR_MASK                     (0x800U)
#define CAN_ESR1_FRMERR_SHIFT                    (11U)
#define CAN_ESR1_FRMERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_SHIFT)) & CAN_ESR1_FRMERR_MASK)
/*! CRCERR - Cyclic Redundancy Check Error
 *  0b0..No such occurrence.
 *  0b1..A CRC error occurred since last read of this register.
 */
#define CAN_ESR1_CRCERR_MASK                     (0x1000U)
#define CAN_ESR1_CRCERR_SHIFT                    (12U)
#define CAN_ESR1_CRCERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_SHIFT)) & CAN_ESR1_CRCERR_MASK)
/*! ACKERR - Acknowledge Error
 *  0b0..No such occurrence.
 *  0b1..An ACK error occurred since last read of this register.
 */
#define CAN_ESR1_ACKERR_MASK                     (0x2000U)
#define CAN_ESR1_ACKERR_SHIFT                    (13U)
#define CAN_ESR1_ACKERR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ACKERR_SHIFT)) & CAN_ESR1_ACKERR_MASK)
/*! BIT0ERR - Bit0 Error
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as dominant is received as recessive.
 */
#define CAN_ESR1_BIT0ERR_MASK                    (0x4000U)
#define CAN_ESR1_BIT0ERR_SHIFT                   (14U)
#define CAN_ESR1_BIT0ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_SHIFT)) & CAN_ESR1_BIT0ERR_MASK)
/*! BIT1ERR - Bit1 Error
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as recessive is received as dominant.
 */
#define CAN_ESR1_BIT1ERR_MASK                    (0x8000U)
#define CAN_ESR1_BIT1ERR_SHIFT                   (15U)
#define CAN_ESR1_BIT1ERR(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_SHIFT)) & CAN_ESR1_BIT1ERR_MASK)
/*! RWRNINT - Rx Warning Interrupt Flag
 *  0b0..No such occurrence.
 *  0b1..The Rx error counter transitioned from less than 96 to greater than or equal to 96.
 */
#define CAN_ESR1_RWRNINT_MASK                    (0x10000U)
#define CAN_ESR1_RWRNINT_SHIFT                   (16U)
#define CAN_ESR1_RWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_RWRNINT_SHIFT)) & CAN_ESR1_RWRNINT_MASK)
/*! TWRNINT - Tx Warning Interrupt Flag
 *  0b0..No such occurrence.
 *  0b1..The Tx error counter transitioned from less than 96 to greater than or equal to 96.
 */
#define CAN_ESR1_TWRNINT_MASK                    (0x20000U)
#define CAN_ESR1_TWRNINT_SHIFT                   (17U)
#define CAN_ESR1_TWRNINT(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_TWRNINT_SHIFT)) & CAN_ESR1_TWRNINT_MASK)
/*! SYNCH - CAN Synchronization Status
 *  0b0..FlexCAN is not synchronized to the CAN bus.
 *  0b1..FlexCAN is synchronized to the CAN bus.
 */
#define CAN_ESR1_SYNCH_MASK                      (0x40000U)
#define CAN_ESR1_SYNCH_SHIFT                     (18U)
#define CAN_ESR1_SYNCH(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_SYNCH_SHIFT)) & CAN_ESR1_SYNCH_MASK)
/*! BOFFDONEINT - Bus Off Done Interrupt
 *  0b0..No such occurrence.
 *  0b1..FlexCAN module has completed Bus Off process.
 */
#define CAN_ESR1_BOFFDONEINT_MASK                (0x80000U)
#define CAN_ESR1_BOFFDONEINT_SHIFT               (19U)
#define CAN_ESR1_BOFFDONEINT(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BOFFDONEINT_SHIFT)) & CAN_ESR1_BOFFDONEINT_MASK)
/*! ERRINT_FAST - Error interrupt for errors detected in Data Phase of CAN FD frames with BRS bit set
 *  0b0..No such occurrence.
 *  0b1..Indicates setting of any error bit detected in the data phase of CAN FD frames with the BRS bit set.
 */
#define CAN_ESR1_ERRINT_FAST_MASK                (0x100000U)
#define CAN_ESR1_ERRINT_FAST_SHIFT               (20U)
#define CAN_ESR1_ERRINT_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERRINT_FAST_SHIFT)) & CAN_ESR1_ERRINT_FAST_MASK)
/*! ERROVR - Error Overrun
 *  0b0..Overrun has not occurred.
 *  0b1..Overrun has occurred.
 */
#define CAN_ESR1_ERROVR_MASK                     (0x200000U)
#define CAN_ESR1_ERROVR_SHIFT                    (21U)
#define CAN_ESR1_ERROVR(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_ERROVR_SHIFT)) & CAN_ESR1_ERROVR_MASK)
/*! STFERR_FAST - Stuffing Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A stuffing error occurred since last read of this register.
 */
#define CAN_ESR1_STFERR_FAST_MASK                (0x4000000U)
#define CAN_ESR1_STFERR_FAST_SHIFT               (26U)
#define CAN_ESR1_STFERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_STFERR_FAST_SHIFT)) & CAN_ESR1_STFERR_FAST_MASK)
/*! FRMERR_FAST - Form Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A form error occurred since last read of this register.
 */
#define CAN_ESR1_FRMERR_FAST_MASK                (0x8000000U)
#define CAN_ESR1_FRMERR_FAST_SHIFT               (27U)
#define CAN_ESR1_FRMERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_FRMERR_FAST_SHIFT)) & CAN_ESR1_FRMERR_FAST_MASK)
/*! CRCERR_FAST - Cyclic Redundancy Check Error in the CRC field of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..A CRC error occurred since last read of this register.
 */
#define CAN_ESR1_CRCERR_FAST_MASK                (0x10000000U)
#define CAN_ESR1_CRCERR_FAST_SHIFT               (28U)
#define CAN_ESR1_CRCERR_FAST(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_CRCERR_FAST_SHIFT)) & CAN_ESR1_CRCERR_FAST_MASK)
/*! BIT0ERR_FAST - Bit0 Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as dominant is received as recessive.
 */
#define CAN_ESR1_BIT0ERR_FAST_MASK               (0x40000000U)
#define CAN_ESR1_BIT0ERR_FAST_SHIFT              (30U)
#define CAN_ESR1_BIT0ERR_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT0ERR_FAST_SHIFT)) & CAN_ESR1_BIT0ERR_FAST_MASK)
/*! BIT1ERR_FAST - Bit1 Error in the Data Phase of CAN FD frames with the BRS bit set
 *  0b0..No such occurrence.
 *  0b1..At least one bit sent as recessive is received as dominant.
 */
#define CAN_ESR1_BIT1ERR_FAST_MASK               (0x80000000U)
#define CAN_ESR1_BIT1ERR_FAST_SHIFT              (31U)
#define CAN_ESR1_BIT1ERR_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_ESR1_BIT1ERR_FAST_SHIFT)) & CAN_ESR1_BIT1ERR_FAST_MASK)
/*! @} */

/*! @name IMASK1 - Interrupt Masks 1 register */
/*! @{ */
#define CAN_IMASK1_BUF31TO0M_MASK                (0xFFFFFFFFU)
#define CAN_IMASK1_BUF31TO0M_SHIFT               (0U)
#define CAN_IMASK1_BUF31TO0M(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IMASK1_BUF31TO0M_SHIFT)) & CAN_IMASK1_BUF31TO0M_MASK)
/*! @} */

/*! @name IFLAG1 - Interrupt Flags 1 register */
/*! @{ */
/*! BUF0I - Buffer MB0 Interrupt Or Clear FIFO bit
 *  0b0..The corresponding buffer has no occurrence of successfully completed transmission or reception when MCR[RFEN]=0.
 *  0b1..The corresponding buffer has successfully completed transmission or reception when MCR[RFEN]=0.
 */
#define CAN_IFLAG1_BUF0I_MASK                    (0x1U)
#define CAN_IFLAG1_BUF0I_SHIFT                   (0U)
#define CAN_IFLAG1_BUF0I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF0I_SHIFT)) & CAN_IFLAG1_BUF0I_MASK)
#define CAN_IFLAG1_BUF4TO1I_MASK                 (0x1EU)
#define CAN_IFLAG1_BUF4TO1I_SHIFT                (1U)
#define CAN_IFLAG1_BUF4TO1I(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF4TO1I_SHIFT)) & CAN_IFLAG1_BUF4TO1I_MASK)
/*! BUF5I - Buffer MB5 Interrupt Or Frames available in Rx FIFO
 *  0b0..No occurrence of MB5 completing transmission/reception when MCR[RFEN]=0, or of frame(s) available in the FIFO, when MCR[RFEN]=1
 *  0b1..MB5 completed transmission/reception when MCR[RFEN]=0, or frame(s) available in the Rx FIFO when
 *       MCR[RFEN]=1. It generates a DMA request in case of MCR[RFEN] and MCR[DMA] are enabled.
 */
#define CAN_IFLAG1_BUF5I_MASK                    (0x20U)
#define CAN_IFLAG1_BUF5I_SHIFT                   (5U)
#define CAN_IFLAG1_BUF5I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF5I_SHIFT)) & CAN_IFLAG1_BUF5I_MASK)
/*! BUF6I - Buffer MB6 Interrupt Or Rx FIFO Warning
 *  0b0..No occurrence of MB6 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO almost full when MCR[RFEN]=1
 *  0b1..MB6 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO almost full when MCR[RFEN]=1
 */
#define CAN_IFLAG1_BUF6I_MASK                    (0x40U)
#define CAN_IFLAG1_BUF6I_SHIFT                   (6U)
#define CAN_IFLAG1_BUF6I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF6I_SHIFT)) & CAN_IFLAG1_BUF6I_MASK)
/*! BUF7I - Buffer MB7 Interrupt Or Rx FIFO Overflow
 *  0b0..No occurrence of MB7 completing transmission/reception when MCR[RFEN]=0, or of Rx FIFO overflow when MCR[RFEN]=1
 *  0b1..MB7 completed transmission/reception when MCR[RFEN]=0, or Rx FIFO overflow when MCR[RFEN]=1
 */
#define CAN_IFLAG1_BUF7I_MASK                    (0x80U)
#define CAN_IFLAG1_BUF7I_SHIFT                   (7U)
#define CAN_IFLAG1_BUF7I(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF7I_SHIFT)) & CAN_IFLAG1_BUF7I_MASK)
#define CAN_IFLAG1_BUF31TO8I_MASK                (0xFFFFFF00U)
#define CAN_IFLAG1_BUF31TO8I_SHIFT               (8U)
#define CAN_IFLAG1_BUF31TO8I(x)                  (((uint32_t)(((uint32_t)(x)) << CAN_IFLAG1_BUF31TO8I_SHIFT)) & CAN_IFLAG1_BUF31TO8I_MASK)
/*! @} */

/*! @name CTRL2 - Control 2 register */
/*! @{ */
/*! TIMER_SRC - Timer Source
 *  0b0..The free running timer is clocked by the CAN bit clock, which defines the baud rate on the CAN bus.
 *  0b1..The free running timer is clocked by an external time tick. The period can be either adjusted to be equal
 *       to the baud rate on the CAN bus, or a different value as required. See the device-specific section for
 *       details about the external time tick.
 */
#define CAN_CTRL2_TIMER_SRC_MASK                 (0x8000U)
#define CAN_CTRL2_TIMER_SRC_SHIFT                (15U)
#define CAN_CTRL2_TIMER_SRC(x)                   (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TIMER_SRC_SHIFT)) & CAN_CTRL2_TIMER_SRC_MASK)
/*! EACEN - Entire Frame Arbitration Field Comparison Enable For Rx Mailboxes
 *  0b0..Rx mailbox filter's IDE bit is always compared and RTR is never compared despite mask bits.
 *  0b1..Enables the comparison of both Rx mailbox filter's IDE and RTR bit with their corresponding bits within
 *       the incoming frame. Mask bits do apply.
 */
#define CAN_CTRL2_EACEN_MASK                     (0x10000U)
#define CAN_CTRL2_EACEN_SHIFT                    (16U)
#define CAN_CTRL2_EACEN(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_EACEN_SHIFT)) & CAN_CTRL2_EACEN_MASK)
/*! RRS - Remote Request Storing
 *  0b0..Remote response frame is generated.
 *  0b1..Remote request frame is stored.
 */
#define CAN_CTRL2_RRS_MASK                       (0x20000U)
#define CAN_CTRL2_RRS_SHIFT                      (17U)
#define CAN_CTRL2_RRS(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RRS_SHIFT)) & CAN_CTRL2_RRS_MASK)
/*! MRP - Mailboxes Reception Priority
 *  0b0..Matching starts from Rx FIFO or Enhanced Rx FIFO and continues on mailboxes.
 *  0b1..Matching starts from mailboxes and continues on Rx FIFO.
 */
#define CAN_CTRL2_MRP_MASK                       (0x40000U)
#define CAN_CTRL2_MRP_SHIFT                      (18U)
#define CAN_CTRL2_MRP(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_MRP_SHIFT)) & CAN_CTRL2_MRP_MASK)
#define CAN_CTRL2_TASD_MASK                      (0xF80000U)
#define CAN_CTRL2_TASD_SHIFT                     (19U)
#define CAN_CTRL2_TASD(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_TASD_SHIFT)) & CAN_CTRL2_TASD_MASK)
#define CAN_CTRL2_RFFN_MASK                      (0xF000000U)
#define CAN_CTRL2_RFFN_SHIFT                     (24U)
#define CAN_CTRL2_RFFN(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_RFFN_SHIFT)) & CAN_CTRL2_RFFN_MASK)
/*! BOFFDONEMSK - Bus Off Done Interrupt Mask
 *  0b0..Bus off done interrupt disabled.
 *  0b1..Bus off done interrupt enabled.
 */
#define CAN_CTRL2_BOFFDONEMSK_MASK               (0x40000000U)
#define CAN_CTRL2_BOFFDONEMSK_SHIFT              (30U)
#define CAN_CTRL2_BOFFDONEMSK(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_BOFFDONEMSK_SHIFT)) & CAN_CTRL2_BOFFDONEMSK_MASK)
/*! ERRMSK_FAST - Error Interrupt Mask for errors detected in the data phase of fast CAN FD frames
 *  0b0..ERRINT_FAST error interrupt disabled.
 *  0b1..ERRINT_FAST error interrupt enabled.
 */
#define CAN_CTRL2_ERRMSK_FAST_MASK               (0x80000000U)
#define CAN_CTRL2_ERRMSK_FAST_SHIFT              (31U)
#define CAN_CTRL2_ERRMSK_FAST(x)                 (((uint32_t)(((uint32_t)(x)) << CAN_CTRL2_ERRMSK_FAST_SHIFT)) & CAN_CTRL2_ERRMSK_FAST_MASK)
/*! @} */

/*! @name ESR2 - Error and Status 2 register */
/*! @{ */
/*! IMB - Inactive Mailbox
 *  0b0..If ESR2[VPS] is asserted, the ESR2[LPTM] is not an inactive mailbox.
 *  0b1..If ESR2[VPS] is asserted, there is at least one inactive mailbox. LPTM content is the number of the first one.
 */
#define CAN_ESR2_IMB_MASK                        (0x2000U)
#define CAN_ESR2_IMB_SHIFT                       (13U)
#define CAN_ESR2_IMB(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_IMB_SHIFT)) & CAN_ESR2_IMB_MASK)
/*! VPS - Valid Priority Status
 *  0b0..Contents of IMB and LPTM are invalid.
 *  0b1..Contents of IMB and LPTM are valid.
 */
#define CAN_ESR2_VPS_MASK                        (0x4000U)
#define CAN_ESR2_VPS_SHIFT                       (14U)
#define CAN_ESR2_VPS(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_VPS_SHIFT)) & CAN_ESR2_VPS_MASK)
#define CAN_ESR2_LPTM_MASK                       (0x7F0000U)
#define CAN_ESR2_LPTM_SHIFT                      (16U)
#define CAN_ESR2_LPTM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_ESR2_LPTM_SHIFT)) & CAN_ESR2_LPTM_MASK)
/*! @} */

/*! @name CRCR - CRC register */
/*! @{ */
#define CAN_CRCR_TXCRC_MASK                      (0x7FFFU)
#define CAN_CRCR_TXCRC_SHIFT                     (0U)
#define CAN_CRCR_TXCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_TXCRC_SHIFT)) & CAN_CRCR_TXCRC_MASK)
#define CAN_CRCR_MBCRC_MASK                      (0x7F0000U)
#define CAN_CRCR_MBCRC_SHIFT                     (16U)
#define CAN_CRCR_MBCRC(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CRCR_MBCRC_SHIFT)) & CAN_CRCR_MBCRC_MASK)
/*! @} */

/*! @name RXFGMASK - Rx FIFO Global Mask register */
/*! @{ */
#define CAN_RXFGMASK_FGM_MASK                    (0xFFFFFFFFU)
#define CAN_RXFGMASK_FGM_SHIFT                   (0U)
#define CAN_RXFGMASK_FGM(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_RXFGMASK_FGM_SHIFT)) & CAN_RXFGMASK_FGM_MASK)
/*! @} */

/*! @name RXFIR - Rx FIFO Information register */
/*! @{ */
#define CAN_RXFIR_IDHIT_MASK                     (0x1FFU)
#define CAN_RXFIR_IDHIT_SHIFT                    (0U)
#define CAN_RXFIR_IDHIT(x)                       (((uint32_t)(((uint32_t)(x)) << CAN_RXFIR_IDHIT_SHIFT)) & CAN_RXFIR_IDHIT_MASK)
/*! @} */

/*! @name CBT - CAN Bit Timing register */
/*! @{ */
#define CAN_CBT_EPSEG2_MASK                      (0x1FU)
#define CAN_CBT_EPSEG2_SHIFT                     (0U)
#define CAN_CBT_EPSEG2(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG2_SHIFT)) & CAN_CBT_EPSEG2_MASK)
#define CAN_CBT_EPSEG1_MASK                      (0x3E0U)
#define CAN_CBT_EPSEG1_SHIFT                     (5U)
#define CAN_CBT_EPSEG1(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPSEG1_SHIFT)) & CAN_CBT_EPSEG1_MASK)
#define CAN_CBT_EPROPSEG_MASK                    (0xFC00U)
#define CAN_CBT_EPROPSEG_SHIFT                   (10U)
#define CAN_CBT_EPROPSEG(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPROPSEG_SHIFT)) & CAN_CBT_EPROPSEG_MASK)
#define CAN_CBT_ERJW_MASK                        (0xF0000U)
#define CAN_CBT_ERJW_SHIFT                       (16U)
#define CAN_CBT_ERJW(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_CBT_ERJW_SHIFT)) & CAN_CBT_ERJW_MASK)
#define CAN_CBT_EPRESDIV_MASK                    (0x7FE00000U)
#define CAN_CBT_EPRESDIV_SHIFT                   (21U)
#define CAN_CBT_EPRESDIV(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_CBT_EPRESDIV_SHIFT)) & CAN_CBT_EPRESDIV_MASK)
/*! BTF - Bit Timing Format Enable
 *  0b0..Extended bit time definitions disabled.
 *  0b1..Extended bit time definitions enabled.
 */
#define CAN_CBT_BTF_MASK                         (0x80000000U)
#define CAN_CBT_BTF_SHIFT                        (31U)
#define CAN_CBT_BTF(x)                           (((uint32_t)(((uint32_t)(x)) << CAN_CBT_BTF_SHIFT)) & CAN_CBT_BTF_MASK)
/*! @} */

/*! @name DBG1 - Debug 1 register */
/*! @{ */
#define CAN_DBG1_CFSM_MASK                       (0x7FU)
#define CAN_DBG1_CFSM_SHIFT                      (0U)
#define CAN_DBG1_CFSM(x)                         (((uint32_t)(((uint32_t)(x)) << CAN_DBG1_CFSM_SHIFT)) & CAN_DBG1_CFSM_MASK)
#define CAN_DBG1_CBN_MASK                        (0x3FF0000U)
#define CAN_DBG1_CBN_SHIFT                       (16U)
#define CAN_DBG1_CBN(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_DBG1_CBN_SHIFT)) & CAN_DBG1_CBN_MASK)
/*! @} */

/*! @name DBG2 - Debug 2 register */
/*! @{ */
#define CAN_DBG2_RMP_MASK                        (0x7FU)
#define CAN_DBG2_RMP_SHIFT                       (0U)
#define CAN_DBG2_RMP(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_DBG2_RMP_SHIFT)) & CAN_DBG2_RMP_MASK)
/*! MPP - Matching Process in Progress
 *  0b0..No matching process ongoing
 *  0b1..Matching process is in progress.
 */
#define CAN_DBG2_MPP_MASK                        (0x80U)
#define CAN_DBG2_MPP_SHIFT                       (7U)
#define CAN_DBG2_MPP(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_DBG2_MPP_SHIFT)) & CAN_DBG2_MPP_MASK)
#define CAN_DBG2_TAP_MASK                        (0x7F00U)
#define CAN_DBG2_TAP_SHIFT                       (8U)
#define CAN_DBG2_TAP(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_DBG2_TAP_SHIFT)) & CAN_DBG2_TAP_MASK)
/*! APP - Arbitration Process in Progress
 *  0b0..No arbitration process ongoing
 *  0b1..Arbitration process is in progress.
 */
#define CAN_DBG2_APP_MASK                        (0x8000U)
#define CAN_DBG2_APP_SHIFT                       (15U)
#define CAN_DBG2_APP(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_DBG2_APP_SHIFT)) & CAN_DBG2_APP_MASK)
/*! @} */

/*! @name RXIMR - Rx Individual Mask registers */
/*! @{ */
#define CAN_RXIMR_MI_MASK                        (0xFFFFFFFFU)
#define CAN_RXIMR_MI_SHIFT                       (0U)
#define CAN_RXIMR_MI(x)                          (((uint32_t)(((uint32_t)(x)) << CAN_RXIMR_MI_SHIFT)) & CAN_RXIMR_MI_MASK)
/*! @} */

/* The count of CAN_RXIMR */
#define CAN_RXIMR_COUNT                          (32U)

/*! @name FDCTRL - CAN FD Control register */
/*! @{ */
#define CAN_FDCTRL_TDCVAL_MASK                   (0x3FU)
#define CAN_FDCTRL_TDCVAL_SHIFT                  (0U)
#define CAN_FDCTRL_TDCVAL(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCVAL_SHIFT)) & CAN_FDCTRL_TDCVAL_MASK)
#define CAN_FDCTRL_TDCOFF_MASK                   (0x1F00U)
#define CAN_FDCTRL_TDCOFF_SHIFT                  (8U)
#define CAN_FDCTRL_TDCOFF(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCOFF_SHIFT)) & CAN_FDCTRL_TDCOFF_MASK)
/*! TDCFAIL - Transceiver Delay Compensation Fail
 *  0b0..Measured loop delay is in range.
 *  0b1..Measured loop delay is out of range.
 */
#define CAN_FDCTRL_TDCFAIL_MASK                  (0x4000U)
#define CAN_FDCTRL_TDCFAIL_SHIFT                 (14U)
#define CAN_FDCTRL_TDCFAIL(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCFAIL_SHIFT)) & CAN_FDCTRL_TDCFAIL_MASK)
/*! TDCEN - Transceiver Delay Compensation Enable
 *  0b0..TDC is disabled
 *  0b1..TDC is enabled
 */
#define CAN_FDCTRL_TDCEN_MASK                    (0x8000U)
#define CAN_FDCTRL_TDCEN_SHIFT                   (15U)
#define CAN_FDCTRL_TDCEN(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_TDCEN_SHIFT)) & CAN_FDCTRL_TDCEN_MASK)
/*! MBDSR0 - Message Buffer Data Size for Region 0
 *  0b00..Selects 8 bytes per message buffer.
 *  0b01..Selects 16 bytes per message buffer.
 *  0b10..Selects 32 bytes per message buffer.
 *  0b11..Selects 64 bytes per message buffer.
 */
#define CAN_FDCTRL_MBDSR0_MASK                   (0x30000U)
#define CAN_FDCTRL_MBDSR0_SHIFT                  (16U)
#define CAN_FDCTRL_MBDSR0(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_MBDSR0_SHIFT)) & CAN_FDCTRL_MBDSR0_MASK)
/*! FDRATE - Bit Rate Switch Enable
 *  0b0..Transmit a frame in nominal rate. The BRS bit in the Tx MB has no effect.
 *  0b1..Transmit a frame with bit rate switching if the BRS bit in the Tx MB is recessive.
 */
#define CAN_FDCTRL_FDRATE_MASK                   (0x80000000U)
#define CAN_FDCTRL_FDRATE_SHIFT                  (31U)
#define CAN_FDCTRL_FDRATE(x)                     (((uint32_t)(((uint32_t)(x)) << CAN_FDCTRL_FDRATE_SHIFT)) & CAN_FDCTRL_FDRATE_MASK)
/*! @} */

/*! @name FDCBT - CAN FD Bit Timing register */
/*! @{ */
#define CAN_FDCBT_FPSEG2_MASK                    (0x7U)
#define CAN_FDCBT_FPSEG2_SHIFT                   (0U)
#define CAN_FDCBT_FPSEG2(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG2_SHIFT)) & CAN_FDCBT_FPSEG2_MASK)
#define CAN_FDCBT_FPSEG1_MASK                    (0xE0U)
#define CAN_FDCBT_FPSEG1_SHIFT                   (5U)
#define CAN_FDCBT_FPSEG1(x)                      (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPSEG1_SHIFT)) & CAN_FDCBT_FPSEG1_MASK)
#define CAN_FDCBT_FPROPSEG_MASK                  (0x7C00U)
#define CAN_FDCBT_FPROPSEG_SHIFT                 (10U)
#define CAN_FDCBT_FPROPSEG(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPROPSEG_SHIFT)) & CAN_FDCBT_FPROPSEG_MASK)
#define CAN_FDCBT_FRJW_MASK                      (0x30000U)
#define CAN_FDCBT_FRJW_SHIFT                     (16U)
#define CAN_FDCBT_FRJW(x)                        (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FRJW_SHIFT)) & CAN_FDCBT_FRJW_MASK)
#define CAN_FDCBT_FPRESDIV_MASK                  (0x3FF00000U)
#define CAN_FDCBT_FPRESDIV_SHIFT                 (20U)
#define CAN_FDCBT_FPRESDIV(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCBT_FPRESDIV_SHIFT)) & CAN_FDCBT_FPRESDIV_MASK)
/*! @} */

/*! @name FDCRC - CAN FD CRC register */
/*! @{ */
#define CAN_FDCRC_FD_TXCRC_MASK                  (0x1FFFFFU)
#define CAN_FDCRC_FD_TXCRC_SHIFT                 (0U)
#define CAN_FDCRC_FD_TXCRC(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_TXCRC_SHIFT)) & CAN_FDCRC_FD_TXCRC_MASK)
#define CAN_FDCRC_FD_MBCRC_MASK                  (0x7F000000U)
#define CAN_FDCRC_FD_MBCRC_SHIFT                 (24U)
#define CAN_FDCRC_FD_MBCRC(x)                    (((uint32_t)(((uint32_t)(x)) << CAN_FDCRC_FD_MBCRC_SHIFT)) & CAN_FDCRC_FD_MBCRC_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CAN_Register_Masks */


/* CAN - Peripheral instance base addresses */
/** Peripheral CAN base address */
#define CAN_BASE                                 (0xF000u)
/** Peripheral CAN base pointer */
#define CAN                                      ((CAN_Type *)CAN_BASE)
/** Array initializer of CAN peripheral base addresses */
#define CAN_BASE_ADDRS                           { CAN_BASE }
/** Array initializer of CAN peripheral base pointers */
#define CAN_BASE_PTRS                            { CAN }

/*!
 * @}
 */ /* end of group CAN_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint16_t CR0;                               /**< CMP Control Register 0, offset: 0x0 */
  __IO uint16_t CR1;                               /**< CMP Control Register 1, offset: 0x1 */
  __IO uint16_t FPR;                               /**< CMP Filter Period Register, offset: 0x2 */
  __IO uint16_t SCR;                               /**< CMP Status and Control Register, offset: 0x3 */
  __IO uint16_t DACCR;                             /**< DAC Control Register, offset: 0x4 */
  __IO uint16_t MUXCR;                             /**< MUX Control Register, offset: 0x5 */
} CMP_Type;

/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/*! @name CR0 - CMP Control Register 0 */
/*! @{ */
/*! HYSTCTR - Comparator hard block hysteresis control
 *  0b00..Level 0
 *  0b01..Level 1
 *  0b10..Level 2
 *  0b11..Level 3
 */
#define CMP_CR0_HYSTCTR_MASK                     (0x3U)
#define CMP_CR0_HYSTCTR_SHIFT                    (0U)
#define CMP_CR0_HYSTCTR(x)                       (((uint16_t)(((uint16_t)(x)) << CMP_CR0_HYSTCTR_SHIFT)) & CMP_CR0_HYSTCTR_MASK)
/*! FILTER_CNT - Filter Sample Count
 *  0b000..Filter is disabled. If SE = 1, then COUT is a logic 0. This is not a legal state, and is not recommended. If SE = 0, COUT = COUTA.
 *  0b001..One sample must agree. The comparator output is simply sampled.
 *  0b010..2 consecutive samples must agree.
 *  0b011..3 consecutive samples must agree.
 *  0b100..4 consecutive samples must agree.
 *  0b101..5 consecutive samples must agree.
 *  0b110..6 consecutive samples must agree.
 *  0b111..7 consecutive samples must agree.
 */
#define CMP_CR0_FILTER_CNT_MASK                  (0x70U)
#define CMP_CR0_FILTER_CNT_SHIFT                 (4U)
#define CMP_CR0_FILTER_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << CMP_CR0_FILTER_CNT_SHIFT)) & CMP_CR0_FILTER_CNT_MASK)
/*! @} */

/*! @name CR1 - CMP Control Register 1 */
/*! @{ */
/*! EN - Comparator Module Enable
 *  0b0..Analog Comparator is disabled.
 *  0b1..Analog Comparator is enabled.
 */
#define CMP_CR1_EN_MASK                          (0x1U)
#define CMP_CR1_EN_SHIFT                         (0U)
#define CMP_CR1_EN(x)                            (((uint16_t)(((uint16_t)(x)) << CMP_CR1_EN_SHIFT)) & CMP_CR1_EN_MASK)
/*! OPE - Comparator Output Pin Enable
 *  0b0..CMPO is not available on the associated CMPO output pin. If the comparator does not own the pin, this field has no effect.
 *  0b1..CMPO is available on the associated CMPO output pin. The comparator output (CMPO) is driven out on the
 *       associated CMPO output pin if the comparator owns the pin. If the comparator does not own the field, this
 *       bit has no effect.
 */
#define CMP_CR1_OPE_MASK                         (0x2U)
#define CMP_CR1_OPE_SHIFT                        (1U)
#define CMP_CR1_OPE(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_CR1_OPE_SHIFT)) & CMP_CR1_OPE_MASK)
/*! COS - Comparator Output Select
 *  0b0..Set the filtered comparator output (CMPO) to equal COUT.
 *  0b1..Set the unfiltered comparator output (CMPO) to equal COUTA.
 */
#define CMP_CR1_COS_MASK                         (0x4U)
#define CMP_CR1_COS_SHIFT                        (2U)
#define CMP_CR1_COS(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_CR1_COS_SHIFT)) & CMP_CR1_COS_MASK)
/*! INV - Comparator INVERT
 *  0b0..Does not invert the comparator output.
 *  0b1..Inverts the comparator output.
 */
#define CMP_CR1_INV_MASK                         (0x8U)
#define CMP_CR1_INV_SHIFT                        (3U)
#define CMP_CR1_INV(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_CR1_INV_SHIFT)) & CMP_CR1_INV_MASK)
/*! PMODE - Power Mode Select
 *  0b0..Low-Speed (LS) Comparison mode selected. In this mode, CMP has slower output propagation delay and lower current consumption.
 *  0b1..High-Speed (HS) Comparison mode selected. In this mode, CMP has faster output propagation delay and higher current consumption.
 */
#define CMP_CR1_PMODE_MASK                       (0x10U)
#define CMP_CR1_PMODE_SHIFT                      (4U)
#define CMP_CR1_PMODE(x)                         (((uint16_t)(((uint16_t)(x)) << CMP_CR1_PMODE_SHIFT)) & CMP_CR1_PMODE_MASK)
/*! COWZ - COUTA out of window is zero enable.
 *  0b0..In windowing mode, when WINDOW signal changes from 1 to 0, COUTA holds the last latched value before WINDOW signal falls to 0.
 *  0b1..In windowing mode, when WINDOW signal changes from 1 to 0, COUTA is forced to 0.
 */
#define CMP_CR1_COWZ_MASK                        (0x20U)
#define CMP_CR1_COWZ_SHIFT                       (5U)
#define CMP_CR1_COWZ(x)                          (((uint16_t)(((uint16_t)(x)) << CMP_CR1_COWZ_SHIFT)) & CMP_CR1_COWZ_MASK)
/*! WE - Windowing Enable
 *  0b0..Windowing mode is not selected.
 *  0b1..Windowing mode is selected.
 */
#define CMP_CR1_WE_MASK                          (0x40U)
#define CMP_CR1_WE_SHIFT                         (6U)
#define CMP_CR1_WE(x)                            (((uint16_t)(((uint16_t)(x)) << CMP_CR1_WE_SHIFT)) & CMP_CR1_WE_MASK)
/*! SE - Sample Enable
 *  0b0..Sampling mode is not selected.
 *  0b1..Sampling mode is selected.
 */
#define CMP_CR1_SE_MASK                          (0x80U)
#define CMP_CR1_SE_SHIFT                         (7U)
#define CMP_CR1_SE(x)                            (((uint16_t)(((uint16_t)(x)) << CMP_CR1_SE_SHIFT)) & CMP_CR1_SE_MASK)
/*! @} */

/*! @name FPR - CMP Filter Period Register */
/*! @{ */
#define CMP_FPR_FILT_PER_MASK                    (0xFFU)
#define CMP_FPR_FILT_PER_SHIFT                   (0U)
#define CMP_FPR_FILT_PER(x)                      (((uint16_t)(((uint16_t)(x)) << CMP_FPR_FILT_PER_SHIFT)) & CMP_FPR_FILT_PER_MASK)
/*! @} */

/*! @name SCR - CMP Status and Control Register */
/*! @{ */
#define CMP_SCR_COUT_MASK                        (0x1U)
#define CMP_SCR_COUT_SHIFT                       (0U)
#define CMP_SCR_COUT(x)                          (((uint16_t)(((uint16_t)(x)) << CMP_SCR_COUT_SHIFT)) & CMP_SCR_COUT_MASK)
/*! CFF - Analog Comparator Flag Falling
 *  0b0..Falling-edge on COUT has not been detected.
 *  0b1..Falling-edge on COUT has occurred.
 */
#define CMP_SCR_CFF_MASK                         (0x2U)
#define CMP_SCR_CFF_SHIFT                        (1U)
#define CMP_SCR_CFF(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_SCR_CFF_SHIFT)) & CMP_SCR_CFF_MASK)
/*! CFR - Analog Comparator Flag Rising
 *  0b0..Rising-edge on COUT has not been detected.
 *  0b1..Rising-edge on COUT has occurred.
 */
#define CMP_SCR_CFR_MASK                         (0x4U)
#define CMP_SCR_CFR_SHIFT                        (2U)
#define CMP_SCR_CFR(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_SCR_CFR_SHIFT)) & CMP_SCR_CFR_MASK)
/*! IEF - Comparator Interrupt Enable Falling
 *  0b0..Interrupt is disabled.
 *  0b1..Interrupt is enabled.
 */
#define CMP_SCR_IEF_MASK                         (0x8U)
#define CMP_SCR_IEF_SHIFT                        (3U)
#define CMP_SCR_IEF(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_SCR_IEF_SHIFT)) & CMP_SCR_IEF_MASK)
/*! IER - Comparator Interrupt Enable Rising
 *  0b0..Interrupt is disabled.
 *  0b1..Interrupt is enabled.
 */
#define CMP_SCR_IER_MASK                         (0x10U)
#define CMP_SCR_IER_SHIFT                        (4U)
#define CMP_SCR_IER(x)                           (((uint16_t)(((uint16_t)(x)) << CMP_SCR_IER_SHIFT)) & CMP_SCR_IER_MASK)
/*! DMAEN - DMA Enable Control
 *  0b0..DMA is disabled.
 *  0b1..DMA is enabled.
 */
#define CMP_SCR_DMAEN_MASK                       (0x40U)
#define CMP_SCR_DMAEN_SHIFT                      (6U)
#define CMP_SCR_DMAEN(x)                         (((uint16_t)(((uint16_t)(x)) << CMP_SCR_DMAEN_SHIFT)) & CMP_SCR_DMAEN_MASK)
/*! @} */

/*! @name DACCR - DAC Control Register */
/*! @{ */
#define CMP_DACCR_VOSEL_MASK                     (0xFFU)
#define CMP_DACCR_VOSEL_SHIFT                    (0U)
#define CMP_DACCR_VOSEL(x)                       (((uint16_t)(((uint16_t)(x)) << CMP_DACCR_VOSEL_SHIFT)) & CMP_DACCR_VOSEL_MASK)
/*! @} */

/*! @name MUXCR - MUX Control Register */
/*! @{ */
/*! MSEL - Minus Input Mux Control
 *  0b000..IN0
 *  0b001..IN1
 *  0b010..IN2
 *  0b011..IN3
 *  0b100..IN4
 *  0b101..IN5
 *  0b110..IN6
 *  0b111..IN7
 */
#define CMP_MUXCR_MSEL_MASK                      (0x7U)
#define CMP_MUXCR_MSEL_SHIFT                     (0U)
#define CMP_MUXCR_MSEL(x)                        (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_MSEL_SHIFT)) & CMP_MUXCR_MSEL_MASK)
/*! PSEL - Plus Input Mux Control
 *  0b000..IN0
 *  0b001..IN1
 *  0b010..IN2
 *  0b011..IN3
 *  0b100..IN4
 *  0b101..IN5
 *  0b110..IN6
 *  0b111..IN7
 */
#define CMP_MUXCR_PSEL_MASK                      (0x38U)
#define CMP_MUXCR_PSEL_SHIFT                     (3U)
#define CMP_MUXCR_PSEL(x)                        (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_PSEL_SHIFT)) & CMP_MUXCR_PSEL_MASK)
/*! VRSEL - Supply Voltage Reference Source Select
 *  0b0..Vin1 is selected as resistor ladder network supply reference.
 *  0b1..Vin2 is selected as resistor ladder network supply reference.
 */
#define CMP_MUXCR_VRSEL_MASK                     (0x40U)
#define CMP_MUXCR_VRSEL_SHIFT                    (6U)
#define CMP_MUXCR_VRSEL(x)                       (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_VRSEL_SHIFT)) & CMP_MUXCR_VRSEL_MASK)
/*! DACEN - DAC Enable
 *  0b0..DAC is disabled.
 *  0b1..DAC is enabled.
 */
#define CMP_MUXCR_DACEN_MASK                     (0x80U)
#define CMP_MUXCR_DACEN_SHIFT                    (7U)
#define CMP_MUXCR_DACEN(x)                       (((uint16_t)(((uint16_t)(x)) << CMP_MUXCR_DACEN_SHIFT)) & CMP_MUXCR_DACEN_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMPA base address */
#define CMPA_BASE                                (0xE020u)
/** Peripheral CMPA base pointer */
#define CMPA                                     ((CMP_Type *)CMPA_BASE)
/** Peripheral CMPB base address */
#define CMPB_BASE                                (0xE028u)
/** Peripheral CMPB base pointer */
#define CMPB                                     ((CMP_Type *)CMPB_BASE)
/** Peripheral CMPC base address */
#define CMPC_BASE                                (0xE030u)
/** Peripheral CMPC base pointer */
#define CMPC                                     ((CMP_Type *)CMPC_BASE)
/** Peripheral CMPD base address */
#define CMPD_BASE                                (0xE038u)
/** Peripheral CMPD base pointer */
#define CMPD                                     ((CMP_Type *)CMPD_BASE)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMPA_BASE, CMPB_BASE, CMPC_BASE, CMPD_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMPA, CMPB, CMPC, CMPD }

/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- COP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Peripheral_Access_Layer COP Peripheral Access Layer
 * @{
 */

/** COP - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< COP Control Register, offset: 0x0 */
  __IO uint16_t TOUT;                              /**< COP Timeout Register, offset: 0x1 */
  __IO uint16_t CNTR;                              /**< COP Counter Register, offset: 0x2 */
  __IO uint16_t INTVAL;                            /**< COP Interrupt Value Register, offset: 0x3 */
  __IO uint16_t WINDOW;                            /**< COP Window Timeout Register, offset: 0x4 */
} COP_Type;

/* ----------------------------------------------------------------------------
   -- COP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup COP_Register_Masks COP Register Masks
 * @{
 */

/*! @name CTRL - COP Control Register */
/*! @{ */
/*! CWP - COP Write Protect
 *  0b0..The CTRL, INTVAL , WINDOW and TOUT registers are readable and writable. (default)
 *  0b1..The CTRL, INTVAL , WINDOW and TOUT registers are read-only.
 */
#define COP_CTRL_CWP_MASK                        (0x1U)
#define COP_CTRL_CWP_SHIFT                       (0U)
#define COP_CTRL_CWP(x)                          (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CWP_SHIFT)) & COP_CTRL_CWP_MASK)
/*! CEN - COP Enable
 *  0b0..COP counter is disabled.
 *  0b1..COP counter is enabled. (default)
 */
#define COP_CTRL_CEN_MASK                        (0x2U)
#define COP_CTRL_CEN_SHIFT                       (1U)
#define COP_CTRL_CEN(x)                          (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CEN_SHIFT)) & COP_CTRL_CEN_MASK)
/*! CWEN - COP Wait Mode Enable
 *  0b0..COP counter stops in wait mode. (default)
 *  0b1..COP counter runs in wait mode if CEN is set to one.
 */
#define COP_CTRL_CWEN_MASK                       (0x4U)
#define COP_CTRL_CWEN_SHIFT                      (2U)
#define COP_CTRL_CWEN(x)                         (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CWEN_SHIFT)) & COP_CTRL_CWEN_MASK)
/*! CSEN - COP Stop Mode Enable
 *  0b0..COP counter stops in stop mode. (default)
 *  0b1..COP counter runs in stop mode if CEN is set to one.
 */
#define COP_CTRL_CSEN_MASK                       (0x8U)
#define COP_CTRL_CSEN_SHIFT                      (3U)
#define COP_CTRL_CSEN(x)                         (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CSEN_SHIFT)) & COP_CTRL_CSEN_MASK)
/*! CLOREN - COP Loss of Reference Enable
 *  0b0..COP loss of reference counter is disabled. (default)
 *  0b1..COP loss of reference counter is enabled.
 */
#define COP_CTRL_CLOREN_MASK                     (0x10U)
#define COP_CTRL_CLOREN_SHIFT                    (4U)
#define COP_CTRL_CLOREN(x)                       (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CLOREN_SHIFT)) & COP_CTRL_CLOREN_MASK)
/*! CLKSEL - Clock Source Select
 *  0b00..Relaxation oscillator output (ROSC) is used to clock the counter (default)
 *  0b01..Crystal oscillator output (COSC) is used to clock the counter
 *  0b10..IP bus clock is used to clock the counter Do not select the IP bus clock to clock the counter if the
 *        application requires the COP to wake the device from stop mode.
 *  0b11..Low speed oscillator is used to clock the counter
 */
#define COP_CTRL_CLKSEL_MASK                     (0x60U)
#define COP_CTRL_CLKSEL_SHIFT                    (5U)
#define COP_CTRL_CLKSEL(x)                       (((uint16_t)(((uint16_t)(x)) << COP_CTRL_CLKSEL_SHIFT)) & COP_CTRL_CLKSEL_MASK)
/*! INTEN - Interrupt Enable
 *  0b0..COP interrupt is disabled. (default)
 *  0b1..COP interrupt is enabled.
 */
#define COP_CTRL_INTEN_MASK                      (0x80U)
#define COP_CTRL_INTEN_SHIFT                     (7U)
#define COP_CTRL_INTEN(x)                        (((uint16_t)(((uint16_t)(x)) << COP_CTRL_INTEN_SHIFT)) & COP_CTRL_INTEN_MASK)
/*! PSS - Prescaler Select
 *  0b00..No division
 *  0b01..Divide by 16
 *  0b10..Divide by 256
 *  0b11..Divide by 1024
 */
#define COP_CTRL_PSS_MASK                        (0x300U)
#define COP_CTRL_PSS_SHIFT                       (8U)
#define COP_CTRL_PSS(x)                          (((uint16_t)(((uint16_t)(x)) << COP_CTRL_PSS_SHIFT)) & COP_CTRL_PSS_MASK)
/*! @} */

/*! @name TOUT - COP Timeout Register */
/*! @{ */
#define COP_TOUT_TIMEOUT_MASK                    (0xFFFFU)
#define COP_TOUT_TIMEOUT_SHIFT                   (0U)
#define COP_TOUT_TIMEOUT(x)                      (((uint16_t)(((uint16_t)(x)) << COP_TOUT_TIMEOUT_SHIFT)) & COP_TOUT_TIMEOUT_MASK)
/*! @} */

/*! @name CNTR - COP Counter Register */
/*! @{ */
#define COP_CNTR_COUNT_SERVICE_MASK              (0xFFFFU)
#define COP_CNTR_COUNT_SERVICE_SHIFT             (0U)
#define COP_CNTR_COUNT_SERVICE(x)                (((uint16_t)(((uint16_t)(x)) << COP_CNTR_COUNT_SERVICE_SHIFT)) & COP_CNTR_COUNT_SERVICE_MASK)
/*! @} */

/*! @name INTVAL - COP Interrupt Value Register */
/*! @{ */
#define COP_INTVAL_INTERRUPT_VALUE_MASK          (0xFFFFU)
#define COP_INTVAL_INTERRUPT_VALUE_SHIFT         (0U)
#define COP_INTVAL_INTERRUPT_VALUE(x)            (((uint16_t)(((uint16_t)(x)) << COP_INTVAL_INTERRUPT_VALUE_SHIFT)) & COP_INTVAL_INTERRUPT_VALUE_MASK)
/*! @} */

/*! @name WINDOW - COP Window Timeout Register */
/*! @{ */
#define COP_WINDOW_WINDOW_VALUE_MASK             (0xFFFFU)
#define COP_WINDOW_WINDOW_VALUE_SHIFT            (0U)
#define COP_WINDOW_WINDOW_VALUE(x)               (((uint16_t)(((uint16_t)(x)) << COP_WINDOW_WINDOW_VALUE_SHIFT)) & COP_WINDOW_WINDOW_VALUE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group COP_Register_Masks */


/* COP - Peripheral instance base addresses */
/** Peripheral COP base address */
#define COP_BASE                                 (0xE320u)
/** Peripheral COP base pointer */
#define COP                                      ((COP_Type *)COP_BASE)
/** Array initializer of COP peripheral base addresses */
#define COP_BASE_ADDRS                           { COP_BASE }
/** Array initializer of COP peripheral base pointers */
#define COP_BASE_PTRS                            { COP }

/*!
 * @}
 */ /* end of group COP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  __IO uint32_t DATA;                              /**< CRC Data register, offset: 0x0 */
  __IO uint32_t GPOLY;                             /**< CRC Polynomial register, offset: 0x2 */
  __IO uint32_t CTRL;                              /**< CRC Control register, offset: 0x4 */
} CRC_Type;

/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/*! @name DATA - CRC Data register */
/*! @{ */
#define CRC_DATA_LL_MASK                         (0xFFU)
#define CRC_DATA_LL_SHIFT                        (0U)
#define CRC_DATA_LL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_LL_SHIFT)) & CRC_DATA_LL_MASK)
#define CRC_DATA_LU_MASK                         (0xFF00U)
#define CRC_DATA_LU_SHIFT                        (8U)
#define CRC_DATA_LU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_LU_SHIFT)) & CRC_DATA_LU_MASK)
#define CRC_DATA_HL_MASK                         (0xFF0000U)
#define CRC_DATA_HL_SHIFT                        (16U)
#define CRC_DATA_HL(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_HL_SHIFT)) & CRC_DATA_HL_MASK)
#define CRC_DATA_HU_MASK                         (0xFF000000U)
#define CRC_DATA_HU_SHIFT                        (24U)
#define CRC_DATA_HU(x)                           (((uint32_t)(((uint32_t)(x)) << CRC_DATA_HU_SHIFT)) & CRC_DATA_HU_MASK)
/*! @} */

/*! @name GPOLY - CRC Polynomial register */
/*! @{ */
#define CRC_GPOLY_LOW_MASK                       (0xFFFFU)
#define CRC_GPOLY_LOW_SHIFT                      (0U)
#define CRC_GPOLY_LOW(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_LOW_SHIFT)) & CRC_GPOLY_LOW_MASK)
#define CRC_GPOLY_HIGH_MASK                      (0xFFFF0000U)
#define CRC_GPOLY_HIGH_SHIFT                     (16U)
#define CRC_GPOLY_HIGH(x)                        (((uint32_t)(((uint32_t)(x)) << CRC_GPOLY_HIGH_SHIFT)) & CRC_GPOLY_HIGH_MASK)
/*! @} */

/*! @name CTRL - CRC Control register */
/*! @{ */
/*! TCRC
 *  0b0..16-bit CRC protocol.
 *  0b1..32-bit CRC protocol.
 */
#define CRC_CTRL_TCRC_MASK                       (0x1000000U)
#define CRC_CTRL_TCRC_SHIFT                      (24U)
#define CRC_CTRL_TCRC(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TCRC_SHIFT)) & CRC_CTRL_TCRC_MASK)
/*! WAS - Write CRC Data Register As Seed
 *  0b0..Writes to the CRC data register are data values.
 *  0b1..Writes to the CRC data register are seed values.
 */
#define CRC_CTRL_WAS_MASK                        (0x2000000U)
#define CRC_CTRL_WAS_SHIFT                       (25U)
#define CRC_CTRL_WAS(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_WAS_SHIFT)) & CRC_CTRL_WAS_MASK)
/*! FXOR - Complement Read Of CRC Data Register
 *  0b0..No XOR on reading.
 *  0b1..Invert or complement the read value of the CRC Data register.
 */
#define CRC_CTRL_FXOR_MASK                       (0x4000000U)
#define CRC_CTRL_FXOR_SHIFT                      (26U)
#define CRC_CTRL_FXOR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_FXOR_SHIFT)) & CRC_CTRL_FXOR_MASK)
/*! TOTR - Type Of Transpose For Read
 *  0b00..No transposition.
 *  0b01..Bits in bytes are transposed; bytes are not transposed.
 *  0b10..Both bits in bytes and bytes are transposed.
 *  0b11..Only bytes are transposed; no bits in a byte are transposed.
 */
#define CRC_CTRL_TOTR_MASK                       (0x30000000U)
#define CRC_CTRL_TOTR_SHIFT                      (28U)
#define CRC_CTRL_TOTR(x)                         (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOTR_SHIFT)) & CRC_CTRL_TOTR_MASK)
/*! TOT - Type Of Transpose For Writes
 *  0b00..No transposition.
 *  0b01..Bits in bytes are transposed; bytes are not transposed.
 *  0b10..Both bits in bytes and bytes are transposed.
 *  0b11..Only bytes are transposed; no bits in a byte are transposed.
 */
#define CRC_CTRL_TOT_MASK                        (0xC0000000U)
#define CRC_CTRL_TOT_SHIFT                       (30U)
#define CRC_CTRL_TOT(x)                          (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_TOT_SHIFT)) & CRC_CTRL_TOT_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE                                 (0xE3A0u)
/** Peripheral CRC base pointer */
#define CRC                                      ((CRC_Type *)CRC_BASE)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC }

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DAC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral_Access_Layer DAC Peripheral Access Layer
 * @{
 */

/** DAC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL0;                             /**< Control Register 0, offset: 0x0 */
  union {                                          /* offset: 0x1 */
    __IO uint16_t DATAREG_FMT0;                      /**< Buffered Data Register, offset: 0x1 */
    __IO uint16_t DATAREG_FMT1;                      /**< Buffered Data Register, offset: 0x1 */
  };
  union {                                          /* offset: 0x2 */
    __IO uint16_t STEPVAL_FMT0;                      /**< Step Size Register, offset: 0x2 */
    __IO uint16_t STEPVAL_FMT1;                      /**< Step Size Register, offset: 0x2 */
  };
  union {                                          /* offset: 0x3 */
    __IO uint16_t MINVAL_FMT0;                       /**< Minimum Value Register, offset: 0x3 */
    __IO uint16_t MINVAL_FMT1;                       /**< Minimum Value Register, offset: 0x3 */
  };
  union {                                          /* offset: 0x4 */
    __IO uint16_t MAXVAL_FMT0;                       /**< Maximum Value Register, offset: 0x4 */
    __IO uint16_t MAXVAL_FMT1;                       /**< Maximum Value Register, offset: 0x4 */
  };
  __I  uint16_t STATUS;                            /**< Status Register, offset: 0x5 */
  __IO uint16_t CTRL1;                             /**< Control Register 1, offset: 0x6 */
  __IO uint16_t COMPARE;                           /**< Compare Register, offset: 0x7 */
} DAC_Type;

/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/*! @name CTRL0 - Control Register 0 */
/*! @{ */
/*! PDN - Power Down
 *  0b0..DAC is operational.
 *  0b1..DAC is powered down. (default)
 */
#define DAC_CTRL0_PDN_MASK                       (0x1U)
#define DAC_CTRL0_PDN_SHIFT                      (0U)
#define DAC_CTRL0_PDN(x)                         (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_PDN_SHIFT)) & DAC_CTRL0_PDN_MASK)
/*! FORMAT - Data Format
 *  0b0..Data words are right-justified (default)
 *  0b1..Data words are left-justified
 */
#define DAC_CTRL0_FORMAT_MASK                    (0x2U)
#define DAC_CTRL0_FORMAT_SHIFT                   (1U)
#define DAC_CTRL0_FORMAT(x)                      (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_FORMAT_SHIFT)) & DAC_CTRL0_FORMAT_MASK)
/*! SYNC_EN - Sync Enable
 *  0b0..Asynchronous mode. Data written to the buffered registers is used on the next clock cycle.
 *  0b1..Synchronous mode. SYNC_IN signal updates data in the buffered registers.
 */
#define DAC_CTRL0_SYNC_EN_MASK                   (0x4U)
#define DAC_CTRL0_SYNC_EN_SHIFT                  (2U)
#define DAC_CTRL0_SYNC_EN(x)                     (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_SYNC_EN_SHIFT)) & DAC_CTRL0_SYNC_EN_MASK)
/*! AUTO - Automatic Mode
 *  0b0..Normal mode. Automatic waveform generation disabled.
 *  0b1..Automatic waveform generation enabled.
 */
#define DAC_CTRL0_AUTO_MASK                      (0x8U)
#define DAC_CTRL0_AUTO_SHIFT                     (3U)
#define DAC_CTRL0_AUTO(x)                        (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_AUTO_SHIFT)) & DAC_CTRL0_AUTO_MASK)
/*! DOWN - Enable Down Counting
 *  0b0..Disable down-counting
 *  0b1..Enable down-counting
 */
#define DAC_CTRL0_DOWN_MASK                      (0x10U)
#define DAC_CTRL0_DOWN_SHIFT                     (4U)
#define DAC_CTRL0_DOWN(x)                        (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_DOWN_SHIFT)) & DAC_CTRL0_DOWN_MASK)
/*! UP - Enable Up-Counting
 *  0b0..Disable up-counting
 *  0b1..Enable up-counting
 */
#define DAC_CTRL0_UP_MASK                        (0x20U)
#define DAC_CTRL0_UP_SHIFT                       (5U)
#define DAC_CTRL0_UP(x)                          (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_UP_SHIFT)) & DAC_CTRL0_UP_MASK)
/*! HSLS - High/Low Speed
 *  0b0..High speed mode (default)
 *  0b1..Low speed mode
 */
#define DAC_CTRL0_HSLS_MASK                      (0x40U)
#define DAC_CTRL0_HSLS_SHIFT                     (6U)
#define DAC_CTRL0_HSLS(x)                        (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_HSLS_SHIFT)) & DAC_CTRL0_HSLS_MASK)
/*! DMA_EN - Enable DMA Support
 *  0b0..Disable DMA support (default)
 *  0b1..Enable DMA support
 */
#define DAC_CTRL0_DMA_EN_MASK                    (0x80U)
#define DAC_CTRL0_DMA_EN_SHIFT                   (7U)
#define DAC_CTRL0_DMA_EN(x)                      (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_DMA_EN_SHIFT)) & DAC_CTRL0_DMA_EN_MASK)
/*! WTMK_LVL - Watermark Level
 *  0b00..Watermark value is 0
 *  0b01..Watermark value is 2 (default)
 *  0b10..Watermark value is 4
 *  0b11..Watermark value is 6
 */
#define DAC_CTRL0_WTMK_LVL_MASK                  (0x300U)
#define DAC_CTRL0_WTMK_LVL_SHIFT                 (8U)
#define DAC_CTRL0_WTMK_LVL(x)                    (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_WTMK_LVL_SHIFT)) & DAC_CTRL0_WTMK_LVL_MASK)
/*! SYNCEDGE - Sync edge
 *  0b00..No active edge is selected, therefore the SYNC input is ignored
 *  0b01..Updates occur on the falling edge of the SYNC input
 *  0b10..Updates occur on the rising edge of the SYNC input
 *  0b11..Updates occur on both edges of the SYNC input
 */
#define DAC_CTRL0_SYNCEDGE_MASK                  (0xC00U)
#define DAC_CTRL0_SYNCEDGE_SHIFT                 (10U)
#define DAC_CTRL0_SYNCEDGE(x)                    (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_SYNCEDGE_SHIFT)) & DAC_CTRL0_SYNCEDGE_MASK)
/*! FILT_EN - Glitch Filter Enable
 *  0b0..Disable glitch filter
 *  0b1..Enable glitch filter
 */
#define DAC_CTRL0_FILT_EN_MASK                   (0x1000U)
#define DAC_CTRL0_FILT_EN_SHIFT                  (12U)
#define DAC_CTRL0_FILT_EN(x)                     (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_FILT_EN_SHIFT)) & DAC_CTRL0_FILT_EN_MASK)
/*! LDOK - Load Okay
 *  0b0..Buffered values of STEPVAL, MINVAL, and MAXVAL will not be updated and the existing values will be reused.
 *  0b1..Buffered values of STEPVAL, MINVAL, and MAXVAL will be updated and used at active edge of SYNC_IN.
 */
#define DAC_CTRL0_LDOK_MASK                      (0x4000U)
#define DAC_CTRL0_LDOK_SHIFT                     (14U)
#define DAC_CTRL0_LDOK(x)                        (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_LDOK_SHIFT)) & DAC_CTRL0_LDOK_MASK)
/*! ONESHOT - One shot
 *  0b0..Automatic waveform generation logic will create a repeated (continuous) waveform upon receiving an active
 *       SYNC edge, otherwise the waveform repeats when it reaches its MIN or MAX value.
 *  0b1..Automatic waveform generation logic will create a single pattern and stop at the final value. It will
 *       remain at this final value until a new active edge occurs on the SYNC input, and then the waveform will be
 *       repeated.
 */
#define DAC_CTRL0_ONESHOT_MASK                   (0x8000U)
#define DAC_CTRL0_ONESHOT_SHIFT                  (15U)
#define DAC_CTRL0_ONESHOT(x)                     (((uint16_t)(((uint16_t)(x)) << DAC_CTRL0_ONESHOT_SHIFT)) & DAC_CTRL0_ONESHOT_MASK)
/*! @} */

/*! @name DATAREG_FMT0 - Buffered Data Register */
/*! @{ */
#define DAC_DATAREG_FMT0_DATA_MASK               (0xFFFU)
#define DAC_DATAREG_FMT0_DATA_SHIFT              (0U)
#define DAC_DATAREG_FMT0_DATA(x)                 (((uint16_t)(((uint16_t)(x)) << DAC_DATAREG_FMT0_DATA_SHIFT)) & DAC_DATAREG_FMT0_DATA_MASK)
/*! @} */

/*! @name DATAREG_FMT1 - Buffered Data Register */
/*! @{ */
#define DAC_DATAREG_FMT1_DATA_MASK               (0xFFF0U)
#define DAC_DATAREG_FMT1_DATA_SHIFT              (4U)
#define DAC_DATAREG_FMT1_DATA(x)                 (((uint16_t)(((uint16_t)(x)) << DAC_DATAREG_FMT1_DATA_SHIFT)) & DAC_DATAREG_FMT1_DATA_MASK)
/*! @} */

/*! @name STEPVAL_FMT0 - Step Size Register */
/*! @{ */
#define DAC_STEPVAL_FMT0_STEP_MASK               (0xFFFU)
#define DAC_STEPVAL_FMT0_STEP_SHIFT              (0U)
#define DAC_STEPVAL_FMT0_STEP(x)                 (((uint16_t)(((uint16_t)(x)) << DAC_STEPVAL_FMT0_STEP_SHIFT)) & DAC_STEPVAL_FMT0_STEP_MASK)
/*! @} */

/*! @name STEPVAL_FMT1 - Step Size Register */
/*! @{ */
#define DAC_STEPVAL_FMT1_STEP_MASK               (0xFFF0U)
#define DAC_STEPVAL_FMT1_STEP_SHIFT              (4U)
#define DAC_STEPVAL_FMT1_STEP(x)                 (((uint16_t)(((uint16_t)(x)) << DAC_STEPVAL_FMT1_STEP_SHIFT)) & DAC_STEPVAL_FMT1_STEP_MASK)
/*! @} */

/*! @name MINVAL_FMT0 - Minimum Value Register */
/*! @{ */
#define DAC_MINVAL_FMT0_MINVAL_MASK              (0xFFFU)
#define DAC_MINVAL_FMT0_MINVAL_SHIFT             (0U)
#define DAC_MINVAL_FMT0_MINVAL(x)                (((uint16_t)(((uint16_t)(x)) << DAC_MINVAL_FMT0_MINVAL_SHIFT)) & DAC_MINVAL_FMT0_MINVAL_MASK)
/*! @} */

/*! @name MINVAL_FMT1 - Minimum Value Register */
/*! @{ */
#define DAC_MINVAL_FMT1_MINVAL_MASK              (0xFFF0U)
#define DAC_MINVAL_FMT1_MINVAL_SHIFT             (4U)
#define DAC_MINVAL_FMT1_MINVAL(x)                (((uint16_t)(((uint16_t)(x)) << DAC_MINVAL_FMT1_MINVAL_SHIFT)) & DAC_MINVAL_FMT1_MINVAL_MASK)
/*! @} */

/*! @name MAXVAL_FMT0 - Maximum Value Register */
/*! @{ */
#define DAC_MAXVAL_FMT0_MAXVAL_MASK              (0xFFFU)
#define DAC_MAXVAL_FMT0_MAXVAL_SHIFT             (0U)
#define DAC_MAXVAL_FMT0_MAXVAL(x)                (((uint16_t)(((uint16_t)(x)) << DAC_MAXVAL_FMT0_MAXVAL_SHIFT)) & DAC_MAXVAL_FMT0_MAXVAL_MASK)
/*! @} */

/*! @name MAXVAL_FMT1 - Maximum Value Register */
/*! @{ */
#define DAC_MAXVAL_FMT1_MAXVAL_MASK              (0xFFF0U)
#define DAC_MAXVAL_FMT1_MAXVAL_SHIFT             (4U)
#define DAC_MAXVAL_FMT1_MAXVAL(x)                (((uint16_t)(((uint16_t)(x)) << DAC_MAXVAL_FMT1_MAXVAL_SHIFT)) & DAC_MAXVAL_FMT1_MAXVAL_MASK)
/*! @} */

/*! @name STATUS - Status Register */
/*! @{ */
/*! EMPTY - Indicates that the FIFO is empty
 *  0b0..FIFO is not empty
 *  0b1..FIFO is empty (on reset)
 */
#define DAC_STATUS_EMPTY_MASK                    (0x1U)
#define DAC_STATUS_EMPTY_SHIFT                   (0U)
#define DAC_STATUS_EMPTY(x)                      (((uint16_t)(((uint16_t)(x)) << DAC_STATUS_EMPTY_SHIFT)) & DAC_STATUS_EMPTY_MASK)
/*! FULL - Indicates that the FIFO is full
 *  0b0..FIFO is not full (on reset).
 *  0b1..FIFO is full.
 */
#define DAC_STATUS_FULL_MASK                     (0x2U)
#define DAC_STATUS_FULL_SHIFT                    (1U)
#define DAC_STATUS_FULL(x)                       (((uint16_t)(((uint16_t)(x)) << DAC_STATUS_FULL_SHIFT)) & DAC_STATUS_FULL_MASK)
/*! @} */

/*! @name CTRL1 - Control Register 1 */
/*! @{ */
#define DAC_CTRL1_FILT_CNT_MASK                  (0x3FU)
#define DAC_CTRL1_FILT_CNT_SHIFT                 (0U)
#define DAC_CTRL1_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << DAC_CTRL1_FILT_CNT_SHIFT)) & DAC_CTRL1_FILT_CNT_MASK)
/*! @} */

/*! @name COMPARE - Compare Register */
/*! @{ */
#define DAC_COMPARE_COMPARE_MASK                 (0xFFFFU)
#define DAC_COMPARE_COMPARE_SHIFT                (0U)
#define DAC_COMPARE_COMPARE(x)                   (((uint16_t)(((uint16_t)(x)) << DAC_COMPARE_COMPARE_SHIFT)) & DAC_COMPARE_COMPARE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DACA base address */
#define DACA_BASE                                (0xE000u)
/** Peripheral DACA base pointer */
#define DACA                                     ((DAC_Type *)DACA_BASE)
/** Peripheral DACB base address */
#define DACB_BASE                                (0xE010u)
/** Peripheral DACB base pointer */
#define DACB                                     ((DAC_Type *)DACB_BASE)
/** Array initializer of DAC peripheral base addresses */
#define DAC_BASE_ADDRS                           { DACA_BASE, DACB_BASE }
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DACA, DACB }

/*!
 * @}
 */ /* end of group DAC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< Control Register, offset: 0x0 */
  __I  uint32_t ES;                                /**< Error Status Register, offset: 0x4 */
       uint8_t RESERVED_0[4];
  __IO uint32_t ERQ;                               /**< Enable Request Register, offset: 0xC */
       uint8_t RESERVED_1[4];
  __IO uint32_t EEI;                               /**< Enable Error Interrupt Register, offset: 0x14 */
  __O  uint8_t CEEI;                               /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  __O  uint8_t SEEI;                               /**< Set Enable Error Interrupt Register, offset: 0x19 */
  __O  uint8_t CERQ;                               /**< Clear Enable Request Register, offset: 0x1A */
  __O  uint8_t SERQ;                               /**< Set Enable Request Register, offset: 0x1B */
  __O  uint8_t CDNE;                               /**< Clear DONE Status Bit Register, offset: 0x1C */
  __O  uint8_t SSRT;                               /**< Set START Bit Register, offset: 0x1D */
  __O  uint8_t CERR;                               /**< Clear Error Register, offset: 0x1E */
  __O  uint8_t CINT;                               /**< Clear Interrupt Request Register, offset: 0x1F */
       uint8_t RESERVED_2[4];
  __IO uint32_t INT;                               /**< Interrupt Request Register, offset: 0x24 */
       uint8_t RESERVED_3[4];
  __IO uint32_t ERR;                               /**< Error Register, offset: 0x2C */
       uint8_t RESERVED_4[4];
  __I  uint32_t HRS;                               /**< Hardware Request Status Register, offset: 0x34 */
       uint8_t RESERVED_5[12];
  __IO uint32_t EARS;                              /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
       uint8_t RESERVED_6[184];
  __IO uint8_t DCHPRI[4];                          /**< Channel Priority Register, array offset: 0x100, array step: 0x1 */
       uint8_t RESERVED_7[3836];
  struct {                                         /* offset: 0x1000, array step: 0x20 */
    __IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    __IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    __IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLNO;                       /**< TCD Minor Byte Count (Minor Loop Mapping Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    };
    __IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    __IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    __IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    };
    __IO uint32_t DLASTSGA;                          /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    __IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    };
  } TCD[4];
} DMA_Type;

/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/*! @name CR - Control Register */
/*! @{ */
/*! EBWR - Enable Buffered Writes
 *  0b0..Buffered writes are disabled.
 *  0b1..Buffered writes are enabled.
 */
#define DMA_CR_EBWR_MASK                         (0x1U)
#define DMA_CR_EBWR_SHIFT                        (0U)
#define DMA_CR_EBWR(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EBWR_SHIFT)) & DMA_CR_EBWR_MASK)
/*! EDBG - Enable Debug
 *  0b0..When in debug mode, the DMA continues to operate.
 *  0b1..When in debug mode, the DMA stalls the start of a new channel. Executing channels are allowed to
 *       complete. Channel execution resumes when the system exits debug mode or the EDBG bit is cleared.
 */
#define DMA_CR_EDBG_MASK                         (0x2U)
#define DMA_CR_EDBG_SHIFT                        (1U)
#define DMA_CR_EDBG(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EDBG_SHIFT)) & DMA_CR_EDBG_MASK)
/*! ERCA - Enable Round Robin Channel Arbitration
 *  0b0..Fixed priority arbitration is used for channel selection .
 *  0b1..Round robin arbitration is used for channel selection .
 */
#define DMA_CR_ERCA_MASK                         (0x4U)
#define DMA_CR_ERCA_SHIFT                        (2U)
#define DMA_CR_ERCA(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_ERCA_SHIFT)) & DMA_CR_ERCA_MASK)
/*! HOE - Halt On Error
 *  0b0..Normal operation
 *  0b1..Any error causes the HALT bit to set. Subsequently, all service requests are ignored until the HALT bit is cleared.
 */
#define DMA_CR_HOE_MASK                          (0x10U)
#define DMA_CR_HOE_SHIFT                         (4U)
#define DMA_CR_HOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_HOE_SHIFT)) & DMA_CR_HOE_MASK)
/*! HALT - Halt DMA Operations
 *  0b0..Normal operation
 *  0b1..Stall the start of any new channels. Executing channels are allowed to complete. Channel execution resumes when this bit is cleared.
 */
#define DMA_CR_HALT_MASK                         (0x20U)
#define DMA_CR_HALT_SHIFT                        (5U)
#define DMA_CR_HALT(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_HALT_SHIFT)) & DMA_CR_HALT_MASK)
/*! CLM - Continuous Link Mode
 *  0b0..A minor loop channel link made to itself goes through channel arbitration before being activated again.
 *  0b1..A minor loop channel link made to itself does not go through channel arbitration before being activated
 *       again. Upon minor loop completion, the channel activates again if that channel has a minor loop channel
 *       link enabled and the link channel is itself. This effectively applies the minor loop offsets and restarts the
 *       next minor loop.
 */
#define DMA_CR_CLM_MASK                          (0x40U)
#define DMA_CR_CLM_SHIFT                         (6U)
#define DMA_CR_CLM(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_CLM_SHIFT)) & DMA_CR_CLM_MASK)
/*! EMLM - Enable Minor Loop Mapping
 *  0b0..Disabled. TCDn.word2 is defined as a 32-bit NBYTES field.
 *  0b1..Enabled. TCDn.word2 is redefined to include individual enable fields, an offset field, and the NBYTES
 *       field. The individual enable fields allow the minor loop offset to be applied to the source address, the
 *       destination address, or both. The NBYTES field is reduced when either offset is enabled.
 */
#define DMA_CR_EMLM_MASK                         (0x80U)
#define DMA_CR_EMLM_SHIFT                        (7U)
#define DMA_CR_EMLM(x)                           (((uint32_t)(((uint32_t)(x)) << DMA_CR_EMLM_SHIFT)) & DMA_CR_EMLM_MASK)
/*! ECX - Error Cancel Transfer
 *  0b0..Normal operation
 *  0b1..Cancel the remaining data transfer in the same fashion as the CX bit. Stop the executing channel and
 *       force the minor loop to finish. The cancel takes effect after the last write of the current read/write
 *       sequence. The ECX bit clears itself after the cancel is honored. In addition to cancelling the transfer, ECX
 *       treats the cancel as an error condition, thus updating the Error Status register (DMAx_ES) and generating an
 *       optional error interrupt.
 */
#define DMA_CR_ECX_MASK                          (0x10000U)
#define DMA_CR_ECX_SHIFT                         (16U)
#define DMA_CR_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_CR_ECX_SHIFT)) & DMA_CR_ECX_MASK)
/*! CX - Cancel Transfer
 *  0b0..Normal operation
 *  0b1..Cancel the remaining data transfer. Stop the executing channel and force the minor loop to finish. The
 *       cancel takes effect after the last write of the current read/write sequence. The CX bit clears itself after
 *       the cancel has been honored. This cancel retires the channel normally as if the minor loop was completed.
 */
#define DMA_CR_CX_MASK                           (0x20000U)
#define DMA_CR_CX_SHIFT                          (17U)
#define DMA_CR_CX(x)                             (((uint32_t)(((uint32_t)(x)) << DMA_CR_CX_SHIFT)) & DMA_CR_CX_MASK)
/*! ACTIVE - DMA Active Status
 *  0b0..eDMA is idle.
 *  0b1..eDMA is executing a channel.
 */
#define DMA_CR_ACTIVE_MASK                       (0x80000000U)
#define DMA_CR_ACTIVE_SHIFT                      (31U)
#define DMA_CR_ACTIVE(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_CR_ACTIVE_SHIFT)) & DMA_CR_ACTIVE_MASK)
/*! @} */

/*! @name ES - Error Status Register */
/*! @{ */
/*! DBE - Destination Bus Error
 *  0b0..No destination bus error
 *  0b1..The last recorded error was a bus error on a destination write
 */
#define DMA_ES_DBE_MASK                          (0x1U)
#define DMA_ES_DBE_SHIFT                         (0U)
#define DMA_ES_DBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DBE_SHIFT)) & DMA_ES_DBE_MASK)
/*! SBE - Source Bus Error
 *  0b0..No source bus error
 *  0b1..The last recorded error was a bus error on a source read
 */
#define DMA_ES_SBE_MASK                          (0x2U)
#define DMA_ES_SBE_SHIFT                         (1U)
#define DMA_ES_SBE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SBE_SHIFT)) & DMA_ES_SBE_MASK)
/*! SGE - Scatter/Gather Configuration Error
 *  0b0..No scatter/gather configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DLASTSGA field. This field is
 *       checked at the beginning of a scatter/gather operation after major loop completion if TCDn_CSR[ESG] is
 *       enabled. TCDn_DLASTSGA is not on a 32 byte boundary.
 */
#define DMA_ES_SGE_MASK                          (0x4U)
#define DMA_ES_SGE_SHIFT                         (2U)
#define DMA_ES_SGE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SGE_SHIFT)) & DMA_ES_SGE_MASK)
/*! NCE - NBYTES/CITER Configuration Error
 *  0b0..No NBYTES/CITER configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_NBYTES or TCDn_CITER fields.
 *       TCDn_NBYTES is not a multiple of TCDn_ATTR[SSIZE] and TCDn_ATTR[DSIZE], or TCDn_CITER[CITER] is equal to zero,
 *       or TCDn_CITER[ELINK] is not equal to TCDn_BITER[ELINK]
 */
#define DMA_ES_NCE_MASK                          (0x8U)
#define DMA_ES_NCE_SHIFT                         (3U)
#define DMA_ES_NCE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_NCE_SHIFT)) & DMA_ES_NCE_MASK)
/*! DOE - Destination Offset Error
 *  0b0..No destination offset configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DOFF field. TCDn_DOFF is inconsistent with TCDn_ATTR[DSIZE].
 */
#define DMA_ES_DOE_MASK                          (0x10U)
#define DMA_ES_DOE_SHIFT                         (4U)
#define DMA_ES_DOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DOE_SHIFT)) & DMA_ES_DOE_MASK)
/*! DAE - Destination Address Error
 *  0b0..No destination address configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_DADDR field. TCDn_DADDR is inconsistent with TCDn_ATTR[DSIZE].
 */
#define DMA_ES_DAE_MASK                          (0x20U)
#define DMA_ES_DAE_SHIFT                         (5U)
#define DMA_ES_DAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_DAE_SHIFT)) & DMA_ES_DAE_MASK)
/*! SOE - Source Offset Error
 *  0b0..No source offset configuration error
 *  0b1..The last recorded error was a configuration error detected in the TCDn_SOFF field. TCDn_SOFF is inconsistent with TCDn_ATTR[SSIZE].
 */
#define DMA_ES_SOE_MASK                          (0x40U)
#define DMA_ES_SOE_SHIFT                         (6U)
#define DMA_ES_SOE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SOE_SHIFT)) & DMA_ES_SOE_MASK)
/*! SAE - Source Address Error
 *  0b0..No source address configuration error.
 *  0b1..The last recorded error was a configuration error detected in the TCDn_SADDR field. TCDn_SADDR is inconsistent with TCDn_ATTR[SSIZE].
 */
#define DMA_ES_SAE_MASK                          (0x80U)
#define DMA_ES_SAE_SHIFT                         (7U)
#define DMA_ES_SAE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_SAE_SHIFT)) & DMA_ES_SAE_MASK)
#define DMA_ES_ERRCHN_MASK                       (0x300U)
#define DMA_ES_ERRCHN_SHIFT                      (8U)
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x)) << DMA_ES_ERRCHN_SHIFT)) & DMA_ES_ERRCHN_MASK)
/*! CPE - Channel Priority Error
 *  0b0..No channel priority error
 *  0b1..The last recorded error was a configuration error in the channel priorities . Channel priorities are not unique.
 */
#define DMA_ES_CPE_MASK                          (0x4000U)
#define DMA_ES_CPE_SHIFT                         (14U)
#define DMA_ES_CPE(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_CPE_SHIFT)) & DMA_ES_CPE_MASK)
/*! ECX - Transfer Canceled
 *  0b0..No canceled transfers
 *  0b1..The last recorded entry was a canceled transfer by the error cancel transfer input
 */
#define DMA_ES_ECX_MASK                          (0x10000U)
#define DMA_ES_ECX_SHIFT                         (16U)
#define DMA_ES_ECX(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_ECX_SHIFT)) & DMA_ES_ECX_MASK)
/*! VLD - VLD
 *  0b0..No ERR bits are set.
 *  0b1..At least one ERR bit is set indicating a valid error exists that has not been cleared.
 */
#define DMA_ES_VLD_MASK                          (0x80000000U)
#define DMA_ES_VLD_SHIFT                         (31U)
#define DMA_ES_VLD(x)                            (((uint32_t)(((uint32_t)(x)) << DMA_ES_VLD_SHIFT)) & DMA_ES_VLD_MASK)
/*! @} */

/*! @name ERQ - Enable Request Register */
/*! @{ */
/*! ERQ0 - Enable DMA Request 0
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ0_MASK                        (0x1U)
#define DMA_ERQ_ERQ0_SHIFT                       (0U)
#define DMA_ERQ_ERQ0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ0_SHIFT)) & DMA_ERQ_ERQ0_MASK)
/*! ERQ1 - Enable DMA Request 1
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ1_MASK                        (0x2U)
#define DMA_ERQ_ERQ1_SHIFT                       (1U)
#define DMA_ERQ_ERQ1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ1_SHIFT)) & DMA_ERQ_ERQ1_MASK)
/*! ERQ2 - Enable DMA Request 2
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ2_MASK                        (0x4U)
#define DMA_ERQ_ERQ2_SHIFT                       (2U)
#define DMA_ERQ_ERQ2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ2_SHIFT)) & DMA_ERQ_ERQ2_MASK)
/*! ERQ3 - Enable DMA Request 3
 *  0b0..The DMA request signal for the corresponding channel is disabled
 *  0b1..The DMA request signal for the corresponding channel is enabled
 */
#define DMA_ERQ_ERQ3_MASK                        (0x8U)
#define DMA_ERQ_ERQ3_SHIFT                       (3U)
#define DMA_ERQ_ERQ3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERQ_ERQ3_SHIFT)) & DMA_ERQ_ERQ3_MASK)
/*! @} */

/*! @name EEI - Enable Error Interrupt Register */
/*! @{ */
/*! EEI0 - Enable Error Interrupt 0
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI0_MASK                        (0x1U)
#define DMA_EEI_EEI0_SHIFT                       (0U)
#define DMA_EEI_EEI0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI0_SHIFT)) & DMA_EEI_EEI0_MASK)
/*! EEI1 - Enable Error Interrupt 1
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI1_MASK                        (0x2U)
#define DMA_EEI_EEI1_SHIFT                       (1U)
#define DMA_EEI_EEI1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI1_SHIFT)) & DMA_EEI_EEI1_MASK)
/*! EEI2 - Enable Error Interrupt 2
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI2_MASK                        (0x4U)
#define DMA_EEI_EEI2_SHIFT                       (2U)
#define DMA_EEI_EEI2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI2_SHIFT)) & DMA_EEI_EEI2_MASK)
/*! EEI3 - Enable Error Interrupt 3
 *  0b0..The error signal for corresponding channel does not generate an error interrupt
 *  0b1..The assertion of the error signal for corresponding channel generates an error interrupt request
 */
#define DMA_EEI_EEI3_MASK                        (0x8U)
#define DMA_EEI_EEI3_SHIFT                       (3U)
#define DMA_EEI_EEI3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_EEI_EEI3_SHIFT)) & DMA_EEI_EEI3_MASK)
/*! @} */

/*! @name CEEI - Clear Enable Error Interrupt Register */
/*! @{ */
#define DMA_CEEI_CEEI_MASK                       (0x3U)
#define DMA_CEEI_CEEI_SHIFT                      (0U)
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CEEI_SHIFT)) & DMA_CEEI_CEEI_MASK)
/*! CAEE - Clear All Enable Error Interrupts
 *  0b0..Clear only the EEI bit specified in the CEEI field
 *  0b1..Clear all bits in EEI
 */
#define DMA_CEEI_CAEE_MASK                       (0x40U)
#define DMA_CEEI_CAEE_SHIFT                      (6U)
#define DMA_CEEI_CAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_CAEE_SHIFT)) & DMA_CEEI_CAEE_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CEEI_NOP_MASK                        (0x80U)
#define DMA_CEEI_NOP_SHIFT                       (7U)
#define DMA_CEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CEEI_NOP_SHIFT)) & DMA_CEEI_NOP_MASK)
/*! @} */

/*! @name SEEI - Set Enable Error Interrupt Register */
/*! @{ */
#define DMA_SEEI_SEEI_MASK                       (0x3U)
#define DMA_SEEI_SEEI_SHIFT                      (0U)
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SEEI_SHIFT)) & DMA_SEEI_SEEI_MASK)
/*! SAEE - Sets All Enable Error Interrupts
 *  0b0..Set only the EEI bit specified in the SEEI field.
 *  0b1..Sets all bits in EEI
 */
#define DMA_SEEI_SAEE_MASK                       (0x40U)
#define DMA_SEEI_SAEE_SHIFT                      (6U)
#define DMA_SEEI_SAEE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_SAEE_SHIFT)) & DMA_SEEI_SAEE_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SEEI_NOP_MASK                        (0x80U)
#define DMA_SEEI_NOP_SHIFT                       (7U)
#define DMA_SEEI_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SEEI_NOP_SHIFT)) & DMA_SEEI_NOP_MASK)
/*! @} */

/*! @name CERQ - Clear Enable Request Register */
/*! @{ */
#define DMA_CERQ_CERQ_MASK                       (0x3U)
#define DMA_CERQ_CERQ_SHIFT                      (0U)
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CERQ_SHIFT)) & DMA_CERQ_CERQ_MASK)
/*! CAER - Clear All Enable Requests
 *  0b0..Clear only the ERQ bit specified in the CERQ field
 *  0b1..Clear all bits in ERQ
 */
#define DMA_CERQ_CAER_MASK                       (0x40U)
#define DMA_CERQ_CAER_SHIFT                      (6U)
#define DMA_CERQ_CAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_CAER_SHIFT)) & DMA_CERQ_CAER_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CERQ_NOP_MASK                        (0x80U)
#define DMA_CERQ_NOP_SHIFT                       (7U)
#define DMA_CERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERQ_NOP_SHIFT)) & DMA_CERQ_NOP_MASK)
/*! @} */

/*! @name SERQ - Set Enable Request Register */
/*! @{ */
#define DMA_SERQ_SERQ_MASK                       (0x3U)
#define DMA_SERQ_SERQ_SHIFT                      (0U)
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SERQ_SHIFT)) & DMA_SERQ_SERQ_MASK)
/*! SAER - Set All Enable Requests
 *  0b0..Set only the ERQ bit specified in the SERQ field
 *  0b1..Set all bits in ERQ
 */
#define DMA_SERQ_SAER_MASK                       (0x40U)
#define DMA_SERQ_SAER_SHIFT                      (6U)
#define DMA_SERQ_SAER(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_SAER_SHIFT)) & DMA_SERQ_SAER_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SERQ_NOP_MASK                        (0x80U)
#define DMA_SERQ_NOP_SHIFT                       (7U)
#define DMA_SERQ_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SERQ_NOP_SHIFT)) & DMA_SERQ_NOP_MASK)
/*! @} */

/*! @name CDNE - Clear DONE Status Bit Register */
/*! @{ */
#define DMA_CDNE_CDNE_MASK                       (0x3U)
#define DMA_CDNE_CDNE_SHIFT                      (0U)
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CDNE_SHIFT)) & DMA_CDNE_CDNE_MASK)
/*! CADN - Clears All DONE Bits
 *  0b0..Clears only the TCDn_CSR[DONE] bit specified in the CDNE field
 *  0b1..Clears all bits in TCDn_CSR[DONE]
 */
#define DMA_CDNE_CADN_MASK                       (0x40U)
#define DMA_CDNE_CADN_SHIFT                      (6U)
#define DMA_CDNE_CADN(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_CADN_SHIFT)) & DMA_CDNE_CADN_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CDNE_NOP_MASK                        (0x80U)
#define DMA_CDNE_NOP_SHIFT                       (7U)
#define DMA_CDNE_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CDNE_NOP_SHIFT)) & DMA_CDNE_NOP_MASK)
/*! @} */

/*! @name SSRT - Set START Bit Register */
/*! @{ */
#define DMA_SSRT_SSRT_MASK                       (0x3U)
#define DMA_SSRT_SSRT_SHIFT                      (0U)
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SSRT_SHIFT)) & DMA_SSRT_SSRT_MASK)
/*! SAST - Set All START Bits (activates all channels)
 *  0b0..Set only the TCDn_CSR[START] bit specified in the SSRT field
 *  0b1..Set all bits in TCDn_CSR[START]
 */
#define DMA_SSRT_SAST_MASK                       (0x40U)
#define DMA_SSRT_SAST_SHIFT                      (6U)
#define DMA_SSRT_SAST(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_SAST_SHIFT)) & DMA_SSRT_SAST_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_SSRT_NOP_MASK                        (0x80U)
#define DMA_SSRT_NOP_SHIFT                       (7U)
#define DMA_SSRT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_SSRT_NOP_SHIFT)) & DMA_SSRT_NOP_MASK)
/*! @} */

/*! @name CERR - Clear Error Register */
/*! @{ */
#define DMA_CERR_CERR_MASK                       (0x3U)
#define DMA_CERR_CERR_SHIFT                      (0U)
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CERR_SHIFT)) & DMA_CERR_CERR_MASK)
/*! CAEI - Clear All Error Indicators
 *  0b0..Clear only the ERR bit specified in the CERR field
 *  0b1..Clear all bits in ERR
 */
#define DMA_CERR_CAEI_MASK                       (0x40U)
#define DMA_CERR_CAEI_SHIFT                      (6U)
#define DMA_CERR_CAEI(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CERR_CAEI_SHIFT)) & DMA_CERR_CAEI_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CERR_NOP_MASK                        (0x80U)
#define DMA_CERR_NOP_SHIFT                       (7U)
#define DMA_CERR_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CERR_NOP_SHIFT)) & DMA_CERR_NOP_MASK)
/*! @} */

/*! @name CINT - Clear Interrupt Request Register */
/*! @{ */
#define DMA_CINT_CINT_MASK                       (0x3U)
#define DMA_CINT_CINT_SHIFT                      (0U)
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CINT_SHIFT)) & DMA_CINT_CINT_MASK)
/*! CAIR - Clear All Interrupt Requests
 *  0b0..Clear only the INT bit specified in the CINT field
 *  0b1..Clear all bits in INT
 */
#define DMA_CINT_CAIR_MASK                       (0x40U)
#define DMA_CINT_CAIR_SHIFT                      (6U)
#define DMA_CINT_CAIR(x)                         (((uint8_t)(((uint8_t)(x)) << DMA_CINT_CAIR_SHIFT)) & DMA_CINT_CAIR_MASK)
/*! NOP - No Op enable
 *  0b0..Normal operation
 *  0b1..No operation, ignore the other bits in this register
 */
#define DMA_CINT_NOP_MASK                        (0x80U)
#define DMA_CINT_NOP_SHIFT                       (7U)
#define DMA_CINT_NOP(x)                          (((uint8_t)(((uint8_t)(x)) << DMA_CINT_NOP_SHIFT)) & DMA_CINT_NOP_MASK)
/*! @} */

/*! @name INT - Interrupt Request Register */
/*! @{ */
/*! INT0 - Interrupt Request 0
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT0_MASK                        (0x1U)
#define DMA_INT_INT0_SHIFT                       (0U)
#define DMA_INT_INT0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT0_SHIFT)) & DMA_INT_INT0_MASK)
/*! INT1 - Interrupt Request 1
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT1_MASK                        (0x2U)
#define DMA_INT_INT1_SHIFT                       (1U)
#define DMA_INT_INT1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT1_SHIFT)) & DMA_INT_INT1_MASK)
/*! INT2 - Interrupt Request 2
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT2_MASK                        (0x4U)
#define DMA_INT_INT2_SHIFT                       (2U)
#define DMA_INT_INT2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT2_SHIFT)) & DMA_INT_INT2_MASK)
/*! INT3 - Interrupt Request 3
 *  0b0..The interrupt request for corresponding channel is cleared
 *  0b1..The interrupt request for corresponding channel is active
 */
#define DMA_INT_INT3_MASK                        (0x8U)
#define DMA_INT_INT3_SHIFT                       (3U)
#define DMA_INT_INT3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_INT_INT3_SHIFT)) & DMA_INT_INT3_MASK)
/*! @} */

/*! @name ERR - Error Register */
/*! @{ */
/*! ERR0 - Error In Channel 0
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR0_MASK                        (0x1U)
#define DMA_ERR_ERR0_SHIFT                       (0U)
#define DMA_ERR_ERR0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR0_SHIFT)) & DMA_ERR_ERR0_MASK)
/*! ERR1 - Error In Channel 1
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR1_MASK                        (0x2U)
#define DMA_ERR_ERR1_SHIFT                       (1U)
#define DMA_ERR_ERR1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR1_SHIFT)) & DMA_ERR_ERR1_MASK)
/*! ERR2 - Error In Channel 2
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR2_MASK                        (0x4U)
#define DMA_ERR_ERR2_SHIFT                       (2U)
#define DMA_ERR_ERR2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR2_SHIFT)) & DMA_ERR_ERR2_MASK)
/*! ERR3 - Error In Channel 3
 *  0b0..An error in this channel has not occurred
 *  0b1..An error in this channel has occurred
 */
#define DMA_ERR_ERR3_MASK                        (0x8U)
#define DMA_ERR_ERR3_SHIFT                       (3U)
#define DMA_ERR_ERR3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_ERR_ERR3_SHIFT)) & DMA_ERR_ERR3_MASK)
/*! @} */

/*! @name HRS - Hardware Request Status Register */
/*! @{ */
/*! HRS0 - Hardware Request Status Channel 0
 *  0b0..A hardware service request for channel 0 is not present
 *  0b1..A hardware service request for channel 0 is present
 */
#define DMA_HRS_HRS0_MASK                        (0x1U)
#define DMA_HRS_HRS0_SHIFT                       (0U)
#define DMA_HRS_HRS0(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS0_SHIFT)) & DMA_HRS_HRS0_MASK)
/*! HRS1 - Hardware Request Status Channel 1
 *  0b0..A hardware service request for channel 1 is not present
 *  0b1..A hardware service request for channel 1 is present
 */
#define DMA_HRS_HRS1_MASK                        (0x2U)
#define DMA_HRS_HRS1_SHIFT                       (1U)
#define DMA_HRS_HRS1(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS1_SHIFT)) & DMA_HRS_HRS1_MASK)
/*! HRS2 - Hardware Request Status Channel 2
 *  0b0..A hardware service request for channel 2 is not present
 *  0b1..A hardware service request for channel 2 is present
 */
#define DMA_HRS_HRS2_MASK                        (0x4U)
#define DMA_HRS_HRS2_SHIFT                       (2U)
#define DMA_HRS_HRS2(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS2_SHIFT)) & DMA_HRS_HRS2_MASK)
/*! HRS3 - Hardware Request Status Channel 3
 *  0b0..A hardware service request for channel 3 is not present
 *  0b1..A hardware service request for channel 3 is present
 */
#define DMA_HRS_HRS3_MASK                        (0x8U)
#define DMA_HRS_HRS3_SHIFT                       (3U)
#define DMA_HRS_HRS3(x)                          (((uint32_t)(((uint32_t)(x)) << DMA_HRS_HRS3_SHIFT)) & DMA_HRS_HRS3_MASK)
/*! @} */

/*! @name EARS - Enable Asynchronous Request in Stop Register */
/*! @{ */
/*! EDREQ_0 - Enable asynchronous DMA request in stop mode for channel 0.
 *  0b0..Disable asynchronous DMA request for channel 0.
 *  0b1..Enable asynchronous DMA request for channel 0.
 */
#define DMA_EARS_EDREQ_0_MASK                    (0x1U)
#define DMA_EARS_EDREQ_0_SHIFT                   (0U)
#define DMA_EARS_EDREQ_0(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_0_SHIFT)) & DMA_EARS_EDREQ_0_MASK)
/*! EDREQ_1 - Enable asynchronous DMA request in stop mode for channel 1.
 *  0b0..Disable asynchronous DMA request for channel 1
 *  0b1..Enable asynchronous DMA request for channel 1.
 */
#define DMA_EARS_EDREQ_1_MASK                    (0x2U)
#define DMA_EARS_EDREQ_1_SHIFT                   (1U)
#define DMA_EARS_EDREQ_1(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_1_SHIFT)) & DMA_EARS_EDREQ_1_MASK)
/*! EDREQ_2 - Enable asynchronous DMA request in stop mode for channel 2.
 *  0b0..Disable asynchronous DMA request for channel 2.
 *  0b1..Enable asynchronous DMA request for channel 2.
 */
#define DMA_EARS_EDREQ_2_MASK                    (0x4U)
#define DMA_EARS_EDREQ_2_SHIFT                   (2U)
#define DMA_EARS_EDREQ_2(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_2_SHIFT)) & DMA_EARS_EDREQ_2_MASK)
/*! EDREQ_3 - Enable asynchronous DMA request in stop mode for channel 3.
 *  0b0..Disable asynchronous DMA request for channel 3.
 *  0b1..Enable asynchronous DMA request for channel 3.
 */
#define DMA_EARS_EDREQ_3_MASK                    (0x8U)
#define DMA_EARS_EDREQ_3_SHIFT                   (3U)
#define DMA_EARS_EDREQ_3(x)                      (((uint32_t)(((uint32_t)(x)) << DMA_EARS_EDREQ_3_SHIFT)) & DMA_EARS_EDREQ_3_MASK)
/*! @} */

/*! @name DCHPRI - Channel Priority Register */
/*! @{ */
#define DMA_DCHPRI_CHPRI_MASK                    (0x3U)
#define DMA_DCHPRI_CHPRI_SHIFT                   (0U)
#define DMA_DCHPRI_CHPRI(x)                      (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_CHPRI_SHIFT)) & DMA_DCHPRI_CHPRI_MASK)
/*! DPA - Disable Preempt Ability. This field resets to 0.
 *  0b0..Channel n can suspend a lower priority channel.
 *  0b1..Channel n cannot suspend any channel, regardless of channel priority.
 */
#define DMA_DCHPRI_DPA_MASK                      (0x40U)
#define DMA_DCHPRI_DPA_SHIFT                     (6U)
#define DMA_DCHPRI_DPA(x)                        (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_DPA_SHIFT)) & DMA_DCHPRI_DPA_MASK)
/*! ECP - Enable Channel Preemption. This field resets to 0.
 *  0b0..Channel n cannot be suspended by a higher priority channel's service request.
 *  0b1..Channel n can be temporarily suspended by the service request of a higher priority channel.
 */
#define DMA_DCHPRI_ECP_MASK                      (0x80U)
#define DMA_DCHPRI_ECP_SHIFT                     (7U)
#define DMA_DCHPRI_ECP(x)                        (((uint8_t)(((uint8_t)(x)) << DMA_DCHPRI_ECP_SHIFT)) & DMA_DCHPRI_ECP_MASK)
/*! @} */

/* The count of DMA_DCHPRI */
#define DMA_DCHPRI_COUNT                         (4U)

/*! @name SADDR - TCD Source Address */
/*! @{ */
#define DMA_SADDR_SADDR_MASK                     (0xFFFFFFFFU)
#define DMA_SADDR_SADDR_SHIFT                    (0U)
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SADDR_SADDR_SHIFT)) & DMA_SADDR_SADDR_MASK)
/*! @} */

/* The count of DMA_SADDR */
#define DMA_SADDR_COUNT                          (4U)

/*! @name SOFF - TCD Signed Source Address Offset */
/*! @{ */
#define DMA_SOFF_SOFF_MASK                       (0xFFFFU)
#define DMA_SOFF_SOFF_SHIFT                      (0U)
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_SOFF_SOFF_SHIFT)) & DMA_SOFF_SOFF_MASK)
/*! @} */

/* The count of DMA_SOFF */
#define DMA_SOFF_COUNT                           (4U)

/*! @name ATTR - TCD Transfer Attributes */
/*! @{ */
#define DMA_ATTR_DSIZE_MASK                      (0x7U)
#define DMA_ATTR_DSIZE_SHIFT                     (0U)
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DSIZE_SHIFT)) & DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       (0xF8U)
#define DMA_ATTR_DMOD_SHIFT                      (3U)
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_DMOD_SHIFT)) & DMA_ATTR_DMOD_MASK)
/*! SSIZE - Source data transfer size
 *  0b000..8-bit
 *  0b001..16-bit
 *  0b010..32-bit
 *  0b011..Reserved
 *  0b100..16-byte
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define DMA_ATTR_SSIZE_MASK                      (0x700U)
#define DMA_ATTR_SSIZE_SHIFT                     (8U)
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SSIZE_SHIFT)) & DMA_ATTR_SSIZE_MASK)
/*! SMOD - Source Address Modulo
 *  0b00000..Source address modulo feature is disabled
 *  0b00001-0b11111..This value defines a specific address range specified to be the value after SADDR + SOFF
 *                   calculation is performed on the original register value. Setting this field provides the ability
 *                   to implement a circular data queue easily. For data queues requiring power-of-2 size bytes, the
 *                   queue should start at a 0-modulo-size address and the SMOD field should be set to the
 *                   appropriate value for the queue, freezing the desired number of upper address bits. The value
 *                   programmed into this field specifies the number of lower address bits allowed to change. For a
 *                   circular queue application, the SOFF is typically set to the transfer size to implement
 *                   post-increment addressing with the SMOD function constraining the addresses to a 0-modulo-size range.
 */
#define DMA_ATTR_SMOD_MASK                       (0xF800U)
#define DMA_ATTR_SMOD_SHIFT                      (11U)
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_ATTR_SMOD_SHIFT)) & DMA_ATTR_SMOD_MASK)
/*! @} */

/* The count of DMA_ATTR */
#define DMA_ATTR_COUNT                           (4U)

/*! @name NBYTES_MLNO - TCD Minor Byte Count (Minor Loop Mapping Disabled) */
/*! @{ */
#define DMA_NBYTES_MLNO_NBYTES_MASK              (0xFFFFFFFFU)
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             (0U)
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLNO_NBYTES_SHIFT)) & DMA_NBYTES_MLNO_NBYTES_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLNO */
#define DMA_NBYTES_MLNO_COUNT                    (4U)

/*! @name NBYTES_MLOFFNO - TCD Signed Minor Loop Offset (Minor Loop Mapping Enabled and Offset Disabled) */
/*! @{ */
#define DMA_NBYTES_MLOFFNO_NBYTES_MASK           (0x3FFFFFFFU)
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT          (0U)
#define DMA_NBYTES_MLOFFNO_NBYTES(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFNO_NBYTES_MASK)
/*! DMLOE - Destination Minor Loop Offset enable
 *  0b0..The minor loop offset is not applied to the DADDR
 *  0b1..The minor loop offset is applied to the DADDR
 */
#define DMA_NBYTES_MLOFFNO_DMLOE_MASK            (0x40000000U)
#define DMA_NBYTES_MLOFFNO_DMLOE_SHIFT           (30U)
#define DMA_NBYTES_MLOFFNO_DMLOE(x)              (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_DMLOE_MASK)
/*! SMLOE - Source Minor Loop Offset Enable
 *  0b0..The minor loop offset is not applied to the SADDR
 *  0b1..The minor loop offset is applied to the SADDR
 */
#define DMA_NBYTES_MLOFFNO_SMLOE_MASK            (0x80000000U)
#define DMA_NBYTES_MLOFFNO_SMLOE_SHIFT           (31U)
#define DMA_NBYTES_MLOFFNO_SMLOE(x)              (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFNO_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFNO_SMLOE_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLOFFNO */
#define DMA_NBYTES_MLOFFNO_COUNT                 (4U)

/*! @name NBYTES_MLOFFYES - TCD Signed Minor Loop Offset (Minor Loop Mapping and Offset Enabled) */
/*! @{ */
#define DMA_NBYTES_MLOFFYES_NBYTES_MASK          (0x3FFU)
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT         (0U)
#define DMA_NBYTES_MLOFFYES_NBYTES(x)            (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_NBYTES_SHIFT)) & DMA_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           (0x3FFFFC00U)
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          (10U)
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_MLOFF_SHIFT)) & DMA_NBYTES_MLOFFYES_MLOFF_MASK)
/*! DMLOE - Destination Minor Loop Offset enable
 *  0b0..The minor loop offset is not applied to the DADDR
 *  0b1..The minor loop offset is applied to the DADDR
 */
#define DMA_NBYTES_MLOFFYES_DMLOE_MASK           (0x40000000U)
#define DMA_NBYTES_MLOFFYES_DMLOE_SHIFT          (30U)
#define DMA_NBYTES_MLOFFYES_DMLOE(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_DMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_DMLOE_MASK)
/*! SMLOE - Source Minor Loop Offset Enable
 *  0b0..The minor loop offset is not applied to the SADDR
 *  0b1..The minor loop offset is applied to the SADDR
 */
#define DMA_NBYTES_MLOFFYES_SMLOE_MASK           (0x80000000U)
#define DMA_NBYTES_MLOFFYES_SMLOE_SHIFT          (31U)
#define DMA_NBYTES_MLOFFYES_SMLOE(x)             (((uint32_t)(((uint32_t)(x)) << DMA_NBYTES_MLOFFYES_SMLOE_SHIFT)) & DMA_NBYTES_MLOFFYES_SMLOE_MASK)
/*! @} */

/* The count of DMA_NBYTES_MLOFFYES */
#define DMA_NBYTES_MLOFFYES_COUNT                (4U)

/*! @name SLAST - TCD Last Source Address Adjustment */
/*! @{ */
#define DMA_SLAST_SLAST_MASK                     (0xFFFFFFFFU)
#define DMA_SLAST_SLAST_SHIFT                    (0U)
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_SLAST_SLAST_SHIFT)) & DMA_SLAST_SLAST_MASK)
/*! @} */

/* The count of DMA_SLAST */
#define DMA_SLAST_COUNT                          (4U)

/*! @name DADDR - TCD Destination Address */
/*! @{ */
#define DMA_DADDR_DADDR_MASK                     (0xFFFFFFFFU)
#define DMA_DADDR_DADDR_SHIFT                    (0U)
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x)) << DMA_DADDR_DADDR_SHIFT)) & DMA_DADDR_DADDR_MASK)
/*! @} */

/* The count of DMA_DADDR */
#define DMA_DADDR_COUNT                          (4U)

/*! @name DOFF - TCD Signed Destination Address Offset */
/*! @{ */
#define DMA_DOFF_DOFF_MASK                       (0xFFFFU)
#define DMA_DOFF_DOFF_SHIFT                      (0U)
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_DOFF_DOFF_SHIFT)) & DMA_DOFF_DOFF_MASK)
/*! @} */

/* The count of DMA_DOFF */
#define DMA_DOFF_COUNT                           (4U)

/*! @name CITER_ELINKNO - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */
#define DMA_CITER_ELINKNO_CITER_MASK             (0x7FFFU)
#define DMA_CITER_ELINKNO_CITER_SHIFT            (0U)
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_CITER_SHIFT)) & DMA_CITER_ELINKNO_CITER_MASK)
/*! ELINK - Enable channel-to-channel linking on minor-loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_CITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_CITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_CITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKNO_ELINK_SHIFT)) & DMA_CITER_ELINKNO_ELINK_MASK)
/*! @} */

/* The count of DMA_CITER_ELINKNO */
#define DMA_CITER_ELINKNO_COUNT                  (4U)

/*! @name CITER_ELINKYES - TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */
#define DMA_CITER_ELINKYES_CITER_MASK            (0x1FFU)
#define DMA_CITER_ELINKYES_CITER_SHIFT           (0U)
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_CITER_SHIFT)) & DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           (0x600U)
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_LINKCH_SHIFT)) & DMA_CITER_ELINKYES_LINKCH_MASK)
/*! ELINK - Enable channel-to-channel linking on minor-loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_CITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_CITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_CITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_CITER_ELINKYES_ELINK_SHIFT)) & DMA_CITER_ELINKYES_ELINK_MASK)
/*! @} */

/* The count of DMA_CITER_ELINKYES */
#define DMA_CITER_ELINKYES_COUNT                 (4U)

/*! @name DLASTSGA - TCD Last Destination Address Adjustment/Scatter Gather Address */
/*! @{ */
#define DMA_DLASTSGA_DLASTSGA_MASK               (0xFFFFFFFFU)
#define DMA_DLASTSGA_DLASTSGA_SHIFT              (0U)
#define DMA_DLASTSGA_DLASTSGA(x)                 (((uint32_t)(((uint32_t)(x)) << DMA_DLASTSGA_DLASTSGA_SHIFT)) & DMA_DLASTSGA_DLASTSGA_MASK)
/*! @} */

/* The count of DMA_DLASTSGA */
#define DMA_DLASTSGA_COUNT                       (4U)

/*! @name CSR - TCD Control and Status */
/*! @{ */
/*! START - Channel Start
 *  0b0..The channel is not explicitly started.
 *  0b1..The channel is explicitly started via a software initiated service request.
 */
#define DMA_CSR_START_MASK                       (0x1U)
#define DMA_CSR_START_SHIFT                      (0U)
#define DMA_CSR_START(x)                         (((uint16_t)(((uint16_t)(x)) << DMA_CSR_START_SHIFT)) & DMA_CSR_START_MASK)
/*! INTMAJOR - Enable an interrupt when major iteration count completes.
 *  0b0..The end-of-major loop interrupt is disabled.
 *  0b1..The end-of-major loop interrupt is enabled.
 */
#define DMA_CSR_INTMAJOR_MASK                    (0x2U)
#define DMA_CSR_INTMAJOR_SHIFT                   (1U)
#define DMA_CSR_INTMAJOR(x)                      (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTMAJOR_SHIFT)) & DMA_CSR_INTMAJOR_MASK)
/*! INTHALF - Enable an interrupt when major counter is half complete.
 *  0b0..The half-point interrupt is disabled.
 *  0b1..The half-point interrupt is enabled.
 */
#define DMA_CSR_INTHALF_MASK                     (0x4U)
#define DMA_CSR_INTHALF_SHIFT                    (2U)
#define DMA_CSR_INTHALF(x)                       (((uint16_t)(((uint16_t)(x)) << DMA_CSR_INTHALF_SHIFT)) & DMA_CSR_INTHALF_MASK)
/*! DREQ - Disable Request
 *  0b0..The channel's ERQ bit is not affected.
 *  0b1..The channel's ERQ bit is cleared when the major loop is complete.
 */
#define DMA_CSR_DREQ_MASK                        (0x8U)
#define DMA_CSR_DREQ_SHIFT                       (3U)
#define DMA_CSR_DREQ(x)                          (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DREQ_SHIFT)) & DMA_CSR_DREQ_MASK)
/*! ESG - Enable Scatter/Gather Processing
 *  0b0..The current channel's TCD is normal format.
 *  0b1..The current channel's TCD specifies a scatter gather format. The DLASTSGA field provides a memory pointer
 *       to the next TCD to be loaded into this channel after the major loop completes its execution.
 */
#define DMA_CSR_ESG_MASK                         (0x10U)
#define DMA_CSR_ESG_SHIFT                        (4U)
#define DMA_CSR_ESG(x)                           (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ESG_SHIFT)) & DMA_CSR_ESG_MASK)
/*! MAJORELINK - Enable channel-to-channel linking on major loop complete
 *  0b0..The channel-to-channel linking is disabled.
 *  0b1..The channel-to-channel linking is enabled.
 */
#define DMA_CSR_MAJORELINK_MASK                  (0x20U)
#define DMA_CSR_MAJORELINK_SHIFT                 (5U)
#define DMA_CSR_MAJORELINK(x)                    (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORELINK_SHIFT)) & DMA_CSR_MAJORELINK_MASK)
#define DMA_CSR_ACTIVE_MASK                      (0x40U)
#define DMA_CSR_ACTIVE_SHIFT                     (6U)
#define DMA_CSR_ACTIVE(x)                        (((uint16_t)(((uint16_t)(x)) << DMA_CSR_ACTIVE_SHIFT)) & DMA_CSR_ACTIVE_MASK)
#define DMA_CSR_DONE_MASK                        (0x80U)
#define DMA_CSR_DONE_SHIFT                       (7U)
#define DMA_CSR_DONE(x)                          (((uint16_t)(((uint16_t)(x)) << DMA_CSR_DONE_SHIFT)) & DMA_CSR_DONE_MASK)
#define DMA_CSR_MAJORLINKCH_MASK                 (0x300U)
#define DMA_CSR_MAJORLINKCH_SHIFT                (8U)
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x)) << DMA_CSR_MAJORLINKCH_SHIFT)) & DMA_CSR_MAJORLINKCH_MASK)
/*! BWC - Bandwidth Control
 *  0b00..No eDMA engine stalls.
 *  0b01..Reserved
 *  0b10..eDMA engine stalls for 4 cycles after each R/W.
 *  0b11..eDMA engine stalls for 8 cycles after each R/W.
 */
#define DMA_CSR_BWC_MASK                         (0xC000U)
#define DMA_CSR_BWC_SHIFT                        (14U)
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x)) << DMA_CSR_BWC_SHIFT)) & DMA_CSR_BWC_MASK)
/*! @} */

/* The count of DMA_CSR */
#define DMA_CSR_COUNT                            (4U)

/*! @name BITER_ELINKNO - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled) */
/*! @{ */
#define DMA_BITER_ELINKNO_BITER_MASK             (0x7FFFU)
#define DMA_BITER_ELINKNO_BITER_SHIFT            (0U)
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_BITER_SHIFT)) & DMA_BITER_ELINKNO_BITER_MASK)
/*! ELINK - Enables channel-to-channel linking on minor loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_BITER_ELINKNO_ELINK_MASK             (0x8000U)
#define DMA_BITER_ELINKNO_ELINK_SHIFT            (15U)
#define DMA_BITER_ELINKNO_ELINK(x)               (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKNO_ELINK_SHIFT)) & DMA_BITER_ELINKNO_ELINK_MASK)
/*! @} */

/* The count of DMA_BITER_ELINKNO */
#define DMA_BITER_ELINKNO_COUNT                  (4U)

/*! @name BITER_ELINKYES - TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled) */
/*! @{ */
#define DMA_BITER_ELINKYES_BITER_MASK            (0x1FFU)
#define DMA_BITER_ELINKYES_BITER_SHIFT           (0U)
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_BITER_SHIFT)) & DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           (0x600U)
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          (9U)
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_LINKCH_SHIFT)) & DMA_BITER_ELINKYES_LINKCH_MASK)
/*! ELINK - Enables channel-to-channel linking on minor loop complete
 *  0b0..The channel-to-channel linking is disabled
 *  0b1..The channel-to-channel linking is enabled
 */
#define DMA_BITER_ELINKYES_ELINK_MASK            (0x8000U)
#define DMA_BITER_ELINKYES_ELINK_SHIFT           (15U)
#define DMA_BITER_ELINKYES_ELINK(x)              (((uint16_t)(((uint16_t)(x)) << DMA_BITER_ELINKYES_ELINK_SHIFT)) & DMA_BITER_ELINKYES_ELINK_MASK)
/*! @} */

/* The count of DMA_BITER_ELINKYES */
#define DMA_BITER_ELINKYES_COUNT                 (4U)


/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base address */
#define DMA_BASE                                 (0xC800u)
/** Peripheral DMA base pointer */
#define DMA                                      ((DMA_Type *)DMA_BASE)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA }

/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Register Layout Typedef */
typedef struct {
  __IO uint8_t CHCFG[4];                           /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_Type;

/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/*! @name CHCFG - Channel Configuration register */
/*! @{ */
#define DMAMUX_CHCFG_SOURCE_MASK                 (0x3FU)
#define DMAMUX_CHCFG_SOURCE_SHIFT                (0U)
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_SOURCE_SHIFT)) & DMAMUX_CHCFG_SOURCE_MASK)
/*! ENBL - DMA Channel Enable
 *  0b0..DMA channel is disabled. This mode is primarily used during configuration of the DMAMux. The DMA has
 *       separate channel enables/disables, which should be used to disable or reconfigure a DMA channel.
 *  0b1..DMA channel is enabled
 */
#define DMAMUX_CHCFG_ENBL_MASK                   (0x80U)
#define DMAMUX_CHCFG_ENBL_SHIFT                  (7U)
#define DMAMUX_CHCFG_ENBL(x)                     (((uint8_t)(((uint8_t)(x)) << DMAMUX_CHCFG_ENBL_SHIFT)) & DMAMUX_CHCFG_ENBL_MASK)
/*! @} */

/* The count of DMAMUX_CHCFG */
#define DMAMUX_CHCFG_COUNT                       (4U)


/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE                              (0xE3B0u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX                                   ((DMAMUX_Type *)DMAMUX_BASE)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX }

/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EVTG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EVTG_Peripheral_Access_Layer EVTG Peripheral Access Layer
 * @{
 */

/** EVTG - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x8 */
    __IO uint16_t EVTG_AOI0_BFT01;                   /**< AOI0 Boolean Function Term 0 and 1 Configuration Register, array offset: 0x0, array step: 0x8 */
    __IO uint16_t EVTG_AOI0_BFT23;                   /**< AOI0 Boolean Function Term 2 and 3 Configuration Register, array offset: 0x1, array step: 0x8 */
    __IO uint16_t EVTG_AOI1_BFT01;                   /**< AOI1 Boolean Function Term 0 and 1 Configuration Register, array offset: 0x2, array step: 0x8 */
    __IO uint16_t EVTG_AOI1_BFT23;                   /**< AOI1 Boolean Function Term 2 and 3 Configuration Register, array offset: 0x3, array step: 0x8 */
         uint16_t RESERVED_0[1];
    __IO uint16_t EVTG_CTRL;                         /**< Control/Status Register, array offset: 0x5, array step: 0x8 */
    __IO uint16_t EVTG_AOI0_FILT;                    /**< AOI0 Input Filter Register, array offset: 0x6, array step: 0x8 */
    __IO uint16_t EVTG_AOI1_FILT;                    /**< AOI1 Input Filter Register, array offset: 0x7, array step: 0x8 */
  } EVTG_INST[4];
} EVTG_Type;

/* ----------------------------------------------------------------------------
   -- EVTG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EVTG_Register_Masks EVTG Register Masks
 * @{
 */

/*! @name EVTG_AOI0_BFT01 - AOI0 Boolean Function Term 0 and 1 Configuration Register */
/*! @{ */
#define EVTG_EVTG_AOI0_BFT01_PT1_DC_MASK         (0x3U)
#define EVTG_EVTG_AOI0_BFT01_PT1_DC_SHIFT        (0U)
#define EVTG_EVTG_AOI0_BFT01_PT1_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_DC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT1_CC_MASK         (0xCU)
#define EVTG_EVTG_AOI0_BFT01_PT1_CC_SHIFT        (2U)
#define EVTG_EVTG_AOI0_BFT01_PT1_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_CC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT1_BC_MASK         (0x30U)
#define EVTG_EVTG_AOI0_BFT01_PT1_BC_SHIFT        (4U)
#define EVTG_EVTG_AOI0_BFT01_PT1_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_BC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT1_AC_MASK         (0xC0U)
#define EVTG_EVTG_AOI0_BFT01_PT1_AC_SHIFT        (6U)
#define EVTG_EVTG_AOI0_BFT01_PT1_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT1_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT1_AC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK         (0x300U)
#define EVTG_EVTG_AOI0_BFT01_PT0_DC_SHIFT        (8U)
#define EVTG_EVTG_AOI0_BFT01_PT0_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_DC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT0_CC_MASK         (0xC00U)
#define EVTG_EVTG_AOI0_BFT01_PT0_CC_SHIFT        (10U)
#define EVTG_EVTG_AOI0_BFT01_PT0_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_CC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT0_BC_MASK         (0x3000U)
#define EVTG_EVTG_AOI0_BFT01_PT0_BC_SHIFT        (12U)
#define EVTG_EVTG_AOI0_BFT01_PT0_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_BC_MASK)
#define EVTG_EVTG_AOI0_BFT01_PT0_AC_MASK         (0xC000U)
#define EVTG_EVTG_AOI0_BFT01_PT0_AC_SHIFT        (14U)
#define EVTG_EVTG_AOI0_BFT01_PT0_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT01_PT0_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT01_PT0_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_BFT01 */
#define EVTG_EVTG_AOI0_BFT01_COUNT               (4U)

/*! @name EVTG_AOI0_BFT23 - AOI0 Boolean Function Term 2 and 3 Configuration Register */
/*! @{ */
#define EVTG_EVTG_AOI0_BFT23_PT3_DC_MASK         (0x3U)
#define EVTG_EVTG_AOI0_BFT23_PT3_DC_SHIFT        (0U)
#define EVTG_EVTG_AOI0_BFT23_PT3_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_DC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT3_CC_MASK         (0xCU)
#define EVTG_EVTG_AOI0_BFT23_PT3_CC_SHIFT        (2U)
#define EVTG_EVTG_AOI0_BFT23_PT3_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_CC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT3_BC_MASK         (0x30U)
#define EVTG_EVTG_AOI0_BFT23_PT3_BC_SHIFT        (4U)
#define EVTG_EVTG_AOI0_BFT23_PT3_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_BC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT3_AC_MASK         (0xC0U)
#define EVTG_EVTG_AOI0_BFT23_PT3_AC_SHIFT        (6U)
#define EVTG_EVTG_AOI0_BFT23_PT3_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT3_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT3_AC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT2_DC_MASK         (0x300U)
#define EVTG_EVTG_AOI0_BFT23_PT2_DC_SHIFT        (8U)
#define EVTG_EVTG_AOI0_BFT23_PT2_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_DC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_DC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT2_CC_MASK         (0xC00U)
#define EVTG_EVTG_AOI0_BFT23_PT2_CC_SHIFT        (10U)
#define EVTG_EVTG_AOI0_BFT23_PT2_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_CC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_CC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT2_BC_MASK         (0x3000U)
#define EVTG_EVTG_AOI0_BFT23_PT2_BC_SHIFT        (12U)
#define EVTG_EVTG_AOI0_BFT23_PT2_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_BC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_BC_MASK)
#define EVTG_EVTG_AOI0_BFT23_PT2_AC_MASK         (0xC000U)
#define EVTG_EVTG_AOI0_BFT23_PT2_AC_SHIFT        (14U)
#define EVTG_EVTG_AOI0_BFT23_PT2_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_BFT23_PT2_AC_SHIFT)) & EVTG_EVTG_AOI0_BFT23_PT2_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_BFT23 */
#define EVTG_EVTG_AOI0_BFT23_COUNT               (4U)

/*! @name EVTG_AOI1_BFT01 - AOI1 Boolean Function Term 0 and 1 Configuration Register */
/*! @{ */
#define EVTG_EVTG_AOI1_BFT01_PT1_DC_MASK         (0x3U)
#define EVTG_EVTG_AOI1_BFT01_PT1_DC_SHIFT        (0U)
#define EVTG_EVTG_AOI1_BFT01_PT1_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_DC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT1_CC_MASK         (0xCU)
#define EVTG_EVTG_AOI1_BFT01_PT1_CC_SHIFT        (2U)
#define EVTG_EVTG_AOI1_BFT01_PT1_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_CC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT1_BC_MASK         (0x30U)
#define EVTG_EVTG_AOI1_BFT01_PT1_BC_SHIFT        (4U)
#define EVTG_EVTG_AOI1_BFT01_PT1_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_BC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT1_AC_MASK         (0xC0U)
#define EVTG_EVTG_AOI1_BFT01_PT1_AC_SHIFT        (6U)
#define EVTG_EVTG_AOI1_BFT01_PT1_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT1_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT1_AC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK         (0x300U)
#define EVTG_EVTG_AOI1_BFT01_PT0_DC_SHIFT        (8U)
#define EVTG_EVTG_AOI1_BFT01_PT0_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_DC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT0_CC_MASK         (0xC00U)
#define EVTG_EVTG_AOI1_BFT01_PT0_CC_SHIFT        (10U)
#define EVTG_EVTG_AOI1_BFT01_PT0_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_CC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT0_BC_MASK         (0x3000U)
#define EVTG_EVTG_AOI1_BFT01_PT0_BC_SHIFT        (12U)
#define EVTG_EVTG_AOI1_BFT01_PT0_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_BC_MASK)
#define EVTG_EVTG_AOI1_BFT01_PT0_AC_MASK         (0xC000U)
#define EVTG_EVTG_AOI1_BFT01_PT0_AC_SHIFT        (14U)
#define EVTG_EVTG_AOI1_BFT01_PT0_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT01_PT0_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT01_PT0_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_BFT01 */
#define EVTG_EVTG_AOI1_BFT01_COUNT               (4U)

/*! @name EVTG_AOI1_BFT23 - AOI1 Boolean Function Term 2 and 3 Configuration Register */
/*! @{ */
#define EVTG_EVTG_AOI1_BFT23_PT3_DC_MASK         (0x3U)
#define EVTG_EVTG_AOI1_BFT23_PT3_DC_SHIFT        (0U)
#define EVTG_EVTG_AOI1_BFT23_PT3_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_DC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT3_CC_MASK         (0xCU)
#define EVTG_EVTG_AOI1_BFT23_PT3_CC_SHIFT        (2U)
#define EVTG_EVTG_AOI1_BFT23_PT3_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_CC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT3_BC_MASK         (0x30U)
#define EVTG_EVTG_AOI1_BFT23_PT3_BC_SHIFT        (4U)
#define EVTG_EVTG_AOI1_BFT23_PT3_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_BC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT3_AC_MASK         (0xC0U)
#define EVTG_EVTG_AOI1_BFT23_PT3_AC_SHIFT        (6U)
#define EVTG_EVTG_AOI1_BFT23_PT3_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT3_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT3_AC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT2_DC_MASK         (0x300U)
#define EVTG_EVTG_AOI1_BFT23_PT2_DC_SHIFT        (8U)
#define EVTG_EVTG_AOI1_BFT23_PT2_DC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_DC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_DC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT2_CC_MASK         (0xC00U)
#define EVTG_EVTG_AOI1_BFT23_PT2_CC_SHIFT        (10U)
#define EVTG_EVTG_AOI1_BFT23_PT2_CC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_CC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_CC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT2_BC_MASK         (0x3000U)
#define EVTG_EVTG_AOI1_BFT23_PT2_BC_SHIFT        (12U)
#define EVTG_EVTG_AOI1_BFT23_PT2_BC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_BC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_BC_MASK)
#define EVTG_EVTG_AOI1_BFT23_PT2_AC_MASK         (0xC000U)
#define EVTG_EVTG_AOI1_BFT23_PT2_AC_SHIFT        (14U)
#define EVTG_EVTG_AOI1_BFT23_PT2_AC(x)           (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_BFT23_PT2_AC_SHIFT)) & EVTG_EVTG_AOI1_BFT23_PT2_AC_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_BFT23 */
#define EVTG_EVTG_AOI1_BFT23_COUNT               (4U)

/*! @name EVTG_CTRL - Control/Status Register */
/*! @{ */
#define EVTG_EVTG_CTRL_FF_INIT_MASK              (0x1U)
#define EVTG_EVTG_CTRL_FF_INIT_SHIFT             (0U)
#define EVTG_EVTG_CTRL_FF_INIT(x)                (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_FF_INIT_SHIFT)) & EVTG_EVTG_CTRL_FF_INIT_MASK)
#define EVTG_EVTG_CTRL_INIT_EN_MASK              (0x2U)
#define EVTG_EVTG_CTRL_INIT_EN_SHIFT             (1U)
#define EVTG_EVTG_CTRL_INIT_EN(x)                (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_INIT_EN_SHIFT)) & EVTG_EVTG_CTRL_INIT_EN_MASK)
#define EVTG_EVTG_CTRL_MODE_SEL_MASK             (0x1CU)
#define EVTG_EVTG_CTRL_MODE_SEL_SHIFT            (2U)
#define EVTG_EVTG_CTRL_MODE_SEL(x)               (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_MODE_SEL_SHIFT)) & EVTG_EVTG_CTRL_MODE_SEL_MASK)
#define EVTG_EVTG_CTRL_FB_OVRD_MASK              (0xC0U)
#define EVTG_EVTG_CTRL_FB_OVRD_SHIFT             (6U)
#define EVTG_EVTG_CTRL_FB_OVRD(x)                (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_FB_OVRD_SHIFT)) & EVTG_EVTG_CTRL_FB_OVRD_MASK)
#define EVTG_EVTG_CTRL_SYNC_CTRL_MASK            (0xF00U)
#define EVTG_EVTG_CTRL_SYNC_CTRL_SHIFT           (8U)
#define EVTG_EVTG_CTRL_SYNC_CTRL(x)              (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_CTRL_SYNC_CTRL_SHIFT)) & EVTG_EVTG_CTRL_SYNC_CTRL_MASK)
/*! @} */

/* The count of EVTG_EVTG_CTRL */
#define EVTG_EVTG_CTRL_COUNT                     (4U)

/*! @name EVTG_AOI0_FILT - AOI0 Input Filter Register */
/*! @{ */
#define EVTG_EVTG_AOI0_FILT_FILT_PER_MASK        (0xFFU)
#define EVTG_EVTG_AOI0_FILT_FILT_PER_SHIFT       (0U)
#define EVTG_EVTG_AOI0_FILT_FILT_PER(x)          (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_FILT_FILT_PER_SHIFT)) & EVTG_EVTG_AOI0_FILT_FILT_PER_MASK)
#define EVTG_EVTG_AOI0_FILT_FILT_CNT_MASK        (0x700U)
#define EVTG_EVTG_AOI0_FILT_FILT_CNT_SHIFT       (8U)
#define EVTG_EVTG_AOI0_FILT_FILT_CNT(x)          (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI0_FILT_FILT_CNT_SHIFT)) & EVTG_EVTG_AOI0_FILT_FILT_CNT_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI0_FILT */
#define EVTG_EVTG_AOI0_FILT_COUNT                (4U)

/*! @name EVTG_AOI1_FILT - AOI1 Input Filter Register */
/*! @{ */
#define EVTG_EVTG_AOI1_FILT_FILT_PER_MASK        (0xFFU)
#define EVTG_EVTG_AOI1_FILT_FILT_PER_SHIFT       (0U)
#define EVTG_EVTG_AOI1_FILT_FILT_PER(x)          (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_FILT_FILT_PER_SHIFT)) & EVTG_EVTG_AOI1_FILT_FILT_PER_MASK)
#define EVTG_EVTG_AOI1_FILT_FILT_CNT_MASK        (0x700U)
#define EVTG_EVTG_AOI1_FILT_FILT_CNT_SHIFT       (8U)
#define EVTG_EVTG_AOI1_FILT_FILT_CNT(x)          (((uint16_t)(((uint16_t)(x)) << EVTG_EVTG_AOI1_FILT_FILT_CNT_SHIFT)) & EVTG_EVTG_AOI1_FILT_FILT_CNT_MASK)
/*! @} */

/* The count of EVTG_EVTG_AOI1_FILT */
#define EVTG_EVTG_AOI1_FILT_COUNT                (4U)


/*!
 * @}
 */ /* end of group EVTG_Register_Masks */


/* EVTG - Peripheral instance base addresses */
/** Peripheral EVTG base address */
#define EVTG_BASE                                (0xE380u)
/** Peripheral EVTG base pointer */
#define EVTG                                     ((EVTG_Type *)EVTG_BASE)
/** Array initializer of EVTG peripheral base addresses */
#define EVTG_BASE_ADDRS                          { EVTG_BASE }
/** Array initializer of EVTG peripheral base pointers */
#define EVTG_BASE_PTRS                           { EVTG }

/*!
 * @}
 */ /* end of group EVTG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x0 */
  __O  uint16_t SERV;                              /**< Service Register, offset: 0x1 */
  __IO uint16_t CMPL;                              /**< Compare Low Register, offset: 0x2 */
  __IO uint16_t CMPH;                              /**< Compare High Register, offset: 0x3 */
  __IO uint16_t CLKCTRL;                           /**< Clock Control Register, offset: 0x4 */
  __IO uint16_t CLKPRESCALER;                      /**< Clock Prescaler Register, offset: 0x5 */
} EWM_Type;

/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */
#define EWM_CTRL_EWMEN_MASK                      (0x1U)
#define EWM_CTRL_EWMEN_SHIFT                     (0U)
#define EWM_CTRL_EWMEN(x)                        (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_EWMEN_SHIFT)) & EWM_CTRL_EWMEN_MASK)
#define EWM_CTRL_ASSIN_MASK                      (0x2U)
#define EWM_CTRL_ASSIN_SHIFT                     (1U)
#define EWM_CTRL_ASSIN(x)                        (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_ASSIN_SHIFT)) & EWM_CTRL_ASSIN_MASK)
#define EWM_CTRL_INEN_MASK                       (0x4U)
#define EWM_CTRL_INEN_SHIFT                      (2U)
#define EWM_CTRL_INEN(x)                         (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_INEN_SHIFT)) & EWM_CTRL_INEN_MASK)
#define EWM_CTRL_INTEN_MASK                      (0x8U)
#define EWM_CTRL_INTEN_SHIFT                     (3U)
#define EWM_CTRL_INTEN(x)                        (((uint16_t)(((uint16_t)(x)) << EWM_CTRL_INTEN_SHIFT)) & EWM_CTRL_INTEN_MASK)
/*! @} */

/*! @name SERV - Service Register */
/*! @{ */
#define EWM_SERV_SERVICE_MASK                    (0xFFU)
#define EWM_SERV_SERVICE_SHIFT                   (0U)
#define EWM_SERV_SERVICE(x)                      (((uint16_t)(((uint16_t)(x)) << EWM_SERV_SERVICE_SHIFT)) & EWM_SERV_SERVICE_MASK)
/*! @} */

/*! @name CMPL - Compare Low Register */
/*! @{ */
#define EWM_CMPL_COMPAREL_MASK                   (0xFFU)
#define EWM_CMPL_COMPAREL_SHIFT                  (0U)
#define EWM_CMPL_COMPAREL(x)                     (((uint16_t)(((uint16_t)(x)) << EWM_CMPL_COMPAREL_SHIFT)) & EWM_CMPL_COMPAREL_MASK)
/*! @} */

/*! @name CMPH - Compare High Register */
/*! @{ */
#define EWM_CMPH_COMPAREH_MASK                   (0xFFU)
#define EWM_CMPH_COMPAREH_SHIFT                  (0U)
#define EWM_CMPH_COMPAREH(x)                     (((uint16_t)(((uint16_t)(x)) << EWM_CMPH_COMPAREH_SHIFT)) & EWM_CMPH_COMPAREH_MASK)
/*! @} */

/*! @name CLKCTRL - Clock Control Register */
/*! @{ */
#define EWM_CLKCTRL_CLKSEL_MASK                  (0x3U)
#define EWM_CLKCTRL_CLKSEL_SHIFT                 (0U)
#define EWM_CLKCTRL_CLKSEL(x)                    (((uint16_t)(((uint16_t)(x)) << EWM_CLKCTRL_CLKSEL_SHIFT)) & EWM_CLKCTRL_CLKSEL_MASK)
/*! @} */

/*! @name CLKPRESCALER - Clock Prescaler Register */
/*! @{ */
#define EWM_CLKPRESCALER_CLK_DIV_MASK            (0xFFU)
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT           (0U)
#define EWM_CLKPRESCALER_CLK_DIV(x)              (((uint16_t)(((uint16_t)(x)) << EWM_CLKPRESCALER_CLK_DIV_SHIFT)) & EWM_CLKPRESCALER_CLK_DIV_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group EWM_Register_Masks */


/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE                                 (0xE330u)
/** Peripheral EWM base pointer */
#define EWM                                      ((EWM_Type *)EWM_BASE)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS                           { EWM_BASE }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM }

/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFE Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Peripheral_Access_Layer FTFE Peripheral Access Layer
 * @{
 */

/** FTFE - Register Layout Typedef */
typedef struct {
  __IO uint8_t FSTAT;                              /**< Flash Status Register, offset: 0x0 */
  __IO uint8_t FCNFG;                              /**< Flash Configuration Register, offset: 0x1 */
  __I  uint8_t FSEC;                               /**< Flash Security Register, offset: 0x2 */
  __I  uint8_t FOPT;                               /**< Flash Option Register, offset: 0x3 */
  __IO uint8_t FCCOB3;                             /**< Flash Common Command Object Registers, offset: 0x4 */
  __IO uint8_t FCCOB2;                             /**< Flash Common Command Object Registers, offset: 0x5 */
  __IO uint8_t FCCOB1;                             /**< Flash Common Command Object Registers, offset: 0x6 */
  __IO uint8_t FCCOB0;                             /**< Flash Common Command Object Registers, offset: 0x7 */
  __IO uint8_t FCCOB7;                             /**< Flash Common Command Object Registers, offset: 0x8 */
  __IO uint8_t FCCOB6;                             /**< Flash Common Command Object Registers, offset: 0x9 */
  __IO uint8_t FCCOB5;                             /**< Flash Common Command Object Registers, offset: 0xA */
  __IO uint8_t FCCOB4;                             /**< Flash Common Command Object Registers, offset: 0xB */
  __IO uint8_t FCCOBB;                             /**< Flash Common Command Object Registers, offset: 0xC */
  __IO uint8_t FCCOBA;                             /**< Flash Common Command Object Registers, offset: 0xD */
  __IO uint8_t FCCOB9;                             /**< Flash Common Command Object Registers, offset: 0xE */
  __IO uint8_t FCCOB8;                             /**< Flash Common Command Object Registers, offset: 0xF */
  __IO uint8_t FPROT3;                             /**< Program Flash Protection Registers, offset: 0x10 */
  __IO uint8_t FPROT2;                             /**< Program Flash Protection Registers, offset: 0x11 */
  __IO uint8_t FPROT1;                             /**< Program Flash Protection Registers, offset: 0x12 */
  __IO uint8_t FPROT0;                             /**< Program Flash Protection Registers, offset: 0x13 */
       uint8_t RESERVED_0[26];
  __IO uint8_t FERSTAT;                            /**< Flash Error Status Register, offset: 0x2E */
  __IO uint8_t FERCNFG;                            /**< Flash Error Configuration Register, offset: 0x2F */
} FTFE_Type;

/* ----------------------------------------------------------------------------
   -- FTFE Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Register_Masks FTFE Register Masks
 * @{
 */

/*! @name FSTAT - Flash Status Register */
/*! @{ */
#define FTFE_FSTAT_MGSTAT0_MASK                  (0x1U)
#define FTFE_FSTAT_MGSTAT0_SHIFT                 (0U)
#define FTFE_FSTAT_MGSTAT0(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_MGSTAT0_SHIFT)) & FTFE_FSTAT_MGSTAT0_MASK)
/*! FPVIOL - Flash Protection Violation Flag
 *  0b0..No protection violation detected
 *  0b1..Protection violation detected
 */
#define FTFE_FSTAT_FPVIOL_MASK                   (0x10U)
#define FTFE_FSTAT_FPVIOL_SHIFT                  (4U)
#define FTFE_FSTAT_FPVIOL(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_FPVIOL_SHIFT)) & FTFE_FSTAT_FPVIOL_MASK)
/*! ACCERR - Flash Access Error Flag
 *  0b0..No access error detected
 *  0b1..Access error detected
 */
#define FTFE_FSTAT_ACCERR_MASK                   (0x20U)
#define FTFE_FSTAT_ACCERR_SHIFT                  (5U)
#define FTFE_FSTAT_ACCERR(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_ACCERR_SHIFT)) & FTFE_FSTAT_ACCERR_MASK)
/*! RDCOLERR - FTFE Read Collision Error Flag
 *  0b0..No collision error detected
 *  0b1..Collision error detected
 */
#define FTFE_FSTAT_RDCOLERR_MASK                 (0x40U)
#define FTFE_FSTAT_RDCOLERR_SHIFT                (6U)
#define FTFE_FSTAT_RDCOLERR(x)                   (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_RDCOLERR_SHIFT)) & FTFE_FSTAT_RDCOLERR_MASK)
/*! CCIF - Command Complete Interrupt Flag
 *  0b0..FTFE command in progress
 *  0b1..FTFE command has completed
 */
#define FTFE_FSTAT_CCIF_MASK                     (0x80U)
#define FTFE_FSTAT_CCIF_SHIFT                    (7U)
#define FTFE_FSTAT_CCIF(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FSTAT_CCIF_SHIFT)) & FTFE_FSTAT_CCIF_MASK)
/*! @} */

/*! @name FCNFG - Flash Configuration Register */
/*! @{ */
/*! EEERDY
 *  0b0..See RAMRDY for availability of programming acceleration RAM
 *  0b1..Reserved
 */
#define FTFE_FCNFG_EEERDY_MASK                   (0x1U)
#define FTFE_FCNFG_EEERDY_SHIFT                  (0U)
#define FTFE_FCNFG_EEERDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_EEERDY_SHIFT)) & FTFE_FCNFG_EEERDY_MASK)
/*! RAMRDY - RAM Ready
 *  0b0..Programming acceleration RAM is not available
 *  0b1..Programming acceleration RAM is available
 */
#define FTFE_FCNFG_RAMRDY_MASK                   (0x2U)
#define FTFE_FCNFG_RAMRDY_SHIFT                  (1U)
#define FTFE_FCNFG_RAMRDY(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RAMRDY_SHIFT)) & FTFE_FCNFG_RAMRDY_MASK)
#define FTFE_FCNFG_PFLSH_MASK                    (0x4U)
#define FTFE_FCNFG_PFLSH_SHIFT                   (2U)
#define FTFE_FCNFG_PFLSH(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_PFLSH_SHIFT)) & FTFE_FCNFG_PFLSH_MASK)
/*! SWAP - Swap
 *  0b0..Program flash 0 block is located at relative address 0x0000
 *  0b1..Program flash 1 block is located at relative address 0x0000
 */
#define FTFE_FCNFG_SWAP_MASK                     (0x8U)
#define FTFE_FCNFG_SWAP_SHIFT                    (3U)
#define FTFE_FCNFG_SWAP(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_SWAP_SHIFT)) & FTFE_FCNFG_SWAP_MASK)
/*! ERSSUSP - Erase Suspend
 *  0b0..No suspend requested
 *  0b1..Suspend the current Erase Flash Sector command execution
 */
#define FTFE_FCNFG_ERSSUSP_MASK                  (0x10U)
#define FTFE_FCNFG_ERSSUSP_SHIFT                 (4U)
#define FTFE_FCNFG_ERSSUSP(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSSUSP_SHIFT)) & FTFE_FCNFG_ERSSUSP_MASK)
/*! ERSAREQ - Erase All Request
 *  0b0..No request or request complete
 *  0b1..Request to: run the Erase All Blocks command, verify the erased state, program the security byte in the
 *       Flash Configuration Field to the unsecure state, and release MCU security by setting the FSEC[SEC] field to
 *       the unsecure state
 */
#define FTFE_FCNFG_ERSAREQ_MASK                  (0x20U)
#define FTFE_FCNFG_ERSAREQ_SHIFT                 (5U)
#define FTFE_FCNFG_ERSAREQ(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_ERSAREQ_SHIFT)) & FTFE_FCNFG_ERSAREQ_MASK)
/*! RDCOLLIE - Read Collision Error Interrupt Enable
 *  0b0..Read collision error interrupt disabled
 *  0b1..Read collision error interrupt enabled. An interrupt request is generated whenever an FTFE read collision
 *       error is detected (see the description of FSTAT[RDCOLERR]).
 */
#define FTFE_FCNFG_RDCOLLIE_MASK                 (0x40U)
#define FTFE_FCNFG_RDCOLLIE_SHIFT                (6U)
#define FTFE_FCNFG_RDCOLLIE(x)                   (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_RDCOLLIE_SHIFT)) & FTFE_FCNFG_RDCOLLIE_MASK)
/*! CCIE - Command Complete Interrupt Enable
 *  0b0..Command complete interrupt disabled
 *  0b1..Command complete interrupt enabled. An interrupt request is generated whenever the FSTAT[CCIF] flag is set.
 */
#define FTFE_FCNFG_CCIE_MASK                     (0x80U)
#define FTFE_FCNFG_CCIE_SHIFT                    (7U)
#define FTFE_FCNFG_CCIE(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FCNFG_CCIE_SHIFT)) & FTFE_FCNFG_CCIE_MASK)
/*! @} */

/*! @name FSEC - Flash Security Register */
/*! @{ */
/*! SEC - Flash Security
 *  0b00..MCU security status is secure
 *  0b01..MCU security status is secure
 *  0b10..MCU security status is unsecure (The standard shipping condition of the FTFE is unsecure.)
 *  0b11..MCU security status is secure
 */
#define FTFE_FSEC_SEC_MASK                       (0x3U)
#define FTFE_FSEC_SEC_SHIFT                      (0U)
#define FTFE_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_SEC_SHIFT)) & FTFE_FSEC_SEC_MASK)
/*! FSLACC - Factory Security Level Access Code
 *  0b00..Factory access granted
 *  0b01..Factory access denied
 *  0b10..Factory access denied
 *  0b11..Factory access granted
 */
#define FTFE_FSEC_FSLACC_MASK                    (0xCU)
#define FTFE_FSEC_FSLACC_SHIFT                   (2U)
#define FTFE_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_FSLACC_SHIFT)) & FTFE_FSEC_FSLACC_MASK)
/*! MEEN - Mass Erase Enable Bits
 *  0b00..Mass erase is enabled
 *  0b01..Mass erase is enabled
 *  0b10..Mass erase is disabled
 *  0b11..Mass erase is enabled
 */
#define FTFE_FSEC_MEEN_MASK                      (0x30U)
#define FTFE_FSEC_MEEN_SHIFT                     (4U)
#define FTFE_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_MEEN_SHIFT)) & FTFE_FSEC_MEEN_MASK)
/*! KEYEN - Backdoor Key Security Enable
 *  0b00..Backdoor key access disabled
 *  0b01..Backdoor key access disabled (preferred KEYEN state to disable backdoor key access)
 *  0b10..Backdoor key access enabled
 *  0b11..Backdoor key access disabled
 */
#define FTFE_FSEC_KEYEN_MASK                     (0xC0U)
#define FTFE_FSEC_KEYEN_SHIFT                    (6U)
#define FTFE_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x)) << FTFE_FSEC_KEYEN_SHIFT)) & FTFE_FSEC_KEYEN_MASK)
/*! @} */

/*! @name FOPT - Flash Option Register */
/*! @{ */
#define FTFE_FOPT_OPT_MASK                       (0xFFU)
#define FTFE_FOPT_OPT_SHIFT                      (0U)
#define FTFE_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x)) << FTFE_FOPT_OPT_SHIFT)) & FTFE_FOPT_OPT_MASK)
/*! @} */

/*! @name FCCOB3 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB3_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB3_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB3_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB3_CCOBN_SHIFT)) & FTFE_FCCOB3_CCOBN_MASK)
/*! @} */

/*! @name FCCOB2 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB2_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB2_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB2_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB2_CCOBN_SHIFT)) & FTFE_FCCOB2_CCOBN_MASK)
/*! @} */

/*! @name FCCOB1 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB1_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB1_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB1_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB1_CCOBN_SHIFT)) & FTFE_FCCOB1_CCOBN_MASK)
/*! @} */

/*! @name FCCOB0 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB0_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB0_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB0_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB0_CCOBN_SHIFT)) & FTFE_FCCOB0_CCOBN_MASK)
/*! @} */

/*! @name FCCOB7 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB7_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB7_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB7_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB7_CCOBN_SHIFT)) & FTFE_FCCOB7_CCOBN_MASK)
/*! @} */

/*! @name FCCOB6 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB6_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB6_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB6_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB6_CCOBN_SHIFT)) & FTFE_FCCOB6_CCOBN_MASK)
/*! @} */

/*! @name FCCOB5 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB5_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB5_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB5_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB5_CCOBN_SHIFT)) & FTFE_FCCOB5_CCOBN_MASK)
/*! @} */

/*! @name FCCOB4 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB4_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB4_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB4_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB4_CCOBN_SHIFT)) & FTFE_FCCOB4_CCOBN_MASK)
/*! @} */

/*! @name FCCOBB - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOBB_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOBB_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOBB_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBB_CCOBN_SHIFT)) & FTFE_FCCOBB_CCOBN_MASK)
/*! @} */

/*! @name FCCOBA - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOBA_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOBA_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOBA_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOBA_CCOBN_SHIFT)) & FTFE_FCCOBA_CCOBN_MASK)
/*! @} */

/*! @name FCCOB9 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB9_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB9_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB9_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB9_CCOBN_SHIFT)) & FTFE_FCCOB9_CCOBN_MASK)
/*! @} */

/*! @name FCCOB8 - Flash Common Command Object Registers */
/*! @{ */
#define FTFE_FCCOB8_CCOBN_MASK                   (0xFFU)
#define FTFE_FCCOB8_CCOBN_SHIFT                  (0U)
#define FTFE_FCCOB8_CCOBN(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FCCOB8_CCOBN_SHIFT)) & FTFE_FCCOB8_CCOBN_MASK)
/*! @} */

/*! @name FPROT3 - Program Flash Protection Registers */
/*! @{ */
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT3_PROT_MASK                    (0xFFU)
#define FTFE_FPROT3_PROT_SHIFT                   (0U)
#define FTFE_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT3_PROT_SHIFT)) & FTFE_FPROT3_PROT_MASK)
/*! @} */

/*! @name FPROT2 - Program Flash Protection Registers */
/*! @{ */
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT2_PROT_MASK                    (0xFFU)
#define FTFE_FPROT2_PROT_SHIFT                   (0U)
#define FTFE_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT2_PROT_SHIFT)) & FTFE_FPROT2_PROT_MASK)
/*! @} */

/*! @name FPROT1 - Program Flash Protection Registers */
/*! @{ */
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT1_PROT_MASK                    (0xFFU)
#define FTFE_FPROT1_PROT_SHIFT                   (0U)
#define FTFE_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT1_PROT_SHIFT)) & FTFE_FPROT1_PROT_MASK)
/*! @} */

/*! @name FPROT0 - Program Flash Protection Registers */
/*! @{ */
/*! PROT - Program Flash Region Protect
 *  0b00000000..Program flash region is protected.
 *  0b00000001..Program flash region is not protected
 */
#define FTFE_FPROT0_PROT_MASK                    (0xFFU)
#define FTFE_FPROT0_PROT_SHIFT                   (0U)
#define FTFE_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x)) << FTFE_FPROT0_PROT_SHIFT)) & FTFE_FPROT0_PROT_MASK)
/*! @} */

/*! @name FERSTAT - Flash Error Status Register */
/*! @{ */
/*! DFDIF - Double Bit Fault Detect Interrupt Flag
 *  0b0..Double bit fault not detected during a valid flash read access from the platform flash controller
 *  0b1..Double bit fault detected (or FERCNFG[FDFD] is set) during a valid flash read access from the platform flash controller
 */
#define FTFE_FERSTAT_DFDIF_MASK                  (0x2U)
#define FTFE_FERSTAT_DFDIF_SHIFT                 (1U)
#define FTFE_FERSTAT_DFDIF(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FERSTAT_DFDIF_SHIFT)) & FTFE_FERSTAT_DFDIF_MASK)
/*! @} */

/*! @name FERCNFG - Flash Error Configuration Register */
/*! @{ */
/*! DFDIE - Double Bit Fault Detect Interrupt Enable
 *  0b0..Double bit fault detect interrupt disabled
 *  0b1..Double bit fault detect interrupt enabled. An interrupt request is generated whenever the FERSTAT[DFDIF] flag is set.
 */
#define FTFE_FERCNFG_DFDIE_MASK                  (0x2U)
#define FTFE_FERCNFG_DFDIE_SHIFT                 (1U)
#define FTFE_FERCNFG_DFDIE(x)                    (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_DFDIE_SHIFT)) & FTFE_FERCNFG_DFDIE_MASK)
/*! FDFD - Force Double Bit Fault Detect
 *  0b0..FERSTAT[DFDIF] sets only if a double bit fault is detected during read access from the platform flash controller
 *  0b1..FERSTAT[DFDIF] sets during any valid flash read access from the platform flash controller. An interrupt
 *       request is generated if the DFDIE bit is set.
 */
#define FTFE_FERCNFG_FDFD_MASK                   (0x20U)
#define FTFE_FERCNFG_FDFD_SHIFT                  (5U)
#define FTFE_FERCNFG_FDFD(x)                     (((uint8_t)(((uint8_t)(x)) << FTFE_FERCNFG_FDFD_SHIFT)) & FTFE_FERCNFG_FDFD_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group FTFE_Register_Masks */


/* FTFE - Peripheral instance base addresses */
/** Peripheral FTFE base address */
#define FTFE_BASE                                (0xE3C0u)
/** Peripheral FTFE base pointer */
#define FTFE                                     ((FTFE_Type *)FTFE_BASE)
/** Array initializer of FTFE peripheral base addresses */
#define FTFE_BASE_ADDRS                          { FTFE_BASE }
/** Array initializer of FTFE peripheral base pointers */
#define FTFE_BASE_PTRS                           { FTFE }

/*!
 * @}
 */ /* end of group FTFE_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint16_t PUR;                               /**< GPIO Pull Resistor Enable Register, offset: 0x0 */
  __IO uint16_t DR;                                /**< GPIO Data Register, offset: 0x1 */
  __IO uint16_t DDR;                               /**< GPIO Data Direction Register, offset: 0x2 */
  __IO uint16_t PER;                               /**< GPIO Peripheral Enable Register, offset: 0x3 */
  __IO uint16_t IAR;                               /**< GPIO Interrupt Assert Register, offset: 0x4 */
  __IO uint16_t IENR;                              /**< GPIO Interrupt Enable Register, offset: 0x5 */
  __IO uint16_t IPOLR;                             /**< GPIO Interrupt Polarity Register, offset: 0x6 */
  __I  uint16_t IPR;                               /**< GPIO Interrupt Pending Register, offset: 0x7 */
  __IO uint16_t IESR;                              /**< GPIO Interrupt Edge Sensitive Register, offset: 0x8 */
  __IO uint16_t PPMODE;                            /**< GPIO Push-Pull Mode Register, offset: 0x9 */
  __I  uint16_t RAWDATA;                           /**< GPIO Raw Data Register, offset: 0xA */
  __IO uint16_t DRIVE;                             /**< GPIO Drive Strength Control Register, offset: 0xB */
  __IO uint16_t PUS;                               /**< GPIO Pull Resistor Type Select, offset: 0xC */
  __IO uint16_t SRE;                               /**< Slew Rate Control Register, offset: 0xD */
} GPIO_Type;

/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/*! @name PUR - GPIO Pull Resistor Enable Register */
/*! @{ */
/*! PU - Pull Resistor Enable Bits
 *  0b0000000000000000..Pull resistor is disabled
 *  0b0000000000000001..Pull resistor is enabled
 */
#define GPIO_PUR_PU_MASK                         (0xFFFFU)
#define GPIO_PUR_PU_SHIFT                        (0U)
#define GPIO_PUR_PU(x)                           (((uint16_t)(((uint16_t)(x)) << GPIO_PUR_PU_SHIFT)) & GPIO_PUR_PU_MASK)
/*! @} */

/*! @name DR - GPIO Data Register */
/*! @{ */
#define GPIO_DR_D_MASK                           (0xFFFFU)
#define GPIO_DR_D_SHIFT                          (0U)
#define GPIO_DR_D(x)                             (((uint16_t)(((uint16_t)(x)) << GPIO_DR_D_SHIFT)) & GPIO_DR_D_MASK)
/*! @} */

/*! @name DDR - GPIO Data Direction Register */
/*! @{ */
/*! DD - Data Direction Bits
 *  0b0000000000000000..Pin is an input
 *  0b0000000000000001..Pin is an output
 */
#define GPIO_DDR_DD_MASK                         (0xFFFFU)
#define GPIO_DDR_DD_SHIFT                        (0U)
#define GPIO_DDR_DD(x)                           (((uint16_t)(((uint16_t)(x)) << GPIO_DDR_DD_SHIFT)) & GPIO_DDR_DD_MASK)
/*! @} */

/*! @name PER - GPIO Peripheral Enable Register */
/*! @{ */
/*! PE - Peripheral Enable Bits
 *  0b0000000000000000..Pin is for GPIO (GPIO mode)
 *  0b0000000000000001..Pin is for peripheral (peripheral mode)
 */
#define GPIO_PER_PE_MASK                         (0xFFFFU)
#define GPIO_PER_PE_SHIFT                        (0U)
#define GPIO_PER_PE(x)                           (((uint16_t)(((uint16_t)(x)) << GPIO_PER_PE_SHIFT)) & GPIO_PER_PE_MASK)
/*! @} */

/*! @name IAR - GPIO Interrupt Assert Register */
/*! @{ */
/*! IA - Interrupt Assert Bits
 *  0b0000000000000000..Deassert software interrupt
 *  0b0000000000000001..Assert software interrupt
 */
#define GPIO_IAR_IA_MASK                         (0xFFFFU)
#define GPIO_IAR_IA_SHIFT                        (0U)
#define GPIO_IAR_IA(x)                           (((uint16_t)(((uint16_t)(x)) << GPIO_IAR_IA_SHIFT)) & GPIO_IAR_IA_MASK)
/*! @} */

/*! @name IENR - GPIO Interrupt Enable Register */
/*! @{ */
/*! IEN - Interrupt Enable Bits
 *  0b0000000000000000..External Interrupt is disabled
 *  0b0000000000000001..External Interrupt is enabled
 */
#define GPIO_IENR_IEN_MASK                       (0xFFFFU)
#define GPIO_IENR_IEN_SHIFT                      (0U)
#define GPIO_IENR_IEN(x)                         (((uint16_t)(((uint16_t)(x)) << GPIO_IENR_IEN_SHIFT)) & GPIO_IENR_IEN_MASK)
/*! @} */

/*! @name IPOLR - GPIO Interrupt Polarity Register */
/*! @{ */
/*! IPOL - Interrupt Polarity Bits
 *  0b0000000000000000..Interrupt occurred on rising edge
 *  0b0000000000000001..Interrupt occurred on falling edge
 */
#define GPIO_IPOLR_IPOL_MASK                     (0xFFFFU)
#define GPIO_IPOLR_IPOL_SHIFT                    (0U)
#define GPIO_IPOLR_IPOL(x)                       (((uint16_t)(((uint16_t)(x)) << GPIO_IPOLR_IPOL_SHIFT)) & GPIO_IPOLR_IPOL_MASK)
/*! @} */

/*! @name IPR - GPIO Interrupt Pending Register */
/*! @{ */
/*! IP - Interrupt Pending Bits
 *  0b0000000000000000..No Interrupt
 *  0b0000000000000001..Interrupt occurred
 */
#define GPIO_IPR_IP_MASK                         (0xFFFFU)
#define GPIO_IPR_IP_SHIFT                        (0U)
#define GPIO_IPR_IP(x)                           (((uint16_t)(((uint16_t)(x)) << GPIO_IPR_IP_SHIFT)) & GPIO_IPR_IP_MASK)
/*! @} */

/*! @name IESR - GPIO Interrupt Edge Sensitive Register */
/*! @{ */
/*! IES - Interrupt Edge-Sensitive Bits
 *  0b0000000000000000..No edge detected if read; no effect if writing
 *  0b0000000000000001..An edge detected if read; clear corresponding Interrupt Pending bit if writing
 */
#define GPIO_IESR_IES_MASK                       (0xFFFFU)
#define GPIO_IESR_IES_SHIFT                      (0U)
#define GPIO_IESR_IES(x)                         (((uint16_t)(((uint16_t)(x)) << GPIO_IESR_IES_SHIFT)) & GPIO_IESR_IES_MASK)
/*! @} */

/*! @name PPMODE - GPIO Push-Pull Mode Register */
/*! @{ */
/*! PPMODE - Push-Pull Mode Bits
 *  0b0000000000000000..Open Drain Mode
 *  0b0000000000000001..Push-Pull Mode
 */
#define GPIO_PPMODE_PPMODE_MASK                  (0xFFFFU)
#define GPIO_PPMODE_PPMODE_SHIFT                 (0U)
#define GPIO_PPMODE_PPMODE(x)                    (((uint16_t)(((uint16_t)(x)) << GPIO_PPMODE_PPMODE_SHIFT)) & GPIO_PPMODE_PPMODE_MASK)
/*! @} */

/*! @name RAWDATA - GPIO Raw Data Register */
/*! @{ */
#define GPIO_RAWDATA_RAW_DATA_MASK               (0xFFFFU)
#define GPIO_RAWDATA_RAW_DATA_SHIFT              (0U)
#define GPIO_RAWDATA_RAW_DATA(x)                 (((uint16_t)(((uint16_t)(x)) << GPIO_RAWDATA_RAW_DATA_SHIFT)) & GPIO_RAWDATA_RAW_DATA_MASK)
/*! @} */

/*! @name DRIVE - GPIO Drive Strength Control Register */
/*! @{ */
/*! DRIVE - Drive Strength Selector Bits
 *  0b0000000000000000..Low drive strength
 *  0b0000000000000001..High drive strength
 */
#define GPIO_DRIVE_DRIVE_MASK                    (0xFFFFU)
#define GPIO_DRIVE_DRIVE_SHIFT                   (0U)
#define GPIO_DRIVE_DRIVE(x)                      (((uint16_t)(((uint16_t)(x)) << GPIO_DRIVE_DRIVE_SHIFT)) & GPIO_DRIVE_DRIVE_MASK)
/*! @} */

/*! @name PUS - GPIO Pull Resistor Type Select */
/*! @{ */
/*! PUS - Pull Resistor Type Select Bits
 *  0b0000000000000000..Pulldown resistor
 *  0b0000000000000001..Pullup resistor
 */
#define GPIO_PUS_PUS_MASK                        (0xFFFFU)
#define GPIO_PUS_PUS_SHIFT                       (0U)
#define GPIO_PUS_PUS(x)                          (((uint16_t)(((uint16_t)(x)) << GPIO_PUS_PUS_SHIFT)) & GPIO_PUS_PUS_MASK)
/*! @} */

/*! @name SRE - Slew Rate Control Register */
/*! @{ */
/*! SRE - Slew Rate Enable
 *  0b0000000000000000..Slew rate is enabled (the turn-on time of the output transistor is faster)
 *  0b0000000000000001..Slew rate is disabled (the turn-on time of the output transistor is slower)
 */
#define GPIO_SRE_SRE_MASK                        (0xFFFFU)
#define GPIO_SRE_SRE_SHIFT                       (0U)
#define GPIO_SRE_SRE(x)                          (((uint16_t)(((uint16_t)(x)) << GPIO_SRE_SRE_SHIFT)) & GPIO_SRE_SRE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral GPIOA base address */
#define GPIOA_BASE                               (0xE200u)
/** Peripheral GPIOA base pointer */
#define GPIOA                                    ((GPIO_Type *)GPIOA_BASE)
/** Peripheral GPIOB base address */
#define GPIOB_BASE                               (0xE210u)
/** Peripheral GPIOB base pointer */
#define GPIOB                                    ((GPIO_Type *)GPIOB_BASE)
/** Peripheral GPIOC base address */
#define GPIOC_BASE                               (0xE220u)
/** Peripheral GPIOC base pointer */
#define GPIOC                                    ((GPIO_Type *)GPIOC_BASE)
/** Peripheral GPIOD base address */
#define GPIOD_BASE                               (0xE230u)
/** Peripheral GPIOD base pointer */
#define GPIOD                                    ((GPIO_Type *)GPIOD_BASE)
/** Peripheral GPIOE base address */
#define GPIOE_BASE                               (0xE240u)
/** Peripheral GPIOE base pointer */
#define GPIOE                                    ((GPIO_Type *)GPIOE_BASE)
/** Peripheral GPIOE base address */
#define GPIOF_BASE                               (0xE250u)
/** Peripheral GPIOE base pointer */
#define GPIOF                                    ((GPIO_Type *)GPIOF_BASE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE, GPIOF_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF }

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct {
  __IO uint16_t A1;                                /**< I2C Address Register 1, offset: 0x0 */
  __IO uint16_t F;                                 /**< I2C Frequency Divider register, offset: 0x1 */
  __IO uint16_t C1;                                /**< I2C Control Register 1, offset: 0x2 */
  __IO uint16_t S;                                 /**< I2C Status register, offset: 0x3 */
  __IO uint16_t D;                                 /**< I2C Data I/O register, offset: 0x4 */
  __IO uint16_t C2;                                /**< I2C Control Register 2, offset: 0x5 */
  __IO uint16_t FLT;                               /**< I2C Programmable Input Glitch Filter Register, offset: 0x6 */
  __IO uint16_t RA;                                /**< I2C Range Address register, offset: 0x7 */
  __IO uint16_t SMB;                               /**< I2C SMBus Control and Status register, offset: 0x8 */
  __IO uint16_t A2;                                /**< I2C Address Register 2, offset: 0x9 */
  __IO uint16_t SLTH;                              /**< I2C SCL Low Timeout Register High, offset: 0xA */
  __IO uint16_t SLTL;                              /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_Type;

/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/*! @name A1 - I2C Address Register 1 */
/*! @{ */
#define I2C_A1_AD_MASK                           (0xFEU)
#define I2C_A1_AD_SHIFT                          (1U)
#define I2C_A1_AD(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_A1_AD_SHIFT)) & I2C_A1_AD_MASK)
/*! @} */

/*! @name F - I2C Frequency Divider register */
/*! @{ */
#define I2C_F_ICR_MASK                           (0x3FU)
#define I2C_F_ICR_SHIFT                          (0U)
#define I2C_F_ICR(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_F_ICR_SHIFT)) & I2C_F_ICR_MASK)
/*! MULT - Multiplier Factor
 *  0b00..mul = 1
 *  0b01..mul = 2
 *  0b10..mul = 4
 *  0b11..Reserved
 */
#define I2C_F_MULT_MASK                          (0xC0U)
#define I2C_F_MULT_SHIFT                         (6U)
#define I2C_F_MULT(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_F_MULT_SHIFT)) & I2C_F_MULT_MASK)
/*! @} */

/*! @name C1 - I2C Control Register 1 */
/*! @{ */
/*! DMAEN - DMA Enable
 *  0b0..All DMA signalling disabled.
 *  0b1..DMA transfer is enabled. While SMB[FACK] = 0, the following conditions trigger the DMA request: a data
 *       byte is received, and either address or data is transmitted. (ACK/NACK is automatic) the first byte received
 *       matches the A1 register or is a general call address. If any address matching occurs, S[IAAS] and S[TCF]
 *       are set. If the direction of transfer is known from master to slave, then it is not required to check
 *       S[SRW]. With this assumption, DMA can also be used in this case. In other cases, if the master reads data from
 *       the slave, then it is required to rewrite the C1 register operation. With this assumption, DMA cannot be
 *       used. When FACK = 1, an address or a data byte is transmitted.
 */
#define I2C_C1_DMAEN_MASK                        (0x1U)
#define I2C_C1_DMAEN_SHIFT                       (0U)
#define I2C_C1_DMAEN(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_C1_DMAEN_SHIFT)) & I2C_C1_DMAEN_MASK)
/*! WUEN - Wakeup Enable
 *  0b0..Normal operation. No interrupt generated when address matching in low power mode.
 *  0b1..Enables the wakeup function in low power mode.
 */
#define I2C_C1_WUEN_MASK                         (0x2U)
#define I2C_C1_WUEN_SHIFT                        (1U)
#define I2C_C1_WUEN(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_C1_WUEN_SHIFT)) & I2C_C1_WUEN_MASK)
#define I2C_C1_RSTA_MASK                         (0x4U)
#define I2C_C1_RSTA_SHIFT                        (2U)
#define I2C_C1_RSTA(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_C1_RSTA_SHIFT)) & I2C_C1_RSTA_MASK)
/*! TXAK - Transmit Acknowledge Enable
 *  0b0..An acknowledge signal is sent to the bus on the following receiving byte (if FACK is cleared) or the
 *       current receiving byte (if FACK is set).
 *  0b1..No acknowledge signal is sent to the bus on the following receiving data byte (if FACK is cleared) or the
 *       current receiving data byte (if FACK is set).
 */
#define I2C_C1_TXAK_MASK                         (0x8U)
#define I2C_C1_TXAK_SHIFT                        (3U)
#define I2C_C1_TXAK(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_C1_TXAK_SHIFT)) & I2C_C1_TXAK_MASK)
/*! TX - Transmit Mode Select
 *  0b0..Receive
 *  0b1..Transmit
 */
#define I2C_C1_TX_MASK                           (0x10U)
#define I2C_C1_TX_SHIFT                          (4U)
#define I2C_C1_TX(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_C1_TX_SHIFT)) & I2C_C1_TX_MASK)
/*! MST - Master Mode Select
 *  0b0..Slave mode
 *  0b1..Master mode
 */
#define I2C_C1_MST_MASK                          (0x20U)
#define I2C_C1_MST_SHIFT                         (5U)
#define I2C_C1_MST(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_C1_MST_SHIFT)) & I2C_C1_MST_MASK)
/*! IICIE - I2C Interrupt Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C1_IICIE_MASK                        (0x40U)
#define I2C_C1_IICIE_SHIFT                       (6U)
#define I2C_C1_IICIE(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_C1_IICIE_SHIFT)) & I2C_C1_IICIE_MASK)
/*! IICEN - I2C Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C1_IICEN_MASK                        (0x80U)
#define I2C_C1_IICEN_SHIFT                       (7U)
#define I2C_C1_IICEN(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_C1_IICEN_SHIFT)) & I2C_C1_IICEN_MASK)
/*! @} */

/*! @name S - I2C Status register */
/*! @{ */
/*! RXAK - Receive Acknowledge
 *  0b0..Acknowledge signal was received after the completion of one byte of data transmission on the bus
 *  0b1..No acknowledge signal detected
 */
#define I2C_S_RXAK_MASK                          (0x1U)
#define I2C_S_RXAK_SHIFT                         (0U)
#define I2C_S_RXAK(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_S_RXAK_SHIFT)) & I2C_S_RXAK_MASK)
/*! IICIF - Interrupt Flag
 *  0b0..No interrupt pending
 *  0b1..Interrupt pending
 */
#define I2C_S_IICIF_MASK                         (0x2U)
#define I2C_S_IICIF_SHIFT                        (1U)
#define I2C_S_IICIF(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_S_IICIF_SHIFT)) & I2C_S_IICIF_MASK)
/*! SRW - Slave Read/Write
 *  0b0..Slave receive, master writing to slave
 *  0b1..Slave transmit, master reading from slave
 */
#define I2C_S_SRW_MASK                           (0x4U)
#define I2C_S_SRW_SHIFT                          (2U)
#define I2C_S_SRW(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_S_SRW_SHIFT)) & I2C_S_SRW_MASK)
/*! RAM - Range Address Match
 *  0b0..Not addressed
 *  0b1..Addressed as a slave
 */
#define I2C_S_RAM_MASK                           (0x8U)
#define I2C_S_RAM_SHIFT                          (3U)
#define I2C_S_RAM(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_S_RAM_SHIFT)) & I2C_S_RAM_MASK)
/*! ARBL - Arbitration Lost
 *  0b0..Standard bus operation.
 *  0b1..Loss of arbitration.
 */
#define I2C_S_ARBL_MASK                          (0x10U)
#define I2C_S_ARBL_SHIFT                         (4U)
#define I2C_S_ARBL(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_S_ARBL_SHIFT)) & I2C_S_ARBL_MASK)
/*! BUSY - Bus Busy
 *  0b0..Bus is idle
 *  0b1..Bus is busy
 */
#define I2C_S_BUSY_MASK                          (0x20U)
#define I2C_S_BUSY_SHIFT                         (5U)
#define I2C_S_BUSY(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_S_BUSY_SHIFT)) & I2C_S_BUSY_MASK)
/*! IAAS - Addressed As A Slave
 *  0b0..Not addressed
 *  0b1..Addressed as a slave
 */
#define I2C_S_IAAS_MASK                          (0x40U)
#define I2C_S_IAAS_SHIFT                         (6U)
#define I2C_S_IAAS(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_S_IAAS_SHIFT)) & I2C_S_IAAS_MASK)
/*! TCF - Transfer Complete Flag
 *  0b0..Transfer in progress
 *  0b1..Transfer complete
 */
#define I2C_S_TCF_MASK                           (0x80U)
#define I2C_S_TCF_SHIFT                          (7U)
#define I2C_S_TCF(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_S_TCF_SHIFT)) & I2C_S_TCF_MASK)
/*! @} */

/*! @name D - I2C Data I/O register */
/*! @{ */
#define I2C_D_DATA_MASK                          (0xFFU)
#define I2C_D_DATA_SHIFT                         (0U)
#define I2C_D_DATA(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_D_DATA_SHIFT)) & I2C_D_DATA_MASK)
/*! @} */

/*! @name C2 - I2C Control Register 2 */
/*! @{ */
#define I2C_C2_AD_MASK                           (0x7U)
#define I2C_C2_AD_SHIFT                          (0U)
#define I2C_C2_AD(x)                             (((uint16_t)(((uint16_t)(x)) << I2C_C2_AD_SHIFT)) & I2C_C2_AD_MASK)
/*! RMEN - Range Address Matching Enable
 *  0b0..Range mode disabled. No address matching occurs for an address within the range of values of the A1 and RA registers.
 *  0b1..Range mode enabled. Address matching occurs when a slave receives an address within the range of values of the A1 and RA registers.
 */
#define I2C_C2_RMEN_MASK                         (0x8U)
#define I2C_C2_RMEN_SHIFT                        (3U)
#define I2C_C2_RMEN(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_C2_RMEN_SHIFT)) & I2C_C2_RMEN_MASK)
/*! HDRS - High Drive Select
 *  0b0..Normal drive mode
 *  0b1..High drive mode
 */
#define I2C_C2_HDRS_MASK                         (0x20U)
#define I2C_C2_HDRS_SHIFT                        (5U)
#define I2C_C2_HDRS(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_C2_HDRS_SHIFT)) & I2C_C2_HDRS_MASK)
/*! ADEXT - Address Extension
 *  0b0..7-bit address scheme
 *  0b1..10-bit address scheme
 */
#define I2C_C2_ADEXT_MASK                        (0x40U)
#define I2C_C2_ADEXT_SHIFT                       (6U)
#define I2C_C2_ADEXT(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_C2_ADEXT_SHIFT)) & I2C_C2_ADEXT_MASK)
/*! GCAEN - General Call Address Enable
 *  0b0..Disabled
 *  0b1..Enabled
 */
#define I2C_C2_GCAEN_MASK                        (0x80U)
#define I2C_C2_GCAEN_SHIFT                       (7U)
#define I2C_C2_GCAEN(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_C2_GCAEN_SHIFT)) & I2C_C2_GCAEN_MASK)
/*! @} */

/*! @name FLT - I2C Programmable Input Glitch Filter Register */
/*! @{ */
/*! FLT - I2C Programmable Filter Factor
 *  0b0000..No filter/bypass
 */
#define I2C_FLT_FLT_MASK                         (0xFU)
#define I2C_FLT_FLT_SHIFT                        (0U)
#define I2C_FLT_FLT(x)                           (((uint16_t)(((uint16_t)(x)) << I2C_FLT_FLT_SHIFT)) & I2C_FLT_FLT_MASK)
/*! STARTF - I2C Bus Start Detect Flag
 *  0b0..No start happens on I2C bus
 *  0b1..Start detected on I2C bus
 */
#define I2C_FLT_STARTF_MASK                      (0x10U)
#define I2C_FLT_STARTF_SHIFT                     (4U)
#define I2C_FLT_STARTF(x)                        (((uint16_t)(((uint16_t)(x)) << I2C_FLT_STARTF_SHIFT)) & I2C_FLT_STARTF_MASK)
/*! SSIE - I2C Bus Stop or Start Interrupt Enable
 *  0b0..Stop or start detection interrupt is disabled
 *  0b1..Stop or start detection interrupt is enabled
 */
#define I2C_FLT_SSIE_MASK                        (0x20U)
#define I2C_FLT_SSIE_SHIFT                       (5U)
#define I2C_FLT_SSIE(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_FLT_SSIE_SHIFT)) & I2C_FLT_SSIE_MASK)
/*! STOPF - I2C Bus Stop Detect Flag
 *  0b0..No stop happens on I2C bus
 *  0b1..Stop detected on I2C bus
 */
#define I2C_FLT_STOPF_MASK                       (0x40U)
#define I2C_FLT_STOPF_SHIFT                      (6U)
#define I2C_FLT_STOPF(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_FLT_STOPF_SHIFT)) & I2C_FLT_STOPF_MASK)
/*! SHEN - Stop Hold Enable
 *  0b0..Stop holdoff is disabled. The MCU's entry to stop mode is not gated.
 *  0b1..Stop holdoff is enabled.
 */
#define I2C_FLT_SHEN_MASK                        (0x80U)
#define I2C_FLT_SHEN_SHIFT                       (7U)
#define I2C_FLT_SHEN(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_FLT_SHEN_SHIFT)) & I2C_FLT_SHEN_MASK)
/*! @} */

/*! @name RA - I2C Range Address register */
/*! @{ */
#define I2C_RA_RAD_MASK                          (0xFEU)
#define I2C_RA_RAD_SHIFT                         (1U)
#define I2C_RA_RAD(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_RA_RAD_SHIFT)) & I2C_RA_RAD_MASK)
/*! @} */

/*! @name SMB - I2C SMBus Control and Status register */
/*! @{ */
/*! SHTF2IE - SHTF2 Interrupt Enable
 *  0b0..SHTF2 interrupt is disabled
 *  0b1..SHTF2 interrupt is enabled
 */
#define I2C_SMB_SHTF2IE_MASK                     (0x1U)
#define I2C_SMB_SHTF2IE_SHIFT                    (0U)
#define I2C_SMB_SHTF2IE(x)                       (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF2IE_SHIFT)) & I2C_SMB_SHTF2IE_MASK)
/*! SHTF2 - SCL High Timeout Flag 2
 *  0b0..No SCL high and SDA low timeout occurs
 *  0b1..SCL high and SDA low timeout occurs
 */
#define I2C_SMB_SHTF2_MASK                       (0x2U)
#define I2C_SMB_SHTF2_SHIFT                      (1U)
#define I2C_SMB_SHTF2(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF2_SHIFT)) & I2C_SMB_SHTF2_MASK)
/*! SHTF1 - SCL High Timeout Flag 1
 *  0b0..No SCL high and SDA high timeout occurs
 *  0b1..SCL high and SDA high timeout occurs
 */
#define I2C_SMB_SHTF1_MASK                       (0x4U)
#define I2C_SMB_SHTF1_SHIFT                      (2U)
#define I2C_SMB_SHTF1(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SHTF1_SHIFT)) & I2C_SMB_SHTF1_MASK)
/*! SLTF - SCL Low Timeout Flag
 *  0b0..No low timeout occurs
 *  0b1..Low timeout occurs
 */
#define I2C_SMB_SLTF_MASK                        (0x8U)
#define I2C_SMB_SLTF_SHIFT                       (3U)
#define I2C_SMB_SLTF(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SLTF_SHIFT)) & I2C_SMB_SLTF_MASK)
/*! TCKSEL - Timeout Counter Clock Select
 *  0b0..Timeout counter counts at the frequency of the I2C module clock / 64
 *  0b1..Timeout counter counts at the frequency of the I2C module clock
 */
#define I2C_SMB_TCKSEL_MASK                      (0x10U)
#define I2C_SMB_TCKSEL_SHIFT                     (4U)
#define I2C_SMB_TCKSEL(x)                        (((uint16_t)(((uint16_t)(x)) << I2C_SMB_TCKSEL_SHIFT)) & I2C_SMB_TCKSEL_MASK)
/*! SIICAEN - Second I2C Address Enable
 *  0b0..I2C address register 2 matching is disabled
 *  0b1..I2C address register 2 matching is enabled
 */
#define I2C_SMB_SIICAEN_MASK                     (0x20U)
#define I2C_SMB_SIICAEN_SHIFT                    (5U)
#define I2C_SMB_SIICAEN(x)                       (((uint16_t)(((uint16_t)(x)) << I2C_SMB_SIICAEN_SHIFT)) & I2C_SMB_SIICAEN_MASK)
/*! ALERTEN - SMBus Alert Response Address Enable
 *  0b0..SMBus alert response address matching is disabled
 *  0b1..SMBus alert response address matching is enabled
 */
#define I2C_SMB_ALERTEN_MASK                     (0x40U)
#define I2C_SMB_ALERTEN_SHIFT                    (6U)
#define I2C_SMB_ALERTEN(x)                       (((uint16_t)(((uint16_t)(x)) << I2C_SMB_ALERTEN_SHIFT)) & I2C_SMB_ALERTEN_MASK)
/*! FACK - Fast NACK/ACK Enable
 *  0b0..An ACK or NACK is sent on the following receiving data byte
 *  0b1..Writing 0 to TXAK after receiving a data byte generates an ACK. Writing 1 to TXAK after receiving a data byte generates a NACK.
 */
#define I2C_SMB_FACK_MASK                        (0x80U)
#define I2C_SMB_FACK_SHIFT                       (7U)
#define I2C_SMB_FACK(x)                          (((uint16_t)(((uint16_t)(x)) << I2C_SMB_FACK_SHIFT)) & I2C_SMB_FACK_MASK)
/*! @} */

/*! @name A2 - I2C Address Register 2 */
/*! @{ */
#define I2C_A2_SAD_MASK                          (0xFEU)
#define I2C_A2_SAD_SHIFT                         (1U)
#define I2C_A2_SAD(x)                            (((uint16_t)(((uint16_t)(x)) << I2C_A2_SAD_SHIFT)) & I2C_A2_SAD_MASK)
/*! @} */

/*! @name SLTH - I2C SCL Low Timeout Register High */
/*! @{ */
#define I2C_SLTH_SSLT_MASK                       (0xFFU)
#define I2C_SLTH_SSLT_SHIFT                      (0U)
#define I2C_SLTH_SSLT(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_SLTH_SSLT_SHIFT)) & I2C_SLTH_SSLT_MASK)
/*! @} */

/*! @name SLTL - I2C SCL Low Timeout Register Low */
/*! @{ */
#define I2C_SLTL_SSLT_MASK                       (0xFFU)
#define I2C_SLTL_SSLT_SHIFT                      (0U)
#define I2C_SLTL_SSLT(x)                         (((uint16_t)(((uint16_t)(x)) << I2C_SLTL_SSLT_SHIFT)) & I2C_SLTL_SSLT_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE                                (0xE0E0u)
/** Peripheral I2C0 base pointer */
#define I2C0                                     ((I2C_Type *)I2C0_BASE)
/** Peripheral I2C1 base address */
#define I2C1_BASE                                (0xE0F0u)
/** Peripheral I2C1 base pointer */
#define I2C1                                     ((I2C_Type *)I2C1_BASE)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE, I2C1_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0, I2C1 }

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- INTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Peripheral_Access_Layer INTC Peripheral Access Layer
 * @{
 */

/** INTC - Register Layout Typedef */
typedef struct {
  __IO uint16_t IPR0;                              /**< Interrupt Priority Register 0, offset: 0x0 */
  __IO uint16_t IPR1;                              /**< Interrupt Priority Register 1, offset: 0x1 */
  __IO uint16_t IPR2;                              /**< Interrupt Priority Register 2, offset: 0x2 */
  __IO uint16_t IPR3;                              /**< Interrupt Priority Register 3, offset: 0x3 */
  __IO uint16_t IPR4;                              /**< Interrupt Priority Register 4, offset: 0x4 */
  __IO uint16_t IPR5;                              /**< Interrupt Priority Register 5, offset: 0x5 */
  __IO uint16_t IPR6;                              /**< Interrupt Priority Register 6, offset: 0x6 */
  __IO uint16_t IPR7;                              /**< Interrupt Priority Register 7, offset: 0x7 */
  __IO uint16_t IPR8;                              /**< Interrupt Priority Register 8, offset: 0x8 */
  __IO uint16_t IPR9;                              /**< Interrupt Priority Register 9, offset: 0x9 */
  __IO uint16_t IPR10;                             /**< Interrupt Priority Register 10, offset: 0xA */
  __IO uint16_t IPR11;                             /**< Interrupt Priority Register 11, offset: 0xB */
  __IO uint16_t IPR12;                             /**< Interrupt Priority Register 12, offset: 0xC */
  __IO uint16_t VBA;                               /**< Vector Base Address Register, offset: 0xD */
  __IO uint16_t FIM0;                              /**< Fast Interrupt 0 Match Register, offset: 0xE */
  __IO uint16_t FIVAL0;                            /**< Fast Interrupt 0 Vector Address Low Register, offset: 0xF */
  __IO uint16_t FIVAH0;                            /**< Fast Interrupt 0 Vector Address High Register, offset: 0x10 */
  __IO uint16_t FIM1;                              /**< Fast Interrupt 1 Match Register, offset: 0x11 */
  __IO uint16_t FIVAL1;                            /**< Fast Interrupt 1 Vector Address Low Register, offset: 0x12 */
  __IO uint16_t FIVAH1;                            /**< Fast Interrupt 1 Vector Address High Register, offset: 0x13 */
  __I  uint16_t IRQP0;                             /**< IRQ Pending Register 0, offset: 0x14 */
  __I  uint16_t IRQP1;                             /**< IRQ Pending Register 1, offset: 0x15 */
  __I  uint16_t IRQP2;                             /**< IRQ Pending Register 2, offset: 0x16 */
  __I  uint16_t IRQP3;                             /**< IRQ Pending Register 3, offset: 0x17 */
  __I  uint16_t IRQP4;                             /**< IRQ Pending Register 4, offset: 0x18 */
  __I  uint16_t IRQP5;                             /**< IRQ Pending Register 5, offset: 0x19 */
  __I  uint16_t IRQP6;                             /**< IRQ Pending Register 6, offset: 0x1A */
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x1B */
} INTC_Type;

/* ----------------------------------------------------------------------------
   -- INTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup INTC_Register_Masks INTC Register Masks
 * @{
 */

/*! @name IPR0 - Interrupt Priority Register 0 */
/*! @{ */
/*! STPCNT - EOnCE Step Counter Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_STPCNT_MASK                    (0x3U)
#define INTC_IPR0_STPCNT_SHIFT                   (0U)
#define INTC_IPR0_STPCNT(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_STPCNT_SHIFT)) & INTC_IPR0_STPCNT_MASK)
/*! BKPT - EOnCE Breakpoint Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_BKPT_MASK                      (0xCU)
#define INTC_IPR0_BKPT_SHIFT                     (2U)
#define INTC_IPR0_BKPT(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_BKPT_SHIFT)) & INTC_IPR0_BKPT_MASK)
/*! TRBUF - EOnCE Trace Buffer Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_TRBUF_MASK                     (0x30U)
#define INTC_IPR0_TRBUF_SHIFT                    (4U)
#define INTC_IPR0_TRBUF(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_TRBUF_SHIFT)) & INTC_IPR0_TRBUF_MASK)
/*! TX_REG - EOnCE Transmit Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_TX_REG_MASK                    (0xC0U)
#define INTC_IPR0_TX_REG_SHIFT                   (6U)
#define INTC_IPR0_TX_REG(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_TX_REG_SHIFT)) & INTC_IPR0_TX_REG_MASK)
/*! RX_REG - EOnCE Receive Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_RX_REG_MASK                    (0x300U)
#define INTC_IPR0_RX_REG_SHIFT                   (8U)
#define INTC_IPR0_RX_REG(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_RX_REG_SHIFT)) & INTC_IPR0_RX_REG_MASK)
/*! BUS_ERR - Bus Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR0_BUS_ERR_MASK                   (0xC00U)
#define INTC_IPR0_BUS_ERR_SHIFT                  (10U)
#define INTC_IPR0_BUS_ERR(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_IPR0_BUS_ERR_SHIFT)) & INTC_IPR0_BUS_ERR_MASK)
/*! @} */

/*! @name IPR1 - Interrupt Priority Register 1 */
/*! @{ */
/*! XBARA - Inter-Peripheral Crossbar Switch A (XBARA) Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_XBARA_MASK                     (0xCU)
#define INTC_IPR1_XBARA_SHIFT                    (2U)
#define INTC_IPR1_XBARA(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_XBARA_SHIFT)) & INTC_IPR1_XBARA_MASK)
/*! LVI1 - Low Voltage Detector Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_LVI1_MASK                      (0x30U)
#define INTC_IPR1_LVI1_SHIFT                     (4U)
#define INTC_IPR1_LVI1(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_LVI1_SHIFT)) & INTC_IPR1_LVI1_MASK)
/*! OCCS - PLL Loss of Reference or Change in Lock Status Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 1
 *  0b10..IRQ Priority Level 2
 *  0b11..IRQ Priority Level 3
 */
#define INTC_IPR1_OCCS_MASK                      (0xC0U)
#define INTC_IPR1_OCCS_SHIFT                     (6U)
#define INTC_IPR1_OCCS(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_OCCS_SHIFT)) & INTC_IPR1_OCCS_MASK)
/*! TMRB_3 - Timer B Channel 3 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_3_MASK                    (0x300U)
#define INTC_IPR1_TMRB_3_SHIFT                   (8U)
#define INTC_IPR1_TMRB_3(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_3_SHIFT)) & INTC_IPR1_TMRB_3_MASK)
/*! TMRB_2 - Timer B Channel 2 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_2_MASK                    (0xC00U)
#define INTC_IPR1_TMRB_2_SHIFT                   (10U)
#define INTC_IPR1_TMRB_2(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_2_SHIFT)) & INTC_IPR1_TMRB_2_MASK)
/*! TMRB_1 - Timer B Channel 1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_1_MASK                    (0x3000U)
#define INTC_IPR1_TMRB_1_SHIFT                   (12U)
#define INTC_IPR1_TMRB_1(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_1_SHIFT)) & INTC_IPR1_TMRB_1_MASK)
/*! TMRB_0 - Timer B Channel 0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR1_TMRB_0_MASK                    (0xC000U)
#define INTC_IPR1_TMRB_0_SHIFT                   (14U)
#define INTC_IPR1_TMRB_0(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR1_TMRB_0_SHIFT)) & INTC_IPR1_TMRB_0_MASK)
/*! @} */

/*! @name IPR2 - Interrupt Priority Register 2 */
/*! @{ */
/*! TMRA_3 - Timer A Channel 3 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_3_MASK                    (0x3U)
#define INTC_IPR2_TMRA_3_SHIFT                   (0U)
#define INTC_IPR2_TMRA_3(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_3_SHIFT)) & INTC_IPR2_TMRA_3_MASK)
/*! TMRA_2 - Timer A Channel 2 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_2_MASK                    (0xCU)
#define INTC_IPR2_TMRA_2_SHIFT                   (2U)
#define INTC_IPR2_TMRA_2(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_2_SHIFT)) & INTC_IPR2_TMRA_2_MASK)
/*! TMRA_1 - Timer A Channel 1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_1_MASK                    (0x30U)
#define INTC_IPR2_TMRA_1_SHIFT                   (4U)
#define INTC_IPR2_TMRA_1(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_1_SHIFT)) & INTC_IPR2_TMRA_1_MASK)
/*! TMRA_0 - Timer A Channel 0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_TMRA_0_MASK                    (0xC0U)
#define INTC_IPR2_TMRA_0_SHIFT                   (6U)
#define INTC_IPR2_TMRA_0(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_TMRA_0_SHIFT)) & INTC_IPR2_TMRA_0_MASK)
/*! ADC_CC1 - ADC_CYC Conversion Complete Interrupt Priority Level (converter B in non-simultaneous parallel scan mode)
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_CC1_MASK                   (0x300U)
#define INTC_IPR2_ADC_CC1_SHIFT                  (8U)
#define INTC_IPR2_ADC_CC1(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_CC1_SHIFT)) & INTC_IPR2_ADC_CC1_MASK)
/*! ADC_CC0 - ADC_CYC Conversion Complete Interrupt Priority Level (any scan type except converter B
 *    in non-simultaneous parallel scan mode)
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_CC0_MASK                   (0xC00U)
#define INTC_IPR2_ADC_CC0_SHIFT                  (10U)
#define INTC_IPR2_ADC_CC0(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_CC0_SHIFT)) & INTC_IPR2_ADC_CC0_MASK)
/*! ADC_ERR - ADC_CYC Zero Crossing, High Limit, or Low Limit Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_ADC_ERR_MASK                   (0x3000U)
#define INTC_IPR2_ADC_ERR_SHIFT                  (12U)
#define INTC_IPR2_ADC_ERR(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_ADC_ERR_SHIFT)) & INTC_IPR2_ADC_ERR_MASK)
/*! DMA_ERR - DMA Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR2_DMA_ERR_MASK                   (0xC000U)
#define INTC_IPR2_DMA_ERR_SHIFT                  (14U)
#define INTC_IPR2_DMA_ERR(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_IPR2_DMA_ERR_SHIFT)) & INTC_IPR2_DMA_ERR_MASK)
/*! @} */

/*! @name IPR3 - Interrupt Priority Register 3 */
/*! @{ */
/*! DMACH3 - DMA Channel 3 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH3_MASK                    (0x3U)
#define INTC_IPR3_DMACH3_SHIFT                   (0U)
#define INTC_IPR3_DMACH3(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH3_SHIFT)) & INTC_IPR3_DMACH3_MASK)
/*! DMACH2 - DMA Channel 2 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH2_MASK                    (0xCU)
#define INTC_IPR3_DMACH2_SHIFT                   (2U)
#define INTC_IPR3_DMACH2(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH2_SHIFT)) & INTC_IPR3_DMACH2_MASK)
/*! DMACH1 - DMA Channel 1 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH1_MASK                    (0x30U)
#define INTC_IPR3_DMACH1_SHIFT                   (4U)
#define INTC_IPR3_DMACH1(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH1_SHIFT)) & INTC_IPR3_DMACH1_MASK)
/*! DMACH0 - DMA Channel 0 Service Request Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_DMACH0_MASK                    (0xC0U)
#define INTC_IPR3_DMACH0_SHIFT                   (6U)
#define INTC_IPR3_DMACH0(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_DMACH0_SHIFT)) & INTC_IPR3_DMACH0_MASK)
/*! CAN_MB_OR - CAN Message Buffer Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_MB_OR_MASK                 (0x300U)
#define INTC_IPR3_CAN_MB_OR_SHIFT                (8U)
#define INTC_IPR3_CAN_MB_OR(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_MB_OR_SHIFT)) & INTC_IPR3_CAN_MB_OR_MASK)
/*! CAN_BUS_OFF - CAN Bus Off Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_BUS_OFF_MASK               (0xC00U)
#define INTC_IPR3_CAN_BUS_OFF_SHIFT              (10U)
#define INTC_IPR3_CAN_BUS_OFF(x)                 (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_BUS_OFF_SHIFT)) & INTC_IPR3_CAN_BUS_OFF_MASK)
/*! CAN_ERROR - CAN Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_ERROR_MASK                 (0x3000U)
#define INTC_IPR3_CAN_ERROR_SHIFT                (12U)
#define INTC_IPR3_CAN_ERROR(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_ERROR_SHIFT)) & INTC_IPR3_CAN_ERROR_MASK)
/*! CAN_TX_WARN - CAN Transmit Warning Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR3_CAN_TX_WARN_MASK               (0xC000U)
#define INTC_IPR3_CAN_TX_WARN_SHIFT              (14U)
#define INTC_IPR3_CAN_TX_WARN(x)                 (((uint16_t)(((uint16_t)(x)) << INTC_IPR3_CAN_TX_WARN_SHIFT)) & INTC_IPR3_CAN_TX_WARN_MASK)
/*! @} */

/*! @name IPR4 - Interrupt Priority Register 4 */
/*! @{ */
/*! CAN_RX_WARN - CAN Receive Warning Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_CAN_RX_WARN_MASK               (0x3U)
#define INTC_IPR4_CAN_RX_WARN_SHIFT              (0U)
#define INTC_IPR4_CAN_RX_WARN(x)                 (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_CAN_RX_WARN_SHIFT)) & INTC_IPR4_CAN_RX_WARN_MASK)
/*! CAN_WAKEUP - CAN Wakeup Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_CAN_WAKEUP_MASK                (0xCU)
#define INTC_IPR4_CAN_WAKEUP_SHIFT               (2U)
#define INTC_IPR4_CAN_WAKEUP(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_CAN_WAKEUP_SHIFT)) & INTC_IPR4_CAN_WAKEUP_MASK)
/*! QSCI2_RERR - QSCI 2 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_RERR_MASK                (0x30U)
#define INTC_IPR4_QSCI2_RERR_SHIFT               (4U)
#define INTC_IPR4_QSCI2_RERR(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_RERR_SHIFT)) & INTC_IPR4_QSCI2_RERR_MASK)
/*! QSCI2_RCV - QSCI 2 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_RCV_MASK                 (0xC0U)
#define INTC_IPR4_QSCI2_RCV_SHIFT                (6U)
#define INTC_IPR4_QSCI2_RCV(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_RCV_SHIFT)) & INTC_IPR4_QSCI2_RCV_MASK)
/*! QSCI2_TRIDLE - QSCI 2 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_TRIDLE_MASK              (0x300U)
#define INTC_IPR4_QSCI2_TRIDLE_SHIFT             (8U)
#define INTC_IPR4_QSCI2_TRIDLE(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_TRIDLE_SHIFT)) & INTC_IPR4_QSCI2_TRIDLE_MASK)
/*! QSCI2_TDRE - QSCI 2 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI2_TDRE_MASK                (0xC00U)
#define INTC_IPR4_QSCI2_TDRE_SHIFT               (10U)
#define INTC_IPR4_QSCI2_TDRE(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI2_TDRE_SHIFT)) & INTC_IPR4_QSCI2_TDRE_MASK)
/*! QSCI1_RERR - QSCI 1 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI1_RERR_MASK                (0x3000U)
#define INTC_IPR4_QSCI1_RERR_SHIFT               (12U)
#define INTC_IPR4_QSCI1_RERR(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI1_RERR_SHIFT)) & INTC_IPR4_QSCI1_RERR_MASK)
/*! QSCI1_RCV - QSCI 1 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR4_QSCI1_RCV_MASK                 (0xC000U)
#define INTC_IPR4_QSCI1_RCV_SHIFT                (14U)
#define INTC_IPR4_QSCI1_RCV(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR4_QSCI1_RCV_SHIFT)) & INTC_IPR4_QSCI1_RCV_MASK)
/*! @} */

/*! @name IPR5 - Interrupt Priority Register 5 */
/*! @{ */
/*! QSCI1_TRIDLE - QSCI 1 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI1_TRIDLE_MASK              (0x3U)
#define INTC_IPR5_QSCI1_TRIDLE_SHIFT             (0U)
#define INTC_IPR5_QSCI1_TRIDLE(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI1_TRIDLE_SHIFT)) & INTC_IPR5_QSCI1_TRIDLE_MASK)
/*! QSCI1_TDRE - QSCI 1 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI1_TDRE_MASK                (0xCU)
#define INTC_IPR5_QSCI1_TDRE_SHIFT               (2U)
#define INTC_IPR5_QSCI1_TDRE(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI1_TDRE_SHIFT)) & INTC_IPR5_QSCI1_TDRE_MASK)
/*! QSCI0_RERR - QSCI0 Receiver Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_RERR_MASK                (0x30U)
#define INTC_IPR5_QSCI0_RERR_SHIFT               (4U)
#define INTC_IPR5_QSCI0_RERR(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_RERR_SHIFT)) & INTC_IPR5_QSCI0_RERR_MASK)
/*! QSCI0_RCV - QSCI 0 Receive Data Register Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_RCV_MASK                 (0xC0U)
#define INTC_IPR5_QSCI0_RCV_SHIFT                (6U)
#define INTC_IPR5_QSCI0_RCV(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_RCV_SHIFT)) & INTC_IPR5_QSCI0_RCV_MASK)
/*! QSCI0_TRIDLE - QSCI 0 Transmitter and Receiver Idle Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_TRIDLE_MASK              (0x300U)
#define INTC_IPR5_QSCI0_TRIDLE_SHIFT             (8U)
#define INTC_IPR5_QSCI0_TRIDLE(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_TRIDLE_SHIFT)) & INTC_IPR5_QSCI0_TRIDLE_MASK)
/*! QSCI0_TDRE - QSCI 0 Transmit Data Register Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR5_QSCI0_TDRE_MASK                (0xC00U)
#define INTC_IPR5_QSCI0_TDRE_SHIFT               (10U)
#define INTC_IPR5_QSCI0_TDRE(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR5_QSCI0_TDRE_SHIFT)) & INTC_IPR5_QSCI0_TDRE_MASK)
/*! @} */

/*! @name IPR6 - Interrupt Priority Register 6 */
/*! @{ */
/*! QSPI1_XMIT - QSPI1 Transmitter Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI1_XMIT_MASK                (0x3U)
#define INTC_IPR6_QSPI1_XMIT_SHIFT               (0U)
#define INTC_IPR6_QSPI1_XMIT(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI1_XMIT_SHIFT)) & INTC_IPR6_QSPI1_XMIT_MASK)
/*! QSPI1_RCV - QSPI1 Receiver Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI1_RCV_MASK                 (0xCU)
#define INTC_IPR6_QSPI1_RCV_SHIFT                (2U)
#define INTC_IPR6_QSPI1_RCV(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI1_RCV_SHIFT)) & INTC_IPR6_QSPI1_RCV_MASK)
/*! QSPI0_XMIT - QSPI0 Transmitter Empty Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI0_XMIT_MASK                (0x30U)
#define INTC_IPR6_QSPI0_XMIT_SHIFT               (4U)
#define INTC_IPR6_QSPI0_XMIT(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI0_XMIT_SHIFT)) & INTC_IPR6_QSPI0_XMIT_MASK)
/*! QSPI0_RCV - QSPI0 Receiver Full Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_QSPI0_RCV_MASK                 (0xC0U)
#define INTC_IPR6_QSPI0_RCV_SHIFT                (6U)
#define INTC_IPR6_QSPI0_RCV(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_QSPI0_RCV_SHIFT)) & INTC_IPR6_QSPI0_RCV_MASK)
/*! IIC1 - IIC1 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_IIC1_MASK                      (0x300U)
#define INTC_IPR6_IIC1_SHIFT                     (8U)
#define INTC_IPR6_IIC1(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_IIC1_SHIFT)) & INTC_IPR6_IIC1_MASK)
/*! IIC0 - IIC0 Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR6_IIC0_MASK                      (0xC00U)
#define INTC_IPR6_IIC0_SHIFT                     (10U)
#define INTC_IPR6_IIC0(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR6_IIC0_SHIFT)) & INTC_IPR6_IIC0_MASK)
/*! @} */

/*! @name IPR7 - Interrupt Priority Register 7 */
/*! @{ */
/*! PWMB_FAULT - PWMB Fault Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_FAULT_MASK                (0xCU)
#define INTC_IPR7_PWMB_FAULT_SHIFT               (2U)
#define INTC_IPR7_PWMB_FAULT(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_FAULT_SHIFT)) & INTC_IPR7_PWMB_FAULT_MASK)
/*! PWMB_RERR - PWMB Reload Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RERR_MASK                 (0x30U)
#define INTC_IPR7_PWMB_RERR_SHIFT                (4U)
#define INTC_IPR7_PWMB_RERR(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RERR_SHIFT)) & INTC_IPR7_PWMB_RERR_MASK)
/*! PWMB_CAP - PWMB Submodule Capture Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CAP_MASK                  (0xC0U)
#define INTC_IPR7_PWMB_CAP_SHIFT                 (6U)
#define INTC_IPR7_PWMB_CAP(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CAP_SHIFT)) & INTC_IPR7_PWMB_CAP_MASK)
/*! PWMB_RELOAD3 - PWMB Submodule 3 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RELOAD3_MASK              (0x300U)
#define INTC_IPR7_PWMB_RELOAD3_SHIFT             (8U)
#define INTC_IPR7_PWMB_RELOAD3(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RELOAD3_SHIFT)) & INTC_IPR7_PWMB_RELOAD3_MASK)
/*! PWMB_CMP3 - PWMB Submodule 3 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CMP3_MASK                 (0xC00U)
#define INTC_IPR7_PWMB_CMP3_SHIFT                (10U)
#define INTC_IPR7_PWMB_CMP3(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CMP3_SHIFT)) & INTC_IPR7_PWMB_CMP3_MASK)
/*! PWMB_RELOAD2 - PWMB Submodule 2 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_RELOAD2_MASK              (0x3000U)
#define INTC_IPR7_PWMB_RELOAD2_SHIFT             (12U)
#define INTC_IPR7_PWMB_RELOAD2(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_RELOAD2_SHIFT)) & INTC_IPR7_PWMB_RELOAD2_MASK)
/*! PWMB_CMP2 - PWMB Submodule 2 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR7_PWMB_CMP2_MASK                 (0xC000U)
#define INTC_IPR7_PWMB_CMP2_SHIFT                (14U)
#define INTC_IPR7_PWMB_CMP2(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR7_PWMB_CMP2_SHIFT)) & INTC_IPR7_PWMB_CMP2_MASK)
/*! @} */

/*! @name IPR8 - Interrupt Priority Register 8 */
/*! @{ */
/*! PWMB_RELOAD1 - PWMB Submodule 1 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_RELOAD1_MASK              (0x3U)
#define INTC_IPR8_PWMB_RELOAD1_SHIFT             (0U)
#define INTC_IPR8_PWMB_RELOAD1(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_RELOAD1_SHIFT)) & INTC_IPR8_PWMB_RELOAD1_MASK)
/*! PWMB_CMP1 - PWMB Submodule 1 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_CMP1_MASK                 (0xCU)
#define INTC_IPR8_PWMB_CMP1_SHIFT                (2U)
#define INTC_IPR8_PWMB_CMP1(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_CMP1_SHIFT)) & INTC_IPR8_PWMB_CMP1_MASK)
/*! PWMB_RELOAD0 - PWMB Submodule 0 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_RELOAD0_MASK              (0x30U)
#define INTC_IPR8_PWMB_RELOAD0_SHIFT             (4U)
#define INTC_IPR8_PWMB_RELOAD0(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_RELOAD0_SHIFT)) & INTC_IPR8_PWMB_RELOAD0_MASK)
/*! PWMB_CMP0 - PWMB Submodule 0 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMB_CMP0_MASK                 (0xC0U)
#define INTC_IPR8_PWMB_CMP0_SHIFT                (6U)
#define INTC_IPR8_PWMB_CMP0(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMB_CMP0_SHIFT)) & INTC_IPR8_PWMB_CMP0_MASK)
/*! PWMA_FAULT - PWMA Fault Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_FAULT_MASK                (0x300U)
#define INTC_IPR8_PWMA_FAULT_SHIFT               (8U)
#define INTC_IPR8_PWMA_FAULT(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_FAULT_SHIFT)) & INTC_IPR8_PWMA_FAULT_MASK)
/*! PWMA_RERR - PWMA Reload Error Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_RERR_MASK                 (0xC00U)
#define INTC_IPR8_PWMA_RERR_SHIFT                (10U)
#define INTC_IPR8_PWMA_RERR(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_RERR_SHIFT)) & INTC_IPR8_PWMA_RERR_MASK)
/*! PWMA_CAP - PWMA Submodule Capture Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_CAP_MASK                  (0x3000U)
#define INTC_IPR8_PWMA_CAP_SHIFT                 (12U)
#define INTC_IPR8_PWMA_CAP(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_CAP_SHIFT)) & INTC_IPR8_PWMA_CAP_MASK)
/*! PWMA_RELOAD3 - PWMA Submodule 3 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR8_PWMA_RELOAD3_MASK              (0xC000U)
#define INTC_IPR8_PWMA_RELOAD3_SHIFT             (14U)
#define INTC_IPR8_PWMA_RELOAD3(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR8_PWMA_RELOAD3_SHIFT)) & INTC_IPR8_PWMA_RELOAD3_MASK)
/*! @} */

/*! @name IPR9 - Interrupt Priority Register 9 */
/*! @{ */
/*! PWMA_CMP3 - PWMA Submodule 3 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP3_MASK                 (0x3U)
#define INTC_IPR9_PWMA_CMP3_SHIFT                (0U)
#define INTC_IPR9_PWMA_CMP3(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP3_SHIFT)) & INTC_IPR9_PWMA_CMP3_MASK)
/*! PWMA_RELOAD2 - PWMA Submodule 2 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD2_MASK              (0xCU)
#define INTC_IPR9_PWMA_RELOAD2_SHIFT             (2U)
#define INTC_IPR9_PWMA_RELOAD2(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD2_SHIFT)) & INTC_IPR9_PWMA_RELOAD2_MASK)
/*! PWMA_CMP2 - PWMA Submodule 2 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP2_MASK                 (0x30U)
#define INTC_IPR9_PWMA_CMP2_SHIFT                (4U)
#define INTC_IPR9_PWMA_CMP2(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP2_SHIFT)) & INTC_IPR9_PWMA_CMP2_MASK)
/*! PWMA_RELOAD1 - PWMA Submodule 1 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD1_MASK              (0xC0U)
#define INTC_IPR9_PWMA_RELOAD1_SHIFT             (6U)
#define INTC_IPR9_PWMA_RELOAD1(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD1_SHIFT)) & INTC_IPR9_PWMA_RELOAD1_MASK)
/*! PWMA_CMP1 - PWMA Submodule 1 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP1_MASK                 (0x300U)
#define INTC_IPR9_PWMA_CMP1_SHIFT                (8U)
#define INTC_IPR9_PWMA_CMP1(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP1_SHIFT)) & INTC_IPR9_PWMA_CMP1_MASK)
/*! PWMA_RELOAD0 - PWMA Submodule 0 Reload Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_RELOAD0_MASK              (0xC00U)
#define INTC_IPR9_PWMA_RELOAD0_SHIFT             (10U)
#define INTC_IPR9_PWMA_RELOAD0(x)                (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_RELOAD0_SHIFT)) & INTC_IPR9_PWMA_RELOAD0_MASK)
/*! PWMA_CMP0 - PWMA Submodule 0 Compare Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_PWMA_CMP0_MASK                 (0x3000U)
#define INTC_IPR9_PWMA_CMP0_SHIFT                (12U)
#define INTC_IPR9_PWMA_CMP0(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_PWMA_CMP0_SHIFT)) & INTC_IPR9_PWMA_CMP0_MASK)
/*! FTFE_RDCOL - Flash module Access Error Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR9_FTFE_RDCOL_MASK                (0xC000U)
#define INTC_IPR9_FTFE_RDCOL_SHIFT               (14U)
#define INTC_IPR9_FTFE_RDCOL(x)                  (((uint16_t)(((uint16_t)(x)) << INTC_IPR9_FTFE_RDCOL_SHIFT)) & INTC_IPR9_FTFE_RDCOL_MASK)
/*! @} */

/*! @name IPR10 - Interrupt Priority Register 10 */
/*! @{ */
/*! FTFE_CC - Flash module Command Complete Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_FTFE_CC_MASK                  (0x3U)
#define INTC_IPR10_FTFE_CC_SHIFT                 (0U)
#define INTC_IPR10_FTFE_CC(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_FTFE_CC_SHIFT)) & INTC_IPR10_FTFE_CC_MASK)
/*! CMPD - Comparator D Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPD_MASK                     (0xCU)
#define INTC_IPR10_CMPD_SHIFT                    (2U)
#define INTC_IPR10_CMPD(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPD_SHIFT)) & INTC_IPR10_CMPD_MASK)
/*! CMPC - Comparator C Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPC_MASK                     (0x30U)
#define INTC_IPR10_CMPC_SHIFT                    (4U)
#define INTC_IPR10_CMPC(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPC_SHIFT)) & INTC_IPR10_CMPC_MASK)
/*! CMPB - Comparator B Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPB_MASK                     (0xC0U)
#define INTC_IPR10_CMPB_SHIFT                    (6U)
#define INTC_IPR10_CMPB(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPB_SHIFT)) & INTC_IPR10_CMPB_MASK)
/*! CMPA - Comparator A Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_CMPA_MASK                     (0x300U)
#define INTC_IPR10_CMPA_SHIFT                    (8U)
#define INTC_IPR10_CMPA(x)                       (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_CMPA_SHIFT)) & INTC_IPR10_CMPA_MASK)
/*! PIT1_ROLLOVR - PIT1 Roll Over Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_PIT1_ROLLOVR_MASK             (0xC00U)
#define INTC_IPR10_PIT1_ROLLOVR_SHIFT            (10U)
#define INTC_IPR10_PIT1_ROLLOVR(x)               (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_PIT1_ROLLOVR_SHIFT)) & INTC_IPR10_PIT1_ROLLOVR_MASK)
/*! PIT0_ROLLOVR - PIT0 Roll Over Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_PIT0_ROLLOVR_MASK             (0x3000U)
#define INTC_IPR10_PIT0_ROLLOVR_SHIFT            (12U)
#define INTC_IPR10_PIT0_ROLLOVR(x)               (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_PIT0_ROLLOVR_SHIFT)) & INTC_IPR10_PIT0_ROLLOVR_MASK)
/*! FTFE_DFD - Flash module Double-bit Fault Detect Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR10_FTFE_DFD_MASK                 (0xC000U)
#define INTC_IPR10_FTFE_DFD_SHIFT                (14U)
#define INTC_IPR10_FTFE_DFD(x)                   (((uint16_t)(((uint16_t)(x)) << INTC_IPR10_FTFE_DFD_SHIFT)) & INTC_IPR10_FTFE_DFD_MASK)
/*! @} */

/*! @name IPR11 - Interrupt Priority Register 11 */
/*! @{ */
/*! GPIOG - GPIO G Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOG_MASK                    (0x300U)
#define INTC_IPR11_GPIOG_SHIFT                   (8U)
#define INTC_IPR11_GPIOG(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOG_SHIFT)) & INTC_IPR11_GPIOG_MASK)
/*! GPIOF - GPIO F Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOF_MASK                    (0xC00U)
#define INTC_IPR11_GPIOF_SHIFT                   (10U)
#define INTC_IPR11_GPIOF(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOF_SHIFT)) & INTC_IPR11_GPIOF_MASK)
/*! GPIOE - GPIO E Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOE_MASK                    (0x3000U)
#define INTC_IPR11_GPIOE_SHIFT                   (12U)
#define INTC_IPR11_GPIOE(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOE_SHIFT)) & INTC_IPR11_GPIOE_MASK)
/*! GPIOD - GPIO D Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR11_GPIOD_MASK                    (0xC000U)
#define INTC_IPR11_GPIOD_SHIFT                   (14U)
#define INTC_IPR11_GPIOD(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR11_GPIOD_SHIFT)) & INTC_IPR11_GPIOD_MASK)
/*! @} */

/*! @name IPR12 - Interrupt Priority Register 12 */
/*! @{ */
/*! GPIOC - GPIO C Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOC_MASK                    (0x3U)
#define INTC_IPR12_GPIOC_SHIFT                   (0U)
#define INTC_IPR12_GPIOC(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOC_SHIFT)) & INTC_IPR12_GPIOC_MASK)
/*! GPIOB - GPIO B Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOB_MASK                    (0xCU)
#define INTC_IPR12_GPIOB_SHIFT                   (2U)
#define INTC_IPR12_GPIOB(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOB_SHIFT)) & INTC_IPR12_GPIOB_MASK)
/*! GPIOA - GPIO A Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_GPIOA_MASK                    (0x30U)
#define INTC_IPR12_GPIOA_SHIFT                   (4U)
#define INTC_IPR12_GPIOA(x)                      (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_GPIOA_SHIFT)) & INTC_IPR12_GPIOA_MASK)
/*! COP_INT - COP Watchdog Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_COP_INT_MASK                  (0xC0U)
#define INTC_IPR12_COP_INT_SHIFT                 (6U)
#define INTC_IPR12_COP_INT(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_COP_INT_SHIFT)) & INTC_IPR12_COP_INT_MASK)
/*! EWM_INT - External Watchdog Monitor Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_EWM_INT_MASK                  (0x300U)
#define INTC_IPR12_EWM_INT_SHIFT                 (8U)
#define INTC_IPR12_EWM_INT(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_EWM_INT_SHIFT)) & INTC_IPR12_EWM_INT_MASK)
/*! USB - USB Interrupt Priority Level
 *  0b00..IRQ disabled (default)
 *  0b01..IRQ Priority Level 0
 *  0b10..IRQ Priority Level 1
 *  0b11..IRQ Priority Level 2
 */
#define INTC_IPR12_USB_MASK                      (0x3000U)
#define INTC_IPR12_USB_SHIFT                     (12U)
#define INTC_IPR12_USB(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_IPR12_USB_SHIFT)) & INTC_IPR12_USB_MASK)
/*! @} */

/*! @name VBA - Vector Base Address Register */
/*! @{ */
#define INTC_VBA_VECTOR_BASE_ADDRESS_MASK        (0x1FFFU)
#define INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT       (0U)
#define INTC_VBA_VECTOR_BASE_ADDRESS(x)          (((uint16_t)(((uint16_t)(x)) << INTC_VBA_VECTOR_BASE_ADDRESS_SHIFT)) & INTC_VBA_VECTOR_BASE_ADDRESS_MASK)
/*! @} */

/*! @name FIM0 - Fast Interrupt 0 Match Register */
/*! @{ */
#define INTC_FIM0_FAST_INTERRUPT_0_MASK          (0x7FU)
#define INTC_FIM0_FAST_INTERRUPT_0_SHIFT         (0U)
#define INTC_FIM0_FAST_INTERRUPT_0(x)            (((uint16_t)(((uint16_t)(x)) << INTC_FIM0_FAST_INTERRUPT_0_SHIFT)) & INTC_FIM0_FAST_INTERRUPT_0_MASK)
/*! @} */

/*! @name FIVAL0 - Fast Interrupt 0 Vector Address Low Register */
/*! @{ */
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK (0xFFFFU)
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT (0U)
#define INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW(x)   (((uint16_t)(((uint16_t)(x)) << INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_SHIFT)) & INTC_FIVAL0_FI_0_VECTOR_ADDRESS_LOW_MASK)
/*! @} */

/*! @name FIVAH0 - Fast Interrupt 0 Vector Address High Register */
/*! @{ */
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK (0x1FU)
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT (0U)
#define INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH(x)  (((uint16_t)(((uint16_t)(x)) << INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_SHIFT)) & INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK)
/*! @} */

/*! @name FIM1 - Fast Interrupt 1 Match Register */
/*! @{ */
#define INTC_FIM1_FAST_INTERRUPT_1_MASK          (0x7FU)
#define INTC_FIM1_FAST_INTERRUPT_1_SHIFT         (0U)
#define INTC_FIM1_FAST_INTERRUPT_1(x)            (((uint16_t)(((uint16_t)(x)) << INTC_FIM1_FAST_INTERRUPT_1_SHIFT)) & INTC_FIM1_FAST_INTERRUPT_1_MASK)
/*! @} */

/*! @name FIVAL1 - Fast Interrupt 1 Vector Address Low Register */
/*! @{ */
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK (0xFFFFU)
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT (0U)
#define INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW(x)   (((uint16_t)(((uint16_t)(x)) << INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_SHIFT)) & INTC_FIVAL1_FI_1_VECTOR_ADDRESS_LOW_MASK)
/*! @} */

/*! @name FIVAH1 - Fast Interrupt 1 Vector Address High Register */
/*! @{ */
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK (0x1FU)
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT (0U)
#define INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH(x)  (((uint16_t)(((uint16_t)(x)) << INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_SHIFT)) & INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK)
/*! @} */

/*! @name IRQP0 - IRQ Pending Register 0 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b000000000000000..IRQ pending for this vector number
 *  0b000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP0_PENDING_MASK                  (0xFFFEU)
#define INTC_IRQP0_PENDING_SHIFT                 (1U)
#define INTC_IRQP0_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP0_PENDING_SHIFT)) & INTC_IRQP0_PENDING_MASK)
/*! @} */

/*! @name IRQP1 - IRQ Pending Register 1 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP1_PENDING_MASK                  (0xFFFFU)
#define INTC_IRQP1_PENDING_SHIFT                 (0U)
#define INTC_IRQP1_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP1_PENDING_SHIFT)) & INTC_IRQP1_PENDING_MASK)
/*! @} */

/*! @name IRQP2 - IRQ Pending Register 2 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP2_PENDING_MASK                  (0xFFFFU)
#define INTC_IRQP2_PENDING_SHIFT                 (0U)
#define INTC_IRQP2_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP2_PENDING_SHIFT)) & INTC_IRQP2_PENDING_MASK)
/*! @} */

/*! @name IRQP3 - IRQ Pending Register 3 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP3_PENDING_MASK                  (0xFFFFU)
#define INTC_IRQP3_PENDING_SHIFT                 (0U)
#define INTC_IRQP3_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP3_PENDING_SHIFT)) & INTC_IRQP3_PENDING_MASK)
/*! @} */

/*! @name IRQP4 - IRQ Pending Register 4 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP4_PENDING_MASK                  (0xFFFFU)
#define INTC_IRQP4_PENDING_SHIFT                 (0U)
#define INTC_IRQP4_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP4_PENDING_SHIFT)) & INTC_IRQP4_PENDING_MASK)
/*! @} */

/*! @name IRQP5 - IRQ Pending Register 5 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b0000000000000000..IRQ pending for this vector number
 *  0b0000000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP5_PENDING_MASK                  (0xFFFFU)
#define INTC_IRQP5_PENDING_SHIFT                 (0U)
#define INTC_IRQP5_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP5_PENDING_SHIFT)) & INTC_IRQP5_PENDING_MASK)
/*! @} */

/*! @name IRQP6 - IRQ Pending Register 6 */
/*! @{ */
/*! PENDING - Pending IRQs
 *  0b00000000000000..IRQ pending for this vector number
 *  0b00000000000001..No IRQ pending for this vector number
 */
#define INTC_IRQP6_PENDING_MASK                  (0x3FFFU)
#define INTC_IRQP6_PENDING_SHIFT                 (0U)
#define INTC_IRQP6_PENDING(x)                    (((uint16_t)(((uint16_t)(x)) << INTC_IRQP6_PENDING_SHIFT)) & INTC_IRQP6_PENDING_MASK)
/*! @} */

/*! @name CTRL - Control Register */
/*! @{ */
/*! INT_DIS - Interrupt disable
 *  0b0..Normal operation. (default)
 *  0b1..All interrupts disabled.
 */
#define INTC_CTRL_INT_DIS_MASK                   (0x20U)
#define INTC_CTRL_INT_DIS_SHIFT                  (5U)
#define INTC_CTRL_INT_DIS(x)                     (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_INT_DIS_SHIFT)) & INTC_CTRL_INT_DIS_MASK)
#define INTC_CTRL_VAB_MASK                       (0x1FC0U)
#define INTC_CTRL_VAB_SHIFT                      (6U)
#define INTC_CTRL_VAB(x)                         (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_VAB_SHIFT)) & INTC_CTRL_VAB_MASK)
/*! IPIC - Interrupt Priority Level
 *  0b00..Required nested exception priority levels are 0, 1, 2, or 3.
 *  0b01..Required nested exception priority levels are 1, 2, or 3.
 *  0b10..Required nested exception priority levels are 2 or3.
 *  0b11..Required nested exception priority level is 3.
 */
#define INTC_CTRL_IPIC_MASK                      (0x6000U)
#define INTC_CTRL_IPIC_SHIFT                     (13U)
#define INTC_CTRL_IPIC(x)                        (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_IPIC_SHIFT)) & INTC_CTRL_IPIC_MASK)
/*! INT - Interrupt
 *  0b0..No interrupt is being sent to the core.
 *  0b1..An interrupt is being sent to the core.
 */
#define INTC_CTRL_INT_MASK                       (0x8000U)
#define INTC_CTRL_INT_SHIFT                      (15U)
#define INTC_CTRL_INT(x)                         (((uint16_t)(((uint16_t)(x)) << INTC_CTRL_INT_SHIFT)) & INTC_CTRL_INT_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group INTC_Register_Masks */


/* INTC - Peripheral instance base addresses */
/** Peripheral INTC base address */
#define INTC_BASE                                (0xE300u)
/** Peripheral INTC base pointer */
#define INTC                                     ((INTC_Type *)INTC_BASE)
/** Array initializer of INTC peripheral base addresses */
#define INTC_BASE_ADDRS                          { INTC_BASE }
/** Array initializer of INTC peripheral base pointers */
#define INTC_BASE_PTRS                           { INTC }

/*!
 * @}
 */ /* end of group INTC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[8];
  __I  uint16_t PLASC;                             /**< Crossbar switch (AXBS) slave configuration, offset: 0x8 */
  __I  uint16_t PLAMC;                             /**< Crossbar switch (AXBS) master configuration, offset: 0xA */
  __IO uint32_t CPCR;                              /**< Core control register, offset: 0xC */
  __I  uint32_t CFADR;                             /**< Core fault address register, offset: 0x10 */
  __I  uint8_t CFATR;                              /**< Core fault attributes register, offset: 0x14 */
  __I  uint8_t CFLOC;                              /**< Core fault location register, offset: 0x15 */
  __IO uint8_t CFIER;                              /**< Core fault interrupt enable register, offset: 0x16 */
  __IO uint8_t CFISR;                              /**< MCM interrupt status register, offset: 0x17 */
  __I  uint32_t CFDTR;                             /**< Core fault data register, offset: 0x18 */
       uint8_t RESERVED_1[4];
  __IO uint32_t RPCR;                              /**< Resource Protection Control Register, offset: 0x20 */
  __IO uint32_t UFLASHBAR;                         /**< User Flash Base Address Register, offset: 0x24 */
  __IO uint32_t UPRAMBAR;                          /**< User Program RAM Base Address Register, offset: 0x28 */
  __IO uint32_t UBROMBAR;                          /**< User Boot ROM Base Address Register, offset: 0x2C */
  __IO uint32_t SRPOSP;                            /**< Resource Protection Other Stack Pointer, offset: 0x30 */
  __IO uint32_t SRPIPC;                            /**< Memory Protection Illegal PC, offset: 0x34 */
  __IO uint32_t SRPMPC;                            /**< Resource Protection Misaligned PC, offset: 0x38 */
} MCM_Type;

/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/*! @name PLASC - Crossbar switch (AXBS) slave configuration */
/*! @{ */
/*! ASC - Each bit in the ASC field indicates if there is a corresponding connection to the AXBS
 *    slave input port. For this device, this field always read 0x0F.
 *  0b00000000..A bus slave connection to AXBS input port n is absent
 *  0b00000001..A bus slave connection to AXBS input port n is present
 */
#define MCM_PLASC_ASC_MASK                       (0xFFU)
#define MCM_PLASC_ASC_SHIFT                      (0U)
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_ASC_SHIFT)) & MCM_PLASC_ASC_MASK)
/*! DP64 - Indicates if the datapath is 32 or 64 bits wide
 *  0b0..Datapath width is 32 bits
 *  0b1..Datapath width is 64 bits
 */
#define MCM_PLASC_DP64_MASK                      (0x8000U)
#define MCM_PLASC_DP64_SHIFT                     (15U)
#define MCM_PLASC_DP64(x)                        (((uint16_t)(((uint16_t)(x)) << MCM_PLASC_DP64_SHIFT)) & MCM_PLASC_DP64_MASK)
/*! @} */

/*! @name PLAMC - Crossbar switch (AXBS) master configuration */
/*! @{ */
/*! AMC - Each bit in the AMC field indicates if there is a corresponding connection to the AXBS
 *    master input port. For this device, this field always reads 0x0F.
 *  0b00000000..A bus master connection to AXBS input port n is absent
 *  0b00000001..A bus master connection to AXBS input port n is present
 */
#define MCM_PLAMC_AMC_MASK                       (0xFFU)
#define MCM_PLAMC_AMC_SHIFT                      (0U)
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x)) << MCM_PLAMC_AMC_SHIFT)) & MCM_PLAMC_AMC_MASK)
/*! @} */

/*! @name CPCR - Core control register */
/*! @{ */
/*! FCCCLR - Clear Flash Controller Cache
 *  0b0..Writing logic 0 to this field is ignored.
 *  0b1..Writing logic 1 to this field clears the cache, 1 cycle active.
 */
#define MCM_CPCR_FCCCLR_MASK                     (0x400U)
#define MCM_CPCR_FCCCLR_SHIFT                    (10U)
#define MCM_CPCR_FCCCLR(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCCCLR_SHIFT)) & MCM_CPCR_FCCCLR_MASK)
/*! FCDCDIS - Disable Flash Controller Data Caching
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCDCDIS_MASK                    (0x800U)
#define MCM_CPCR_FCDCDIS_SHIFT                   (11U)
#define MCM_CPCR_FCDCDIS(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCDCDIS_SHIFT)) & MCM_CPCR_FCDCDIS_MASK)
/*! FCICDIS - Disable Flash Controller Instruction Caching
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCICDIS_MASK                    (0x1000U)
#define MCM_CPCR_FCICDIS_SHIFT                   (12U)
#define MCM_CPCR_FCICDIS(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCICDIS_SHIFT)) & MCM_CPCR_FCICDIS_MASK)
/*! FCCDIS - Disable Flash Controller Cache
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCCDIS_MASK                     (0x2000U)
#define MCM_CPCR_FCCDIS_SHIFT                    (13U)
#define MCM_CPCR_FCCDIS(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCCDIS_SHIFT)) & MCM_CPCR_FCCDIS_MASK)
/*! FCDSPDIS - Disable Flash Controller Data Speculation
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCDSPDIS_MASK                   (0x4000U)
#define MCM_CPCR_FCDSPDIS_SHIFT                  (14U)
#define MCM_CPCR_FCDSPDIS(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCDSPDIS_SHIFT)) & MCM_CPCR_FCDSPDIS_MASK)
/*! FCSPDIS - Disable Flash Controller Speculation
 *  0b0..enable
 *  0b1..disable
 */
#define MCM_CPCR_FCSPDIS_MASK                    (0x8000U)
#define MCM_CPCR_FCSPDIS_SHIFT                   (15U)
#define MCM_CPCR_FCSPDIS(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCSPDIS_SHIFT)) & MCM_CPCR_FCSPDIS_MASK)
/*! INSDIS - Disable instructions supported only by DSP56800EX core
 *  0b0..BFSC and 32-bit multiply and MAC instructions enabled
 *  0b1..BFSC and 32-bit multiply and MAC instructions disabled
 */
#define MCM_CPCR_INSDIS_MASK                     (0x10000U)
#define MCM_CPCR_INSDIS_SHIFT                    (16U)
#define MCM_CPCR_INSDIS(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_INSDIS_SHIFT)) & MCM_CPCR_INSDIS_MASK)
/*! RCDIS - Disable core reverse carry
 *  0b0..Core reverse carry enabled
 *  0b1..Core reverse carry disabled
 */
#define MCM_CPCR_RCDIS_MASK                      (0x20000U)
#define MCM_CPCR_RCDIS_SHIFT                     (17U)
#define MCM_CPCR_RCDIS(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_RCDIS_SHIFT)) & MCM_CPCR_RCDIS_MASK)
/*! SRDIS - Disable core new shadow region
 *  0b0..Core new shadow region enabled
 *  0b1..Core new shadow region disabled
 */
#define MCM_CPCR_SRDIS_MASK                      (0x40000U)
#define MCM_CPCR_SRDIS_SHIFT                     (18U)
#define MCM_CPCR_SRDIS(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_SRDIS_SHIFT)) & MCM_CPCR_SRDIS_MASK)
/*! IBDIS - Disable core instruction buffer
 *  0b0..Core longword instruction buffer enabled
 *  0b1..Core longword instruction buffer disabled
 */
#define MCM_CPCR_IBDIS_MASK                      (0x80000U)
#define MCM_CPCR_IBDIS_SHIFT                     (19U)
#define MCM_CPCR_IBDIS(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_IBDIS_SHIFT)) & MCM_CPCR_IBDIS_MASK)
/*! FCSDIS - Disable Flash Memory Controller stall
 *  0b0..Stall logic is enabled. While a flash memory command is executing, a flash memory access can occur
 *       without causing a bus error. The flash memory command completes execution, and then the flash memory access
 *       occurs.
 *  0b1..Stall logic is disabled. While a flash memory command is executing, an attempted flash memory access causes a bus error.
 */
#define MCM_CPCR_FCSDIS_MASK                     (0x100000U)
#define MCM_CPCR_FCSDIS_SHIFT                    (20U)
#define MCM_CPCR_FCSDIS(x)                       (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_FCSDIS_SHIFT)) & MCM_CPCR_FCSDIS_MASK)
/*! XBARARB - Select DMA Controller priority in AXBS Crossbar Switch arbitration scheme
 *  0b0..Fixed-priority arbitration is selected: DSC core has a higher priority than the DMA Controller's priority
 *  0b1..Round-robin priority arbitration is selected: DMA Controller and DSC core have equal priority
 */
#define MCM_CPCR_XBARARB_MASK                    (0x80000000U)
#define MCM_CPCR_XBARARB_SHIFT                   (31U)
#define MCM_CPCR_XBARARB(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_CPCR_XBARARB_SHIFT)) & MCM_CPCR_XBARARB_MASK)
/*! @} */

/*! @name CFADR - Core fault address register */
/*! @{ */
#define MCM_CFADR_ADDR_MASK                      (0xFFFFFFFFU)
#define MCM_CFADR_ADDR_SHIFT                     (0U)
#define MCM_CFADR_ADDR(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CFADR_ADDR_SHIFT)) & MCM_CFADR_ADDR_MASK)
/*! @} */

/*! @name CFATR - Core fault attributes register */
/*! @{ */
/*! TYPE - Type of last faulted core access
 *  0b0..Instruction
 *  0b1..Data
 */
#define MCM_CFATR_TYPE_MASK                      (0x1U)
#define MCM_CFATR_TYPE_SHIFT                     (0U)
#define MCM_CFATR_TYPE(x)                        (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_TYPE_SHIFT)) & MCM_CFATR_TYPE_MASK)
/*! BUFFER - Indicates if last faulted core access was bufferable
 *  0b0..Non-bufferable
 *  0b1..Bufferable
 */
#define MCM_CFATR_BUFFER_MASK                    (0x4U)
#define MCM_CFATR_BUFFER_SHIFT                   (2U)
#define MCM_CFATR_BUFFER(x)                      (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_BUFFER_SHIFT)) & MCM_CFATR_BUFFER_MASK)
/*! SIZE - Size of last faulted core access
 *  0b000..8-bit
 *  0b001..16-bit
 *  0b010..32-bit
 */
#define MCM_CFATR_SIZE_MASK                      (0x70U)
#define MCM_CFATR_SIZE_SHIFT                     (4U)
#define MCM_CFATR_SIZE(x)                        (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_SIZE_SHIFT)) & MCM_CFATR_SIZE_MASK)
/*! DIR - Direction of last faulted core access
 *  0b0..Core read access
 *  0b1..Core write access
 */
#define MCM_CFATR_DIR_MASK                       (0x80U)
#define MCM_CFATR_DIR_SHIFT                      (7U)
#define MCM_CFATR_DIR(x)                         (((uint8_t)(((uint8_t)(x)) << MCM_CFATR_DIR_SHIFT)) & MCM_CFATR_DIR_MASK)
/*! @} */

/*! @name CFLOC - Core fault location register */
/*! @{ */
/*! LOC - Location of last captured fault
 *  0b00..Error occurred on M0 (instruction bus)
 *  0b01..Error occured on M1 (operand A bus)
 *  0b10..Error occured on M2 (operand B bus)
 *  0b11..Reserved
 */
#define MCM_CFLOC_LOC_MASK                       (0xC0U)
#define MCM_CFLOC_LOC_SHIFT                      (6U)
#define MCM_CFLOC_LOC(x)                         (((uint8_t)(((uint8_t)(x)) << MCM_CFLOC_LOC_SHIFT)) & MCM_CFLOC_LOC_MASK)
/*! @} */

/*! @name CFIER - Core fault interrupt enable register */
/*! @{ */
/*! ECFEI - Enable core fault error interrupt
 *  0b0..Do not generate an error interrupt on a faulted system bus cycle
 *  0b1..Generate an error interrupt to the interrupt controller on a faulted system bus cycle
 */
#define MCM_CFIER_ECFEI_MASK                     (0x80U)
#define MCM_CFIER_ECFEI_SHIFT                    (7U)
#define MCM_CFIER_ECFEI(x)                       (((uint8_t)(((uint8_t)(x)) << MCM_CFIER_ECFEI_SHIFT)) & MCM_CFIER_ECFEI_MASK)
/*! @} */

/*! @name CFISR - MCM interrupt status register */
/*! @{ */
/*! CFEDL - Core fault error data lost flag
 *  0b0..No bus error data lost
 *  0b1..A bus error has occured before the previous error condition was cleared.
 */
#define MCM_CFISR_CFEDL_MASK                     (0x40U)
#define MCM_CFISR_CFEDL_SHIFT                    (6U)
#define MCM_CFISR_CFEDL(x)                       (((uint8_t)(((uint8_t)(x)) << MCM_CFISR_CFEDL_SHIFT)) & MCM_CFISR_CFEDL_MASK)
/*! CFEI - Core fault error interrupt flag
 *  0b0..No bus error
 *  0b1..A bus error has occurred. The faulting address, attributes (and possibly write data) are captured in the
 *       CFADR, CFATR, and CFDTR registers. The error interrupt is enabled only if CFIER[ECFEI] is set.
 */
#define MCM_CFISR_CFEI_MASK                      (0x80U)
#define MCM_CFISR_CFEI_SHIFT                     (7U)
#define MCM_CFISR_CFEI(x)                        (((uint8_t)(((uint8_t)(x)) << MCM_CFISR_CFEI_SHIFT)) & MCM_CFISR_CFEI_MASK)
/*! @} */

/*! @name CFDTR - Core fault data register */
/*! @{ */
#define MCM_CFDTR_DATA_MASK                      (0xFFFFFFFFU)
#define MCM_CFDTR_DATA_SHIFT                     (0U)
#define MCM_CFDTR_DATA(x)                        (((uint32_t)(((uint32_t)(x)) << MCM_CFDTR_DATA_SHIFT)) & MCM_CFDTR_DATA_MASK)
/*! @} */

/*! @name RPCR - Resource Protection Control Register */
/*! @{ */
/*! RPE - Resource Protection Enable
 *  0b0..Resource protection disabled
 *  0b1..Resource protection enabled
 */
#define MCM_RPCR_RPE_MASK                        (0x1U)
#define MCM_RPCR_RPE_SHIFT                       (0U)
#define MCM_RPCR_RPE(x)                          (((uint32_t)(((uint32_t)(x)) << MCM_RPCR_RPE_SHIFT)) & MCM_RPCR_RPE_MASK)
/*! RL - Register Lock
 *  0b0..RP register values may be changed
 *  0b1..RP registers are locked and may not be changed until after a system reset
 */
#define MCM_RPCR_RL_MASK                         (0x2U)
#define MCM_RPCR_RL_SHIFT                        (1U)
#define MCM_RPCR_RL(x)                           (((uint32_t)(((uint32_t)(x)) << MCM_RPCR_RL_SHIFT)) & MCM_RPCR_RL_MASK)
/*! @} */

/*! @name UFLASHBAR - User Flash Base Address Register */
/*! @{ */
#define MCM_UFLASHBAR_FBA_MASK                   (0x3F000U)
#define MCM_UFLASHBAR_FBA_SHIFT                  (12U)
#define MCM_UFLASHBAR_FBA(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_UFLASHBAR_FBA_SHIFT)) & MCM_UFLASHBAR_FBA_MASK)
/*! @} */

/*! @name UPRAMBAR - User Program RAM Base Address Register */
/*! @{ */
#define MCM_UPRAMBAR_RBA_MASK                    (0x7F00U)
#define MCM_UPRAMBAR_RBA_SHIFT                   (8U)
#define MCM_UPRAMBAR_RBA(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_UPRAMBAR_RBA_SHIFT)) & MCM_UPRAMBAR_RBA_MASK)
/*! @} */

/*! @name UBROMBAR - User Boot ROM Base Address Register */
/*! @{ */
#define MCM_UBROMBAR_RBA_MASK                    (0xFFFFFFFFU)
#define MCM_UBROMBAR_RBA_SHIFT                   (0U)
#define MCM_UBROMBAR_RBA(x)                      (((uint32_t)(((uint32_t)(x)) << MCM_UBROMBAR_RBA_SHIFT)) & MCM_UBROMBAR_RBA_MASK)
/*! @} */

/*! @name SRPOSP - Resource Protection Other Stack Pointer */
/*! @{ */
#define MCM_SRPOSP_SRPOSP_MASK                   (0xFFFFFFU)
#define MCM_SRPOSP_SRPOSP_SHIFT                  (0U)
#define MCM_SRPOSP_SRPOSP(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_SRPOSP_SRPOSP_SHIFT)) & MCM_SRPOSP_SRPOSP_MASK)
/*! @} */

/*! @name SRPIPC - Memory Protection Illegal PC */
/*! @{ */
#define MCM_SRPIPC_SRPIFPC_MASK                  (0x1FFFFFU)
#define MCM_SRPIPC_SRPIFPC_SHIFT                 (0U)
#define MCM_SRPIPC_SRPIFPC(x)                    (((uint32_t)(((uint32_t)(x)) << MCM_SRPIPC_SRPIFPC_SHIFT)) & MCM_SRPIPC_SRPIFPC_MASK)
#define MCM_SRPIPC_SRPIFV_MASK                   (0x80000000U)
#define MCM_SRPIPC_SRPIFV_SHIFT                  (31U)
#define MCM_SRPIPC_SRPIFV(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_SRPIPC_SRPIFV_SHIFT)) & MCM_SRPIPC_SRPIFV_MASK)
/*! @} */

/*! @name SRPMPC - Resource Protection Misaligned PC */
/*! @{ */
#define MCM_SRPMPC_SRPMFPC_MASK                  (0x1FFFFFU)
#define MCM_SRPMPC_SRPMFPC_SHIFT                 (0U)
#define MCM_SRPMPC_SRPMFPC(x)                    (((uint32_t)(((uint32_t)(x)) << MCM_SRPMPC_SRPMFPC_SHIFT)) & MCM_SRPMPC_SRPMFPC_MASK)
#define MCM_SRPMPC_SRPMFV_MASK                   (0x80000000U)
#define MCM_SRPMPC_SRPMFV_SHIFT                  (31U)
#define MCM_SRPMPC_SRPMFV(x)                     (((uint32_t)(((uint32_t)(x)) << MCM_SRPMPC_SRPMFV_SHIFT)) & MCM_SRPMPC_SRPMFV_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xC000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_Type *)MCM_BASE)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }

/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- OCCS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Peripheral_Access_Layer OCCS Peripheral Access Layer
 * @{
 */

/** OCCS - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< PLL Control Register, offset: 0x0 */
  __IO uint16_t DIVBY;                             /**< PLL Divide-By Register, offset: 0x1 */
  __IO uint16_t STAT;                              /**< OCCS Status Register, offset: 0x2 */
       uint16_t RESERVED_0[1];
  __IO uint16_t OSCTL1;                            /**< Oscillator Control Register 1, offset: 0x4 */
  __IO uint16_t OSCTL2;                            /**< Oscillator Control Register 2, offset: 0x5 */
  __IO uint16_t CLKCHKR;                           /**< External Clock Check Reference, offset: 0x6 */
  __I  uint16_t CLKCHKT;                           /**< External Clock Check Target, offset: 0x7 */
  __IO uint16_t PROT;                              /**< Protection Register, offset: 0x8 */
} OCCS_Type;

/* ----------------------------------------------------------------------------
   -- OCCS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OCCS_Register_Masks OCCS Register Masks
 * @{
 */

/*! @name CTRL - PLL Control Register */
/*! @{ */
/*! ZSRC - CLOCK Source
 *  0b0..MSTR_OSC
 *  0b1..PLL output divided by 2
 */
#define OCCS_CTRL_ZSRC_MASK                      (0x1U)
#define OCCS_CTRL_ZSRC_SHIFT                     (0U)
#define OCCS_CTRL_ZSRC(x)                        (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_ZSRC_SHIFT)) & OCCS_CTRL_ZSRC_MASK)
/*! PRECS - Prescaler Clock Select
 *  0b00..8 MHz relaxation oscillator selected (reset value)
 *  0b01..External reference selected
 *  0b10..200 kHz relaxation oscillator selected
 *  0b11..48 MHz relaxation oscillator selected
 */
#define OCCS_CTRL_PRECS_MASK                     (0xCU)
#define OCCS_CTRL_PRECS_SHIFT                    (2U)
#define OCCS_CTRL_PRECS(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PRECS_SHIFT)) & OCCS_CTRL_PRECS_MASK)
/*! PLLPD - PLL Power Down
 *  0b0..PLL enabled
 *  0b1..PLL powered down
 */
#define OCCS_CTRL_PLLPD_MASK                     (0x10U)
#define OCCS_CTRL_PLLPD_SHIFT                    (4U)
#define OCCS_CTRL_PLLPD(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLPD_SHIFT)) & OCCS_CTRL_PLLPD_MASK)
/*! LCKON - Lock Detector On
 *  0b0..Lock detector disabled
 *  0b1..Lock detector enabled
 */
#define OCCS_CTRL_LCKON_MASK                     (0x80U)
#define OCCS_CTRL_LCKON_SHIFT                    (7U)
#define OCCS_CTRL_LCKON(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_LCKON_SHIFT)) & OCCS_CTRL_LCKON_MASK)
/*! LOCIE - Loss of Reference Clock Interrupt Enable
 *  0b0..Interrupt disabled.
 *  0b1..Interrupt enabled.
 */
#define OCCS_CTRL_LOCIE_MASK                     (0x800U)
#define OCCS_CTRL_LOCIE_SHIFT                    (11U)
#define OCCS_CTRL_LOCIE(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_LOCIE_SHIFT)) & OCCS_CTRL_LOCIE_MASK)
/*! PLLIE0 - PLL Interrupt Enable 0
 *  0b00..Disable interrupt.
 *  0b01..Enable interrupt on any rising edge of LCK0.
 *  0b10..Enable interrupt on falling edge of LCK0.
 *  0b11..Enable interrupt on any edge change of LCK0.
 */
#define OCCS_CTRL_PLLIE0_MASK                    (0x3000U)
#define OCCS_CTRL_PLLIE0_SHIFT                   (12U)
#define OCCS_CTRL_PLLIE0(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLIE0_SHIFT)) & OCCS_CTRL_PLLIE0_MASK)
/*! PLLIE1 - PLL Interrupt Enable 1
 *  0b00..Disable interrupt.
 *  0b01..Enable interrupt on any rising edge of LCK1.
 *  0b10..Enable interrupt on falling edge of LCK1.
 *  0b11..Enable interrupt on any edge change of LCK1.
 */
#define OCCS_CTRL_PLLIE1_MASK                    (0xC000U)
#define OCCS_CTRL_PLLIE1_SHIFT                   (14U)
#define OCCS_CTRL_PLLIE1(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_CTRL_PLLIE1_SHIFT)) & OCCS_CTRL_PLLIE1_MASK)
/*! @} */

/*! @name DIVBY - PLL Divide-By Register */
/*! @{ */
#define OCCS_DIVBY_PLLDB_MASK                    (0x3FU)
#define OCCS_DIVBY_PLLDB_SHIFT                   (0U)
#define OCCS_DIVBY_PLLDB(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_PLLDB_SHIFT)) & OCCS_DIVBY_PLLDB_MASK)
/*! COD - Clock Output Divide or Postscaler
 *  0b0000..Divide clock output by 1.
 *  0b0001..Divide clock output by 2.
 *  0b0010..Divide clock output by 4.
 *  0b0011..Divide clock output by 8.
 *  0b0100..Divide clock output by 16.
 *  0b0101..Divide clock output by 32.
 *  0b0110..Divide clock output by 64.
 *  0b0111..Divide clock output by 128.
 *  0b1xxx..Divide clock output by 256.
 */
#define OCCS_DIVBY_COD_MASK                      (0xF00U)
#define OCCS_DIVBY_COD_SHIFT                     (8U)
#define OCCS_DIVBY_COD(x)                        (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_COD_SHIFT)) & OCCS_DIVBY_COD_MASK)
#define OCCS_DIVBY_LORTP_MASK                    (0xF000U)
#define OCCS_DIVBY_LORTP_SHIFT                   (12U)
#define OCCS_DIVBY_LORTP(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_DIVBY_LORTP_SHIFT)) & OCCS_DIVBY_LORTP_MASK)
/*! @} */

/*! @name STAT - OCCS Status Register */
/*! @{ */
/*! ZSRCS - CLOCK Source Status
 *  0b00..MSTR_OSC
 *  0b01..PLL output divided by 2
 *  0b1x..Synchronization in progress
 */
#define OCCS_STAT_ZSRCS_MASK                     (0x3U)
#define OCCS_STAT_ZSRCS_SHIFT                    (0U)
#define OCCS_STAT_ZSRCS(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_ZSRCS_SHIFT)) & OCCS_STAT_ZSRCS_MASK)
/*! PLLPDN - PLL Power Down
 *  0b0..PLL not powered down.
 *  0b1..PLL powered down.
 */
#define OCCS_STAT_PLLPDN_MASK                    (0x10U)
#define OCCS_STAT_PLLPDN_SHIFT                   (4U)
#define OCCS_STAT_PLLPDN(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_PLLPDN_SHIFT)) & OCCS_STAT_PLLPDN_MASK)
/*! LCK0 - PLL Lock 0 Status
 *  0b0..PLL is unlocked.
 *  0b1..PLL is locked (coarse).
 */
#define OCCS_STAT_LCK0_MASK                      (0x20U)
#define OCCS_STAT_LCK0_SHIFT                     (5U)
#define OCCS_STAT_LCK0(x)                        (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LCK0_SHIFT)) & OCCS_STAT_LCK0_MASK)
/*! LCK1 - PLL Lock 1 Status
 *  0b0..PLL is unlocked.
 *  0b1..PLL is locked (fine).
 */
#define OCCS_STAT_LCK1_MASK                      (0x40U)
#define OCCS_STAT_LCK1_SHIFT                     (6U)
#define OCCS_STAT_LCK1(x)                        (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LCK1_SHIFT)) & OCCS_STAT_LCK1_MASK)
/*! OSC_OK - OSC_OK Indicator from XOSC
 *  0b0..Oscillator clock is still not stable, or XOSC is disabled.
 *  0b1..XOSC OK indicator after crystal oscillator startup.
 */
#define OCCS_STAT_OSC_OK_MASK                    (0x80U)
#define OCCS_STAT_OSC_OK_SHIFT                   (7U)
#define OCCS_STAT_OSC_OK(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_OSC_OK_SHIFT)) & OCCS_STAT_OSC_OK_MASK)
/*! MON_FAILURE - XOSC Clock Monitor Failure Indicator
 *  0b0..No clock failure, or clock monitor is disabled.
 *  0b1..Clock failure detected on XOSC reference clock when clock monitor is enabled.
 */
#define OCCS_STAT_MON_FAILURE_MASK               (0x100U)
#define OCCS_STAT_MON_FAILURE_SHIFT              (8U)
#define OCCS_STAT_MON_FAILURE(x)                 (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_MON_FAILURE_SHIFT)) & OCCS_STAT_MON_FAILURE_MASK)
/*! LOCI - Loss of Reference Clock Interrupt
 *  0b0..Oscillator clock normal.
 *  0b1..Loss of oscillator clock detected.
 */
#define OCCS_STAT_LOCI_MASK                      (0x2000U)
#define OCCS_STAT_LOCI_SHIFT                     (13U)
#define OCCS_STAT_LOCI(x)                        (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOCI_SHIFT)) & OCCS_STAT_LOCI_MASK)
/*! LOLI0 - PLL Lock or Loss of Lock Interrupt 0
 *  0b0..No lock or loss of lock event has occurred.
 *  0b1..PLL lock status based on PLLIE0.
 */
#define OCCS_STAT_LOLI0_MASK                     (0x4000U)
#define OCCS_STAT_LOLI0_SHIFT                    (14U)
#define OCCS_STAT_LOLI0(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOLI0_SHIFT)) & OCCS_STAT_LOLI0_MASK)
/*! LOLI1 - PLL Lock or Loss of Lock Interrupt 1
 *  0b0..No lock or loss of lock event has occurred.
 *  0b1..PLL lock status based on PLLIE1.
 */
#define OCCS_STAT_LOLI1_MASK                     (0x8000U)
#define OCCS_STAT_LOLI1_SHIFT                    (15U)
#define OCCS_STAT_LOLI1(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_STAT_LOLI1_SHIFT)) & OCCS_STAT_LOLI1_MASK)
/*! @} */

/*! @name OSCTL1 - Oscillator Control Register 1 */
/*! @{ */
#define OCCS_OSCTL1_FREQ_TRIM8M_MASK             (0x3FFU)
#define OCCS_OSCTL1_FREQ_TRIM8M_SHIFT            (0U)
#define OCCS_OSCTL1_FREQ_TRIM8M(x)               (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_FREQ_TRIM8M_SHIFT)) & OCCS_OSCTL1_FREQ_TRIM8M_MASK)
/*! EXT_SEL - External Clock In Select
 *  0b0..Use the output of the crystal oscillator as the external clock input.
 *  0b1..Use CLKIN as the external clock input.
 */
#define OCCS_OSCTL1_EXT_SEL_MASK                 (0x400U)
#define OCCS_OSCTL1_EXT_SEL_SHIFT                (10U)
#define OCCS_OSCTL1_EXT_SEL(x)                   (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_EXT_SEL_SHIFT)) & OCCS_OSCTL1_EXT_SEL_MASK)
/*! CLK_MODE - Crystal Oscillator Clock Mode
 *  0b0..Crystal oscillator enabled.
 *  0b1..External clock bypass mode. This enables the crystal oscillator's external clock bypass mode and allows
 *       an external clock source on the EXTAL input of the oscillator to propagate directly to the oscillator's
 *       clock output.
 */
#define OCCS_OSCTL1_CLK_MODE_MASK                (0x1000U)
#define OCCS_OSCTL1_CLK_MODE_SHIFT               (12U)
#define OCCS_OSCTL1_CLK_MODE(x)                  (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_CLK_MODE_SHIFT)) & OCCS_OSCTL1_CLK_MODE_MASK)
/*! COHL - Crystal Oscillator High/Low Power Level
 *  0b0..High power mode.
 *  0b1..Low power mode.
 */
#define OCCS_OSCTL1_COHL_MASK                    (0x2000U)
#define OCCS_OSCTL1_COHL_SHIFT                   (13U)
#define OCCS_OSCTL1_COHL(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_COHL_SHIFT)) & OCCS_OSCTL1_COHL_MASK)
/*! ROSB - 8 MHz Relaxation Oscillator Standby
 *  0b0..Normal mode. The relaxation oscillator output frequency is 8 MHz.
 *  0b1..Standby mode. The relaxation oscillator output frequency is reduced to 400 kHz (+/-50%). The PLL should
 *       be disabled in this mode and MSTR_OSC should be selected as the output clock.
 */
#define OCCS_OSCTL1_ROSB_MASK                    (0x4000U)
#define OCCS_OSCTL1_ROSB_SHIFT                   (14U)
#define OCCS_OSCTL1_ROSB(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_ROSB_SHIFT)) & OCCS_OSCTL1_ROSB_MASK)
/*! ROPD - 8 MHz Relaxation Oscillator Power Down
 *  0b0..Relaxation oscillator enabled.
 *  0b1..Relaxation oscillator powered down.
 */
#define OCCS_OSCTL1_ROPD_MASK                    (0x8000U)
#define OCCS_OSCTL1_ROPD_SHIFT                   (15U)
#define OCCS_OSCTL1_ROPD(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL1_ROPD_SHIFT)) & OCCS_OSCTL1_ROPD_MASK)
/*! @} */

/*! @name OSCTL2 - Oscillator Control Register 2 */
/*! @{ */
#define OCCS_OSCTL2_FREQ_TRIM200K_MASK           (0x1FFU)
#define OCCS_OSCTL2_FREQ_TRIM200K_SHIFT          (0U)
#define OCCS_OSCTL2_FREQ_TRIM200K(x)             (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_FREQ_TRIM200K_SHIFT)) & OCCS_OSCTL2_FREQ_TRIM200K_MASK)
#define OCCS_OSCTL2_MON_ENABLE_MASK              (0x200U)
#define OCCS_OSCTL2_MON_ENABLE_SHIFT             (9U)
#define OCCS_OSCTL2_MON_ENABLE(x)                (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_MON_ENABLE_SHIFT)) & OCCS_OSCTL2_MON_ENABLE_MASK)
#define OCCS_OSCTL2_TEMP_TRIM8M_MASK             (0x3C00U)
#define OCCS_OSCTL2_TEMP_TRIM8M_SHIFT            (10U)
#define OCCS_OSCTL2_TEMP_TRIM8M(x)               (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_TEMP_TRIM8M_SHIFT)) & OCCS_OSCTL2_TEMP_TRIM8M_MASK)
/*! COPD - Crystal Oscillator Power Down
 *  0b0..Crystal oscillator enabled.
 *  0b1..Crystal oscillator powered down.
 */
#define OCCS_OSCTL2_COPD_MASK                    (0x4000U)
#define OCCS_OSCTL2_COPD_SHIFT                   (14U)
#define OCCS_OSCTL2_COPD(x)                      (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_COPD_SHIFT)) & OCCS_OSCTL2_COPD_MASK)
#define OCCS_OSCTL2_ROPD200K_MASK                (0x8000U)
#define OCCS_OSCTL2_ROPD200K_SHIFT               (15U)
#define OCCS_OSCTL2_ROPD200K(x)                  (((uint16_t)(((uint16_t)(x)) << OCCS_OSCTL2_ROPD200K_SHIFT)) & OCCS_OSCTL2_ROPD200K_MASK)
/*! @} */

/*! @name CLKCHKR - External Clock Check Reference */
/*! @{ */
#define OCCS_CLKCHKR_REF_CNT_MASK                (0x7FU)
#define OCCS_CLKCHKR_REF_CNT_SHIFT               (0U)
#define OCCS_CLKCHKR_REF_CNT(x)                  (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKR_REF_CNT_SHIFT)) & OCCS_CLKCHKR_REF_CNT_MASK)
/*! CHK_ENA - Check Enable
 *  0b0..Writing a low while the clock checking operation is in progress stops the check in its current state.
 *       Reading a low after a check has been started indicates that the check operation is complete and the final
 *       values are valid in the REF_CNT and TARGET_CNT fields.
 *  0b1..Writing a one clears the REF_CNT and TARGET_CNT fields and starts the clock checking function. The
 *       CLK_ENA bit remains high while the operation is in progress.
 */
#define OCCS_CLKCHKR_CHK_ENA_MASK                (0x8000U)
#define OCCS_CLKCHKR_CHK_ENA_SHIFT               (15U)
#define OCCS_CLKCHKR_CHK_ENA(x)                  (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKR_CHK_ENA_SHIFT)) & OCCS_CLKCHKR_CHK_ENA_MASK)
/*! @} */

/*! @name CLKCHKT - External Clock Check Target */
/*! @{ */
#define OCCS_CLKCHKT_TARGET_CNT_MASK             (0x7FFU)
#define OCCS_CLKCHKT_TARGET_CNT_SHIFT            (0U)
#define OCCS_CLKCHKT_TARGET_CNT(x)               (((uint16_t)(((uint16_t)(x)) << OCCS_CLKCHKT_TARGET_CNT_SHIFT)) & OCCS_CLKCHKT_TARGET_CNT_MASK)
/*! @} */

/*! @name PROT - Protection Register */
/*! @{ */
/*! PLLEP - PLL Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_PLLEP_MASK                     (0x3U)
#define OCCS_PROT_PLLEP_SHIFT                    (0U)
#define OCCS_PROT_PLLEP(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_PLLEP_SHIFT)) & OCCS_PROT_PLLEP_MASK)
/*! OSCEP - Oscillator Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_OSCEP_MASK                     (0xCU)
#define OCCS_PROT_OSCEP_SHIFT                    (2U)
#define OCCS_PROT_OSCEP(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_OSCEP_SHIFT)) & OCCS_PROT_OSCEP_MASK)
/*! FRQEP - Frequency Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define OCCS_PROT_FRQEP_MASK                     (0x30U)
#define OCCS_PROT_FRQEP_SHIFT                    (4U)
#define OCCS_PROT_FRQEP(x)                       (((uint16_t)(((uint16_t)(x)) << OCCS_PROT_FRQEP_SHIFT)) & OCCS_PROT_FRQEP_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group OCCS_Register_Masks */


/* OCCS - Peripheral instance base addresses */
/** Peripheral OCCS base address */
#define OCCS_BASE                                (0xE2B0u)
/** Peripheral OCCS base pointer */
#define OCCS                                     ((OCCS_Type *)OCCS_BASE)
/** Array initializer of OCCS peripheral base addresses */
#define OCCS_BASE_ADDRS                          { OCCS_BASE }
/** Array initializer of OCCS peripheral base pointers */
#define OCCS_BASE_PTRS                           { OCCS }

/*!
 * @}
 */ /* end of group OCCS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< PIT Control Register, offset: 0x0 */
  __IO uint16_t MOD;                               /**< PIT Modulo Register, offset: 0x1 */
  __I  uint16_t CNTR;                              /**< PIT Counter Register, offset: 0x2 */
} PIT_Type;

/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/*! @name CTRL - PIT Control Register */
/*! @{ */
/*! CNT_EN - Count Enable
 *  0b0..PIT counter reset (default).
 *  0b1..PIT counter active.
 */
#define PIT_CTRL_CNT_EN_MASK                     (0x1U)
#define PIT_CTRL_CNT_EN_SHIFT                    (0U)
#define PIT_CTRL_CNT_EN(x)                       (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_CNT_EN_SHIFT)) & PIT_CTRL_CNT_EN_MASK)
/*! PRIE - PIT Roll-Over Interrupt Enable.
 *  0b0..PIT roll-over interrupt disabled (default).
 *  0b1..PIT roll-over interrupt enabled.
 */
#define PIT_CTRL_PRIE_MASK                       (0x2U)
#define PIT_CTRL_PRIE_SHIFT                      (1U)
#define PIT_CTRL_PRIE(x)                         (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRIE_SHIFT)) & PIT_CTRL_PRIE_MASK)
/*! PRF - PIT Roll-Over Flag.
 *  0b0..PIT counter has not reached the modulo value. (default)
 *  0b1..PIT counter has reached the modulo value.
 */
#define PIT_CTRL_PRF_MASK                        (0x4U)
#define PIT_CTRL_PRF_SHIFT                       (2U)
#define PIT_CTRL_PRF(x)                          (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRF_SHIFT)) & PIT_CTRL_PRF_MASK)
/*! PRESCALER
 *  0b0000..Clock
 *  0b0001..Clock divided by 2
 *  0b0010..Clock divided by 4
 *  0b0011..Clock divided by 8
 *  0b0100..Clock divided by 16
 *  0b0101..Clock divided by 32
 *  0b0110..Clock divided by 64
 *  0b0111..Clock divided by 128
 *  0b1000..Clock divided by 256
 *  0b1001..Clock divided by 512
 *  0b1010..Clock divided by 1024
 *  0b1011..Clock divided by 2048
 *  0b1100..Clock divided by 4096
 *  0b1101..Clock divided by 8192
 *  0b1110..Clock divided by 16384
 *  0b1111..Clock divided by 32768
 */
#define PIT_CTRL_PRESCALER_MASK                  (0x78U)
#define PIT_CTRL_PRESCALER_SHIFT                 (3U)
#define PIT_CTRL_PRESCALER(x)                    (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_PRESCALER_SHIFT)) & PIT_CTRL_PRESCALER_MASK)
/*! CLKSEL
 *  0b00..Selects IPBus clock
 *  0b01..Selects alternate clock 1
 *  0b10..Selects alternate clock 2
 *  0b11..Selects alternate clock 3
 */
#define PIT_CTRL_CLKSEL_MASK                     (0x300U)
#define PIT_CTRL_CLKSEL_SHIFT                    (8U)
#define PIT_CTRL_CLKSEL(x)                       (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_CLKSEL_SHIFT)) & PIT_CTRL_CLKSEL_MASK)
/*! SLAVE
 *  0b0..CNT_EN from this PIT is used to control operation (default).
 *  0b1..CNT_EN from master PIT is used to control operation.
 */
#define PIT_CTRL_SLAVE_MASK                      (0x8000U)
#define PIT_CTRL_SLAVE_SHIFT                     (15U)
#define PIT_CTRL_SLAVE(x)                        (((uint16_t)(((uint16_t)(x)) << PIT_CTRL_SLAVE_SHIFT)) & PIT_CTRL_SLAVE_MASK)
/*! @} */

/*! @name MOD - PIT Modulo Register */
/*! @{ */
#define PIT_MOD_MODULO_VALUE_MASK                (0xFFFFU)
#define PIT_MOD_MODULO_VALUE_SHIFT               (0U)
#define PIT_MOD_MODULO_VALUE(x)                  (((uint16_t)(((uint16_t)(x)) << PIT_MOD_MODULO_VALUE_SHIFT)) & PIT_MOD_MODULO_VALUE_MASK)
/*! @} */

/*! @name CNTR - PIT Counter Register */
/*! @{ */
#define PIT_CNTR_COUNTER_VALUE_MASK              (0xFFFFU)
#define PIT_CNTR_COUNTER_VALUE_SHIFT             (0U)
#define PIT_CNTR_COUNTER_VALUE(x)                (((uint16_t)(((uint16_t)(x)) << PIT_CNTR_COUNTER_VALUE_SHIFT)) & PIT_CNTR_COUNTER_VALUE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT0 base address */
#define PIT0_BASE                                (0xE100u)
/** Peripheral PIT0 base pointer */
#define PIT0                                     ((PIT_Type *)PIT0_BASE)
/** Peripheral PIT1 base address */
#define PIT1_BASE                                (0xE110u)
/** Peripheral PIT1 base pointer */
#define PIT1                                     ((PIT_Type *)PIT1_BASE)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS                           { PIT0_BASE, PIT1_BASE }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT0, PIT1 }

/*!
 * @}
 */ /* end of group PIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x0 */
  __IO uint16_t STS;                               /**< Status Register, offset: 0x1 */
} PMC_Type;

/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */
/*! LV22IE - 2.2 V Low Voltage Interrupt Enable
 *  0b0..Disable setting the low voltage interrupt.
 *  0b1..Enable setting the low voltage interrupt.
 */
#define PMC_CTRL_LV22IE_MASK                     (0x1U)
#define PMC_CTRL_LV22IE_SHIFT                    (0U)
#define PMC_CTRL_LV22IE(x)                       (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_LV22IE_SHIFT)) & PMC_CTRL_LV22IE_MASK)
/*! LV27IE - 2.7 V Low Voltage Interrupt Enable
 *  0b0..Disable setting the low voltage interrupt.
 *  0b1..Enable setting the low voltage interrupt.
 */
#define PMC_CTRL_LV27IE_MASK                     (0x2U)
#define PMC_CTRL_LV27IE_SHIFT                    (1U)
#define PMC_CTRL_LV27IE(x)                       (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_LV27IE_SHIFT)) & PMC_CTRL_LV27IE_MASK)
/*! HV22IE - 2.2 V High Voltage Interrupt Enable
 *  0b0..Disable setting the high voltage interrupt.
 *  0b1..Enable setting the high voltage interrupt.
 */
#define PMC_CTRL_HV22IE_MASK                     (0x4U)
#define PMC_CTRL_HV22IE_SHIFT                    (2U)
#define PMC_CTRL_HV22IE(x)                       (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_HV22IE_SHIFT)) & PMC_CTRL_HV22IE_MASK)
/*! HV27IE - 2.7 V High Voltage Interrupt Enable
 *  0b0..Disable setting the high voltage interrupt.
 *  0b1..Enable setting the high voltage interrupt.
 */
#define PMC_CTRL_HV27IE_MASK                     (0x8U)
#define PMC_CTRL_HV27IE_SHIFT                    (3U)
#define PMC_CTRL_HV27IE(x)                       (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_HV27IE_SHIFT)) & PMC_CTRL_HV27IE_MASK)
/*! VRBEN - Voltage Reference Buffer Enable
 *  0b0..Disable voltage reference buffering.
 *  0b1..Enable voltage reference buffering.
 */
#define PMC_CTRL_VRBEN_MASK                      (0x80U)
#define PMC_CTRL_VRBEN_SHIFT                     (7U)
#define PMC_CTRL_VRBEN(x)                        (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_VRBEN_SHIFT)) & PMC_CTRL_VRBEN_MASK)
#define PMC_CTRL_TRIM_MASK                       (0xF000U)
#define PMC_CTRL_TRIM_SHIFT                      (12U)
#define PMC_CTRL_TRIM(x)                         (((uint16_t)(((uint16_t)(x)) << PMC_CTRL_TRIM_SHIFT)) & PMC_CTRL_TRIM_MASK)
/*! @} */

/*! @name STS - Status Register */
/*! @{ */
/*! LV22F - 2.2 V Low Voltage Flag
 *  0b0..3.3 V supply is not below the 2.2 V threshold.
 *  0b1..3.3 V supply is below the 2.2 V threshold.
 */
#define PMC_STS_LV22F_MASK                       (0x1U)
#define PMC_STS_LV22F_SHIFT                      (0U)
#define PMC_STS_LV22F(x)                         (((uint16_t)(((uint16_t)(x)) << PMC_STS_LV22F_SHIFT)) & PMC_STS_LV22F_MASK)
/*! LV27F - 2.7 V Low Voltage Flag
 *  0b0..3.3 V supply is not below the 2.7 V threshold.
 *  0b1..3.3 V supply is below the 2.7 V threshold.
 */
#define PMC_STS_LV27F_MASK                       (0x2U)
#define PMC_STS_LV27F_SHIFT                      (1U)
#define PMC_STS_LV27F(x)                         (((uint16_t)(((uint16_t)(x)) << PMC_STS_LV27F_SHIFT)) & PMC_STS_LV27F_MASK)
/*! SLV22F - Sticky 2.2 V Low Voltage Flag
 *  0b0..3.3 V supply has not dropped below the 2.2 V threshold.
 *  0b1..3.3 V supply has dropped below the 2.2 V threshold.
 */
#define PMC_STS_SLV22F_MASK                      (0x4U)
#define PMC_STS_SLV22F_SHIFT                     (2U)
#define PMC_STS_SLV22F(x)                        (((uint16_t)(((uint16_t)(x)) << PMC_STS_SLV22F_SHIFT)) & PMC_STS_SLV22F_MASK)
/*! SLV27F - Sticky 2.7 V Low Voltage Flag
 *  0b0..3.3 V supply has not dropped below the 2.7 V threshold.
 *  0b1..3.3 V supply has dropped below the 2.7 V threshold.
 */
#define PMC_STS_SLV27F_MASK                      (0x8U)
#define PMC_STS_SLV27F_SHIFT                     (3U)
#define PMC_STS_SLV27F(x)                        (((uint16_t)(((uint16_t)(x)) << PMC_STS_SLV27F_SHIFT)) & PMC_STS_SLV27F_MASK)
/*! LVI - Low Voltage Interrupt
 *  0b0..Low voltage interrupt cleared.
 *  0b1..Low voltage interrupt asserted.
 */
#define PMC_STS_LVI_MASK                         (0x10U)
#define PMC_STS_LVI_SHIFT                        (4U)
#define PMC_STS_LVI(x)                           (((uint16_t)(((uint16_t)(x)) << PMC_STS_LVI_SHIFT)) & PMC_STS_LVI_MASK)
/*! SR27 - Small Regulator 2.7 V Active Flag
 *  0b0..The small regulator 2.7 V supply is not ready to be used.
 *  0b1..The small regulator 2.7 V supply is ready to be used.
 */
#define PMC_STS_SR27_MASK                        (0x20U)
#define PMC_STS_SR27_SHIFT                       (5U)
#define PMC_STS_SR27(x)                          (((uint16_t)(((uint16_t)(x)) << PMC_STS_SR27_SHIFT)) & PMC_STS_SR27_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PMC_Register_Masks */


/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE                                 (0xE2A0u)
/** Peripheral PMC base pointer */
#define PMC                                      ((PMC_Type *)PMC_BASE)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS                           { PMC_BASE }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC }

/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Peripheral_Access_Layer PWM Peripheral Access Layer
 * @{
 */

/** PWM - Register Layout Typedef */
typedef struct {
  __I  uint16_t SM0CNT;                            /**< Counter Register, offset: 0x0 */
  __IO uint16_t SM0INIT;                           /**< Initial Count Register, offset: 0x2 */
  __IO uint16_t SM0CTRL2;                          /**< Control 2 Register, offset: 0x4 */
  __IO uint16_t SM0CTRL;                           /**< Control Register, offset: 0x6 */
       uint8_t RESERVED_0[2];
  __IO uint16_t SM0VAL0;                           /**< Value Register 0, offset: 0xA */
  __IO uint16_t SM0FRACVAL1;                       /**< Fractional Value Register 1, offset: 0xC */
  __IO uint16_t SM0VAL1;                           /**< Value Register 1, offset: 0xE */
  __IO uint16_t SM0FRACVAL2;                       /**< Fractional Value Register 2, offset: 0x10 */
  __IO uint16_t SM0VAL2;                           /**< Value Register 2, offset: 0x12 */
  __IO uint16_t SM0FRACVAL3;                       /**< Fractional Value Register 3, offset: 0x14 */
  __IO uint16_t SM0VAL3;                           /**< Value Register 3, offset: 0x16 */
  __IO uint16_t SM0FRACVAL4;                       /**< Fractional Value Register 4, offset: 0x18 */
  __IO uint16_t SM0VAL4;                           /**< Value Register 4, offset: 0x1A */
  __IO uint16_t SM0FRACVAL5;                       /**< Fractional Value Register 5, offset: 0x1C */
  __IO uint16_t SM0VAL5;                           /**< Value Register 5, offset: 0x1E */
  __IO uint16_t SM0FRCTRL;                         /**< Fractional Control Register, offset: 0x20 */
  __IO uint16_t SM0OCTRL;                          /**< Output Control Register, offset: 0x22 */
  __IO uint16_t SM0STS;                            /**< Status Register, offset: 0x24 */
  __IO uint16_t SM0INTEN;                          /**< Interrupt Enable Register, offset: 0x26 */
  __IO uint16_t SM0DMAEN;                          /**< DMA Enable Register, offset: 0x28 */
  __IO uint16_t SM0TCTRL;                          /**< Output Trigger Control Register, offset: 0x2A */
  __IO uint16_t SM0DISMAP0;                        /**< Fault Disable Mapping Register 0, offset: 0x2C */
  __IO uint16_t SM0DISMAP1;                        /**< Fault Disable Mapping Register 1, offset: 0x2E */
  __IO uint16_t SM0DTCNT0;                         /**< Deadtime Count Register 0, offset: 0x30 */
  __IO uint16_t SM0DTCNT1;                         /**< Deadtime Count Register 1, offset: 0x32 */
  __IO uint16_t SM0CAPTCTRLA;                      /**< Capture Control A Register, offset: 0x34 */
  __IO uint16_t SM0CAPTCOMPA;                      /**< Capture Compare A Register, offset: 0x36 */
  __IO uint16_t SM0CAPTCTRLB;                      /**< Capture Control B Register, offset: 0x38 */
  __IO uint16_t SM0CAPTCOMPB;                      /**< Capture Compare B Register, offset: 0x3A */
  __IO uint16_t SM0CAPTCTRLX;                      /**< Capture Control X Register, offset: 0x3C */
  __IO uint16_t SM0CAPTCOMPX;                      /**< Capture Compare X Register, offset: 0x3E */
  __I  uint16_t SM0CVAL0;                          /**< Capture Value 0 Register, offset: 0x40 */
  __I  uint16_t SM0CVAL0CYC;                       /**< Capture Value 0 Cycle Register, offset: 0x42 */
  __I  uint16_t SM0CVAL1;                          /**< Capture Value 1 Register, offset: 0x44 */
  __I  uint16_t SM0CVAL1CYC;                       /**< Capture Value 1 Cycle Register, offset: 0x46 */
  __I  uint16_t SM0CVAL2;                          /**< Capture Value 2 Register, offset: 0x48 */
  __I  uint16_t SM0CVAL2CYC;                       /**< Capture Value 2 Cycle Register, offset: 0x4A */
  __I  uint16_t SM0CVAL3;                          /**< Capture Value 3 Register, offset: 0x4C */
  __I  uint16_t SM0CVAL3CYC;                       /**< Capture Value 3 Cycle Register, offset: 0x4E */
  __I  uint16_t SM0CVAL4;                          /**< Capture Value 4 Register, offset: 0x50 */
  __I  uint16_t SM0CVAL4CYC;                       /**< Capture Value 4 Cycle Register, offset: 0x52 */
  __I  uint16_t SM0CVAL5;                          /**< Capture Value 5 Register, offset: 0x54 */
  __I  uint16_t SM0CVAL5CYC;                       /**< Capture Value 5 Cycle Register, offset: 0x56 */
  __IO uint16_t SM0PHASEDLY;                       /**< Phase Delay Register, offset: 0x58 */
       uint8_t RESERVED_1[6];
  __I  uint16_t SM1CNT;                            /**< Counter Register, offset: 0x60 */
  __IO uint16_t SM1INIT;                           /**< Initial Count Register, offset: 0x62 */
  __IO uint16_t SM1CTRL2;                          /**< Control 2 Register, offset: 0x64 */
  __IO uint16_t SM1CTRL;                           /**< Control Register, offset: 0x66 */
       uint8_t RESERVED_2[2];
  __IO uint16_t SM1VAL0;                           /**< Value Register 0, offset: 0x6A */
  __IO uint16_t SM1FRACVAL1;                       /**< Fractional Value Register 1, offset: 0x6C */
  __IO uint16_t SM1VAL1;                           /**< Value Register 1, offset: 0x6E */
  __IO uint16_t SM1FRACVAL2;                       /**< Fractional Value Register 2, offset: 0x70 */
  __IO uint16_t SM1VAL2;                           /**< Value Register 2, offset: 0x72 */
  __IO uint16_t SM1FRACVAL3;                       /**< Fractional Value Register 3, offset: 0x74 */
  __IO uint16_t SM1VAL3;                           /**< Value Register 3, offset: 0x76 */
  __IO uint16_t SM1FRACVAL4;                       /**< Fractional Value Register 4, offset: 0x78 */
  __IO uint16_t SM1VAL4;                           /**< Value Register 4, offset: 0x7A */
  __IO uint16_t SM1FRACVAL5;                       /**< Fractional Value Register 5, offset: 0x7C */
  __IO uint16_t SM1VAL5;                           /**< Value Register 5, offset: 0x7E */
  __IO uint16_t SM1FRCTRL;                         /**< Fractional Control Register, offset: 0x80 */
  __IO uint16_t SM1OCTRL;                          /**< Output Control Register, offset: 0x82 */
  __IO uint16_t SM1STS;                            /**< Status Register, offset: 0x84 */
  __IO uint16_t SM1INTEN;                          /**< Interrupt Enable Register, offset: 0x86 */
  __IO uint16_t SM1DMAEN;                          /**< DMA Enable Register, offset: 0x88 */
  __IO uint16_t SM1TCTRL;                          /**< Output Trigger Control Register, offset: 0x8A */
  __IO uint16_t SM1DISMAP0;                        /**< Fault Disable Mapping Register 0, offset: 0x8C */
  __IO uint16_t SM1DISMAP1;                        /**< Fault Disable Mapping Register 1, offset: 0x8E */
  __IO uint16_t SM1DTCNT0;                         /**< Deadtime Count Register 0, offset: 0x90 */
  __IO uint16_t SM1DTCNT1;                         /**< Deadtime Count Register 1, offset: 0x92 */
  __IO uint16_t SM1CAPTCTRLA;                      /**< Capture Control A Register, offset: 0x94 */
  __IO uint16_t SM1CAPTCOMPA;                      /**< Capture Compare A Register, offset: 0x96 */
  __IO uint16_t SM1CAPTCTRLB;                      /**< Capture Control B Register, offset: 0x98 */
  __IO uint16_t SM1CAPTCOMPB;                      /**< Capture Compare B Register, offset: 0x9A */
  __IO uint16_t SM1CAPTCTRLX;                      /**< Capture Control X Register, offset: 0x9C */
  __IO uint16_t SM1CAPTCOMPX;                      /**< Capture Compare X Register, offset: 0x9E */
  __I  uint16_t SM1CVAL0;                          /**< Capture Value 0 Register, offset: 0xA0 */
  __I  uint16_t SM1CVAL0CYC;                       /**< Capture Value 0 Cycle Register, offset: 0xA2 */
  __I  uint16_t SM1CVAL1;                          /**< Capture Value 1 Register, offset: 0xA4 */
  __I  uint16_t SM1CVAL1CYC;                       /**< Capture Value 1 Cycle Register, offset: 0xA6 */
  __I  uint16_t SM1CVAL2;                          /**< Capture Value 2 Register, offset: 0xA8 */
  __I  uint16_t SM1CVAL2CYC;                       /**< Capture Value 2 Cycle Register, offset: 0xAA */
  __I  uint16_t SM1CVAL3;                          /**< Capture Value 3 Register, offset: 0xAC */
  __I  uint16_t SM1CVAL3CYC;                       /**< Capture Value 3 Cycle Register, offset: 0xAE */
  __I  uint16_t SM1CVAL4;                          /**< Capture Value 4 Register, offset: 0xB0 */
  __I  uint16_t SM1CVAL4CYC;                       /**< Capture Value 4 Cycle Register, offset: 0xB2 */
  __I  uint16_t SM1CVAL5;                          /**< Capture Value 5 Register, offset: 0xB4 */
  __I  uint16_t SM1CVAL5CYC;                       /**< Capture Value 5 Cycle Register, offset: 0xB6 */
  __IO uint16_t SM1PHASEDLY;                       /**< Phase Delay Register, offset: 0xB8 */
       uint8_t RESERVED_3[6];
  __I  uint16_t SM2CNT;                            /**< Counter Register, offset: 0xC0 */
  __IO uint16_t SM2INIT;                           /**< Initial Count Register, offset: 0xC2 */
  __IO uint16_t SM2CTRL2;                          /**< Control 2 Register, offset: 0xC4 */
  __IO uint16_t SM2CTRL;                           /**< Control Register, offset: 0xC6 */
       uint8_t RESERVED_4[2];
  __IO uint16_t SM2VAL0;                           /**< Value Register 0, offset: 0xCA */
  __IO uint16_t SM2FRACVAL1;                       /**< Fractional Value Register 1, offset: 0xCC */
  __IO uint16_t SM2VAL1;                           /**< Value Register 1, offset: 0xCE */
  __IO uint16_t SM2FRACVAL2;                       /**< Fractional Value Register 2, offset: 0xD0 */
  __IO uint16_t SM2VAL2;                           /**< Value Register 2, offset: 0xD2 */
  __IO uint16_t SM2FRACVAL3;                       /**< Fractional Value Register 3, offset: 0xD4 */
  __IO uint16_t SM2VAL3;                           /**< Value Register 3, offset: 0xD6 */
  __IO uint16_t SM2FRACVAL4;                       /**< Fractional Value Register 4, offset: 0xD8 */
  __IO uint16_t SM2VAL4;                           /**< Value Register 4, offset: 0xDA */
  __IO uint16_t SM2FRACVAL5;                       /**< Fractional Value Register 5, offset: 0xDC */
  __IO uint16_t SM2VAL5;                           /**< Value Register 5, offset: 0xDE */
  __IO uint16_t SM2FRCTRL;                         /**< Fractional Control Register, offset: 0xE0 */
  __IO uint16_t SM2OCTRL;                          /**< Output Control Register, offset: 0xE2 */
  __IO uint16_t SM2STS;                            /**< Status Register, offset: 0xE4 */
  __IO uint16_t SM2INTEN;                          /**< Interrupt Enable Register, offset: 0xE6 */
  __IO uint16_t SM2DMAEN;                          /**< DMA Enable Register, offset: 0xE8 */
  __IO uint16_t SM2TCTRL;                          /**< Output Trigger Control Register, offset: 0xEA */
  __IO uint16_t SM2DISMAP0;                        /**< Fault Disable Mapping Register 0, offset: 0xEC */
  __IO uint16_t SM2DISMAP1;                        /**< Fault Disable Mapping Register 1, offset: 0xEE */
  __IO uint16_t SM2DTCNT0;                         /**< Deadtime Count Register 0, offset: 0xF0 */
  __IO uint16_t SM2DTCNT1;                         /**< Deadtime Count Register 1, offset: 0xF2 */
  __IO uint16_t SM2CAPTCTRLA;                      /**< Capture Control A Register, offset: 0xF4 */
  __IO uint16_t SM2CAPTCOMPA;                      /**< Capture Compare A Register, offset: 0xF6 */
  __IO uint16_t SM2CAPTCTRLB;                      /**< Capture Control B Register, offset: 0xF8 */
  __IO uint16_t SM2CAPTCOMPB;                      /**< Capture Compare B Register, offset: 0xFA */
  __IO uint16_t SM2CAPTCTRLX;                      /**< Capture Control X Register, offset: 0xFC */
  __IO uint16_t SM2CAPTCOMPX;                      /**< Capture Compare X Register, offset: 0xFE */
  __I  uint16_t SM2CVAL0;                          /**< Capture Value 0 Register, offset: 0x100 */
  __I  uint16_t SM2CVAL0CYC;                       /**< Capture Value 0 Cycle Register, offset: 0x102 */
  __I  uint16_t SM2CVAL1;                          /**< Capture Value 1 Register, offset: 0x104 */
  __I  uint16_t SM2CVAL1CYC;                       /**< Capture Value 1 Cycle Register, offset: 0x106 */
  __I  uint16_t SM2CVAL2;                          /**< Capture Value 2 Register, offset: 0x108 */
  __I  uint16_t SM2CVAL2CYC;                       /**< Capture Value 2 Cycle Register, offset: 0x10A */
  __I  uint16_t SM2CVAL3;                          /**< Capture Value 3 Register, offset: 0x10C */
  __I  uint16_t SM2CVAL3CYC;                       /**< Capture Value 3 Cycle Register, offset: 0x10E */
  __I  uint16_t SM2CVAL4;                          /**< Capture Value 4 Register, offset: 0x110 */
  __I  uint16_t SM2CVAL4CYC;                       /**< Capture Value 4 Cycle Register, offset: 0x112 */
  __I  uint16_t SM2CVAL5;                          /**< Capture Value 5 Register, offset: 0x114 */
  __I  uint16_t SM2CVAL5CYC;                       /**< Capture Value 5 Cycle Register, offset: 0x116 */
  __IO uint16_t SM2PHASEDLY;                       /**< Phase Delay Register, offset: 0x118 */
       uint8_t RESERVED_5[6];
  __I  uint16_t SM3CNT;                            /**< Counter Register, offset: 0x120 */
  __IO uint16_t SM3INIT;                           /**< Initial Count Register, offset: 0x122 */
  __IO uint16_t SM3CTRL2;                          /**< Control 2 Register, offset: 0x124 */
  __IO uint16_t SM3CTRL;                           /**< Control Register, offset: 0x126 */
       uint8_t RESERVED_6[2];
  __IO uint16_t SM3VAL0;                           /**< Value Register 0, offset: 0x12A */
  __IO uint16_t SM3FRACVAL1;                       /**< Fractional Value Register 1, offset: 0x12C */
  __IO uint16_t SM3VAL1;                           /**< Value Register 1, offset: 0x12E */
  __IO uint16_t SM3FRACVAL2;                       /**< Fractional Value Register 2, offset: 0x130 */
  __IO uint16_t SM3VAL2;                           /**< Value Register 2, offset: 0x132 */
  __IO uint16_t SM3FRACVAL3;                       /**< Fractional Value Register 3, offset: 0x134 */
  __IO uint16_t SM3VAL3;                           /**< Value Register 3, offset: 0x136 */
  __IO uint16_t SM3FRACVAL4;                       /**< Fractional Value Register 4, offset: 0x138 */
  __IO uint16_t SM3VAL4;                           /**< Value Register 4, offset: 0x13A */
  __IO uint16_t SM3FRACVAL5;                       /**< Fractional Value Register 5, offset: 0x13C */
  __IO uint16_t SM3VAL5;                           /**< Value Register 5, offset: 0x13E */
  __IO uint16_t SM3FRCTRL;                         /**< Fractional Control Register, offset: 0x140 */
  __IO uint16_t SM3OCTRL;                          /**< Output Control Register, offset: 0x142 */
  __IO uint16_t SM3STS;                            /**< Status Register, offset: 0x144 */
  __IO uint16_t SM3INTEN;                          /**< Interrupt Enable Register, offset: 0x146 */
  __IO uint16_t SM3DMAEN;                          /**< DMA Enable Register, offset: 0x148 */
  __IO uint16_t SM3TCTRL;                          /**< Output Trigger Control Register, offset: 0x14A */
  __IO uint16_t SM3DISMAP0;                        /**< Fault Disable Mapping Register 0, offset: 0x14C */
  __IO uint16_t SM3DISMAP1;                        /**< Fault Disable Mapping Register 1, offset: 0x14E */
  __IO uint16_t SM3DTCNT0;                         /**< Deadtime Count Register 0, offset: 0x150 */
  __IO uint16_t SM3DTCNT1;                         /**< Deadtime Count Register 1, offset: 0x152 */
  __IO uint16_t SM3CAPTCTRLA;                      /**< Capture Control A Register, offset: 0x154 */
  __IO uint16_t SM3CAPTCOMPA;                      /**< Capture Compare A Register, offset: 0x156 */
  __IO uint16_t SM3CAPTCTRLB;                      /**< Capture Control B Register, offset: 0x158 */
  __IO uint16_t SM3CAPTCOMPB;                      /**< Capture Compare B Register, offset: 0x15A */
  __IO uint16_t SM3CAPTCTRLX;                      /**< Capture Control X Register, offset: 0x15C */
  __IO uint16_t SM3CAPTCOMPX;                      /**< Capture Compare X Register, offset: 0x15E */
  __I  uint16_t SM3CVAL0;                          /**< Capture Value 0 Register, offset: 0x160 */
  __I  uint16_t SM3CVAL0CYC;                       /**< Capture Value 0 Cycle Register, offset: 0x162 */
  __I  uint16_t SM3CVAL1;                          /**< Capture Value 1 Register, offset: 0x164 */
  __I  uint16_t SM3CVAL1CYC;                       /**< Capture Value 1 Cycle Register, offset: 0x166 */
  __I  uint16_t SM3CVAL2;                          /**< Capture Value 2 Register, offset: 0x168 */
  __I  uint16_t SM3CVAL2CYC;                       /**< Capture Value 2 Cycle Register, offset: 0x16A */
  __I  uint16_t SM3CVAL3;                          /**< Capture Value 3 Register, offset: 0x16C */
  __I  uint16_t SM3CVAL3CYC;                       /**< Capture Value 3 Cycle Register, offset: 0x16E */
  __I  uint16_t SM3CVAL4;                          /**< Capture Value 4 Register, offset: 0x170 */
  __I  uint16_t SM3CVAL4CYC;                       /**< Capture Value 4 Cycle Register, offset: 0x172 */
  __I  uint16_t SM3CVAL5;                          /**< Capture Value 5 Register, offset: 0x174 */
  __I  uint16_t SM3CVAL5CYC;                       /**< Capture Value 5 Cycle Register, offset: 0x176 */
  __IO uint16_t SM3PHASEDLY;                       /**< Phase Delay Register, offset: 0x178 */
       uint8_t RESERVED_7[6];
  __IO uint16_t OUTEN;                             /**< Output Enable Register, offset: 0x180 */
  __IO uint16_t MASK;                              /**< Mask Register, offset: 0x182 */
  __IO uint16_t SWCOUT;                            /**< Software Controlled Output Register, offset: 0x184 */
  __IO uint16_t DTSRCSEL;                          /**< PWM Source Select Register, offset: 0x186 */
  __IO uint16_t MCTRL;                             /**< Master Control Register, offset: 0x188 */
  __IO uint16_t MCTRL2;                            /**< Master Control 2 Register, offset: 0x18A */
  __IO uint16_t FCTRL0;                            /**< Fault Control Register, offset: 0x18C */
  __IO uint16_t FSTS0;                             /**< Fault Status Register, offset: 0x18E */
  __IO uint16_t FFILT0;                            /**< Fault Filter Register, offset: 0x190 */
  __IO uint16_t FTST0;                             /**< Fault Test Register, offset: 0x192 */
  __IO uint16_t FCTRL20;                           /**< Fault Control 2 Register, offset: 0x194 */
       uint8_t RESERVED_8[2];
  __IO uint16_t FCTRL1;                            /**< Fault Control Register, offset: 0x198 */
  __IO uint16_t FSTS1;                             /**< Fault Status Register, offset: 0x19A */
  __IO uint16_t FFILT1;                            /**< Fault Filter Register, offset: 0x19C */
  __IO uint16_t FTST1;                             /**< Fault Test Register, offset: 0x19E */
  __IO uint16_t FCTRL21;                           /**< Fault Control 2 Register, offset: 0x1A0 */
} PWM_Type;

/* ----------------------------------------------------------------------------
   -- PWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PWM_Register_Masks PWM Register Masks
 * @{
 */

/*! @name SM0CNT - Counter Register */
/*! @{ */
#define PWM_SM0CNT_CNT_MASK                      (0xFFFFU)
#define PWM_SM0CNT_CNT_SHIFT                     (0U)
#define PWM_SM0CNT_CNT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM0CNT_CNT_SHIFT)) & PWM_SM0CNT_CNT_MASK)
/*! @} */

/*! @name SM0INIT - Initial Count Register */
/*! @{ */
#define PWM_SM0INIT_INIT_MASK                    (0xFFFFU)
#define PWM_SM0INIT_INIT_SHIFT                   (0U)
#define PWM_SM0INIT_INIT(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0INIT_INIT_SHIFT)) & PWM_SM0INIT_INIT_MASK)
/*! @} */

/*! @name SM0CTRL2 - Control 2 Register */
/*! @{ */
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it will force the clock to logic 0.
 *  0b11..reserved
 */
#define PWM_SM0CTRL2_CLK_SEL_MASK                (0x3U)
#define PWM_SM0CTRL2_CLK_SEL_SHIFT               (0U)
#define PWM_SM0CTRL2_CLK_SEL(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_CLK_SEL_SHIFT)) & PWM_SM0CTRL2_CLK_SEL_MASK)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it will force the RELOAD signal to logic 0.
 */
#define PWM_SM0CTRL2_RELOAD_SEL_MASK             (0x4U)
#define PWM_SM0CTRL2_RELOAD_SEL_SHIFT            (2U)
#define PWM_SM0CTRL2_RELOAD_SEL(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_RELOAD_SEL_SHIFT)) & PWM_SM0CTRL2_RELOAD_SEL_MASK)
/*! FORCE_SEL - This read/write bit determines the source of the FORCE OUTPUT signal for this submodule.
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_SM0CTRL2_FORCE_SEL_MASK              (0x38U)
#define PWM_SM0CTRL2_FORCE_SEL_SHIFT             (3U)
#define PWM_SM0CTRL2_FORCE_SEL(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_FORCE_SEL_SHIFT)) & PWM_SM0CTRL2_FORCE_SEL_MASK)
#define PWM_SM0CTRL2_FORCE_MASK                  (0x40U)
#define PWM_SM0CTRL2_FORCE_SHIFT                 (6U)
#define PWM_SM0CTRL2_FORCE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_FORCE_SHIFT)) & PWM_SM0CTRL2_FORCE_MASK)
/*! FRCEN - FRCEN
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_SM0CTRL2_FRCEN_MASK                  (0x80U)
#define PWM_SM0CTRL2_FRCEN_SHIFT                 (7U)
#define PWM_SM0CTRL2_FRCEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_FRCEN_SHIFT)) & PWM_SM0CTRL2_FRCEN_MASK)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it will force the INIT signal to logic 0. The submodule counter will only reinitialize when a master
 *        reload occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it
 *        will force the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_SM0CTRL2_INIT_SEL_MASK               (0x300U)
#define PWM_SM0CTRL2_INIT_SEL_SHIFT              (8U)
#define PWM_SM0CTRL2_INIT_SEL(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_INIT_SEL_SHIFT)) & PWM_SM0CTRL2_INIT_SEL_MASK)
#define PWM_SM0CTRL2_PWMX_INIT_MASK              (0x400U)
#define PWM_SM0CTRL2_PWMX_INIT_SHIFT             (10U)
#define PWM_SM0CTRL2_PWMX_INIT(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_PWMX_INIT_SHIFT)) & PWM_SM0CTRL2_PWMX_INIT_MASK)
#define PWM_SM0CTRL2_PWM45_INIT_MASK             (0x800U)
#define PWM_SM0CTRL2_PWM45_INIT_SHIFT            (11U)
#define PWM_SM0CTRL2_PWM45_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_PWM45_INIT_SHIFT)) & PWM_SM0CTRL2_PWM45_INIT_MASK)
#define PWM_SM0CTRL2_PWM23_INIT_MASK             (0x1000U)
#define PWM_SM0CTRL2_PWM23_INIT_SHIFT            (12U)
#define PWM_SM0CTRL2_PWM23_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_PWM23_INIT_SHIFT)) & PWM_SM0CTRL2_PWM23_INIT_MASK)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_SM0CTRL2_INDEP_MASK                  (0x2000U)
#define PWM_SM0CTRL2_INDEP_SHIFT                 (13U)
#define PWM_SM0CTRL2_INDEP(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_INDEP_SHIFT)) & PWM_SM0CTRL2_INDEP_MASK)
#define PWM_SM0CTRL2_WAITEN_MASK                 (0x4000U)
#define PWM_SM0CTRL2_WAITEN_SHIFT                (14U)
#define PWM_SM0CTRL2_WAITEN(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_WAITEN_SHIFT)) & PWM_SM0CTRL2_WAITEN_MASK)
#define PWM_SM0CTRL2_DBGEN_MASK                  (0x8000U)
#define PWM_SM0CTRL2_DBGEN_SHIFT                 (15U)
#define PWM_SM0CTRL2_DBGEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL2_DBGEN_SHIFT)) & PWM_SM0CTRL2_DBGEN_MASK)
/*! @} */

/*! @name SM0CTRL - Control Register */
/*! @{ */
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_SM0CTRL_DBLEN_MASK                   (0x1U)
#define PWM_SM0CTRL_DBLEN_SHIFT                  (0U)
#define PWM_SM0CTRL_DBLEN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_DBLEN_SHIFT)) & PWM_SM0CTRL_DBLEN_MASK)
/*! DBLX - PWMX Double Switching Enable
 *  0b0..PWMX double pulse disabled.
 *  0b1..PWMX double pulse enabled.
 */
#define PWM_SM0CTRL_DBLX_MASK                    (0x2U)
#define PWM_SM0CTRL_DBLX_SHIFT                   (1U)
#define PWM_SM0CTRL_DBLX(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_DBLX_SHIFT)) & PWM_SM0CTRL_DBLX_MASK)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_SM0CTRL_LDMOD_MASK                   (0x4U)
#define PWM_SM0CTRL_LDMOD_SHIFT                  (2U)
#define PWM_SM0CTRL_LDMOD(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_LDMOD_SHIFT)) & PWM_SM0CTRL_LDMOD_MASK)
/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
 *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
 *  0b1..DBLPWM is split to PWMA and PWMB.
 */
#define PWM_SM0CTRL_SPLIT_MASK                   (0x8U)
#define PWM_SM0CTRL_SPLIT_SHIFT                  (3U)
#define PWM_SM0CTRL_SPLIT(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_SPLIT_SHIFT)) & PWM_SM0CTRL_SPLIT_MASK)
/*! PRSC - Prescaler
 *  0b000..PWM clock frequency = fclk
 *  0b001..PWM clock frequency = fclk/2
 *  0b010..PWM clock frequency = fclk/4
 *  0b011..PWM clock frequency = fclk/8
 *  0b100..PWM clock frequency = fclk/16
 *  0b101..PWM clock frequency = fclk/32
 *  0b110..PWM clock frequency = fclk/64
 *  0b111..PWM clock frequency = fclk/128
 */
#define PWM_SM0CTRL_PRSC_MASK                    (0x70U)
#define PWM_SM0CTRL_PRSC_SHIFT                   (4U)
#define PWM_SM0CTRL_PRSC(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_PRSC_SHIFT)) & PWM_SM0CTRL_PRSC_MASK)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
 *       output in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_SM0CTRL_COMPMODE_MASK                (0x80U)
#define PWM_SM0CTRL_COMPMODE_SHIFT               (7U)
#define PWM_SM0CTRL_COMPMODE(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_COMPMODE_SHIFT)) & PWM_SM0CTRL_COMPMODE_MASK)
#define PWM_SM0CTRL_DT_MASK                      (0x300U)
#define PWM_SM0CTRL_DT_SHIFT                     (8U)
#define PWM_SM0CTRL_DT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_DT_SHIFT)) & PWM_SM0CTRL_DT_MASK)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_SM0CTRL_FULL_MASK                    (0x400U)
#define PWM_SM0CTRL_FULL_SHIFT                   (10U)
#define PWM_SM0CTRL_FULL(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_FULL_SHIFT)) & PWM_SM0CTRL_FULL_MASK)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_SM0CTRL_HALF_MASK                    (0x800U)
#define PWM_SM0CTRL_HALF_SHIFT                   (11U)
#define PWM_SM0CTRL_HALF(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_HALF_SHIFT)) & PWM_SM0CTRL_HALF_MASK)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_SM0CTRL_LDFQ_MASK                    (0xF000U)
#define PWM_SM0CTRL_LDFQ_SHIFT                   (12U)
#define PWM_SM0CTRL_LDFQ(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0CTRL_LDFQ_SHIFT)) & PWM_SM0CTRL_LDFQ_MASK)
/*! @} */

/*! @name SM0VAL0 - Value Register 0 */
/*! @{ */
#define PWM_SM0VAL0_VAL0_MASK                    (0xFFFFU)
#define PWM_SM0VAL0_VAL0_SHIFT                   (0U)
#define PWM_SM0VAL0_VAL0(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL0_VAL0_SHIFT)) & PWM_SM0VAL0_VAL0_MASK)
/*! @} */

/*! @name SM0FRACVAL1 - Fractional Value Register 1 */
/*! @{ */
#define PWM_SM0FRACVAL1_FRACVAL1_MASK            (0xF800U)
#define PWM_SM0FRACVAL1_FRACVAL1_SHIFT           (11U)
#define PWM_SM0FRACVAL1_FRACVAL1(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRACVAL1_FRACVAL1_SHIFT)) & PWM_SM0FRACVAL1_FRACVAL1_MASK)
/*! @} */

/*! @name SM0VAL1 - Value Register 1 */
/*! @{ */
#define PWM_SM0VAL1_VAL1_MASK                    (0xFFFFU)
#define PWM_SM0VAL1_VAL1_SHIFT                   (0U)
#define PWM_SM0VAL1_VAL1(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL1_VAL1_SHIFT)) & PWM_SM0VAL1_VAL1_MASK)
/*! @} */

/*! @name SM0FRACVAL2 - Fractional Value Register 2 */
/*! @{ */
#define PWM_SM0FRACVAL2_FRACVAL2_MASK            (0xF800U)
#define PWM_SM0FRACVAL2_FRACVAL2_SHIFT           (11U)
#define PWM_SM0FRACVAL2_FRACVAL2(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRACVAL2_FRACVAL2_SHIFT)) & PWM_SM0FRACVAL2_FRACVAL2_MASK)
/*! @} */

/*! @name SM0VAL2 - Value Register 2 */
/*! @{ */
#define PWM_SM0VAL2_VAL2_MASK                    (0xFFFFU)
#define PWM_SM0VAL2_VAL2_SHIFT                   (0U)
#define PWM_SM0VAL2_VAL2(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL2_VAL2_SHIFT)) & PWM_SM0VAL2_VAL2_MASK)
/*! @} */

/*! @name SM0FRACVAL3 - Fractional Value Register 3 */
/*! @{ */
#define PWM_SM0FRACVAL3_FRACVAL3_MASK            (0xF800U)
#define PWM_SM0FRACVAL3_FRACVAL3_SHIFT           (11U)
#define PWM_SM0FRACVAL3_FRACVAL3(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRACVAL3_FRACVAL3_SHIFT)) & PWM_SM0FRACVAL3_FRACVAL3_MASK)
/*! @} */

/*! @name SM0VAL3 - Value Register 3 */
/*! @{ */
#define PWM_SM0VAL3_VAL3_MASK                    (0xFFFFU)
#define PWM_SM0VAL3_VAL3_SHIFT                   (0U)
#define PWM_SM0VAL3_VAL3(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL3_VAL3_SHIFT)) & PWM_SM0VAL3_VAL3_MASK)
/*! @} */

/*! @name SM0FRACVAL4 - Fractional Value Register 4 */
/*! @{ */
#define PWM_SM0FRACVAL4_FRACVAL4_MASK            (0xF800U)
#define PWM_SM0FRACVAL4_FRACVAL4_SHIFT           (11U)
#define PWM_SM0FRACVAL4_FRACVAL4(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRACVAL4_FRACVAL4_SHIFT)) & PWM_SM0FRACVAL4_FRACVAL4_MASK)
/*! @} */

/*! @name SM0VAL4 - Value Register 4 */
/*! @{ */
#define PWM_SM0VAL4_VAL4_MASK                    (0xFFFFU)
#define PWM_SM0VAL4_VAL4_SHIFT                   (0U)
#define PWM_SM0VAL4_VAL4(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL4_VAL4_SHIFT)) & PWM_SM0VAL4_VAL4_MASK)
/*! @} */

/*! @name SM0FRACVAL5 - Fractional Value Register 5 */
/*! @{ */
#define PWM_SM0FRACVAL5_FRACVAL5_MASK            (0xF800U)
#define PWM_SM0FRACVAL5_FRACVAL5_SHIFT           (11U)
#define PWM_SM0FRACVAL5_FRACVAL5(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRACVAL5_FRACVAL5_SHIFT)) & PWM_SM0FRACVAL5_FRACVAL5_MASK)
/*! @} */

/*! @name SM0VAL5 - Value Register 5 */
/*! @{ */
#define PWM_SM0VAL5_VAL5_MASK                    (0xFFFFU)
#define PWM_SM0VAL5_VAL5_SHIFT                   (0U)
#define PWM_SM0VAL5_VAL5(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0VAL5_VAL5_SHIFT)) & PWM_SM0VAL5_VAL5_MASK)
/*! @} */

/*! @name SM0FRCTRL - Fractional Control Register */
/*! @{ */
/*! FRAC1_EN - Fractional Cycle PWM Period Enable
 *  0b0..Disable fractional cycle length for the PWM period.
 *  0b1..Enable fractional cycle length for the PWM period.
 */
#define PWM_SM0FRCTRL_FRAC1_EN_MASK              (0x2U)
#define PWM_SM0FRCTRL_FRAC1_EN_SHIFT             (1U)
#define PWM_SM0FRCTRL_FRAC1_EN(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRCTRL_FRAC1_EN_SHIFT)) & PWM_SM0FRCTRL_FRAC1_EN_MASK)
/*! FRAC23_EN - Fractional Cycle Placement Enable for PWM_A
 *  0b0..Disable fractional cycle placement for PWM_A.
 *  0b1..Enable fractional cycle placement for PWM_A.
 */
#define PWM_SM0FRCTRL_FRAC23_EN_MASK             (0x4U)
#define PWM_SM0FRCTRL_FRAC23_EN_SHIFT            (2U)
#define PWM_SM0FRCTRL_FRAC23_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRCTRL_FRAC23_EN_SHIFT)) & PWM_SM0FRCTRL_FRAC23_EN_MASK)
/*! FRAC45_EN - Fractional Cycle Placement Enable for PWM_B
 *  0b0..Disable fractional cycle placement for PWM_B.
 *  0b1..Enable fractional cycle placement for PWM_B.
 */
#define PWM_SM0FRCTRL_FRAC45_EN_MASK             (0x10U)
#define PWM_SM0FRCTRL_FRAC45_EN_SHIFT            (4U)
#define PWM_SM0FRCTRL_FRAC45_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRCTRL_FRAC45_EN_SHIFT)) & PWM_SM0FRCTRL_FRAC45_EN_MASK)
/*! FRAC_PU - Fractional Delay Circuit Power Up
 *  0b0..Turn off fractional delay logic.
 *  0b1..Power up fractional delay logic.
 */
#define PWM_SM0FRCTRL_FRAC_PU_MASK               (0x100U)
#define PWM_SM0FRCTRL_FRAC_PU_SHIFT              (8U)
#define PWM_SM0FRCTRL_FRAC_PU(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRCTRL_FRAC_PU_SHIFT)) & PWM_SM0FRCTRL_FRAC_PU_MASK)
#define PWM_SM0FRCTRL_TEST_MASK                  (0x8000U)
#define PWM_SM0FRCTRL_TEST_SHIFT                 (15U)
#define PWM_SM0FRCTRL_TEST(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0FRCTRL_TEST_SHIFT)) & PWM_SM0FRCTRL_TEST_MASK)
/*! @} */

/*! @name SM0OCTRL - Output Control Register */
/*! @{ */
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM0OCTRL_PWMXFS_MASK                 (0x3U)
#define PWM_SM0OCTRL_PWMXFS_SHIFT                (0U)
#define PWM_SM0OCTRL_PWMXFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMXFS_SHIFT)) & PWM_SM0OCTRL_PWMXFS_MASK)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM0OCTRL_PWMBFS_MASK                 (0xCU)
#define PWM_SM0OCTRL_PWMBFS_SHIFT                (2U)
#define PWM_SM0OCTRL_PWMBFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMBFS_SHIFT)) & PWM_SM0OCTRL_PWMBFS_MASK)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM0OCTRL_PWMAFS_MASK                 (0x30U)
#define PWM_SM0OCTRL_PWMAFS_SHIFT                (4U)
#define PWM_SM0OCTRL_PWMAFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMAFS_SHIFT)) & PWM_SM0OCTRL_PWMAFS_MASK)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_SM0OCTRL_POLX_MASK                   (0x100U)
#define PWM_SM0OCTRL_POLX_SHIFT                  (8U)
#define PWM_SM0OCTRL_POLX(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_POLX_SHIFT)) & PWM_SM0OCTRL_POLX_MASK)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_SM0OCTRL_POLB_MASK                   (0x200U)
#define PWM_SM0OCTRL_POLB_SHIFT                  (9U)
#define PWM_SM0OCTRL_POLB(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_POLB_SHIFT)) & PWM_SM0OCTRL_POLB_MASK)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_SM0OCTRL_POLA_MASK                   (0x400U)
#define PWM_SM0OCTRL_POLA_SHIFT                  (10U)
#define PWM_SM0OCTRL_POLA(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_POLA_SHIFT)) & PWM_SM0OCTRL_POLA_MASK)
#define PWM_SM0OCTRL_PWMX_IN_MASK                (0x2000U)
#define PWM_SM0OCTRL_PWMX_IN_SHIFT               (13U)
#define PWM_SM0OCTRL_PWMX_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMX_IN_SHIFT)) & PWM_SM0OCTRL_PWMX_IN_MASK)
#define PWM_SM0OCTRL_PWMB_IN_MASK                (0x4000U)
#define PWM_SM0OCTRL_PWMB_IN_SHIFT               (14U)
#define PWM_SM0OCTRL_PWMB_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMB_IN_SHIFT)) & PWM_SM0OCTRL_PWMB_IN_MASK)
#define PWM_SM0OCTRL_PWMA_IN_MASK                (0x8000U)
#define PWM_SM0OCTRL_PWMA_IN_SHIFT               (15U)
#define PWM_SM0OCTRL_PWMA_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0OCTRL_PWMA_IN_SHIFT)) & PWM_SM0OCTRL_PWMA_IN_MASK)
/*! @} */

/*! @name SM0STS - Status Register */
/*! @{ */
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_SM0STS_CMPF_MASK                     (0x3FU)
#define PWM_SM0STS_CMPF_SHIFT                    (0U)
#define PWM_SM0STS_CMPF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CMPF_SHIFT)) & PWM_SM0STS_CMPF_MASK)
#define PWM_SM0STS_CFX0_MASK                     (0x40U)
#define PWM_SM0STS_CFX0_SHIFT                    (6U)
#define PWM_SM0STS_CFX0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFX0_SHIFT)) & PWM_SM0STS_CFX0_MASK)
#define PWM_SM0STS_CFX1_MASK                     (0x80U)
#define PWM_SM0STS_CFX1_SHIFT                    (7U)
#define PWM_SM0STS_CFX1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFX1_SHIFT)) & PWM_SM0STS_CFX1_MASK)
#define PWM_SM0STS_CFB0_MASK                     (0x100U)
#define PWM_SM0STS_CFB0_SHIFT                    (8U)
#define PWM_SM0STS_CFB0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFB0_SHIFT)) & PWM_SM0STS_CFB0_MASK)
#define PWM_SM0STS_CFB1_MASK                     (0x200U)
#define PWM_SM0STS_CFB1_SHIFT                    (9U)
#define PWM_SM0STS_CFB1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFB1_SHIFT)) & PWM_SM0STS_CFB1_MASK)
#define PWM_SM0STS_CFA0_MASK                     (0x400U)
#define PWM_SM0STS_CFA0_SHIFT                    (10U)
#define PWM_SM0STS_CFA0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFA0_SHIFT)) & PWM_SM0STS_CFA0_MASK)
#define PWM_SM0STS_CFA1_MASK                     (0x800U)
#define PWM_SM0STS_CFA1_SHIFT                    (11U)
#define PWM_SM0STS_CFA1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_CFA1_SHIFT)) & PWM_SM0STS_CFA1_MASK)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_SM0STS_RF_MASK                       (0x1000U)
#define PWM_SM0STS_RF_SHIFT                      (12U)
#define PWM_SM0STS_RF(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_RF_SHIFT)) & PWM_SM0STS_RF_MASK)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_SM0STS_REF_MASK                      (0x2000U)
#define PWM_SM0STS_REF_SHIFT                     (13U)
#define PWM_SM0STS_REF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_REF_SHIFT)) & PWM_SM0STS_REF_MASK)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_SM0STS_RUF_MASK                      (0x4000U)
#define PWM_SM0STS_RUF_SHIFT                     (14U)
#define PWM_SM0STS_RUF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM0STS_RUF_SHIFT)) & PWM_SM0STS_RUF_MASK)
/*! @} */

/*! @name SM0INTEN - Interrupt Enable Register */
/*! @{ */
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_SM0INTEN_CMPIE_MASK                  (0x3FU)
#define PWM_SM0INTEN_CMPIE_SHIFT                 (0U)
#define PWM_SM0INTEN_CMPIE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CMPIE_SHIFT)) & PWM_SM0INTEN_CMPIE_MASK)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_SM0INTEN_CX0IE_MASK                  (0x40U)
#define PWM_SM0INTEN_CX0IE_SHIFT                 (6U)
#define PWM_SM0INTEN_CX0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CX0IE_SHIFT)) & PWM_SM0INTEN_CX0IE_MASK)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_SM0INTEN_CX1IE_MASK                  (0x80U)
#define PWM_SM0INTEN_CX1IE_SHIFT                 (7U)
#define PWM_SM0INTEN_CX1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CX1IE_SHIFT)) & PWM_SM0INTEN_CX1IE_MASK)
/*! CB0IE - Capture B 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB0].
 *  0b1..Interrupt request enabled for STS[CFB0].
 */
#define PWM_SM0INTEN_CB0IE_MASK                  (0x100U)
#define PWM_SM0INTEN_CB0IE_SHIFT                 (8U)
#define PWM_SM0INTEN_CB0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CB0IE_SHIFT)) & PWM_SM0INTEN_CB0IE_MASK)
/*! CB1IE - Capture B 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB1].
 *  0b1..Interrupt request enabled for STS[CFB1].
 */
#define PWM_SM0INTEN_CB1IE_MASK                  (0x200U)
#define PWM_SM0INTEN_CB1IE_SHIFT                 (9U)
#define PWM_SM0INTEN_CB1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CB1IE_SHIFT)) & PWM_SM0INTEN_CB1IE_MASK)
/*! CA0IE - Capture A 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA0].
 *  0b1..Interrupt request enabled for STS[CFA0].
 */
#define PWM_SM0INTEN_CA0IE_MASK                  (0x400U)
#define PWM_SM0INTEN_CA0IE_SHIFT                 (10U)
#define PWM_SM0INTEN_CA0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CA0IE_SHIFT)) & PWM_SM0INTEN_CA0IE_MASK)
/*! CA1IE - Capture A 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA1].
 *  0b1..Interrupt request enabled for STS[CFA1].
 */
#define PWM_SM0INTEN_CA1IE_MASK                  (0x800U)
#define PWM_SM0INTEN_CA1IE_SHIFT                 (11U)
#define PWM_SM0INTEN_CA1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_CA1IE_SHIFT)) & PWM_SM0INTEN_CA1IE_MASK)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_SM0INTEN_RIE_MASK                    (0x1000U)
#define PWM_SM0INTEN_RIE_SHIFT                   (12U)
#define PWM_SM0INTEN_RIE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_RIE_SHIFT)) & PWM_SM0INTEN_RIE_MASK)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_SM0INTEN_REIE_MASK                   (0x2000U)
#define PWM_SM0INTEN_REIE_SHIFT                  (13U)
#define PWM_SM0INTEN_REIE(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0INTEN_REIE_SHIFT)) & PWM_SM0INTEN_REIE_MASK)
/*! @} */

/*! @name SM0DMAEN - DMA Enable Register */
/*! @{ */
#define PWM_SM0DMAEN_CX0DE_MASK                  (0x1U)
#define PWM_SM0DMAEN_CX0DE_SHIFT                 (0U)
#define PWM_SM0DMAEN_CX0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CX0DE_SHIFT)) & PWM_SM0DMAEN_CX0DE_MASK)
#define PWM_SM0DMAEN_CX1DE_MASK                  (0x2U)
#define PWM_SM0DMAEN_CX1DE_SHIFT                 (1U)
#define PWM_SM0DMAEN_CX1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CX1DE_SHIFT)) & PWM_SM0DMAEN_CX1DE_MASK)
#define PWM_SM0DMAEN_CB0DE_MASK                  (0x4U)
#define PWM_SM0DMAEN_CB0DE_SHIFT                 (2U)
#define PWM_SM0DMAEN_CB0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CB0DE_SHIFT)) & PWM_SM0DMAEN_CB0DE_MASK)
#define PWM_SM0DMAEN_CB1DE_MASK                  (0x8U)
#define PWM_SM0DMAEN_CB1DE_SHIFT                 (3U)
#define PWM_SM0DMAEN_CB1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CB1DE_SHIFT)) & PWM_SM0DMAEN_CB1DE_MASK)
#define PWM_SM0DMAEN_CA0DE_MASK                  (0x10U)
#define PWM_SM0DMAEN_CA0DE_SHIFT                 (4U)
#define PWM_SM0DMAEN_CA0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CA0DE_SHIFT)) & PWM_SM0DMAEN_CA0DE_MASK)
#define PWM_SM0DMAEN_CA1DE_MASK                  (0x20U)
#define PWM_SM0DMAEN_CA1DE_SHIFT                 (5U)
#define PWM_SM0DMAEN_CA1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CA1DE_SHIFT)) & PWM_SM0DMAEN_CA1DE_MASK)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to also be set in order to determine to
 *        which watermark(s) the DMA request is sensitive.
 *  0b10..A local sync (VAL1 matches counter) sets the read DMA request.
 *  0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_SM0DMAEN_CAPTDE_MASK                 (0xC0U)
#define PWM_SM0DMAEN_CAPTDE_SHIFT                (6U)
#define PWM_SM0DMAEN_CAPTDE(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_CAPTDE_SHIFT)) & PWM_SM0DMAEN_CAPTDE_MASK)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_SM0DMAEN_FAND_MASK                   (0x100U)
#define PWM_SM0DMAEN_FAND_SHIFT                  (8U)
#define PWM_SM0DMAEN_FAND(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_FAND_SHIFT)) & PWM_SM0DMAEN_FAND_MASK)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..DMA write requests for the VALx and FRACVALx registers enabled
 */
#define PWM_SM0DMAEN_VALDE_MASK                  (0x200U)
#define PWM_SM0DMAEN_VALDE_SHIFT                 (9U)
#define PWM_SM0DMAEN_VALDE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM0DMAEN_VALDE_SHIFT)) & PWM_SM0DMAEN_VALDE_MASK)
/*! @} */

/*! @name SM0TCTRL - Output Trigger Control Register */
/*! @{ */
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b000000..PWM_OUT_TRIGx will not set when the counter value matches the VALx value.
 *  0b000001..PWM_OUT_TRIGx will set when the counter value matches the VALx value.
 */
#define PWM_SM0TCTRL_OUT_TRIG_EN_MASK            (0x3FU)
#define PWM_SM0TCTRL_OUT_TRIG_EN_SHIFT           (0U)
#define PWM_SM0TCTRL_OUT_TRIG_EN(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_SM0TCTRL_OUT_TRIG_EN_MASK)
/*! TRGFRQ - Trigger frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 *  0b1..Trigger outputs are generated only during the final PWM period prior to a reload opportunity when the PWM
 *       is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_SM0TCTRL_TRGFRQ_MASK                 (0x1000U)
#define PWM_SM0TCTRL_TRGFRQ_SHIFT                (12U)
#define PWM_SM0TCTRL_TRGFRQ(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0TCTRL_TRGFRQ_SHIFT)) & PWM_SM0TCTRL_TRGFRQ_MASK)
/*! PWBOT1 - Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port.
 *  0b1..Route the PWMB output to the PWM_OUT_TRIG1 port.
 */
#define PWM_SM0TCTRL_PWBOT1_MASK                 (0x4000U)
#define PWM_SM0TCTRL_PWBOT1_SHIFT                (14U)
#define PWM_SM0TCTRL_PWBOT1(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0TCTRL_PWBOT1_SHIFT)) & PWM_SM0TCTRL_PWBOT1_MASK)
/*! PWAOT0 - Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port.
 *  0b1..Route the PWMA output to the PWM_OUT_TRIG0 port.
 */
#define PWM_SM0TCTRL_PWAOT0_MASK                 (0x8000U)
#define PWM_SM0TCTRL_PWAOT0_SHIFT                (15U)
#define PWM_SM0TCTRL_PWAOT0(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM0TCTRL_PWAOT0_SHIFT)) & PWM_SM0TCTRL_PWAOT0_MASK)
/*! @} */

/*! @name SM0DISMAP0 - Fault Disable Mapping Register 0 */
/*! @{ */
#define PWM_SM0DISMAP0_DIS0A_MASK                (0xFU)
#define PWM_SM0DISMAP0_DIS0A_SHIFT               (0U)
#define PWM_SM0DISMAP0_DIS0A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP0_DIS0A_SHIFT)) & PWM_SM0DISMAP0_DIS0A_MASK)
#define PWM_SM0DISMAP0_DIS0B_MASK                (0xF0U)
#define PWM_SM0DISMAP0_DIS0B_SHIFT               (4U)
#define PWM_SM0DISMAP0_DIS0B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP0_DIS0B_SHIFT)) & PWM_SM0DISMAP0_DIS0B_MASK)
#define PWM_SM0DISMAP0_DIS0X_MASK                (0xF00U)
#define PWM_SM0DISMAP0_DIS0X_SHIFT               (8U)
#define PWM_SM0DISMAP0_DIS0X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP0_DIS0X_SHIFT)) & PWM_SM0DISMAP0_DIS0X_MASK)
/*! @} */

/*! @name SM0DISMAP1 - Fault Disable Mapping Register 1 */
/*! @{ */
#define PWM_SM0DISMAP1_DIS1A_MASK                (0xFU)
#define PWM_SM0DISMAP1_DIS1A_SHIFT               (0U)
#define PWM_SM0DISMAP1_DIS1A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP1_DIS1A_SHIFT)) & PWM_SM0DISMAP1_DIS1A_MASK)
#define PWM_SM0DISMAP1_DIS1B_MASK                (0xF0U)
#define PWM_SM0DISMAP1_DIS1B_SHIFT               (4U)
#define PWM_SM0DISMAP1_DIS1B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP1_DIS1B_SHIFT)) & PWM_SM0DISMAP1_DIS1B_MASK)
#define PWM_SM0DISMAP1_DIS1X_MASK                (0xF00U)
#define PWM_SM0DISMAP1_DIS1X_SHIFT               (8U)
#define PWM_SM0DISMAP1_DIS1X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DISMAP1_DIS1X_SHIFT)) & PWM_SM0DISMAP1_DIS1X_MASK)
/*! @} */

/*! @name SM0DTCNT0 - Deadtime Count Register 0 */
/*! @{ */
#define PWM_SM0DTCNT0_DTCNT0_MASK                (0xFFFFU)
#define PWM_SM0DTCNT0_DTCNT0_SHIFT               (0U)
#define PWM_SM0DTCNT0_DTCNT0(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DTCNT0_DTCNT0_SHIFT)) & PWM_SM0DTCNT0_DTCNT0_MASK)
/*! @} */

/*! @name SM0DTCNT1 - Deadtime Count Register 1 */
/*! @{ */
#define PWM_SM0DTCNT1_DTCNT1_MASK                (0xFFFFU)
#define PWM_SM0DTCNT1_DTCNT1_SHIFT               (0U)
#define PWM_SM0DTCNT1_DTCNT1(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM0DTCNT1_DTCNT1_SHIFT)) & PWM_SM0DTCNT1_DTCNT1_MASK)
/*! @} */

/*! @name SM0CAPTCTRLA - Capture Control A Register */
/*! @{ */
/*! ARMA - Arm A
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLA[EDGAx] is enabled.
 */
#define PWM_SM0CAPTCTRLA_ARMA_MASK               (0x1U)
#define PWM_SM0CAPTCTRLA_ARMA_SHIFT              (0U)
#define PWM_SM0CAPTCTRLA_ARMA(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_ARMA_SHIFT)) & PWM_SM0CAPTCTRLA_ARMA_MASK)
/*! ONESHOTA - One Shot Mode A
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLA[ARMA] is cleared. No
 *       further captures will be performed until CAPTCTRLA[ARMA] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLA[ARMA] is then cleared.
 */
#define PWM_SM0CAPTCTRLA_ONESHOTA_MASK           (0x2U)
#define PWM_SM0CAPTCTRLA_ONESHOTA_SHIFT          (1U)
#define PWM_SM0CAPTCTRLA_ONESHOTA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_ONESHOTA_SHIFT)) & PWM_SM0CAPTCTRLA_ONESHOTA_MASK)
/*! EDGA0 - Edge A 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLA_EDGA0_MASK              (0xCU)
#define PWM_SM0CAPTCTRLA_EDGA0_SHIFT             (2U)
#define PWM_SM0CAPTCTRLA_EDGA0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_EDGA0_SHIFT)) & PWM_SM0CAPTCTRLA_EDGA0_MASK)
/*! EDGA1 - Edge A 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLA_EDGA1_MASK              (0x30U)
#define PWM_SM0CAPTCTRLA_EDGA1_SHIFT             (4U)
#define PWM_SM0CAPTCTRLA_EDGA1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_EDGA1_SHIFT)) & PWM_SM0CAPTCTRLA_EDGA1_MASK)
/*! INP_SELA - Input Select A
 *  0b0..Raw PWM_A input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLA[EDGA0] and
 *       CAPTCTRLA[EDGA1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRA[EDGA0] and/or CAPTCTRLA[EDGA1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM0CAPTCTRLA_INP_SELA_MASK           (0x40U)
#define PWM_SM0CAPTCTRLA_INP_SELA_SHIFT          (6U)
#define PWM_SM0CAPTCTRLA_INP_SELA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_INP_SELA_SHIFT)) & PWM_SM0CAPTCTRLA_INP_SELA_MASK)
/*! EDGCNTA_EN - Edge Counter A Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM0CAPTCTRLA_EDGCNTA_EN_MASK         (0x80U)
#define PWM_SM0CAPTCTRLA_EDGCNTA_EN_SHIFT        (7U)
#define PWM_SM0CAPTCTRLA_EDGCNTA_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_EDGCNTA_EN_SHIFT)) & PWM_SM0CAPTCTRLA_EDGCNTA_EN_MASK)
#define PWM_SM0CAPTCTRLA_CFAWM_MASK              (0x300U)
#define PWM_SM0CAPTCTRLA_CFAWM_SHIFT             (8U)
#define PWM_SM0CAPTCTRLA_CFAWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_CFAWM_SHIFT)) & PWM_SM0CAPTCTRLA_CFAWM_MASK)
#define PWM_SM0CAPTCTRLA_CA0CNT_MASK             (0x1C00U)
#define PWM_SM0CAPTCTRLA_CA0CNT_SHIFT            (10U)
#define PWM_SM0CAPTCTRLA_CA0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_CA0CNT_SHIFT)) & PWM_SM0CAPTCTRLA_CA0CNT_MASK)
#define PWM_SM0CAPTCTRLA_CA1CNT_MASK             (0xE000U)
#define PWM_SM0CAPTCTRLA_CA1CNT_SHIFT            (13U)
#define PWM_SM0CAPTCTRLA_CA1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLA_CA1CNT_SHIFT)) & PWM_SM0CAPTCTRLA_CA1CNT_MASK)
/*! @} */

/*! @name SM0CAPTCOMPA - Capture Compare A Register */
/*! @{ */
#define PWM_SM0CAPTCOMPA_EDGCMPA_MASK            (0xFFU)
#define PWM_SM0CAPTCOMPA_EDGCMPA_SHIFT           (0U)
#define PWM_SM0CAPTCOMPA_EDGCMPA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPA_EDGCMPA_SHIFT)) & PWM_SM0CAPTCOMPA_EDGCMPA_MASK)
#define PWM_SM0CAPTCOMPA_EDGCNTA_MASK            (0xFF00U)
#define PWM_SM0CAPTCOMPA_EDGCNTA_SHIFT           (8U)
#define PWM_SM0CAPTCOMPA_EDGCNTA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPA_EDGCNTA_SHIFT)) & PWM_SM0CAPTCOMPA_EDGCNTA_MASK)
/*! @} */

/*! @name SM0CAPTCTRLB - Capture Control B Register */
/*! @{ */
/*! ARMB - Arm B
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLB[EDGBx] is enabled.
 */
#define PWM_SM0CAPTCTRLB_ARMB_MASK               (0x1U)
#define PWM_SM0CAPTCTRLB_ARMB_SHIFT              (0U)
#define PWM_SM0CAPTCTRLB_ARMB(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_ARMB_SHIFT)) & PWM_SM0CAPTCTRLB_ARMB_MASK)
/*! ONESHOTB - One Shot Mode B
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLB[ARMB] is cleared. No
 *       further captures will be performed until CAPTCTRLB[ARMB] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLB[ARMB] is then cleared.
 */
#define PWM_SM0CAPTCTRLB_ONESHOTB_MASK           (0x2U)
#define PWM_SM0CAPTCTRLB_ONESHOTB_SHIFT          (1U)
#define PWM_SM0CAPTCTRLB_ONESHOTB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_ONESHOTB_SHIFT)) & PWM_SM0CAPTCTRLB_ONESHOTB_MASK)
/*! EDGB0 - Edge B 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLB_EDGB0_MASK              (0xCU)
#define PWM_SM0CAPTCTRLB_EDGB0_SHIFT             (2U)
#define PWM_SM0CAPTCTRLB_EDGB0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_EDGB0_SHIFT)) & PWM_SM0CAPTCTRLB_EDGB0_MASK)
/*! EDGB1 - Edge B 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLB_EDGB1_MASK              (0x30U)
#define PWM_SM0CAPTCTRLB_EDGB1_SHIFT             (4U)
#define PWM_SM0CAPTCTRLB_EDGB1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_EDGB1_SHIFT)) & PWM_SM0CAPTCTRLB_EDGB1_MASK)
/*! INP_SELB - Input Select B
 *  0b0..Raw PWM_B input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLB[EDGB0] and
 *       CAPTCTRLB[EDGB1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRB[EDGB0] and/or CAPTCTRLB[EDGB1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM0CAPTCTRLB_INP_SELB_MASK           (0x40U)
#define PWM_SM0CAPTCTRLB_INP_SELB_SHIFT          (6U)
#define PWM_SM0CAPTCTRLB_INP_SELB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_INP_SELB_SHIFT)) & PWM_SM0CAPTCTRLB_INP_SELB_MASK)
/*! EDGCNTB_EN - Edge Counter B Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM0CAPTCTRLB_EDGCNTB_EN_MASK         (0x80U)
#define PWM_SM0CAPTCTRLB_EDGCNTB_EN_SHIFT        (7U)
#define PWM_SM0CAPTCTRLB_EDGCNTB_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_EDGCNTB_EN_SHIFT)) & PWM_SM0CAPTCTRLB_EDGCNTB_EN_MASK)
#define PWM_SM0CAPTCTRLB_CFBWM_MASK              (0x300U)
#define PWM_SM0CAPTCTRLB_CFBWM_SHIFT             (8U)
#define PWM_SM0CAPTCTRLB_CFBWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_CFBWM_SHIFT)) & PWM_SM0CAPTCTRLB_CFBWM_MASK)
#define PWM_SM0CAPTCTRLB_CB0CNT_MASK             (0x1C00U)
#define PWM_SM0CAPTCTRLB_CB0CNT_SHIFT            (10U)
#define PWM_SM0CAPTCTRLB_CB0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_CB0CNT_SHIFT)) & PWM_SM0CAPTCTRLB_CB0CNT_MASK)
#define PWM_SM0CAPTCTRLB_CB1CNT_MASK             (0xE000U)
#define PWM_SM0CAPTCTRLB_CB1CNT_SHIFT            (13U)
#define PWM_SM0CAPTCTRLB_CB1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLB_CB1CNT_SHIFT)) & PWM_SM0CAPTCTRLB_CB1CNT_MASK)
/*! @} */

/*! @name SM0CAPTCOMPB - Capture Compare B Register */
/*! @{ */
#define PWM_SM0CAPTCOMPB_EDGCMPB_MASK            (0xFFU)
#define PWM_SM0CAPTCOMPB_EDGCMPB_SHIFT           (0U)
#define PWM_SM0CAPTCOMPB_EDGCMPB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPB_EDGCMPB_SHIFT)) & PWM_SM0CAPTCOMPB_EDGCMPB_MASK)
#define PWM_SM0CAPTCOMPB_EDGCNTB_MASK            (0xFF00U)
#define PWM_SM0CAPTCOMPB_EDGCNTB_SHIFT           (8U)
#define PWM_SM0CAPTCOMPB_EDGCNTB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPB_EDGCNTB_SHIFT)) & PWM_SM0CAPTCOMPB_EDGCNTB_MASK)
/*! @} */

/*! @name SM0CAPTCTRLX - Capture Control X Register */
/*! @{ */
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_SM0CAPTCTRLX_ARMX_MASK               (0x1U)
#define PWM_SM0CAPTCTRLX_ARMX_SHIFT              (0U)
#define PWM_SM0CAPTCTRLX_ARMX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_ARMX_SHIFT)) & PWM_SM0CAPTCTRLX_ARMX_MASK)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The
 *       process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and the ARMX bit is cleared. No further
 *       captures will be performed until the ARMX bit is set again.If only one of the capture circuits is enabled,
 *       then a single capture will occur on the enabled capture circuit and the ARMX bit is then cleared.
 */
#define PWM_SM0CAPTCTRLX_ONESHOTX_MASK           (0x2U)
#define PWM_SM0CAPTCTRLX_ONESHOTX_SHIFT          (1U)
#define PWM_SM0CAPTCTRLX_ONESHOTX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_SM0CAPTCTRLX_ONESHOTX_MASK)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLX_EDGX0_MASK              (0xCU)
#define PWM_SM0CAPTCTRLX_EDGX0_SHIFT             (2U)
#define PWM_SM0CAPTCTRLX_EDGX0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_EDGX0_SHIFT)) & PWM_SM0CAPTCTRLX_EDGX0_MASK)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM0CAPTCTRLX_EDGX1_MASK              (0x30U)
#define PWM_SM0CAPTCTRLX_EDGX1_SHIFT             (4U)
#define PWM_SM0CAPTCTRLX_EDGX1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_EDGX1_SHIFT)) & PWM_SM0CAPTCTRLX_EDGX1_MASK)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLX[EDGX0] and
 *       CAPTCTRLX[EDGX1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRX[EDGX0] and/or CAPTCTRLX[EDGX1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM0CAPTCTRLX_INP_SELX_MASK           (0x40U)
#define PWM_SM0CAPTCTRLX_INP_SELX_SHIFT          (6U)
#define PWM_SM0CAPTCTRLX_INP_SELX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_INP_SELX_SHIFT)) & PWM_SM0CAPTCTRLX_INP_SELX_MASK)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM0CAPTCTRLX_EDGCNTX_EN_MASK         (0x80U)
#define PWM_SM0CAPTCTRLX_EDGCNTX_EN_SHIFT        (7U)
#define PWM_SM0CAPTCTRLX_EDGCNTX_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_SM0CAPTCTRLX_EDGCNTX_EN_MASK)
#define PWM_SM0CAPTCTRLX_CFXWM_MASK              (0x300U)
#define PWM_SM0CAPTCTRLX_CFXWM_SHIFT             (8U)
#define PWM_SM0CAPTCTRLX_CFXWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_CFXWM_SHIFT)) & PWM_SM0CAPTCTRLX_CFXWM_MASK)
#define PWM_SM0CAPTCTRLX_CX0CNT_MASK             (0x1C00U)
#define PWM_SM0CAPTCTRLX_CX0CNT_SHIFT            (10U)
#define PWM_SM0CAPTCTRLX_CX0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_CX0CNT_SHIFT)) & PWM_SM0CAPTCTRLX_CX0CNT_MASK)
#define PWM_SM0CAPTCTRLX_CX1CNT_MASK             (0xE000U)
#define PWM_SM0CAPTCTRLX_CX1CNT_SHIFT            (13U)
#define PWM_SM0CAPTCTRLX_CX1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCTRLX_CX1CNT_SHIFT)) & PWM_SM0CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/*! @name SM0CAPTCOMPX - Capture Compare X Register */
/*! @{ */
#define PWM_SM0CAPTCOMPX_EDGCMPX_MASK            (0xFFU)
#define PWM_SM0CAPTCOMPX_EDGCMPX_SHIFT           (0U)
#define PWM_SM0CAPTCOMPX_EDGCMPX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_SM0CAPTCOMPX_EDGCMPX_MASK)
#define PWM_SM0CAPTCOMPX_EDGCNTX_MASK            (0xFF00U)
#define PWM_SM0CAPTCOMPX_EDGCNTX_SHIFT           (8U)
#define PWM_SM0CAPTCOMPX_EDGCNTX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_SM0CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/*! @name SM0CVAL0 - Capture Value 0 Register */
/*! @{ */
#define PWM_SM0CVAL0_CAPTVAL0_MASK               (0xFFFFU)
#define PWM_SM0CVAL0_CAPTVAL0_SHIFT              (0U)
#define PWM_SM0CVAL0_CAPTVAL0(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL0_CAPTVAL0_SHIFT)) & PWM_SM0CVAL0_CAPTVAL0_MASK)
/*! @} */

/*! @name SM0CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL0CYC_CVAL0CYC_MASK            (0xFU)
#define PWM_SM0CVAL0CYC_CVAL0CYC_SHIFT           (0U)
#define PWM_SM0CVAL0CYC_CVAL0CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_SM0CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/*! @name SM0CVAL1 - Capture Value 1 Register */
/*! @{ */
#define PWM_SM0CVAL1_CAPTVAL1_MASK               (0xFFFFU)
#define PWM_SM0CVAL1_CAPTVAL1_SHIFT              (0U)
#define PWM_SM0CVAL1_CAPTVAL1(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL1_CAPTVAL1_SHIFT)) & PWM_SM0CVAL1_CAPTVAL1_MASK)
/*! @} */

/*! @name SM0CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL1CYC_CVAL1CYC_MASK            (0xFU)
#define PWM_SM0CVAL1CYC_CVAL1CYC_SHIFT           (0U)
#define PWM_SM0CVAL1CYC_CVAL1CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_SM0CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/*! @name SM0CVAL2 - Capture Value 2 Register */
/*! @{ */
#define PWM_SM0CVAL2_CAPTVAL2_MASK               (0xFFFFU)
#define PWM_SM0CVAL2_CAPTVAL2_SHIFT              (0U)
#define PWM_SM0CVAL2_CAPTVAL2(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL2_CAPTVAL2_SHIFT)) & PWM_SM0CVAL2_CAPTVAL2_MASK)
/*! @} */

/*! @name SM0CVAL2CYC - Capture Value 2 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL2CYC_CVAL2CYC_MASK            (0xFU)
#define PWM_SM0CVAL2CYC_CVAL2CYC_SHIFT           (0U)
#define PWM_SM0CVAL2CYC_CVAL2CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL2CYC_CVAL2CYC_SHIFT)) & PWM_SM0CVAL2CYC_CVAL2CYC_MASK)
/*! @} */

/*! @name SM0CVAL3 - Capture Value 3 Register */
/*! @{ */
#define PWM_SM0CVAL3_CAPTVAL3_MASK               (0xFFFFU)
#define PWM_SM0CVAL3_CAPTVAL3_SHIFT              (0U)
#define PWM_SM0CVAL3_CAPTVAL3(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL3_CAPTVAL3_SHIFT)) & PWM_SM0CVAL3_CAPTVAL3_MASK)
/*! @} */

/*! @name SM0CVAL3CYC - Capture Value 3 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL3CYC_CVAL3CYC_MASK            (0xFU)
#define PWM_SM0CVAL3CYC_CVAL3CYC_SHIFT           (0U)
#define PWM_SM0CVAL3CYC_CVAL3CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL3CYC_CVAL3CYC_SHIFT)) & PWM_SM0CVAL3CYC_CVAL3CYC_MASK)
/*! @} */

/*! @name SM0CVAL4 - Capture Value 4 Register */
/*! @{ */
#define PWM_SM0CVAL4_CAPTVAL4_MASK               (0xFFFFU)
#define PWM_SM0CVAL4_CAPTVAL4_SHIFT              (0U)
#define PWM_SM0CVAL4_CAPTVAL4(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL4_CAPTVAL4_SHIFT)) & PWM_SM0CVAL4_CAPTVAL4_MASK)
/*! @} */

/*! @name SM0CVAL4CYC - Capture Value 4 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL4CYC_CVAL4CYC_MASK            (0xFU)
#define PWM_SM0CVAL4CYC_CVAL4CYC_SHIFT           (0U)
#define PWM_SM0CVAL4CYC_CVAL4CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL4CYC_CVAL4CYC_SHIFT)) & PWM_SM0CVAL4CYC_CVAL4CYC_MASK)
/*! @} */

/*! @name SM0CVAL5 - Capture Value 5 Register */
/*! @{ */
#define PWM_SM0CVAL5_CAPTVAL5_MASK               (0xFFFFU)
#define PWM_SM0CVAL5_CAPTVAL5_SHIFT              (0U)
#define PWM_SM0CVAL5_CAPTVAL5(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL5_CAPTVAL5_SHIFT)) & PWM_SM0CVAL5_CAPTVAL5_MASK)
/*! @} */

/*! @name SM0CVAL5CYC - Capture Value 5 Cycle Register */
/*! @{ */
#define PWM_SM0CVAL5CYC_CVAL5CYC_MASK            (0xFU)
#define PWM_SM0CVAL5CYC_CVAL5CYC_SHIFT           (0U)
#define PWM_SM0CVAL5CYC_CVAL5CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0CVAL5CYC_CVAL5CYC_SHIFT)) & PWM_SM0CVAL5CYC_CVAL5CYC_MASK)
/*! @} */

/*! @name SM0PHASEDLY - Phase Delay Register */
/*! @{ */
#define PWM_SM0PHASEDLY_PHASEDLY_MASK            (0xFFFFU)
#define PWM_SM0PHASEDLY_PHASEDLY_SHIFT           (0U)
#define PWM_SM0PHASEDLY_PHASEDLY(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM0PHASEDLY_PHASEDLY_SHIFT)) & PWM_SM0PHASEDLY_PHASEDLY_MASK)
/*! @} */

/*! @name SM1CNT - Counter Register */
/*! @{ */
#define PWM_SM1CNT_CNT_MASK                      (0xFFFFU)
#define PWM_SM1CNT_CNT_SHIFT                     (0U)
#define PWM_SM1CNT_CNT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM1CNT_CNT_SHIFT)) & PWM_SM1CNT_CNT_MASK)
/*! @} */

/*! @name SM1INIT - Initial Count Register */
/*! @{ */
#define PWM_SM1INIT_INIT_MASK                    (0xFFFFU)
#define PWM_SM1INIT_INIT_SHIFT                   (0U)
#define PWM_SM1INIT_INIT(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1INIT_INIT_SHIFT)) & PWM_SM1INIT_INIT_MASK)
/*! @} */

/*! @name SM1CTRL2 - Control 2 Register */
/*! @{ */
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it will force the clock to logic 0.
 *  0b11..reserved
 */
#define PWM_SM1CTRL2_CLK_SEL_MASK                (0x3U)
#define PWM_SM1CTRL2_CLK_SEL_SHIFT               (0U)
#define PWM_SM1CTRL2_CLK_SEL(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_CLK_SEL_SHIFT)) & PWM_SM1CTRL2_CLK_SEL_MASK)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it will force the RELOAD signal to logic 0.
 */
#define PWM_SM1CTRL2_RELOAD_SEL_MASK             (0x4U)
#define PWM_SM1CTRL2_RELOAD_SEL_SHIFT            (2U)
#define PWM_SM1CTRL2_RELOAD_SEL(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_RELOAD_SEL_SHIFT)) & PWM_SM1CTRL2_RELOAD_SEL_MASK)
/*! FORCE_SEL - This read/write bit determines the source of the FORCE OUTPUT signal for this submodule.
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_SM1CTRL2_FORCE_SEL_MASK              (0x38U)
#define PWM_SM1CTRL2_FORCE_SEL_SHIFT             (3U)
#define PWM_SM1CTRL2_FORCE_SEL(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_FORCE_SEL_SHIFT)) & PWM_SM1CTRL2_FORCE_SEL_MASK)
#define PWM_SM1CTRL2_FORCE_MASK                  (0x40U)
#define PWM_SM1CTRL2_FORCE_SHIFT                 (6U)
#define PWM_SM1CTRL2_FORCE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_FORCE_SHIFT)) & PWM_SM1CTRL2_FORCE_MASK)
/*! FRCEN - FRCEN
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_SM1CTRL2_FRCEN_MASK                  (0x80U)
#define PWM_SM1CTRL2_FRCEN_SHIFT                 (7U)
#define PWM_SM1CTRL2_FRCEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_FRCEN_SHIFT)) & PWM_SM1CTRL2_FRCEN_MASK)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it will force the INIT signal to logic 0. The submodule counter will only reinitialize when a master
 *        reload occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it
 *        will force the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_SM1CTRL2_INIT_SEL_MASK               (0x300U)
#define PWM_SM1CTRL2_INIT_SEL_SHIFT              (8U)
#define PWM_SM1CTRL2_INIT_SEL(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_INIT_SEL_SHIFT)) & PWM_SM1CTRL2_INIT_SEL_MASK)
#define PWM_SM1CTRL2_PWMX_INIT_MASK              (0x400U)
#define PWM_SM1CTRL2_PWMX_INIT_SHIFT             (10U)
#define PWM_SM1CTRL2_PWMX_INIT(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_PWMX_INIT_SHIFT)) & PWM_SM1CTRL2_PWMX_INIT_MASK)
#define PWM_SM1CTRL2_PWM45_INIT_MASK             (0x800U)
#define PWM_SM1CTRL2_PWM45_INIT_SHIFT            (11U)
#define PWM_SM1CTRL2_PWM45_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_PWM45_INIT_SHIFT)) & PWM_SM1CTRL2_PWM45_INIT_MASK)
#define PWM_SM1CTRL2_PWM23_INIT_MASK             (0x1000U)
#define PWM_SM1CTRL2_PWM23_INIT_SHIFT            (12U)
#define PWM_SM1CTRL2_PWM23_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_PWM23_INIT_SHIFT)) & PWM_SM1CTRL2_PWM23_INIT_MASK)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_SM1CTRL2_INDEP_MASK                  (0x2000U)
#define PWM_SM1CTRL2_INDEP_SHIFT                 (13U)
#define PWM_SM1CTRL2_INDEP(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_INDEP_SHIFT)) & PWM_SM1CTRL2_INDEP_MASK)
#define PWM_SM1CTRL2_WAITEN_MASK                 (0x4000U)
#define PWM_SM1CTRL2_WAITEN_SHIFT                (14U)
#define PWM_SM1CTRL2_WAITEN(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_WAITEN_SHIFT)) & PWM_SM1CTRL2_WAITEN_MASK)
#define PWM_SM1CTRL2_DBGEN_MASK                  (0x8000U)
#define PWM_SM1CTRL2_DBGEN_SHIFT                 (15U)
#define PWM_SM1CTRL2_DBGEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL2_DBGEN_SHIFT)) & PWM_SM1CTRL2_DBGEN_MASK)
/*! @} */

/*! @name SM1CTRL - Control Register */
/*! @{ */
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_SM1CTRL_DBLEN_MASK                   (0x1U)
#define PWM_SM1CTRL_DBLEN_SHIFT                  (0U)
#define PWM_SM1CTRL_DBLEN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_DBLEN_SHIFT)) & PWM_SM1CTRL_DBLEN_MASK)
/*! DBLX - PWMX Double Switching Enable
 *  0b0..PWMX double pulse disabled.
 *  0b1..PWMX double pulse enabled.
 */
#define PWM_SM1CTRL_DBLX_MASK                    (0x2U)
#define PWM_SM1CTRL_DBLX_SHIFT                   (1U)
#define PWM_SM1CTRL_DBLX(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_DBLX_SHIFT)) & PWM_SM1CTRL_DBLX_MASK)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_SM1CTRL_LDMOD_MASK                   (0x4U)
#define PWM_SM1CTRL_LDMOD_SHIFT                  (2U)
#define PWM_SM1CTRL_LDMOD(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_LDMOD_SHIFT)) & PWM_SM1CTRL_LDMOD_MASK)
/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
 *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
 *  0b1..DBLPWM is split to PWMA and PWMB.
 */
#define PWM_SM1CTRL_SPLIT_MASK                   (0x8U)
#define PWM_SM1CTRL_SPLIT_SHIFT                  (3U)
#define PWM_SM1CTRL_SPLIT(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_SPLIT_SHIFT)) & PWM_SM1CTRL_SPLIT_MASK)
/*! PRSC - Prescaler
 *  0b000..PWM clock frequency = fclk
 *  0b001..PWM clock frequency = fclk/2
 *  0b010..PWM clock frequency = fclk/4
 *  0b011..PWM clock frequency = fclk/8
 *  0b100..PWM clock frequency = fclk/16
 *  0b101..PWM clock frequency = fclk/32
 *  0b110..PWM clock frequency = fclk/64
 *  0b111..PWM clock frequency = fclk/128
 */
#define PWM_SM1CTRL_PRSC_MASK                    (0x70U)
#define PWM_SM1CTRL_PRSC_SHIFT                   (4U)
#define PWM_SM1CTRL_PRSC(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_PRSC_SHIFT)) & PWM_SM1CTRL_PRSC_MASK)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
 *       output in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_SM1CTRL_COMPMODE_MASK                (0x80U)
#define PWM_SM1CTRL_COMPMODE_SHIFT               (7U)
#define PWM_SM1CTRL_COMPMODE(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_COMPMODE_SHIFT)) & PWM_SM1CTRL_COMPMODE_MASK)
#define PWM_SM1CTRL_DT_MASK                      (0x300U)
#define PWM_SM1CTRL_DT_SHIFT                     (8U)
#define PWM_SM1CTRL_DT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_DT_SHIFT)) & PWM_SM1CTRL_DT_MASK)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_SM1CTRL_FULL_MASK                    (0x400U)
#define PWM_SM1CTRL_FULL_SHIFT                   (10U)
#define PWM_SM1CTRL_FULL(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_FULL_SHIFT)) & PWM_SM1CTRL_FULL_MASK)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_SM1CTRL_HALF_MASK                    (0x800U)
#define PWM_SM1CTRL_HALF_SHIFT                   (11U)
#define PWM_SM1CTRL_HALF(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_HALF_SHIFT)) & PWM_SM1CTRL_HALF_MASK)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_SM1CTRL_LDFQ_MASK                    (0xF000U)
#define PWM_SM1CTRL_LDFQ_SHIFT                   (12U)
#define PWM_SM1CTRL_LDFQ(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1CTRL_LDFQ_SHIFT)) & PWM_SM1CTRL_LDFQ_MASK)
/*! @} */

/*! @name SM1VAL0 - Value Register 0 */
/*! @{ */
#define PWM_SM1VAL0_VAL0_MASK                    (0xFFFFU)
#define PWM_SM1VAL0_VAL0_SHIFT                   (0U)
#define PWM_SM1VAL0_VAL0(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL0_VAL0_SHIFT)) & PWM_SM1VAL0_VAL0_MASK)
/*! @} */

/*! @name SM1FRACVAL1 - Fractional Value Register 1 */
/*! @{ */
#define PWM_SM1FRACVAL1_FRACVAL1_MASK            (0xF800U)
#define PWM_SM1FRACVAL1_FRACVAL1_SHIFT           (11U)
#define PWM_SM1FRACVAL1_FRACVAL1(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRACVAL1_FRACVAL1_SHIFT)) & PWM_SM1FRACVAL1_FRACVAL1_MASK)
/*! @} */

/*! @name SM1VAL1 - Value Register 1 */
/*! @{ */
#define PWM_SM1VAL1_VAL1_MASK                    (0xFFFFU)
#define PWM_SM1VAL1_VAL1_SHIFT                   (0U)
#define PWM_SM1VAL1_VAL1(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL1_VAL1_SHIFT)) & PWM_SM1VAL1_VAL1_MASK)
/*! @} */

/*! @name SM1FRACVAL2 - Fractional Value Register 2 */
/*! @{ */
#define PWM_SM1FRACVAL2_FRACVAL2_MASK            (0xF800U)
#define PWM_SM1FRACVAL2_FRACVAL2_SHIFT           (11U)
#define PWM_SM1FRACVAL2_FRACVAL2(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRACVAL2_FRACVAL2_SHIFT)) & PWM_SM1FRACVAL2_FRACVAL2_MASK)
/*! @} */

/*! @name SM1VAL2 - Value Register 2 */
/*! @{ */
#define PWM_SM1VAL2_VAL2_MASK                    (0xFFFFU)
#define PWM_SM1VAL2_VAL2_SHIFT                   (0U)
#define PWM_SM1VAL2_VAL2(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL2_VAL2_SHIFT)) & PWM_SM1VAL2_VAL2_MASK)
/*! @} */

/*! @name SM1FRACVAL3 - Fractional Value Register 3 */
/*! @{ */
#define PWM_SM1FRACVAL3_FRACVAL3_MASK            (0xF800U)
#define PWM_SM1FRACVAL3_FRACVAL3_SHIFT           (11U)
#define PWM_SM1FRACVAL3_FRACVAL3(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRACVAL3_FRACVAL3_SHIFT)) & PWM_SM1FRACVAL3_FRACVAL3_MASK)
/*! @} */

/*! @name SM1VAL3 - Value Register 3 */
/*! @{ */
#define PWM_SM1VAL3_VAL3_MASK                    (0xFFFFU)
#define PWM_SM1VAL3_VAL3_SHIFT                   (0U)
#define PWM_SM1VAL3_VAL3(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL3_VAL3_SHIFT)) & PWM_SM1VAL3_VAL3_MASK)
/*! @} */

/*! @name SM1FRACVAL4 - Fractional Value Register 4 */
/*! @{ */
#define PWM_SM1FRACVAL4_FRACVAL4_MASK            (0xF800U)
#define PWM_SM1FRACVAL4_FRACVAL4_SHIFT           (11U)
#define PWM_SM1FRACVAL4_FRACVAL4(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRACVAL4_FRACVAL4_SHIFT)) & PWM_SM1FRACVAL4_FRACVAL4_MASK)
/*! @} */

/*! @name SM1VAL4 - Value Register 4 */
/*! @{ */
#define PWM_SM1VAL4_VAL4_MASK                    (0xFFFFU)
#define PWM_SM1VAL4_VAL4_SHIFT                   (0U)
#define PWM_SM1VAL4_VAL4(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL4_VAL4_SHIFT)) & PWM_SM1VAL4_VAL4_MASK)
/*! @} */

/*! @name SM1FRACVAL5 - Fractional Value Register 5 */
/*! @{ */
#define PWM_SM1FRACVAL5_FRACVAL5_MASK            (0xF800U)
#define PWM_SM1FRACVAL5_FRACVAL5_SHIFT           (11U)
#define PWM_SM1FRACVAL5_FRACVAL5(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRACVAL5_FRACVAL5_SHIFT)) & PWM_SM1FRACVAL5_FRACVAL5_MASK)
/*! @} */

/*! @name SM1VAL5 - Value Register 5 */
/*! @{ */
#define PWM_SM1VAL5_VAL5_MASK                    (0xFFFFU)
#define PWM_SM1VAL5_VAL5_SHIFT                   (0U)
#define PWM_SM1VAL5_VAL5(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1VAL5_VAL5_SHIFT)) & PWM_SM1VAL5_VAL5_MASK)
/*! @} */

/*! @name SM1FRCTRL - Fractional Control Register */
/*! @{ */
/*! FRAC1_EN - Fractional Cycle PWM Period Enable
 *  0b0..Disable fractional cycle length for the PWM period.
 *  0b1..Enable fractional cycle length for the PWM period.
 */
#define PWM_SM1FRCTRL_FRAC1_EN_MASK              (0x2U)
#define PWM_SM1FRCTRL_FRAC1_EN_SHIFT             (1U)
#define PWM_SM1FRCTRL_FRAC1_EN(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRCTRL_FRAC1_EN_SHIFT)) & PWM_SM1FRCTRL_FRAC1_EN_MASK)
/*! FRAC23_EN - Fractional Cycle Placement Enable for PWM_A
 *  0b0..Disable fractional cycle placement for PWM_A.
 *  0b1..Enable fractional cycle placement for PWM_A.
 */
#define PWM_SM1FRCTRL_FRAC23_EN_MASK             (0x4U)
#define PWM_SM1FRCTRL_FRAC23_EN_SHIFT            (2U)
#define PWM_SM1FRCTRL_FRAC23_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRCTRL_FRAC23_EN_SHIFT)) & PWM_SM1FRCTRL_FRAC23_EN_MASK)
/*! FRAC45_EN - Fractional Cycle Placement Enable for PWM_B
 *  0b0..Disable fractional cycle placement for PWM_B.
 *  0b1..Enable fractional cycle placement for PWM_B.
 */
#define PWM_SM1FRCTRL_FRAC45_EN_MASK             (0x10U)
#define PWM_SM1FRCTRL_FRAC45_EN_SHIFT            (4U)
#define PWM_SM1FRCTRL_FRAC45_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRCTRL_FRAC45_EN_SHIFT)) & PWM_SM1FRCTRL_FRAC45_EN_MASK)
/*! FRAC_PU - Fractional Delay Circuit Power Up
 *  0b0..Turn off fractional delay logic.
 *  0b1..Power up fractional delay logic.
 */
#define PWM_SM1FRCTRL_FRAC_PU_MASK               (0x100U)
#define PWM_SM1FRCTRL_FRAC_PU_SHIFT              (8U)
#define PWM_SM1FRCTRL_FRAC_PU(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRCTRL_FRAC_PU_SHIFT)) & PWM_SM1FRCTRL_FRAC_PU_MASK)
#define PWM_SM1FRCTRL_TEST_MASK                  (0x8000U)
#define PWM_SM1FRCTRL_TEST_SHIFT                 (15U)
#define PWM_SM1FRCTRL_TEST(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1FRCTRL_TEST_SHIFT)) & PWM_SM1FRCTRL_TEST_MASK)
/*! @} */

/*! @name SM1OCTRL - Output Control Register */
/*! @{ */
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM1OCTRL_PWMXFS_MASK                 (0x3U)
#define PWM_SM1OCTRL_PWMXFS_SHIFT                (0U)
#define PWM_SM1OCTRL_PWMXFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMXFS_SHIFT)) & PWM_SM1OCTRL_PWMXFS_MASK)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM1OCTRL_PWMBFS_MASK                 (0xCU)
#define PWM_SM1OCTRL_PWMBFS_SHIFT                (2U)
#define PWM_SM1OCTRL_PWMBFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMBFS_SHIFT)) & PWM_SM1OCTRL_PWMBFS_MASK)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM1OCTRL_PWMAFS_MASK                 (0x30U)
#define PWM_SM1OCTRL_PWMAFS_SHIFT                (4U)
#define PWM_SM1OCTRL_PWMAFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMAFS_SHIFT)) & PWM_SM1OCTRL_PWMAFS_MASK)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_SM1OCTRL_POLX_MASK                   (0x100U)
#define PWM_SM1OCTRL_POLX_SHIFT                  (8U)
#define PWM_SM1OCTRL_POLX(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_POLX_SHIFT)) & PWM_SM1OCTRL_POLX_MASK)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_SM1OCTRL_POLB_MASK                   (0x200U)
#define PWM_SM1OCTRL_POLB_SHIFT                  (9U)
#define PWM_SM1OCTRL_POLB(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_POLB_SHIFT)) & PWM_SM1OCTRL_POLB_MASK)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_SM1OCTRL_POLA_MASK                   (0x400U)
#define PWM_SM1OCTRL_POLA_SHIFT                  (10U)
#define PWM_SM1OCTRL_POLA(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_POLA_SHIFT)) & PWM_SM1OCTRL_POLA_MASK)
#define PWM_SM1OCTRL_PWMX_IN_MASK                (0x2000U)
#define PWM_SM1OCTRL_PWMX_IN_SHIFT               (13U)
#define PWM_SM1OCTRL_PWMX_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMX_IN_SHIFT)) & PWM_SM1OCTRL_PWMX_IN_MASK)
#define PWM_SM1OCTRL_PWMB_IN_MASK                (0x4000U)
#define PWM_SM1OCTRL_PWMB_IN_SHIFT               (14U)
#define PWM_SM1OCTRL_PWMB_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMB_IN_SHIFT)) & PWM_SM1OCTRL_PWMB_IN_MASK)
#define PWM_SM1OCTRL_PWMA_IN_MASK                (0x8000U)
#define PWM_SM1OCTRL_PWMA_IN_SHIFT               (15U)
#define PWM_SM1OCTRL_PWMA_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1OCTRL_PWMA_IN_SHIFT)) & PWM_SM1OCTRL_PWMA_IN_MASK)
/*! @} */

/*! @name SM1STS - Status Register */
/*! @{ */
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_SM1STS_CMPF_MASK                     (0x3FU)
#define PWM_SM1STS_CMPF_SHIFT                    (0U)
#define PWM_SM1STS_CMPF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CMPF_SHIFT)) & PWM_SM1STS_CMPF_MASK)
#define PWM_SM1STS_CFX0_MASK                     (0x40U)
#define PWM_SM1STS_CFX0_SHIFT                    (6U)
#define PWM_SM1STS_CFX0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFX0_SHIFT)) & PWM_SM1STS_CFX0_MASK)
#define PWM_SM1STS_CFX1_MASK                     (0x80U)
#define PWM_SM1STS_CFX1_SHIFT                    (7U)
#define PWM_SM1STS_CFX1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFX1_SHIFT)) & PWM_SM1STS_CFX1_MASK)
#define PWM_SM1STS_CFB0_MASK                     (0x100U)
#define PWM_SM1STS_CFB0_SHIFT                    (8U)
#define PWM_SM1STS_CFB0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFB0_SHIFT)) & PWM_SM1STS_CFB0_MASK)
#define PWM_SM1STS_CFB1_MASK                     (0x200U)
#define PWM_SM1STS_CFB1_SHIFT                    (9U)
#define PWM_SM1STS_CFB1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFB1_SHIFT)) & PWM_SM1STS_CFB1_MASK)
#define PWM_SM1STS_CFA0_MASK                     (0x400U)
#define PWM_SM1STS_CFA0_SHIFT                    (10U)
#define PWM_SM1STS_CFA0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFA0_SHIFT)) & PWM_SM1STS_CFA0_MASK)
#define PWM_SM1STS_CFA1_MASK                     (0x800U)
#define PWM_SM1STS_CFA1_SHIFT                    (11U)
#define PWM_SM1STS_CFA1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_CFA1_SHIFT)) & PWM_SM1STS_CFA1_MASK)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_SM1STS_RF_MASK                       (0x1000U)
#define PWM_SM1STS_RF_SHIFT                      (12U)
#define PWM_SM1STS_RF(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_RF_SHIFT)) & PWM_SM1STS_RF_MASK)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_SM1STS_REF_MASK                      (0x2000U)
#define PWM_SM1STS_REF_SHIFT                     (13U)
#define PWM_SM1STS_REF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_REF_SHIFT)) & PWM_SM1STS_REF_MASK)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_SM1STS_RUF_MASK                      (0x4000U)
#define PWM_SM1STS_RUF_SHIFT                     (14U)
#define PWM_SM1STS_RUF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM1STS_RUF_SHIFT)) & PWM_SM1STS_RUF_MASK)
/*! @} */

/*! @name SM1INTEN - Interrupt Enable Register */
/*! @{ */
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_SM1INTEN_CMPIE_MASK                  (0x3FU)
#define PWM_SM1INTEN_CMPIE_SHIFT                 (0U)
#define PWM_SM1INTEN_CMPIE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CMPIE_SHIFT)) & PWM_SM1INTEN_CMPIE_MASK)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_SM1INTEN_CX0IE_MASK                  (0x40U)
#define PWM_SM1INTEN_CX0IE_SHIFT                 (6U)
#define PWM_SM1INTEN_CX0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CX0IE_SHIFT)) & PWM_SM1INTEN_CX0IE_MASK)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_SM1INTEN_CX1IE_MASK                  (0x80U)
#define PWM_SM1INTEN_CX1IE_SHIFT                 (7U)
#define PWM_SM1INTEN_CX1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CX1IE_SHIFT)) & PWM_SM1INTEN_CX1IE_MASK)
/*! CB0IE - Capture B 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB0].
 *  0b1..Interrupt request enabled for STS[CFB0].
 */
#define PWM_SM1INTEN_CB0IE_MASK                  (0x100U)
#define PWM_SM1INTEN_CB0IE_SHIFT                 (8U)
#define PWM_SM1INTEN_CB0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CB0IE_SHIFT)) & PWM_SM1INTEN_CB0IE_MASK)
/*! CB1IE - Capture B 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB1].
 *  0b1..Interrupt request enabled for STS[CFB1].
 */
#define PWM_SM1INTEN_CB1IE_MASK                  (0x200U)
#define PWM_SM1INTEN_CB1IE_SHIFT                 (9U)
#define PWM_SM1INTEN_CB1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CB1IE_SHIFT)) & PWM_SM1INTEN_CB1IE_MASK)
/*! CA0IE - Capture A 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA0].
 *  0b1..Interrupt request enabled for STS[CFA0].
 */
#define PWM_SM1INTEN_CA0IE_MASK                  (0x400U)
#define PWM_SM1INTEN_CA0IE_SHIFT                 (10U)
#define PWM_SM1INTEN_CA0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CA0IE_SHIFT)) & PWM_SM1INTEN_CA0IE_MASK)
/*! CA1IE - Capture A 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA1].
 *  0b1..Interrupt request enabled for STS[CFA1].
 */
#define PWM_SM1INTEN_CA1IE_MASK                  (0x800U)
#define PWM_SM1INTEN_CA1IE_SHIFT                 (11U)
#define PWM_SM1INTEN_CA1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_CA1IE_SHIFT)) & PWM_SM1INTEN_CA1IE_MASK)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_SM1INTEN_RIE_MASK                    (0x1000U)
#define PWM_SM1INTEN_RIE_SHIFT                   (12U)
#define PWM_SM1INTEN_RIE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_RIE_SHIFT)) & PWM_SM1INTEN_RIE_MASK)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_SM1INTEN_REIE_MASK                   (0x2000U)
#define PWM_SM1INTEN_REIE_SHIFT                  (13U)
#define PWM_SM1INTEN_REIE(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1INTEN_REIE_SHIFT)) & PWM_SM1INTEN_REIE_MASK)
/*! @} */

/*! @name SM1DMAEN - DMA Enable Register */
/*! @{ */
#define PWM_SM1DMAEN_CX0DE_MASK                  (0x1U)
#define PWM_SM1DMAEN_CX0DE_SHIFT                 (0U)
#define PWM_SM1DMAEN_CX0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CX0DE_SHIFT)) & PWM_SM1DMAEN_CX0DE_MASK)
#define PWM_SM1DMAEN_CX1DE_MASK                  (0x2U)
#define PWM_SM1DMAEN_CX1DE_SHIFT                 (1U)
#define PWM_SM1DMAEN_CX1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CX1DE_SHIFT)) & PWM_SM1DMAEN_CX1DE_MASK)
#define PWM_SM1DMAEN_CB0DE_MASK                  (0x4U)
#define PWM_SM1DMAEN_CB0DE_SHIFT                 (2U)
#define PWM_SM1DMAEN_CB0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CB0DE_SHIFT)) & PWM_SM1DMAEN_CB0DE_MASK)
#define PWM_SM1DMAEN_CB1DE_MASK                  (0x8U)
#define PWM_SM1DMAEN_CB1DE_SHIFT                 (3U)
#define PWM_SM1DMAEN_CB1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CB1DE_SHIFT)) & PWM_SM1DMAEN_CB1DE_MASK)
#define PWM_SM1DMAEN_CA0DE_MASK                  (0x10U)
#define PWM_SM1DMAEN_CA0DE_SHIFT                 (4U)
#define PWM_SM1DMAEN_CA0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CA0DE_SHIFT)) & PWM_SM1DMAEN_CA0DE_MASK)
#define PWM_SM1DMAEN_CA1DE_MASK                  (0x20U)
#define PWM_SM1DMAEN_CA1DE_SHIFT                 (5U)
#define PWM_SM1DMAEN_CA1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CA1DE_SHIFT)) & PWM_SM1DMAEN_CA1DE_MASK)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to also be set in order to determine to
 *        which watermark(s) the DMA request is sensitive.
 *  0b10..A local sync (VAL1 matches counter) sets the read DMA request.
 *  0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_SM1DMAEN_CAPTDE_MASK                 (0xC0U)
#define PWM_SM1DMAEN_CAPTDE_SHIFT                (6U)
#define PWM_SM1DMAEN_CAPTDE(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_CAPTDE_SHIFT)) & PWM_SM1DMAEN_CAPTDE_MASK)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_SM1DMAEN_FAND_MASK                   (0x100U)
#define PWM_SM1DMAEN_FAND_SHIFT                  (8U)
#define PWM_SM1DMAEN_FAND(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_FAND_SHIFT)) & PWM_SM1DMAEN_FAND_MASK)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..DMA write requests for the VALx and FRACVALx registers enabled
 */
#define PWM_SM1DMAEN_VALDE_MASK                  (0x200U)
#define PWM_SM1DMAEN_VALDE_SHIFT                 (9U)
#define PWM_SM1DMAEN_VALDE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM1DMAEN_VALDE_SHIFT)) & PWM_SM1DMAEN_VALDE_MASK)
/*! @} */

/*! @name SM1TCTRL - Output Trigger Control Register */
/*! @{ */
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b000000..PWM_OUT_TRIGx will not set when the counter value matches the VALx value.
 *  0b000001..PWM_OUT_TRIGx will set when the counter value matches the VALx value.
 */
#define PWM_SM1TCTRL_OUT_TRIG_EN_MASK            (0x3FU)
#define PWM_SM1TCTRL_OUT_TRIG_EN_SHIFT           (0U)
#define PWM_SM1TCTRL_OUT_TRIG_EN(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_SM1TCTRL_OUT_TRIG_EN_MASK)
/*! TRGFRQ - Trigger frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 *  0b1..Trigger outputs are generated only during the final PWM period prior to a reload opportunity when the PWM
 *       is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_SM1TCTRL_TRGFRQ_MASK                 (0x1000U)
#define PWM_SM1TCTRL_TRGFRQ_SHIFT                (12U)
#define PWM_SM1TCTRL_TRGFRQ(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1TCTRL_TRGFRQ_SHIFT)) & PWM_SM1TCTRL_TRGFRQ_MASK)
/*! PWBOT1 - Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port.
 *  0b1..Route the PWMB output to the PWM_OUT_TRIG1 port.
 */
#define PWM_SM1TCTRL_PWBOT1_MASK                 (0x4000U)
#define PWM_SM1TCTRL_PWBOT1_SHIFT                (14U)
#define PWM_SM1TCTRL_PWBOT1(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1TCTRL_PWBOT1_SHIFT)) & PWM_SM1TCTRL_PWBOT1_MASK)
/*! PWAOT0 - Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port.
 *  0b1..Route the PWMA output to the PWM_OUT_TRIG0 port.
 */
#define PWM_SM1TCTRL_PWAOT0_MASK                 (0x8000U)
#define PWM_SM1TCTRL_PWAOT0_SHIFT                (15U)
#define PWM_SM1TCTRL_PWAOT0(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM1TCTRL_PWAOT0_SHIFT)) & PWM_SM1TCTRL_PWAOT0_MASK)
/*! @} */

/*! @name SM1DISMAP0 - Fault Disable Mapping Register 0 */
/*! @{ */
#define PWM_SM1DISMAP0_DIS0A_MASK                (0xFU)
#define PWM_SM1DISMAP0_DIS0A_SHIFT               (0U)
#define PWM_SM1DISMAP0_DIS0A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP0_DIS0A_SHIFT)) & PWM_SM1DISMAP0_DIS0A_MASK)
#define PWM_SM1DISMAP0_DIS0B_MASK                (0xF0U)
#define PWM_SM1DISMAP0_DIS0B_SHIFT               (4U)
#define PWM_SM1DISMAP0_DIS0B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP0_DIS0B_SHIFT)) & PWM_SM1DISMAP0_DIS0B_MASK)
#define PWM_SM1DISMAP0_DIS0X_MASK                (0xF00U)
#define PWM_SM1DISMAP0_DIS0X_SHIFT               (8U)
#define PWM_SM1DISMAP0_DIS0X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP0_DIS0X_SHIFT)) & PWM_SM1DISMAP0_DIS0X_MASK)
/*! @} */

/*! @name SM1DISMAP1 - Fault Disable Mapping Register 1 */
/*! @{ */
#define PWM_SM1DISMAP1_DIS1A_MASK                (0xFU)
#define PWM_SM1DISMAP1_DIS1A_SHIFT               (0U)
#define PWM_SM1DISMAP1_DIS1A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP1_DIS1A_SHIFT)) & PWM_SM1DISMAP1_DIS1A_MASK)
#define PWM_SM1DISMAP1_DIS1B_MASK                (0xF0U)
#define PWM_SM1DISMAP1_DIS1B_SHIFT               (4U)
#define PWM_SM1DISMAP1_DIS1B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP1_DIS1B_SHIFT)) & PWM_SM1DISMAP1_DIS1B_MASK)
#define PWM_SM1DISMAP1_DIS1X_MASK                (0xF00U)
#define PWM_SM1DISMAP1_DIS1X_SHIFT               (8U)
#define PWM_SM1DISMAP1_DIS1X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DISMAP1_DIS1X_SHIFT)) & PWM_SM1DISMAP1_DIS1X_MASK)
/*! @} */

/*! @name SM1DTCNT0 - Deadtime Count Register 0 */
/*! @{ */
#define PWM_SM1DTCNT0_DTCNT0_MASK                (0xFFFFU)
#define PWM_SM1DTCNT0_DTCNT0_SHIFT               (0U)
#define PWM_SM1DTCNT0_DTCNT0(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DTCNT0_DTCNT0_SHIFT)) & PWM_SM1DTCNT0_DTCNT0_MASK)
/*! @} */

/*! @name SM1DTCNT1 - Deadtime Count Register 1 */
/*! @{ */
#define PWM_SM1DTCNT1_DTCNT1_MASK                (0xFFFFU)
#define PWM_SM1DTCNT1_DTCNT1_SHIFT               (0U)
#define PWM_SM1DTCNT1_DTCNT1(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM1DTCNT1_DTCNT1_SHIFT)) & PWM_SM1DTCNT1_DTCNT1_MASK)
/*! @} */

/*! @name SM1CAPTCTRLA - Capture Control A Register */
/*! @{ */
/*! ARMA - Arm A
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLA[EDGAx] is enabled.
 */
#define PWM_SM1CAPTCTRLA_ARMA_MASK               (0x1U)
#define PWM_SM1CAPTCTRLA_ARMA_SHIFT              (0U)
#define PWM_SM1CAPTCTRLA_ARMA(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_ARMA_SHIFT)) & PWM_SM1CAPTCTRLA_ARMA_MASK)
/*! ONESHOTA - One Shot Mode A
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLA[ARMA] is cleared. No
 *       further captures will be performed until CAPTCTRLA[ARMA] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLA[ARMA] is then cleared.
 */
#define PWM_SM1CAPTCTRLA_ONESHOTA_MASK           (0x2U)
#define PWM_SM1CAPTCTRLA_ONESHOTA_SHIFT          (1U)
#define PWM_SM1CAPTCTRLA_ONESHOTA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_ONESHOTA_SHIFT)) & PWM_SM1CAPTCTRLA_ONESHOTA_MASK)
/*! EDGA0 - Edge A 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLA_EDGA0_MASK              (0xCU)
#define PWM_SM1CAPTCTRLA_EDGA0_SHIFT             (2U)
#define PWM_SM1CAPTCTRLA_EDGA0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_EDGA0_SHIFT)) & PWM_SM1CAPTCTRLA_EDGA0_MASK)
/*! EDGA1 - Edge A 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLA_EDGA1_MASK              (0x30U)
#define PWM_SM1CAPTCTRLA_EDGA1_SHIFT             (4U)
#define PWM_SM1CAPTCTRLA_EDGA1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_EDGA1_SHIFT)) & PWM_SM1CAPTCTRLA_EDGA1_MASK)
/*! INP_SELA - Input Select A
 *  0b0..Raw PWM_A input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLA[EDGA0] and
 *       CAPTCTRLA[EDGA1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRA[EDGA0] and/or CAPTCTRLA[EDGA1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM1CAPTCTRLA_INP_SELA_MASK           (0x40U)
#define PWM_SM1CAPTCTRLA_INP_SELA_SHIFT          (6U)
#define PWM_SM1CAPTCTRLA_INP_SELA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_INP_SELA_SHIFT)) & PWM_SM1CAPTCTRLA_INP_SELA_MASK)
/*! EDGCNTA_EN - Edge Counter A Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM1CAPTCTRLA_EDGCNTA_EN_MASK         (0x80U)
#define PWM_SM1CAPTCTRLA_EDGCNTA_EN_SHIFT        (7U)
#define PWM_SM1CAPTCTRLA_EDGCNTA_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_EDGCNTA_EN_SHIFT)) & PWM_SM1CAPTCTRLA_EDGCNTA_EN_MASK)
#define PWM_SM1CAPTCTRLA_CFAWM_MASK              (0x300U)
#define PWM_SM1CAPTCTRLA_CFAWM_SHIFT             (8U)
#define PWM_SM1CAPTCTRLA_CFAWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_CFAWM_SHIFT)) & PWM_SM1CAPTCTRLA_CFAWM_MASK)
#define PWM_SM1CAPTCTRLA_CA0CNT_MASK             (0x1C00U)
#define PWM_SM1CAPTCTRLA_CA0CNT_SHIFT            (10U)
#define PWM_SM1CAPTCTRLA_CA0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_CA0CNT_SHIFT)) & PWM_SM1CAPTCTRLA_CA0CNT_MASK)
#define PWM_SM1CAPTCTRLA_CA1CNT_MASK             (0xE000U)
#define PWM_SM1CAPTCTRLA_CA1CNT_SHIFT            (13U)
#define PWM_SM1CAPTCTRLA_CA1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLA_CA1CNT_SHIFT)) & PWM_SM1CAPTCTRLA_CA1CNT_MASK)
/*! @} */

/*! @name SM1CAPTCOMPA - Capture Compare A Register */
/*! @{ */
#define PWM_SM1CAPTCOMPA_EDGCMPA_MASK            (0xFFU)
#define PWM_SM1CAPTCOMPA_EDGCMPA_SHIFT           (0U)
#define PWM_SM1CAPTCOMPA_EDGCMPA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPA_EDGCMPA_SHIFT)) & PWM_SM1CAPTCOMPA_EDGCMPA_MASK)
#define PWM_SM1CAPTCOMPA_EDGCNTA_MASK            (0xFF00U)
#define PWM_SM1CAPTCOMPA_EDGCNTA_SHIFT           (8U)
#define PWM_SM1CAPTCOMPA_EDGCNTA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPA_EDGCNTA_SHIFT)) & PWM_SM1CAPTCOMPA_EDGCNTA_MASK)
/*! @} */

/*! @name SM1CAPTCTRLB - Capture Control B Register */
/*! @{ */
/*! ARMB - Arm B
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLB[EDGBx] is enabled.
 */
#define PWM_SM1CAPTCTRLB_ARMB_MASK               (0x1U)
#define PWM_SM1CAPTCTRLB_ARMB_SHIFT              (0U)
#define PWM_SM1CAPTCTRLB_ARMB(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_ARMB_SHIFT)) & PWM_SM1CAPTCTRLB_ARMB_MASK)
/*! ONESHOTB - One Shot Mode B
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLB[ARMB] is cleared. No
 *       further captures will be performed until CAPTCTRLB[ARMB] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLB[ARMB] is then cleared.
 */
#define PWM_SM1CAPTCTRLB_ONESHOTB_MASK           (0x2U)
#define PWM_SM1CAPTCTRLB_ONESHOTB_SHIFT          (1U)
#define PWM_SM1CAPTCTRLB_ONESHOTB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_ONESHOTB_SHIFT)) & PWM_SM1CAPTCTRLB_ONESHOTB_MASK)
/*! EDGB0 - Edge B 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLB_EDGB0_MASK              (0xCU)
#define PWM_SM1CAPTCTRLB_EDGB0_SHIFT             (2U)
#define PWM_SM1CAPTCTRLB_EDGB0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_EDGB0_SHIFT)) & PWM_SM1CAPTCTRLB_EDGB0_MASK)
/*! EDGB1 - Edge B 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLB_EDGB1_MASK              (0x30U)
#define PWM_SM1CAPTCTRLB_EDGB1_SHIFT             (4U)
#define PWM_SM1CAPTCTRLB_EDGB1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_EDGB1_SHIFT)) & PWM_SM1CAPTCTRLB_EDGB1_MASK)
/*! INP_SELB - Input Select B
 *  0b0..Raw PWM_B input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLB[EDGB0] and
 *       CAPTCTRLB[EDGB1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRB[EDGB0] and/or CAPTCTRLB[EDGB1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM1CAPTCTRLB_INP_SELB_MASK           (0x40U)
#define PWM_SM1CAPTCTRLB_INP_SELB_SHIFT          (6U)
#define PWM_SM1CAPTCTRLB_INP_SELB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_INP_SELB_SHIFT)) & PWM_SM1CAPTCTRLB_INP_SELB_MASK)
/*! EDGCNTB_EN - Edge Counter B Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM1CAPTCTRLB_EDGCNTB_EN_MASK         (0x80U)
#define PWM_SM1CAPTCTRLB_EDGCNTB_EN_SHIFT        (7U)
#define PWM_SM1CAPTCTRLB_EDGCNTB_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_EDGCNTB_EN_SHIFT)) & PWM_SM1CAPTCTRLB_EDGCNTB_EN_MASK)
#define PWM_SM1CAPTCTRLB_CFBWM_MASK              (0x300U)
#define PWM_SM1CAPTCTRLB_CFBWM_SHIFT             (8U)
#define PWM_SM1CAPTCTRLB_CFBWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_CFBWM_SHIFT)) & PWM_SM1CAPTCTRLB_CFBWM_MASK)
#define PWM_SM1CAPTCTRLB_CB0CNT_MASK             (0x1C00U)
#define PWM_SM1CAPTCTRLB_CB0CNT_SHIFT            (10U)
#define PWM_SM1CAPTCTRLB_CB0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_CB0CNT_SHIFT)) & PWM_SM1CAPTCTRLB_CB0CNT_MASK)
#define PWM_SM1CAPTCTRLB_CB1CNT_MASK             (0xE000U)
#define PWM_SM1CAPTCTRLB_CB1CNT_SHIFT            (13U)
#define PWM_SM1CAPTCTRLB_CB1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLB_CB1CNT_SHIFT)) & PWM_SM1CAPTCTRLB_CB1CNT_MASK)
/*! @} */

/*! @name SM1CAPTCOMPB - Capture Compare B Register */
/*! @{ */
#define PWM_SM1CAPTCOMPB_EDGCMPB_MASK            (0xFFU)
#define PWM_SM1CAPTCOMPB_EDGCMPB_SHIFT           (0U)
#define PWM_SM1CAPTCOMPB_EDGCMPB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPB_EDGCMPB_SHIFT)) & PWM_SM1CAPTCOMPB_EDGCMPB_MASK)
#define PWM_SM1CAPTCOMPB_EDGCNTB_MASK            (0xFF00U)
#define PWM_SM1CAPTCOMPB_EDGCNTB_SHIFT           (8U)
#define PWM_SM1CAPTCOMPB_EDGCNTB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPB_EDGCNTB_SHIFT)) & PWM_SM1CAPTCOMPB_EDGCNTB_MASK)
/*! @} */

/*! @name SM1CAPTCTRLX - Capture Control X Register */
/*! @{ */
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_SM1CAPTCTRLX_ARMX_MASK               (0x1U)
#define PWM_SM1CAPTCTRLX_ARMX_SHIFT              (0U)
#define PWM_SM1CAPTCTRLX_ARMX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_ARMX_SHIFT)) & PWM_SM1CAPTCTRLX_ARMX_MASK)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The
 *       process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and the ARMX bit is cleared. No further
 *       captures will be performed until the ARMX bit is set again.If only one of the capture circuits is enabled,
 *       then a single capture will occur on the enabled capture circuit and the ARMX bit is then cleared.
 */
#define PWM_SM1CAPTCTRLX_ONESHOTX_MASK           (0x2U)
#define PWM_SM1CAPTCTRLX_ONESHOTX_SHIFT          (1U)
#define PWM_SM1CAPTCTRLX_ONESHOTX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_SM1CAPTCTRLX_ONESHOTX_MASK)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLX_EDGX0_MASK              (0xCU)
#define PWM_SM1CAPTCTRLX_EDGX0_SHIFT             (2U)
#define PWM_SM1CAPTCTRLX_EDGX0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_EDGX0_SHIFT)) & PWM_SM1CAPTCTRLX_EDGX0_MASK)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM1CAPTCTRLX_EDGX1_MASK              (0x30U)
#define PWM_SM1CAPTCTRLX_EDGX1_SHIFT             (4U)
#define PWM_SM1CAPTCTRLX_EDGX1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_EDGX1_SHIFT)) & PWM_SM1CAPTCTRLX_EDGX1_MASK)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLX[EDGX0] and
 *       CAPTCTRLX[EDGX1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRX[EDGX0] and/or CAPTCTRLX[EDGX1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM1CAPTCTRLX_INP_SELX_MASK           (0x40U)
#define PWM_SM1CAPTCTRLX_INP_SELX_SHIFT          (6U)
#define PWM_SM1CAPTCTRLX_INP_SELX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_INP_SELX_SHIFT)) & PWM_SM1CAPTCTRLX_INP_SELX_MASK)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM1CAPTCTRLX_EDGCNTX_EN_MASK         (0x80U)
#define PWM_SM1CAPTCTRLX_EDGCNTX_EN_SHIFT        (7U)
#define PWM_SM1CAPTCTRLX_EDGCNTX_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_SM1CAPTCTRLX_EDGCNTX_EN_MASK)
#define PWM_SM1CAPTCTRLX_CFXWM_MASK              (0x300U)
#define PWM_SM1CAPTCTRLX_CFXWM_SHIFT             (8U)
#define PWM_SM1CAPTCTRLX_CFXWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_CFXWM_SHIFT)) & PWM_SM1CAPTCTRLX_CFXWM_MASK)
#define PWM_SM1CAPTCTRLX_CX0CNT_MASK             (0x1C00U)
#define PWM_SM1CAPTCTRLX_CX0CNT_SHIFT            (10U)
#define PWM_SM1CAPTCTRLX_CX0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_CX0CNT_SHIFT)) & PWM_SM1CAPTCTRLX_CX0CNT_MASK)
#define PWM_SM1CAPTCTRLX_CX1CNT_MASK             (0xE000U)
#define PWM_SM1CAPTCTRLX_CX1CNT_SHIFT            (13U)
#define PWM_SM1CAPTCTRLX_CX1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCTRLX_CX1CNT_SHIFT)) & PWM_SM1CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/*! @name SM1CAPTCOMPX - Capture Compare X Register */
/*! @{ */
#define PWM_SM1CAPTCOMPX_EDGCMPX_MASK            (0xFFU)
#define PWM_SM1CAPTCOMPX_EDGCMPX_SHIFT           (0U)
#define PWM_SM1CAPTCOMPX_EDGCMPX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_SM1CAPTCOMPX_EDGCMPX_MASK)
#define PWM_SM1CAPTCOMPX_EDGCNTX_MASK            (0xFF00U)
#define PWM_SM1CAPTCOMPX_EDGCNTX_SHIFT           (8U)
#define PWM_SM1CAPTCOMPX_EDGCNTX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_SM1CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/*! @name SM1CVAL0 - Capture Value 0 Register */
/*! @{ */
#define PWM_SM1CVAL0_CAPTVAL0_MASK               (0xFFFFU)
#define PWM_SM1CVAL0_CAPTVAL0_SHIFT              (0U)
#define PWM_SM1CVAL0_CAPTVAL0(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL0_CAPTVAL0_SHIFT)) & PWM_SM1CVAL0_CAPTVAL0_MASK)
/*! @} */

/*! @name SM1CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL0CYC_CVAL0CYC_MASK            (0xFU)
#define PWM_SM1CVAL0CYC_CVAL0CYC_SHIFT           (0U)
#define PWM_SM1CVAL0CYC_CVAL0CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_SM1CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/*! @name SM1CVAL1 - Capture Value 1 Register */
/*! @{ */
#define PWM_SM1CVAL1_CAPTVAL1_MASK               (0xFFFFU)
#define PWM_SM1CVAL1_CAPTVAL1_SHIFT              (0U)
#define PWM_SM1CVAL1_CAPTVAL1(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL1_CAPTVAL1_SHIFT)) & PWM_SM1CVAL1_CAPTVAL1_MASK)
/*! @} */

/*! @name SM1CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL1CYC_CVAL1CYC_MASK            (0xFU)
#define PWM_SM1CVAL1CYC_CVAL1CYC_SHIFT           (0U)
#define PWM_SM1CVAL1CYC_CVAL1CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_SM1CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/*! @name SM1CVAL2 - Capture Value 2 Register */
/*! @{ */
#define PWM_SM1CVAL2_CAPTVAL2_MASK               (0xFFFFU)
#define PWM_SM1CVAL2_CAPTVAL2_SHIFT              (0U)
#define PWM_SM1CVAL2_CAPTVAL2(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL2_CAPTVAL2_SHIFT)) & PWM_SM1CVAL2_CAPTVAL2_MASK)
/*! @} */

/*! @name SM1CVAL2CYC - Capture Value 2 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL2CYC_CVAL2CYC_MASK            (0xFU)
#define PWM_SM1CVAL2CYC_CVAL2CYC_SHIFT           (0U)
#define PWM_SM1CVAL2CYC_CVAL2CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL2CYC_CVAL2CYC_SHIFT)) & PWM_SM1CVAL2CYC_CVAL2CYC_MASK)
/*! @} */

/*! @name SM1CVAL3 - Capture Value 3 Register */
/*! @{ */
#define PWM_SM1CVAL3_CAPTVAL3_MASK               (0xFFFFU)
#define PWM_SM1CVAL3_CAPTVAL3_SHIFT              (0U)
#define PWM_SM1CVAL3_CAPTVAL3(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL3_CAPTVAL3_SHIFT)) & PWM_SM1CVAL3_CAPTVAL3_MASK)
/*! @} */

/*! @name SM1CVAL3CYC - Capture Value 3 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL3CYC_CVAL3CYC_MASK            (0xFU)
#define PWM_SM1CVAL3CYC_CVAL3CYC_SHIFT           (0U)
#define PWM_SM1CVAL3CYC_CVAL3CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL3CYC_CVAL3CYC_SHIFT)) & PWM_SM1CVAL3CYC_CVAL3CYC_MASK)
/*! @} */

/*! @name SM1CVAL4 - Capture Value 4 Register */
/*! @{ */
#define PWM_SM1CVAL4_CAPTVAL4_MASK               (0xFFFFU)
#define PWM_SM1CVAL4_CAPTVAL4_SHIFT              (0U)
#define PWM_SM1CVAL4_CAPTVAL4(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL4_CAPTVAL4_SHIFT)) & PWM_SM1CVAL4_CAPTVAL4_MASK)
/*! @} */

/*! @name SM1CVAL4CYC - Capture Value 4 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL4CYC_CVAL4CYC_MASK            (0xFU)
#define PWM_SM1CVAL4CYC_CVAL4CYC_SHIFT           (0U)
#define PWM_SM1CVAL4CYC_CVAL4CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL4CYC_CVAL4CYC_SHIFT)) & PWM_SM1CVAL4CYC_CVAL4CYC_MASK)
/*! @} */

/*! @name SM1CVAL5 - Capture Value 5 Register */
/*! @{ */
#define PWM_SM1CVAL5_CAPTVAL5_MASK               (0xFFFFU)
#define PWM_SM1CVAL5_CAPTVAL5_SHIFT              (0U)
#define PWM_SM1CVAL5_CAPTVAL5(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL5_CAPTVAL5_SHIFT)) & PWM_SM1CVAL5_CAPTVAL5_MASK)
/*! @} */

/*! @name SM1CVAL5CYC - Capture Value 5 Cycle Register */
/*! @{ */
#define PWM_SM1CVAL5CYC_CVAL5CYC_MASK            (0xFU)
#define PWM_SM1CVAL5CYC_CVAL5CYC_SHIFT           (0U)
#define PWM_SM1CVAL5CYC_CVAL5CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1CVAL5CYC_CVAL5CYC_SHIFT)) & PWM_SM1CVAL5CYC_CVAL5CYC_MASK)
/*! @} */

/*! @name SM1PHASEDLY - Phase Delay Register */
/*! @{ */
#define PWM_SM1PHASEDLY_PHASEDLY_MASK            (0xFFFFU)
#define PWM_SM1PHASEDLY_PHASEDLY_SHIFT           (0U)
#define PWM_SM1PHASEDLY_PHASEDLY(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM1PHASEDLY_PHASEDLY_SHIFT)) & PWM_SM1PHASEDLY_PHASEDLY_MASK)
/*! @} */

/*! @name SM2CNT - Counter Register */
/*! @{ */
#define PWM_SM2CNT_CNT_MASK                      (0xFFFFU)
#define PWM_SM2CNT_CNT_SHIFT                     (0U)
#define PWM_SM2CNT_CNT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM2CNT_CNT_SHIFT)) & PWM_SM2CNT_CNT_MASK)
/*! @} */

/*! @name SM2INIT - Initial Count Register */
/*! @{ */
#define PWM_SM2INIT_INIT_MASK                    (0xFFFFU)
#define PWM_SM2INIT_INIT_SHIFT                   (0U)
#define PWM_SM2INIT_INIT(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2INIT_INIT_SHIFT)) & PWM_SM2INIT_INIT_MASK)
/*! @} */

/*! @name SM2CTRL2 - Control 2 Register */
/*! @{ */
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it will force the clock to logic 0.
 *  0b11..reserved
 */
#define PWM_SM2CTRL2_CLK_SEL_MASK                (0x3U)
#define PWM_SM2CTRL2_CLK_SEL_SHIFT               (0U)
#define PWM_SM2CTRL2_CLK_SEL(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_CLK_SEL_SHIFT)) & PWM_SM2CTRL2_CLK_SEL_MASK)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it will force the RELOAD signal to logic 0.
 */
#define PWM_SM2CTRL2_RELOAD_SEL_MASK             (0x4U)
#define PWM_SM2CTRL2_RELOAD_SEL_SHIFT            (2U)
#define PWM_SM2CTRL2_RELOAD_SEL(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_RELOAD_SEL_SHIFT)) & PWM_SM2CTRL2_RELOAD_SEL_MASK)
/*! FORCE_SEL - This read/write bit determines the source of the FORCE OUTPUT signal for this submodule.
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_SM2CTRL2_FORCE_SEL_MASK              (0x38U)
#define PWM_SM2CTRL2_FORCE_SEL_SHIFT             (3U)
#define PWM_SM2CTRL2_FORCE_SEL(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_FORCE_SEL_SHIFT)) & PWM_SM2CTRL2_FORCE_SEL_MASK)
#define PWM_SM2CTRL2_FORCE_MASK                  (0x40U)
#define PWM_SM2CTRL2_FORCE_SHIFT                 (6U)
#define PWM_SM2CTRL2_FORCE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_FORCE_SHIFT)) & PWM_SM2CTRL2_FORCE_MASK)
/*! FRCEN - FRCEN
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_SM2CTRL2_FRCEN_MASK                  (0x80U)
#define PWM_SM2CTRL2_FRCEN_SHIFT                 (7U)
#define PWM_SM2CTRL2_FRCEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_FRCEN_SHIFT)) & PWM_SM2CTRL2_FRCEN_MASK)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it will force the INIT signal to logic 0. The submodule counter will only reinitialize when a master
 *        reload occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it
 *        will force the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_SM2CTRL2_INIT_SEL_MASK               (0x300U)
#define PWM_SM2CTRL2_INIT_SEL_SHIFT              (8U)
#define PWM_SM2CTRL2_INIT_SEL(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_INIT_SEL_SHIFT)) & PWM_SM2CTRL2_INIT_SEL_MASK)
#define PWM_SM2CTRL2_PWMX_INIT_MASK              (0x400U)
#define PWM_SM2CTRL2_PWMX_INIT_SHIFT             (10U)
#define PWM_SM2CTRL2_PWMX_INIT(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_PWMX_INIT_SHIFT)) & PWM_SM2CTRL2_PWMX_INIT_MASK)
#define PWM_SM2CTRL2_PWM45_INIT_MASK             (0x800U)
#define PWM_SM2CTRL2_PWM45_INIT_SHIFT            (11U)
#define PWM_SM2CTRL2_PWM45_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_PWM45_INIT_SHIFT)) & PWM_SM2CTRL2_PWM45_INIT_MASK)
#define PWM_SM2CTRL2_PWM23_INIT_MASK             (0x1000U)
#define PWM_SM2CTRL2_PWM23_INIT_SHIFT            (12U)
#define PWM_SM2CTRL2_PWM23_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_PWM23_INIT_SHIFT)) & PWM_SM2CTRL2_PWM23_INIT_MASK)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_SM2CTRL2_INDEP_MASK                  (0x2000U)
#define PWM_SM2CTRL2_INDEP_SHIFT                 (13U)
#define PWM_SM2CTRL2_INDEP(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_INDEP_SHIFT)) & PWM_SM2CTRL2_INDEP_MASK)
#define PWM_SM2CTRL2_WAITEN_MASK                 (0x4000U)
#define PWM_SM2CTRL2_WAITEN_SHIFT                (14U)
#define PWM_SM2CTRL2_WAITEN(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_WAITEN_SHIFT)) & PWM_SM2CTRL2_WAITEN_MASK)
#define PWM_SM2CTRL2_DBGEN_MASK                  (0x8000U)
#define PWM_SM2CTRL2_DBGEN_SHIFT                 (15U)
#define PWM_SM2CTRL2_DBGEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL2_DBGEN_SHIFT)) & PWM_SM2CTRL2_DBGEN_MASK)
/*! @} */

/*! @name SM2CTRL - Control Register */
/*! @{ */
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_SM2CTRL_DBLEN_MASK                   (0x1U)
#define PWM_SM2CTRL_DBLEN_SHIFT                  (0U)
#define PWM_SM2CTRL_DBLEN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_DBLEN_SHIFT)) & PWM_SM2CTRL_DBLEN_MASK)
/*! DBLX - PWMX Double Switching Enable
 *  0b0..PWMX double pulse disabled.
 *  0b1..PWMX double pulse enabled.
 */
#define PWM_SM2CTRL_DBLX_MASK                    (0x2U)
#define PWM_SM2CTRL_DBLX_SHIFT                   (1U)
#define PWM_SM2CTRL_DBLX(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_DBLX_SHIFT)) & PWM_SM2CTRL_DBLX_MASK)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_SM2CTRL_LDMOD_MASK                   (0x4U)
#define PWM_SM2CTRL_LDMOD_SHIFT                  (2U)
#define PWM_SM2CTRL_LDMOD(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_LDMOD_SHIFT)) & PWM_SM2CTRL_LDMOD_MASK)
/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
 *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
 *  0b1..DBLPWM is split to PWMA and PWMB.
 */
#define PWM_SM2CTRL_SPLIT_MASK                   (0x8U)
#define PWM_SM2CTRL_SPLIT_SHIFT                  (3U)
#define PWM_SM2CTRL_SPLIT(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_SPLIT_SHIFT)) & PWM_SM2CTRL_SPLIT_MASK)
/*! PRSC - Prescaler
 *  0b000..PWM clock frequency = fclk
 *  0b001..PWM clock frequency = fclk/2
 *  0b010..PWM clock frequency = fclk/4
 *  0b011..PWM clock frequency = fclk/8
 *  0b100..PWM clock frequency = fclk/16
 *  0b101..PWM clock frequency = fclk/32
 *  0b110..PWM clock frequency = fclk/64
 *  0b111..PWM clock frequency = fclk/128
 */
#define PWM_SM2CTRL_PRSC_MASK                    (0x70U)
#define PWM_SM2CTRL_PRSC_SHIFT                   (4U)
#define PWM_SM2CTRL_PRSC(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_PRSC_SHIFT)) & PWM_SM2CTRL_PRSC_MASK)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
 *       output in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_SM2CTRL_COMPMODE_MASK                (0x80U)
#define PWM_SM2CTRL_COMPMODE_SHIFT               (7U)
#define PWM_SM2CTRL_COMPMODE(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_COMPMODE_SHIFT)) & PWM_SM2CTRL_COMPMODE_MASK)
#define PWM_SM2CTRL_DT_MASK                      (0x300U)
#define PWM_SM2CTRL_DT_SHIFT                     (8U)
#define PWM_SM2CTRL_DT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_DT_SHIFT)) & PWM_SM2CTRL_DT_MASK)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_SM2CTRL_FULL_MASK                    (0x400U)
#define PWM_SM2CTRL_FULL_SHIFT                   (10U)
#define PWM_SM2CTRL_FULL(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_FULL_SHIFT)) & PWM_SM2CTRL_FULL_MASK)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_SM2CTRL_HALF_MASK                    (0x800U)
#define PWM_SM2CTRL_HALF_SHIFT                   (11U)
#define PWM_SM2CTRL_HALF(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_HALF_SHIFT)) & PWM_SM2CTRL_HALF_MASK)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_SM2CTRL_LDFQ_MASK                    (0xF000U)
#define PWM_SM2CTRL_LDFQ_SHIFT                   (12U)
#define PWM_SM2CTRL_LDFQ(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2CTRL_LDFQ_SHIFT)) & PWM_SM2CTRL_LDFQ_MASK)
/*! @} */

/*! @name SM2VAL0 - Value Register 0 */
/*! @{ */
#define PWM_SM2VAL0_VAL0_MASK                    (0xFFFFU)
#define PWM_SM2VAL0_VAL0_SHIFT                   (0U)
#define PWM_SM2VAL0_VAL0(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL0_VAL0_SHIFT)) & PWM_SM2VAL0_VAL0_MASK)
/*! @} */

/*! @name SM2FRACVAL1 - Fractional Value Register 1 */
/*! @{ */
#define PWM_SM2FRACVAL1_FRACVAL1_MASK            (0xF800U)
#define PWM_SM2FRACVAL1_FRACVAL1_SHIFT           (11U)
#define PWM_SM2FRACVAL1_FRACVAL1(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRACVAL1_FRACVAL1_SHIFT)) & PWM_SM2FRACVAL1_FRACVAL1_MASK)
/*! @} */

/*! @name SM2VAL1 - Value Register 1 */
/*! @{ */
#define PWM_SM2VAL1_VAL1_MASK                    (0xFFFFU)
#define PWM_SM2VAL1_VAL1_SHIFT                   (0U)
#define PWM_SM2VAL1_VAL1(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL1_VAL1_SHIFT)) & PWM_SM2VAL1_VAL1_MASK)
/*! @} */

/*! @name SM2FRACVAL2 - Fractional Value Register 2 */
/*! @{ */
#define PWM_SM2FRACVAL2_FRACVAL2_MASK            (0xF800U)
#define PWM_SM2FRACVAL2_FRACVAL2_SHIFT           (11U)
#define PWM_SM2FRACVAL2_FRACVAL2(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRACVAL2_FRACVAL2_SHIFT)) & PWM_SM2FRACVAL2_FRACVAL2_MASK)
/*! @} */

/*! @name SM2VAL2 - Value Register 2 */
/*! @{ */
#define PWM_SM2VAL2_VAL2_MASK                    (0xFFFFU)
#define PWM_SM2VAL2_VAL2_SHIFT                   (0U)
#define PWM_SM2VAL2_VAL2(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL2_VAL2_SHIFT)) & PWM_SM2VAL2_VAL2_MASK)
/*! @} */

/*! @name SM2FRACVAL3 - Fractional Value Register 3 */
/*! @{ */
#define PWM_SM2FRACVAL3_FRACVAL3_MASK            (0xF800U)
#define PWM_SM2FRACVAL3_FRACVAL3_SHIFT           (11U)
#define PWM_SM2FRACVAL3_FRACVAL3(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRACVAL3_FRACVAL3_SHIFT)) & PWM_SM2FRACVAL3_FRACVAL3_MASK)
/*! @} */

/*! @name SM2VAL3 - Value Register 3 */
/*! @{ */
#define PWM_SM2VAL3_VAL3_MASK                    (0xFFFFU)
#define PWM_SM2VAL3_VAL3_SHIFT                   (0U)
#define PWM_SM2VAL3_VAL3(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL3_VAL3_SHIFT)) & PWM_SM2VAL3_VAL3_MASK)
/*! @} */

/*! @name SM2FRACVAL4 - Fractional Value Register 4 */
/*! @{ */
#define PWM_SM2FRACVAL4_FRACVAL4_MASK            (0xF800U)
#define PWM_SM2FRACVAL4_FRACVAL4_SHIFT           (11U)
#define PWM_SM2FRACVAL4_FRACVAL4(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRACVAL4_FRACVAL4_SHIFT)) & PWM_SM2FRACVAL4_FRACVAL4_MASK)
/*! @} */

/*! @name SM2VAL4 - Value Register 4 */
/*! @{ */
#define PWM_SM2VAL4_VAL4_MASK                    (0xFFFFU)
#define PWM_SM2VAL4_VAL4_SHIFT                   (0U)
#define PWM_SM2VAL4_VAL4(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL4_VAL4_SHIFT)) & PWM_SM2VAL4_VAL4_MASK)
/*! @} */

/*! @name SM2FRACVAL5 - Fractional Value Register 5 */
/*! @{ */
#define PWM_SM2FRACVAL5_FRACVAL5_MASK            (0xF800U)
#define PWM_SM2FRACVAL5_FRACVAL5_SHIFT           (11U)
#define PWM_SM2FRACVAL5_FRACVAL5(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRACVAL5_FRACVAL5_SHIFT)) & PWM_SM2FRACVAL5_FRACVAL5_MASK)
/*! @} */

/*! @name SM2VAL5 - Value Register 5 */
/*! @{ */
#define PWM_SM2VAL5_VAL5_MASK                    (0xFFFFU)
#define PWM_SM2VAL5_VAL5_SHIFT                   (0U)
#define PWM_SM2VAL5_VAL5(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2VAL5_VAL5_SHIFT)) & PWM_SM2VAL5_VAL5_MASK)
/*! @} */

/*! @name SM2FRCTRL - Fractional Control Register */
/*! @{ */
/*! FRAC1_EN - Fractional Cycle PWM Period Enable
 *  0b0..Disable fractional cycle length for the PWM period.
 *  0b1..Enable fractional cycle length for the PWM period.
 */
#define PWM_SM2FRCTRL_FRAC1_EN_MASK              (0x2U)
#define PWM_SM2FRCTRL_FRAC1_EN_SHIFT             (1U)
#define PWM_SM2FRCTRL_FRAC1_EN(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRCTRL_FRAC1_EN_SHIFT)) & PWM_SM2FRCTRL_FRAC1_EN_MASK)
/*! FRAC23_EN - Fractional Cycle Placement Enable for PWM_A
 *  0b0..Disable fractional cycle placement for PWM_A.
 *  0b1..Enable fractional cycle placement for PWM_A.
 */
#define PWM_SM2FRCTRL_FRAC23_EN_MASK             (0x4U)
#define PWM_SM2FRCTRL_FRAC23_EN_SHIFT            (2U)
#define PWM_SM2FRCTRL_FRAC23_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRCTRL_FRAC23_EN_SHIFT)) & PWM_SM2FRCTRL_FRAC23_EN_MASK)
/*! FRAC45_EN - Fractional Cycle Placement Enable for PWM_B
 *  0b0..Disable fractional cycle placement for PWM_B.
 *  0b1..Enable fractional cycle placement for PWM_B.
 */
#define PWM_SM2FRCTRL_FRAC45_EN_MASK             (0x10U)
#define PWM_SM2FRCTRL_FRAC45_EN_SHIFT            (4U)
#define PWM_SM2FRCTRL_FRAC45_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRCTRL_FRAC45_EN_SHIFT)) & PWM_SM2FRCTRL_FRAC45_EN_MASK)
/*! FRAC_PU - Fractional Delay Circuit Power Up
 *  0b0..Turn off fractional delay logic.
 *  0b1..Power up fractional delay logic.
 */
#define PWM_SM2FRCTRL_FRAC_PU_MASK               (0x100U)
#define PWM_SM2FRCTRL_FRAC_PU_SHIFT              (8U)
#define PWM_SM2FRCTRL_FRAC_PU(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRCTRL_FRAC_PU_SHIFT)) & PWM_SM2FRCTRL_FRAC_PU_MASK)
#define PWM_SM2FRCTRL_TEST_MASK                  (0x8000U)
#define PWM_SM2FRCTRL_TEST_SHIFT                 (15U)
#define PWM_SM2FRCTRL_TEST(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2FRCTRL_TEST_SHIFT)) & PWM_SM2FRCTRL_TEST_MASK)
/*! @} */

/*! @name SM2OCTRL - Output Control Register */
/*! @{ */
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM2OCTRL_PWMXFS_MASK                 (0x3U)
#define PWM_SM2OCTRL_PWMXFS_SHIFT                (0U)
#define PWM_SM2OCTRL_PWMXFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMXFS_SHIFT)) & PWM_SM2OCTRL_PWMXFS_MASK)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM2OCTRL_PWMBFS_MASK                 (0xCU)
#define PWM_SM2OCTRL_PWMBFS_SHIFT                (2U)
#define PWM_SM2OCTRL_PWMBFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMBFS_SHIFT)) & PWM_SM2OCTRL_PWMBFS_MASK)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM2OCTRL_PWMAFS_MASK                 (0x30U)
#define PWM_SM2OCTRL_PWMAFS_SHIFT                (4U)
#define PWM_SM2OCTRL_PWMAFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMAFS_SHIFT)) & PWM_SM2OCTRL_PWMAFS_MASK)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_SM2OCTRL_POLX_MASK                   (0x100U)
#define PWM_SM2OCTRL_POLX_SHIFT                  (8U)
#define PWM_SM2OCTRL_POLX(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_POLX_SHIFT)) & PWM_SM2OCTRL_POLX_MASK)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_SM2OCTRL_POLB_MASK                   (0x200U)
#define PWM_SM2OCTRL_POLB_SHIFT                  (9U)
#define PWM_SM2OCTRL_POLB(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_POLB_SHIFT)) & PWM_SM2OCTRL_POLB_MASK)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_SM2OCTRL_POLA_MASK                   (0x400U)
#define PWM_SM2OCTRL_POLA_SHIFT                  (10U)
#define PWM_SM2OCTRL_POLA(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_POLA_SHIFT)) & PWM_SM2OCTRL_POLA_MASK)
#define PWM_SM2OCTRL_PWMX_IN_MASK                (0x2000U)
#define PWM_SM2OCTRL_PWMX_IN_SHIFT               (13U)
#define PWM_SM2OCTRL_PWMX_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMX_IN_SHIFT)) & PWM_SM2OCTRL_PWMX_IN_MASK)
#define PWM_SM2OCTRL_PWMB_IN_MASK                (0x4000U)
#define PWM_SM2OCTRL_PWMB_IN_SHIFT               (14U)
#define PWM_SM2OCTRL_PWMB_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMB_IN_SHIFT)) & PWM_SM2OCTRL_PWMB_IN_MASK)
#define PWM_SM2OCTRL_PWMA_IN_MASK                (0x8000U)
#define PWM_SM2OCTRL_PWMA_IN_SHIFT               (15U)
#define PWM_SM2OCTRL_PWMA_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2OCTRL_PWMA_IN_SHIFT)) & PWM_SM2OCTRL_PWMA_IN_MASK)
/*! @} */

/*! @name SM2STS - Status Register */
/*! @{ */
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_SM2STS_CMPF_MASK                     (0x3FU)
#define PWM_SM2STS_CMPF_SHIFT                    (0U)
#define PWM_SM2STS_CMPF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CMPF_SHIFT)) & PWM_SM2STS_CMPF_MASK)
#define PWM_SM2STS_CFX0_MASK                     (0x40U)
#define PWM_SM2STS_CFX0_SHIFT                    (6U)
#define PWM_SM2STS_CFX0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFX0_SHIFT)) & PWM_SM2STS_CFX0_MASK)
#define PWM_SM2STS_CFX1_MASK                     (0x80U)
#define PWM_SM2STS_CFX1_SHIFT                    (7U)
#define PWM_SM2STS_CFX1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFX1_SHIFT)) & PWM_SM2STS_CFX1_MASK)
#define PWM_SM2STS_CFB0_MASK                     (0x100U)
#define PWM_SM2STS_CFB0_SHIFT                    (8U)
#define PWM_SM2STS_CFB0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFB0_SHIFT)) & PWM_SM2STS_CFB0_MASK)
#define PWM_SM2STS_CFB1_MASK                     (0x200U)
#define PWM_SM2STS_CFB1_SHIFT                    (9U)
#define PWM_SM2STS_CFB1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFB1_SHIFT)) & PWM_SM2STS_CFB1_MASK)
#define PWM_SM2STS_CFA0_MASK                     (0x400U)
#define PWM_SM2STS_CFA0_SHIFT                    (10U)
#define PWM_SM2STS_CFA0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFA0_SHIFT)) & PWM_SM2STS_CFA0_MASK)
#define PWM_SM2STS_CFA1_MASK                     (0x800U)
#define PWM_SM2STS_CFA1_SHIFT                    (11U)
#define PWM_SM2STS_CFA1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_CFA1_SHIFT)) & PWM_SM2STS_CFA1_MASK)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_SM2STS_RF_MASK                       (0x1000U)
#define PWM_SM2STS_RF_SHIFT                      (12U)
#define PWM_SM2STS_RF(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_RF_SHIFT)) & PWM_SM2STS_RF_MASK)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_SM2STS_REF_MASK                      (0x2000U)
#define PWM_SM2STS_REF_SHIFT                     (13U)
#define PWM_SM2STS_REF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_REF_SHIFT)) & PWM_SM2STS_REF_MASK)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_SM2STS_RUF_MASK                      (0x4000U)
#define PWM_SM2STS_RUF_SHIFT                     (14U)
#define PWM_SM2STS_RUF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM2STS_RUF_SHIFT)) & PWM_SM2STS_RUF_MASK)
/*! @} */

/*! @name SM2INTEN - Interrupt Enable Register */
/*! @{ */
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_SM2INTEN_CMPIE_MASK                  (0x3FU)
#define PWM_SM2INTEN_CMPIE_SHIFT                 (0U)
#define PWM_SM2INTEN_CMPIE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CMPIE_SHIFT)) & PWM_SM2INTEN_CMPIE_MASK)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_SM2INTEN_CX0IE_MASK                  (0x40U)
#define PWM_SM2INTEN_CX0IE_SHIFT                 (6U)
#define PWM_SM2INTEN_CX0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CX0IE_SHIFT)) & PWM_SM2INTEN_CX0IE_MASK)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_SM2INTEN_CX1IE_MASK                  (0x80U)
#define PWM_SM2INTEN_CX1IE_SHIFT                 (7U)
#define PWM_SM2INTEN_CX1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CX1IE_SHIFT)) & PWM_SM2INTEN_CX1IE_MASK)
/*! CB0IE - Capture B 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB0].
 *  0b1..Interrupt request enabled for STS[CFB0].
 */
#define PWM_SM2INTEN_CB0IE_MASK                  (0x100U)
#define PWM_SM2INTEN_CB0IE_SHIFT                 (8U)
#define PWM_SM2INTEN_CB0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CB0IE_SHIFT)) & PWM_SM2INTEN_CB0IE_MASK)
/*! CB1IE - Capture B 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB1].
 *  0b1..Interrupt request enabled for STS[CFB1].
 */
#define PWM_SM2INTEN_CB1IE_MASK                  (0x200U)
#define PWM_SM2INTEN_CB1IE_SHIFT                 (9U)
#define PWM_SM2INTEN_CB1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CB1IE_SHIFT)) & PWM_SM2INTEN_CB1IE_MASK)
/*! CA0IE - Capture A 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA0].
 *  0b1..Interrupt request enabled for STS[CFA0].
 */
#define PWM_SM2INTEN_CA0IE_MASK                  (0x400U)
#define PWM_SM2INTEN_CA0IE_SHIFT                 (10U)
#define PWM_SM2INTEN_CA0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CA0IE_SHIFT)) & PWM_SM2INTEN_CA0IE_MASK)
/*! CA1IE - Capture A 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA1].
 *  0b1..Interrupt request enabled for STS[CFA1].
 */
#define PWM_SM2INTEN_CA1IE_MASK                  (0x800U)
#define PWM_SM2INTEN_CA1IE_SHIFT                 (11U)
#define PWM_SM2INTEN_CA1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_CA1IE_SHIFT)) & PWM_SM2INTEN_CA1IE_MASK)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_SM2INTEN_RIE_MASK                    (0x1000U)
#define PWM_SM2INTEN_RIE_SHIFT                   (12U)
#define PWM_SM2INTEN_RIE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_RIE_SHIFT)) & PWM_SM2INTEN_RIE_MASK)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_SM2INTEN_REIE_MASK                   (0x2000U)
#define PWM_SM2INTEN_REIE_SHIFT                  (13U)
#define PWM_SM2INTEN_REIE(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2INTEN_REIE_SHIFT)) & PWM_SM2INTEN_REIE_MASK)
/*! @} */

/*! @name SM2DMAEN - DMA Enable Register */
/*! @{ */
#define PWM_SM2DMAEN_CX0DE_MASK                  (0x1U)
#define PWM_SM2DMAEN_CX0DE_SHIFT                 (0U)
#define PWM_SM2DMAEN_CX0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CX0DE_SHIFT)) & PWM_SM2DMAEN_CX0DE_MASK)
#define PWM_SM2DMAEN_CX1DE_MASK                  (0x2U)
#define PWM_SM2DMAEN_CX1DE_SHIFT                 (1U)
#define PWM_SM2DMAEN_CX1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CX1DE_SHIFT)) & PWM_SM2DMAEN_CX1DE_MASK)
#define PWM_SM2DMAEN_CB0DE_MASK                  (0x4U)
#define PWM_SM2DMAEN_CB0DE_SHIFT                 (2U)
#define PWM_SM2DMAEN_CB0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CB0DE_SHIFT)) & PWM_SM2DMAEN_CB0DE_MASK)
#define PWM_SM2DMAEN_CB1DE_MASK                  (0x8U)
#define PWM_SM2DMAEN_CB1DE_SHIFT                 (3U)
#define PWM_SM2DMAEN_CB1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CB1DE_SHIFT)) & PWM_SM2DMAEN_CB1DE_MASK)
#define PWM_SM2DMAEN_CA0DE_MASK                  (0x10U)
#define PWM_SM2DMAEN_CA0DE_SHIFT                 (4U)
#define PWM_SM2DMAEN_CA0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CA0DE_SHIFT)) & PWM_SM2DMAEN_CA0DE_MASK)
#define PWM_SM2DMAEN_CA1DE_MASK                  (0x20U)
#define PWM_SM2DMAEN_CA1DE_SHIFT                 (5U)
#define PWM_SM2DMAEN_CA1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CA1DE_SHIFT)) & PWM_SM2DMAEN_CA1DE_MASK)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to also be set in order to determine to
 *        which watermark(s) the DMA request is sensitive.
 *  0b10..A local sync (VAL1 matches counter) sets the read DMA request.
 *  0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_SM2DMAEN_CAPTDE_MASK                 (0xC0U)
#define PWM_SM2DMAEN_CAPTDE_SHIFT                (6U)
#define PWM_SM2DMAEN_CAPTDE(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_CAPTDE_SHIFT)) & PWM_SM2DMAEN_CAPTDE_MASK)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_SM2DMAEN_FAND_MASK                   (0x100U)
#define PWM_SM2DMAEN_FAND_SHIFT                  (8U)
#define PWM_SM2DMAEN_FAND(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_FAND_SHIFT)) & PWM_SM2DMAEN_FAND_MASK)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..DMA write requests for the VALx and FRACVALx registers enabled
 */
#define PWM_SM2DMAEN_VALDE_MASK                  (0x200U)
#define PWM_SM2DMAEN_VALDE_SHIFT                 (9U)
#define PWM_SM2DMAEN_VALDE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM2DMAEN_VALDE_SHIFT)) & PWM_SM2DMAEN_VALDE_MASK)
/*! @} */

/*! @name SM2TCTRL - Output Trigger Control Register */
/*! @{ */
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b000000..PWM_OUT_TRIGx will not set when the counter value matches the VALx value.
 *  0b000001..PWM_OUT_TRIGx will set when the counter value matches the VALx value.
 */
#define PWM_SM2TCTRL_OUT_TRIG_EN_MASK            (0x3FU)
#define PWM_SM2TCTRL_OUT_TRIG_EN_SHIFT           (0U)
#define PWM_SM2TCTRL_OUT_TRIG_EN(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_SM2TCTRL_OUT_TRIG_EN_MASK)
/*! TRGFRQ - Trigger frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 *  0b1..Trigger outputs are generated only during the final PWM period prior to a reload opportunity when the PWM
 *       is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_SM2TCTRL_TRGFRQ_MASK                 (0x1000U)
#define PWM_SM2TCTRL_TRGFRQ_SHIFT                (12U)
#define PWM_SM2TCTRL_TRGFRQ(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2TCTRL_TRGFRQ_SHIFT)) & PWM_SM2TCTRL_TRGFRQ_MASK)
/*! PWBOT1 - Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port.
 *  0b1..Route the PWMB output to the PWM_OUT_TRIG1 port.
 */
#define PWM_SM2TCTRL_PWBOT1_MASK                 (0x4000U)
#define PWM_SM2TCTRL_PWBOT1_SHIFT                (14U)
#define PWM_SM2TCTRL_PWBOT1(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2TCTRL_PWBOT1_SHIFT)) & PWM_SM2TCTRL_PWBOT1_MASK)
/*! PWAOT0 - Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port.
 *  0b1..Route the PWMA output to the PWM_OUT_TRIG0 port.
 */
#define PWM_SM2TCTRL_PWAOT0_MASK                 (0x8000U)
#define PWM_SM2TCTRL_PWAOT0_SHIFT                (15U)
#define PWM_SM2TCTRL_PWAOT0(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM2TCTRL_PWAOT0_SHIFT)) & PWM_SM2TCTRL_PWAOT0_MASK)
/*! @} */

/*! @name SM2DISMAP0 - Fault Disable Mapping Register 0 */
/*! @{ */
#define PWM_SM2DISMAP0_DIS0A_MASK                (0xFU)
#define PWM_SM2DISMAP0_DIS0A_SHIFT               (0U)
#define PWM_SM2DISMAP0_DIS0A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP0_DIS0A_SHIFT)) & PWM_SM2DISMAP0_DIS0A_MASK)
#define PWM_SM2DISMAP0_DIS0B_MASK                (0xF0U)
#define PWM_SM2DISMAP0_DIS0B_SHIFT               (4U)
#define PWM_SM2DISMAP0_DIS0B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP0_DIS0B_SHIFT)) & PWM_SM2DISMAP0_DIS0B_MASK)
#define PWM_SM2DISMAP0_DIS0X_MASK                (0xF00U)
#define PWM_SM2DISMAP0_DIS0X_SHIFT               (8U)
#define PWM_SM2DISMAP0_DIS0X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP0_DIS0X_SHIFT)) & PWM_SM2DISMAP0_DIS0X_MASK)
/*! @} */

/*! @name SM2DISMAP1 - Fault Disable Mapping Register 1 */
/*! @{ */
#define PWM_SM2DISMAP1_DIS1A_MASK                (0xFU)
#define PWM_SM2DISMAP1_DIS1A_SHIFT               (0U)
#define PWM_SM2DISMAP1_DIS1A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP1_DIS1A_SHIFT)) & PWM_SM2DISMAP1_DIS1A_MASK)
#define PWM_SM2DISMAP1_DIS1B_MASK                (0xF0U)
#define PWM_SM2DISMAP1_DIS1B_SHIFT               (4U)
#define PWM_SM2DISMAP1_DIS1B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP1_DIS1B_SHIFT)) & PWM_SM2DISMAP1_DIS1B_MASK)
#define PWM_SM2DISMAP1_DIS1X_MASK                (0xF00U)
#define PWM_SM2DISMAP1_DIS1X_SHIFT               (8U)
#define PWM_SM2DISMAP1_DIS1X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DISMAP1_DIS1X_SHIFT)) & PWM_SM2DISMAP1_DIS1X_MASK)
/*! @} */

/*! @name SM2DTCNT0 - Deadtime Count Register 0 */
/*! @{ */
#define PWM_SM2DTCNT0_DTCNT0_MASK                (0xFFFFU)
#define PWM_SM2DTCNT0_DTCNT0_SHIFT               (0U)
#define PWM_SM2DTCNT0_DTCNT0(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DTCNT0_DTCNT0_SHIFT)) & PWM_SM2DTCNT0_DTCNT0_MASK)
/*! @} */

/*! @name SM2DTCNT1 - Deadtime Count Register 1 */
/*! @{ */
#define PWM_SM2DTCNT1_DTCNT1_MASK                (0xFFFFU)
#define PWM_SM2DTCNT1_DTCNT1_SHIFT               (0U)
#define PWM_SM2DTCNT1_DTCNT1(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM2DTCNT1_DTCNT1_SHIFT)) & PWM_SM2DTCNT1_DTCNT1_MASK)
/*! @} */

/*! @name SM2CAPTCTRLA - Capture Control A Register */
/*! @{ */
/*! ARMA - Arm A
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLA[EDGAx] is enabled.
 */
#define PWM_SM2CAPTCTRLA_ARMA_MASK               (0x1U)
#define PWM_SM2CAPTCTRLA_ARMA_SHIFT              (0U)
#define PWM_SM2CAPTCTRLA_ARMA(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_ARMA_SHIFT)) & PWM_SM2CAPTCTRLA_ARMA_MASK)
/*! ONESHOTA - One Shot Mode A
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLA[ARMA] is cleared. No
 *       further captures will be performed until CAPTCTRLA[ARMA] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLA[ARMA] is then cleared.
 */
#define PWM_SM2CAPTCTRLA_ONESHOTA_MASK           (0x2U)
#define PWM_SM2CAPTCTRLA_ONESHOTA_SHIFT          (1U)
#define PWM_SM2CAPTCTRLA_ONESHOTA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_ONESHOTA_SHIFT)) & PWM_SM2CAPTCTRLA_ONESHOTA_MASK)
/*! EDGA0 - Edge A 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLA_EDGA0_MASK              (0xCU)
#define PWM_SM2CAPTCTRLA_EDGA0_SHIFT             (2U)
#define PWM_SM2CAPTCTRLA_EDGA0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_EDGA0_SHIFT)) & PWM_SM2CAPTCTRLA_EDGA0_MASK)
/*! EDGA1 - Edge A 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLA_EDGA1_MASK              (0x30U)
#define PWM_SM2CAPTCTRLA_EDGA1_SHIFT             (4U)
#define PWM_SM2CAPTCTRLA_EDGA1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_EDGA1_SHIFT)) & PWM_SM2CAPTCTRLA_EDGA1_MASK)
/*! INP_SELA - Input Select A
 *  0b0..Raw PWM_A input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLA[EDGA0] and
 *       CAPTCTRLA[EDGA1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRA[EDGA0] and/or CAPTCTRLA[EDGA1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM2CAPTCTRLA_INP_SELA_MASK           (0x40U)
#define PWM_SM2CAPTCTRLA_INP_SELA_SHIFT          (6U)
#define PWM_SM2CAPTCTRLA_INP_SELA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_INP_SELA_SHIFT)) & PWM_SM2CAPTCTRLA_INP_SELA_MASK)
/*! EDGCNTA_EN - Edge Counter A Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM2CAPTCTRLA_EDGCNTA_EN_MASK         (0x80U)
#define PWM_SM2CAPTCTRLA_EDGCNTA_EN_SHIFT        (7U)
#define PWM_SM2CAPTCTRLA_EDGCNTA_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_EDGCNTA_EN_SHIFT)) & PWM_SM2CAPTCTRLA_EDGCNTA_EN_MASK)
#define PWM_SM2CAPTCTRLA_CFAWM_MASK              (0x300U)
#define PWM_SM2CAPTCTRLA_CFAWM_SHIFT             (8U)
#define PWM_SM2CAPTCTRLA_CFAWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_CFAWM_SHIFT)) & PWM_SM2CAPTCTRLA_CFAWM_MASK)
#define PWM_SM2CAPTCTRLA_CA0CNT_MASK             (0x1C00U)
#define PWM_SM2CAPTCTRLA_CA0CNT_SHIFT            (10U)
#define PWM_SM2CAPTCTRLA_CA0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_CA0CNT_SHIFT)) & PWM_SM2CAPTCTRLA_CA0CNT_MASK)
#define PWM_SM2CAPTCTRLA_CA1CNT_MASK             (0xE000U)
#define PWM_SM2CAPTCTRLA_CA1CNT_SHIFT            (13U)
#define PWM_SM2CAPTCTRLA_CA1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLA_CA1CNT_SHIFT)) & PWM_SM2CAPTCTRLA_CA1CNT_MASK)
/*! @} */

/*! @name SM2CAPTCOMPA - Capture Compare A Register */
/*! @{ */
#define PWM_SM2CAPTCOMPA_EDGCMPA_MASK            (0xFFU)
#define PWM_SM2CAPTCOMPA_EDGCMPA_SHIFT           (0U)
#define PWM_SM2CAPTCOMPA_EDGCMPA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPA_EDGCMPA_SHIFT)) & PWM_SM2CAPTCOMPA_EDGCMPA_MASK)
#define PWM_SM2CAPTCOMPA_EDGCNTA_MASK            (0xFF00U)
#define PWM_SM2CAPTCOMPA_EDGCNTA_SHIFT           (8U)
#define PWM_SM2CAPTCOMPA_EDGCNTA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPA_EDGCNTA_SHIFT)) & PWM_SM2CAPTCOMPA_EDGCNTA_MASK)
/*! @} */

/*! @name SM2CAPTCTRLB - Capture Control B Register */
/*! @{ */
/*! ARMB - Arm B
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLB[EDGBx] is enabled.
 */
#define PWM_SM2CAPTCTRLB_ARMB_MASK               (0x1U)
#define PWM_SM2CAPTCTRLB_ARMB_SHIFT              (0U)
#define PWM_SM2CAPTCTRLB_ARMB(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_ARMB_SHIFT)) & PWM_SM2CAPTCTRLB_ARMB_MASK)
/*! ONESHOTB - One Shot Mode B
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLB[ARMB] is cleared. No
 *       further captures will be performed until CAPTCTRLB[ARMB] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLB[ARMB] is then cleared.
 */
#define PWM_SM2CAPTCTRLB_ONESHOTB_MASK           (0x2U)
#define PWM_SM2CAPTCTRLB_ONESHOTB_SHIFT          (1U)
#define PWM_SM2CAPTCTRLB_ONESHOTB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_ONESHOTB_SHIFT)) & PWM_SM2CAPTCTRLB_ONESHOTB_MASK)
/*! EDGB0 - Edge B 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLB_EDGB0_MASK              (0xCU)
#define PWM_SM2CAPTCTRLB_EDGB0_SHIFT             (2U)
#define PWM_SM2CAPTCTRLB_EDGB0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_EDGB0_SHIFT)) & PWM_SM2CAPTCTRLB_EDGB0_MASK)
/*! EDGB1 - Edge B 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLB_EDGB1_MASK              (0x30U)
#define PWM_SM2CAPTCTRLB_EDGB1_SHIFT             (4U)
#define PWM_SM2CAPTCTRLB_EDGB1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_EDGB1_SHIFT)) & PWM_SM2CAPTCTRLB_EDGB1_MASK)
/*! INP_SELB - Input Select B
 *  0b0..Raw PWM_B input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLB[EDGB0] and
 *       CAPTCTRLB[EDGB1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRB[EDGB0] and/or CAPTCTRLB[EDGB1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM2CAPTCTRLB_INP_SELB_MASK           (0x40U)
#define PWM_SM2CAPTCTRLB_INP_SELB_SHIFT          (6U)
#define PWM_SM2CAPTCTRLB_INP_SELB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_INP_SELB_SHIFT)) & PWM_SM2CAPTCTRLB_INP_SELB_MASK)
/*! EDGCNTB_EN - Edge Counter B Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM2CAPTCTRLB_EDGCNTB_EN_MASK         (0x80U)
#define PWM_SM2CAPTCTRLB_EDGCNTB_EN_SHIFT        (7U)
#define PWM_SM2CAPTCTRLB_EDGCNTB_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_EDGCNTB_EN_SHIFT)) & PWM_SM2CAPTCTRLB_EDGCNTB_EN_MASK)
#define PWM_SM2CAPTCTRLB_CFBWM_MASK              (0x300U)
#define PWM_SM2CAPTCTRLB_CFBWM_SHIFT             (8U)
#define PWM_SM2CAPTCTRLB_CFBWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_CFBWM_SHIFT)) & PWM_SM2CAPTCTRLB_CFBWM_MASK)
#define PWM_SM2CAPTCTRLB_CB0CNT_MASK             (0x1C00U)
#define PWM_SM2CAPTCTRLB_CB0CNT_SHIFT            (10U)
#define PWM_SM2CAPTCTRLB_CB0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_CB0CNT_SHIFT)) & PWM_SM2CAPTCTRLB_CB0CNT_MASK)
#define PWM_SM2CAPTCTRLB_CB1CNT_MASK             (0xE000U)
#define PWM_SM2CAPTCTRLB_CB1CNT_SHIFT            (13U)
#define PWM_SM2CAPTCTRLB_CB1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLB_CB1CNT_SHIFT)) & PWM_SM2CAPTCTRLB_CB1CNT_MASK)
/*! @} */

/*! @name SM2CAPTCOMPB - Capture Compare B Register */
/*! @{ */
#define PWM_SM2CAPTCOMPB_EDGCMPB_MASK            (0xFFU)
#define PWM_SM2CAPTCOMPB_EDGCMPB_SHIFT           (0U)
#define PWM_SM2CAPTCOMPB_EDGCMPB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPB_EDGCMPB_SHIFT)) & PWM_SM2CAPTCOMPB_EDGCMPB_MASK)
#define PWM_SM2CAPTCOMPB_EDGCNTB_MASK            (0xFF00U)
#define PWM_SM2CAPTCOMPB_EDGCNTB_SHIFT           (8U)
#define PWM_SM2CAPTCOMPB_EDGCNTB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPB_EDGCNTB_SHIFT)) & PWM_SM2CAPTCOMPB_EDGCNTB_MASK)
/*! @} */

/*! @name SM2CAPTCTRLX - Capture Control X Register */
/*! @{ */
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_SM2CAPTCTRLX_ARMX_MASK               (0x1U)
#define PWM_SM2CAPTCTRLX_ARMX_SHIFT              (0U)
#define PWM_SM2CAPTCTRLX_ARMX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_ARMX_SHIFT)) & PWM_SM2CAPTCTRLX_ARMX_MASK)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The
 *       process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and the ARMX bit is cleared. No further
 *       captures will be performed until the ARMX bit is set again.If only one of the capture circuits is enabled,
 *       then a single capture will occur on the enabled capture circuit and the ARMX bit is then cleared.
 */
#define PWM_SM2CAPTCTRLX_ONESHOTX_MASK           (0x2U)
#define PWM_SM2CAPTCTRLX_ONESHOTX_SHIFT          (1U)
#define PWM_SM2CAPTCTRLX_ONESHOTX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_SM2CAPTCTRLX_ONESHOTX_MASK)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLX_EDGX0_MASK              (0xCU)
#define PWM_SM2CAPTCTRLX_EDGX0_SHIFT             (2U)
#define PWM_SM2CAPTCTRLX_EDGX0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_EDGX0_SHIFT)) & PWM_SM2CAPTCTRLX_EDGX0_MASK)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM2CAPTCTRLX_EDGX1_MASK              (0x30U)
#define PWM_SM2CAPTCTRLX_EDGX1_SHIFT             (4U)
#define PWM_SM2CAPTCTRLX_EDGX1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_EDGX1_SHIFT)) & PWM_SM2CAPTCTRLX_EDGX1_MASK)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLX[EDGX0] and
 *       CAPTCTRLX[EDGX1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRX[EDGX0] and/or CAPTCTRLX[EDGX1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM2CAPTCTRLX_INP_SELX_MASK           (0x40U)
#define PWM_SM2CAPTCTRLX_INP_SELX_SHIFT          (6U)
#define PWM_SM2CAPTCTRLX_INP_SELX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_INP_SELX_SHIFT)) & PWM_SM2CAPTCTRLX_INP_SELX_MASK)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM2CAPTCTRLX_EDGCNTX_EN_MASK         (0x80U)
#define PWM_SM2CAPTCTRLX_EDGCNTX_EN_SHIFT        (7U)
#define PWM_SM2CAPTCTRLX_EDGCNTX_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_SM2CAPTCTRLX_EDGCNTX_EN_MASK)
#define PWM_SM2CAPTCTRLX_CFXWM_MASK              (0x300U)
#define PWM_SM2CAPTCTRLX_CFXWM_SHIFT             (8U)
#define PWM_SM2CAPTCTRLX_CFXWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_CFXWM_SHIFT)) & PWM_SM2CAPTCTRLX_CFXWM_MASK)
#define PWM_SM2CAPTCTRLX_CX0CNT_MASK             (0x1C00U)
#define PWM_SM2CAPTCTRLX_CX0CNT_SHIFT            (10U)
#define PWM_SM2CAPTCTRLX_CX0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_CX0CNT_SHIFT)) & PWM_SM2CAPTCTRLX_CX0CNT_MASK)
#define PWM_SM2CAPTCTRLX_CX1CNT_MASK             (0xE000U)
#define PWM_SM2CAPTCTRLX_CX1CNT_SHIFT            (13U)
#define PWM_SM2CAPTCTRLX_CX1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCTRLX_CX1CNT_SHIFT)) & PWM_SM2CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/*! @name SM2CAPTCOMPX - Capture Compare X Register */
/*! @{ */
#define PWM_SM2CAPTCOMPX_EDGCMPX_MASK            (0xFFU)
#define PWM_SM2CAPTCOMPX_EDGCMPX_SHIFT           (0U)
#define PWM_SM2CAPTCOMPX_EDGCMPX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_SM2CAPTCOMPX_EDGCMPX_MASK)
#define PWM_SM2CAPTCOMPX_EDGCNTX_MASK            (0xFF00U)
#define PWM_SM2CAPTCOMPX_EDGCNTX_SHIFT           (8U)
#define PWM_SM2CAPTCOMPX_EDGCNTX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_SM2CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/*! @name SM2CVAL0 - Capture Value 0 Register */
/*! @{ */
#define PWM_SM2CVAL0_CAPTVAL0_MASK               (0xFFFFU)
#define PWM_SM2CVAL0_CAPTVAL0_SHIFT              (0U)
#define PWM_SM2CVAL0_CAPTVAL0(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL0_CAPTVAL0_SHIFT)) & PWM_SM2CVAL0_CAPTVAL0_MASK)
/*! @} */

/*! @name SM2CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL0CYC_CVAL0CYC_MASK            (0xFU)
#define PWM_SM2CVAL0CYC_CVAL0CYC_SHIFT           (0U)
#define PWM_SM2CVAL0CYC_CVAL0CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_SM2CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/*! @name SM2CVAL1 - Capture Value 1 Register */
/*! @{ */
#define PWM_SM2CVAL1_CAPTVAL1_MASK               (0xFFFFU)
#define PWM_SM2CVAL1_CAPTVAL1_SHIFT              (0U)
#define PWM_SM2CVAL1_CAPTVAL1(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL1_CAPTVAL1_SHIFT)) & PWM_SM2CVAL1_CAPTVAL1_MASK)
/*! @} */

/*! @name SM2CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL1CYC_CVAL1CYC_MASK            (0xFU)
#define PWM_SM2CVAL1CYC_CVAL1CYC_SHIFT           (0U)
#define PWM_SM2CVAL1CYC_CVAL1CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_SM2CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/*! @name SM2CVAL2 - Capture Value 2 Register */
/*! @{ */
#define PWM_SM2CVAL2_CAPTVAL2_MASK               (0xFFFFU)
#define PWM_SM2CVAL2_CAPTVAL2_SHIFT              (0U)
#define PWM_SM2CVAL2_CAPTVAL2(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL2_CAPTVAL2_SHIFT)) & PWM_SM2CVAL2_CAPTVAL2_MASK)
/*! @} */

/*! @name SM2CVAL2CYC - Capture Value 2 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL2CYC_CVAL2CYC_MASK            (0xFU)
#define PWM_SM2CVAL2CYC_CVAL2CYC_SHIFT           (0U)
#define PWM_SM2CVAL2CYC_CVAL2CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL2CYC_CVAL2CYC_SHIFT)) & PWM_SM2CVAL2CYC_CVAL2CYC_MASK)
/*! @} */

/*! @name SM2CVAL3 - Capture Value 3 Register */
/*! @{ */
#define PWM_SM2CVAL3_CAPTVAL3_MASK               (0xFFFFU)
#define PWM_SM2CVAL3_CAPTVAL3_SHIFT              (0U)
#define PWM_SM2CVAL3_CAPTVAL3(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL3_CAPTVAL3_SHIFT)) & PWM_SM2CVAL3_CAPTVAL3_MASK)
/*! @} */

/*! @name SM2CVAL3CYC - Capture Value 3 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL3CYC_CVAL3CYC_MASK            (0xFU)
#define PWM_SM2CVAL3CYC_CVAL3CYC_SHIFT           (0U)
#define PWM_SM2CVAL3CYC_CVAL3CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL3CYC_CVAL3CYC_SHIFT)) & PWM_SM2CVAL3CYC_CVAL3CYC_MASK)
/*! @} */

/*! @name SM2CVAL4 - Capture Value 4 Register */
/*! @{ */
#define PWM_SM2CVAL4_CAPTVAL4_MASK               (0xFFFFU)
#define PWM_SM2CVAL4_CAPTVAL4_SHIFT              (0U)
#define PWM_SM2CVAL4_CAPTVAL4(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL4_CAPTVAL4_SHIFT)) & PWM_SM2CVAL4_CAPTVAL4_MASK)
/*! @} */

/*! @name SM2CVAL4CYC - Capture Value 4 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL4CYC_CVAL4CYC_MASK            (0xFU)
#define PWM_SM2CVAL4CYC_CVAL4CYC_SHIFT           (0U)
#define PWM_SM2CVAL4CYC_CVAL4CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL4CYC_CVAL4CYC_SHIFT)) & PWM_SM2CVAL4CYC_CVAL4CYC_MASK)
/*! @} */

/*! @name SM2CVAL5 - Capture Value 5 Register */
/*! @{ */
#define PWM_SM2CVAL5_CAPTVAL5_MASK               (0xFFFFU)
#define PWM_SM2CVAL5_CAPTVAL5_SHIFT              (0U)
#define PWM_SM2CVAL5_CAPTVAL5(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL5_CAPTVAL5_SHIFT)) & PWM_SM2CVAL5_CAPTVAL5_MASK)
/*! @} */

/*! @name SM2CVAL5CYC - Capture Value 5 Cycle Register */
/*! @{ */
#define PWM_SM2CVAL5CYC_CVAL5CYC_MASK            (0xFU)
#define PWM_SM2CVAL5CYC_CVAL5CYC_SHIFT           (0U)
#define PWM_SM2CVAL5CYC_CVAL5CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2CVAL5CYC_CVAL5CYC_SHIFT)) & PWM_SM2CVAL5CYC_CVAL5CYC_MASK)
/*! @} */

/*! @name SM2PHASEDLY - Phase Delay Register */
/*! @{ */
#define PWM_SM2PHASEDLY_PHASEDLY_MASK            (0xFFFFU)
#define PWM_SM2PHASEDLY_PHASEDLY_SHIFT           (0U)
#define PWM_SM2PHASEDLY_PHASEDLY(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM2PHASEDLY_PHASEDLY_SHIFT)) & PWM_SM2PHASEDLY_PHASEDLY_MASK)
/*! @} */

/*! @name SM3CNT - Counter Register */
/*! @{ */
#define PWM_SM3CNT_CNT_MASK                      (0xFFFFU)
#define PWM_SM3CNT_CNT_SHIFT                     (0U)
#define PWM_SM3CNT_CNT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM3CNT_CNT_SHIFT)) & PWM_SM3CNT_CNT_MASK)
/*! @} */

/*! @name SM3INIT - Initial Count Register */
/*! @{ */
#define PWM_SM3INIT_INIT_MASK                    (0xFFFFU)
#define PWM_SM3INIT_INIT_SHIFT                   (0U)
#define PWM_SM3INIT_INIT(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3INIT_INIT_SHIFT)) & PWM_SM3INIT_INIT_MASK)
/*! @} */

/*! @name SM3CTRL2 - Control 2 Register */
/*! @{ */
/*! CLK_SEL - Clock Source Select
 *  0b00..The IPBus clock is used as the clock for the local prescaler and counter.
 *  0b01..EXT_CLK is used as the clock for the local prescaler and counter.
 *  0b10..Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This
 *        setting should not be used in submodule 0 as it will force the clock to logic 0.
 *  0b11..reserved
 */
#define PWM_SM3CTRL2_CLK_SEL_MASK                (0x3U)
#define PWM_SM3CTRL2_CLK_SEL_SHIFT               (0U)
#define PWM_SM3CTRL2_CLK_SEL(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_CLK_SEL_SHIFT)) & PWM_SM3CTRL2_CLK_SEL_MASK)
/*! RELOAD_SEL - Reload Source Select
 *  0b0..The local RELOAD signal is used to reload registers.
 *  0b1..The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used
 *       in submodule 0 as it will force the RELOAD signal to logic 0.
 */
#define PWM_SM3CTRL2_RELOAD_SEL_MASK             (0x4U)
#define PWM_SM3CTRL2_RELOAD_SEL_SHIFT            (2U)
#define PWM_SM3CTRL2_RELOAD_SEL(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_RELOAD_SEL_SHIFT)) & PWM_SM3CTRL2_RELOAD_SEL_MASK)
/*! FORCE_SEL - This read/write bit determines the source of the FORCE OUTPUT signal for this submodule.
 *  0b000..The local force signal, CTRL2[FORCE], from this submodule is used to force updates.
 *  0b001..The master force signal from submodule 0 is used to force updates. This setting should not be used in
 *         submodule 0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b010..The local reload signal from this submodule is used to force updates without regard to the state of LDOK.
 *  0b011..The master reload signal from submodule0 is used to force updates if LDOK is set. This setting should
 *         not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b100..The local sync signal from this submodule is used to force updates.
 *  0b101..The master sync signal from submodule0 is used to force updates. This setting should not be used in
 *         submodule0 as it will hold the FORCE OUTPUT signal to logic 0.
 *  0b110..The external force signal, EXT_FORCE, from outside the PWM module causes updates.
 *  0b111..The external sync signal, EXT_SYNC, from outside the PWM module causes updates.
 */
#define PWM_SM3CTRL2_FORCE_SEL_MASK              (0x38U)
#define PWM_SM3CTRL2_FORCE_SEL_SHIFT             (3U)
#define PWM_SM3CTRL2_FORCE_SEL(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_FORCE_SEL_SHIFT)) & PWM_SM3CTRL2_FORCE_SEL_MASK)
#define PWM_SM3CTRL2_FORCE_MASK                  (0x40U)
#define PWM_SM3CTRL2_FORCE_SHIFT                 (6U)
#define PWM_SM3CTRL2_FORCE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_FORCE_SHIFT)) & PWM_SM3CTRL2_FORCE_MASK)
/*! FRCEN - FRCEN
 *  0b0..Initialization from a FORCE_OUT is disabled.
 *  0b1..Initialization from a FORCE_OUT is enabled.
 */
#define PWM_SM3CTRL2_FRCEN_MASK                  (0x80U)
#define PWM_SM3CTRL2_FRCEN_SHIFT                 (7U)
#define PWM_SM3CTRL2_FRCEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_FRCEN_SHIFT)) & PWM_SM3CTRL2_FRCEN_MASK)
/*! INIT_SEL - Initialization Control Select
 *  0b00..Local sync (PWM_X) causes initialization.
 *  0b01..Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as
 *        it will force the INIT signal to logic 0. The submodule counter will only reinitialize when a master
 *        reload occurs.
 *  0b10..Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it
 *        will force the INIT signal to logic 0.
 *  0b11..EXT_SYNC causes initialization.
 */
#define PWM_SM3CTRL2_INIT_SEL_MASK               (0x300U)
#define PWM_SM3CTRL2_INIT_SEL_SHIFT              (8U)
#define PWM_SM3CTRL2_INIT_SEL(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_INIT_SEL_SHIFT)) & PWM_SM3CTRL2_INIT_SEL_MASK)
#define PWM_SM3CTRL2_PWMX_INIT_MASK              (0x400U)
#define PWM_SM3CTRL2_PWMX_INIT_SHIFT             (10U)
#define PWM_SM3CTRL2_PWMX_INIT(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_PWMX_INIT_SHIFT)) & PWM_SM3CTRL2_PWMX_INIT_MASK)
#define PWM_SM3CTRL2_PWM45_INIT_MASK             (0x800U)
#define PWM_SM3CTRL2_PWM45_INIT_SHIFT            (11U)
#define PWM_SM3CTRL2_PWM45_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_PWM45_INIT_SHIFT)) & PWM_SM3CTRL2_PWM45_INIT_MASK)
#define PWM_SM3CTRL2_PWM23_INIT_MASK             (0x1000U)
#define PWM_SM3CTRL2_PWM23_INIT_SHIFT            (12U)
#define PWM_SM3CTRL2_PWM23_INIT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_PWM23_INIT_SHIFT)) & PWM_SM3CTRL2_PWM23_INIT_MASK)
/*! INDEP - Independent or Complementary Pair Operation
 *  0b0..PWM_A and PWM_B form a complementary PWM pair.
 *  0b1..PWM_A and PWM_B outputs are independent PWMs.
 */
#define PWM_SM3CTRL2_INDEP_MASK                  (0x2000U)
#define PWM_SM3CTRL2_INDEP_SHIFT                 (13U)
#define PWM_SM3CTRL2_INDEP(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_INDEP_SHIFT)) & PWM_SM3CTRL2_INDEP_MASK)
#define PWM_SM3CTRL2_WAITEN_MASK                 (0x4000U)
#define PWM_SM3CTRL2_WAITEN_SHIFT                (14U)
#define PWM_SM3CTRL2_WAITEN(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_WAITEN_SHIFT)) & PWM_SM3CTRL2_WAITEN_MASK)
#define PWM_SM3CTRL2_DBGEN_MASK                  (0x8000U)
#define PWM_SM3CTRL2_DBGEN_SHIFT                 (15U)
#define PWM_SM3CTRL2_DBGEN(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL2_DBGEN_SHIFT)) & PWM_SM3CTRL2_DBGEN_MASK)
/*! @} */

/*! @name SM3CTRL - Control Register */
/*! @{ */
/*! DBLEN - Double Switching Enable
 *  0b0..Double switching disabled.
 *  0b1..Double switching enabled.
 */
#define PWM_SM3CTRL_DBLEN_MASK                   (0x1U)
#define PWM_SM3CTRL_DBLEN_SHIFT                  (0U)
#define PWM_SM3CTRL_DBLEN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_DBLEN_SHIFT)) & PWM_SM3CTRL_DBLEN_MASK)
/*! DBLX - PWMX Double Switching Enable
 *  0b0..PWMX double pulse disabled.
 *  0b1..PWMX double pulse enabled.
 */
#define PWM_SM3CTRL_DBLX_MASK                    (0x2U)
#define PWM_SM3CTRL_DBLX_SHIFT                   (1U)
#define PWM_SM3CTRL_DBLX(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_DBLX_SHIFT)) & PWM_SM3CTRL_DBLX_MASK)
/*! LDMOD - Load Mode Select
 *  0b0..Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set.
 *  0b1..Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set.
 *       In this case it is not necessary to set CTRL[FULL] or CTRL[HALF].
 */
#define PWM_SM3CTRL_LDMOD_MASK                   (0x4U)
#define PWM_SM3CTRL_LDMOD_SHIFT                  (2U)
#define PWM_SM3CTRL_LDMOD(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_LDMOD_SHIFT)) & PWM_SM3CTRL_LDMOD_MASK)
/*! SPLIT - Split the DBLPWM signal to PWMA and PWMB
 *  0b0..DBLPWM is not split. PWMA and PWMB each have double pulses.
 *  0b1..DBLPWM is split to PWMA and PWMB.
 */
#define PWM_SM3CTRL_SPLIT_MASK                   (0x8U)
#define PWM_SM3CTRL_SPLIT_SHIFT                  (3U)
#define PWM_SM3CTRL_SPLIT(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_SPLIT_SHIFT)) & PWM_SM3CTRL_SPLIT_MASK)
/*! PRSC - Prescaler
 *  0b000..PWM clock frequency = fclk
 *  0b001..PWM clock frequency = fclk/2
 *  0b010..PWM clock frequency = fclk/4
 *  0b011..PWM clock frequency = fclk/8
 *  0b100..PWM clock frequency = fclk/16
 *  0b101..PWM clock frequency = fclk/32
 *  0b110..PWM clock frequency = fclk/64
 *  0b111..PWM clock frequency = fclk/128
 */
#define PWM_SM3CTRL_PRSC_MASK                    (0x70U)
#define PWM_SM3CTRL_PRSC_SHIFT                   (4U)
#define PWM_SM3CTRL_PRSC(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_PRSC_SHIFT)) & PWM_SM3CTRL_PRSC_MASK)
/*! COMPMODE - Compare Mode
 *  0b0..The VAL* registers and the PWM counter are compared using an "equal to" method. This means that PWM edges
 *       are only produced when the counter is equal to one of the VAL* register values. This implies that a PWMA
 *       output that is high at the end of a period will maintain this state until a match with VAL3 clears the
 *       output in the following period.
 *  0b1..The VAL* registers and the PWM counter are compared using an "equal to or greater than" method. This
 *       means that PWM edges are produced when the counter is equal to or greater than one of the VAL* register
 *       values. This implies that a PWMA output that is high at the end of a period could go low at the start of the
 *       next period if the starting counter value is greater than (but not necessarily equal to) the new VAL3 value.
 */
#define PWM_SM3CTRL_COMPMODE_MASK                (0x80U)
#define PWM_SM3CTRL_COMPMODE_SHIFT               (7U)
#define PWM_SM3CTRL_COMPMODE(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_COMPMODE_SHIFT)) & PWM_SM3CTRL_COMPMODE_MASK)
#define PWM_SM3CTRL_DT_MASK                      (0x300U)
#define PWM_SM3CTRL_DT_SHIFT                     (8U)
#define PWM_SM3CTRL_DT(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_DT_SHIFT)) & PWM_SM3CTRL_DT_MASK)
/*! FULL - Full Cycle Reload
 *  0b0..Full-cycle reloads disabled.
 *  0b1..Full-cycle reloads enabled.
 */
#define PWM_SM3CTRL_FULL_MASK                    (0x400U)
#define PWM_SM3CTRL_FULL_SHIFT                   (10U)
#define PWM_SM3CTRL_FULL(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_FULL_SHIFT)) & PWM_SM3CTRL_FULL_MASK)
/*! HALF - Half Cycle Reload
 *  0b0..Half-cycle reloads disabled.
 *  0b1..Half-cycle reloads enabled.
 */
#define PWM_SM3CTRL_HALF_MASK                    (0x800U)
#define PWM_SM3CTRL_HALF_SHIFT                   (11U)
#define PWM_SM3CTRL_HALF(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_HALF_SHIFT)) & PWM_SM3CTRL_HALF_MASK)
/*! LDFQ - Load Frequency
 *  0b0000..Every PWM opportunity
 *  0b0001..Every 2 PWM opportunities
 *  0b0010..Every 3 PWM opportunities
 *  0b0011..Every 4 PWM opportunities
 *  0b0100..Every 5 PWM opportunities
 *  0b0101..Every 6 PWM opportunities
 *  0b0110..Every 7 PWM opportunities
 *  0b0111..Every 8 PWM opportunities
 *  0b1000..Every 9 PWM opportunities
 *  0b1001..Every 10 PWM opportunities
 *  0b1010..Every 11 PWM opportunities
 *  0b1011..Every 12 PWM opportunities
 *  0b1100..Every 13 PWM opportunities
 *  0b1101..Every 14 PWM opportunities
 *  0b1110..Every 15 PWM opportunities
 *  0b1111..Every 16 PWM opportunities
 */
#define PWM_SM3CTRL_LDFQ_MASK                    (0xF000U)
#define PWM_SM3CTRL_LDFQ_SHIFT                   (12U)
#define PWM_SM3CTRL_LDFQ(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3CTRL_LDFQ_SHIFT)) & PWM_SM3CTRL_LDFQ_MASK)
/*! @} */

/*! @name SM3VAL0 - Value Register 0 */
/*! @{ */
#define PWM_SM3VAL0_VAL0_MASK                    (0xFFFFU)
#define PWM_SM3VAL0_VAL0_SHIFT                   (0U)
#define PWM_SM3VAL0_VAL0(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL0_VAL0_SHIFT)) & PWM_SM3VAL0_VAL0_MASK)
/*! @} */

/*! @name SM3FRACVAL1 - Fractional Value Register 1 */
/*! @{ */
#define PWM_SM3FRACVAL1_FRACVAL1_MASK            (0xF800U)
#define PWM_SM3FRACVAL1_FRACVAL1_SHIFT           (11U)
#define PWM_SM3FRACVAL1_FRACVAL1(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRACVAL1_FRACVAL1_SHIFT)) & PWM_SM3FRACVAL1_FRACVAL1_MASK)
/*! @} */

/*! @name SM3VAL1 - Value Register 1 */
/*! @{ */
#define PWM_SM3VAL1_VAL1_MASK                    (0xFFFFU)
#define PWM_SM3VAL1_VAL1_SHIFT                   (0U)
#define PWM_SM3VAL1_VAL1(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL1_VAL1_SHIFT)) & PWM_SM3VAL1_VAL1_MASK)
/*! @} */

/*! @name SM3FRACVAL2 - Fractional Value Register 2 */
/*! @{ */
#define PWM_SM3FRACVAL2_FRACVAL2_MASK            (0xF800U)
#define PWM_SM3FRACVAL2_FRACVAL2_SHIFT           (11U)
#define PWM_SM3FRACVAL2_FRACVAL2(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRACVAL2_FRACVAL2_SHIFT)) & PWM_SM3FRACVAL2_FRACVAL2_MASK)
/*! @} */

/*! @name SM3VAL2 - Value Register 2 */
/*! @{ */
#define PWM_SM3VAL2_VAL2_MASK                    (0xFFFFU)
#define PWM_SM3VAL2_VAL2_SHIFT                   (0U)
#define PWM_SM3VAL2_VAL2(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL2_VAL2_SHIFT)) & PWM_SM3VAL2_VAL2_MASK)
/*! @} */

/*! @name SM3FRACVAL3 - Fractional Value Register 3 */
/*! @{ */
#define PWM_SM3FRACVAL3_FRACVAL3_MASK            (0xF800U)
#define PWM_SM3FRACVAL3_FRACVAL3_SHIFT           (11U)
#define PWM_SM3FRACVAL3_FRACVAL3(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRACVAL3_FRACVAL3_SHIFT)) & PWM_SM3FRACVAL3_FRACVAL3_MASK)
/*! @} */

/*! @name SM3VAL3 - Value Register 3 */
/*! @{ */
#define PWM_SM3VAL3_VAL3_MASK                    (0xFFFFU)
#define PWM_SM3VAL3_VAL3_SHIFT                   (0U)
#define PWM_SM3VAL3_VAL3(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL3_VAL3_SHIFT)) & PWM_SM3VAL3_VAL3_MASK)
/*! @} */

/*! @name SM3FRACVAL4 - Fractional Value Register 4 */
/*! @{ */
#define PWM_SM3FRACVAL4_FRACVAL4_MASK            (0xF800U)
#define PWM_SM3FRACVAL4_FRACVAL4_SHIFT           (11U)
#define PWM_SM3FRACVAL4_FRACVAL4(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRACVAL4_FRACVAL4_SHIFT)) & PWM_SM3FRACVAL4_FRACVAL4_MASK)
/*! @} */

/*! @name SM3VAL4 - Value Register 4 */
/*! @{ */
#define PWM_SM3VAL4_VAL4_MASK                    (0xFFFFU)
#define PWM_SM3VAL4_VAL4_SHIFT                   (0U)
#define PWM_SM3VAL4_VAL4(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL4_VAL4_SHIFT)) & PWM_SM3VAL4_VAL4_MASK)
/*! @} */

/*! @name SM3FRACVAL5 - Fractional Value Register 5 */
/*! @{ */
#define PWM_SM3FRACVAL5_FRACVAL5_MASK            (0xF800U)
#define PWM_SM3FRACVAL5_FRACVAL5_SHIFT           (11U)
#define PWM_SM3FRACVAL5_FRACVAL5(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRACVAL5_FRACVAL5_SHIFT)) & PWM_SM3FRACVAL5_FRACVAL5_MASK)
/*! @} */

/*! @name SM3VAL5 - Value Register 5 */
/*! @{ */
#define PWM_SM3VAL5_VAL5_MASK                    (0xFFFFU)
#define PWM_SM3VAL5_VAL5_SHIFT                   (0U)
#define PWM_SM3VAL5_VAL5(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3VAL5_VAL5_SHIFT)) & PWM_SM3VAL5_VAL5_MASK)
/*! @} */

/*! @name SM3FRCTRL - Fractional Control Register */
/*! @{ */
/*! FRAC1_EN - Fractional Cycle PWM Period Enable
 *  0b0..Disable fractional cycle length for the PWM period.
 *  0b1..Enable fractional cycle length for the PWM period.
 */
#define PWM_SM3FRCTRL_FRAC1_EN_MASK              (0x2U)
#define PWM_SM3FRCTRL_FRAC1_EN_SHIFT             (1U)
#define PWM_SM3FRCTRL_FRAC1_EN(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRCTRL_FRAC1_EN_SHIFT)) & PWM_SM3FRCTRL_FRAC1_EN_MASK)
/*! FRAC23_EN - Fractional Cycle Placement Enable for PWM_A
 *  0b0..Disable fractional cycle placement for PWM_A.
 *  0b1..Enable fractional cycle placement for PWM_A.
 */
#define PWM_SM3FRCTRL_FRAC23_EN_MASK             (0x4U)
#define PWM_SM3FRCTRL_FRAC23_EN_SHIFT            (2U)
#define PWM_SM3FRCTRL_FRAC23_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRCTRL_FRAC23_EN_SHIFT)) & PWM_SM3FRCTRL_FRAC23_EN_MASK)
/*! FRAC45_EN - Fractional Cycle Placement Enable for PWM_B
 *  0b0..Disable fractional cycle placement for PWM_B.
 *  0b1..Enable fractional cycle placement for PWM_B.
 */
#define PWM_SM3FRCTRL_FRAC45_EN_MASK             (0x10U)
#define PWM_SM3FRCTRL_FRAC45_EN_SHIFT            (4U)
#define PWM_SM3FRCTRL_FRAC45_EN(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRCTRL_FRAC45_EN_SHIFT)) & PWM_SM3FRCTRL_FRAC45_EN_MASK)
/*! FRAC_PU - Fractional Delay Circuit Power Up
 *  0b0..Turn off fractional delay logic.
 *  0b1..Power up fractional delay logic.
 */
#define PWM_SM3FRCTRL_FRAC_PU_MASK               (0x100U)
#define PWM_SM3FRCTRL_FRAC_PU_SHIFT              (8U)
#define PWM_SM3FRCTRL_FRAC_PU(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRCTRL_FRAC_PU_SHIFT)) & PWM_SM3FRCTRL_FRAC_PU_MASK)
#define PWM_SM3FRCTRL_TEST_MASK                  (0x8000U)
#define PWM_SM3FRCTRL_TEST_SHIFT                 (15U)
#define PWM_SM3FRCTRL_TEST(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3FRCTRL_TEST_SHIFT)) & PWM_SM3FRCTRL_TEST_MASK)
/*! @} */

/*! @name SM3OCTRL - Output Control Register */
/*! @{ */
/*! PWMXFS - PWM_X Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM3OCTRL_PWMXFS_MASK                 (0x3U)
#define PWM_SM3OCTRL_PWMXFS_SHIFT                (0U)
#define PWM_SM3OCTRL_PWMXFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMXFS_SHIFT)) & PWM_SM3OCTRL_PWMXFS_MASK)
/*! PWMBFS - PWM_B Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM3OCTRL_PWMBFS_MASK                 (0xCU)
#define PWM_SM3OCTRL_PWMBFS_SHIFT                (2U)
#define PWM_SM3OCTRL_PWMBFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMBFS_SHIFT)) & PWM_SM3OCTRL_PWMBFS_MASK)
/*! PWMAFS - PWM_A Fault State
 *  0b00..Output is forced to logic 0 state prior to consideration of output polarity control.
 *  0b01..Output is forced to logic 1 state prior to consideration of output polarity control.
 *  0b10..Output is tristated.
 *  0b11..Output is tristated.
 */
#define PWM_SM3OCTRL_PWMAFS_MASK                 (0x30U)
#define PWM_SM3OCTRL_PWMAFS_SHIFT                (4U)
#define PWM_SM3OCTRL_PWMAFS(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMAFS_SHIFT)) & PWM_SM3OCTRL_PWMAFS_MASK)
/*! POLX - PWM_X Output Polarity
 *  0b0..PWM_X output not inverted. A high level on the PWM_X pin represents the "on" or "active" state.
 *  0b1..PWM_X output inverted. A low level on the PWM_X pin represents the "on" or "active" state.
 */
#define PWM_SM3OCTRL_POLX_MASK                   (0x100U)
#define PWM_SM3OCTRL_POLX_SHIFT                  (8U)
#define PWM_SM3OCTRL_POLX(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_POLX_SHIFT)) & PWM_SM3OCTRL_POLX_MASK)
/*! POLB - PWM_B Output Polarity
 *  0b0..PWM_B output not inverted. A high level on the PWM_B pin represents the "on" or "active" state.
 *  0b1..PWM_B output inverted. A low level on the PWM_B pin represents the "on" or "active" state.
 */
#define PWM_SM3OCTRL_POLB_MASK                   (0x200U)
#define PWM_SM3OCTRL_POLB_SHIFT                  (9U)
#define PWM_SM3OCTRL_POLB(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_POLB_SHIFT)) & PWM_SM3OCTRL_POLB_MASK)
/*! POLA - PWM_A Output Polarity
 *  0b0..PWM_A output not inverted. A high level on the PWM_A pin represents the "on" or "active" state.
 *  0b1..PWM_A output inverted. A low level on the PWM_A pin represents the "on" or "active" state.
 */
#define PWM_SM3OCTRL_POLA_MASK                   (0x400U)
#define PWM_SM3OCTRL_POLA_SHIFT                  (10U)
#define PWM_SM3OCTRL_POLA(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_POLA_SHIFT)) & PWM_SM3OCTRL_POLA_MASK)
#define PWM_SM3OCTRL_PWMX_IN_MASK                (0x2000U)
#define PWM_SM3OCTRL_PWMX_IN_SHIFT               (13U)
#define PWM_SM3OCTRL_PWMX_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMX_IN_SHIFT)) & PWM_SM3OCTRL_PWMX_IN_MASK)
#define PWM_SM3OCTRL_PWMB_IN_MASK                (0x4000U)
#define PWM_SM3OCTRL_PWMB_IN_SHIFT               (14U)
#define PWM_SM3OCTRL_PWMB_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMB_IN_SHIFT)) & PWM_SM3OCTRL_PWMB_IN_MASK)
#define PWM_SM3OCTRL_PWMA_IN_MASK                (0x8000U)
#define PWM_SM3OCTRL_PWMA_IN_SHIFT               (15U)
#define PWM_SM3OCTRL_PWMA_IN(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3OCTRL_PWMA_IN_SHIFT)) & PWM_SM3OCTRL_PWMA_IN_MASK)
/*! @} */

/*! @name SM3STS - Status Register */
/*! @{ */
/*! CMPF - Compare Flags
 *  0b000000..No compare event has occurred for a particular VALx value.
 *  0b000001..A compare event has occurred for a particular VALx value.
 */
#define PWM_SM3STS_CMPF_MASK                     (0x3FU)
#define PWM_SM3STS_CMPF_SHIFT                    (0U)
#define PWM_SM3STS_CMPF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CMPF_SHIFT)) & PWM_SM3STS_CMPF_MASK)
#define PWM_SM3STS_CFX0_MASK                     (0x40U)
#define PWM_SM3STS_CFX0_SHIFT                    (6U)
#define PWM_SM3STS_CFX0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFX0_SHIFT)) & PWM_SM3STS_CFX0_MASK)
#define PWM_SM3STS_CFX1_MASK                     (0x80U)
#define PWM_SM3STS_CFX1_SHIFT                    (7U)
#define PWM_SM3STS_CFX1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFX1_SHIFT)) & PWM_SM3STS_CFX1_MASK)
#define PWM_SM3STS_CFB0_MASK                     (0x100U)
#define PWM_SM3STS_CFB0_SHIFT                    (8U)
#define PWM_SM3STS_CFB0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFB0_SHIFT)) & PWM_SM3STS_CFB0_MASK)
#define PWM_SM3STS_CFB1_MASK                     (0x200U)
#define PWM_SM3STS_CFB1_SHIFT                    (9U)
#define PWM_SM3STS_CFB1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFB1_SHIFT)) & PWM_SM3STS_CFB1_MASK)
#define PWM_SM3STS_CFA0_MASK                     (0x400U)
#define PWM_SM3STS_CFA0_SHIFT                    (10U)
#define PWM_SM3STS_CFA0(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFA0_SHIFT)) & PWM_SM3STS_CFA0_MASK)
#define PWM_SM3STS_CFA1_MASK                     (0x800U)
#define PWM_SM3STS_CFA1_SHIFT                    (11U)
#define PWM_SM3STS_CFA1(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_CFA1_SHIFT)) & PWM_SM3STS_CFA1_MASK)
/*! RF - Reload Flag
 *  0b0..No new reload cycle since last STS[RF] clearing
 *  0b1..New reload cycle since last STS[RF] clearing
 */
#define PWM_SM3STS_RF_MASK                       (0x1000U)
#define PWM_SM3STS_RF_SHIFT                      (12U)
#define PWM_SM3STS_RF(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_RF_SHIFT)) & PWM_SM3STS_RF_MASK)
/*! REF - Reload Error Flag
 *  0b0..No reload error occurred.
 *  0b1..Reload signal occurred with non-coherent data and MCTRL[LDOK] = 0.
 */
#define PWM_SM3STS_REF_MASK                      (0x2000U)
#define PWM_SM3STS_REF_SHIFT                     (13U)
#define PWM_SM3STS_REF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_REF_SHIFT)) & PWM_SM3STS_REF_MASK)
/*! RUF - Registers Updated Flag
 *  0b0..No register update has occurred since last reload.
 *  0b1..At least one of the double buffered registers has been updated since the last reload.
 */
#define PWM_SM3STS_RUF_MASK                      (0x4000U)
#define PWM_SM3STS_RUF_SHIFT                     (14U)
#define PWM_SM3STS_RUF(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_SM3STS_RUF_SHIFT)) & PWM_SM3STS_RUF_MASK)
/*! @} */

/*! @name SM3INTEN - Interrupt Enable Register */
/*! @{ */
/*! CMPIE - Compare Interrupt Enables
 *  0b000000..The corresponding STS[CMPF] bit will not cause an interrupt request.
 *  0b000001..The corresponding STS[CMPF] bit will cause an interrupt request.
 */
#define PWM_SM3INTEN_CMPIE_MASK                  (0x3FU)
#define PWM_SM3INTEN_CMPIE_SHIFT                 (0U)
#define PWM_SM3INTEN_CMPIE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CMPIE_SHIFT)) & PWM_SM3INTEN_CMPIE_MASK)
/*! CX0IE - Capture X 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX0].
 *  0b1..Interrupt request enabled for STS[CFX0].
 */
#define PWM_SM3INTEN_CX0IE_MASK                  (0x40U)
#define PWM_SM3INTEN_CX0IE_SHIFT                 (6U)
#define PWM_SM3INTEN_CX0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CX0IE_SHIFT)) & PWM_SM3INTEN_CX0IE_MASK)
/*! CX1IE - Capture X 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFX1].
 *  0b1..Interrupt request enabled for STS[CFX1].
 */
#define PWM_SM3INTEN_CX1IE_MASK                  (0x80U)
#define PWM_SM3INTEN_CX1IE_SHIFT                 (7U)
#define PWM_SM3INTEN_CX1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CX1IE_SHIFT)) & PWM_SM3INTEN_CX1IE_MASK)
/*! CB0IE - Capture B 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB0].
 *  0b1..Interrupt request enabled for STS[CFB0].
 */
#define PWM_SM3INTEN_CB0IE_MASK                  (0x100U)
#define PWM_SM3INTEN_CB0IE_SHIFT                 (8U)
#define PWM_SM3INTEN_CB0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CB0IE_SHIFT)) & PWM_SM3INTEN_CB0IE_MASK)
/*! CB1IE - Capture B 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFB1].
 *  0b1..Interrupt request enabled for STS[CFB1].
 */
#define PWM_SM3INTEN_CB1IE_MASK                  (0x200U)
#define PWM_SM3INTEN_CB1IE_SHIFT                 (9U)
#define PWM_SM3INTEN_CB1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CB1IE_SHIFT)) & PWM_SM3INTEN_CB1IE_MASK)
/*! CA0IE - Capture A 0 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA0].
 *  0b1..Interrupt request enabled for STS[CFA0].
 */
#define PWM_SM3INTEN_CA0IE_MASK                  (0x400U)
#define PWM_SM3INTEN_CA0IE_SHIFT                 (10U)
#define PWM_SM3INTEN_CA0IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CA0IE_SHIFT)) & PWM_SM3INTEN_CA0IE_MASK)
/*! CA1IE - Capture A 1 Interrupt Enable
 *  0b0..Interrupt request disabled for STS[CFA1].
 *  0b1..Interrupt request enabled for STS[CFA1].
 */
#define PWM_SM3INTEN_CA1IE_MASK                  (0x800U)
#define PWM_SM3INTEN_CA1IE_SHIFT                 (11U)
#define PWM_SM3INTEN_CA1IE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_CA1IE_SHIFT)) & PWM_SM3INTEN_CA1IE_MASK)
/*! RIE - Reload Interrupt Enable
 *  0b0..STS[RF] CPU interrupt requests disabled
 *  0b1..STS[RF] CPU interrupt requests enabled
 */
#define PWM_SM3INTEN_RIE_MASK                    (0x1000U)
#define PWM_SM3INTEN_RIE_SHIFT                   (12U)
#define PWM_SM3INTEN_RIE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_RIE_SHIFT)) & PWM_SM3INTEN_RIE_MASK)
/*! REIE - Reload Error Interrupt Enable
 *  0b0..STS[REF] CPU interrupt requests disabled
 *  0b1..STS[REF] CPU interrupt requests enabled
 */
#define PWM_SM3INTEN_REIE_MASK                   (0x2000U)
#define PWM_SM3INTEN_REIE_SHIFT                  (13U)
#define PWM_SM3INTEN_REIE(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3INTEN_REIE_SHIFT)) & PWM_SM3INTEN_REIE_MASK)
/*! @} */

/*! @name SM3DMAEN - DMA Enable Register */
/*! @{ */
#define PWM_SM3DMAEN_CX0DE_MASK                  (0x1U)
#define PWM_SM3DMAEN_CX0DE_SHIFT                 (0U)
#define PWM_SM3DMAEN_CX0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CX0DE_SHIFT)) & PWM_SM3DMAEN_CX0DE_MASK)
#define PWM_SM3DMAEN_CX1DE_MASK                  (0x2U)
#define PWM_SM3DMAEN_CX1DE_SHIFT                 (1U)
#define PWM_SM3DMAEN_CX1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CX1DE_SHIFT)) & PWM_SM3DMAEN_CX1DE_MASK)
#define PWM_SM3DMAEN_CB0DE_MASK                  (0x4U)
#define PWM_SM3DMAEN_CB0DE_SHIFT                 (2U)
#define PWM_SM3DMAEN_CB0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CB0DE_SHIFT)) & PWM_SM3DMAEN_CB0DE_MASK)
#define PWM_SM3DMAEN_CB1DE_MASK                  (0x8U)
#define PWM_SM3DMAEN_CB1DE_SHIFT                 (3U)
#define PWM_SM3DMAEN_CB1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CB1DE_SHIFT)) & PWM_SM3DMAEN_CB1DE_MASK)
#define PWM_SM3DMAEN_CA0DE_MASK                  (0x10U)
#define PWM_SM3DMAEN_CA0DE_SHIFT                 (4U)
#define PWM_SM3DMAEN_CA0DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CA0DE_SHIFT)) & PWM_SM3DMAEN_CA0DE_MASK)
#define PWM_SM3DMAEN_CA1DE_MASK                  (0x20U)
#define PWM_SM3DMAEN_CA1DE_SHIFT                 (5U)
#define PWM_SM3DMAEN_CA1DE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CA1DE_SHIFT)) & PWM_SM3DMAEN_CA1DE_MASK)
/*! CAPTDE - Capture DMA Enable Source Select
 *  0b00..Read DMA requests disabled.
 *  0b01..Exceeding a FIFO watermark sets the DMA read request. This requires at least one of DMAEN[CA1DE],
 *        DMAEN[CA0DE], DMAEN[CB1DE], DMAEN[CB0DE], DMAEN[CX1DE], or DMAEN[CX0DE] to also be set in order to determine to
 *        which watermark(s) the DMA request is sensitive.
 *  0b10..A local sync (VAL1 matches counter) sets the read DMA request.
 *  0b11..A local reload (STS[RF] being set) sets the read DMA request.
 */
#define PWM_SM3DMAEN_CAPTDE_MASK                 (0xC0U)
#define PWM_SM3DMAEN_CAPTDE_SHIFT                (6U)
#define PWM_SM3DMAEN_CAPTDE(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_CAPTDE_SHIFT)) & PWM_SM3DMAEN_CAPTDE_MASK)
/*! FAND - FIFO Watermark AND Control
 *  0b0..Selected FIFO watermarks are OR'ed together.
 *  0b1..Selected FIFO watermarks are AND'ed together.
 */
#define PWM_SM3DMAEN_FAND_MASK                   (0x100U)
#define PWM_SM3DMAEN_FAND_SHIFT                  (8U)
#define PWM_SM3DMAEN_FAND(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_FAND_SHIFT)) & PWM_SM3DMAEN_FAND_MASK)
/*! VALDE - Value Registers DMA Enable
 *  0b0..DMA write requests disabled
 *  0b1..DMA write requests for the VALx and FRACVALx registers enabled
 */
#define PWM_SM3DMAEN_VALDE_MASK                  (0x200U)
#define PWM_SM3DMAEN_VALDE_SHIFT                 (9U)
#define PWM_SM3DMAEN_VALDE(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_SM3DMAEN_VALDE_SHIFT)) & PWM_SM3DMAEN_VALDE_MASK)
/*! @} */

/*! @name SM3TCTRL - Output Trigger Control Register */
/*! @{ */
/*! OUT_TRIG_EN - Output Trigger Enables
 *  0b000000..PWM_OUT_TRIGx will not set when the counter value matches the VALx value.
 *  0b000001..PWM_OUT_TRIGx will set when the counter value matches the VALx value.
 */
#define PWM_SM3TCTRL_OUT_TRIG_EN_MASK            (0x3FU)
#define PWM_SM3TCTRL_OUT_TRIG_EN_SHIFT           (0U)
#define PWM_SM3TCTRL_OUT_TRIG_EN(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3TCTRL_OUT_TRIG_EN_SHIFT)) & PWM_SM3TCTRL_OUT_TRIG_EN_MASK)
/*! TRGFRQ - Trigger frequency
 *  0b0..Trigger outputs are generated during every PWM period even if the PWM is not reloaded every period due to CTRL[LDFQ] being non-zero.
 *  0b1..Trigger outputs are generated only during the final PWM period prior to a reload opportunity when the PWM
 *       is not reloaded every period due to CTRL[LDFQ] being non-zero.
 */
#define PWM_SM3TCTRL_TRGFRQ_MASK                 (0x1000U)
#define PWM_SM3TCTRL_TRGFRQ_SHIFT                (12U)
#define PWM_SM3TCTRL_TRGFRQ(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3TCTRL_TRGFRQ_SHIFT)) & PWM_SM3TCTRL_TRGFRQ_MASK)
/*! PWBOT1 - Output Trigger 1 Source Select
 *  0b0..Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port.
 *  0b1..Route the PWMB output to the PWM_OUT_TRIG1 port.
 */
#define PWM_SM3TCTRL_PWBOT1_MASK                 (0x4000U)
#define PWM_SM3TCTRL_PWBOT1_SHIFT                (14U)
#define PWM_SM3TCTRL_PWBOT1(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3TCTRL_PWBOT1_SHIFT)) & PWM_SM3TCTRL_PWBOT1_MASK)
/*! PWAOT0 - Output Trigger 0 Source Select
 *  0b0..Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port.
 *  0b1..Route the PWMA output to the PWM_OUT_TRIG0 port.
 */
#define PWM_SM3TCTRL_PWAOT0_MASK                 (0x8000U)
#define PWM_SM3TCTRL_PWAOT0_SHIFT                (15U)
#define PWM_SM3TCTRL_PWAOT0(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SM3TCTRL_PWAOT0_SHIFT)) & PWM_SM3TCTRL_PWAOT0_MASK)
/*! @} */

/*! @name SM3DISMAP0 - Fault Disable Mapping Register 0 */
/*! @{ */
#define PWM_SM3DISMAP0_DIS0A_MASK                (0xFU)
#define PWM_SM3DISMAP0_DIS0A_SHIFT               (0U)
#define PWM_SM3DISMAP0_DIS0A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP0_DIS0A_SHIFT)) & PWM_SM3DISMAP0_DIS0A_MASK)
#define PWM_SM3DISMAP0_DIS0B_MASK                (0xF0U)
#define PWM_SM3DISMAP0_DIS0B_SHIFT               (4U)
#define PWM_SM3DISMAP0_DIS0B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP0_DIS0B_SHIFT)) & PWM_SM3DISMAP0_DIS0B_MASK)
#define PWM_SM3DISMAP0_DIS0X_MASK                (0xF00U)
#define PWM_SM3DISMAP0_DIS0X_SHIFT               (8U)
#define PWM_SM3DISMAP0_DIS0X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP0_DIS0X_SHIFT)) & PWM_SM3DISMAP0_DIS0X_MASK)
/*! @} */

/*! @name SM3DISMAP1 - Fault Disable Mapping Register 1 */
/*! @{ */
#define PWM_SM3DISMAP1_DIS1A_MASK                (0xFU)
#define PWM_SM3DISMAP1_DIS1A_SHIFT               (0U)
#define PWM_SM3DISMAP1_DIS1A(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP1_DIS1A_SHIFT)) & PWM_SM3DISMAP1_DIS1A_MASK)
#define PWM_SM3DISMAP1_DIS1B_MASK                (0xF0U)
#define PWM_SM3DISMAP1_DIS1B_SHIFT               (4U)
#define PWM_SM3DISMAP1_DIS1B(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP1_DIS1B_SHIFT)) & PWM_SM3DISMAP1_DIS1B_MASK)
#define PWM_SM3DISMAP1_DIS1X_MASK                (0xF00U)
#define PWM_SM3DISMAP1_DIS1X_SHIFT               (8U)
#define PWM_SM3DISMAP1_DIS1X(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DISMAP1_DIS1X_SHIFT)) & PWM_SM3DISMAP1_DIS1X_MASK)
/*! @} */

/*! @name SM3DTCNT0 - Deadtime Count Register 0 */
/*! @{ */
#define PWM_SM3DTCNT0_DTCNT0_MASK                (0xFFFFU)
#define PWM_SM3DTCNT0_DTCNT0_SHIFT               (0U)
#define PWM_SM3DTCNT0_DTCNT0(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DTCNT0_DTCNT0_SHIFT)) & PWM_SM3DTCNT0_DTCNT0_MASK)
/*! @} */

/*! @name SM3DTCNT1 - Deadtime Count Register 1 */
/*! @{ */
#define PWM_SM3DTCNT1_DTCNT1_MASK                (0xFFFFU)
#define PWM_SM3DTCNT1_DTCNT1_SHIFT               (0U)
#define PWM_SM3DTCNT1_DTCNT1(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_SM3DTCNT1_DTCNT1_SHIFT)) & PWM_SM3DTCNT1_DTCNT1_MASK)
/*! @} */

/*! @name SM3CAPTCTRLA - Capture Control A Register */
/*! @{ */
/*! ARMA - Arm A
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLA[EDGAx] is enabled.
 */
#define PWM_SM3CAPTCTRLA_ARMA_MASK               (0x1U)
#define PWM_SM3CAPTCTRLA_ARMA_SHIFT              (0U)
#define PWM_SM3CAPTCTRLA_ARMA(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_ARMA_SHIFT)) & PWM_SM3CAPTCTRLA_ARMA_MASK)
/*! ONESHOTA - One Shot Mode A
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLA[ARMA] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLA[ARMA] is cleared. No
 *       further captures will be performed until CAPTCTRLA[ARMA] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLA[ARMA] is then cleared.
 */
#define PWM_SM3CAPTCTRLA_ONESHOTA_MASK           (0x2U)
#define PWM_SM3CAPTCTRLA_ONESHOTA_SHIFT          (1U)
#define PWM_SM3CAPTCTRLA_ONESHOTA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_ONESHOTA_SHIFT)) & PWM_SM3CAPTCTRLA_ONESHOTA_MASK)
/*! EDGA0 - Edge A 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLA_EDGA0_MASK              (0xCU)
#define PWM_SM3CAPTCTRLA_EDGA0_SHIFT             (2U)
#define PWM_SM3CAPTCTRLA_EDGA0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_EDGA0_SHIFT)) & PWM_SM3CAPTCTRLA_EDGA0_MASK)
/*! EDGA1 - Edge A 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLA_EDGA1_MASK              (0x30U)
#define PWM_SM3CAPTCTRLA_EDGA1_SHIFT             (4U)
#define PWM_SM3CAPTCTRLA_EDGA1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_EDGA1_SHIFT)) & PWM_SM3CAPTCTRLA_EDGA1_MASK)
/*! INP_SELA - Input Select A
 *  0b0..Raw PWM_A input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLA[EDGA0] and
 *       CAPTCTRLA[EDGA1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRA[EDGA0] and/or CAPTCTRLA[EDGA1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM3CAPTCTRLA_INP_SELA_MASK           (0x40U)
#define PWM_SM3CAPTCTRLA_INP_SELA_SHIFT          (6U)
#define PWM_SM3CAPTCTRLA_INP_SELA(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_INP_SELA_SHIFT)) & PWM_SM3CAPTCTRLA_INP_SELA_MASK)
/*! EDGCNTA_EN - Edge Counter A Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM3CAPTCTRLA_EDGCNTA_EN_MASK         (0x80U)
#define PWM_SM3CAPTCTRLA_EDGCNTA_EN_SHIFT        (7U)
#define PWM_SM3CAPTCTRLA_EDGCNTA_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_EDGCNTA_EN_SHIFT)) & PWM_SM3CAPTCTRLA_EDGCNTA_EN_MASK)
#define PWM_SM3CAPTCTRLA_CFAWM_MASK              (0x300U)
#define PWM_SM3CAPTCTRLA_CFAWM_SHIFT             (8U)
#define PWM_SM3CAPTCTRLA_CFAWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_CFAWM_SHIFT)) & PWM_SM3CAPTCTRLA_CFAWM_MASK)
#define PWM_SM3CAPTCTRLA_CA0CNT_MASK             (0x1C00U)
#define PWM_SM3CAPTCTRLA_CA0CNT_SHIFT            (10U)
#define PWM_SM3CAPTCTRLA_CA0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_CA0CNT_SHIFT)) & PWM_SM3CAPTCTRLA_CA0CNT_MASK)
#define PWM_SM3CAPTCTRLA_CA1CNT_MASK             (0xE000U)
#define PWM_SM3CAPTCTRLA_CA1CNT_SHIFT            (13U)
#define PWM_SM3CAPTCTRLA_CA1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLA_CA1CNT_SHIFT)) & PWM_SM3CAPTCTRLA_CA1CNT_MASK)
/*! @} */

/*! @name SM3CAPTCOMPA - Capture Compare A Register */
/*! @{ */
#define PWM_SM3CAPTCOMPA_EDGCMPA_MASK            (0xFFU)
#define PWM_SM3CAPTCOMPA_EDGCMPA_SHIFT           (0U)
#define PWM_SM3CAPTCOMPA_EDGCMPA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPA_EDGCMPA_SHIFT)) & PWM_SM3CAPTCOMPA_EDGCMPA_MASK)
#define PWM_SM3CAPTCOMPA_EDGCNTA_MASK            (0xFF00U)
#define PWM_SM3CAPTCOMPA_EDGCNTA_SHIFT           (8U)
#define PWM_SM3CAPTCOMPA_EDGCNTA(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPA_EDGCNTA_SHIFT)) & PWM_SM3CAPTCOMPA_EDGCNTA_MASK)
/*! @} */

/*! @name SM3CAPTCTRLB - Capture Control B Register */
/*! @{ */
/*! ARMB - Arm B
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLB[EDGBx] is enabled.
 */
#define PWM_SM3CAPTCTRLB_ARMB_MASK               (0x1U)
#define PWM_SM3CAPTCTRLB_ARMB_SHIFT              (0U)
#define PWM_SM3CAPTCTRLB_ARMB(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_ARMB_SHIFT)) & PWM_SM3CAPTCTRLB_ARMB_MASK)
/*! ONESHOTB - One Shot Mode B
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1
 *       is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed.
 *       The process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after CAPTCTRLB[ARMB] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLB[ARMB] is cleared. No
 *       further captures will be performed until CAPTCTRLB[ARMB] is set again.If only one of the capture circuits is
 *       enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLB[ARMB] is then cleared.
 */
#define PWM_SM3CAPTCTRLB_ONESHOTB_MASK           (0x2U)
#define PWM_SM3CAPTCTRLB_ONESHOTB_SHIFT          (1U)
#define PWM_SM3CAPTCTRLB_ONESHOTB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_ONESHOTB_SHIFT)) & PWM_SM3CAPTCTRLB_ONESHOTB_MASK)
/*! EDGB0 - Edge B 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLB_EDGB0_MASK              (0xCU)
#define PWM_SM3CAPTCTRLB_EDGB0_SHIFT             (2U)
#define PWM_SM3CAPTCTRLB_EDGB0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_EDGB0_SHIFT)) & PWM_SM3CAPTCTRLB_EDGB0_MASK)
/*! EDGB1 - Edge B 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLB_EDGB1_MASK              (0x30U)
#define PWM_SM3CAPTCTRLB_EDGB1_SHIFT             (4U)
#define PWM_SM3CAPTCTRLB_EDGB1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_EDGB1_SHIFT)) & PWM_SM3CAPTCTRLB_EDGB1_MASK)
/*! INP_SELB - Input Select B
 *  0b0..Raw PWM_B input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLB[EDGB0] and
 *       CAPTCTRLB[EDGB1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRB[EDGB0] and/or CAPTCTRLB[EDGB1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM3CAPTCTRLB_INP_SELB_MASK           (0x40U)
#define PWM_SM3CAPTCTRLB_INP_SELB_SHIFT          (6U)
#define PWM_SM3CAPTCTRLB_INP_SELB(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_INP_SELB_SHIFT)) & PWM_SM3CAPTCTRLB_INP_SELB_MASK)
/*! EDGCNTB_EN - Edge Counter B Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM3CAPTCTRLB_EDGCNTB_EN_MASK         (0x80U)
#define PWM_SM3CAPTCTRLB_EDGCNTB_EN_SHIFT        (7U)
#define PWM_SM3CAPTCTRLB_EDGCNTB_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_EDGCNTB_EN_SHIFT)) & PWM_SM3CAPTCTRLB_EDGCNTB_EN_MASK)
#define PWM_SM3CAPTCTRLB_CFBWM_MASK              (0x300U)
#define PWM_SM3CAPTCTRLB_CFBWM_SHIFT             (8U)
#define PWM_SM3CAPTCTRLB_CFBWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_CFBWM_SHIFT)) & PWM_SM3CAPTCTRLB_CFBWM_MASK)
#define PWM_SM3CAPTCTRLB_CB0CNT_MASK             (0x1C00U)
#define PWM_SM3CAPTCTRLB_CB0CNT_SHIFT            (10U)
#define PWM_SM3CAPTCTRLB_CB0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_CB0CNT_SHIFT)) & PWM_SM3CAPTCTRLB_CB0CNT_MASK)
#define PWM_SM3CAPTCTRLB_CB1CNT_MASK             (0xE000U)
#define PWM_SM3CAPTCTRLB_CB1CNT_SHIFT            (13U)
#define PWM_SM3CAPTCTRLB_CB1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLB_CB1CNT_SHIFT)) & PWM_SM3CAPTCTRLB_CB1CNT_MASK)
/*! @} */

/*! @name SM3CAPTCOMPB - Capture Compare B Register */
/*! @{ */
#define PWM_SM3CAPTCOMPB_EDGCMPB_MASK            (0xFFU)
#define PWM_SM3CAPTCOMPB_EDGCMPB_SHIFT           (0U)
#define PWM_SM3CAPTCOMPB_EDGCMPB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPB_EDGCMPB_SHIFT)) & PWM_SM3CAPTCOMPB_EDGCMPB_MASK)
#define PWM_SM3CAPTCOMPB_EDGCNTB_MASK            (0xFF00U)
#define PWM_SM3CAPTCOMPB_EDGCNTB_SHIFT           (8U)
#define PWM_SM3CAPTCOMPB_EDGCNTB(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPB_EDGCNTB_SHIFT)) & PWM_SM3CAPTCOMPB_EDGCNTB_MASK)
/*! @} */

/*! @name SM3CAPTCTRLX - Capture Control X Register */
/*! @{ */
/*! ARMX - Arm X
 *  0b0..Input capture operation is disabled.
 *  0b1..Input capture operation as specified by CAPTCTRLX[EDGXx] is enabled.
 */
#define PWM_SM3CAPTCTRLX_ARMX_MASK               (0x1U)
#define PWM_SM3CAPTCTRLX_ARMX_SHIFT              (0U)
#define PWM_SM3CAPTCTRLX_ARMX(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_ARMX_SHIFT)) & PWM_SM3CAPTCTRLX_ARMX_MASK)
/*! ONESHOTX - One Shot Mode Aux
 *  0b0..Free running mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed
 *       first after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The
 *       process continues indefinitely.If only one of the capture circuits is enabled, then captures continue
 *       indefinitely on the enabled capture circuit.
 *  0b1..One shot mode is selected. If both capture circuits are enabled, then capture circuit 0 is armed first
 *       after the ARMX bit is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is
 *       armed. After capture circuit 1 performs a capture, it is disarmed and the ARMX bit is cleared. No further
 *       captures will be performed until the ARMX bit is set again.If only one of the capture circuits is enabled,
 *       then a single capture will occur on the enabled capture circuit and the ARMX bit is then cleared.
 */
#define PWM_SM3CAPTCTRLX_ONESHOTX_MASK           (0x2U)
#define PWM_SM3CAPTCTRLX_ONESHOTX_SHIFT          (1U)
#define PWM_SM3CAPTCTRLX_ONESHOTX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_ONESHOTX_SHIFT)) & PWM_SM3CAPTCTRLX_ONESHOTX_MASK)
/*! EDGX0 - Edge X 0
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLX_EDGX0_MASK              (0xCU)
#define PWM_SM3CAPTCTRLX_EDGX0_SHIFT             (2U)
#define PWM_SM3CAPTCTRLX_EDGX0(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_EDGX0_SHIFT)) & PWM_SM3CAPTCTRLX_EDGX0_MASK)
/*! EDGX1 - Edge X 1
 *  0b00..Disabled
 *  0b01..Capture falling edges
 *  0b10..Capture rising edges
 *  0b11..Capture any edge
 */
#define PWM_SM3CAPTCTRLX_EDGX1_MASK              (0x30U)
#define PWM_SM3CAPTCTRLX_EDGX1_SHIFT             (4U)
#define PWM_SM3CAPTCTRLX_EDGX1(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_EDGX1_SHIFT)) & PWM_SM3CAPTCTRLX_EDGX1_MASK)
/*! INP_SELX - Input Select X
 *  0b0..Raw PWM_X input signal selected as source.
 *  0b1..Output of edge counter/compare selected as source. Note that when this bitfield is set to 1, the internal
 *       edge counter is enabled and the rising and/or falling edges specified by the CAPTCTRLX[EDGX0] and
 *       CAPTCTRLX[EDGX1] fields are ignored. The software must still place a value other than 00 in either or both of the
 *       CAPTCTLRX[EDGX0] and/or CAPTCTRLX[EDGX1] fields in order to enable one or both of the capture registers.
 */
#define PWM_SM3CAPTCTRLX_INP_SELX_MASK           (0x40U)
#define PWM_SM3CAPTCTRLX_INP_SELX_SHIFT          (6U)
#define PWM_SM3CAPTCTRLX_INP_SELX(x)             (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_INP_SELX_SHIFT)) & PWM_SM3CAPTCTRLX_INP_SELX_MASK)
/*! EDGCNTX_EN - Edge Counter X Enable
 *  0b0..Edge counter disabled and held in reset
 *  0b1..Edge counter enabled
 */
#define PWM_SM3CAPTCTRLX_EDGCNTX_EN_MASK         (0x80U)
#define PWM_SM3CAPTCTRLX_EDGCNTX_EN_SHIFT        (7U)
#define PWM_SM3CAPTCTRLX_EDGCNTX_EN(x)           (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_EDGCNTX_EN_SHIFT)) & PWM_SM3CAPTCTRLX_EDGCNTX_EN_MASK)
#define PWM_SM3CAPTCTRLX_CFXWM_MASK              (0x300U)
#define PWM_SM3CAPTCTRLX_CFXWM_SHIFT             (8U)
#define PWM_SM3CAPTCTRLX_CFXWM(x)                (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_CFXWM_SHIFT)) & PWM_SM3CAPTCTRLX_CFXWM_MASK)
#define PWM_SM3CAPTCTRLX_CX0CNT_MASK             (0x1C00U)
#define PWM_SM3CAPTCTRLX_CX0CNT_SHIFT            (10U)
#define PWM_SM3CAPTCTRLX_CX0CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_CX0CNT_SHIFT)) & PWM_SM3CAPTCTRLX_CX0CNT_MASK)
#define PWM_SM3CAPTCTRLX_CX1CNT_MASK             (0xE000U)
#define PWM_SM3CAPTCTRLX_CX1CNT_SHIFT            (13U)
#define PWM_SM3CAPTCTRLX_CX1CNT(x)               (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCTRLX_CX1CNT_SHIFT)) & PWM_SM3CAPTCTRLX_CX1CNT_MASK)
/*! @} */

/*! @name SM3CAPTCOMPX - Capture Compare X Register */
/*! @{ */
#define PWM_SM3CAPTCOMPX_EDGCMPX_MASK            (0xFFU)
#define PWM_SM3CAPTCOMPX_EDGCMPX_SHIFT           (0U)
#define PWM_SM3CAPTCOMPX_EDGCMPX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPX_EDGCMPX_SHIFT)) & PWM_SM3CAPTCOMPX_EDGCMPX_MASK)
#define PWM_SM3CAPTCOMPX_EDGCNTX_MASK            (0xFF00U)
#define PWM_SM3CAPTCOMPX_EDGCNTX_SHIFT           (8U)
#define PWM_SM3CAPTCOMPX_EDGCNTX(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CAPTCOMPX_EDGCNTX_SHIFT)) & PWM_SM3CAPTCOMPX_EDGCNTX_MASK)
/*! @} */

/*! @name SM3CVAL0 - Capture Value 0 Register */
/*! @{ */
#define PWM_SM3CVAL0_CAPTVAL0_MASK               (0xFFFFU)
#define PWM_SM3CVAL0_CAPTVAL0_SHIFT              (0U)
#define PWM_SM3CVAL0_CAPTVAL0(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL0_CAPTVAL0_SHIFT)) & PWM_SM3CVAL0_CAPTVAL0_MASK)
/*! @} */

/*! @name SM3CVAL0CYC - Capture Value 0 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL0CYC_CVAL0CYC_MASK            (0xFU)
#define PWM_SM3CVAL0CYC_CVAL0CYC_SHIFT           (0U)
#define PWM_SM3CVAL0CYC_CVAL0CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL0CYC_CVAL0CYC_SHIFT)) & PWM_SM3CVAL0CYC_CVAL0CYC_MASK)
/*! @} */

/*! @name SM3CVAL1 - Capture Value 1 Register */
/*! @{ */
#define PWM_SM3CVAL1_CAPTVAL1_MASK               (0xFFFFU)
#define PWM_SM3CVAL1_CAPTVAL1_SHIFT              (0U)
#define PWM_SM3CVAL1_CAPTVAL1(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL1_CAPTVAL1_SHIFT)) & PWM_SM3CVAL1_CAPTVAL1_MASK)
/*! @} */

/*! @name SM3CVAL1CYC - Capture Value 1 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL1CYC_CVAL1CYC_MASK            (0xFU)
#define PWM_SM3CVAL1CYC_CVAL1CYC_SHIFT           (0U)
#define PWM_SM3CVAL1CYC_CVAL1CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL1CYC_CVAL1CYC_SHIFT)) & PWM_SM3CVAL1CYC_CVAL1CYC_MASK)
/*! @} */

/*! @name SM3CVAL2 - Capture Value 2 Register */
/*! @{ */
#define PWM_SM3CVAL2_CAPTVAL2_MASK               (0xFFFFU)
#define PWM_SM3CVAL2_CAPTVAL2_SHIFT              (0U)
#define PWM_SM3CVAL2_CAPTVAL2(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL2_CAPTVAL2_SHIFT)) & PWM_SM3CVAL2_CAPTVAL2_MASK)
/*! @} */

/*! @name SM3CVAL2CYC - Capture Value 2 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL2CYC_CVAL2CYC_MASK            (0xFU)
#define PWM_SM3CVAL2CYC_CVAL2CYC_SHIFT           (0U)
#define PWM_SM3CVAL2CYC_CVAL2CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL2CYC_CVAL2CYC_SHIFT)) & PWM_SM3CVAL2CYC_CVAL2CYC_MASK)
/*! @} */

/*! @name SM3CVAL3 - Capture Value 3 Register */
/*! @{ */
#define PWM_SM3CVAL3_CAPTVAL3_MASK               (0xFFFFU)
#define PWM_SM3CVAL3_CAPTVAL3_SHIFT              (0U)
#define PWM_SM3CVAL3_CAPTVAL3(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL3_CAPTVAL3_SHIFT)) & PWM_SM3CVAL3_CAPTVAL3_MASK)
/*! @} */

/*! @name SM3CVAL3CYC - Capture Value 3 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL3CYC_CVAL3CYC_MASK            (0xFU)
#define PWM_SM3CVAL3CYC_CVAL3CYC_SHIFT           (0U)
#define PWM_SM3CVAL3CYC_CVAL3CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL3CYC_CVAL3CYC_SHIFT)) & PWM_SM3CVAL3CYC_CVAL3CYC_MASK)
/*! @} */

/*! @name SM3CVAL4 - Capture Value 4 Register */
/*! @{ */
#define PWM_SM3CVAL4_CAPTVAL4_MASK               (0xFFFFU)
#define PWM_SM3CVAL4_CAPTVAL4_SHIFT              (0U)
#define PWM_SM3CVAL4_CAPTVAL4(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL4_CAPTVAL4_SHIFT)) & PWM_SM3CVAL4_CAPTVAL4_MASK)
/*! @} */

/*! @name SM3CVAL4CYC - Capture Value 4 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL4CYC_CVAL4CYC_MASK            (0xFU)
#define PWM_SM3CVAL4CYC_CVAL4CYC_SHIFT           (0U)
#define PWM_SM3CVAL4CYC_CVAL4CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL4CYC_CVAL4CYC_SHIFT)) & PWM_SM3CVAL4CYC_CVAL4CYC_MASK)
/*! @} */

/*! @name SM3CVAL5 - Capture Value 5 Register */
/*! @{ */
#define PWM_SM3CVAL5_CAPTVAL5_MASK               (0xFFFFU)
#define PWM_SM3CVAL5_CAPTVAL5_SHIFT              (0U)
#define PWM_SM3CVAL5_CAPTVAL5(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL5_CAPTVAL5_SHIFT)) & PWM_SM3CVAL5_CAPTVAL5_MASK)
/*! @} */

/*! @name SM3CVAL5CYC - Capture Value 5 Cycle Register */
/*! @{ */
#define PWM_SM3CVAL5CYC_CVAL5CYC_MASK            (0xFU)
#define PWM_SM3CVAL5CYC_CVAL5CYC_SHIFT           (0U)
#define PWM_SM3CVAL5CYC_CVAL5CYC(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3CVAL5CYC_CVAL5CYC_SHIFT)) & PWM_SM3CVAL5CYC_CVAL5CYC_MASK)
/*! @} */

/*! @name SM3PHASEDLY - Phase Delay Register */
/*! @{ */
#define PWM_SM3PHASEDLY_PHASEDLY_MASK            (0xFFFFU)
#define PWM_SM3PHASEDLY_PHASEDLY_SHIFT           (0U)
#define PWM_SM3PHASEDLY_PHASEDLY(x)              (((uint16_t)(((uint16_t)(x)) << PWM_SM3PHASEDLY_PHASEDLY_SHIFT)) & PWM_SM3PHASEDLY_PHASEDLY_MASK)
/*! @} */

/*! @name OUTEN - Output Enable Register */
/*! @{ */
/*! PWMX_EN - PWM_X Output Enables
 *  0b0000..PWM_X output disabled.
 *  0b0001..PWM_X output enabled.
 */
#define PWM_OUTEN_PWMX_EN_MASK                   (0xFU)
#define PWM_OUTEN_PWMX_EN_SHIFT                  (0U)
#define PWM_OUTEN_PWMX_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMX_EN_SHIFT)) & PWM_OUTEN_PWMX_EN_MASK)
/*! PWMB_EN - PWM_B Output Enables
 *  0b0000..PWM_B output disabled.
 *  0b0001..PWM_B output enabled.
 */
#define PWM_OUTEN_PWMB_EN_MASK                   (0xF0U)
#define PWM_OUTEN_PWMB_EN_SHIFT                  (4U)
#define PWM_OUTEN_PWMB_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMB_EN_SHIFT)) & PWM_OUTEN_PWMB_EN_MASK)
/*! PWMA_EN - PWM_A Output Enables
 *  0b0000..PWM_A output disabled.
 *  0b0001..PWM_A output enabled.
 */
#define PWM_OUTEN_PWMA_EN_MASK                   (0xF00U)
#define PWM_OUTEN_PWMA_EN_SHIFT                  (8U)
#define PWM_OUTEN_PWMA_EN(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_OUTEN_PWMA_EN_SHIFT)) & PWM_OUTEN_PWMA_EN_MASK)
/*! @} */

/*! @name MASK - Mask Register */
/*! @{ */
/*! MASKX - PWM_X Masks
 *  0b0000..PWM_X output normal.
 *  0b0001..PWM_X output masked.
 */
#define PWM_MASK_MASKX_MASK                      (0xFU)
#define PWM_MASK_MASKX_SHIFT                     (0U)
#define PWM_MASK_MASKX(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKX_SHIFT)) & PWM_MASK_MASKX_MASK)
/*! MASKB - PWM_B Masks
 *  0b0000..PWM_B output normal.
 *  0b0001..PWM_B output masked.
 */
#define PWM_MASK_MASKB_MASK                      (0xF0U)
#define PWM_MASK_MASKB_SHIFT                     (4U)
#define PWM_MASK_MASKB(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKB_SHIFT)) & PWM_MASK_MASKB_MASK)
/*! MASKA - PWM_A Masks
 *  0b0000..PWM_A output normal.
 *  0b0001..PWM_A output masked.
 */
#define PWM_MASK_MASKA_MASK                      (0xF00U)
#define PWM_MASK_MASKA_SHIFT                     (8U)
#define PWM_MASK_MASKA(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MASK_MASKA_SHIFT)) & PWM_MASK_MASKA_MASK)
/*! UPDATE_MASK - Update Mask Bits Immediately
 *  0b0000..Normal operation. MASK* bits within the corresponding submodule are not updated until a FORCE_OUT event occurs within the submodule.
 *  0b0001..Immediate operation. MASK* bits within the corresponding submodule are updated on the following clock edge after setting this bit.
 */
#define PWM_MASK_UPDATE_MASK_MASK                (0xF000U)
#define PWM_MASK_UPDATE_MASK_SHIFT               (12U)
#define PWM_MASK_UPDATE_MASK(x)                  (((uint16_t)(((uint16_t)(x)) << PWM_MASK_UPDATE_MASK_SHIFT)) & PWM_MASK_UPDATE_MASK_MASK)
/*! @} */

/*! @name SWCOUT - Software Controlled Output Register */
/*! @{ */
/*! SM0OUT45 - Submodule 0 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM45.
 */
#define PWM_SWCOUT_SM0OUT45_MASK                 (0x1U)
#define PWM_SWCOUT_SM0OUT45_SHIFT                (0U)
#define PWM_SWCOUT_SM0OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT45_SHIFT)) & PWM_SWCOUT_SM0OUT45_MASK)
/*! SM0OUT23 - Submodule 0 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 0 instead of PWM23.
 */
#define PWM_SWCOUT_SM0OUT23_MASK                 (0x2U)
#define PWM_SWCOUT_SM0OUT23_SHIFT                (1U)
#define PWM_SWCOUT_SM0OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM0OUT23_SHIFT)) & PWM_SWCOUT_SM0OUT23_MASK)
/*! SM1OUT45 - Submodule 1 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM45.
 */
#define PWM_SWCOUT_SM1OUT45_MASK                 (0x4U)
#define PWM_SWCOUT_SM1OUT45_SHIFT                (2U)
#define PWM_SWCOUT_SM1OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT45_SHIFT)) & PWM_SWCOUT_SM1OUT45_MASK)
/*! SM1OUT23 - Submodule 1 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 1 instead of PWM23.
 */
#define PWM_SWCOUT_SM1OUT23_MASK                 (0x8U)
#define PWM_SWCOUT_SM1OUT23_SHIFT                (3U)
#define PWM_SWCOUT_SM1OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM1OUT23_SHIFT)) & PWM_SWCOUT_SM1OUT23_MASK)
/*! SM2OUT45 - Submodule 2 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM45.
 */
#define PWM_SWCOUT_SM2OUT45_MASK                 (0x10U)
#define PWM_SWCOUT_SM2OUT45_SHIFT                (4U)
#define PWM_SWCOUT_SM2OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT45_SHIFT)) & PWM_SWCOUT_SM2OUT45_MASK)
/*! SM2OUT23 - Submodule 2 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 2 instead of PWM23.
 */
#define PWM_SWCOUT_SM2OUT23_MASK                 (0x20U)
#define PWM_SWCOUT_SM2OUT23_SHIFT                (5U)
#define PWM_SWCOUT_SM2OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM2OUT23_SHIFT)) & PWM_SWCOUT_SM2OUT23_MASK)
/*! SM3OUT45 - Submodule 3 Software Controlled Output 45
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 3 instead of PWM45.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 3 instead of PWM45.
 */
#define PWM_SWCOUT_SM3OUT45_MASK                 (0x40U)
#define PWM_SWCOUT_SM3OUT45_SHIFT                (6U)
#define PWM_SWCOUT_SM3OUT45(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM3OUT45_SHIFT)) & PWM_SWCOUT_SM3OUT45_MASK)
/*! SM3OUT23 - Submodule 3 Software Controlled Output 23
 *  0b0..A logic 0 is supplied to the deadtime generator of submodule 3 instead of PWM23.
 *  0b1..A logic 1 is supplied to the deadtime generator of submodule 3 instead of PWM23.
 */
#define PWM_SWCOUT_SM3OUT23_MASK                 (0x80U)
#define PWM_SWCOUT_SM3OUT23_SHIFT                (7U)
#define PWM_SWCOUT_SM3OUT23(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_SWCOUT_SM3OUT23_SHIFT)) & PWM_SWCOUT_SM3OUT23_MASK)
/*! @} */

/*! @name DTSRCSEL - PWM Source Select Register */
/*! @{ */
/*! SM0SEL45 - Submodule 0 PWM45 Control Select
 *  0b00..Generated SM0PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT45] is used by the deadtime logic.
 *  0b11..PWM0_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM0SEL45_MASK               (0x3U)
#define PWM_DTSRCSEL_SM0SEL45_SHIFT              (0U)
#define PWM_DTSRCSEL_SM0SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL45_SHIFT)) & PWM_DTSRCSEL_SM0SEL45_MASK)
/*! SM0SEL23 - Submodule 0 PWM23 Control Select
 *  0b00..Generated SM0PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM0PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM0OUT23] is used by the deadtime logic.
 *  0b11..PWM0_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM0SEL23_MASK               (0xCU)
#define PWM_DTSRCSEL_SM0SEL23_SHIFT              (2U)
#define PWM_DTSRCSEL_SM0SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM0SEL23_SHIFT)) & PWM_DTSRCSEL_SM0SEL23_MASK)
/*! SM1SEL45 - Submodule 1 PWM45 Control Select
 *  0b00..Generated SM1PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT45] is used by the deadtime logic.
 *  0b11..PWM1_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM1SEL45_MASK               (0x30U)
#define PWM_DTSRCSEL_SM1SEL45_SHIFT              (4U)
#define PWM_DTSRCSEL_SM1SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL45_SHIFT)) & PWM_DTSRCSEL_SM1SEL45_MASK)
/*! SM1SEL23 - Submodule 1 PWM23 Control Select
 *  0b00..Generated SM1PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM1PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM1OUT23] is used by the deadtime logic.
 *  0b11..PWM1_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM1SEL23_MASK               (0xC0U)
#define PWM_DTSRCSEL_SM1SEL23_SHIFT              (6U)
#define PWM_DTSRCSEL_SM1SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM1SEL23_SHIFT)) & PWM_DTSRCSEL_SM1SEL23_MASK)
/*! SM2SEL45 - Submodule 2 PWM45 Control Select
 *  0b00..Generated SM2PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT45] is used by the deadtime logic.
 *  0b11..PWM2_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM2SEL45_MASK               (0x300U)
#define PWM_DTSRCSEL_SM2SEL45_SHIFT              (8U)
#define PWM_DTSRCSEL_SM2SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL45_SHIFT)) & PWM_DTSRCSEL_SM2SEL45_MASK)
/*! SM2SEL23 - Submodule 2 PWM23 Control Select
 *  0b00..Generated SM2PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM2PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM2OUT23] is used by the deadtime logic.
 *  0b11..PWM2_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM2SEL23_MASK               (0xC00U)
#define PWM_DTSRCSEL_SM2SEL23_SHIFT              (10U)
#define PWM_DTSRCSEL_SM2SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM2SEL23_SHIFT)) & PWM_DTSRCSEL_SM2SEL23_MASK)
/*! SM3SEL45 - Submodule 3 PWM45 Control Select
 *  0b00..Generated SM3PWM45 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM3PWM45 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM3OUT45] is used by the deadtime logic.
 *  0b11..PWM3_EXTB signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM3SEL45_MASK               (0x3000U)
#define PWM_DTSRCSEL_SM3SEL45_SHIFT              (12U)
#define PWM_DTSRCSEL_SM3SEL45(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM3SEL45_SHIFT)) & PWM_DTSRCSEL_SM3SEL45_MASK)
/*! SM3SEL23 - Submodule 3 PWM23 Control Select
 *  0b00..Generated SM3PWM23 signal is used by the deadtime logic.
 *  0b01..Inverted generated SM3PWM23 signal is used by the deadtime logic.
 *  0b10..SWCOUT[SM3OUT23] is used by the deadtime logic.
 *  0b11..PWM3_EXTA signal is used by the deadtime logic.
 */
#define PWM_DTSRCSEL_SM3SEL23_MASK               (0xC000U)
#define PWM_DTSRCSEL_SM3SEL23_SHIFT              (14U)
#define PWM_DTSRCSEL_SM3SEL23(x)                 (((uint16_t)(((uint16_t)(x)) << PWM_DTSRCSEL_SM3SEL23_SHIFT)) & PWM_DTSRCSEL_SM3SEL23_MASK)
/*! @} */

/*! @name MCTRL - Master Control Register */
/*! @{ */
/*! LDOK - Load Okay
 *  0b0000..Do not load new values.
 *  0b0001..Load prescaler, modulus, and PWM values of the corresponding submodule.
 */
#define PWM_MCTRL_LDOK_MASK                      (0xFU)
#define PWM_MCTRL_LDOK_SHIFT                     (0U)
#define PWM_MCTRL_LDOK(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_LDOK_SHIFT)) & PWM_MCTRL_LDOK_MASK)
#define PWM_MCTRL_CLDOK_MASK                     (0xF0U)
#define PWM_MCTRL_CLDOK_SHIFT                    (4U)
#define PWM_MCTRL_CLDOK(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_CLDOK_SHIFT)) & PWM_MCTRL_CLDOK_MASK)
/*! RUN - Run
 *  0b0000..PWM generator is disabled in the corresponding submodule.
 *  0b0001..PWM generator is enabled in the corresponding submodule.
 */
#define PWM_MCTRL_RUN_MASK                       (0xF00U)
#define PWM_MCTRL_RUN_SHIFT                      (8U)
#define PWM_MCTRL_RUN(x)                         (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_RUN_SHIFT)) & PWM_MCTRL_RUN_MASK)
/*! IPOL - Current Polarity
 *  0b0000..PWM23 is used to generate complementary PWM pair in the corresponding submodule.
 *  0b0001..PWM45 is used to generate complementary PWM pair in the corresponding submodule.
 */
#define PWM_MCTRL_IPOL_MASK                      (0xF000U)
#define PWM_MCTRL_IPOL_SHIFT                     (12U)
#define PWM_MCTRL_IPOL(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL_IPOL_SHIFT)) & PWM_MCTRL_IPOL_MASK)
/*! @} */

/*! @name MCTRL2 - Master Control 2 Register */
/*! @{ */
/*! MONPLL - Monitor PLL State
 *  0b00..Not locked. Do not monitor PLL operation. Resetting of the fractional delay block in case of PLL losing lock will be controlled by software.
 *  0b01..Not locked. Monitor PLL operation to automatically disable the fractional delay block when the PLL encounters problems.
 *  0b10..Locked. Do not monitor PLL operation. Resetting of the fractional delay block in case of PLL losing lock
 *        will be controlled by software. These bits are write protected until the next reset.
 *  0b11..Locked. Monitor PLL operation to automatically disable the fractional delay block when the PLL
 *        encounters problems. These bits are write protected until the next reset.
 */
#define PWM_MCTRL2_MONPLL_MASK                   (0x3U)
#define PWM_MCTRL2_MONPLL_SHIFT                  (0U)
#define PWM_MCTRL2_MONPLL(x)                     (((uint16_t)(((uint16_t)(x)) << PWM_MCTRL2_MONPLL_SHIFT)) & PWM_MCTRL2_MONPLL_MASK)
/*! @} */

/*! @name FCTRL0 - Fault Control Register */
/*! @{ */
/*! FIE - Fault Interrupt Enables
 *  0b0000..FAULTx CPU interrupt requests disabled.
 *  0b0001..FAULTx CPU interrupt requests enabled.
 */
#define PWM_FCTRL0_FIE_MASK                      (0xFU)
#define PWM_FCTRL0_FIE_SHIFT                     (0U)
#define PWM_FCTRL0_FIE(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL0_FIE_SHIFT)) & PWM_FCTRL0_FIE_MASK)
/*! FSAFE - Fault Safety Mode
 *  0b0000..Normal mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the
 *          start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without regard
 *          to the state of FSTS[FFPINx]. If neither FHALF nor FFULL is setm then the fault condition cannot be
 *          cleared. The PWM outputs disabled by this fault input will not be re-enabled until the actual FAULTx input
 *          signal de-asserts since the fault input will combinationally disable the PWM outputs (as programmed in
 *          DISMAPn).
 *  0b0001..Safe mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear and
 *          FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and
 *          FSTS[FFULL]. If neither FHLAF nor FFULL is set, then the fault condition cannot be cleared.
 */
#define PWM_FCTRL0_FSAFE_MASK                    (0xF0U)
#define PWM_FCTRL0_FSAFE_SHIFT                   (4U)
#define PWM_FCTRL0_FSAFE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL0_FSAFE_SHIFT)) & PWM_FCTRL0_FSAFE_MASK)
/*! FAUTO - Automatic Fault Clearing
 *  0b0000..Manual fault clearing. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear
 *          at the start of a half cycle or full cycle depending the states of FSTS[FHALF] and FSTS[FFULL]. If
 *          neither FFULL nor FHALF is set, then the fault condition cannot be cleared. This is further controlled by
 *          FCTRL[FSAFE].
 *  0b0001..Automatic fault clearing. PWM outputs disabled by this fault are enabled when FSTS[FFPINx] is clear at
 *          the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without
 *          regard to the state of FSTS[FFLAGx]. If neither FFULL nor FHALF is set, then the fault condition
 *          cannot be cleared.
 */
#define PWM_FCTRL0_FAUTO_MASK                    (0xF00U)
#define PWM_FCTRL0_FAUTO_SHIFT                   (8U)
#define PWM_FCTRL0_FAUTO(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL0_FAUTO_SHIFT)) & PWM_FCTRL0_FAUTO_MASK)
/*! FLVL - Fault Level
 *  0b0000..A logic 0 on the fault input indicates a fault condition.
 *  0b0001..A logic 1 on the fault input indicates a fault condition.
 */
#define PWM_FCTRL0_FLVL_MASK                     (0xF000U)
#define PWM_FCTRL0_FLVL_SHIFT                    (12U)
#define PWM_FCTRL0_FLVL(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL0_FLVL_SHIFT)) & PWM_FCTRL0_FLVL_MASK)
/*! @} */

/*! @name FSTS0 - Fault Status Register */
/*! @{ */
/*! FFLAG - Fault Flags
 *  0b0000..No fault on the FAULTx pin.
 *  0b0001..Fault on the FAULTx pin.
 */
#define PWM_FSTS0_FFLAG_MASK                     (0xFU)
#define PWM_FSTS0_FFLAG_SHIFT                    (0U)
#define PWM_FSTS0_FFLAG(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS0_FFLAG_SHIFT)) & PWM_FSTS0_FFLAG_MASK)
/*! FFULL - Full Cycle
 *  0b0000..PWM outputs are not re-enabled at the start of a full cycle
 *  0b0001..PWM outputs are re-enabled at the start of a full cycle
 */
#define PWM_FSTS0_FFULL_MASK                     (0xF0U)
#define PWM_FSTS0_FFULL_SHIFT                    (4U)
#define PWM_FSTS0_FFULL(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS0_FFULL_SHIFT)) & PWM_FSTS0_FFULL_MASK)
#define PWM_FSTS0_FFPIN_MASK                     (0xF00U)
#define PWM_FSTS0_FFPIN_SHIFT                    (8U)
#define PWM_FSTS0_FFPIN(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS0_FFPIN_SHIFT)) & PWM_FSTS0_FFPIN_MASK)
/*! FHALF - Half Cycle Fault Recovery
 *  0b0000..PWM outputs are not re-enabled at the start of a half cycle.
 *  0b0001..PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0).
 */
#define PWM_FSTS0_FHALF_MASK                     (0xF000U)
#define PWM_FSTS0_FHALF_SHIFT                    (12U)
#define PWM_FSTS0_FHALF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS0_FHALF_SHIFT)) & PWM_FSTS0_FHALF_MASK)
/*! @} */

/*! @name FFILT0 - Fault Filter Register */
/*! @{ */
#define PWM_FFILT0_FILT_PER_MASK                 (0xFFU)
#define PWM_FFILT0_FILT_PER_SHIFT                (0U)
#define PWM_FFILT0_FILT_PER(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_FFILT0_FILT_PER_SHIFT)) & PWM_FFILT0_FILT_PER_MASK)
#define PWM_FFILT0_FILT_CNT_MASK                 (0x700U)
#define PWM_FFILT0_FILT_CNT_SHIFT                (8U)
#define PWM_FFILT0_FILT_CNT(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_FFILT0_FILT_CNT_SHIFT)) & PWM_FFILT0_FILT_CNT_MASK)
/*! GSTR - Fault Glitch Stretch Enable
 *  0b0..Fault input glitch stretching is disabled.
 *  0b1..Input fault signals will be stretched to at least 2 IPBus clock cycles.
 */
#define PWM_FFILT0_GSTR_MASK                     (0x8000U)
#define PWM_FFILT0_GSTR_SHIFT                    (15U)
#define PWM_FFILT0_GSTR(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FFILT0_GSTR_SHIFT)) & PWM_FFILT0_GSTR_MASK)
/*! @} */

/*! @name FTST0 - Fault Test Register */
/*! @{ */
/*! FTEST - Fault Test
 *  0b0..No fault
 *  0b1..Cause a simulated fault
 */
#define PWM_FTST0_FTEST_MASK                     (0x1U)
#define PWM_FTST0_FTEST_SHIFT                    (0U)
#define PWM_FTST0_FTEST(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FTST0_FTEST_SHIFT)) & PWM_FTST0_FTEST_MASK)
/*! @} */

/*! @name FCTRL20 - Fault Control 2 Register */
/*! @{ */
/*! NOCOMB - No Combinational Path From Fault Input To PWM Output
 *  0b0000..There is a combinational link from the fault inputs to the PWM outputs. The fault inputs are combined
 *          with the filtered and latched fault signals to disable the PWM outputs.
 *  0b0001..The direct combinational path from the fault inputs to the PWM outputs is disabled and the filtered
 *          and latched fault signals are used to disable the PWM outputs.
 */
#define PWM_FCTRL20_NOCOMB_MASK                  (0xFU)
#define PWM_FCTRL20_NOCOMB_SHIFT                 (0U)
#define PWM_FCTRL20_NOCOMB(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL20_NOCOMB_SHIFT)) & PWM_FCTRL20_NOCOMB_MASK)
/*! @} */

/*! @name FCTRL1 - Fault Control Register */
/*! @{ */
/*! FIE - Fault Interrupt Enables
 *  0b0000..FAULTx CPU interrupt requests disabled.
 *  0b0001..FAULTx CPU interrupt requests enabled.
 */
#define PWM_FCTRL1_FIE_MASK                      (0xFU)
#define PWM_FCTRL1_FIE_SHIFT                     (0U)
#define PWM_FCTRL1_FIE(x)                        (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL1_FIE_SHIFT)) & PWM_FCTRL1_FIE_MASK)
/*! FSAFE - Fault Safety Mode
 *  0b0000..Normal mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the
 *          start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without regard
 *          to the state of FSTS[FFPINx]. If neither FHALF nor FFULL is setm then the fault condition cannot be
 *          cleared. The PWM outputs disabled by this fault input will not be re-enabled until the actual FAULTx input
 *          signal de-asserts since the fault input will combinationally disable the PWM outputs (as programmed in
 *          DISMAPn).
 *  0b0001..Safe mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear and
 *          FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and
 *          FSTS[FFULL]. If neither FHLAF nor FFULL is set, then the fault condition cannot be cleared.
 */
#define PWM_FCTRL1_FSAFE_MASK                    (0xF0U)
#define PWM_FCTRL1_FSAFE_SHIFT                   (4U)
#define PWM_FCTRL1_FSAFE(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL1_FSAFE_SHIFT)) & PWM_FCTRL1_FSAFE_MASK)
/*! FAUTO - Automatic Fault Clearing
 *  0b0000..Manual fault clearing. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear
 *          at the start of a half cycle or full cycle depending the states of FSTS[FHALF] and FSTS[FFULL]. If
 *          neither FFULL nor FHALF is set, then the fault condition cannot be cleared. This is further controlled by
 *          FCTRL[FSAFE].
 *  0b0001..Automatic fault clearing. PWM outputs disabled by this fault are enabled when FSTS[FFPINx] is clear at
 *          the start of a half cycle or full cycle depending on the states of FSTS[FHALF] and FSTS[FFULL] without
 *          regard to the state of FSTS[FFLAGx]. If neither FFULL nor FHALF is set, then the fault condition
 *          cannot be cleared.
 */
#define PWM_FCTRL1_FAUTO_MASK                    (0xF00U)
#define PWM_FCTRL1_FAUTO_SHIFT                   (8U)
#define PWM_FCTRL1_FAUTO(x)                      (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL1_FAUTO_SHIFT)) & PWM_FCTRL1_FAUTO_MASK)
/*! FLVL - Fault Level
 *  0b0000..A logic 0 on the fault input indicates a fault condition.
 *  0b0001..A logic 1 on the fault input indicates a fault condition.
 */
#define PWM_FCTRL1_FLVL_MASK                     (0xF000U)
#define PWM_FCTRL1_FLVL_SHIFT                    (12U)
#define PWM_FCTRL1_FLVL(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL1_FLVL_SHIFT)) & PWM_FCTRL1_FLVL_MASK)
/*! @} */

/*! @name FSTS1 - Fault Status Register */
/*! @{ */
/*! FFLAG - Fault Flags
 *  0b0000..No fault on the FAULTx pin.
 *  0b0001..Fault on the FAULTx pin.
 */
#define PWM_FSTS1_FFLAG_MASK                     (0xFU)
#define PWM_FSTS1_FFLAG_SHIFT                    (0U)
#define PWM_FSTS1_FFLAG(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS1_FFLAG_SHIFT)) & PWM_FSTS1_FFLAG_MASK)
/*! FFULL - Full Cycle
 *  0b0000..PWM outputs are not re-enabled at the start of a full cycle
 *  0b0001..PWM outputs are re-enabled at the start of a full cycle
 */
#define PWM_FSTS1_FFULL_MASK                     (0xF0U)
#define PWM_FSTS1_FFULL_SHIFT                    (4U)
#define PWM_FSTS1_FFULL(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS1_FFULL_SHIFT)) & PWM_FSTS1_FFULL_MASK)
#define PWM_FSTS1_FFPIN_MASK                     (0xF00U)
#define PWM_FSTS1_FFPIN_SHIFT                    (8U)
#define PWM_FSTS1_FFPIN(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS1_FFPIN_SHIFT)) & PWM_FSTS1_FFPIN_MASK)
/*! FHALF - Half Cycle Fault Recovery
 *  0b0000..PWM outputs are not re-enabled at the start of a half cycle.
 *  0b0001..PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0).
 */
#define PWM_FSTS1_FHALF_MASK                     (0xF000U)
#define PWM_FSTS1_FHALF_SHIFT                    (12U)
#define PWM_FSTS1_FHALF(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FSTS1_FHALF_SHIFT)) & PWM_FSTS1_FHALF_MASK)
/*! @} */

/*! @name FFILT1 - Fault Filter Register */
/*! @{ */
#define PWM_FFILT1_FILT_PER_MASK                 (0xFFU)
#define PWM_FFILT1_FILT_PER_SHIFT                (0U)
#define PWM_FFILT1_FILT_PER(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_FFILT1_FILT_PER_SHIFT)) & PWM_FFILT1_FILT_PER_MASK)
#define PWM_FFILT1_FILT_CNT_MASK                 (0x700U)
#define PWM_FFILT1_FILT_CNT_SHIFT                (8U)
#define PWM_FFILT1_FILT_CNT(x)                   (((uint16_t)(((uint16_t)(x)) << PWM_FFILT1_FILT_CNT_SHIFT)) & PWM_FFILT1_FILT_CNT_MASK)
/*! GSTR - Fault Glitch Stretch Enable
 *  0b0..Fault input glitch stretching is disabled.
 *  0b1..Input fault signals will be stretched to at least 2 IPBus clock cycles.
 */
#define PWM_FFILT1_GSTR_MASK                     (0x8000U)
#define PWM_FFILT1_GSTR_SHIFT                    (15U)
#define PWM_FFILT1_GSTR(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FFILT1_GSTR_SHIFT)) & PWM_FFILT1_GSTR_MASK)
/*! @} */

/*! @name FTST1 - Fault Test Register */
/*! @{ */
/*! FTEST - Fault Test
 *  0b0..No fault
 *  0b1..Cause a simulated fault
 */
#define PWM_FTST1_FTEST_MASK                     (0x1U)
#define PWM_FTST1_FTEST_SHIFT                    (0U)
#define PWM_FTST1_FTEST(x)                       (((uint16_t)(((uint16_t)(x)) << PWM_FTST1_FTEST_SHIFT)) & PWM_FTST1_FTEST_MASK)
/*! @} */

/*! @name FCTRL21 - Fault Control 2 Register */
/*! @{ */
/*! NOCOMB - No Combinational Path From Fault Input To PWM Output
 *  0b0000..There is a combinational link from the fault inputs to the PWM outputs. The fault inputs are combined
 *          with the filtered and latched fault signals to disable the PWM outputs.
 *  0b0001..The direct combinational path from the fault inputs to the PWM outputs is disabled and the filtered
 *          and latched fault signals are used to disable the PWM outputs.
 */
#define PWM_FCTRL21_NOCOMB_MASK                  (0xFU)
#define PWM_FCTRL21_NOCOMB_SHIFT                 (0U)
#define PWM_FCTRL21_NOCOMB(x)                    (((uint16_t)(((uint16_t)(x)) << PWM_FCTRL21_NOCOMB_SHIFT)) & PWM_FCTRL21_NOCOMB_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group PWM_Register_Masks */


/* PWM - Peripheral instance base addresses */
/** Peripheral PWMA base address */
#define PWMA_BASE                                (0xE600u)
/** Peripheral PWMA base pointer */
#define PWMA                                     ((PWM_Type *)PWMA_BASE)
/** Peripheral PWMB base address */
#define PWMB_BASE                                (0xE700u)
/** Peripheral PWMB base pointer */
#define PWMB                                     ((PWM_Type *)PWMB_BASE)
/** Array initializer of PWM peripheral base addresses */
#define PWM_BASE_ADDRS                           { PWMA_BASE, PWMB_BASE }
/** Array initializer of PWM peripheral base pointers */
#define PWM_BASE_PTRS                            { PWMA, PWMB }

/*!
 * @}
 */ /* end of group PWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- QSCI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Peripheral_Access_Layer QSCI Peripheral Access Layer
 * @{
 */

/** QSCI - Register Layout Typedef */
typedef struct {
  __IO uint16_t RATE;                              /**< QSCI Baud Rate Register, offset: 0x0 */
  __IO uint16_t CTRL1;                             /**< QSCI Control Register 1, offset: 0x1 */
  __IO uint16_t CTRL2;                             /**< QSCI Control Register 2, offset: 0x2 */
  __I  uint16_t STAT;                              /**< QSCI Status Register, offset: 0x3 */
  __IO uint16_t DATA;                              /**< QSCI Data Register, offset: 0x4 */
  __IO uint16_t CTRL3;                             /**< QSCI Control Register 3, offset: 0x5 */
} QSCI_Type;

/* ----------------------------------------------------------------------------
   -- QSCI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSCI_Register_Masks QSCI Register Masks
 * @{
 */

/*! @name RATE - QSCI Baud Rate Register */
/*! @{ */
#define QSCI_RATE_FRAC_SBR_MASK                  (0x7U)
#define QSCI_RATE_FRAC_SBR_SHIFT                 (0U)
#define QSCI_RATE_FRAC_SBR(x)                    (((uint16_t)(((uint16_t)(x)) << QSCI_RATE_FRAC_SBR_SHIFT)) & QSCI_RATE_FRAC_SBR_MASK)
#define QSCI_RATE_SBRL_MASK                      (0xFFF8U)
#define QSCI_RATE_SBRL_SHIFT                     (3U)
#define QSCI_RATE_SBRL(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_RATE_SBRL_SHIFT)) & QSCI_RATE_SBRL_MASK)
/*! @} */

/*! @name CTRL1 - QSCI Control Register 1 */
/*! @{ */
/*! SBK - Send Break
 *  0b0..No break characters
 *  0b1..Transmit break characters
 */
#define QSCI_CTRL1_SBK_MASK                      (0x1U)
#define QSCI_CTRL1_SBK_SHIFT                     (0U)
#define QSCI_CTRL1_SBK(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_SBK_SHIFT)) & QSCI_CTRL1_SBK_MASK)
/*! RWU - Receiver Wake-up
 *  0b0..Normal operation
 *  0b1..Standby state
 */
#define QSCI_CTRL1_RWU_MASK                      (0x2U)
#define QSCI_CTRL1_RWU_SHIFT                     (1U)
#define QSCI_CTRL1_RWU(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RWU_SHIFT)) & QSCI_CTRL1_RWU_MASK)
/*! RE - Receiver Enable
 *  0b0..Receiver disabled
 *  0b1..Receiver enabled
 */
#define QSCI_CTRL1_RE_MASK                       (0x4U)
#define QSCI_CTRL1_RE_SHIFT                      (2U)
#define QSCI_CTRL1_RE(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RE_SHIFT)) & QSCI_CTRL1_RE_MASK)
/*! TE - Transmitter Enable
 *  0b0..Transmitter disabled
 *  0b1..Transmitter enabled
 */
#define QSCI_CTRL1_TE_MASK                       (0x8U)
#define QSCI_CTRL1_TE_SHIFT                      (3U)
#define QSCI_CTRL1_TE(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TE_SHIFT)) & QSCI_CTRL1_TE_MASK)
/*! REIE - Receive Error Interrupt Enable
 *  0b0..Error interrupt requests disabled
 *  0b1..Error interrupt requests enabled
 */
#define QSCI_CTRL1_REIE_MASK                     (0x10U)
#define QSCI_CTRL1_REIE_SHIFT                    (4U)
#define QSCI_CTRL1_REIE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_REIE_SHIFT)) & QSCI_CTRL1_REIE_MASK)
/*! RFIE - Receiver Full Interrupt Enable
 *  0b0..STAT[RDRF] and STAT[OR] interrupt requests disabled
 *  0b1..STAT[RDRF] and STAT[OR] interrupt requests enabled
 */
#define QSCI_CTRL1_RFIE_MASK                     (0x20U)
#define QSCI_CTRL1_RFIE_SHIFT                    (5U)
#define QSCI_CTRL1_RFIE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RFIE_SHIFT)) & QSCI_CTRL1_RFIE_MASK)
/*! TIIE - Transmitter Idle Interrupt Enable
 *  0b0..STAT[TIDLE] interrupt requests disabled
 *  0b1..STAT[TIDLE] interrupt requests enabled
 */
#define QSCI_CTRL1_TIIE_MASK                     (0x40U)
#define QSCI_CTRL1_TIIE_SHIFT                    (6U)
#define QSCI_CTRL1_TIIE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TIIE_SHIFT)) & QSCI_CTRL1_TIIE_MASK)
/*! TEIE - Transmitter Empty Interrupt Enable
 *  0b0..STAT[TDRE] interrupt requests disabled
 *  0b1..STAT[TDRE] interrupt requests enabled
 */
#define QSCI_CTRL1_TEIE_MASK                     (0x80U)
#define QSCI_CTRL1_TEIE_SHIFT                    (7U)
#define QSCI_CTRL1_TEIE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_TEIE_SHIFT)) & QSCI_CTRL1_TEIE_MASK)
/*! PT - Parity Type
 *  0b0..Even parity
 *  0b1..Odd parity
 */
#define QSCI_CTRL1_PT_MASK                       (0x100U)
#define QSCI_CTRL1_PT_SHIFT                      (8U)
#define QSCI_CTRL1_PT(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_PT_SHIFT)) & QSCI_CTRL1_PT_MASK)
/*! PE - Parity Enable
 *  0b0..Parity function disabled
 *  0b1..Parity function enabled
 */
#define QSCI_CTRL1_PE_MASK                       (0x200U)
#define QSCI_CTRL1_PE_SHIFT                      (9U)
#define QSCI_CTRL1_PE(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_PE_SHIFT)) & QSCI_CTRL1_PE_MASK)
/*! POL - Polarity
 *  0b0..Don't invert transmit and receive data bits (normal mode)
 *  0b1..Invert transmit and receive data bits (inverted mode)
 */
#define QSCI_CTRL1_POL_MASK                      (0x400U)
#define QSCI_CTRL1_POL_SHIFT                     (10U)
#define QSCI_CTRL1_POL(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_POL_SHIFT)) & QSCI_CTRL1_POL_MASK)
/*! WAKE - Wake-up Condition
 *  0b0..Idle line wake-up
 *  0b1..Address mark wake-up
 */
#define QSCI_CTRL1_WAKE_MASK                     (0x800U)
#define QSCI_CTRL1_WAKE_SHIFT                    (11U)
#define QSCI_CTRL1_WAKE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_WAKE_SHIFT)) & QSCI_CTRL1_WAKE_MASK)
/*! M - Data Format Mode
 *  0b0..One start bit, eight data bits, one stop bit
 *  0b1..One start bit, nine data bits, one stop bit
 */
#define QSCI_CTRL1_M_MASK                        (0x1000U)
#define QSCI_CTRL1_M_SHIFT                       (12U)
#define QSCI_CTRL1_M(x)                          (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_M_SHIFT)) & QSCI_CTRL1_M_MASK)
/*! RSRC - Receiver Source
 *  0b0..Receiver input internally connected to transmitter output
 *  0b1..Receiver input connected to TXD pin
 */
#define QSCI_CTRL1_RSRC_MASK                     (0x2000U)
#define QSCI_CTRL1_RSRC_SHIFT                    (13U)
#define QSCI_CTRL1_RSRC(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_RSRC_SHIFT)) & QSCI_CTRL1_RSRC_MASK)
/*! SWAI - Stop in Wait Mode
 *  0b0..SCI enabled in wait mode
 *  0b1..SCI disabled in wait mode
 */
#define QSCI_CTRL1_SWAI_MASK                     (0x4000U)
#define QSCI_CTRL1_SWAI_SHIFT                    (14U)
#define QSCI_CTRL1_SWAI(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_SWAI_SHIFT)) & QSCI_CTRL1_SWAI_MASK)
/*! LOOP - Loop Select
 *  0b0..Normal operation, regardless of the value of RSRC
 *  0b1..When RSRC = 0: Loop mode with internal TXD fed back to RXD
 *  0b1..When RSRC = 1: Single-wire mode with TXD output fed back to RXD
 */
#define QSCI_CTRL1_LOOP_MASK                     (0x8000U)
#define QSCI_CTRL1_LOOP_SHIFT                    (15U)
#define QSCI_CTRL1_LOOP(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL1_LOOP_SHIFT)) & QSCI_CTRL1_LOOP_MASK)
/*! @} */

/*! @name CTRL2 - QSCI Control Register 2 */
/*! @{ */
/*! RDE - Receiver DMA Enable
 *  0b0..Receive DMA disabled
 *  0b1..Receive DMA enabled
 */
#define QSCI_CTRL2_RDE_MASK                      (0x1U)
#define QSCI_CTRL2_RDE_SHIFT                     (0U)
#define QSCI_CTRL2_RDE(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RDE_SHIFT)) & QSCI_CTRL2_RDE_MASK)
/*! TDE - Transmitter DMA Enable
 *  0b0..Transmit DMA disabled
 *  0b1..Transmit DMA enabled
 */
#define QSCI_CTRL2_TDE_MASK                      (0x2U)
#define QSCI_CTRL2_TDE_SHIFT                     (1U)
#define QSCI_CTRL2_TDE(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TDE_SHIFT)) & QSCI_CTRL2_TDE_MASK)
#define QSCI_CTRL2_RIIE_MASK                     (0x4U)
#define QSCI_CTRL2_RIIE_SHIFT                    (2U)
#define QSCI_CTRL2_RIIE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RIIE_SHIFT)) & QSCI_CTRL2_RIIE_MASK)
/*! LINMODE - Enable LIN Slave Mode
 *  0b0..The LIN auto baud feature is disabled and the RATE register maintains whatever value the processor writes to it.
 *  0b1..Enable LIN slave functionality. This includes a search for the break character followed by a sync
 *       character (0x55) from the master LIN device. When the break is detected (11 consecutive samples of zero), the
 *       subsequent sync character is used to measure the baud rate of the transmitting master, and the RATE register
 *       is automatically reloaded with the value needed to "match" that baud rate.
 */
#define QSCI_CTRL2_LINMODE_MASK                  (0x8U)
#define QSCI_CTRL2_LINMODE_SHIFT                 (3U)
#define QSCI_CTRL2_LINMODE(x)                    (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_LINMODE_SHIFT)) & QSCI_CTRL2_LINMODE_MASK)
/*! RIEIE - Receiver Input Edge Interrupt Enable
 *  0b0..Receiver input edge interrupt request disabled.
 *  0b1..Receiver input edge interrupt request enabled.
 */
#define QSCI_CTRL2_RIEIE_MASK                    (0x10U)
#define QSCI_CTRL2_RIEIE_SHIFT                   (4U)
#define QSCI_CTRL2_RIEIE(x)                      (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RIEIE_SHIFT)) & QSCI_CTRL2_RIEIE_MASK)
/*! FIFO_EN - FIFO Enable
 *  0b0..FIFOs are disabled.
 *  0b1..FIFOs are enabled.
 */
#define QSCI_CTRL2_FIFO_EN_MASK                  (0x20U)
#define QSCI_CTRL2_FIFO_EN_SHIFT                 (5U)
#define QSCI_CTRL2_FIFO_EN(x)                    (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_FIFO_EN_SHIFT)) & QSCI_CTRL2_FIFO_EN_MASK)
/*! RFWM - Receive FIFO Full Water Mark
 *  0b00..RDRF is set when at least 1 word is in the FIFO
 *  0b01..RDRF is set when at least 2 words are in the FIFO
 *  0b10..RDRF is set when at least 3 words are in the FIFO
 *  0b11..RDRF is set when at least 4 words are in the FIFO
 */
#define QSCI_CTRL2_RFWM_MASK                     (0xC0U)
#define QSCI_CTRL2_RFWM_SHIFT                    (6U)
#define QSCI_CTRL2_RFWM(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RFWM_SHIFT)) & QSCI_CTRL2_RFWM_MASK)
/*! RFCNT - Receive FIFO Count
 *  0b000..0 words in RX FIFO
 *  0b001..1 word in RX FIFO
 *  0b010..2 words in RX FIFO
 *  0b011..3 words in RX FIFO
 *  0b100..4 words in RX FIFO
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define QSCI_CTRL2_RFCNT_MASK                    (0x700U)
#define QSCI_CTRL2_RFCNT_SHIFT                   (8U)
#define QSCI_CTRL2_RFCNT(x)                      (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_RFCNT_SHIFT)) & QSCI_CTRL2_RFCNT_MASK)
/*! TFWM - Transmit FIFO Empty Water Mark
 *  0b00..TDRE is set when 0 words are in the FIFO
 *  0b01..TDRE is set when 1 or fewer words are in the FIFO
 *  0b10..TDRE is set when 2 or fewer words are in the FIFO
 *  0b11..TDRE is set when 3 or fewer words are in the FIFO
 */
#define QSCI_CTRL2_TFWM_MASK                     (0x1800U)
#define QSCI_CTRL2_TFWM_SHIFT                    (11U)
#define QSCI_CTRL2_TFWM(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TFWM_SHIFT)) & QSCI_CTRL2_TFWM_MASK)
/*! TFCNT - Transmit FIFO Count
 *  0b000..0 words in Tx FIFO
 *  0b001..1 word in Tx FIFO
 *  0b010..2 words in Tx FIFO
 *  0b011..3 words in Tx FIFO
 *  0b100..4 words in Tx FIFO
 *  0b101..Reserved
 *  0b110..Reserved
 *  0b111..Reserved
 */
#define QSCI_CTRL2_TFCNT_MASK                    (0xE000U)
#define QSCI_CTRL2_TFCNT_SHIFT                   (13U)
#define QSCI_CTRL2_TFCNT(x)                      (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL2_TFCNT_SHIFT)) & QSCI_CTRL2_TFCNT_MASK)
/*! @} */

/*! @name STAT - QSCI Status Register */
/*! @{ */
/*! RAF - Receiver Active Flag
 *  0b0..No reception in progress
 *  0b1..Reception in progress
 */
#define QSCI_STAT_RAF_MASK                       (0x1U)
#define QSCI_STAT_RAF_SHIFT                      (0U)
#define QSCI_STAT_RAF(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RAF_SHIFT)) & QSCI_STAT_RAF_MASK)
/*! RDMA - Receive DMA Request
 *  0b0..Either CTRL2[RDE] is cleared or CTRL2[RDE] is set and CTRL2[RFCNT] is 0.
 *  0b1..CTRL2[RDE] is set and CTRL2[RFCNT] is currently above 0.
 */
#define QSCI_STAT_RDMA_MASK                      (0x2U)
#define QSCI_STAT_RDMA_SHIFT                     (1U)
#define QSCI_STAT_RDMA(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RDMA_SHIFT)) & QSCI_STAT_RDMA_MASK)
/*! TDMA - Transmit DMA Request
 *  0b0..Either CTRL2[TDE] is cleared or CTRL2[TDE] is set and CTRL2[TFCNT] is at its maximum value.
 *  0b1..CTRL2[TDE] is set and CTRL2[TFCNT] is currently below its maximum value.
 */
#define QSCI_STAT_TDMA_MASK                      (0x4U)
#define QSCI_STAT_TDMA_SHIFT                     (2U)
#define QSCI_STAT_TDMA(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TDMA_SHIFT)) & QSCI_STAT_TDMA_MASK)
/*! LSE - LIN Sync Error
 *  0b0..No error occurred since CTRL2[LINMODE] was enabled or the bit was last cleared
 *  0b1..A sync error prevented loading of the RATE register with a revised value after the break was detected.
 */
#define QSCI_STAT_LSE_MASK                       (0x8U)
#define QSCI_STAT_LSE_SHIFT                      (3U)
#define QSCI_STAT_LSE(x)                         (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_LSE_SHIFT)) & QSCI_STAT_LSE_MASK)
/*! RIEF - Receiver Input Edge Flag
 *  0b0..No active edge on the receive pin has occured.
 *  0b1..An active edge on the receive pin has occured.
 */
#define QSCI_STAT_RIEF_MASK                      (0x10U)
#define QSCI_STAT_RIEF_SHIFT                     (4U)
#define QSCI_STAT_RIEF(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RIEF_SHIFT)) & QSCI_STAT_RIEF_MASK)
/*! PF - Parity Error Flag
 *  0b0..No parity error
 *  0b1..Parity error
 */
#define QSCI_STAT_PF_MASK                        (0x100U)
#define QSCI_STAT_PF_SHIFT                       (8U)
#define QSCI_STAT_PF(x)                          (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_PF_SHIFT)) & QSCI_STAT_PF_MASK)
/*! FE - Framing Error Flag
 *  0b0..No framing error
 *  0b1..Framing error
 */
#define QSCI_STAT_FE_MASK                        (0x200U)
#define QSCI_STAT_FE_SHIFT                       (9U)
#define QSCI_STAT_FE(x)                          (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_FE_SHIFT)) & QSCI_STAT_FE_MASK)
/*! NF - Noise Flag
 *  0b0..No noise
 *  0b1..Noise
 */
#define QSCI_STAT_NF_MASK                        (0x400U)
#define QSCI_STAT_NF_SHIFT                       (10U)
#define QSCI_STAT_NF(x)                          (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_NF_SHIFT)) & QSCI_STAT_NF_MASK)
/*! OR - Overrun Flag
 *  0b0..No overrun
 *  0b1..Overrun
 */
#define QSCI_STAT_OR_MASK                        (0x800U)
#define QSCI_STAT_OR_SHIFT                       (11U)
#define QSCI_STAT_OR(x)                          (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_OR_SHIFT)) & QSCI_STAT_OR_MASK)
/*! RIDLE - Receiver Idle Line Flag
 *  0b0..Receiver input is either active now or has never become active since RIDLE was last cleared
 *  0b1..Receiver input has become idle (after receiving a valid frame)
 */
#define QSCI_STAT_RIDLE_MASK                     (0x1000U)
#define QSCI_STAT_RIDLE_SHIFT                    (12U)
#define QSCI_STAT_RIDLE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RIDLE_SHIFT)) & QSCI_STAT_RIDLE_MASK)
/*! RDRF - Receive Data Register Full Flag
 *  0b0..RX FIFO word count is at or below watermark
 *  0b1..RX FIFO word count is above watermark
 */
#define QSCI_STAT_RDRF_MASK                      (0x2000U)
#define QSCI_STAT_RDRF_SHIFT                     (13U)
#define QSCI_STAT_RDRF(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_RDRF_SHIFT)) & QSCI_STAT_RDRF_MASK)
/*! TIDLE - Transmitter Idle Flag
 *  0b0..Transmission in progress
 *  0b1..No transmission in progress
 */
#define QSCI_STAT_TIDLE_MASK                     (0x4000U)
#define QSCI_STAT_TIDLE_SHIFT                    (14U)
#define QSCI_STAT_TIDLE(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TIDLE_SHIFT)) & QSCI_STAT_TIDLE_MASK)
/*! TDRE - Transmit Data Register Empty Flag
 *  0b0..TX FIFO word count is above watermark
 *  0b1..TX FIFO word count is at or below watermark
 */
#define QSCI_STAT_TDRE_MASK                      (0x8000U)
#define QSCI_STAT_TDRE_SHIFT                     (15U)
#define QSCI_STAT_TDRE(x)                        (((uint16_t)(((uint16_t)(x)) << QSCI_STAT_TDRE_SHIFT)) & QSCI_STAT_TDRE_MASK)
/*! @} */

/*! @name DATA - QSCI Data Register */
/*! @{ */
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK     (0x1FFU)
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT    (0U)
#define QSCI_DATA_RECEIVE_TRANSMIT_DATA(x)       (((uint16_t)(((uint16_t)(x)) << QSCI_DATA_RECEIVE_TRANSMIT_DATA_SHIFT)) & QSCI_DATA_RECEIVE_TRANSMIT_DATA_MASK)
/*! @} */

/*! @name CTRL3 - QSCI Control Register 3 */
/*! @{ */
/*! SHEN - Stop mode entry hold off
 *  0b0..Stop mode hold off is disabled.
 *  0b1..Stop mode holdoff is enabled.
 */
#define QSCI_CTRL3_SHEN_MASK                     (0x1U)
#define QSCI_CTRL3_SHEN_SHIFT                    (0U)
#define QSCI_CTRL3_SHEN(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL3_SHEN_SHIFT)) & QSCI_CTRL3_SHEN_MASK)
#define QSCI_CTRL3_SBRH_MASK                     (0xE000U)
#define QSCI_CTRL3_SBRH_SHIFT                    (13U)
#define QSCI_CTRL3_SBRH(x)                       (((uint16_t)(((uint16_t)(x)) << QSCI_CTRL3_SBRH_SHIFT)) & QSCI_CTRL3_SBRH_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group QSCI_Register_Masks */


/* QSCI - Peripheral instance base addresses */
/** Peripheral QSCI0 base address */
#define QSCI0_BASE                               (0xE080u)
/** Peripheral QSCI0 base pointer */
#define QSCI0                                    ((QSCI_Type *)QSCI0_BASE)
/** Peripheral QSCI1 base address */
#define QSCI1_BASE                               (0xE090u)
/** Peripheral QSCI1 base pointer */
#define QSCI1                                    ((QSCI_Type *)QSCI1_BASE)
/** Array initializer of QSCI peripheral base addresses */
#define QSCI_BASE_ADDRS                          { QSCI0_BASE, QSCI1_BASE }
/** Array initializer of QSCI peripheral base pointers */
#define QSCI_BASE_PTRS                           { QSCI0, QSCI1 }

/*!
 * @}
 */ /* end of group QSCI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- QSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Peripheral_Access_Layer QSPI Peripheral Access Layer
 * @{
 */

/** QSPI - Register Layout Typedef */
typedef struct {
  __IO uint16_t SPSCR;                             /**< SPI Status and Control Register, offset: 0x0 */
  __IO uint16_t SPDSR;                             /**< SPI Data Size and Control Register, offset: 0x1 */
  __I  uint16_t SPDRR;                             /**< SPI Data Receive Register, offset: 0x2 */
  __O  uint16_t SPDTR;                             /**< SPI Data Transmit Register, offset: 0x3 */
  __IO uint16_t SPFIFO;                            /**< SPI FIFO Control Register, offset: 0x4 */
  __IO uint16_t SPWAIT;                            /**< SPI Word Delay Register, offset: 0x5 */
  __IO uint16_t SPCTL2;                            /**< SPI Control Register 2, offset: 0x6 */
} QSPI_Type;

/* ----------------------------------------------------------------------------
   -- QSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QSPI_Register_Masks QSPI Register Masks
 * @{
 */

/*! @name SPSCR - SPI Status and Control Register */
/*! @{ */
/*! SPTE - SPI Transmitter Empty
 *  0b0..Transmit data register or FIFO is not empty. (If using the FIFO, read TFCNT to determine how many words can be written safely.)
 *  0b1..Transmit data register or FIFO is empty.
 */
#define QSPI_SPSCR_SPTE_MASK                     (0x1U)
#define QSPI_SPSCR_SPTE_SHIFT                    (0U)
#define QSPI_SPSCR_SPTE(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPTE_SHIFT)) & QSPI_SPSCR_SPTE_MASK)
/*! MODF - Mode Fault
 *  0b0..SS_B pin at appropriate logic level
 *  0b1..SS_B pin at inappropriate logic level
 */
#define QSPI_SPSCR_MODF_MASK                     (0x2U)
#define QSPI_SPSCR_MODF_SHIFT                    (1U)
#define QSPI_SPSCR_MODF(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_MODF_SHIFT)) & QSPI_SPSCR_MODF_MASK)
/*! OVRF - Overflow
 *  0b0..No overflow
 *  0b1..Overflow
 */
#define QSPI_SPSCR_OVRF_MASK                     (0x4U)
#define QSPI_SPSCR_OVRF_SHIFT                    (2U)
#define QSPI_SPSCR_OVRF(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_OVRF_SHIFT)) & QSPI_SPSCR_OVRF_MASK)
/*! SPRF - SPI Receiver Full
 *  0b0..Receive data register or FIFO is not full. (If using the FIFO, read RFCNT to determine the number of valid words available.)
 *  0b1..Receive data register or FIFO is full.
 */
#define QSPI_SPSCR_SPRF_MASK                     (0x8U)
#define QSPI_SPSCR_SPRF_SHIFT                    (3U)
#define QSPI_SPSCR_SPRF(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPRF_SHIFT)) & QSPI_SPSCR_SPRF_MASK)
/*! SPTIE - Transmit Interrupt Enable
 *  0b0..SPTE interrupt requests disabled
 *  0b1..SPTE interrupt requests enabled
 */
#define QSPI_SPSCR_SPTIE_MASK                    (0x10U)
#define QSPI_SPSCR_SPTIE_SHIFT                   (4U)
#define QSPI_SPSCR_SPTIE(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPTIE_SHIFT)) & QSPI_SPSCR_SPTIE_MASK)
/*! SPE - SPI Enable
 *  0b0..SPI module disabled
 *  0b1..SPI module enabled
 */
#define QSPI_SPSCR_SPE_MASK                      (0x20U)
#define QSPI_SPSCR_SPE_SHIFT                     (5U)
#define QSPI_SPSCR_SPE(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPE_SHIFT)) & QSPI_SPSCR_SPE_MASK)
#define QSPI_SPSCR_CPHA_MASK                     (0x40U)
#define QSPI_SPSCR_CPHA_SHIFT                    (6U)
#define QSPI_SPSCR_CPHA(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_CPHA_SHIFT)) & QSPI_SPSCR_CPHA_MASK)
/*! CPOL - Clock Polarity
 *  0b0..Rising edge of SCLK starts transaction
 *  0b1..Falling edge of SCLK starts transaction
 */
#define QSPI_SPSCR_CPOL_MASK                     (0x80U)
#define QSPI_SPSCR_CPOL_SHIFT                    (7U)
#define QSPI_SPSCR_CPOL(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_CPOL_SHIFT)) & QSPI_SPSCR_CPOL_MASK)
/*! SPMSTR - SPI Master
 *  0b0..Slave mode
 *  0b1..Master mode
 */
#define QSPI_SPSCR_SPMSTR_MASK                   (0x100U)
#define QSPI_SPSCR_SPMSTR_SHIFT                  (8U)
#define QSPI_SPSCR_SPMSTR(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPMSTR_SHIFT)) & QSPI_SPSCR_SPMSTR_MASK)
/*! SPRIE - SPI Receiver Interrupt Enable
 *  0b0..SPRF interrupt requests disabled
 *  0b1..SPRF interrupt requests enabled
 */
#define QSPI_SPSCR_SPRIE_MASK                    (0x200U)
#define QSPI_SPSCR_SPRIE_SHIFT                   (9U)
#define QSPI_SPSCR_SPRIE(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPRIE_SHIFT)) & QSPI_SPSCR_SPRIE_MASK)
#define QSPI_SPSCR_MODFEN_MASK                   (0x400U)
#define QSPI_SPSCR_MODFEN_SHIFT                  (10U)
#define QSPI_SPSCR_MODFEN(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_MODFEN_SHIFT)) & QSPI_SPSCR_MODFEN_MASK)
/*! ERRIE - Error Interrupt Enable
 *  0b0..MODF and OVRF cannot generate device interrupt requests
 *  0b1..MODF and OVRF can generate device interrupt requests
 */
#define QSPI_SPSCR_ERRIE_MASK                    (0x800U)
#define QSPI_SPSCR_ERRIE_SHIFT                   (11U)
#define QSPI_SPSCR_ERRIE(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_ERRIE_SHIFT)) & QSPI_SPSCR_ERRIE_MASK)
/*! DSO - Data Shift Order
 *  0b0..MSB transmitted first (MSB -> LSB)
 *  0b1..LSB transmitted first (LSB -> MSB)
 */
#define QSPI_SPSCR_DSO_MASK                      (0x1000U)
#define QSPI_SPSCR_DSO_SHIFT                     (12U)
#define QSPI_SPSCR_DSO(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_DSO_SHIFT)) & QSPI_SPSCR_DSO_MASK)
/*! SPR - SPI Baud Rate Select
 *  0b000..BD = 2 when SPR3 = 0, BD = 512 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b001..BD = 4 when SPR3 = 0, BD = 1024 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b010..BD = 8 when SPR3 = 0, BD = 2048 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b011..BD = 16 when SPR3 = 0, BD = 4096 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b100..BD = 32 when SPR3 = 0, BD = 8192 when SPR3 = 1 (double BD when BD2X = 1)
 *  0b101..BD = 64 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 *  0b110..BD = 128 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 *  0b111..BD = 256 when SPR3 = 0 (double BD when BD2X = 1), BD = 16384 when SPR3 = 1 (regardless of BD2X)
 */
#define QSPI_SPSCR_SPR_MASK                      (0xE000U)
#define QSPI_SPSCR_SPR_SHIFT                     (13U)
#define QSPI_SPSCR_SPR(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPSCR_SPR_SHIFT)) & QSPI_SPSCR_SPR_MASK)
/*! @} */

/*! @name SPDSR - SPI Data Size and Control Register */
/*! @{ */
/*! DS - Transaction data size
 *  0b0000..Not allowed
 *  0b0001..2 bits transaction data size
 *  0b0010..3 bits transaction data size
 *  0b0011..4 bits transaction data size
 *  0b0100..5 bits transaction data size
 *  0b0101..6 bits transaction data size
 *  0b0110..7 bits transaction data size
 *  0b0111..8 bits transaction data size
 *  0b1000..9 bits transaction data size
 *  0b1001..10 bits transaction data size
 *  0b1010..11 bits transaction data size
 *  0b1011..12 bits transaction data size
 *  0b1100..13 bits transaction data size
 *  0b1101..14 bits transaction data size
 *  0b1110..15 bits transaction data size
 *  0b1111..16 bits transaction data size
 */
#define QSPI_SPDSR_DS_MASK                       (0xFU)
#define QSPI_SPDSR_DS_SHIFT                      (0U)
#define QSPI_SPDSR_DS(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_DS_SHIFT)) & QSPI_SPDSR_DS_MASK)
#define QSPI_SPDSR_SPR3_MASK                     (0x10U)
#define QSPI_SPDSR_SPR3_SHIFT                    (4U)
#define QSPI_SPDSR_SPR3(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SPR3_SHIFT)) & QSPI_SPDSR_SPR3_MASK)
/*! SSB_OVER - SS_B Override
 *  0b0..SS_B internal module input is selected to be connected to a GPIO pin.
 *  0b1..SS_B internal module input is selected to be equal to SPMSTR.
 */
#define QSPI_SPDSR_SSB_OVER_MASK                 (0x20U)
#define QSPI_SPDSR_SSB_OVER_SHIFT                (5U)
#define QSPI_SPDSR_SSB_OVER(x)                   (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_OVER_SHIFT)) & QSPI_SPDSR_SSB_OVER_MASK)
/*! SSB_STRB - SS_B Strobe Mode
 *  0b0..No SS_B pulse between words.
 *  0b1..SS_B output signal is pulsed high between words. This adds 1.5 baud clocks to the total word period. The
 *       idle state of SS_B is low unless SSB_AUTO is high and then the idle state is high. Do not use if MODFEN =
 *       1.
 */
#define QSPI_SPDSR_SSB_STRB_MASK                 (0x40U)
#define QSPI_SPDSR_SSB_STRB_SHIFT                (6U)
#define QSPI_SPDSR_SSB_STRB(x)                   (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_STRB_SHIFT)) & QSPI_SPDSR_SSB_STRB_MASK)
/*! SSB_DDR - SS_B Data Direction
 *  0b0..SS_B is configured as an input pin. Use this setting in slave mode or in master mode with MODFEN=1.
 *  0b1..SS_B is configured as an output pin. Use this setting in master mode with MODFEN=0.
 */
#define QSPI_SPDSR_SSB_DDR_MASK                  (0x80U)
#define QSPI_SPDSR_SSB_DDR_SHIFT                 (7U)
#define QSPI_SPDSR_SSB_DDR(x)                    (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_DDR_SHIFT)) & QSPI_SPDSR_SSB_DDR_MASK)
/*! SSB_AUTO - SS_B Automatic Mode
 *  0b0..SS_B output signal is software generated by directly manipulating the various bits in this register or
 *       the GPIO registers (compatible with legacy SPI software).
 *  0b1..SS_B output signal is hardware generated to create the initial falling edge and final rising edge. The
 *       idle state of the SS_B is high. Do not use if MODFEN = 1.
 */
#define QSPI_SPDSR_SSB_AUTO_MASK                 (0x100U)
#define QSPI_SPDSR_SSB_AUTO_SHIFT                (8U)
#define QSPI_SPDSR_SSB_AUTO(x)                   (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_AUTO_SHIFT)) & QSPI_SPDSR_SSB_AUTO_MASK)
/*! SSB_ODM - SS_B Open Drain Mode
 *  0b0..SS_B is configured for high and low drive. This mode is generally used in single master systems.
 *  0b1..SS_B is configured as an open drain pin (only drives low output level). This mode is useful for multiple master systems.
 */
#define QSPI_SPDSR_SSB_ODM_MASK                  (0x200U)
#define QSPI_SPDSR_SSB_ODM_SHIFT                 (9U)
#define QSPI_SPDSR_SSB_ODM(x)                    (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_ODM_SHIFT)) & QSPI_SPDSR_SSB_ODM_MASK)
/*! SSB_DATA - SS_B Data
 *  0b0..SS_B pin is driven low if SSB_DDR=1
 *  0b1..SS_B pin is driven high if SSB_DDR=1
 */
#define QSPI_SPDSR_SSB_DATA_MASK                 (0x400U)
#define QSPI_SPDSR_SSB_DATA_SHIFT                (10U)
#define QSPI_SPDSR_SSB_DATA(x)                   (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_DATA_SHIFT)) & QSPI_SPDSR_SSB_DATA_MASK)
#define QSPI_SPDSR_SSB_IN_MASK                   (0x800U)
#define QSPI_SPDSR_SSB_IN_SHIFT                  (11U)
#define QSPI_SPDSR_SSB_IN(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_SSB_IN_SHIFT)) & QSPI_SPDSR_SSB_IN_MASK)
#define QSPI_SPDSR_BD2X_MASK                     (0x1000U)
#define QSPI_SPDSR_BD2X_SHIFT                    (12U)
#define QSPI_SPDSR_BD2X(x)                       (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_BD2X_SHIFT)) & QSPI_SPDSR_BD2X_MASK)
#define QSPI_SPDSR_RDMAEN_MASK                   (0x2000U)
#define QSPI_SPDSR_RDMAEN_SHIFT                  (13U)
#define QSPI_SPDSR_RDMAEN(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_RDMAEN_SHIFT)) & QSPI_SPDSR_RDMAEN_MASK)
#define QSPI_SPDSR_TDMAEN_MASK                   (0x4000U)
#define QSPI_SPDSR_TDMAEN_SHIFT                  (14U)
#define QSPI_SPDSR_TDMAEN(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_TDMAEN_SHIFT)) & QSPI_SPDSR_TDMAEN_MASK)
/*! WOM - Wired-OR Mode
 *  0b0..The SPI pins are configured as push-pull drivers.
 *  0b1..The SPI pins are configured as open-drain drivers with the pull-ups disabled.
 */
#define QSPI_SPDSR_WOM_MASK                      (0x8000U)
#define QSPI_SPDSR_WOM_SHIFT                     (15U)
#define QSPI_SPDSR_WOM(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDSR_WOM_SHIFT)) & QSPI_SPDSR_WOM_MASK)
/*! @} */

/*! @name SPDRR - SPI Data Receive Register */
/*! @{ */
#define QSPI_SPDRR_R0_MASK                       (0x1U)
#define QSPI_SPDRR_R0_SHIFT                      (0U)
#define QSPI_SPDRR_R0(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R0_SHIFT)) & QSPI_SPDRR_R0_MASK)
#define QSPI_SPDRR_R1_MASK                       (0x2U)
#define QSPI_SPDRR_R1_SHIFT                      (1U)
#define QSPI_SPDRR_R1(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R1_SHIFT)) & QSPI_SPDRR_R1_MASK)
#define QSPI_SPDRR_R2_MASK                       (0x4U)
#define QSPI_SPDRR_R2_SHIFT                      (2U)
#define QSPI_SPDRR_R2(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R2_SHIFT)) & QSPI_SPDRR_R2_MASK)
#define QSPI_SPDRR_R3_MASK                       (0x8U)
#define QSPI_SPDRR_R3_SHIFT                      (3U)
#define QSPI_SPDRR_R3(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R3_SHIFT)) & QSPI_SPDRR_R3_MASK)
#define QSPI_SPDRR_R4_MASK                       (0x10U)
#define QSPI_SPDRR_R4_SHIFT                      (4U)
#define QSPI_SPDRR_R4(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R4_SHIFT)) & QSPI_SPDRR_R4_MASK)
#define QSPI_SPDRR_R5_MASK                       (0x20U)
#define QSPI_SPDRR_R5_SHIFT                      (5U)
#define QSPI_SPDRR_R5(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R5_SHIFT)) & QSPI_SPDRR_R5_MASK)
#define QSPI_SPDRR_R6_MASK                       (0x40U)
#define QSPI_SPDRR_R6_SHIFT                      (6U)
#define QSPI_SPDRR_R6(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R6_SHIFT)) & QSPI_SPDRR_R6_MASK)
#define QSPI_SPDRR_R7_MASK                       (0x80U)
#define QSPI_SPDRR_R7_SHIFT                      (7U)
#define QSPI_SPDRR_R7(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R7_SHIFT)) & QSPI_SPDRR_R7_MASK)
#define QSPI_SPDRR_R8_MASK                       (0x100U)
#define QSPI_SPDRR_R8_SHIFT                      (8U)
#define QSPI_SPDRR_R8(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R8_SHIFT)) & QSPI_SPDRR_R8_MASK)
#define QSPI_SPDRR_R9_MASK                       (0x200U)
#define QSPI_SPDRR_R9_SHIFT                      (9U)
#define QSPI_SPDRR_R9(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R9_SHIFT)) & QSPI_SPDRR_R9_MASK)
#define QSPI_SPDRR_R10_MASK                      (0x400U)
#define QSPI_SPDRR_R10_SHIFT                     (10U)
#define QSPI_SPDRR_R10(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R10_SHIFT)) & QSPI_SPDRR_R10_MASK)
#define QSPI_SPDRR_R11_MASK                      (0x800U)
#define QSPI_SPDRR_R11_SHIFT                     (11U)
#define QSPI_SPDRR_R11(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R11_SHIFT)) & QSPI_SPDRR_R11_MASK)
#define QSPI_SPDRR_R12_MASK                      (0x1000U)
#define QSPI_SPDRR_R12_SHIFT                     (12U)
#define QSPI_SPDRR_R12(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R12_SHIFT)) & QSPI_SPDRR_R12_MASK)
#define QSPI_SPDRR_R13_MASK                      (0x2000U)
#define QSPI_SPDRR_R13_SHIFT                     (13U)
#define QSPI_SPDRR_R13(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R13_SHIFT)) & QSPI_SPDRR_R13_MASK)
#define QSPI_SPDRR_R14_MASK                      (0x4000U)
#define QSPI_SPDRR_R14_SHIFT                     (14U)
#define QSPI_SPDRR_R14(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R14_SHIFT)) & QSPI_SPDRR_R14_MASK)
#define QSPI_SPDRR_R15_MASK                      (0x8000U)
#define QSPI_SPDRR_R15_SHIFT                     (15U)
#define QSPI_SPDRR_R15(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDRR_R15_SHIFT)) & QSPI_SPDRR_R15_MASK)
/*! @} */

/*! @name SPDTR - SPI Data Transmit Register */
/*! @{ */
#define QSPI_SPDTR_T0_MASK                       (0x1U)
#define QSPI_SPDTR_T0_SHIFT                      (0U)
#define QSPI_SPDTR_T0(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T0_SHIFT)) & QSPI_SPDTR_T0_MASK)
#define QSPI_SPDTR_T1_MASK                       (0x2U)
#define QSPI_SPDTR_T1_SHIFT                      (1U)
#define QSPI_SPDTR_T1(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T1_SHIFT)) & QSPI_SPDTR_T1_MASK)
#define QSPI_SPDTR_T2_MASK                       (0x4U)
#define QSPI_SPDTR_T2_SHIFT                      (2U)
#define QSPI_SPDTR_T2(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T2_SHIFT)) & QSPI_SPDTR_T2_MASK)
#define QSPI_SPDTR_T3_MASK                       (0x8U)
#define QSPI_SPDTR_T3_SHIFT                      (3U)
#define QSPI_SPDTR_T3(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T3_SHIFT)) & QSPI_SPDTR_T3_MASK)
#define QSPI_SPDTR_T4_MASK                       (0x10U)
#define QSPI_SPDTR_T4_SHIFT                      (4U)
#define QSPI_SPDTR_T4(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T4_SHIFT)) & QSPI_SPDTR_T4_MASK)
#define QSPI_SPDTR_T5_MASK                       (0x20U)
#define QSPI_SPDTR_T5_SHIFT                      (5U)
#define QSPI_SPDTR_T5(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T5_SHIFT)) & QSPI_SPDTR_T5_MASK)
#define QSPI_SPDTR_T6_MASK                       (0x40U)
#define QSPI_SPDTR_T6_SHIFT                      (6U)
#define QSPI_SPDTR_T6(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T6_SHIFT)) & QSPI_SPDTR_T6_MASK)
#define QSPI_SPDTR_T7_MASK                       (0x80U)
#define QSPI_SPDTR_T7_SHIFT                      (7U)
#define QSPI_SPDTR_T7(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T7_SHIFT)) & QSPI_SPDTR_T7_MASK)
#define QSPI_SPDTR_T8_MASK                       (0x100U)
#define QSPI_SPDTR_T8_SHIFT                      (8U)
#define QSPI_SPDTR_T8(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T8_SHIFT)) & QSPI_SPDTR_T8_MASK)
#define QSPI_SPDTR_T9_MASK                       (0x200U)
#define QSPI_SPDTR_T9_SHIFT                      (9U)
#define QSPI_SPDTR_T9(x)                         (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T9_SHIFT)) & QSPI_SPDTR_T9_MASK)
#define QSPI_SPDTR_T10_MASK                      (0x400U)
#define QSPI_SPDTR_T10_SHIFT                     (10U)
#define QSPI_SPDTR_T10(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T10_SHIFT)) & QSPI_SPDTR_T10_MASK)
#define QSPI_SPDTR_T11_MASK                      (0x800U)
#define QSPI_SPDTR_T11_SHIFT                     (11U)
#define QSPI_SPDTR_T11(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T11_SHIFT)) & QSPI_SPDTR_T11_MASK)
#define QSPI_SPDTR_T12_MASK                      (0x1000U)
#define QSPI_SPDTR_T12_SHIFT                     (12U)
#define QSPI_SPDTR_T12(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T12_SHIFT)) & QSPI_SPDTR_T12_MASK)
#define QSPI_SPDTR_T13_MASK                      (0x2000U)
#define QSPI_SPDTR_T13_SHIFT                     (13U)
#define QSPI_SPDTR_T13(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T13_SHIFT)) & QSPI_SPDTR_T13_MASK)
#define QSPI_SPDTR_T14_MASK                      (0x4000U)
#define QSPI_SPDTR_T14_SHIFT                     (14U)
#define QSPI_SPDTR_T14(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T14_SHIFT)) & QSPI_SPDTR_T14_MASK)
#define QSPI_SPDTR_T15_MASK                      (0x8000U)
#define QSPI_SPDTR_T15_SHIFT                     (15U)
#define QSPI_SPDTR_T15(x)                        (((uint16_t)(((uint16_t)(x)) << QSPI_SPDTR_T15_SHIFT)) & QSPI_SPDTR_T15_MASK)
/*! @} */

/*! @name SPFIFO - SPI FIFO Control Register */
/*! @{ */
/*! FIFO_ENA - FIFO Enable
 *  0b0..FIFOs are disabled and reset.
 *  0b1..FIFOs are enabled. FIFOs retain their status even if SPE is set to 0.
 */
#define QSPI_SPFIFO_FIFO_ENA_MASK                (0x1U)
#define QSPI_SPFIFO_FIFO_ENA_SHIFT               (0U)
#define QSPI_SPFIFO_FIFO_ENA(x)                  (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_FIFO_ENA_SHIFT)) & QSPI_SPFIFO_FIFO_ENA_MASK)
/*! RFWM - Rx FIFO Watermark
 *  0b00..Receive interrupt active when Rx FIFO has at least one word used
 *  0b01..Receive interrupt active when Rx FIFO has at least two words used
 *  0b10..Receive interrupt active when Rx FIFO has at least three words used
 *  0b11..Receive interrupt active when Rx FIFO is full
 */
#define QSPI_SPFIFO_RFWM_MASK                    (0xCU)
#define QSPI_SPFIFO_RFWM_SHIFT                   (2U)
#define QSPI_SPFIFO_RFWM(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_RFWM_SHIFT)) & QSPI_SPFIFO_RFWM_MASK)
/*! TFWM - Tx FIFO Watermark
 *  0b00..Transmit interrupt active when Tx FIFO is empty
 *  0b01..Transmit interrupt active when Tx FIFO has one or fewer words available
 *  0b10..Transmit interrupt active when Tx FIFO has two or fewer words available
 *  0b11..Transmit interrupt active when Tx FIFO has three or fewer words available
 */
#define QSPI_SPFIFO_TFWM_MASK                    (0x60U)
#define QSPI_SPFIFO_TFWM_SHIFT                   (5U)
#define QSPI_SPFIFO_TFWM(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_TFWM_SHIFT)) & QSPI_SPFIFO_TFWM_MASK)
/*! RFCNT - RX FIFO Level
 *  0b000..Rx FIFO empty
 *  0b001..One word used in Rx FIFO
 *  0b010..Two words used in Rx FIFO
 *  0b011..Three words used in Rx FIFO
 *  0b100..Rx FIFO full (if enabled Receiver Full Interrupt asserted)
 */
#define QSPI_SPFIFO_RFCNT_MASK                   (0x700U)
#define QSPI_SPFIFO_RFCNT_SHIFT                  (8U)
#define QSPI_SPFIFO_RFCNT(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_RFCNT_SHIFT)) & QSPI_SPFIFO_RFCNT_MASK)
/*! TFCNT - TX FIFO Level
 *  0b000..Tx FIFO empty (if enabled Transmit Empty Interrupt asserted)
 *  0b001..One word used in Tx FIFO
 *  0b010..Two words used in Tx FIFO
 *  0b011..Three words used in Tx FIFO
 *  0b100..Tx FIFO full
 */
#define QSPI_SPFIFO_TFCNT_MASK                   (0x7000U)
#define QSPI_SPFIFO_TFCNT_SHIFT                  (12U)
#define QSPI_SPFIFO_TFCNT(x)                     (((uint16_t)(((uint16_t)(x)) << QSPI_SPFIFO_TFCNT_SHIFT)) & QSPI_SPFIFO_TFCNT_MASK)
/*! @} */

/*! @name SPWAIT - SPI Word Delay Register */
/*! @{ */
#define QSPI_SPWAIT_WAIT_MASK                    (0x1FFFU)
#define QSPI_SPWAIT_WAIT_SHIFT                   (0U)
#define QSPI_SPWAIT_WAIT(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPWAIT_WAIT_SHIFT)) & QSPI_SPWAIT_WAIT_MASK)
/*! @} */

/*! @name SPCTL2 - SPI Control Register 2 */
/*! @{ */
/*! SHEN - Stop Mode Holdoff Enable
 *  0b0..Disable stop mode holdoff .
 *  0b1..Enable stop mode holdoff while the SPI is transmitting/receiving.
 */
#define QSPI_SPCTL2_SHEN_MASK                    (0x1U)
#define QSPI_SPCTL2_SHEN_SHIFT                   (0U)
#define QSPI_SPCTL2_SHEN(x)                      (((uint16_t)(((uint16_t)(x)) << QSPI_SPCTL2_SHEN_SHIFT)) & QSPI_SPCTL2_SHEN_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group QSPI_Register_Masks */


/* QSPI - Peripheral instance base addresses */
/** Peripheral QSPI0 base address */
#define QSPI0_BASE                               (0xE0B0u)
/** Peripheral QSPI0 base pointer */
#define QSPI0                                    ((QSPI_Type *)QSPI0_BASE)
/** Array initializer of QSPI peripheral base addresses */
#define QSPI_BASE_ADDRS                          { QSPI0_BASE }
/** Array initializer of QSPI peripheral base pointers */
#define QSPI_BASE_PTRS                           { QSPI0 }

/*!
 * @}
 */ /* end of group QSPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  __IO uint16_t CTRL;                              /**< Control Register, offset: 0x0 */
  __I  uint16_t RSTAT;                             /**< Reset Status Register, offset: 0x1 */
  __IO uint16_t SCR0;                              /**< Software Control Register, offset: 0x2 */
  __IO uint16_t SCR1;                              /**< Software Control Register, offset: 0x3 */
  __IO uint16_t SCR2;                              /**< Software Control Register, offset: 0x4 */
  __IO uint16_t SCR3;                              /**< Software Control Register, offset: 0x5 */
       uint16_t MSHID;                             /**< Most Significant Half of JTAG ID, offset: 0x6 */
       uint16_t LSHID;                             /**< Least Significant Half of JTAG ID, offset: 0x7 */
  __IO uint16_t PWR;                               /**< Power Control Register, offset: 0x8 */
       uint16_t RESERVED_0[1];
  __IO uint16_t CLKOUT;                            /**< Clock Output Select Register, offset: 0xA */
  __IO uint16_t PCR;                               /**< Peripheral Clock Rate Register, offset: 0xB */
  __IO uint16_t PCE0;                              /**< Peripheral Clock Enable Register 0, offset: 0xC */
  __IO uint16_t PCE1;                              /**< Peripheral Clock Enable Register 1, offset: 0xD */
  __IO uint16_t PCE2;                              /**< Peripheral Clock Enable Register 2, offset: 0xE */
  __IO uint16_t PCE3;                              /**< Peripheral Clock Enable Register 3, offset: 0xF */
  __IO uint16_t SD0;                               /**< Peripheral Clock STOP Disable Register 0, offset: 0x10 */
  __IO uint16_t SD1;                               /**< Peripheral Clock STOP Disable Register 1, offset: 0x11 */
  __IO uint16_t SD2;                               /**< Peripheral Clock STOP Disable Register 2, offset: 0x12 */
  __IO uint16_t SD3;                               /**< Peripheral Clock STOP Disable Register 3, offset: 0x13 */
  __IO uint16_t IOSAHI;                            /**< I/O Short Address Location Register, offset: 0x14 */
  __IO uint16_t IOSALO;                            /**< I/O Short Address Location Register, offset: 0x15 */
  __IO uint16_t PROT;                              /**< Protection Register, offset: 0x16 */
  __IO uint16_t GPSAL;                             /**< GPIOA LSBs Peripheral Select Register, offset: 0x17 */
  __IO uint16_t GPSAH;                             /**< GPIOA MSBs Peripheral Select Register, offset: 0x18 */
  __IO uint16_t GPSBL;                             /**< GPIOB LSBs Peripheral Select Register, offset: 0x19 */
  __IO uint16_t GPSBH;                             /**< GPIOB MSBs Peripheral Select Register, offset: 0x1A */
  __IO uint16_t GPSCL;                             /**< GPIOC LSBs Peripheral Select Register, offset: 0x1B */
  __IO uint16_t GPSCH;                             /**< GPIOC MSBs Peripheral Select Register, offset: 0x1C */
  __IO uint16_t GPSDL;                             /**< GPIOD LSBs Peripheral Select Register, offset: 0x1D */
       uint16_t RESERVED_1[1];
  __IO uint16_t GPSEL;                             /**< GPIOE LSBs Peripheral Select Register, offset: 0x1F */
  __IO uint16_t GPSEH;                             /**< GPIOE MSBs Peripheral Select Register, offset: 0x20 */
  __IO uint16_t GPSFL;                             /**< GPIOF LSBs Peripheral Select Register, offset: 0x21 */
  __IO uint16_t GPSFH;                             /**< GPIOF MSBs Peripheral Select Register, offset: 0x22 */
  __IO uint16_t GPSGL;                             /**< GPIOG LSBs Peripheral Select Register, offset: 0x23 */
  __IO uint16_t GPSGH;                             /**< GPIOG MSBs Peripheral Select Register, offset: 0x24 */
       uint16_t RESERVED_2[4];
  __IO uint16_t IPS0;                              /**< Internal Peripheral Select Register 0, offset: 0x29 */
  __IO uint16_t MISC0;                             /**< Miscellaneous Register 0, offset: 0x2A */
  __IO uint16_t PSWR0;                             /**< Peripheral Software Reset Register 0, offset: 0x2B */
  __IO uint16_t PSWR1;                             /**< Peripheral Software Reset Register 1, offset: 0x2C */
  __IO uint16_t PSWR2;                             /**< Peripheral Software Reset Register 2, offset: 0x2D */
  __IO uint16_t PSWR3;                             /**< Peripheral Software Reset Register 3, offset: 0x2E */
  __IO uint16_t PWRMODE;                           /**< Power Mode Register, offset: 0x2F */
       uint16_t RESERVED_3[11];
  __I  uint16_t NVMOPT6_HIGH;                      /**< Non-Volatile Memory Option Register 6 (High), offset: 0x3B */
  __I  uint16_t NVMOPT6_LOW;                       /**< Non-Volatile Memory Option Register 6 (Low), offset: 0x3C */
       uint16_t RESERVED_4[15];
  __IO uint16_t PWM_SEL;                           /**< PWM Select Register, offset: 0x4C */
  __IO uint16_t ADC_TMR_SEL;                       /**< ADC and TMR Select Register, offset: 0x4D */
  __IO uint16_t BOOT_MODE_OVERRIDE;                /**< Boot Mode Override Register, offset: 0x4E */
} SIM_Type;

/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/*! @name CTRL - Control Register */
/*! @{ */
/*! WAIT_DISABLE - WAIT Disable
 *  0b00..Wait mode is entered when the DSC core executes a WAIT instruction.
 *  0b01..The DSC core WAIT instruction does not cause entry into wait mode.
 *  0b10..Wait mode is entered when the DSC core executes a WAIT instruction, and the WAIT_disable field is write protected until the next reset.
 *  0b11..The DSC core WAIT instruction does not cause entry into wait mode, and the WAIT_disable field is write protected until the next reset.
 */
#define SIM_CTRL_WAIT_DISABLE_MASK               (0x3U)
#define SIM_CTRL_WAIT_DISABLE_SHIFT              (0U)
#define SIM_CTRL_WAIT_DISABLE(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_WAIT_DISABLE_SHIFT)) & SIM_CTRL_WAIT_DISABLE_MASK)
/*! STOP_DISABLE - STOP Disable
 *  0b00..Stop mode is entered when the DSC core executes a STOP instruction.
 *  0b01..The DSC core STOP instruction does not cause entry into stop mode.
 *  0b10..Stop mode is entered when the DSC core executes a STOP instruction, and the STOP_disable field is write protected until the next reset.
 *  0b11..The DSC core STOP instruction does not cause entry into stop mode, and the STOP_disable field is write protected until the next reset.
 */
#define SIM_CTRL_STOP_DISABLE_MASK               (0xCU)
#define SIM_CTRL_STOP_DISABLE_SHIFT              (2U)
#define SIM_CTRL_STOP_DISABLE(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_STOP_DISABLE_SHIFT)) & SIM_CTRL_STOP_DISABLE_MASK)
#define SIM_CTRL_SWRST_MASK                      (0x10U)
#define SIM_CTRL_SWRST_SHIFT                     (4U)
#define SIM_CTRL_SWRST(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_SWRST_SHIFT)) & SIM_CTRL_SWRST_MASK)
/*! ONCEEBL - OnCE Enable
 *  0b0..The OnCE clock to the DSC core is enabled when the core TAP is enabled.
 *  0b1..The OnCE clock to the DSC core is always enabled.
 */
#define SIM_CTRL_ONCEEBL_MASK                    (0x20U)
#define SIM_CTRL_ONCEEBL_SHIFT                   (5U)
#define SIM_CTRL_ONCEEBL(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_ONCEEBL_SHIFT)) & SIM_CTRL_ONCEEBL_MASK)
/*! DMAEBL - DMA Enable
 *  0b000..DMA module is disabled.
 *  0b001..DMA module is enabled in run mode only.
 *  0b010..DMA module is enabled in run and wait modes only.
 *  0b011..DMA module is enabled in all power modes.
 *  0b100..DMA module is disabled and the DMAEbl field is write protected until the next reset.
 *  0b101..DMA module is enabled in run mode only and the DMAEbl field is write protected until the next reset.
 *  0b110..DMA module is enabled in run and wait modes only and the DMAEbl field is write protected until the next reset.
 *  0b111..DMA module is enabled in all low power modes and the DMAEbl field is write protected until the next reset.
 */
#define SIM_CTRL_DMAEBL_MASK                     (0x1C0U)
#define SIM_CTRL_DMAEBL_SHIFT                    (6U)
#define SIM_CTRL_DMAEBL(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_DMAEBL_SHIFT)) & SIM_CTRL_DMAEBL_MASK)
/*! RST_FILT - External Reset Padcell Input Filter Enable
 *  0b0..Input filter on external reset disabled
 *  0b1..Input filter on external reset enabled
 */
#define SIM_CTRL_RST_FILT_MASK                   (0x400U)
#define SIM_CTRL_RST_FILT_SHIFT                  (10U)
#define SIM_CTRL_RST_FILT(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_CTRL_RST_FILT_SHIFT)) & SIM_CTRL_RST_FILT_MASK)
/*! @} */

/*! @name RSTAT - Reset Status Register */
/*! @{ */
#define SIM_RSTAT_POR_MASK                       (0x4U)
#define SIM_RSTAT_POR_SHIFT                      (2U)
#define SIM_RSTAT_POR(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_POR_SHIFT)) & SIM_RSTAT_POR_MASK)
#define SIM_RSTAT_EXTR_MASK                      (0x8U)
#define SIM_RSTAT_EXTR_SHIFT                     (3U)
#define SIM_RSTAT_EXTR(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_EXTR_SHIFT)) & SIM_RSTAT_EXTR_MASK)
#define SIM_RSTAT_COP_LOR_MASK                   (0x10U)
#define SIM_RSTAT_COP_LOR_SHIFT                  (4U)
#define SIM_RSTAT_COP_LOR(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_LOR_SHIFT)) & SIM_RSTAT_COP_LOR_MASK)
#define SIM_RSTAT_COP_CPU_MASK                   (0x20U)
#define SIM_RSTAT_COP_CPU_SHIFT                  (5U)
#define SIM_RSTAT_COP_CPU(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_CPU_SHIFT)) & SIM_RSTAT_COP_CPU_MASK)
#define SIM_RSTAT_SWR_MASK                       (0x40U)
#define SIM_RSTAT_SWR_SHIFT                      (6U)
#define SIM_RSTAT_SWR(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_SWR_SHIFT)) & SIM_RSTAT_SWR_MASK)
#define SIM_RSTAT_COP_WIN_MASK                   (0x100U)
#define SIM_RSTAT_COP_WIN_SHIFT                  (8U)
#define SIM_RSTAT_COP_WIN(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_COP_WIN_SHIFT)) & SIM_RSTAT_COP_WIN_MASK)
#define SIM_RSTAT_BOOT_MODE_STATUS_MASK          (0x600U)
#define SIM_RSTAT_BOOT_MODE_STATUS_SHIFT         (9U)
#define SIM_RSTAT_BOOT_MODE_STATUS(x)            (((uint16_t)(((uint16_t)(x)) << SIM_RSTAT_BOOT_MODE_STATUS_SHIFT)) & SIM_RSTAT_BOOT_MODE_STATUS_MASK)
/*! @} */

/*! @name SCR0 - Software Control Register */
/*! @{ */
#define SIM_SCR0_SCR0_MASK                       (0xFFFFU)
#define SIM_SCR0_SCR0_SHIFT                      (0U)
#define SIM_SCR0_SCR0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SCR0_SCR0_SHIFT)) & SIM_SCR0_SCR0_MASK)
/*! @} */

/*! @name SCR1 - Software Control Register */
/*! @{ */
#define SIM_SCR1_SCR1_MASK                       (0xFFFFU)
#define SIM_SCR1_SCR1_SHIFT                      (0U)
#define SIM_SCR1_SCR1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SCR1_SCR1_SHIFT)) & SIM_SCR1_SCR1_MASK)
/*! @} */

/*! @name SCR2 - Software Control Register */
/*! @{ */
#define SIM_SCR2_SCR2_MASK                       (0xFFFFU)
#define SIM_SCR2_SCR2_SHIFT                      (0U)
#define SIM_SCR2_SCR2(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SCR2_SCR2_SHIFT)) & SIM_SCR2_SCR2_MASK)
/*! @} */

/*! @name SCR3 - Software Control Register */
/*! @{ */
#define SIM_SCR3_SCR3_MASK                       (0xFFFFU)
#define SIM_SCR3_SCR3_SHIFT                      (0U)
#define SIM_SCR3_SCR3(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SCR3_SCR3_SHIFT)) & SIM_SCR3_SCR3_MASK)
/*! @} */

/*! @name PWR - Power Control Register */
/*! @{ */
/*! LRSTDBY - Large Regulator Standby Control
 *  0b00..Large regulator placed in normal mode (default).
 *  0b01..Large regulator placed in standby mode.
 *  0b10..Large regulator placed in normal mode and LRSTDBY is write protected until device reset.
 *  0b11..Large regulator placed in standby mode and LRSTDBY is write protected until device reset.
 */
#define SIM_PWR_LRSTDBY_MASK                     (0x3U)
#define SIM_PWR_LRSTDBY_SHIFT                    (0U)
#define SIM_PWR_LRSTDBY(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PWR_LRSTDBY_SHIFT)) & SIM_PWR_LRSTDBY_MASK)
/*! SR27STDBY - Small Regulator 2.7 V Supply Standby Control
 *  0b00..Small regulator 2.7 V supply placed in normal mode (default).
 *  0b01..Small regulator 2.7 V supply placed in standby mode.
 *  0b10..Small regulator 2.7 V supply placed in normal mode and SR27STDBY is write protected until chip reset.
 *  0b11..Small regulator 2.7 V supply placed in standby mode and SR27STDBY is write protected until chip reset.
 */
#define SIM_PWR_SR27STDBY_MASK                   (0xCU)
#define SIM_PWR_SR27STDBY_SHIFT                  (2U)
#define SIM_PWR_SR27STDBY(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR27STDBY_SHIFT)) & SIM_PWR_SR27STDBY_MASK)
/*! SR27PDN - Small Regulator 2.7 V Supply Powerdown Control
 *  0b00..Small regulator placed in normal mode (default).
 *  0b01..Small regulator placed in powerdown mode.
 *  0b10..Small regulator placed in normal mode and SR27PDN is write protected until chip reset.
 *  0b11..Small regulator placed in powerdown mode and SR27PDN is write protected until chip reset.
 */
#define SIM_PWR_SR27PDN_MASK                     (0x30U)
#define SIM_PWR_SR27PDN_SHIFT                    (4U)
#define SIM_PWR_SR27PDN(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR27PDN_SHIFT)) & SIM_PWR_SR27PDN_MASK)
/*! SR12STDBY - Small Regulator 1.2 V Supply Standby Control
 *  0b00..Small regulator 1.2 V supply placed in normal mode (default).
 *  0b01..Small regulator 1.2 V supply placed in standby mode.
 *  0b10..Small regulator 1.2 V supply placed in normal mode and SR12STDBY is write protected until chip reset.
 *  0b11..Small regulator 1.2 V supply placed in standby mode and SR12STDBY is write protected until chip reset.
 */
#define SIM_PWR_SR12STDBY_MASK                   (0xC0U)
#define SIM_PWR_SR12STDBY_SHIFT                  (6U)
#define SIM_PWR_SR12STDBY(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PWR_SR12STDBY_SHIFT)) & SIM_PWR_SR12STDBY_MASK)
/*! @} */

/*! @name CLKOUT - Clock Output Select Register */
/*! @{ */
/*! CLKOSEL0 - CLKOUT0 Select
 *  0b000..Function = BUS_CLK continuous after reset
 *  0b001..Function = 2X_BUS_CLK continuous after reset
 *  0b010..Function = DIV4_BUS_CLK continuous after reset
 *  0b011..Function = MSTR_OSC (master clock) continuous after reset
 *  0b100..output of the MUX choosing between 8M RC and 48M RC div 6
 *  0b101..ROSC_200K (200 kHz relaxation oscillator clock)
 *  0b110..Reserved. For normal operation, do not write 11x.
 *  0b111..Reserved. For normal operation, do not write 11x.
 */
#define SIM_CLKOUT_CLKOSEL0_MASK                 (0x7U)
#define SIM_CLKOUT_CLKOSEL0_SHIFT                (0U)
#define SIM_CLKOUT_CLKOSEL0(x)                   (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKOSEL0_SHIFT)) & SIM_CLKOUT_CLKOSEL0_MASK)
/*! CLKDIS0 - Disable for CLKOUT0
 *  0b0..CLKOUT0 output is enabled and outputs the signal indicated by CLKOSEL0
 *  0b1..CLKOUT0 is disabled
 */
#define SIM_CLKOUT_CLKDIS0_MASK                  (0x20U)
#define SIM_CLKOUT_CLKDIS0_SHIFT                 (5U)
#define SIM_CLKOUT_CLKDIS0(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKDIS0_SHIFT)) & SIM_CLKOUT_CLKDIS0_MASK)
/*! CLKOSEL1 - CLKOUT1 Select
 *  0b000..Function = BUS_CLK continuous after reset
 *  0b001..Function = 2X_BUS_CLK continuous after reset
 *  0b010..Function = DIV4_BUS_CLK continuous after reset
 *  0b011..Function = MSTR_OSC (master clock) continuous after reset
 *  0b100..output of the MUX choosing between 8M RC and 48M RC div 6
 *  0b101..ROSC_200K (200 kHz relaxation oscillator clock)
 *  0b110..Reserved. For normal operation, do not write 11x.
 *  0b111..Reserved. For normal operation, do not write 11x.
 */
#define SIM_CLKOUT_CLKOSEL1_MASK                 (0x380U)
#define SIM_CLKOUT_CLKOSEL1_SHIFT                (7U)
#define SIM_CLKOUT_CLKOSEL1(x)                   (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKOSEL1_SHIFT)) & SIM_CLKOUT_CLKOSEL1_MASK)
/*! CLKDIS1 - Disable for CLKOUT1
 *  0b0..CLKOUT1 output is enabled and outputs the signal indicated by CLKOSEL1
 *  0b1..CLKOUT1 is disabled
 */
#define SIM_CLKOUT_CLKDIS1_MASK                  (0x1000U)
#define SIM_CLKOUT_CLKDIS1_SHIFT                 (12U)
#define SIM_CLKOUT_CLKDIS1(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKDIS1_SHIFT)) & SIM_CLKOUT_CLKDIS1_MASK)
/*! CLKODIV - CLKOUT divide factor
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 4
 *  0b011..Divide by 8
 *  0b100..Divide by 16
 *  0b101..Divide by 32
 *  0b110..Divide by 64
 *  0b111..Divide by 128
 */
#define SIM_CLKOUT_CLKODIV_MASK                  (0xE000U)
#define SIM_CLKOUT_CLKODIV_SHIFT                 (13U)
#define SIM_CLKOUT_CLKODIV(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_CLKOUT_CLKODIV_SHIFT)) & SIM_CLKOUT_CLKODIV_MASK)
/*! @} */

/*! @name PCR - Peripheral Clock Rate Register */
/*! @{ */
/*! IIC1_CR - IIC1 filter Clock Rate
 *  0b0..IIC1 filter clock rate equals core clock rate (default)
 *  0b1..IIC1 filter clock rate equals 2X core clock rate
 */
#define SIM_PCR_IIC1_CR_MASK                     (0x200U)
#define SIM_PCR_IIC1_CR_SHIFT                    (9U)
#define SIM_PCR_IIC1_CR(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCR_IIC1_CR_SHIFT)) & SIM_PCR_IIC1_CR_MASK)
/*! IIC0_CR - IIC0 filter Clock Rate
 *  0b0..IIC0 filter clock rate equals core clock rate (default)
 *  0b1..IIC0 filter clock rate equals 2X core clock rate
 */
#define SIM_PCR_IIC0_CR_MASK                     (0x400U)
#define SIM_PCR_IIC0_CR_SHIFT                    (10U)
#define SIM_PCR_IIC0_CR(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCR_IIC0_CR_SHIFT)) & SIM_PCR_IIC0_CR_MASK)
/*! SCI2_CR - SCI2 Clock Rate
 *  0b0..SCI2 clock rate equals core clock rate (default)
 *  0b1..SCI2 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI2_CR_MASK                     (0x800U)
#define SIM_PCR_SCI2_CR_SHIFT                    (11U)
#define SIM_PCR_SCI2_CR(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI2_CR_SHIFT)) & SIM_PCR_SCI2_CR_MASK)
/*! SCI1_CR - SCI1 Clock Rate
 *  0b0..SCI1 clock rate equals core clock rate (default)
 *  0b1..SCI1 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI1_CR_MASK                     (0x1000U)
#define SIM_PCR_SCI1_CR_SHIFT                    (12U)
#define SIM_PCR_SCI1_CR(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI1_CR_SHIFT)) & SIM_PCR_SCI1_CR_MASK)
/*! SCI0_CR - SCI0 Clock Rate
 *  0b0..SCI0 clock rate equals core clock rate (default)
 *  0b1..SCI0 clock rate equals 2X core clock rate
 */
#define SIM_PCR_SCI0_CR_MASK                     (0x2000U)
#define SIM_PCR_SCI0_CR_SHIFT                    (13U)
#define SIM_PCR_SCI0_CR(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCR_SCI0_CR_SHIFT)) & SIM_PCR_SCI0_CR_MASK)
/*! @} */

/*! @name PCE0 - Peripheral Clock Enable Register 0 */
/*! @{ */
/*! GPIOG - GPIOG IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOG_MASK                      (0x1U)
#define SIM_PCE0_GPIOG_SHIFT                     (0U)
#define SIM_PCE0_GPIOG(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOG_SHIFT)) & SIM_PCE0_GPIOG_MASK)
/*! GPIOF - GPIOF IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOF_MASK                      (0x2U)
#define SIM_PCE0_GPIOF_SHIFT                     (1U)
#define SIM_PCE0_GPIOF(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOF_SHIFT)) & SIM_PCE0_GPIOF_MASK)
/*! GPIOE - GPIOE IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOE_MASK                      (0x4U)
#define SIM_PCE0_GPIOE_SHIFT                     (2U)
#define SIM_PCE0_GPIOE(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOE_SHIFT)) & SIM_PCE0_GPIOE_MASK)
/*! GPIOD - GPIOD IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOD_MASK                      (0x8U)
#define SIM_PCE0_GPIOD_SHIFT                     (3U)
#define SIM_PCE0_GPIOD(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOD_SHIFT)) & SIM_PCE0_GPIOD_MASK)
/*! GPIOC - GPIOC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOC_MASK                      (0x10U)
#define SIM_PCE0_GPIOC_SHIFT                     (4U)
#define SIM_PCE0_GPIOC(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOC_SHIFT)) & SIM_PCE0_GPIOC_MASK)
/*! GPIOB - GPIOB IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOB_MASK                      (0x20U)
#define SIM_PCE0_GPIOB_SHIFT                     (5U)
#define SIM_PCE0_GPIOB(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOB_SHIFT)) & SIM_PCE0_GPIOB_MASK)
/*! GPIOA - GPIOA IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_GPIOA_MASK                      (0x40U)
#define SIM_PCE0_GPIOA_SHIFT                     (6U)
#define SIM_PCE0_GPIOA(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_GPIOA_SHIFT)) & SIM_PCE0_GPIOA_MASK)
/*! TB3 - TMRB3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB3_MASK                        (0x100U)
#define SIM_PCE0_TB3_SHIFT                       (8U)
#define SIM_PCE0_TB3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB3_SHIFT)) & SIM_PCE0_TB3_MASK)
/*! TB2 - TMRB2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB2_MASK                        (0x200U)
#define SIM_PCE0_TB2_SHIFT                       (9U)
#define SIM_PCE0_TB2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB2_SHIFT)) & SIM_PCE0_TB2_MASK)
/*! TB1 - TMRB1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB1_MASK                        (0x400U)
#define SIM_PCE0_TB1_SHIFT                       (10U)
#define SIM_PCE0_TB1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB1_SHIFT)) & SIM_PCE0_TB1_MASK)
/*! TB0 - TMRB0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TB0_MASK                        (0x800U)
#define SIM_PCE0_TB0_SHIFT                       (11U)
#define SIM_PCE0_TB0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TB0_SHIFT)) & SIM_PCE0_TB0_MASK)
/*! TA3 - TMRA3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA3_MASK                        (0x1000U)
#define SIM_PCE0_TA3_SHIFT                       (12U)
#define SIM_PCE0_TA3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA3_SHIFT)) & SIM_PCE0_TA3_MASK)
/*! TA2 - TMRA2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA2_MASK                        (0x2000U)
#define SIM_PCE0_TA2_SHIFT                       (13U)
#define SIM_PCE0_TA2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA2_SHIFT)) & SIM_PCE0_TA2_MASK)
/*! TA1 - TMRA1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA1_MASK                        (0x4000U)
#define SIM_PCE0_TA1_SHIFT                       (14U)
#define SIM_PCE0_TA1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA1_SHIFT)) & SIM_PCE0_TA1_MASK)
/*! TA0 - TMRA0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE0_TA0_MASK                        (0x8000U)
#define SIM_PCE0_TA0_SHIFT                       (15U)
#define SIM_PCE0_TA0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE0_TA0_SHIFT)) & SIM_PCE0_TA0_MASK)
/*! @} */

/*! @name PCE1 - Peripheral Clock Enable Register 1 */
/*! @{ */
/*! FLEXCAN - FlexCAN IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_FLEXCAN_MASK                    (0x1U)
#define SIM_PCE1_FLEXCAN_SHIFT                   (0U)
#define SIM_PCE1_FLEXCAN(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_FLEXCAN_SHIFT)) & SIM_PCE1_FLEXCAN_MASK)
/*! IIC1 - IIC1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_IIC1_MASK                       (0x20U)
#define SIM_PCE1_IIC1_SHIFT                      (5U)
#define SIM_PCE1_IIC1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_IIC1_SHIFT)) & SIM_PCE1_IIC1_MASK)
/*! IIC0 - IIC0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_IIC0_MASK                       (0x40U)
#define SIM_PCE1_IIC0_SHIFT                      (6U)
#define SIM_PCE1_IIC0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_IIC0_SHIFT)) & SIM_PCE1_IIC0_MASK)
/*! QSPI1 - QSPI1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_QSPI1_MASK                      (0x100U)
#define SIM_PCE1_QSPI1_SHIFT                     (8U)
#define SIM_PCE1_QSPI1(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_QSPI1_SHIFT)) & SIM_PCE1_QSPI1_MASK)
/*! QSPI0 - QSPI0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_QSPI0_MASK                      (0x200U)
#define SIM_PCE1_QSPI0_SHIFT                     (9U)
#define SIM_PCE1_QSPI0(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_QSPI0_SHIFT)) & SIM_PCE1_QSPI0_MASK)
/*! SCI2 - SCI2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI2_MASK                       (0x400U)
#define SIM_PCE1_SCI2_SHIFT                      (10U)
#define SIM_PCE1_SCI2(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI2_SHIFT)) & SIM_PCE1_SCI2_MASK)
/*! SCI1 - SCI1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI1_MASK                       (0x800U)
#define SIM_PCE1_SCI1_SHIFT                      (11U)
#define SIM_PCE1_SCI1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI1_SHIFT)) & SIM_PCE1_SCI1_MASK)
/*! SCI0 - SCI0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_SCI0_MASK                       (0x1000U)
#define SIM_PCE1_SCI0_SHIFT                      (12U)
#define SIM_PCE1_SCI0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_SCI0_SHIFT)) & SIM_PCE1_SCI0_MASK)
/*! DAC0 - DAC0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_DAC0_MASK                       (0x2000U)
#define SIM_PCE1_DAC0_SHIFT                      (13U)
#define SIM_PCE1_DAC0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_DAC0_SHIFT)) & SIM_PCE1_DAC0_MASK)
/*! DAC1 - DAC1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE1_DAC1_MASK                       (0x4000U)
#define SIM_PCE1_DAC1_SHIFT                      (14U)
#define SIM_PCE1_DAC1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE1_DAC1_SHIFT)) & SIM_PCE1_DAC1_MASK)
/*! @} */

/*! @name PCE2 - Peripheral Clock Enable Register 2 */
/*! @{ */
/*! PIT1 - Programmable Interval Timer IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_PIT1_MASK                       (0x4U)
#define SIM_PCE2_PIT1_SHIFT                      (2U)
#define SIM_PCE2_PIT1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_PIT1_SHIFT)) & SIM_PCE2_PIT1_MASK)
/*! PIT0 - Programmable Interval Timer IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_PIT0_MASK                       (0x8U)
#define SIM_PCE2_PIT0_SHIFT                      (3U)
#define SIM_PCE2_PIT0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_PIT0_SHIFT)) & SIM_PCE2_PIT0_MASK)
/*! CRC - CRC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CRC_MASK                        (0x20U)
#define SIM_PCE2_CRC_SHIFT                       (5U)
#define SIM_PCE2_CRC(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CRC_SHIFT)) & SIM_PCE2_CRC_MASK)
/*! CYCADC - Cyclic ADC IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CYCADC_MASK                     (0x80U)
#define SIM_PCE2_CYCADC_SHIFT                    (7U)
#define SIM_PCE2_CYCADC(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CYCADC_SHIFT)) & SIM_PCE2_CYCADC_MASK)
/*! CMPD - CMPD IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPD_MASK                       (0x200U)
#define SIM_PCE2_CMPD_SHIFT                      (9U)
#define SIM_PCE2_CMPD(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPD_SHIFT)) & SIM_PCE2_CMPD_MASK)
/*! CMPC - CMPC IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPC_MASK                       (0x400U)
#define SIM_PCE2_CMPC_SHIFT                      (10U)
#define SIM_PCE2_CMPC(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPC_SHIFT)) & SIM_PCE2_CMPC_MASK)
/*! CMPB - CMPB IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPB_MASK                       (0x800U)
#define SIM_PCE2_CMPB_SHIFT                      (11U)
#define SIM_PCE2_CMPB(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPB_SHIFT)) & SIM_PCE2_CMPB_MASK)
/*! CMPA - CMPA IPBus Clock Enable (enables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE2_CMPA_MASK                       (0x1000U)
#define SIM_PCE2_CMPA_SHIFT                      (12U)
#define SIM_PCE2_CMPA(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PCE2_CMPA_SHIFT)) & SIM_PCE2_CMPA_MASK)
/*! @} */

/*! @name PCE3 - Peripheral Clock Enable Register 3 */
/*! @{ */
/*! PWMBCH3 - PWMB Channel 3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH3_MASK                    (0x1U)
#define SIM_PCE3_PWMBCH3_SHIFT                   (0U)
#define SIM_PCE3_PWMBCH3(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH3_SHIFT)) & SIM_PCE3_PWMBCH3_MASK)
/*! PWMBCH2 - PWMB Channel 2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH2_MASK                    (0x2U)
#define SIM_PCE3_PWMBCH2_SHIFT                   (1U)
#define SIM_PCE3_PWMBCH2(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH2_SHIFT)) & SIM_PCE3_PWMBCH2_MASK)
/*! PWMBCH1 - PWMB Channel 1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH1_MASK                    (0x4U)
#define SIM_PCE3_PWMBCH1_SHIFT                   (2U)
#define SIM_PCE3_PWMBCH1(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH1_SHIFT)) & SIM_PCE3_PWMBCH1_MASK)
/*! PWMBCH0 - PWMB Channel 0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMBCH0_MASK                    (0x8U)
#define SIM_PCE3_PWMBCH0_SHIFT                   (3U)
#define SIM_PCE3_PWMBCH0(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMBCH0_SHIFT)) & SIM_PCE3_PWMBCH0_MASK)
/*! PWMACH3 - PWMA Channel 3 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH3_MASK                    (0x10U)
#define SIM_PCE3_PWMACH3_SHIFT                   (4U)
#define SIM_PCE3_PWMACH3(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH3_SHIFT)) & SIM_PCE3_PWMACH3_MASK)
/*! PWMACH2 - PWMA Channel 2 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH2_MASK                    (0x20U)
#define SIM_PCE3_PWMACH2_SHIFT                   (5U)
#define SIM_PCE3_PWMACH2(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH2_SHIFT)) & SIM_PCE3_PWMACH2_MASK)
/*! PWMACH1 - PWMA Channel 1 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH1_MASK                    (0x40U)
#define SIM_PCE3_PWMACH1_SHIFT                   (6U)
#define SIM_PCE3_PWMACH1(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH1_SHIFT)) & SIM_PCE3_PWMACH1_MASK)
/*! PWMACH0 - PWMA Channel 0 IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_PWMACH0_MASK                    (0x80U)
#define SIM_PCE3_PWMACH0_SHIFT                   (7U)
#define SIM_PCE3_PWMACH0(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_PWMACH0_SHIFT)) & SIM_PCE3_PWMACH0_MASK)
/*! USB - USB IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_USB_MASK                        (0x100U)
#define SIM_PCE3_USB_SHIFT                       (8U)
#define SIM_PCE3_USB(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_USB_SHIFT)) & SIM_PCE3_USB_MASK)
/*! ROM - ROM IPBus Clock Enable
 *  0b0..The peripheral is not clocked.
 *  0b1..The peripheral is clocked.
 */
#define SIM_PCE3_ROM_MASK                        (0x200U)
#define SIM_PCE3_ROM_SHIFT                       (9U)
#define SIM_PCE3_ROM(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PCE3_ROM_SHIFT)) & SIM_PCE3_ROM_MASK)
/*! @} */

/*! @name SD0 - Peripheral Clock STOP Disable Register 0 */
/*! @{ */
/*! GPIOG - GPIOG IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOG_MASK                       (0x1U)
#define SIM_SD0_GPIOG_SHIFT                      (0U)
#define SIM_SD0_GPIOG(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOG_SHIFT)) & SIM_SD0_GPIOG_MASK)
/*! GPIOF - GPIOF IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOF_MASK                       (0x2U)
#define SIM_SD0_GPIOF_SHIFT                      (1U)
#define SIM_SD0_GPIOF(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOF_SHIFT)) & SIM_SD0_GPIOF_MASK)
/*! GPIOE - GPIOE IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOE_MASK                       (0x4U)
#define SIM_SD0_GPIOE_SHIFT                      (2U)
#define SIM_SD0_GPIOE(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOE_SHIFT)) & SIM_SD0_GPIOE_MASK)
/*! GPIOD - GPIOD IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOD_MASK                       (0x8U)
#define SIM_SD0_GPIOD_SHIFT                      (3U)
#define SIM_SD0_GPIOD(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOD_SHIFT)) & SIM_SD0_GPIOD_MASK)
/*! GPIOC - GPIOC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOC_MASK                       (0x10U)
#define SIM_SD0_GPIOC_SHIFT                      (4U)
#define SIM_SD0_GPIOC(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOC_SHIFT)) & SIM_SD0_GPIOC_MASK)
/*! GPIOB - GPIOB IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOB_MASK                       (0x20U)
#define SIM_SD0_GPIOB_SHIFT                      (5U)
#define SIM_SD0_GPIOB(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOB_SHIFT)) & SIM_SD0_GPIOB_MASK)
/*! GPIOA - GPIOA IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_GPIOA_MASK                       (0x40U)
#define SIM_SD0_GPIOA_SHIFT                      (6U)
#define SIM_SD0_GPIOA(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD0_GPIOA_SHIFT)) & SIM_SD0_GPIOA_MASK)
/*! TB3 - TMRB3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB3_MASK                         (0x100U)
#define SIM_SD0_TB3_SHIFT                        (8U)
#define SIM_SD0_TB3(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB3_SHIFT)) & SIM_SD0_TB3_MASK)
/*! TB2 - TMRB2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB2_MASK                         (0x200U)
#define SIM_SD0_TB2_SHIFT                        (9U)
#define SIM_SD0_TB2(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB2_SHIFT)) & SIM_SD0_TB2_MASK)
/*! TB1 - TMRB1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB1_MASK                         (0x400U)
#define SIM_SD0_TB1_SHIFT                        (10U)
#define SIM_SD0_TB1(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB1_SHIFT)) & SIM_SD0_TB1_MASK)
/*! TB0 - TMRB0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TB0_MASK                         (0x800U)
#define SIM_SD0_TB0_SHIFT                        (11U)
#define SIM_SD0_TB0(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TB0_SHIFT)) & SIM_SD0_TB0_MASK)
/*! TA3 - TMRA3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA3_MASK                         (0x1000U)
#define SIM_SD0_TA3_SHIFT                        (12U)
#define SIM_SD0_TA3(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA3_SHIFT)) & SIM_SD0_TA3_MASK)
/*! TA2 - TMRA2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA2_MASK                         (0x2000U)
#define SIM_SD0_TA2_SHIFT                        (13U)
#define SIM_SD0_TA2(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA2_SHIFT)) & SIM_SD0_TA2_MASK)
/*! TA1 - TMRA1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA1_MASK                         (0x4000U)
#define SIM_SD0_TA1_SHIFT                        (14U)
#define SIM_SD0_TA1(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA1_SHIFT)) & SIM_SD0_TA1_MASK)
/*! TA0 - TMRA0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD0_TA0_MASK                         (0x8000U)
#define SIM_SD0_TA0_SHIFT                        (15U)
#define SIM_SD0_TA0(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD0_TA0_SHIFT)) & SIM_SD0_TA0_MASK)
/*! @} */

/*! @name SD1 - Peripheral Clock STOP Disable Register 1 */
/*! @{ */
/*! FLEXCAN - FlexCAN IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_FLEXCAN_MASK                     (0x1U)
#define SIM_SD1_FLEXCAN_SHIFT                    (0U)
#define SIM_SD1_FLEXCAN(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD1_FLEXCAN_SHIFT)) & SIM_SD1_FLEXCAN_MASK)
/*! IIC1 - IIC1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode, but the IIC1 module will not enter stop mode.
 */
#define SIM_SD1_IIC1_MASK                        (0x20U)
#define SIM_SD1_IIC1_SHIFT                       (5U)
#define SIM_SD1_IIC1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_IIC1_SHIFT)) & SIM_SD1_IIC1_MASK)
/*! IIC0 - IIC0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode, but the IIC0 module will not enter stop mode.
 */
#define SIM_SD1_IIC0_MASK                        (0x40U)
#define SIM_SD1_IIC0_SHIFT                       (6U)
#define SIM_SD1_IIC0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_IIC0_SHIFT)) & SIM_SD1_IIC0_MASK)
/*! QSPI1 - QSPI1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_QSPI1_MASK                       (0x100U)
#define SIM_SD1_QSPI1_SHIFT                      (8U)
#define SIM_SD1_QSPI1(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD1_QSPI1_SHIFT)) & SIM_SD1_QSPI1_MASK)
/*! QSPI0 - QSPI0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_QSPI0_MASK                       (0x200U)
#define SIM_SD1_QSPI0_SHIFT                      (9U)
#define SIM_SD1_QSPI0(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_SD1_QSPI0_SHIFT)) & SIM_SD1_QSPI0_MASK)
/*! SCI2 - SCI2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI2_MASK                        (0x400U)
#define SIM_SD1_SCI2_SHIFT                       (10U)
#define SIM_SD1_SCI2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI2_SHIFT)) & SIM_SD1_SCI2_MASK)
/*! SCI1 - SCI1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI1_MASK                        (0x800U)
#define SIM_SD1_SCI1_SHIFT                       (11U)
#define SIM_SD1_SCI1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI1_SHIFT)) & SIM_SD1_SCI1_MASK)
/*! SCI0 - SCI0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_SCI0_MASK                        (0x1000U)
#define SIM_SD1_SCI0_SHIFT                       (12U)
#define SIM_SD1_SCI0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_SCI0_SHIFT)) & SIM_SD1_SCI0_MASK)
/*! DAC0 - DAC0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_DAC0_MASK                        (0x2000U)
#define SIM_SD1_DAC0_SHIFT                       (13U)
#define SIM_SD1_DAC0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_DAC0_SHIFT)) & SIM_SD1_DAC0_MASK)
/*! DAC1 - DAC1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD1_DAC1_MASK                        (0x4000U)
#define SIM_SD1_DAC1_SHIFT                       (14U)
#define SIM_SD1_DAC1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD1_DAC1_SHIFT)) & SIM_SD1_DAC1_MASK)
/*! @} */

/*! @name SD2 - Peripheral Clock STOP Disable Register 2 */
/*! @{ */
/*! PIT1 - Programmable Interval Timer IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_PIT1_MASK                        (0x4U)
#define SIM_SD2_PIT1_SHIFT                       (2U)
#define SIM_SD2_PIT1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_PIT1_SHIFT)) & SIM_SD2_PIT1_MASK)
/*! PIT0 - Programmable Interval Timer IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_PIT0_MASK                        (0x8U)
#define SIM_SD2_PIT0_SHIFT                       (3U)
#define SIM_SD2_PIT0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_PIT0_SHIFT)) & SIM_SD2_PIT0_MASK)
/*! CRC - CRC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CRC_MASK                         (0x20U)
#define SIM_SD2_CRC_SHIFT                        (5U)
#define SIM_SD2_CRC(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CRC_SHIFT)) & SIM_SD2_CRC_MASK)
/*! CYCADC - Cyclic ADC IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CYCADC_MASK                      (0x80U)
#define SIM_SD2_CYCADC_SHIFT                     (7U)
#define SIM_SD2_CYCADC(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CYCADC_SHIFT)) & SIM_SD2_CYCADC_MASK)
/*! CMPD - CMPD IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPD_MASK                        (0x200U)
#define SIM_SD2_CMPD_SHIFT                       (9U)
#define SIM_SD2_CMPD(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPD_SHIFT)) & SIM_SD2_CMPD_MASK)
/*! CMPC - CMPC IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPC_MASK                        (0x400U)
#define SIM_SD2_CMPC_SHIFT                       (10U)
#define SIM_SD2_CMPC(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPC_SHIFT)) & SIM_SD2_CMPC_MASK)
/*! CMPB - CMPB IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPB_MASK                        (0x800U)
#define SIM_SD2_CMPB_SHIFT                       (11U)
#define SIM_SD2_CMPB(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPB_SHIFT)) & SIM_SD2_CMPB_MASK)
/*! CMPA - CMPA IPBus STOP Disable (disables both CMP and 8-bit reference DAC)
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD2_CMPA_MASK                        (0x1000U)
#define SIM_SD2_CMPA_SHIFT                       (12U)
#define SIM_SD2_CMPA(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_SD2_CMPA_SHIFT)) & SIM_SD2_CMPA_MASK)
/*! @} */

/*! @name SD3 - Peripheral Clock STOP Disable Register 3 */
/*! @{ */
/*! PWMBCH3 - PWMB Channel 3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH3_MASK                     (0x1U)
#define SIM_SD3_PWMBCH3_SHIFT                    (0U)
#define SIM_SD3_PWMBCH3(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH3_SHIFT)) & SIM_SD3_PWMBCH3_MASK)
/*! PWMBCH2 - PWMB Channel 2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH2_MASK                     (0x2U)
#define SIM_SD3_PWMBCH2_SHIFT                    (1U)
#define SIM_SD3_PWMBCH2(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH2_SHIFT)) & SIM_SD3_PWMBCH2_MASK)
/*! PWMBCH1 - PWMB Channel 1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH1_MASK                     (0x4U)
#define SIM_SD3_PWMBCH1_SHIFT                    (2U)
#define SIM_SD3_PWMBCH1(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH1_SHIFT)) & SIM_SD3_PWMBCH1_MASK)
/*! PWMBCH0 - PWMB Channel 0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMBCH0_MASK                     (0x8U)
#define SIM_SD3_PWMBCH0_SHIFT                    (3U)
#define SIM_SD3_PWMBCH0(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMBCH0_SHIFT)) & SIM_SD3_PWMBCH0_MASK)
/*! PWMACH3 - PWMA Channel 3 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH3_MASK                     (0x10U)
#define SIM_SD3_PWMACH3_SHIFT                    (4U)
#define SIM_SD3_PWMACH3(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH3_SHIFT)) & SIM_SD3_PWMACH3_MASK)
/*! PWMACH2 - PWMA Channel 2 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH2_MASK                     (0x20U)
#define SIM_SD3_PWMACH2_SHIFT                    (5U)
#define SIM_SD3_PWMACH2(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH2_SHIFT)) & SIM_SD3_PWMACH2_MASK)
/*! PWMACH1 - PWMA Channel 1 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH1_MASK                     (0x40U)
#define SIM_SD3_PWMACH1_SHIFT                    (6U)
#define SIM_SD3_PWMACH1(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH1_SHIFT)) & SIM_SD3_PWMACH1_MASK)
/*! PWMACH0 - PWMA Channel 0 IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_PWMACH0_MASK                     (0x80U)
#define SIM_SD3_PWMACH0_SHIFT                    (7U)
#define SIM_SD3_PWMACH0(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_SD3_PWMACH0_SHIFT)) & SIM_SD3_PWMACH0_MASK)
/*! USB - USB IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_USB_MASK                         (0x100U)
#define SIM_SD3_USB_SHIFT                        (8U)
#define SIM_SD3_USB(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD3_USB_SHIFT)) & SIM_SD3_USB_MASK)
/*! ROM - ROM IPBus STOP Disable
 *  0b0..The peripheral is not clocked in stop mode.
 *  0b1..The peripheral is clocked in stop mode.
 */
#define SIM_SD3_ROM_MASK                         (0x200U)
#define SIM_SD3_ROM_SHIFT                        (9U)
#define SIM_SD3_ROM(x)                           (((uint16_t)(((uint16_t)(x)) << SIM_SD3_ROM_SHIFT)) & SIM_SD3_ROM_MASK)
/*! @} */

/*! @name IOSAHI - I/O Short Address Location Register */
/*! @{ */
#define SIM_IOSAHI_ISAL23_22_MASK                (0x3U)
#define SIM_IOSAHI_ISAL23_22_SHIFT               (0U)
#define SIM_IOSAHI_ISAL23_22(x)                  (((uint16_t)(((uint16_t)(x)) << SIM_IOSAHI_ISAL23_22_SHIFT)) & SIM_IOSAHI_ISAL23_22_MASK)
/*! @} */

/*! @name IOSALO - I/O Short Address Location Register */
/*! @{ */
#define SIM_IOSALO_ISAL21_6_MASK                 (0xFFFFU)
#define SIM_IOSALO_ISAL21_6_SHIFT                (0U)
#define SIM_IOSALO_ISAL21_6(x)                   (((uint16_t)(((uint16_t)(x)) << SIM_IOSALO_ISAL21_6_SHIFT)) & SIM_IOSALO_ISAL21_6_MASK)
/*! @} */

/*! @name PROT - Protection Register */
/*! @{ */
/*! GIPSP - GPIO and Internal Peripheral Select Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_GIPSP_MASK                      (0x3U)
#define SIM_PROT_GIPSP_SHIFT                     (0U)
#define SIM_PROT_GIPSP(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PROT_GIPSP_SHIFT)) & SIM_PROT_GIPSP_MASK)
/*! PCEP - Peripheral Clock Enable Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_PCEP_MASK                       (0xCU)
#define SIM_PROT_PCEP_SHIFT                      (2U)
#define SIM_PROT_PCEP(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PROT_PCEP_SHIFT)) & SIM_PROT_PCEP_MASK)
/*! GDP - GPIO Port D Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_GDP_MASK                        (0x30U)
#define SIM_PROT_GDP_SHIFT                       (4U)
#define SIM_PROT_GDP(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PROT_GDP_SHIFT)) & SIM_PROT_GDP_MASK)
/*! PMODE - Power Mode Control Write Protection
 *  0b00..Write protection off (default).
 *  0b01..Write protection on.
 *  0b10..Write protection off and locked until chip reset.
 *  0b11..Write protection on and locked until chip reset.
 */
#define SIM_PROT_PMODE_MASK                      (0xC0U)
#define SIM_PROT_PMODE_SHIFT                     (6U)
#define SIM_PROT_PMODE(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PROT_PMODE_SHIFT)) & SIM_PROT_PMODE_MASK)
/*! @} */

/*! @name GPSAL - GPIOA LSBs Peripheral Select Register */
/*! @{ */
/*! A0 - Configure GPIO A0
 *  0b0..Function = ANA0/CMPA_IN3; Peripheral = ADC/CMPA; Direction = IN
 *  0b1..Function = CMPC_O; Peripheral = CMPC; Direction = OUT
 */
#define SIM_GPSAL_A0_MASK                        (0x1U)
#define SIM_GPSAL_A0_SHIFT                       (0U)
#define SIM_GPSAL_A0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSAL_A0_SHIFT)) & SIM_GPSAL_A0_MASK)
/*! @} */

/*! @name GPSAH - GPIOA MSBs Peripheral Select Register */
/*! @{ */
/*! A11 - Configure GPIO A11
 *  0b00..Function = CMPC_O; Peripheral = CMPC; Direction = OUT
 *  0b01..Function = XB_IN9; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 *  0b11..Function = USB_SOFOUT; Peripheral = USB; Direction = OUT
 */
#define SIM_GPSAH_A11_MASK                       (0xC0U)
#define SIM_GPSAH_A11_SHIFT                      (6U)
#define SIM_GPSAH_A11(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSAH_A11_SHIFT)) & SIM_GPSAH_A11_MASK)
/*! @} */

/*! @name GPSBL - GPIOB LSBs Peripheral Select Register */
/*! @{ */
/*! B1 - Configure GPIO B1
 *  0b0..Function = ANB1/CMPB_IN0; Peripheral = ADC/CMPB; Direction = IN
 *  0b1..Function = DACB_O; Peripheral = DAC; Direction = OUT
 */
#define SIM_GPSBL_B1_MASK                        (0x4U)
#define SIM_GPSBL_B1_SHIFT                       (2U)
#define SIM_GPSBL_B1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSBL_B1_SHIFT)) & SIM_GPSBL_B1_MASK)
/*! @} */

/*! @name GPSBH - GPIOB MSBs Peripheral Select Register */
/*! @{ */
/*! B8 - Configure GPIO B8
 *  0b00..Function = CMPD_O; Peripheral = CMPD; Direction = OUT
 *  0b01..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 *  0b11..reserved
 */
#define SIM_GPSBH_B8_MASK                        (0x3U)
#define SIM_GPSBH_B8_SHIFT                       (0U)
#define SIM_GPSBH_B8(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSBH_B8_SHIFT)) & SIM_GPSBH_B8_MASK)
/*! @} */

/*! @name GPSCL - GPIOC LSBs Peripheral Select Register */
/*! @{ */
/*! C0 - Configure GPIO C0
 *  0b0..Function = EXTAL; Peripheral = OSC; Direction = IN
 *  0b1..Function = CLKIN0; Peripheral = OCCS; Direction = IN
 */
#define SIM_GPSCL_C0_MASK                        (0x1U)
#define SIM_GPSCL_C0_SHIFT                       (0U)
#define SIM_GPSCL_C0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C0_SHIFT)) & SIM_GPSCL_C0_MASK)
/*! C2 - Configure GPIO C2
 *  0b00..Function = TXD0; Peripheral = SCI0; Direction = IO
 *  0b01..Function = TB0; Peripheral = TMRB; Direction = IO
 *  0b10..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 *  0b11..Function = CLKOUT0; Peripheral = OCCS; Direction = OUT
 */
#define SIM_GPSCL_C2_MASK                        (0x30U)
#define SIM_GPSCL_C2_SHIFT                       (4U)
#define SIM_GPSCL_C2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C2_SHIFT)) & SIM_GPSCL_C2_MASK)
/*! C3
 *  0b00..Function = TA0; Peripheral = TMRA; Direction = IO
 *  0b01..Function = CMPA_O; Peripheral = CMPA; Direction = OUT
 *  0b10..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b11..Function = CLKIN1; Peripheral = OCCS; Direction = IN
 */
#define SIM_GPSCL_C3_MASK                        (0xC0U)
#define SIM_GPSCL_C3_SHIFT                       (6U)
#define SIM_GPSCL_C3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C3_SHIFT)) & SIM_GPSCL_C3_MASK)
/*! C4 - Configure GPIO C4
 *  0b00..Function = TA1; Peripheral = TMRA; Direction = IO
 *  0b01..Function = CMPB_O; Peripheral = CMPB; Direction = OUT
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Function = EWM_OUT_B; Peripheral = EWM; Direction = OUT
 */
#define SIM_GPSCL_C4_MASK                        (0x300U)
#define SIM_GPSCL_C4_SHIFT                       (8U)
#define SIM_GPSCL_C4(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C4_SHIFT)) & SIM_GPSCL_C4_MASK)
/*! C5 - Configure GPIO C5
 *  0b0..Function = DACA_O; Peripheral = DAC; Direction = OUT
 *  0b1..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSCL_C5_MASK                        (0x400U)
#define SIM_GPSCL_C5_SHIFT                       (10U)
#define SIM_GPSCL_C5(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C5_SHIFT)) & SIM_GPSCL_C5_MASK)
/*! C6 - Configure GPIO C6
 *  0b00..Function = TA2; Peripheral = TMRA; Direction = IO
 *  0b01..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 *  0b10..Function = CMP_REF; Peripheral = CMP A/B/C/D; Direction = IN
 *  0b11..Function = SS0_B; Peripheral = SPI0; Direction = IO
 */
#define SIM_GPSCL_C6_MASK                        (0x3000U)
#define SIM_GPSCL_C6_SHIFT                       (12U)
#define SIM_GPSCL_C6(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C6_SHIFT)) & SIM_GPSCL_C6_MASK)
/*! C7 - Configure GPIO C7
 *  0b00..Function = SS0_B; Peripheral = SPI0; Direction = IO
 *  0b01..Function = TXD0; Peripheral = SCI0; Direction = OUT
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCL_C7_MASK                        (0xC000U)
#define SIM_GPSCL_C7_SHIFT                       (14U)
#define SIM_GPSCL_C7(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCL_C7_SHIFT)) & SIM_GPSCL_C7_MASK)
/*! @} */

/*! @name GPSCH - GPIOC MSBs Peripheral Select Register */
/*! @{ */
/*! C8 - Configure GPIO C8
 *  0b00..Function = MISO0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b10..Function = XB_IN9; Peripheral = XBAR; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSCH_C8_MASK                        (0x3U)
#define SIM_GPSCH_C8_SHIFT                       (0U)
#define SIM_GPSCH_C8(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C8_SHIFT)) & SIM_GPSCH_C8_MASK)
/*! C9 - Configure GPIO C9
 *  0b00..Function = SCLK0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = TXD0; Peripheral = SCI0; Direction = OUT
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCH_C9_MASK                        (0xCU)
#define SIM_GPSCH_C9_SHIFT                       (2U)
#define SIM_GPSCH_C9(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C9_SHIFT)) & SIM_GPSCH_C9_MASK)
/*! C10 - Configure GPIO C10
 *  0b00..Function = MOSI0; Peripheral = SPI0; Direction = IO
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = MISO0; Peripheral = SPI0; Direction = IO
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSCH_C10_MASK                       (0x30U)
#define SIM_GPSCH_C10_SHIFT                      (4U)
#define SIM_GPSCH_C10(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C10_SHIFT)) & SIM_GPSCH_C10_MASK)
/*! C11 - Configure GPIO C11
 *  0b00..Function = CANTX; Peripheral = FlexCAN; Direction = OUT
 *  0b01..Function = SCL1; Peripheral = IIC1; Direction = IO
 *  0b10..Function = TXD1; Peripheral = SCI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSCH_C11_MASK                       (0xC0U)
#define SIM_GPSCH_C11_SHIFT                      (6U)
#define SIM_GPSCH_C11(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C11_SHIFT)) & SIM_GPSCH_C11_MASK)
/*! C12 - Configure GPIO C12
 *  0b00..Function = CANRX; Peripheral = FlexCAN; Direction = IN
 *  0b01..Function = SDA1; Peripheral = IIC1; Direction = IO
 *  0b10..Function = RXD1; Peripheral = SCI1; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSCH_C12_MASK                       (0x300U)
#define SIM_GPSCH_C12_SHIFT                      (8U)
#define SIM_GPSCH_C12(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C12_SHIFT)) & SIM_GPSCH_C12_MASK)
/*! C13 - Configure GPIO C13
 *  0b00..Function = TA3; Peripheral = TMRA; Direction = IO
 *  0b01..Function = XB_IN6; Peripheral = XBAR; Direction = IN
 *  0b10..Function = EWM_OUT_B; Peripheral = EWM; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSCH_C13_MASK                       (0xC00U)
#define SIM_GPSCH_C13_SHIFT                      (10U)
#define SIM_GPSCH_C13(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C13_SHIFT)) & SIM_GPSCH_C13_MASK)
/*! C14 - Configure GPIO C14
 *  0b00..Function = SDA0; Peripheral = IIC0; Direction = IO
 *  0b01..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_FAULT4; Peripheral = PWMA; Direction = IN
 *  0b11..reserved
 */
#define SIM_GPSCH_C14_MASK                       (0x3000U)
#define SIM_GPSCH_C14_SHIFT                      (12U)
#define SIM_GPSCH_C14(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C14_SHIFT)) & SIM_GPSCH_C14_MASK)
/*! C15 - Configure GPIO C15
 *  0b00..Function = SCL0; Peripheral = IIC0; Direction = IO
 *  0b01..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_FAULT5; Peripheral = PWMA; Direction = IN
 *  0b11..reserved
 */
#define SIM_GPSCH_C15_MASK                       (0xC000U)
#define SIM_GPSCH_C15_SHIFT                      (14U)
#define SIM_GPSCH_C15(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSCH_C15_SHIFT)) & SIM_GPSCH_C15_MASK)
/*! @} */

/*! @name GPSDL - GPIOD LSBs Peripheral Select Register */
/*! @{ */
/*! D5 - Configure GPIO D5
 *  0b00..Function = RXD2; Peripheral = SCI2; Direction = IN
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSDL_D5_MASK                        (0xC00U)
#define SIM_GPSDL_D5_SHIFT                       (10U)
#define SIM_GPSDL_D5(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D5_SHIFT)) & SIM_GPSDL_D5_MASK)
/*! D6 - Configure GPIO D6
 *  0b00..Function = TXD2; Peripheral = SCI2; Direction = OUT
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSDL_D6_MASK                        (0x3000U)
#define SIM_GPSDL_D6_SHIFT                       (12U)
#define SIM_GPSDL_D6(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D6_SHIFT)) & SIM_GPSDL_D6_MASK)
/*! D7 - Configure GPIO D7
 *  0b00..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 *  0b01..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 *  0b10..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSDL_D7_MASK                        (0xC000U)
#define SIM_GPSDL_D7_SHIFT                       (14U)
#define SIM_GPSDL_D7(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSDL_D7_SHIFT)) & SIM_GPSDL_D7_MASK)
/*! @} */

/*! @name GPSEL - GPIOE LSBs Peripheral Select Register */
/*! @{ */
/*! E0 - Configure GPIO E0
 *  0b00..Function = PWMA_0B; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E0_MASK                        (0x3U)
#define SIM_GPSEL_E0_SHIFT                       (0U)
#define SIM_GPSEL_E0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E0_SHIFT)) & SIM_GPSEL_E0_MASK)
/*! E1 - Configure GPIO E1
 *  0b00..Function = PWMA_0A; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E1_MASK                        (0xCU)
#define SIM_GPSEL_E1_SHIFT                       (2U)
#define SIM_GPSEL_E1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E1_SHIFT)) & SIM_GPSEL_E1_MASK)
/*! E2 - Configure GPIO E2
 *  0b00..Function = PWMA_1B; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E2_MASK                        (0x30U)
#define SIM_GPSEL_E2_SHIFT                       (4U)
#define SIM_GPSEL_E2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E2_SHIFT)) & SIM_GPSEL_E2_MASK)
/*! E3 - Configure GPIO E3
 *  0b00..Function = PWMA_1A; Peripheral = PWMA; Direction = IO
 *  0b01..reserved
 *  0b10..reserved
 *  0b11..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E3_MASK                        (0xC0U)
#define SIM_GPSEL_E3_SHIFT                       (6U)
#define SIM_GPSEL_E3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E3_SHIFT)) & SIM_GPSEL_E3_MASK)
/*! E4 - Configure GPIO E4
 *  0b00..Function = PWMA_2B; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E4_MASK                        (0x300U)
#define SIM_GPSEL_E4_SHIFT                       (8U)
#define SIM_GPSEL_E4(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E4_SHIFT)) & SIM_GPSEL_E4_MASK)
/*! E5 - Configure GPIO E5
 *  0b00..Function = PWMA_2A; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E5_MASK                        (0xC00U)
#define SIM_GPSEL_E5_SHIFT                       (10U)
#define SIM_GPSEL_E5(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E5_SHIFT)) & SIM_GPSEL_E5_MASK)
/*! E6 - Configure GPIO E6
 *  0b00..Function = PWMA_3B; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN4; Peripheral = XBAR; Direction = IN
 *  0b10..Function = PWMB_2B; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E6_MASK                        (0x3000U)
#define SIM_GPSEL_E6_SHIFT                       (12U)
#define SIM_GPSEL_E6(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E6_SHIFT)) & SIM_GPSEL_E6_MASK)
/*! E7 - Configure GPIO E7
 *  0b00..Function = PWMA_3A; Peripheral = PWMA; Direction = IO
 *  0b01..Function = XB_IN5; Peripheral = XBAR; Direction = IN
 *  0b10..Function = PWMB_2A; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEL_E7_MASK                        (0xC000U)
#define SIM_GPSEL_E7_SHIFT                       (14U)
#define SIM_GPSEL_E7(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEL_E7_SHIFT)) & SIM_GPSEL_E7_MASK)
/*! @} */

/*! @name GPSEH - GPIOE MSBs Peripheral Select Register */
/*! @{ */
/*! E8 - Configure GPIO E8
 *  0b00..Function = PWMB_2B; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT0; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEH_E8_MASK                        (0x3U)
#define SIM_GPSEH_E8_SHIFT                       (0U)
#define SIM_GPSEH_E8(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEH_E8_SHIFT)) & SIM_GPSEH_E8_MASK)
/*! E9 - Configure GPIO E9
 *  0b00..Function = PWMB_2A; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT1; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSEH_E9_MASK                        (0xCU)
#define SIM_GPSEH_E9_SHIFT                       (2U)
#define SIM_GPSEH_E9(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSEH_E9_SHIFT)) & SIM_GPSEH_E9_MASK)
/*! @} */

/*! @name GPSFL - GPIOF LSBs Peripheral Select Register */
/*! @{ */
/*! F0 - Configure GPIO F0
 *  0b00..Function = XB_IN6; Peripheral = XBAR; Direction = IN
 *  0b01..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b10..Function = SCLK1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSFL_F0_MASK                        (0x3U)
#define SIM_GPSFL_F0_SHIFT                       (0U)
#define SIM_GPSFL_F0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F0_SHIFT)) & SIM_GPSFL_F0_MASK)
/*! F1 - Configure GPIO F1
 *  0b00..Function = CLKOUT1; Peripheral = OCCS; Direction = OUT
 *  0b01..Function = XB_IN7; Peripheral = XBAR; Direction = IN
 *  0b10..Function = CMPD_O; Peripheral = CMPD; Direction = OUT
 *  0b11..Reserved
 */
#define SIM_GPSFL_F1_MASK                        (0xCU)
#define SIM_GPSFL_F1_SHIFT                       (2U)
#define SIM_GPSFL_F1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F1_SHIFT)) & SIM_GPSFL_F1_MASK)
/*! F2 - Configure GPIO F2
 *  0b00..Function = SCL1; Peripheral = IIC1; Direction = IO
 *  0b01..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b11..reserved
 */
#define SIM_GPSFL_F2_MASK                        (0x30U)
#define SIM_GPSFL_F2_SHIFT                       (4U)
#define SIM_GPSFL_F2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F2_SHIFT)) & SIM_GPSFL_F2_MASK)
/*! F3 - Configure GPIO F3
 *  0b00..Function = SDA1; Peripheral = IIC1; Direction = IO
 *  0b01..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b11..reserved
 */
#define SIM_GPSFL_F3_MASK                        (0xC0U)
#define SIM_GPSFL_F3_SHIFT                       (6U)
#define SIM_GPSFL_F3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F3_SHIFT)) & SIM_GPSFL_F3_MASK)
/*! F4 - Configure GPIO F4
 *  0b00..Function = TXD1; Peripheral = SCI1; Direction = IO
 *  0b01..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_0X; Peripheral = PWMA; Direction = IO
 *  0b11..Function = PWMA_FAULT6; Peripheral = PWMA; Direction = IN
 */
#define SIM_GPSFL_F4_MASK                        (0x300U)
#define SIM_GPSFL_F4_SHIFT                       (8U)
#define SIM_GPSFL_F4(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F4_SHIFT)) & SIM_GPSFL_F4_MASK)
/*! F5 - Configure GPIO F5
 *  0b00..Function = RXD1; Peripheral = SCI1; Direction = IN
 *  0b01..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b10..Function = PWMA_1X; Peripheral = PWMA; Direction = IO
 *  0b11..Function = PWMA_FAULT7; Peripheral = PWMA; Direction = IN
 */
#define SIM_GPSFL_F5_MASK                        (0xC00U)
#define SIM_GPSFL_F5_SHIFT                       (10U)
#define SIM_GPSFL_F5(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F5_SHIFT)) & SIM_GPSFL_F5_MASK)
/*! F6 - Configure GPIO F6
 *  0b00..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b01..Function = PWMA_3X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = PWMB_3X; Peripheral = PWMB; Direction = IO
 *  0b11..Function = XB_IN2; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFL_F6_MASK                        (0x3000U)
#define SIM_GPSFL_F6_SHIFT                       (12U)
#define SIM_GPSFL_F6(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F6_SHIFT)) & SIM_GPSFL_F6_MASK)
/*! F7 - Configure GPIO F7
 *  0b00..Function = TB3; Peripheral = TMRB; Direction = IO
 *  0b01..Function = CMPC_O; Peripheral = HSCMPC; Direction = OUT
 *  0b10..Function = SS1_B; Peripheral = SPI1; Direction = IO
 *  0b11..Function = XB_IN3; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFL_F7_MASK                        (0xC000U)
#define SIM_GPSFL_F7_SHIFT                       (14U)
#define SIM_GPSFL_F7(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFL_F7_SHIFT)) & SIM_GPSFL_F7_MASK)
/*! @} */

/*! @name GPSFH - GPIOF MSBs Peripheral Select Register */
/*! @{ */
/*! F8 - Configure GPIO F8
 *  0b00..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b01..Function =TB1; Peripheral = TMRB; Direction = IO
 *  0b10..Function = CMPD_O; Peripheral = HSCMPD; Direction = OUT
 *  0b11..Function = PWMA_2X; Peripheral = PWMA; Direction = IO
 */
#define SIM_GPSFH_F8_MASK                        (0x3U)
#define SIM_GPSFH_F8_SHIFT                       (0U)
#define SIM_GPSFH_F8(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F8_SHIFT)) & SIM_GPSFH_F8_MASK)
/*! F9 - Configure GPIO F9
 *  0b00..Function = RXD2; Peripheral = SCI2; Direction = IN
 *  0b01..Function = PWMA_FAULT7; Peripheral = PWMA; Direction = IN
 *  0b10..Function = PWMB_FAULT7; Peripheral = PWMB; Direction = IN
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSFH_F9_MASK                        (0xCU)
#define SIM_GPSFH_F9_SHIFT                       (2U)
#define SIM_GPSFH_F9(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F9_SHIFT)) & SIM_GPSFH_F9_MASK)
/*! F10 - Configure GPIO F10
 *  0b00..Function = TXD2; Peripheral = SCI2; Direction = IO
 *  0b01..Function = PWMA_FAULT6; Peripheral = PWMA; Direction = IN
 *  0b10..Function = PWMB_FAULT6; Peripheral = PWMB; Direction = IN
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSFH_F10_MASK                       (0x30U)
#define SIM_GPSFH_F10_SHIFT                      (4U)
#define SIM_GPSFH_F10(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F10_SHIFT)) & SIM_GPSFH_F10_MASK)
/*! F11 - Configure GPIO F11
 *  0b0..Function = TXD0; Peripheral = SCI0; Direction = IO
 *  0b1..Function = XB_IN11; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFH_F11_MASK                       (0x40U)
#define SIM_GPSFH_F11_SHIFT                      (6U)
#define SIM_GPSFH_F11(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F11_SHIFT)) & SIM_GPSFH_F11_MASK)
/*! F12 - Configure GPIO F12
 *  0b0..Function = MISO1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT2; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F12_MASK                       (0x100U)
#define SIM_GPSFH_F12_SHIFT                      (8U)
#define SIM_GPSFH_F12(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F12_SHIFT)) & SIM_GPSFH_F12_MASK)
/*! F13 - Configure GPIO F13
 *  0b0..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT1; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F13_MASK                       (0x400U)
#define SIM_GPSFH_F13_SHIFT                      (10U)
#define SIM_GPSFH_F13(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F13_SHIFT)) & SIM_GPSFH_F13_MASK)
/*! F14 - Configure GPIO F14
 *  0b0..Function = SCLK1; Peripheral = SPI1; Direction = IO
 *  0b1..Function = PWMB_FAULT0; Peripheral = PWMB; Direction = IN
 */
#define SIM_GPSFH_F14_MASK                       (0x1000U)
#define SIM_GPSFH_F14_SHIFT                      (12U)
#define SIM_GPSFH_F14(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F14_SHIFT)) & SIM_GPSFH_F14_MASK)
/*! F15 - Configure GPIO F15
 *  0b0..Function = RXD0; Peripheral = SCI0; Direction = IN
 *  0b1..Function = XB_IN10; Peripheral = XBAR; Direction = IN
 */
#define SIM_GPSFH_F15_MASK                       (0x4000U)
#define SIM_GPSFH_F15_SHIFT                      (14U)
#define SIM_GPSFH_F15(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSFH_F15_SHIFT)) & SIM_GPSFH_F15_MASK)
/*! @} */

/*! @name GPSGL - GPIOG LSBs Peripheral Select Register */
/*! @{ */
/*! G0 - Configure GPIO G0
 *  0b0..Function = PWMB_1B; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT6; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G0_MASK                        (0x1U)
#define SIM_GPSGL_G0_SHIFT                       (0U)
#define SIM_GPSGL_G0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G0_SHIFT)) & SIM_GPSGL_G0_MASK)
/*! G1 - Configure GPIO G1
 *  0b0..Function = PWMB_1A; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT7; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G1_MASK                        (0x4U)
#define SIM_GPSGL_G1_SHIFT                       (2U)
#define SIM_GPSGL_G1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G1_SHIFT)) & SIM_GPSGL_G1_MASK)
/*! G2 - Configure GPIO G2
 *  0b0..Function = PWMB_0B; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT4; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G2_MASK                        (0x10U)
#define SIM_GPSGL_G2_SHIFT                       (4U)
#define SIM_GPSGL_G2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G2_SHIFT)) & SIM_GPSGL_G2_MASK)
/*! G3 - Configure GPIO G3
 *  0b0..Function = PWMB_0A; Peripheral = PWMB; Direction = IO
 *  0b1..Function = XB_OUT5; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G3_MASK                        (0x40U)
#define SIM_GPSGL_G3_SHIFT                       (6U)
#define SIM_GPSGL_G3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G3_SHIFT)) & SIM_GPSGL_G3_MASK)
/*! G4 - Configure GPIO G4
 *  0b00..Function = PWMB_3B; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT2; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G4_MASK                        (0x300U)
#define SIM_GPSGL_G4_SHIFT                       (8U)
#define SIM_GPSGL_G4(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G4_SHIFT)) & SIM_GPSGL_G4_MASK)
/*! G5 - Configure GPIO G5
 *  0b00..Function = PWMB_3A; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_FAULT3; Peripheral = PWMA; Direction = IN
 *  0b10..reserved
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G5_MASK                        (0xC00U)
#define SIM_GPSGL_G5_SHIFT                       (10U)
#define SIM_GPSGL_G5(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G5_SHIFT)) & SIM_GPSGL_G5_MASK)
/*! G6 - Configure GPIO G6
 *  0b00..Function = PWMA_FAULT4; Peripheral = PWMA; Direction = IN
 *  0b01..Function = PWMB_FAULT4; Peripheral = PWMB; Direction = IN
 *  0b10..Function = TB2; Peripheral = TMRB; Direction = IO
 *  0b11..Function = XB_OUT8; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGL_G6_MASK                        (0x3000U)
#define SIM_GPSGL_G6_SHIFT                       (12U)
#define SIM_GPSGL_G6(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G6_SHIFT)) & SIM_GPSGL_G6_MASK)
/*! G7 - Configure GPIO G7
 *  0b00..Function = PWMA_FAULT5; Peripheral = PWMA; Direction = IN
 *  0b01..Function = PWMB_FAULT5; Peripheral = PWMB; Direction = IN
 *  0b10..Function = XB_OUT9; Peripheral = XBAR; Direction = OUT
 *  0b11..Function = CLKIN2; Peripheral = USB; Direction = IN
 */
#define SIM_GPSGL_G7_MASK                        (0xC000U)
#define SIM_GPSGL_G7_SHIFT                       (14U)
#define SIM_GPSGL_G7(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGL_G7_SHIFT)) & SIM_GPSGL_G7_MASK)
/*! @} */

/*! @name GPSGH - GPIOG MSBs Peripheral Select Register */
/*! @{ */
/*! G8 - Configure GPIO G8
 *  0b00..Function = PWMB_0X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_0X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = TA2; Peripheral = TMRA; Direction = IO
 *  0b11..Function = XB_OUT10; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGH_G8_MASK                        (0x3U)
#define SIM_GPSGH_G8_SHIFT                       (0U)
#define SIM_GPSGH_G8(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G8_SHIFT)) & SIM_GPSGH_G8_MASK)
/*! G9 - Configure GPIO G9
 *  0b00..Function = PWMB_1X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_1X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = TA3; Peripheral = TMRA; Direction = IO
 *  0b11..Function = XB_OUT11; Peripheral = XBAR; Direction = OUT
 */
#define SIM_GPSGH_G9_MASK                        (0xCU)
#define SIM_GPSGH_G9_SHIFT                       (2U)
#define SIM_GPSGH_G9(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G9_SHIFT)) & SIM_GPSGH_G9_MASK)
/*! G10 - Configure GPIO G10
 *  0b00..Function = PWMB_2X; Peripheral = PWMB; Direction = IO
 *  0b01..Function = PWMA_2X; Peripheral = PWMA; Direction = IO
 *  0b10..Function = XB_IN8; Peripheral = XBAR; Direction = IN
 *  0b11..Reserved
 */
#define SIM_GPSGH_G10_MASK                       (0x30U)
#define SIM_GPSGH_G10_SHIFT                      (4U)
#define SIM_GPSGH_G10(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G10_SHIFT)) & SIM_GPSGH_G10_MASK)
/*! G11 - Configure GPIO G11
 *  0b00..Function = TB3; Peripheral = TMRB; Direction = IO
 *  0b01..Function = CLKOUT0; Peripheral = OCCS; Direction = OUT
 *  0b10..Function = MOSI1; Peripheral = SPI1; Direction = IO
 *  0b11..Reserved
 */
#define SIM_GPSGH_G11_MASK                       (0xC0U)
#define SIM_GPSGH_G11_SHIFT                      (6U)
#define SIM_GPSGH_G11(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_GPSGH_G11_SHIFT)) & SIM_GPSGH_G11_MASK)
/*! @} */

/*! @name IPS0 - Internal Peripheral Select Register 0 */
/*! @{ */
/*! PWMAF0 - Select PWMA Fault 0 Input
 *  0b0..Function = GPIOE8; Peripheral = GPIOE; Direction = IN
 *  0b1..Function = XB_OUT29; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF0_MASK                     (0x1U)
#define SIM_IPS0_PWMAF0_SHIFT                    (0U)
#define SIM_IPS0_PWMAF0(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF0_SHIFT)) & SIM_IPS0_PWMAF0_MASK)
/*! PWMAF1 - Select PWMA Fault 1 Input
 *  0b0..Function = GPIOE9; Peripheral = GPIOE; Direction = IN
 *  0b1..Function = XB_OUT30; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF1_MASK                     (0x2U)
#define SIM_IPS0_PWMAF1_SHIFT                    (1U)
#define SIM_IPS0_PWMAF1(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF1_SHIFT)) & SIM_IPS0_PWMAF1_MASK)
/*! PWMAF2 - Select PWMA Fault 2 Input
 *  0b0..Function = GPIOG4; Peripheral = GPIOG; Direction = IN
 *  0b1..Function = XB_OUT31; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF2_MASK                     (0x4U)
#define SIM_IPS0_PWMAF2_SHIFT                    (2U)
#define SIM_IPS0_PWMAF2(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF2_SHIFT)) & SIM_IPS0_PWMAF2_MASK)
/*! PWMAF3 - Select PWMA Fault 3 Input
 *  0b0..Function = GPIOG5; Peripheral = GPIOG; Direction = IN
 *  0b1..Function = XB_OUT32; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMAF3_MASK                     (0x8U)
#define SIM_IPS0_PWMAF3_SHIFT                    (3U)
#define SIM_IPS0_PWMAF3(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMAF3_SHIFT)) & SIM_IPS0_PWMAF3_MASK)
/*! PWMBF0 - Select PWMB Fault 0 Input
 *  0b0..Function = GPIOF14; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT29; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF0_MASK                     (0x10U)
#define SIM_IPS0_PWMBF0_SHIFT                    (4U)
#define SIM_IPS0_PWMBF0(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF0_SHIFT)) & SIM_IPS0_PWMBF0_MASK)
/*! PWMBF1 - Select PWMB Fault 1 Input
 *  0b0..Function = GPIOF13; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT30; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF1_MASK                     (0x20U)
#define SIM_IPS0_PWMBF1_SHIFT                    (5U)
#define SIM_IPS0_PWMBF1(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF1_SHIFT)) & SIM_IPS0_PWMBF1_MASK)
/*! PWMBF2 - Select PWMB Fault 2 Input
 *  0b0..Function = GPIOF12; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT31; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_PWMBF2_MASK                     (0x40U)
#define SIM_IPS0_PWMBF2_SHIFT                    (6U)
#define SIM_IPS0_PWMBF2(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_PWMBF2_SHIFT)) & SIM_IPS0_PWMBF2_MASK)
/*! TA0 - Select TMRA0 Input
 *  0b0..Function = GPIOC3; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT49; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA0_MASK                        (0x100U)
#define SIM_IPS0_TA0_SHIFT                       (8U)
#define SIM_IPS0_TA0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA0_SHIFT)) & SIM_IPS0_TA0_MASK)
/*! TA1 - Select TMRA1 Input
 *  0b0..Function = GPIOC4; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT50; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA1_MASK                        (0x200U)
#define SIM_IPS0_TA1_SHIFT                       (9U)
#define SIM_IPS0_TA1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA1_SHIFT)) & SIM_IPS0_TA1_MASK)
/*! TA2 - Select TMRA2 Input
 *  0b0..Function = GPIOC6 or GPIOG8; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT51; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA2_MASK                        (0x400U)
#define SIM_IPS0_TA2_SHIFT                       (10U)
#define SIM_IPS0_TA2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA2_SHIFT)) & SIM_IPS0_TA2_MASK)
/*! TA3 - Select TMRA3 Input
 *  0b0..Function = GPIOC13 or GPIOG9; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT52; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TA3_MASK                        (0x800U)
#define SIM_IPS0_TA3_SHIFT                       (11U)
#define SIM_IPS0_TA3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TA3_SHIFT)) & SIM_IPS0_TA3_MASK)
/*! TB0 - Select TMRB0 Input
 *  0b0..Function = GPIOC2; Peripheral = GPIOC; Direction = IN
 *  0b1..Function = XB_OUT34; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB0_MASK                        (0x1000U)
#define SIM_IPS0_TB0_SHIFT                       (12U)
#define SIM_IPS0_TB0(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB0_SHIFT)) & SIM_IPS0_TB0_MASK)
/*! TB1 - Select TMRB1 Input
 *  0b0..Function = GPIOF8; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT35; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB1_MASK                        (0x2000U)
#define SIM_IPS0_TB1_SHIFT                       (13U)
#define SIM_IPS0_TB1(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB1_SHIFT)) & SIM_IPS0_TB1_MASK)
/*! TB2 - Select TMRB2 Input
 *  0b0..Function = GPIOF6, GPIOF0, or GPIOG6; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT36; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB2_MASK                        (0x4000U)
#define SIM_IPS0_TB2_SHIFT                       (14U)
#define SIM_IPS0_TB2(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB2_SHIFT)) & SIM_IPS0_TB2_MASK)
/*! TB3 - Select TMRB3 Input
 *  0b0..Function = GPIOF7 or GPIOG11; Peripheral = GPIOF; Direction = IN
 *  0b1..Function = XB_OUT37; Peripheral = XBAR; Direction = IN
 */
#define SIM_IPS0_TB3_MASK                        (0x8000U)
#define SIM_IPS0_TB3_SHIFT                       (15U)
#define SIM_IPS0_TB3(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_IPS0_TB3_SHIFT)) & SIM_IPS0_TB3_MASK)
/*! @} */

/*! @name MISC0 - Miscellaneous Register 0 */
/*! @{ */
/*! PIT_MSTR - Select Master Programmable Interval Timer (PIT)
 *  0b0..PIT0 is master PIT and PIT1 is slave PIT
 *  0b1..PIT1 is master PIT and PIT0 is slave PIT
 */
#define SIM_MISC0_PIT_MSTR_MASK                  (0x1U)
#define SIM_MISC0_PIT_MSTR_SHIFT                 (0U)
#define SIM_MISC0_PIT_MSTR(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_PIT_MSTR_SHIFT)) & SIM_MISC0_PIT_MSTR_MASK)
/*! CLKINSEL - CLKIN Select
 *  0b0..CLKIN0 (GPIOC0 alt1) is selected as CLKIN
 *  0b1..CLKIN1 (GPIOC3 alt3) is selected as CLKIN
 */
#define SIM_MISC0_CLKINSEL_MASK                  (0x2U)
#define SIM_MISC0_CLKINSEL_SHIFT                 (1U)
#define SIM_MISC0_CLKINSEL(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_CLKINSEL_SHIFT)) & SIM_MISC0_CLKINSEL_MASK)
/*! SCTRL_REORDER
 *  0b0..Normal order
 *  0b1..Enable the re-ordering of ADC scan control bits
 */
#define SIM_MISC0_SCTRL_REORDER_MASK             (0x8U)
#define SIM_MISC0_SCTRL_REORDER_SHIFT            (3U)
#define SIM_MISC0_SCTRL_REORDER(x)               (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_SCTRL_REORDER_SHIFT)) & SIM_MISC0_SCTRL_REORDER_MASK)
#define SIM_MISC0_TMR_CNTS_FREEZE_MASK           (0x30U)
#define SIM_MISC0_TMR_CNTS_FREEZE_SHIFT          (4U)
#define SIM_MISC0_TMR_CNTS_FREEZE(x)             (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_TMR_CNTS_FREEZE_SHIFT)) & SIM_MISC0_TMR_CNTS_FREEZE_MASK)
/*! IRC48M_EN
 *  0b0..IRC48MHz disable
 *  0b1..IRC48MHz enable
 */
#define SIM_MISC0_IRC48M_EN_MASK                 (0x40U)
#define SIM_MISC0_IRC48M_EN_SHIFT                (6U)
#define SIM_MISC0_IRC48M_EN(x)                   (((uint16_t)(((uint16_t)(x)) << SIM_MISC0_IRC48M_EN_SHIFT)) & SIM_MISC0_IRC48M_EN_MASK)
/*! @} */

/*! @name PSWR0 - Peripheral Software Reset Register 0 */
/*! @{ */
/*! GPIO - GPIO Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_GPIO_MASK                      (0x40U)
#define SIM_PSWR0_GPIO_SHIFT                     (6U)
#define SIM_PSWR0_GPIO(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_GPIO_SHIFT)) & SIM_PSWR0_GPIO_MASK)
/*! TB - TMRB Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_TB_MASK                        (0x800U)
#define SIM_PSWR0_TB_SHIFT                       (11U)
#define SIM_PSWR0_TB(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_TB_SHIFT)) & SIM_PSWR0_TB_MASK)
/*! TA - TMRA Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR0_TA_MASK                        (0x8000U)
#define SIM_PSWR0_TA_SHIFT                       (15U)
#define SIM_PSWR0_TA(x)                          (((uint16_t)(((uint16_t)(x)) << SIM_PSWR0_TA_SHIFT)) & SIM_PSWR0_TA_MASK)
/*! @} */

/*! @name PSWR1 - Peripheral Software Reset Register 1 */
/*! @{ */
/*! FLEXCAN - FlexCAN Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_FLEXCAN_MASK                   (0x1U)
#define SIM_PSWR1_FLEXCAN_SHIFT                  (0U)
#define SIM_PSWR1_FLEXCAN(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_FLEXCAN_SHIFT)) & SIM_PSWR1_FLEXCAN_MASK)
/*! IIC1 - IIC1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_IIC1_MASK                      (0x20U)
#define SIM_PSWR1_IIC1_SHIFT                     (5U)
#define SIM_PSWR1_IIC1(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_IIC1_SHIFT)) & SIM_PSWR1_IIC1_MASK)
/*! IIC0 - IIC0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_IIC0_MASK                      (0x40U)
#define SIM_PSWR1_IIC0_SHIFT                     (6U)
#define SIM_PSWR1_IIC0(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_IIC0_SHIFT)) & SIM_PSWR1_IIC0_MASK)
/*! QSPI1 - QSPI1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_QSPI1_MASK                     (0x100U)
#define SIM_PSWR1_QSPI1_SHIFT                    (8U)
#define SIM_PSWR1_QSPI1(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_QSPI1_SHIFT)) & SIM_PSWR1_QSPI1_MASK)
/*! QSPI0 - QSPI0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_QSPI0_MASK                     (0x200U)
#define SIM_PSWR1_QSPI0_SHIFT                    (9U)
#define SIM_PSWR1_QSPI0(x)                       (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_QSPI0_SHIFT)) & SIM_PSWR1_QSPI0_MASK)
/*! SCI2 - SCI2 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI2_MASK                      (0x400U)
#define SIM_PSWR1_SCI2_SHIFT                     (10U)
#define SIM_PSWR1_SCI2(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI2_SHIFT)) & SIM_PSWR1_SCI2_MASK)
/*! SCI1 - SCI1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI1_MASK                      (0x800U)
#define SIM_PSWR1_SCI1_SHIFT                     (11U)
#define SIM_PSWR1_SCI1(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI1_SHIFT)) & SIM_PSWR1_SCI1_MASK)
/*! SCI0 - SCI0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_SCI0_MASK                      (0x1000U)
#define SIM_PSWR1_SCI0_SHIFT                     (12U)
#define SIM_PSWR1_SCI0(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_SCI0_SHIFT)) & SIM_PSWR1_SCI0_MASK)
/*! DAC0 - DAC0 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_DAC0_MASK                      (0x2000U)
#define SIM_PSWR1_DAC0_SHIFT                     (13U)
#define SIM_PSWR1_DAC0(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_DAC0_SHIFT)) & SIM_PSWR1_DAC0_MASK)
/*! DAC1 - DAC1 Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR1_DAC1_MASK                      (0x4000U)
#define SIM_PSWR1_DAC1_SHIFT                     (14U)
#define SIM_PSWR1_DAC1(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR1_DAC1_SHIFT)) & SIM_PSWR1_DAC1_MASK)
/*! @} */

/*! @name PSWR2 - Peripheral Software Reset Register 2 */
/*! @{ */
/*! PIT1 - Programmable Interval Timer Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_PIT1_MASK                      (0x4U)
#define SIM_PSWR2_PIT1_SHIFT                     (2U)
#define SIM_PSWR2_PIT1(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_PIT1_SHIFT)) & SIM_PSWR2_PIT1_MASK)
/*! PIT0 - Programmable Interval Timer Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_PIT0_MASK                      (0x8U)
#define SIM_PSWR2_PIT0_SHIFT                     (3U)
#define SIM_PSWR2_PIT0(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_PIT0_SHIFT)) & SIM_PSWR2_PIT0_MASK)
/*! CRC - CRC Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CRC_MASK                       (0x20U)
#define SIM_PSWR2_CRC_SHIFT                      (5U)
#define SIM_PSWR2_CRC(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CRC_SHIFT)) & SIM_PSWR2_CRC_MASK)
/*! CYCADC - Cyclic ADC Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CYCADC_MASK                    (0x80U)
#define SIM_PSWR2_CYCADC_SHIFT                   (7U)
#define SIM_PSWR2_CYCADC(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CYCADC_SHIFT)) & SIM_PSWR2_CYCADC_MASK)
/*! CMP - CMP Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_CMP_MASK                       (0x1000U)
#define SIM_PSWR2_CMP_SHIFT                      (12U)
#define SIM_PSWR2_CMP(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_CMP_SHIFT)) & SIM_PSWR2_CMP_MASK)
/*! EWM - EWM Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR2_EWM_MASK                       (0x8000U)
#define SIM_PSWR2_EWM_SHIFT                      (15U)
#define SIM_PSWR2_EWM(x)                         (((uint16_t)(((uint16_t)(x)) << SIM_PSWR2_EWM_SHIFT)) & SIM_PSWR2_EWM_MASK)
/*! @} */

/*! @name PSWR3 - Peripheral Software Reset Register 3 */
/*! @{ */
/*! PWMB - PWMB Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_PWMB_MASK                      (0x8U)
#define SIM_PSWR3_PWMB_SHIFT                     (3U)
#define SIM_PSWR3_PWMB(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_PWMB_SHIFT)) & SIM_PSWR3_PWMB_MASK)
/*! PWMA - PWMA Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_PWMA_MASK                      (0x80U)
#define SIM_PSWR3_PWMA_SHIFT                     (7U)
#define SIM_PSWR3_PWMA(x)                        (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_PWMA_SHIFT)) & SIM_PSWR3_PWMA_MASK)
/*! USB_OTG - USB_OTG Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_USB_OTG_MASK                   (0x100U)
#define SIM_PSWR3_USB_OTG_SHIFT                  (8U)
#define SIM_PSWR3_USB_OTG(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_USB_OTG_SHIFT)) & SIM_PSWR3_USB_OTG_MASK)
/*! DMA_MUX - DMA_MUX Software Reset
 *  0b0..The corresponding peripheral is not reset.
 *  0b1..The corresponding peripheral is reset.
 */
#define SIM_PSWR3_DMA_MUX_MASK                   (0x200U)
#define SIM_PSWR3_DMA_MUX_SHIFT                  (9U)
#define SIM_PSWR3_DMA_MUX(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PSWR3_DMA_MUX_SHIFT)) & SIM_PSWR3_DMA_MUX_MASK)
/*! @} */

/*! @name PWRMODE - Power Mode Register */
/*! @{ */
/*! VLPMODE - VLPMODE Entry/Exit
 *  0b0..Start exit from VLPMODE
 *  0b1..Start entry to VLPMODE
 */
#define SIM_PWRMODE_VLPMODE_MASK                 (0x1U)
#define SIM_PWRMODE_VLPMODE_SHIFT                (0U)
#define SIM_PWRMODE_VLPMODE(x)                   (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_VLPMODE_SHIFT)) & SIM_PWRMODE_VLPMODE_MASK)
/*! LPMODE - LPMODE Entry/Exit
 *  0b0..Start exit from LPMODE
 *  0b1..Start entry to LPMODE
 */
#define SIM_PWRMODE_LPMODE_MASK                  (0x2U)
#define SIM_PWRMODE_LPMODE_SHIFT                 (1U)
#define SIM_PWRMODE_LPMODE(x)                    (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_LPMODE_SHIFT)) & SIM_PWRMODE_LPMODE_MASK)
/*! VLPMS - VLPMODE Status Indicator
 *  0b0..Not in VLPMODE
 *  0b1..In VLPMODE
 */
#define SIM_PWRMODE_VLPMS_MASK                   (0x100U)
#define SIM_PWRMODE_VLPMS_SHIFT                  (8U)
#define SIM_PWRMODE_VLPMS(x)                     (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_VLPMS_SHIFT)) & SIM_PWRMODE_VLPMS_MASK)
/*! LPMS - LPMODE Status Indicator
 *  0b0..Not in LPMODE
 *  0b1..In LPMODE
 */
#define SIM_PWRMODE_LPMS_MASK                    (0x200U)
#define SIM_PWRMODE_LPMS_SHIFT                   (9U)
#define SIM_PWRMODE_LPMS(x)                      (((uint16_t)(((uint16_t)(x)) << SIM_PWRMODE_LPMS_SHIFT)) & SIM_PWRMODE_LPMS_MASK)
/*! @} */

/*! @name NVMOPT6_HIGH - Non-Volatile Memory Option Register 6 (High) */
/*! @{ */
#define SIM_NVMOPT6_HIGH_ROSC_8M_FTRIM_MASK      (0x3FFU)
#define SIM_NVMOPT6_HIGH_ROSC_8M_FTRIM_SHIFT     (0U)
#define SIM_NVMOPT6_HIGH_ROSC_8M_FTRIM(x)        (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_HIGH_ROSC_8M_FTRIM_SHIFT)) & SIM_NVMOPT6_HIGH_ROSC_8M_FTRIM_MASK)
#define SIM_NVMOPT6_HIGH_ROSC_8M_TTRIM_MASK      (0x3C00U)
#define SIM_NVMOPT6_HIGH_ROSC_8M_TTRIM_SHIFT     (10U)
#define SIM_NVMOPT6_HIGH_ROSC_8M_TTRIM(x)        (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_HIGH_ROSC_8M_TTRIM_SHIFT)) & SIM_NVMOPT6_HIGH_ROSC_8M_TTRIM_MASK)
#define SIM_NVMOPT6_HIGH_ROSC_8M_EN_MASK         (0x4000U)
#define SIM_NVMOPT6_HIGH_ROSC_8M_EN_SHIFT        (14U)
#define SIM_NVMOPT6_HIGH_ROSC_8M_EN(x)           (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_HIGH_ROSC_8M_EN_SHIFT)) & SIM_NVMOPT6_HIGH_ROSC_8M_EN_MASK)
/*! @} */

/*! @name NVMOPT6_LOW - Non-Volatile Memory Option Register 6 (Low) */
/*! @{ */
#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_MASK     (0x1FFU)
#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_SHIFT    (0U)
#define SIM_NVMOPT6_LOW_ROSC_200K_FTRIM(x)       (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_SHIFT)) & SIM_NVMOPT6_LOW_ROSC_200K_FTRIM_MASK)
#define SIM_NVMOPT6_LOW_PMC_BGTRIM_MASK          (0xF000U)
#define SIM_NVMOPT6_LOW_PMC_BGTRIM_SHIFT         (12U)
#define SIM_NVMOPT6_LOW_PMC_BGTRIM(x)            (((uint16_t)(((uint16_t)(x)) << SIM_NVMOPT6_LOW_PMC_BGTRIM_SHIFT)) & SIM_NVMOPT6_LOW_PMC_BGTRIM_MASK)
/*! @} */

/*! @name PWM_SEL - PWM Select Register */
/*! @{ */
/*! XBAR_IN20
 *  0b0..PWMA mux0_trig0
 *  0b1..PWMB out0_trig0
 */
#define SIM_PWM_SEL_XBAR_IN20_MASK               (0x1U)
#define SIM_PWM_SEL_XBAR_IN20_SHIFT              (0U)
#define SIM_PWM_SEL_XBAR_IN20(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN20_SHIFT)) & SIM_PWM_SEL_XBAR_IN20_MASK)
/*! XBAR_IN21
 *  0b0..PWMA mux1_trig0
 *  0b1..PWMB out1_trig0
 */
#define SIM_PWM_SEL_XBAR_IN21_MASK               (0x2U)
#define SIM_PWM_SEL_XBAR_IN21_SHIFT              (1U)
#define SIM_PWM_SEL_XBAR_IN21(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN21_SHIFT)) & SIM_PWM_SEL_XBAR_IN21_MASK)
/*! XBAR_IN22
 *  0b0..PWMA mux0_trig1
 *  0b1..PWMB out0_trig1
 */
#define SIM_PWM_SEL_XBAR_IN22_MASK               (0x4U)
#define SIM_PWM_SEL_XBAR_IN22_SHIFT              (2U)
#define SIM_PWM_SEL_XBAR_IN22(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN22_SHIFT)) & SIM_PWM_SEL_XBAR_IN22_MASK)
/*! XBAR_IN23
 *  0b0..PWMA mux1_trig1
 *  0b1..PWMB out1_trig1
 */
#define SIM_PWM_SEL_XBAR_IN23_MASK               (0x8U)
#define SIM_PWM_SEL_XBAR_IN23_SHIFT              (3U)
#define SIM_PWM_SEL_XBAR_IN23(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN23_SHIFT)) & SIM_PWM_SEL_XBAR_IN23_MASK)
/*! XBAR_IN24
 *  0b0..PWMA mux0_trig2
 *  0b1..PWMB out0_trig2
 */
#define SIM_PWM_SEL_XBAR_IN24_MASK               (0x10U)
#define SIM_PWM_SEL_XBAR_IN24_SHIFT              (4U)
#define SIM_PWM_SEL_XBAR_IN24(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN24_SHIFT)) & SIM_PWM_SEL_XBAR_IN24_MASK)
/*! XBAR_IN25
 *  0b0..PWMA mux1_trig2
 *  0b1..PWMB out1_trig2
 */
#define SIM_PWM_SEL_XBAR_IN25_MASK               (0x20U)
#define SIM_PWM_SEL_XBAR_IN25_SHIFT              (5U)
#define SIM_PWM_SEL_XBAR_IN25(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN25_SHIFT)) & SIM_PWM_SEL_XBAR_IN25_MASK)
/*! XBAR_IN26
 *  0b0..PWMA mux0_trig3
 *  0b1..PWMB out0_trig3
 */
#define SIM_PWM_SEL_XBAR_IN26_MASK               (0x40U)
#define SIM_PWM_SEL_XBAR_IN26_SHIFT              (6U)
#define SIM_PWM_SEL_XBAR_IN26(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN26_SHIFT)) & SIM_PWM_SEL_XBAR_IN26_MASK)
/*! XBAR_IN27
 *  0b0..PWMA mux1_trig3
 *  0b1..PWMB out1_trig3
 */
#define SIM_PWM_SEL_XBAR_IN27_MASK               (0x80U)
#define SIM_PWM_SEL_XBAR_IN27_SHIFT              (7U)
#define SIM_PWM_SEL_XBAR_IN27(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN27_SHIFT)) & SIM_PWM_SEL_XBAR_IN27_MASK)
/*! XBAR_IN28
 *  0b0..PWMB mux0_trig0
 *  0b1..PWMA out0_trig0
 */
#define SIM_PWM_SEL_XBAR_IN28_MASK               (0x100U)
#define SIM_PWM_SEL_XBAR_IN28_SHIFT              (8U)
#define SIM_PWM_SEL_XBAR_IN28(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN28_SHIFT)) & SIM_PWM_SEL_XBAR_IN28_MASK)
/*! XBAR_IN29
 *  0b0..PWMB mux1_trig0
 *  0b1..PWMA out1_trig0
 */
#define SIM_PWM_SEL_XBAR_IN29_MASK               (0x200U)
#define SIM_PWM_SEL_XBAR_IN29_SHIFT              (9U)
#define SIM_PWM_SEL_XBAR_IN29(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN29_SHIFT)) & SIM_PWM_SEL_XBAR_IN29_MASK)
/*! XBAR_IN30
 *  0b0..PWMB mux0_trig1
 *  0b1..PWMA out0_trig1
 */
#define SIM_PWM_SEL_XBAR_IN30_MASK               (0x400U)
#define SIM_PWM_SEL_XBAR_IN30_SHIFT              (10U)
#define SIM_PWM_SEL_XBAR_IN30(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN30_SHIFT)) & SIM_PWM_SEL_XBAR_IN30_MASK)
/*! XBAR_IN31
 *  0b0..PWMB mux1_trig1
 *  0b1..PWMA out1_trig1
 */
#define SIM_PWM_SEL_XBAR_IN31_MASK               (0x800U)
#define SIM_PWM_SEL_XBAR_IN31_SHIFT              (11U)
#define SIM_PWM_SEL_XBAR_IN31(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN31_SHIFT)) & SIM_PWM_SEL_XBAR_IN31_MASK)
/*! XBAR_IN32
 *  0b0..PWMB mux0_trig2
 *  0b1..PWMA out0_trig2
 */
#define SIM_PWM_SEL_XBAR_IN32_MASK               (0x1000U)
#define SIM_PWM_SEL_XBAR_IN32_SHIFT              (12U)
#define SIM_PWM_SEL_XBAR_IN32(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN32_SHIFT)) & SIM_PWM_SEL_XBAR_IN32_MASK)
/*! XBAR_IN33
 *  0b0..PWMB mux1_trig2
 *  0b1..PWMA out1_trig2
 */
#define SIM_PWM_SEL_XBAR_IN33_MASK               (0x2000U)
#define SIM_PWM_SEL_XBAR_IN33_SHIFT              (13U)
#define SIM_PWM_SEL_XBAR_IN33(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN33_SHIFT)) & SIM_PWM_SEL_XBAR_IN33_MASK)
/*! XBAR_IN34
 *  0b0..PWMB mux0_trig3
 *  0b1..PWMB out0_trig3
 */
#define SIM_PWM_SEL_XBAR_IN34_MASK               (0x4000U)
#define SIM_PWM_SEL_XBAR_IN34_SHIFT              (14U)
#define SIM_PWM_SEL_XBAR_IN34(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN34_SHIFT)) & SIM_PWM_SEL_XBAR_IN34_MASK)
/*! XBAR_IN35
 *  0b0..PWMB mux1_trig3
 *  0b1..PWMA out1_trig3
 */
#define SIM_PWM_SEL_XBAR_IN35_MASK               (0x8000U)
#define SIM_PWM_SEL_XBAR_IN35_SHIFT              (15U)
#define SIM_PWM_SEL_XBAR_IN35(x)                 (((uint16_t)(((uint16_t)(x)) << SIM_PWM_SEL_XBAR_IN35_SHIFT)) & SIM_PWM_SEL_XBAR_IN35_MASK)
/*! @} */

/*! @name ADC_TMR_SEL - ADC and TMR Select Register */
/*! @{ */
/*! XBAR_IN36
 *  0b0..TMRA0
 *  0b1..ADC AN0 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN36_MASK           (0x1U)
#define SIM_ADC_TMR_SEL_XBAR_IN36_SHIFT          (0U)
#define SIM_ADC_TMR_SEL_XBAR_IN36(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN36_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN36_MASK)
/*! XBAR_IN37
 *  0b0..TMRA1
 *  0b1..ADC AN1 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN37_MASK           (0x2U)
#define SIM_ADC_TMR_SEL_XBAR_IN37_SHIFT          (1U)
#define SIM_ADC_TMR_SEL_XBAR_IN37(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN37_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN37_MASK)
/*! XBAR_IN38
 *  0b0..TMRA2
 *  0b1..ADC AN2 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN38_MASK           (0x4U)
#define SIM_ADC_TMR_SEL_XBAR_IN38_SHIFT          (2U)
#define SIM_ADC_TMR_SEL_XBAR_IN38(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN38_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN38_MASK)
/*! XBAR_IN39
 *  0b0..TMRA3
 *  0b1..ADC AN3 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN39_MASK           (0x8U)
#define SIM_ADC_TMR_SEL_XBAR_IN39_SHIFT          (3U)
#define SIM_ADC_TMR_SEL_XBAR_IN39(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN39_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN39_MASK)
/*! XBAR_IN16
 *  0b0..TMRB0
 *  0b1..ADC AN8 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN16_MASK           (0x100U)
#define SIM_ADC_TMR_SEL_XBAR_IN16_SHIFT          (8U)
#define SIM_ADC_TMR_SEL_XBAR_IN16(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN16_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN16_MASK)
/*! XBAR_IN17
 *  0b0..TMRB1
 *  0b1..ADC AN9 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN17_MASK           (0x200U)
#define SIM_ADC_TMR_SEL_XBAR_IN17_SHIFT          (9U)
#define SIM_ADC_TMR_SEL_XBAR_IN17(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN17_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN17_MASK)
/*! XBAR_IN18
 *  0b0..TMRB2
 *  0b1..ADC AN10 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN18_MASK           (0x400U)
#define SIM_ADC_TMR_SEL_XBAR_IN18_SHIFT          (10U)
#define SIM_ADC_TMR_SEL_XBAR_IN18(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN18_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN18_MASK)
/*! XBAR_IN19
 *  0b0..TMRB3
 *  0b1..ADC AN11 limit
 */
#define SIM_ADC_TMR_SEL_XBAR_IN19_MASK           (0x800U)
#define SIM_ADC_TMR_SEL_XBAR_IN19_SHIFT          (11U)
#define SIM_ADC_TMR_SEL_XBAR_IN19(x)             (((uint16_t)(((uint16_t)(x)) << SIM_ADC_TMR_SEL_XBAR_IN19_SHIFT)) & SIM_ADC_TMR_SEL_XBAR_IN19_MASK)
/*! @} */

/*! @name BOOT_MODE_OVERRIDE - Boot Mode Override Register */
/*! @{ */
/*! BOOT_OVERRIDE
 *  0b00..FOPT[7:6] not masked.
 *  0b01..FOPT[7] is not masked. FOPT[6] is masked (~, then AND).
 *  0b10..FOPT[7] is masked. FOPT[6] is not masked.
 *  0b11..FOPT[7:6] masked.
 */
#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK (0x3U)
#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_SHIFT (0U)
#define SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE(x)  (((uint16_t)(((uint16_t)(x)) << SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_SHIFT)) & SIM_BOOT_MODE_OVERRIDE_BOOT_OVERRIDE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0xE400u)
/** Peripheral SIM base pointer */
#define SIM                                      ((SIM_Type *)SIM_BASE)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS                           { SIM_BASE }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM }

/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- TMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Peripheral_Access_Layer TMR Peripheral Access Layer
 * @{
 */

/** TMR - Register Layout Typedef */
typedef struct {
  __IO uint16_t COMP10;                            /**< Timer Channel Compare Register 1, offset: 0x0 */
  __IO uint16_t COMP20;                            /**< Timer Channel Compare Register 2, offset: 0x1 */
  __IO uint16_t CAPT0;                             /**< Timer Channel Capture Register, offset: 0x2 */
  __IO uint16_t LOAD0;                             /**< Timer Channel Load Register, offset: 0x3 */
  __IO uint16_t HOLD0;                             /**< Timer Channel Hold Register, offset: 0x4 */
  __IO uint16_t CNTR0;                             /**< Timer Channel Counter Register, offset: 0x5 */
  __IO uint16_t CTRL0;                             /**< Timer Channel Control Register, offset: 0x6 */
  __IO uint16_t SCTRL0;                            /**< Timer Channel Status and Control Register, offset: 0x7 */
  __IO uint16_t CMPLD10;                           /**< Timer Channel Comparator Load Register 1, offset: 0x8 */
  __IO uint16_t CMPLD20;                           /**< Timer Channel Comparator Load Register 2, offset: 0x9 */
  __IO uint16_t CSCTRL0;                           /**< Timer Channel Comparator Status and Control Register, offset: 0xA */
  __IO uint16_t FILT0;                             /**< Timer Channel Input Filter Register, offset: 0xB */
  __IO uint16_t DMA0;                              /**< Timer Channel DMA Enable Register, offset: 0xC */
       uint16_t RESERVED_0[2];
  __IO uint16_t ENBL;                              /**< Timer Channel Enable Register, offset: 0xF */
  __IO uint16_t COMP11;                            /**< Timer Channel Compare Register 1, offset: 0x10 */
  __IO uint16_t COMP21;                            /**< Timer Channel Compare Register 2, offset: 0x11 */
  __IO uint16_t CAPT1;                             /**< Timer Channel Capture Register, offset: 0x12 */
  __IO uint16_t LOAD1;                             /**< Timer Channel Load Register, offset: 0x13 */
  __IO uint16_t HOLD1;                             /**< Timer Channel Hold Register, offset: 0x14 */
  __IO uint16_t CNTR1;                             /**< Timer Channel Counter Register, offset: 0x15 */
  __IO uint16_t CTRL1;                             /**< Timer Channel Control Register, offset: 0x16 */
  __IO uint16_t SCTRL1;                            /**< Timer Channel Status and Control Register, offset: 0x17 */
  __IO uint16_t CMPLD11;                           /**< Timer Channel Comparator Load Register 1, offset: 0x18 */
  __IO uint16_t CMPLD21;                           /**< Timer Channel Comparator Load Register 2, offset: 0x19 */
  __IO uint16_t CSCTRL1;                           /**< Timer Channel Comparator Status and Control Register, offset: 0x1A */
  __IO uint16_t FILT1;                             /**< Timer Channel Input Filter Register, offset: 0x1B */
  __IO uint16_t DMA1;                              /**< Timer Channel DMA Enable Register, offset: 0x1C */
       uint16_t RESERVED_1[3];
  __IO uint16_t COMP12;                            /**< Timer Channel Compare Register 1, offset: 0x20 */
  __IO uint16_t COMP22;                            /**< Timer Channel Compare Register 2, offset: 0x21 */
  __IO uint16_t CAPT2;                             /**< Timer Channel Capture Register, offset: 0x22 */
  __IO uint16_t LOAD2;                             /**< Timer Channel Load Register, offset: 0x23 */
  __IO uint16_t HOLD2;                             /**< Timer Channel Hold Register, offset: 0x24 */
  __IO uint16_t CNTR2;                             /**< Timer Channel Counter Register, offset: 0x25 */
  __IO uint16_t CTRL2;                             /**< Timer Channel Control Register, offset: 0x26 */
  __IO uint16_t SCTRL2;                            /**< Timer Channel Status and Control Register, offset: 0x27 */
  __IO uint16_t CMPLD12;                           /**< Timer Channel Comparator Load Register 1, offset: 0x28 */
  __IO uint16_t CMPLD22;                           /**< Timer Channel Comparator Load Register 2, offset: 0x29 */
  __IO uint16_t CSCTRL2;                           /**< Timer Channel Comparator Status and Control Register, offset: 0x2A */
  __IO uint16_t FILT2;                             /**< Timer Channel Input Filter Register, offset: 0x2B */
  __IO uint16_t DMA2;                              /**< Timer Channel DMA Enable Register, offset: 0x2C */
       uint16_t RESERVED_2[3];
  __IO uint16_t COMP13;                            /**< Timer Channel Compare Register 1, offset: 0x30 */
  __IO uint16_t COMP23;                            /**< Timer Channel Compare Register 2, offset: 0x31 */
  __IO uint16_t CAPT3;                             /**< Timer Channel Capture Register, offset: 0x32 */
  __IO uint16_t LOAD3;                             /**< Timer Channel Load Register, offset: 0x33 */
  __IO uint16_t HOLD3;                             /**< Timer Channel Hold Register, offset: 0x34 */
  __IO uint16_t CNTR3;                             /**< Timer Channel Counter Register, offset: 0x35 */
  __IO uint16_t CTRL3;                             /**< Timer Channel Control Register, offset: 0x36 */
  __IO uint16_t SCTRL3;                            /**< Timer Channel Status and Control Register, offset: 0x37 */
  __IO uint16_t CMPLD13;                           /**< Timer Channel Comparator Load Register 1, offset: 0x38 */
  __IO uint16_t CMPLD23;                           /**< Timer Channel Comparator Load Register 2, offset: 0x39 */
  __IO uint16_t CSCTRL3;                           /**< Timer Channel Comparator Status and Control Register, offset: 0x3A */
  __IO uint16_t FILT3;                             /**< Timer Channel Input Filter Register, offset: 0x3B */
  __IO uint16_t DMA3;                              /**< Timer Channel DMA Enable Register, offset: 0x3C */
} TMR_Type;

/* ----------------------------------------------------------------------------
   -- TMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup TMR_Register_Masks TMR Register Masks
 * @{
 */

/*! @name COMP10 - Timer Channel Compare Register 1 */
/*! @{ */
#define TMR_COMP10_COMPARISON_1_MASK             (0xFFFFU)
#define TMR_COMP10_COMPARISON_1_SHIFT            (0U)
#define TMR_COMP10_COMPARISON_1(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP10_COMPARISON_1_SHIFT)) & TMR_COMP10_COMPARISON_1_MASK)
/*! @} */

/*! @name COMP20 - Timer Channel Compare Register 2 */
/*! @{ */
#define TMR_COMP20_COMPARISON_2_MASK             (0xFFFFU)
#define TMR_COMP20_COMPARISON_2_SHIFT            (0U)
#define TMR_COMP20_COMPARISON_2(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP20_COMPARISON_2_SHIFT)) & TMR_COMP20_COMPARISON_2_MASK)
/*! @} */

/*! @name CAPT0 - Timer Channel Capture Register */
/*! @{ */
#define TMR_CAPT0_CAPTURE_MASK                   (0xFFFFU)
#define TMR_CAPT0_CAPTURE_SHIFT                  (0U)
#define TMR_CAPT0_CAPTURE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CAPT0_CAPTURE_SHIFT)) & TMR_CAPT0_CAPTURE_MASK)
/*! @} */

/*! @name LOAD0 - Timer Channel Load Register */
/*! @{ */
#define TMR_LOAD0_LOAD_MASK                      (0xFFFFU)
#define TMR_LOAD0_LOAD_SHIFT                     (0U)
#define TMR_LOAD0_LOAD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_LOAD0_LOAD_SHIFT)) & TMR_LOAD0_LOAD_MASK)
/*! @} */

/*! @name HOLD0 - Timer Channel Hold Register */
/*! @{ */
#define TMR_HOLD0_HOLD_MASK                      (0xFFFFU)
#define TMR_HOLD0_HOLD_SHIFT                     (0U)
#define TMR_HOLD0_HOLD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_HOLD0_HOLD_SHIFT)) & TMR_HOLD0_HOLD_MASK)
/*! @} */

/*! @name CNTR0 - Timer Channel Counter Register */
/*! @{ */
#define TMR_CNTR0_COUNTER_MASK                   (0xFFFFU)
#define TMR_CNTR0_COUNTER_SHIFT                  (0U)
#define TMR_CNTR0_COUNTER(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CNTR0_COUNTER_SHIFT)) & TMR_CNTR0_COUNTER_MASK)
/*! @} */

/*! @name CTRL0 - Timer Channel Control Register */
/*! @{ */
/*! OUTMODE - Output Mode
 *  0b000..Asserted while counter is active
 *  0b001..Clear OFLAG output on successful compare
 *  0b010..Set OFLAG output on successful compare
 *  0b011..Toggle OFLAG output on successful compare
 *  0b100..Toggle OFLAG output using alternating compare registers
 *  0b101..Set on compare, cleared on secondary source input edge
 *  0b110..Set on compare, cleared on counter rollover
 *  0b111..Enable gated clock output while counter is active
 */
#define TMR_CTRL0_OUTMODE_MASK                   (0x7U)
#define TMR_CTRL0_OUTMODE_SHIFT                  (0U)
#define TMR_CTRL0_OUTMODE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_OUTMODE_SHIFT)) & TMR_CTRL0_OUTMODE_MASK)
/*! COINIT - Co-Channel Initialization
 *  0b0..Co-channel counter/timers cannot force a re-initialization of this counter/timer
 *  0b1..Co-channel counter/timers may force a re-initialization of this counter/timer
 */
#define TMR_CTRL0_COINIT_MASK                    (0x8U)
#define TMR_CTRL0_COINIT_SHIFT                   (3U)
#define TMR_CTRL0_COINIT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_COINIT_SHIFT)) & TMR_CTRL0_COINIT_MASK)
/*! DIR - Count Direction
 *  0b0..Count up.
 *  0b1..Count down.
 */
#define TMR_CTRL0_DIR_MASK                       (0x10U)
#define TMR_CTRL0_DIR_SHIFT                      (4U)
#define TMR_CTRL0_DIR(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_DIR_SHIFT)) & TMR_CTRL0_DIR_MASK)
/*! LENGTH - Count Length
 *  0b0..Count until roll over at $FFFF and then continue by re-initializing the counter from the LOAD register.
 *  0b1..Count until compare, then re-initialize using the LOAD regsiter. If counting up, a successful compare
 *       occurs when the counter reaches a COMP1 value. If counting down, a successful compare occurs when the counter
 *       reaches a COMP2 value. When output mode $4 is used, alternating values of COMP1 and COMP2 are used to
 *       generate successful comparisons. For example, the counter counts until a COMP1 value is reached,
 *       re-initializes, counts until COMP2 value is reached, re-initializes, counts until COMP1 value is reached, and so on.
 */
#define TMR_CTRL0_LENGTH_MASK                    (0x20U)
#define TMR_CTRL0_LENGTH_SHIFT                   (5U)
#define TMR_CTRL0_LENGTH(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_LENGTH_SHIFT)) & TMR_CTRL0_LENGTH_MASK)
/*! ONCE - Count Once
 *  0b0..Count repeatedly.
 *  0b1..Count until compare and then stop. If counting up, a successful compare occurs when the counter reaches a
 *       COMP1 value. If counting down, a successful compare occurs when the counter reaches a COMP2 value. When
 *       output mode $4 is used, the counter re-initializes after reaching the COMP1 value, continues to count to
 *       the COMP2 value, and then stops.
 */
#define TMR_CTRL0_ONCE_MASK                      (0x40U)
#define TMR_CTRL0_ONCE_SHIFT                     (6U)
#define TMR_CTRL0_ONCE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_ONCE_SHIFT)) & TMR_CTRL0_ONCE_MASK)
/*! SCS - Secondary Count Source
 *  0b00..Counter 0 input pin
 *  0b01..Counter 1 input pin
 *  0b10..Counter 2 input pin
 *  0b11..Counter 3 input pin
 */
#define TMR_CTRL0_SCS_MASK                       (0x180U)
#define TMR_CTRL0_SCS_SHIFT                      (7U)
#define TMR_CTRL0_SCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_SCS_SHIFT)) & TMR_CTRL0_SCS_MASK)
/*! PCS - Primary Count Source
 *  0b0000..Counter 0 input pin
 *  0b0001..Counter 1 input pin
 *  0b0010..Counter 2 input pin
 *  0b0011..Counter 3 input pin
 *  0b0100..Counter 0 output
 *  0b0101..Counter 1 output
 *  0b0110..Counter 2 output
 *  0b0111..Counter 3 output
 *  0b1000..IP bus clock divide by 1 prescaler
 *  0b1001..IP bus clock divide by 2 prescaler
 *  0b1010..IP bus clock divide by 4 prescaler
 *  0b1011..IP bus clock divide by 8 prescaler
 *  0b1100..IP bus clock divide by 16 prescaler
 *  0b1101..IP bus clock divide by 32 prescaler
 *  0b1110..IP bus clock divide by 64 prescaler
 *  0b1111..IP bus clock divide by 128 prescaler
 */
#define TMR_CTRL0_PCS_MASK                       (0x1E00U)
#define TMR_CTRL0_PCS_SHIFT                      (9U)
#define TMR_CTRL0_PCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_PCS_SHIFT)) & TMR_CTRL0_PCS_MASK)
/*! CM - Count Mode
 *  0b000..No operation
 *  0b001..Count rising edges of primary sourceRising edges are counted only when SCTRL[IPS] = 0. Falling edges
 *         are counted when SCTRL[IPS] = 1. If the primary count source is IP bus clock divide by 1, only rising
 *         edges are counted regardless of the value of SCTRL[IPS].
 *  0b010..Count rising and falling edges of primary sourceIP bus clock divide by 1 cannot be used as a primary count source in edge count mode.
 *  0b011..Count rising edges of primary source while secondary input high active
 *  0b100..Quadrature count mode, uses primary and secondary sources
 *  0b101..Count rising edges of primary source; secondary source specifies directionRising edges are counted only
 *         when SCTRL[IPS] = 0. Falling edges are counted when SCTRL[IPS] = 1.
 *  0b110..Edge of secondary source triggers primary count until compare
 *  0b111..Cascaded counter mode (up/down)The primary count source must be set to one of the counter outputs.
 */
#define TMR_CTRL0_CM_MASK                        (0xE000U)
#define TMR_CTRL0_CM_SHIFT                       (13U)
#define TMR_CTRL0_CM(x)                          (((uint16_t)(((uint16_t)(x)) << TMR_CTRL0_CM_SHIFT)) & TMR_CTRL0_CM_MASK)
/*! @} */

/*! @name SCTRL0 - Timer Channel Status and Control Register */
/*! @{ */
/*! OEN - Output Enable
 *  0b0..The external pin is configured as an input.
 *  0b1..The OFLAG output signal is driven on the external pin. Other timer groups using this external pin as
 *       their input see the driven value. The polarity of the signal is determined by OPS.
 */
#define TMR_SCTRL0_OEN_MASK                      (0x1U)
#define TMR_SCTRL0_OEN_SHIFT                     (0U)
#define TMR_SCTRL0_OEN(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_OEN_SHIFT)) & TMR_SCTRL0_OEN_MASK)
/*! OPS - Output Polarity Select
 *  0b0..True polarity.
 *  0b1..Inverted polarity.
 */
#define TMR_SCTRL0_OPS_MASK                      (0x2U)
#define TMR_SCTRL0_OPS_SHIFT                     (1U)
#define TMR_SCTRL0_OPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_OPS_SHIFT)) & TMR_SCTRL0_OPS_MASK)
#define TMR_SCTRL0_FORCE_MASK                    (0x4U)
#define TMR_SCTRL0_FORCE_SHIFT                   (2U)
#define TMR_SCTRL0_FORCE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_FORCE_SHIFT)) & TMR_SCTRL0_FORCE_MASK)
#define TMR_SCTRL0_VAL_MASK                      (0x8U)
#define TMR_SCTRL0_VAL_SHIFT                     (3U)
#define TMR_SCTRL0_VAL(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_VAL_SHIFT)) & TMR_SCTRL0_VAL_MASK)
#define TMR_SCTRL0_EEOF_MASK                     (0x10U)
#define TMR_SCTRL0_EEOF_SHIFT                    (4U)
#define TMR_SCTRL0_EEOF(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_EEOF_SHIFT)) & TMR_SCTRL0_EEOF_MASK)
#define TMR_SCTRL0_MSTR_MASK                     (0x20U)
#define TMR_SCTRL0_MSTR_SHIFT                    (5U)
#define TMR_SCTRL0_MSTR(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_MSTR_SHIFT)) & TMR_SCTRL0_MSTR_MASK)
/*! CAPTURE_MODE - Input Capture Mode
 *  0b00..Capture function is disabled
 *  0b01..Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input
 *  0b10..Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input
 *  0b11..Load capture register on both edges of input
 */
#define TMR_SCTRL0_CAPTURE_MODE_MASK             (0xC0U)
#define TMR_SCTRL0_CAPTURE_MODE_SHIFT            (6U)
#define TMR_SCTRL0_CAPTURE_MODE(x)               (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_CAPTURE_MODE_SHIFT)) & TMR_SCTRL0_CAPTURE_MODE_MASK)
#define TMR_SCTRL0_INPUT_MASK                    (0x100U)
#define TMR_SCTRL0_INPUT_SHIFT                   (8U)
#define TMR_SCTRL0_INPUT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_INPUT_SHIFT)) & TMR_SCTRL0_INPUT_MASK)
#define TMR_SCTRL0_IPS_MASK                      (0x200U)
#define TMR_SCTRL0_IPS_SHIFT                     (9U)
#define TMR_SCTRL0_IPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_IPS_SHIFT)) & TMR_SCTRL0_IPS_MASK)
#define TMR_SCTRL0_IEFIE_MASK                    (0x400U)
#define TMR_SCTRL0_IEFIE_SHIFT                   (10U)
#define TMR_SCTRL0_IEFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_IEFIE_SHIFT)) & TMR_SCTRL0_IEFIE_MASK)
#define TMR_SCTRL0_IEF_MASK                      (0x800U)
#define TMR_SCTRL0_IEF_SHIFT                     (11U)
#define TMR_SCTRL0_IEF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_IEF_SHIFT)) & TMR_SCTRL0_IEF_MASK)
#define TMR_SCTRL0_TOFIE_MASK                    (0x1000U)
#define TMR_SCTRL0_TOFIE_SHIFT                   (12U)
#define TMR_SCTRL0_TOFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_TOFIE_SHIFT)) & TMR_SCTRL0_TOFIE_MASK)
#define TMR_SCTRL0_TOF_MASK                      (0x2000U)
#define TMR_SCTRL0_TOF_SHIFT                     (13U)
#define TMR_SCTRL0_TOF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_TOF_SHIFT)) & TMR_SCTRL0_TOF_MASK)
#define TMR_SCTRL0_TCFIE_MASK                    (0x4000U)
#define TMR_SCTRL0_TCFIE_SHIFT                   (14U)
#define TMR_SCTRL0_TCFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_TCFIE_SHIFT)) & TMR_SCTRL0_TCFIE_MASK)
#define TMR_SCTRL0_TCF_MASK                      (0x8000U)
#define TMR_SCTRL0_TCF_SHIFT                     (15U)
#define TMR_SCTRL0_TCF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL0_TCF_SHIFT)) & TMR_SCTRL0_TCF_MASK)
/*! @} */

/*! @name CMPLD10 - Timer Channel Comparator Load Register 1 */
/*! @{ */
#define TMR_CMPLD10_COMPARATOR_LOAD_1_MASK       (0xFFFFU)
#define TMR_CMPLD10_COMPARATOR_LOAD_1_SHIFT      (0U)
#define TMR_CMPLD10_COMPARATOR_LOAD_1(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD10_COMPARATOR_LOAD_1_SHIFT)) & TMR_CMPLD10_COMPARATOR_LOAD_1_MASK)
/*! @} */

/*! @name CMPLD20 - Timer Channel Comparator Load Register 2 */
/*! @{ */
#define TMR_CMPLD20_COMPARATOR_LOAD_2_MASK       (0xFFFFU)
#define TMR_CMPLD20_COMPARATOR_LOAD_2_SHIFT      (0U)
#define TMR_CMPLD20_COMPARATOR_LOAD_2(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD20_COMPARATOR_LOAD_2_SHIFT)) & TMR_CMPLD20_COMPARATOR_LOAD_2_MASK)
/*! @} */

/*! @name CSCTRL0 - Timer Channel Comparator Status and Control Register */
/*! @{ */
/*! CL1 - Compare Load Control 1
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL0_CL1_MASK                     (0x3U)
#define TMR_CSCTRL0_CL1_SHIFT                    (0U)
#define TMR_CSCTRL0_CL1(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_CL1_SHIFT)) & TMR_CSCTRL0_CL1_MASK)
/*! CL2 - Compare Load Control 2
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL0_CL2_MASK                     (0xCU)
#define TMR_CSCTRL0_CL2_SHIFT                    (2U)
#define TMR_CSCTRL0_CL2(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_CL2_SHIFT)) & TMR_CSCTRL0_CL2_MASK)
#define TMR_CSCTRL0_TCF1_MASK                    (0x10U)
#define TMR_CSCTRL0_TCF1_SHIFT                   (4U)
#define TMR_CSCTRL0_TCF1(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_TCF1_SHIFT)) & TMR_CSCTRL0_TCF1_MASK)
#define TMR_CSCTRL0_TCF2_MASK                    (0x20U)
#define TMR_CSCTRL0_TCF2_SHIFT                   (5U)
#define TMR_CSCTRL0_TCF2(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_TCF2_SHIFT)) & TMR_CSCTRL0_TCF2_MASK)
#define TMR_CSCTRL0_TCF1EN_MASK                  (0x40U)
#define TMR_CSCTRL0_TCF1EN_SHIFT                 (6U)
#define TMR_CSCTRL0_TCF1EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_TCF1EN_SHIFT)) & TMR_CSCTRL0_TCF1EN_MASK)
#define TMR_CSCTRL0_TCF2EN_MASK                  (0x80U)
#define TMR_CSCTRL0_TCF2EN_SHIFT                 (7U)
#define TMR_CSCTRL0_TCF2EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_TCF2EN_SHIFT)) & TMR_CSCTRL0_TCF2EN_MASK)
#define TMR_CSCTRL0_OFLAG_MASK                   (0x100U)
#define TMR_CSCTRL0_OFLAG_SHIFT                  (8U)
#define TMR_CSCTRL0_OFLAG(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_OFLAG_SHIFT)) & TMR_CSCTRL0_OFLAG_MASK)
/*! UP - Counting Direction Indicator
 *  0b0..The last count was in the DOWN direction.
 *  0b1..The last count was in the UP direction.
 */
#define TMR_CSCTRL0_UP_MASK                      (0x200U)
#define TMR_CSCTRL0_UP_SHIFT                     (9U)
#define TMR_CSCTRL0_UP(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_UP_SHIFT)) & TMR_CSCTRL0_UP_MASK)
/*! TCI - Triggered Count Initialization Control
 *  0b0..Stop counter upon receiving a second trigger event while still counting from the first trigger event.
 *  0b1..Reload the counter upon receiving a second trigger event while still counting from the first trigger event.
 */
#define TMR_CSCTRL0_TCI_MASK                     (0x400U)
#define TMR_CSCTRL0_TCI_SHIFT                    (10U)
#define TMR_CSCTRL0_TCI(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_TCI_SHIFT)) & TMR_CSCTRL0_TCI_MASK)
/*! ROC - Reload on Capture
 *  0b0..Do not reload the counter on a capture event.
 *  0b1..Reload the counter on a capture event.
 */
#define TMR_CSCTRL0_ROC_MASK                     (0x800U)
#define TMR_CSCTRL0_ROC_SHIFT                    (11U)
#define TMR_CSCTRL0_ROC(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_ROC_SHIFT)) & TMR_CSCTRL0_ROC_MASK)
/*! ALT_LOAD - Alternative Load Enable
 *  0b0..Counter can be re-initialized only with the LOAD register.
 *  0b1..Counter can be re-initialized with the LOAD or CMPLD2 registers depending on count direction.
 */
#define TMR_CSCTRL0_ALT_LOAD_MASK                (0x1000U)
#define TMR_CSCTRL0_ALT_LOAD_SHIFT               (12U)
#define TMR_CSCTRL0_ALT_LOAD(x)                  (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_ALT_LOAD_SHIFT)) & TMR_CSCTRL0_ALT_LOAD_MASK)
/*! FAULT - Fault Enable
 *  0b0..Fault function disabled.
 *  0b1..Fault function enabled.
 */
#define TMR_CSCTRL0_FAULT_MASK                   (0x2000U)
#define TMR_CSCTRL0_FAULT_SHIFT                  (13U)
#define TMR_CSCTRL0_FAULT(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_FAULT_SHIFT)) & TMR_CSCTRL0_FAULT_MASK)
/*! DBG_EN - Debug Actions Enable
 *  0b00..Continue with normal operation during debug mode. (default)
 *  0b01..Halt TMR counter during debug mode.
 *  0b10..Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]).
 *  0b11..Both halt counter and force output to 0 during debug mode.
 */
#define TMR_CSCTRL0_DBG_EN_MASK                  (0xC000U)
#define TMR_CSCTRL0_DBG_EN_SHIFT                 (14U)
#define TMR_CSCTRL0_DBG_EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL0_DBG_EN_SHIFT)) & TMR_CSCTRL0_DBG_EN_MASK)
/*! @} */

/*! @name FILT0 - Timer Channel Input Filter Register */
/*! @{ */
#define TMR_FILT0_FILT_PER_MASK                  (0xFFU)
#define TMR_FILT0_FILT_PER_SHIFT                 (0U)
#define TMR_FILT0_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT0_FILT_PER_SHIFT)) & TMR_FILT0_FILT_PER_MASK)
#define TMR_FILT0_FILT_CNT_MASK                  (0x700U)
#define TMR_FILT0_FILT_CNT_SHIFT                 (8U)
#define TMR_FILT0_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT0_FILT_CNT_SHIFT)) & TMR_FILT0_FILT_CNT_MASK)
/*! @} */

/*! @name DMA0 - Timer Channel DMA Enable Register */
/*! @{ */
#define TMR_DMA0_IEFDE_MASK                      (0x1U)
#define TMR_DMA0_IEFDE_SHIFT                     (0U)
#define TMR_DMA0_IEFDE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_DMA0_IEFDE_SHIFT)) & TMR_DMA0_IEFDE_MASK)
#define TMR_DMA0_CMPLD1DE_MASK                   (0x2U)
#define TMR_DMA0_CMPLD1DE_SHIFT                  (1U)
#define TMR_DMA0_CMPLD1DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA0_CMPLD1DE_SHIFT)) & TMR_DMA0_CMPLD1DE_MASK)
#define TMR_DMA0_CMPLD2DE_MASK                   (0x4U)
#define TMR_DMA0_CMPLD2DE_SHIFT                  (2U)
#define TMR_DMA0_CMPLD2DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA0_CMPLD2DE_SHIFT)) & TMR_DMA0_CMPLD2DE_MASK)
/*! @} */

/*! @name ENBL - Timer Channel Enable Register */
/*! @{ */
/*! ENBL - Timer Channel Enable
 *  0b0000..Timer channel is disabled.
 *  0b0001..Timer channel is enabled. (default)
 */
#define TMR_ENBL_ENBL_MASK                       (0xFU)
#define TMR_ENBL_ENBL_SHIFT                      (0U)
#define TMR_ENBL_ENBL(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_ENBL_ENBL_SHIFT)) & TMR_ENBL_ENBL_MASK)
/*! @} */

/*! @name COMP11 - Timer Channel Compare Register 1 */
/*! @{ */
#define TMR_COMP11_COMPARISON_1_MASK             (0xFFFFU)
#define TMR_COMP11_COMPARISON_1_SHIFT            (0U)
#define TMR_COMP11_COMPARISON_1(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP11_COMPARISON_1_SHIFT)) & TMR_COMP11_COMPARISON_1_MASK)
/*! @} */

/*! @name COMP21 - Timer Channel Compare Register 2 */
/*! @{ */
#define TMR_COMP21_COMPARISON_2_MASK             (0xFFFFU)
#define TMR_COMP21_COMPARISON_2_SHIFT            (0U)
#define TMR_COMP21_COMPARISON_2(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP21_COMPARISON_2_SHIFT)) & TMR_COMP21_COMPARISON_2_MASK)
/*! @} */

/*! @name CAPT1 - Timer Channel Capture Register */
/*! @{ */
#define TMR_CAPT1_CAPTURE_MASK                   (0xFFFFU)
#define TMR_CAPT1_CAPTURE_SHIFT                  (0U)
#define TMR_CAPT1_CAPTURE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CAPT1_CAPTURE_SHIFT)) & TMR_CAPT1_CAPTURE_MASK)
/*! @} */

/*! @name LOAD1 - Timer Channel Load Register */
/*! @{ */
#define TMR_LOAD1_LOAD_MASK                      (0xFFFFU)
#define TMR_LOAD1_LOAD_SHIFT                     (0U)
#define TMR_LOAD1_LOAD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_LOAD1_LOAD_SHIFT)) & TMR_LOAD1_LOAD_MASK)
/*! @} */

/*! @name HOLD1 - Timer Channel Hold Register */
/*! @{ */
#define TMR_HOLD1_HOLD_MASK                      (0xFFFFU)
#define TMR_HOLD1_HOLD_SHIFT                     (0U)
#define TMR_HOLD1_HOLD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_HOLD1_HOLD_SHIFT)) & TMR_HOLD1_HOLD_MASK)
/*! @} */

/*! @name CNTR1 - Timer Channel Counter Register */
/*! @{ */
#define TMR_CNTR1_COUNTER_MASK                   (0xFFFFU)
#define TMR_CNTR1_COUNTER_SHIFT                  (0U)
#define TMR_CNTR1_COUNTER(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CNTR1_COUNTER_SHIFT)) & TMR_CNTR1_COUNTER_MASK)
/*! @} */

/*! @name CTRL1 - Timer Channel Control Register */
/*! @{ */
/*! OUTMODE - Output Mode
 *  0b000..Asserted while counter is active
 *  0b001..Clear OFLAG output on successful compare
 *  0b010..Set OFLAG output on successful compare
 *  0b011..Toggle OFLAG output on successful compare
 *  0b100..Toggle OFLAG output using alternating compare registers
 *  0b101..Set on compare, cleared on secondary source input edge
 *  0b110..Set on compare, cleared on counter rollover
 *  0b111..Enable gated clock output while counter is active
 */
#define TMR_CTRL1_OUTMODE_MASK                   (0x7U)
#define TMR_CTRL1_OUTMODE_SHIFT                  (0U)
#define TMR_CTRL1_OUTMODE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_OUTMODE_SHIFT)) & TMR_CTRL1_OUTMODE_MASK)
/*! COINIT - Co-Channel Initialization
 *  0b0..Co-channel counter/timers cannot force a re-initialization of this counter/timer
 *  0b1..Co-channel counter/timers may force a re-initialization of this counter/timer
 */
#define TMR_CTRL1_COINIT_MASK                    (0x8U)
#define TMR_CTRL1_COINIT_SHIFT                   (3U)
#define TMR_CTRL1_COINIT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_COINIT_SHIFT)) & TMR_CTRL1_COINIT_MASK)
/*! DIR - Count Direction
 *  0b0..Count up.
 *  0b1..Count down.
 */
#define TMR_CTRL1_DIR_MASK                       (0x10U)
#define TMR_CTRL1_DIR_SHIFT                      (4U)
#define TMR_CTRL1_DIR(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_DIR_SHIFT)) & TMR_CTRL1_DIR_MASK)
/*! LENGTH - Count Length
 *  0b0..Count until roll over at $FFFF and then continue by re-initializing the counter from the LOAD register.
 *  0b1..Count until compare, then re-initialize using the LOAD regsiter. If counting up, a successful compare
 *       occurs when the counter reaches a COMP1 value. If counting down, a successful compare occurs when the counter
 *       reaches a COMP2 value. When output mode $4 is used, alternating values of COMP1 and COMP2 are used to
 *       generate successful comparisons. For example, the counter counts until a COMP1 value is reached,
 *       re-initializes, counts until COMP2 value is reached, re-initializes, counts until COMP1 value is reached, and so on.
 */
#define TMR_CTRL1_LENGTH_MASK                    (0x20U)
#define TMR_CTRL1_LENGTH_SHIFT                   (5U)
#define TMR_CTRL1_LENGTH(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_LENGTH_SHIFT)) & TMR_CTRL1_LENGTH_MASK)
/*! ONCE - Count Once
 *  0b0..Count repeatedly.
 *  0b1..Count until compare and then stop. If counting up, a successful compare occurs when the counter reaches a
 *       COMP1 value. If counting down, a successful compare occurs when the counter reaches a COMP2 value. When
 *       output mode $4 is used, the counter re-initializes after reaching the COMP1 value, continues to count to
 *       the COMP2 value, and then stops.
 */
#define TMR_CTRL1_ONCE_MASK                      (0x40U)
#define TMR_CTRL1_ONCE_SHIFT                     (6U)
#define TMR_CTRL1_ONCE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_ONCE_SHIFT)) & TMR_CTRL1_ONCE_MASK)
/*! SCS - Secondary Count Source
 *  0b00..Counter 0 input pin
 *  0b01..Counter 1 input pin
 *  0b10..Counter 2 input pin
 *  0b11..Counter 3 input pin
 */
#define TMR_CTRL1_SCS_MASK                       (0x180U)
#define TMR_CTRL1_SCS_SHIFT                      (7U)
#define TMR_CTRL1_SCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_SCS_SHIFT)) & TMR_CTRL1_SCS_MASK)
/*! PCS - Primary Count Source
 *  0b0000..Counter 0 input pin
 *  0b0001..Counter 1 input pin
 *  0b0010..Counter 2 input pin
 *  0b0011..Counter 3 input pin
 *  0b0100..Counter 0 output
 *  0b0101..Counter 1 output
 *  0b0110..Counter 2 output
 *  0b0111..Counter 3 output
 *  0b1000..IP bus clock divide by 1 prescaler
 *  0b1001..IP bus clock divide by 2 prescaler
 *  0b1010..IP bus clock divide by 4 prescaler
 *  0b1011..IP bus clock divide by 8 prescaler
 *  0b1100..IP bus clock divide by 16 prescaler
 *  0b1101..IP bus clock divide by 32 prescaler
 *  0b1110..IP bus clock divide by 64 prescaler
 *  0b1111..IP bus clock divide by 128 prescaler
 */
#define TMR_CTRL1_PCS_MASK                       (0x1E00U)
#define TMR_CTRL1_PCS_SHIFT                      (9U)
#define TMR_CTRL1_PCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_PCS_SHIFT)) & TMR_CTRL1_PCS_MASK)
/*! CM - Count Mode
 *  0b000..No operation
 *  0b001..Count rising edges of primary sourceRising edges are counted only when SCTRL[IPS] = 0. Falling edges
 *         are counted when SCTRL[IPS] = 1. If the primary count source is IP bus clock divide by 1, only rising
 *         edges are counted regardless of the value of SCTRL[IPS].
 *  0b010..Count rising and falling edges of primary sourceIP bus clock divide by 1 cannot be used as a primary count source in edge count mode.
 *  0b011..Count rising edges of primary source while secondary input high active
 *  0b100..Quadrature count mode, uses primary and secondary sources
 *  0b101..Count rising edges of primary source; secondary source specifies directionRising edges are counted only
 *         when SCTRL[IPS] = 0. Falling edges are counted when SCTRL[IPS] = 1.
 *  0b110..Edge of secondary source triggers primary count until compare
 *  0b111..Cascaded counter mode (up/down)The primary count source must be set to one of the counter outputs.
 */
#define TMR_CTRL1_CM_MASK                        (0xE000U)
#define TMR_CTRL1_CM_SHIFT                       (13U)
#define TMR_CTRL1_CM(x)                          (((uint16_t)(((uint16_t)(x)) << TMR_CTRL1_CM_SHIFT)) & TMR_CTRL1_CM_MASK)
/*! @} */

/*! @name SCTRL1 - Timer Channel Status and Control Register */
/*! @{ */
/*! OEN - Output Enable
 *  0b0..The external pin is configured as an input.
 *  0b1..The OFLAG output signal is driven on the external pin. Other timer groups using this external pin as
 *       their input see the driven value. The polarity of the signal is determined by OPS.
 */
#define TMR_SCTRL1_OEN_MASK                      (0x1U)
#define TMR_SCTRL1_OEN_SHIFT                     (0U)
#define TMR_SCTRL1_OEN(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_OEN_SHIFT)) & TMR_SCTRL1_OEN_MASK)
/*! OPS - Output Polarity Select
 *  0b0..True polarity.
 *  0b1..Inverted polarity.
 */
#define TMR_SCTRL1_OPS_MASK                      (0x2U)
#define TMR_SCTRL1_OPS_SHIFT                     (1U)
#define TMR_SCTRL1_OPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_OPS_SHIFT)) & TMR_SCTRL1_OPS_MASK)
#define TMR_SCTRL1_FORCE_MASK                    (0x4U)
#define TMR_SCTRL1_FORCE_SHIFT                   (2U)
#define TMR_SCTRL1_FORCE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_FORCE_SHIFT)) & TMR_SCTRL1_FORCE_MASK)
#define TMR_SCTRL1_VAL_MASK                      (0x8U)
#define TMR_SCTRL1_VAL_SHIFT                     (3U)
#define TMR_SCTRL1_VAL(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_VAL_SHIFT)) & TMR_SCTRL1_VAL_MASK)
#define TMR_SCTRL1_EEOF_MASK                     (0x10U)
#define TMR_SCTRL1_EEOF_SHIFT                    (4U)
#define TMR_SCTRL1_EEOF(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_EEOF_SHIFT)) & TMR_SCTRL1_EEOF_MASK)
#define TMR_SCTRL1_MSTR_MASK                     (0x20U)
#define TMR_SCTRL1_MSTR_SHIFT                    (5U)
#define TMR_SCTRL1_MSTR(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_MSTR_SHIFT)) & TMR_SCTRL1_MSTR_MASK)
/*! CAPTURE_MODE - Input Capture Mode
 *  0b00..Capture function is disabled
 *  0b01..Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input
 *  0b10..Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input
 *  0b11..Load capture register on both edges of input
 */
#define TMR_SCTRL1_CAPTURE_MODE_MASK             (0xC0U)
#define TMR_SCTRL1_CAPTURE_MODE_SHIFT            (6U)
#define TMR_SCTRL1_CAPTURE_MODE(x)               (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_CAPTURE_MODE_SHIFT)) & TMR_SCTRL1_CAPTURE_MODE_MASK)
#define TMR_SCTRL1_INPUT_MASK                    (0x100U)
#define TMR_SCTRL1_INPUT_SHIFT                   (8U)
#define TMR_SCTRL1_INPUT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_INPUT_SHIFT)) & TMR_SCTRL1_INPUT_MASK)
#define TMR_SCTRL1_IPS_MASK                      (0x200U)
#define TMR_SCTRL1_IPS_SHIFT                     (9U)
#define TMR_SCTRL1_IPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_IPS_SHIFT)) & TMR_SCTRL1_IPS_MASK)
#define TMR_SCTRL1_IEFIE_MASK                    (0x400U)
#define TMR_SCTRL1_IEFIE_SHIFT                   (10U)
#define TMR_SCTRL1_IEFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_IEFIE_SHIFT)) & TMR_SCTRL1_IEFIE_MASK)
#define TMR_SCTRL1_IEF_MASK                      (0x800U)
#define TMR_SCTRL1_IEF_SHIFT                     (11U)
#define TMR_SCTRL1_IEF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_IEF_SHIFT)) & TMR_SCTRL1_IEF_MASK)
#define TMR_SCTRL1_TOFIE_MASK                    (0x1000U)
#define TMR_SCTRL1_TOFIE_SHIFT                   (12U)
#define TMR_SCTRL1_TOFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_TOFIE_SHIFT)) & TMR_SCTRL1_TOFIE_MASK)
#define TMR_SCTRL1_TOF_MASK                      (0x2000U)
#define TMR_SCTRL1_TOF_SHIFT                     (13U)
#define TMR_SCTRL1_TOF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_TOF_SHIFT)) & TMR_SCTRL1_TOF_MASK)
#define TMR_SCTRL1_TCFIE_MASK                    (0x4000U)
#define TMR_SCTRL1_TCFIE_SHIFT                   (14U)
#define TMR_SCTRL1_TCFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_TCFIE_SHIFT)) & TMR_SCTRL1_TCFIE_MASK)
#define TMR_SCTRL1_TCF_MASK                      (0x8000U)
#define TMR_SCTRL1_TCF_SHIFT                     (15U)
#define TMR_SCTRL1_TCF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL1_TCF_SHIFT)) & TMR_SCTRL1_TCF_MASK)
/*! @} */

/*! @name CMPLD11 - Timer Channel Comparator Load Register 1 */
/*! @{ */
#define TMR_CMPLD11_COMPARATOR_LOAD_1_MASK       (0xFFFFU)
#define TMR_CMPLD11_COMPARATOR_LOAD_1_SHIFT      (0U)
#define TMR_CMPLD11_COMPARATOR_LOAD_1(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD11_COMPARATOR_LOAD_1_SHIFT)) & TMR_CMPLD11_COMPARATOR_LOAD_1_MASK)
/*! @} */

/*! @name CMPLD21 - Timer Channel Comparator Load Register 2 */
/*! @{ */
#define TMR_CMPLD21_COMPARATOR_LOAD_2_MASK       (0xFFFFU)
#define TMR_CMPLD21_COMPARATOR_LOAD_2_SHIFT      (0U)
#define TMR_CMPLD21_COMPARATOR_LOAD_2(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD21_COMPARATOR_LOAD_2_SHIFT)) & TMR_CMPLD21_COMPARATOR_LOAD_2_MASK)
/*! @} */

/*! @name CSCTRL1 - Timer Channel Comparator Status and Control Register */
/*! @{ */
/*! CL1 - Compare Load Control 1
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL1_CL1_MASK                     (0x3U)
#define TMR_CSCTRL1_CL1_SHIFT                    (0U)
#define TMR_CSCTRL1_CL1(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_CL1_SHIFT)) & TMR_CSCTRL1_CL1_MASK)
/*! CL2 - Compare Load Control 2
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL1_CL2_MASK                     (0xCU)
#define TMR_CSCTRL1_CL2_SHIFT                    (2U)
#define TMR_CSCTRL1_CL2(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_CL2_SHIFT)) & TMR_CSCTRL1_CL2_MASK)
#define TMR_CSCTRL1_TCF1_MASK                    (0x10U)
#define TMR_CSCTRL1_TCF1_SHIFT                   (4U)
#define TMR_CSCTRL1_TCF1(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_TCF1_SHIFT)) & TMR_CSCTRL1_TCF1_MASK)
#define TMR_CSCTRL1_TCF2_MASK                    (0x20U)
#define TMR_CSCTRL1_TCF2_SHIFT                   (5U)
#define TMR_CSCTRL1_TCF2(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_TCF2_SHIFT)) & TMR_CSCTRL1_TCF2_MASK)
#define TMR_CSCTRL1_TCF1EN_MASK                  (0x40U)
#define TMR_CSCTRL1_TCF1EN_SHIFT                 (6U)
#define TMR_CSCTRL1_TCF1EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_TCF1EN_SHIFT)) & TMR_CSCTRL1_TCF1EN_MASK)
#define TMR_CSCTRL1_TCF2EN_MASK                  (0x80U)
#define TMR_CSCTRL1_TCF2EN_SHIFT                 (7U)
#define TMR_CSCTRL1_TCF2EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_TCF2EN_SHIFT)) & TMR_CSCTRL1_TCF2EN_MASK)
#define TMR_CSCTRL1_OFLAG_MASK                   (0x100U)
#define TMR_CSCTRL1_OFLAG_SHIFT                  (8U)
#define TMR_CSCTRL1_OFLAG(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_OFLAG_SHIFT)) & TMR_CSCTRL1_OFLAG_MASK)
/*! UP - Counting Direction Indicator
 *  0b0..The last count was in the DOWN direction.
 *  0b1..The last count was in the UP direction.
 */
#define TMR_CSCTRL1_UP_MASK                      (0x200U)
#define TMR_CSCTRL1_UP_SHIFT                     (9U)
#define TMR_CSCTRL1_UP(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_UP_SHIFT)) & TMR_CSCTRL1_UP_MASK)
/*! TCI - Triggered Count Initialization Control
 *  0b0..Stop counter upon receiving a second trigger event while still counting from the first trigger event.
 *  0b1..Reload the counter upon receiving a second trigger event while still counting from the first trigger event.
 */
#define TMR_CSCTRL1_TCI_MASK                     (0x400U)
#define TMR_CSCTRL1_TCI_SHIFT                    (10U)
#define TMR_CSCTRL1_TCI(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_TCI_SHIFT)) & TMR_CSCTRL1_TCI_MASK)
/*! ROC - Reload on Capture
 *  0b0..Do not reload the counter on a capture event.
 *  0b1..Reload the counter on a capture event.
 */
#define TMR_CSCTRL1_ROC_MASK                     (0x800U)
#define TMR_CSCTRL1_ROC_SHIFT                    (11U)
#define TMR_CSCTRL1_ROC(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_ROC_SHIFT)) & TMR_CSCTRL1_ROC_MASK)
/*! ALT_LOAD - Alternative Load Enable
 *  0b0..Counter can be re-initialized only with the LOAD register.
 *  0b1..Counter can be re-initialized with the LOAD or CMPLD2 registers depending on count direction.
 */
#define TMR_CSCTRL1_ALT_LOAD_MASK                (0x1000U)
#define TMR_CSCTRL1_ALT_LOAD_SHIFT               (12U)
#define TMR_CSCTRL1_ALT_LOAD(x)                  (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_ALT_LOAD_SHIFT)) & TMR_CSCTRL1_ALT_LOAD_MASK)
/*! FAULT - Fault Enable
 *  0b0..Fault function disabled.
 *  0b1..Fault function enabled.
 */
#define TMR_CSCTRL1_FAULT_MASK                   (0x2000U)
#define TMR_CSCTRL1_FAULT_SHIFT                  (13U)
#define TMR_CSCTRL1_FAULT(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_FAULT_SHIFT)) & TMR_CSCTRL1_FAULT_MASK)
/*! DBG_EN - Debug Actions Enable
 *  0b00..Continue with normal operation during debug mode. (default)
 *  0b01..Halt TMR counter during debug mode.
 *  0b10..Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]).
 *  0b11..Both halt counter and force output to 0 during debug mode.
 */
#define TMR_CSCTRL1_DBG_EN_MASK                  (0xC000U)
#define TMR_CSCTRL1_DBG_EN_SHIFT                 (14U)
#define TMR_CSCTRL1_DBG_EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL1_DBG_EN_SHIFT)) & TMR_CSCTRL1_DBG_EN_MASK)
/*! @} */

/*! @name FILT1 - Timer Channel Input Filter Register */
/*! @{ */
#define TMR_FILT1_FILT_PER_MASK                  (0xFFU)
#define TMR_FILT1_FILT_PER_SHIFT                 (0U)
#define TMR_FILT1_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT1_FILT_PER_SHIFT)) & TMR_FILT1_FILT_PER_MASK)
#define TMR_FILT1_FILT_CNT_MASK                  (0x700U)
#define TMR_FILT1_FILT_CNT_SHIFT                 (8U)
#define TMR_FILT1_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT1_FILT_CNT_SHIFT)) & TMR_FILT1_FILT_CNT_MASK)
/*! @} */

/*! @name DMA1 - Timer Channel DMA Enable Register */
/*! @{ */
#define TMR_DMA1_IEFDE_MASK                      (0x1U)
#define TMR_DMA1_IEFDE_SHIFT                     (0U)
#define TMR_DMA1_IEFDE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_DMA1_IEFDE_SHIFT)) & TMR_DMA1_IEFDE_MASK)
#define TMR_DMA1_CMPLD1DE_MASK                   (0x2U)
#define TMR_DMA1_CMPLD1DE_SHIFT                  (1U)
#define TMR_DMA1_CMPLD1DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA1_CMPLD1DE_SHIFT)) & TMR_DMA1_CMPLD1DE_MASK)
#define TMR_DMA1_CMPLD2DE_MASK                   (0x4U)
#define TMR_DMA1_CMPLD2DE_SHIFT                  (2U)
#define TMR_DMA1_CMPLD2DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA1_CMPLD2DE_SHIFT)) & TMR_DMA1_CMPLD2DE_MASK)
/*! @} */

/*! @name COMP12 - Timer Channel Compare Register 1 */
/*! @{ */
#define TMR_COMP12_COMPARISON_1_MASK             (0xFFFFU)
#define TMR_COMP12_COMPARISON_1_SHIFT            (0U)
#define TMR_COMP12_COMPARISON_1(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP12_COMPARISON_1_SHIFT)) & TMR_COMP12_COMPARISON_1_MASK)
/*! @} */

/*! @name COMP22 - Timer Channel Compare Register 2 */
/*! @{ */
#define TMR_COMP22_COMPARISON_2_MASK             (0xFFFFU)
#define TMR_COMP22_COMPARISON_2_SHIFT            (0U)
#define TMR_COMP22_COMPARISON_2(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP22_COMPARISON_2_SHIFT)) & TMR_COMP22_COMPARISON_2_MASK)
/*! @} */

/*! @name CAPT2 - Timer Channel Capture Register */
/*! @{ */
#define TMR_CAPT2_CAPTURE_MASK                   (0xFFFFU)
#define TMR_CAPT2_CAPTURE_SHIFT                  (0U)
#define TMR_CAPT2_CAPTURE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CAPT2_CAPTURE_SHIFT)) & TMR_CAPT2_CAPTURE_MASK)
/*! @} */

/*! @name LOAD2 - Timer Channel Load Register */
/*! @{ */
#define TMR_LOAD2_LOAD_MASK                      (0xFFFFU)
#define TMR_LOAD2_LOAD_SHIFT                     (0U)
#define TMR_LOAD2_LOAD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_LOAD2_LOAD_SHIFT)) & TMR_LOAD2_LOAD_MASK)
/*! @} */

/*! @name HOLD2 - Timer Channel Hold Register */
/*! @{ */
#define TMR_HOLD2_HOLD_MASK                      (0xFFFFU)
#define TMR_HOLD2_HOLD_SHIFT                     (0U)
#define TMR_HOLD2_HOLD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_HOLD2_HOLD_SHIFT)) & TMR_HOLD2_HOLD_MASK)
/*! @} */

/*! @name CNTR2 - Timer Channel Counter Register */
/*! @{ */
#define TMR_CNTR2_COUNTER_MASK                   (0xFFFFU)
#define TMR_CNTR2_COUNTER_SHIFT                  (0U)
#define TMR_CNTR2_COUNTER(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CNTR2_COUNTER_SHIFT)) & TMR_CNTR2_COUNTER_MASK)
/*! @} */

/*! @name CTRL2 - Timer Channel Control Register */
/*! @{ */
/*! OUTMODE - Output Mode
 *  0b000..Asserted while counter is active
 *  0b001..Clear OFLAG output on successful compare
 *  0b010..Set OFLAG output on successful compare
 *  0b011..Toggle OFLAG output on successful compare
 *  0b100..Toggle OFLAG output using alternating compare registers
 *  0b101..Set on compare, cleared on secondary source input edge
 *  0b110..Set on compare, cleared on counter rollover
 *  0b111..Enable gated clock output while counter is active
 */
#define TMR_CTRL2_OUTMODE_MASK                   (0x7U)
#define TMR_CTRL2_OUTMODE_SHIFT                  (0U)
#define TMR_CTRL2_OUTMODE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_OUTMODE_SHIFT)) & TMR_CTRL2_OUTMODE_MASK)
/*! COINIT - Co-Channel Initialization
 *  0b0..Co-channel counter/timers cannot force a re-initialization of this counter/timer
 *  0b1..Co-channel counter/timers may force a re-initialization of this counter/timer
 */
#define TMR_CTRL2_COINIT_MASK                    (0x8U)
#define TMR_CTRL2_COINIT_SHIFT                   (3U)
#define TMR_CTRL2_COINIT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_COINIT_SHIFT)) & TMR_CTRL2_COINIT_MASK)
/*! DIR - Count Direction
 *  0b0..Count up.
 *  0b1..Count down.
 */
#define TMR_CTRL2_DIR_MASK                       (0x10U)
#define TMR_CTRL2_DIR_SHIFT                      (4U)
#define TMR_CTRL2_DIR(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_DIR_SHIFT)) & TMR_CTRL2_DIR_MASK)
/*! LENGTH - Count Length
 *  0b0..Count until roll over at $FFFF and then continue by re-initializing the counter from the LOAD register.
 *  0b1..Count until compare, then re-initialize using the LOAD regsiter. If counting up, a successful compare
 *       occurs when the counter reaches a COMP1 value. If counting down, a successful compare occurs when the counter
 *       reaches a COMP2 value. When output mode $4 is used, alternating values of COMP1 and COMP2 are used to
 *       generate successful comparisons. For example, the counter counts until a COMP1 value is reached,
 *       re-initializes, counts until COMP2 value is reached, re-initializes, counts until COMP1 value is reached, and so on.
 */
#define TMR_CTRL2_LENGTH_MASK                    (0x20U)
#define TMR_CTRL2_LENGTH_SHIFT                   (5U)
#define TMR_CTRL2_LENGTH(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_LENGTH_SHIFT)) & TMR_CTRL2_LENGTH_MASK)
/*! ONCE - Count Once
 *  0b0..Count repeatedly.
 *  0b1..Count until compare and then stop. If counting up, a successful compare occurs when the counter reaches a
 *       COMP1 value. If counting down, a successful compare occurs when the counter reaches a COMP2 value. When
 *       output mode $4 is used, the counter re-initializes after reaching the COMP1 value, continues to count to
 *       the COMP2 value, and then stops.
 */
#define TMR_CTRL2_ONCE_MASK                      (0x40U)
#define TMR_CTRL2_ONCE_SHIFT                     (6U)
#define TMR_CTRL2_ONCE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_ONCE_SHIFT)) & TMR_CTRL2_ONCE_MASK)
/*! SCS - Secondary Count Source
 *  0b00..Counter 0 input pin
 *  0b01..Counter 1 input pin
 *  0b10..Counter 2 input pin
 *  0b11..Counter 3 input pin
 */
#define TMR_CTRL2_SCS_MASK                       (0x180U)
#define TMR_CTRL2_SCS_SHIFT                      (7U)
#define TMR_CTRL2_SCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_SCS_SHIFT)) & TMR_CTRL2_SCS_MASK)
/*! PCS - Primary Count Source
 *  0b0000..Counter 0 input pin
 *  0b0001..Counter 1 input pin
 *  0b0010..Counter 2 input pin
 *  0b0011..Counter 3 input pin
 *  0b0100..Counter 0 output
 *  0b0101..Counter 1 output
 *  0b0110..Counter 2 output
 *  0b0111..Counter 3 output
 *  0b1000..IP bus clock divide by 1 prescaler
 *  0b1001..IP bus clock divide by 2 prescaler
 *  0b1010..IP bus clock divide by 4 prescaler
 *  0b1011..IP bus clock divide by 8 prescaler
 *  0b1100..IP bus clock divide by 16 prescaler
 *  0b1101..IP bus clock divide by 32 prescaler
 *  0b1110..IP bus clock divide by 64 prescaler
 *  0b1111..IP bus clock divide by 128 prescaler
 */
#define TMR_CTRL2_PCS_MASK                       (0x1E00U)
#define TMR_CTRL2_PCS_SHIFT                      (9U)
#define TMR_CTRL2_PCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_PCS_SHIFT)) & TMR_CTRL2_PCS_MASK)
/*! CM - Count Mode
 *  0b000..No operation
 *  0b001..Count rising edges of primary sourceRising edges are counted only when SCTRL[IPS] = 0. Falling edges
 *         are counted when SCTRL[IPS] = 1. If the primary count source is IP bus clock divide by 1, only rising
 *         edges are counted regardless of the value of SCTRL[IPS].
 *  0b010..Count rising and falling edges of primary sourceIP bus clock divide by 1 cannot be used as a primary count source in edge count mode.
 *  0b011..Count rising edges of primary source while secondary input high active
 *  0b100..Quadrature count mode, uses primary and secondary sources
 *  0b101..Count rising edges of primary source; secondary source specifies directionRising edges are counted only
 *         when SCTRL[IPS] = 0. Falling edges are counted when SCTRL[IPS] = 1.
 *  0b110..Edge of secondary source triggers primary count until compare
 *  0b111..Cascaded counter mode (up/down)The primary count source must be set to one of the counter outputs.
 */
#define TMR_CTRL2_CM_MASK                        (0xE000U)
#define TMR_CTRL2_CM_SHIFT                       (13U)
#define TMR_CTRL2_CM(x)                          (((uint16_t)(((uint16_t)(x)) << TMR_CTRL2_CM_SHIFT)) & TMR_CTRL2_CM_MASK)
/*! @} */

/*! @name SCTRL2 - Timer Channel Status and Control Register */
/*! @{ */
/*! OEN - Output Enable
 *  0b0..The external pin is configured as an input.
 *  0b1..The OFLAG output signal is driven on the external pin. Other timer groups using this external pin as
 *       their input see the driven value. The polarity of the signal is determined by OPS.
 */
#define TMR_SCTRL2_OEN_MASK                      (0x1U)
#define TMR_SCTRL2_OEN_SHIFT                     (0U)
#define TMR_SCTRL2_OEN(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_OEN_SHIFT)) & TMR_SCTRL2_OEN_MASK)
/*! OPS - Output Polarity Select
 *  0b0..True polarity.
 *  0b1..Inverted polarity.
 */
#define TMR_SCTRL2_OPS_MASK                      (0x2U)
#define TMR_SCTRL2_OPS_SHIFT                     (1U)
#define TMR_SCTRL2_OPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_OPS_SHIFT)) & TMR_SCTRL2_OPS_MASK)
#define TMR_SCTRL2_FORCE_MASK                    (0x4U)
#define TMR_SCTRL2_FORCE_SHIFT                   (2U)
#define TMR_SCTRL2_FORCE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_FORCE_SHIFT)) & TMR_SCTRL2_FORCE_MASK)
#define TMR_SCTRL2_VAL_MASK                      (0x8U)
#define TMR_SCTRL2_VAL_SHIFT                     (3U)
#define TMR_SCTRL2_VAL(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_VAL_SHIFT)) & TMR_SCTRL2_VAL_MASK)
#define TMR_SCTRL2_EEOF_MASK                     (0x10U)
#define TMR_SCTRL2_EEOF_SHIFT                    (4U)
#define TMR_SCTRL2_EEOF(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_EEOF_SHIFT)) & TMR_SCTRL2_EEOF_MASK)
#define TMR_SCTRL2_MSTR_MASK                     (0x20U)
#define TMR_SCTRL2_MSTR_SHIFT                    (5U)
#define TMR_SCTRL2_MSTR(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_MSTR_SHIFT)) & TMR_SCTRL2_MSTR_MASK)
/*! CAPTURE_MODE - Input Capture Mode
 *  0b00..Capture function is disabled
 *  0b01..Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input
 *  0b10..Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input
 *  0b11..Load capture register on both edges of input
 */
#define TMR_SCTRL2_CAPTURE_MODE_MASK             (0xC0U)
#define TMR_SCTRL2_CAPTURE_MODE_SHIFT            (6U)
#define TMR_SCTRL2_CAPTURE_MODE(x)               (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_CAPTURE_MODE_SHIFT)) & TMR_SCTRL2_CAPTURE_MODE_MASK)
#define TMR_SCTRL2_INPUT_MASK                    (0x100U)
#define TMR_SCTRL2_INPUT_SHIFT                   (8U)
#define TMR_SCTRL2_INPUT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_INPUT_SHIFT)) & TMR_SCTRL2_INPUT_MASK)
#define TMR_SCTRL2_IPS_MASK                      (0x200U)
#define TMR_SCTRL2_IPS_SHIFT                     (9U)
#define TMR_SCTRL2_IPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_IPS_SHIFT)) & TMR_SCTRL2_IPS_MASK)
#define TMR_SCTRL2_IEFIE_MASK                    (0x400U)
#define TMR_SCTRL2_IEFIE_SHIFT                   (10U)
#define TMR_SCTRL2_IEFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_IEFIE_SHIFT)) & TMR_SCTRL2_IEFIE_MASK)
#define TMR_SCTRL2_IEF_MASK                      (0x800U)
#define TMR_SCTRL2_IEF_SHIFT                     (11U)
#define TMR_SCTRL2_IEF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_IEF_SHIFT)) & TMR_SCTRL2_IEF_MASK)
#define TMR_SCTRL2_TOFIE_MASK                    (0x1000U)
#define TMR_SCTRL2_TOFIE_SHIFT                   (12U)
#define TMR_SCTRL2_TOFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_TOFIE_SHIFT)) & TMR_SCTRL2_TOFIE_MASK)
#define TMR_SCTRL2_TOF_MASK                      (0x2000U)
#define TMR_SCTRL2_TOF_SHIFT                     (13U)
#define TMR_SCTRL2_TOF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_TOF_SHIFT)) & TMR_SCTRL2_TOF_MASK)
#define TMR_SCTRL2_TCFIE_MASK                    (0x4000U)
#define TMR_SCTRL2_TCFIE_SHIFT                   (14U)
#define TMR_SCTRL2_TCFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_TCFIE_SHIFT)) & TMR_SCTRL2_TCFIE_MASK)
#define TMR_SCTRL2_TCF_MASK                      (0x8000U)
#define TMR_SCTRL2_TCF_SHIFT                     (15U)
#define TMR_SCTRL2_TCF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL2_TCF_SHIFT)) & TMR_SCTRL2_TCF_MASK)
/*! @} */

/*! @name CMPLD12 - Timer Channel Comparator Load Register 1 */
/*! @{ */
#define TMR_CMPLD12_COMPARATOR_LOAD_1_MASK       (0xFFFFU)
#define TMR_CMPLD12_COMPARATOR_LOAD_1_SHIFT      (0U)
#define TMR_CMPLD12_COMPARATOR_LOAD_1(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD12_COMPARATOR_LOAD_1_SHIFT)) & TMR_CMPLD12_COMPARATOR_LOAD_1_MASK)
/*! @} */

/*! @name CMPLD22 - Timer Channel Comparator Load Register 2 */
/*! @{ */
#define TMR_CMPLD22_COMPARATOR_LOAD_2_MASK       (0xFFFFU)
#define TMR_CMPLD22_COMPARATOR_LOAD_2_SHIFT      (0U)
#define TMR_CMPLD22_COMPARATOR_LOAD_2(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD22_COMPARATOR_LOAD_2_SHIFT)) & TMR_CMPLD22_COMPARATOR_LOAD_2_MASK)
/*! @} */

/*! @name CSCTRL2 - Timer Channel Comparator Status and Control Register */
/*! @{ */
/*! CL1 - Compare Load Control 1
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL2_CL1_MASK                     (0x3U)
#define TMR_CSCTRL2_CL1_SHIFT                    (0U)
#define TMR_CSCTRL2_CL1(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_CL1_SHIFT)) & TMR_CSCTRL2_CL1_MASK)
/*! CL2 - Compare Load Control 2
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL2_CL2_MASK                     (0xCU)
#define TMR_CSCTRL2_CL2_SHIFT                    (2U)
#define TMR_CSCTRL2_CL2(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_CL2_SHIFT)) & TMR_CSCTRL2_CL2_MASK)
#define TMR_CSCTRL2_TCF1_MASK                    (0x10U)
#define TMR_CSCTRL2_TCF1_SHIFT                   (4U)
#define TMR_CSCTRL2_TCF1(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_TCF1_SHIFT)) & TMR_CSCTRL2_TCF1_MASK)
#define TMR_CSCTRL2_TCF2_MASK                    (0x20U)
#define TMR_CSCTRL2_TCF2_SHIFT                   (5U)
#define TMR_CSCTRL2_TCF2(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_TCF2_SHIFT)) & TMR_CSCTRL2_TCF2_MASK)
#define TMR_CSCTRL2_TCF1EN_MASK                  (0x40U)
#define TMR_CSCTRL2_TCF1EN_SHIFT                 (6U)
#define TMR_CSCTRL2_TCF1EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_TCF1EN_SHIFT)) & TMR_CSCTRL2_TCF1EN_MASK)
#define TMR_CSCTRL2_TCF2EN_MASK                  (0x80U)
#define TMR_CSCTRL2_TCF2EN_SHIFT                 (7U)
#define TMR_CSCTRL2_TCF2EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_TCF2EN_SHIFT)) & TMR_CSCTRL2_TCF2EN_MASK)
#define TMR_CSCTRL2_OFLAG_MASK                   (0x100U)
#define TMR_CSCTRL2_OFLAG_SHIFT                  (8U)
#define TMR_CSCTRL2_OFLAG(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_OFLAG_SHIFT)) & TMR_CSCTRL2_OFLAG_MASK)
/*! UP - Counting Direction Indicator
 *  0b0..The last count was in the DOWN direction.
 *  0b1..The last count was in the UP direction.
 */
#define TMR_CSCTRL2_UP_MASK                      (0x200U)
#define TMR_CSCTRL2_UP_SHIFT                     (9U)
#define TMR_CSCTRL2_UP(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_UP_SHIFT)) & TMR_CSCTRL2_UP_MASK)
/*! TCI - Triggered Count Initialization Control
 *  0b0..Stop counter upon receiving a second trigger event while still counting from the first trigger event.
 *  0b1..Reload the counter upon receiving a second trigger event while still counting from the first trigger event.
 */
#define TMR_CSCTRL2_TCI_MASK                     (0x400U)
#define TMR_CSCTRL2_TCI_SHIFT                    (10U)
#define TMR_CSCTRL2_TCI(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_TCI_SHIFT)) & TMR_CSCTRL2_TCI_MASK)
/*! ROC - Reload on Capture
 *  0b0..Do not reload the counter on a capture event.
 *  0b1..Reload the counter on a capture event.
 */
#define TMR_CSCTRL2_ROC_MASK                     (0x800U)
#define TMR_CSCTRL2_ROC_SHIFT                    (11U)
#define TMR_CSCTRL2_ROC(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_ROC_SHIFT)) & TMR_CSCTRL2_ROC_MASK)
/*! ALT_LOAD - Alternative Load Enable
 *  0b0..Counter can be re-initialized only with the LOAD register.
 *  0b1..Counter can be re-initialized with the LOAD or CMPLD2 registers depending on count direction.
 */
#define TMR_CSCTRL2_ALT_LOAD_MASK                (0x1000U)
#define TMR_CSCTRL2_ALT_LOAD_SHIFT               (12U)
#define TMR_CSCTRL2_ALT_LOAD(x)                  (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_ALT_LOAD_SHIFT)) & TMR_CSCTRL2_ALT_LOAD_MASK)
/*! FAULT - Fault Enable
 *  0b0..Fault function disabled.
 *  0b1..Fault function enabled.
 */
#define TMR_CSCTRL2_FAULT_MASK                   (0x2000U)
#define TMR_CSCTRL2_FAULT_SHIFT                  (13U)
#define TMR_CSCTRL2_FAULT(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_FAULT_SHIFT)) & TMR_CSCTRL2_FAULT_MASK)
/*! DBG_EN - Debug Actions Enable
 *  0b00..Continue with normal operation during debug mode. (default)
 *  0b01..Halt TMR counter during debug mode.
 *  0b10..Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]).
 *  0b11..Both halt counter and force output to 0 during debug mode.
 */
#define TMR_CSCTRL2_DBG_EN_MASK                  (0xC000U)
#define TMR_CSCTRL2_DBG_EN_SHIFT                 (14U)
#define TMR_CSCTRL2_DBG_EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL2_DBG_EN_SHIFT)) & TMR_CSCTRL2_DBG_EN_MASK)
/*! @} */

/*! @name FILT2 - Timer Channel Input Filter Register */
/*! @{ */
#define TMR_FILT2_FILT_PER_MASK                  (0xFFU)
#define TMR_FILT2_FILT_PER_SHIFT                 (0U)
#define TMR_FILT2_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT2_FILT_PER_SHIFT)) & TMR_FILT2_FILT_PER_MASK)
#define TMR_FILT2_FILT_CNT_MASK                  (0x700U)
#define TMR_FILT2_FILT_CNT_SHIFT                 (8U)
#define TMR_FILT2_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT2_FILT_CNT_SHIFT)) & TMR_FILT2_FILT_CNT_MASK)
/*! @} */

/*! @name DMA2 - Timer Channel DMA Enable Register */
/*! @{ */
#define TMR_DMA2_IEFDE_MASK                      (0x1U)
#define TMR_DMA2_IEFDE_SHIFT                     (0U)
#define TMR_DMA2_IEFDE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_DMA2_IEFDE_SHIFT)) & TMR_DMA2_IEFDE_MASK)
#define TMR_DMA2_CMPLD1DE_MASK                   (0x2U)
#define TMR_DMA2_CMPLD1DE_SHIFT                  (1U)
#define TMR_DMA2_CMPLD1DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA2_CMPLD1DE_SHIFT)) & TMR_DMA2_CMPLD1DE_MASK)
#define TMR_DMA2_CMPLD2DE_MASK                   (0x4U)
#define TMR_DMA2_CMPLD2DE_SHIFT                  (2U)
#define TMR_DMA2_CMPLD2DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA2_CMPLD2DE_SHIFT)) & TMR_DMA2_CMPLD2DE_MASK)
/*! @} */

/*! @name COMP13 - Timer Channel Compare Register 1 */
/*! @{ */
#define TMR_COMP13_COMPARISON_1_MASK             (0xFFFFU)
#define TMR_COMP13_COMPARISON_1_SHIFT            (0U)
#define TMR_COMP13_COMPARISON_1(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP13_COMPARISON_1_SHIFT)) & TMR_COMP13_COMPARISON_1_MASK)
/*! @} */

/*! @name COMP23 - Timer Channel Compare Register 2 */
/*! @{ */
#define TMR_COMP23_COMPARISON_2_MASK             (0xFFFFU)
#define TMR_COMP23_COMPARISON_2_SHIFT            (0U)
#define TMR_COMP23_COMPARISON_2(x)               (((uint16_t)(((uint16_t)(x)) << TMR_COMP23_COMPARISON_2_SHIFT)) & TMR_COMP23_COMPARISON_2_MASK)
/*! @} */

/*! @name CAPT3 - Timer Channel Capture Register */
/*! @{ */
#define TMR_CAPT3_CAPTURE_MASK                   (0xFFFFU)
#define TMR_CAPT3_CAPTURE_SHIFT                  (0U)
#define TMR_CAPT3_CAPTURE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CAPT3_CAPTURE_SHIFT)) & TMR_CAPT3_CAPTURE_MASK)
/*! @} */

/*! @name LOAD3 - Timer Channel Load Register */
/*! @{ */
#define TMR_LOAD3_LOAD_MASK                      (0xFFFFU)
#define TMR_LOAD3_LOAD_SHIFT                     (0U)
#define TMR_LOAD3_LOAD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_LOAD3_LOAD_SHIFT)) & TMR_LOAD3_LOAD_MASK)
/*! @} */

/*! @name HOLD3 - Timer Channel Hold Register */
/*! @{ */
#define TMR_HOLD3_HOLD_MASK                      (0xFFFFU)
#define TMR_HOLD3_HOLD_SHIFT                     (0U)
#define TMR_HOLD3_HOLD(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_HOLD3_HOLD_SHIFT)) & TMR_HOLD3_HOLD_MASK)
/*! @} */

/*! @name CNTR3 - Timer Channel Counter Register */
/*! @{ */
#define TMR_CNTR3_COUNTER_MASK                   (0xFFFFU)
#define TMR_CNTR3_COUNTER_SHIFT                  (0U)
#define TMR_CNTR3_COUNTER(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CNTR3_COUNTER_SHIFT)) & TMR_CNTR3_COUNTER_MASK)
/*! @} */

/*! @name CTRL3 - Timer Channel Control Register */
/*! @{ */
/*! OUTMODE - Output Mode
 *  0b000..Asserted while counter is active
 *  0b001..Clear OFLAG output on successful compare
 *  0b010..Set OFLAG output on successful compare
 *  0b011..Toggle OFLAG output on successful compare
 *  0b100..Toggle OFLAG output using alternating compare registers
 *  0b101..Set on compare, cleared on secondary source input edge
 *  0b110..Set on compare, cleared on counter rollover
 *  0b111..Enable gated clock output while counter is active
 */
#define TMR_CTRL3_OUTMODE_MASK                   (0x7U)
#define TMR_CTRL3_OUTMODE_SHIFT                  (0U)
#define TMR_CTRL3_OUTMODE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_OUTMODE_SHIFT)) & TMR_CTRL3_OUTMODE_MASK)
/*! COINIT - Co-Channel Initialization
 *  0b0..Co-channel counter/timers cannot force a re-initialization of this counter/timer
 *  0b1..Co-channel counter/timers may force a re-initialization of this counter/timer
 */
#define TMR_CTRL3_COINIT_MASK                    (0x8U)
#define TMR_CTRL3_COINIT_SHIFT                   (3U)
#define TMR_CTRL3_COINIT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_COINIT_SHIFT)) & TMR_CTRL3_COINIT_MASK)
/*! DIR - Count Direction
 *  0b0..Count up.
 *  0b1..Count down.
 */
#define TMR_CTRL3_DIR_MASK                       (0x10U)
#define TMR_CTRL3_DIR_SHIFT                      (4U)
#define TMR_CTRL3_DIR(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_DIR_SHIFT)) & TMR_CTRL3_DIR_MASK)
/*! LENGTH - Count Length
 *  0b0..Count until roll over at $FFFF and then continue by re-initializing the counter from the LOAD register.
 *  0b1..Count until compare, then re-initialize using the LOAD regsiter. If counting up, a successful compare
 *       occurs when the counter reaches a COMP1 value. If counting down, a successful compare occurs when the counter
 *       reaches a COMP2 value. When output mode $4 is used, alternating values of COMP1 and COMP2 are used to
 *       generate successful comparisons. For example, the counter counts until a COMP1 value is reached,
 *       re-initializes, counts until COMP2 value is reached, re-initializes, counts until COMP1 value is reached, and so on.
 */
#define TMR_CTRL3_LENGTH_MASK                    (0x20U)
#define TMR_CTRL3_LENGTH_SHIFT                   (5U)
#define TMR_CTRL3_LENGTH(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_LENGTH_SHIFT)) & TMR_CTRL3_LENGTH_MASK)
/*! ONCE - Count Once
 *  0b0..Count repeatedly.
 *  0b1..Count until compare and then stop. If counting up, a successful compare occurs when the counter reaches a
 *       COMP1 value. If counting down, a successful compare occurs when the counter reaches a COMP2 value. When
 *       output mode $4 is used, the counter re-initializes after reaching the COMP1 value, continues to count to
 *       the COMP2 value, and then stops.
 */
#define TMR_CTRL3_ONCE_MASK                      (0x40U)
#define TMR_CTRL3_ONCE_SHIFT                     (6U)
#define TMR_CTRL3_ONCE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_ONCE_SHIFT)) & TMR_CTRL3_ONCE_MASK)
/*! SCS - Secondary Count Source
 *  0b00..Counter 0 input pin
 *  0b01..Counter 1 input pin
 *  0b10..Counter 2 input pin
 *  0b11..Counter 3 input pin
 */
#define TMR_CTRL3_SCS_MASK                       (0x180U)
#define TMR_CTRL3_SCS_SHIFT                      (7U)
#define TMR_CTRL3_SCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_SCS_SHIFT)) & TMR_CTRL3_SCS_MASK)
/*! PCS - Primary Count Source
 *  0b0000..Counter 0 input pin
 *  0b0001..Counter 1 input pin
 *  0b0010..Counter 2 input pin
 *  0b0011..Counter 3 input pin
 *  0b0100..Counter 0 output
 *  0b0101..Counter 1 output
 *  0b0110..Counter 2 output
 *  0b0111..Counter 3 output
 *  0b1000..IP bus clock divide by 1 prescaler
 *  0b1001..IP bus clock divide by 2 prescaler
 *  0b1010..IP bus clock divide by 4 prescaler
 *  0b1011..IP bus clock divide by 8 prescaler
 *  0b1100..IP bus clock divide by 16 prescaler
 *  0b1101..IP bus clock divide by 32 prescaler
 *  0b1110..IP bus clock divide by 64 prescaler
 *  0b1111..IP bus clock divide by 128 prescaler
 */
#define TMR_CTRL3_PCS_MASK                       (0x1E00U)
#define TMR_CTRL3_PCS_SHIFT                      (9U)
#define TMR_CTRL3_PCS(x)                         (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_PCS_SHIFT)) & TMR_CTRL3_PCS_MASK)
/*! CM - Count Mode
 *  0b000..No operation
 *  0b001..Count rising edges of primary sourceRising edges are counted only when SCTRL[IPS] = 0. Falling edges
 *         are counted when SCTRL[IPS] = 1. If the primary count source is IP bus clock divide by 1, only rising
 *         edges are counted regardless of the value of SCTRL[IPS].
 *  0b010..Count rising and falling edges of primary sourceIP bus clock divide by 1 cannot be used as a primary count source in edge count mode.
 *  0b011..Count rising edges of primary source while secondary input high active
 *  0b100..Quadrature count mode, uses primary and secondary sources
 *  0b101..Count rising edges of primary source; secondary source specifies directionRising edges are counted only
 *         when SCTRL[IPS] = 0. Falling edges are counted when SCTRL[IPS] = 1.
 *  0b110..Edge of secondary source triggers primary count until compare
 *  0b111..Cascaded counter mode (up/down)The primary count source must be set to one of the counter outputs.
 */
#define TMR_CTRL3_CM_MASK                        (0xE000U)
#define TMR_CTRL3_CM_SHIFT                       (13U)
#define TMR_CTRL3_CM(x)                          (((uint16_t)(((uint16_t)(x)) << TMR_CTRL3_CM_SHIFT)) & TMR_CTRL3_CM_MASK)
/*! @} */

/*! @name SCTRL3 - Timer Channel Status and Control Register */
/*! @{ */
/*! OEN - Output Enable
 *  0b0..The external pin is configured as an input.
 *  0b1..The OFLAG output signal is driven on the external pin. Other timer groups using this external pin as
 *       their input see the driven value. The polarity of the signal is determined by OPS.
 */
#define TMR_SCTRL3_OEN_MASK                      (0x1U)
#define TMR_SCTRL3_OEN_SHIFT                     (0U)
#define TMR_SCTRL3_OEN(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_OEN_SHIFT)) & TMR_SCTRL3_OEN_MASK)
/*! OPS - Output Polarity Select
 *  0b0..True polarity.
 *  0b1..Inverted polarity.
 */
#define TMR_SCTRL3_OPS_MASK                      (0x2U)
#define TMR_SCTRL3_OPS_SHIFT                     (1U)
#define TMR_SCTRL3_OPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_OPS_SHIFT)) & TMR_SCTRL3_OPS_MASK)
#define TMR_SCTRL3_FORCE_MASK                    (0x4U)
#define TMR_SCTRL3_FORCE_SHIFT                   (2U)
#define TMR_SCTRL3_FORCE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_FORCE_SHIFT)) & TMR_SCTRL3_FORCE_MASK)
#define TMR_SCTRL3_VAL_MASK                      (0x8U)
#define TMR_SCTRL3_VAL_SHIFT                     (3U)
#define TMR_SCTRL3_VAL(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_VAL_SHIFT)) & TMR_SCTRL3_VAL_MASK)
#define TMR_SCTRL3_EEOF_MASK                     (0x10U)
#define TMR_SCTRL3_EEOF_SHIFT                    (4U)
#define TMR_SCTRL3_EEOF(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_EEOF_SHIFT)) & TMR_SCTRL3_EEOF_MASK)
#define TMR_SCTRL3_MSTR_MASK                     (0x20U)
#define TMR_SCTRL3_MSTR_SHIFT                    (5U)
#define TMR_SCTRL3_MSTR(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_MSTR_SHIFT)) & TMR_SCTRL3_MSTR_MASK)
/*! CAPTURE_MODE - Input Capture Mode
 *  0b00..Capture function is disabled
 *  0b01..Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input
 *  0b10..Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input
 *  0b11..Load capture register on both edges of input
 */
#define TMR_SCTRL3_CAPTURE_MODE_MASK             (0xC0U)
#define TMR_SCTRL3_CAPTURE_MODE_SHIFT            (6U)
#define TMR_SCTRL3_CAPTURE_MODE(x)               (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_CAPTURE_MODE_SHIFT)) & TMR_SCTRL3_CAPTURE_MODE_MASK)
#define TMR_SCTRL3_INPUT_MASK                    (0x100U)
#define TMR_SCTRL3_INPUT_SHIFT                   (8U)
#define TMR_SCTRL3_INPUT(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_INPUT_SHIFT)) & TMR_SCTRL3_INPUT_MASK)
#define TMR_SCTRL3_IPS_MASK                      (0x200U)
#define TMR_SCTRL3_IPS_SHIFT                     (9U)
#define TMR_SCTRL3_IPS(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_IPS_SHIFT)) & TMR_SCTRL3_IPS_MASK)
#define TMR_SCTRL3_IEFIE_MASK                    (0x400U)
#define TMR_SCTRL3_IEFIE_SHIFT                   (10U)
#define TMR_SCTRL3_IEFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_IEFIE_SHIFT)) & TMR_SCTRL3_IEFIE_MASK)
#define TMR_SCTRL3_IEF_MASK                      (0x800U)
#define TMR_SCTRL3_IEF_SHIFT                     (11U)
#define TMR_SCTRL3_IEF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_IEF_SHIFT)) & TMR_SCTRL3_IEF_MASK)
#define TMR_SCTRL3_TOFIE_MASK                    (0x1000U)
#define TMR_SCTRL3_TOFIE_SHIFT                   (12U)
#define TMR_SCTRL3_TOFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_TOFIE_SHIFT)) & TMR_SCTRL3_TOFIE_MASK)
#define TMR_SCTRL3_TOF_MASK                      (0x2000U)
#define TMR_SCTRL3_TOF_SHIFT                     (13U)
#define TMR_SCTRL3_TOF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_TOF_SHIFT)) & TMR_SCTRL3_TOF_MASK)
#define TMR_SCTRL3_TCFIE_MASK                    (0x4000U)
#define TMR_SCTRL3_TCFIE_SHIFT                   (14U)
#define TMR_SCTRL3_TCFIE(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_TCFIE_SHIFT)) & TMR_SCTRL3_TCFIE_MASK)
#define TMR_SCTRL3_TCF_MASK                      (0x8000U)
#define TMR_SCTRL3_TCF_SHIFT                     (15U)
#define TMR_SCTRL3_TCF(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_SCTRL3_TCF_SHIFT)) & TMR_SCTRL3_TCF_MASK)
/*! @} */

/*! @name CMPLD13 - Timer Channel Comparator Load Register 1 */
/*! @{ */
#define TMR_CMPLD13_COMPARATOR_LOAD_1_MASK       (0xFFFFU)
#define TMR_CMPLD13_COMPARATOR_LOAD_1_SHIFT      (0U)
#define TMR_CMPLD13_COMPARATOR_LOAD_1(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD13_COMPARATOR_LOAD_1_SHIFT)) & TMR_CMPLD13_COMPARATOR_LOAD_1_MASK)
/*! @} */

/*! @name CMPLD23 - Timer Channel Comparator Load Register 2 */
/*! @{ */
#define TMR_CMPLD23_COMPARATOR_LOAD_2_MASK       (0xFFFFU)
#define TMR_CMPLD23_COMPARATOR_LOAD_2_SHIFT      (0U)
#define TMR_CMPLD23_COMPARATOR_LOAD_2(x)         (((uint16_t)(((uint16_t)(x)) << TMR_CMPLD23_COMPARATOR_LOAD_2_SHIFT)) & TMR_CMPLD23_COMPARATOR_LOAD_2_MASK)
/*! @} */

/*! @name CSCTRL3 - Timer Channel Comparator Status and Control Register */
/*! @{ */
/*! CL1 - Compare Load Control 1
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL3_CL1_MASK                     (0x3U)
#define TMR_CSCTRL3_CL1_SHIFT                    (0U)
#define TMR_CSCTRL3_CL1(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_CL1_SHIFT)) & TMR_CSCTRL3_CL1_MASK)
/*! CL2 - Compare Load Control 2
 *  0b00..Never preload
 *  0b01..Load upon successful compare with the value in COMP1
 *  0b10..Load upon successful compare with the value in COMP2
 *  0b11..Reserved
 */
#define TMR_CSCTRL3_CL2_MASK                     (0xCU)
#define TMR_CSCTRL3_CL2_SHIFT                    (2U)
#define TMR_CSCTRL3_CL2(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_CL2_SHIFT)) & TMR_CSCTRL3_CL2_MASK)
#define TMR_CSCTRL3_TCF1_MASK                    (0x10U)
#define TMR_CSCTRL3_TCF1_SHIFT                   (4U)
#define TMR_CSCTRL3_TCF1(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_TCF1_SHIFT)) & TMR_CSCTRL3_TCF1_MASK)
#define TMR_CSCTRL3_TCF2_MASK                    (0x20U)
#define TMR_CSCTRL3_TCF2_SHIFT                   (5U)
#define TMR_CSCTRL3_TCF2(x)                      (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_TCF2_SHIFT)) & TMR_CSCTRL3_TCF2_MASK)
#define TMR_CSCTRL3_TCF1EN_MASK                  (0x40U)
#define TMR_CSCTRL3_TCF1EN_SHIFT                 (6U)
#define TMR_CSCTRL3_TCF1EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_TCF1EN_SHIFT)) & TMR_CSCTRL3_TCF1EN_MASK)
#define TMR_CSCTRL3_TCF2EN_MASK                  (0x80U)
#define TMR_CSCTRL3_TCF2EN_SHIFT                 (7U)
#define TMR_CSCTRL3_TCF2EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_TCF2EN_SHIFT)) & TMR_CSCTRL3_TCF2EN_MASK)
#define TMR_CSCTRL3_OFLAG_MASK                   (0x100U)
#define TMR_CSCTRL3_OFLAG_SHIFT                  (8U)
#define TMR_CSCTRL3_OFLAG(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_OFLAG_SHIFT)) & TMR_CSCTRL3_OFLAG_MASK)
/*! UP - Counting Direction Indicator
 *  0b0..The last count was in the DOWN direction.
 *  0b1..The last count was in the UP direction.
 */
#define TMR_CSCTRL3_UP_MASK                      (0x200U)
#define TMR_CSCTRL3_UP_SHIFT                     (9U)
#define TMR_CSCTRL3_UP(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_UP_SHIFT)) & TMR_CSCTRL3_UP_MASK)
/*! TCI - Triggered Count Initialization Control
 *  0b0..Stop counter upon receiving a second trigger event while still counting from the first trigger event.
 *  0b1..Reload the counter upon receiving a second trigger event while still counting from the first trigger event.
 */
#define TMR_CSCTRL3_TCI_MASK                     (0x400U)
#define TMR_CSCTRL3_TCI_SHIFT                    (10U)
#define TMR_CSCTRL3_TCI(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_TCI_SHIFT)) & TMR_CSCTRL3_TCI_MASK)
/*! ROC - Reload on Capture
 *  0b0..Do not reload the counter on a capture event.
 *  0b1..Reload the counter on a capture event.
 */
#define TMR_CSCTRL3_ROC_MASK                     (0x800U)
#define TMR_CSCTRL3_ROC_SHIFT                    (11U)
#define TMR_CSCTRL3_ROC(x)                       (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_ROC_SHIFT)) & TMR_CSCTRL3_ROC_MASK)
/*! ALT_LOAD - Alternative Load Enable
 *  0b0..Counter can be re-initialized only with the LOAD register.
 *  0b1..Counter can be re-initialized with the LOAD or CMPLD2 registers depending on count direction.
 */
#define TMR_CSCTRL3_ALT_LOAD_MASK                (0x1000U)
#define TMR_CSCTRL3_ALT_LOAD_SHIFT               (12U)
#define TMR_CSCTRL3_ALT_LOAD(x)                  (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_ALT_LOAD_SHIFT)) & TMR_CSCTRL3_ALT_LOAD_MASK)
/*! FAULT - Fault Enable
 *  0b0..Fault function disabled.
 *  0b1..Fault function enabled.
 */
#define TMR_CSCTRL3_FAULT_MASK                   (0x2000U)
#define TMR_CSCTRL3_FAULT_SHIFT                  (13U)
#define TMR_CSCTRL3_FAULT(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_FAULT_SHIFT)) & TMR_CSCTRL3_FAULT_MASK)
/*! DBG_EN - Debug Actions Enable
 *  0b00..Continue with normal operation during debug mode. (default)
 *  0b01..Halt TMR counter during debug mode.
 *  0b10..Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]).
 *  0b11..Both halt counter and force output to 0 during debug mode.
 */
#define TMR_CSCTRL3_DBG_EN_MASK                  (0xC000U)
#define TMR_CSCTRL3_DBG_EN_SHIFT                 (14U)
#define TMR_CSCTRL3_DBG_EN(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_CSCTRL3_DBG_EN_SHIFT)) & TMR_CSCTRL3_DBG_EN_MASK)
/*! @} */

/*! @name FILT3 - Timer Channel Input Filter Register */
/*! @{ */
#define TMR_FILT3_FILT_PER_MASK                  (0xFFU)
#define TMR_FILT3_FILT_PER_SHIFT                 (0U)
#define TMR_FILT3_FILT_PER(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT3_FILT_PER_SHIFT)) & TMR_FILT3_FILT_PER_MASK)
#define TMR_FILT3_FILT_CNT_MASK                  (0x700U)
#define TMR_FILT3_FILT_CNT_SHIFT                 (8U)
#define TMR_FILT3_FILT_CNT(x)                    (((uint16_t)(((uint16_t)(x)) << TMR_FILT3_FILT_CNT_SHIFT)) & TMR_FILT3_FILT_CNT_MASK)
/*! @} */

/*! @name DMA3 - Timer Channel DMA Enable Register */
/*! @{ */
#define TMR_DMA3_IEFDE_MASK                      (0x1U)
#define TMR_DMA3_IEFDE_SHIFT                     (0U)
#define TMR_DMA3_IEFDE(x)                        (((uint16_t)(((uint16_t)(x)) << TMR_DMA3_IEFDE_SHIFT)) & TMR_DMA3_IEFDE_MASK)
#define TMR_DMA3_CMPLD1DE_MASK                   (0x2U)
#define TMR_DMA3_CMPLD1DE_SHIFT                  (1U)
#define TMR_DMA3_CMPLD1DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA3_CMPLD1DE_SHIFT)) & TMR_DMA3_CMPLD1DE_MASK)
#define TMR_DMA3_CMPLD2DE_MASK                   (0x4U)
#define TMR_DMA3_CMPLD2DE_SHIFT                  (2U)
#define TMR_DMA3_CMPLD2DE(x)                     (((uint16_t)(((uint16_t)(x)) << TMR_DMA3_CMPLD2DE_SHIFT)) & TMR_DMA3_CMPLD2DE_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group TMR_Register_Masks */


/* TMR - Peripheral instance base addresses */
/** Peripheral TMRA base address */
#define TMRA_BASE                                (0xE140u)
/** Peripheral TMRA base pointer */
#define TMRA                                     ((TMR_Type *)TMRA_BASE)
/** Peripheral TMRB base address */
#define TMRB_BASE                                (0xE180u)
/** Peripheral TMRB base pointer */
#define TMRB                                     ((TMR_Type *)TMRB_BASE)
/** Array initializer of TMR peripheral base addresses */
#define TMR_BASE_ADDRS                           { TMRA_BASE, TMRB_BASE }
/** Array initializer of TMR peripheral base pointers */
#define TMR_BASE_PTRS                            { TMRA, TMRB }

/*!
 * @}
 */ /* end of group TMR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- XBARA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Peripheral_Access_Layer XBARA Peripheral Access Layer
 * @{
 */

/** XBARA - Register Layout Typedef */
typedef struct {
  __IO uint16_t SEL0;                              /**< Crossbar A Select Register 0, offset: 0x0 */
  __IO uint16_t SEL1;                              /**< Crossbar A Select Register 1, offset: 0x1 */
  __IO uint16_t SEL2;                              /**< Crossbar A Select Register 2, offset: 0x2 */
  __IO uint16_t SEL3;                              /**< Crossbar A Select Register 3, offset: 0x3 */
  __IO uint16_t SEL4;                              /**< Crossbar A Select Register 4, offset: 0x4 */
  __IO uint16_t SEL5;                              /**< Crossbar A Select Register 5, offset: 0x5 */
  __IO uint16_t SEL6;                              /**< Crossbar A Select Register 6, offset: 0x6 */
  __IO uint16_t SEL7;                              /**< Crossbar A Select Register 7, offset: 0x7 */
  __IO uint16_t SEL8;                              /**< Crossbar A Select Register 8, offset: 0x8 */
  __IO uint16_t SEL9;                              /**< Crossbar A Select Register 9, offset: 0x9 */
  __IO uint16_t SEL10;                             /**< Crossbar A Select Register 10, offset: 0xA */
  __IO uint16_t SEL11;                             /**< Crossbar A Select Register 11, offset: 0xB */
  __IO uint16_t SEL12;                             /**< Crossbar A Select Register 12, offset: 0xC */
  __IO uint16_t SEL13;                             /**< Crossbar A Select Register 13, offset: 0xD */
  __IO uint16_t SEL14;                             /**< Crossbar A Select Register 14, offset: 0xE */
  __IO uint16_t SEL15;                             /**< Crossbar A Select Register 15, offset: 0xF */
  __IO uint16_t SEL16;                             /**< Crossbar A Select Register 16, offset: 0x10 */
  __IO uint16_t SEL17;                             /**< Crossbar A Select Register 17, offset: 0x11 */
  __IO uint16_t SEL18;                             /**< Crossbar A Select Register 18, offset: 0x12 */
  __IO uint16_t SEL19;                             /**< Crossbar A Select Register 19, offset: 0x13 */
  __IO uint16_t SEL20;                             /**< Crossbar A Select Register 20, offset: 0x14 */
  __IO uint16_t SEL21;                             /**< Crossbar A Select Register 21, offset: 0x15 */
  __IO uint16_t SEL22;                             /**< Crossbar A Select Register 22, offset: 0x16 */
  __IO uint16_t SEL23;                             /**< Crossbar A Select Register 23, offset: 0x17 */
  __IO uint16_t SEL24;                             /**< Crossbar A Select Register 24, offset: 0x18 */
  __IO uint16_t SEL25;                             /**< Crossbar A Select Register 25, offset: 0x19 */
  __IO uint16_t SEL26;                             /**< Crossbar A Select Register 26, offset: 0x1A */
  __IO uint16_t SEL27;                             /**< Crossbar A Select Register 27, offset: 0x1B */
  __IO uint16_t SEL28;                             /**< Crossbar A Select Register 28, offset: 0x1C */
  __IO uint16_t SEL29;                             /**< Crossbar A Select Register 29, offset: 0x1D */
  __IO uint16_t SEL30;                             /**< Crossbar A Select Register 30, offset: 0x1E */
  __IO uint16_t SEL31;                             /**< Crossbar A Select Register 31, offset: 0x1F */
  __IO uint16_t CTRL0;                             /**< Crossbar A Control Register 0, offset: 0x20 */
  __IO uint16_t CTRL1;                             /**< Crossbar A Control Register 1, offset: 0x21 */
} XBARA_Type;

/* ----------------------------------------------------------------------------
   -- XBARA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup XBARA_Register_Masks XBARA Register Masks
 * @{
 */

/*! @name SEL0 - Crossbar A Select Register 0 */
/*! @{ */
#define XBARA_SEL0_SEL0_MASK                     (0x3FU)
#define XBARA_SEL0_SEL0_SHIFT                    (0U)
#define XBARA_SEL0_SEL0(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL0_SEL0_SHIFT)) & XBARA_SEL0_SEL0_MASK)
#define XBARA_SEL0_SEL1_MASK                     (0x3F00U)
#define XBARA_SEL0_SEL1_SHIFT                    (8U)
#define XBARA_SEL0_SEL1(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL0_SEL1_SHIFT)) & XBARA_SEL0_SEL1_MASK)
/*! @} */

/*! @name SEL1 - Crossbar A Select Register 1 */
/*! @{ */
#define XBARA_SEL1_SEL2_MASK                     (0x3FU)
#define XBARA_SEL1_SEL2_SHIFT                    (0U)
#define XBARA_SEL1_SEL2(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL1_SEL2_SHIFT)) & XBARA_SEL1_SEL2_MASK)
#define XBARA_SEL1_SEL3_MASK                     (0x3F00U)
#define XBARA_SEL1_SEL3_SHIFT                    (8U)
#define XBARA_SEL1_SEL3(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL1_SEL3_SHIFT)) & XBARA_SEL1_SEL3_MASK)
/*! @} */

/*! @name SEL2 - Crossbar A Select Register 2 */
/*! @{ */
#define XBARA_SEL2_SEL4_MASK                     (0x3FU)
#define XBARA_SEL2_SEL4_SHIFT                    (0U)
#define XBARA_SEL2_SEL4(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL2_SEL4_SHIFT)) & XBARA_SEL2_SEL4_MASK)
#define XBARA_SEL2_SEL5_MASK                     (0x3F00U)
#define XBARA_SEL2_SEL5_SHIFT                    (8U)
#define XBARA_SEL2_SEL5(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL2_SEL5_SHIFT)) & XBARA_SEL2_SEL5_MASK)
/*! @} */

/*! @name SEL3 - Crossbar A Select Register 3 */
/*! @{ */
#define XBARA_SEL3_SEL6_MASK                     (0x3FU)
#define XBARA_SEL3_SEL6_SHIFT                    (0U)
#define XBARA_SEL3_SEL6(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL3_SEL6_SHIFT)) & XBARA_SEL3_SEL6_MASK)
#define XBARA_SEL3_SEL7_MASK                     (0x3F00U)
#define XBARA_SEL3_SEL7_SHIFT                    (8U)
#define XBARA_SEL3_SEL7(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL3_SEL7_SHIFT)) & XBARA_SEL3_SEL7_MASK)
/*! @} */

/*! @name SEL4 - Crossbar A Select Register 4 */
/*! @{ */
#define XBARA_SEL4_SEL8_MASK                     (0x3FU)
#define XBARA_SEL4_SEL8_SHIFT                    (0U)
#define XBARA_SEL4_SEL8(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL4_SEL8_SHIFT)) & XBARA_SEL4_SEL8_MASK)
#define XBARA_SEL4_SEL9_MASK                     (0x3F00U)
#define XBARA_SEL4_SEL9_SHIFT                    (8U)
#define XBARA_SEL4_SEL9(x)                       (((uint16_t)(((uint16_t)(x)) << XBARA_SEL4_SEL9_SHIFT)) & XBARA_SEL4_SEL9_MASK)
/*! @} */

/*! @name SEL5 - Crossbar A Select Register 5 */
/*! @{ */
#define XBARA_SEL5_SEL10_MASK                    (0x3FU)
#define XBARA_SEL5_SEL10_SHIFT                   (0U)
#define XBARA_SEL5_SEL10(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL5_SEL10_SHIFT)) & XBARA_SEL5_SEL10_MASK)
#define XBARA_SEL5_SEL11_MASK                    (0x3F00U)
#define XBARA_SEL5_SEL11_SHIFT                   (8U)
#define XBARA_SEL5_SEL11(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL5_SEL11_SHIFT)) & XBARA_SEL5_SEL11_MASK)
/*! @} */

/*! @name SEL6 - Crossbar A Select Register 6 */
/*! @{ */
#define XBARA_SEL6_SEL12_MASK                    (0x3FU)
#define XBARA_SEL6_SEL12_SHIFT                   (0U)
#define XBARA_SEL6_SEL12(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL6_SEL12_SHIFT)) & XBARA_SEL6_SEL12_MASK)
#define XBARA_SEL6_SEL13_MASK                    (0x3F00U)
#define XBARA_SEL6_SEL13_SHIFT                   (8U)
#define XBARA_SEL6_SEL13(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL6_SEL13_SHIFT)) & XBARA_SEL6_SEL13_MASK)
/*! @} */

/*! @name SEL7 - Crossbar A Select Register 7 */
/*! @{ */
#define XBARA_SEL7_SEL14_MASK                    (0x3FU)
#define XBARA_SEL7_SEL14_SHIFT                   (0U)
#define XBARA_SEL7_SEL14(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL7_SEL14_SHIFT)) & XBARA_SEL7_SEL14_MASK)
#define XBARA_SEL7_SEL15_MASK                    (0x3F00U)
#define XBARA_SEL7_SEL15_SHIFT                   (8U)
#define XBARA_SEL7_SEL15(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL7_SEL15_SHIFT)) & XBARA_SEL7_SEL15_MASK)
/*! @} */

/*! @name SEL8 - Crossbar A Select Register 8 */
/*! @{ */
#define XBARA_SEL8_SEL16_MASK                    (0x3FU)
#define XBARA_SEL8_SEL16_SHIFT                   (0U)
#define XBARA_SEL8_SEL16(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL8_SEL16_SHIFT)) & XBARA_SEL8_SEL16_MASK)
#define XBARA_SEL8_SEL17_MASK                    (0x3F00U)
#define XBARA_SEL8_SEL17_SHIFT                   (8U)
#define XBARA_SEL8_SEL17(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL8_SEL17_SHIFT)) & XBARA_SEL8_SEL17_MASK)
/*! @} */

/*! @name SEL9 - Crossbar A Select Register 9 */
/*! @{ */
#define XBARA_SEL9_SEL18_MASK                    (0x3FU)
#define XBARA_SEL9_SEL18_SHIFT                   (0U)
#define XBARA_SEL9_SEL18(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL9_SEL18_SHIFT)) & XBARA_SEL9_SEL18_MASK)
#define XBARA_SEL9_SEL19_MASK                    (0x3F00U)
#define XBARA_SEL9_SEL19_SHIFT                   (8U)
#define XBARA_SEL9_SEL19(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_SEL9_SEL19_SHIFT)) & XBARA_SEL9_SEL19_MASK)
/*! @} */

/*! @name SEL10 - Crossbar A Select Register 10 */
/*! @{ */
#define XBARA_SEL10_SEL20_MASK                   (0x3FU)
#define XBARA_SEL10_SEL20_SHIFT                  (0U)
#define XBARA_SEL10_SEL20(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL10_SEL20_SHIFT)) & XBARA_SEL10_SEL20_MASK)
#define XBARA_SEL10_SEL21_MASK                   (0x3F00U)
#define XBARA_SEL10_SEL21_SHIFT                  (8U)
#define XBARA_SEL10_SEL21(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL10_SEL21_SHIFT)) & XBARA_SEL10_SEL21_MASK)
/*! @} */

/*! @name SEL11 - Crossbar A Select Register 11 */
/*! @{ */
#define XBARA_SEL11_SEL22_MASK                   (0x3FU)
#define XBARA_SEL11_SEL22_SHIFT                  (0U)
#define XBARA_SEL11_SEL22(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL11_SEL22_SHIFT)) & XBARA_SEL11_SEL22_MASK)
#define XBARA_SEL11_SEL23_MASK                   (0x3F00U)
#define XBARA_SEL11_SEL23_SHIFT                  (8U)
#define XBARA_SEL11_SEL23(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL11_SEL23_SHIFT)) & XBARA_SEL11_SEL23_MASK)
/*! @} */

/*! @name SEL12 - Crossbar A Select Register 12 */
/*! @{ */
#define XBARA_SEL12_SEL24_MASK                   (0x3FU)
#define XBARA_SEL12_SEL24_SHIFT                  (0U)
#define XBARA_SEL12_SEL24(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL12_SEL24_SHIFT)) & XBARA_SEL12_SEL24_MASK)
#define XBARA_SEL12_SEL25_MASK                   (0x3F00U)
#define XBARA_SEL12_SEL25_SHIFT                  (8U)
#define XBARA_SEL12_SEL25(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL12_SEL25_SHIFT)) & XBARA_SEL12_SEL25_MASK)
/*! @} */

/*! @name SEL13 - Crossbar A Select Register 13 */
/*! @{ */
#define XBARA_SEL13_SEL26_MASK                   (0x3FU)
#define XBARA_SEL13_SEL26_SHIFT                  (0U)
#define XBARA_SEL13_SEL26(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL13_SEL26_SHIFT)) & XBARA_SEL13_SEL26_MASK)
#define XBARA_SEL13_SEL27_MASK                   (0x3F00U)
#define XBARA_SEL13_SEL27_SHIFT                  (8U)
#define XBARA_SEL13_SEL27(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL13_SEL27_SHIFT)) & XBARA_SEL13_SEL27_MASK)
/*! @} */

/*! @name SEL14 - Crossbar A Select Register 14 */
/*! @{ */
#define XBARA_SEL14_SEL28_MASK                   (0x3FU)
#define XBARA_SEL14_SEL28_SHIFT                  (0U)
#define XBARA_SEL14_SEL28(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL14_SEL28_SHIFT)) & XBARA_SEL14_SEL28_MASK)
#define XBARA_SEL14_SEL29_MASK                   (0x3F00U)
#define XBARA_SEL14_SEL29_SHIFT                  (8U)
#define XBARA_SEL14_SEL29(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL14_SEL29_SHIFT)) & XBARA_SEL14_SEL29_MASK)
/*! @} */

/*! @name SEL15 - Crossbar A Select Register 15 */
/*! @{ */
#define XBARA_SEL15_SEL30_MASK                   (0x3FU)
#define XBARA_SEL15_SEL30_SHIFT                  (0U)
#define XBARA_SEL15_SEL30(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL15_SEL30_SHIFT)) & XBARA_SEL15_SEL30_MASK)
#define XBARA_SEL15_SEL31_MASK                   (0x3F00U)
#define XBARA_SEL15_SEL31_SHIFT                  (8U)
#define XBARA_SEL15_SEL31(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL15_SEL31_SHIFT)) & XBARA_SEL15_SEL31_MASK)
/*! @} */

/*! @name SEL16 - Crossbar A Select Register 16 */
/*! @{ */
#define XBARA_SEL16_SEL32_MASK                   (0x3FU)
#define XBARA_SEL16_SEL32_SHIFT                  (0U)
#define XBARA_SEL16_SEL32(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL16_SEL32_SHIFT)) & XBARA_SEL16_SEL32_MASK)
#define XBARA_SEL16_SEL33_MASK                   (0x3F00U)
#define XBARA_SEL16_SEL33_SHIFT                  (8U)
#define XBARA_SEL16_SEL33(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL16_SEL33_SHIFT)) & XBARA_SEL16_SEL33_MASK)
/*! @} */

/*! @name SEL17 - Crossbar A Select Register 17 */
/*! @{ */
#define XBARA_SEL17_SEL34_MASK                   (0x3FU)
#define XBARA_SEL17_SEL34_SHIFT                  (0U)
#define XBARA_SEL17_SEL34(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL17_SEL34_SHIFT)) & XBARA_SEL17_SEL34_MASK)
#define XBARA_SEL17_SEL35_MASK                   (0x3F00U)
#define XBARA_SEL17_SEL35_SHIFT                  (8U)
#define XBARA_SEL17_SEL35(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL17_SEL35_SHIFT)) & XBARA_SEL17_SEL35_MASK)
/*! @} */

/*! @name SEL18 - Crossbar A Select Register 18 */
/*! @{ */
#define XBARA_SEL18_SEL36_MASK                   (0x3FU)
#define XBARA_SEL18_SEL36_SHIFT                  (0U)
#define XBARA_SEL18_SEL36(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL18_SEL36_SHIFT)) & XBARA_SEL18_SEL36_MASK)
#define XBARA_SEL18_SEL37_MASK                   (0x3F00U)
#define XBARA_SEL18_SEL37_SHIFT                  (8U)
#define XBARA_SEL18_SEL37(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL18_SEL37_SHIFT)) & XBARA_SEL18_SEL37_MASK)
/*! @} */

/*! @name SEL19 - Crossbar A Select Register 19 */
/*! @{ */
#define XBARA_SEL19_SEL38_MASK                   (0x3FU)
#define XBARA_SEL19_SEL38_SHIFT                  (0U)
#define XBARA_SEL19_SEL38(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL19_SEL38_SHIFT)) & XBARA_SEL19_SEL38_MASK)
#define XBARA_SEL19_SEL39_MASK                   (0x3F00U)
#define XBARA_SEL19_SEL39_SHIFT                  (8U)
#define XBARA_SEL19_SEL39(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL19_SEL39_SHIFT)) & XBARA_SEL19_SEL39_MASK)
/*! @} */

/*! @name SEL20 - Crossbar A Select Register 20 */
/*! @{ */
#define XBARA_SEL20_SEL40_MASK                   (0x3FU)
#define XBARA_SEL20_SEL40_SHIFT                  (0U)
#define XBARA_SEL20_SEL40(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL20_SEL40_SHIFT)) & XBARA_SEL20_SEL40_MASK)
#define XBARA_SEL20_SEL41_MASK                   (0x3F00U)
#define XBARA_SEL20_SEL41_SHIFT                  (8U)
#define XBARA_SEL20_SEL41(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL20_SEL41_SHIFT)) & XBARA_SEL20_SEL41_MASK)
/*! @} */

/*! @name SEL21 - Crossbar A Select Register 21 */
/*! @{ */
#define XBARA_SEL21_SEL42_MASK                   (0x3FU)
#define XBARA_SEL21_SEL42_SHIFT                  (0U)
#define XBARA_SEL21_SEL42(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL21_SEL42_SHIFT)) & XBARA_SEL21_SEL42_MASK)
#define XBARA_SEL21_SEL43_MASK                   (0x3F00U)
#define XBARA_SEL21_SEL43_SHIFT                  (8U)
#define XBARA_SEL21_SEL43(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL21_SEL43_SHIFT)) & XBARA_SEL21_SEL43_MASK)
/*! @} */

/*! @name SEL22 - Crossbar A Select Register 22 */
/*! @{ */
#define XBARA_SEL22_SEL44_MASK                   (0x3FU)
#define XBARA_SEL22_SEL44_SHIFT                  (0U)
#define XBARA_SEL22_SEL44(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL22_SEL44_SHIFT)) & XBARA_SEL22_SEL44_MASK)
#define XBARA_SEL22_SEL45_MASK                   (0x3F00U)
#define XBARA_SEL22_SEL45_SHIFT                  (8U)
#define XBARA_SEL22_SEL45(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL22_SEL45_SHIFT)) & XBARA_SEL22_SEL45_MASK)
/*! @} */

/*! @name SEL23 - Crossbar A Select Register 23 */
/*! @{ */
#define XBARA_SEL23_SEL46_MASK                   (0x3FU)
#define XBARA_SEL23_SEL46_SHIFT                  (0U)
#define XBARA_SEL23_SEL46(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL23_SEL46_SHIFT)) & XBARA_SEL23_SEL46_MASK)
#define XBARA_SEL23_SEL47_MASK                   (0x3F00U)
#define XBARA_SEL23_SEL47_SHIFT                  (8U)
#define XBARA_SEL23_SEL47(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL23_SEL47_SHIFT)) & XBARA_SEL23_SEL47_MASK)
/*! @} */

/*! @name SEL24 - Crossbar A Select Register 24 */
/*! @{ */
#define XBARA_SEL24_SEL48_MASK                   (0x3FU)
#define XBARA_SEL24_SEL48_SHIFT                  (0U)
#define XBARA_SEL24_SEL48(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL24_SEL48_SHIFT)) & XBARA_SEL24_SEL48_MASK)
#define XBARA_SEL24_SEL49_MASK                   (0x3F00U)
#define XBARA_SEL24_SEL49_SHIFT                  (8U)
#define XBARA_SEL24_SEL49(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL24_SEL49_SHIFT)) & XBARA_SEL24_SEL49_MASK)
/*! @} */

/*! @name SEL25 - Crossbar A Select Register 25 */
/*! @{ */
#define XBARA_SEL25_SEL50_MASK                   (0x3FU)
#define XBARA_SEL25_SEL50_SHIFT                  (0U)
#define XBARA_SEL25_SEL50(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL25_SEL50_SHIFT)) & XBARA_SEL25_SEL50_MASK)
#define XBARA_SEL25_SEL51_MASK                   (0x3F00U)
#define XBARA_SEL25_SEL51_SHIFT                  (8U)
#define XBARA_SEL25_SEL51(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL25_SEL51_SHIFT)) & XBARA_SEL25_SEL51_MASK)
/*! @} */

/*! @name SEL26 - Crossbar A Select Register 26 */
/*! @{ */
#define XBARA_SEL26_SEL52_MASK                   (0x3FU)
#define XBARA_SEL26_SEL52_SHIFT                  (0U)
#define XBARA_SEL26_SEL52(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL26_SEL52_SHIFT)) & XBARA_SEL26_SEL52_MASK)
#define XBARA_SEL26_SEL53_MASK                   (0x3F00U)
#define XBARA_SEL26_SEL53_SHIFT                  (8U)
#define XBARA_SEL26_SEL53(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL26_SEL53_SHIFT)) & XBARA_SEL26_SEL53_MASK)
/*! @} */

/*! @name SEL27 - Crossbar A Select Register 27 */
/*! @{ */
#define XBARA_SEL27_SEL54_MASK                   (0x3FU)
#define XBARA_SEL27_SEL54_SHIFT                  (0U)
#define XBARA_SEL27_SEL54(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL27_SEL54_SHIFT)) & XBARA_SEL27_SEL54_MASK)
#define XBARA_SEL27_SEL55_MASK                   (0x3F00U)
#define XBARA_SEL27_SEL55_SHIFT                  (8U)
#define XBARA_SEL27_SEL55(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL27_SEL55_SHIFT)) & XBARA_SEL27_SEL55_MASK)
/*! @} */

/*! @name SEL28 - Crossbar A Select Register 28 */
/*! @{ */
#define XBARA_SEL28_SEL56_MASK                   (0x3FU)
#define XBARA_SEL28_SEL56_SHIFT                  (0U)
#define XBARA_SEL28_SEL56(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL28_SEL56_SHIFT)) & XBARA_SEL28_SEL56_MASK)
#define XBARA_SEL28_SEL57_MASK                   (0x3F00U)
#define XBARA_SEL28_SEL57_SHIFT                  (8U)
#define XBARA_SEL28_SEL57(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL28_SEL57_SHIFT)) & XBARA_SEL28_SEL57_MASK)
/*! @} */

/*! @name SEL29 - Crossbar A Select Register 29 */
/*! @{ */
#define XBARA_SEL29_SEL58_MASK                   (0x3FU)
#define XBARA_SEL29_SEL58_SHIFT                  (0U)
#define XBARA_SEL29_SEL58(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL29_SEL58_SHIFT)) & XBARA_SEL29_SEL58_MASK)
#define XBARA_SEL29_SEL59_MASK                   (0x3F00U)
#define XBARA_SEL29_SEL59_SHIFT                  (8U)
#define XBARA_SEL29_SEL59(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL29_SEL59_SHIFT)) & XBARA_SEL29_SEL59_MASK)
/*! @} */

/*! @name SEL30 - Crossbar A Select Register 30 */
/*! @{ */
#define XBARA_SEL30_SEL60_MASK                   (0x3FU)
#define XBARA_SEL30_SEL60_SHIFT                  (0U)
#define XBARA_SEL30_SEL60(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL30_SEL60_SHIFT)) & XBARA_SEL30_SEL60_MASK)
#define XBARA_SEL30_SEL61_MASK                   (0x3F00U)
#define XBARA_SEL30_SEL61_SHIFT                  (8U)
#define XBARA_SEL30_SEL61(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL30_SEL61_SHIFT)) & XBARA_SEL30_SEL61_MASK)
/*! @} */

/*! @name SEL31 - Crossbar A Select Register 31 */
/*! @{ */
#define XBARA_SEL31_SEL62_MASK                   (0x3FU)
#define XBARA_SEL31_SEL62_SHIFT                  (0U)
#define XBARA_SEL31_SEL62(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL31_SEL62_SHIFT)) & XBARA_SEL31_SEL62_MASK)
#define XBARA_SEL31_SEL63_MASK                   (0x3F00U)
#define XBARA_SEL31_SEL63_SHIFT                  (8U)
#define XBARA_SEL31_SEL63(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_SEL31_SEL63_SHIFT)) & XBARA_SEL31_SEL63_MASK)
/*! @} */

/*! @name CTRL0 - Crossbar A Control Register 0 */
/*! @{ */
/*! DEN0 - DMA Enable for XBAR_OUT0
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL0_DEN0_MASK                    (0x1U)
#define XBARA_CTRL0_DEN0_SHIFT                   (0U)
#define XBARA_CTRL0_DEN0(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_DEN0_SHIFT)) & XBARA_CTRL0_DEN0_MASK)
/*! IEN0 - Interrupt Enable for XBAR_OUT0
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL0_IEN0_MASK                    (0x2U)
#define XBARA_CTRL0_IEN0_SHIFT                   (1U)
#define XBARA_CTRL0_IEN0(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_IEN0_SHIFT)) & XBARA_CTRL0_IEN0_MASK)
/*! EDGE0 - Active edge for edge detection on XBAR_OUT0
 *  0b00..STS0 never asserts
 *  0b01..STS0 asserts on rising edges of XBAR_OUT0
 *  0b10..STS0 asserts on falling edges of XBAR_OUT0
 *  0b11..STS0 asserts on rising and falling edges of XBAR_OUT0
 */
#define XBARA_CTRL0_EDGE0_MASK                   (0xCU)
#define XBARA_CTRL0_EDGE0_SHIFT                  (2U)
#define XBARA_CTRL0_EDGE0(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_EDGE0_SHIFT)) & XBARA_CTRL0_EDGE0_MASK)
/*! STS0 - Edge detection status for XBAR_OUT0
 *  0b0..Active edge not yet detected on XBAR_OUT0
 *  0b1..Active edge detected on XBAR_OUT0
 */
#define XBARA_CTRL0_STS0_MASK                    (0x10U)
#define XBARA_CTRL0_STS0_SHIFT                   (4U)
#define XBARA_CTRL0_STS0(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_STS0_SHIFT)) & XBARA_CTRL0_STS0_MASK)
/*! DEN1 - DMA Enable for XBAR_OUT1
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL0_DEN1_MASK                    (0x100U)
#define XBARA_CTRL0_DEN1_SHIFT                   (8U)
#define XBARA_CTRL0_DEN1(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_DEN1_SHIFT)) & XBARA_CTRL0_DEN1_MASK)
/*! IEN1 - Interrupt Enable for XBAR_OUT1
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL0_IEN1_MASK                    (0x200U)
#define XBARA_CTRL0_IEN1_SHIFT                   (9U)
#define XBARA_CTRL0_IEN1(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_IEN1_SHIFT)) & XBARA_CTRL0_IEN1_MASK)
/*! EDGE1 - Active edge for edge detection on XBAR_OUT1
 *  0b00..STS1 never asserts
 *  0b01..STS1 asserts on rising edges of XBAR_OUT1
 *  0b10..STS1 asserts on falling edges of XBAR_OUT1
 *  0b11..STS1 asserts on rising and falling edges of XBAR_OUT1
 */
#define XBARA_CTRL0_EDGE1_MASK                   (0xC00U)
#define XBARA_CTRL0_EDGE1_SHIFT                  (10U)
#define XBARA_CTRL0_EDGE1(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_EDGE1_SHIFT)) & XBARA_CTRL0_EDGE1_MASK)
/*! STS1 - Edge detection status for XBAR_OUT1
 *  0b0..Active edge not yet detected on XBAR_OUT1
 *  0b1..Active edge detected on XBAR_OUT1
 */
#define XBARA_CTRL0_STS1_MASK                    (0x1000U)
#define XBARA_CTRL0_STS1_SHIFT                   (12U)
#define XBARA_CTRL0_STS1(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL0_STS1_SHIFT)) & XBARA_CTRL0_STS1_MASK)
/*! @} */

/*! @name CTRL1 - Crossbar A Control Register 1 */
/*! @{ */
/*! DEN2 - DMA Enable for XBAR_OUT2
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL1_DEN2_MASK                    (0x1U)
#define XBARA_CTRL1_DEN2_SHIFT                   (0U)
#define XBARA_CTRL1_DEN2(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_DEN2_SHIFT)) & XBARA_CTRL1_DEN2_MASK)
/*! IEN2 - Interrupt Enable for XBAR_OUT2
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL1_IEN2_MASK                    (0x2U)
#define XBARA_CTRL1_IEN2_SHIFT                   (1U)
#define XBARA_CTRL1_IEN2(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_IEN2_SHIFT)) & XBARA_CTRL1_IEN2_MASK)
/*! EDGE2 - Active edge for edge detection on XBAR_OUT2
 *  0b00..STS2 never asserts
 *  0b01..STS2 asserts on rising edges of XBAR_OUT2
 *  0b10..STS2 asserts on falling edges of XBAR_OUT2
 *  0b11..STS2 asserts on rising and falling edges of XBAR_OUT2
 */
#define XBARA_CTRL1_EDGE2_MASK                   (0xCU)
#define XBARA_CTRL1_EDGE2_SHIFT                  (2U)
#define XBARA_CTRL1_EDGE2(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_EDGE2_SHIFT)) & XBARA_CTRL1_EDGE2_MASK)
/*! STS2 - Edge detection status for XBAR_OUT2
 *  0b0..Active edge not yet detected on XBAR_OUT2
 *  0b1..Active edge detected on XBAR_OUT2
 */
#define XBARA_CTRL1_STS2_MASK                    (0x10U)
#define XBARA_CTRL1_STS2_SHIFT                   (4U)
#define XBARA_CTRL1_STS2(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_STS2_SHIFT)) & XBARA_CTRL1_STS2_MASK)
/*! DEN3 - DMA Enable for XBAR_OUT3
 *  0b0..DMA disabled
 *  0b1..DMA enabled
 */
#define XBARA_CTRL1_DEN3_MASK                    (0x100U)
#define XBARA_CTRL1_DEN3_SHIFT                   (8U)
#define XBARA_CTRL1_DEN3(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_DEN3_SHIFT)) & XBARA_CTRL1_DEN3_MASK)
/*! IEN3 - Interrupt Enable for XBAR_OUT3
 *  0b0..Interrupt disabled
 *  0b1..Interrupt enabled
 */
#define XBARA_CTRL1_IEN3_MASK                    (0x200U)
#define XBARA_CTRL1_IEN3_SHIFT                   (9U)
#define XBARA_CTRL1_IEN3(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_IEN3_SHIFT)) & XBARA_CTRL1_IEN3_MASK)
/*! EDGE3 - Active edge for edge detection on XBAR_OUT3
 *  0b00..STS3 never asserts
 *  0b01..STS3 asserts on rising edges of XBAR_OUT3
 *  0b10..STS3 asserts on falling edges of XBAR_OUT3
 *  0b11..STS3 asserts on rising and falling edges of XBAR_OUT3
 */
#define XBARA_CTRL1_EDGE3_MASK                   (0xC00U)
#define XBARA_CTRL1_EDGE3_SHIFT                  (10U)
#define XBARA_CTRL1_EDGE3(x)                     (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_EDGE3_SHIFT)) & XBARA_CTRL1_EDGE3_MASK)
/*! STS3 - Edge detection status for XBAR_OUT3
 *  0b0..Active edge not yet detected on XBAR_OUT3
 *  0b1..Active edge detected on XBAR_OUT3
 */
#define XBARA_CTRL1_STS3_MASK                    (0x1000U)
#define XBARA_CTRL1_STS3_SHIFT                   (12U)
#define XBARA_CTRL1_STS3(x)                      (((uint16_t)(((uint16_t)(x)) << XBARA_CTRL1_STS3_SHIFT)) & XBARA_CTRL1_STS3_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group XBARA_Register_Masks */


/* XBARA - Peripheral instance base addresses */
/** Peripheral XBARA base address */
#define XBARA_BASE                               (0xE340u)
/** Peripheral XBARA base pointer */
#define XBARA                                    ((XBARA_Type *)XBARA_BASE)
/** Array initializer of XBARA peripheral base addresses */
#define XBARA_BASE_ADDRS                         { XBARA_BASE }
/** Array initializer of XBARA peripheral base pointers */
#define XBARA_BASE_PTRS                          { XBARA }

/*!
 * @}
 */ /* end of group XBARA_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Bit_Field_Generic_Macros Macros for use with bit field definitions (xxx_SHIFT, xxx_MASK).
 * @{
 */

/**
 * @brief Mask and left-shift a bit field value for use in a register bit range.
 * @param field Name of the register bit field.
 * @param value Value of the bit field.
 * @return Masked and shifted value.
 */
#define NXP_VAL2FLD(field, value)    (((value) << (field ## _SHIFT)) & (field ## _MASK))
/**
 * @brief Mask and right-shift a register value to extract a bit field value.
 * @param field Name of the register bit field.
 * @param value Value of the register.
 * @return Masked and shifted bit field value.
 */
#define NXP_FLD2VAL(field, value)    (((value) & (field ## _MASK)) >> (field ## _SHIFT))

/*!
 * @}
 */ /* end of group Bit_Field_Generic_Macros */


/* ----------------------------------------------------------------------------
   -- SDK Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDK_Compatibility_Symbols SDK Compatibility
 * @{
 */

/* No SDK compatibility issues. */

/*!
 * @}
 */ /* end of group SDK_Compatibility_Symbols */


#endif  /* _MC56F83783_H_ */

