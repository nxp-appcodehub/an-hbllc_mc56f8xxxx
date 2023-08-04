/*
  PDD layer implementation for peripheral type XBARA
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(XBARA_PDD_H_)
#define XBARA_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error XBARA PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* XBARA */ && \
      !defined(MCU_MC56F82316) /* XBARA */ && \
      !defined(MCU_MC56F82723) /* XBARA */ && \
      !defined(MCU_MC56F82726) /* XBARA */ && \
      !defined(MCU_MC56F82728) /* XBARA */ && \
      !defined(MCU_MC56F82733) /* XBARA */ && \
      !defined(MCU_MC56F82736) /* XBARA */ && \
      !defined(MCU_MC56F82738) /* XBARA */ && \
      !defined(MCU_MC56F82743) /* XBARA */ && \
      !defined(MCU_MC56F82746) /* XBARA */ && \
      !defined(MCU_MC56F82748) /* XBARA */
  // Unsupported MCU is active
  #error XBARA PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* XBAR input constants. They are used to specify signal connected with XBAR
   output. */
#define XBARA_PDD_VSS             0U             /**< Selects VSS as input. */
#define XBARA_PDD_VDD             0x1U           /**< Selects VDD as input. */
#define XBARA_PDD_XB_IN2          0x2U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN3          0x3U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN4          0x4U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN5          0x5U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN6          0x6U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN7          0x7U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN8          0x8U           /**< Selects Package Pin as input. */
#define XBARA_PDD_XB_IN9          0x9U           /**< Selects Package Pin as input. */
#define XBARA_PDD_CMPA_OUT        0xAU           /**< Selects Comparator A Output as input. */
#define XBARA_PDD_CMPB_OUT        0xBU           /**< Selects Comparator B Output as input. */
#define XBARA_PDD_CMPC_OUT        0xCU           /**< Selects Comparator C Output as input. */
#define XBARA_PDD_CMPD_OUT        0xDU           /**< Selects Comparator D Output as input. */
#define XBARA_PDD_TA0_OUT         0xEU           /**< Selects Timer A0 OUT as input. */
#define XBARA_PDD_TA1_OUT         0xFU           /**< Selects Timer A1 OUT as input. */
#define XBARA_PDD_TA2_OUT         0x10U          /**< Selects Timer A2 OUT as input. */
#define XBARA_PDD_TA3_OUT         0x11U          /**< Selects Timer A3 OUT as input. */
#define XBARA_PDD_PWM0_TRG0       0x12U          /**< Selects PWM0 Trigger 0 as input. */
#define XBARA_PDD_PWM0_TRG1       0x13U          /**< Selects PWM0 Trigger 1 as input. */
#define XBARA_PDD_PWM1_TRG0       0x14U          /**< Selects PWM1 Trigger 0 as input. */
#define XBARA_PDD_PWM1_TRG1       0x15U          /**< Selects PWM1 Trigger 1 as input. */
#define XBARA_PDD_PWM2_TRG0       0x16U          /**< Selects PWM2 Trigger 0 as input. */
#define XBARA_PDD_PWM2_TRG1       0x17U          /**< Selects PWM2 Trigger 1 as input. */
#define XBARA_PDD_PWM3_TRG0       0x18U          /**< Selects PWM3 Trigger 0 as input. */
#define XBARA_PDD_PWM3_TRG1       0x19U          /**< Selects PWM3 Trigger 1 as input. */
#define XBARA_PDD_PIT0_SYNC_OUT   0x1AU          /**< Selects PIT0 SYNC as input. */
#define XBARA_PDD_PIT1_SYNC_OUT   0x1BU          /**< Selects PIT1 SYNC as input. */
#define XBARA_PDD_AND_OR_INVERT_0 0x1CU          /**< Selects AOI Output 0 as input. */
#define XBARA_PDD_AND_OR_INVERT_1 0x1DU          /**< Selects AOI Output 1 as input. */
#define XBARA_PDD_AND_OR_INVERT_2 0x1EU          /**< Selects AOI Output 2 as input. */
#define XBARA_PDD_AND_OR_INVERT_3 0x1FU          /**< Selects AOI Output 3 as input. */

/* Request destination constants. */
#define XBARA_PDD_REQ_DMA       0U               /**< The DMA_REQ output is selected. */
#define XBARA_PDD_REQ_INTERRUPT 0x1U             /**< The INT_REQ output is selected. */
#define XBARA_PDD_REQ_DISABLE   0x2U             /**< Request outputs are disabled. */

/* This constants specifies types of edges which causes request destination
   assert. */
#define XBARA_PDD_RISING_EDGE  0x1U              /**< Asserts on rising edges. */
#define XBARA_PDD_FALLING_EDGE 0x2U              /**< Asserts on falling edges. */
#define XBARA_PDD_BOTH_EDGES   0x3U              /**< Asserts on rising and falling edges. */
#define XBARA_PDD_NEITHER_EDGE 0U                /**< Never asserts. */


/* ----------------------------------------------------------------------------
   -- SetDMA_REQ0
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 0 (i.e. XBAR DMA Request
 * 0).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ0(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ0(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL0_REG(PeripheralBase), \
     XBARA_SEL0_SEL0_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ1
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 1 (i.e. XBAR DMA Request
 * 1).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ1(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ1(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL0_REG(PeripheralBase), \
     XBARA_SEL0_SEL1_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL0_SEL1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ2
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 2 (i.e. XBAR DMA Request
 * 2).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ2(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ2(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL1_REG(PeripheralBase), \
     XBARA_SEL1_SEL2_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ3
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 3 (i.e. XBAR DMA Request
 * 3).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ3(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ3(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL1_REG(PeripheralBase), \
     XBARA_SEL1_SEL3_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL1_SEL3_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT4
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 4 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT4(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT4(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL2_REG(PeripheralBase), \
     XBARA_SEL2_SEL4_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT5
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 5 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT5(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT5(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL2_REG(PeripheralBase), \
     XBARA_SEL2_SEL5_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL2_SEL5_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT6
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 6 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT6(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT6(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL3_REG(PeripheralBase), \
     XBARA_SEL3_SEL6_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT7
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 7 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT7(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT7(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL3_REG(PeripheralBase), \
     XBARA_SEL3_SEL7_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL3_SEL7_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT8
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 8 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT8(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT8(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL4_REG(PeripheralBase), \
     XBARA_SEL4_SEL8_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT9
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 9 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT9(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT9(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL4_REG(PeripheralBase), \
     XBARA_SEL4_SEL9_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL4_SEL9_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT10
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 10 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT10(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT10(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL5_REG(PeripheralBase), \
     XBARA_SEL5_SEL10_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetXB_OUT11
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 11 (i.e. Package Pin).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetXB_OUT11(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetXB_OUT11(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL5_REG(PeripheralBase), \
     XBARA_SEL5_SEL11_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL5_SEL11_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetADCA_TRIG
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 12 (i.e. ADCA (Cyclic
 * ADC) Trigger).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetADCA_TRIG(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetADCA_TRIG(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL6_REG(PeripheralBase), \
     XBARA_SEL6_SEL12_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetADCB_TRIG
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 13 (i.e. ADCB (Cyclic
 * ADC) Trigger).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetADCB_TRIG(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetADCB_TRIG(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL6_REG(PeripheralBase), \
     XBARA_SEL6_SEL13_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL6_SEL13_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDACB_12B_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 14 (i.e. 12bit DACB
 * Trigger).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDACB_12B_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDACB_12B_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL7_REG(PeripheralBase), \
     XBARA_SEL7_SEL14_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDACA_12B_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 15 (i.e. 12bit DACA
 * Trigger).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDACA_12B_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetDACA_12B_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL7_REG(PeripheralBase), \
     XBARA_SEL7_SEL15_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL7_SEL15_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCMPA
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 16 (i.e. Comparator A
 * Window/Sample).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetCMPA(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetCMPA(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL8_REG(PeripheralBase), \
     XBARA_SEL8_SEL16_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCMPB
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 17 (i.e. Comparator B
 * Window/Sample).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetCMPB(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetCMPB(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL8_REG(PeripheralBase), \
     XBARA_SEL8_SEL17_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL8_SEL17_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCMPC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 18 (i.e. Comparator C
 * Window/Sample).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetCMPC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetCMPC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL9_REG(PeripheralBase), \
     XBARA_SEL9_SEL18_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCMPD
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 19 (i.e. Comparator D
 * Window/Sample).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetCMPD(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetCMPD(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL9_REG(PeripheralBase), \
     XBARA_SEL9_SEL19_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL9_SEL19_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM0_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 20 (i.e. PWM0 EXTA).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM0_EXTA(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM0_EXTA(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL10_REG(PeripheralBase), \
     XBARA_SEL10_SEL20_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM1_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 21 (i.e. PWM1 EXTA).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM1_EXTA(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM1_EXTA(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL10_REG(PeripheralBase), \
     XBARA_SEL10_SEL21_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL10_SEL21_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM2_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 22 (i.e. PWM2 EXTA).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM2_EXTA(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM2_EXTA(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL11_REG(PeripheralBase), \
     XBARA_SEL11_SEL22_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM3_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 23 (i.e. PWM3 EXTA).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM3_EXTA(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM3_EXTA(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL11_REG(PeripheralBase), \
     XBARA_SEL11_SEL23_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL11_SEL23_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM0_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 24 (i.e. PWM0 Ext Synch).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM0_EXT_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM0_EXT_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL12_REG(PeripheralBase), \
     XBARA_SEL12_SEL24_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM1_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 25 (i.e. PWM1 Ext Synch).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM1_EXT_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM1_EXT_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL12_REG(PeripheralBase), \
     XBARA_SEL12_SEL25_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL12_SEL25_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM2_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 26 (i.e. PWM2 Ext Synch).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM2_EXT_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM2_EXT_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL13_REG(PeripheralBase), \
     XBARA_SEL13_SEL26_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM3_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 27 (i.e. PWM3 Ext Synch).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM3_EXT_SYNC(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM3_EXT_SYNC(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL13_REG(PeripheralBase), \
     XBARA_SEL13_SEL27_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL13_SEL27_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_EXT_CLK
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 28 (i.e. PWM Ext Clock).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_EXT_CLK(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_EXT_CLK(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL14_REG(PeripheralBase), \
     XBARA_SEL14_SEL28_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_FAULT0
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 29 (i.e. PWM Fault0).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_FAULT0(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_FAULT0(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL14_REG(PeripheralBase), \
     XBARA_SEL14_SEL29_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL14_SEL29_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_FAULT1
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 30 (i.e. PWM Fault1).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_FAULT1(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_FAULT1(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL15_REG(PeripheralBase), \
     XBARA_SEL15_SEL30_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_FAULT2
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 31 (i.e. PWM Fault2).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_FAULT2(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_FAULT2(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL15_REG(PeripheralBase), \
     XBARA_SEL15_SEL31_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL15_SEL31_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_FAULT3
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 32 (i.e. PWMAFault3).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_FAULT3(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_FAULT3(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL16_REG(PeripheralBase), \
     XBARA_SEL16_SEL32_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPWM_FORCE
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 33 (i.e. PWM Force).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetPWM_FORCE(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetPWM_FORCE(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL16_REG(PeripheralBase), \
     XBARA_SEL16_SEL33_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL16_SEL33_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTA0_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 34 (i.e. Timer A0).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetTA0_IN(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetTA0_IN(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL17_REG(PeripheralBase), \
     XBARA_SEL17_SEL34_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTA1_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 35 (i.e. TimerA1).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetTA1_IN(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetTA1_IN(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL17_REG(PeripheralBase), \
     XBARA_SEL17_SEL35_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL17_SEL35_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTA2_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 36 (i.e. Timer A2).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetTA2_IN(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetTA2_IN(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL18_REG(PeripheralBase), \
     XBARA_SEL18_SEL36_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTA3_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 37 (i.e. Timer A3).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetTA3_IN(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetTA3_IN(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL18_REG(PeripheralBase), \
     XBARA_SEL18_SEL37_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL18_SEL37_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCI0_RXD
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 38 (i.e. SCI0 RXD).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetSCI0_RXD(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetSCI0_RXD(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL19_REG(PeripheralBase), \
     XBARA_SEL19_SEL38_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCI1_RXD
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 39 (i.e. SCI1 RXD).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetSCI1_RXD(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetSCI1_RXD(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL19_REG(PeripheralBase), \
     XBARA_SEL19_SEL39_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARA_SEL19_SEL39_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEWM_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 40 (i.e. External
 * Watchdog Monitor).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL20.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetEWM_IN(deviceID, XBARA_PDD_VSS);
 *      @endcode
 */
#define XBARA_PDD_SetEWM_IN(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARA_SEL20_REG(PeripheralBase), \
     XBARA_SEL20_SEL40_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of XBAR DMA Request 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ0(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ0(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL0_REG(PeripheralBase), XBARA_SEL0_SEL0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of XBAR DMA Request 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ1(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ1(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL0_REG(PeripheralBase), XBARA_SEL0_SEL1_MASK)) >> ( \
     XBARA_SEL0_SEL1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of XBAR DMA Request 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ2(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ2(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL1_REG(PeripheralBase), XBARA_SEL1_SEL2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of XBAR DMA Request 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ3(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ3(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL1_REG(PeripheralBase), XBARA_SEL1_SEL3_MASK)) >> ( \
     XBARA_SEL1_SEL3_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT4(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT4(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL2_REG(PeripheralBase), XBARA_SEL2_SEL4_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT5(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT5(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL2_REG(PeripheralBase), XBARA_SEL2_SEL5_MASK)) >> ( \
     XBARA_SEL2_SEL5_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT6(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT6(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL3_REG(PeripheralBase), XBARA_SEL3_SEL6_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT7
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT7(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT7(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL3_REG(PeripheralBase), XBARA_SEL3_SEL7_MASK)) >> ( \
     XBARA_SEL3_SEL7_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT8(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT8(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL4_REG(PeripheralBase), XBARA_SEL4_SEL8_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT9
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT9(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT9(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL4_REG(PeripheralBase), XBARA_SEL4_SEL9_MASK)) >> ( \
     XBARA_SEL4_SEL9_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT10
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT10(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT10(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL5_REG(PeripheralBase), XBARA_SEL5_SEL10_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetXB_OUT11
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Package Pin.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetXB_OUT11(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetXB_OUT11(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL5_REG(PeripheralBase), XBARA_SEL5_SEL11_MASK)) >> ( \
     XBARA_SEL5_SEL11_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetADCA_TRIG
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of ADCA (Cyclic ADC) Trigger.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetADCA_TRIG(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetADCA_TRIG(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL6_REG(PeripheralBase), XBARA_SEL6_SEL12_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetADCB_TRIG
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of ADCB (Cyclic ADC) Trigger.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetADCB_TRIG(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetADCB_TRIG(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL6_REG(PeripheralBase), XBARA_SEL6_SEL13_MASK)) >> ( \
     XBARA_SEL6_SEL13_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDACB_12B_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of 12bit DACB Trigger.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDACB_12B_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDACB_12B_SYNC(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL7_REG(PeripheralBase), XBARA_SEL7_SEL14_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDACA_12B_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of 12bit DACA Trigger.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDACA_12B_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDACA_12B_SYNC(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL7_REG(PeripheralBase), XBARA_SEL7_SEL15_MASK)) >> ( \
     XBARA_SEL7_SEL15_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCMPA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Comparator A
 * Window/Sample.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetCMPA(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetCMPA(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL8_REG(PeripheralBase), XBARA_SEL8_SEL16_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCMPB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Comparator B
 * Window/Sample.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetCMPB(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetCMPB(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL8_REG(PeripheralBase), XBARA_SEL8_SEL17_MASK)) >> ( \
     XBARA_SEL8_SEL17_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCMPC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Comparator C
 * Window/Sample.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetCMPC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetCMPC(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL9_REG(PeripheralBase), XBARA_SEL9_SEL18_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCMPD
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Comparator D
 * Window/Sample.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetCMPD(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetCMPD(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL9_REG(PeripheralBase), XBARA_SEL9_SEL19_MASK)) >> ( \
     XBARA_SEL9_SEL19_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM0_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM0 EXTA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM0_EXTA(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM0_EXTA(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL10_REG(PeripheralBase), XBARA_SEL10_SEL20_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM1_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM1 EXTA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM1_EXTA(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM1_EXTA(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL10_REG(PeripheralBase), XBARA_SEL10_SEL21_MASK)) >> ( \
     XBARA_SEL10_SEL21_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM2_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM2 EXTA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM2_EXTA(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM2_EXTA(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL11_REG(PeripheralBase), XBARA_SEL11_SEL22_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM3_EXTA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM3 EXTA.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM3_EXTA(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM3_EXTA(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL11_REG(PeripheralBase), XBARA_SEL11_SEL23_MASK)) >> ( \
     XBARA_SEL11_SEL23_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM0_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM0 Ext Synch.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM0_EXT_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM0_EXT_SYNC(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL12_REG(PeripheralBase), XBARA_SEL12_SEL24_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM1_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM1 Ext Synch.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM1_EXT_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM1_EXT_SYNC(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL12_REG(PeripheralBase), XBARA_SEL12_SEL25_MASK)) >> ( \
     XBARA_SEL12_SEL25_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM2_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM2 Ext Synch.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM2_EXT_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM2_EXT_SYNC(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL13_REG(PeripheralBase), XBARA_SEL13_SEL26_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM3_EXT_SYNC
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM3 Ext Synch.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM3_EXT_SYNC(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM3_EXT_SYNC(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL13_REG(PeripheralBase), XBARA_SEL13_SEL27_MASK)) >> ( \
     XBARA_SEL13_SEL27_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_EXT_CLK
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM Ext Clock.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_EXT_CLK(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_EXT_CLK(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL14_REG(PeripheralBase), XBARA_SEL14_SEL28_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_FAULT0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM Fault0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_FAULT0(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_FAULT0(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL14_REG(PeripheralBase), XBARA_SEL14_SEL29_MASK)) >> ( \
     XBARA_SEL14_SEL29_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_FAULT1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM Fault1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_FAULT1(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_FAULT1(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL15_REG(PeripheralBase), XBARA_SEL15_SEL30_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_FAULT2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM Fault2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_FAULT2(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_FAULT2(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL15_REG(PeripheralBase), XBARA_SEL15_SEL31_MASK)) >> ( \
     XBARA_SEL15_SEL31_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_FAULT3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWMAFault3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_FAULT3(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_FAULT3(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL16_REG(PeripheralBase), XBARA_SEL16_SEL32_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPWM_FORCE
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of PWM Force.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetPWM_FORCE(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetPWM_FORCE(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL16_REG(PeripheralBase), XBARA_SEL16_SEL33_MASK)) >> ( \
     XBARA_SEL16_SEL33_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTA0_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Timer A0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetTA0_IN(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetTA0_IN(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL17_REG(PeripheralBase), XBARA_SEL17_SEL34_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetTA1_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of TimerA1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetTA1_IN(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetTA1_IN(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL17_REG(PeripheralBase), XBARA_SEL17_SEL35_MASK)) >> ( \
     XBARA_SEL17_SEL35_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTA2_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Timer A2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetTA2_IN(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetTA2_IN(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL18_REG(PeripheralBase), XBARA_SEL18_SEL36_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetTA3_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of Timer A3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetTA3_IN(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetTA3_IN(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL18_REG(PeripheralBase), XBARA_SEL18_SEL37_MASK)) >> ( \
     XBARA_SEL18_SEL37_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSCI0_RXD
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of SCI0 RXD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetSCI0_RXD(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetSCI0_RXD(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL19_REG(PeripheralBase), XBARA_SEL19_SEL38_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetSCI1_RXD
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of SCI1 RXD.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetSCI1_RXD(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetSCI1_RXD(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_SEL19_REG(PeripheralBase), XBARA_SEL19_SEL39_MASK)) >> ( \
     XBARA_SEL19_SEL39_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEWM_IN
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of External Watchdog Monitor.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_SEL20.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetEWM_IN(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetEWM_IN(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARA_SEL20_REG(PeripheralBase), XBARA_SEL20_SEL40_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 0. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect0Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 1. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect1Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 2. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect2Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 3. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect3Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 4.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 4. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect4Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect4Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL4_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 5.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 5. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect5Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect5Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL5_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 6.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 6. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect6Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect6Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL6_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 7.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 7. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect7Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect7Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL7_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect8Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 8.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 8. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect8Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect8Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL8_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect9Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 9.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 9. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect9Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect9Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL9_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect10Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 10.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 10. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect10Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect10Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL10_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect11Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 11.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 11. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect11Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect11Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL11_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect12Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 12.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 12. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect12Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect12Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL12_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect13Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 13.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 13. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect13Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect13Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL13_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect14Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 14.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 14. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect14Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect14Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL14_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 15.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 15. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect15Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect15Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL15_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect16Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 16.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 16. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect16Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect16Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL16_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect17Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 17.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 17. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect17Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect17Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL17_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect18Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 18.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 18. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect18Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect18Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL18_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect19Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 19.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 19. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect19Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect19Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL19_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarSelect20Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar select register 20.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar select register 20. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_SEL20.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarSelect20Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarSelect20Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_SEL20_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect0Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect0Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect1Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect1Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL2.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect2Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect2Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL3.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect3Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect3Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL4.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect4Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect4Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL5.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect5Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect5Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL6.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect6Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect6Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL6_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 7.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL7.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect7Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect7Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL7_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect8Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 8.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL8.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect8Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect8Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL8_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect9Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 9.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL9.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect9Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect9Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL9_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect10Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 10.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL10.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect10Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect10Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL10_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect11Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 11.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL11.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect11Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect11Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL11_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect12Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 12.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL12.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect12Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect12Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL12_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect13Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 13.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL13.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect13Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect13Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL13_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect14Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 14.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL14.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect14Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect14Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL14_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 15.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL15.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect15Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect15Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL15_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect16Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 16.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL16.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect16Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect16Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL16_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect17Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 17.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL17.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect17Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect17Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL17_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect18Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 18.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL18.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect18Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect18Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL18_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect19Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 19.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL19.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect19Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect19Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL19_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect20Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 20.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_SEL20.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarSelect20Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarSelect20Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_SEL20_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ0Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the destination for the XBAR DMA Request 0 output.
 * @param PeripheralBase Peripheral base address.
 * @param Destination Name of the required destination. This parameter is of
 *        "Request destination constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ0Destination(deviceID, XBARA_PDD_REQ_DMA);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ0Destination(PeripheralBase, Destination) ( \
    ((Destination) == XBARA_PDD_REQ_DMA) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS1_MASK, \
       XBARA_CTRL0_IEN0_MASK, \
       (uint16_t)(XBARA_CTRL0_DEN0_MASK | XBARA_CTRL0_STS0_MASK) \
      )) : (((Destination) == XBARA_PDD_REQ_INTERRUPT) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS1_MASK, \
       XBARA_CTRL0_DEN0_MASK, \
       (uint16_t)(XBARA_CTRL0_IEN0_MASK | XBARA_CTRL0_STS0_MASK) \
      )) : ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS1_MASK, \
       (uint16_t)(XBARA_CTRL0_IEN0_MASK | XBARA_CTRL0_DEN0_MASK), \
       XBARA_CTRL0_STS0_MASK \
      )) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ1Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the destination for the XBAR DMA Request 1 output.
 * @param PeripheralBase Peripheral base address.
 * @param Destination Name of the required destination. This parameter is of
 *        "Request destination constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ1Destination(deviceID, XBARA_PDD_REQ_DMA);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ1Destination(PeripheralBase, Destination) ( \
    ((Destination) == XBARA_PDD_REQ_DMA) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS0_MASK, \
       XBARA_CTRL0_IEN1_MASK, \
       (uint16_t)(XBARA_CTRL0_DEN1_MASK | XBARA_CTRL0_STS1_MASK) \
      )) : (((Destination) == XBARA_PDD_REQ_INTERRUPT) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS0_MASK, \
       XBARA_CTRL0_DEN1_MASK, \
       (uint16_t)(XBARA_CTRL0_IEN1_MASK | XBARA_CTRL0_STS1_MASK) \
      )) : ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       XBARA_CTRL0_STS0_MASK, \
       (uint16_t)(XBARA_CTRL0_IEN1_MASK | XBARA_CTRL0_DEN1_MASK), \
       XBARA_CTRL0_STS1_MASK \
      )) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ2Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the destination for the XBAR DMA Request 2 output.
 * @param PeripheralBase Peripheral base address.
 * @param Destination Name of the required destination. This parameter is of
 *        "Request destination constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ2Destination(deviceID, XBARA_PDD_REQ_DMA);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ2Destination(PeripheralBase, Destination) ( \
    ((Destination) == XBARA_PDD_REQ_DMA) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS3_MASK, \
       XBARA_CTRL1_IEN2_MASK, \
       (uint16_t)(XBARA_CTRL1_DEN2_MASK | XBARA_CTRL1_STS2_MASK) \
      )) : (((Destination) == XBARA_PDD_REQ_INTERRUPT) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS3_MASK, \
       XBARA_CTRL1_DEN2_MASK, \
       (uint16_t)(XBARA_CTRL1_IEN2_MASK | XBARA_CTRL1_STS2_MASK) \
      )) : ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS3_MASK, \
       (uint16_t)(XBARA_CTRL1_IEN2_MASK | XBARA_CTRL1_DEN2_MASK), \
       XBARA_CTRL1_STS2_MASK \
      )) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ3Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the destination for the XBAR DMA Request 3 output.
 * @param PeripheralBase Peripheral base address.
 * @param Destination Name of the required destination. This parameter is of
 *        "Request destination constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ3Destination(deviceID, XBARA_PDD_REQ_DMA);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ3Destination(PeripheralBase, Destination) ( \
    ((Destination) == XBARA_PDD_REQ_DMA) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS2_MASK, \
       XBARA_CTRL1_IEN3_MASK, \
       (uint16_t)(XBARA_CTRL1_DEN3_MASK | XBARA_CTRL1_STS3_MASK) \
      )) : (((Destination) == XBARA_PDD_REQ_INTERRUPT) ? ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS2_MASK, \
       XBARA_CTRL1_DEN3_MASK, \
       (uint16_t)(XBARA_CTRL1_IEN3_MASK | XBARA_CTRL1_STS3_MASK) \
      )) : ( \
      PeriphSafeSetBits16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       XBARA_CTRL1_STS2_MASK, \
       (uint16_t)(XBARA_CTRL1_IEN3_MASK | XBARA_CTRL1_DEN3_MASK), \
       XBARA_CTRL1_STS3_MASK \
      )) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ0Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the request destination for the XBAR DMA Request 0 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Request destination constants." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ0Destination(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ0Destination(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_IEN0_MASK | XBARA_CTRL0_DEN0_MASK) \
      )) == ( \
      XBARA_CTRL0_IEN0_MASK)) ? ( \
      XBARA_PDD_REQ_INTERRUPT) : ((( \
      PeriphGetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_IEN0_MASK | XBARA_CTRL0_DEN0_MASK) \
      )) == ( \
      XBARA_CTRL0_DEN0_MASK)) ? ( \
      XBARA_PDD_REQ_DMA) : ( \
      XBARA_PDD_REQ_DISABLE) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ1Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the request destination for the XBAR DMA Request 1 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Request destination constants." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ1Destination(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ1Destination(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_IEN1_MASK | XBARA_CTRL0_DEN1_MASK) \
      )) == ( \
      XBARA_CTRL0_IEN1_MASK)) ? ( \
      XBARA_PDD_REQ_INTERRUPT) : ((( \
      PeriphGetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_IEN1_MASK | XBARA_CTRL0_DEN1_MASK) \
      )) == ( \
      XBARA_CTRL0_DEN1_MASK)) ? ( \
      XBARA_PDD_REQ_DMA) : ( \
      XBARA_PDD_REQ_DISABLE) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ2Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the request destination for the XBAR DMA Request 2 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Request destination constants." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ2Destination(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ2Destination(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_IEN2_MASK | XBARA_CTRL1_DEN2_MASK) \
      )) == ( \
      XBARA_CTRL1_IEN2_MASK)) ? ( \
      XBARA_PDD_REQ_INTERRUPT) : ((( \
      PeriphGetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_IEN2_MASK | XBARA_CTRL1_DEN2_MASK) \
      )) == ( \
      XBARA_CTRL1_DEN2_MASK)) ? ( \
      XBARA_PDD_REQ_DMA) : ( \
      XBARA_PDD_REQ_DISABLE) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ3Destination
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the request destination for the XBAR DMA Request 3 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Request destination constants." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ3Destination(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ3Destination(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_IEN3_MASK | XBARA_CTRL1_DEN3_MASK) \
      )) == ( \
      XBARA_CTRL1_IEN3_MASK)) ? ( \
      XBARA_PDD_REQ_INTERRUPT) : ((( \
      PeriphGetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_IEN3_MASK | XBARA_CTRL1_DEN3_MASK) \
      )) == ( \
      XBARA_CTRL1_DEN3_MASK)) ? ( \
      XBARA_PDD_REQ_DMA) : ( \
      XBARA_PDD_REQ_DISABLE) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterruptForDMA_REQ0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the interrupt function on the XBAR DMA Request 0 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the interrupt function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableInterruptForDMA_REQ0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableInterruptForDMA_REQ0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_IEN0_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_IEN0_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterruptForDMA_REQ1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the interrupt function on the XBAR DMA Request 1 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the interrupt function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableInterruptForDMA_REQ1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableInterruptForDMA_REQ1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_IEN1_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_IEN1_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterruptForDMA_REQ2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the interrupt function on the XBAR DMA Request 2 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the interrupt function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableInterruptForDMA_REQ2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableInterruptForDMA_REQ2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_IEN2_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_IEN2_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterruptForDMA_REQ3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the interrupt function on the XBAR DMA Request 3 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the interrupt function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableInterruptForDMA_REQ3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableInterruptForDMA_REQ3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_IEN3_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_IEN3_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptForDMA_REQ0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the interrupt function on the XBAR DMA Request 0
 * output enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetInterruptForDMA_REQ0Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetInterruptForDMA_REQ0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_IEN0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptForDMA_REQ1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the interrupt function on the XBAR DMA Request 1
 * output enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetInterruptForDMA_REQ1Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetInterruptForDMA_REQ1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_IEN1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptForDMA_REQ2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the interrupt function on the XBAR DMA Request 2
 * output enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetInterruptForDMA_REQ2Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetInterruptForDMA_REQ2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_IEN2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptForDMA_REQ3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the interrupt function on the XBAR DMA Request 3
 * output enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetInterruptForDMA_REQ3Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetInterruptForDMA_REQ3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_IEN3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaForDMA_REQ0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the DMA function on the XBAR DMA Request 0 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the DMA function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableDmaForDMA_REQ0(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableDmaForDMA_REQ0(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_DEN0_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_DEN0_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaForDMA_REQ1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the DMA function on the XBAR DMA Request 1 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the DMA function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableDmaForDMA_REQ1(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableDmaForDMA_REQ1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_DEN1_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL0_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
       XBARA_CTRL0_DEN1_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaForDMA_REQ2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the DMA function on the XBAR DMA Request 2 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the DMA function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableDmaForDMA_REQ2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableDmaForDMA_REQ2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_DEN2_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_DEN2_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaForDMA_REQ3
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the DMA function on the XBAR DMA Request 3 output.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying the DMA function will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_EnableDmaForDMA_REQ3(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define XBARA_PDD_EnableDmaForDMA_REQ3(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_DEN3_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       XBARA_CTRL1_REG(PeripheralBase), \
       (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
       XBARA_CTRL1_DEN3_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaForDMA_REQ0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the DMA function on the XBAR DMA Request 0 output
 * enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDmaForDMA_REQ0Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDmaForDMA_REQ0Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_DEN0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaForDMA_REQ1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the DMA function on the XBAR DMA Request 1 output
 * enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDmaForDMA_REQ1Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDmaForDMA_REQ1Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_DEN1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaForDMA_REQ2Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the DMA function on the XBAR DMA Request 2 output
 * enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDmaForDMA_REQ2Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDmaForDMA_REQ2Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_DEN2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaForDMA_REQ3Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the DMA function on the XBAR DMA Request 3 output
 * enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDmaForDMA_REQ3Enabled(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDmaForDMA_REQ3Enabled(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_DEN3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ0EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets active edge for edge detection on the XBAR DMA Request 0 output.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter selects which edges on the XBAR DMA Request 0 cause
 *        destination to assert. This parameter is of "This constants specifies
 *        types of edges which causes request destination assert." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ0EdgeDetection(deviceID, XBARA_PDD_RISING_EDGE);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ0EdgeDetection(PeripheralBase, Edge) ( \
    PeriphSafeSetBits16( \
     XBARA_CTRL0_REG(PeripheralBase), \
     (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
     XBARA_CTRL0_EDGE0_MASK, \
     (uint16_t)((uint16_t)(Edge) << XBARA_CTRL0_EDGE0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ1EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets active edge for edge detection on the XBAR DMA Request 1 output.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter selects which edges on the XBAR DMA Request 1 cause
 *        destination to assert. This parameter is of "This constants specifies
 *        types of edges which causes request destination assert." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ1EdgeDetection(deviceID, XBARA_PDD_RISING_EDGE);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ1EdgeDetection(PeripheralBase, Edge) ( \
    PeriphSafeSetBits16( \
     XBARA_CTRL0_REG(PeripheralBase), \
     (uint16_t)(XBARA_CTRL0_STS0_MASK | XBARA_CTRL0_STS1_MASK), \
     XBARA_CTRL0_EDGE1_MASK, \
     (uint16_t)((uint16_t)(Edge) << XBARA_CTRL0_EDGE1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ2EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets active edge for edge detection on the XBAR DMA Request 2 output.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter selects which edges on the XBAR DMA Request 2 cause
 *        destination to assert. This parameter is of "This constants specifies
 *        types of edges which causes request destination assert." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ2EdgeDetection(deviceID, XBARA_PDD_RISING_EDGE);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ2EdgeDetection(PeripheralBase, Edge) ( \
    PeriphSafeSetBits16( \
     XBARA_CTRL1_REG(PeripheralBase), \
     (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
     XBARA_CTRL1_EDGE2_MASK, \
     (uint16_t)((uint16_t)(Edge) << XBARA_CTRL1_EDGE2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDMA_REQ3EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets active edge for edge detection on the XBAR DMA Request 3 output.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter selects which edges on the XBAR DMA Request 3 cause
 *        destination to assert. This parameter is of "This constants specifies
 *        types of edges which causes request destination assert." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_SetDMA_REQ3EdgeDetection(deviceID, XBARA_PDD_RISING_EDGE);
 *      @endcode
 */
#define XBARA_PDD_SetDMA_REQ3EdgeDetection(PeripheralBase, Edge) ( \
    PeriphSafeSetBits16( \
     XBARA_CTRL1_REG(PeripheralBase), \
     (uint16_t)(XBARA_CTRL1_STS2_MASK | XBARA_CTRL1_STS3_MASK), \
     XBARA_CTRL1_EDGE3_MASK, \
     (uint16_t)((uint16_t)(Edge) << XBARA_CTRL1_EDGE3_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ0EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current active edge for edge detection on the XBAR DMA Request
 * 0 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ0EdgeDetection(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ0EdgeDetection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_EDGE0_MASK)) >> ( \
     XBARA_CTRL0_EDGE0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ1EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current active edge for edge detection on the XBAR DMA Request
 * 1 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ1EdgeDetection(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ1EdgeDetection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_EDGE1_MASK)) >> ( \
     XBARA_CTRL0_EDGE1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ2EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current active edge for edge detection on the XBAR DMA Request
 * 2 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ2EdgeDetection(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ2EdgeDetection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_EDGE2_MASK)) >> ( \
     XBARA_CTRL1_EDGE2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ3EdgeDetection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current active edge for edge detection on the XBAR DMA Request
 * 3 output.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARA_PDD_GetDMA_REQ3EdgeDetection(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ3EdgeDetection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_EDGE3_MASK)) >> ( \
     XBARA_CTRL1_EDGE3_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearDMA_REQ0InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears XBAR DMA Request 0 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_ClearDMA_REQ0InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ClearDMA_REQ0InterruptFlag(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     XBARA_CTRL0_REG(PeripheralBase), \
     XBARA_CTRL0_STS1_MASK, \
     XBARA_CTRL0_STS0_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearDMA_REQ1InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears XBAR DMA Request 1 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_ClearDMA_REQ1InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ClearDMA_REQ1InterruptFlag(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     XBARA_CTRL0_REG(PeripheralBase), \
     XBARA_CTRL0_STS0_MASK, \
     XBARA_CTRL0_STS1_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearDMA_REQ2InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears XBAR DMA Request 2 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_ClearDMA_REQ2InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ClearDMA_REQ2InterruptFlag(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     XBARA_CTRL1_REG(PeripheralBase), \
     XBARA_CTRL1_STS3_MASK, \
     XBARA_CTRL1_STS2_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearDMA_REQ3InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears XBAR DMA Request 3 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_ClearDMA_REQ3InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ClearDMA_REQ3InterruptFlag(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     XBARA_CTRL1_REG(PeripheralBase), \
     XBARA_CTRL1_STS2_MASK, \
     XBARA_CTRL1_STS3_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ0InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns XBAR DMA Request 0 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ0InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ0InterruptFlag(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_STS0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ1InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns XBAR DMA Request 1 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ1InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ1InterruptFlag(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL0_REG(PeripheralBase), XBARA_CTRL0_STS1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ2InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns XBAR DMA Request 2 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ2InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ2InterruptFlag(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_STS2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetDMA_REQ3InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns XBAR DMA Request 3 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_GetDMA_REQ3InterruptFlag(deviceID);
 *      @endcode
 */
#define XBARA_PDD_GetDMA_REQ3InterruptFlag(PeripheralBase) ( \
    PeriphGetBitMask16(XBARA_CTRL1_REG(PeripheralBase), XBARA_CTRL1_STS3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar control register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar control register 0. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarControl0Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarControl0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_CTRL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCrossbarControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes crossbar control register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into crossbar control register 1. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      XBARA_PDD_WriteCrossbarControl1Reg(deviceID, 1);
 *      @endcode
 */
#define XBARA_PDD_WriteCrossbarControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARA_CTRL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar control register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarControl0Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarControl0Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_CTRL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar control register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARA_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARA_PDD_ReadCrossbarControl1Reg(deviceID);
 *      @endcode
 */
#define XBARA_PDD_ReadCrossbarControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARA_CTRL1_REG(PeripheralBase)) \
  )
#endif  /* #if defined(XBARA_PDD_H_) */

/* XBARA_PDD.h, eof. */
