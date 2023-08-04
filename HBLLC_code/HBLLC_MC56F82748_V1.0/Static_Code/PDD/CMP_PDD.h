/*
  PDD layer implementation for peripheral type CMP
  Copyright 2010 Freescale, Inc.
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(CMP_PDD_H_)
#define CMP_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error CMP PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* CMPA, CMPB */ && \
      !defined(MCU_MC56F82316) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82723) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82726) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82728) /* CMPA, CMPB, CMPC, CMPD */ && \
      !defined(MCU_MC56F82733) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82736) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82738) /* CMPA, CMPB, CMPC, CMPD */ && \
      !defined(MCU_MC56F82743) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82746) /* CMPA, CMPB, CMPC */ && \
      !defined(MCU_MC56F82748) /* CMPA, CMPB, CMPC, CMPD */
  // Unsupported MCU is active
  #error CMP PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Interrupts' mask */
#define CMP_PDD_FALLING_EDGE_INTERRUPT CMP_SCR_IEF_MASK /**< Falling edge interrupt enable mask. */
#define CMP_PDD_RISING_EDGE_INTERRUPT  CMP_SCR_IER_MASK /**< Rising edge interrupt enable mask. */

/* Interrupts' flags */
#define CMP_PDD_FALLING_EDGE_FLAG CMP_SCR_CFF_MASK /**< Falling edge flag. */
#define CMP_PDD_RISING_EDGE_FLAG  CMP_SCR_CFR_MASK /**< Rising edge flag. */

/* Comparator modes */
#define CMP_PDD_FALLING_EDGE_MODE CMP_SCR_IEF_MASK /**< Falling edge detection mode. */
#define CMP_PDD_RISING_EDGE_MODE  CMP_SCR_IER_MASK /**< Rising edge detection mode. */
#define CMP_PDD_BOTH_EDGES_MODE   0x18U          /**< Both falling and rising edge detection mode. */

/* Hysteresis constants. For exact value representation see peripheral device
   documentation. */
#define CMP_PDD_HYSTERESIS_LEVEL_0 0U            /**< Level 0 hysteresis used. */
#define CMP_PDD_HYSTERESIS_LEVEL_1 0x1U          /**< Level 1 hysteresis used. */
#define CMP_PDD_HYSTERESIS_LEVEL_2 0x2U          /**< Level 2 hysteresis used. */
#define CMP_PDD_HYSTERESIS_LEVEL_3 0x3U          /**< Level 3 hysteresis used. */

/* Filter sample count constants. */
#define CMP_PDD_FILTER_DISABLED  0U              /**< Filter disabled. */
#define CMP_PDD_FILTER_1_SAMPLE  0x10U           /**< 1 consecutive sample must agree. */
#define CMP_PDD_FILTER_2_SAMPLES 0x20U           /**< 2 consecutive samples must agree. */
#define CMP_PDD_FILTER_3_SAMPLES 0x30U           /**< 3 consecutive samples must agree. */
#define CMP_PDD_FILTER_4_SAMPLES 0x40U           /**< 4 consecutive samples must agree. */
#define CMP_PDD_FILTER_5_SAMPLES 0x50U           /**< 5 consecutive samples must agree. */
#define CMP_PDD_FILTER_6_SAMPLES 0x60U           /**< 6 consecutive samples must agree. */
#define CMP_PDD_FILTER_7_SAMPLES 0x70U           /**< 7 consecutive samples must agree. */

/* Comparator output filtration constant */
#define CMP_PDD_FILTERED_OUTPUT   0U             /**< Filtered comparator output. */
#define CMP_PDD_UNFILTERED_OUTPUT CMP_CR1_COS_MASK /**< Filter on comparator output is bypassed. */

/* Comparator output inversion constant. */
#define CMP_PDD_NOT_INVERTED_OUTPUT 0U           /**< Comparator output is not inverted. */
#define CMP_PDD_INVERTED_OUTPUT     CMP_CR1_INV_MASK /**< Comparator output is inverted. */

/* Comparator power mode constant. */
#define CMP_PDD_LOW_POWER_MODE  0U               /**< Low power/low speed mode. */
#define CMP_PDD_HIGH_SPEED_MODE CMP_CR1_PMODE_MASK /**< High power/high speed mode. */

/* Comparator's interrupt stop mode constant. */
#define CMP_PDD_LEVEL_SENSITIVE_MODE 0U          /**< Comparator is level sensitive in stop mode */
#define CMP_PDD_EDGE_SENSITIVE_MODE  0x1U        /**< Comparator is edge sensitive in stop mode */

/* Analog comparator 6-bit DAC supply voltage reference source constant. */
#define CMP_PDD_V_REF_INPUT_1 0U                 /**< Vin1 reference. */
#define CMP_PDD_V_REF_INPUT_2 CMP_DACCR_VRSEL_MASK /**< Vin2 reference */

/* Negative input constant. */
#define CMP_PDD_NEGATIVE_INPUT_0 0U              /**< Negative input 0. */
#define CMP_PDD_NEGATIVE_INPUT_1 0x1U            /**< Negative input 1. */
#define CMP_PDD_NEGATIVE_INPUT_2 0x2U            /**< Negative input 2. */
#define CMP_PDD_NEGATIVE_INPUT_3 0x3U            /**< Negative input 3. */
#define CMP_PDD_NEGATIVE_INPUT_4 0x4U            /**< Negative input 4. */
#define CMP_PDD_NEGATIVE_INPUT_5 0x5U            /**< Negative input 5. */
#define CMP_PDD_NEGATIVE_INPUT_6 0x6U            /**< Negative input 6. */
#define CMP_PDD_NEGATIVE_INPUT_7 0x7U            /**< Negative input 7. */

/* Positive input constant. */
#define CMP_PDD_POSITIVE_INPUT_0 0U              /**< Positive input 0. */
#define CMP_PDD_POSITIVE_INPUT_1 0x8U            /**< Positive input 1. */
#define CMP_PDD_POSITIVE_INPUT_2 0x10U           /**< Positive input 2. */
#define CMP_PDD_POSITIVE_INPUT_3 0x18U           /**< Positive input 3. */
#define CMP_PDD_POSITIVE_INPUT_4 0x20U           /**< Positive input 4. */
#define CMP_PDD_POSITIVE_INPUT_5 0x28U           /**< Positive input 5. */
#define CMP_PDD_POSITIVE_INPUT_6 0x30U           /**< Positive input 6. */
#define CMP_PDD_POSITIVE_INPUT_7 0x38U           /**< Positive input 7. */


/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the comparator's device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of comparator. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define CMP_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_EN_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_EN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDeviceEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of comparator's enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetDeviceEnabled(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetDeviceEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_EN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to enable. Use constants from group
 *        "Interrupts' mask". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableInterrupts(deviceID, CMP_PDD_FALLING_EDGE_INTERRUPT);
 *      @endcode
 */
#define CMP_PDD_EnableInterrupts(PeripheralBase, Mask) ( \
    PeriphSafeSetBitMask16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to disable. Use constants from group
 *        "Interrupts' mask". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_DisableInterrupts(deviceID, CMP_PDD_FALLING_EDGE_INTERRUPT);
 *      @endcode
 */
#define CMP_PDD_DisableInterrupts(PeripheralBase, Mask) ( \
    PeriphSafeClearBitMask16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets all interrupts with value according to mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to enable - rest will be disabled. Use
 *        constants from group "Interrupts' mask". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetInterruptMask(deviceID, CMP_PDD_FALLING_EDGE_INTERRUPT);
 *      @endcode
 */
#define CMP_PDD_SetInterruptMask(PeripheralBase, Mask) ( \
    PeriphSafeSetBits16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK), \
     (uint16_t)(CMP_SCR_IEF_MASK | CMP_SCR_IER_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupts mask.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Interrupts' mask" for processing return
 *         value.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetInterruptMask(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetInterruptMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_IEF_MASK | CMP_SCR_IER_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Interrupts' mask" for processing return
 *         value.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetInterruptFlags(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt's flags to clear. Use constants from group
 *        "Interrupts' mask". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_ClearInterruptFlags(deviceID, CMP_PDD_FALLING_EDGE_INTERRUPT);
 *      @endcode
 */
#define CMP_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     CMP_SCR_REG(PeripheralBase), \
     (uint16_t)(CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetHysteresis
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets comparator hard block hysteresis control bits.
 * @param PeripheralBase Peripheral base address.
 * @param Hysteresis Hysteresis value. This parameter is of "Hysteresis
 *        constants. For exact value representation see peripheral device documentation."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR0, CMPB_CR0,
 *          CMPC_CR0, CMPD_CR0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetHysteresis(deviceID, CMP_PDD_HYSTERESIS_LEVEL_0);
 *      @endcode
 */
#define CMP_PDD_SetHysteresis(PeripheralBase, Hysteresis) ( \
    PeriphSetBits16( \
     CMP_CR0_REG(PeripheralBase), \
     CMP_CR0_HYSTCTR_MASK, \
     (uint16_t)(Hysteresis) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetHysteresis
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current level of hysteresis.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Hysteresis constants. For exact value
 *         representation see peripheral device documentation." type. The value is cast to
 *         "uint8_t".
 * @remarks The macro accesses the following registers: CMPA_CR0, CMPB_CR0,
 *          CMPC_CR0, CMPD_CR0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = CMP_PDD_GetHysteresis(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetHysteresis(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR0_REG(PeripheralBase), CMP_CR0_HYSTCTR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetFilterCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets filter sample count bits.
 * @param PeripheralBase Peripheral base address.
 * @param Count Filter sample count. This parameter is of "Filter sample count
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR0, CMPB_CR0,
 *          CMPC_CR0, CMPD_CR0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetFilterCount(deviceID, CMP_PDD_FILTER_DISABLED);
 *      @endcode
 */
#define CMP_PDD_SetFilterCount(PeripheralBase, Count) ( \
    PeriphSetBits16( \
     CMP_CR0_REG(PeripheralBase), \
     CMP_CR0_FILTER_CNT_MASK, \
     (uint16_t)(Count) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFilterCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current level of hysteresis.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Filter sample count constants." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_CR0, CMPB_CR0,
 *          CMPC_CR0, CMPD_CR0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetFilterCount(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetFilterCount(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR0_REG(PeripheralBase), CMP_CR0_FILTER_CNT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOutputPin
   ---------------------------------------------------------------------------- */

/**
 * @brief Comparator output pin enable.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if output pin will be enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableOutputPin(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define CMP_PDD_EnableOutputPin(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_OPE_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_OPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputPinEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether is the output pin enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetOutputPinEnabled(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetOutputPinEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_OPE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetComparatorOutputFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects comparator output filtration.
 * @param PeripheralBase Peripheral base address.
 * @param Output Parameter specifying if output will be filtered or not. This
 *        parameter is of "Comparator output filtration constant" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetComparatorOutputFilter(deviceID, CMP_PDD_FILTERED_OUTPUT);
 *      @endcode
 */
#define CMP_PDD_SetComparatorOutputFilter(PeripheralBase, Output) ( \
    ((Output) == CMP_PDD_FILTERED_OUTPUT) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_COS_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_COS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetComparatorOutputFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of comparator output filtration.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Comparator output filtration constant" type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetComparatorOutputFilter(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetComparatorOutputFilter(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_COS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetComparatorOutputInversion
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects comparator output inversion.
 * @param PeripheralBase Peripheral base address.
 * @param Output Parameter specifying if output will be inverted or not. This
 *        parameter is of "Comparator output inversion constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetComparatorOutputInversion(deviceID,
 *      CMP_PDD_NOT_INVERTED_OUTPUT);
 *      @endcode
 */
#define CMP_PDD_SetComparatorOutputInversion(PeripheralBase, Output) ( \
    ((Output) == CMP_PDD_NOT_INVERTED_OUTPUT) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_INV_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_INV_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetComparatorOutputInversion
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether the comparator output is inverted.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Comparator output inversion constant." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetComparatorOutputInversion(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetComparatorOutputInversion(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_INV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects comparator power mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Power mode. This parameter is of "Comparator power mode
 *        constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetPowerMode(deviceID, CMP_PDD_LOW_POWER_MODE);
 *      @endcode
 */
#define CMP_PDD_SetPowerMode(PeripheralBase, Mode) ( \
    ((Mode) == CMP_PDD_LOW_POWER_MODE) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_PMODE_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_PMODE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current comparator power mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Comparator power mode constant." type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetPowerMode(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetPowerMode(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_PMODE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWindowing
   ---------------------------------------------------------------------------- */

/**
 * @brief Windowing enable.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if windowing will be enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableWindowing(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define CMP_PDD_EnableWindowing(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_WE_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_WE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetWindowingEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether the windowing is enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetWindowingEnabled(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetWindowingEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_WE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSampling
   ---------------------------------------------------------------------------- */

/**
 * @brief Sampling enable.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if sampling will be enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableSampling(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define CMP_PDD_EnableSampling(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_SE_MASK)) : ( \
      PeriphSetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_SE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSamplingEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns whether the sampling is enabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_CR1, CMPB_CR1,
 *          CMPC_CR1, CMPD_CR1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetSamplingEnabled(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetSamplingEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_CR1_REG(PeripheralBase), CMP_CR1_SE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetFilterPeriod
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets filter sample period.
 * @param PeripheralBase Peripheral base address.
 * @param Period Filter sample period value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_FPR, CMPB_FPR,
 *          CMPC_FPR, CMPD_FPR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetFilterPeriod(deviceID, 1);
 *      @endcode
 */
#define CMP_PDD_SetFilterPeriod(PeripheralBase, Period) ( \
    PeriphSetBits16( \
     CMP_FPR_REG(PeripheralBase), \
     CMP_FPR_FILT_PER_MASK, \
     (uint16_t)(Period) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFilterPeriod
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current filter sample period.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CMPA_FPR, CMPB_FPR,
 *          CMPC_FPR, CMPD_FPR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = CMP_PDD_GetFilterPeriod(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetFilterPeriod(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CMP_FPR_REG(PeripheralBase), CMP_FPR_FILT_PER_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetComparatorOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current comparator output state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_SCR, CMPB_SCR,
 *          CMPC_SCR, CMPD_SCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetComparatorOutput(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetComparatorOutput(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_SCR_REG(PeripheralBase), CMP_SCR_COUT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetComparatorMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets comparator's interrupt mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Comparator interrupt mode. Use constants from group "Comparator
 *        modes". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @par Example:
 *      @code
 *      CMP_PDD_SetComparatorMode(deviceID, CMP_PDD_FALLING_EDGE_MODE);
 *      @endcode
 */
#define CMP_PDD_SetComparatorMode(PeripheralBase, Mode) ( \
    CMP_PDD_SetInterruptMask(PeripheralBase, (uint16_t)(Mode)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCompareStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns last detected edge on comparator's output.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Interrupts' mask" for processing return
 *         value.
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetCompareStatus(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetCompareStatus(PeripheralBase) ( \
    CMP_PDD_GetInterruptFlags(PeripheralBase) \
  )

/* ----------------------------------------------------------------------------
   -- SetVoltage
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DAC6b output voltage.
 * @param PeripheralBase Peripheral base address.
 * @param Value Output voltage value. This parameter is a 6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetVoltage(deviceID, 1);
 *      @endcode
 */
#define CMP_PDD_SetVoltage(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CMP_DACCR_REG(PeripheralBase), \
     CMP_DACCR_VOSEL_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVoltage
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current DAC6b output voltage.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 6-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = CMP_PDD_GetVoltage(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetVoltage(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_VOSEL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetReference
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects analog comparator 6-bit DAC supply voltage reference source.
 * @param PeripheralBase Peripheral base address.
 * @param Reference Supply voltage source. This parameter is of "Analog
 *        comparator 6-bit DAC supply voltage reference source constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetReference(deviceID, CMP_PDD_V_REF_INPUT_1);
 *      @endcode
 */
#define CMP_PDD_SetReference(PeripheralBase, Reference) ( \
    ((Reference) == CMP_PDD_V_REF_INPUT_1) ? ( \
      PeriphClearBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_VRSEL_MASK)) : ( \
      PeriphSetBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_VRSEL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetReference
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns selected reference input.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Analog comparator 6-bit DAC supply voltage
 *         reference source constant." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetReference(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetReference(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_VRSEL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDac
   ---------------------------------------------------------------------------- */

/**
 * @brief Analog comparator 6-bit DAC enable control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if DAC will be enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_EnableDac(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define CMP_PDD_EnableDac(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_DACEN_MASK)) : ( \
      PeriphSetBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_DACEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDacEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of analog comparator 6-bit DAC enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPA_DACCR, CMPB_DACCR,
 *          CMPC_DACCR, CMPD_DACCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetDacEnabled(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetDacEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_DACCR_REG(PeripheralBase), CMP_DACCR_DACEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetNegativeInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets negative comparator input.
 * @param PeripheralBase Peripheral base address.
 * @param NegativeInput Negative input number. This parameter is of "Negative
 *        input constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_MUXCR, CMPB_MUXCR,
 *          CMPC_MUXCR, CMPD_MUXCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetNegativeInput(deviceID, CMP_PDD_NEGATIVE_INPUT_0);
 *      @endcode
 */
#define CMP_PDD_SetNegativeInput(PeripheralBase, NegativeInput) ( \
    PeriphSetBits16( \
     CMP_MUXCR_REG(PeripheralBase), \
     CMP_MUXCR_MSEL_MASK, \
     (uint16_t)(NegativeInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetNegativeInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns selected negative input.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Negative input constant." type. The value is cast
 *         to "uint8_t".
 * @remarks The macro accesses the following registers: CMPA_MUXCR, CMPB_MUXCR,
 *          CMPC_MUXCR, CMPD_MUXCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = CMP_PDD_GetNegativeInput(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetNegativeInput(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_MUXCR_REG(PeripheralBase), CMP_MUXCR_MSEL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPositiveInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets positive comparator input.
 * @param PeripheralBase Peripheral base address.
 * @param PositiveInput Positive input number. This parameter is of "Positive
 *        input constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPA_MUXCR, CMPB_MUXCR,
 *          CMPC_MUXCR, CMPD_MUXCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      CMP_PDD_SetPositiveInput(deviceID, CMP_PDD_POSITIVE_INPUT_0);
 *      @endcode
 */
#define CMP_PDD_SetPositiveInput(PeripheralBase, PositiveInput) ( \
    PeriphSetBits16( \
     CMP_MUXCR_REG(PeripheralBase), \
     CMP_MUXCR_PSEL_MASK, \
     (uint16_t)(PositiveInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPositiveInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns selected positive input.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Positive input constant." type. The value is cast
 *         to "uint16_t".
 * @remarks The macro accesses the following registers: CMPA_MUXCR, CMPB_MUXCR,
 *          CMPC_MUXCR, CMPD_MUXCR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = CMP_PDD_GetPositiveInput(deviceID);
 *      @endcode
 */
#define CMP_PDD_GetPositiveInput(PeripheralBase) ( \
    PeriphGetBitMask16(CMP_MUXCR_REG(PeripheralBase), CMP_MUXCR_PSEL_MASK) \
  )
#endif  /* #if defined(CMP_PDD_H_) */

/* CMP_PDD.h, eof. */
