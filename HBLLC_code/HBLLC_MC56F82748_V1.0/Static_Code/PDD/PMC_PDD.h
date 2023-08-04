/*
  PDD layer implementation for peripheral type PMC
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(PMC_PDD_H_)
#define PMC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error PMC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* PMC */ && \
      !defined(MCU_MC56F82316) /* PMC */ && \
      !defined(MCU_MC56F82723) /* PMC */ && \
      !defined(MCU_MC56F82726) /* PMC */ && \
      !defined(MCU_MC56F82728) /* PMC */ && \
      !defined(MCU_MC56F82733) /* PMC */ && \
      !defined(MCU_MC56F82736) /* PMC */ && \
      !defined(MCU_MC56F82738) /* PMC */ && \
      !defined(MCU_MC56F82743) /* PMC */ && \
      !defined(MCU_MC56F82746) /* PMC */ && \
      !defined(MCU_MC56F82748) /* PMC */
  // Unsupported MCU is active
  #error PMC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Interrupts' enable mask. */
#define PMC_PDD_2_7_V_HIGH_VOLTAGE_INTERRUPT PMC_CTRL_HV27IE_MASK /**< 2.7 V high voltage interrupt mask. */
#define PMC_PDD_2_2_V_HIGH_VOLTAGE_INTERRUPT PMC_CTRL_HV22IE_MASK /**< 2.2 V high voltage interrupt mask. */
#define PMC_PDD_2_7_V_LOW_VOLTAGE_INTERRUPT  PMC_CTRL_LV27IE_MASK /**< 2.7 V low voltage interrupt mask. */
#define PMC_PDD_2_2_V_LOW_VOLTAGE_INTERRUPT  PMC_CTRL_LV22IE_MASK /**< 2.2 V low voltage interrupt mask. */

/* Interrupts' flags. */
#define PMC_PDD_LOW_VOLTAGE_FLAG              PMC_STS_LVI_MASK /**< Low voltage flag. */
#define PMC_PDD_STICKY_2_7_V_LOW_VOLTAGE_FLAG PMC_STS_SLV27F_MASK /**< Sticky 2.7 V low voltage flag. */
#define PMC_PDD_STICKY_2_2_V_LOW_VOLTAGE_FLAG PMC_STS_SLV22F_MASK /**< Sticky 2.2 V low voltage flag. */

/* Status flags. */
#define PMC_PDD_2_7_V_SMALL_REGULATOR_FLAG PMC_STS_SR27_MASK /**< Small regulator 2.7 V active flag. */
#define PMC_PDD_2_7_V_LOW_VOLTAGE_FLAG     PMC_STS_LV27F_MASK /**< 2.7 V low voltage flag. */
#define PMC_PDD_2_2_V_LOW_VOLTAGE_FLAG     PMC_STS_LV22F_MASK /**< 2.2 V low voltage flag. */


/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to enable. Use constants from group
 *        "Interrupts' enable mask.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      PMC_PDD_EnableInterrupts(deviceID,
 *      PMC_PDD_2_7_V_HIGH_VOLTAGE_INTERRUPT);
 *      @endcode
 */
#define PMC_PDD_EnableInterrupts(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(PMC_CTRL_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to disable. Use constants from group
 *        "Interrupts' enable mask.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      PMC_PDD_DisableInterrupts(deviceID,
 *      PMC_PDD_2_7_V_HIGH_VOLTAGE_INTERRUPT);
 *      @endcode
 */
#define PMC_PDD_DisableInterrupts(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(PMC_CTRL_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Interrupts' flags." for processing return
 *         value.
 * @remarks The macro accesses the following registers: PMC_STS.
 * @par Example:
 *      @code
 *      uint16_t result = PMC_PDD_GetInterruptFlags(deviceID);
 *      @endcode
 */
#define PMC_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     PMC_STS_REG(PeripheralBase), \
     (uint16_t)(PMC_STS_LVI_MASK | (PMC_STS_SLV27F_MASK | PMC_STS_SLV22F_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt's flags to clear. Use constants from group
 *        "Interrupts' flags.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_STS.
 * @par Example:
 *      @code
 *      PMC_PDD_ClearInterruptFlags(deviceID, PMC_PDD_LOW_VOLTAGE_FLAG);
 *      @endcode
 */
#define PMC_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     PMC_STS_REG(PeripheralBase), \
     (uint16_t)(PMC_STS_LVI_MASK | (PMC_STS_SLV27F_MASK | PMC_STS_SLV22F_MASK)), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns all status flag bits.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PMC_STS.
 * @par Example:
 *      @code
 *      uint16_t result = PMC_PDD_GetStatusFlags(deviceID);
 *      @endcode
 */
#define PMC_PDD_GetStatusFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     PMC_STS_REG(PeripheralBase), \
     (uint16_t)(( \
      PMC_STS_SR27_MASK) | (( \
      PMC_STS_LVI_MASK) | (( \
      PMC_STS_SLV27F_MASK) | (( \
      PMC_STS_SLV22F_MASK) | (( \
      PMC_STS_LV27F_MASK) | ( \
      PMC_STS_LV22F_MASK)))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetBandgapTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets bandgap trim value.
 * @param PeripheralBase Peripheral base address.
 * @param Value Bandgap trim value. This parameter is a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      PMC_PDD_SetBandgapTrim(deviceID, 1);
 *      @endcode
 */
#define PMC_PDD_SetBandgapTrim(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     PMC_CTRL_REG(PeripheralBase), \
     PMC_CTRL_TRIM_MASK, \
     (uint16_t)((uint16_t)(Value) << PMC_CTRL_TRIM_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBandgapTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns bandgap trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = PMC_PDD_GetBandgapTrim(deviceID);
 *      @endcode
 */
#define PMC_PDD_GetBandgapTrim(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(PMC_CTRL_REG(PeripheralBase), PMC_CTRL_TRIM_MASK)) >> ( \
     PMC_CTRL_TRIM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVoltageReferenceBuffer
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables voltage reference buffer.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if voltage reference buffer will be enabled
 *        or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE defined
 *        in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      PMC_PDD_EnableVoltageReferenceBuffer(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define PMC_PDD_EnableVoltageReferenceBuffer(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(PMC_CTRL_REG(PeripheralBase), PMC_CTRL_VRBEN_MASK)) : ( \
      PeriphSetBitMask16(PMC_CTRL_REG(PeripheralBase), PMC_CTRL_VRBEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetVoltageReferenceBufferEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns voltage reference buffer state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = PMC_PDD_GetVoltageReferenceBufferEnabled(deviceID);
 *      @endcode
 */
#define PMC_PDD_GetVoltageReferenceBufferEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(PMC_CTRL_REG(PeripheralBase), PMC_CTRL_VRBEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to PMC control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value PMC control register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      PMC_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define PMC_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PMC_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns PMC control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PMC_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = PMC_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define PMC_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(PMC_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to PMC status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value PMC status register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PMC_STS.
 * @par Example:
 *      @code
 *      PMC_PDD_WriteStatusReg(deviceID, 1);
 *      @endcode
 */
#define PMC_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PMC_STS_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns PMC status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PMC_STS.
 * @par Example:
 *      @code
 *      uint16_t result = PMC_PDD_ReadStatusReg(deviceID);
 *      @endcode
 */
#define PMC_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(PMC_STS_REG(PeripheralBase)) \
  )
#endif  /* #if defined(PMC_PDD_H_) */

/* PMC_PDD.h, eof. */
