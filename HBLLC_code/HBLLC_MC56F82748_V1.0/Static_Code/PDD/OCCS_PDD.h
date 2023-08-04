/*
  PDD layer implementation for peripheral type OCCS
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(OCCS_PDD_H_)
#define OCCS_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error OCCS PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* OCCS */ && \
      !defined(MCU_MC56F82316) /* OCCS */ && \
      !defined(MCU_MC56F82723) /* OCCS */ && \
      !defined(MCU_MC56F82726) /* OCCS */ && \
      !defined(MCU_MC56F82728) /* OCCS */ && \
      !defined(MCU_MC56F82733) /* OCCS */ && \
      !defined(MCU_MC56F82736) /* OCCS */ && \
      !defined(MCU_MC56F82738) /* OCCS */ && \
      !defined(MCU_MC56F82743) /* OCCS */ && \
      !defined(MCU_MC56F82746) /* OCCS */ && \
      !defined(MCU_MC56F82748) /* OCCS */
  // Unsupported MCU is active
  #error OCCS PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Interrupts' flags. */
#define OCCS_PDD_PLL_LOCK1_FLAG     OCCS_STAT_LOLI1_MASK /**< PLL lock or loss of lock 1 interrupt flag. */
#define OCCS_PDD_PLL_LOCK0_FLAG     OCCS_STAT_LOLI0_MASK /**< PLL lock or loss of lock 0 interrupt flag. */
#define OCCS_PDD_LOSS_OF_CLOCK_FLAG OCCS_STAT_LOCI_MASK /**< Loss of reference clock interrupt flag. */

/* SIM sys_clk_x2 clock source constants. */
#define OCCS_PDD_CLOCK_SOURCE_MASTER_OSCILLATOR 0U /**< MSTR_OSC. */
#define OCCS_PDD_CLOCK_SOURCE_PLL_DIV2          OCCS_CTRL_ZSRC_MASK /**< PLL output divided by 2. */

/* Clock output divider constants. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_1   0U     /**< Divide clock output by 1. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_2   0x100U /**< Divide clock output by 2. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_4   0x200U /**< Divide clock output by 4. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_8   0x300U /**< Divide clock output by 8. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_16  0x400U /**< Divide clock output by 16. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_32  0x500U /**< Divide clock output by 32. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_64  0x600U /**< Divide clock output by 64. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_128 0x700U /**< Divide clock output by 128. */
#define OCCS_PDD_CLOCK_OUTPUT_DIVIDER_256 0x800U /**< Divide clock output by 256. */

/* PWM nano edge clock source constants. */
#define OCCS_PDD_PWM_PLL      0U                 /**< Raw PLL output. */
#define OCCS_PDD_PWM_PLL_DIV2 OCCS_DIVBY_PWM_DIV2_MASK /**< PLL output divided by 2. */

/* Status flags. */
#define OCCS_PDD_XOSC_FAILURE                OCCS_STAT_MON_FAILURE_MASK /**< XOSC clock monitor failure flag. */
#define OCCS_PDD_XOSC_STABLE                 OCCS_STAT_OSC_OK_MASK /**< Oscillator clock stable flag. */
#define OCCS_PDD_PLL_LOCK1                   OCCS_STAT_LCK1_MASK /**< PLL lock 1 flag. */
#define OCCS_PDD_PLL_LOCK0                   OCCS_STAT_LCK0_MASK /**< PLL lock 0 flag. */
#define OCCS_PDD_PLL_POWER_DOWN              OCCS_STAT_PLLPDN_MASK /**< PLL powered down flag. */
#define OCCS_PDD_MASTER_OSCILLATOR           0U  /**< MSTR_OSC as clock source flag. */
#define OCCS_PDD_PLL_DIV2                    0x1U /**< PLL output divided by 2 as clock source flag. */
#define OCCS_PDD_SYNCHRONIZATION_IN_PROGRESS 0x2U /**< Clock source synchronization in progress flag. */

/* Crystal oscillator power mode constants. */
#define OCCS_PDD_OSCILLATOR_POWER_HIGH 0U        /**< High power mode. */
#define OCCS_PDD_OSCILLATOR_POWER_LOW  OCCS_OSCTL1_COHL_MASK /**< Low power mode. */

/* Crystal oscillator power clock constants. */
#define OCCS_PDD_XOSC_CLOCK_ENABLE 0U            /**< Crystal oscillator enabled. */
#define OCCS_PDD_XOSC_CLOCK_BYPASS OCCS_OSCTL1_CLK_MODE_MASK /**< External clock bypass mode. */

/* External clock source constants. */
#define OCCS_PDD_EXTERNAL_CLOCK_OSCILLATOR 0U    /**< Crystal oscillator as the external clock input. */
#define OCCS_PDD_EXTERNAL_CLOCK_CLKIN      OCCS_OSCTL1_EXT_SEL_MASK /**< CLKIN as the external clock input. */

/* External clock checking status constants. */
#define OCCS_PDD_EXTERNAL_CLOCK_CHECKING_DONE 0U /**< Check operation is complete. */
#define OCCS_PDD_EXTERNAL_CLOCK_CHECKING_BUSY OCCS_CLKCHKR_CHK_ENA_MASK /**< Check operation is in progress. */

/* Interrupts' edge detection constants. */
#define OCCS_PDD_EDGE_NONE    0U                 /**< Disable interrupt. */
#define OCCS_PDD_EDGE_RISING  0x1U               /**< Enable interrupt on any rising edge. */
#define OCCS_PDD_EDGE_FALLING 0x2U               /**< Enable interrupt on falling edge. */
#define OCCS_PDD_EDGE_BOTH    0x3U               /**< Enable interrupt on any edge change. */

/* Clock source constants. */
#define OCCS_PDD_PRESCALER_SOURCE_ROSC_8M            0U /**< 8 MHz relaxation oscillator. */
#define OCCS_PDD_PRESCALER_SOURCE_EXTERNAL_REFERENCE 0x4U /**< External reference. */
#define OCCS_PDD_PRESCALER_SOURCE_ROSC_200K          0x8U /**< 200 kHz RC oscillator. */

/* Write protection constants. */
#define OCCS_PDD_PROTECTION_DISABLE        0U    /**< Write protection off. */
#define OCCS_PDD_PROTECTION_ENABLE         0x1U  /**< Write protection on. */
#define OCCS_PDD_PROTECTION_DISABLE_LOCKED 0x2U  /**< Write protection off and locked until chip reset. */
#define OCCS_PDD_PROTECTION_ENABLE_LOCKED  0x3U  /**< Write protection on and locked until chip reset. */


/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_STAT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetInterruptFlags(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     OCCS_STAT_REG(PeripheralBase), \
     (uint16_t)(OCCS_STAT_LOLI1_MASK | (OCCS_STAT_LOLI0_MASK | OCCS_STAT_LOCI_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt's flags to clear. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_STAT.
 * @par Example:
 *      @code
 *      OCCS_PDD_ClearInterruptFlags(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(OCCS_STAT_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInterruptEgdePll1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the PLL 1 interrupt generation condition.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter specifying the edge change of LCK1 causing interrupt
 *        generation. This parameter is of "Interrupts' edge detection constants."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetInterruptEgdePll1(deviceID, OCCS_PDD_EDGE_NONE);
 *      @endcode
 */
#define OCCS_PDD_SetInterruptEgdePll1(PeripheralBase, Edge) ( \
    PeriphSetBits16( \
     OCCS_CTRL_REG(PeripheralBase), \
     OCCS_CTRL_PLLIE1_MASK, \
     (uint16_t)((uint16_t)(Edge) << OCCS_CTRL_PLLIE1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptEgdePll1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PLL 1 interrupt generation condition.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetInterruptEgdePll1(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetInterruptEgdePll1(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PLLIE1_MASK)) >> ( \
     OCCS_CTRL_PLLIE1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInterruptEgdePll0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the PLL 0 interrupt generation condition.
 * @param PeripheralBase Peripheral base address.
 * @param Edge Parameter specifying the edge change of LCK0 causing interrupt
 *        generation. This parameter is of "Interrupts' edge detection constants."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetInterruptEgdePll0(deviceID, OCCS_PDD_EDGE_NONE);
 *      @endcode
 */
#define OCCS_PDD_SetInterruptEgdePll0(PeripheralBase, Edge) ( \
    PeriphSetBits16( \
     OCCS_CTRL_REG(PeripheralBase), \
     OCCS_CTRL_PLLIE0_MASK, \
     (uint16_t)((uint16_t)(Edge) << OCCS_CTRL_PLLIE0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptEgdePll0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PLL 0 interrupt generation condition.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetInterruptEgdePll0(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetInterruptEgdePll0(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PLLIE0_MASK)) >> ( \
     OCCS_CTRL_PLLIE0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLossOfReferenceClockInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the loss of reference clock interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableLossOfReferenceClockInterrupt(deviceID);
 *      @endcode
 */
#define OCCS_PDD_EnableLossOfReferenceClockInterrupt(PeripheralBase) ( \
    PeriphSetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_LOCIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- DisableLossOfReferenceClockInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the loss of reference clock interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_DisableLossOfReferenceClockInterrupt(deviceID);
 *      @endcode
 */
#define OCCS_PDD_DisableLossOfReferenceClockInterrupt(PeripheralBase) ( \
    PeriphClearBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_LOCIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLockDetector
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the lock detector.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the lock detector is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableLockDetector(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableLockDetector(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_LCKON_MASK)) : ( \
      PeriphSetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_LCKON_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetLockDetectorEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the lock detector enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetLockDetectorEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetLockDetectorEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_LCKON_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePll
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the PLL.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the PLL is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnablePll(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnablePll(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PLLPD_MASK)) : ( \
      PeriphClearBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PLLPD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPllEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PLL enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetPllEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPllEnabled(PeripheralBase) ( \
    (PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PLLPD_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescalerClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the prescaler clock source.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock source. This parameter is of
 *        "Clock source constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetPrescalerClockSource(deviceID,
 *      OCCS_PDD_PRESCALER_SOURCE_ROSC_8M);
 *      @endcode
 */
#define OCCS_PDD_SetPrescalerClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     OCCS_CTRL_REG(PeripheralBase), \
     OCCS_CTRL_PRECS_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPrescalerClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the prescaler clock source.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Clock source constants." type. The value is cast
 *         to "uint16_t".
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetPrescalerClockSource(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPrescalerClockSource(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_PRECS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SIM sys_clk_x2 clock source.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock source. Use constants from group
 *        "SIM sys_clk_x2 clock source constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetClockSource(deviceID,
 *      OCCS_PDD_CLOCK_SOURCE_MASTER_OSCILLATOR);
 *      @endcode
 */
#define OCCS_PDD_SetClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     OCCS_CTRL_REG(PeripheralBase), \
     OCCS_CTRL_ZSRC_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the SIM sys_clk_x2 clock source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "SIM sys_clk_x2 clock source constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetClockSource(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetClockSource(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_CTRL_REG(PeripheralBase), OCCS_CTRL_ZSRC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetLossOfReferenceClockTripPoint
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the loss of reference clock trip point value used to calculate
 * failure detection time.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the trip point value. This parameter is a
 *        4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetLossOfReferenceClockTripPoint(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetLossOfReferenceClockTripPoint(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_DIVBY_REG(PeripheralBase), \
     OCCS_DIVBY_LORTP_MASK, \
     (uint16_t)((uint16_t)(Value) << OCCS_DIVBY_LORTP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetLossOfReferenceClockTripPoint
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the loss of reference clock trip point value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetLossOfReferenceClockTripPoint(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetLossOfReferenceClockTripPoint(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(OCCS_DIVBY_REG(PeripheralBase), OCCS_DIVBY_LORTP_MASK)) >> ( \
     OCCS_DIVBY_LORTP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockOutputDivider
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the clock output divider.
 * @param PeripheralBase Peripheral base address.
 * @param Divider Parameter specifying the divider. Use constants from group
 *        "Clock output divider constants.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetClockOutputDivider(deviceID,
 *      OCCS_PDD_CLOCK_OUTPUT_DIVIDER_1);
 *      @endcode
 */
#define OCCS_PDD_SetClockOutputDivider(PeripheralBase, Divider) ( \
    PeriphSetBits16( \
     OCCS_DIVBY_REG(PeripheralBase), \
     OCCS_DIVBY_COD_MASK, \
     (uint16_t)(Divider) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockOutputDivider
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the clock output divider.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Clock output divider constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetClockOutputDivider(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetClockOutputDivider(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_DIVBY_REG(PeripheralBase), OCCS_DIVBY_COD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPwmNanoEdgeClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the PWM nano edge clock source.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the clock source. Use constants from group
 *        "PWM nano edge clock source constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetPwmNanoEdgeClockSource(deviceID, OCCS_PDD_PWM_PLL);
 *      @endcode
 */
#define OCCS_PDD_SetPwmNanoEdgeClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     OCCS_DIVBY_REG(PeripheralBase), \
     OCCS_DIVBY_PWM_DIV2_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPwmNanoEdgeClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PWM nano edge clock source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "PWM nano edge clock source constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetPwmNanoEdgeClockSource(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPwmNanoEdgeClockSource(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_DIVBY_REG(PeripheralBase), OCCS_DIVBY_PWM_DIV2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPllMultiplier
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the PLL multiplier value. Direct value which should be used to
 * multiply the PLL input frequency is expected. Additional arithmetic operation is
 * performed to obtain raw value written to the PLL multiplier bitfield.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the requested multiplier factor. Value
 *        should be in range from 1 to 64. This parameter is a 7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetPllMultiplier(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetPllMultiplier(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_DIVBY_REG(PeripheralBase), \
     OCCS_DIVBY_PLLDB_MASK, \
     (uint16_t)(uint8_t)(Value) - 1U \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPllMultiplier
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PLL multiplier factor value used to multiply the PLL input
 * frequency. Additional arithmetic operation is performed to obtain miltiplier
 * factor value from the PLL multiplier bitfield.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 7-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetPllMultiplier(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPllMultiplier(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_DIVBY_REG(PeripheralBase), OCCS_DIVBY_PLLDB_MASK) + 1U \
  )

/* ----------------------------------------------------------------------------
   -- SetPllMultiplierRaw
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the PLL multiplier bitfield raw value.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the bitfield value. This parameter is a
 *        6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetPllMultiplierRaw(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetPllMultiplierRaw(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_DIVBY_REG(PeripheralBase), \
     OCCS_DIVBY_PLLDB_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPllMultiplierRaw
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the PLL multiplier bitfield raw value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetPllMultiplierRaw(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPllMultiplierRaw(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_DIVBY_REG(PeripheralBase), OCCS_DIVBY_PLLDB_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns status flag bits (including intterrupt flag bits).
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Status flags." for processing return value.
 * @remarks The macro accesses the following registers: OCCS_STAT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetStatusFlags(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetStatusFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     OCCS_STAT_REG(PeripheralBase), \
     (uint16_t)(( \
      OCCS_STAT_LOLI1_MASK) | (( \
      OCCS_STAT_LOLI0_MASK) | (( \
      OCCS_STAT_LOCI_MASK) | (( \
      OCCS_STAT_MON_FAILURE_MASK) | (( \
      OCCS_STAT_OSC_OK_MASK) | (( \
      OCCS_STAT_LCK1_MASK) | (( \
      OCCS_STAT_LCK0_MASK) | (( \
      OCCS_STAT_PLLPDN_MASK) | ( \
      OCCS_STAT_ZSRCS_MASK))))))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRosc8M
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or powers down the 8 MHz relaxation oscillator.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the relaxation oscillator is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableRosc8M(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableRosc8M(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROPD_MASK)) : ( \
      PeriphClearBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROPD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc8MEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetRosc8MEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc8MEnabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROPD_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRosc8MStandby
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the 8 MHz relaxation oscillator standby mode (in
 * standby mode oscillator frequency changes to 400 kHz).
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the relaxation oscillator is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableRosc8MStandby(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableRosc8MStandby(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROSB_MASK)) : ( \
      PeriphSetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROSB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc8MStandbyEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetRosc8MStandbyEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc8MStandbyEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_ROPD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetXoscPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the crystal oscillator power mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Parameter specifying if the oscillator power level. Use constants
 *        from group "Crystal oscillator power mode constants.". This parameter
 *        is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetXoscPowerMode(deviceID, OCCS_PDD_OSCILLATOR_POWER_HIGH);
 *      @endcode
 */
#define OCCS_PDD_SetXoscPowerMode(PeripheralBase, Mode) ( \
    PeriphSetBits16( \
     OCCS_OSCTL1_REG(PeripheralBase), \
     OCCS_OSCTL1_COHL_MASK, \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetXoscPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the crystal oscillator power mode.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Crystal oscillator power mode constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetXoscPowerMode(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetXoscPowerMode(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_COHL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetXoscClockMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the crystal oscillator clock mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Parameter specifying if the oscillator clock mode. Use constants
 *        from group "Crystal oscillator power clock constants.". This parameter
 *        is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetXoscClockMode(deviceID, OCCS_PDD_XOSC_CLOCK_ENABLE);
 *      @endcode
 */
#define OCCS_PDD_SetXoscClockMode(PeripheralBase, Mode) ( \
    PeriphSetBits16( \
     OCCS_OSCTL1_REG(PeripheralBase), \
     OCCS_OSCTL1_CLK_MODE_MASK, \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetXoscMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the crystal oscillator clock mode.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Crystal oscillator power clock constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetXoscMode(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetXoscMode(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_CLK_MODE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetExternalClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the external clock source.
 * @param PeripheralBase Peripheral base address.
 * @param Source Parameter specifying the external clock source. Use constants
 *        from group "External clock source constants.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetExternalClockSource(deviceID,
 *      OCCS_PDD_EXTERNAL_CLOCK_OSCILLATOR);
 *      @endcode
 */
#define OCCS_PDD_SetExternalClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     OCCS_OSCTL1_REG(PeripheralBase), \
     OCCS_OSCTL1_EXT_SEL_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the external clock source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "External clock source constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetExternalClockSource(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetExternalClockSource(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_OSCTL1_REG(PeripheralBase), OCCS_OSCTL1_EXT_SEL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetRosc8MFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the 8 MHz relaxation oscillator frequency trim.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the trim value. This parameter is a 10-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetRosc8MFrequencyTrim(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetRosc8MFrequencyTrim(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_OSCTL1_REG(PeripheralBase), \
     OCCS_OSCTL1_FREQ_TRIM8M_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc8MFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator frequency trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 10-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetRosc8MFrequencyTrim(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc8MFrequencyTrim(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     OCCS_OSCTL1_REG(PeripheralBase), \
     OCCS_OSCTL1_FREQ_TRIM8M_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRosc200k
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or powers down the 200 kHz RC oscillator.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the RC oscillator is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableRosc200k(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableRosc200k(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_ROPD200K_MASK)) : ( \
      PeriphClearBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_ROPD200K_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc200kEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 200 kHz RC oscillator enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetRosc200kEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc200kEnabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_ROPD200K_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXosc
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or powers down the crystal oscillator.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the crystal oscillator is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableXosc(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableXosc(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_COPD_MASK)) : ( \
      PeriphClearBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_COPD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXoscEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the crystaloscillator enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetXoscEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetXoscEnabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_COPD_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- SetRosc8MTemperatureTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the 8 MHz relaxation oscillator temperature trim.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the trim value. This parameter is a 4-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetRosc8MTemperatureTrim(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetRosc8MTemperatureTrim(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_OSCTL2_REG(PeripheralBase), \
     OCCS_OSCTL2_TEMP_TRIM8M_MASK, \
     (uint16_t)((uint16_t)(Value) << OCCS_OSCTL2_TEMP_TRIM8M_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc8MTemperatureTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 MHz relaxation oscillator temperature trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetRosc8MTemperatureTrim(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc8MTemperatureTrim(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      OCCS_OSCTL2_REG(PeripheralBase), \
      OCCS_OSCTL2_TEMP_TRIM8M_MASK \
     )) >> ( \
     OCCS_OSCTL2_TEMP_TRIM8M_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXoscMonitor
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables down the crystal oscillator clock monitor.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the monitor is enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_EnableXoscMonitor(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_EnableXoscMonitor(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       OCCS_OSCTL2_REG(PeripheralBase), \
       OCCS_OSCTL2_MON_ENABLE_MASK \
      )) : ( \
      PeriphSetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_MON_ENABLE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXoscMonitorEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the crystal oscillator clock monitor enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetXoscMonitorEnabled(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetXoscMonitorEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_OSCTL2_REG(PeripheralBase), OCCS_OSCTL2_MON_ENABLE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetRosc200kFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the 200 kHz relaxation oscillator frequency trim.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the trim value. This parameter is a 9-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetRosc200kFrequencyTrim(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_SetRosc200kFrequencyTrim(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     OCCS_OSCTL2_REG(PeripheralBase), \
     OCCS_OSCTL2_FREQ_TRIM200K_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetRosc200kFrequencyTrim
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 200 kHz relaxation oscillator frequency trim value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 9-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetRosc200kFrequencyTrim(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetRosc200kFrequencyTrim(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     OCCS_OSCTL2_REG(PeripheralBase), \
     OCCS_OSCTL2_FREQ_TRIM200K_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- StartExternalClockCheck
   ---------------------------------------------------------------------------- */

/**
 * @brief Starts the external clock checking function.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      OCCS_PDD_StartExternalClockCheck(deviceID);
 *      @endcode
 */
#define OCCS_PDD_StartExternalClockCheck(PeripheralBase) ( \
    PeriphSetBitMask16(OCCS_CLKCHKR_REG(PeripheralBase), OCCS_CLKCHKR_CHK_ENA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- StopExternalClockCheck
   ---------------------------------------------------------------------------- */

/**
 * @brief Stops the external clock checking function.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      OCCS_PDD_StopExternalClockCheck(deviceID);
 *      @endcode
 */
#define OCCS_PDD_StopExternalClockCheck(PeripheralBase) ( \
    PeriphClearBitMask16( \
     OCCS_CLKCHKR_REG(PeripheralBase), \
     OCCS_CLKCHKR_CHK_ENA_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalClockCheckStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the external clock checking status.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "External clock checking status constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetExternalClockCheckStatus(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetExternalClockCheckStatus(PeripheralBase) ( \
    PeriphGetBitMask16(OCCS_CLKCHKR_REG(PeripheralBase), OCCS_CLKCHKR_CHK_ENA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalClockCheckReference
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the external clock checking reference count value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 7-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetExternalClockCheckReference(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetExternalClockCheckReference(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(OCCS_CLKCHKR_REG(PeripheralBase), OCCS_CLKCHKR_REF_CNT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetExternalClockCheckTarget
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the external clock checking target count value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 11-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: OCCS_CLKCHKT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_GetExternalClockCheckTarget(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetExternalClockCheckTarget(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     OCCS_CLKCHKT_REG(PeripheralBase), \
     OCCS_CLKCHKT_TARGET_CNT_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFrequencyProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of the COD and ZSRC registers.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the COD and
 *        ZSRC registers. This parameter is of "Write protection constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetFrequencyProtection(deviceID, OCCS_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_SetFrequencyProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     OCCS_PROT_REG(PeripheralBase), \
     OCCS_PROT_FRQEP_MASK, \
     (uint16_t)((uint16_t)(Protection) << OCCS_PROT_FRQEP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFrequencyProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the COD and ZSRC registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetFrequencyProtection(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetFrequencyProtection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(OCCS_PROT_REG(PeripheralBase), OCCS_PROT_FRQEP_MASK)) >> ( \
     OCCS_PROT_FRQEP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetOscillatorProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of the OSCTL1, OSCTL2, and PRECS registers.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the OSCTL1,
 *        OSCTL2, and PRECS registers. This parameter is of "Write protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetOscillatorProtection(deviceID, OCCS_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_SetOscillatorProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     OCCS_PROT_REG(PeripheralBase), \
     OCCS_PROT_OSCEP_MASK, \
     (uint16_t)((uint16_t)(Protection) << OCCS_PROT_OSCEP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOscillatorProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the OSCTL1, OSCTL2, and PRECS
 * registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetOscillatorProtection(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetOscillatorProtection(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(OCCS_PROT_REG(PeripheralBase), OCCS_PROT_OSCEP_MASK)) >> ( \
     OCCS_PROT_OSCEP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPllProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets write protection of the PLLPD, LOCIE, LORTP, and PLLDB registers.
 * @param PeripheralBase Peripheral base address.
 * @param Protection Parameter specifying the write protection of the PLLPD,
 *        LOCIE, LORTP, and PLLDB registers. This parameter is of "Write protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      OCCS_PDD_SetPllProtection(deviceID, OCCS_PDD_PROTECTION_DISABLE);
 *      @endcode
 */
#define OCCS_PDD_SetPllProtection(PeripheralBase, Protection) ( \
    PeriphSetBits16( \
     OCCS_PROT_REG(PeripheralBase), \
     OCCS_PROT_PLLEP_MASK, \
     (uint16_t)(Protection) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPllProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns write protection mode of the PLLPD, LOCIE, LORTP, and PLLDB
 * registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      uint8_t result = OCCS_PDD_GetPllProtection(deviceID);
 *      @endcode
 */
#define OCCS_PDD_GetPllProtection(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(OCCS_PROT_REG(PeripheralBase), OCCS_PROT_PLLEP_MASK) \
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
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePllControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the PLL divide-by control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      OCCS_PDD_WritePllControlReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WritePllControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_DIVBY_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPllControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the PLL divide-by control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_DIVBY.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadPllControlReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadPllControlReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_DIVBY_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_STAT.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteStatusReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_STAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_STAT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadStatusReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_STAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOscillatorControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the oscillator control register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteOscillatorControl1Reg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteOscillatorControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_OSCTL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOscillatorControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the oscillator control register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_OSCTL1.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadOscillatorControl1Reg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadOscillatorControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_OSCTL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOscillatorControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the oscillator control register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteOscillatorControl2Reg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteOscillatorControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_OSCTL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOscillatorControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the oscillator control register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_OSCTL2.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadOscillatorControl2Reg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadOscillatorControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_OSCTL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteExternalClockCheckReferenceReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the external clock check reference register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteExternalClockCheckReferenceReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteExternalClockCheckReferenceReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_CLKCHKR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadExternalClockCheckReferenceReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the external clock check reference register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKR.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadExternalClockCheckReferenceReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadExternalClockCheckReferenceReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_CLKCHKR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteExternalClockCheckTargetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the external clock check target register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKT.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteExternalClockCheckTargetReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteExternalClockCheckTargetReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_CLKCHKT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadExternalClockCheckTargetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the external clock check target register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_CLKCHKT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadExternalClockCheckTargetReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadExternalClockCheckTargetReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_CLKCHKT_REG(PeripheralBase)) \
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
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      OCCS_PDD_WriteProtectionReg(deviceID, 1);
 *      @endcode
 */
#define OCCS_PDD_WriteProtectionReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(OCCS_PROT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadProtectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the protection register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCCS_PROT.
 * @par Example:
 *      @code
 *      uint16_t result = OCCS_PDD_ReadProtectionReg(deviceID);
 *      @endcode
 */
#define OCCS_PDD_ReadProtectionReg(PeripheralBase) ( \
    PeriphReadReg16(OCCS_PROT_REG(PeripheralBase)) \
  )
#endif  /* #if defined(OCCS_PDD_H_) */

/* OCCS_PDD.h, eof. */
