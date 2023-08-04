/*
  PDD layer implementation for peripheral type EWM
  Copyright 2010 Freescale, Inc.
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(EWM_PDD_H_)
#define EWM_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error EWM PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* EWM */ && \
      !defined(MCU_MC56F82316) /* EWM */ && \
      !defined(MCU_MC56F82723) /* EWM */ && \
      !defined(MCU_MC56F82726) /* EWM */ && \
      !defined(MCU_MC56F82728) /* EWM */ && \
      !defined(MCU_MC56F82733) /* EWM */ && \
      !defined(MCU_MC56F82736) /* EWM */ && \
      !defined(MCU_MC56F82738) /* EWM */ && \
      !defined(MCU_MC56F82743) /* EWM */ && \
      !defined(MCU_MC56F82746) /* EWM */ && \
      !defined(MCU_MC56F82748) /* EWM */
  // Unsupported MCU is active
  #error EWM PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Service constants */
#define EWM_PDD_KEY_1 0xB4U                      /**< First key */
#define EWM_PDD_KEY_2 0x2CU                      /**< Second key */

/* Clock source constants. */
#define EWM_PDD_SOURCE_ROSC_8M   0U              /**< Relaxation oscillator clock */
#define EWM_PDD_SOURCE_XTAL_OSC  0x1U            /**< Crystal oscillator clock */
#define EWM_PDD_SOURCE_BUS_CLK   0x2U            /**< IP bus clock */
#define EWM_PDD_SOURCE_ROSC_200K 0x3U            /**< Low speed oscillator clock */


/* ----------------------------------------------------------------------------
   -- GetEnableDeviceStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of EWM device.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_GetEnableDeviceStatus(deviceID);
 *      @endcode
 */
#define EWM_PDD_GetEnableDeviceStatus(PeripheralBase) ( \
    PeriphGetBitMask16(EWM_CTRL_REG(PeripheralBase), EWM_CTRL_EWMEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SelectClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects clock source.
 * @param PeripheralBase Peripheral base address.
 * @param Source New value of the source. This parameter is of "Clock source
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CLKCTRL.
 * @par Example:
 *      @code
 *      EWM_PDD_SelectClockSource(deviceID, EWM_PDD_SOURCE_ROSC_8M);
 *      @endcode
 */
#define EWM_PDD_SelectClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     EWM_CLKCTRL_REG(PeripheralBase), \
     EWM_CLKCTRL_CLKSEL_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescaler
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets prescale value.
 * @param PeripheralBase Peripheral base address.
 * @param Value New value of the prescaler. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CLKPRESCALER.
 * @par Example:
 *      @code
 *      EWM_PDD_SetPrescaler(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_SetPrescaler(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     EWM_CLKPRESCALER_REG(PeripheralBase), \
     EWM_CLKPRESCALER_CLK_DIV_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the control register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CTRL.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define EWM_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(EWM_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteServiceReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the service register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Service constant. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_SERV.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteServiceReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteServiceReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_SERV_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCompareLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the compare low register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the compare low register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CMPL.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteCompareLowReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteCompareLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_CMPL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCompareLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the compare low register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CMPL.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_ReadCompareLowReg(deviceID);
 *      @endcode
 */
#define EWM_PDD_ReadCompareLowReg(PeripheralBase) ( \
    PeriphReadReg16(EWM_CMPL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCompareHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the compare high register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the compare high register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CMPH.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteCompareHighReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteCompareHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_CMPH_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCompareHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the compare high register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CMPH.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_ReadCompareHighReg(deviceID);
 *      @endcode
 */
#define EWM_PDD_ReadCompareHighReg(PeripheralBase) ( \
    PeriphReadReg16(EWM_CMPH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteClockControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the clock control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the clock control register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CLKCTRL.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteClockControlReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteClockControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_CLKCTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadClockControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the clock control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CLKCTRL.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_ReadClockControlReg(deviceID);
 *      @endcode
 */
#define EWM_PDD_ReadClockControlReg(PeripheralBase) ( \
    PeriphReadReg16(EWM_CLKCTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteClockPrescalerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the clock prescaler register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the clock prescaler register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: EWM_CLKPRESCALER.
 * @par Example:
 *      @code
 *      EWM_PDD_WriteClockPrescalerReg(deviceID, 1);
 *      @endcode
 */
#define EWM_PDD_WriteClockPrescalerReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(EWM_CLKPRESCALER_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadClockPrescalerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the clock prescaler register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: EWM_CLKPRESCALER.
 * @par Example:
 *      @code
 *      uint16_t result = EWM_PDD_ReadClockPrescalerReg(deviceID);
 *      @endcode
 */
#define EWM_PDD_ReadClockPrescalerReg(PeripheralBase) ( \
    PeriphReadReg16(EWM_CLKPRESCALER_REG(PeripheralBase)) \
  )
#endif  /* #if defined(EWM_PDD_H_) */

/* EWM_PDD.h, eof. */
