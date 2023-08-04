/*
  PDD layer implementation for peripheral type PIT
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(PIT_PDD_H_)
#define PIT_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error PIT PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82316) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82723) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82726) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82728) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82733) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82736) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82738) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82743) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82746) /* PIT0, PIT1 */ && \
      !defined(MCU_MC56F82748) /* PIT0, PIT1 */
  // Unsupported MCU is active
  #error PIT PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Prescaler constants */
#define PIT_PDD_DIVIDE_1     0U                  /**< 1 */
#define PIT_PDD_DIVIDE_2     0x1U                /**< 2 */
#define PIT_PDD_DIVIDE_4     0x2U                /**< 4 */
#define PIT_PDD_DIVIDE_8     0x3U                /**< 8 */
#define PIT_PDD_DIVIDE_16    0x4U                /**< 16 */
#define PIT_PDD_DIVIDE_32    0x5U                /**< 32 */
#define PIT_PDD_DIVIDE_64    0x6U                /**< 64 */
#define PIT_PDD_DIVIDE_128   0x7U                /**< 128 */
#define PIT_PDD_DIVIDE_256   0x8U                /**< 256 */
#define PIT_PDD_DIVIDE_512   0x9U                /**< 512 */
#define PIT_PDD_DIVIDE_1024  0xAU                /**< 1024 */
#define PIT_PDD_DIVIDE_2048  0xBU                /**< 2048 */
#define PIT_PDD_DIVIDE_4096  0xCU                /**< 4096 */
#define PIT_PDD_DIVIDE_8192  0xDU                /**< 8192 */
#define PIT_PDD_DIVIDE_16384 0xEU                /**< 16384 */
#define PIT_PDD_DIVIDE_32768 0xFU                /**< 32768 */

/* Clock source constants. */
#define PIT_PDD_SOURCE_BUS_CLK   0U              /**< IP bus clock */
#define PIT_PDD_SOURCE_XTAL_OSC  0x100U          /**< Crystal oscillator clock */
#define PIT_PDD_SOURCE_ROSC_8M   0x200U          /**< Relaxation oscillator clock */
#define PIT_PDD_SOURCE_ROSC_200K 0x300U          /**< Low speed oscillator clock */


/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the PIT channel device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of PIT channel. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define PIT_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PIT_CTRL_REG(PeripheralBase), \
       PIT_CTRL_CNT_EN_MASK, \
       PIT_CTRL_PRF_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PIT_CTRL_REG(PeripheralBase), \
       (uint16_t)(PIT_CTRL_CNT_EN_MASK | PIT_CTRL_PRF_MASK) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetEnableDeviceStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of PIT channel device.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_GetEnableDeviceStatus(deviceID);
 *      @endcode
 */
#define PIT_PDD_GetEnableDeviceStatus(PeripheralBase) ( \
    PeriphGetBitMask16(PIT_CTRL_REG(PeripheralBase), PIT_CTRL_CNT_EN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the PIT interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_EnableInterrupt(deviceID);
 *      @endcode
 */
#define PIT_PDD_EnableInterrupt(PeripheralBase) ( \
    PeriphSetBitMask16( \
     PIT_CTRL_REG(PeripheralBase), \
     (uint16_t)(PIT_CTRL_PRIE_MASK | PIT_CTRL_PRF_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the PIT interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_DisableInterrupt(deviceID);
 *      @endcode
 */
#define PIT_PDD_DisableInterrupt(PeripheralBase) ( \
    PeriphSetBits16( \
     PIT_CTRL_REG(PeripheralBase), \
     PIT_CTRL_PRIE_MASK, \
     PIT_CTRL_PRF_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt mask.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_GetInterruptMask(deviceID);
 *      @endcode
 */
#define PIT_PDD_GetInterruptMask(PeripheralBase) ( \
    PeriphGetBitMask16(PIT_CTRL_REG(PeripheralBase), PIT_CTRL_PRIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_GetInterruptFlag(deviceID);
 *      @endcode
 */
#define PIT_PDD_GetInterruptFlag(PeripheralBase) ( \
    PeriphGetBitMask16(PIT_CTRL_REG(PeripheralBase), PIT_CTRL_PRF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears PIT interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_ClearInterruptFlag(deviceID);
 *      @endcode
 */
#define PIT_PDD_ClearInterruptFlag(PeripheralBase) ( \
    PeriphClearBitMask16(PIT_CTRL_REG(PeripheralBase), PIT_CTRL_PRF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescaler
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets prescale value.
 * @param PeripheralBase Peripheral base address.
 * @param Prescaler New value of the prescaler. This parameter is a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_SetPrescaler(deviceID, 1);
 *      @endcode
 */
#define PIT_PDD_SetPrescaler(PeripheralBase, Prescaler) ( \
    PeriphSetBits16( \
     PIT_CTRL_REG(PeripheralBase), \
     PIT_CTRL_PRESCALER_MASK, \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(Prescaler) << PIT_CTRL_PRESCALER_SHIFT)) | ( \
      PIT_CTRL_PRF_MASK)) \
    ) \
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
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_SelectClockSource(deviceID, PIT_PDD_SOURCE_BUS_CLK);
 *      @endcode
 */
#define PIT_PDD_SelectClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     PIT_CTRL_REG(PeripheralBase), \
     PIT_CTRL_CLKSEL_MASK, \
     (uint16_t)((uint16_t)(Source) | PIT_CTRL_PRF_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts this PIT module in slave mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of PIT device in slave mode. This parameter is
 *        of "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_EnableSlaveMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define PIT_PDD_EnableSlaveMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PIT_CTRL_REG(PeripheralBase), \
       PIT_CTRL_SLAVE_MASK, \
       PIT_CTRL_PRF_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PIT_CTRL_REG(PeripheralBase), \
       (uint16_t)(PIT_CTRL_SLAVE_MASK | PIT_CTRL_PRF_MASK) \
      )) \
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
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define PIT_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PIT_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_CTRL, PIT1_CTRL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define PIT_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(PIT_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteModuloReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the modulo register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the modulo register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PIT0_MOD, PIT1_MOD
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      PIT_PDD_WriteModuloReg(deviceID, 1);
 *      @endcode
 */
#define PIT_PDD_WriteModuloReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PIT_MOD_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadModuloReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the modulo register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_MOD, PIT1_MOD
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_ReadModuloReg(deviceID);
 *      @endcode
 */
#define PIT_PDD_ReadModuloReg(PeripheralBase) ( \
    PeriphReadReg16(PIT_MOD_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the counter register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PIT0_CNTR, PIT1_CNTR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = PIT_PDD_ReadCounterReg(deviceID);
 *      @endcode
 */
#define PIT_PDD_ReadCounterReg(PeripheralBase) ( \
    PeriphReadReg16(PIT_CNTR_REG(PeripheralBase)) \
  )
#endif  /* #if defined(PIT_PDD_H_) */

/* PIT_PDD.h, eof. */
