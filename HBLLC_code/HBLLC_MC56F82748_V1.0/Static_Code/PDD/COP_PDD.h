/*
  PDD layer implementation for peripheral type COP
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(COP_PDD_H_)
#define COP_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error COP PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* COP */ && \
      !defined(MCU_MC56F82316) /* COP */ && \
      !defined(MCU_MC56F82723) /* COP */ && \
      !defined(MCU_MC56F82726) /* COP */ && \
      !defined(MCU_MC56F82728) /* COP */ && \
      !defined(MCU_MC56F82733) /* COP */ && \
      !defined(MCU_MC56F82736) /* COP */ && \
      !defined(MCU_MC56F82738) /* COP */ && \
      !defined(MCU_MC56F82743) /* COP */ && \
      !defined(MCU_MC56F82746) /* COP */ && \
      !defined(MCU_MC56F82748) /* COP */
  // Unsupported MCU is active
  #error COP PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Prescaler constants */
#define COP_PDD_DIVIDE_1    0U                   /**< 1 */
#define COP_PDD_DIVIDE_16   0x1U                 /**< 16 */
#define COP_PDD_DIVIDE_256  0x2U                 /**< 256 */
#define COP_PDD_DIVIDE_1024 0x3U                 /**< 1024 */

/* Service constants */
#define COP_PDD_KEY_1 0x5555U                    /**< First key */
#define COP_PDD_KEY_2 0xAAAAU                    /**< Second key */

/* Clock source constants. */
#define COP_PDD_SOURCE_ROSC_8M   0U              /**< Relaxation oscillator clock */
#define COP_PDD_SOURCE_XTAL_OSC  0x20U           /**< Crystal oscillator clock */
#define COP_PDD_SOURCE_BUS_CLK   0x40U           /**< IP bus clock */
#define COP_PDD_SOURCE_ROSC_200K 0x60U           /**< Low speed oscillator clock */


/* ----------------------------------------------------------------------------
   -- EnableWriteProtect
   ---------------------------------------------------------------------------- */

/**
 * @brief No further updates allowed to COP registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableWriteProtect(deviceID);
 *      @endcode
 */
#define COP_PDD_EnableWriteProtect(PeripheralBase) ( \
    PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CWP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetWriteProtectStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns zero if COP registers are writeable.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_GetWriteProtectStatus(deviceID);
 *      @endcode
 */
#define COP_PDD_GetWriteProtectStatus(PeripheralBase) ( \
    PeriphGetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CWP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the COP device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of COP device. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define COP_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CEN_MASK)) : ( \
      PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEnableDeviceStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of the COP device.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_GetEnableDeviceStatus(deviceID);
 *      @endcode
 */
#define COP_PDD_GetEnableDeviceStatus(PeripheralBase) ( \
    PeriphGetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWaitMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the COP device in wait mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of COP device in wait mode. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableWaitMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define COP_PDD_EnableWaitMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CWEN_MASK)) : ( \
      PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CWEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableStopMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the COP device in stop mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of COP device in stop mode. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableStopMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define COP_PDD_EnableStopMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CSEN_MASK)) : ( \
      PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CSEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLossOfReference
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the operation of the COP loss of reference counter.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of reference counter. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableLossOfReference(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define COP_PDD_EnableLossOfReference(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CLOREN_MASK)) : ( \
      PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_CLOREN_MASK)) \
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
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_SelectClockSource(deviceID, COP_PDD_SOURCE_ROSC_8M);
 *      @endcode
 */
#define COP_PDD_SelectClockSource(PeripheralBase, Source) ( \
    PeriphSetBits16( \
     COP_CTRL_REG(PeripheralBase), \
     COP_CTRL_CLKSEL_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt mask.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_GetInterruptMask(deviceID);
 *      @endcode
 */
#define COP_PDD_GetInterruptMask(PeripheralBase) ( \
    PeriphGetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_INTEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the COP interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_EnableInterrupt(deviceID);
 *      @endcode
 */
#define COP_PDD_EnableInterrupt(PeripheralBase) ( \
    PeriphSetBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_INTEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the COP interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_DisableInterrupt(deviceID);
 *      @endcode
 */
#define COP_PDD_DisableInterrupt(PeripheralBase) ( \
    PeriphClearBitMask16(COP_CTRL_REG(PeripheralBase), COP_CTRL_INTEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescaler
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets prescale value.
 * @param PeripheralBase Peripheral base address.
 * @param Prescaler New value of the prescaler. This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_SetPrescaler(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_SetPrescaler(PeripheralBase, Prescaler) ( \
    PeriphSetBits16( \
     COP_CTRL_REG(PeripheralBase), \
     COP_CTRL_PSS_MASK, \
     (uint16_t)((uint16_t)(Prescaler) << COP_CTRL_PSS_SHIFT) \
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
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      COP_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(COP_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define COP_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(COP_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTimeoutReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the timeout register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the timeout register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_TOUT.
 * @par Example:
 *      @code
 *      COP_PDD_WriteTimeoutReg(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_WriteTimeoutReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(COP_TOUT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTimeoutReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the timeout register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_TOUT.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_ReadTimeoutReg(deviceID);
 *      @endcode
 */
#define COP_PDD_ReadTimeoutReg(PeripheralBase) ( \
    PeriphReadReg16(COP_TOUT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteServiceReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the service COP register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Service constant. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_CNTR.
 * @par Example:
 *      @code
 *      COP_PDD_WriteServiceReg(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_WriteServiceReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(COP_CNTR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the counter register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_CNTR.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_ReadCounterReg(deviceID);
 *      @endcode
 */
#define COP_PDD_ReadCounterReg(PeripheralBase) ( \
    PeriphReadReg16(COP_CNTR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the interrupt value register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the interrupt value register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_INTVAL.
 * @par Example:
 *      @code
 *      COP_PDD_WriteInterruptValueReg(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_WriteInterruptValueReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(COP_INTVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt value register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_INTVAL.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_ReadInterruptValueReg(deviceID);
 *      @endcode
 */
#define COP_PDD_ReadInterruptValueReg(PeripheralBase) ( \
    PeriphReadReg16(COP_INTVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteWindowTimeoutReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the window timeout register.
 * @param PeripheralBase Peripheral base address.
 * @param Value New content of the window timeout register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COP_TOUT.
 * @par Example:
 *      @code
 *      COP_PDD_WriteWindowTimeoutReg(deviceID, 1);
 *      @endcode
 */
#define COP_PDD_WriteWindowTimeoutReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(COP_TOUT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadWindowTimeoutReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the window timeout register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COP_WINDOW.
 * @par Example:
 *      @code
 *      uint16_t result = COP_PDD_ReadWindowTimeoutReg(deviceID);
 *      @endcode
 */
#define COP_PDD_ReadWindowTimeoutReg(PeripheralBase) ( \
    PeriphReadReg16(COP_WINDOW_REG(PeripheralBase)) \
  )
#endif  /* #if defined(COP_PDD_H_) */

/* COP_PDD.h, eof. */
