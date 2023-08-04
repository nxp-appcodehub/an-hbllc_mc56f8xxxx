/*
  PDD layer implementation for peripheral type INTC
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(INTC_PDD_H_)
#define INTC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error INTC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* INTC */ && \
      !defined(MCU_MC56F82316) /* INTC */ && \
      !defined(MCU_MC56F82723) /* INTC */ && \
      !defined(MCU_MC56F82726) /* INTC */ && \
      !defined(MCU_MC56F82728) /* INTC */ && \
      !defined(MCU_MC56F82733) /* INTC */ && \
      !defined(MCU_MC56F82736) /* INTC */ && \
      !defined(MCU_MC56F82738) /* INTC */ && \
      !defined(MCU_MC56F82743) /* INTC */ && \
      !defined(MCU_MC56F82746) /* INTC */ && \
      !defined(MCU_MC56F82748) /* INTC */
  // Unsupported MCU is active
  #error INTC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
   -- SetPriorityVector109
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EWM indicate interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector109(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector109(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR12_REG(PeripheralBase), \
     INTC_IPR12_EWM_INT_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR12_EWM_INT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector109
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EWM indicate interrupt. Returned
 * value has the following meaning: Returned value = 0: Disabled; Returned value
 * = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector109(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector109(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR12_REG(PeripheralBase), INTC_IPR12_EWM_INT_MASK)) >> ( \
     INTC_IPR12_EWM_INT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector108
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the COP warning interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector108(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector108(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR12_REG(PeripheralBase), \
     INTC_IPR12_COP_INT_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR12_COP_INT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector108
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the COP warning interrupt. Returned
 * value has the following meaning: Returned value = 0: Disabled; Returned value
 * = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector108(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector108(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR12_REG(PeripheralBase), INTC_IPR12_COP_INT_MASK)) >> ( \
     INTC_IPR12_COP_INT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector107
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOA interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector107(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector107(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR12_REG(PeripheralBase), \
     INTC_IPR12_GPIOA_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR12_GPIOA_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector107
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOA interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector107(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector107(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR12_REG(PeripheralBase), INTC_IPR12_GPIOA_MASK)) >> ( \
     INTC_IPR12_GPIOA_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector106
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOB interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector106(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector106(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR12_REG(PeripheralBase), \
     INTC_IPR12_GPIOB_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR12_GPIOB_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector106
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOB interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector106(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector106(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR12_REG(PeripheralBase), INTC_IPR12_GPIOB_MASK)) >> ( \
     INTC_IPR12_GPIOB_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector105
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOC interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector105(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector105(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR12_REG(PeripheralBase), \
     INTC_IPR12_GPIOC_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector105
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOC interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector105(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector105(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR12_REG(PeripheralBase), INTC_IPR12_GPIOC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector104
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOD interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector104(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector104(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR11_REG(PeripheralBase), \
     INTC_IPR11_GPIOD_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR11_GPIOD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector104
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOD interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector104(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector104(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR11_REG(PeripheralBase), INTC_IPR11_GPIOD_MASK)) >> ( \
     INTC_IPR11_GPIOD_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector103
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOE interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector103(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector103(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR11_REG(PeripheralBase), \
     INTC_IPR11_GPIOE_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR11_GPIOE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector103
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOE interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector103(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector103(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR11_REG(PeripheralBase), INTC_IPR11_GPIOE_MASK)) >> ( \
     INTC_IPR11_GPIOE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector102
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the GPIOF interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector102(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector102(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR11_REG(PeripheralBase), \
     INTC_IPR11_GPIOF_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR11_GPIOF_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector102
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the GPIOF interrupt. Returned value
 * has the following meaning: Returned value = 0: Disabled; Returned value = 1:
 * Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector102(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector102(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR11_REG(PeripheralBase), INTC_IPR11_GPIOF_MASK)) >> ( \
     INTC_IPR11_GPIOF_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector95
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PIT0 roll over interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector95(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector95(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_PIT0_ROLLOVR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_PIT0_ROLLOVR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector95
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PIT0 roll over interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector95(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector95(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_PIT0_ROLLOVR_MASK)) >> ( \
     INTC_IPR10_PIT0_ROLLOVR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector94
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PIT1 roll over interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector94(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector94(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_PIT1_ROLLOVR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_PIT1_ROLLOVR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector94
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PIT1 roll over interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector94(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector94(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_PIT1_ROLLOVR_MASK)) >> ( \
     INTC_IPR10_PIT1_ROLLOVR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector93
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the CMPA rising edge / falling edge
 * interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector93(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector93(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_CMPA_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_CMPA_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector93
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the CMPA rising edge / falling edge
 * interrupts. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector93(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector93(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_CMPA_MASK)) >> ( \
     INTC_IPR10_CMPA_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector92
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the CMPB rising edge / falling edge
 * interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector92(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector92(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_CMPB_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_CMPB_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector92
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the CMPB rising edge / falling edge
 * interrupts. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector92(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector92(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_CMPB_MASK)) >> ( \
     INTC_IPR10_CMPB_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector89
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the FTFA command complete interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector89(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector89(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_FTFA_CC_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector89
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the FTFA command complete interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector89(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector89(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_FTFA_CC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector88
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the FTFA access error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector88(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector88(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_FTFA_RDCOL_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_FTFA_RDCOL_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector88
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the FTFA access error interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector88(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector88(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_FTFA_RDCOL_MASK)) >> ( \
     INTC_IPR9_FTFA_RDCOL_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector87
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 0 compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector87(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector87(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_CMP0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_CMP0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector87
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 0 compare
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector87(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector87(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_CMP0_MASK)) >> ( \
     INTC_IPR9_PWMA_CMP0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector86
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 0 reload interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector86(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector86(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_RELOAD0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_RELOAD0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector86
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 0 reload
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector86(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector86(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_RELOAD0_MASK)) >> ( \
     INTC_IPR9_PWMA_RELOAD0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector85
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 1 compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector85(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector85(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_CMP1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_CMP1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector85
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 1 compare
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector85(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector85(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_CMP1_MASK)) >> ( \
     INTC_IPR9_PWMA_CMP1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector84
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 1 reload interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector84(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector84(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_RELOAD1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_RELOAD1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector84
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 1 reload
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector84(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector84(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_RELOAD1_MASK)) >> ( \
     INTC_IPR9_PWMA_RELOAD1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector83
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 2 compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector83(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector83(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_CMP2_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_CMP2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector83
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 2 compare
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector83(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector83(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_CMP2_MASK)) >> ( \
     INTC_IPR9_PWMA_CMP2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector82
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 2 reload interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector82(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector82(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_RELOAD2_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR9_PWMA_RELOAD2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector82
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 2 reload
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector82(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector82(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_RELOAD2_MASK)) >> ( \
     INTC_IPR9_PWMA_RELOAD2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector81
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 3 compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector81(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector81(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR9_REG(PeripheralBase), \
     INTC_IPR9_PWMA_CMP3_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector81
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 3 compare
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector81(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector81(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR9_REG(PeripheralBase), INTC_IPR9_PWMA_CMP3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector80
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA logic or'ed all submodule 0-3
 * input captures interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector80(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector80(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR8_REG(PeripheralBase), \
     INTC_IPR8_PWMA_CAP_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR8_PWMA_CAP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector80
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA logic or'ed all submodule
 * 0-3 input captures interrupts. Returned value has the following meaning:
 * Returned value = 0: Disabled; Returned value = 1: Level 0; Returned value = 2: Level
 * 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector80(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector80(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR8_REG(PeripheralBase), INTC_IPR8_PWMA_CAP_MASK)) >> ( \
     INTC_IPR8_PWMA_CAP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector79
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA submodule 3 reload interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector79(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector79(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR8_REG(PeripheralBase), \
     INTC_IPR8_PWMA_RELOAD3_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR8_PWMA_RELOAD3_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector79
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA submodule 3 reload
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector79(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector79(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR8_REG(PeripheralBase), INTC_IPR8_PWMA_RELOAD3_MASK)) >> ( \
     INTC_IPR8_PWMA_RELOAD3_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector78
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA reload error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector78(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector78(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR8_REG(PeripheralBase), \
     INTC_IPR8_PWMA_RERR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR8_PWMA_RERR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector78
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA reload error interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector78(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector78(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR8_REG(PeripheralBase), INTC_IPR8_PWMA_RERR_MASK)) >> ( \
     INTC_IPR8_PWMA_RERR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector77
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PWMA fault condition interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector77(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector77(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR8_REG(PeripheralBase), \
     INTC_IPR8_PWMA_FAULT_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR8_PWMA_FAULT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector77
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PWMA fault condition interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector77(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector77(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR8_REG(PeripheralBase), INTC_IPR8_PWMA_FAULT_MASK)) >> ( \
     INTC_IPR8_PWMA_FAULT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector62
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the IIC0 complete 1-byte transfer
 * (TCF), match of received calling address (IAAS), arbitration lost (ARBL), SMBus
 * timeout (SLTF) interrupts and stop detect interrupt for which local enable is
 * CR1[IICIE] and FLT[STOPIE].
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector62(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector62(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR6_REG(PeripheralBase), \
     INTC_IPR6_IIC0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR6_IIC0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector62
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the IIC0 complete 1-byte transfer
 * (TCF), match of received calling address (IAAS), arbitration lost (ARBL), SMBus
 * timeout (SLTF) interrupts and stop detect interrupt for which local enable is
 * CR1[IICIE] and FLT[STOPIE]. Returned value has the following meaning: Returned
 * value = 0: Disabled; Returned value = 1: Level 0; Returned value = 2: Level
 * 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector62(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector62(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR6_REG(PeripheralBase), INTC_IPR6_IIC0_MASK)) >> ( \
     INTC_IPR6_IIC0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector60
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSPI0 receiver full interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector60(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector60(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR6_REG(PeripheralBase), \
     INTC_IPR6_QSPI0_RCV_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR6_QSPI0_RCV_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector60
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSPI0 receiver full interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector60(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector60(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR6_REG(PeripheralBase), INTC_IPR6_QSPI0_RCV_MASK)) >> ( \
     INTC_IPR6_QSPI0_RCV_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector59
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSPI0 transmitter empty interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector59(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector59(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR6_REG(PeripheralBase), \
     INTC_IPR6_QSPI0_XMIT_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR6_QSPI0_XMIT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector59
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSPI0 transmitter empty
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector59(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector59(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR6_REG(PeripheralBase), INTC_IPR6_QSPI0_XMIT_MASK)) >> ( \
     INTC_IPR6_QSPI0_XMIT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector54
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI0 transmit data register empty
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector54(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector54(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI0_TDRE_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR5_QSCI0_TDRE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector54
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI0 transmit data register
 * empty interrupt. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned
 * value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector54(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector54(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI0_TDRE_MASK)) >> ( \
     INTC_IPR5_QSCI0_TDRE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector53
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI0 transmitter idle interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector53(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector53(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI0_TIDLE_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR5_QSCI0_TIDLE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector53
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI0 transmitter idle
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector53(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector53(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI0_TIDLE_MASK)) >> ( \
     INTC_IPR5_QSCI0_TIDLE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector52
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI0 receive data register full
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector52(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector52(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI0_RCV_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR5_QSCI0_RCV_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector52
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI0 receive data register full
 * interrupt. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector52(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector52(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI0_RCV_MASK)) >> ( \
     INTC_IPR5_QSCI0_RCV_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector51
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI0 Receiver Error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector51(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector51(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI0_RERR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR5_QSCI0_RERR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector51
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI0 Receiver Error interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector51(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector51(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI0_RERR_MASK)) >> ( \
     INTC_IPR5_QSCI0_RERR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector36
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the DMA0 service request interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector36(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector36(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_DMACH0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR3_DMACH0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector36
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the DMA0 service request interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector36(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector36(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_DMACH0_MASK)) >> ( \
     INTC_IPR3_DMACH0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector35
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the DMA1 service request interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector35(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector35(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_DMACH1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR3_DMACH1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector35
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the DMA1 service request interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector35(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector35(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_DMACH1_MASK)) >> ( \
     INTC_IPR3_DMACH1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector34
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the DMA2 service request interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector34(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector34(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_DMACH2_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR3_DMACH2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector34
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the DMA2 service request interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector34(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector34(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_DMACH2_MASK)) >> ( \
     INTC_IPR3_DMACH2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector33
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the DMA3 service request interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector33(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector33(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_DMACH3_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector33
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the DMA3 service request interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector33(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector33(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_DMACH3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector31
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the ADC12 zero crossing, low limit and
 * high limit interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector31(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector31(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_ADC_ERR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_ADC_ERR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector31
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the ADC12 zero crossing, low limit
 * and high limit interrupt. Returned value has the following meaning: Returned
 * value = 0: Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1;
 * Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector31(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector31(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_ADC_ERR_MASK)) >> ( \
     INTC_IPR2_ADC_ERR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector30
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the ADC12 ADC conversion complete, any
 * scan type except Converter B in non simultaneous parallel scan mode.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector30(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector30(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_ADC_CC0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_ADC_CC0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector30
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the ADC12 ADC conversion complete,
 * any scan type except Converter B in non simultaneous parallel scan mode.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector30(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector30(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_ADC_CC0_MASK)) >> ( \
     INTC_IPR2_ADC_CC0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector29
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the ADC12 conversion complete,
 * converter B in non-simultaneous parallel scan mode.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector29(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector29(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_ADC_CC1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_ADC_CC1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector29
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the ADC12 conversion complete,
 * converter B in non-simultaneous parallel scan mode. Returned value has the
 * following meaning: Returned value = 0: Disabled; Returned value = 1: Level 0; Returned
 * value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector29(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector29(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_ADC_CC1_MASK)) >> ( \
     INTC_IPR2_ADC_CC1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector28
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the TIMERA0 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2 interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector28(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector28(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_TMRA_0_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_TMRA_0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector28
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the TIMERA0 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2
 * interrupts. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector28(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector28(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_TMRA_0_MASK)) >> ( \
     INTC_IPR2_TMRA_0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector27
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the TIMERA1 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2 interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector27(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector27(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_TMRA_1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_TMRA_1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector27
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the TIMERA1 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2
 * interrupts. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector27(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector27(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_TMRA_1_MASK)) >> ( \
     INTC_IPR2_TMRA_1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector26
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the TIMERA2 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2 interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector26(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector26(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_TMRA_2_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR2_TMRA_2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector26
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the TIMERA2 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2
 * interrupts. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector26(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector26(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_TMRA_2_MASK)) >> ( \
     INTC_IPR2_TMRA_2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector25
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the TIMERA3 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2 interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector25(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector25(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR2_REG(PeripheralBase), \
     INTC_IPR2_TMRA_3_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector25
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the TIMERA3 timer compare / timer
 * overflow / input edge flag / timer compare flag 1 / timer compare flag 2
 * interrupts. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector25(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector25(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR2_REG(PeripheralBase), INTC_IPR2_TMRA_3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector20
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the OCCS PLL loss of lock 1 / PLL loss
 * of lock 0 / PLL loss of reference clock interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector20(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector20(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR1_REG(PeripheralBase), \
     INTC_IPR1_OCCS_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR1_OCCS_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector20
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the OCCS PLL loss of lock 1 / PLL
 * loss of lock 0 / PLL loss of reference clock interrupts. Returned value has the
 * following meaning: Returned value = 0: Disabled; Returned value = 1: Level 1;
 * Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector20(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector20(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR1_REG(PeripheralBase), INTC_IPR1_OCCS_MASK)) >> ( \
     INTC_IPR1_OCCS_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector19
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the PMC low voltage interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector19(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector19(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR1_REG(PeripheralBase), \
     INTC_IPR1_LVI1_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR1_LVI1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector19
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the PMC low voltage interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector19(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector19(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR1_REG(PeripheralBase), INTC_IPR1_LVI1_MASK)) >> ( \
     INTC_IPR1_LVI1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector18
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the XBARA crossbar interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector18(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector18(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR1_REG(PeripheralBase), \
     INTC_IPR1_XBARA_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR1_XBARA_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector18
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the XBARA crossbar interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector18(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector18(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR1_REG(PeripheralBase), INTC_IPR1_XBARA_MASK)) >> ( \
     INTC_IPR1_XBARA_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector11
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the core bus error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector11(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector11(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_BUS_ERR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR0_BUS_ERR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector11
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the core bus error interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector11(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector11(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_BUS_ERR_MASK)) >> ( \
     INTC_IPR0_BUS_ERR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector10
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EOnCE receive register full
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector10(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector10(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_RX_REG_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR0_RX_REG_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector10
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EOnCE receive register full
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3:
 * Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector10(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector10(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_RX_REG_MASK)) >> ( \
     INTC_IPR0_RX_REG_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector9
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EOnCE transmit register empty
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector9(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector9(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_TX_REG_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR0_TX_REG_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector9
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EOnCE transmit register empty
 * interrupt. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 1; Returned value = 2: Level 2; Returned value =
 * 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector9(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector9(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_TX_REG_MASK)) >> ( \
     INTC_IPR0_TX_REG_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector8
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EOnCE trace buffer interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector8(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector8(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_TRBUF_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR0_TRBUF_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EOnCE trace buffer interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector8(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector8(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_TRBUF_MASK)) >> ( \
     INTC_IPR0_TRBUF_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector6
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EOnCE breakpoint unit interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector6(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector6(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_BKPT_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR0_BKPT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EOnCE breakpoint unit interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level
 * 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector6(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector6(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_BKPT_MASK)) >> ( \
     INTC_IPR0_BKPT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector5
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the EOnCE step counter interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 1; Priority = 2: Level 2; Priority = 3: Level 3. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector5(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector5(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR0_REG(PeripheralBase), \
     INTC_IPR0_STPCNT_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the EOnCE step counter interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled; Returned
 * value = 1: Level 1; Returned value = 2: Level 2; Returned value = 3: Level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector5(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector5(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR0_REG(PeripheralBase), INTC_IPR0_STPCNT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetVectorBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt vector base address.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying the interrupt vector base address. This
 *        parameter is a 13-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_VBA.
 * @par Example:
 *      @code
 *      INTC_PDD_SetVectorBaseAddress(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetVectorBaseAddress(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_VBA_REG(PeripheralBase), \
     INTC_VBA_VECTOR_BASE_ADDRESS_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVectorBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt vector base address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 13-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: INTC_VBA.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_GetVectorBaseAddress(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetVectorBaseAddress(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16( \
     INTC_VBA_REG(PeripheralBase), \
     INTC_VBA_VECTOR_BASE_ADDRESS_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptNumber0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the fast interrupt 0 vector number.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the vector number. This parameter is a
 *        7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIM0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptNumber0(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptNumber0(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIM0_REG(PeripheralBase), \
     INTC_FIM0_FAST_INTERRUPT_0_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptNumber0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the fast interrupt 0 vector number.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 7-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_FIM0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetFastInterruptNumber0(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptNumber0(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16( \
     INTC_FIM0_REG(PeripheralBase), \
     INTC_FIM0_FAST_INTERRUPT_0_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the vector address. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH0, INTC_FIVAL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress0(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress0(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIVAH0_REG(PeripheralBase), \
     INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK, \
     (uint16_t)((uint32_t)(Value) >> 16U) \
    ), \
    PeriphWriteReg16(INTC_FIVAL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL0, INTC_FIVAH0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result = INTC_PDD_GetFastInterruptAddress0(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress0(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(INTC_FIVAL0_REG(PeripheralBase))) | ( \
     (uint32_t)(( \
      (uint32_t)PeriphGetBitMask16( \
       INTC_FIVAH0_REG(PeripheralBase), \
       INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK \
      )) << ( \
      16U)))) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress0Low
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the low part of the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the low part of the vector address. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAL0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress0Low(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress0Low(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress0Low
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the low part of the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL0.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_GetFastInterruptAddress0Low(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress0Low(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress0High
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the high part of the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the high part of the vector address. This
 *        parameter is a 5-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH0.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress0High(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress0High(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIVAH0_REG(PeripheralBase), \
     INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress0High
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the high part of the fast interrupt 0 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_FIVAH0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetFastInterruptAddress0High(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress0High(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16( \
     INTC_FIVAH0_REG(PeripheralBase), \
     INTC_FIVAH0_FI_0_VECTOR_ADDRESS_HIGH_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptNumber1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the fast interrupt 1 vector number.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the vector number. This parameter is a
 *        7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIM1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptNumber1(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptNumber1(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIM1_REG(PeripheralBase), \
     INTC_FIM1_FAST_INTERRUPT_1_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptNumber1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the fast interrupt 1 vector number.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 7-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_FIM1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetFastInterruptNumber1(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptNumber1(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16( \
     INTC_FIM1_REG(PeripheralBase), \
     INTC_FIM1_FAST_INTERRUPT_1_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the vector address. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH1, INTC_FIVAL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress1(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress1(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIVAH1_REG(PeripheralBase), \
     INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK, \
     (uint16_t)((uint32_t)(Value) >> 16U) \
    ), \
    PeriphWriteReg16(INTC_FIVAL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL1, INTC_FIVAH1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result = INTC_PDD_GetFastInterruptAddress1(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress1(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(INTC_FIVAL1_REG(PeripheralBase))) | ( \
     (uint32_t)(( \
      (uint32_t)PeriphGetBitMask16( \
       INTC_FIVAH1_REG(PeripheralBase), \
       INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK \
      )) << ( \
      16U)))) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress1Low
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the low part of the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the low part of the vector address. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAL1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress1Low(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress1Low(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress1Low
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the low part of the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL1.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_GetFastInterruptAddress1Low(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress1Low(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFastInterruptAddress1High
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the high part of the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying the high part of the vector address. This
 *        parameter is a 5-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH1.
 * @par Example:
 *      @code
 *      INTC_PDD_SetFastInterruptAddress1High(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetFastInterruptAddress1High(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     INTC_FIVAH1_REG(PeripheralBase), \
     INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastInterruptAddress1High
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the high part of the fast interrupt 1 vector address.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_FIVAH1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetFastInterruptAddress1High(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetFastInterruptAddress1High(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16( \
     INTC_FIVAH1_REG(PeripheralBase), \
     INTC_FIVAH1_FI_1_VECTOR_ADDRESS_HIGH_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq2(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq2(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq3(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq3(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq4(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq4(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq5(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq5(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq6
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq6(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq6(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq7
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 7.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq7(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq7(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 8.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq8(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq8(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq9
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 9.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq9(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq9(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq10
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 10.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq10(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq10(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq11
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 11.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq11(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq11(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq12
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 12.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq12(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq12(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq13
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 13.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq13(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq13(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq14
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 14.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq14(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq14(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq15
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 15.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq15(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq15(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq16
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 16.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq16(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq16(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) & INTC_IRQP0_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq17
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 17.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq17(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq17(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq18
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 18.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq18(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq18(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq19
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 19.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq19(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq19(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq20
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 20.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq20(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq20(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq21
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 21.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq21(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq21(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq22
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 22.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq22(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq22(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq23
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 23.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq23(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq23(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq24
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 24.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq24(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq24(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq25
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 25.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq25(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq25(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq26
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 26.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq26(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq26(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq27
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 27.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq27(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq27(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq28
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 28.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq28(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq28(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq29
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 29.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq29(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq29(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq30
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 30.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq30(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq30(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq31
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 31.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq31(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq31(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq32
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 32.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq32(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq32(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) & INTC_IRQP1_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq33
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 33.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq33(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq33(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq34
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 34.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq34(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq34(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq35
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 35.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq35(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq35(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq36
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 36.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq36(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq36(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq37
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 37.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq37(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq37(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq38
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 38.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq38(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq38(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq39
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 39.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq39(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq39(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq40
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 40.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq40(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq40(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq41
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 41.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq41(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq41(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq42
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 42.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq42(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq42(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq43
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 43.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq43(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq43(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq44
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 44.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq44(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq44(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq45
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 45.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq45(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq45(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq46
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 46.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq46(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq46(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq47
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 47.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq47(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq47(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq48
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 48.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq48(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq48(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) & INTC_IRQP2_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq49
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 49.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq49(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq49(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq50
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 50.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq50(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq50(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq51
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 51.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq51(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq51(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq52
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 52.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq52(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq52(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq53
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 53.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq53(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq53(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq54
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 54.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq54(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq54(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq55
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 55.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq55(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq55(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq56
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 56.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq56(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq56(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq57
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 57.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq57(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq57(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq58
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 58.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq58(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq58(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq59
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 59.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq59(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq59(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq60
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 60.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq60(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq60(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq61
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 61.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq61(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq61(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq62
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 62.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq62(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq62(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq63
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 63.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq63(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq63(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq64
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 64.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq64(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq64(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) & INTC_IRQP3_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq65
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 65.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq65(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq65(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq66
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 66.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq66(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq66(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq67
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 67.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq67(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq67(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq68
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 68.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq68(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq68(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq69
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 69.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq69(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq69(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq70
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 70.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq70(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq70(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq71
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 71.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq71(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq71(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq72
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 72.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq72(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq72(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq73
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 73.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq73(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq73(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq74
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 74.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq74(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq74(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq75
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 75.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq75(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq75(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq76
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 76.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq76(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq76(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq77
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 77.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq77(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq77(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq78
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 78.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq78(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq78(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq79
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 79.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq79(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq79(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq80
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 80.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq80(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq80(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) & INTC_IRQP4_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq81
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 81.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq81(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq81(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq82
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 82.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq82(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq82(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq83
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 83.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq83(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq83(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq84
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 84.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq84(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq84(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq85
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 85.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq85(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq85(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq86
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 86.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq86(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq86(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq87
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 87.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq87(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq87(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq88
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 88.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq88(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq88(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq89
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 89.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq89(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq89(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq90
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 90.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq90(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq90(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq91
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 91.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq91(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq91(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq92
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 92.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq92(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq92(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq93
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 93.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq93(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq93(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq94
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 94.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq94(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq94(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq95
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 95.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq95(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq95(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(14))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq96
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 96.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq96(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq96(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) & INTC_IRQP5_PENDING(15))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq97
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 97.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq97(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq97(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(0))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq98
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 98.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq98(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq98(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(1))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq99
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 99.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq99(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq99(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(2))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq100
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 100.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq100(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq100(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(3))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq101
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 101.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq101(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq101(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(4))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq102
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 102.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq102(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq102(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(5))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq103
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 103.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq103(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq103(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(6))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq104
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 104.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq104(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq104(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(7))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq105
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 105.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq105(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq105(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(8))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq106
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 106.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq106(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq106(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(9))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq107
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 107.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq107(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq107(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(10))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq108
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 108.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq108(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq108(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(11))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq109
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 109.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq109(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq109(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(12))) \
  )

/* ----------------------------------------------------------------------------
   -- GetPendingIrq110
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the IRQ is pending for vector number 110.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPendingIrq110(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPendingIrq110(PeripheralBase) ( \
    (uint16_t)(~(uint16_t)(PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) & INTC_IRQP6_PENDING(13))) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptSentToCoreFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns non-zero value if the interrupt is being sent to the core.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_GetInterruptSentToCoreFlag(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetInterruptSentToCoreFlag(PeripheralBase) ( \
    PeriphGetBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_INT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetPrioritySentToCore
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the level of the interrupt priority needed for a new IRQ to
 * interrupt the current interrupt being sent to the core. Returned value has the
 * following meaning: Returned value = 0: Required levels 0, 1, 2 or 3; Returned
 * value = 1: Required levels 1, 2 or 3; Returned value = 2: Required levels 2 or
 * 3; Returned value = 3: Required level 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPrioritySentToCore(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPrioritySentToCore(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_IPIC_MASK)) >> ( \
     INTC_CTRL_IPIC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetVectorAddressBus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns bits [7:1] of the vector address bus used at the time the last
 * IRQ was taken.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 7-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetVectorAddressBus(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetVectorAddressBus(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_VAB_MASK)) >> ( \
     INTC_CTRL_VAB_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable all interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if the interrupts are enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      INTC_PDD_EnableInterrupts(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define INTC_PDD_EnableInterrupts(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_INT_DIS_MASK)) : ( \
      PeriphClearBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_INT_DIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptsEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns all interrupts enable state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetInterruptsEnabled(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetInterruptsEnabled(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16(INTC_CTRL_REG(PeripheralBase), INTC_CTRL_INT_DIS_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority0Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR0.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority0Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority0Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority1Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority1eg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR1.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority1eg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority1eg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 2.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority2Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR2.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority2Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority2Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 3.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority3Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority3Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority3Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 4.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority4Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority4Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR4_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority4Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority4Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 5.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority5Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority5Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR5_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority5Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority5Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 6.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority6Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority6Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR6_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority6Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority6Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR6_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority8Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 8.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority8Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority8Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR8_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority8Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 8.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR8.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority8Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority8Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR8_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority9Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 9.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority9Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority9Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR9_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority9Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 9.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR9.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority9Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority9Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR9_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority10Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 10.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority10Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority10Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR10_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority10Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 10.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority10Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority10Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR10_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority11Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 11.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority11Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority11Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR11_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority11Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 11.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR11.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority11Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority11Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR11_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPriority12Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the interrupt priority register 12.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteInterruptPriority12Reg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteInterruptPriority12Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_IPR12_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPriority12Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the interrupt priority register 12.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IPR12.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadInterruptPriority12Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadInterruptPriority12Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IPR12_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteVectorBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the vector base address register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_VBA.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteVectorBaseAddressReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteVectorBaseAddressReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_VBA_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadVectorBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the vector base address register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_VBA.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadVectorBaseAddressReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadVectorBaseAddressReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_VBA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt0MatchReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 0 match register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIM0.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt0MatchReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt0MatchReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIM0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt0MatchReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 0 match register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIM0.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadFastInterrupt0MatchReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt0MatchReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIM0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt0VectorAddressLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 0 vector address low register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAL0.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt0VectorAddressLowReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt0VectorAddressLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt0VectorAddressLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 0 vector address low
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      INTC_PDD_ReadFastInterrupt0VectorAddressLowReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt0VectorAddressLowReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt0VectorAddressHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 0 vector address high register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH0.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt0VectorAddressHighReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt0VectorAddressHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAH0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt0VectorAddressHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 0 vector address high
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAH0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      INTC_PDD_ReadFastInterrupt0VectorAddressHighReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt0VectorAddressHighReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAH0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt1MatchReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 1 match register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIM1.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt1MatchReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt1MatchReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIM1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt1MatchReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 1 match register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIM1.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadFastInterrupt1MatchReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt1MatchReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIM1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt1VectorAddressLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 1 vector address low register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAL1.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt1VectorAddressLowReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt1VectorAddressLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt1VectorAddressLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 1 vector address low
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAL1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      INTC_PDD_ReadFastInterrupt1VectorAddressLowReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt1VectorAddressLowReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFastInterrupt1VectorAddressHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the fast interrupt 1 vector address high register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_FIVAH1.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteFastInterrupt1VectorAddressHighReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteFastInterrupt1VectorAddressHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_FIVAH1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFastInterrupt1VectorAddressHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the fast interrupt 1 vector address high
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_FIVAH1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      INTC_PDD_ReadFastInterrupt1VectorAddressHighReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadFastInterrupt1VectorAddressHighReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_FIVAH1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP0.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending0Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending0Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP1.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending1Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending1Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP2.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending2Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending2Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP3.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending3Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending3Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP4.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending4Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending4Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP5.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending5Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending5Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIrqPending6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the IRQ pending register 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_IRQP6.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadIrqPending6Reg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadIrqPending6Reg(PeripheralBase) ( \
    PeriphReadReg16(INTC_IRQP6_REG(PeripheralBase)) \
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
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      INTC_PDD_WriteControlReg(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_WriteControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(INTC_CTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTC_CTRL.
 * @par Example:
 *      @code
 *      uint16_t result = INTC_PDD_ReadControlReg(deviceID);
 *      @endcode
 */
#define INTC_PDD_ReadControlReg(PeripheralBase) ( \
    PeriphReadReg16(INTC_CTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector91
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the CMPC rising edge / falling edge
 * interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector91(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector91(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_CMPC_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_CMPC_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector91
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the CMPC rising edge / falling edge
 * interrupts. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector91(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector91(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_CMPC_MASK)) >> ( \
     INTC_IPR10_CMPC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector50
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI1 transmit data register empty
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector50(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector50(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI1_TDRE_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR5_QSCI1_TDRE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector50
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI1 transmit data register
 * empty interrupt. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned
 * value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector50(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector50(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI1_TDRE_MASK)) >> ( \
     INTC_IPR5_QSCI1_TDRE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector49
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI1 transmitter idle interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector49(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector49(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR5_REG(PeripheralBase), \
     INTC_IPR5_QSCI1_TIDLE_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector49
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI1 transmitter idle
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR5.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector49(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector49(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR5_REG(PeripheralBase), INTC_IPR5_QSCI1_TIDLE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector48
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI1 receive data register full
 * interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector48(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector48(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR4_REG(PeripheralBase), \
     INTC_IPR4_QSCI1_RCV_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR4_QSCI1_RCV_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector48
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI1 receive data register full
 * interrupt. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector48(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector48(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR4_REG(PeripheralBase), INTC_IPR4_QSCI1_RCV_MASK)) >> ( \
     INTC_IPR4_QSCI1_RCV_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector47
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSCI1 receiver error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector47(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector47(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR4_REG(PeripheralBase), \
     INTC_IPR4_QSCI1_RERR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR4_QSCI1_RERR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector47
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSCI1 receiver error interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector47(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector47(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR4_REG(PeripheralBase), INTC_IPR4_QSCI1_RERR_MASK)) >> ( \
     INTC_IPR4_QSCI1_RERR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector42
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the MSCAN Wakeup interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector42(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector42(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR4_REG(PeripheralBase), \
     INTC_IPR4_CAN_WAKEUP_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR4_CAN_WAKEUP_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector42
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the MSCAN Wakeup interrupt. Returned
 * value has the following meaning: Returned value = 0: Disabled; Returned value
 * = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector42(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector42(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR4_REG(PeripheralBase), INTC_IPR4_CAN_WAKEUP_MASK)) >> ( \
     INTC_IPR4_CAN_WAKEUP_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector41
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the MSCAN receive warning interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector41(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector41(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR4_REG(PeripheralBase), \
     INTC_IPR4_CAN_RX_WARN_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector41
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the MSCAN receive warning interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level
 * 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR4.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector41(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector41(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR4_REG(PeripheralBase), INTC_IPR4_CAN_RX_WARN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector40
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the MSCAN transmit warning interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector40(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector40(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_CAN_TX_WARN_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR3_CAN_TX_WARN_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector40
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the MSCAN transmit warning
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector40(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector40(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_CAN_TX_WARN_MASK)) >> ( \
     INTC_IPR3_CAN_TX_WARN_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector39
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the MSCAN error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector39(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector39(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR3_REG(PeripheralBase), \
     INTC_IPR3_CAN_ERROR_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR3_CAN_ERROR_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector39
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the MSCAN error interrupt. Returned
 * value has the following meaning: Returned value = 0: Disabled; Returned value
 * = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR3.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector39(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector39(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR3_REG(PeripheralBase), INTC_IPR3_CAN_ERROR_MASK)) >> ( \
     INTC_IPR3_CAN_ERROR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector90
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the CMPD rising edge / falling edge
 * interrupts.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector90(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector90(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR10_REG(PeripheralBase), \
     INTC_IPR10_CMPD_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR10_CMPD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector90
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the CMPD rising edge / falling edge
 * interrupts. Returned value has the following meaning: Returned value = 0:
 * Disabled; Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value
 * = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR10.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector90(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector90(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR10_REG(PeripheralBase), INTC_IPR10_CMPD_MASK)) >> ( \
     INTC_IPR10_CMPD_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector58
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSPI1 receiver full interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector58(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector58(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR6_REG(PeripheralBase), \
     INTC_IPR6_QSPI1_RCV_MASK, \
     (uint16_t)((uint16_t)(Priority) << INTC_IPR6_QSPI1_RCV_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector58
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSPI1 receiver full interrupt.
 * Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3: Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector58(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector58(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(INTC_IPR6_REG(PeripheralBase), INTC_IPR6_QSPI1_RCV_MASK)) >> ( \
     INTC_IPR6_QSPI1_RCV_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPriorityVector57
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the interrupt priority of the QSPI1 transmitter empty interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Priority Parameter specifying interrupt vector priority. Parameter
 *        values have the following meaning: Priority = 0: Disabled; Priority = 1:
 *        Level 0; Priority = 2: Level 1; Priority = 3: Level 2. This parameter is
 *        a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      INTC_PDD_SetPriorityVector57(deviceID, 1);
 *      @endcode
 */
#define INTC_PDD_SetPriorityVector57(PeripheralBase, Priority) ( \
    PeriphSetBits16( \
     INTC_IPR6_REG(PeripheralBase), \
     INTC_IPR6_QSPI1_XMIT_MASK, \
     (uint16_t)(Priority) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPriorityVector57
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the interrupt priority of the QSPI1 transmitter empty
 * interrupt. Returned value has the following meaning: Returned value = 0: Disabled;
 * Returned value = 1: Level 0; Returned value = 2: Level 1; Returned value = 3:
 * Level 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: INTC_IPR6.
 * @par Example:
 *      @code
 *      uint8_t result = INTC_PDD_GetPriorityVector57(deviceID);
 *      @endcode
 */
#define INTC_PDD_GetPriorityVector57(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(INTC_IPR6_REG(PeripheralBase), INTC_IPR6_QSPI1_XMIT_MASK) \
  )
#endif  /* #if defined(INTC_PDD_H_) */

/* INTC_PDD.h, eof. */
