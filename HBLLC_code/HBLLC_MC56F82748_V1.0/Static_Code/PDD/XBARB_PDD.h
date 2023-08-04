/*
  PDD layer implementation for peripheral type XBARB
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(XBARB_PDD_H_)
#define XBARB_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error XBARB PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* XBARB */ && \
      !defined(MCU_MC56F82316) /* XBARB */ && \
      !defined(MCU_MC56F82723) /* XBARB */ && \
      !defined(MCU_MC56F82726) /* XBARB */ && \
      !defined(MCU_MC56F82728) /* XBARB */ && \
      !defined(MCU_MC56F82733) /* XBARB */ && \
      !defined(MCU_MC56F82736) /* XBARB */ && \
      !defined(MCU_MC56F82738) /* XBARB */ && \
      !defined(MCU_MC56F82743) /* XBARB */ && \
      !defined(MCU_MC56F82746) /* XBARB */ && \
      !defined(MCU_MC56F82748) /* XBARB */
  // Unsupported MCU is active
  #error XBARB PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* XBAR input constants. They are used to specify signal connected with XBAR
   output. */
#define XBARB_PDD_SCI0_TXD  0U                   /**< Selects SCI0 TXD as input. */
#define XBARB_PDD_SCI1_TXD  0x1U                 /**< Selects SCI1 TXD as input. */
#define XBARB_PDD_XB_IN2    0x2U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN3    0x3U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN4    0x4U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN5    0x5U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN6    0x6U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN7    0x7U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN8    0x8U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_XB_IN9    0x9U                 /**< Selects Package Pin as input. */
#define XBARB_PDD_CMPA_OUT  0xAU                 /**< Selects Comparator A Output as input. */
#define XBARB_PDD_CMPB_OUT  0xBU                 /**< Selects Comparator B Output as input. */
#define XBARB_PDD_CMPC_OUT  0xCU                 /**< Selects Comparator C Output as input. */
#define XBARB_PDD_CMPD_OUT  0xDU                 /**< Selects Comparator D Output as input. */
#define XBARB_PDD_TA0_OUT   0xEU                 /**< Selects Timer A0 OUT as input. */
#define XBARB_PDD_TA1_OUT   0xFU                 /**< Selects Timer A1 OUT as input. */
#define XBARB_PDD_TA2_OUT   0x10U                /**< Selects Timer A2 OUT as input. */
#define XBARB_PDD_TA3_OUT   0x11U                /**< Selects Timer A3 OUT as input. */
#define XBARB_PDD_PWM0_TRG0 0x12U                /**< Selects PWM0 Trigger 0 as input. */
#define XBARB_PDD_PWM0_TRG1 0x13U                /**< Selects PWM0 Trigger 1 as input. */
#define XBARB_PDD_PWM1_TRG0 0x14U                /**< Selects PWM1 Trigger 0 as input. */
#define XBARB_PDD_PWM1_TRG1 0x15U                /**< Selects PWM1 Trigger 1 as input. */
#define XBARB_PDD_PWM2_TRG0 0x16U                /**< Selects PWM2 Trigger 0 as input. */
#define XBARB_PDD_PWM2_TRG1 0x17U                /**< Selects PWM2 Trigger 1 as input. */
#define XBARB_PDD_PWM3_TRG0 0x18U                /**< Selects PWM3 Trigger 0 as input. */
#define XBARB_PDD_PWM3_TRG1 0x19U                /**< Selects PWM3 Trigger 1 as input. */


/* ----------------------------------------------------------------------------
   -- SetAOI_Input0A
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 0 (i.e. AOI Input 0 A).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input0A(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input0A(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL0_REG(PeripheralBase), \
     XBARB_SEL0_SEL0_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input0B
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 1 (i.e. AOI Input 0 B).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input0B(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input0B(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL0_REG(PeripheralBase), \
     XBARB_SEL0_SEL1_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL0_SEL1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input0C
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 2 (i.e. AOI Input 0 C).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input0C(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input0C(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL1_REG(PeripheralBase), \
     XBARB_SEL1_SEL2_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input0D
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 3 (i.e. AOI Input 0 D).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input0D(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input0D(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL1_REG(PeripheralBase), \
     XBARB_SEL1_SEL3_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL1_SEL3_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input1A
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 4 (i.e. AOI Input 1 A).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input1A(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input1A(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL2_REG(PeripheralBase), \
     XBARB_SEL2_SEL4_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input1B
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 5 (i.e. AOI Input 1 B).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input1B(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input1B(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL2_REG(PeripheralBase), \
     XBARB_SEL2_SEL5_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL2_SEL5_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input1C
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 6 (i.e. AOI Input 1 C).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input1C(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input1C(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL3_REG(PeripheralBase), \
     XBARB_SEL3_SEL6_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input1D
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 7 (i.e. AOI Input 1 D).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input1D(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input1D(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL3_REG(PeripheralBase), \
     XBARB_SEL3_SEL7_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL3_SEL7_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input2A
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 8 (i.e. AOI Input 2 A).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input2A(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input2A(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL4_REG(PeripheralBase), \
     XBARB_SEL4_SEL8_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input2B
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 9 (i.e. AOI Input 2 B).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input2B(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input2B(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL4_REG(PeripheralBase), \
     XBARB_SEL4_SEL9_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL4_SEL9_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input2C
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 10 (i.e. AOI Input 2 C).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input2C(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input2C(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL5_REG(PeripheralBase), \
     XBARB_SEL5_SEL10_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input2D
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 11 (i.e. AOI Input 2 D).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input2D(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input2D(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL5_REG(PeripheralBase), \
     XBARB_SEL5_SEL11_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL5_SEL11_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input3A
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 12 (i.e. AOI Input 3 A).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input3A(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input3A(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL6_REG(PeripheralBase), \
     XBARB_SEL6_SEL12_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input3B
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 13 (i.e. AOI Input 3 B).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input3B(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input3B(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL6_REG(PeripheralBase), \
     XBARB_SEL6_SEL13_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL6_SEL13_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input3C
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 14 (i.e. AOI Input 3 C).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input3C(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input3C(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL7_REG(PeripheralBase), \
     XBARB_SEL7_SEL14_MASK, \
     (uint16_t)(XbarInput) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAOI_Input3D
   ---------------------------------------------------------------------------- */

/**
 * @brief Connects selected XBAR input with XBAR output 15 (i.e. AOI Input 3 D).
 * @param PeripheralBase Peripheral base address.
 * @param XbarInput Required XBAR input. This parameter is of "XBAR input
 *        constants. They are used to specify signal connected with XBAR output." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      XBARB_PDD_SetAOI_Input3D(deviceID, XBARB_PDD_SCI0_TXD);
 *      @endcode
 */
#define XBARB_PDD_SetAOI_Input3D(PeripheralBase, XbarInput) ( \
    PeriphSetBits16( \
     XBARB_SEL7_REG(PeripheralBase), \
     XBARB_SEL7_SEL15_MASK, \
     (uint16_t)((uint16_t)(XbarInput) << XBARB_SEL7_SEL15_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input0A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 0 A.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input0A(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input0A(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL0_REG(PeripheralBase), XBARB_SEL0_SEL0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input0B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 0 B.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input0B(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input0B(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL0_REG(PeripheralBase), XBARB_SEL0_SEL1_MASK)) >> ( \
     XBARB_SEL0_SEL1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input0C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 0 C.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input0C(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input0C(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL1_REG(PeripheralBase), XBARB_SEL1_SEL2_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input0D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 0 D.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input0D(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input0D(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL1_REG(PeripheralBase), XBARB_SEL1_SEL3_MASK)) >> ( \
     XBARB_SEL1_SEL3_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input1A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 1 A.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input1A(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input1A(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL2_REG(PeripheralBase), XBARB_SEL2_SEL4_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input1B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 1 B.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input1B(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input1B(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL2_REG(PeripheralBase), XBARB_SEL2_SEL5_MASK)) >> ( \
     XBARB_SEL2_SEL5_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input1C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 1 C.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input1C(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input1C(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL3_REG(PeripheralBase), XBARB_SEL3_SEL6_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input1D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 1 D.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input1D(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input1D(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL3_REG(PeripheralBase), XBARB_SEL3_SEL7_MASK)) >> ( \
     XBARB_SEL3_SEL7_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input2A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 2 A.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input2A(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input2A(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL4_REG(PeripheralBase), XBARB_SEL4_SEL8_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input2B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 2 B.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input2B(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input2B(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL4_REG(PeripheralBase), XBARB_SEL4_SEL9_MASK)) >> ( \
     XBARB_SEL4_SEL9_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input2C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 2 C.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input2C(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input2C(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL5_REG(PeripheralBase), XBARB_SEL5_SEL10_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input2D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 2 D.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input2D(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input2D(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL5_REG(PeripheralBase), XBARB_SEL5_SEL11_MASK)) >> ( \
     XBARB_SEL5_SEL11_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input3A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 3 A.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input3A(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input3A(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL6_REG(PeripheralBase), XBARB_SEL6_SEL12_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input3B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 3 B.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input3B(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input3B(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL6_REG(PeripheralBase), XBARB_SEL6_SEL13_MASK)) >> ( \
     XBARB_SEL6_SEL13_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input3C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 3 C.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input3C(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input3C(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(XBARB_SEL7_REG(PeripheralBase), XBARB_SEL7_SEL14_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOI_Input3D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current InputID selected as input of AOI Input 3 D.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      uint8_t result = XBARB_PDD_GetAOI_Input3D(deviceID);
 *      @endcode
 */
#define XBARB_PDD_GetAOI_Input3D(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(XBARB_SEL7_REG(PeripheralBase), XBARB_SEL7_SEL15_MASK)) >> ( \
     XBARB_SEL7_SEL15_SHIFT)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect0Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL0_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect1Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL1_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect2Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL2_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect3Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL3_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect4Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect4Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL4_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect5Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect5Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL5_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect6Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect6Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL6_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      XBARB_PDD_WriteCrossbarSelect7Reg(deviceID, 1);
 *      @endcode
 */
#define XBARB_PDD_WriteCrossbarSelect7Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(XBARB_SEL7_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL0.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect0Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect0Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL1.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect1Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect1Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL2.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect2Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect2Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL3.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect3Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect3Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 4.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL4.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect4Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect4Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 5.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL5.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect5Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect5Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect6Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 6.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL6.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect6Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect6Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL6_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCrossbarSelect7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the crossbar select register 7.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: XBARB_SEL7.
 * @par Example:
 *      @code
 *      uint16_t result = XBARB_PDD_ReadCrossbarSelect7Reg(deviceID);
 *      @endcode
 */
#define XBARB_PDD_ReadCrossbarSelect7Reg(PeripheralBase) ( \
    PeriphReadReg16(XBARB_SEL7_REG(PeripheralBase)) \
  )
#endif  /* #if defined(XBARB_PDD_H_) */

/* XBARB_PDD.h, eof. */
