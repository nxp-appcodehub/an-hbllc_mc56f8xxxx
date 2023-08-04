/*
  PDD layer implementation for peripheral type AOI
  Copyright 2010 Freescale, Inc. All rights reserved.
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(AOI_PDD_H_)
#define AOI_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error AOI PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* AOI */ && \
      !defined(MCU_MC56F82316) /* AOI */ && \
      !defined(MCU_MC56F82723) /* AOI */ && \
      !defined(MCU_MC56F82726) /* AOI */ && \
      !defined(MCU_MC56F82728) /* AOI */ && \
      !defined(MCU_MC56F82733) /* AOI */ && \
      !defined(MCU_MC56F82736) /* AOI */ && \
      !defined(MCU_MC56F82738) /* AOI */ && \
      !defined(MCU_MC56F82743) /* AOI */ && \
      !defined(MCU_MC56F82746) /* AOI */ && \
      !defined(MCU_MC56F82748) /* AOI */
  // Unsupported MCU is active
  #error AOI PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Input configuration constants. They are used in product terms. */
#define AOI_PDD_Zero       0U                    /**< Force a input in a product term to a logical zero. */
#define AOI_PDD_Pass       0x1U                  /**< Pass a input in a product term. */
#define AOI_PDD_Complement 0x2U                  /**< Complement a input in a product term. */
#define AOI_PDD_One        0x3U                  /**< Force a input in a product term to a logical one. */


/* ----------------------------------------------------------------------------
   -- SetFunctionTerm0
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets function term 0.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param InputA Defines state of term 0 input A. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputB Defines state of term 0 input B. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputC Defines state of term 0 input C. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputD Defines state of term 0 input D. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetFunctionTerm0(deviceID, periphID, AOI_PDD_Zero,
 *      AOI_PDD_Zero, AOI_PDD_Zero, AOI_PDD_Zero);
 *      @endcode
 */
#define AOI_PDD_SetFunctionTerm0(PeripheralBase, FunctionIndex, InputA, InputB, InputC, InputD) ( \
    PeriphSetBits16( \
     AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(( \
      AOI_BFCRT01_PT0_AC_MASK) | (( \
      AOI_BFCRT01_PT0_BC_MASK) | (( \
      AOI_BFCRT01_PT0_CC_MASK) | ( \
      AOI_BFCRT01_PT0_DC_MASK)))), \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(InputA) << AOI_BFCRT01_PT0_AC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputB) << AOI_BFCRT01_PT0_BC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputC) << AOI_BFCRT01_PT0_CC_SHIFT)) | ( \
      (uint16_t)((uint16_t)(InputD) << AOI_BFCRT01_PT0_DC_SHIFT))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFunctionTerm1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets function term 1.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param InputA Defines state of term 1 input A. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputB Defines state of term 1 input B. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputC Defines state of term 1 input C. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputD Defines state of term 1 input D. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetFunctionTerm1(deviceID, periphID, AOI_PDD_Zero,
 *      AOI_PDD_Zero, AOI_PDD_Zero, AOI_PDD_Zero);
 *      @endcode
 */
#define AOI_PDD_SetFunctionTerm1(PeripheralBase, FunctionIndex, InputA, InputB, InputC, InputD) ( \
    PeriphSetBits16( \
     AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(( \
      AOI_BFCRT01_PT1_AC_MASK) | (( \
      AOI_BFCRT01_PT1_BC_MASK) | (( \
      AOI_BFCRT01_PT1_CC_MASK) | ( \
      AOI_BFCRT01_PT1_DC_MASK)))), \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(InputA) << AOI_BFCRT01_PT1_AC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputB) << AOI_BFCRT01_PT1_BC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputC) << AOI_BFCRT01_PT1_CC_SHIFT)) | ( \
      (uint16_t)(InputD))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFunctionTerm2
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets function term 2.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param InputA Defines state of term 2 input A. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputB Defines state of term 2 input B. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputC Defines state of term 2 input C. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputD Defines state of term 2 input D. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetFunctionTerm2(deviceID, periphID, AOI_PDD_Zero,
 *      AOI_PDD_Zero, AOI_PDD_Zero, AOI_PDD_Zero);
 *      @endcode
 */
#define AOI_PDD_SetFunctionTerm2(PeripheralBase, FunctionIndex, InputA, InputB, InputC, InputD) ( \
    PeriphSetBits16( \
     AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(( \
      AOI_BFCRT23_PT2_AC_MASK) | (( \
      AOI_BFCRT23_PT2_BC_MASK) | (( \
      AOI_BFCRT23_PT2_CC_MASK) | ( \
      AOI_BFCRT23_PT2_DC_MASK)))), \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(InputA) << AOI_BFCRT23_PT2_AC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputB) << AOI_BFCRT23_PT2_BC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputC) << AOI_BFCRT23_PT2_CC_SHIFT)) | ( \
      (uint16_t)((uint16_t)(InputD) << AOI_BFCRT23_PT2_DC_SHIFT))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFunctionTerm3
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets function term 3.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param InputA Defines state of term 3 input A. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputB Defines state of term 3 input B. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputC Defines state of term 3 input C. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @param InputD Defines state of term 3 input D. This parameter is of "Input
 *        configuration constants. They are used in product terms." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetFunctionTerm3(deviceID, periphID, AOI_PDD_Zero,
 *      AOI_PDD_Zero, AOI_PDD_Zero, AOI_PDD_Zero);
 *      @endcode
 */
#define AOI_PDD_SetFunctionTerm3(PeripheralBase, FunctionIndex, InputA, InputB, InputC, InputD) ( \
    PeriphSetBits16( \
     AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(( \
      AOI_BFCRT23_PT3_AC_MASK) | (( \
      AOI_BFCRT23_PT3_BC_MASK) | (( \
      AOI_BFCRT23_PT3_CC_MASK) | ( \
      AOI_BFCRT23_PT3_DC_MASK)))), \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(InputA) << AOI_BFCRT23_PT3_AC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputB) << AOI_BFCRT23_PT3_BC_SHIFT)) | (( \
      (uint16_t)((uint16_t)(InputC) << AOI_BFCRT23_PT3_CC_SHIFT)) | ( \
      (uint16_t)(InputD))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm0A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input A state of the term 0.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm0A(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm0A(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT0_AC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT0_AC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm0B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input B state of the term 0.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm0B(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm0B(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT0_BC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT0_BC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm0C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input C state of the term 0.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm0C(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm0C(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT0_CC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT0_CC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm0D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input D state of the term 0.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm0D(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm0D(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT0_DC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT0_DC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm1A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input A state of the term 1.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm1A(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm1A(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT1_AC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT1_AC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm1B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input B state of the term 1.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm1B(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm1B(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT1_BC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT1_BC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm1C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input C state of the term 1.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm1C(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm1C(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT01_PT1_CC_MASK \
     )) >> ( \
     AOI_BFCRT01_PT1_CC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm1D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input D state of the term 1.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm1D(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm1D(PeripheralBase, FunctionIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
     AOI_BFCRT01_PT1_DC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm2A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input A state of the term 2.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm2A(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm2A(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT2_AC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT2_AC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm2B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input B state of the term 2.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm2B(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm2B(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT2_BC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT2_BC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm2C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input C state of the term 2.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm2C(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm2C(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT2_CC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT2_CC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm2D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input D state of the term 2.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm2D(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm2D(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT2_DC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT2_DC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm3A
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input A state of the term 3.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm3A(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm3A(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT3_AC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT3_AC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm3B
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input B state of the term 3.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm3B(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm3B(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT3_BC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT3_BC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm3C
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input C state of the term 3.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm3C(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm3C(PeripheralBase, FunctionIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
      AOI_BFCRT23_PT3_CC_MASK \
     )) >> ( \
     AOI_BFCRT23_PT3_CC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFunctionTerm3D
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current input D state of the term 3.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint8_t result = AOI_PDD_GetFunctionTerm3D(deviceID, periphID);
 *      @endcode
 */
#define AOI_PDD_GetFunctionTerm3D(PeripheralBase, FunctionIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
     AOI_BFCRT23_PT3_DC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetBooleanFunctionTerm0And1ForEvent
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes boolean function term 0 and 1 configuration register for EVENT.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param Value The value is written into boolean function term 0 and 1
 *        configuration register for EVENT. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetBooleanFunctionTerm0And1ForEvent(deviceID, periphID, 1);
 *      @endcode
 */
#define AOI_PDD_SetBooleanFunctionTerm0And1ForEvent(PeripheralBase, FunctionIndex, Value) ( \
    PeriphWriteReg16( \
     AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetBooleanFunctionTerm2And3ForEvent
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes boolean function term 2 and 3 configuration register for EVENT.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @param Value The value is written into boolean function term 2 and 3
 *        configuration register for EVENT. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      AOI_PDD_SetBooleanFunctionTerm2And3ForEvent(deviceID, periphID, 1);
 *      @endcode
 */
#define AOI_PDD_SetBooleanFunctionTerm2And3ForEvent(PeripheralBase, FunctionIndex, Value) ( \
    PeriphWriteReg16( \
     AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex)), \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBooleanFunctionTerm0And1ForEvent
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the boolean function term 0 and 1 configuration
 * register for EVENT.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT01[FunctionIndex].
 * @par Example:
 *      @code
 *      uint16_t result = AOI_PDD_GetBooleanFunctionTerm0And1ForEvent(deviceID,
 *      periphID);
 *      @endcode
 */
#define AOI_PDD_GetBooleanFunctionTerm0And1ForEvent(PeripheralBase, FunctionIndex) ( \
    PeriphReadReg16(AOI_BFCRT01_REG(PeripheralBase,(FunctionIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetBooleanFunctionTerm2And3ForEvent
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the boolean function term 2 and 3 configuration
 * register for EVENT.
 * @param PeripheralBase Peripheral base address.
 * @param FunctionIndex Function index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT23[FunctionIndex].
 * @par Example:
 *      @code
 *      uint16_t result = AOI_PDD_GetBooleanFunctionTerm2And3ForEvent(deviceID,
 *      periphID);
 *      @endcode
 */
#define AOI_PDD_GetBooleanFunctionTerm2And3ForEvent(PeripheralBase, FunctionIndex) ( \
    PeriphReadReg16(AOI_BFCRT23_REG(PeripheralBase,(FunctionIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction0Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 0 and 1 Configuration Register for EVENT0.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 0 and 1
 *        Configuration Register for EVENT0. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction0Term0And1ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction0Term0And1ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT01_REG(PeripheralBase,0U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction1Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 0 and 1 Configuration Register for EVENT1.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 0 and 1
 *        Configuration Register for EVENT1. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction1Term0And1ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction1Term0And1ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT01_REG(PeripheralBase,1U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction2Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 0 and 1 Configuration Register for EVENT2.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 0 and 1
 *        Configuration Register for EVENT2. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction2Term0And1ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction2Term0And1ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT01_REG(PeripheralBase,2U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction3Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 0 and 1 Configuration Register for EVENT3.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 0 and 1
 *        Configuration Register for EVENT3. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction3Term0And1ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction3Term0And1ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT01_REG(PeripheralBase,3U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction0Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 2 and 3 Configuration Register for EVENT0.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 2 and 3
 *        Configuration Register for EVENT0. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction0Term2And3ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction0Term2And3ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT23_REG(PeripheralBase,0U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction1Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 2 and 3 Configuration Register for EVENT1.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 2 and 3
 *        Configuration Register for EVENT1. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction1Term2And3ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction1Term2And3ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT23_REG(PeripheralBase,1U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction2Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 2 and 3 Configuration Register for EVENT2.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 2 and 3
 *        Configuration Register for EVENT2. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction2Term2And3ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction2Term2And3ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT23_REG(PeripheralBase,2U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFunction3Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Boolean Function Term 2 and 3 Configuration Register for EVENT3.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into Boolean Function Term 2 and 3
 *        Configuration Register for EVENT3. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      AOI_PDD_WriteFunction3Term2And3ConfigurationReg(deviceID, 1);
 *      @endcode
 */
#define AOI_PDD_WriteFunction3Term2And3ConfigurationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(AOI_BFCRT23_REG(PeripheralBase,3U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction0Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 0 and 1 Configuration
 * Register for EVENT0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction0Term0And1ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction0Term0And1ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT01_REG(PeripheralBase,0U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction1Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 0 and 1 Configuration
 * Register for EVENT1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction1Term0And1ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction1Term0And1ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT01_REG(PeripheralBase,1U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction2Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 0 and 1 Configuration
 * Register for EVENT2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction2Term0And1ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction2Term0And1ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT01_REG(PeripheralBase,2U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction3Term0And1ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 0 and 1 Configuration
 * Register for EVENT3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT01[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction3Term0And1ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction3Term0And1ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT01_REG(PeripheralBase,3U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction0Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 2 and 3 Configuration
 * Register for EVENT0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction0Term2And3ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction0Term2And3ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT23_REG(PeripheralBase,0U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction1Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 2 and 3 Configuration
 * Register for EVENT1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction1Term2And3ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction1Term2And3ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT23_REG(PeripheralBase,1U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction2Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 2 and 3 Configuration
 * Register for EVENT2.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction2Term2And3ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction2Term2And3ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT23_REG(PeripheralBase,2U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFunction3Term2And3ConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Boolean Function Term 2 and 3 Configuration
 * Register for EVENT3.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: BFCRT23[].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      AOI_PDD_ReadFunction3Term2And3ConfigurationReg(deviceID);
 *      @endcode
 */
#define AOI_PDD_ReadFunction3Term2And3ConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(AOI_BFCRT23_REG(PeripheralBase,3U)) \
  )
#endif  /* #if defined(AOI_PDD_H_) */

/* AOI_PDD.h, eof. */
