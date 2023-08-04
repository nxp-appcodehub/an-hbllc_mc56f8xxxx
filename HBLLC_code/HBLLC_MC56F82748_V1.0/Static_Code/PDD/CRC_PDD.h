/*
  PDD layer implementation for peripheral type CRC
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(CRC_PDD_H_)
#define CRC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error CRC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* CRC */ && \
      !defined(MCU_MC56F82316) /* CRC */ && \
      !defined(MCU_MC56F82723) /* CRC */ && \
      !defined(MCU_MC56F82726) /* CRC */ && \
      !defined(MCU_MC56F82728) /* CRC */ && \
      !defined(MCU_MC56F82733) /* CRC */ && \
      !defined(MCU_MC56F82736) /* CRC */ && \
      !defined(MCU_MC56F82738) /* CRC */ && \
      !defined(MCU_MC56F82743) /* CRC */ && \
      !defined(MCU_MC56F82746) /* CRC */ && \
      !defined(MCU_MC56F82748) /* CRC */
  // Unsupported MCU is active
  #error CRC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
   -- Transpose
   ---------------------------------------------------------------------------- */

/**
 * @brief This function is used to transpose data, converting from LSb to MSb.
 * Returns the transposed value written byte.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value has to be transposed. This parameter is a 16-bit value.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CRC_TRANSPOSE.
 * @par Example:
 *      @code
 *      uint8_t result = CRC_PDD_Transpose(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_Transpose(PeripheralBase, Value) ( \
    PeriphWriteReg16(CRC_TRANSPOSE_REG(PeripheralBase), (uint16_t)(Value)), \
    (uint8_t)PeriphGetBitMask16( \
     CRC_TRANSPOSE_REG(PeripheralBase), \
     CRC_TRANSPOSE_TRANSPOSE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTransposeRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes transpose register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into transpose register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CRC_TRANSPOSE.
 * @par Example:
 *      @code
 *      CRC_PDD_SetTransposeRegister(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_SetTransposeRegister(PeripheralBase, Value) ( \
    PeriphWriteReg16(CRC_TRANSPOSE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTransposeRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the transpose register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CRC_TRANSPOSE.
 * @par Example:
 *      @code
 *      uint16_t result = CRC_PDD_GetTransposeRegister(deviceID);
 *      @endcode
 */
#define CRC_PDD_GetTransposeRegister(PeripheralBase) ( \
    PeriphReadReg16(CRC_TRANSPOSE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSeedLow
   ---------------------------------------------------------------------------- */

/**
 * @brief Triggers the seed mechanism to start a new CRC calculation.
 * @param PeripheralBase Peripheral base address.
 * @param Seed The value is written into generator shift register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CRC_CRCH, CRC_CRCL
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      CRC_PDD_SetSeedLow(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_SetSeedLow(PeripheralBase, Seed) ( \
    PeriphSetBits16( \
     CRC_CRCH_REG(PeripheralBase), \
     CRC_CRCH_CRCH_MASK, \
     (uint16_t)((uint16_t)(Seed) >> 8U) \
    ), \
    PeriphWriteReg16(CRC_CRCL_REG(PeripheralBase), (uint16_t)(Seed)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCRCDataLLRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Set CRC data register (1 byte).
 * @param PeripheralBase Peripheral base address.
 * @param Data New data for CRC computation. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CRC_CRCL.
 * @par Example:
 *      @code
 *      CRC_PDD_SetCRCDataLLRegister(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_SetCRCDataLLRegister(PeripheralBase, Data) ( \
    PeriphWriteReg16(CRC_CRCL_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCRCDataLLRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current CRC 8bit result from data register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CRC_CRCL.
 * @par Example:
 *      @code
 *      uint8_t result = CRC_PDD_GetCRCDataLLRegister(deviceID);
 *      @endcode
 */
#define CRC_PDD_GetCRCDataLLRegister(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CRC_CRCL_REG(PeripheralBase), CRC_CRCL_CRCL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetCRCLowRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes CRC high register.
 * @param PeripheralBase Peripheral base address.
 * @param Data The value is written into CRC high register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CRC_CRCL.
 * @par Example:
 *      @code
 *      CRC_PDD_SetCRCLowRegister(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_SetCRCLowRegister(PeripheralBase, Data) ( \
    PeriphWriteReg16(CRC_CRCL_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCRCLowRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the CRC low register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CRC_CRCL.
 * @par Example:
 *      @code
 *      uint16_t result = CRC_PDD_GetCRCLowRegister(deviceID);
 *      @endcode
 */
#define CRC_PDD_GetCRCLowRegister(PeripheralBase) ( \
    PeriphReadReg16(CRC_CRCL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCRCHighRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes CRC high register.
 * @param PeripheralBase Peripheral base address.
 * @param Data The value is written into CRC high register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CRC_CRCH.
 * @par Example:
 *      @code
 *      CRC_PDD_SetCRCHighRegister(deviceID, 1);
 *      @endcode
 */
#define CRC_PDD_SetCRCHighRegister(PeripheralBase, Data) ( \
    PeriphWriteReg16(CRC_CRCH_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCRCHighRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the CRC high register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CRC_CRCH.
 * @par Example:
 *      @code
 *      uint16_t result = CRC_PDD_GetCRCHighRegister(deviceID);
 *      @endcode
 */
#define CRC_PDD_GetCRCHighRegister(PeripheralBase) ( \
    PeriphReadReg16(CRC_CRCH_REG(PeripheralBase)) \
  )
#endif  /* #if defined(CRC_PDD_H_) */

/* CRC_PDD.h, eof. */
