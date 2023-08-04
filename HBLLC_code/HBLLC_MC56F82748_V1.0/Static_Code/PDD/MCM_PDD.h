/*
  PDD layer implementation for peripheral type MCM
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(MCM_PDD_H_)
#define MCM_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error MCM PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* MCM */ && \
      !defined(MCU_MC56F82316) /* MCM */ && \
      !defined(MCU_MC56F82723) /* MCM */ && \
      !defined(MCU_MC56F82726) /* MCM */ && \
      !defined(MCU_MC56F82728) /* MCM */ && \
      !defined(MCU_MC56F82733) /* MCM */ && \
      !defined(MCU_MC56F82736) /* MCM */ && \
      !defined(MCU_MC56F82738) /* MCM */ && \
      !defined(MCU_MC56F82743) /* MCM */ && \
      !defined(MCU_MC56F82746) /* MCM */ && \
      !defined(MCU_MC56F82748) /* MCM */
  // Unsupported MCU is active
  #error MCM PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Interrupts' enable mask. */
#define MCM_PDD_CORE_FAULT_INTERRUPT MCM_CFIER_ECFEI_MASK /**< Core fault error interrupt mask. */

/* Interrupts' flags. */
#define MCM_PDD_CORE_FAULT_FLAG MCM_CFISR_CFEI_MASK /**< Core fault error flag. */

/* AXBS slave connections' mask constants. */
#define MCM_PDD_SLAVE_0 0x1U                     /**< Bus slave 0. */
#define MCM_PDD_SLAVE_1 0x2U                     /**< Bus slave 1. */
#define MCM_PDD_SLAVE_2 0x4U                     /**< Bus slave 2. */
#define MCM_PDD_SLAVE_3 0x8U                     /**< Bus slave 3. */
#define MCM_PDD_SLAVE_4 0x10U                    /**< Bus slave 4. */
#define MCM_PDD_SLAVE_5 0x20U                    /**< Bus slave 5. */
#define MCM_PDD_SLAVE_6 0x40U                    /**< Bus slave 6. */
#define MCM_PDD_SLAVE_7 0x80U                    /**< Bus slave 7. */

/* AXBS master connections' mask constants. */
#define MCM_PDD_MASTER_0 0x1U                    /**< Bus master 0. */
#define MCM_PDD_MASTER_1 0x2U                    /**< Bus master 1. */
#define MCM_PDD_MASTER_2 0x4U                    /**< Bus master 2. */
#define MCM_PDD_MASTER_3 0x8U                    /**< Bus master 3. */
#define MCM_PDD_MASTER_4 0x10U                   /**< Bus master 4. */
#define MCM_PDD_MASTER_5 0x20U                   /**< Bus master 5. */
#define MCM_PDD_MASTER_6 0x40U                   /**< Bus master 6. */
#define MCM_PDD_MASTER_7 0x80U                   /**< Bus master 7. */

/* Datapath width constants. */
#define MCM_PDD_32_BIT 0U                        /**< 32-bit datapath. */
#define MCM_PDD_64_BIT MCM_PLASC_DP64_MASK       /**< 64-bit datapath. */

/* Direction of last faulted core access. */
#define MCM_PDD_FAULT_READ  0U                   /**< Core read access. */
#define MCM_PDD_FAULT_WRITE MCM_CFATR_DIR_MASK   /**< Core write access. */

/* Constants defining size of last faulted core access. */
#define MCM_PDD_FAULT_8_BIT  0U                  /**< 8-bit. */
#define MCM_PDD_FAULT_16_BIT 0x10U               /**< 16-bit. */
#define MCM_PDD_FAULT_32_BIT 0x20U               /**< 32-bit. */

/* Constatnts defining if last faulted core access was bufferable. */
#define MCM_PDD_FAULT_NON_BUFFERABLE 0U          /**< Non-bufferable. */
#define MCM_PDD_FAULT_BUFFERABLE     MCM_CFATR_BUFFER_MASK /**< Bufferable. */

/* Type of last faulted core access constants. */
#define MCM_PDD_FAULT_INSTRUCTION 0U             /**< Instruction. */
#define MCM_PDD_FAULT_DATA        MCM_CFATR_TYPE_MASK /**< Data. */

/* Constants defining location of last faulted core access. */
#define MCM_PDD_FAULT_INSTRUCTION_BUS 0U         /**< Error occurred on M0 (instruction bus). */
#define MCM_PDD_FAULT_OPERAND_A_BUS   0x40U      /**< Error occured on M1 (operand A bus). */
#define MCM_PDD_FAULT_OPERAND_B_BUS   0x80U      /**< Error occured on M2 (operand B bus). */


/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to enable. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableInterrupts(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_EnableInterrupts(PeripheralBase, Mask) ( \
    PeriphSetBitMask8(MCM_CFIER_REG(PeripheralBase), (uint8_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupts to disable. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      MCM_PDD_DisableInterrupts(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_DisableInterrupts(PeripheralBase, Mask) ( \
    PeriphClearBitMask8(MCM_CFIER_REG(PeripheralBase), (uint8_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetInterruptFlags(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFISR_REG(PeripheralBase), MCM_CFISR_CFEI_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt's flags to clear. This parameter is a 8-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      MCM_PDD_ClearInterruptFlags(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBitMask8(MCM_CFISR_REG(PeripheralBase), (uint8_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDatapathWidth
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns datapath width.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: MCM_PLASC.
 * @par Example:
 *      @code
 *      uint16_t result = MCM_PDD_GetDatapathWidth(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetDatapathWidth(PeripheralBase) ( \
    PeriphGetBitMask16(MCM_PLASC_REG(PeripheralBase), MCM_PLASC_DP64_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetBusSlaveConnectedMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns state of connections to the AXBS slave input port.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "AXBS slave connections' mask constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: MCM_PLASC.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetBusSlaveConnectedMask(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetBusSlaveConnectedMask(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(MCM_PLASC_REG(PeripheralBase), MCM_PLASC_ASC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetBusMasterConnectedMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns state of connection to the AXBS master input port.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "AXBS master connections' mask constants."
 *         for processing return value.
 * @remarks The macro accesses the following registers: MCM_PLAMC.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetBusMasterConnectedMask(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetBusMasterConnectedMask(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(MCM_PLAMC_REG(PeripheralBase), MCM_PLAMC_AMC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFlashMemoryControllerStall
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable the flash memory controller's ability to allow a
 * flash memory access to initiate when a flash memory command is executing.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if flash memory controller stalls. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableFlashMemoryControllerStall(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableFlashMemoryControllerStall(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_FCSDIS_MASK)) : ( \
      PeriphClearBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_FCSDIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFlashMemoryControllerStallEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns flash memory controller stalls state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetFlashMemoryControllerStallEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetFlashMemoryControllerStallEnabled(PeripheralBase) ( \
    (PeriphGetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_FCSDIS_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCoreInstructionBuffer
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable core instruction buffer.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if core instruction buffer is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableCoreInstructionBuffer(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableCoreInstructionBuffer(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_IBDIS_MASK)) : ( \
      PeriphClearBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_IBDIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreInstructionBufferEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns core instruction buffer enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreInstructionBufferEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreInstructionBufferEnabled(PeripheralBase) ( \
    (PeriphGetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_IBDIS_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCoreNewShadowRegion
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable core new shadow region.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if core new shadow region is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableCoreNewShadowRegion(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableCoreNewShadowRegion(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_SRDIS_MASK)) : ( \
      PeriphClearBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_SRDIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreNewShadowRegionEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns core new shadow region enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreNewShadowRegionEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreNewShadowRegionEnabled(PeripheralBase) ( \
    (PeriphGetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_SRDIS_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCoreCoreReverseCarry
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable core reverse carry region.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if core reverse carry is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableCoreCoreReverseCarry(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableCoreCoreReverseCarry(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_RCDIS_MASK)) : ( \
      PeriphClearBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_RCDIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreCoreReverseCarryEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns core reverse carry enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreCoreReverseCarryEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreCoreReverseCarryEnabled(PeripheralBase) ( \
    (PeriphGetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_RCDIS_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- Enable56800EXInstructions
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable instructions supported only by DSP56800EX core.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if instructions supported only by
 *        DSP56800EX core are enabled or disabled. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_Enable56800EXInstructions(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_Enable56800EXInstructions(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_INSDIS_MASK)) : ( \
      PeriphClearBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_INSDIS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- Get56800EXInstructionsEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns instructions supported only by DSP56800EX core enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_Get56800EXInstructionsEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_Get56800EXInstructionsEnabled(PeripheralBase) ( \
    (PeriphGetBitMask32(MCM_CPCR_REG(PeripheralBase), MCM_CPCR_INSDIS_MASK) == 0U) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the address of the last core access terminated with an error
 * response.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_CFADR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetCoreFaultAddress(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultAddress(PeripheralBase) ( \
    PeriphReadReg32(MCM_CFADR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultDirection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the direction of last faulted core access.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Direction of last faulted core access." type. The
 *         value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CFATR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultDirection(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultDirection(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFATR_REG(PeripheralBase), MCM_CFATR_DIR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultSize
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the size of last faulted core access.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFATR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultSize(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultSize(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFATR_REG(PeripheralBase), MCM_CFATR_SIZE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultBuffer
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns information if the last faulted core access was bufferable or
 * not.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Constatnts defining if last faulted core access
 *         was bufferable." type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CFATR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultBuffer(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultBuffer(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFATR_REG(PeripheralBase), MCM_CFATR_BUFFER_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultType
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns type of last faulted core access.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Type of last faulted core access constants."
 *         type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CFATR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultType(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultType(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFATR_REG(PeripheralBase), MCM_CFATR_TYPE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultLocation
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the location of last faulted core access.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFLOC.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultLocation(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultLocation(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFLOC_REG(PeripheralBase), MCM_CFLOC_LOC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCoreFaultInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable core fault error interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if core fault error interrupt is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableCoreFaultInterrupt(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableCoreFaultInterrupt(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask8(MCM_CFIER_REG(PeripheralBase), MCM_CFIER_ECFEI_MASK)) : ( \
      PeriphSetBitMask8(MCM_CFIER_REG(PeripheralBase), MCM_CFIER_ECFEI_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultInterruptEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns core fault error interrupt enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultInterruptEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultInterruptEnabled(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFIER_REG(PeripheralBase), MCM_CFIER_ECFEI_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns core fault flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetCoreFaultFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultFlag(PeripheralBase) ( \
    PeriphGetBitMask8(MCM_CFISR_REG(PeripheralBase), MCM_CFISR_CFEI_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearCoreFaultFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears core fault flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      MCM_PDD_ClearCoreFaultFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_ClearCoreFaultFlag(PeripheralBase) ( \
    PeriphSetBitMask8(MCM_CFISR_REG(PeripheralBase), MCM_CFISR_CFEI_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetCoreFaultData
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the write data associated with the faulting access of last
 * faulted core access.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_CFDTR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetCoreFaultData(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetCoreFaultData(PeripheralBase) ( \
    PeriphReadReg32(MCM_CFDTR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableResourceProtectionLock
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable resource protection registers lock.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if resource protection registers lock is
 *        enabled or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableResourceProtectionLock(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableResourceProtectionLock(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RL_MASK)) : ( \
      PeriphSetBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionLockEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection registers lock enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetResourceProtectionLockEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionLockEnabled(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableResourceProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable or disable resource protection lock.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if resource protection is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_EnableResourceProtection(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define MCM_PDD_EnableResourceProtection(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RPE_MASK)) : ( \
      PeriphSetBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection registers lock enabled state.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_GetResourceProtectionEnabled(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionEnabled(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_RPCR_REG(PeripheralBase), MCM_RPCR_RPE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetUserFlashBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets flash base address for user region with 4KB granularity.
 * @param PeripheralBase Peripheral base address.
 * @param Value Flash base address value for user region with 4KB granularity.
 *        This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_UFLASHBAR.
 * @par Example:
 *      @code
 *      MCM_PDD_SetUserFlashBaseAddress(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_SetUserFlashBaseAddress(PeripheralBase, Value) ( \
    PeriphWriteReg32( \
     MCM_UFLASHBAR_REG(PeripheralBase), \
     MCM_UFLASHBAR_FBA_MASK & ((uint32_t)(Value)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetUserFlashBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns flash base address for user region with 4KB granularity.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_UFLASHBAR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetUserFlashBaseAddress(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetUserFlashBaseAddress(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_UFLASHBAR_REG(PeripheralBase), MCM_UFLASHBAR_FBA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetUserProgramRamBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets program RAM base address for user region with 256 byte
 * granularity.
 * @param PeripheralBase Peripheral base address.
 * @param Value Program RAM base address value for user region with 256 byte
 *        granularity. This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_UPRAMBAR.
 * @par Example:
 *      @code
 *      MCM_PDD_SetUserProgramRamBaseAddress(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_SetUserProgramRamBaseAddress(PeripheralBase, Value) ( \
    PeriphWriteReg32( \
     MCM_UPRAMBAR_REG(PeripheralBase), \
     MCM_UPRAMBAR_RBA_MASK & ((uint32_t)(Value)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetUserProgramRamBaseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns program RAM base address for user region with 256 byte
 * granularity.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_UPRAMBAR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetUserProgramRamBaseAddress(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetUserProgramRamBaseAddress(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_UPRAMBAR_REG(PeripheralBase), MCM_UPRAMBAR_RBA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetOtherStackPointer
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets resource protection 'other' stack pointer.
 * @param PeripheralBase Peripheral base address.
 * @param Value Resource protection 'other' stack pointer value. This parameter
 *        is a 24-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPOSP.
 * @par Example:
 *      @code
 *      MCM_PDD_SetOtherStackPointer(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_SetOtherStackPointer(PeripheralBase, Value) ( \
    PeriphSetBits32( \
     MCM_SRPOSP_REG(PeripheralBase), \
     MCM_SRPOSP_SRPOSP_MASK, \
     (uint32_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOtherStackPointer
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection 'other' stack pointer value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPOSP.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetOtherStackPointer(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetOtherStackPointer(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_SRPOSP_REG(PeripheralBase), MCM_SRPOSP_SRPOSP_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionIllegalFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection illegal PC fault valid flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPIPC.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetResourceProtectionIllegalFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionIllegalFlag(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_SRPIPC_REG(PeripheralBase), MCM_SRPIPC_SRPIFV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearResourceProtectionIllegalFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears resource protection illegal PC fault valid flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPIPC.
 * @par Example:
 *      @code
 *      MCM_PDD_ClearResourceProtectionIllegalFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_ClearResourceProtectionIllegalFlag(PeripheralBase) ( \
    PeriphSetBitMask32(MCM_SRPIPC_REG(PeripheralBase), MCM_SRPIPC_SRPIFV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionIllegalProgramCounter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection illegal PC value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPIPC.
 * @par Example:
 *      @code
 *      uint32_t result =
 *      MCM_PDD_GetResourceProtectionIllegalProgramCounter(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionIllegalProgramCounter(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_SRPIPC_REG(PeripheralBase), MCM_SRPIPC_SRPIFPC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionMisalignedFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection misaligned PC fault valid flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPMPC.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_GetResourceProtectionMisalignedFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionMisalignedFlag(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_SRPMPC_REG(PeripheralBase), MCM_SRPMPC_SRPMFV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearResourceProtectionMisalignedFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears resource protection misaligned PC fault valid flag.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPMPC.
 * @par Example:
 *      @code
 *      MCM_PDD_ClearResourceProtectionMisalignedFlag(deviceID);
 *      @endcode
 */
#define MCM_PDD_ClearResourceProtectionMisalignedFlag(PeripheralBase) ( \
    PeriphSetBitMask32(MCM_SRPMPC_REG(PeripheralBase), MCM_SRPMPC_SRPMFV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetResourceProtectionMisalignedProgramCounter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns resource protection misaligned PC value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPMPC.
 * @par Example:
 *      @code
 *      uint32_t result =
 *      MCM_PDD_GetResourceProtectionMisalignedProgramCounter(deviceID);
 *      @endcode
 */
#define MCM_PDD_GetResourceProtectionMisalignedProgramCounter(PeripheralBase) ( \
    PeriphGetBitMask32(MCM_SRPMPC_REG(PeripheralBase), MCM_SRPMPC_SRPMFPC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadAxbsSlaveConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the crossbar switch (AXBS) slave configuration
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: MCM_PLASC.
 * @par Example:
 *      @code
 *      uint16_t result = MCM_PDD_ReadAxbsSlaveConfigurationReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadAxbsSlaveConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(MCM_PLASC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadAxbsMasterConfigurationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the crossbar switch (AXBS) master configuration
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: MCM_PLAMC.
 * @par Example:
 *      @code
 *      uint16_t result = MCM_PDD_ReadAxbsMasterConfigurationReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadAxbsMasterConfigurationReg(PeripheralBase) ( \
    PeriphReadReg16(MCM_PLAMC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCoreControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the core control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteCoreControlReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteCoreControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_CPCR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_CPCR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadCoreControlReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreControlReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_CPCR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault address register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_CFADR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadCoreFaultAddressReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultAddressReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_CFADR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultAttributesReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault attributes register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFATR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_ReadCoreFaultAttributesReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultAttributesReg(PeripheralBase) ( \
    PeriphReadReg8(MCM_CFATR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultLocationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault location register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFLOC.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_ReadCoreFaultLocationReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultLocationReg(PeripheralBase) ( \
    PeriphReadReg8(MCM_CFLOC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCoreFaultInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the core fault interrupt enable register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteCoreFaultInterruptEnableReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteCoreFaultInterruptEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg8(MCM_CFIER_REG(PeripheralBase), (uint8_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault interrupt enable register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFIER.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_ReadCoreFaultInterruptEnableReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultInterruptEnableReg(PeripheralBase) ( \
    PeriphReadReg8(MCM_CFIER_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCoreFaultInterruptStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the core fault interrupt status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteCoreFaultInterruptStatusReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteCoreFaultInterruptStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg8(MCM_CFISR_REG(PeripheralBase), (uint8_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultInterruptStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault interrupt status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: MCM_CFISR.
 * @par Example:
 *      @code
 *      uint8_t result = MCM_PDD_ReadCoreFaultInterruptStatusReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultInterruptStatusReg(PeripheralBase) ( \
    PeriphReadReg8(MCM_CFISR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCoreFaultDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the core fault data register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_CFDTR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadCoreFaultDataReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadCoreFaultDataReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_CFDTR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteResourceProtectionControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the resource protection control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteResourceProtectionControlReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteResourceProtectionControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_RPCR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadResourceProtectionControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the resource protection control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_RPCR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadResourceProtectionControlReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadResourceProtectionControlReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_RPCR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteUserFlashBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the user flash base address register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_UFLASHBAR.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteUserFlashBaseAddressReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteUserFlashBaseAddressReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_UFLASHBAR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadUserFlashBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the user flash base address register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_UFLASHBAR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadUserFlashBaseAddressReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadUserFlashBaseAddressReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_UFLASHBAR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteUserProgramRamBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the user program RAM base address register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_UPRAMBAR.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteUserProgramRamBaseAddressReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteUserProgramRamBaseAddressReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_UPRAMBAR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadUserProgramRamBaseAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the user program RAM base address register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_UPRAMBAR.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadUserProgramRamBaseAddressReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadUserProgramRamBaseAddressReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_UPRAMBAR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteResourceProtectionOtherStackPointerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the resource protection other stack pointer register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPOSP.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteResourceProtectionOtherStackPointerReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteResourceProtectionOtherStackPointerReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_SRPOSP_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadResourceProtectionOtherStackPointerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the resource protection other stack pointer
 * register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPOSP.
 * @par Example:
 *      @code
 *      uint32_t result =
 *      MCM_PDD_ReadResourceProtectionOtherStackPointerReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadResourceProtectionOtherStackPointerReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_SRPOSP_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMemoryProtectionIllegalPcReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the memory protection illegal PC register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPIPC.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteMemoryProtectionIllegalPcReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteMemoryProtectionIllegalPcReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_SRPIPC_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMemoryProtectionIllegalPcReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the memory protection illegal PC register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPIPC.
 * @par Example:
 *      @code
 *      uint32_t result = MCM_PDD_ReadMemoryProtectionIllegalPcReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadMemoryProtectionIllegalPcReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_SRPIPC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteResourceProtectionMisalignedPcReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes the resource protection misaligned PC register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Parameter specifying new register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: MCM_SRPMPC.
 * @par Example:
 *      @code
 *      MCM_PDD_WriteResourceProtectionMisalignedPcReg(deviceID, 1);
 *      @endcode
 */
#define MCM_PDD_WriteResourceProtectionMisalignedPcReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(MCM_SRPMPC_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadResourceProtectionMisalignedPcReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the resource protection misaligned PC register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: MCM_SRPMPC.
 * @par Example:
 *      @code
 *      uint32_t result =
 *      MCM_PDD_ReadResourceProtectionMisalignedPcReg(deviceID);
 *      @endcode
 */
#define MCM_PDD_ReadResourceProtectionMisalignedPcReg(PeripheralBase) ( \
    PeriphReadReg32(MCM_SRPMPC_REG(PeripheralBase)) \
  )
#endif  /* #if defined(MCM_PDD_H_) */

/* MCM_PDD.h, eof. */
