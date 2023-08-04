/*
  PDD layer implementation for peripheral type FMC
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(FMC_PDD_H_)
#define FMC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error FMC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* FMC */ && \
      !defined(MCU_MC56F82316) /* FMC */ && \
      !defined(MCU_MC56F82723) /* FMC */ && \
      !defined(MCU_MC56F82726) /* FMC */ && \
      !defined(MCU_MC56F82728) /* FMC */ && \
      !defined(MCU_MC56F82733) /* FMC */ && \
      !defined(MCU_MC56F82736) /* FMC */ && \
      !defined(MCU_MC56F82738) /* FMC */ && \
      !defined(MCU_MC56F82743) /* FMC */ && \
      !defined(MCU_MC56F82746) /* FMC */ && \
      !defined(MCU_MC56F82748) /* FMC */
  // Unsupported MCU is active
  #error FMC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Master acces protection constants. */
#define FMC_PDD_NO_ACCESS        0U              /**< No access may be performed by master */
#define FMC_PDD_READ_ONLY_ACCESS 0x1U            /**< Only read accesses may be performed by master */

/* Wait state required to access the flash memory constants. */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_1  0U   /**< 1 wait state required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_2  0x1U /**< 2 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_3  0x2U /**< 3 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_4  0x3U /**< 4 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_5  0x4U /**< 5 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_6  0x5U /**< 6 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_7  0x6U /**< 7 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_8  0x7U /**< 8 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_9  0x8U /**< 9 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_10 0x9U /**< 10 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_11 0xAU /**< 11 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_12 0xBU /**< 12 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_13 0xCU /**< 13 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_14 0xDU /**< 14 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_15 0xEU /**< 15 wait states required to access the flash memory */
#define FMC_PDD_ACCESS_TIME_SYSTEM_CLOCK_16 0xFU /**< 16 wait states required to access the flash memory */

/* Cache lock way constants. */
#define FMC_PDD_CACHE_LOCK_WAY_0   0x1U          /**< Cache lock way 0 */
#define FMC_PDD_CACHE_LOCK_WAY_1   0x2U          /**< Cache lock way 1 */
#define FMC_PDD_CACHE_LOCK_WAY_2   0x4U          /**< Cache lock way 2 */
#define FMC_PDD_CACHE_LOCK_WAY_3   0x8U          /**< Cache lock way 3 */
#define FMC_PDD_CACHE_LOCK_WAY_ALL 0xFU          /**< Cache lock all ways */

/* Invalide flash cache way mask constants. */
#define FMC_PDD_CACHE_INVALIDATE_WAY_0   0x100000U /**< Cache invalidate way 0 mask */
#define FMC_PDD_CACHE_INVALIDATE_WAY_1   0x200000U /**< Cache invalidate way 1 mask */
#define FMC_PDD_CACHE_INVALIDATE_WAY_2   0x400000U /**< Cache invalidate way 2 mask */
#define FMC_PDD_CACHE_INVALIDATE_WAY_3   0x800000U /**< Cache invalidate way 3 mask */
#define FMC_PDD_CACHE_INVALIDATE_WAY_ALL 0xF00000U /**< Cache invalidate all ways mask */

/* Memory width constants. */
#define FMC_PDD_MEMORY_WIDTH_32BITS 0U           /**< Memory width 32 bits constant */
#define FMC_PDD_MEMORY_WIDTH_64BITS 0x20000U     /**< Memory width 64 bits constant */

/* Cache replacement control constants */
#define FMC_PDD_LRU_REPLACEMENT_FOR_ALL_4_WAYS    0U /**< LRU replacement algorithm per set across all four ways */
#define FMC_PDD_LRU_WITH_01_IFETCHES_23_DATA_WAYS 0x40U /**< Independent LRU with ways [0-1] for ifetches, [2-3] for data */
#define FMC_PDD_LRU_WITH_02_IFETCHES_3_DATA_WAYS  0x60U /**< Independent LRU with ways [0-2] for ifetches, [3] for data */


/* ----------------------------------------------------------------------------
   -- EnableMaster3Prefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables master 3 prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of master 3 prefetch. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableMaster3Prefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableMaster3Prefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M3PFD_MASK)) : ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M3PFD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableMaster2Prefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables master 2 prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of master 2 prefetch. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableMaster2Prefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableMaster2Prefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M2PFD_MASK)) : ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M2PFD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableMaster1Prefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables master 1 prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of master 1 prefetch. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableMaster1Prefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableMaster1Prefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M1PFD_MASK)) : ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M1PFD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableMaster0Prefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables master 0 prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of master 0 prefetch. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableMaster0Prefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableMaster0Prefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M0PFD_MASK)) : ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M0PFD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaster3AccessProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets read and write control to the flash memory on the logical master
 * number of the requesting crossbar switch master.
 * @param PeripheralBase Peripheral base address.
 * @param Value Master 3 acces protection value. The user should use one from
 *        the enumerated values. This parameter is of "Master acces protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetMaster3AccessProtection(deviceID, FMC_PDD_NO_ACCESS);
 *      @endcode
 */
#define FMC_PDD_SetMaster3AccessProtection(PeripheralBase, Value) ( \
    ((Value) == FMC_PDD_NO_ACCESS) ? ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M3AP_MASK)) : ( \
      PeriphSetBits32( \
       FMC_PFAPR_REG(PeripheralBase), \
       FMC_PFAPR_M3AP_MASK, \
       (uint32_t)((uint32_t)0x1U << FMC_PFAPR_M3AP_SHIFT) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaster2AccessProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets read and write control to the flash memory on the logical master
 * number of the requesting crossbar switch master.
 * @param PeripheralBase Peripheral base address.
 * @param Value Master 2 acces protection value. The user should use one from
 *        the enumerated values. This parameter is of "Master acces protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetMaster2AccessProtection(deviceID, FMC_PDD_NO_ACCESS);
 *      @endcode
 */
#define FMC_PDD_SetMaster2AccessProtection(PeripheralBase, Value) ( \
    ((Value) == FMC_PDD_NO_ACCESS) ? ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M2AP_MASK)) : ( \
      PeriphSetBits32( \
       FMC_PFAPR_REG(PeripheralBase), \
       FMC_PFAPR_M2AP_MASK, \
       (uint32_t)((uint32_t)0x1U << FMC_PFAPR_M2AP_SHIFT) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaster1AccessProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets read and write control to the flash memory on the logical master
 * number of the requesting crossbar switch master.
 * @param PeripheralBase Peripheral base address.
 * @param Value Master 1 acces protection value. The user should use one from
 *        the enumerated values. This parameter is of "Master acces protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetMaster1AccessProtection(deviceID, FMC_PDD_NO_ACCESS);
 *      @endcode
 */
#define FMC_PDD_SetMaster1AccessProtection(PeripheralBase, Value) ( \
    ((Value) == FMC_PDD_NO_ACCESS) ? ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M1AP_MASK)) : ( \
      PeriphSetBits32( \
       FMC_PFAPR_REG(PeripheralBase), \
       FMC_PFAPR_M1AP_MASK, \
       (uint32_t)((uint32_t)0x1U << FMC_PFAPR_M1AP_SHIFT) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaster0AccessProtection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets read and write control to the flash memory on the logical master
 * number of the requesting crossbar switch master.
 * @param PeripheralBase Peripheral base address.
 * @param Value Master 0 acces protection value. The user should use one from
 *        the enumerated values. This parameter is of "Master acces protection
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetMaster0AccessProtection(deviceID, FMC_PDD_NO_ACCESS);
 *      @endcode
 */
#define FMC_PDD_SetMaster0AccessProtection(PeripheralBase, Value) ( \
    ((Value) == FMC_PDD_NO_ACCESS) ? ( \
      PeriphClearBitMask32(FMC_PFAPR_REG(PeripheralBase), FMC_PFAPR_M0AP_MASK)) : ( \
      PeriphSetBits32( \
       FMC_PFAPR_REG(PeripheralBase), \
       FMC_PFAPR_M0AP_MASK, \
       (uint32_t)0x1U \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFlashAccessProtectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads flash access protection register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      uint32_t result = FMC_PDD_ReadFlashAccessProtectionReg(deviceID);
 *      @endcode
 */
#define FMC_PDD_ReadFlashAccessProtectionReg(PeripheralBase) ( \
    PeriphReadReg32(FMC_PFAPR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFlashAccessProtectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into flash access
 * protection register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the flash access protection register.
 *        This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFAPR.
 * @par Example:
 *      @code
 *      FMC_PDD_WriteFlashAccessProtectionReg(deviceID, 1);
 *      @endcode
 */
#define FMC_PDD_WriteFlashAccessProtectionReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(FMC_PFAPR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetReadWaitStateControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the wait state control value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      uint8_t result = FMC_PDD_GetReadWaitStateControl(deviceID);
 *      @endcode
 */
#define FMC_PDD_GetReadWaitStateControl(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0RWSC_MASK)) >> ( \
     FMC_PFB0CR_B0RWSC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCacheLockWayMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the cache lock way defined by mas parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Cache lock way mask. This parameter is a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetCacheLockWayMask(deviceID, 1);
 *      @endcode
 */
#define FMC_PDD_SetCacheLockWayMask(PeripheralBase, Mask) ( \
    PeriphSetBits32( \
     FMC_PFB0CR_REG(PeripheralBase), \
     FMC_PFB0CR_CLCK_WAY_MASK, \
     (uint32_t)((uint32_t)(Mask) << FMC_PFB0CR_CLCK_WAY_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCacheLockWay
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cache lock way mask.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      uint8_t result = FMC_PDD_GetCacheLockWay(deviceID);
 *      @endcode
 */
#define FMC_PDD_GetCacheLockWay(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_CLCK_WAY_MASK)) >> ( \
     FMC_PFB0CR_CLCK_WAY_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- InvalideFlashCacheWay
   ---------------------------------------------------------------------------- */

/**
 * @brief Invalide flash cache way requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of invalide flash cache way. This parameter is a 32-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_InvalideFlashCacheWay(deviceID, 1);
 *      @endcode
 */
#define FMC_PDD_InvalideFlashCacheWay(PeripheralBase, Mask) ( \
    PeriphSetBits32( \
     FMC_PFB0CR_REG(PeripheralBase), \
     FMC_PFB0CR_CINV_WAY_MASK, \
     (uint32_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- InvalidePrefetchSpeculationBuffer
   ---------------------------------------------------------------------------- */

/**
 * @brief Invalidate (clear) speculation buffer and single entry buffer.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_InvalidePrefetchSpeculationBuffer(deviceID);
 *      @endcode
 */
#define FMC_PDD_InvalidePrefetchSpeculationBuffer(PeripheralBase) ( \
    PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_S_B_INV_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- InvalidateFlashCache
   ---------------------------------------------------------------------------- */

/**
 * @brief Invalidates flash cache.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_InvalidateFlashCache(deviceID);
 *      @endcode
 */
#define FMC_PDD_InvalidateFlashCache(PeripheralBase) ( \
    PeriphSetBitMask32( \
     FMC_PFB0CR_REG(PeripheralBase), \
     (uint32_t)(FMC_PFB0CR_CINV_WAY_MASK | FMC_PFB0CR_S_B_INV_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetMemoryWidth
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the memory width.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Memory width constants." type. The value is cast
 *         to "uint32_t".
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      uint32_t result = FMC_PDD_GetMemoryWidth(deviceID);
 *      @endcode
 */
#define FMC_PDD_GetMemoryWidth(PeripheralBase) ( \
    PeriphGetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0MW_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetCacheReplacementControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the cache replacement control.
 * @param PeripheralBase Peripheral base address.
 * @param Control Cache replacement control value. The user should use one from
 *        the enumerated values. This parameter is of "Cache replacement control
 *        constants" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_SetCacheReplacementControl(deviceID,
 *      FMC_PDD_LRU_REPLACEMENT_FOR_ALL_4_WAYS);
 *      @endcode
 */
#define FMC_PDD_SetCacheReplacementControl(PeripheralBase, Control) ( \
    PeriphSetBits32( \
     FMC_PFB0CR_REG(PeripheralBase), \
     FMC_PFB0CR_CRC_MASK, \
     (uint32_t)(Control) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDataCache
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a data cache.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of data cache request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableDataCache(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableDataCache(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0DCE_MASK)) : ( \
      PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0DCE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInstructionCache
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a instruction cache.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of instruction cache request. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableInstructionCache(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableInstructionCache(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0ICE_MASK)) : ( \
      PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0ICE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDataPrefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a data prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of data prefetch request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableDataPrefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableDataPrefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0DPE_MASK)) : ( \
      PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0DPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInstructionPrefetch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a instruction prefetch.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of instruction prefetch request. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableInstructionPrefetch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableInstructionPrefetch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0IPE_MASK)) : ( \
      PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0IPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSingleEntryBuffer
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a single entry buffer.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of single entry buffer request. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_EnableSingleEntryBuffer(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define FMC_PDD_EnableSingleEntryBuffer(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0SEBE_MASK)) : ( \
      PeriphSetBitMask32(FMC_PFB0CR_REG(PeripheralBase), FMC_PFB0CR_B0SEBE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFlashControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads flash control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      uint32_t result = FMC_PDD_ReadFlashControlReg(deviceID);
 *      @endcode
 */
#define FMC_PDD_ReadFlashControlReg(PeripheralBase) ( \
    PeriphReadReg32(FMC_PFB0CR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFlashControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into flash control
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the flash control register. This
 *        parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FMC_PFB0CR.
 * @par Example:
 *      @code
 *      FMC_PDD_WriteFlashControlReg(deviceID, 1);
 *      @endcode
 */
#define FMC_PDD_WriteFlashControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(FMC_PFB0CR_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCacheTagStorageWaySetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads cache tag way and set storage register.
 * @param PeripheralBase Peripheral base address.
 * @param WayIdx Cache tag storage way index. This parameter is of index type.
 * @param SetIdx Cache tag storage set index. This parameter is of index type.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: TAGVD[WayIdx][SetIdx].
 * @par Example:
 *      @code
 *      uint32_t result = FMC_PDD_ReadCacheTagStorageWaySetReg(deviceID,
 *      periphID, periphID);
 *      @endcode
 */
#define FMC_PDD_ReadCacheTagStorageWaySetReg(PeripheralBase, WayIdx, SetIdx) ( \
    PeriphReadReg32(FMC_TAGVD_REG(PeripheralBase,(WayIdx),(SetIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCacheTagStorageWaySetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into cache tag way
 * and set storage register.
 * @param PeripheralBase Peripheral base address.
 * @param WayIdx Cache tag storage way index. This parameter is of index type.
 * @param SetIdx Cache tag storage set index. This parameter is of index type.
 * @param Value Value to be written to the cache tag way and set storage
 *        register. This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TAGVD[WayIdx][SetIdx].
 * @par Example:
 *      @code
 *      FMC_PDD_WriteCacheTagStorageWaySetReg(deviceID, periphID, periphID, 1);
 *      @endcode
 */
#define FMC_PDD_WriteCacheTagStorageWaySetReg(PeripheralBase, WayIdx, SetIdx, Value) ( \
    PeriphWriteReg32( \
     FMC_TAGVD_REG(PeripheralBase,(WayIdx),(SetIdx)), \
     (uint32_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCacheDataStorageWaySetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads cache data way and set storage register.
 * @param PeripheralBase Peripheral base address.
 * @param WayIdx Cache data storage way index. This parameter is of index type.
 * @param SetIdx Cache data storage set index. This parameter is of index type.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DATA[WayIdx][SetIdx].
 * @par Example:
 *      @code
 *      uint32_t result = FMC_PDD_ReadCacheDataStorageWaySetReg(deviceID,
 *      periphID, periphID);
 *      @endcode
 */
#define FMC_PDD_ReadCacheDataStorageWaySetReg(PeripheralBase, WayIdx, SetIdx) ( \
    PeriphReadReg32(FMC_DATA_REG(PeripheralBase,(WayIdx),(SetIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCacheDataStorageWaySetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into cache data way
 * and set storage register.
 * @param PeripheralBase Peripheral base address.
 * @param WayIdx Cache data storage way index. This parameter is of index type.
 * @param SetIdx Cache data storage set index. This parameter is of index type.
 * @param Value Value to be written to the cache data way and set storage
 *        register. This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DATA[WayIdx][SetIdx].
 * @par Example:
 *      @code
 *      FMC_PDD_WriteCacheDataStorageWaySetReg(deviceID, periphID, periphID, 1);
 *      @endcode
 */
#define FMC_PDD_WriteCacheDataStorageWaySetReg(PeripheralBase, WayIdx, SetIdx, Value) ( \
    PeriphWriteReg32( \
     FMC_DATA_REG(PeripheralBase,(WayIdx),(SetIdx)), \
     (uint32_t)(Value) \
    ) \
  )
#endif  /* #if defined(FMC_PDD_H_) */

/* FMC_PDD.h, eof. */
