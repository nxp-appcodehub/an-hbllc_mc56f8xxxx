/*
  PDD layer implementation for peripheral type DMA
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(DMA_PDD_H_)
#define DMA_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error DMA PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* DMA */ && \
      !defined(MCU_MC56F82316) /* DMA */ && \
      !defined(MCU_MC56F82723) /* DMA */ && \
      !defined(MCU_MC56F82726) /* DMA */ && \
      !defined(MCU_MC56F82728) /* DMA */ && \
      !defined(MCU_MC56F82733) /* DMA */ && \
      !defined(MCU_MC56F82736) /* DMA */ && \
      !defined(MCU_MC56F82738) /* DMA */ && \
      !defined(MCU_MC56F82743) /* DMA */ && \
      !defined(MCU_MC56F82746) /* DMA */ && \
      !defined(MCU_MC56F82748) /* DMA */
  // Unsupported MCU is active
  #error DMA PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* DMA channel constants. */
#define DMA_PDD_CHANNEL_0 0U                     /**< Channel 0. */
#define DMA_PDD_CHANNEL_1 0x1U                   /**< Channel 1. */
#define DMA_PDD_CHANNEL_2 0x2U                   /**< Channel 2. */
#define DMA_PDD_CHANNEL_3 0x3U                   /**< Channel 3. */

/* Interrupts' mask. */
#define DMA_PDD_TRANSFER_COMPLETE_INTERRUPT DMA_DCR_EINT_MASK /**< Transfer complete interrupt mask. */

/* Interrupts' flags. */
#define DMA_PDD_TRANSFER_COMPLETE_FLAG DMA_DSR_BCR_DONE_MASK /**< Transfer complete flag. */

/* Channel request source constants. */
#define DMA_PDD_CHANNEL_SOURCE_0  0U             /**< Channel request source 0. */
#define DMA_PDD_CHANNEL_SOURCE_1  0x1U           /**< Channel request source 1. */
#define DMA_PDD_CHANNEL_SOURCE_2  0x2U           /**< Channel request source 2. */
#define DMA_PDD_CHANNEL_SOURCE_3  0x3U           /**< Channel request source 3. */
#define DMA_PDD_CHANNEL_SOURCE_4  0x4U           /**< Channel request source 4. */
#define DMA_PDD_CHANNEL_SOURCE_5  0x5U           /**< Channel request source 5. */
#define DMA_PDD_CHANNEL_SOURCE_6  0x6U           /**< Channel request source 6. */
#define DMA_PDD_CHANNEL_SOURCE_7  0x7U           /**< Channel request source 7. */
#define DMA_PDD_CHANNEL_SOURCE_8  0x8U           /**< Channel request source 8. */
#define DMA_PDD_CHANNEL_SOURCE_9  0x9U           /**< Channel request source 9. */
#define DMA_PDD_CHANNEL_SOURCE_10 0xAU           /**< Channel request source 10. */
#define DMA_PDD_CHANNEL_SOURCE_11 0xBU           /**< Channel request source 11. */
#define DMA_PDD_CHANNEL_SOURCE_12 0xCU           /**< Channel request source 12. */
#define DMA_PDD_CHANNEL_SOURCE_13 0xDU           /**< Channel request source 13. */
#define DMA_PDD_CHANNEL_SOURCE_14 0xEU           /**< Channel request source 14. */
#define DMA_PDD_CHANNEL_SOURCE_15 0xFU           /**< Channel request source 15. */

/* Channel activity status constants. */
#define DMA_PDD_TRANSFER_DONE_FLAG            DMA_DSR_BCR_DONE_MASK /**< Transfer done flag. */
#define DMA_PDD_TRANSFER_BUSY_FLAG            DMA_DSR_BCR_BSY_MASK /**< Transfer in progress flag. */
#define DMA_PDD_TRANSFER_REQUEST_PENDING_FLAG DMA_DSR_BCR_REQ_MASK /**< Transfer request pending flag. */

/* Channel error status constants. */
#define DMA_PDD_CONFIGURATION_ERROR_FLAG  DMA_DSR_BCR_CE_MASK /**< Configuration error flag. */
#define DMA_PDD_ERROR_ON_SOURCE_FLAG      DMA_DSR_BCR_BES_MASK /**< Error on source side flag. */
#define DMA_PDD_ERROR_ON_DESTINATION_FLAG DMA_DSR_BCR_BED_MASK /**< Error on destination side flag. */

/* Transfer complete interrupt enable/disable constants. */
#define DMA_PDD_TRANSFER_COMPLETE_ENABLE  DMA_DCR_EINT_MASK /**< Transfer complete interrupt enabled. */
#define DMA_PDD_TRANSFER_COMPLETE_DISABLE 0U     /**< Transfer complete interrupt disabled. */

/* Circular buffer size constants. */
#define DMA_PDD_CIRCULAR_BUFFER_DISABLED   0U    /**< Circular buffer disabled. */
#define DMA_PDD_CIRCULAR_BUFFER_16_BYTES   0x1U  /**< 16 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_32_BYTES   0x2U  /**< 32 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_64_BYTES   0x3U  /**< 64 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_128_BYTES  0x4U  /**< 128 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_256_BYTES  0x5U  /**< 256 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_512_BYTES  0x6U  /**< 512 bytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_1_KBYTE    0x7U  /**< 1 Kbyte circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_2_KBYTES   0x8U  /**< 2 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_4_KBYTES   0x9U  /**< 4 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_8_KBYTES   0xAU  /**< 8 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_16_KBYTES  0xBU  /**< 16 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_32_KBYTES  0xCU  /**< 32 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_64_KBYTES  0xDU  /**< 64 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_128_KBYTES 0xEU  /**< 128 Kbytes circular buffer. */
#define DMA_PDD_CIRCULAR_BUFFER_256_KBYTES 0xFU  /**< 256 Kbytes circular buffer. */

/* DMA data transfer block size. */
#define DMA_PDD_8_BIT  0x1U                      /**< 8-bit transfer size. */
#define DMA_PDD_16_BIT 0x2U                      /**< 16-bit transfer size. */
#define DMA_PDD_32_BIT 0U                        /**< 32-bit transfer size. */

/* DMA data transfer size. */
#define DMA_PDD_LINKING_DISABLED                          0U /**< Channel linking disabled. */
#define DMA_PDD_CYCLE_STEAL_AND_TRANSFER_COMPLETE_LINKING 0x10U /**< Channel linked after each cycle-steal transfer and after transfer complete. */
#define DMA_PDD_CYCLE_STEAL_LINKING                       0x20U /**< Channel linked only after each cycle-steal transfer. */
#define DMA_PDD_TRANSFER_COMPLETE_LINKING                 0x30U /**< Channel linked only after transfer complete. */


/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Mask Mask of interrupts to enable. Use constants from group
 *        "Interrupts' mask.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableInterrupts(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_TRANSFER_COMPLETE_INTERRUPT);
 *      @endcode
 */
#define DMA_PDD_EnableInterrupts(PeripheralBase, Channel, Mask) ( \
    PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), (uint32_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Mask Mask of interrupts to disable. Use constants from group
 *        "Interrupts' mask.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_DisableInterrupts(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_TRANSFER_COMPLETE_INTERRUPT);
 *      @endcode
 */
#define DMA_PDD_DisableInterrupts(PeripheralBase, Channel, Mask) ( \
    PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), (uint32_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Interrupts' flags." for processing return
 *         value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetInterruptFlags(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetInterruptFlags(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_DONE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Mask Mask of interrupt's flags to clear. Use constants from group
 *        "Interrupts' flags.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_ClearInterruptFlags(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_TRANSFER_COMPLETE_FLAG);
 *      @endcode
 */
#define DMA_PDD_ClearInterruptFlags(PeripheralBase, Channel, Mask) ( \
    PeriphSetBitMask32(DMA_DSR_BCR_REG(PeripheralBase,(Channel)), (uint32_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRequestControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to DMA request control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value DMA request control register value. This parameter is a 32-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_WriteRequestControlReg(deviceID, 1);
 *      @endcode
 */
#define DMA_PDD_WriteRequestControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg32(DMA_REQC_REG(PeripheralBase), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRequestControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA request control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_ReadRequestControlReg(deviceID);
 *      @endcode
 */
#define DMA_PDD_ReadRequestControlReg(PeripheralBase) ( \
    PeriphReadReg32(DMA_REQC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannelSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel request source.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel. This parameter is a 2-bit value.
 * @param Value DMA channel request source. Use constants from group "Channel
 *        request source constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannelSource(deviceID, 1, DMA_PDD_CHANNEL_SOURCE_0);
 *      @endcode
 */
#define DMA_PDD_SetChannelSource(PeripheralBase, Channel, Value) ( \
    PeriphWriteReg32( \
     DMA_REQC_REG(PeripheralBase), \
     (uint32_t)(( \
      (uint32_t)(( \
       PeriphReadReg32(DMA_REQC_REG(PeripheralBase))) & ( \
       (uint32_t)(~(uint32_t)((uint32_t)0xFU << (uint8_t)((uint8_t)((uint8_t)(Channel) ^ 0x3U) << 3U)))))) | ( \
      (uint32_t)((uint32_t)(Value) << (uint8_t)((uint8_t)((uint8_t)(Channel) ^ 0x3U) << 3U)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannelSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel request source.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Channel request source constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannelSource(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetChannelSource(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     (uint8_t)(( \
      PeriphReadReg32(DMA_REQC_REG(PeripheralBase))) >> ( \
      (uint8_t)((uint8_t)((uint8_t)(Channel) ^ 0x3U) << 3U)))) & ( \
     (uint8_t)0xFU)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearStateMachine
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears DMA channel state machine.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel. This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_ClearStateMachine(deviceID, 1);
 *      @endcode
 */
#define DMA_PDD_ClearStateMachine(PeripheralBase, Channel) ( \
    PeriphSetBitMask32( \
     DMA_REQC_REG(PeripheralBase), \
     (uint32_t)((uint32_t)0x80U << (uint8_t)((uint8_t)((uint8_t)(Channel) ^ 0x3U) << 3U)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannel0Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel 0 request source.
 * @param PeripheralBase Peripheral base address.
 * @param Value DMA channel request source. Use constants from group "Channel
 *        request source constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannel0Source(deviceID, DMA_PDD_CHANNEL_SOURCE_0);
 *      @endcode
 */
#define DMA_PDD_SetChannel0Source(PeripheralBase, Value) ( \
    PeriphSetBits32( \
     DMA_REQC_REG(PeripheralBase), \
     DMA_REQC_DMAC0_MASK, \
     (uint32_t)((uint32_t)(Value) << DMA_REQC_DMAC0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannel0Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel 0 request source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Channel request source constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannel0Source(deviceID);
 *      @endcode
 */
#define DMA_PDD_GetChannel0Source(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_REQC_REG(PeripheralBase), DMA_REQC_DMAC0_MASK)) >> ( \
     DMA_REQC_DMAC0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannel1Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel 1 request source.
 * @param PeripheralBase Peripheral base address.
 * @param Value DMA channel request source. Use constants from group "Channel
 *        request source constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannel1Source(deviceID, DMA_PDD_CHANNEL_SOURCE_0);
 *      @endcode
 */
#define DMA_PDD_SetChannel1Source(PeripheralBase, Value) ( \
    PeriphSetBits32( \
     DMA_REQC_REG(PeripheralBase), \
     DMA_REQC_DMAC1_MASK, \
     (uint32_t)((uint32_t)(Value) << DMA_REQC_DMAC1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannel1Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel 1 request source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Channel request source constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannel1Source(deviceID);
 *      @endcode
 */
#define DMA_PDD_GetChannel1Source(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_REQC_REG(PeripheralBase), DMA_REQC_DMAC1_MASK)) >> ( \
     DMA_REQC_DMAC1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannel2Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel 2 request source.
 * @param PeripheralBase Peripheral base address.
 * @param Value DMA channel request source. Use constants from group "Channel
 *        request source constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannel2Source(deviceID, DMA_PDD_CHANNEL_SOURCE_0);
 *      @endcode
 */
#define DMA_PDD_SetChannel2Source(PeripheralBase, Value) ( \
    PeriphSetBits32( \
     DMA_REQC_REG(PeripheralBase), \
     DMA_REQC_DMAC2_MASK, \
     (uint32_t)((uint32_t)(Value) << DMA_REQC_DMAC2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannel2Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel 2 request source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Channel request source constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannel2Source(deviceID);
 *      @endcode
 */
#define DMA_PDD_GetChannel2Source(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_REQC_REG(PeripheralBase), DMA_REQC_DMAC2_MASK)) >> ( \
     DMA_REQC_DMAC2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannel3Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel 3 request source.
 * @param PeripheralBase Peripheral base address.
 * @param Value DMA channel request source. Use constants from group "Channel
 *        request source constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannel3Source(deviceID, DMA_PDD_CHANNEL_SOURCE_0);
 *      @endcode
 */
#define DMA_PDD_SetChannel3Source(PeripheralBase, Value) ( \
    PeriphSetBits32( \
     DMA_REQC_REG(PeripheralBase), \
     DMA_REQC_DMAC3_MASK, \
     (uint32_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannel3Source
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel 3 request source.
 * @param PeripheralBase Peripheral base address.
 * @return Use constants from group "Channel request source constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DMA_REQC.
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannel3Source(deviceID);
 *      @endcode
 */
#define DMA_PDD_GetChannel3Source(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask32(DMA_REQC_REG(PeripheralBase), DMA_REQC_DMAC3_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetSourceAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel source address.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Address DMA channel source address. This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SAR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetSourceAddress(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_SetSourceAddress(PeripheralBase, Channel, Address) ( \
    PeriphWriteReg32(DMA_SAR_REG(PeripheralBase,(Channel)), (uint32_t)(Address)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSourceAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel source address.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: SAR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetSourceAddress(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetSourceAddress(PeripheralBase, Channel) ( \
    PeriphReadReg32(DMA_SAR_REG(PeripheralBase,(Channel))) \
  )

/* ----------------------------------------------------------------------------
   -- SetDestinationAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel destination address.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Address DMA channel destination address. This parameter is a 32-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DAR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetDestinationAddress(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_SetDestinationAddress(PeripheralBase, Channel, Address) ( \
    PeriphWriteReg32(DMA_DAR_REG(PeripheralBase,(Channel)), (uint32_t)(Address)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDestinationAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel destination address.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DAR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetDestinationAddress(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetDestinationAddress(PeripheralBase, Channel) ( \
    PeriphReadReg32(DMA_DAR_REG(PeripheralBase,(Channel))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusByteCountReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to DMA channel status and byte count control register.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value DMA channel request control register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_WriteStatusByteCountReg(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_WriteStatusByteCountReg(PeripheralBase, Channel, Value) ( \
    PeriphWriteReg32(DMA_DSR_BCR_REG(PeripheralBase,(Channel)), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusByteCountRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Deprecated. Use WriteStatusByteCountReg PDD macro instead.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value DMA channel request control register value. This parameter is a
 *        32-bit value.
 * @return Returns a value of void type.
 * @par Example:
 *      @code
 *      DMA_PDD_WriteStatusByteCountRegister(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_WriteStatusByteCountRegister(PeripheralBase, Channel, Value) ( \
    DMA_PDD_WriteStatusByteCountReg(PeripheralBase, (Channel), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusByteCountReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel status and byte count register.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_ReadStatusByteCountReg(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_ReadStatusByteCountReg(PeripheralBase, Channel) ( \
    PeriphReadReg32(DMA_DSR_BCR_REG(PeripheralBase,(Channel))) \
  )

/* ----------------------------------------------------------------------------
   -- GetStatusByteCountRegister
   ---------------------------------------------------------------------------- */

/**
 * @brief Deprecated. Use ReadStatusByteCountReg PDD macro instead.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetStatusByteCountRegister(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetStatusByteCountRegister(PeripheralBase, Channel) ( \
    PeriphReadReg32(DMA_DSR_BCR_REG(PeripheralBase,(Channel))) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannelActivityFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel request pending, busy and done status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Channel activity status constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetChannelActivityFlags(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetChannelActivityFlags(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     (uint32_t)(DMA_DSR_BCR_REQ_MASK | (DMA_DSR_BCR_BSY_MASK | DMA_DSR_BCR_DONE_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearDoneFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears channel done status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_ClearDoneFlag(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_ClearDoneFlag(PeripheralBase, Channel) ( \
    PeriphSetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_DONE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDoneFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns channel done status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetDoneFlag(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetDoneFlag(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_DONE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBusyFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns channel busy status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetBusyFlag(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetBusyFlag(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_BSY_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetRequestPendingFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns channel request pending status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetRequestPendingFlag(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetRequestPendingFlag(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_REQ_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannelErrorFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel configuration error, bus error on source and bus
 * error on destination status.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Channel error status constants." for
 *         processing return value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetChannelErrorFlags(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetChannelErrorFlags(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     (uint32_t)(DMA_DSR_BCR_CE_MASK | (DMA_DSR_BCR_BES_MASK | DMA_DSR_BCR_BED_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetByteCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel byte transfer count.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value 24-bit DMA channel byte count. This parameter is a 24-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetByteCount(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_SetByteCount(PeripheralBase, Channel, Value) ( \
    PeriphSafeSetBits32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_DONE_MASK, \
     DMA_DSR_BCR_BCR_MASK, \
     (uint32_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetByteCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel byte transfer count.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetByteCount(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetByteCount(PeripheralBase, Channel) ( \
    PeriphGetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_BCR_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to DMA channel control register.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value DMA channel control register value. This parameter is a 32-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_WriteControlReg(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_WriteControlReg(PeripheralBase, Channel, Value) ( \
    PeriphWriteReg32(DMA_DCR_REG(PeripheralBase,(Channel)), (uint32_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA control register.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_ReadControlReg(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_ReadControlReg(PeripheralBase, Channel) ( \
    PeriphReadReg32(DMA_DCR_REG(PeripheralBase,(Channel))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTransferCompleteInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables channel transfer complete interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel transfer complete interrupt
 *        will be enabled or disabled. This parameter is of "Global enumeration
 *        used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableTransferCompleteInterrupt(deviceID, DMA_PDD_CHANNEL_0,
 *      PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableTransferCompleteInterrupt(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_EINT_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_EINT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTransferCompleteInterruptEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transfer complete enable interrupt state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetTransferCompleteInterruptEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetTransferCompleteInterruptEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_EINT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralRequest
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables peripheral requests.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel peripheral requests will be
 *        enabled or ignored. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnablePeripheralRequest(deviceID, DMA_PDD_CHANNEL_0,
 *      PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnablePeripheralRequest(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_ERQ_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_ERQ_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPeripheralRequestEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns peripheral requests enable state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetPeripheralRequestEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetPeripheralRequestEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_ERQ_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCycleSteal
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables cycle steal mode (single read/write transfer per
 * request).
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel cycle-steal mode will be
 *        enabled or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableCycleSteal(deviceID, DMA_PDD_CHANNEL_0, PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableCycleSteal(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCycleStealEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns cycle steal mode state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetCycleStealEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetCycleStealEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableContinuousMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables continuous mode (whole transfer per request).
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel continuous mode will be
 *        enabled or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableContinuousMode(deviceID, DMA_PDD_CHANNEL_0, PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableContinuousMode(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK)) : ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetContinuousModeEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns continuous mode state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetContinuousModeEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetContinuousModeEnabled(PeripheralBase, Channel) ( \
    (( \
      PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_CS_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAutoAlign
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables auto-align mode.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel transfers will be transfers
 *        are optimized based on the address and size or not. This parameter is
 *        of "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableAutoAlign(deviceID, DMA_PDD_CHANNEL_0, PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableAutoAlign(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_AA_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_AA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAutoAlignEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns auto-align mode state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetAutoAlignEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetAutoAlignEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_AA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSourceAddressIncrement
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables source address incrementation.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel source address increments
 *        after each successful transfer (according to transfer size) or not. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableSourceAddressIncrement(deviceID, DMA_PDD_CHANNEL_0,
 *      PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableSourceAddressIncrement(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_SINC_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_SINC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSourceAddressIncrementEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns source address incrementation enable state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetSourceAddressIncrementEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetSourceAddressIncrementEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_SINC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetSourceDataTransferSize
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel source data tranfer size.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Size DMA channel source data tranfer block size. This parameter is of
 *        "DMA data transfer block size." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetSourceDataTransferSize(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_8_BIT);
 *      @endcode
 */
#define DMA_PDD_SetSourceDataTransferSize(PeripheralBase, Channel, Size) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_SSIZE_MASK, \
     (uint32_t)((uint32_t)(Size) << DMA_DCR_SSIZE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetSourceDataTransferSize
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel source data tranfer size.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetSourceDataTransferSize(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetSourceDataTransferSize(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_SSIZE_MASK)) >> ( \
     DMA_DCR_SSIZE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDestinationAddressIncrement
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables destination address incrementation.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel destination address
 *        increments after each successful transfer (according to transfer size) or not.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableDestinationAddressIncrement(deviceID, DMA_PDD_CHANNEL_0,
 *      PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableDestinationAddressIncrement(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_DINC_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_DINC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDestinationAddressIncrementEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns destination address incrementation enable state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result =
 *      DMA_PDD_GetDestinationAddressIncrementEnabled(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetDestinationAddressIncrementEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_DINC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetDestinationDataTransferSize
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel destination data tranfer size.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Size DMA channel destination data tranfer block size. This parameter
 *        is of "DMA data transfer block size." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetDestinationDataTransferSize(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_8_BIT);
 *      @endcode
 */
#define DMA_PDD_SetDestinationDataTransferSize(PeripheralBase, Channel, Size) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_DSIZE_MASK, \
     (uint32_t)((uint32_t)(Size) << DMA_DCR_DSIZE_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDestinationDataTransferSize
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel destination data tranfer size.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetDestinationDataTransferSize(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetDestinationDataTransferSize(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_DSIZE_MASK)) >> ( \
     DMA_DCR_DSIZE_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- StartTransfer
   ---------------------------------------------------------------------------- */

/**
 * @brief Starts DMA channel transfer request via a software initiated service
 * request. If AllChannels = '1' then transfer start is requested for all channels
 * regardless of Channel parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_StartTransfer(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_StartTransfer(PeripheralBase, Channel) ( \
    PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_START_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- CancelTransfer
   ---------------------------------------------------------------------------- */

/**
 * @brief Cancels remaining data transfer.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DSR_BCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_CancelTransfer(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_CancelTransfer(PeripheralBase, Channel) ( \
    PeriphSetBitMask32( \
     DMA_DSR_BCR_REG(PeripheralBase,(Channel)), \
     DMA_DSR_BCR_DONE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetSourceAddressModulo
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel source address modulo.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Modulo DMA channel source address modulo. Use constants from group
 *        "Circular buffer size constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetSourceAddressModulo(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_CIRCULAR_BUFFER_DISABLED);
 *      @endcode
 */
#define DMA_PDD_SetSourceAddressModulo(PeripheralBase, Channel, Modulo) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_SMOD_MASK, \
     (uint32_t)((uint32_t)(Modulo) << DMA_DCR_SMOD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetSourceAddressModulo
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel source address modulo.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Circular buffer size constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetSourceAddressModulo(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetSourceAddressModulo(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_SMOD_MASK)) >> ( \
     DMA_DCR_SMOD_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDestinationAddressModulo
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel destination address modulo.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Modulo DMA channel destination address modulo. Use constants from
 *        group "Circular buffer size constants.". This parameter is 4 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetDestinationAddressModulo(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_CIRCULAR_BUFFER_DISABLED);
 *      @endcode
 */
#define DMA_PDD_SetDestinationAddressModulo(PeripheralBase, Channel, Modulo) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_DMOD_MASK, \
     (uint32_t)((uint32_t)(Modulo) << DMA_DCR_DMOD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetDestinationAddressModulo
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel destination address modulo.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Use constants from group "Circular buffer size constants." for
 *         processing return value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetDestinationAddressModulo(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetDestinationAddressModulo(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_DMOD_MASK)) >> ( \
     DMA_DCR_DMOD_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRequestAutoDisable
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA channel automaticall request clearing.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param State Parameter specifying if DMA channel automaticall request
 *        clearing will be enabled or disabled. This parameter is of "Global enumeration
 *        used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_EnableRequestAutoDisable(deviceID, DMA_PDD_CHANNEL_0,
 *      PDD_DISABLE);
 *      @endcode
 */
#define DMA_PDD_EnableRequestAutoDisable(PeripheralBase, Channel, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_D_REQ_MASK)) : ( \
      PeriphSetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_D_REQ_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRequestAutoDisableEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns automaticall request clearing state.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint32_t result = DMA_PDD_GetRequestAutoDisableEnabled(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetRequestAutoDisableEnabled(PeripheralBase, Channel) ( \
    PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_D_REQ_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannelLinkingMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel linking mode.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Mode DMA channel linking mode. This parameter is of "DMA data transfer
 *        size." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetChannelLinkingMode(deviceID, DMA_PDD_CHANNEL_0,
 *      DMA_PDD_LINKING_DISABLED);
 *      @endcode
 */
#define DMA_PDD_SetChannelLinkingMode(PeripheralBase, Channel, Mode) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_LINKCC_MASK, \
     (uint32_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetChannelLinkingMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel linking mode.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetChannelLinkingMode(deviceID,
 *      DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetChannelLinkingMode(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_LINKCC_MASK)) >> ( \
     DMA_DCR_LINKCC_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetLinkChannel1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel link 1.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value Linked DMA channel number. Use constants from group "DMA channel
 *        constants". This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetLinkChannel1(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_SetLinkChannel1(PeripheralBase, Channel, Value) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_LCH1_MASK, \
     (uint32_t)((uint32_t)(Value) << DMA_DCR_LCH1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetLinkChannel1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel link 1 channel number. Use constants from group
 * "DMA channel constants" for processing return value.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetLinkChannel1(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetLinkChannel1(PeripheralBase, Channel) ( \
    (uint8_t)(( \
     PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_LCH1_MASK)) >> ( \
     DMA_DCR_LCH1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetLinkChannel2
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets DMA channel link 2.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @param Value Linked DMA channel number. Use constants from group "DMA channel
 *        constants". This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      DMA_PDD_SetLinkChannel2(deviceID, DMA_PDD_CHANNEL_0, 1);
 *      @endcode
 */
#define DMA_PDD_SetLinkChannel2(PeripheralBase, Channel, Value) ( \
    PeriphSetBits32( \
     DMA_DCR_REG(PeripheralBase,(Channel)), \
     DMA_DCR_LCH2_MASK, \
     (uint32_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetLinkChannel2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns DMA channel link 2 channel number. Use constants from group
 * "DMA channel constants" for processing return value.
 * @param PeripheralBase Peripheral base address.
 * @param Channel DMA channel number. Use constants from group "DMA channel
 *        constants.". This parameter is 2 bits wide.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DCR[Channel].
 * @par Example:
 *      @code
 *      uint8_t result = DMA_PDD_GetLinkChannel2(deviceID, DMA_PDD_CHANNEL_0);
 *      @endcode
 */
#define DMA_PDD_GetLinkChannel2(PeripheralBase, Channel) ( \
    (uint8_t)PeriphGetBitMask32(DMA_DCR_REG(PeripheralBase,(Channel)), DMA_DCR_LCH2_MASK) \
  )
#endif  /* #if defined(DMA_PDD_H_) */

/* DMA_PDD.h, eof. */
