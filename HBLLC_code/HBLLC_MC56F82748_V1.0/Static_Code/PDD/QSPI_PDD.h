/*
  PDD layer implementation for peripheral type QSPI
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(QSPI_PDD_H_)
#define QSPI_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error QSPI PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* QSPI0 */ && \
      !defined(MCU_MC56F82316) /* QSPI0 */ && \
      !defined(MCU_MC56F82723) /* QSPI0 */ && \
      !defined(MCU_MC56F82726) /* QSPI0 */ && \
      !defined(MCU_MC56F82728) /* QSPI0, QSPI1 */ && \
      !defined(MCU_MC56F82733) /* QSPI0 */ && \
      !defined(MCU_MC56F82736) /* QSPI0 */ && \
      !defined(MCU_MC56F82738) /* QSPI0, QSPI1 */ && \
      !defined(MCU_MC56F82743) /* QSPI0 */ && \
      !defined(MCU_MC56F82746) /* QSPI0 */ && \
      !defined(MCU_MC56F82748) /* QSPI0, QSPI1 */
  // Unsupported MCU is active
  #error QSPI PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Baud rate divisor constants */
#define QSPI_PDD_BAUD_RATE_DIV_2    0U           /**< Baud rate divisor 2 */
#define QSPI_PDD_BAUD_RATE_DIV_4    0x1U         /**< Baud rate divisor 4 */
#define QSPI_PDD_BAUD_RATE_DIV_8    0x2U         /**< Baud rate divisor 8 */
#define QSPI_PDD_BAUD_RATE_DIV_16   0x3U         /**< Baud rate divisor 16 */
#define QSPI_PDD_BAUD_RATE_DIV_32   0x4U         /**< Baud rate divisor 32 */
#define QSPI_PDD_BAUD_RATE_DIV_64   0x5U         /**< Baud rate divisor 64 */
#define QSPI_PDD_BAUD_RATE_DIV_128  0x6U         /**< Baud rate divisor 128 */
#define QSPI_PDD_BAUD_RATE_DIV_256  0x7U         /**< Baud rate divisor 256 */
#define QSPI_PDD_BAUD_RATE_DIV_512  0x8U         /**< Baud rate divisor 512 */
#define QSPI_PDD_BAUD_RATE_DIV_1024 0x9U         /**< Baud rate divisor 1024 */
#define QSPI_PDD_BAUD_RATE_DIV_2048 0xAU         /**< Baud rate divisor 2048 */
#define QSPI_PDD_BAUD_RATE_DIV_4096 0xBU         /**< Baud rate divisor 4096 */

/* Rx buffer full, Tx buffer empty and error interrupt masks constant */
#define QSPI_PDD_ERROR           QSPI_SPSCR_ERRIE_MASK /**< Error interrupt mask */
#define QSPI_PDD_RX_BUFFER_FULL  QSPI_SPSCR_SPRIE_MASK /**< Receiver buffer full mask */
#define QSPI_PDD_TX_BUFFER_EMPTY QSPI_SPSCR_SPTIE_MASK /**< Transmitter buffer empty mask */

/* Status flags constants. */
#define QSPI_PDD_RX_BUFFER_FULL_FLAG     QSPI_SPSCR_SPRF_MASK /**< Receive buffer or FIFO full flag */
#define QSPI_PDD_RX_BUFFER_OVERFLOW_FLAG QSPI_SPSCR_OVRF_MASK /**< Receive buffer everflow flag */
#define QSPI_PDD_MASTER_MODE_FAULT_FLAG  QSPI_SPSCR_MODF_MASK /**< Master mode fault flag */
#define QSPI_PDD_TX_BUFFER_EMPTY_FLAG    QSPI_SPSCR_SPTE_MASK /**< Transmit buffer or FIFO empty flag */

/* SPI data shift order constants */
#define QSPI_PDD_LSB_FIRST 0x1000U               /**< Data transfers start with least significant bit */
#define QSPI_PDD_MSB_FIRST 0U                    /**< Data transfers start with most significant bit */

/* SPI mode constants. */
#define QSPI_PDD_MASTER_MODE 0x100U              /**< Module configured as a master */
#define QSPI_PDD_SLAVE_MODE  0U                  /**< Module configured as a slave */

/* SPI clock polarity constants. */
#define QSPI_PDD_ACTIVE_HIGH 0U                  /**< Active-high SPI clock (idles low, rising edge of SPI clock starts transaction) */
#define QSPI_PDD_ACTIVE_LOW  0x80U               /**< Active-low SPI clock (idles high, falling edge of SPI clock starts transaction) */

/* SPI clock phase constants. */
#define QSPI_PDD_FIRST_EDGE  0U                  /**< First edge on SPSCK occurs at the middle of the first cycle of a data transfer */
#define QSPI_PDD_SECOND_EDGE 0x40U               /**< First edge on SPSCK occurs at the start of the first cycle of a data transfer */

/* Slave select pin level constants. */
#define QSPI_PDD_SS_IN_LOW  0U                   /**< Slave select pin in low level */
#define QSPI_PDD_SS_IN_HIGH 0x800U               /**< Slave select pin in high level */

/* Slave select pin level constants. */
#define QSPI_PDD_SS_TO_HIGH 0x400U               /**< Slave select pin in high level */
#define QSPI_PDD_SS_TO_LOW  0U                   /**< Slave select pin in low level */

/* Master mode constants. */
#define QSPI_PDD_SS_AS_INPUT  0U                 /**< Slave select as input pin */
#define QSPI_PDD_SS_AS_OUTPUT 0x80U              /**< Slave select as output pin */

/* SPI transaction data size constants. */
#define QSPI_PDD_2_BITS  0x1U                    /**< 2-bits transaction data size */
#define QSPI_PDD_3_BITS  0x2U                    /**< 3-bits transaction data size */
#define QSPI_PDD_4_BITS  0x3U                    /**< 4-bits transaction data size */
#define QSPI_PDD_5_BITS  0x4U                    /**< 5-bits transaction data size */
#define QSPI_PDD_6_BITS  0x5U                    /**< 6-bits transaction data size */
#define QSPI_PDD_7_BITS  0x6U                    /**< 7-bits transaction data size */
#define QSPI_PDD_8_BITS  0x7U                    /**< 8-bits transaction data size */
#define QSPI_PDD_9_BITS  0x8U                    /**< 9-bits transaction data size */
#define QSPI_PDD_10_BITS 0x9U                    /**< 10-bits transaction data size */
#define QSPI_PDD_11_BITS 0xAU                    /**< 11-bits transaction data size */
#define QSPI_PDD_12_BITS 0xBU                    /**< 12-bits transaction data size */
#define QSPI_PDD_13_BITS 0xCU                    /**< 13-bits transaction data size */
#define QSPI_PDD_14_BITS 0xDU                    /**< 14-bits transaction data size */
#define QSPI_PDD_15_BITS 0xEU                    /**< 15-bits transaction data size */
#define QSPI_PDD_16_BITS 0xFU                    /**< 16-bits transaction data size */

/* Transmit FIFO nearly empty watermark constants. */
#define QSPI_PDD_TX_FIFO_EMPTY     0U            /**< Transmit interrupt active when Tx FIFO is empty */
#define QSPI_PDD_TX_FIFO_1_OR_LESS 0x20U         /**< Transmit interrupt active when Tx FIFO has one or fewer words available */
#define QSPI_PDD_TX_FIFO_2_OR_LESS 0x40U         /**< Transmit interrupt active when Tx FIFO has two or fewer words available */
#define QSPI_PDD_TX_FIFO_3_OR_LESS 0x60U         /**< Transmit interrupt active when Tx FIFO has three or fewer words available */

/* Receive FIFO nearly full watermark constants. */
#define QSPI_PDD_RX_FIFO_1_OR_MORE 0U            /**< Receive interrupt active when Rx FIFO has at least one word used */
#define QSPI_PDD_RX_FIFO_2_OR_MORE 0x4U          /**< Receive interrupt active when Rx FIFO has at least two words used */
#define QSPI_PDD_RX_FIFO_3_OR_MORE 0x8U          /**< Receive interrupt active when Rx FIFO has at least three words used */
#define QSPI_PDD_RX_FIFO_4_OR_MORE 0xCU          /**< Receive interrupt active when Rx FIFO has at least four words used */


/* ----------------------------------------------------------------------------
   -- SetBaudRateDivisor
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SPI baud rate divisor.
 * @param PeripheralBase Peripheral base address.
 * @param Divisor Baud rate divisor select. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR,
 *          QSPI0_SPDSR, QSPI1_SPSCR, QSPI1_SPDSR (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetBaudRateDivisor(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_SetBaudRateDivisor(PeripheralBase, Divisor) ( \
    PeriphSetBits16( \
     QSPI_SPSCR_REG(PeripheralBase), \
     QSPI_SPSCR_SPR_MASK, \
     (uint16_t)((uint16_t)((uint16_t)(Divisor) & 0x7U) << QSPI_SPSCR_SPR_SHIFT) \
    ), \
    PeriphSetBits16( \
     QSPI_SPDSR_REG(PeripheralBase), \
     QSPI_SPDSR_SPR3_MASK, \
     (uint16_t)(( \
      (uint16_t)((uint16_t)((uint16_t)(Divisor) >> 3U) & 0x1U)) << ( \
      QSPI_SPDSR_SPR3_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDataShiftOrder
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SPI data shift order.
 * @param PeripheralBase Peripheral base address.
 * @param Order SPI data shift order value. The user should use one from the
 *        enumerated values. This parameter is of "SPI data shift order constants"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetDataShiftOrder(deviceID, QSPI_PDD_LSB_FIRST);
 *      @endcode
 */
#define QSPI_PDD_SetDataShiftOrder(PeripheralBase, Order) ( \
    ((Order) == QSPI_PDD_MSB_FIRST) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_DSO_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_DSO_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableInterruptMask(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_EnableInterruptMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_DisableInterruptMask(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_DisableInterruptMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFaultMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a fault mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of fault mode request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableFaultMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableFaultMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_MODFEN_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_MODFEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMasterSlaveMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Select master or slave mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode SPI mode value. The user should use one from the enumerated
 *        values. This parameter is of "SPI mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetMasterSlaveMode(deviceID, QSPI_PDD_MASTER_MODE);
 *      @endcode
 */
#define QSPI_PDD_SetMasterSlaveMode(PeripheralBase, Mode) ( \
    ((Mode) == QSPI_PDD_SLAVE_MODE) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_SPMSTR_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_SPMSTR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SPI clock polarity.
 * @param PeripheralBase Peripheral base address.
 * @param Polarity SPI polarity value. The user should use one from the
 *        enumerated values. This parameter is of "SPI clock polarity constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetClockPolarity(deviceID, QSPI_PDD_ACTIVE_HIGH);
 *      @endcode
 */
#define QSPI_PDD_SetClockPolarity(PeripheralBase, Polarity) ( \
    ((Polarity) == QSPI_PDD_ACTIVE_HIGH) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_CPOL_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_CPOL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockPhase
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SPI clock phase.
 * @param PeripheralBase Peripheral base address.
 * @param Phase SPI phase value. The user should use one from the enumerated
 *        values. This parameter is of "SPI clock phase constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetClockPhase(deviceID, QSPI_PDD_FIRST_EDGE);
 *      @endcode
 */
#define QSPI_PDD_SetClockPhase(PeripheralBase, Phase) ( \
    ((Phase) == QSPI_PDD_FIRST_EDGE) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_CPHA_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_CPHA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables SPI device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of device. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_SPE_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPSCR_REG(PeripheralBase), QSPI_SPSCR_SPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadStatusFlags(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadStatusFlags(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPSCR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDataFeaturesReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets data transmission features(shift order, clock polarity and phase)
 * defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of data features requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetDataFeaturesReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_SetDataFeaturesReg(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     QSPI_SPSCR_REG(PeripheralBase), \
     (uint16_t)(QSPI_SPSCR_DSO_MASK | (QSPI_SPSCR_CPHA_MASK | QSPI_SPSCR_CPOL_MASK)), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusAndControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the status and control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadStatusAndControlReg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadStatusAndControlReg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPSCR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusAndControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the status and control register.
 * @param PeripheralBase Peripheral base address.
 * @param Data 16 bits value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPSCR, QSPI1_SPSCR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteStatusAndControlReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteStatusAndControlReg(PeripheralBase, Data) ( \
    PeriphWriteReg16(QSPI_SPSCR_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWiredOrMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a Wired-OR mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of wired-or mode request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableWiredOrMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableWiredOrMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_WOM_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_WOM_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTransmitDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a transmit DMA request.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of transmit DMA request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableTransmitDma(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableTransmitDma(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_TDMAEN_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_TDMAEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableReceiveDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a receive DMA request.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of receive DMA request. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableReceiveDma(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableReceiveDma(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_RDMAEN_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_RDMAEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBaudDivisorTimes2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a baud divisor times by 2.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of baud divisor times. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableBaudDivisorTimes2(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableBaudDivisorTimes2(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_BD2X_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_BD2X_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSlaveSelectPinValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current slave select pin state value.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Slave select pin level constants." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_GetSlaveSelectPinValue(deviceID);
 *      @endcode
 */
#define QSPI_PDD_GetSlaveSelectPinValue(PeripheralBase) ( \
    PeriphGetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_IN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetSlaveSelectPinLevel
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets slave select pin level.
 * @param PeripheralBase Peripheral base address.
 * @param Level Slave select pin level value. The user should use one from the
 *        enumerated values. This parameter is of "Slave select pin level
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetSlaveSelectPinLevel(deviceID, QSPI_PDD_SS_TO_HIGH);
 *      @endcode
 */
#define QSPI_PDD_SetSlaveSelectPinLevel(PeripheralBase, Level) ( \
    ((Level) == QSPI_PDD_SS_TO_LOW) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_DATA_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_DATA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveSelectOpenDrainMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a open drain mode on slave select .
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of open drain mode. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableSlaveSelectOpenDrainMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableSlaveSelectOpenDrainMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_ODM_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_ODM_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveSelectAutomaticMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a slave select pin automatic mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of slave select automatic mode. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableSlaveSelectAutomaticMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableSlaveSelectAutomaticMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_AUTO_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_AUTO_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSlaveSelectPinDataDirection
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets direction of the slave select pin.
 * @param PeripheralBase Peripheral base address.
 * @param Direction Slave select pin direction value. The user should use one
 *        from the enumerated values. This parameter is of "Master mode constants."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetSlaveSelectPinDataDirection(deviceID, QSPI_PDD_SS_AS_INPUT);
 *      @endcode
 */
#define QSPI_PDD_SetSlaveSelectPinDataDirection(PeripheralBase, Direction) ( \
    ((Direction) == QSPI_PDD_SS_AS_INPUT) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_DDR_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_DDR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveSelectStrobeMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a slave select pin strobe mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of slave select strobe mode. This parameter is
 *        of "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableSlaveSelectStrobeMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableSlaveSelectStrobeMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_STRB_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_STRB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveSelectOverrideMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables a slave select pin override mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of slave select override mode. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableSlaveSelectOverrideMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableSlaveSelectOverrideMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_OVER_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_SSB_OVER_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetWordLength
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the SPI word length.
 * @param PeripheralBase Peripheral base address.
 * @param Length SPI transaction data size value. The user should use one from
 *        the enumerated values. This parameter is of "SPI transaction data size
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetWordLength(deviceID, QSPI_PDD_2_BITS);
 *      @endcode
 */
#define QSPI_PDD_SetWordLength(PeripheralBase, Length) ( \
    PeriphSetBits16( \
     QSPI_SPDSR_REG(PeripheralBase), \
     QSPI_SPDSR_DS_MASK, \
     (uint16_t)(Length) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetWordLength
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current data word length.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "SPI transaction data size constants." type. The
 *         value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSPI_PDD_GetWordLength(deviceID);
 *      @endcode
 */
#define QSPI_PDD_GetWordLength(PeripheralBase) ( \
    PeriphGetBitMask16(QSPI_SPDSR_REG(PeripheralBase), QSPI_SPDSR_DS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetWordLengthAndStrobeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets data transmission size and chip select strobe.
 * @param PeripheralBase Peripheral base address.
 * @param Value Mask of data features requests. This parameter is a 16-bit value.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_SetWordLengthAndStrobeReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_SetWordLengthAndStrobeReg(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSPI_SPDSR_REG(PeripheralBase), \
     (uint16_t)(QSPI_SPDSR_DS_MASK | QSPI_SPDSR_SSB_STRB_MASK), \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataSizeAndControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the data size and control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadDataSizeAndControlReg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadDataSizeAndControlReg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPDSR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataSizeAndControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes a new value to the data size and control register.
 * @param PeripheralBase Peripheral base address.
 * @param Data 16 bits value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDSR, QSPI1_SPDSR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteDataSizeAndControlReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteDataSizeAndControlReg(PeripheralBase, Data) ( \
    PeriphWriteReg16(QSPI_SPDSR_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadData8bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 8 bits value of the data registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPDRR, QSPI1_SPDRR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSPI_PDD_ReadData8bits(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadData8bits(PeripheralBase) ( \
    (uint8_t)PeriphReadReg16(QSPI_SPDRR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadData16bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the 16 bits value of the data registers.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPDRR, QSPI1_SPDRR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadData16bits(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadData16bits(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPDRR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataReceiveReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Read data receive register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPDRR, QSPI1_SPDRR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadDataReceiveReg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadDataReceiveReg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPDRR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteData8Bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes 8 bits data value to the data registers.
 * @param PeripheralBase Peripheral base address.
 * @param Data 8 bits data value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDTR, QSPI1_SPDTR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteData8Bits(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteData8Bits(PeripheralBase, Data) ( \
    PeriphWriteReg16(QSPI_SPDTR_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteData16Bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes 16 bits data value to the data registers.
 * @param PeripheralBase Peripheral base address.
 * @param Data 16 bits data value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDTR, QSPI1_SPDTR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteData16Bits(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteData16Bits(PeripheralBase, Data) ( \
    PeriphWriteReg16(QSPI_SPDTR_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataTransmitReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes 16 bits data value to the data registers.
 * @param PeripheralBase Peripheral base address.
 * @param Data 16 bits data value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPDTR, QSPI1_SPDTR
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteDataTransmitReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteDataTransmitReg(PeripheralBase, Data) ( \
    PeriphWriteReg16(QSPI_SPDTR_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxFifoLevel
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the Tx FIFO level.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSPI_PDD_GetTxFifoLevel(deviceID);
 *      @endcode
 */
#define QSPI_PDD_GetTxFifoLevel(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(QSPI_SPFIFO_REG(PeripheralBase), QSPI_SPFIFO_TFCNT_MASK)) >> ( \
     QSPI_SPFIFO_TFCNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxFifoLevel
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the Rx FIFO level.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSPI_PDD_GetRxFifoLevel(deviceID);
 *      @endcode
 */
#define QSPI_PDD_GetRxFifoLevel(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(QSPI_SPFIFO_REG(PeripheralBase), QSPI_SPFIFO_RFCNT_MASK)) >> ( \
     QSPI_SPFIFO_RFCNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxFifoWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of transmit FIFO nearly empty watermark.
 * @param PeripheralBase Peripheral base address.
 * @param Value Transmit FIFO nearly empty watermark value. The user should use
 *        one from the enumerated values. This parameter is of "Transmit FIFO
 *        nearly empty watermark constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetTxFifoWatermark(deviceID, QSPI_PDD_TX_FIFO_EMPTY);
 *      @endcode
 */
#define QSPI_PDD_SetTxFifoWatermark(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSPI_SPFIFO_REG(PeripheralBase), \
     QSPI_SPFIFO_TFWM_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetRxFifoWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of receive FIFO nearly full watermark.
 * @param PeripheralBase Peripheral base address.
 * @param Value Receive FIFO nearly full watermark value. The user should use
 *        one from the enumerated values. This parameter is of "Receive FIFO nearly
 *        full watermark constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_SetRxFifoWatermark(deviceID, QSPI_PDD_RX_FIFO_1_OR_MORE);
 *      @endcode
 */
#define QSPI_PDD_SetRxFifoWatermark(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSPI_SPFIFO_REG(PeripheralBase), \
     QSPI_SPFIFO_RFWM_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFifoMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables FIFO mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of FIFO mode. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableFifoMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableFifoMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPFIFO_REG(PeripheralBase), QSPI_SPFIFO_FIFO_ENA_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPFIFO_REG(PeripheralBase), QSPI_SPFIFO_FIFO_ENA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFifoControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the FIFO control register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadFifoControlReg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadFifoControlReg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPFIFO_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFifoControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes a new value to the FIFO control register.
 * @param PeripheralBase Peripheral base address.
 * @param Value 16 bits value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPFIFO,
 *          QSPI1_SPFIFO (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteFifoControlReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteFifoControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSPI_SPFIFO_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetWaitDelay
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the delay between words.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 13-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: QSPI0_SPWAIT,
 *          QSPI1_SPWAIT (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_GetWaitDelay(deviceID);
 *      @endcode
 */
#define QSPI_PDD_GetWaitDelay(PeripheralBase) ( \
    (uint16_t)PeriphGetBitMask16(QSPI_SPWAIT_REG(PeripheralBase), QSPI_SPWAIT_WAIT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetWaitDelay
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the delay between words to be a number of Peripheral Bus Clocks
 * equal to (Value + 1).
 * @param PeripheralBase Peripheral base address.
 * @param Value Wait delay value. This parameter is a 13-bit value.
 * @return Returns a 13-bit value. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: QSPI0_SPWAIT,
 *          QSPI1_SPWAIT (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_SetWaitDelay(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_SetWaitDelay(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSPI_SPWAIT_REG(PeripheralBase), \
     QSPI_SPWAIT_WAIT_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadWordDelayReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the word delay register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPWAIT,
 *          QSPI1_SPWAIT (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadWordDelayReg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadWordDelayReg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPWAIT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteWordDelayReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes a new value to the word delay register.
 * @param PeripheralBase Peripheral base address.
 * @param Value 16 bits value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPWAIT,
 *          QSPI1_SPWAIT (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteWordDelayReg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteWordDelayReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSPI_SPWAIT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableStopHoldOffMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables stop mode holdoff.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of stop mode holdoff. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPCTL2,
 *          QSPI1_SPCTL2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_EnableStopHoldOffMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSPI_PDD_EnableStopHoldOffMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSPI_SPCTL2_REG(PeripheralBase), QSPI_SPCTL2_SHEN_MASK)) : ( \
      PeriphSetBitMask16(QSPI_SPCTL2_REG(PeripheralBase), QSPI_SPCTL2_SHEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control 2 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSPI0_SPCTL2,
 *          QSPI1_SPCTL2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSPI_PDD_ReadControl2Reg(deviceID);
 *      @endcode
 */
#define QSPI_PDD_ReadControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(QSPI_SPCTL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes a new value to the control 2 register.
 * @param PeripheralBase Peripheral base address.
 * @param Value 16 bits value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSPI0_SPCTL2,
 *          QSPI1_SPCTL2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSPI_PDD_WriteControl2Reg(deviceID, 1);
 *      @endcode
 */
#define QSPI_PDD_WriteControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSPI_SPCTL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )
#endif  /* #if defined(QSPI_PDD_H_) */

/* QSPI_PDD.h, eof. */
