/*
  PDD layer implementation for peripheral type QSCI
  Copyright 2010 Freescale, Inc.
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(QSCI_PDD_H_)
#define QSCI_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error QSCI PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* QSCI0 */ && \
      !defined(MCU_MC56F82316) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82723) /* QSCI0 */ && \
      !defined(MCU_MC56F82726) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82728) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82733) /* QSCI0 */ && \
      !defined(MCU_MC56F82736) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82738) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82743) /* QSCI0 */ && \
      !defined(MCU_MC56F82746) /* QSCI0, QSCI1 */ && \
      !defined(MCU_MC56F82748) /* QSCI0, QSCI1 */
  // Unsupported MCU is active
  #error QSCI PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Rx buffer full, Rx error, Tx buffer empty and Tx idle interrupt masks
   constant */
#define QSCI_PDD_INTERRUPT_TRANSMITTER          QSCI_CTRL1_TEIE_MASK /**< Transmitter buffer empty mask */
#define QSCI_PDD_INTERRUPT_TRANSMITTER_COMPLETE QSCI_CTRL1_TIIE_MASK /**< Transmitter idle mask */
#define QSCI_PDD_INTERRUPT_RECEIVER             QSCI_CTRL1_RFIE_MASK /**< Receiver buffer full mask */
#define QSCI_PDD_INTERRUPT_RECEIVER_ERROR       QSCI_CTRL1_REIE_MASK /**< Receiver error interrupt mask */

/* Receiver input edge and idle interrupt masks constant */
#define QSCI_PDD_RX_INPUT_EDGE QSCI_CTRL2_RIEIE_MASK /**< Receiver input active edge */

/* Status flags constants. */
#define QSCI_PDD_TX_DATA_EMPTY_FLAG    QSCI_STAT_TDRE_MASK /**< Transmitter FIFO word count is at or below watermark */
#define QSCI_PDD_TX_IDLE_FLAG          QSCI_STAT_TIDLE_MASK /**< No transmission in progress */
#define QSCI_PDD_RX_DATA_FULL_FLAG     QSCI_STAT_RDRF_MASK /**< Receiver FIFO word count is above watermark */
#define QSCI_PDD_RX_IDLE_FLAG          QSCI_STAT_RIDLE_MASK /**< Receiver input has become idle (after receiving a valid frame) */
#define QSCI_PDD_RX_OVERRUN_FLAG       QSCI_STAT_OR_MASK /**< Receiver buffer overrun */
#define QSCI_PDD_RX_NOISE_FLAG         QSCI_STAT_NF_MASK /**< Receiver input detect a noise. */
#define QSCI_PDD_RX_FRAMING_ERROR_FLAG QSCI_STAT_FE_MASK /**< Receiver framing error detect */
#define QSCI_PDD_RX_PARITY_ERROR_FLAG  QSCI_STAT_PF_MASK /**< Receiver parity error detect */
#define QSCI_PDD_RX_INPUT_EDGE_FLAG    QSCI_STAT_RIEF_MASK /**< An active edge on the receive pin has occured */
#define QSCI_PDD_LIN_SYNC_ERROR_FLAG   QSCI_STAT_LSE_MASK /**< A sync error prevented loading of the RATE register with a revised value after the break was detected */
#define QSCI_PDD_TX_DMA_REQUEST_FLAG   QSCI_STAT_TDMA_MASK /**< Number words in Tx FIFO(data register) is equal or less as Tx watermark value */
#define QSCI_PDD_RX_DMA_REQUEST_FLAG   QSCI_STAT_RDMA_MASK /**< Number words in Rx FIFO(data register) is equal or more as Rx watermark value */
#define QSCI_PDD_RX_ACTIVE_FLAG        QSCI_STAT_RAF_MASK /**< Reception is in progress */

/* Loop mode constants. */
#define QSCI_PDD_LOOP_MODE_NORMAL       0U       /**< Normal operation mode. No loopback selected */
#define QSCI_PDD_LOOP_MODE_LOCAL_LOOP   0x8000U  /**< Local loopback mode. Receiver input internally connected to transmitter output */
#define QSCI_PDD_LOOP_MODE_RX_TO_TX_PIN 0xA000U  /**< Receiver input connected to TXD pin */

/* Data width */
#define QSCI_PDD_WIDTH_8 0U                      /**< 8-bits communication */
#define QSCI_PDD_WIDTH_9 0x1000U                 /**< 9-bits communication */

/* Wake-up condition constants. */
#define QSCI_PDD_BY_IDLE_LINE    0U              /**< Idle line wake-up */
#define QSCI_PDD_BY_ADDRESS_MARK 0x800U          /**< Address mark wake-up */

/* Polarity constants. */
#define QSCI_PDD_NORMAL_POLARITY   0U            /**< Don't invert transmit and receive data bits */
#define QSCI_PDD_INVERTED_POLARITY 0x400U        /**< Invert transmit and receive data bits */

/* Parity types */
#define QSCI_PDD_PARITY_NONE 0U                  /**< No parity */
#define QSCI_PDD_PARITY_EVEN 0x200U              /**< Even parity */
#define QSCI_PDD_PARITY_ODD  0x300U              /**< Odd parity */

/* Receiver wakeup mode constants. */
#define QSCI_PDD_NORMAL_OPERATION 0U             /**< Receiver in normal operation */
#define QSCI_PDD_STANDBY_STATE    0x2U           /**< Receiver in standby mode */

/* Transmit FIFO nearly empty watermark constants. */
#define QSCI_PDD_TX_FIFO_EMPTY     0U            /**< Transmit interrupt active when Tx FIFO is empty */
#define QSCI_PDD_TX_FIFO_1_OR_LESS 0x800U        /**< Transmit interrupt active when Tx FIFO has one or fewer words available */
#define QSCI_PDD_TX_FIFO_2_OR_LESS 0x1000U       /**< Transmit interrupt active when Tx FIFO has two or fewer words available */
#define QSCI_PDD_TX_FIFO_3_OR_LESS 0x1800U       /**< Transmit interrupt active when Tx FIFO has three or fewer words available */

/* Receive FIFO nearly full watermark constants. */
#define QSCI_PDD_RX_FIFO_1_OR_MORE 0U            /**< Receive interrupt active when Rx FIFO has at least 1 word used */
#define QSCI_PDD_RX_FIFO_2_OR_MORE 0x40U         /**< Receive interrupt active when Rx FIFO has at least 2 words used */
#define QSCI_PDD_RX_FIFO_3_OR_MORE 0x80U         /**< Receive interrupt active when Rx FIFO has at least 3 words used */
#define QSCI_PDD_RX_FIFO_4_OR_MORE 0xC0U         /**< Receive interrupt active when Rx FIFO has at least 4 words used */


/* ----------------------------------------------------------------------------
   -- SetBaudRate
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets new baud rate value.
 * @param PeripheralBase Peripheral base address.
 * @param Value New baud rate value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_RATE, QSCI0_CTRL3,
 *          QSCI1_RATE, QSCI1_CTRL3 (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetBaudRate(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_SetBaudRate(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSCI_RATE_REG(PeripheralBase), \
     QSCI_RATE_SBRL_MASK, \
     (uint16_t)((uint16_t)((uint16_t)(Value) & 0x1FFFU) << QSCI_RATE_SBRL_SHIFT) \
    ), \
    PeriphSetBits16( \
     QSCI_CTRL3_REG(PeripheralBase), \
     QSCI_CTRL3_SBRH_MASK, \
     (uint16_t)((uint16_t)(Value) & (uint16_t)(~(uint16_t)0x1FFFU)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalBaudRate
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets new fractional baud rate value.
 * @param PeripheralBase Peripheral base address.
 * @param Value New fractional baud rate value. This parameter is a 3-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_RATE, QSCI1_RATE
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetFractionalBaudRate(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_SetFractionalBaudRate(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSCI_RATE_REG(PeripheralBase), \
     QSCI_RATE_FRAC_SBR_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBaudRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads baud rate register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_RATE, QSCI1_RATE
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadBaudRateReg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadBaudRateReg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_RATE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteBaudRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into baud rate
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the baud rate register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_RATE, QSCI1_RATE
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_WriteBaudRateReg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteBaudRateReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_RATE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetLoopMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the loop mode operation.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Loop mode. This parameter is of "Loop mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetLoopMode(deviceID, QSCI_PDD_LOOP_MODE_NORMAL);
 *      @endcode
 */
#define QSCI_PDD_SetLoopMode(PeripheralBase, Mode) ( \
    PeriphSetBits16( \
     QSCI_CTRL1_REG(PeripheralBase), \
     (uint16_t)(QSCI_CTRL1_LOOP_MASK | QSCI_CTRL1_RSRC_MASK), \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOperateInWaitMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables operate in wait mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of device in wait mode. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableOperateInWaitMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableOperateInWaitMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SWAI_MASK)) : ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SWAI_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDataWidth
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the communication width.
 * @param PeripheralBase Peripheral base address.
 * @param Width Data width. This parameter is of "Data width" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetDataWidth(deviceID, QSCI_PDD_WIDTH_8);
 *      @endcode
 */
#define QSCI_PDD_SetDataWidth(PeripheralBase, Width) ( \
    ((Width) == QSCI_PDD_WIDTH_8) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_M_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_M_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetWakeupCondition
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the wake-up condition.
 * @param PeripheralBase Peripheral base address.
 * @param Condition Wake-up condition. This parameter is of "Wake-up condition
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetWakeupCondition(deviceID, QSCI_PDD_BY_IDLE_LINE);
 *      @endcode
 */
#define QSCI_PDD_SetWakeupCondition(PeripheralBase, Condition) ( \
    ((Condition) == QSCI_PDD_BY_IDLE_LINE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_WAKE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_WAKE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetRxTxPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets polarity of the received/transmitted data.
 * @param PeripheralBase Peripheral base address.
 * @param Polarity Transmit and receive data bits polarity. This parameter is of
 *        "Polarity constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetRxTxPolarity(deviceID, QSCI_PDD_NORMAL_POLARITY);
 *      @endcode
 */
#define QSCI_PDD_SetRxTxPolarity(PeripheralBase, Polarity) ( \
    ((Polarity) == QSCI_PDD_NORMAL_POLARITY) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_POL_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_POL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetParity
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets a communication parity type.
 * @param PeripheralBase Peripheral base address.
 * @param Parity Parity type. This parameter is of "Parity types" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetParity(deviceID, QSCI_PDD_PARITY_NONE);
 *      @endcode
 */
#define QSCI_PDD_SetParity(PeripheralBase, Parity) ( \
    PeriphSetBits16( \
     QSCI_CTRL1_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x3U << 8U), \
     (uint16_t)(Parity) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableInterrupt(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_EnableInterrupt(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_DisableInterrupt(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_DisableInterrupt(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxCompleteInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Gets the status of the transmiter idle interrupt enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_GetTxCompleteInterruptMask(deviceID);
 *      @endcode
 */
#define QSCI_PDD_GetTxCompleteInterruptMask(PeripheralBase) ( \
    PeriphGetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_TIIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTransmitter
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables UART transmitter.
 * @param PeripheralBase Peripheral base address.
 * @param State Enables or disables transmitter. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableTransmitter(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableTransmitter(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_TE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_TE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableReceiver
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables UART receiver.
 * @param PeripheralBase Peripheral base address.
 * @param State Enables or disables receiver. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableReceiver(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableReceiver(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_RE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_RE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReceiverWakeup
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the wake-up function and inhibits further receiver interrupt
 * requests.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Receiver mode. This parameter is of "Receiver wakeup mode
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_ReceiverWakeup(deviceID, QSCI_PDD_NORMAL_OPERATION);
 *      @endcode
 */
#define QSCI_PDD_ReceiverWakeup(PeripheralBase, Mode) ( \
    ((Mode) == QSCI_PDD_NORMAL_OPERATION) ? ( \
      PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_RWU_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_RWU_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetBreak
   ---------------------------------------------------------------------------- */

/**
 * @brief Set the break signal.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetBreak(deviceID);
 *      @endcode
 */
#define QSCI_PDD_SetBreak(PeripheralBase) ( \
    PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SBK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearBreak
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears the break signal.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_ClearBreak(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ClearBreak(PeripheralBase) ( \
    PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SBK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SendBreak
   ---------------------------------------------------------------------------- */

/**
 * @brief Send the break character.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SendBreak(deviceID);
 *      @endcode
 */
#define QSCI_PDD_SendBreak(PeripheralBase) ( \
    PeriphSetBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SBK_MASK), \
    PeriphClearBitMask16(QSCI_CTRL1_REG(PeripheralBase), QSCI_CTRL1_SBK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 1 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadControl1Reg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_CTRL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into control 1
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the control 1 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL1, QSCI1_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_WriteControl1Reg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_CTRL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxFifoWordCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns a word count in the transmit FIFO.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSCI_PDD_GetTxFifoWordCount(deviceID);
 *      @endcode
 */
#define QSCI_PDD_GetTxFifoWordCount(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_TFCNT_MASK)) >> ( \
     QSCI_CTRL2_TFCNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxFifoWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of transmit FIFO nearly empty watermark.
 * @param PeripheralBase Peripheral base address.
 * @param Value Transmit FIFO nearly empty watermark value. This parameter is of
 *        "Transmit FIFO nearly empty watermark constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetTxFifoWatermark(deviceID, QSCI_PDD_TX_FIFO_EMPTY);
 *      @endcode
 */
#define QSCI_PDD_SetTxFifoWatermark(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSCI_CTRL2_REG(PeripheralBase), \
     QSCI_CTRL2_TFWM_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxFifoWordCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns a word count in the receive FIFO.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSCI_PDD_GetRxFifoWordCount(deviceID);
 *      @endcode
 */
#define QSCI_PDD_GetRxFifoWordCount(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_RFCNT_MASK)) >> ( \
     QSCI_CTRL2_RFCNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetRxFifoWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of receive FIFO nearly full watermark.
 * @param PeripheralBase Peripheral base address.
 * @param Value Receive FIFO nearly full watermark value. This parameter is of
 *        "Receive FIFO nearly full watermark constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_SetRxFifoWatermark(deviceID, QSCI_PDD_RX_FIFO_1_OR_MORE);
 *      @endcode
 */
#define QSCI_PDD_SetRxFifoWatermark(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     QSCI_CTRL2_REG(PeripheralBase), \
     QSCI_CTRL2_RFWM_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFifo
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables FIFO specified by the FifoMask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param State Enables or disables receiver and transmit FIFO. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableFifo(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableFifo(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_FIFO_EN_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_FIFO_EN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRxStateInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableRxStateInterruptMask(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_EnableRxStateInterruptMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(QSCI_CTRL2_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableRxStateInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupt requests defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_DisableRxStateInterruptMask(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_DisableRxStateInterruptMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(QSCI_CTRL2_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLinSlaveMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables LIN slave mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Enables or disables LIN slave mode. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableLinSlaveMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableLinSlaveMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_LINMODE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_LINMODE_MASK)) \
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
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableTransmitDma(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableTransmitDma(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_TDE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_TDE_MASK)) \
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
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableReceiveDma(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableReceiveDma(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_RDE_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL2_REG(PeripheralBase), QSCI_CTRL2_RDE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 2 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadControl2Reg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_CTRL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into control 2
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the control 2 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL2, QSCI1_CTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_WriteControl2Reg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_CTRL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_STAT, QSCI1_STAT
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadStatusFlags(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadStatusFlags(PeripheralBase) ( \
    PeriphReadReg16(QSCI_STAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_STAT, QSCI1_STAT
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadStatusReg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_STAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the status register. This parameter is a
 *        16-bit value.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_STAT, QSCI1_STAT
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_WriteStatusReg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_STAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- PutChar8
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts 8-bits character into the transmit buffer.
 * @param PeripheralBase Peripheral base address.
 * @param Char 8-bits character to be written to the data register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_PutChar8(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_PutChar8(PeripheralBase, Char) ( \
    PeriphWriteReg16(QSCI_DATA_REG(PeripheralBase), (uint16_t)(Char)) \
  )

/* ----------------------------------------------------------------------------
   -- PutChar9
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts 9-bits character into the transmit buffer.
 * @param PeripheralBase Peripheral base address.
 * @param Char 9-bits character to be written to the data register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_PutChar9(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_PutChar9(PeripheralBase, Char) ( \
    PeriphWriteReg16(QSCI_DATA_REG(PeripheralBase), (uint16_t)(Char)) \
  )

/* ----------------------------------------------------------------------------
   -- GetChar8
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns 8-bits character from the receive buffer.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = QSCI_PDD_GetChar8(deviceID);
 *      @endcode
 */
#define QSCI_PDD_GetChar8(PeripheralBase) ( \
    PeriphGetBitMask16(QSCI_DATA_REG(PeripheralBase), 0xFFU) \
  )

/* ----------------------------------------------------------------------------
   -- GetChar9
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns 9-bits character from the receive buffer.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_GetChar9(deviceID);
 *      @endcode
 */
#define QSCI_PDD_GetChar9(PeripheralBase) ( \
    PeriphReadReg16(QSCI_DATA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads data register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadDataReg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadDataReg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_DATA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into data register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the data register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_DATA, QSCI1_DATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_WriteDataReg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteDataReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_DATA_REG(PeripheralBase), (uint16_t)(Value)) \
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
 * @remarks The macro accesses the following registers: QSCI0_CTRL3, QSCI1_CTRL3
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_EnableStopHoldOffMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define QSCI_PDD_EnableStopHoldOffMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(QSCI_CTRL3_REG(PeripheralBase), QSCI_CTRL3_SHEN_MASK)) : ( \
      PeriphSetBitMask16(QSCI_CTRL3_REG(PeripheralBase), QSCI_CTRL3_SHEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 3 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: QSCI0_CTRL3, QSCI1_CTRL3
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = QSCI_PDD_ReadControl3Reg(deviceID);
 *      @endcode
 */
#define QSCI_PDD_ReadControl3Reg(PeripheralBase) ( \
    PeriphReadReg16(QSCI_CTRL3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into control 3
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the control 3 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: QSCI0_CTRL3, QSCI1_CTRL3
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      QSCI_PDD_WriteControl3Reg(deviceID, 1);
 *      @endcode
 */
#define QSCI_PDD_WriteControl3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(QSCI_CTRL3_REG(PeripheralBase), (uint16_t)(Value)) \
  )
#endif  /* #if defined(QSCI_PDD_H_) */

/* QSCI_PDD.h, eof. */
