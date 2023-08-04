/*
  PDD layer implementation for peripheral type CAN
  Copyright 2013 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(MSCAN_PDD_H_)
#define MSCAN_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error CAN PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82726) /* CAN */ && \
      !defined(MCU_MC56F82728) /* CAN */ && \
      !defined(MCU_MC56F82736) /* CAN */ && \
      !defined(MCU_MC56F82738) /* CAN */ && \
      !defined(MCU_MC56F82746) /* CAN */ && \
      !defined(MCU_MC56F82748) /* CAN */
  // Unsupported MCU is active
  #error CAN PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Status flags constants. */
#define CAN_PDD_WAKEUP_REQUESTED_FLAG     CAN_RFLG_WUPIF_MASK /**< Wake-up Interrupt Flag */
#define CAN_PDD_ERROR_COUNTER_CHANGE_FLAG CAN_RFLG_CSCIF_MASK /**< MSCAN changed current CAN bus status */
#define CAN_PDD_RX_BUFFER_OVERRUN_FLAG    CAN_RFLG_OVRIF_MASK /**< A data overrun detected */
#define CAN_PDD_RX_BUFFER_FULL_FLAG       CAN_RFLG_RXF_MASK /**< The receiver FIFO is not empty */

/* Status flags constants. */
#define CAN_PDD_WAKEUP_REQUESTED_INT     CAN_RIER_WUPIE_MASK /**< Wake-up Interrupt Flag */
#define CAN_PDD_ERROR_COUNTER_CHANGE_INT CAN_RIER_CSCIE_MASK /**< MSCAN changed current CAN bus status */
#define CAN_PDD_RX_BUFFER_OVERRUN_INT    CAN_RIER_OVRIE_MASK /**< A data overrun detected */
#define CAN_PDD_RX_BUFFER_FULL_INT       CAN_RIER_RXFIE_MASK /**< The receiver FIFO is not empty */

/* Receiver active status constants. */
#define CAN_PDD_TRANSMITTING_OR_IDLE 0U          /**< MSCAN is transmitting or idle */
#define CAN_PDD_RECEIVING            0x40U       /**< MSCAN is receiving a message (including when arbitration is lost) */

/* MSCAN synchronized constants. */
#define CAN_PDD_NOT_SYNCHRONIZED 0U              /**< MSCAN is not synchronized to the CAN bus */
#define CAN_PDD_SYNCHRONIZED     0x10U           /**< MSCAN is synchronized to the CAN bus */

/* MSCAN Engine Clock Source constants (for SetClockSource macro). */
#define CAN_PDD_XTAL_CLOCK 0U                    /**< The MSCAN engine clock source is the oscillator clock */
#define CAN_PDD_BUS_CLOCK  0x40U                 /**< The MSCAN engine clock source is the peripheral clock */

/* The synchronization jump width constants. */
#define CAN_PDD_SYNC_JUMP_WIDTH_1_TQ 0U          /**< Synchronization jump width 1 time quanta clock cycle */
#define CAN_PDD_SYNC_JUMP_WIDTH_2_TQ 0x1U        /**< Synchronization jump width 2 time quanta clock cycles */
#define CAN_PDD_SYNC_JUMP_WIDTH_3_TQ 0x2U        /**< Synchronization jump width 3 time quanta clock cycles */
#define CAN_PDD_SYNC_JUMP_WIDTH_4_TQ 0x3U        /**< Synchronization jump width 4 time quanta clock cycles */

/* Rx sampling mode constants (for SetBitSampling macro). */
#define CAN_PDD_ONE_SAMPLE    0U                 /**< Just one sample is used to determine the bit value */
#define CAN_PDD_THREE_SAMPLES 0x80U              /**< Three samples are used to determine the value of the received bit */

/* Time segment 2 constants. */
#define CAN_PDD_TIME_SEGMENT2_1_TQ 0U            /**< Time segment2 1 time quanta clock cycle */
#define CAN_PDD_TIME_SEGMENT2_2_TQ 0x1U          /**< Time segment2 2 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_3_TQ 0x2U          /**< Time segment2 3 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_4_TQ 0x3U          /**< Time segment2 4 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_5_TQ 0x4U          /**< Time segment2 5 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_6_TQ 0x5U          /**< Time segment2 6 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_7_TQ 0x6U          /**< Time segment2 7 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT2_8_TQ 0x7U          /**< Time segment2 8 time quanta clock cycles */

/* Time segment 1 constants. */
#define CAN_PDD_TIME_SEGMENT1_1_TQ  0U           /**< Time segment1 1 time quanta clock cycle */
#define CAN_PDD_TIME_SEGMENT1_2_TQ  0x1U         /**< Time segment1 2 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_3_TQ  0x2U         /**< Time segment1 3 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_4_TQ  0x3U         /**< Time segment1 4 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_5_TQ  0x4U         /**< Time segment1 5 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_6_TQ  0x5U         /**< Time segment1 6 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_7_TQ  0x6U         /**< Time segment1 7 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_8_TQ  0x7U         /**< Time segment1 8 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_9_TQ  0x8U         /**< Time segment1 9 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_10_TQ 0x9U         /**< Time segment1 10 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_11_TQ 0xAU         /**< Time segment1 11 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_12_TQ 0xBU         /**< Time segment1 12 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_13_TQ 0xCU         /**< Time segment1 13 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_14_TQ 0xDU         /**< Time segment1 14 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_15_TQ 0xEU         /**< Time segment1 15 time quanta clock cycles */
#define CAN_PDD_TIME_SEGMENT1_16_TQ 0xFU         /**< Time segment1 16 time quanta clock cycles */

/* Receiver status bits constants */
#define CAN_PDD_RX_OK      0U                    /**< Receiver ok */
#define CAN_PDD_RX_WARNING 0x10U                 /**< Receiver warning. */
#define CAN_PDD_RX_ERROR   0x20U                 /**< Receiver error */
#define CAN_PDD_RX_BUS_OFF 0x30U                 /**< Receiver bus-off */

/* Transmitter status bits constants */
#define CAN_PDD_TX_OK      0U                    /**< Transmitter ok */
#define CAN_PDD_TX_WARNING 0x4U                  /**< Transmitter warning */
#define CAN_PDD_TX_ERROR   0x8U                  /**< Transmitter error */
#define CAN_PDD_TX_BUS_OFF 0xCU                  /**< Transmitter bus-off */

/* Receiver status change constants. */
#define CAN_PDD_RX_STATUS_DISABLE_INT      0U    /**< Do not set status change flag caused by receiver state changes */
#define CAN_PDD_RX_BUS_OFF_INT             0x10U /**< Set status change flag only if the receiver enters or leaves bus-off state */
#define CAN_PDD_RX_TX_ERROR_OR_BUS_OFF_INT 0x20U /**< Set status change flag only if the receiver enters or leaves TxError or bus-off state */
#define CAN_PDD_RX_ALL_STATE_INT           0x30U /**< Set status change flag on all state changes */

/* Transmitter status change constants. */
#define CAN_PDD_TX_STATUS_DISABLE_INT      0U    /**< Do not set status change flag caused by transmitter state changes */
#define CAN_PDD_TX_BUS_OFF_INT             0x4U  /**< Set status change flag only if the transmitter enters or leaves bus-off state */
#define CAN_PDD_TX_TX_ERROR_OR_BUS_OFF_INT 0x8U  /**< Set status change flag only if the transmitter enters or leaves TxError or bus-off state */
#define CAN_PDD_TX_ALL_STATE_INT           0xCU  /**< Set status change flag on all state changes */

/* Transmit message buffer flag mask constants. */
#define CAN_PDD_TX_MESSAGE_BUFFER_0   0x1U       /**< Transmitter message buffer 0 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_1   0x2U       /**< Transmitter message buffer 1 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_2   0x4U       /**< Transmitter message buffer 2 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_ALL 0x7U       /**< Transmitter all message buffers mask */

/* Transmit message buffer interrupt mask constants. */
#define CAN_PDD_TX_MESSAGE_BUFFER_0_INT   0x1U   /**< Transmitter message buffer 0 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_1_INT   0x2U   /**< Transmitter message buffer 1 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_2_INT   0x4U   /**< Transmitter message buffer 2 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_ALL_INT 0x7U   /**< Transmitter all message buffers mask */

/* Transmit message buffer abort constants. */
#define CAN_PDD_TX_MESSAGE_BUFFER_0_ABORT   0x1U /**< Transmitter message buffer 0 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_1_ABORT   0x2U /**< Transmitter message buffer 1 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_2_ABORT   0x4U /**< Transmitter message buffer 2 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_ALL_ABORT 0x7U /**< Transmitter all message buffers mask */

/* Transmit message buffer abort constants. */
#define CAN_PDD_TX_MESSAGE_BUFFER_0_ABORT_ACK   0x1U /**< Transmitter message buffer 0 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_1_ABORT_ACK   0x2U /**< Transmitter message buffer 1 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_2_ABORT_ACK   0x4U /**< Transmitter message buffer 2 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_ALL_ABORT_ACK 0x7U /**< Transmitter all message buffers mask */

/* Transmit message buffer select constants. */
#define CAN_PDD_TX_MESSAGE_BUFFER_0_SELECT 0x1U  /**< Transmitter message buffer 0 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_1_SELECT 0x2U  /**< Transmitter message buffer 1 mask */
#define CAN_PDD_TX_MESSAGE_BUFFER_2_SELECT 0x4U  /**< Transmitter message buffer 2 mask */

/* Identifier acceptance filter constants. */
#define CAN_PDD_TWO_32BIT_ACCEPTANCE_FILTER  0U  /**< Two 32-Bit Acceptance Filters */
#define CAN_PDD_FOUR_16BIT_ACCEPTAN_FILTER   0x1U /**< Four 16-Bit Acceptance Filters */
#define CAN_PDD_EIGHT_8BIT_ACCEPTANCE_FILTER 0x2U /**< Eight 8-Bit Acceptance Filters */
#define CAN_PDD_FILTER_CLOSED                0x3U /**< Filter Closed */

/* Identifier acceptance hit indicator constants */
#define CAN_PDD_FILTER_HIT_0 0U                  /**< Filter 0 Hit */
#define CAN_PDD_FILTER_HIT_1 0x1U                /**< Filter 1 Hit */
#define CAN_PDD_FILTER_HIT_2 0x2U                /**< Filter 2 Hit */
#define CAN_PDD_FILTER_HIT_3 0x3U                /**< Filter 3 Hit */
#define CAN_PDD_FILTER_HIT_4 0x4U                /**< Filter 4 Hit */
#define CAN_PDD_FILTER_HIT_5 0x5U                /**< Filter 5 Hit */
#define CAN_PDD_FILTER_HIT_6 0x6U                /**< Filter 6 Hit */
#define CAN_PDD_FILTER_HIT_7 0x7U                /**< Filter 7 Hit */

/* Type of message buffer ID constants (for SetRxMessageBufferID,
   SetTxMessageBufferID macros). */
#define CAN_PDD_BUFFER_ID_EXT 0U                 /**< Extended frame format */
#define CAN_PDD_BUFFER_ID_STD 0x1U               /**< Standard frame format */


/* ----------------------------------------------------------------------------
   -- GetReceivedFrameFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Return value of the received frame flag.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_GetReceivedFrameFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetReceivedFrameFlag(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_RXFRM_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearReceivedFrameFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears received frame interrupt flag.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_ClearReceivedFrameFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ClearReceivedFrameFlag(PeripheralBase) ( \
    PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_RXFRM_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetReceiverActiveStatusFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current receiver active status.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Receiver active status constants." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetReceiverActiveStatusFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetReceiverActiveStatusFlag(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_RXACT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOperateInWaitMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables operate in wait mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested state of device in wait mode. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableOperateInWaitMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableOperateInWaitMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_CSWAI_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_CSWAI_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSynchronizedStatusFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current MSCAN synchronized status flag.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "MSCAN synchronized constants." type. The value is
 *         cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetSynchronizedStatusFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetSynchronizedStatusFlag(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_SYNCH_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTimerSynchronization
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables timer synchronization.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if timer synchronization will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTimerSynchronization(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableTimerSynchronization(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_TIME_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_TIME_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWakeUp
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables wake up feature.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if wake up will be enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableWakeUp(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableWakeUp(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_WUPE_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_WUPE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSleepMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables MSCAN sleep mode feature.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if MSCAN will be in running or sleep mode.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableSleepMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableSleepMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_SLPRQ_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_SLPRQ_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSleepModeState
   ---------------------------------------------------------------------------- */

/**
 * @brief Return state of the sleep mode request.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_GetSleepModeState(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetSleepModeState(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_SLPRQ_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInitializationMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables MSCAN initialization mode feature.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if MSCAN will be in normal or
 *        initialization mode. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableInitializationMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableInitializationMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_INITRQ_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL0_REG(PeripheralBase), CAN_CTL0_INITRQ_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 0 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadControl0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadControl0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_CTL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into control 0
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the control 0 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL0.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteControl0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteControl0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_CTL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables MSCAN device.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested state of device. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableDevice(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_CANE_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_CANE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Set CAN Engine Clock Source.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param ClkSource Parameter specifying clock source of CAN Engine. This
 *        parameter is of "MSCAN Engine Clock Source constants (for SetClockSource
 *        macro)." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_SetClockSource(<peripheral>_BASE_PTR, CAN_PDD_XTAL_CLOCK);
 *      @endcode
 */
#define CAN_PDD_SetClockSource(PeripheralBase, ClkSource) ( \
    ((ClkSource) == CAN_PDD_XTAL_CLOCK) ? ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_CLKSRC_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_CLKSRC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableLoopBack
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables loop back mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if Loop Back will be enabled or disabled.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableLoopBack(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableLoopBack(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_LOOPB_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_LOOPB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableListenOnlyMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables listen only mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if listen only mode will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableListenOnlyMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableListenOnlyMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_LISTEN_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_LISTEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAutomaticBusOffRecovery
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables automatic bus off recovery mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if bus off recovery will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableAutomaticBusOffRecovery(<peripheral>_BASE_PTR,
 *      PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableAutomaticBusOffRecovery(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_BORM_MASK)) : ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_BORM_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWakeUpFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables low-pass filter on wakeup pin.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if low-pass filter will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableWakeUpFilter(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableWakeUpFilter(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_WUPM_MASK)) : ( \
      PeriphSetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_WUPM_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSleepModeAcknoledgeFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Return value is nonzero if MSCAN is in sleep mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetSleepModeAcknoledgeFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetSleepModeAcknoledgeFlag(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_SLPAK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitializationModeAcknowledgeFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Return value is nonzero if MSCAN is in initialization mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetInitializationModeAcknowledgeFlag(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetInitializationModeAcknowledgeFlag(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_CTL1_REG(PeripheralBase), CAN_CTL1_INITAK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadControl1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_CTL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into control 1
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the control 1 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_CTL1.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteControl1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_CTL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSyncJumpWidthValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of the synchronization jump width.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Resync jump width value[0..3]. This parameter is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR0.
 * @par Example:
 *      @code
 *      CAN_PDD_SetSyncJumpWidthValue(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetSyncJumpWidthValue(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_BTR0_REG(PeripheralBase), \
     CAN_BTR0_SJW_MASK, \
     (uint16_t)((uint16_t)(Value) << CAN_BTR0_SJW_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescalerValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of the prescaler division factor.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Prescaler division factor value[0..63]. This parameter is a
 *        6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR0.
 * @par Example:
 *      @code
 *      CAN_PDD_SetPrescalerValue(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetPrescalerValue(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_BTR0_REG(PeripheralBase), \
     CAN_BTR0_BRP_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBusTiming0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads BUS timing 0 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_BTR0.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadBusTiming0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadBusTiming0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_BTR0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteBusTiming0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into BUS timing 0
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the BUS timing 0 register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR0.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteBusTiming0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteBusTiming0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_BTR0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetBitSampling
   ---------------------------------------------------------------------------- */

/**
 * @brief Set CAN bit sampling at the Rx input.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Sampling Parameter specifying bit sampling at the Rx input. This
 *        parameter is of "Rx sampling mode constants (for SetBitSampling macro)."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR1.
 * @par Example:
 *      @code
 *      CAN_PDD_SetBitSampling(<peripheral>_BASE_PTR, CAN_PDD_ONE_SAMPLE);
 *      @endcode
 */
#define CAN_PDD_SetBitSampling(PeripheralBase, Sampling) ( \
    ((Sampling) == CAN_PDD_ONE_SAMPLE) ? ( \
      PeriphClearBitMask16(CAN_BTR1_REG(PeripheralBase), CAN_BTR1_SAMP_MASK)) : ( \
      PeriphSetBitMask16(CAN_BTR1_REG(PeripheralBase), CAN_BTR1_SAMP_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPhaseSegment2Value
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of phase segment 2.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Phase segment 2 value[0..7]. This parameter is a 3-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR1.
 * @par Example:
 *      @code
 *      CAN_PDD_SetPhaseSegment2Value(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetPhaseSegment2Value(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_BTR1_REG(PeripheralBase), \
     CAN_BTR1_TSEG2_MASK, \
     (uint16_t)((uint16_t)(Value) << CAN_BTR1_TSEG2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPhaseSegment1Value
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value of phase segment 1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Phase segment 1 value[0..15]. This parameter is a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR1.
 * @par Example:
 *      @code
 *      CAN_PDD_SetPhaseSegment1Value(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetPhaseSegment1Value(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_BTR1_REG(PeripheralBase), \
     CAN_BTR1_TSEG1_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBusTiming1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads BUS timing 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_BTR1.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadBusTiming1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadBusTiming1Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_BTR1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteBusTiming1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into BUS timing 1
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the BUS timing 1 register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_BTR1.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteBusTiming1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteBusTiming1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_BTR1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxStatusInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the status register 1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetRxStatusInterruptFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxStatusInterruptFlags(PeripheralBase) ( \
    PeriphReadReg16(CAN_RFLG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearRxStatusInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears receiver interrupt flags of interrupts specified by Mask.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt clear requests. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      CAN_PDD_ClearRxStatusInterruptFlags(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_ClearRxStatusInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16(CAN_RFLG_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxStatusBits
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns receiver status according the receiver error counter value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Receiver status bits constants" type. The value
 *         is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_GetRxStatusBits(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxStatusBits(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_RFLG_REG(PeripheralBase), CAN_RFLG_RSTAT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxStatusBits
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns receiver status according the transmitter error counter value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Transmitter status bits constants" type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_GetTxStatusBits(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxStatusBits(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_RFLG_REG(PeripheralBase), CAN_RFLG_TSTAT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxFlagReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads receiver flag register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadRxFlagReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxFlagReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RFLG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxFlagReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receiver flag
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receiver flag register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RFLG.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxFlagReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxFlagReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RFLG_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRxInterruptsMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables receiver interrupts defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_EnableRxInterruptsMask(<peripheral>_BASE_PTR,
 *      1);
 *      @endcode
 */
#define CAN_PDD_EnableRxInterruptsMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(CAN_RIER_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableRxInterruptsMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables receiver interrupts defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_DisableRxInterruptsMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_DisableRxInterruptsMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(CAN_RIER_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetRxInterruptFlagStatusChange
   ---------------------------------------------------------------------------- */

/**
 * @brief Configures receiver status change flag.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying setting status change flag. This parameter
 *        is of "Receiver status change constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      CAN_PDD_SetRxInterruptFlagStatusChange(<peripheral>_BASE_PTR,
 *      CAN_PDD_RX_STATUS_DISABLE_INT);
 *      @endcode
 */
#define CAN_PDD_SetRxInterruptFlagStatusChange(PeripheralBase, State) ( \
    PeriphSetBits16( \
     CAN_RIER_REG(PeripheralBase), \
     CAN_RIER_RSTATE_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxInterruptFlagStatusChange
   ---------------------------------------------------------------------------- */

/**
 * @brief Configures transmitter status change flag.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying setting status change flag. This parameter
 *        is of "Transmitter status change constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      CAN_PDD_SetTxInterruptFlagStatusChange(<peripheral>_BASE_PTR,
 *      CAN_PDD_TX_STATUS_DISABLE_INT);
 *      @endcode
 */
#define CAN_PDD_SetTxInterruptFlagStatusChange(PeripheralBase, State) ( \
    PeriphSetBits16( \
     CAN_RIER_REG(PeripheralBase), \
     CAN_RIER_TSTATE_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads receiver interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxInterruptEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxInterruptEnableReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RIER_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receiver
 * interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receiver interrupt enable register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RIER.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxInterruptEnableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxInterruptEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RIER_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearTxMessageBufferFlagMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears message buffer flag (load a message due for transmission)
 * defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TFLG.
 * @par Example:
 *      @code
 *      CAN_PDD_ClearTxMessageBufferFlagMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_ClearTxMessageBufferFlagMask(PeripheralBase, Mask) ( \
    PeriphWriteReg16(CAN_TFLG_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxMessageBufferFlagMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transmitter message buffer flags.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TFLG.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetTxMessageBufferFlagMask(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxMessageBufferFlagMask(PeripheralBase) ( \
    PeriphReadReg16(CAN_TFLG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxFlagReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmitter flag register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TFLG.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadTxFlagReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxFlagReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TFLG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxFlagReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmitter
 * flag register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmitter flag register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TFLG.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxFlagReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxFlagReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TFLG_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTxMessageBufferInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables transmit message buffer empty interrupt defined by mask
 * parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TIER.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTxMessageBufferInterruptMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_EnableTxMessageBufferInterruptMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(CAN_TIER_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableTxMessageBufferInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables transmit message buffer empty interrupt defined by mask
 * parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupt requests. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TIER.
 * @par Example:
 *      @code
 *      CAN_PDD_DisableTxMessageBufferInterruptMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_DisableTxMessageBufferInterruptMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(CAN_TIER_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxMessageBufferInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns enabled transmit message buffer interrupt mask.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TIER.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetTxMessageBufferInterruptMask(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxMessageBufferInterruptMask(PeripheralBase) ( \
    PeriphReadReg16(CAN_TIER_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmitter interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TIER.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxInterruptEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxInterruptEnableReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TIER_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmitter
 * interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmitter interrupt enable
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TIER.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxInterruptEnableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxInterruptEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TIER_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- TxMessageAbortRequestMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Aborts transmit message buffer defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of message buffer abort requests. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TARQ.
 * @par Example:
 *      @code
 *      CAN_PDD_TxMessageAbortRequestMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_TxMessageAbortRequestMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(CAN_TARQ_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxMessageAbortRequestReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmitter abort request register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TARQ.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxMessageAbortRequestReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxMessageAbortRequestReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TARQ_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxMessageAbortRequestReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmitter
 * abort request register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmitter abort request register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TARQ.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxMessageAbortRequestReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxMessageAbortRequestReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TARQ_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxMessageAbortAcknowledgeMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transmitter abort acknowledge mask.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TAAK.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetTxMessageAbortAcknowledgeMask(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxMessageAbortAcknowledgeMask(PeripheralBase) ( \
    PeriphReadReg16(CAN_TAAK_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxMessageAbortAcknowledgeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmitter abort acknowledge register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TAAK.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxMessageAbortAcknowledgeReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxMessageAbortAcknowledgeReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TAAK_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectTxMessageBufferMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects transmit message buffer defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of select tranmit buffer. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TBSEL.
 * @par Example:
 *      @code
 *      CAN_PDD_SelectTxMessageBufferMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SelectTxMessageBufferMask(PeripheralBase, Mask) ( \
    PeriphWriteReg16(CAN_TBSEL_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferSelectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmit buffer selection register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TBSEL.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferSelectionReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferSelectionReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TBSEL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferSelectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * selection register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer selection register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TBSEL.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferSelectionReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferSelectionReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TBSEL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetIdAcceptanceFilterMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets identifier acceptance filter mode defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mode Identifier acceptance filter mode. This parameter is a 2-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_IDAC.
 * @par Example:
 *      @code
 *      CAN_PDD_SetIdAcceptanceFilterMode(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetIdAcceptanceFilterMode(PeripheralBase, Mode) ( \
    PeriphSetBits16( \
     CAN_IDAC_REG(PeripheralBase), \
     CAN_IDAC_IDAM_MASK, \
     (uint16_t)((uint16_t)(Mode) << CAN_IDAC_IDAM_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetIdAcceptanceHitIndicator
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns identifier acceptance hit indicator.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Identifier acceptance hit indicator constants"
 *         type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAN_IDAC.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetIdAcceptanceHitIndicator(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetIdAcceptanceHitIndicator(PeripheralBase) ( \
    PeriphGetBitMask16(CAN_IDAC_REG(PeripheralBase), CAN_IDAC_IDHIT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIdAcceptanceControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads identifier acceptance control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_IDAC.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadIdAcceptanceControlReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadIdAcceptanceControlReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_IDAC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIdAcceptanceControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into identifier
 * acceptance control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the identifier acceptance control
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_IDAC.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteIdAcceptanceControlReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteIdAcceptanceControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_IDAC_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- BusOffRecoveryRequest
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the request for recovery from the bus-off mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_MISC.
 * @par Example:
 *      @code
 *      CAN_PDD_BusOffRecoveryRequest(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_BusOffRecoveryRequest(PeripheralBase) ( \
    PeriphClearBitMask16(CAN_MISC_REG(PeripheralBase), CAN_MISC_BOHOLD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBusOffRecovery
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables Bus Off Recovery.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if Bus Off Recovery will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_MISC.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableBusOffRecovery(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableBusOffRecovery(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(CAN_MISC_REG(PeripheralBase), CAN_MISC_BOHOLD_MASK)) : ( \
      PeriphClearBitMask16(CAN_MISC_REG(PeripheralBase), CAN_MISC_BOHOLD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMiscellaneousReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads miscellaneous register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_MISC.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadMiscellaneousReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadMiscellaneousReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_MISC_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMiscellaneousReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into miscellaneous
 * register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the miscellaneous register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_MISC.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteMiscellaneousReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteMiscellaneousReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_MISC_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxErrorCounter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns receive error counter value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAN_RXERR.
 * @par Example:
 *      @code
 *      uint8_t result = CAN_PDD_GetRxErrorCounter(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxErrorCounter(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CAN_RXERR_REG(PeripheralBase), CAN_RXERR_RXERR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxErrorCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads receive error counter register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXERR.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadRxErrorCounterReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxErrorCounterReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXERR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxErrorCounter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transmit error counter value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAN_TXERR.
 * @par Example:
 *      @code
 *      uint8_t result = CAN_PDD_GetTxErrorCounter(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxErrorCounter(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CAN_TXERR_REG(PeripheralBase), CAN_TXERR_TXERR_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxErrorCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads transmit error counter register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXERR.
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadTxErrorCounterReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxErrorCounterReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXERR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIdAcceptanceCode1stBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance code (for first bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: IDAR_BANK_1[].
 * @par Example:
 *      @code
 *      uint32_t result =
 *      CAN_PDD_GetIdAcceptanceCode1stBank(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetIdAcceptanceCode1stBank(PeripheralBase) ( \
    (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,0U), \
     CAN_IDAR_BANK_1_AC_MASK \
    )) << 24) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,1U), \
     CAN_IDAR_BANK_1_AC_MASK \
    )) << 16) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,2U), \
     CAN_IDAR_BANK_1_AC_MASK \
    )) << 8) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,3U), \
     CAN_IDAR_BANK_1_AC_MASK \
    )) << 0) \
  )

/* ----------------------------------------------------------------------------
   -- SetIdAcceptanceCode1stBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value identifier acceptance code (for first bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Id Identifier acceptance code value for the first bank. This parameter
 *        is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDAR_BANK_1[].
 * @par Example:
 *      @code
 *      CAN_PDD_SetIdAcceptanceCode1stBank(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetIdAcceptanceCode1stBank(PeripheralBase, Id) ( \
    PeriphWriteReg16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,3U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,2U), \
     (uint16_t)(uint8_t)((uint32_t)((uint32_t)(Id) >> 8U) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,1U), \
     (uint16_t)(uint8_t)((uint16_t)((uint32_t)(Id) >> 16U) & 0xFFU) \
    ), \
    PeriphWriteReg16( \
     CAN_IDAR_BANK_1_REG(PeripheralBase,0U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) >> 24U) \
    ))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIdAcceptance1stBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance (first bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: IDAR_BANK_1[Index].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadIdAcceptance1stBankReg(<peripheral>_BASE_PTR, periphID);
 *      @endcode
 */
#define CAN_PDD_ReadIdAcceptance1stBankReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_IDAR_BANK_1_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIdAcceptance1stBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into identifier
 * acceptance (first bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @param Value Value to be written to the identifier acceptance (first bank)
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDAR_BANK_1[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteIdAcceptance1stBankReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteIdAcceptance1stBankReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_IDAR_BANK_1_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIdAcceptanceMask1stBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance mask (for first bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: IDMR_BANK_1[].
 * @par Example:
 *      @code
 *      uint32_t result =
 *      CAN_PDD_GetIdAcceptanceMask1stBank(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetIdAcceptanceMask1stBank(PeripheralBase) ( \
    (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,0U), \
     CAN_IDMR_BANK_1_AM_MASK \
    )) << 24) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,1U), \
     CAN_IDMR_BANK_1_AM_MASK \
    )) << 16) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,2U), \
     CAN_IDMR_BANK_1_AM_MASK \
    )) << 8) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,3U), \
     CAN_IDMR_BANK_1_AM_MASK \
    )) << 0) \
  )

/* ----------------------------------------------------------------------------
   -- SetIdAcceptanceMask1stBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value identifier acceptance mask (for first bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Id Identifier acceptance mask value for the first bank. This parameter
 *        is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDMR_BANK_1[].
 * @par Example:
 *      @code
 *      CAN_PDD_SetIdAcceptanceMask1stBank(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetIdAcceptanceMask1stBank(PeripheralBase, Id) ( \
    PeriphWriteReg16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,3U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,2U), \
     (uint16_t)(uint8_t)((uint32_t)((uint32_t)(Id) >> 8U) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,1U), \
     (uint16_t)(uint8_t)((uint16_t)((uint32_t)(Id) >> 16U) & 0xFFU) \
    ), \
    PeriphWriteReg16( \
     CAN_IDMR_BANK_1_REG(PeripheralBase,0U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) >> 24U) \
    ))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIdMask1stBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier mask (first bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: IDMR_BANK_1[Index].
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadIdMask1stBankReg(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define CAN_PDD_ReadIdMask1stBankReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_IDMR_BANK_1_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIdMask1StBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into identifier mask
 * (first bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier mask register index. This parameter is of index type.
 * @param Value Value to be written to the identifier mask (first bank)
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDMR_BANK_1[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteIdMask1StBankReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteIdMask1StBankReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_IDMR_BANK_1_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIdAcceptanceCode2ndBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance code (for second bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: IDAR_BANK_2[].
 * @par Example:
 *      @code
 *      uint32_t result =
 *      CAN_PDD_GetIdAcceptanceCode2ndBank(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetIdAcceptanceCode2ndBank(PeripheralBase) ( \
    (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,0U), \
     CAN_IDAR_BANK_2_AC_MASK \
    )) << 24) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,1U), \
     CAN_IDAR_BANK_2_AC_MASK \
    )) << 16) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,2U), \
     CAN_IDAR_BANK_2_AC_MASK \
    )) << 8) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,3U), \
     CAN_IDAR_BANK_2_AC_MASK \
    )) << 0) \
  )

/* ----------------------------------------------------------------------------
   -- SetIdAcceptanceCode2ndBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value identifier acceptance code (for second bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Id Identifier acceptance code value for the second bank. This
 *        parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDAR_BANK_2[].
 * @par Example:
 *      @code
 *      CAN_PDD_SetIdAcceptanceCode2ndBank(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetIdAcceptanceCode2ndBank(PeripheralBase, Id) ( \
    PeriphWriteReg16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,3U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,2U), \
     (uint16_t)(uint8_t)((uint32_t)((uint32_t)(Id) >> 8U) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,1U), \
     (uint16_t)(uint8_t)((uint16_t)((uint32_t)(Id) >> 16U) & 0xFFU) \
    ), \
    PeriphWriteReg16( \
     CAN_IDAR_BANK_2_REG(PeripheralBase,0U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) >> 24U) \
    ))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIdAcceptance2ndBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance (second bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: IDAR_BANK_2[Index].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadIdAcceptance2ndBankReg(<peripheral>_BASE_PTR, periphID);
 *      @endcode
 */
#define CAN_PDD_ReadIdAcceptance2ndBankReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_IDAR_BANK_2_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIdAcceptance2ndBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into identifier
 * acceptance (second bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @param Value Value to be written to the identifier acceptance (second bank)
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDAR_BANK_2[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteIdAcceptance2ndBankReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteIdAcceptance2ndBankReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_IDAR_BANK_2_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetIdAcceptanceMask2ndBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier acceptance mask (for second bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 32-bit value.
 * @remarks The macro accesses the following registers: IDMR_BANK_2[].
 * @par Example:
 *      @code
 *      uint32_t result =
 *      CAN_PDD_GetIdAcceptanceMask2ndBank(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetIdAcceptanceMask2ndBank(PeripheralBase) ( \
    (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,0U), \
     CAN_IDMR_BANK_2_AM_MASK \
    )) << 24) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,1U), \
     CAN_IDMR_BANK_2_AM_MASK \
    )) << 16) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,2U), \
     CAN_IDMR_BANK_2_AM_MASK \
    )) << 8) | (((uint32_t)(uint8_t)PeriphGetBitMask16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,3U), \
     CAN_IDMR_BANK_2_AM_MASK \
    )) << 0) \
  )

/* ----------------------------------------------------------------------------
   -- SetIdAcceptanceMask2ndBank
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the value identifier acceptance mask (for second bank).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Id Identifier acceptance mask value for the second bank. This
 *        parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDMR_BANK_2[].
 * @par Example:
 *      @code
 *      CAN_PDD_SetIdAcceptanceMask2ndBank(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetIdAcceptanceMask2ndBank(PeripheralBase, Id) ( \
    PeriphWriteReg16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,3U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,2U), \
     (uint16_t)(uint8_t)((uint32_t)((uint32_t)(Id) >> 8U) & 0xFFU) \
    ), \
    (PeriphWriteReg16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,1U), \
     (uint16_t)(uint8_t)((uint16_t)((uint32_t)(Id) >> 16U) & 0xFFU) \
    ), \
    PeriphWriteReg16( \
     CAN_IDMR_BANK_2_REG(PeripheralBase,0U), \
     (uint16_t)(uint8_t)((uint32_t)(Id) >> 24U) \
    ))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadIdMask2ndBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value identifier mask (second bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier acceptance register index. This parameter is of index
 *        type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: IDMR_BANK_2[Index].
 * @par Example:
 *      @code
 *      uint16_t result = CAN_PDD_ReadIdMask2ndBankReg(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define CAN_PDD_ReadIdMask2ndBankReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_IDMR_BANK_2_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteIdMask2ndBankReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into identifier mask
 * (second bank) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Identifier mask register index. This parameter is of index type.
 * @param Value Value to be written to the identifier mask (second bank)
 *        register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: IDMR_BANK_2[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteIdMask2ndBankReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteIdMask2ndBankReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_IDMR_BANK_2_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferID
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the received message buffer ID value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param TypeID Requested ID format. This parameter is of "Type of message
 *        buffer ID constants (for SetRxMessageBufferID, SetTxMessageBufferID
 *        macros)." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR0_STD,
 *          CAN_RXFG_IDR1_STD, CAN_RXFG_IDR0_EXT, CAN_RXFG_IDR1_EXT,
 *          CAN_RXFG_IDR2_EXT, CAN_RXFG_IDR3_EXT (depending on the peripheral).
 * @par Example:
 *      @code
 *      CAN_PDD_GetRxMessageBufferID(<peripheral>_BASE_PTR,
 *      CAN_PDD_BUFFER_ID_EXT);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferID(PeripheralBase, TypeID) ( \
    ((TypeID) == CAN_PDD_BUFFER_ID_STD) ? ( \
      (((uint32_t)(uint8_t)PeriphGetBitMask16( \
       CAN_RXFG_IDR0_STD_REG(PeripheralBase), \
       CAN_RXFG_IDR0_STD_ID_MASK \
      )) << 3) | \
      ((uint32_t)(uint8_t)(( \
       PeriphGetBitMask16( \
        CAN_RXFG_IDR1_STD_REG(PeripheralBase), \
        CAN_RXFG_IDR1_STD_ID_MASK \
       )) >> ( \
       CAN_RXFG_IDR1_STD_ID_SHIFT)))) : ( \
      (((uint32_t)(uint8_t)PeriphGetBitMask16( \
       CAN_RXFG_IDR0_EXT_REG(PeripheralBase), \
       CAN_RXFG_IDR0_EXT_ID_MASK \
      )) << 21) | \
      (((uint32_t)(uint8_t)(( \
       PeriphGetBitMask16( \
        CAN_RXFG_IDR1_EXT_REG(PeripheralBase), \
        CAN_RXFG_IDR1_EXT_ID_20_18_MASK \
       )) >> ( \
       CAN_RXFG_IDR1_EXT_ID_20_18_SHIFT))) << 18) | \
      (((uint32_t)(uint8_t)PeriphGetBitMask16( \
       CAN_RXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_RXFG_IDR1_EXT_ID_17_15_MASK \
      )) << 15) | \
      (((uint32_t)(uint8_t)PeriphGetBitMask16( \
       CAN_RXFG_IDR2_EXT_REG(PeripheralBase), \
       CAN_RXFG_IDR2_EXT_ID_MASK \
      )) << 7) | \
      ((uint32_t)(uint8_t)(( \
       PeriphGetBitMask16( \
        CAN_RXFG_IDR3_EXT_REG(PeripheralBase), \
        CAN_RXFG_IDR3_EXT_ID_MASK \
       )) >> ( \
       CAN_RXFG_IDR3_EXT_ID_SHIFT)))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferExtIdBits28To21Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (extended part ID
 * bits[28:21]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR0_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferExtIdBits28To21Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferExtIdBits28To21Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR0_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferIdExtBits28To21Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (extended part ID bits[28:21]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (extended
 *        part ID bits[28:21]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR0_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferIdExtBits28To21Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferIdExtBits28To21Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR0_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferStdIdBits10To3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (statnard part ID
 * bits[10:3]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR0_STD.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferStdIdBits10To3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferStdIdBits10To3Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR0_STD_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferStdIdBits10To3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (standard part ID bits[10:3]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (standard
 *        part ID bits[10:3]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR0_STD.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferStdIdBits10To3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferStdIdBits10To3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR0_STD_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferSRRExtId
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of substitute remote request bit in extended ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      CAN_PDD_GetRxMessageBufferSRRExtId(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferSRRExtId(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      CAN_RXFG_IDR1_EXT_REG(PeripheralBase), \
      CAN_RXFG_IDR1_EXT_SRR_MASK \
     )) >> ( \
     CAN_RXFG_IDR1_EXT_SRR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferIDExt
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of ID extended bit.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      uint8_t result = CAN_PDD_GetRxMessageBufferIDExt(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferIDExt(PeripheralBase) ( \
    (( \
      PeriphGetBitMask16( \
       CAN_RXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_RXFG_IDR1_EXT_IDE_MASK \
      )) == ( \
      0U)) ? ( \
      PDD_DISABLE) : ( \
      PDD_ENABLE) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferExtIdBits20To15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (extended part ID
 * bits[20:15]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferExtIdBits20To15Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferExtIdBits20To15Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR1_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferExtIdBits20To15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (extended part ID bits[20:15]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (extended
 *        part ID bits[20:15]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferExtIdBits20To15Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferExtIdBits20To15Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR1_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferRTRStdId
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of remote transmission request bit in received
 * standard ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      CAN_PDD_GetRxMessageBufferRTRStdId(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferRTRStdId(PeripheralBase) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      CAN_RXFG_IDR1_STD_REG(PeripheralBase), \
      CAN_RXFG_IDR1_STD_RTR_MASK \
     )) >> ( \
     CAN_RXFG_IDR1_STD_RTR_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferStdIdBits2To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (statnard part ID
 * bits[2:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferStdIdBits2To0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferStdIdBits2To0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR1_STD_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferStdIdBits2To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (standard part ID bits[2:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (standard
 *        part ID bits[2:0]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferStdIdBits2To0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferStdIdBits2To0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR1_STD_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferExtIdBits14To7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (extended part ID
 * bits[14:7]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR2_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferExtIdBits14To7Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferExtIdBits14To7Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR2_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferExtIdBits14To7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (extended part ID bits[14:7]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (extended
 *        part ID bits[14:7]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR2_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferExtIdBits14To7Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferExtIdBits14To7Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR2_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferRTRExtId
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of remote transmission request bit in receive
 * extended ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      CAN_PDD_GetRxMessageBufferRTRExtId(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferRTRExtId(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16( \
     CAN_RXFG_IDR3_EXT_REG(PeripheralBase), \
     CAN_RXFG_IDR3_EXT_RTR_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferExtIdBits6To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer indetifier (extended part ID
 * bits[6:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferExtIdBits6To0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferExtIdBits6To0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_IDR3_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferExtIdBits6To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * indetifier (extended part ID bits[6:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer indetifier (extended
 *        part ID bits[6:0]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferExtIdBits6To0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferExtIdBits6To0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_IDR3_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxMessageBufferID
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the transmit message buffer ID value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param TypeID Requested ID format. This parameter is of "Type of message
 *        buffer ID constants (for SetRxMessageBufferID, SetTxMessageBufferID
 *        macros)." type.
 * @param Value ID value. This parameter is a 32-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR0_STD,
 *          CAN_TXFG_IDR1_STD, CAN_TXFG_IDR0_EXT, CAN_TXFG_IDR1_EXT,
 *          CAN_TXFG_IDR2_EXT, CAN_TXFG_IDR3_EXT (depending on the peripheral).
 * @par Example:
 *      @code
 *      CAN_PDD_SetTxMessageBufferID(<peripheral>_BASE_PTR,
 *      CAN_PDD_BUFFER_ID_EXT, 1);
 *      @endcode
 */
#define CAN_PDD_SetTxMessageBufferID(PeripheralBase, TypeID, Value) ( \
    ((TypeID) == CAN_PDD_BUFFER_ID_STD) ? ( \
      PeriphSetBits16( \
       CAN_TXFG_IDR0_STD_REG(PeripheralBase), \
       CAN_TXFG_IDR0_STD_ID_MASK, \
       (uint16_t)((uint32_t)((uint32_t)(Value) >> 3U) & 0xFFU) \
      ), \
      PeriphSetBits16( \
       CAN_TXFG_IDR1_STD_REG(PeripheralBase), \
       (uint16_t)(CAN_TXFG_IDR1_STD_ID_MASK | CAN_TXFG_IDR1_STD_IDE_MASK), \
       (uint16_t)((uint16_t)((uint32_t)(Value) & 0x7U) << CAN_TXFG_IDR1_STD_ID_SHIFT) \
      )) : ( \
      PeriphSetBits16( \
       CAN_TXFG_IDR0_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR0_EXT_ID_MASK, \
       (uint16_t)((uint16_t)((uint32_t)(Value) >> 21U) & 0xFFU) \
      ), \
      (PeriphSetBits16( \
       CAN_TXFG_IDR1_EXT_REG(PeripheralBase), \
       (uint16_t)(CAN_TXFG_IDR1_EXT_ID_20_18_MASK | CAN_TXFG_IDR1_EXT_ID_17_15_MASK), \
       (uint16_t)(( \
        (uint16_t)(( \
         (uint16_t)((uint16_t)((uint32_t)(Value) >> 18U) & 0x7U)) << ( \
         CAN_TXFG_IDR1_EXT_ID_20_18_SHIFT))) | (( \
        CAN_TXFG_IDR1_EXT_IDE_MASK) | ( \
        (uint16_t)((uint32_t)((uint32_t)(Value) >> 15U) & 0x7U)))) \
      ), \
      (PeriphSetBits16( \
       CAN_TXFG_IDR2_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR2_EXT_ID_MASK, \
       (uint16_t)((uint32_t)((uint32_t)(Value) >> 7U) & 0xFFU) \
      ), \
      PeriphSetBits16( \
       CAN_TXFG_IDR3_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR3_EXT_ID_MASK, \
       (uint16_t)((uint16_t)((uint32_t)(Value) & 0x7FU) << CAN_TXFG_IDR3_EXT_ID_SHIFT) \
      )))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferExtIdBits28To21Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (extended part ID
 * bits[28:21]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR0_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferExtIdBits28To21Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferExtIdBits28To21Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR0_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferIdExtBits28To21Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (extended part ID bits[28:21]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (extended
 *        part ID bits[28:21]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR0_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferIdExtBits28To21Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferIdExtBits28To21Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR0_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferStdIdBits10To3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (statnard part ID
 * bits[10:3]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR0_STD.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferStdIdBits10To3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferStdIdBits10To3Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR0_STD_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferStdIdBits10To3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (standard part ID bits[10:3]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (standard
 *        part ID bits[10:3]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR0_STD.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferStdIdBits10To3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferStdIdBits10To3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR0_STD_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTxMessageBufferSRRExtId
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables substitute remote request bit in transmit extended ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested state of Substitute Remote Request bit. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTxMessageBufferSRRExtId(<peripheral>_BASE_PTR,
 *      PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableTxMessageBufferSRRExtId(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       CAN_TXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR1_EXT_SRR_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       CAN_TXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR1_EXT_SRR_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTxMessageBufferIDExt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables extended ID of the transmit message buffer.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested frame format. This parameter is of "Global enumeration
 *        used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTxMessageBufferIDExt(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableTxMessageBufferIDExt(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       CAN_TXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR1_EXT_IDE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       CAN_TXFG_IDR1_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR1_EXT_IDE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferExtIdBits20To15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (extended part ID
 * bits[20:15]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferExtIdBits20To15Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferExtIdBits20To15Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR1_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferExtIdBits20To15Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (extended part ID bits[20:15]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (extended
 *        part ID bits[20:15]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferExtIdBits20To15Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferExtIdBits20To15Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR1_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTxMessageBufferRTRStdId
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables remote transmission request bit in standard ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested state of Remote Transmission Request bit. This
 *        parameter is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTxMessageBufferRTRStdId(<peripheral>_BASE_PTR,
 *      PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableTxMessageBufferRTRStdId(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       CAN_TXFG_IDR1_STD_REG(PeripheralBase), \
       CAN_TXFG_IDR1_STD_RTR_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       CAN_TXFG_IDR1_STD_REG(PeripheralBase), \
       CAN_TXFG_IDR1_STD_RTR_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferStdIdBits2To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (statnard part ID
 * bits[2:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferStdIdBits2To0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferStdIdBits2To0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR1_STD_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferStdIdBits2To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (standard part ID bits[2:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (standard
 *        part ID bits[2:0]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR1_STD.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferStdIdBits2To0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferStdIdBits2To0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR1_STD_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferExtIdBits14To7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (extended part ID
 * bits[14:7]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR2_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferExtIdBits14To7Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferExtIdBits14To7Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR2_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferExtIdBits14To7Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (extended part ID bits[14:7]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (extended
 *        part ID bits[14:7]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR2_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferExtIdBits14To7Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferExtIdBits14To7Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR2_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTxMessageBufferRTRExtId
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables remote transmission request bit in extended ID.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Requested state of Remote Transmission Request bit. This
 *        parameter is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_EnableTxMessageBufferRTRExtId(<peripheral>_BASE_PTR,
 *      PDD_DISABLE);
 *      @endcode
 */
#define CAN_PDD_EnableTxMessageBufferRTRExtId(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       CAN_TXFG_IDR3_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR3_EXT_RTR_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       CAN_TXFG_IDR3_EXT_REG(PeripheralBase), \
       CAN_TXFG_IDR3_EXT_RTR_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferExtIdBits6To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer indetifier (extended part ID
 * bits[6:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferExtIdBits6To0Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferExtIdBits6To0Reg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_IDR3_EXT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferExtIdBits6To0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * indetifier (extended part ID bits[6:0]) register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer indetifier (extended
 *        part ID bits[6:0]) register. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_IDR3_EXT.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferExtIdBits6To0Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferExtIdBits6To0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_IDR3_EXT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferData
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the receive data buffer byte.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param DataIndex Data byte index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: RXFG_DSR[DataIndex].
 * @par Example:
 *      @code
 *      uint8_t result = CAN_PDD_GetRxMessageBufferData(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferData(PeripheralBase, DataIndex) ( \
    (uint8_t)PeriphReadReg16(CAN_RXFG_DSR_REG(PeripheralBase,(DataIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferDataSegmentReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive data buffer registers.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Data byte index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: RXFG_DSR[Index].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferDataSegmentReg(<peripheral>_BASE_PTR, periphID);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferDataSegmentReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_RXFG_DSR_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferDataSegmentReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive data
 * buffer registers.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Data byte index. This parameter is of index type.
 * @param Value Value to be written to the receive data buffer register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: RXFG_DSR[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferDataSegmentReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferDataSegmentReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_RXFG_DSR_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxMessageBufferDataLength
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of receive message buffer data length.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_RXFG_DLR.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      CAN_PDD_GetRxMessageBufferDataLength(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxMessageBufferDataLength(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(CAN_RXFG_DLR_REG(PeripheralBase), CAN_RXFG_DLR_DLC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferDataLengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer data length register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_DLR.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferDataLengthReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferDataLengthReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_DLR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRxBufferDataLengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into receive buffer
 * data length register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the receive buffer data length register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_RXFG_DLR.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteRxBufferDataLengthReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteRxBufferDataLengthReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_RXFG_DLR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetRxBufferTimeStampValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transmit time stamp value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_TSRH,
 *          CAN_RXFG_TSRL (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetRxBufferTimeStampValue(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetRxBufferTimeStampValue(PeripheralBase) ( \
    (uint16_t)((((uint16_t)(uint8_t)PeriphGetBitMask16(CAN_RXFG_TSRH_REG(PeripheralBase), CAN_RXFG_TSRH_TSR_MASK)) << 8) | ((uint16_t)(uint8_t)PeriphGetBitMask16(CAN_RXFG_TSRL_REG(PeripheralBase), CAN_RXFG_TSRL_TSR_MASK))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferTimeStampHighByteReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer time stamp - high byte register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_TSRH.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferTimeStampHighByteReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferTimeStampHighByteReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_TSRH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRxBufferTimeStampLowByteReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value receive buffer time stamp - low byte register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_RXFG_TSRL.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadRxBufferTimeStampLowByteReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadRxBufferTimeStampLowByteReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_RXFG_TSRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxMessageBufferData
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the data field in the message buffer denominated by DataIndex.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param DataIndex Data byte index. This parameter is of index type.
 * @param Value Data byte value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TXFG_DSR[DataIndex].
 * @par Example:
 *      @code
 *      CAN_PDD_SetTxMessageBufferData(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_SetTxMessageBufferData(PeripheralBase, DataIndex, Value) ( \
    PeriphWriteReg16( \
     CAN_TXFG_DSR_REG(PeripheralBase,(DataIndex)), \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferDataSegmentReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit data buffer registers.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Data byte index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TXFG_DSR[Index].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferDataSegmentReg(<peripheral>_BASE_PTR, periphID);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferDataSegmentReg(PeripheralBase, Index) ( \
    PeriphReadReg16(CAN_TXFG_DSR_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferDataSegmentReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit data
 * buffer registers.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Data byte index. This parameter is of index type.
 * @param Value Value to be written to the transmit data buffer register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TXFG_DSR[Index].
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferDataSegmentReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferDataSegmentReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(CAN_TXFG_DSR_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxMessageBufferDataLength
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of receive message buffer data length.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Message buffer code value[0..8]. This parameter is a 4-bit value.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAN_TXFG_DLR.
 * @par Example:
 *      @code
 *      uint8_t result =
 *      CAN_PDD_SetTxMessageBufferDataLength(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetTxMessageBufferDataLength(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_TXFG_DLR_REG(PeripheralBase), \
     CAN_TXFG_DLR_DLC_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferDataLengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer data length register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_DLR.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferDataLengthReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferDataLengthReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_DLR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferDataLengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * data length register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer data length register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_DLR.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferDataLengthReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferDataLengthReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_DLR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetTxBufferPriority
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * priority register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer priority register.
 *        This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_TBPR.
 * @par Example:
 *      @code
 *      CAN_PDD_SetTxBufferPriority(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_SetTxBufferPriority(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     CAN_TXFG_TBPR_REG(PeripheralBase), \
     CAN_TXFG_TBPR_PRIO_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferPriorityReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer priority register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_TBPR.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferPriorityReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferPriorityReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_TBPR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteTxBufferPriorityReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into transmit buffer
 * priority register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Value to be written to the transmit buffer priority register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAN_TXFG_TBPR.
 * @par Example:
 *      @code
 *      CAN_PDD_WriteTxBufferPriorityReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define CAN_PDD_WriteTxBufferPriorityReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(CAN_TXFG_TBPR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTxBufferTimeStampValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns transmit time stamp value.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_TSRH,
 *          CAN_TXFG_TSRL (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_GetTxBufferTimeStampValue(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_GetTxBufferTimeStampValue(PeripheralBase) ( \
    (uint16_t)((((uint16_t)(uint8_t)PeriphGetBitMask16(CAN_TXFG_TSRH_REG(PeripheralBase), CAN_TXFG_TSRH_TSR_MASK)) << 8) | ((uint16_t)(uint8_t)PeriphGetBitMask16(CAN_TXFG_TSRL_REG(PeripheralBase), CAN_TXFG_TSRL_TSR_MASK))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferTimeStampHighByteReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer time stamp - high byte register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_TSRH.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferTimeStampHighByteReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferTimeStampHighByteReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_TSRH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadTxBufferTimeStampLowByteReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value transmit buffer time stamp - low byte register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAN_TXFG_TSRL.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      CAN_PDD_ReadTxBufferTimeStampLowByteReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define CAN_PDD_ReadTxBufferTimeStampLowByteReg(PeripheralBase) ( \
    PeriphReadReg16(CAN_TXFG_TSRL_REG(PeripheralBase)) \
  )
#endif  /* #if defined(MSCAN_PDD_H_) */

/* MSCAN_PDD.h, eof. */
