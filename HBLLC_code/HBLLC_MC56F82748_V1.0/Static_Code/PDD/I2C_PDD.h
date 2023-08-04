/*
  PDD layer implementation for peripheral type I2C
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(I2C_PDD_H_)
#define I2C_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error I2C PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* I2C */ && \
      !defined(MCU_MC56F82316) /* I2C */ && \
      !defined(MCU_MC56F82723) /* I2C */ && \
      !defined(MCU_MC56F82726) /* I2C */ && \
      !defined(MCU_MC56F82728) /* I2C */ && \
      !defined(MCU_MC56F82733) /* I2C */ && \
      !defined(MCU_MC56F82736) /* I2C */ && \
      !defined(MCU_MC56F82738) /* I2C */ && \
      !defined(MCU_MC56F82743) /* I2C */ && \
      !defined(MCU_MC56F82746) /* I2C */ && \
      !defined(MCU_MC56F82748) /* I2C */
  // Unsupported MCU is active
  #error I2C PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Status flags constants (for ReadStatusReg, GetInterruptFlags,
   ClearInterruptFlags macros). */
#define I2C_PDD_RX_ACKNOWLEDGE      I2C_S_RXAK_MASK /**< Receive acknowledge. */
#define I2C_PDD_INTERRUPT_FLAG      I2C_S_IICIF_MASK /**< Interrupt flag. */
#define I2C_PDD_SLAVE_TRANSMIT      I2C_S_SRW_MASK /**< Slave read/write request. */
#define I2C_PDD_RANGE_ADDRESS_MATCH I2C_S_RAM_MASK /**< Range address match. */
#define I2C_PDD_ARBIT_LOST          I2C_S_ARBL_MASK /**< Arbitration lost. */
#define I2C_PDD_BUS_IS_BUSY         I2C_S_BUSY_MASK /**< Bus busy - set when a START signal is detected. */
#define I2C_PDD_ADDRESSED_AS_SLAVE  I2C_S_IAAS_MASK /**< Addressed as a slave. */
#define I2C_PDD_TX_COMPLETE         I2C_S_TCF_MASK /**< Transfer complete flag. */

/* Status flags constants. */
#define I2C_PDD_BUS_STOP_FLAG  I2C_FLT_STOPF_MASK /**< Stop detected on I2C bus */
#define I2C_PDD_BUS_START_FLAG I2C_FLT_STARTF_MASK /**< Start detected on I2C bus */

/* SCL timeout flags constants (for GetSCLTimeoutInterruptFlags,
   ClearSCLTimeoutInterruptFlags macros). */
#define I2C_PDD_SCL_LOW_TIMEOUT            I2C_SMB_SLTF_MASK /**< SCL low timeout flag. */
#define I2C_PDD_SCL_HI_AND_SDA_HI_TIMEOUT  I2C_SMB_SHTF1_MASK /**< SCL high timeout flag - sets when SCL and SDA are held high more than clock × LoValue / 512. */
#define I2C_PDD_SCL_HI_AND_SDA_LOW_TIMEOUT I2C_SMB_SHTF2_MASK /**< SCL high timeout flag - sets when SCL is held high and SDA is held low more than clock × LoValue/512. */

/* Frequency multiplier constants (for SetFrequencyMultiplier macro). */
#define I2C_PDD_FREQUENCY_MUL_1 0U               /**< Multiplier factor = 1 */
#define I2C_PDD_FREQUENCY_MUL_2 0x1U             /**< Multiplier factor = 2 */
#define I2C_PDD_FREQUENCY_MUL_4 0x2U             /**< Multiplier factor = 4 */

/* Master mode constants (for MasterMode, GetMasterMode macros). */
#define I2C_PDD_MASTER_MODE 0x20U                /**< Master mode. */
#define I2C_PDD_SLAVE_MODE  0U                   /**< Slave mode. */

/* Transmit mode constants (for SetTransmitMode, GetTransmitMode macros). */
#define I2C_PDD_TX_DIRECTION 0x10U               /**< SDA pin set as output. */
#define I2C_PDD_RX_DIRECTION 0U                  /**< SDA pin set as input. */

/* BUS status constants (for GetBusStatus macro). */
#define I2C_PDD_BUS_IDLE 0U                      /**< Bus is idle. */
#define I2C_PDD_BUS_BUSY 0x20U                   /**< Bus is busy. */

/* Fast SMBus Acknowledge constants (for GetFastSmBusAcknowledge macro). */
#define I2C_PDD_ACK_FOLLOWING_RX_DATA 0U         /**< ACK or NACK is sent on the following receiving data byte. */
#define I2C_PDD_ACK_AFTER_RX_DATA     0x80U      /**< ACK or NACK is sent after receiving a data byte. */

/* Clock source constants (for SetSCLTimeoutBusClockSource macro). */
#define I2C_PDD_BUS_CLOCK       0x10U            /**< Bus clock frequency */
#define I2C_PDD_BUS_CLOCK_DIV64 0U               /**< Bus clock / 64 frequency */


/* ----------------------------------------------------------------------------
   -- SetSlaveAddress7bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to the address register and set address length to 7 bits.
 * @param PeripheralBase Peripheral base address.
 * @param AddrValue Slave address value[0..127]. This parameter is a 7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_A1, I2C_C2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      I2C_PDD_SetSlaveAddress7bits(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetSlaveAddress7bits(PeripheralBase, AddrValue) ( \
    PeriphSetBits16( \
     I2C_A1_REG(PeripheralBase), \
     I2C_A1_AD_MASK, \
     (uint16_t)((uint16_t)(AddrValue) << I2C_A1_AD_SHIFT) \
    ), \
    PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_ADEXT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetSlaveAddress10bits
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes to the address register and set address length to 10 bits.
 * @param PeripheralBase Peripheral base address.
 * @param AddrValue Slave address value[0..1023]. This parameter is a 10-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_A1, I2C_C2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      I2C_PDD_SetSlaveAddress10bits(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetSlaveAddress10bits(PeripheralBase, AddrValue) ( \
    PeriphSetBits16( \
     I2C_A1_REG(PeripheralBase), \
     I2C_A1_AD_MASK, \
     (uint16_t)((uint16_t)((uint16_t)(AddrValue) & 0x7FU) << I2C_A1_AD_SHIFT) \
    ), \
    PeriphSetBits16( \
     I2C_C2_REG(PeripheralBase), \
     I2C_C2_AD_MASK, \
     (uint16_t)((uint16_t)((uint16_t)(AddrValue) >> 7U) | I2C_C2_ADEXT_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadAddress1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads address 1 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_A1.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadAddress1Reg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadAddress1Reg(PeripheralBase) ( \
    PeriphReadReg16(I2C_A1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteAddress1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into address 1
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the baud rate register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_A1.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteAddress1Reg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteAddress1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_A1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFrequencyDivider
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the prescaler to configure the I2C clock for the bit-rate
 * selection.
 * @param PeripheralBase Peripheral base address.
 * @param FreqDividerValue Frequency divider value[0..63]. This parameter is a
 *        6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_F.
 * @par Example:
 *      @code
 *      I2C_PDD_SetFrequencyDivider(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetFrequencyDivider(PeripheralBase, FreqDividerValue) ( \
    PeriphSetBits16( \
     I2C_F_REG(PeripheralBase), \
     I2C_F_ICR_MASK, \
     (uint16_t)(FreqDividerValue) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFrequencyMultiplier
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the factor, what is used along with frequency divider to generate
 * the I2C baud rate.
 * @param PeripheralBase Peripheral base address.
 * @param FreqMultiplierValue Frequency multiplier value[0..2]. This parameter
 *        is a 2-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_F.
 * @par Example:
 *      @code
 *      I2C_PDD_SetFrequencyMultiplier(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetFrequencyMultiplier(PeripheralBase, FreqMultiplierValue) ( \
    PeriphSetBits16( \
     I2C_F_REG(PeripheralBase), \
     I2C_F_MULT_MASK, \
     (uint16_t)((uint16_t)(FreqMultiplierValue) << I2C_F_MULT_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFrequencyDividerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads frequency divider register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_F.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadFrequencyDividerReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadFrequencyDividerReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_F_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFrequencyDividerReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into frequency
 * divider register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the frequency divider register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_F.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteFrequencyDividerReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteFrequencyDividerReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_F_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables I2C device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of device. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_IICEN_MASK)) : ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_IICEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the I2C interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_EnableInterrupt(PeripheralBase) ( \
    PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_IICIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the I2C interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_DisableInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_DisableInterrupt(PeripheralBase) ( \
    PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_IICIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetMasterMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts the I2C to the master or slave mode.
 * @param PeripheralBase Peripheral base address.
 * @param MasterMode I2C mode value. The user should use one from the enumerated
 *        values. This parameter is of "Master mode constants (for MasterMode,
 *        GetMasterMode macros)." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_SetMasterMode(deviceID, I2C_PDD_MASTER_MODE);
 *      @endcode
 */
#define I2C_PDD_SetMasterMode(PeripheralBase, MasterMode) ( \
    ((MasterMode) == I2C_PDD_SLAVE_MODE) ? ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_MST_MASK)) : ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_MST_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMasterMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current operating mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Master mode constants (for MasterMode,
 *         GetMasterMode macros)." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetMasterMode(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetMasterMode(PeripheralBase) ( \
    PeriphGetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_MST_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetTransmitMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets data pin to the output or input direction.
 * @param PeripheralBase Peripheral base address.
 * @param TransmitMode Direction I2C pins. The user should use one from the
 *        enumerated values. This parameter is of "Transmit mode constants (for
 *        SetTransmitMode, GetTransmitMode macros)." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_SetTransmitMode(deviceID, I2C_PDD_TX_DIRECTION);
 *      @endcode
 */
#define I2C_PDD_SetTransmitMode(PeripheralBase, TransmitMode) ( \
    ((TransmitMode) == I2C_PDD_RX_DIRECTION) ? ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_TX_MASK)) : ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_TX_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTransmitMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current direction mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Transmit mode constants (for SetTransmitMode,
 *         GetTransmitMode macros)." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetTransmitMode(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetTransmitMode(PeripheralBase) ( \
    PeriphGetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_TX_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableTransmitAcknowledge
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/disables an acknowledge signal to be sent to the bus at the
 * ninth clock bit after receiving one byte of data.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of acknowledge signal. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableTransmitAcknowledge(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableTransmitAcknowledge(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_TXAK_MASK)) : ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_TXAK_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- RepeatStart
   ---------------------------------------------------------------------------- */

/**
 * @brief Generates a repeated START condition.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_RepeatStart(deviceID);
 *      @endcode
 */
#define I2C_PDD_RepeatStart(PeripheralBase) ( \
    PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_RSTA_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWakeUp
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the wakeup function in stop3 mode.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of wakeup function. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableWakeUp(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableWakeUp(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_WUEN_MASK)) : ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_WUEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaRequest
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the DMA transfer.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of DMA function. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableDmaRequest(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableDmaRequest(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_DMAEN_MASK)) : ( \
      PeriphSetBitMask16(I2C_C1_REG(PeripheralBase), I2C_C1_DMAEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 1 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadControl1Reg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(I2C_C1_REG(PeripheralBase)) \
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
 * @remarks The macro accesses the following registers: I2C_C1.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteControl1Reg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_C1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_S.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadStatusReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_S_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetBusStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns status of the BUS.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "BUS status constants (for GetBusStatus macro)."
 *         type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: I2C_S.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetBusStatus(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetBusStatus(PeripheralBase) ( \
    PeriphGetBitMask16(I2C_S_REG(PeripheralBase), I2C_S_BUSY_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns a value that represents a mask of active (pending) interrupts.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_S.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetInterruptFlags(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     I2C_S_REG(PeripheralBase), \
     (uint16_t)(( \
      I2C_S_TCF_MASK) | (( \
      I2C_S_IICIF_MASK) | (( \
      I2C_S_RAM_MASK) | (( \
      I2C_S_ARBL_MASK) | ( \
      I2C_S_IAAS_MASK))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flags of interrupts specified by Mask.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_S.
 * @par Example:
 *      @code
 *      I2C_PDD_ClearInterruptFlags(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSafeSetBits16( \
     I2C_S_REG(PeripheralBase), \
     I2C_S_ARBL_MASK, \
     I2C_S_IICIF_MASK, \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the status register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_S.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteStatusReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_S_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the data register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_D.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadDataReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadDataReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_D_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes data to the data register.
 * @param PeripheralBase Peripheral base address.
 * @param Data Data value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_D.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteDataReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteDataReg(PeripheralBase, Data) ( \
    PeriphWriteReg16(I2C_D_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableGeneralCallAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables general call address.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of general call address function. This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableGeneralCallAddress(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableGeneralCallAddress(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_GCAEN_MASK)) : ( \
      PeriphSetBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_GCAEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAddressExtension
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables extension address.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of extension address. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableAddressExtension(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableAddressExtension(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_ADEXT_MASK)) : ( \
      PeriphSetBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_ADEXT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPadsNormalDriveMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets control the drive capability of the I2C pads to normal drive mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_SetPadsNormalDriveMode(deviceID);
 *      @endcode
 */
#define I2C_PDD_SetPadsNormalDriveMode(PeripheralBase) ( \
    PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_HDRS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPadsHighDriveMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets control the drive capability of the I2C pads to high drive mode.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_SetPadsHighDriveMode(deviceID);
 *      @endcode
 */
#define I2C_PDD_SetPadsHighDriveMode(PeripheralBase) ( \
    PeriphSetBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_HDRS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSlaveBaudControlByMaster
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables slave baud rate control by master device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of slave baud rate control. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableSlaveBaudControlByMaster(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableSlaveBaudControlByMaster(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_SBRC_MASK)) : ( \
      PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_SBRC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRangeAddressMatch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables range address matching.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of range address matching function. This
 *        parameter is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableRangeAddressMatch(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableRangeAddressMatch(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_RMEN_MASK)) : ( \
      PeriphSetBitMask16(I2C_C2_REG(PeripheralBase), I2C_C2_RMEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads control 2 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadControl2Reg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(I2C_C2_REG(PeripheralBase)) \
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
 * @remarks The macro accesses the following registers: I2C_C2.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteControl2Reg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_C2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputGlitchFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the programming controls for the width of glitch (in terms of bus
 * clock cycles) the filter must absorb.
 * @param PeripheralBase Peripheral base address.
 * @param FilterFactorValue Input glitch filter value[0..15]. This parameter is
 *        a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_SetInputGlitchFilter(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetInputGlitchFilter(PeripheralBase, FilterFactorValue) ( \
    PeriphSafeSetBits16( \
     I2C_FLT_REG(PeripheralBase), \
     (uint16_t)(I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK), \
     I2C_FLT_FLT_MASK, \
     (uint16_t)(FilterFactorValue) \
    ) \
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
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableStopHoldOffMode(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableStopHoldOffMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       I2C_FLT_REG(PeripheralBase), \
       (uint16_t)(I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK), \
       I2C_FLT_SHEN_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       I2C_FLT_REG(PeripheralBase), \
       (uint16_t)(I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK), \
       I2C_FLT_SHEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBusStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value of the bus status flags.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadBusStatusFlags(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadBusStatusFlags(PeripheralBase) ( \
    PeriphReadReg16(I2C_FLT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearBusStatusInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears bus status interrupt flags of interrupts specified by Mask.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_ClearBusStatusInterruptFlags(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_ClearBusStatusInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     I2C_FLT_REG(PeripheralBase), \
     (uint16_t)(I2C_FLT_STOPF_MASK | I2C_FLT_STARTF_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBusStopOrStartInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the bus stop or start interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableBusStopOrStartInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_EnableBusStopOrStartInterrupt(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     I2C_FLT_REG(PeripheralBase), \
     (uint16_t)(I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK), \
     I2C_FLT_SSIE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableBusStopOrStartInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the bus stop or start interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_DisableBusStopOrStartInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_DisableBusStopOrStartInterrupt(PeripheralBase) ( \
    PeriphSafeClearBitMask16( \
     I2C_FLT_REG(PeripheralBase), \
     (uint16_t)(I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK), \
     I2C_FLT_SSIE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInputGlitchFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads input glitch filter register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadInputGlitchFilterReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadInputGlitchFilterReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_FLT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInputGlitchFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into input glitch
 * filter register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the input glitch filter register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_FLT.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteInputGlitchFilterReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteInputGlitchFilterReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_FLT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetRangeAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the range slave address.
 * @param PeripheralBase Peripheral base address.
 * @param RangeAddressValue Range Address value[0..127]. This parameter is a
 *        7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_RA.
 * @par Example:
 *      @code
 *      I2C_PDD_SetRangeAddress(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetRangeAddress(PeripheralBase, RangeAddressValue) ( \
    PeriphSetBits16( \
     I2C_RA_REG(PeripheralBase), \
     I2C_RA_RAD_MASK, \
     (uint16_t)((uint16_t)(RangeAddressValue) << I2C_RA_RAD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRangeAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads value of range address register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_RA.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadRangeAddressReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadRangeAddressReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_RA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteRangeAddressReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into range address
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the range address register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_RA.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteRangeAddressReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteRangeAddressReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_RA_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFastSmBusNackAck
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables fast SMBus NACK/ACK.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus alert response will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableFastSmBusNackAck(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableFastSmBusNackAck(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_FACK_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_FACK_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFastSmBusAcknowledge
   ---------------------------------------------------------------------------- */

/**
 * @brief Fast NACK/ACK enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Fast SMBus Acknowledge constants (for
 *         GetFastSmBusAcknowledge macro)." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetFastSmBusAcknowledge(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetFastSmBusAcknowledge(PeripheralBase) ( \
    PeriphGetBitMask16(I2C_SMB_REG(PeripheralBase), I2C_SMB_FACK_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSmBusAlertResponseAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables SMBus alert response address.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus alert response will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableSmBusAlertResponseAddress(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableSmBusAlertResponseAddress(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_ALERTEN_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_ALERTEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSecondI2CAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables SMBus device default address.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if SMBus device default address will be
 *        enabled or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableSecondI2CAddress(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define I2C_PDD_EnableSecondI2CAddress(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeClearBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_SIICAEN_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_SIICAEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCLTimeoutBusClockSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets clock source of the timeout counter to Bus clock or Bus clock/64
 * frequency.
 * @param PeripheralBase Peripheral base address.
 * @param ClockSource SCL timeout BUS clock source value. The user should use
 *        one from the enumerated values. This parameter is of "Clock source
 *        constants (for SetSCLTimeoutBusClockSource macro)." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_SetSCLTimeoutBusClockSource(deviceID, I2C_PDD_BUS_CLOCK);
 *      @endcode
 */
#define I2C_PDD_SetSCLTimeoutBusClockSource(PeripheralBase, ClockSource) ( \
    ((ClockSource) == I2C_PDD_BUS_CLOCK_DIV64) ? ( \
      PeriphSafeClearBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_TCKSEL_MASK \
      )) : ( \
      PeriphSafeSetBitMask16( \
       I2C_SMB_REG(PeripheralBase), \
       (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
       I2C_SMB_TCKSEL_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetSCLTimeoutInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns a value that represents a mask of active (pending) interrupts.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_GetSCLTimeoutInterruptFlags(deviceID);
 *      @endcode
 */
#define I2C_PDD_GetSCLTimeoutInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     I2C_SMB_REG(PeripheralBase), \
     (uint16_t)(I2C_SMB_SLTF_MASK | (I2C_SMB_SHTF1_MASK | I2C_SMB_SHTF2_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearSCLTimeoutInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flags of interrupts specified by Mask.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Mask of interrupt clear requests. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_ClearSCLTimeoutInterruptFlags(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_ClearSCLTimeoutInterruptFlags(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     I2C_SMB_REG(PeripheralBase), \
     (uint16_t)(I2C_SMB_SLTF_MASK | I2C_SMB_SHTF2_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSCLTimeoutInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables SCL high and SDA low timeout interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_EnableSCLTimeoutInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_EnableSCLTimeoutInterrupt(PeripheralBase) ( \
    PeriphSafeSetBitMask16( \
     I2C_SMB_REG(PeripheralBase), \
     (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
     I2C_SMB_SHTF2IE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableSCLTimeoutInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables SCL high and SDA low timeout interrupt.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_DisableSCLTimeoutInterrupt(deviceID);
 *      @endcode
 */
#define I2C_PDD_DisableSCLTimeoutInterrupt(PeripheralBase) ( \
    PeriphSafeClearBitMask16( \
     I2C_SMB_REG(PeripheralBase), \
     (uint16_t)(I2C_SMB_SHTF2_MASK | I2C_SMB_SLTF_MASK), \
     I2C_SMB_SHTF2IE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSMBusControlAndStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads SMBus control and status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadSMBusControlAndStatusReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadSMBusControlAndStatusReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_SMB_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSMBusControlAndStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into SMBus control
 * and status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the SMBus control and status register.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SMB.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteSMBusControlAndStatusReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteSMBusControlAndStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_SMB_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSMBusSlaveAddress
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets SMBus address to the address register.
 * @param PeripheralBase Peripheral base address.
 * @param SMBusSlaveAddrValue SMBus slave address value[0..127]. This parameter
 *        is a 7-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_A2.
 * @par Example:
 *      @code
 *      I2C_PDD_SetSMBusSlaveAddress(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetSMBusSlaveAddress(PeripheralBase, SMBusSlaveAddrValue) ( \
    PeriphSetBits16( \
     I2C_A2_REG(PeripheralBase), \
     I2C_A2_SAD_MASK, \
     (uint16_t)((uint16_t)(SMBusSlaveAddrValue) << I2C_A2_SAD_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadAddress2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads address 2 register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_A2.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadAddress2Reg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadAddress2Reg(PeripheralBase) ( \
    PeriphReadReg16(I2C_A2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteAddress2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into address 2
 * register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the address 2 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_A2.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteAddress2Reg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteAddress2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_A2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSCLLowTimeout
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets SCL low timeout value that determines the timeout period of SCL
 * low.
 * @param PeripheralBase Peripheral base address.
 * @param SCLLowTimeoutValue SCL low timeout value[0..65535]. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SLTL, I2C_SLTH
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      I2C_PDD_SetSCLLowTimeout(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_SetSCLLowTimeout(PeripheralBase, SCLLowTimeoutValue) ( \
    PeriphSetBits16( \
     I2C_SLTL_REG(PeripheralBase), \
     I2C_SLTL_SSLT_MASK, \
     (uint16_t)((uint16_t)(SCLLowTimeoutValue) & 0xFFU) \
    ), \
    PeriphSetBits16( \
     I2C_SLTH_REG(PeripheralBase), \
     I2C_SLTH_SSLT_MASK, \
     (uint16_t)((uint16_t)(SCLLowTimeoutValue) >> 8U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSclLowTimeoutHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads SCL low timeout high register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_SLTH.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadSclLowTimeoutHighReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadSclLowTimeoutHighReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_SLTH_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSclLowTimeoutHighReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into SCL low timeout
 * high register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the SCL low timeout high register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SLTH.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteSclLowTimeoutHighReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteSclLowTimeoutHighReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_SLTH_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSclLowTimeoutLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads SCL low timeout high register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: I2C_SLTL.
 * @par Example:
 *      @code
 *      uint16_t result = I2C_PDD_ReadSclLowTimeoutLowReg(deviceID);
 *      @endcode
 */
#define I2C_PDD_ReadSclLowTimeoutLowReg(PeripheralBase) ( \
    PeriphReadReg16(I2C_SLTL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSclLowTimeoutLowReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes new value specified by the Value parameter into SCL low timeout
 * low register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Value to be written to the SCL low timeout low register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: I2C_SLTL.
 * @par Example:
 *      @code
 *      I2C_PDD_WriteSclLowTimeoutLowReg(deviceID, 1);
 *      @endcode
 */
#define I2C_PDD_WriteSclLowTimeoutLowReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(I2C_SLTL_REG(PeripheralBase), (uint16_t)(Value)) \
  )
#endif  /* #if defined(I2C_PDD_H_) */

/* I2C_PDD.h, eof. */
