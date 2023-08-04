/*
  PDD layer implementation for peripheral type DAC
  Copyright 2010 Freescale, Inc.
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(DAC_PDD_H_)
#define DAC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error DAC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82316) /* DACA, DACB */ && \
      !defined(MCU_MC56F82723) /* DACA, DACB */ && \
      !defined(MCU_MC56F82726) /* DACA, DACB */ && \
      !defined(MCU_MC56F82728) /* DACA, DACB */ && \
      !defined(MCU_MC56F82733) /* DACA, DACB */ && \
      !defined(MCU_MC56F82736) /* DACA, DACB */ && \
      !defined(MCU_MC56F82738) /* DACA, DACB */ && \
      !defined(MCU_MC56F82743) /* DACA, DACB */ && \
      !defined(MCU_MC56F82746) /* DACA, DACB */ && \
      !defined(MCU_MC56F82748) /* DACA, DACB */
  // Unsupported MCU is active
  #error DAC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* D/A converter's trigger source constant. */
#define DAC_PDD_HW_TRIGGER 0x4U                  /**< Rising edge of SYNC_IN updates data in the buffered data register presented to the DAC input. */
#define DAC_PDD_SW_TRIGGER 0U                    /**< Data written to the buffered data register is presented to DAC inputs on the next clock cycle. */

/* D/A converter's buffer watermark constant. For exact value representation see
   peripheral device documentation. */
#define DAC_PDD_BUFFER_WATERMARK_L0 0U           /**< Level 0 watermark (0 word). */
#define DAC_PDD_BUFFER_WATERMARK_L2 0x100U       /**< Level 2 watermark (2 words). */
#define DAC_PDD_BUFFER_WATERMARK_L4 0x200U       /**< Level 4 watermark (4 words). */
#define DAC_PDD_BUFFER_WATERMARK_L6 0x300U       /**< Level 6 watermark (6 words). */

/* D/A converter's buffer low power mode constant. */
#define DAC_PDD_HIGH_POWER 0U                    /**< High power mode. */
#define DAC_PDD_LOW_POWER  DAC_CTRL0_HSLS_MASK   /**< Low power mode. */

/* Defines counting modes constant. */
#define DAC_PDD_UP_COUNTING   DAC_CTRL0_UP_MASK  /**< Mask enables counting up mode. */
#define DAC_PDD_DOWN_COUNTING DAC_CTRL0_DOWN_MASK /**< Mask enables counting down mode. */

/* Automatic mode constants. */
#define DAC_PDD_NORMAL_MODE        0U            /**< Normal mode. Automatic waveform generation disabled. */
#define DAC_PDD_AUTOMATIC_WAVEFORM 0x8U          /**< Automatic waveform generation enabled. */

/* Data format constants. */
#define DAC_PDD_JUSTIFIED_RIGHT 0U               /**< Data words are right justified. */
#define DAC_PDD_JUSTIFIED_LEFT  0x2U             /**< Data words are left justified. */

/* Indicates status of FIFO. */
#define DAC_PDD_FIFO_FULL  DAC_STATUS_FULL_MASK  /**< FIFO is full. */
#define DAC_PDD_FIFO_EMPTY DAC_STATUS_EMPTY_MASK /**< FIFO is empty. */


/* ----------------------------------------------------------------------------
   -- EnableDevice
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the D/A converter's device.
 * @param PeripheralBase Peripheral base address.
 * @param State Requested state of D/A converter. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_EnableDevice(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define DAC_PDD_EnableDevice(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_PDN_MASK)) : ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_PDN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDeviceEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of D/A converter enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = DAC_PDD_GetDeviceEnabled(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetDeviceEnabled(PeripheralBase) ( \
    (( \
      (uint8_t)PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_PDN_MASK)) == ( \
      0U)) ? ( \
      PDD_ENABLE) : ( \
      PDD_DISABLE) \
  )

/* ----------------------------------------------------------------------------
   -- SetTrigger
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects D/A converter's trigger.
 * @param PeripheralBase Peripheral base address.
 * @param Trigger Parameter specifying which trigger source will be used. This
 *        parameter is of "D/A converter's trigger source constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetTrigger(deviceID, DAC_PDD_HW_TRIGGER);
 *      @endcode
 */
#define DAC_PDD_SetTrigger(PeripheralBase, Trigger) ( \
    ((Trigger) == DAC_PDD_SW_TRIGGER) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_SYNC_EN_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_SYNC_EN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetTriggerSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current trigger source.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "D/A converter's trigger source constant." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetTriggerSource(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetTriggerSource(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_SYNC_EN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetBufferWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets D/A converter's buffer watermark.
 * @param PeripheralBase Peripheral base address.
 * @param Watermark Parameter specifying data buffer watermark level. This
 *        parameter is of "D/A converter's buffer watermark constant. For exact value
 *        representation see peripheral device documentation." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetBufferWatermark(deviceID, DAC_PDD_BUFFER_WATERMARK_L0);
 *      @endcode
 */
#define DAC_PDD_SetBufferWatermark(PeripheralBase, Watermark) ( \
    PeriphSetBits16( \
     DAC_CTRL0_REG(PeripheralBase), \
     DAC_CTRL0_WTMK_LVL_MASK, \
     (uint16_t)(Watermark) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBufferWatermark
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current data buffer watermark level.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "D/A converter's buffer watermark constant. For
 *         exact value representation see peripheral device documentation." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetBufferWatermark(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetBufferWatermark(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_WTMK_LVL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects D/A converter's low power mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Parameter specifying if high or low power mode will be used. This
 *        parameter is of "D/A converter's buffer low power mode constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetPowerMode(deviceID, DAC_PDD_HIGH_POWER);
 *      @endcode
 */
#define DAC_PDD_SetPowerMode(PeripheralBase, Mode) ( \
    ((Mode) == DAC_PDD_HIGH_POWER) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_HSLS_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_HSLS_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current trigger source.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "D/A converter's buffer low power mode constant."
 *         type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetPowerMode(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetPowerMode(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_HSLS_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaRequest
   ---------------------------------------------------------------------------- */

/**
 * @brief DMA enable control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if DMA requests will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_EnableDmaRequest(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define DAC_PDD_EnableDmaRequest(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_DMA_EN_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_DMA_EN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDmaRequestEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of DMA enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetDmaRequestEnabled(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetDmaRequestEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_DMA_EN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableGlitchFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the glitch suppression filter.
 * @param PeripheralBase Peripheral base address.
 * @param FilterState Defines state of the glitch suppression filter. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_EnableGlitchFilter(deviceID, PDD_DISABLE);
 *      @endcode
 */
#define DAC_PDD_EnableGlitchFilter(PeripheralBase, FilterState) ( \
    ((FilterState) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FILT_EN_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FILT_EN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetGlitchFilterEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Retrieves wether the glitch suppression filter is enabled or not.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetGlitchFilterEnabled(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetGlitchFilterEnabled(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FILT_EN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetCountingMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables counting Up and Down modes.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Parameter accepts UP_COUNTING | DOWN_COUNTING masks. When a mask
 *        is not used the desired mode will be disabled. This parameter is of
 *        "Defines counting modes constant." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetCountingMask(deviceID, DAC_PDD_UP_COUNTING);
 *      @endcode
 */
#define DAC_PDD_SetCountingMask(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     DAC_CTRL0_REG(PeripheralBase), \
     (uint16_t)(DAC_CTRL0_UP_MASK | DAC_CTRL0_DOWN_MASK), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCountingMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current counting Up and Down mask. The result should be tested
 * whether UP_COUNTING or DOWN_COUNTING are presented, if not the desired mode
 * is disabled.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetCountingMask(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetCountingMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     DAC_CTRL0_REG(PeripheralBase), \
     (uint16_t)(DAC_CTRL0_UP_MASK | DAC_CTRL0_DOWN_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetAutomaticMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets normal or automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Defines normal or automatic waveform generation mode. This
 *        parameter is of "Automatic mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetAutomaticMode(deviceID, DAC_PDD_NORMAL_MODE);
 *      @endcode
 */
#define DAC_PDD_SetAutomaticMode(PeripheralBase, Mode) ( \
    ((Mode) == DAC_PDD_NORMAL_MODE) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_AUTO_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_AUTO_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetAutomaticMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Retrieves wether the normal or automatic waveform generation mode is
 * selected.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Automatic mode constants." type. The value is
 *         cast to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetAutomaticMode(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetAutomaticMode(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_AUTO_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetWaveformMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets waveform generation parameters.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Parameter accepts AUTOMATIC_WAVEFORM | UP_COUNTING |
 *        DOWN_COUNTING masks. When a mask is not used the desired mode will be disabled.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetWaveformMask(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetWaveformMask(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     DAC_CTRL0_REG(PeripheralBase), \
     (uint16_t)(DAC_CTRL0_UP_MASK | (DAC_CTRL0_DOWN_MASK | DAC_CTRL0_AUTO_MASK)), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDataFormat
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets normal or automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Mode Defines normal or automatic waveform generation mode. This
 *        parameter is of "Data format constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetDataFormat(deviceID, DAC_PDD_JUSTIFIED_RIGHT);
 *      @endcode
 */
#define DAC_PDD_SetDataFormat(PeripheralBase, Mode) ( \
    ((Mode) == DAC_PDD_JUSTIFIED_RIGHT) ? ( \
      PeriphClearBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FORMAT_MASK)) : ( \
      PeriphSetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FORMAT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDataFormat
   ---------------------------------------------------------------------------- */

/**
 * @brief Retrieves wether the normal or automatic waveform generation mode is
 * selected.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Data format constants." type. The value is cast
 *         to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetDataFormat(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetDataFormat(PeripheralBase) ( \
    PeriphGetBitMask16(DAC_CTRL0_REG(PeripheralBase), DAC_CTRL0_FORMAT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetGlitchFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the glitch filter count.
 * @param PeripheralBase Peripheral base address.
 * @param Count Defines the number of IP Bus clock cycles for which the DAC
 *        output is held unchanged after new data is presented to the analog DAC's
 *        inputs. This parameter is a 6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL1, DACB_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetGlitchFilter(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetGlitchFilter(PeripheralBase, Count) ( \
    PeriphSetBits16( \
     DAC_CTRL1_REG(PeripheralBase), \
     DAC_CTRL1_FILT_CNT_MASK, \
     (uint16_t)(Count) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetGlitchFilter
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the glitch filter count. The value specifies the number of IP
 * Bus clock cycles for which the DAC output is held unchanged after new data is
 * presented to the analog DAC's inputs.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 6-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: DACA_CTRL1, DACB_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint8_t result = DAC_PDD_GetGlitchFilter(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetGlitchFilter(PeripheralBase) ( \
    (uint8_t)PeriphGetBitMask16(DAC_CTRL1_REG(PeripheralBase), DAC_CTRL1_FILT_CNT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetFifoStatusMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current mask status of FIFO. The result should be tested
 * whether FIFO_FULL or FIFO_EMPTY are presented, if not the desired status is not
 * present.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a value of "Indicates status of FIFO." type. The value is
 *         cast to "uint16_t".
 * @remarks The macro accesses the following registers: DACA_STATUS, DACB_STATUS
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetFifoStatusMask(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetFifoStatusMask(PeripheralBase) ( \
    PeriphGetBitMask16( \
     DAC_STATUS_REG(PeripheralBase), \
     (uint16_t)(DAC_STATUS_FULL_MASK | DAC_STATUS_EMPTY_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetData
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes data into DAC buffer.
 * @param PeripheralBase Peripheral base address.
 * @param Data The right justified data. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT0,
 *          DACB_DATAREG_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetData(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetData(PeripheralBase, Data) ( \
    PeriphWriteReg16(DAC_DATAREG_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetData
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the top of DAC buffer. The data are right justified.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT0,
 *          DACB_DATAREG_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetData(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetData(PeripheralBase) ( \
    PeriphReadReg16(DAC_DATAREG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDataLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes data into DAC buffer.
 * @param PeripheralBase Peripheral base address.
 * @param Data The left justified data. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT1,
 *          DACB_DATAREG_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetDataLeft(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetDataLeft(PeripheralBase, Data) ( \
    PeriphWriteReg16(DAC_DATAREG_FMT1_REG(PeripheralBase), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDataLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the top of DAC buffer. The data are left justified.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT1,
 *          DACB_DATAREG_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetDataLeft(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetDataLeft(PeripheralBase) ( \
    PeriphReadReg16(DAC_DATAREG_FMT1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetStep
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the step size in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Step The right justified value will be added to or subtracted from the
 *        current value to create the next value presented to the DAC inputs.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT0,
 *          DACB_STEPVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetStep(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetStep(PeripheralBase, Step) ( \
    PeriphWriteReg16(DAC_STEPVAL_REG(PeripheralBase), (uint16_t)(Step)) \
  )

/* ----------------------------------------------------------------------------
   -- GetStep
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the step size in automatic waveform generation mode. The right
 * justified value will be added to or subtracted from the current value to
 * create the next value presented to the DAC inputs.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT0,
 *          DACB_STEPVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetStep(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetStep(PeripheralBase) ( \
    PeriphReadReg16(DAC_STEPVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetStepLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the step size in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Step The left justified value will be added to or subtracted from the
 *        current value to create the next value presented to the DAC inputs.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT1,
 *          DACB_STEPVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetStepLeft(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetStepLeft(PeripheralBase, Step) ( \
    PeriphWriteReg16(DAC_STEPVAL_FMT1_REG(PeripheralBase), (uint16_t)(Step)) \
  )

/* ----------------------------------------------------------------------------
   -- GetStepLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the step size in automatic waveform generation mode. The left
 * justified value will be added to or subtracted from the current value to
 * create the next value presented to the DAC inputs.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT1,
 *          DACB_STEPVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetStepLeft(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetStepLeft(PeripheralBase) ( \
    PeriphReadReg16(DAC_STEPVAL_FMT1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMinimum
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the minimum value in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Value The right justified minimum value acts as the lower range limit
 *        during automatic waveform generation. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT0,
 *          DACB_MINVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetMinimum(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetMinimum(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MINVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMinimum
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the minimum value in automatic waveform generation mode. The
 * right justified minimum value acts as the lower range limit during automatic
 * waveform generation.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT0,
 *          DACB_MINVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetMinimum(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetMinimum(PeripheralBase) ( \
    PeriphReadReg16(DAC_MINVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMinimumLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the minimum value in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Value The left justified minimum value acts as the lower range limit
 *        during automatic waveform generation. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT1,
 *          DACB_MINVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetMinimumLeft(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetMinimumLeft(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MINVAL_FMT1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMinimumLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the minimum value in automatic waveform generation mode. The
 * left justified minimum value acts as the lower range limit during automatic
 * waveform generation.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT1,
 *          DACB_MINVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetMinimumLeft(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetMinimumLeft(PeripheralBase) ( \
    PeriphReadReg16(DAC_MINVAL_FMT1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaximum
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the maximum value in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Value The right justified maximum value acts as the upper range limit
 *        during automatic waveform generation. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT0,
 *          DACB_MAXVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetMaximum(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetMaximum(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MAXVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMaximum
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the maximum value in automatic waveform generation mode. The
 * right justified maximum value acts as the upper range limit during automatic
 * waveform generation.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT0,
 *          DACB_MAXVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetMaximum(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetMaximum(PeripheralBase) ( \
    PeriphReadReg16(DAC_MAXVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMaximumLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the maximum value in automatic waveform generation mode.
 * @param PeripheralBase Peripheral base address.
 * @param Value The left justified maximum value acts as the upper range limit
 *        during automatic waveform generation. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT1,
 *          DACB_MAXVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_SetMaximumLeft(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_SetMaximumLeft(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MAXVAL_FMT1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMaximumLeft
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the maximum value in automatic waveform generation mode. The
 * left justified maximum value acts as the upper range limit during automatic
 * waveform generation.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT1,
 *          DACB_MAXVAL_FMT1 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_GetMaximumLeft(deviceID);
 *      @endcode
 */
#define DAC_PDD_GetMaximumLeft(PeripheralBase) ( \
    PeriphReadReg16(DAC_MAXVAL_FMT1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes control register 0.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into control register 0. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteControl0Reg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteControl0Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_CTRL0_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the control register 0.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_CTRL0, DACB_CTRL0
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadControl0Reg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadControl0Reg(PeripheralBase) ( \
    PeriphReadReg16(DAC_CTRL0_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes control register 1.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into control register 1. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_CTRL1, DACB_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteControl1Reg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_CTRL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the control register 1.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_CTRL1, DACB_CTRL1
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadControl1Reg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(DAC_CTRL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes status register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into status register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_STATUS, DACB_STATUS
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteStatusReg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_STATUS_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the status register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_STATUS, DACB_STATUS
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadStatusReg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(DAC_STATUS_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteBufferedDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes buffered data register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into buffered data register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT0,
 *          DACB_DATAREG_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteBufferedDataReg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteBufferedDataReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_DATAREG_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadBufferedDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the buffered data register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_DATAREG_FMT0,
 *          DACB_DATAREG_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadBufferedDataReg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadBufferedDataReg(PeripheralBase) ( \
    PeriphReadReg16(DAC_DATAREG_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStepSizeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes step size register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into step size register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT0,
 *          DACB_STEPVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteStepSizeReg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteStepSizeReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_STEPVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStepSizeReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the step size register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_STEPVAL_FMT0,
 *          DACB_STEPVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadStepSizeReg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadStepSizeReg(PeripheralBase) ( \
    PeriphReadReg16(DAC_STEPVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMinimumValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes minimum value register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into minimum value register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT0,
 *          DACB_MINVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteMinimumValueReg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteMinimumValueReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MINVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMinimumValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the minimum value register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MINVAL_FMT0,
 *          DACB_MINVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadMinimumValueReg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadMinimumValueReg(PeripheralBase) ( \
    PeriphReadReg16(DAC_MINVAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMaximumValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes maximum value register.
 * @param PeripheralBase Peripheral base address.
 * @param Value The value is written into maximum value register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT0,
 *          DACB_MAXVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      DAC_PDD_WriteMaximumValueReg(deviceID, 1);
 *      @endcode
 */
#define DAC_PDD_WriteMaximumValueReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(DAC_MAXVAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMaximumValueReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the maximum value register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DACA_MAXVAL_FMT0,
 *          DACB_MAXVAL_FMT0 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = DAC_PDD_ReadMaximumValueReg(deviceID);
 *      @endcode
 */
#define DAC_PDD_ReadMaximumValueReg(PeripheralBase) ( \
    PeriphReadReg16(DAC_MAXVAL_REG(PeripheralBase)) \
  )
#endif  /* #if defined(DAC_PDD_H_) */

/* DAC_PDD.h, eof. */
