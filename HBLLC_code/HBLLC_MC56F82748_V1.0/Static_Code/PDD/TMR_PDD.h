/*
  PDD layer implementation for peripheral type TMR
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(TMR_PDD_H_)
#define TMR_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error TMR PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* TMR */ && \
      !defined(MCU_MC56F82316) /* TMR */ && \
      !defined(MCU_MC56F82723) /* TMR */ && \
      !defined(MCU_MC56F82726) /* TMR */ && \
      !defined(MCU_MC56F82728) /* TMR */ && \
      !defined(MCU_MC56F82733) /* TMR */ && \
      !defined(MCU_MC56F82736) /* TMR */ && \
      !defined(MCU_MC56F82738) /* TMR */ && \
      !defined(MCU_MC56F82743) /* TMR */ && \
      !defined(MCU_MC56F82746) /* TMR */ && \
      !defined(MCU_MC56F82748) /* TMR */
  // Unsupported MCU is active
  #error TMR PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* GroupIdx constants */
#define TMR_PDD_GROUP_0 0U                       /**< Group/channel 0 in Quad Timer */
#define TMR_PDD_GROUP_1 0x1U                     /**< Group/channel 1 in Quad Timer */
#define TMR_PDD_GROUP_2 0x2U                     /**< Group/channel 2 in Quad Timer */
#define TMR_PDD_GROUP_3 0x3U                     /**< Group/channel 3 in Quad Timer */

/* ENBL channel masks (for EnableChannel, WriteChannelEnableReg) */
#define TMR_PDD_ENBL_0_MASK 0x1U                 /**< Enable channel 0 mask. */
#define TMR_PDD_ENBL_1_MASK 0x2U                 /**< Enable channel 1 mask. */
#define TMR_PDD_ENBL_2_MASK 0x4U                 /**< Enable channel 2 mask. */
#define TMR_PDD_ENBL_3_MASK 0x8U                 /**< Enable channel 3 mask. */

/* Count mode constants. */
#define TMR_PDD_CM_STOP         0U               /**< No operation */
#define TMR_PDD_CM_RISING_EDGES 0x2000U          /**< Count rising edges of primary source */
#define TMR_PDD_CM_BOTH_EDGES   0x4000U          /**< Count rising and falling edges of primary source */
#define TMR_PDD_CM_GATED        0x6000U          /**< Count rising edges of primary source while secondary input high active */
#define TMR_PDD_CM_QUADRATURE   0x8000U          /**< Quadrature count mode, uses primary and secondary sources */
#define TMR_PDD_CM_SIGNED       0xA000U          /**< Count rising edges of primary source; secondary source specifies direction */
#define TMR_PDD_CM_TRIGGERED    0xC000U          /**< Edge of secondary source triggers primary count until compare */
#define TMR_PDD_CM_CASCADED     0xE000U          /**< Cascaded counter mode (up/down) */

/* Primary count source constants. */
#define TMR_PDD_PCS_COUNTER_0_INPUT   0U         /**< Counter 0 input pin */
#define TMR_PDD_PCS_COUNTER_1_INPUT   0x200U     /**< Counter 1 input pin */
#define TMR_PDD_PCS_COUNTER_2_INPUT   0x400U     /**< Counter 2 input pin */
#define TMR_PDD_PCS_COUNTER_3_INPUT   0x600U     /**< Counter 3 input pin */
#define TMR_PDD_PCS_COUNTER_0_OUTPUT  0x800U     /**< Counter 0 output */
#define TMR_PDD_PCS_COUNTER_1_OUTPUT  0xA00U     /**< Counter 1 output */
#define TMR_PDD_PCS_COUNTER_2_OUTPUT  0xC00U     /**< Counter 2 output */
#define TMR_PDD_PCS_COUNTER_3_OUTPUT  0xE00U     /**< Counter 3 output */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_1   0x1000U    /**< IP bus clock divide by 1 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_2   0x1200U    /**< IP bus clock divide by 2 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_4   0x1400U    /**< IP bus clock divide by 4 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_8   0x1600U    /**< IP bus clock divide by 8 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_16  0x1800U    /**< IP bus clock divide by 16 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_32  0x1A00U    /**< IP bus clock divide by 32 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_64  0x1C00U    /**< IP bus clock divide by 64 prescaler */
#define TMR_PDD_PCS_BUSCLK_DIVIDE_128 0x1E00U    /**< IP bus clock divide by 128 prescaler */

/* Secondary count source constants. */
#define TMR_PDD_SCS_COUNTER_0_INPUT 0U           /**< Counter 0 input pin */
#define TMR_PDD_SCS_COUNTER_1_INPUT 0x80U        /**< Counter 1 input pin */
#define TMR_PDD_SCS_COUNTER_2_INPUT 0x100U       /**< Counter 2 input pin */
#define TMR_PDD_SCS_COUNTER_3_INPUT 0x180U       /**< Counter 3 input pin */

/* Count length constants. */
#define TMR_PDD_LENGTH_ROLLOVER 0U               /**< Counter continues counting past the compare value to the binary roll over. */
#define TMR_PDD_LENGTH_REINIT   0x20U            /**< Counter counts to the compare value and then re-initializes itself. */

/* Count direction constants. */
#define TMR_PDD_DIR_COUNT_UP   0U                /**< Count up. */
#define TMR_PDD_DIR_COUNT_DOWN 0x10U             /**< Count down. */

/* Out mode constants. */
#define TMR_PDD_OM_ASSERTED           0U         /**< Asserted while counter is active */
#define TMR_PDD_OM_CLEAR              0x1U       /**< Clear OFLAG output on successful compare */
#define TMR_PDD_OM_SET                0x2U       /**< Set OFLAG output on successful compare */
#define TMR_PDD_OM_TOGGLE             0x3U       /**< Toggle OFLAG output on successful compare */
#define TMR_PDD_OM_TOGGLE_ALT         0x4U       /**< Toggle OFLAG output using alternating compare registers */
#define TMR_PDD_OM_SET_INPUT_CLEAR    0x5U       /**< Set on compare, cleared on secondary source input edge */
#define TMR_PDD_OM_SET_ROLLOVER_CLEAR 0x6U       /**< Set on compare, cleared on counter rollover */
#define TMR_PDD_OM_GATED              0x7U       /**< Enable gated clock output while counter is active */

/* Input/Output signal polarity constants. */
#define TMR_PDD_POLARITY_NORMAL   0U             /**< Set normal polarity */
#define TMR_PDD_POLARITY_INVERTED 0x1U           /**< Set inverted polarity */

/* Capture mode constants. */
#define TMR_PDD_CAPTURE_DISABLED 0U              /**< Capture function is disabled */
#define TMR_PDD_CAPTURE_RISING   0x40U           /**< Load capture register on rising edge (when IPS=0) or falling edge (when IPS=1) of input */
#define TMR_PDD_CAPTURE_FALLING  0x80U           /**< Load capture register on falling edge (when IPS=0) or rising edge (when IPS=1) of input */
#define TMR_PDD_CAPTURE_BOTH     0xC0U           /**< Load capture register on both edges of input */

/* Debug mode constants. */
#define TMR_PDD_DEBUG_NORMAL     0U              /**< Continue with normal operation during debug mode. */
#define TMR_PDD_DEBUG_HALT       0x4000U         /**< Halt TMR counter during debug mode. */
#define TMR_PDD_DEBUG_FORCE      0x8000U         /**< Force TMR output to logic 0 (prior to consideration of SCTRL[OPS]). */
#define TMR_PDD_DEBUG_HALT_FORCE 0xC000U         /**< Both halt counter and force output to 0 during debug mode. */

/* Triggered count initialization constants. */
#define TMR_PDD_TCI_STOP_COUNTER   0U            /**< Stop counter upon receiving a second trigger event while still counting from the first trigger event. */
#define TMR_PDD_TCI_RELOAD_COUNTER 0x400U        /**< Reload the counter upon receiving a second trigger event while still counting from the first trigger event. */

/* Compare load control constants. */
#define TMR_PDD_CLC_NEVER 0U                     /**< Never preload. */
#define TMR_PDD_CLC_COMP1 0x1U                   /**< Load upon successful compare with the value in COMP1. */
#define TMR_PDD_CLC_COMP2 0x2U                   /**< Load upon successful compare with the value in COMP2 */

/* Filter count constants. */
#define TMR_PDD_FILT_COUNT_3  0U                 /**< Input transition is accepted after 3 consecutive samples. */
#define TMR_PDD_FILT_COUNT_4  0x100U             /**< Input transition is accepted after 4 consecutive samples. */
#define TMR_PDD_FILT_COUNT_5  0x200U             /**< Input transition is accepted after 5 consecutive samples. */
#define TMR_PDD_FILT_COUNT_6  0x300U             /**< Input transition is accepted after 6 consecutive samples. */
#define TMR_PDD_FILT_COUNT_7  0x400U             /**< Input transition is accepted after 7 consecutive samples. */
#define TMR_PDD_FILT_COUNT_8  0x500U             /**< Input transition is accepted after 8 consecutive samples. */
#define TMR_PDD_FILT_COUNT_9  0x600U             /**< Input transition is accepted after 9 consecutive samples. */
#define TMR_PDD_FILT_COUNT_10 0x700U             /**< Input transition is accepted after 10 consecutive samples. */


/* ----------------------------------------------------------------------------
   -- SetCountMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Controls the basic counting and behavior of the counter.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Mode New value of the count mode. This parameter is of "Count mode
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCountMode(deviceID, TMR_PDD_GROUP_0, TMR_PDD_CM_STOP);
 *      @endcode
 */
#define TMR_PDD_SetCountMode(PeripheralBase, GroupIdx, Mode) ( \
    PeriphSetBits16( \
     TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CTRL_CM_MASK, \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEnableDeviceStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of TMR device.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetEnableDeviceStatus(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetEnableDeviceStatus(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16(TMR_CTRL_REG(PeripheralBase,(GroupIdx)), TMR_CTRL_CM_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrimaryCountSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the primary count source.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Source New value of the primary count source. This parameter is of
 *        "Primary count source constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetPrimaryCountSource(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_PCS_COUNTER_0_INPUT);
 *      @endcode
 */
#define TMR_PDD_SetPrimaryCountSource(PeripheralBase, GroupIdx, Source) ( \
    PeriphSetBits16( \
     TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CTRL_PCS_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetSecondaryCountSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the secondary count source.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Source New value of the secondary count source. This parameter is of
 *        "Secondary count source constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetSecondaryCountSource(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_SCS_COUNTER_0_INPUT);
 *      @endcode
 */
#define TMR_PDD_SetSecondaryCountSource(PeripheralBase, GroupIdx, Source) ( \
    PeriphSetBits16( \
     TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CTRL_SCS_MASK, \
     (uint16_t)(Source) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCountOnce
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects one shot counting mode.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of TMR device count once mode. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableCountOnce(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableCountOnce(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CTRL_ONCE_MASK \
      )) : ( \
      PeriphSetBitMask16(TMR_CTRL_REG(PeripheralBase,(GroupIdx)), TMR_CTRL_ONCE_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCountLength
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines when the counter initialization occurs.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the count length. This parameter is of "Count
 *        length constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCountLength(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_LENGTH_ROLLOVER);
 *      @endcode
 */
#define TMR_PDD_SetCountLength(PeripheralBase, GroupIdx, State) ( \
    ((State) == TMR_PDD_LENGTH_ROLLOVER) ? ( \
      PeriphClearBitMask16( \
       TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CTRL_LENGTH_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CTRL_LENGTH_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetCountDirection
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects either the normal count direction up, or the reverse
 * direction, down.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the count direction. This parameter is of "Count
 *        direction constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCountDirection(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_DIR_COUNT_UP);
 *      @endcode
 */
#define TMR_PDD_SetCountDirection(PeripheralBase, GroupIdx, State) ( \
    ((State) == TMR_PDD_DIR_COUNT_UP) ? ( \
      PeriphClearBitMask16(TMR_CTRL_REG(PeripheralBase,(GroupIdx)), TMR_CTRL_DIR_MASK)) : ( \
      PeriphSetBitMask16(TMR_CTRL_REG(PeripheralBase,(GroupIdx)), TMR_CTRL_DIR_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCoChannelInit
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables another counter/timer within the module to force the
 * re-initialization of this counter/timer when it has an active compare event.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of TMR device count once mode. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableCoChannelInit(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableCoChannelInit(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CTRL_COINIT_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CTRL_COINIT_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the mode of operation for the OFLAG output signal.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Mode New value of the out mode. This parameter is of "Out mode
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetOutMode(deviceID, TMR_PDD_GROUP_0, TMR_PDD_OM_ASSERTED);
 *      @endcode
 */
#define TMR_PDD_SetOutMode(PeripheralBase, GroupIdx, Mode) ( \
    PeriphSetBits16( \
     TMR_CTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CTRL_OUTMODE_MASK, \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCmpInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns compare interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetCmpInterruptFlag(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetCmpInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16(TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), TMR_SCTRL_TCF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearCmpInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears compare interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ClearCmpInterruptFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ClearCmpInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_TCF_MASK, \
     (uint16_t)(TMR_SCTRL_IEF_MASK | TMR_SCTRL_TOF_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCmpInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableCmpInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_EnableCmpInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_SCTRL_TCFIE_MASK) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableCmpInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the compare interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_DisableCmpInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_DisableCmpInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_TCFIE_MASK, \
     (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOvfInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns overflow interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetOvfInterruptFlag(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetOvfInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16(TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), TMR_SCTRL_TOF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearOvfInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears overflow interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ClearOvfInterruptFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ClearOvfInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_TOF_MASK, \
     (uint16_t)(TMR_SCTRL_IEF_MASK | TMR_SCTRL_TCF_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOvfInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the overflow interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableOvfInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_EnableOvfInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_SCTRL_TOFIE_MASK) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableOvfInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the overflow interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_DisableOvfInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_DisableOvfInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_TOFIE_MASK, \
     (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns input edge interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetEdgeInterruptFlag(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetEdgeInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16(TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), TMR_SCTRL_IEF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearEdgeInterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears input edge interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ClearEdgeInterruptFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ClearEdgeInterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_IEF_MASK, \
     (uint16_t)(TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEdgeInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the input edge interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableEdgeInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_EnableEdgeInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_SCTRL_IEFIE_MASK) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableEdgeInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the input edge interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_DisableEdgeInterrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_DisableEdgeInterrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_IEFIE_MASK, \
     (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects either the normal input signal polarity, or the inverted input
 * signal polarity.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the input signal polarity. This parameter is of
 *        "Input/Output signal polarity constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetInputPolarity(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_POLARITY_NORMAL);
 *      @endcode
 */
#define TMR_PDD_SetInputPolarity(PeripheralBase, GroupIdx, State) ( \
    ((State) == TMR_PDD_POLARITY_NORMAL) ? ( \
      PeriphSetBits16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_SCTRL_IPS_MASK, \
       (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_SCTRL_IPS_MASK) | ( \
        (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetInputState
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current state of the external input pin selected via the
 * secondary count source after application of IPS and filtering.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetInputState(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetInputState(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_INPUT_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCaptureMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Specifies the operation of the capture register as well as the
 * operation of the input edge flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Mode New value of the capture mode. This parameter is of "Capture mode
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCaptureMode(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define TMR_PDD_SetCaptureMode(PeripheralBase, GroupIdx, Mode) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_CAPTURE_MODE_MASK, \
     (uint16_t)(( \
      (uint16_t)(Mode)) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableMasterMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the compare function's output to be broadcasted to the other
 * counters/timers in the module.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of master mode. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableMasterMode(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableMasterMode(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_SCTRL_MSTR_MASK, \
       (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_SCTRL_MSTR_MASK) | ( \
        (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableExternalFlagForce
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the compare from another counter/timer within the same module
 * to force the state of this counter's OFLAG output signal.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of external OFLAG force. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableExternalFlagForce(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableExternalFlagForce(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_SCTRL_EEOF_MASK, \
       (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_SCTRL_EEOF_MASK) | ( \
        (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetForcedFlagValue
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the value of the OFLAG output signal when software triggers
 * a FORCE command.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New value of the forced OFLAG. This parameter is a 1-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetForcedFlagValue(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_SetForcedFlagValue(PeripheralBase, GroupIdx, Value) ( \
    PeriphSetBits16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_SCTRL_VAL_MASK, \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(Value) << TMR_SCTRL_VAL_SHIFT)) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ForceOutputFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Forces the current value of VAL to be written to the OFLAG output.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ForceOutputFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ForceOutputFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_SCTRL_FORCE_MASK) | ( \
      (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the polarity of the OFLAG output signal.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the output signal polarity. This parameter is of
 *        "Input/Output signal polarity constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetOutputPolarity(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_POLARITY_NORMAL);
 *      @endcode
 */
#define TMR_PDD_SetOutputPolarity(PeripheralBase, GroupIdx, State) ( \
    ((State) == TMR_PDD_POLARITY_NORMAL) ? ( \
      PeriphSetBits16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_SCTRL_OPS_MASK, \
       (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_SCTRL_OPS_MASK) | ( \
        (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief The OFLAG output signal is driven on the external pin.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of output pin. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableOutput(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableOutput(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_SCTRL_OEN_MASK, \
       (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_SCTRL_OEN_MASK) | ( \
        (uint16_t)(TMR_SCTRL_IEF_MASK | (TMR_SCTRL_TOF_MASK | TMR_SCTRL_TCF_MASK)))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetDebugMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Allows the TMR module to perform certain actions in response to the
 * chip entering debug mode.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Mode New value of the degug mode. This parameter is of "Debug mode
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetDebugMode(deviceID, TMR_PDD_GROUP_0, TMR_PDD_DEBUG_NORMAL);
 *      @endcode
 */
#define TMR_PDD_SetDebugMode(PeripheralBase, GroupIdx, Mode) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_DBG_EN_MASK, \
     (uint16_t)((uint16_t)(Mode) | (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFault
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables fault. The selected secondary input acts as a fault signal so
 * that the timer OFLAG is cleared when the secondary input is set.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of fault mode. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableFault(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableFault(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CSCTRL_FAULT_MASK, \
       (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_CSCTRL_FAULT_MASK) | ( \
        (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAlternativeLoad
   ---------------------------------------------------------------------------- */

/**
 * @brief Allows for an alternative method for loading the counter during modulo
 * counting. Counter can be re-initialized with the LOAD or CMPLD2 registers
 * depending on count direction.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of alternative load. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableAlternativeLoad(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableAlternativeLoad(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CSCTRL_ALT_LOAD_MASK, \
       (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(( \
        TMR_CSCTRL_ALT_LOAD_MASK) | ( \
        (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK))) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableReloadOnCapture
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the capture function to cause the counter to be reloaded from
 * the LOAD register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of reload on capture. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableReloadOnCapture(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableReloadOnCapture(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CSCTRL_ROC_MASK, \
       (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(TMR_CSCTRL_ROC_MASK | (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetTriggeredCountInit
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable the counter to be re-initialized when a second trigger occurs
 * while the counter is still counting (during triggered count mode CTRL[CM] =
 * 110).
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the triggered count initialization. This parameter
 *        is of "Triggered count initialization constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetTriggeredCountInit(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_TCI_STOP_COUNTER);
 *      @endcode
 */
#define TMR_PDD_SetTriggeredCountInit(PeripheralBase, GroupIdx, State) ( \
    ((State) == TMR_PDD_TCI_STOP_COUNTER) ? ( \
      PeriphSetBits16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       TMR_CSCTRL_TCI_MASK, \
       (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
       (uint16_t)(TMR_CSCTRL_TCI_MASK | (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetCountingDirection
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the direction of the last count (during quadrature count mode
 * CTRL[CM] = 100).
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetCountingDirection(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetCountingDirection(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_UP_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCmp2Interrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the compare 2 interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableCmp2Interrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_EnableCmp2Interrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_CSCTRL_TCF2EN_MASK) | ( \
      (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableCmp2Interrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the compare 2 interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_DisableCmp2Interrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_DisableCmp2Interrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF2EN_MASK, \
     (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCmp1Interrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the compare 1 interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableCmp1Interrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_EnableCmp1Interrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBitMask16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     (uint16_t)(( \
      TMR_CSCTRL_TCF1EN_MASK) | ( \
      (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- DisableCmp1Interrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables the compare 1 interrupt.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_DisableCmp1Interrupt(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_DisableCmp1Interrupt(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF1EN_MASK, \
     (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCmp2InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns compare 2 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetCmp2InterruptFlag(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetCmp2InterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF2_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearCmp2InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears compare 2 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ClearCmp2InterruptFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ClearCmp2InterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF2_MASK, \
     TMR_CSCTRL_TCF1_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCmp1InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns compare 1 interrupt flag bit.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_GetCmp1InterruptFlag(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_GetCmp1InterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphGetBitMask16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF1_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearCmp1InterruptFlag
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears compare 1 interrupt flag.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_ClearCmp1InterruptFlag(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ClearCmp1InterruptFlag(PeripheralBase, GroupIdx) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_TCF1_MASK, \
     TMR_CSCTRL_TCF2_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCompareLoadControl2
   ---------------------------------------------------------------------------- */

/**
 * @brief Controls when COMP2 is preloaded with the value from CMPLD2.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the compare load control 2. This parameter is of
 *        "Compare load control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCompareLoadControl2(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_CLC_NEVER);
 *      @endcode
 */
#define TMR_PDD_SetCompareLoadControl2(PeripheralBase, GroupIdx, State) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_CL2_MASK, \
     (uint16_t)(( \
      (uint16_t)((uint16_t)(State) << TMR_CSCTRL_CL2_SHIFT)) | ( \
      (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCompareLoadControl1
   ---------------------------------------------------------------------------- */

/**
 * @brief Controls when COMP1 is preloaded with the value from CMPLD1.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the compare load control 1. This parameter is of
 *        "Compare load control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetCompareLoadControl1(deviceID, TMR_PDD_GROUP_0,
 *      TMR_PDD_CLC_NEVER);
 *      @endcode
 */
#define TMR_PDD_SetCompareLoadControl1(PeripheralBase, GroupIdx, State) ( \
    PeriphSetBits16( \
     TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), \
     TMR_CSCTRL_CL1_MASK, \
     (uint16_t)((uint16_t)(State) | (uint16_t)(TMR_CSCTRL_TCF1_MASK | TMR_CSCTRL_TCF2_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFilterCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the number of consecutive samples that must agree prior to the
 * input filter accepting an input transition.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State New value of the input filter sample count. This parameter is of
 *        "Filter count constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FILT[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetFilterCount(deviceID, TMR_PDD_GROUP_0, TMR_PDD_FILT_COUNT_3);
 *      @endcode
 */
#define TMR_PDD_SetFilterCount(PeripheralBase, GroupIdx, State) ( \
    PeriphSetBits16( \
     TMR_FILT_REG(PeripheralBase,(GroupIdx)), \
     TMR_FILT_FILT_CNT_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFilterPeriod
   ---------------------------------------------------------------------------- */

/**
 * @brief Represents the sampling period (in IP bus clock cycles) of the TMR
 * input signals. Each input is sampled multiple times at the rate specified by this
 * field. If FILT_PER is 0x00 (default), then the input filter is bypassed.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New value of the input filter sample period. This parameter is a
 *        8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FILT[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_SetFilterPeriod(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_SetFilterPeriod(PeripheralBase, GroupIdx, Value) ( \
    PeriphSetBits16( \
     TMR_FILT_REG(PeripheralBase,(GroupIdx)), \
     TMR_FILT_FILT_PER_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaPreload2
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables DMA write requests for CMPLD2 whenever data is transferred out
 * of the CMPLD2 register into the CNTR or COMP2 registers.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of comparator preload register 2 DMA enable.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableDmaPreload2(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableDmaPreload2(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       TMR_DMA_REG(PeripheralBase,(GroupIdx)), \
       TMR_DMA_CMPLD2DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_DMA_REG(PeripheralBase,(GroupIdx)), \
       TMR_DMA_CMPLD2DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaPreload1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables DMA write requests for CMPLD1 whenever data is transferred out
 * of the CMPLD1 register into the COMP1 registers.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param State Requested state of comparator preload register 1 DMA enable.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableDmaPreload1(deviceID, TMR_PDD_GROUP_0, PDD_DISABLE);
 *      @endcode
 */
#define TMR_PDD_EnableDmaPreload1(PeripheralBase, GroupIdx, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       TMR_DMA_REG(PeripheralBase,(GroupIdx)), \
       TMR_DMA_CMPLD1DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       TMR_DMA_REG(PeripheralBase,(GroupIdx)), \
       TMR_DMA_CMPLD1DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableChannel
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable the prescaler and counter in each channel. Multiple ENBL bits
 * can be set at the same time to synchronize the start of separate counters.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR channel index. Valid index is only 0! Use constants from
 *        group "GroupIdx constants". This parameter is 16 bits wide.
 * @param Value New value of the ENBL. Use constants from group "ENBL channel
 *        masks (for EnableChannel, WriteChannelEnableReg)". This parameter is 4
 *        bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ENBL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_EnableChannel(deviceID, TMR_PDD_GROUP_0, TMR_PDD_ENBL_0_MASK);
 *      @endcode
 */
#define TMR_PDD_EnableChannel(PeripheralBase, GroupIdx, Value) ( \
    PeriphSetBits16( \
     TMR_ENBL_REG(PeripheralBase,(GroupIdx)), \
     TMR_ENBL_ENBL_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCompare1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the compare 1 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the compare 1 register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COMP1[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCompare1Reg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCompare1Reg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_COMP1_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCompare1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the compare 1 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COMP1[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCompare1Reg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCompare1Reg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_COMP1_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCompare2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the compare 2 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the compare 2 register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: COMP2[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCompare2Reg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCompare2Reg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_COMP2_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCompare2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the compare 2 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: COMP2[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCompare2Reg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCompare2Reg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_COMP2_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the capture register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the capture register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPT[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCaptureReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCaptureReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CAPT_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the capture register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPT[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCaptureReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCaptureReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CAPT_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteLoadReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the load register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the load register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: LOAD[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteLoadReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteLoadReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_LOAD_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadLoadReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the load register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: LOAD[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadLoadReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadLoadReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_LOAD_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteHoldReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the hold register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the hold register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: HOLD[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteHoldReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteHoldReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_HOLD_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadHoldReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the hold register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: HOLD[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadHoldReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadHoldReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_HOLD_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the counter register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the counter register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CNTR[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCounterReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCounterReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CNTR_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the counter register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CNTR[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCounterReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCounterReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CNTR_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the control register. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteControlReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteControlReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CTRL_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadControlReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadControlReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CTRL_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the status and control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the status and control register. This parameter
 *        is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteStatusReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteStatusReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_SCTRL_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the status and control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: SCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadStatusReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadStatusReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_SCTRL_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCmpLoad1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the comparator load 1 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the comparator load 1 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPLD1[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCmpLoad1Reg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCmpLoad1Reg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CMPLD1_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCmpLoad1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the comparator load 1 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPLD1[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCmpLoad1Reg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCmpLoad1Reg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CMPLD1_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCmpLoad2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the comparator load 2 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the comparator load 2 register. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CMPLD2[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCmpLoad2Reg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCmpLoad2Reg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CMPLD2_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCmpLoad2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the comparator load 2 register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CMPLD2[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCmpLoad2Reg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCmpLoad2Reg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CMPLD2_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCmpStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the comparator status and control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the comparator status and control register. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteCmpStatusReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteCmpStatusReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_CSCTRL_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCmpStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the comparator status and control register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CSCTRL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadCmpStatusReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadCmpStatusReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_CSCTRL_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInputFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the input filter register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the input filter register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FILT[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteInputFilterReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteInputFilterReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_FILT_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInputFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the input filter register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FILT[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadInputFilterReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadInputFilterReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_FILT_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDmaEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the DMA enable register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @param Value New content of the DMA enable register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMA[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteDmaEnableReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteDmaEnableReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_DMA_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDmaEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the DMA enable register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR group index. Use constants from group "GroupIdx
 *        constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMA[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadDmaEnableReg(deviceID, TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadDmaEnableReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_DMA_REG(PeripheralBase,(GroupIdx))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the group enable register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR channel index. Valid index is only 0! Use constants from
 *        group "GroupIdx constants". This parameter is 16 bits wide.
 * @param Value New content of the channel enable register. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ENBL[GroupIdx].
 * @par Example:
 *      @code
 *      TMR_PDD_WriteChannelEnableReg(deviceID, TMR_PDD_GROUP_0, 1);
 *      @endcode
 */
#define TMR_PDD_WriteChannelEnableReg(PeripheralBase, GroupIdx, Value) ( \
    PeriphWriteReg16(TMR_ENBL_REG(PeripheralBase,(GroupIdx)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the channel enable register.
 * @param PeripheralBase Peripheral base address.
 * @param GroupIdx TMR channel index. Valid index is only 0! Use constants from
 *        group "GroupIdx constants". This parameter is 16 bits wide.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ENBL[GroupIdx].
 * @par Example:
 *      @code
 *      uint16_t result = TMR_PDD_ReadChannelEnableReg(deviceID,
 *      TMR_PDD_GROUP_0);
 *      @endcode
 */
#define TMR_PDD_ReadChannelEnableReg(PeripheralBase, GroupIdx) ( \
    PeriphReadReg16(TMR_ENBL_REG(PeripheralBase,(GroupIdx))) \
  )
#endif  /* #if defined(TMR_PDD_H_) */

/* TMR_PDD.h, eof. */
