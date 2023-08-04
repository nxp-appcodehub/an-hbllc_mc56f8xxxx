/*
  PDD layer implementation for peripheral type ADC_CYCLIC
  Copyright 2013 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(ADC_CYCLIC_PDD_H_)
#define ADC_CYCLIC_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error ADC_CYCLIC PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* ADC */ && \
      !defined(MCU_MC56F82316) /* ADC */ && \
      !defined(MCU_MC56F82723) /* ADC */ && \
      !defined(MCU_MC56F82726) /* ADC */ && \
      !defined(MCU_MC56F82728) /* ADC */ && \
      !defined(MCU_MC56F82733) /* ADC */ && \
      !defined(MCU_MC56F82736) /* ADC */ && \
      !defined(MCU_MC56F82738) /* ADC */ && \
      !defined(MCU_MC56F82743) /* ADC */ && \
      !defined(MCU_MC56F82746) /* ADC */ && \
      !defined(MCU_MC56F82748) /* ADC */
  // Unsupported MCU is active
  #error ADC_CYCLIC PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Sample mask. */
#define ADC_CYCLIC_PDD_SAMPLE0  0x1U             /**< Sample 0 */
#define ADC_CYCLIC_PDD_SAMPLE1  0x2U             /**< Sample 1 */
#define ADC_CYCLIC_PDD_SAMPLE2  0x4U             /**< Sample 2 */
#define ADC_CYCLIC_PDD_SAMPLE3  0x8U             /**< Sample 3 */
#define ADC_CYCLIC_PDD_SAMPLE4  0x10U            /**< Sample 4 */
#define ADC_CYCLIC_PDD_SAMPLE5  0x20U            /**< Sample 5 */
#define ADC_CYCLIC_PDD_SAMPLE6  0x40U            /**< Sample 6 */
#define ADC_CYCLIC_PDD_SAMPLE7  0x80U            /**< Sample 7 */
#define ADC_CYCLIC_PDD_SAMPLE8  0x100U           /**< Sample 8 */
#define ADC_CYCLIC_PDD_SAMPLE9  0x200U           /**< Sample 9 */
#define ADC_CYCLIC_PDD_SAMPLE10 0x400U           /**< Sample 10 */
#define ADC_CYCLIC_PDD_SAMPLE11 0x800U           /**< Sample 11 */
#define ADC_CYCLIC_PDD_SAMPLE12 0x1000U          /**< Sample 12 */
#define ADC_CYCLIC_PDD_SAMPLE13 0x2000U          /**< Sample 13 */
#define ADC_CYCLIC_PDD_SAMPLE14 0x4000U          /**< Sample 14 */
#define ADC_CYCLIC_PDD_SAMPLE15 0x8000U          /**< Sample 15 */
#define ADC_CYCLIC_PDD_SAMPLE16 0x10000U         /**< Sample 16 */
#define ADC_CYCLIC_PDD_SAMPLE17 0x20000U         /**< Sample 17 */
#define ADC_CYCLIC_PDD_SAMPLE18 0x40000U         /**< Sample 18 */
#define ADC_CYCLIC_PDD_SAMPLE19 0x80000U         /**< Sample 19 */

/* Interrupt mask. */
#define ADC_CYCLIC_PDD_END_OF_SCAN_0_INT ADC_CTRL1_EOSIE0_MASK /**< Transfer complete interrupt mask. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_INT ADC_CTRL1_ZCIE_MASK /**< Zero crossing interrupt mask. */
#define ADC_CYCLIC_PDD_LOW_LIMIT_INT     ADC_CTRL1_LLMTIE_MASK /**< Low limit interrupt mask. */
#define ADC_CYCLIC_PDD_HIGH_LIMIT_INT    ADC_CTRL1_HLMTIE_MASK /**< High limit interrupt mask. */

/* End of scan interrupt/status masks. */
#define ADC_CYCLIC_PDD_END_OF_SCAN_0_FLAG    ADC_STAT_EOSI0_MASK /**< End of scan 0 interrupt mask. */
#define ADC_CYCLIC_PDD_END_OF_SCAN_1_FLAG    ADC_STAT_EOSI1_MASK /**< End of scan 1 interrupt mask. */
#define ADC_CYCLIC_PDD_ALL_END_OF_SCAN_FLAGS 0x1800U /**< All End of scan interrupt masks. */

/* High and low limit and zero crossing interrupt/status masks. */
#define ADC_CYCLIC_PDD_HIGH_LIMIT_FLAG    ADC_STAT_HLMTI_MASK /**< High limit interrupt/status mask. */
#define ADC_CYCLIC_PDD_LOW_LIMIT_FLAG     ADC_STAT_LLMTI_MASK /**< Low limit interrupt/status mask. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_FLAG ADC_STAT_ZCI_MASK /**< Zero crossing interrupt/status mask. */

/* Conversion in progress status masks. */
#define ADC_CYCLIC_PDD_CONVERSION_IN_PROGRESS_0_FLAG ADC_STAT_CIP0_MASK /**< Conversion in progress 0 status mask. */
#define ADC_CYCLIC_PDD_CONVERSION_IN_PROGRESS_1_FLAG ADC_STAT_CIP1_MASK /**< Conversion in progress 1 status mask. */

/* Converter A and B power down status masks. */
#define ADC_CYCLIC_PDD_CONVERTER_A_POWER_DOWN_FLAG ADC_PWR_PSTS0_MASK /**< Converter A power down status mask. */
#define ADC_CYCLIC_PDD_CONVERTER_B_POWER_DOWN_FLAG ADC_PWR_PSTS1_MASK /**< Converter B power down status mask. */

/* Channel configure low masks. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA0_1 0x10U /**< ANA0-ANA1 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA2_3 0x20U /**< ANA2-ANA3 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANB0_1 0x40U /**< ANB0-ANB1 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANB2_3 0x80U /**< ANB2-ANB3 channel config. */

/* Channel configure high masks. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA4_5 0x80U /**< ANA4-ANA5 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA6_7 0x100U /**< ANA6-ANA7 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANB4_5 0x200U /**< ANB4-ANB5 channel config. */
#define ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANB6_7 0x400U /**< ANB6-ANB7 channel config. */

/* Channel unipolar differential masks. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANA0_1 0x100U    /**< ANA0-ANA1 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANA2_3 0x200U    /**< ANA2-ANA3 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANB0_1 0x400U    /**< ANB0-ANB1 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANB2_3 0x800U    /**< ANB2-ANB3 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANA4_5 0x1000U   /**< ANA4-ANA5 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANA6_7 0x2000U   /**< ANA6-ANA7 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANB4_5 0x4000U   /**< ANB4-ANB5 channel config. */
#define ADC_CYCLIC_PDD_UNIPOLAR_ANB6_7 0x8000U   /**< ANB6-ANB7 channel config. */

/* Sample analog inputs for Sample 0 to 15. */
#define ADC_CYCLIC_PDD_ANA0 0U                   /**< Single ended: ANA0, Differential: ANA0+, ANA1- */
#define ADC_CYCLIC_PDD_ANA1 0x1U                 /**< Single ended: ANA1, Differential: ANA0+, ANA1- */
#define ADC_CYCLIC_PDD_ANA2 0x2U                 /**< Single ended: ANA2, Differential: ANA2+, ANA3- */
#define ADC_CYCLIC_PDD_ANA3 0x3U                 /**< Single ended: ANA3, Differential: ANA2+, ANA3- */
#define ADC_CYCLIC_PDD_ANA4 0x4U                 /**< Single ended: ANA4, Differential: ANA4+, ANA5- */
#define ADC_CYCLIC_PDD_ANA5 0x5U                 /**< Single ended: ANA5, Differential: ANA4+, ANA5- */
#define ADC_CYCLIC_PDD_ANA6 0x6U                 /**< Single ended: ANA6, Differential: ANA6+, ANA7- */
#define ADC_CYCLIC_PDD_ANA7 0x7U                 /**< Single ended: ANA7, Differential: ANA6+, ANA7- */
#define ADC_CYCLIC_PDD_ANB0 0x8U                 /**< Single ended: ANB0, Differential: ANB0+, ANB1- */
#define ADC_CYCLIC_PDD_ANB1 0x9U                 /**< Single ended: ANB1, Differential: ANB0+, ANB1- */
#define ADC_CYCLIC_PDD_ANB2 0xAU                 /**< Single ended: ANB2, Differential: ANB2+, ANB3- */
#define ADC_CYCLIC_PDD_ANB3 0xBU                 /**< Single ended: ANB3, Differential: ANB2+, ANB3- */
#define ADC_CYCLIC_PDD_ANB4 0xCU                 /**< Single ended: ANB4, Differential: ANB4+, ANB5- */
#define ADC_CYCLIC_PDD_ANB5 0xDU                 /**< Single ended: ANB5, Differential: ANB4+, ANB5- */
#define ADC_CYCLIC_PDD_ANB6 0xEU                 /**< Single ended: ANB6, Differential: ANB6+, ANB7- */
#define ADC_CYCLIC_PDD_ANB7 0xFU                 /**< Single ended: ANB7, Differential: ANB6+, ANB7- */

/* Sample analog inputs for Sample 16-19. */
#define ADC_CYCLIC_PDD_ALTERNATIVE_INPUT_0 0U    /**< Single ended: ADCA temperature sensor. */
#define ADC_CYCLIC_PDD_ALTERNATIVE_INPUT_1 0x1U  /**< Single ended: ADCA analog input for on-chip generated signals. */
#define ADC_CYCLIC_PDD_ALTERNATIVE_INPUT_2 0x2U  /**< Single ended: ADCB temperature sensor. */
#define ADC_CYCLIC_PDD_ALTERNATIVE_INPUT_3 0x3U  /**< Single ended: ADCB analog input for on-chip generated signals. */

/* Power up/down converter masks. */
#define ADC_CYCLIC_PDD_POWER_CONVERTER_A ADC_PWR_PD0_MASK /**< Converter A. */
#define ADC_CYCLIC_PDD_POWER_CONVERTER_B ADC_PWR_PD1_MASK /**< Converter B. */

/* Voltage reference masks. */
#define ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_LOW_A  ADC_CAL_SEL_VREFL_A_MASK /**< Low voltage reference for converter A. */
#define ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_HIGH_A ADC_CAL_SEL_VREFH_A_MASK /**< High voltage reference for converter A. */
#define ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_LOW_B  ADC_CAL_SEL_VREFL_B_MASK /**< Low voltage reference for converter B. */
#define ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_HIGH_B ADC_CAL_SEL_VREFH_B_MASK /**< High voltage reference for converter B. */

/* Constants for ADC scan mode selection. */
#define ADC_CYCLIC_PDD_ONCE_SEQUENTIAL_MODE      0U /**< Once sequential mode. */
#define ADC_CYCLIC_PDD_ONCE_PARALLEL_MODE        0x1U /**< Once parallel mode. */
#define ADC_CYCLIC_PDD_LOOP_SEQUENTIAL_MODE      0x2U /**< Loop sequential mode. */
#define ADC_CYCLIC_PDD_LOOP_PARALLEL_MODE        0x3U /**< Loop parallel mode. */
#define ADC_CYCLIC_PDD_TRIGGERED_SEQUENTIAL_MODE 0x4U /**< Triggered sequential mode. */
#define ADC_CYCLIC_PDD_TRIGGERED_PARALLEL_MODE   0x5U /**< Triggered parallel mode. */

/* Constants for zero crossing mode selection. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_DISABLED             0U /**< Zero Crossing disabled. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_POSITIVE_TO_NEGATIVE 0x1U /**< Zero Crossing enabled for positive to negative sign change. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_NEGATIVE_TO_POSITIVE 0x2U /**< Zero Crossing enabled for negative to positive sign change. */
#define ADC_CYCLIC_PDD_ZERO_CROSSING_ANY_CHANGE           0x3U /**< Zero Crossing enabled for any sign change. */

/* Constants for gain control selection. */
#define ADC_CYCLIC_PDD_AMPLIFICATION_X1 0U       /**< x1 amplification. */
#define ADC_CYCLIC_PDD_AMPLIFICATION_X2 0x1U     /**< x2 amplification. */
#define ADC_CYCLIC_PDD_AMPLIFICATION_X4 0x2U     /**< x4 amplification. */

/* Constants for scan control selection. */
#define ADC_CYCLIC_PDD_CONTINUE 0U               /**< Perform sample immediately after the completion of the current sample. */
#define ADC_CYCLIC_PDD_PAUSE    0x1U             /**< Delay sample until a new sync input occurs. */

/* Constants for DMA source selection. */
#define ADC_CYCLIC_PDD_DMA_SOURCE_EOSI 0U        /**< DMA trigger source is end of scan interrupt. */
#define ADC_CYCLIC_PDD_DMA_SOURCE_RDY  0x40U     /**< DMA trigger source is RDY bits. */


/* ----------------------------------------------------------------------------
   -- EnableDmaRequest
   ---------------------------------------------------------------------------- */

/**
 * @brief DMA enable request for ADC(0).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if DMA requests will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableDmaRequest(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableDmaRequest(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_DMAEN0_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_DMAEN0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetStopMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts ADC(0) into stop mode or normal operation.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if ADC(0) will be stopped (stop mode) or
 *        ready for further conversion (normal mode). This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetStopMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetStopMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_STOP0_MASK)) : ( \
      PeriphClearBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_STOP0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSyncMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables sync mode for ADC(0). In sync mode, a conversion may be
 * initiated by asserting a positive edge on the SYNC0 input.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if triggering by SYNC0 is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableSyncMode(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableSyncMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_SYNC0_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_SYNC0_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- StartConversion
   ---------------------------------------------------------------------------- */

/**
 * @brief Starts conversion in ADC(0).
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_StartConversion(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_StartConversion(PeripheralBase) ( \
    PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_START0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEndOfScanInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable end of scan interrupt.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableEndOfScanInterrupt(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableEndOfScanInterrupt(PeripheralBase) ( \
    PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_EOSIE0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- DisableEndOfScanInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Disable end of scan interrupt.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_DisableEndOfScanInterrupt(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_DisableEndOfScanInterrupt(PeripheralBase) ( \
    PeriphClearBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_EOSIE0_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupts defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupts to enable. Use constants from group "Interrupt
 *        mask.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableInterrupts(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_END_OF_SCAN_0_INT);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableInterrupts(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(ADC_CTRL1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupts defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of interrupts to disable. Use constants from group
 *        "Interrupt mask.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_DisableInterrupts(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_END_OF_SCAN_0_INT);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_DisableInterrupts(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(ADC_CTRL1_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannelConfigureLow
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets single ended or differential mode for low channels specified by
 * mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param SingleEndedMask Mask of channels defining which should be set to
 *        single ended mode. Use constants from group "Channel configure low masks.".
 *        This parameter is 16 bits wide.
 * @param DifferentialMask Mask of channels defining which should be set to
 *        differential mode. Use constants from group "Channel configure low masks.".
 *        This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetChannelConfigureLow(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA0_1, ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA0_1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetChannelConfigureLow(PeripheralBase, SingleEndedMask, DifferentialMask) ( \
    PeriphSetBits16( \
     ADC_CTRL1_REG(PeripheralBase), \
     (uint16_t)(SingleEndedMask), \
     (uint16_t)(DifferentialMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SelectScanMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Select scan mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mode Parameter specifying scan mode. This parameter is of "Constants
 *        for ADC scan mode selection." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectScanMode(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_ONCE_SEQUENTIAL_MODE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectScanMode(PeripheralBase, Mode) ( \
    PeriphSetBits16( \
     ADC_CTRL1_REG(PeripheralBase), \
     ADC_CTRL1_SMODE_MASK, \
     (uint16_t)(Mode) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetScanMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Gets selected scan mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of "Constants for ADC scan mode selection." type. The
 *         value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      uint8_t result = ADC_CYCLIC_PDD_GetScanMode(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetScanMode(PeripheralBase) ( \
    PeriphGetBitMask16(ADC_CTRL1_REG(PeripheralBase), ADC_CTRL1_SMODE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteControl1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CTRL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CTRL1.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadControl1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CTRL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDmaRequest1
   ---------------------------------------------------------------------------- */

/**
 * @brief DMA enable request for ADC1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if DMA requests will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableDmaRequest1(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableDmaRequest1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_DMAEN1_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_DMAEN1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetStopMode1
   ---------------------------------------------------------------------------- */

/**
 * @brief Puts ADC1 into stop mode or normal operation.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if ADC1 will be stopped (stop mode) or
 *        ready for further conversion (normal mode). This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetStopMode1(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetStopMode1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_STOP1_MASK)) : ( \
      PeriphClearBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_STOP1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSyncMode1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables sync mode for ADC1. In sync mode, a conversion may be
 * initiated by asserting a positive edge on the SYNC1 input.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if triggering by SYNC1 is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableSyncMode1(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableSyncMode1(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_SYNC1_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_SYNC1_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- StartConversion1
   ---------------------------------------------------------------------------- */

/**
 * @brief Starts conversion in ADC1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_StartConversion1(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_StartConversion1(PeripheralBase) ( \
    PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_START1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEndOfScanInterrupt1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enable end of scan interrupt for ADC1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableEndOfScanInterrupt1(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableEndOfScanInterrupt1(PeripheralBase) ( \
    PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_EOSIE1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- DisableEndOfScanInterrupt1
   ---------------------------------------------------------------------------- */

/**
 * @brief Disable end of scan interrupt for ADC1.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_DisableEndOfScanInterrupt1(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_DisableEndOfScanInterrupt1(PeripheralBase) ( \
    PeriphClearBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_EOSIE1_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetChannelConfigureHigh
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets single ended or differential mode for high channels specified by
 * mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param SingleEndedMask Mask of channels defining which should be set to
 *        single ended mode. Use constants from group "Channel configure high masks.".
 *        This parameter is 16 bits wide.
 * @param DifferentialMask Mask of channels defining which should be set to
 *        differential mode. Use constants from group "Channel configure high
 *        masks.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetChannelConfigureHigh(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA4_5, ADC_CYCLIC_PDD_CHANNEL_CONFIG_ANA4_5);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetChannelConfigureHigh(PeripheralBase, SingleEndedMask, DifferentialMask) ( \
    PeriphSetBits16( \
     ADC_CTRL2_REG(PeripheralBase), \
     (uint16_t)(SingleEndedMask), \
     (uint16_t)(DifferentialMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSimultaneousMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables Simultaneous mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if simultaneous mode is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableSimultaneousMode(<peripheral>_BASE_PTR,
 *      PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableSimultaneousMode(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_SIMULT_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL2_REG(PeripheralBase), ADC_CTRL2_SIMULT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockDivisor
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets clock divisor.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new clock divisor value. This parameter is
 *        a 6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetClockDivisor(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetClockDivisor(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     ADC_CTRL2_REG(PeripheralBase), \
     ADC_CTRL2_DIV0_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteControl2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CTRL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CTRL2.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadControl2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CTRL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableUnipolarDifferentialMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/Disables differential conversions for either unipolar or fully
 * differential mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param DisableMask Mask of channel pairs which should be set to fully
 *        differential mode. Use constants from group "Channel unipolar differential
 *        masks.". This parameter is 16 bits wide.
 * @param EnableMask Mask of channel pairs which should be set to unipolar
 *        differential mode. Use constants from group "Channel unipolar differential
 *        masks.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableUnipolarDifferentialMode(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_UNIPOLAR_ANA0_1, ADC_CYCLIC_PDD_UNIPOLAR_ANA0_1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableUnipolarDifferentialMode(PeripheralBase, DisableMask, EnableMask) ( \
    PeriphSetBits16( \
     ADC_CTRL3_REG(PeripheralBase), \
     (uint16_t)(DisableMask), \
     (uint16_t)(EnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SelectDMASource
   ---------------------------------------------------------------------------- */

/**
 * @brief Select DMA source to either EOSI or RDY.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Source Parameter specifying DMA source. This parameter is of
 *        "Constants for DMA source selection." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectDMASource(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_DMA_SOURCE_EOSI);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectDMASource(PeripheralBase, Source) ( \
    ((Source) == ADC_PDD_DMA_SOURCE_EOSI) ? ( \
      PeriphClearBitMask16(ADC_CTRL3_REG(PeripheralBase), ADC_CTRL3_DMASRC_MASK)) : ( \
      PeriphSetBitMask16(ADC_CTRL3_REG(PeripheralBase), ADC_CTRL3_DMASRC_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CTRL3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteControl3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteControl3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CTRL3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CTRL3.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadControl3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadControl3Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CTRL3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectZeroCrossingMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects Zero crossing mode for given sample.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Sample index. This parameter is of index type.
 * @param Mode Zero crossing mode select. This parameter is of "Constants for
 *        zero crossing mode selection." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL1,
 *          ADC_ZXCTRL2, ADC_ZXCTRL3 (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectZeroCrossingMode(<peripheral>_BASE_PTR, periphID,
 *      ADC_CYCLIC_PDD_ZERO_CROSSING_DISABLED);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectZeroCrossingMode(PeripheralBase, Index, Mode) ( \
    ((Index) < 8U) ? ( \
      PeriphWriteReg16( \
       ADC_ZXCTRL1_REG(PeripheralBase), \
       (uint16_t)((uint16_t)(Mode) << (uint8_t)((uint8_t)(Index) << 1U)) \
      )) : ( \
       ((Index) < 16U) ? ( \
        PeriphWriteReg16( \
         ADC_ZXCTRL2_REG(PeripheralBase), \
         (uint16_t)((uint16_t)(Mode) << (uint8_t)((uint8_t)((Index) - 8U) << 1U)) \
        )) : ( \
        PeriphWriteReg16( \
         ADC_ZXCTRL3_REG(PeripheralBase), \
         (uint16_t)((uint16_t)(Mode) << (uint8_t)((uint8_t)((Index) - 16U) << 1U)) \
        ))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteZeroCrossingControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Zero crossing control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteZeroCrossingControl1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteZeroCrossingControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_ZXCTRL1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadZeroCrossingControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Zero crossing control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadZeroCrossingControl1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadZeroCrossingControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_ZXCTRL1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteZeroCrossingControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Zero crossing control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteZeroCrossingControl2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteZeroCrossingControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_ZXCTRL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadZeroCrossingControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Zero crossing control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadZeroCrossingControl2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadZeroCrossingControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_ZXCTRL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteZeroCrossingControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Zero crossing control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteZeroCrossingControl3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteZeroCrossingControl3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_ZXCTRL3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadZeroCrossingControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Zero crossing control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_ZXCTRL3.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadZeroCrossingControl3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadZeroCrossingControl3Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_ZXCTRL3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectSampleInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects analog input for given sample.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Sample index. This parameter is of index type.
 * @param Input Analog input select. Use constants from the following groups:
 *        "Interrupt mask.", "Sample analog inputs for Sample 16-19.". This
 *        parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST1, ADC_CLIST2,
 *          ADC_CLIST3, ADC_CLIST4, ADC_CLIST5 (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectSampleInput(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectSampleInput(PeripheralBase, Index, Input) ( \
    ((Index) < 4U) ? ( \
      PeriphSetBits16( \
       ADC_CLIST1_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0xFU << (uint16_t)((uint16_t)(Index) << 2U)), \
       (uint16_t)((uint16_t)(Input) << (uint16_t)((uint16_t)(Index) << 2U)) \
      )) : ( \
       ((Index) < 8U) ? ( \
        PeriphSetBits16( \
         ADC_CLIST2_REG(PeripheralBase), \
         (uint16_t)((uint16_t)0xFU << (uint16_t)((uint16_t)((Index) - 4U) << 2U)), \
         (uint16_t)((uint16_t)(Input) << (uint16_t)((uint16_t)((Index) - 4U) << 2U)) \
        )) : ( \
         ((Index) < 12U) ? ( \
          PeriphSetBits16( \
           ADC_CLIST3_REG(PeripheralBase), \
           (uint16_t)((uint16_t)0xFU << (uint16_t)((uint16_t)((Index) - 8U) << 2U)), \
           (uint16_t)((uint16_t)(Input) << (uint16_t)((uint16_t)((Index) - 8U) << 2U)) \
          )) : ( \
           ((Index) < 16U) ? ( \
            PeriphSetBits16( \
             ADC_CLIST4_REG(PeripheralBase), \
             (uint16_t)((uint16_t)0xFU << (uint16_t)((uint16_t)((Index) - 12U) << 2U)), \
             (uint16_t)((uint16_t)(Input) << (uint16_t)((uint16_t)((Index) - 12U) << 2U)) \
            )) : ( \
            PeriphSetBits16( \
             ADC_CLIST5_REG(PeripheralBase), \
             (uint16_t)((uint16_t)0x3U << (uint8_t)((uint8_t)((Index) - 16U) << 1U)), \
             (uint16_t)((uint16_t)(Input) << (uint8_t)((uint8_t)((Index) - 16U) << 1U)) \
            ))))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelList1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Channel list 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteChannelList1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteChannelList1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CLIST1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelList1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Channel list 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CLIST1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadChannelList1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadChannelList1Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CLIST1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelList2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Channel list 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteChannelList2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteChannelList2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CLIST2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelList2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Channel list 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CLIST2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadChannelList2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadChannelList2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CLIST2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelList3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Channel list 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteChannelList3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteChannelList3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CLIST3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelList3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Channel list 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CLIST3.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadChannelList3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadChannelList3Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CLIST3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelList4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Channel list 4 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST4.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteChannelList4Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteChannelList4Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CLIST4_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelList4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Channel list 4 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CLIST4.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadChannelList4Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadChannelList4Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CLIST4_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteChannelList5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Channel list 5 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CLIST5.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteChannelList5Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteChannelList5Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CLIST5_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadChannelList5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Channel list 5 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CLIST5.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadChannelList5Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadChannelList5Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CLIST5_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSample
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables sample specified by Index parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Sample index. This parameter is of index type.
 * @param State Parameter specifying if Sample will be enabled or disabled. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SDIS, ADC_SDIS2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableSample(<peripheral>_BASE_PTR, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableSample(PeripheralBase, Index, State) ( \
    ((Index) < 16U) ? ( \
      ((State) == PDD_DISABLE) ? ( \
       PeriphSetBitMask16( \
        ADC_SDIS_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)) \
       )) : ( \
       PeriphSetBits16( \
        ADC_SDIS_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)), \
        (uint16_t)((uint16_t)0U << (Index)) \
       )) \
     ) : ( \
      ((State) == PDD_DISABLE) ? ( \
       PeriphSetBitMask16( \
        ADC_SDIS2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)) \
       )) : ( \
       PeriphSetBits16( \
        ADC_SDIS2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)), \
        (uint16_t)((uint16_t)0U << ((Index) - 16U)) \
       )) \
     ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSampleDisableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Sample disable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SDIS.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteSampleDisableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteSampleDisableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SDIS_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSampleDisableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Sample disable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SDIS.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadSampleDisableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadSampleDisableReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SDIS_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSampleDisable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Sample disable 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SDIS2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteSampleDisable2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteSampleDisable2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SDIS2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSampleDisable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Sample disable 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SDIS2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadSampleDisable2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadSampleDisable2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SDIS2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flag bits.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from the following groups to process return value: "End
 *         of scan interrupt/status masks.", "High and low limit and zero
 *         crossing interrupt/status masks.".
 * @remarks The macro accesses the following registers: ADC_STAT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_GetInterruptFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     ADC_STAT_REG(PeripheralBase), \
     (uint16_t)(( \
      ADC_STAT_HLMTI_MASK) | (( \
      ADC_STAT_LLMTI_MASK) | (( \
      ADC_STAT_ZCI_MASK) | (( \
      ADC_STAT_EOSI0_MASK) | ( \
      ADC_STAT_EOSI1_MASK))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearEndOfScanInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears end of scan interrupt flags defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of End of scan interrupt 0/1 to clear. Use constants from
 *        group "End of scan interrupt/status masks.". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_STAT.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_ClearEndOfScanInterruptFlags(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_END_OF_SCAN_0_FLAG);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ClearEndOfScanInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16(ADC_STAT_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt/status flag bits.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from the following groups to process return value:
 *         "Conversion in progress status masks.", "End of scan interrupt/status
 *         masks.", "High and low limit and zero crossing interrupt/status masks.".
 * @remarks The macro accesses the following registers: ADC_STAT.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_GetStatusFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetStatusFlags(PeripheralBase) ( \
    PeriphGetBitMask16( \
     ADC_STAT_REG(PeripheralBase), \
     (uint16_t)(( \
      ADC_STAT_CIP1_MASK) | (( \
      ADC_STAT_CIP0_MASK) | (( \
      ADC_STAT_HLMTI_MASK) | (( \
      ADC_STAT_LLMTI_MASK) | (( \
      ADC_STAT_ZCI_MASK) | (( \
      ADC_STAT_EOSI0_MASK) | ( \
      ADC_STAT_EOSI1_MASK))))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_STAT.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteStatusReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_STAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_STAT.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadStatusReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadStatusReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_STAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSampleReady
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of sample ready flags. Each sample has corresponding bit
 * in the returned value. Bit value 0 means sample not ready or has been read,
 * bit value 1 means sample ready to be read. The bit 0 corresponds with Sample 0,
 * the bit 1 corresponds with Sample 1, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from group "Sample mask." for processing return value.
 * @remarks The macro accesses the following registers: ADC_RDY, ADC_RDY2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result = ADC_CYCLIC_PDD_GetSampleReady(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetSampleReady(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(ADC_RDY_REG(PeripheralBase))) | ( \
     (uint32_t)(( \
      (uint32_t)PeriphGetBitMask16(ADC_RDY2_REG(PeripheralBase), ADC_RDY2_RDY_MASK)) << ( \
      16U)))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadReadyReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Ready register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_RDY.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadReadyReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadReadyReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_RDY_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadReady2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Ready 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_RDY2.
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadReady2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadReady2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_RDY2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetLowLimitStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of low limit flags. Each sample has corresponding bit in
 * the returned value. The bit 0 corresponds with Sample 0, the bit 1 corresponds
 * with Sample 1, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from group "Sample mask." for processing return value.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT,
 *          ADC_LOLIMSTAT2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result =
 *      ADC_CYCLIC_PDD_GetLowLimitStatusFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetLowLimitStatusFlags(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(ADC_LOLIMSTAT_REG(PeripheralBase))) | ( \
     (uint32_t)((uint32_t)PeriphReadReg16(ADC_LOLIMSTAT2_REG(PeripheralBase)) << 16U))) \
  )

/* ----------------------------------------------------------------------------
   -- ClearLowLimitInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears low limit interrupt flags defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of Low limit interrupts to clear. Use constants from group
 *        "Sample mask.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT2,
 *          ADC_LOLIMSTAT (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_ClearLowLimitInterruptFlags(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_SAMPLE0);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ClearLowLimitInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16( \
     ADC_LOLIMSTAT2_REG(PeripheralBase), \
     (uint16_t)((uint32_t)(Mask) >> 16U) \
    ), \
    PeriphWriteReg16(ADC_LOLIMSTAT_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteLowLimitStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Low limit status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteLowLimitStatusReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteLowLimitStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_LOLIMSTAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadLowLimitStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Low limit status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadLowLimitStatusReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadLowLimitStatusReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_LOLIMSTAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteLowLimitStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Low limit status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteLowLimitStatus2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteLowLimitStatus2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_LOLIMSTAT2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadLowLimitStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Low limit status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_LOLIMSTAT2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadLowLimitStatus2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadLowLimitStatus2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_LOLIMSTAT2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetHighLimitStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of high limit flags. Each sample has corresponding bit in
 * the returned value. The bit 0 corresponds with Sample 0, the bit 1
 * corresponds with Sample 1, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from group "Sample mask." for processing return value.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT,
 *          ADC_HILIMSTAT2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result =
 *      ADC_CYCLIC_PDD_GetHighLimitStatusFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetHighLimitStatusFlags(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(ADC_HILIMSTAT_REG(PeripheralBase))) | ( \
     (uint32_t)((uint32_t)PeriphReadReg16(ADC_HILIMSTAT2_REG(PeripheralBase)) << 16U))) \
  )

/* ----------------------------------------------------------------------------
   -- ClearHighLimitInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears high limit interrupt flags defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of High limit interrupts to clear. Use constants from group
 *        "Sample mask.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT2,
 *          ADC_HILIMSTAT (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_ClearHighLimitInterruptFlags(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_SAMPLE0);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ClearHighLimitInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16( \
     ADC_HILIMSTAT2_REG(PeripheralBase), \
     (uint16_t)((uint32_t)(Mask) >> 16U) \
    ), \
    PeriphWriteReg16(ADC_HILIMSTAT_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteHighLimitStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the High limit status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteHighLimitStatusReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteHighLimitStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_HILIMSTAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadHighLimitStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the High limit status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadHighLimitStatusReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadHighLimitStatusReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_HILIMSTAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteHighLimitStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the High limit status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteHighLimitStatus2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteHighLimitStatus2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_HILIMSTAT2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadHighLimitStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the High limit status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_HILIMSTAT2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadHighLimitStatus2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadHighLimitStatus2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_HILIMSTAT2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetZeroCrossingStatusFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns mask of zero crossing flags. Each sample has corresponding bit
 * in the returned value. The bit 0 corresponds with Sample 0, the bit 1
 * corresponds with Sample 1, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from group "Sample mask." for processing return value.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT, ADC_ZXSTAT2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint32_t result =
 *      ADC_CYCLIC_PDD_GetZeroCrossingStatusFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetZeroCrossingStatusFlags(PeripheralBase) ( \
    (uint32_t)(( \
     PeriphReadReg16(ADC_ZXSTAT_REG(PeripheralBase))) | ( \
     (uint32_t)((uint32_t)PeriphReadReg16(ADC_ZXSTAT2_REG(PeripheralBase)) << 16U))) \
  )

/* ----------------------------------------------------------------------------
   -- ClearZeroCrossingInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears zero crossing interrupt flags defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of Zero crossing interrupts to clear. Use constants from
 *        group "Sample mask.". This parameter is 32 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT2, ADC_ZXSTAT
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_ClearZeroCrossingInterruptFlags(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_SAMPLE0);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ClearZeroCrossingInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16( \
     ADC_ZXSTAT2_REG(PeripheralBase), \
     (uint16_t)((uint32_t)(Mask) >> 16U) \
    ), \
    PeriphWriteReg16(ADC_ZXSTAT_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteZeroCrossingStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Zero crossing status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteZeroCrossingStatusReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteZeroCrossingStatusReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_ZXSTAT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadZeroCrossingStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Zero crossing status register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadZeroCrossingStatusReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadZeroCrossingStatusReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_ZXSTAT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteZeroCrossingStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Zero crossing status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteZeroCrossingStatus2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteZeroCrossingStatus2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_ZXSTAT2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadZeroCrossingStatus2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Zero crossing status 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_ZXSTAT2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadZeroCrossingStatus2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadZeroCrossingStatus2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_ZXSTAT2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteResultReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Result n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: RSLT[Index].
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteResultReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteResultReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(ADC_RSLT_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadResultReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Result n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: RSLT[Index].
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadResultReg(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadResultReg(PeripheralBase, Index) ( \
    PeriphReadReg16(ADC_RSLT_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteLowLimitReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Low limit n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: LOLIM[Index].
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteLowLimitReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteLowLimitReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(ADC_LOLIM_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadLowLimitReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Low limit n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: LOLIM[Index].
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadLowLimitReg(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadLowLimitReg(PeripheralBase, Index) ( \
    PeriphReadReg16(ADC_LOLIM_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteHighLimitReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the High limit n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: HILIM[Index].
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteHighLimitReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteHighLimitReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(ADC_HILIM_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadHighLimitReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the High limit n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: HILIM[Index].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadHighLimitReg(<peripheral>_BASE_PTR, periphID);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadHighLimitReg(PeripheralBase, Index) ( \
    PeriphReadReg16(ADC_HILIM_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOffsetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Offset n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OFFST[Index].
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteOffsetReg(<peripheral>_BASE_PTR, periphID, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteOffsetReg(PeripheralBase, Index, Value) ( \
    PeriphWriteReg16(ADC_OFFST_REG(PeripheralBase,(Index)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOffsetReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Offset n register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Register index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OFFST[Index].
 * @par Example:
 *      @code
 *      uint16_t result = ADC_CYCLIC_PDD_ReadOffsetReg(<peripheral>_BASE_PTR,
 *      periphID);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadOffsetReg(PeripheralBase, Index) ( \
    PeriphReadReg16(ADC_OFFST_REG(PeripheralBase,(Index))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAutoStandby
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/Disables auto standby mode. Auto standby mode is ignored if
 * Auto power-down is enabled. When the ADC is idle, autostandby mode selects the
 * standby clock as the ADC clock source and puts the converters into standby
 * current mode. At the start of any scan, the conversion clock is selected as the
 * ADC clock and the ADC will initiate the scan. When the ADC returns to the idle
 * state, the standby clock is again selected and the converters revert to the
 * standby current state.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if Auto standby will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableAutoStandby(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableAutoStandby(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_PWR_REG(PeripheralBase), ADC_PWR_ASB_MASK)) : ( \
      PeriphSetBitMask16(ADC_PWR_REG(PeripheralBase), ADC_PWR_ASB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAutoPowerdown
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/Disables auto powerdown mode. Auto power-down mode powers down
 * converters when not in use for a scan. Auto power-down takes precedence over
 * Auto standby. When a scan is started in Auto power-down mode, a delay
 * specified by Power-up delay item is imposed during which the needed converter(s), if
 * idle, are powered up. The ADC will then initiate a scan equivalent to that done
 * when Auto power-down is not active. When the scan is completed, the
 * converter(s) are powered down again.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param State Parameter specifying if Auto powerdown will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableAutoPowerdown(<peripheral>_BASE_PTR, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableAutoPowerdown(PeripheralBase, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(ADC_PWR_REG(PeripheralBase), ADC_PWR_APD_MASK)) : ( \
      PeriphSetBitMask16(ADC_PWR_REG(PeripheralBase), ADC_PWR_APD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPowerUpDelay
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets power up delay. The power up delay determines the number of ADC
 * clocks required for an ADC converter to exit from Power Down mode or begin
 * conversions after START/SYNC in Auto power down mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new power up delay. This parameter is a
 *        6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetPowerUpDelay(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetPowerUpDelay(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     ADC_PWR_REG(PeripheralBase), \
     ADC_PWR_PUDELAY_MASK, \
     (uint16_t)((uint16_t)(Value) << ADC_PWR_PUDELAY_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePower
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the power for ADC converter A and B.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param DisableMask Mask of converters defining which should be disabled. Use
 *        constants from group "Power up/down converter masks.". This parameter
 *        is 16 bits wide.
 * @param EnableMask Mask of converters defining which should be enabled. Use
 *        constants from group "Power up/down converter masks.". This parameter is
 *        16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnablePower(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_POWER_CONVERTER_A, ADC_CYCLIC_PDD_POWER_CONVERTER_A);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnablePower(PeripheralBase, DisableMask, EnableMask) ( \
    PeriphSetBits16( \
     ADC_PWR_REG(PeripheralBase), \
     (uint16_t)(EnableMask), \
     (uint16_t)(DisableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPowerDownStatus
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns power down status bits.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Use constants from group "Converter A and B power down status masks."
 *         for processing return value.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_GetPowerDownStatus(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_GetPowerDownStatus(PeripheralBase) ( \
    PeriphGetBitMask16( \
     ADC_PWR_REG(PeripheralBase), \
     (uint16_t)(ADC_PWR_PSTS0_MASK | ADC_PWR_PSTS1_MASK) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePowerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Power control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WritePowerControlReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WritePowerControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_PWR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPowerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Power control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_PWR.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadPowerControlReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadPowerControlReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_PWR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockDivisor1
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets clock divisor for converter B. This setting is used only if ADC
 * runs in parallel non-simultaneous scan modes.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new clock divisor value. This parameter is
 *        a 6-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SetClockDivisor1(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SetClockDivisor1(PeripheralBase, Value) ( \
    PeriphSetBits16( \
     ADC_PWR2_REG(PeripheralBase), \
     ADC_PWR2_DIV1_MASK, \
     (uint16_t)((uint16_t)(Value) << ADC_PWR2_DIV1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePowerControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Power control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_PWR2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WritePowerControl2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WritePowerControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_PWR2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPowerControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Power control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_PWR2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadPowerControl2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadPowerControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_PWR2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectVoltageReferenceSources
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects internal reference (VDDA/VSSA) or analog input pin.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param InternalReferencesMask Mask of voltage references for which internal
 *        source should be selected. Use constants from group "Voltage reference
 *        masks.". This parameter is 16 bits wide.
 * @param ExternalReferencesMask Mask of voltage references for which analog
 *        input pin source should be selected. Use constants from group "Voltage
 *        reference masks.". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CAL.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectVoltageReferenceSources(<peripheral>_BASE_PTR,
 *      ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_LOW_A,
 *      ADC_CYCLIC_PDD_VOLTAGE_REFERENCE_LOW_A);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectVoltageReferenceSources(PeripheralBase, InternalReferencesMask, ExternalReferencesMask) ( \
    PeriphSetBits16( \
     ADC_CAL_REG(PeripheralBase), \
     (uint16_t)(InternalReferencesMask), \
     (uint16_t)(ExternalReferencesMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCalibrationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Calibration register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_CAL.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteCalibrationReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteCalibrationReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_CAL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCalibrationReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Calibration register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_CAL.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadCalibrationReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadCalibrationReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_CAL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectGainControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects Gain control for given channel.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Channel index. 0 to 7 stand ANA0 to ANA7, 8 to 15 stand ANB0 to
 *        ANB7, 16 to 19 stand alternative inputs 0 to 3. This parameter is of
 *        index type.
 * @param Mode Gain control select. This parameter is of "Constants for gain
 *        control selection." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_GC1, ADC_GC2,
 *          ADC_GC3 (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectGainControl(<peripheral>_BASE_PTR, periphID,
 *      ADC_CYCLIC_PDD_AMPLIFICATION_X1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectGainControl(PeripheralBase, Index, Mode) ( \
    ((Index) < 8U) ? ( \
      PeriphSetBits16( \
       ADC_GC1_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << (uint16_t)((uint16_t)(Index) << 1U)), \
       (uint16_t)((uint16_t)(Mode) << (uint16_t)((uint16_t)(Index) << 1U)) \
      )) : ( \
       ((Index) < 16U) ? ( \
        PeriphSetBits16( \
         ADC_GC2_REG(PeripheralBase), \
         (uint16_t)((uint16_t)0x3U << (uint16_t)((uint16_t)((Index) - 8U) << 1U)), \
         (uint16_t)((uint16_t)(Mode) << (uint16_t)((uint16_t)((Index) - 8U) << 1U)) \
        )) : ( \
        PeriphSetBits16( \
         ADC_GC3_REG(PeripheralBase), \
         (uint16_t)((uint16_t)0x3U << (uint16_t)((uint16_t)((Index) - 16U) << 1U)), \
         (uint16_t)((uint16_t)(Mode) << (uint16_t)((uint16_t)((Index) - 16U) << 1U)) \
        ))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteGainControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Gain control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_GC1.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteGainControl1Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteGainControl1Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_GC1_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadGainControl1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Gain control 1 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_GC1.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadGainControl1Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadGainControl1Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_GC1_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteGainControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Gain control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_GC2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteGainControl2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteGainControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_GC2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadGainControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Gain control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_GC2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadGainControl2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadGainControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_GC2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteGainControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Gain control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_GC3.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteGainControl3Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteGainControl3Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_GC3_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadGainControl3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Gain control 3 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_GC3.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadGainControl3Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadGainControl3Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_GC3_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SelectScanControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects pause or continue scan mode for given sample.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Sample index. This parameter is of index type.
 * @param State Parameter specifying Scan mode (Pause/Continue). This parameter
 *        is of "Constants for scan control selection." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCTRL, ADC_SCTRL2
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_SelectScanControl(<peripheral>_BASE_PTR, periphID,
 *      ADC_CYCLIC_PDD_CONTINUE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_SelectScanControl(PeripheralBase, Index, State) ( \
    ((Index) < 16U) ? ( \
      ((State) == ADC_PDD_CONTINUE) ? ( \
       PeriphSetBits16( \
        ADC_SCTRL_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)), \
        (uint16_t)((uint16_t)0U << (Index)) \
       )) : ( \
       PeriphSetBitMask16( \
        ADC_SCTRL_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)) \
       )) \
     ) : ( \
      ((State) == ADC_PDD_CONTINUE) ? ( \
       PeriphSetBits16( \
        ADC_SCTRL2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)), \
        (uint16_t)((uint16_t)0U << ((Index) - 16U)) \
       )) : ( \
       PeriphSetBitMask16( \
        ADC_SCTRL2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)) \
       )) \
     ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteScanControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Scan control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCTRL.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteScanControlReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteScanControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SCTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadScanControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Scan control register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SCTRL.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadScanControlReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadScanControlReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SCTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteScanControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Scan control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCTRL2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteScanControl2Reg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteScanControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SCTRL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadScanControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Scan control 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SCTRL2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadScanControl2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadScanControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SCTRL2_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableScanHaltedInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/Disables scan halted interrupt for given sample.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Index Sample index. This parameter is of index type.
 * @param State Parameter specifying if Scan halted interrupt is enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCHLTEN,
 *          ADC_SCHLTEN2 (depending on the peripheral).
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_EnableScanHaltedInterrupt(<peripheral>_BASE_PTR,
 *      periphID, PDD_DISABLE);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_EnableScanHaltedInterrupt(PeripheralBase, Index, State) ( \
    ((Index) < 16U) ? ( \
      ((State) == PDD_DISABLE) ? ( \
       PeriphSetBits16( \
        ADC_SCHLTEN_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)), \
        (uint16_t)((uint16_t)0U << (Index)) \
       )) : ( \
       PeriphSetBitMask16( \
        ADC_SCHLTEN_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << (Index)) \
       )) \
     ) : ( \
      ((State) == PDD_DISABLE) ? ( \
       PeriphSetBits16( \
        ADC_SCHLTEN2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)), \
        (uint16_t)((uint16_t)0U << ((Index) - 16U)) \
       )) : ( \
       PeriphSetBitMask16( \
        ADC_SCHLTEN2_REG(PeripheralBase), \
        (uint16_t)((uint16_t)0x1U << ((Index) - 16U)) \
       )) \
     ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteScanHaltedInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Scan halted interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCHLTEN.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteScanHaltedInterruptEnableReg(<peripheral>_BASE_PTR,
 *      1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteScanHaltedInterruptEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SCHLTEN_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadScanHaltedInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Scan halted interrupt enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SCHLTEN.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadScanHaltedInterruptEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadScanHaltedInterruptEnableReg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SCHLTEN_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteScanHaltedInterruptEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes value to the Scan halted interrupt enable 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: ADC_SCHLTEN2.
 * @par Example:
 *      @code
 *      ADC_CYCLIC_PDD_WriteScanHaltedInterruptEnable2Reg(<peripheral>_BASE_PTR,
 *      1);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_WriteScanHaltedInterruptEnable2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(ADC_SCHLTEN2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadScanHaltedInterruptEnable2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the content of the Scan halted interrupt enable 2 register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: ADC_SCHLTEN2.
 * @par Example:
 *      @code
 *      uint16_t result =
 *      ADC_CYCLIC_PDD_ReadScanHaltedInterruptEnable2Reg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define ADC_CYCLIC_PDD_ReadScanHaltedInterruptEnable2Reg(PeripheralBase) ( \
    PeriphReadReg16(ADC_SCHLTEN2_REG(PeripheralBase)) \
  )
#endif  /* #if defined(ADC_CYCLIC_PDD_H_) */

/* ADC_CYCLIC_PDD.h, eof. */
