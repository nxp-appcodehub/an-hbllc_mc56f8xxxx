/*
  PDD layer implementation for peripheral type GPIO
  Copyright 2013 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(GPIO_PDD_H_)
#define GPIO_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error GPIO PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82316) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82723) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82726) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82728) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82733) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82736) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82738) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82743) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82746) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */ && \
      !defined(MCU_MC56F82748) /* GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF */
  // Unsupported MCU is active
  #error GPIO PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Pin masks */
#define GPIO_PDD_PIN_0  0x1U                     /**< Pin 0 mask */
#define GPIO_PDD_PIN_1  0x2U                     /**< Pin 1 mask */
#define GPIO_PDD_PIN_2  0x4U                     /**< Pin 2 mask */
#define GPIO_PDD_PIN_3  0x8U                     /**< Pin 3 mask */
#define GPIO_PDD_PIN_4  0x10U                    /**< Pin 4 mask */
#define GPIO_PDD_PIN_5  0x20U                    /**< Pin 5 mask */
#define GPIO_PDD_PIN_6  0x40U                    /**< Pin 6 mask */
#define GPIO_PDD_PIN_7  0x80U                    /**< Pin 7 mask */
#define GPIO_PDD_PIN_8  0x100U                   /**< Pin 8 mask */
#define GPIO_PDD_PIN_9  0x200U                   /**< Pin 9 mask */
#define GPIO_PDD_PIN_10 0x400U                   /**< Pin 10 mask */
#define GPIO_PDD_PIN_11 0x800U                   /**< Pin 11 mask */
#define GPIO_PDD_PIN_12 0x1000U                  /**< Pin 12 mask */
#define GPIO_PDD_PIN_13 0x2000U                  /**< Pin 13 mask */
#define GPIO_PDD_PIN_14 0x4000U                  /**< Pin 14 mask */
#define GPIO_PDD_PIN_15 0x8000U                  /**< Pin 15 mask */


/* ----------------------------------------------------------------------------
   -- GetPortDataInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Gets port data input independently of current direction setting nor
 * pin usage. It returns zeros for pins controlled by analog functions.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_RAWDATA,
 *          GPIOB_RAWDATA, GPIOC_RAWDATA, GPIOD_RAWDATA, GPIOE_RAWDATA, GPIOF_RAWDATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_GetPortDataInput(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_GetPortDataInput(PeripheralBase) ( \
    PeriphReadReg16(GPIO_RAWDATA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadRawDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads raw data register in order to get port data input independently
 * of current direction setting nor pin usage. It returns zeros for pins
 * controlled by analog functions.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_RAWDATA,
 *          GPIOB_RAWDATA, GPIOC_RAWDATA, GPIOD_RAWDATA, GPIOE_RAWDATA, GPIOF_RAWDATA
 *          (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadRawDataReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadRawDataReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_RAWDATA_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortData
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets port data value. The value is driven out to the corresponding pin
 * if direction of the pin is set to output.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new data value. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPortData(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_SetPortData(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_DR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetPortData
   ---------------------------------------------------------------------------- */

/**
 * @brief Gets port data value. It returns the last written value for output
 * pins and value on the pin for input pins. It returns zeros for pins controlled by
 * analog functions.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_GetPortData(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_GetPortData(PeripheralBase) ( \
    PeriphReadReg16(GPIO_DR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes data register. The value is driven out to the corresponding pin
 * if direction of the pin is set to output.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new data value. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteDataReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteDataReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_DR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDataReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads data register in order to get port data value. It returns the
 * last written value for output pins and value on the pin for input pins. It
 * returns zeros for pins controlled by analog functions.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadDataReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadDataReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_DR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortDataOutputMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets required bits of port data output. Macro does read-modify-write
 * operation with data register because HW doesn't provide additional
 * functionality for pin setting.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Parameter specifying port pins which should be set. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPortDataOutputMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_SetPortDataOutputMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(GPIO_DR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearPortDataOutputMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears required bits of port data output. Macro does read-modify-write
 * operation with data register because HW doesn't provide additional
 * functionality for pin clearing.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Parameter specifying port pins which should be cleared. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_ClearPortDataOutputMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_ClearPortDataOutputMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(GPIO_DR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- TogglePortDataOutputMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Toggles required bits of port data output. Macro does
 * read-modify-write operation with data register because HW doesn't provide additional
 * functionality for pin toggling.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Parameter specifying port pins which should be inverted. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DR, GPIOB_DR,
 *          GPIOC_DR, GPIOD_DR, GPIOE_DR, GPIOF_DR (depending on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_TogglePortDataOutputMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_TogglePortDataOutputMask(PeripheralBase, Mask) ( \
    PeriphInvertBitMask16(GPIO_DR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortInputDirectionMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets required pins as input.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Parameter specifying port pins which should beset as input. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DDR, GPIOB_DDR,
 *          GPIOC_DDR, GPIOD_DDR, GPIOE_DDR, GPIOF_DDR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPortInputDirectionMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_SetPortInputDirectionMask(PeripheralBase, Mask) ( \
    PeriphClearBitMask16(GPIO_DDR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortOutputDirectionMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets required pins as output.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Parameter specifying port pins which should beset as output. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DDR, GPIOB_DDR,
 *          GPIOC_DDR, GPIOD_DDR, GPIOE_DDR, GPIOF_DDR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPortOutputDirectionMask(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_SetPortOutputDirectionMask(PeripheralBase, Mask) ( \
    PeriphSetBitMask16(GPIO_DDR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPortDirectionMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets direction on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param InputMask Mask of port pins defining which should be set as input. Use
 *        constants from group "Pin masks". This parameter is 16 bits wide.
 * @param OutputMask Mask of port pins defining which should be set as output.
 *        Use constants from group "Pin masks". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DDR, GPIOB_DDR,
 *          GPIOC_DDR, GPIOD_DDR, GPIOE_DDR, GPIOF_DDR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPortDirectionMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetPortDirectionMask(PeripheralBase, InputMask, OutputMask) ( \
    PeriphSetBits16( \
     GPIO_DDR_REG(PeripheralBase), \
     (uint16_t)(InputMask), \
     (uint16_t)(OutputMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePortDirectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes port direction register in order to select input or output
 * direction.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new direction for port pins. Each port pin
 *        has corresponding bit in the value. Bit value 0 means input, bit value
 *        1 means output on corresponding pin. The bit 0 corresponds with the pin
 *        which has index 0 within the port, the bit 1 corresponds with the pin
 *        which has index 1 within the port, etc. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DDR, GPIOB_DDR,
 *          GPIOC_DDR, GPIOD_DDR, GPIOE_DDR, GPIOF_DDR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WritePortDirectionReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WritePortDirectionReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_DDR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPortDirectionReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads port direction register in order to find out direction of every
 * pin in the port. Each port pin has corresponding bit in the returned value.
 * Bit value 0 means input, bit value 1 means output on corresponding pin. The bit
 * 0 corresponds with the pin which has index 0 within the port, the bit 1
 * corresponds with the pin which has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_DDR, GPIOB_DDR,
 *          GPIOC_DDR, GPIOD_DDR, GPIOE_DDR, GPIOF_DDR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadPortDirectionReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadPortDirectionReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_DDR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPullSelectMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets pull-up or pull-down resistors on pins specified by mask
 * parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param PullDownMask Mask of port pins defining which pull-down resistors
 *        should be selected. Use constants from group "Pin masks". This parameter is
 *        16 bits wide.
 * @param PullUpMask Mask of port pins defining which pull-up resistors should
 *        be selected. Use constants from group "Pin masks". This parameter is 16
 *        bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PUS, GPIOB_PUS,
 *          GPIOC_PUS, GPIOD_PUS, GPIOE_PUS, GPIOF_PUS (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPullSelectMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetPullSelectMask(PeripheralBase, PullDownMask, PullUpMask) ( \
    PeriphSetBits16( \
     GPIO_PUS_REG(PeripheralBase), \
     (uint16_t)(PullDownMask), \
     (uint16_t)(PullUpMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePullSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes pull select register in order to select pull-up or pull-down
 * resistors.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new pull select settings for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means
 *        puldown, bit value 1 means pullup on corresponding pin. The bit 0 corresponds
 *        with the pin which has index 0 within the port, the bit 1 corresponds
 *        with the pin which has index 1 within the port, etc. This parameter is
 *        a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PUS, GPIOB_PUS,
 *          GPIOC_PUS, GPIOD_PUS, GPIOE_PUS, GPIOF_PUS (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WritePullSelectReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WritePullSelectReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_PUS_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPullSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads pull select register in order to find out which pull resistor is
 * selected. Each port pin has corresponding bit in the returned value. Bit
 * value 0 means puldown, bit value 1 means pullup on corresponding pin. The bit 0
 * corresponds with the pin which has index 0 within the port, the bit 1
 * corresponds with the pin which has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_PUS, GPIOB_PUS,
 *          GPIOC_PUS, GPIOD_PUS, GPIOE_PUS, GPIOF_PUS (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadPullSelectReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadPullSelectReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_PUS_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePullResistorMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables/Disables pull resistors on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param PullDisableMask Mask of port pins defining which pull resistors should
 *        be disabled. Use constants from group "Pin masks". This parameter is
 *        16 bits wide.
 * @param PullEnableMask Mask of port pins defining which pull resistors should
 *        be enabled. Use constants from group "Pin masks". This parameter is 16
 *        bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PUR, GPIOB_PUR,
 *          GPIOC_PUR, GPIOD_PUR, GPIOE_PUR, GPIOF_PUR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_EnablePullResistorMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_EnablePullResistorMask(PeripheralBase, PullDisableMask, PullEnableMask) ( \
    PeriphSetBits16( \
     GPIO_PUR_REG(PeripheralBase), \
     (uint16_t)(PullDisableMask), \
     (uint16_t)(PullEnableMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePullEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes pull enable register.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new pull enable settings for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means pull
 *        resistor disabled, bit value 1 means pull resistor enabled on
 *        corresponding pin. The bit 0 corresponds with the pin which has index 0 within
 *        the port, the bit 1 corresponds with the pin which has index 1 within the
 *        port, etc. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PUR, GPIOB_PUR,
 *          GPIOC_PUR, GPIOD_PUR, GPIOE_PUR, GPIOF_PUR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WritePullEnableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WritePullEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_PUR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPullEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads pull enable register in order to find out whether pull resistor
 * is enabled. Each port pin has corresponding bit in the returned value. Bit
 * value 0 means pull resistor disabled, bit value 1 means pull resistor enabled on
 * corresponding pin. The bit 0 corresponds with the pin which has index 0 within
 * the port, the bit 1 corresponds with the pin which has index 1 within the
 * port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_PUR, GPIOB_PUR,
 *          GPIOC_PUR, GPIOD_PUR, GPIOE_PUR, GPIOF_PUR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadPullEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadPullEnableReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_PUR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetPushPullMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets push-pull or open drain on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param OpenDrainMask Mask of port pins defining which should be set to open
 *        drain. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @param PushPullMask Mask of port pins defining which should be set to push
 *        pull. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PPMODE,
 *          GPIOB_PPMODE, GPIOC_PPMODE, GPIOD_PPMODE, GPIOE_PPMODE, GPIOF_PPMODE (depending
 *          on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetPushPullMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetPushPullMask(PeripheralBase, OpenDrainMask, PushPullMask) ( \
    PeriphSetBits16( \
     GPIO_PPMODE_REG(PeripheralBase), \
     (uint16_t)(OpenDrainMask), \
     (uint16_t)(PushPullMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePushPullReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Write push-pull register in order to select push-pull or open drain
 * mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new push-pull/open drain settings for port
 *        pins. Each port pin has corresponding bit in the value. Bit value 0
 *        means open drain, bit value 1 means push-pull on corresponding pin. The
 *        bit 0 corresponds with the pin which has index 0 within the port, the bit
 *        1 corresponds with the pin which has index 1 within the port, etc.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PPMODE,
 *          GPIOB_PPMODE, GPIOC_PPMODE, GPIOD_PPMODE, GPIOE_PPMODE, GPIOF_PPMODE (depending
 *          on the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WritePushPullReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WritePushPullReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_PPMODE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPushPullReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads push-pull register in order to get whether push-pull or open
 * drain mode is selected. Each port pin has corresponding bit in the returned
 * value. Bit value 0 means open drain, bit value 1 means push-pull on corresponding
 * pin. The bit 0 corresponds with the pin which has index 0 within the port, the
 * bit 1 corresponds with the pin which has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_PPMODE,
 *          GPIOB_PPMODE, GPIOC_PPMODE, GPIOD_PPMODE, GPIOE_PPMODE, GPIOF_PPMODE (depending
 *          on the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadPushPullReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadPushPullReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_PPMODE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDriveStrengthMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets drive strength on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param LowMask Mask of port pins defining which should be set to low drive
 *        strength. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @param HighMask Mask of port pins defining which should be set to low drive
 *        strength. Use constants from group "Pin masks". This parameter is 16
 *        bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DRIVE,
 *          GPIOB_DRIVE, GPIOC_DRIVE, GPIOD_DRIVE, GPIOE_DRIVE, GPIOF_DRIVE (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetDriveStrengthMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetDriveStrengthMask(PeripheralBase, LowMask, HighMask) ( \
    PeriphSetBits16( \
     GPIO_DRIVE_REG(PeripheralBase), \
     (uint16_t)(LowMask), \
     (uint16_t)(HighMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDriveStrengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes drive strength register in order to select low or high drive
 * strength.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new drive strength settings for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means low
 *        drive strength, bit value 1 means high drive strength on corresponding
 *        pin. The bit 0 corresponds with the pin which has index 0 within the
 *        port, the bit 1 corresponds with the pin which has index 1 within the
 *        port, etc. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_DRIVE,
 *          GPIOB_DRIVE, GPIOC_DRIVE, GPIOD_DRIVE, GPIOE_DRIVE, GPIOF_DRIVE (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteDriveStrengthReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteDriveStrengthReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_DRIVE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDriveStrengthReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads drive strength register in order to find out whether low or high
 * drive strength is selected. Each port pin has corresponding bit in the
 * returned value. Bit value 0 means low drive strength, bit value 1 means high drive
 * strength on corresponding pin. The bit 0 corresponds with the pin which has
 * index 0 within the port, the bit 1 corresponds with the pin which has index 1
 * within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_DRIVE,
 *          GPIOB_DRIVE, GPIOC_DRIVE, GPIOD_DRIVE, GPIOE_DRIVE, GPIOF_DRIVE (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadDriveStrengthReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadDriveStrengthReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_DRIVE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetSlewRateMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets slew rate on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param SlowMask Mask of port pins defining which should be set to slower slew
 *        rate. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @param FastMask Mask of port pins defining which should be set to faster slew
 *        rate. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_SRE, GPIOB_SRE,
 *          GPIOC_SRE, GPIOD_SRE, GPIOE_SRE, GPIOF_SRE (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetSlewRateMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0,
 *      GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetSlewRateMask(PeripheralBase, SlowMask, FastMask) ( \
    PeriphSetBits16( \
     GPIO_SRE_REG(PeripheralBase), \
     (uint16_t)(FastMask), \
     (uint16_t)(SlowMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSlewRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes slew rate register in order to select slower or faster slew
 * rate.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new slew rate settings for port pins. Each
 *        port pin has corresponding bit in the value. Bit value 0 means faster
 *        slew rate, bit value 1 means slower slew rate on corresponding pin. The
 *        bit 0 corresponds with the pin which has index 0 within the port, the
 *        bit 1 corresponds with the pin which has index 1 within the port, etc.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_SRE, GPIOB_SRE,
 *          GPIOC_SRE, GPIOD_SRE, GPIOE_SRE, GPIOF_SRE (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteSlewRateReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteSlewRateReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_SRE_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSlewRateReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads slew rate register in order to find out whether slower or faster
 * slew rate is selected. Each port pin has corresponding bit in the returned
 * value. Bit value 0 means faster slew rate, bit value 1 means slower slew rate on
 * corresponding pin. The bit 0 corresponds with the pin which has index 0
 * within the port, the bit 1 corresponds with the pin which has index 1 within the
 * port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_SRE, GPIOB_SRE,
 *          GPIOC_SRE, GPIOD_SRE, GPIOE_SRE, GPIOF_SRE (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_ReadSlewRateReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadSlewRateReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_SRE_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnablePeripheralModeMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Switches pins specified by mask parameters to peripherals or GPIO.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param GpioMask Mask of port pins defining which should be set to GPIO. Use
 *        constants from group "Pin masks". This parameter is 16 bits wide.
 * @param PeripheralMask Mask of port pins defining which should be set to
 *        peripherals. Use constants from group "Pin masks". This parameter is 16 bits
 *        wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PER, GPIOB_PER,
 *          GPIOC_PER, GPIOD_PER, GPIOE_PER, GPIOF_PER (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_EnablePeripheralModeMask(<peripheral>_BASE_PTR,
 *      GPIO_PDD_PIN_0, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_EnablePeripheralModeMask(PeripheralBase, GpioMask, PeripheralMask) ( \
    PeriphSetBits16( \
     GPIO_PER_REG(PeripheralBase), \
     (uint16_t)(GpioMask), \
     (uint16_t)(PeripheralMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WritePeripheralEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes peripheral enable register in order to select peripheral or
 * GPIO mode.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new peripheral or GPIO modes for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means
 *        GPIO, bit value 1 means peripheral on corresponding pin. The bit 0
 *        corresponds with the pin which has index 0 within the port, the bit 1
 *        corresponds with the pin which has index 1 within the port, etc. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_PER, GPIOB_PER,
 *          GPIOC_PER, GPIOD_PER, GPIOE_PER, GPIOF_PER (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WritePeripheralEnableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WritePeripheralEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_PER_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPeripheralEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads peripheral enable register in order to find out whether
 * peripheral or GPIO slew rate is selected. Each port pin has corresponding bit in the
 * returned value. Bit value 0 means GPIO, bit value 1 means peripheral on
 * corresponding pin. The bit 0 corresponds with the pin which has index 0 within the
 * port, the bit 1 corresponds with the pin which has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_PER, GPIOB_PER,
 *          GPIOC_PER, GPIOD_PER, GPIOE_PER, GPIOF_PER (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadPeripheralEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadPeripheralEnableReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_PER_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- AssertInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Assert software interrupt for pins specified by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param AssertMask Mask of port pins defining software interrupts which should
 *        be asserted. Use constants from group "Pin masks". This parameter is
 *        16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IAR, GPIOB_IAR,
 *          GPIOC_IAR, GPIOD_IAR, GPIOE_IAR, GPIOF_IAR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_AssertInterruptMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_AssertInterruptMask(PeripheralBase, AssertMask) ( \
    PeriphSetBitMask16(GPIO_IAR_REG(PeripheralBase), (uint16_t)(AssertMask)) \
  )

/* ----------------------------------------------------------------------------
   -- DeassertInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Deassert software interrupt for pins specified by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param DeassertMask Mask of port pins defining software interrupts which
 *        should be deasserted. Use constants from group "Pin masks". This parameter
 *        is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IAR, GPIOB_IAR,
 *          GPIOC_IAR, GPIOD_IAR, GPIOE_IAR, GPIOF_IAR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_DeassertInterruptMask(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_DeassertInterruptMask(PeripheralBase, DeassertMask) ( \
    PeriphClearBitMask16(GPIO_IAR_REG(PeripheralBase), (uint16_t)(DeassertMask)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptAssertReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes interrupt assert register in order to assert or deassert
 * software interrupt.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new register value. Each port pin has
 *        corresponding bit in the value. Bit value 0 means deassertion of software
 *        interrupt, bit value 1 means assertion of software interrupt. The bit 0
 *        corresponds with the pin which has index 0 within the port, the bit 1
 *        corresponds with the pin which has index 1 within the port, etc. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IAR, GPIOB_IAR,
 *          GPIOC_IAR, GPIOD_IAR, GPIOE_IAR, GPIOF_IAR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteInterruptAssertReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteInterruptAssertReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_IAR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptAssertReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads interrupt assert register in order to find out whether software
 * interrupt should be asserted or deasserted. Each port pin has corresponding
 * bit in the returned value. Bit value 0 means deassertion of software interrupt,
 * bit value 1 means assertion of software interrupt on corresponding pin. The
 * bit 0 corresponds with the pin which has index 0 within the port, the bit 1
 * corresponds with the pin which has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IAR, GPIOB_IAR,
 *          GPIOC_IAR, GPIOD_IAR, GPIOE_IAR, GPIOF_IAR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadInterruptAssertReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadInterruptAssertReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IAR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables interrupt requests defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param InterruptMask Mask of interrupt requests. Use constants from group
 *        "Pin masks". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IENR, GPIOB_IENR,
 *          GPIOC_IENR, GPIOD_IENR, GPIOE_IENR, GPIOF_IENR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_EnableInterrupts(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_EnableInterrupts(PeripheralBase, InterruptMask) ( \
    PeriphSetBitMask16(GPIO_IENR_REG(PeripheralBase), (uint16_t)(InterruptMask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables interrupt requests defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param InterruptMask Mask of interrupt requests. Use constants from group
 *        "Pin masks". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IENR, GPIOB_IENR,
 *          GPIOC_IENR, GPIOD_IENR, GPIOE_IENR, GPIOF_IENR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_DisableInterrupts(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_DisableInterrupts(PeripheralBase, InterruptMask) ( \
    PeriphClearBitMask16(GPIO_IENR_REG(PeripheralBase), (uint16_t)(InterruptMask)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEnableInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Gets mask of enabled interrupts in order to find out whether pin
 * interrupts are enabled or disabled. Each port pin has corresponding bit in the
 * returned value. Bit value 0 means interrupt disabled, bit value 1 means interrupt
 * enabled on corresponding pin. The bit 0 corresponds with the pin which has
 * index 0 within the port, the bit 1 corresponds with the pin which has index 1
 * within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IENR, GPIOB_IENR,
 *          GPIOC_IENR, GPIOD_IENR, GPIOE_IENR, GPIOF_IENR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_GetEnableInterruptMask(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_GetEnableInterruptMask(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IENR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes interrupt enable register in order to enable or disable pin
 * interrupts.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new enable/disable value for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means
 *        interrupt disabled, bit value 1 means interrupt enabled on corresponding pin.
 *        The bit 0 corresponds with the pin which has index 0 within the port,
 *        the bit 1 corresponds with the pin which has index 1 within the port,
 *        etc. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IENR, GPIOB_IENR,
 *          GPIOC_IENR, GPIOD_IENR, GPIOE_IENR, GPIOF_IENR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteInterruptEnableReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteInterruptEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_IENR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads interrupt enable register in order to find out whether pin
 * interrupts are enabled or disabled. Each port pin has corresponding bit in the
 * returned value. Bit value 0 means interrupt disabled, bit value 1 means interrupt
 * enabled on corresponding pin. The bit 0 corresponds with the pin which has
 * index 0 within the port, the bit 1 corresponds with the pin which has index 1
 * within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IENR, GPIOB_IENR,
 *          GPIOC_IENR, GPIOD_IENR, GPIOE_IENR, GPIOF_IENR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadInterruptEnableReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadInterruptEnableReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IENR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns interrupt flags.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IPR, GPIOB_IPR,
 *          GPIOC_IPR, GPIOD_IPR, GPIOE_IPR, GPIOF_IPR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result = GPIO_PDD_GetInterruptFlags(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_GetInterruptFlags(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IPR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Mask Mask of pins to clearing theirs interrupt flags. Use constants
 *        from group "Pin masks". This parameter is 16 bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IESR, GPIOB_IESR,
 *          GPIOC_IESR, GPIOD_IESR, GPIOE_IESR, GPIOF_IESR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_ClearInterruptFlags(<peripheral>_BASE_PTR, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_ClearInterruptFlags(PeripheralBase, Mask) ( \
    PeriphWriteReg16(GPIO_IESR_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPendingReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads interrupt pending register in order to find out whether pin
 * interrupts occured. Each port pin has corresponding bit in the returned value. Bit
 * value 0 means no interrupt occured, bit value 1 means interrupt occured on
 * corresponding pin. The bit 0 corresponds with the pin which has index 0 within
 * the port, the bit 1 corresponds with the pin which has index 1 within the port,
 * etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IPR, GPIOB_IPR,
 *          GPIOC_IPR, GPIOD_IPR, GPIOE_IPR, GPIOF_IPR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadInterruptPendingReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadInterruptPendingReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IPR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptEdgeSensitiveReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes interrupt edge sensitive register in order to clear pin
 * interrupts.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new enable/disable value for port pins.
 *        Each port pin has corresponding bit in the value. Bit value 0 means no
 *        action, bit value 1 means interrupt pending cleared on corresponding pin.
 *        The bit 0 corresponds with the pin which has index 0 within the port,
 *        the bit 1 corresponds with the pin which has index 1 within the port,
 *        etc. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IESR, GPIOB_IESR,
 *          GPIOC_IESR, GPIOD_IESR, GPIOE_IESR, GPIOF_IESR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteInterruptEdgeSensitiveReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteInterruptEdgeSensitiveReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_IESR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptEdgeSensitiveReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads interrupt edge sensitive register in order to find out whether
 * pin an edge detected. Each port pin has corresponding bit in the returned
 * value. Bit value 0 means no edge detected, bit value 1 means an edge detected on
 * corresponding pin. The bit 0 corresponds with the pin which has index 0 within
 * the port, the bit 1 corresponds with the pin which has index 1 within the port,
 * etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IESR, GPIOB_IESR,
 *          GPIOC_IESR, GPIOD_IESR, GPIOE_IESR, GPIOF_IESR (depending on the
 *          peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadInterruptEdgeSensitiveReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadInterruptEdgeSensitiveReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IESR_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInterruptPolarityMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets rising or falling edges on pins specified by mask parameters.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param FallingMask Mask of port pins defining which should be set to detect
 *        falling edge. Use constants from group "Pin masks". This parameter is 16
 *        bits wide.
 * @param RisingMask Mask of port pins defining which should be set to detect
 *        rising edge. Use constants from group "Pin masks". This parameter is 16
 *        bits wide.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IPOLR,
 *          GPIOB_IPOLR, GPIOC_IPOLR, GPIOD_IPOLR, GPIOE_IPOLR, GPIOF_IPOLR (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_SetInterruptPolarityMask(<peripheral>_BASE_PTR,
 *      GPIO_PDD_PIN_0, GPIO_PDD_PIN_0);
 *      @endcode
 */
#define GPIO_PDD_SetInterruptPolarityMask(PeripheralBase, FallingMask, RisingMask) ( \
    PeriphSetBits16( \
     GPIO_IPOLR_REG(PeripheralBase), \
     (uint16_t)(RisingMask), \
     (uint16_t)(FallingMask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptPolarityReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes interrupt polarity register in order to select falling or
 * rising edge dection.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @param Value Parameter specifying new interrupt polarity settings for port
 *        pins. Each port pin has corresponding bit in the value. Bit value 0 means
 *        rising edge detection, bit value 1 means falling edge detection on
 *        corresponding pin. The bit 0 corresponds with the pin which has index 0
 *        within the port, the bit 1 corresponds with the pin which has index 1
 *        within the port, etc. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: GPIOA_IPOLR,
 *          GPIOB_IPOLR, GPIOC_IPOLR, GPIOD_IPOLR, GPIOE_IPOLR, GPIOF_IPOLR (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      GPIO_PDD_WriteInterruptPolarityReg(<peripheral>_BASE_PTR, 1);
 *      @endcode
 */
#define GPIO_PDD_WriteInterruptPolarityReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(GPIO_IPOLR_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptPolarityReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Reads interrupt polarity register in order to find out whether falling
 * or risingedge detection is selected. Each port pin has corresponding bit in
 * the returned value. Bit value 0 means rising edge detection, bit value 1 means
 * falling edge detection on corresponding pin. The bit 0 corresponds with the
 * pin which has index 0 within the port, the bit 1 corresponds with the pin which
 * has index 1 within the port, etc.
 * @param PeripheralBase Pointer to a peripheral registers structure (peripheral
 *        base address). You can use the constant defined in the registers
 *        definition header file (<peripheral>_BASE_PTR) or the constant defined in
 *        the peripheral initialization component header file
 *        (<component_name>_DEVICE).
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: GPIOA_IPOLR,
 *          GPIOB_IPOLR, GPIOC_IPOLR, GPIOD_IPOLR, GPIOE_IPOLR, GPIOF_IPOLR (depending on
 *          the peripheral).
 * @par Example:
 *      @code
 *      uint16_t result =
 *      GPIO_PDD_ReadInterruptPolarityReg(<peripheral>_BASE_PTR);
 *      @endcode
 */
#define GPIO_PDD_ReadInterruptPolarityReg(PeripheralBase) ( \
    PeriphReadReg16(GPIO_IPOLR_REG(PeripheralBase)) \
  )
#endif  /* #if defined(GPIO_PDD_H_) */

/* GPIO_PDD.h, eof. */
