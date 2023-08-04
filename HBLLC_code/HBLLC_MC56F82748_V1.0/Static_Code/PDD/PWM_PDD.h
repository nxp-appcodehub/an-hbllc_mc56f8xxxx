/*
  PDD layer implementation for peripheral type PWM
  Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause

  This file is static and it is generated from API-Factory
*/

#if !defined(PWM_PDD_H_)
#define PWM_PDD_H_

/* ----------------------------------------------------------------------------
   -- Test if supported MCU is active
   ---------------------------------------------------------------------------- */

#if !defined(MCU_ACTIVE)
  // No MCU is active
  #error PWM PDD library: No derivative is active. Place proper #include with PDD memory map before including PDD library.
#elif \
      !defined(MCU_MC56F82313) /* PWMA */ && \
      !defined(MCU_MC56F82316) /* PWMA */ && \
      !defined(MCU_MC56F82723) /* PWMA */ && \
      !defined(MCU_MC56F82726) /* PWMA */ && \
      !defined(MCU_MC56F82728) /* PWMA */ && \
      !defined(MCU_MC56F82733) /* PWMA */ && \
      !defined(MCU_MC56F82736) /* PWMA */ && \
      !defined(MCU_MC56F82738) /* PWMA */ && \
      !defined(MCU_MC56F82743) /* PWMA */ && \
      !defined(MCU_MC56F82746) /* PWMA */ && \
      !defined(MCU_MC56F82748) /* PWMA */
  // Unsupported MCU is active
  #error PWM PDD library: Unsupported derivative is active.
#endif

#include "PDD_Types.h"

/* ----------------------------------------------------------------------------
   -- Method symbol definitions
   ---------------------------------------------------------------------------- */

/* Submodule Interrupts' flags. */
#define PWM_PDD_REGISTERS_UPDATED_FLAG PWM_STS_RUF_MASK /**< This read only flag is set when one of the INIT, VALx, FRACVALx, or CTRL[PRSC] registers has been written resulting in non-coherent data in the set of double buffered registers. Clear this bit by a proper reload sequence consisting of a reload signal while MCTRL[LDOK] = 1. */
#define PWM_PDD_RELOAD_ERROR_FLAG      PWM_STS_REF_MASK /**< This read/write flag is set when a reload cycle occurs while MCTRL[LDOK] is 0 and the double buffered registers are in a non-coherent state (STS[RUF] = 1). Clear this bit by writing a logic one to this location. */
#define PWM_PDD_RELOAD_FLAG            PWM_STS_RF_MASK /**< This read/write flag is set at the beginning of every reload cycle regardless of the state of MCTRL[LDOK]. Clear this bit by writing a logic one to this location when DMAEN[VALDE] is clear (non-DMA mode). This flag can also be cleared by the DMA done signal when DMAEN[VALDE] is set (DMA mode). */
#define PWM_PDD_CAPTUREA0_FLAG         PWM_STS_CFA0_MASK /**< This bit is set when a capture event occurs on the Capture A0 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CA0DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CA0DE] is set (DMA mode). */
#define PWM_PDD_CAPTUREA1_FLAG         PWM_STS_CFA1_MASK /**< This bit is set when a capture event occurs on the Capture A1 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CA1DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CA1DE] is set (DMA mode). */
#define PWM_PDD_CAPTUREB0_FLAG         PWM_STS_CFB0_MASK /**< This bit is set when a capture event occurs on the Capture B0 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CB0DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CB0DE] is set (DMA mode). */
#define PWM_PDD_CAPTUREB1_FLAG         PWM_STS_CFB1_MASK /**< This bit is set when a capture event occurs on the Capture B1 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CB1DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CB1DE] is set (DMA mode). */
#define PWM_PDD_CAPTUREX0_FLAG         PWM_STS_CFX0_MASK /**< This bit is set when a capture event occurs on the Capture X0 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CX0DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CX0DE] is set (DMA mode). */
#define PWM_PDD_CAPTUREX1_FLAG         PWM_STS_CFX1_MASK /**< This bit is set when a capture event occurs on the Capture X1 circuit. This bit is cleared by writing a one to this bit position if DMAEN[CX1DE] is clear (non-DMA mode) or by the DMA done signal if DMAEN[CX1DE] is set (DMA mode). */
#define PWM_PDD_COMPARE0_FLAG          0x1U      /**< This bit is set when the submodule counter value matches the value of one of the VAL0 register. */
#define PWM_PDD_COMPARE1_FLAG          0x2U      /**< This bit is set when the submodule counter value matches the value of one of the VAL1 register. */
#define PWM_PDD_COMPARE2_FLAG          0x4U      /**< This bit is set when the submodule counter value matches the value of one of the VAL2 register. */
#define PWM_PDD_COMPARE3_FLAG          0x8U      /**< This bit is set when the submodule counter value matches the value of one of the VAL3 register. */
#define PWM_PDD_COMPARE4_FLAG          0x10U     /**< This bit is set when the submodule counter value matches the value of one of the VAL4 register. */
#define PWM_PDD_COMPARE5_FLAG          0x20U     /**< This bit is set when the submodule counter value matches the value of one of the VAL5 register. */

/* Submodule Interrupts' mask. */
#define PWM_PDD_RELOAD_ERROR_INTERRUPT PWM_INTEN_REIE_MASK /**< This read/write bit enables the reload error flag, STS[REF], to generate CPU interrupt requests. */
#define PWM_PDD_RELOAD_INTERRUPT       PWM_INTEN_RIE_MASK /**< This read/write bit enables the reload flag, STS[RF], to generate CPU interrupt requests. */
#define PWM_PDD_CAPTUREA0_INTERRUPT    PWM_INTEN_CA0IE_MASK /**< This bit allows the STS[CFA0] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CA0DE]. */
#define PWM_PDD_CAPTUREA1_INTERRUPT    PWM_INTEN_CA1IE_MASK /**< This bit allows the STS[CFA1] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CA1DE]. */
#define PWM_PDD_CAPTUREB0_INTERRUPT    PWM_INTEN_CB0IE_MASK /**< This bit allows the STS[CFB0] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CB0DE]. */
#define PWM_PDD_CAPTUREB1_INTERRUPT    PWM_INTEN_CB1IE_MASK /**< This bit allows the STS[CFB1] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CB1DE]. */
#define PWM_PDD_CAPTUREX0_INTERRUPT    PWM_INTEN_CX0IE_MASK /**< This bit allows the STS[CFX0] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CX0DE]. */
#define PWM_PDD_CAPTUREX1_INTERRUPT    PWM_INTEN_CX1IE_MASK /**< This bit allows the STS[CFX1] flag to create an interrupt request to the CPU. Do not set both this bit and DMAEN[CX1DE]. */
#define PWM_PDD_COMPARE0_INTERRUPT     0x1U      /**< This bit enable the STS[CMPF0] flag to cause a compare interrupt request to the CPU. */
#define PWM_PDD_COMPARE1_INTERRUPT     0x2U      /**< This bit enable the STS[CMPF1] flag to cause a compare interrupt request to the CPU. */
#define PWM_PDD_COMPARE2_INTERRUPT     0x4U      /**< This bit enable the STS[CMPF2] flag to cause a compare interrupt request to the CPU. */
#define PWM_PDD_COMPARE3_INTERRUPT     0x8U      /**< This bit enable the STS[CMPF3] flag to cause a compare interrupt request to the CPU. */
#define PWM_PDD_COMPARE4_INTERRUPT     0x10U     /**< This bit enable the STS[CMPF4] flag to cause a compare interrupt request to the CPU. */
#define PWM_PDD_COMPARE5_INTERRUPT     0x20U     /**< This bit enable the STS[CMPF5] flag to cause a compare interrupt request to the CPU. */

/* Determines channel output masks. */
#define PWM_PDD_SM0_OUTPUTA_MASK 0x100U          /**< Submodule 0 output A mask. */
#define PWM_PDD_SM0_OUTPUTB_MASK 0x10U           /**< Submodule 0 output B mask. */
#define PWM_PDD_SM0_OUTPUTX_MASK 0x1U            /**< Submodule 0 output X mask. */
#define PWM_PDD_SM1_OUTPUTA_MASK 0x200U          /**< Submodule 1 output A mask. */
#define PWM_PDD_SM1_OUTPUTB_MASK 0x20U           /**< Submodule 1 output B mask. */
#define PWM_PDD_SM1_OUTPUTX_MASK 0x2U            /**< Submodule 1 output X mask. */
#define PWM_PDD_SM2_OUTPUTA_MASK 0x400U          /**< Submodule 2 output A mask. */
#define PWM_PDD_SM2_OUTPUTB_MASK 0x40U           /**< Submodule 2 output B mask. */
#define PWM_PDD_SM2_OUTPUTX_MASK 0x4U            /**< Submodule 2 output X mask. */
#define PWM_PDD_SM3_OUTPUTA_MASK 0x800U          /**< Submodule 3 output A mask. */
#define PWM_PDD_SM3_OUTPUTB_MASK 0x80U           /**< Submodule 3 output B mask. */
#define PWM_PDD_SM3_OUTPUTX_MASK 0x8U            /**< Submodule 3 output X mask. */

/* Determines channel update immediately masks. Forces the MASK* bits to be
   immediately updated within each of the submodules without waiting for a FORCE_OUT
   event. */
#define PWM_PDD_SM0UPDATE_MASK 0x1000U           /**< Update immediately submodules 0 mask of the submodules 0 output A/B/X masks. */
#define PWM_PDD_SM1UPDATE_MASK 0x2000U           /**< Update immediately submodules 1 mask of the submodules 1 output A/B/X masks. */
#define PWM_PDD_SM2UPDATE_MASK 0x4000U           /**< Update immediately submodules 2 mask of the submodules 2 output A/B/X masks. */
#define PWM_PDD_SM3UPDATE_MASK 0x8000U           /**< Update immediately submodules 3 mask of the submodules 3 output A/B/X masks. */

/* Determines software controlled channel output masks. */
#define PWM_PDD_SM0_OUTPUT23_MASK PWM_SWCOUT_SM0OUT23_MASK /**< Submodule 0 output 23 mask. */
#define PWM_PDD_SM0_OUTPUT45_MASK PWM_SWCOUT_SM0OUT45_MASK /**< Submodule 0 output 45 mask. */
#define PWM_PDD_SM1_OUTPUT23_MASK PWM_SWCOUT_SM1OUT23_MASK /**< Submodule 1 output 23 mask. */
#define PWM_PDD_SM1_OUTPUT45_MASK PWM_SWCOUT_SM1OUT45_MASK /**< Submodule 1 output 45 mask. */
#define PWM_PDD_SM2_OUTPUT23_MASK PWM_SWCOUT_SM2OUT23_MASK /**< Submodule 2 output 23 mask. */
#define PWM_PDD_SM2_OUTPUT45_MASK PWM_SWCOUT_SM2OUT45_MASK /**< Submodule 2 output 45 mask. */
#define PWM_PDD_SM3_OUTPUT23_MASK PWM_SWCOUT_SM3OUT23_MASK /**< Submodule 3 output 23 mask. */
#define PWM_PDD_SM3_OUTPUT45_MASK PWM_SWCOUT_SM3OUT45_MASK /**< Submodule 3 output 45 mask. */

/* Determines Deadtime Source Select masks. */
#define PWM_PDD_GENERATED_SM0PWM23_MASK 0U       /**< Generated SM0PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM0PWM23_MASK  0x4U     /**< Inverted generated SM0PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM0PWM23_MASK    0x8U     /**< SWCOUT[SM0PWM23] is used by the deadtime logic. */
#define PWM_PDD_PWM0_EXTA_MASK          0xCU     /**< PWM0_EXTA signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM0PWM45_MASK 0U       /**< Generated SM0PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM0PWM45_MASK  0x1U     /**< Inverted generated SM0PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM0PWM45_MASK    0x2U     /**< SWCOUT[SM0PWM45] is used by the deadtime logic. */
#define PWM_PDD_PWM0_EXTB_MASK          0x3U     /**< PWM0_EXTB signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM1PWM23_MASK 0U       /**< Generated SM1PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM1PWM23_MASK  0x40U    /**< Inverted generated SM1PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM1PWM23_MASK    0x80U    /**< SWCOUT[SM1PWM23] is used by the deadtime logic. */
#define PWM_PDD_PWM1_EXTA_MASK          0xC0U    /**< PWM1_EXTA signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM1PWM45_MASK 0U       /**< Generated SM1PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM1PWM45_MASK  0x10U    /**< Inverted generated SM1PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM1PWM45_MASK    0x20U    /**< SWCOUT[SM1PWM45] is used by the deadtime logic. */
#define PWM_PDD_PWM1_EXTB_MASK          0x30U    /**< PWM1_EXTB signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM2PWM23_MASK 0U       /**< Generated SM2PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM2PWM23_MASK  0x400U   /**< Inverted generated SM2PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM2PWM23_MASK    0x800U   /**< SWCOUT[SM2PWM23] is used by the deadtime logic. */
#define PWM_PDD_PWM2_EXTA_MASK          0xC00U   /**< PWM2_EXTA signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM2PWM45_MASK 0U       /**< Generated SM2PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM2PWM45_MASK  0x100U   /**< Inverted generated SM2PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM2PWM45_MASK    0x200U   /**< SWCOUT[SM2PWM45] is used by the deadtime logic. */
#define PWM_PDD_PWM2_EXTB_MASK          0x300U   /**< PWM2_EXTB signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM3PWM23_MASK 0U       /**< Generated SM3PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM3PWM23_MASK  0x4000U  /**< Inverted generated SM3PWM23 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM3PWM23_MASK    0x8000U  /**< SWCOUT[SM3PWM23] is used by the deadtime logic. */
#define PWM_PDD_PWM3_EXTA_MASK          0xC000U  /**< PWM3_EXTA signal is used by the deadtime logic. */
#define PWM_PDD_GENERATED_SM3PWM45_MASK 0U       /**< Generated SM3PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_SM3PWM45_MASK  0x1000U  /**< Inverted generated SM3PWM45 signal is used by the deadtime logic. */
#define PWM_PDD_SWCOUT_SM3PWM45_MASK    0x2000U  /**< SWCOUT[SM3PWM45] is used by the deadtime logic. */
#define PWM_PDD_PWM3_EXTB_MASK          0x3000U  /**< PWM3_EXTB signal is used by the deadtime logic. */

/* Fault Interrupts' mask. */
#define PWM_PDD_FAULT0_INTERRUPT 0x1U            /**< Enables CPU interrupt requests generated by the FAULT0 pins. */
#define PWM_PDD_FAULT1_INTERRUPT 0x2U            /**< Enables CPU interrupt requests generated by the FAULT1 pins. */
#define PWM_PDD_FAULT2_INTERRUPT 0x4U            /**< Enables CPU interrupt requests generated by the FAULT2 pins. */
#define PWM_PDD_FAULT3_INTERRUPT 0x8U            /**< Enables CPU interrupt requests generated by the FAULT3 pins. */

/* Fault Interrupts' flags. */
#define PWM_PDD_FAULT0_FLAG 0x1U                 /**< Masks the Fault0 interrupt flag. */
#define PWM_PDD_FAULT1_FLAG 0x2U                 /**< Masks the Fault1 interrupt flag. */
#define PWM_PDD_FAULT2_FLAG 0x4U                 /**< Masks the Fault2 interrupt flag. */
#define PWM_PDD_FAULT3_FLAG 0x8U                 /**< Masks the Fault3 interrupt flag. */

/* Determines Half Cycle Fault Recovery inputs masks. */
#define PWM_PDD_HALF_FAULT0_MASK 0x1000U         /**< Half Cycle Fault0 Recovery mask. */
#define PWM_PDD_HALF_FAULT1_MASK 0x2000U         /**< Half Cycle Fault1 Recovery mask. */
#define PWM_PDD_HALF_FAULT2_MASK 0x4000U         /**< Half Cycle Fault2 Recovery mask. */
#define PWM_PDD_HALF_FAULT3_MASK 0x8000U         /**< Half Cycle Fault3 Recovery mask. */

/* Determines Half Cycle Fault Recovery inputs values. */
#define PWM_PDD_HALF_FAULT0_DISABLED 0x1000U     /**< PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0) after a fault0 condition. */
#define PWM_PDD_HALF_FAULT1_DISABLED 0x2000U     /**< PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0) after a fault1 condition. */
#define PWM_PDD_HALF_FAULT2_DISABLED 0x4000U     /**< PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0) after a fault2 condition. */
#define PWM_PDD_HALF_FAULT3_DISABLED 0x8000U     /**< PWM outputs are re-enabled at the start of a half cycle (as defined by VAL0) after a fault3 condition. */
#define PWM_PDD_HALF_FAULT0_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a half cycle after a fault0 condition. */
#define PWM_PDD_HALF_FAULT1_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a half cycle after a fault1 condition. */
#define PWM_PDD_HALF_FAULT2_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a half cycle after a fault2 condition. */
#define PWM_PDD_HALF_FAULT3_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a half cycle after a fault3 condition. */

/* Determines Full Cycle Fault Recovery inputs masks. */
#define PWM_PDD_FULL_FAULT0_MASK 0x10U           /**< Full Cycle Fault0 Recovery mask. */
#define PWM_PDD_FULL_FAULT1_MASK 0x20U           /**< Full Cycle Fault1 Recovery mask. */
#define PWM_PDD_FULL_FAULT2_MASK 0x40U           /**< Full Cycle Fault2 Recovery mask. */
#define PWM_PDD_FULL_FAULT3_MASK 0x80U           /**< Full Cycle Fault3 Recovery mask. */

/* Determines Full Cycle Fault Recovery inputs values. */
#define PWM_PDD_FULL_FAULT0_DISABLED 0U          /**< PWM outputs are re-enabled at the start of a full cycle after a fault0 condition. */
#define PWM_PDD_FULL_FAULT1_DISABLED 0U          /**< PWM outputs are re-enabled at the start of a full cycle after a fault1 condition. */
#define PWM_PDD_FULL_FAULT2_DISABLED 0U          /**< PWM outputs are re-enabled at the start of a full cycle after a fault2 condition. */
#define PWM_PDD_FULL_FAULT3_DISABLED 0U          /**< PWM outputs are re-enabled at the start of a full cycle after a fault3 condition. */
#define PWM_PDD_FULL_FAULT0_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a full cycle after a fault0 condition. */
#define PWM_PDD_FULL_FAULT1_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a full cycle after a fault1 condition. */
#define PWM_PDD_FULL_FAULT2_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a full cycle after a fault2 condition. */
#define PWM_PDD_FULL_FAULT3_ENABLED  0U          /**< PWM outputs are not re-enabled at the start of a full cycle after a fault3 condition. */

/* Determines Filtered Fault Pins masks. */
#define PWM_PDD_FAULT0_PIN_MASK 0x100U           /**< Is used to test whether the FilteredFaultPinsMask indicates fault condition on FAULT0 pin. */
#define PWM_PDD_FAULT1_PIN_MASK 0x200U           /**< Is used to test whether the FilteredFaultPinsMask indicates fault condition on FAULT1 pin. */
#define PWM_PDD_FAULT2_PIN_MASK 0x400U           /**< Is used to test whether the FilteredFaultPinsMask indicates fault condition on FAULT2 pin. */
#define PWM_PDD_FAULT3_PIN_MASK 0x800U           /**< Is used to test whether the FilteredFaultPinsMask indicates fault condition on FAULT3 pin. */

/* Determines Master Control masks. */
#define PWM_PDD_SM0COMPLEMENTARY23_MASK 0U       /**< SM0PWM23 is used to generate complementary PWM pair. */
#define PWM_PDD_SM1COMPLEMENTARY23_MASK 0U       /**< SM1PWM23 is used to generate complementary PWM pair. */
#define PWM_PDD_SM2COMPLEMENTARY23_MASK 0U       /**< SM2PWM23 is used to generate complementary PWM pair. */
#define PWM_PDD_SM3COMPLEMENTARY23_MASK 0U       /**< SM3PWM23 is used to generate complementary PWM pair. */
#define PWM_PDD_SM0COMPLEMENTARY45_MASK 0x1000U  /**< SM0PWM45 is used to generate complementary PWM pair. */
#define PWM_PDD_SM1COMPLEMENTARY45_MASK 0x2000U  /**< SM1PWM45 is used to generate complementary PWM pair. */
#define PWM_PDD_SM2COMPLEMENTARY45_MASK 0x4000U  /**< SM2PWM45 is used to generate complementary PWM pair. */
#define PWM_PDD_SM3COMPLEMENTARY45_MASK 0x8000U  /**< SM3PWM45 is used to generate complementary PWM pair. */
#define PWM_PDD_SM0RUN_DISABLED_MASK    0U       /**< Submodule0 PWM generator disabled. */
#define PWM_PDD_SM1RUN_DISABLED_MASK    0U       /**< Submodule1 PWM generator disabled. */
#define PWM_PDD_SM2RUN_DISABLED_MASK    0U       /**< Submodule2 PWM generator disabled. */
#define PWM_PDD_SM3RUN_DISABLED_MASK    0U       /**< Submodule3 PWM generator disabled. */
#define PWM_PDD_SM0RUN_ENABLED_MASK     0x100U   /**< Submodule0 PWM generator enabled. */
#define PWM_PDD_SM1RUN_ENABLED_MASK     0x200U   /**< Submodule1 PWM generator enabled. */
#define PWM_PDD_SM2RUN_ENABLED_MASK     0x400U   /**< Submodule2 PWM generator enabled. */
#define PWM_PDD_SM3RUN_ENABLED_MASK     0x800U   /**< Submodule3 PWM generator enabled. */
#define PWM_PDD_SM0CLEARLOADOK_MASK     0x10U    /**< Clear MCTRL[LDOK] of submodule0. */
#define PWM_PDD_SM1CLEARLOADOK_MASK     0x20U    /**< Clear MCTRL[LDOK] of submodule1. */
#define PWM_PDD_SM2CLEARLOADOK_MASK     0x40U    /**< Clear MCTRL[LDOK] of submodule2. */
#define PWM_PDD_SM3CLEARLOADOK_MASK     0x80U    /**< Clear MCTRL[LDOK] of submodule3. */
#define PWM_PDD_SM0NOTLOAD_MASK         0U       /**< Do not load new values into submodule0. */
#define PWM_PDD_SM1NOTLOAD_MASK         0U       /**< Do not load new values into submodule1. */
#define PWM_PDD_SM2NOTLOAD_MASK         0U       /**< Do not load new values into submodule2. */
#define PWM_PDD_SM3NOTLOAD_MASK         0U       /**< Do not load new values into submodule3. */
#define PWM_PDD_SM0LOADOK_MASK          0x1U     /**< Load prescaler, submodule0, and PWM values. */
#define PWM_PDD_SM1LOADOK_MASK          0x2U     /**< Load prescaler, submodule1, and PWM values. */
#define PWM_PDD_SM2LOADOK_MASK          0x4U     /**< Load prescaler, submodule2, and PWM values. */
#define PWM_PDD_SM3LOADOK_MASK          0x8U     /**< Load prescaler, submodule3, and PWM values. */

/* Independent or complementary pair operation constants. */
#define PWM_PDD_COMPLEMENTARY 0U                 /**< PWM_A and PWM_B form a complementary PWM pair. */
#define PWM_PDD_INDEPENDENT   PWM_CTRL2_INDEP_MASK /**< PWM_A and PWM_B outputs are independent PWMs. */

/* Determines initial values */
#define PWM_PDD_LOW  0U                          /**< Determines the low initial value. */
#define PWM_PDD_HIGH 0x1U                        /**< Determines the high initial value. */

/* Determines signals which goes to the initialization control select. */
#define PWM_PDD_IS_LOCAL_SYNC    0U              /**< Local sync (PWMX) causes initialization. */
#define PWM_PDD_IS_MASTER_RELOAD 0x100U          /**< Master reload from submodule 0 causes initialization. This setting should not be used in submodule 0 as it will force the INIT signal to logic 0. */
#define PWM_PDD_IS_MASTER_SYNC   0x200U          /**< Master sync from submodule 0 causes initialization. This setting should not be used in submodule 0 as it will force the INIT signal to logic 0. */
#define PWM_PDD_IS_EXTERNAL_SYNC 0x300U          /**< EXT_SYNC signal causes initialization. */

/* Determines signals which act as force output source. */
#define PWM_PDD_FO_LOCAL_FORCE    0U             /**< The local force signal, CR2[FORCE], from this submodule is used to force updates. */
#define PWM_PDD_FO_MASTER_FORCE   0x8U           /**< The master force signal from submodule 0 is used to force updates. This setting should not be used in submodule 0 as it will hold the FORCE OUTPUT signal to logic 0. */
#define PWM_PDD_FO_LOCAL_RELOAD   0x10U          /**< The local reload signal from this submodule is used to force updates. */
#define PWM_PDD_FO_MASTER_RELOAD  0x18U          /**< The master reload signal from submodule 0 is used to force updates. This setting should not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0. */
#define PWM_PDD_FO_LOCAL_SYNC     0x20U          /**< The local sync signal from this submodule is used to force updates. */
#define PWM_PDD_FO_MASTER_SYNC    0x28U          /**< The master sync signal from submodule 0 is used to force updates. This setting should not be used in submodule0 as it will hold the FORCE OUTPUT signal to logic 0. */
#define PWM_PDD_FO_EXTERNAL_FORCE 0x30U          /**< The external force signal, EXT_FORCE, from outside the PWM module causes updates. */

/* Determines signals which goes to the reload source select. */
#define PWM_PDD_RS_LOCAL_RELOAD  0U              /**< The local RELOAD signal is used to reload registers. */
#define PWM_PDD_RS_MASTER_RELOAD 0x4U            /**< The master RELOAD signal (from submodule 0) is used to reload registers. This setting should not be used in submodule 0 as it will force the RELOAD signal to logic 0. */

/* Determines signals which goes to the clock source select. */
#define PWM_PDD_BUS_CLK 0U                       /**< The IPBus clock is used as the clock for the local prescaler and counter. */
#define PWM_PDD_EXT_CLK 0x100U                   /**< EXT_CLK is used as the clock for the local prescaler and counter. */
#define PWM_PDD_AUX_CLK 0x200U                   /**< Submodule 0's clock (AUX_CLK) is used as the source clock for the local prescaler and counter. This setting should not be used in submodule 0 as it will force the clock to logic 0. */

/* Determines prescalers division ratios. */
#define PWM_PDD_FCLK_DIV1   0U                   /**< Clock source is divided by 1. */
#define PWM_PDD_FCLK_DIV2   0x10U                /**< Clock source is divided by 2. */
#define PWM_PDD_FCLK_DIV4   0x20U                /**< Clock source is divided by 4. */
#define PWM_PDD_FCLK_DIV8   0x30U                /**< Clock source is divided by 8. */
#define PWM_PDD_FCLK_DIV16  0x40U                /**< Clock source is divided by 16. */
#define PWM_PDD_FCLK_DIV32  0x50U                /**< Clock source is divided by 32. */
#define PWM_PDD_FCLK_DIV64  0x60U                /**< Clock source is divided by 64. */
#define PWM_PDD_FCLK_DIV128 0x70U                /**< Clock source is divided by 128. */

/* Determines the load mode select constants. */
#define PWM_PDD_LDOK_RELOAD 0U                   /**< Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set. */
#define PWM_PDD_LDOK_ONLY   0x4U                 /**< Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set. */

/* Determines possible states during fault conditions and STOP mode. */
#define PWM_PDD_LOGIC0    0U                     /**< Output is forced to logic 0 state prior to consideration of output polarity control. */
#define PWM_PDD_LOGIC1    0x1U                   /**< Output is forced to logic 1 state prior to consideration of output polarity control. */
#define PWM_PDD_TRISTATED 0x2U                   /**< Output is tristated. */

/* Determines the Output Trigger 0 Sources constants. */
#define PWM_PDD_PWM_OUT_TRIG0 0U                 /**< Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port. */
#define PWM_PDD_PWMA_OUTPUT   0x8000U            /**< Route the PWMA output to the PWM_OUT_TRIG0 port. */

/* Determines the Output Trigger 1 Sources constants. */
#define PWM_PDD_PWM_OUT_TRIG1 0U                 /**< Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port. */
#define PWM_PDD_PWMB_OUTPUT   0x4000U            /**< Route the PWMB output to the PWM_OUT_TRIG1 port. */

/* Determines the FIFO Watermark AND Control constants. */
#define PWM_PDD_WATERMARKS_ORED  0U              /**< Selected FIFO watermarks are ORed together. */
#define PWM_PDD_WATERMARKS_ANDED 0x100U          /**< Selected FIFO watermarks are ANDed together. */

/* Determines the capture DMA enable source select constants. */
#define PWM_PDD_DMA_DISABLED     0U              /**< Read DMA requests disabled. */
#define PWM_PDD_FIFO_WATERMARK   0x40U           /**< Exceeding a FIFO watermark sets the DMA read request. */
#define PWM_PDD_LOCAL_SYNC_DMA   0x80U           /**< A local sync (VAL1 matches counter) sets the read DMA request. */
#define PWM_PDD_LOCAL_RELOAD_DMA 0xC0U           /**< A local reload (STS[RF] being set) sets the read DMA request. */

/* Determines the Input Select constants. */
#define PWM_PDD_RAW_INPUT           0U           /**< Raw PWM_A/B/X input signal selected as source. */
#define PWM_PDD_EDGE_COUNTER_OUTPUT 0x40U        /**< Output of edge counter/compare selected as source. */

/* Determines the Edge constants. */
#define PWM_PDD_CAPTURE_DISABLED 0U              /**< Capture event disabled. */
#define PWM_PDD_CAPTURE_FALLING  0x1U            /**< Capture falling edges. */
#define PWM_PDD_CAPTURE_RISING   0x2U            /**< Capture rising edges. */
#define PWM_PDD_CAPTURE_ANY_EDGE 0x3U            /**< Capture any edge. */

/* Determines the One Shot Mode constants. */
#define PWM_PDD_FREE_RUNNING 0U                  /**< Free running mode. If both capture circuits are enabled, then capture circuit 0 is armed first after CAPTCTRLn[ARMn] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is armed. After capture circuit 1 performs a capture, it is disarmed and capture circuit 0 is re-armed. The process continues indefinitely. If only one of the capture circuits is enabled, then captures continue indefinitely on the enabled capture circuit. */
#define PWM_PDD_ONE_SHOT     0x2U                /**< One shot mode.If both capture circuits are enabled, then capture circuit 0 is armed first after CAPTCTRLn[ARMn] is set. Once a capture occurs, capture circuit 0 is disarmed and capture circuit 1 is armed. After capture circuit 1 performs a capture, it is disarmed and CAPTCTRLn[ARMn] is cleared. No further captures will be performed until CAPTCTRLn[ARMn] is set again. If only one of the capture circuits is enabled, then a single capture will occur on the enabled capture circuit and CAPTCTRLn[ARMn] is then cleared. */

/* Determines the load mode select constants. */
#define PWM_PDD_LOGIC_ZERO 0U                    /**< Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set. */
#define PWM_PDD_LOGIC_ONE  0x1U                  /**< Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set. */

/* Determines the deadtime logic inputs constants. */
#define PWM_PDD_GENERATED_SIGNAL   0U            /**< Generated pwm signal is used by the deadtime logic. */
#define PWM_PDD_INVERTED_GENERATED 0x1U          /**< Inverted generated pwm signal is used by the deadtime logic. */
#define PWM_PDD_SOFTWARE_CONTROLED 0x2U          /**< SWCOUT is used by the deadtime logic. */
#define PWM_PDD_EXTN_SIGNAL        0x3U          /**< PWM_EXTn signal is used by the deadtime logic. */

/* Determines the Current Polarity constants. */
#define PWM_PDD_COMPLEMENTARY_PWM23 0U           /**< Buffered registers of this submodule are loaded and take effect at the next PWM reload if MCTRL[LDOK] is set. */
#define PWM_PDD_COMPLEMENTARY_PWM45 0x1U         /**< Buffered registers of this submodule are loaded and take effect immediately upon MCTRL[LDOK] being set. */

/* Fault Clearing constants. */
#define PWM_PDD_MANUAL_CLEARING    0U            /**< Manual fault clearing. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the start of a half cycle or full cycle depending the state of FSTS[FFULL]. This is further controlled by FCTRL[FSAFE]. */
#define PWM_PDD_AUTOMATIC_CLEARING 0x1U          /**< Automatic fault clearing. PWM outputs disabled by this fault are enabled when FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the state of FSTS[FFULL] without regard to the state of FSTS[FFLAGx]. */

/* Fault Safety Mode constants. */
#define PWM_PDD_NORMAL_MODE 0U                   /**< Normal mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear at the start of a half cycle or full cycle depending on the state of FSTS[FFULL] without regard to the state of FSTS[FFPINx]. The PWM outputs disabled by this fault input will not be re-enabled until the actual FAULTx input signal de-asserts since the fault input will combinationally disable the PWM outputs (as programmed in DISMAPn). */
#define PWM_PDD_SAFE_MODE   0x1U                 /**< Safe mode. PWM outputs disabled by this fault are not enabled until FSTS[FFLAGx] is clear and FSTS[FFPINx] is clear at the start of a half cycle or full cycle depending on the state of FSTS[FFULL]. */

/* Independent or complementary pair operation constants. */
#define PWM_PDD_NOT_MONITOR_NOT_LOCKED 0U        /**< Not locked. Do not monitor PLL operation. Resetting of the fractional delay block in case of PLL losing lock will be controlled by software.. */
#define PWM_PDD_MONITOR_PLL_NOT_LOCKED 0x1U      /**< Not locked. Monitor PLL operation to automatically disable the fractional delay block when the PLL encounters problems. */
#define PWM_PDD_NOT_MONITOR_LOCKED     0x2U      /**< Locked. Do not monitor PLL operation. Resetting of the fractional delay block in case of PLL losing lock will be controlled by software. These bits are write protected until the next reset. */
#define PWM_PDD_MONITOR_PLL_LOCKED     PWM_MCTRL2_MONPLL_MASK /**< Locked. Monitor PLL operation to automatically disable the fractional delay block when the PLL encounters problems. These bits are write protected until the next reset. */


/* ----------------------------------------------------------------------------
   -- SetInitialCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the initial count value for the PWM in PWM clock periods. This is
 * the value loaded into the submodule counter when local sync, master sync, or
 * master reload is asserted.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Count the initial counter value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INIT[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInitialCount(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetInitialCount(PeripheralBase, SmIndex, Count) ( \
    PeriphWriteReg16(PWM_INIT_REG(PeripheralBase,(SmIndex)), (uint16_t)(Count)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitialCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns initial count value for the PWM in PWM clock periods stored in
 * buffer. This is the value loaded into the submodule counter when local sync,
 * master sync, or master reload is asserted.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INIT[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInitialCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInitialCount(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_INIT_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetCountState
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the state of the signed 16-bit submodule counter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CNT[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCountState(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCountState(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CNT_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDebug
   ---------------------------------------------------------------------------- */

/**
 * @brief When set to enable, the PWM will continue to run while the chip is in
 * debug mode. If the device enters debug mode and this bit is zero, then the PWM
 * outputs will be disabled until debug mode is exited. At that point the PWM
 * pins will resume operation as programmed in the PWM registers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter specifying if the debug mode will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableDebug(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableDebug(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_DBGEN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_DBGEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetDebugEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of debug mode bit. When set to one, the PWM will
 * continue to run while the chip is in debug mode. If the device enters debug
 * mode and this bit is zero, then the PWM outputs will be disabled until debug
 * mode is exited. At that point the PWM pins will resume operation as programmed
 * in the PWM registers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetDebugEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetDebugEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_DBGEN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableWait
   ---------------------------------------------------------------------------- */

/**
 * @brief When set to enable, the PWM will continue to run while the chip is in
 * WAIT mode. In this mode, the peripheral clock continues to run but the CPU
 * clock does not. If the device enters WAIT mode and this bit is zero, then the PWM
 * outputs will be disabled until WAIT mode is exited. At that point the PWM
 * pins will resume operation as programmed in the PWM registers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the debug mode will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableWait(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableWait(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_WAITEN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_WAITEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetWaitEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of WAIT mode bit. When set to one, the PWM will
 * continue to run while the chip is in WAIT mode. In this mode, the peripheral
 * clock continues to run but the CPU clock does not. If the device enters WAIT
 * mode and this bit is zero, then the PWM outputs will be disabled until WAIT mode
 * is exited. At that point the PWM pins will resume operation as programmed in
 * the PWM registers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetWaitEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetWaitEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_WAITEN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetPairOperation
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines if the PWM_A and PWM_B channels will be independent PWMs or
 * a complementary PWM pair.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if PWM_A and PWM_B outputs are
 *        independent or complementary. This parameter is of "Independent or complementary
 *        pair operation constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetPairOperation(deviceID, periphID, PWM_PDD_COMPLEMENTARY);
 *      @endcode
 */
#define PWM_PDD_SetPairOperation(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_COMPLEMENTARY) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_INDEP_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_INDEP_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetPairOperation
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of pair operation function.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetPairOperation(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetPairOperation(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_INDEP_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetInitialValueA
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the initial value for PWM_A and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the initial value. This parameter is of "Determines
 *        initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInitialValueA(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetInitialValueA(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWM23_INIT_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWM23_INIT_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetInitialValueB
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the initial value for PWM_B and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the initial value. This parameter is of "Determines
 *        initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInitialValueB(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetInitialValueB(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWM45_INIT_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWM45_INIT_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetInitialValueX
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the initial value for PWM_X and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the initial value. This parameter is of "Determines
 *        initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInitialValueX(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetInitialValueX(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWMX_INIT_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_PWMX_INIT_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitialValueA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current initial value for PWM_A and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetInitialValueA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInitialValueA(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
      PWM_CTRL2_PWM23_INIT_MASK \
     )) >> ( \
     PWM_CTRL2_PWM23_INIT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitialValueB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current initial value for PWM_B and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetInitialValueB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInitialValueB(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
      PWM_CTRL2_PWM45_INIT_MASK \
     )) >> ( \
     PWM_CTRL2_PWM45_INIT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitialValueX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current initial value for PWM_X and the value to which it is
 * forced when FORCE_INIT.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetInitialValueX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInitialValueX(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
      PWM_CTRL2_PWMX_INIT_MASK \
     )) >> ( \
     PWM_CTRL2_PWMX_INIT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInitializationControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of the INIT signal which goes to the counter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the signal which goes to the initialization control
 *        select. This parameter is of "Determines signals which goes to the
 *        initialization control select." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInitializationControlSelect(deviceID, periphID,
 *      PWM_PDD_IS_LOCAL_SYNC);
 *      @endcode
 */
#define PWM_PDD_SetInitializationControlSelect(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_INIT_SEL_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInitializationControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current initialization source.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines signals which goes to the
 *         initialization control select." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInitializationControlSelect(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetInitializationControlSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_INIT_SEL_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableForceInitialization
   ---------------------------------------------------------------------------- */

/**
 * @brief Allows the CTRL2[FORCE] signal to initialize the counter without
 * regard to the signal selected by CTRL2[INIT_SEL]. This is a software controlled
 * initialization.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables or disables the force initialization. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableForceInitialization(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableForceInitialization(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_FRCEN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_FRCEN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetForceInitializationEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of force initialization.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Global enumeration used for specifying general
 *         enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *         PDD_Types.h)" type. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetForceInitializationEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetForceInitializationEnabled(PeripheralBase, SmIndex) ( \
    (( \
      PeriphGetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_FRCEN_MASK \
      )) == ( \
      0U)) ? ( \
      PDD_DISABLE) : ( \
      PDD_ENABLE) \
  )

/* ----------------------------------------------------------------------------
   -- ForceInitialization
   ---------------------------------------------------------------------------- */

/**
 * @brief If SetInitializationControlSelect is set to LOCAL_SYNC, this function
 * causes the FORCE_OUT event. The PWM_A and PWM_B output pins will assume values
 * based on DTSRCSEL[SMxSEL23] and DTSRCSEL[SMxSEL45]. If
 * EnableForceInitialization is ENABLED, the counter value will be initialized with the INIT register
 * value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_ForceInitialization(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ForceInitialization(PeripheralBase, SmIndex) ( \
    PeriphSetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_FORCE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetForceOutputSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of the INIT signal which goes to the counter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the force output source. This parameter is of
 *        "Determines signals which act as force output source." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetForceOutputSource(deviceID, periphID,
 *      PWM_PDD_FO_LOCAL_FORCE);
 *      @endcode
 */
#define PWM_PDD_SetForceOutputSource(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_FORCE_SEL_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetForceOutputSource
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current force output source.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines signals which act as force output
 *         source." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetForceOutputSource(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetForceOutputSource(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_FORCE_SEL_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetReloadSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of the RELOAD signal for this submodule. When
 * this bit is set, MCTRL[LDOK[0]] for submodule 0 should be used since the local
 * MCTRL[LDOK] will be ignored.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the signal which goes to the reload source select.
 *        This parameter is of "Determines signals which goes to the reload source
 *        select." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetReloadSourceSelect(deviceID, periphID,
 *      PWM_PDD_RS_LOCAL_RELOAD);
 *      @endcode
 */
#define PWM_PDD_SetReloadSourceSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_RS_LOCAL_RELOAD) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_RELOAD_SEL_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL2_RELOAD_SEL_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetReloadSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current reload source.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines signals which goes to the reload
 *         source select." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetReloadSourceSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetReloadSourceSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_RELOAD_SEL_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetClockSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of the clock signal for this submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the signal which acts as clock source. This parameter
 *        is of "Determines signals which goes to the clock source select." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetClockSourceSelect(deviceID, periphID, PWM_PDD_BUS_CLK);
 *      @endcode
 */
#define PWM_PDD_SetClockSourceSelect(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_INIT_SEL_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetClockSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current source of the clock signal for this submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines signals which goes to the clock source
 *         select." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetClockSourceSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetClockSourceSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CTRL2_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL2_INIT_SEL_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetReloadFrequency
   ---------------------------------------------------------------------------- */

/**
 * @brief Select from 1 to 16 PWM reload opportunities determined by property
 * Half cycle reload - CTRL[HALF] and Full cycle reload - CTRL[FULL].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Count Defines PWM load frequency (0 means 1 times and 15 means 16
 *        times). This parameter is a 4-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetReloadFrequency(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetReloadFrequency(PeripheralBase, SmIndex, Count) ( \
    PeriphSetBits16( \
     PWM_CTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL_LDFQ_MASK, \
     (uint16_t)((uint16_t)(Count) << PWM_CTRL_LDFQ_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetReloadFrequency
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns number of PWM reload opportunities per reload (0 means 1 times
 * and 15 means 16 times).
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetReloadFrequency(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetReloadFrequency(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_LDFQ_MASK)) >> ( \
     PWM_CTRL_LDFQ_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableHalfCycleReload
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables half-cycle reloads. A half cycle is defined by
 * when the submodule counter matches the VAL0 register and does not have to be half
 * way through the PWM cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if half-cycle reloads will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableHalfCycleReload(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableHalfCycleReload(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_HALF_MASK)) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_HALF_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetHalfCycleReloadEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of half-cycle reload.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetHalfCycleReloadEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetHalfCycleReloadEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_HALF_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFullCycleReload
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables full-cycle reloads. A full cycle is defined by
 * when the submodule counter matches the VAL1 register. Either CTRL[HALF] or
 * CTRL[FULL] must be set in order to move the buffered data into the registers used by
 * the PWM generators. If both CTRL[HALF] and CTRL[FULL] are set, then reloads
 * can occur twice per cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if half-cycle reloads will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFullCycleReload(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFullCycleReload(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_FULL_MASK)) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_FULL_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFullCycleReloadEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of full-cycle reload.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFullCycleReloadEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFullCycleReloadEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_FULL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- XInputInDeadtime0
   ---------------------------------------------------------------------------- */

/**
 * @brief Retrieves the sampled values of the PWM_X input at the end of deadtime
 * 0. Sampling occurs at the end of deadtime 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_XInputInDeadtime0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_XInputInDeadtime0(PeripheralBase, SmIndex) ( \
    (uint16_t)(( \
     0x100U) & ( \
     PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DT_MASK))) \
  )

/* ----------------------------------------------------------------------------
   -- XInputInDeadtime1
   ---------------------------------------------------------------------------- */

/**
 * @brief Retrieves the sampled values of the PWM_X input at the end of deadtime
 * 1. Sampling occurs at the end of deadtime 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_XInputInDeadtime1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_XInputInDeadtime1(PeripheralBase, SmIndex) ( \
    (uint16_t)(( \
     0x200U) & ( \
     PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DT_MASK))) \
  )

/* ----------------------------------------------------------------------------
   -- SetPrescaler
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of the INIT signal which goes to the counter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Ratio Determines the division ratio. This parameter is of "Determines
 *        prescalers division ratios." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetPrescaler(deviceID, periphID, PWM_PDD_FCLK_DIV1);
 *      @endcode
 */
#define PWM_PDD_SetPrescaler(PeripheralBase, SmIndex, Ratio) ( \
    PeriphSetBits16( \
     PWM_CTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_CTRL_PRSC_MASK, \
     (uint16_t)(Ratio) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetPrescaler
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current division ratio of the prescaler.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines prescalers division ratios." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetPrescaler(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetPrescaler(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_PRSC_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableSplitDBLPWM
   ---------------------------------------------------------------------------- */

/**
 * @brief Allows the two PWM pulses generated by DBLEN to be split with one
 * pulse on PWMA and one on PWMB. This function is only used when DBLEN is set. The
 * two pulses within the same PWM period are created by an XOR function of the
 * PWMA and PWMB sources. The splitting function causes PWMA to output the pulse
 * that occurs when the PWMA source is 1 and the PWMB source is 0. The PWMB output
 * occurs when the PWMB source is 1 and the PWMA source is 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DBLPWM is split to PWMA and PWMB.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)"
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableSplitDBLPWM(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableSplitDBLPWM(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL_SPLIT_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_SPLIT_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSplitDBLPWMEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the DBLPWM is split to PWMA and PWMB or not.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetSplitDBLPWMEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetSplitDBLPWMEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_SPLIT_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetLoadModeSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the timing of loading the buffered registers for this
 * submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the conditions for loading buffered registers into
 *        working registers. This parameter is of "Determines the load mode select
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetLoadModeSelect(deviceID, periphID, PWM_PDD_LDOK_RELOAD);
 *      @endcode
 */
#define PWM_PDD_SetLoadModeSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LDOK_RELOAD) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL_LDMOD_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_LDMOD_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetLoadModeSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current conditions for loading buffered registers into working
 * registers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the load mode select constants." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetLoadModeSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetLoadModeSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_LDMOD_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXDoubleSwitching
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the double switching behavior on PWMX. When this
 * bit is set, the PWMX output shall be the exclusive OR combination of PWMA and
 * PWMB prior to polarity and masking considerations.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the  double switching will be enabled
 *        or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE defined
 *        in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableXDoubleSwitching(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableXDoubleSwitching(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DBLX_MASK)) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DBLX_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXDoubleSwitchingEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of double switching behavior on PWMX.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetXDoubleSwitchingEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetXDoubleSwitchingEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DBLX_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableDoubleSwitching
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the double switching PWM behavior.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the  double switching will be enabled
 *        or disabled. This parameter is of "Global enumeration used for
 *        specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE defined
 *        in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableDoubleSwitching(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableDoubleSwitching(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_CTRL_DBLEN_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DBLEN_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDoubleSwitchingEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state double switching behavior.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetDoubleSwitchingEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetDoubleSwitchingEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), PWM_CTRL_DBLEN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetAInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the logic value currently being driven into the PWM_A input.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetAInput(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetAInput(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
      PWM_OCTRL_PWMA_IN_MASK \
     )) >> ( \
     PWM_OCTRL_PWMA_IN_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetBInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the logic value currently being driven into the PWM_B input.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetBInput(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetBInput(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
      PWM_OCTRL_PWMB_IN_MASK \
     )) >> ( \
     PWM_OCTRL_PWMB_IN_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetXInput
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the logic value currently being driven into the PWM_X input.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetXInput(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetXInput(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
      PWM_OCTRL_PWMX_IN_MASK \
     )) >> ( \
     PWM_OCTRL_PWMX_IN_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputAPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which level on the PWM_A pin represents the "on" or
 * "active" state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the high or low level as the active state. This
 *        parameter is of "Determines initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOutputAPolarity(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetOutputAPolarity(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_OCTRL_POLA_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLA_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputBPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which level on the PWM_B pin represents the "on" or
 * "active" state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the high or low level as the active state. This
 *        parameter is of "Determines initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOutputBPolarity(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetOutputBPolarity(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_OCTRL_POLB_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLB_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputXPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which level on the PWM_X pin represents the "on" or
 * "active" state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the high or low level as the active state. This
 *        parameter is of "Determines initial values" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOutputXPolarity(deviceID, periphID, PWM_PDD_LOW);
 *      @endcode
 */
#define PWM_PDD_SetOutputXPolarity(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOW) ? ( \
      PeriphClearBitMask16( \
       PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_OCTRL_POLX_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLX_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputAPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which level on the PWM_A pin represents the "on" or "active"
 * state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetOutputAPolarity(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOutputAPolarity(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLA_MASK)) >> ( \
     PWM_OCTRL_POLA_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputBPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which level on the PWM_B pin represents the "on" or "active"
 * state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetOutputBPolarity(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOutputBPolarity(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLB_MASK)) >> ( \
     PWM_OCTRL_POLB_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputXPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which level on the PWM_X pin represents the "on" or "active"
 * state.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetOutputXPolarity(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOutputXPolarity(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), PWM_OCTRL_POLX_MASK)) >> ( \
     PWM_OCTRL_POLX_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultStateA
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the fault state for the PWM_A output during fault
 * conditions and STOP mode. It may also define the output state during WAIT and DEBUG
 * modes depending on the settings of CTRL2[WAITEN] and CTRL2[DBGEN].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the state during fault condition. This parameter is
 *        of "Determines possible states during fault conditions and STOP mode."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultStateA(deviceID, periphID, PWM_PDD_LOGIC0);
 *      @endcode
 */
#define PWM_PDD_SetFaultStateA(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_OCTRL_PWMAFS_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_OCTRL_PWMAFS_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultStateB
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the fault state for the PWM_B output during fault
 * conditions and STOP mode. It may also define the output state during WAIT and DEBUG
 * modes depending on the settings of CTRL2[WAITEN] and CTRL2[DBGEN].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the state during fault condition. This parameter is
 *        of "Determines possible states during fault conditions and STOP mode."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultStateB(deviceID, periphID, PWM_PDD_LOGIC0);
 *      @endcode
 */
#define PWM_PDD_SetFaultStateB(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_OCTRL_PWMBFS_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_OCTRL_PWMBFS_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultStateX
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the fault state for the PWM_X output during fault
 * conditions and STOP mode. It may also define the output state during WAIT and DEBUG
 * modes depending on the settings of CTRL2[WAITEN] and CTRL2[DBGEN].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the state during fault condition. This parameter is
 *        of "Determines possible states during fault conditions and STOP mode."
 *        type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultStateX(deviceID, periphID, PWM_PDD_LOGIC0);
 *      @endcode
 */
#define PWM_PDD_SetFaultStateX(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_OCTRL_PWMXFS_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultStateA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the state during fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultStateA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultStateA(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
      PWM_OCTRL_PWMAFS_MASK \
     )) >> ( \
     PWM_OCTRL_PWMAFS_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultStateB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the state during fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultStateB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultStateB(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
      PWM_OCTRL_PWMBFS_MASK \
     )) >> ( \
     PWM_OCTRL_PWMBFS_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultStateX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the state during fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultStateX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultStateX(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_OCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_OCTRL_PWMXFS_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns submodule interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: STS[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInterruptFlags(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInterruptFlags(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_STS_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)(( \
      PWM_STS_RUF_MASK) | (( \
      PWM_STS_REF_MASK) | (( \
      PWM_STS_RF_MASK) | (( \
      PWM_STS_CFA1_MASK) | (( \
      PWM_STS_CFA0_MASK) | (( \
      PWM_STS_CFB1_MASK) | (( \
      PWM_STS_CFB0_MASK) | (( \
      PWM_STS_CFX1_MASK) | (( \
      PWM_STS_CFX0_MASK) | ( \
      PWM_STS_CMPF_MASK)))))))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears submodule interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Mask Mask of interrupt's flags to clear. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: STS[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_ClearInterruptFlags(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_ClearInterruptFlags(PeripheralBase, SmIndex, Mask) ( \
    PeriphSetBits16( \
     PWM_STS_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)(( \
      PWM_STS_REF_MASK) | (( \
      PWM_STS_RF_MASK) | (( \
      PWM_STS_CFA1_MASK) | (( \
      PWM_STS_CFA0_MASK) | (( \
      PWM_STS_CFB1_MASK) | (( \
      PWM_STS_CFB0_MASK) | (( \
      PWM_STS_CFX1_MASK) | (( \
      PWM_STS_CFX0_MASK) | ( \
      PWM_STS_CMPF_MASK))))))))), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables submodule interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableInterrupts(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_EnableInterrupts(PeripheralBase, SmIndex, Mask) ( \
    PeriphSetBitMask16(PWM_INTEN_REG(PeripheralBase,(SmIndex)), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables submodule interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_DisableInterrupts(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_DisableInterrupts(PeripheralBase, SmIndex, Mask) ( \
    PeriphClearBitMask16(PWM_INTEN_REG(PeripheralBase,(SmIndex)), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets all submodule interrupts with value according to mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInterruptMask(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetInterruptMask(PeripheralBase, SmIndex, Mask) ( \
    PeriphSetBits16( \
     PWM_INTEN_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)(( \
      PWM_INTEN_REIE_MASK) | (( \
      PWM_INTEN_RIE_MASK) | (( \
      PWM_INTEN_CA1IE_MASK) | (( \
      PWM_INTEN_CA0IE_MASK) | (( \
      PWM_INTEN_CB1IE_MASK) | (( \
      PWM_INTEN_CB0IE_MASK) | (( \
      PWM_INTEN_CX1IE_MASK) | (( \
      PWM_INTEN_CX0IE_MASK) | ( \
      PWM_INTEN_CMPIE_MASK))))))))), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns submodule interrupts mask.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInterruptMask(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInterruptMask(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_INTEN_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)(( \
      PWM_INTEN_REIE_MASK) | (( \
      PWM_INTEN_RIE_MASK) | (( \
      PWM_INTEN_CA1IE_MASK) | (( \
      PWM_INTEN_CA0IE_MASK) | (( \
      PWM_INTEN_CB1IE_MASK) | (( \
      PWM_INTEN_CB0IE_MASK) | (( \
      PWM_INTEN_CX1IE_MASK) | (( \
      PWM_INTEN_CX0IE_MASK) | ( \
      PWM_INTEN_CMPIE_MASK))))))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputTrigger0SourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which signal to bring out on the PWM's PWM_OUT_TRIG0 port.
 * The output trigger port is often connected to routing logic on the chip. This
 * control bit allows the PWMA output signal to be driven onto the output trigger
 * port so it can be sent to the chip routing logic.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines source of output trigger 0. This parameter is of
 *        "Determines the Output Trigger 0 Sources constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOutputTrigger0SourceSelect(deviceID, periphID,
 *      PWM_PDD_PWM_OUT_TRIG0);
 *      @endcode
 */
#define PWM_PDD_SetOutputTrigger0SourceSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_PWM_OUT_TRIG0) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_TCTRL_PWAOT0_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_TCTRL_PWAOT0_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetOutputTrigger1SourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which signal to bring out on the PWM's PWM_OUT_TRIG1 port.
 * The output trigger port is often connected to routing logic on the chip. This
 * control bit allows the PWMB output signal to be driven onto the output trigger
 * port so it can be sent to the chip routing logic.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines source of output trigger 1. This parameter is of
 *        "Determines the Output Trigger 1 Sources constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOutputTrigger1SourceSelect(deviceID, periphID,
 *      PWM_PDD_PWM_OUT_TRIG1);
 *      @endcode
 */
#define PWM_PDD_SetOutputTrigger1SourceSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_PWM_OUT_TRIG1) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_TCTRL_PWBOT1_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_TCTRL_PWBOT1_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputTrigger0SourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns currentli selected source of output trigger 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the Output Trigger 0 Sources
 *         constants." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetOutputTrigger0SourceSelect(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetOutputTrigger0SourceSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_TCTRL_PWAOT0_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOutputTrigger1SourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns currentli selected source of output trigger 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the Output Trigger 1 Sources
 *         constants." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetOutputTrigger1SourceSelect(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetOutputTrigger1SourceSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_TCTRL_PWBOT1_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal0OutputTrigger0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG0 output based on the counter
 * value matching the value in the VAL0 register. The PWM_OUT_TRIG0 signal is only
 * asserted as long as the counter value matches the VAL0 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger0 based on the VAL0
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal0OutputTrigger0(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal0OutputTrigger0(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16(PWM_TCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)0x1U)) : ( \
      PeriphSetBitMask16(PWM_TCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal1OutputTrigger1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG1 output based on the counter
 * value matching the value in the VAL1 register. The PWM_OUT_TRIG1 signal is only
 * asserted as long as the counter value matches the VAL1 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger1 based on the VAL1
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal1OutputTrigger1(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal1OutputTrigger1(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 1U) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 1U) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal2OutputTrigger0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG0 output based on the counter
 * value matching the value in the VAL2 register. The PWM_OUT_TRIG0 signal is only
 * asserted as long as the counter value matches the VAL2 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger0 based on the VAL2
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal2OutputTrigger0(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal2OutputTrigger0(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 2U) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 2U) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal3OutputTrigger1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG1 output based on the counter
 * value matching the value in the VAL3 register. The PWM_OUT_TRIG1 signal is only
 * asserted as long as the counter value matches the VAL3 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger1 based on the VAL3
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal3OutputTrigger1(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal3OutputTrigger1(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 3U) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 3U) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal4OutputTrigger0
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG0 output based on the counter
 * value matching the value in the VAL4 register. The PWM_OUT_TRIG0 signal is only
 * asserted as long as the counter value matches the VAL4 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger0 based on the VAL4
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal4OutputTrigger0(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal4OutputTrigger0(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 4U) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 4U) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableVal5OutputTrigger1
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables the generation of PWM_OUT_TRIG1 output based on the counter
 * value matching the value in the VAL5 register. The PWM_OUT_TRIG1 signal is only
 * asserted as long as the counter value matches the VAL5 value; therefore, up to
 * six triggers can be generated per PWM cycle per submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the output trigger1 based on the VAL5
 *        will be enabled (powered up) or disabled (turned off). This parameter
 *        is of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableVal5OutputTrigger1(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableVal5OutputTrigger1(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 5U) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
       (uint16_t)((uint16_t)0x1U << 5U) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal0OutputTrigger0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger0 based on the VAL0 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal0OutputTrigger0Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal0OutputTrigger0Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_TCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)0x1U) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal1OutputTrigger1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger1 based on the VAL1 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal1OutputTrigger1Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal1OutputTrigger1Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)((uint16_t)0x1U << 1U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal2OutputTrigger0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger0 based on the VAL2 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal2OutputTrigger0Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal2OutputTrigger0Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)((uint16_t)0x1U << 2U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal3OutputTrigger1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger1 based on the VAL3 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal3OutputTrigger1Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal3OutputTrigger1Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)((uint16_t)0x1U << 3U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal4OutputTrigger0Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger0 based on the VAL4 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal4OutputTrigger0Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal4OutputTrigger0Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)((uint16_t)0x1U << 4U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetVal5OutputTrigger1Enabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the output trigger1 based on the VAL5 is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetVal5OutputTrigger1Enabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetVal5OutputTrigger1Enabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_TCTRL_REG(PeripheralBase,(SmIndex)), \
     (uint16_t)((uint16_t)0x1U << 5U) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableValueRegistersDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA write requests for the VALx and FRACVALx
 * registers when STS[RF] is set.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA write requests for the VALx and
 *        FRACVALx registers will be enabled or disabled. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableValueRegistersDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableValueRegistersDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_VALDE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_VALDE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetValueRegistersDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA write requests for the VALx and FRACVALx registers is
 * enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValueRegistersDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetValueRegistersDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_VALDE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFifoWatermarkAndControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Works in conjunction with the DMAEN[CAPTDE] field when it is set to
 * watermark mode (DMAEN[CAPTDE] = 01). While DMAEN[CAxDE], DMAEN[CBxDE], and
 * DMAEN[CXxDE] determine which FIFO watermarks the DMA read request is sensitive to,
 * this bit determines if the selected watermarks are AND'ed together or OR'ed
 * together in order to create the request.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if the selected watermarks are AND'ed together or
 *        OR'ed together in order to create the request. This parameter is of
 *        "Determines the FIFO Watermark AND Control constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFifoWatermarkAndControl(deviceID, periphID,
 *      PWM_PDD_WATERMARKS_ORED);
 *      @endcode
 */
#define PWM_PDD_SetFifoWatermarkAndControl(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_WATERMARKS_ORED) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_FAND_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_DMAEN_REG(PeripheralBase,(SmIndex)), PWM_DMAEN_FAND_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFifoWatermarkAndControl
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the selected watermarks are ANDed together or ORed together
 * in order to create the request.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the FIFO Watermark AND Control
 *         constants." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFifoWatermarkAndControl(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetFifoWatermarkAndControl(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16(PWM_DMAEN_REG(PeripheralBase,(SmIndex)), PWM_DMAEN_FAND_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- SetCaptureDmaSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the source of enabling the DMA read requests for the
 * capture FIFOs.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the capture DMA source request. This parameter is of
 *        "Determines the capture DMA enable source select constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetCaptureDmaSourceSelect(deviceID, periphID,
 *      PWM_PDD_DMA_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetCaptureDmaSourceSelect(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CAPTDE_MASK, \
     (uint16_t)(State) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureDmaSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the source of enabling the DMA read request for the capture
 * FIFOs.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the capture DMA enable source select
 *         constants." type. The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureDmaSourceSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureDmaSourceSelect(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CAPTDE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureA0FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture A0 FIFO data
 * when STS[CFA0] is set. Do not set both this bit and INTEN[CA0IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture A0
 *        FIFO data when STS[CFA0] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureA0FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureA0FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CA0DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CA0DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureA1FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture A1 FIFO data
 * when STS[CFA1] is set. Do not set both this bit and INTEN[CA1IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture A1
 *        FIFO data when STS[CFA1] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureA1FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureA1FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CA1DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CA1DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureB0FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture B0 FIFO data
 * when STS[CFB0] is set. Do not set both this bit and INTEN[CB0IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture B0
 *        FIFO data when STS[CFB0] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureB0FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureB0FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CB0DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CB0DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureB1FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture B1 FIFO data
 * when STS[CFB1] is set. Do not set both this bit and INTEN[CB1IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture B1
 *        FIFO data when STS[CFB1] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureB1FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureB1FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CB1DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CB1DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureX0FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture X0 FIFO data
 * when STS[CFX0] is set. Do not set both this bit and INTEN[CX0IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture X0
 *        FIFO data when STS[CFX0] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureX0FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureX0FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CX0DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CX0DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableCaptureX1FifoDma
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables DMA read requests for the Capture X1 FIFO data
 * when STS[CFX1] is set. Do not set both this bit and INTEN[CX1IE].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if DMA read requests for the Capture X1
 *        FIFO data when STS[CFX1] will be enabled or disabled. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableCaptureX1FifoDma(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableCaptureX1FifoDma(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CX1DE_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
       PWM_DMAEN_CX1DE_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureA0FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture A0 FIFO data when
 * STS[CFA0] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureA0FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureA0FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CA0DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureA1FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture A1 FIFO data when
 * STS[CFA1] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureA1FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureA1FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CA1DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureB0FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture B0 FIFO data when
 * STS[CFB0] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureB0FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureB0FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CB0DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureB1FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture B1 FIFO data when
 * STS[CFB1] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureB1FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureB1FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CB1DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureX0FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture X0 FIFO data when
 * STS[CFX0] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureX0FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureX0FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CX0DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureX1FifoDmaEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if DMA read requests for the Capture X1 FIFO data when
 * STS[CFX1] is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetCaptureX1FifoDmaEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureX1FifoDmaEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_DMAEN_REG(PeripheralBase,(SmIndex)), \
     PWM_DMAEN_CX1DE_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAFault
   ---------------------------------------------------------------------------- */

/**
 * @brief The PWM_A output is turned off if there is a logic 1 on a
 * FAULT[FInIndex] input of a fault channel and the AFault is enabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State Parameter is specifying if the AFault will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableAFault(deviceID, periphID, periphID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableAFault(PeripheralBase, SmIndex, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0A_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_DISMAP_DIS0A_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0A_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBFault
   ---------------------------------------------------------------------------- */

/**
 * @brief The PWM_B output is turned off if there is a logic 1 on a
 * FAULT[FInIndex] input of a fault channel and the BFault is enabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State Parameter is specifying if the BFault will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableBFault(deviceID, periphID, periphID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableBFault(PeripheralBase, SmIndex, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0B_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_DISMAP_DIS0B_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0B_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXFault
   ---------------------------------------------------------------------------- */

/**
 * @brief The PWM_X output is turned off if there is a logic 1 on a
 * FAULT[FInIndex] input of a fault channel and the XFault is enabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State Parameter is specifying if the XFault will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableXFault(deviceID, periphID, periphID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableXFault(PeripheralBase, SmIndex, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0X_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_DISMAP_DIS0X_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0X_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetAFaultEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the AFault is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetAFaultEnabled(deviceID, periphID,
 *      periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetAFaultEnabled(PeripheralBase, SmIndex, FChIndex, FInIndex) ( \
    PeriphGetBitMask16( \
     PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
     (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0A_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBFaultEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the BFault is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetBFaultEnabled(deviceID, periphID,
 *      periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetBFaultEnabled(PeripheralBase, SmIndex, FChIndex, FInIndex) ( \
    PeriphGetBitMask16( \
     PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
     (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0B_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetXFaultEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns wether the XFault is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers:
 *          DISMAP[SmIndex][FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetXFaultEnabled(deviceID, periphID,
 *      periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetXFaultEnabled(PeripheralBase, SmIndex, FChIndex, FInIndex) ( \
    PeriphGetBitMask16( \
     PWM_DISMAP_REG(PeripheralBase,(SmIndex),(FChIndex)), \
     (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_DISMAP_DIS0X_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetDeadtimeCount0
   ---------------------------------------------------------------------------- */

/**
 * @brief Used to control the deadtime during 0 to 1 transitions of the PWM_A
 * output (assuming normal polarity).
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Data Defines dead time 0 in terms of IPBus clock cycles. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DTCNT0[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetDeadtimeCount0(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetDeadtimeCount0(PeripheralBase, SmIndex, Data) ( \
    PeriphWriteReg16(PWM_DTCNT0_REG(PeripheralBase,(SmIndex)), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- SetDeadtimeCount1
   ---------------------------------------------------------------------------- */

/**
 * @brief Used to control the deadtime during 0 to 1 transitions of the PWM_B
 * output (assuming normal polarity).
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Data Defines dead time 1 in terms of IPBus clock cycles. This
 *        parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DTCNT1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetDeadtimeCount1(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetDeadtimeCount1(PeripheralBase, SmIndex, Data) ( \
    PeriphWriteReg16(PWM_DTCNT1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Data)) \
  )

/* ----------------------------------------------------------------------------
   -- GetDeadtimeCount0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the dead time 0 in terms of IPBus clock cycles.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DTCNT0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetDeadtimeCount0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetDeadtimeCount0(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DTCNT0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetDeadtimeCount1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the dead time 1 in terms of IPBus clock cycles.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DTCNT1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetDeadtimeCount1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetDeadtimeCount1(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DTCNT1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue0
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the mid-cycle reload point for the PWM in PWM clock periods.
 * This value also defines when the PWM_X signal is set and the local sync signal
 * isreset.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL0 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL0[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue0(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue0(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL0_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue1
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the modulo count value (maximum count) for the submodule
 * counter. Upon reaching this count value the counter reloads itself with the
 * contents of the INIT register and asserts the local sync signal while resetting PWM_X.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL1 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue1(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue1(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue2
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the count value to set PWM23 high.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL2 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue2(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue2(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL2_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue3
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the count value to set PWM23 low.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL3 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL3[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue3(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue3(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL3_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue4
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the count value to set PWM45 high.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL4 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL4[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue4(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue4(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL4_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- SetValue5
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the count value to set PWM45 low.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines VAL5 value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL5[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetValue5(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetValue5(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL5_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL0 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue0(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL1 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue1(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL2 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue2(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue2(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL3 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue3(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue3(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL3_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL4 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue4(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue4(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL4_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- GetValue5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the VAL5 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetValue5(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetValue5(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL5_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureA0FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture A0 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureA0FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureA0FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLA_CA0CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLA_CA0CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureA1FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture A1 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureA1FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureA1FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLA_CA1CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLA_CA1CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureB0FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture B0 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureB0FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureB0FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLB_CB0CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLB_CB0CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureB1FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture B1 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureB1FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureB1FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLB_CB1CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLB_CB1CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureX0FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture X0 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureX0FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureX0FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLX_CX0CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLX_CX0CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureX1FifoCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the number of words in the Capture X1 FIFO.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureX1FifoCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureX1FifoCount(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLX_CX1CNT_MASK \
     )) >> ( \
     PWM_CAPTCTRLX_CX1CNT_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCaptureAFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the water mark level for capture A FIFOs. The capture flags,
 * STS[CFA1] and STS[CFA0], wont be set until the word count of the corresponding
 * FIFO is greater than this water mark level.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 2 value. This parameter is a 2-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetCaptureAFifoWaterMark(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetCaptureAFifoWaterMark(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_CFAWM_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_CAPTCTRLA_CFAWM_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCaptureBFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the water mark level for capture B FIFOs. The capture flags,
 * STS[CFB1] and STS[CFB0], wont be set until the word count of the corresponding
 * FIFO is greater than this water mark level.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 3 value. This parameter is a 2-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetCaptureBFifoWaterMark(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetCaptureBFifoWaterMark(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_CFBWM_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_CAPTCTRLB_CFBWM_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetCaptureXFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the water mark level for capture X FIFOs. The capture flags,
 * STS[CFX1] and STS[CFX0], wont be set until the word count of the corresponding
 * FIFO is greater than this water mark level.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 4 value. This parameter is a 2-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetCaptureXFifoWaterMark(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetCaptureXFifoWaterMark(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_CFXWM_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_CAPTCTRLX_CFXWM_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureAFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the water mark level for capture A FIFOs.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetCaptureAFifoWaterMark(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureAFifoWaterMark(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLA_CFAWM_MASK \
     )) >> ( \
     PWM_CAPTCTRLA_CFAWM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureBFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the water mark level for capture B FIFOs.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetCaptureBFifoWaterMark(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureBFifoWaterMark(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLB_CFBWM_MASK \
     )) >> ( \
     PWM_CAPTCTRLB_CFBWM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetCaptureXFifoWaterMark
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the water mark level for capture X FIFOs.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetCaptureXFifoWaterMark(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCaptureXFifoWaterMark(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLX_CFXWM_MASK \
     )) >> ( \
     PWM_CAPTCTRLX_CFXWM_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEdgeCounterA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the edge counter which counts rising and falling
 * edges on the PWM_A input signal.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the edge counter which counts rising
 *        and falling edges on the PWM_A input signal will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableEdgeCounterA(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableEdgeCounterA(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_EDGCNTA_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_EDGCNTA_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEdgeCounterB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the edge counter which counts rising and falling
 * edges on the PWM_B input signal.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the edge counter which counts rising
 *        and falling edges on the PWM_B input signal will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableEdgeCounterB(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableEdgeCounterB(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_EDGCNTB_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_EDGCNTB_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableEdgeCounterX
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the edge counter which counts rising and falling
 * edges on the PWM_X input signal.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the edge counter which counts rising
 *        and falling edges on the PWM_X input signal will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableEdgeCounterX(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableEdgeCounterX(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_EDGCNTX_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_EDGCNTX_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCounterAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the edge counter which counts rising and falling edges on
 * the PWM_A input signal is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetEdgeCounterAEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCounterAEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_EDGCNTA_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCounterBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the edge counter which counts rising and falling edges on
 * the PWM_B input signal is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetEdgeCounterBEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCounterBEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_EDGCNTB_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCounterXEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the edge counter which counts rising and falling edges on
 * the PWM_X input signal is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetEdgeCounterXEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCounterXEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_EDGCNTX_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputSelectA
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between the raw PWM_A input signal and the output of the edge
 * counter/compare circuitry as the source for the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if the raw PWM_A input signal ot the output of the
 *        edge counter/compare circuitry is selected as source. This parameter is
 *        of "Determines the Input Select constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInputSelectA(deviceID, periphID, PWM_PDD_RAW_INPUT);
 *      @endcode
 */
#define PWM_PDD_SetInputSelectA(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_RAW_INPUT) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_INP_SELA_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_INP_SELA_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputSelectB
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between the raw PWM_B input signal and the output of the edge
 * counter/compare circuitry as the source for the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if the raw PWM_B input signal ot the output of the
 *        edge counter/compare circuitry is selected as source. This parameter is
 *        of "Determines the Input Select constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInputSelectB(deviceID, periphID, PWM_PDD_RAW_INPUT);
 *      @endcode
 */
#define PWM_PDD_SetInputSelectB(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_RAW_INPUT) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_INP_SELB_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_INP_SELB_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetInputSelectX
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between the raw PWM_X input signal and the output of the edge
 * counter/compare circuitry as the source for the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if the raw PWM_X input signal ot the output of the
 *        edge counter/compare circuitry is selected as source. This parameter is
 *        of "Determines the Input Select constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetInputSelectX(deviceID, periphID, PWM_PDD_RAW_INPUT);
 *      @endcode
 */
#define PWM_PDD_SetInputSelectX(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_RAW_INPUT) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_INP_SELX_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_INP_SELX_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetInputSelectA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the raw PWM_A input signal ot the output of the edge
 * counter/compare circuitry is selected as source of the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the Input Select constants." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInputSelectA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInputSelectA(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_INP_SELA_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInputSelectB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the raw PWM_B input signal ot the output of the edge
 * counter/compare circuitry is selected as source of the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the Input Select constants." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInputSelectB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInputSelectB(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_INP_SELB_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetInputSelectX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the raw PWM_X input signal ot the output of the edge
 * counter/compare circuitry is selected as source of the input capture circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the Input Select constants." type. The
 *         value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetInputSelectX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetInputSelectX(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_INP_SELX_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 0 circuitry by determining which input A
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input A edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeA0(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeA0(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_EDGA0_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLA_EDGA0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 1 circuitry by determining which input A
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input A edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeA1(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeA1(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_EDGA1_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLA_EDGA1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeB0
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 0 circuitry by determining which input B
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input B edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeB0(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeB0(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_EDGB0_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLB_EDGB0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeB1
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 1 circuitry by determining which input B
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input B edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeB1(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeB1(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_EDGB1_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLB_EDGB1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeX0
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 0 circuitry by determining which input X
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input X edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeX0(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeX0(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_EDGX0_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLX_EDGX0_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeX1
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects the input capture 1 circuitry by determining which input X
 * edges cause a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines which input X edges cause a capture event. This
 *        parameter is of "Determines the Edge constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeX1(deviceID, periphID, PWM_PDD_CAPTURE_DISABLED);
 *      @endcode
 */
#define PWM_PDD_SetEdgeX1(PeripheralBase, SmIndex, State) ( \
    PeriphSetBits16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_EDGX1_MASK, \
     (uint16_t)((uint16_t)(State) << PWM_CAPTCTRLX_EDGX1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeA0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input A edges cause a capture event of the input capture
 * 0 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeA0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeA0(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLA_EDGA0_MASK \
     )) >> ( \
     PWM_CAPTCTRLA_EDGA0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeA1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input A edges cause a capture event of the input capture
 * 1 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeA1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeA1(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLA_EDGA1_MASK \
     )) >> ( \
     PWM_CAPTCTRLA_EDGA1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeB0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input B edges cause a capture event of the input capture
 * 0 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeB0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeB0(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLB_EDGB0_MASK \
     )) >> ( \
     PWM_CAPTCTRLB_EDGB0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeB1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input B edges cause a capture event of the input capture
 * 1 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeB1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeB1(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLB_EDGB1_MASK \
     )) >> ( \
     PWM_CAPTCTRLB_EDGB1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeX0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input X edges cause a capture event of the input capture
 * 0 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeX0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeX0(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLX_EDGX0_MASK \
     )) >> ( \
     PWM_CAPTCTRLX_EDGX0_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeX1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns which input X edges cause a capture event of the input capture
 * 1 circuit.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeX1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeX1(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCTRLX_EDGX1_MASK \
     )) >> ( \
     PWM_CAPTCTRLX_EDGX1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetOneShotModeA
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between free running and one shot mode for the input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if free running or one shot mode will be selected.
 *        This parameter is of "Determines the One Shot Mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOneShotModeA(deviceID, periphID, PWM_PDD_FREE_RUNNING);
 *      @endcode
 */
#define PWM_PDD_SetOneShotModeA(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_FREE_RUNNING) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_ONESHOTA_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_ONESHOTA_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetOneShotModeB
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between free running and one shot mode for the input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if free running or one shot mode will be selected.
 *        This parameter is of "Determines the One Shot Mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOneShotModeB(deviceID, periphID, PWM_PDD_FREE_RUNNING);
 *      @endcode
 */
#define PWM_PDD_SetOneShotModeB(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_FREE_RUNNING) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_ONESHOTB_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_ONESHOTB_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetOneShotModeX
   ---------------------------------------------------------------------------- */

/**
 * @brief Selects between free running and one shot mode for the input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines if free running or one shot mode will be selected.
 *        This parameter is of "Determines the One Shot Mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetOneShotModeX(deviceID, periphID, PWM_PDD_FREE_RUNNING);
 *      @endcode
 */
#define PWM_PDD_SetOneShotModeX(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_FREE_RUNNING) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_ONESHOTX_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_ONESHOTX_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetOneShotModeA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the free running or one shot mode is selected.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the One Shot Mode constants." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetOneShotModeA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOneShotModeA(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_ONESHOTA_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOneShotModeB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the free running or one shot mode is selected.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the One Shot Mode constants." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetOneShotModeB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOneShotModeB(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_ONESHOTB_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetOneShotModeX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the free running or one shot mode is selected.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of "Determines the One Shot Mode constants." type.
 *         The value is cast to "uint16_t".
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetOneShotModeX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOneShotModeX(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_ONESHOTX_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableArmA
   ---------------------------------------------------------------------------- */

/**
 * @brief Enabling starts the input A capture process. This function can be
 * disabled at any time to disable input capture operation. This bit is self-cleared
 * when in one shot mode and one or more of the enabled capture circuits has had
 * a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Starts the input capture process. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableArmA(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableArmA(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_ARMA_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLA_ARMA_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableArmB
   ---------------------------------------------------------------------------- */

/**
 * @brief Enabling starts the input B capture process. This function can be
 * disabled at any time to disable input capture operation. This bit is self-cleared
 * when in one shot mode and one or more of the enabled capture circuits has had
 * a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Starts the input capture process. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableArmB(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableArmB(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_ARMB_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLB_ARMB_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableArmX
   ---------------------------------------------------------------------------- */

/**
 * @brief Enabling starts the input X capture process. This function can be
 * disabled at any time to disable input capture operation. This bit is self-cleared
 * when in one shot mode and one or more of the enabled capture circuits has had
 * a capture event.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Starts the input capture process. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableArmX(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableArmX(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_ARMX_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
       PWM_CAPTCTRLX_ARMX_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetArmAEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input A capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetArmAEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetArmAEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLA_ARMA_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetArmBEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input B capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetArmBEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetArmBEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLB_ARMB_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetArmXEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input X capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetArmXEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetArmXEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCTRLX_ARMX_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EdgeCounterA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the the edge counter value for the PWM_A input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_EdgeCounterA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_EdgeCounterA(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCOMPA_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCOMPA_EDGCNTA_MASK \
     )) >> ( \
     PWM_CAPTCOMPA_EDGCNTA_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EdgeCounterB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the the edge counter value for the PWM_B input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_EdgeCounterB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_EdgeCounterB(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCOMPB_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCOMPB_EDGCNTB_MASK \
     )) >> ( \
     PWM_CAPTCOMPB_EDGCNTB_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- EdgeCounterX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the the edge counter value for the PWM_X input capture
 * circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_EdgeCounterX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_EdgeCounterX(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_CAPTCOMPX_REG(PeripheralBase,(SmIndex)), \
      PWM_CAPTCOMPX_EDGCNTX_MASK \
     )) >> ( \
     PWM_CAPTCOMPX_EDGCNTX_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeCompareA
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the compare value associated with the edge counter for the
 * PWM_A input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines the compare value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeCompareA(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetEdgeCompareA(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCOMPA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPA_EDGCMPA_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeCompareB
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the compare value associated with the edge counter for the
 * PWM_B input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines the compare value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeCompareB(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetEdgeCompareB(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCOMPB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPB_EDGCMPB_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetEdgeCompareX
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines the compare value associated with the edge counter for the
 * PWM_X input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines the compare value. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetEdgeCompareX(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetEdgeCompareX(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_CAPTCOMPX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPX_EDGCMPX_MASK, \
     (uint16_t)(Value) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCompareA
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the compare value associated with the edge counter for the
 * PWM_A input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPA[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeCompareA(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCompareA(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CAPTCOMPA_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPA_EDGCMPA_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCompareB
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the compare value associated with the edge counter for the
 * PWM_B input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPB[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeCompareB(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCompareB(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CAPTCOMPB_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPB_EDGCMPB_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetEdgeCompareX
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the compare value associated with the edge counter for the
 * PWM_X input capture circuitry.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPX[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetEdgeCompareX(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetEdgeCompareX(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CAPTCOMPX_REG(PeripheralBase,(SmIndex)), \
     PWM_CAPTCOMPX_EDGCMPX_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue0
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLX[EDGX0]. Each capture will increase
 * the value of CAPTCTRLX[CX0CNT] by 1 until the maximum value is reached. Each
 * read of this register will decrease the value of CAPTCTRLX[CX0CNT] by 1 until 0
 * is reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue0(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue0(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLX[EDGX1]. Each capture increases the
 * value of CAPTCTRLX[CX1CNT] by 1 until the maximum value is reached. Each read
 * of this register decreases the value of CAPTCTRLX[CX1CNT] by 1 until 0 is
 * reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue1(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLA[EDGA0]. Each capture increases the
 * value of CAPTCTRLA[CA0CNT] by 1 until the maximum value is reached. Each read
 * of this register decreases the value of CAPTCTRLA[CA0CNT] by 1 until 0 is
 * reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue2(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue2(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLA[EDGA1]. Each capture increases the
 * value of CAPTCTRLA[CA1CNT] by 1 until the maximum value is reached. Each read
 * of this register decreases the value of CAPTCTRLA[CA1CNT] by 1 until 0 is
 * reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue3(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue3(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL3_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLB[EDGB0]. Each capture increases the
 * value of CAPTCTRLB[CB0CNT] by 1 until the maximum value is reached. Each read
 * of this register decreases the value of CAPTCTRLB[CB0CNT] by 1 until 0 is
 * reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue4(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue4(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL4_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the value captured from the submodule counter. Exactly when
 * this capture occurs is defined by CAPTCTRLB[EDGB1]. Each capture increases the
 * value of CAPTCTRLB[CB1CNT] by 1 until the maximum value is reached. Each read
 * of this register decreases the value of CAPTCTRLB[CB1CNT] by 1 until 0 is
 * reached.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_CaptureValue5(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue5(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL5_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue1Cycle
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cycle number corresponding to the value captured in CVAL1.
 * This register is incremented each time the counter is loaded with the INIT
 * value at the end of a PWM modulo cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CVAL1CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureValue1Cycle(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue1Cycle(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CVAL1CYC_REG(PeripheralBase,(SmIndex)), \
     PWM_CVAL1CYC_CVAL1CYC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue2Cycle
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cycle number corresponding to the value captured in CVAL2.
 * This register is incremented each time the counter is loaded with the INIT
 * value at the end of a PWM modulo cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CVAL2CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureValue2Cycle(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue2Cycle(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CVAL2CYC_REG(PeripheralBase,(SmIndex)), \
     PWM_CVAL2CYC_CVAL2CYC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue3Cycle
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cycle number corresponding to the value captured in CVAL3.
 * This register is incremented each time the counter is loaded with the INIT
 * value at the end of a PWM modulo cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CVAL3CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureValue3Cycle(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue3Cycle(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CVAL3CYC_REG(PeripheralBase,(SmIndex)), \
     PWM_CVAL3CYC_CVAL3CYC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue4Cycle
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cycle number corresponding to the value captured in CVAL4.
 * This register is incremented each time the counter is loaded with the INIT
 * value at the end of a PWM modulo cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CVAL4CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureValue4Cycle(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue4Cycle(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CVAL4CYC_REG(PeripheralBase,(SmIndex)), \
     PWM_CVAL4CYC_CVAL4CYC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- CaptureValue5Cycle
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the cycle number corresponding to the value captured in CVAL5.
 * This register is incremented each time the counter is loaded with the INIT
 * value at the end of a PWM modulo cycle.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 4-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: CVAL5CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_CaptureValue5Cycle(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_CaptureValue5Cycle(PeripheralBase, SmIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_CVAL5CYC_REG(PeripheralBase,(SmIndex)), \
     PWM_CVAL5CYC_CVAL5CYC_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the PWM_A output of each submodule. These function
 * should be set to disabled when a PWM_A pin is being used for input capture.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables the PWM_A output. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableAOutput(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableAOutput(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMA_EN_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_OUTEN_PWMA_EN_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMA_EN_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the PWM_B output of each submodule. These function
 * should be set to disabled when a PWM_B pin is being used for input capture.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables the PWM_B output. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableBOutput(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableBOutput(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMB_EN_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_OUTEN_PWMB_EN_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMB_EN_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the PWM_X output of each submodule. These function
 * should be set to disabled when a PWM_X pin is being used for input capture.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables the PWM_X output. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableXOutput(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableXOutput(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMX_EN_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_OUTEN_PWMX_EN_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_OUTEN_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMX_EN_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetAOutputEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input A capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetAOutputEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetAOutputEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_OUTEN_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMA_EN_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBOutputEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input B capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetBOutputEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetBOutputEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_OUTEN_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMB_EN_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetXOutputEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the input X capture operation is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetXOutputEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetXOutputEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_OUTEN_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_OUTEN_PWMX_EN_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- OutputEnable
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables masked PWM_A/B/X outputs of each submodule.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Defines the "channel output masks" for enabling. Outputs which
 *        are not presented in the mask will be disabled. This parameter is a
 *        16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      PWM_PDD_OutputEnable(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_OutputEnable(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     PWM_OUTEN_REG(PeripheralBase), \
     (uint16_t)(PWM_OUTEN_PWMA_EN_MASK | (PWM_OUTEN_PWMB_EN_MASK | PWM_OUTEN_PWMX_EN_MASK)), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableAMask
   ---------------------------------------------------------------------------- */

/**
 * @brief If Enabled masks the PWM_A outputs of each submodule forcing the
 * output to logic 0 prior to consideration of the output polarity.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables or disables mask of the PWM_A output. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableAMask(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableAMask(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKA_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_MASK_MASKA_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKA_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableBMask
   ---------------------------------------------------------------------------- */

/**
 * @brief If Enabled masks the PWM_B outputs of each submodule forcing the
 * output to logic 0 prior to consideration of the output polarity.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables or disables mask of the PWM_B output. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableBMask(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableBMask(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKB_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_MASK_MASKB_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKB_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableXMask
   ---------------------------------------------------------------------------- */

/**
 * @brief If Enabled masks the PWM_X outputs of each submodule forcing the
 * output to logic 0 prior to consideration of the output polarity.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables or disables mask of the PWM_X output. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableXMask(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableXMask(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKX_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_MASK_MASKX_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_MASK_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKX_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetAMaskEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the mask of the PWM_A output is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetAMaskEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetAMaskEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_MASK_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKA_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetBMaskEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the mask of the PWM_B output is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetBMaskEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetBMaskEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_MASK_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKB_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetXMaskEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the mask of the PWM_X output is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetXMaskEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetXMaskEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_MASK_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_MASKX_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- OutputMasks
   ---------------------------------------------------------------------------- */

/**
 * @brief Masks the PWMA/B/X outputs of each submodule forcing the output to
 * logic 0 prior to consideration of the output polarity. If a desired submodule
 * should be updated immediately (without waiting for a FORCE_OUT event), the
 * "channel update immediately masks" could be added.
 * @param PeripheralBase Peripheral base address.
 * @param Mask Defines the "channel output masks" and the "channel update
 *        immediately masks". This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_OutputMasks(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_OutputMasks(PeripheralBase, Mask) ( \
    PeriphWriteReg16(PWM_MASK_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- UpdateImmediately
   ---------------------------------------------------------------------------- */

/**
 * @brief If Enabled masks the  outputs of each submodule forcing the output to
 * logic 0 prior to consideration of the output polarity.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_UpdateImmediately(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_UpdateImmediately(PeripheralBase, SmIndex) ( \
    PeriphSetBitMask16( \
     PWM_MASK_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MASK_UPDATE_MASK_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetSoftwareControlledOutput23
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which value is supplied to the deadtime generator of that
 * submodule when the "SOFTWARE_CONTROLED" state is selected by the
 * "SetOverride23ControlSelect".
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State The value is supplied to the deadtime generator of submodule
 *        instead of PWM23. This parameter is of "Determines the load mode select
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      PWM_PDD_SetSoftwareControlledOutput23(deviceID, periphID,
 *      PWM_PDD_LOGIC_ZERO);
 *      @endcode
 */
#define PWM_PDD_SetSoftwareControlledOutput23(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOGIC_ZERO) ? ( \
      PeriphSetBits16( \
       PWM_SWCOUT_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*2+PWM_SWCOUT_SM0OUT23_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex)*2+PWM_SWCOUT_SM0OUT23_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_SWCOUT_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*2+PWM_SWCOUT_SM0OUT23_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- SetSoftwareControlledOutput45
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which value is supplied to the deadtime generator of that
 * submodule when the "SOFTWARE_CONTROLED" state is selected by the
 * "SetOverride45ControlSelect".
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State The value is supplied to the deadtime generator of submodule
 *        instead of PWM45. This parameter is of "Determines the load mode select
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      PWM_PDD_SetSoftwareControlledOutput45(deviceID, periphID,
 *      PWM_PDD_LOGIC_ZERO);
 *      @endcode
 */
#define PWM_PDD_SetSoftwareControlledOutput45(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_LOGIC_ZERO) ? ( \
      PeriphSetBits16( \
       PWM_SWCOUT_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*2)), \
       (uint16_t)((uint16_t)0U << ((SmIndex)*2)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_SWCOUT_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*2)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetSoftwareControlledOutput23
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value which is supplied to the deadtime generator of submodule
 * instead of PWM23 when the "SOFTWARE_CONTROLED" state is selected by the
 * "SetOverride23ControlSelect".
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetSoftwareControlledOutput23(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetSoftwareControlledOutput23(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_SWCOUT_REG(PeripheralBase))) >> ( \
      (SmIndex)*2+PWM_SWCOUT_SM0OUT23_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- GetSoftwareControlledOutput45
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value which is supplied to the deadtime generator of submodule
 * instead of PWM45 when the "SOFTWARE_CONTROLED" state is selected by the
 * "SetOverride45ControlSelect".
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetSoftwareControlledOutput45(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetSoftwareControlledOutput45(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(PeriphReadReg16(PWM_SWCOUT_REG(PeripheralBase)) >> ((SmIndex)*2))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- SoftwareControlledOutput
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets masked PWM_A/B/X outputs to the deadtime generator to logic one.
 * Not masked outputs are set to logic zero. This value is only used when
 * "SetOverride23/45ControlSelect" is set to "SOFTWARE_CONTROLED".
 * @param PeripheralBase Peripheral base address.
 * @param Mask Defines the "software controlled channel output masks" for
 *        setting to logic one. Outputs which are not presented in the mask will be set
 *        to logic zero. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      PWM_PDD_SoftwareControlledOutput(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_SoftwareControlledOutput(PeripheralBase, Mask) ( \
    PeriphSetBits16( \
     PWM_SWCOUT_REG(PeripheralBase), \
     (uint16_t)(( \
      PWM_SWCOUT_SM0OUT23_MASK) | (( \
      PWM_SWCOUT_SM0OUT45_MASK) | (( \
      PWM_SWCOUT_SM1OUT23_MASK) | (( \
      PWM_SWCOUT_SM1OUT45_MASK) | (( \
      PWM_SWCOUT_SM2OUT23_MASK) | (( \
      PWM_SWCOUT_SM2OUT45_MASK) | (( \
      PWM_SWCOUT_SM3OUT23_MASK) | ( \
      PWM_SWCOUT_SM3OUT45_MASK)))))))), \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetOverride23ControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which signal is supplied to the deadtime generator of that
 * submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State The signal is supplied to the deadtime generator of submodule
 *        instead of PWM23. This parameter is of "Determines the deadtime logic
 *        inputs constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      PWM_PDD_SetOverride23ControlSelect(deviceID, periphID,
 *      PWM_PDD_GENERATED_SIGNAL);
 *      @endcode
 */
#define PWM_PDD_SetOverride23ControlSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_GENERATED_SIGNAL) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)) \
      )) : (((State) == PWM_PDD_INVERTED_GENERATED) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)) \
      )) : (((State) == PWM_PDD_SOFTWARE_CONTROLED) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)), \
       (uint16_t)((uint16_t)0x2U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT)) \
      )) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- SetOverride45ControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines which signal is supplied to the deadtime generator of that
 * submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State The signal is supplied to the deadtime generator of submodule
 *        instead of PWM45. This parameter is of "Determines the deadtime logic
 *        inputs constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      PWM_PDD_SetOverride45ControlSelect(deviceID, periphID,
 *      PWM_PDD_GENERATED_SIGNAL);
 *      @endcode
 */
#define PWM_PDD_SetOverride45ControlSelect(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_GENERATED_SIGNAL) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4)), \
       (uint16_t)((uint16_t)0U << ((SmIndex)*4)) \
      )) : (((State) == PWM_PDD_INVERTED_GENERATED) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4)), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex)*4)) \
      )) : (((State) == PWM_PDD_SOFTWARE_CONTROLED) ? ( \
      PeriphSetBits16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4)), \
       (uint16_t)((uint16_t)0x2U << ((SmIndex)*4)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_DTSRCSEL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x3U << ((SmIndex)*4)) \
      )) \
    )) \
  )

/* ----------------------------------------------------------------------------
   -- GetOverride23ControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns currently selected signal which is supplied to the deadtime
 * generator of submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetOverride23ControlSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOverride23ControlSelect(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_DTSRCSEL_REG(PeripheralBase))) >> ( \
      (SmIndex)*4+PWM_DTSRCSEL_SM0SEL23_SHIFT))) & ( \
     0x3U)) \
  )

/* ----------------------------------------------------------------------------
   -- GetOverride45ControlSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns currently selected signal which is supplied to the deadtime
 * generator of submodule.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 2-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetOverride45ControlSelect(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetOverride45ControlSelect(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(PeriphReadReg16(PWM_DTSRCSEL_REG(PeripheralBase)) >> ((SmIndex)*4))) & ( \
     0x3U)) \
  )

/* ----------------------------------------------------------------------------
   -- DeadtimeSourceSelect
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines source signal of deadtime logic for each output channel.
 * @param PeripheralBase Peripheral base address.
 * @param Mask The "Deadtime Source Select masks" could be ORed together to
 *        select each channel. When a channel mask is ommited the common pwm output
 *        is selected. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      PWM_PDD_DeadtimeSourceSelect(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_DeadtimeSourceSelect(PeripheralBase, Mask) ( \
    PeriphWriteReg16(PWM_DTSRCSEL_REG(PeripheralBase), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetCurrentPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Is used to select between PWM23 and PWM45 as the source for the
 * generation of the complementary PWM pair output. MCTRL[IPOL] is ignored in
 * independent mode.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Determines the source for the generation of the complementary
 *        PWM pair output. This parameter is of "Determines the Current Polarity
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      PWM_PDD_SetCurrentPolarity(deviceID, periphID,
 *      PWM_PDD_COMPLEMENTARY_PWM23);
 *      @endcode
 */
#define PWM_PDD_SetCurrentPolarity(PeripheralBase, SmIndex, State) ( \
    ((State) == PWM_PDD_COMPLEMENTARY_PWM23) ? ( \
      PeriphSetBits16( \
       PWM_MCTRL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_IPOL_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_MCTRL_IPOL_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_MCTRL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_IPOL_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetCurrentPolarity
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the source for the generation of the complementary PWM pair
 * output.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetCurrentPolarity(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetCurrentPolarity(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_MCTRL_REG(PeripheralBase))) >> ( \
      (SmIndex) + PWM_MCTRL_IPOL_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableRun
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the clocks to the PWM generator.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Enables the clocks to the PWM generator. This parameter is of
 *        "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      PWM_PDD_EnableRun(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableRun(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_MCTRL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_RUN_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((SmIndex) + PWM_MCTRL_RUN_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_MCTRL_REG(PeripheralBase), \
       (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_RUN_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetRunEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the clocks to the PWM generator is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetRunEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetRunEnabled(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_MCTRL_REG(PeripheralBase))) >> ( \
      (SmIndex) + PWM_MCTRL_RUN_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- LoadOkay
   ---------------------------------------------------------------------------- */

/**
 * @brief Loads CTRL[PRSC] and the INIT, FRACVALx, and VALx registers of the
 * corresponding submodule into a set of buffers.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_LoadOkay(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_LoadOkay(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_MCTRL_REG(PeripheralBase)), \
    PeriphSetBitMask16( \
     PWM_MCTRL_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_LDOK_SHIFT) ) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- ClearLoadOkay
   ---------------------------------------------------------------------------- */

/**
 * @brief Is used to clear MCTRL[LDOK].
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ClearLoadOkay(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ClearLoadOkay(PeripheralBase, SmIndex) ( \
    PeriphSetBitMask16( \
     PWM_MCTRL_REG(PeripheralBase), \
     (uint16_t)((uint16_t)0x1U << ((SmIndex) + PWM_MCTRL_CLDOK_SHIFT)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultLevel
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the active logic level of the individual fault inputs.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State The value on the fault input indicates a fault condition. This
 *        parameter is of "Determines the load mode select constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultLevel(deviceID, periphID, periphID, PWM_PDD_LOGIC_ZERO);
 *      @endcode
 */
#define PWM_PDD_SetFaultLevel(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PWM_PDD_LOGIC_ZERO) ? ( \
      PeriphSetBits16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FLVL_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FCTRL_FLVL_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FLVL_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultLevel
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value which determines the active logic level of the
 * individual fault inputs.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultLevel(deviceID, periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultLevel(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FCTRL_FLVL_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultClearing
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines automatic or manual clearing of faults.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State The value select automatic or manual clearing of faults. This
 *        parameter is of "Fault Clearing constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultClearing(deviceID, periphID, periphID,
 *      PWM_PDD_MANUAL_CLEARING);
 *      @endcode
 */
#define PWM_PDD_SetFaultClearing(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PWM_PDD_MANUAL_CLEARING) ? ( \
      PeriphSetBits16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FAUTO_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FCTRL_FAUTO_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FAUTO_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultClearing
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value which determines automatic or manual clearing of faults.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultClearing(deviceID, periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultClearing(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FCTRL_FAUTO_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Determines the safety mode during manual fault clearing.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State The value select normal or safe clearing of faults. This
 *        parameter is of "Fault Safety Mode constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultMode(deviceID, periphID, periphID, PWM_PDD_NORMAL_MODE);
 *      @endcode
 */
#define PWM_PDD_SetFaultMode(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PWM_PDD_NORMAL_MODE) ? ( \
      PeriphSetBits16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FSAFE_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FCTRL_FSAFE_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FSAFE_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultMode
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value which determines normal or safe clearing of faults.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultMode(deviceID, periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultMode(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FCTRL_FSAFE_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFaultInterrupt
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables CPU interrupt requests generated by the FAULTx
 * pins.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State The value enables or disables fault interrupt. This parameter is
 *        of "Global enumeration used for specifying general enable/disable
 *        states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFaultInterrupt(deviceID, periphID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFaultInterrupt(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSetBits16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FIE_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FCTRL_FIE_SHIFT)) \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FCTRL_FIE_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultInterruptEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the CPU interrupt requests generated by the FAULTx pins is
 * enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultInterruptEnabled(deviceID, periphID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultInterruptEnabled(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FCTRL_FIE_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFaultInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables fault interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFaultInterrupts(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_EnableFaultInterrupts(PeripheralBase, FChIndex, Mask) ( \
    PeriphSetBitMask16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- DisableFaultInterrupts
   ---------------------------------------------------------------------------- */

/**
 * @brief Disables fault interrupts defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_DisableFaultInterrupts(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_DisableFaultInterrupts(PeripheralBase, FChIndex, Mask) ( \
    PeriphClearBitMask16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)), (uint16_t)(Mask)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets all fault interrupts with value according to mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Mask of interrupts to enable. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultInterruptMask(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFaultInterruptMask(PeripheralBase, FChIndex, Mask) ( \
    PeriphSetBits16( \
     PWM_FCTRL_REG(PeripheralBase,(FChIndex)), \
     PWM_FCTRL_FIE_MASK, \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultInterruptMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns fault interrupts mask.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFaultInterruptMask(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultInterruptMask(PeripheralBase, FChIndex) ( \
    PeriphGetBitMask16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)), PWM_FCTRL_FIE_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns fault interrupt flag bits.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFaultInterruptFlags(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultInterruptFlags(PeripheralBase, FChIndex) ( \
    PeriphGetBitMask16(PWM_FSTS_REG(PeripheralBase,(FChIndex)), PWM_FSTS_FFLAG_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- ClearFaultInterruptFlags
   ---------------------------------------------------------------------------- */

/**
 * @brief Clears fault interrupt flag bits defined by mask parameter.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Mask of interrupt's flags to clear. This parameter is a 16-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_ClearFaultInterruptFlags(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_ClearFaultInterruptFlags(PeripheralBase, FChIndex, Mask) ( \
    PeriphSetBits16( \
     PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
     PWM_FSTS_FFLAG_MASK, \
     (uint16_t)(Mask) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableHalfCycleFaultRecovery
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the re-enabling the PWM outputs after a fault
 * condition. These bits apply to both automatic and manual clearing of a fault
 * condition.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State Enables the re-enabling at the start of a half cycle. This
 *        parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableHalfCycleFaultRecovery(deviceID, periphID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableHalfCycleFaultRecovery(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeSetBits16( \
       PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
       PWM_FSTS_FFLAG_MASK, \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FSTS_FHALF_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FSTS_FHALF_SHIFT)) \
      )) : ( \
      PeriphSafeSetBitMask16( \
       PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
       PWM_FSTS_FFLAG_MASK, \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FSTS_FHALF_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetHalfCycleFaultRecoveryEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the re-enabling at the start of a half cycle is enabled or
 * disabled.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetHalfCycleFaultRecoveryEnabled(deviceID,
 *      periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetHalfCycleFaultRecoveryEnabled(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FSTS_FHALF_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- HalfCycleFaultRecoveryMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Is used to control the timing for re-enabling the PWM outputs after a
 * fault condition. These bits apply to both automatic and manual clearing of a
 * fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Defines the fault input by mask. Inputs contained in the mask are
 *        set by function. This parameter is a 16-bit value.
 * @param Value Defines the value which specifies the re-enabling at the start
 *        of a half cycle. The value is obtained as OR of values for each input
 *        together. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_HalfCycleFaultRecoveryMask(deviceID, periphID, 1, 1);
 *      @endcode
 */
#define PWM_PDD_HalfCycleFaultRecoveryMask(PeripheralBase, FChIndex, Mask, Value) ( \
    PeriphWriteReg16( \
     PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
     (uint16_t)(( \
      (uint16_t)(( \
       PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)))) & ( \
       (uint16_t)(~(uint16_t)((uint16_t)((uint16_t)(Mask) & PWM_FSTS_FHALF_MASK) | PWM_FSTS_FFLAG_MASK))))) | ( \
      (uint16_t)((uint16_t)(Mask) & (PWM_FSTS_FHALF_MASK & (uint16_t)(Value))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFullCycleFaultRecovery
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the re-enabling the PWM outputs after a full
 * cycle. These bits apply to both automatic and manual clearing of a fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @param State Enables the re-enabling the PWM outputs after a fault condition.
 *        This parameter is of "Global enumeration used for specifying general
 *        enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFullCycleFaultRecovery(deviceID, periphID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFullCycleFaultRecovery(PeripheralBase, FChIndex, FInIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphSafeSetBits16( \
       PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
       PWM_FSTS_FFLAG_MASK, \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FSTS_FFULL_SHIFT)), \
       (uint16_t)((uint16_t)0U << ((FInIndex) + PWM_FSTS_FFULL_SHIFT)) \
      )) : ( \
      PeriphSafeSetBitMask16( \
       PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
       PWM_FSTS_FFLAG_MASK, \
       (uint16_t)((uint16_t)0x1U << ((FInIndex) + PWM_FSTS_FFULL_SHIFT)) \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFullCycleFaultRecoveryEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the re-enabling the PWM outputs after a fault condition is
 * enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFullCycleFaultRecoveryEnabled(deviceID,
 *      periphID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFullCycleFaultRecoveryEnabled(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FSTS_FFULL_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- FullCycleFaultRecoveryMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Is used to control the timing for re-enabling the PWM outputs after a
 * fault condition. These bits apply to both automatic and manual clearing of a
 * fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Mask Defines the fault input by mask. Inputs contained in the mask are
 *        set by function. This parameter is a 16-bit value.
 * @param Value Defines the value which specifies the re-enabling a fault
 *        condition. The value is obtained as OR of values for each input together.
 *        This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_FullCycleFaultRecoveryMask(deviceID, periphID, 1, 1);
 *      @endcode
 */
#define PWM_PDD_FullCycleFaultRecoveryMask(PeripheralBase, FChIndex, Mask, Value) ( \
    PeriphWriteReg16( \
     PWM_FSTS_REG(PeripheralBase,(FChIndex)), \
     (uint16_t)(( \
      (uint16_t)(( \
       PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)))) & ( \
       (uint16_t)(~(uint16_t)((uint16_t)(Mask) & PWM_FSTS_FFULL_MASK))))) | ( \
      (uint16_t)((uint16_t)(Mask) & (PWM_FSTS_FFULL_MASK & (uint16_t)(Value))))) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- FilteredFaultPins
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current state of the filtered FAULTx pins converted to
 * high polarity. A logic 1 indicates a fault condition exists on the filtered
 * FAULTx pin.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param FInIndex Fault input index. This parameter is of index type.
 * @return Returns a 1-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_FilteredFaultPins(deviceID, periphID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_FilteredFaultPins(PeripheralBase, FChIndex, FInIndex) ( \
    (uint8_t)(( \
     (uint16_t)(( \
      PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)))) >> ( \
      (FInIndex) + PWM_FSTS_FFPIN_SHIFT))) & ( \
     0x1U)) \
  )

/* ----------------------------------------------------------------------------
   -- FilteredFaultPinsMask
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the current state of the filtered FAULTx pins converted to
 * high polarity. A logic 1 indicates a fault condition exists on the filtered
 * FAULTx pin. The FAULTx_PIN_MASK ate used to test whether the returned value
 * contains fault condition on FAULTx pin.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_FilteredFaultPinsMask(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_FilteredFaultPinsMask(PeripheralBase, FChIndex) ( \
    PeriphGetBitMask16(PWM_FSTS_REG(PeripheralBase,(FChIndex)), PWM_FSTS_FFPIN_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFaultGlitchStretch
   ---------------------------------------------------------------------------- */

/**
 * @brief Enables or disables the fault glitch stretching logic. This logic
 * ensures that narrow fault glitches are stretched to be at least 2 IPBus clock
 * cycles wide. In some cases a narrow fault input can cause problems due to the
 * short PWM output shutdown/re-activation time. The stretching logic ensures that a
 * glitch on the fault input, when the fault filter is disabled, will be
 * registered in the fault flags.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param State Parameter specifying if the fault glitch stretching logic will
 *        be enabled or disabled. This parameter is of "Global enumeration used
 *        for specifying general enable/disable states (PDD_DISABLE and PDD_ENABLE
 *        defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFaultGlitchStretch(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFaultGlitchStretch(PeripheralBase, FChIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
       PWM_FFILT_GSTR_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
       PWM_FFILT_GSTR_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultGlitchStretchEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the fault glitch stretching logic is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFaultGlitchStretchEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultGlitchStretchEnabled(PeripheralBase, FChIndex) ( \
    PeriphGetBitMask16( \
     PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
     PWM_FFILT_GSTR_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultFilterPeriod
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the glitch filter count.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Period Defines the number of IP Bus clock cycles for which the DAC
 *        output is held unchanged after new data is presented to the analog DAC's
 *        inputs. This parameter is a 8-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultFilterPeriod(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFaultFilterPeriod(PeripheralBase, FChIndex, Period) ( \
    PeriphSetBits16( \
     PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
     PWM_FFILT_FILT_PER_MASK, \
     (uint16_t)(Period) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultFilterPeriod
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the glitch filter count. The value specifies the number of IP
 * Bus clock cycles for which the DAC output is held unchanged after new data is
 * presented to the analog DAC's inputs.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 8-bit value.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultFilterPeriod(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultFilterPeriod(PeripheralBase, FChIndex) ( \
    (uint8_t)PeriphGetBitMask16( \
     PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
     PWM_FFILT_FILT_PER_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFaultFilterCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Sets the glitch filter count.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Count Defines the number of IP Bus clock cycles for which the DAC
 *        output is held unchanged after new data is presented to the analog DAC's
 *        inputs. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFaultFilterCount(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFaultFilterCount(PeripheralBase, FChIndex, Count) ( \
    PeriphSetBits16( \
     PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
     PWM_FFILT_FILT_CNT_MASK, \
     (uint16_t)(( \
      (uint16_t)((uint16_t)((uint16_t)(Count) - 0x03U) << PWM_FFILT_FILT_CNT_SHIFT)) & ( \
      PWM_FFILT_FILT_CNT_MASK)) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultFilterCount
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the glitch filter count. The value specifies the number of IP
 * Bus clock cycles for which the DAC output is held unchanged after new data is
 * presented to the analog DAC's inputs.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 3-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFaultFilterCount(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultFilterCount(PeripheralBase, FChIndex) ( \
    (uint16_t)((uint16_t)(( \
     PeriphGetBitMask16( \
      PWM_FFILT_REG(PeripheralBase,(FChIndex)), \
      PWM_FFILT_FILT_CNT_MASK \
     )) >> ( \
     PWM_FFILT_FILT_CNT_SHIFT)) + 0x03U) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFaultTest
   ---------------------------------------------------------------------------- */

/**
 * @brief When set to enable, the causes a simulated fault to be sent into all
 * of the fault filters. The condition propagates to the fault flags and possibly
 * the PWM outputs depending on the DISMAPn settings. Disabling removes the
 * simulated fault condition.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param State Parameter specifying if the simulated fault will be enabled or
 *        disabled. This parameter is of "Global enumeration used for specifying
 *        general enable/disable states (PDD_DISABLE and PDD_ENABLE defined in
 *        PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FTST[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFaultTest(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFaultTest(PeripheralBase, FChIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FTST_REG(PeripheralBase,(FChIndex)), \
       PWM_FTST_FTEST_MASK \
      )) : ( \
      PeriphSetBitMask16(PWM_FTST_REG(PeripheralBase,(FChIndex)), PWM_FTST_FTEST_MASK)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFaultTestEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns if the simulated fault is enabled or disabled.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FTST[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFaultTestEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFaultTestEnabled(PeripheralBase, FChIndex) ( \
    PeriphGetBitMask16(PWM_FTST_REG(PeripheralBase,(FChIndex)), PWM_FTST_FTEST_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMasterControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Master Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      PWM_PDD_WriteMasterControlReg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteMasterControlReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_MCTRL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMasterControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Master Control Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MCTRL.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadMasterControlReg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadMasterControlReg(PeripheralBase) ( \
    PeriphReadReg16(PWM_MCTRL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Counter Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CNT[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCounterReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCounterReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CNT_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCounterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Counter Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CNT[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCounterReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCounterReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CNT_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInitialCountReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Initial Count Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INIT[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteInitialCountReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteInitialCountReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_INIT_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInitialCountReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Initial Count Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INIT[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadInitialCountReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadInitialCountReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_INIT_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Control 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteControl2Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteControl2Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CTRL2_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Control 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadControl2Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadControl2Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CTRL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteControlReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteControlReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadControlReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadControlReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CTRL_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL0[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue0Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue0Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL0_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue1Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue1Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 2.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue2Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue2Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL2_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 3.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL3[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue3Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue3Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL3_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 4.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL4[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue4Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue4Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL4_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Value Register 5.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: VAL5[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteValue5Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteValue5Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_VAL5_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue0Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue0Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue1Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue1Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 2.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue2Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue2Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 3.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue3Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue3Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL3_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 4.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue4Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue4Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL4_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Value Register 5.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: VAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadValue5Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadValue5Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_VAL5_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOutputControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Output Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteOutputControlReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteOutputControlReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_OCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOutputControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Output Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: OCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadOutputControlReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadOutputControlReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_OCTRL_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Status Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: STS[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteStatusReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteStatusReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_STS_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Status Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: STS[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadStatusReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadStatusReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_STS_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Interrupt Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteInterruptEnableReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteInterruptEnableReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_INTEN_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadInterruptEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Interrupt Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: INTEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadInterruptEnableReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadInterruptEnableReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_INTEN_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDmaEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes DMA Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteDmaEnableReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteDmaEnableReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_DMAEN_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDmaEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the DMA Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DMAEN[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadDmaEnableReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadDmaEnableReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DMAEN_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOutputTriggerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Output Trigger Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteOutputTriggerControlReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteOutputTriggerControlReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_TCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOutputTriggerControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Output Trigger Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: TCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadOutputTriggerControlReg(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_ReadOutputTriggerControlReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_TCTRL_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultDisableMapping0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Disable Mapping Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DISMAP[SmIndex][].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultDisableMapping0Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultDisableMapping0Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_DISMAP_REG(PeripheralBase,(SmIndex),0U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultDisableMapping1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Disable Mapping Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DISMAP[SmIndex][].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultDisableMapping1Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultDisableMapping1Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_DISMAP_REG(PeripheralBase,(SmIndex),1U), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultDisableMapping0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Disable Mapping Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DISMAP[SmIndex][].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultDisableMapping0Reg(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultDisableMapping0Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DISMAP_REG(PeripheralBase,(SmIndex),0U)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultDisableMapping1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Disable Mapping Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DISMAP[SmIndex][].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultDisableMapping1Reg(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultDisableMapping1Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DISMAP_REG(PeripheralBase,(SmIndex),1U)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDeadtimeCount0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Deadtime Count Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DTCNT0[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteDeadtimeCount0Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteDeadtimeCount0Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_DTCNT0_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteDeadtimeCount1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Deadtime Count Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: DTCNT1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteDeadtimeCount1Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteDeadtimeCount1Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_DTCNT1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDeadtimeCount0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Deadtime Count Register 0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DTCNT0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadDeadtimeCount0Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadDeadtimeCount0Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DTCNT0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadDeadtimeCount1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Deadtime Count Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: DTCNT1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadDeadtimeCount1Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadDeadtimeCount1Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_DTCNT1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureControlAReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Control A Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureControlAReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureControlAReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureControlBReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Control B Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureControlBReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureControlBReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureControlXReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Control X Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureControlXReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureControlXReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureControlAReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Control A Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureControlAReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureControlAReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCTRLA_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureControlBReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Control B Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureControlBReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureControlBReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCTRLB_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureControlXReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Control X Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCTRLX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureControlXReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureControlXReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCTRLX_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureCompareAReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Compare A Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPA[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureCompareAReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureCompareAReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCOMPA_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureCompareBReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Compare B Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPB[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureCompareBReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureCompareBReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCOMPB_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureCompareXReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Compare X Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CAPTCOMPX[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureCompareXReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureCompareXReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CAPTCOMPX_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureCompareAReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Compare A Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPA[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureCompareAReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureCompareAReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCOMPA_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureCompareBReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Compare B Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPB[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureCompareBReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureCompareBReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCOMPB_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureCompareXReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Compare X Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CAPTCOMPX[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureCompareXReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureCompareXReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CAPTCOMPX_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 0 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL0[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue0Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue0Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL0_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 1 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue1Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue1Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue2Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue2Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL2_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 3 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL3[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue3Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue3Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL3_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 4 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL4[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue4Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue4Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL4_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 5 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL5[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue5Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue5Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL5_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue0Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 0 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL0[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue0Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue0Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL0_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 1 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue1Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue1Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue2Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue2Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 3 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue3Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue3Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL3_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 4 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue4Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue4Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL4_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 5 Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue5Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue5Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL5_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue0CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 0 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL0CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue0CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue0CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL0CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue1CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 1 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL1CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue1CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue1CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL1CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue2CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 2 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL2CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue2CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue2CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL2CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue3CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 3 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL3CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue3CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue3CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL3CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue4CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 4 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL4CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue4CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue4CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL4CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteCaptureValue5CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Capture Value 5 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: CVAL5CYC[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteCaptureValue5CycleReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteCaptureValue5CycleReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_CVAL5CYC_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue0CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 0 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL0CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue0CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue0CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL0CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue1CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 1 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL1CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue1CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue1CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL1CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue2CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 2 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL2CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue2CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue2CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL2CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue3CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 3 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL3CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue3CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue3CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL3CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue4CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 4 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL4CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue4CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue4CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL4CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadCaptureValue5CycleReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Capture Value 5 Cycle Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: CVAL5CYC[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadCaptureValue5CycleReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadCaptureValue5CycleReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_CVAL5CYC_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteOutputEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Output Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      PWM_PDD_WriteOutputEnableReg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteOutputEnableReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_OUTEN_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadOutputEnableReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Output Enable Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_OUTEN.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadOutputEnableReg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadOutputEnableReg(PeripheralBase) ( \
    PeriphReadReg16(PWM_OUTEN_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMaskReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Mask Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      PWM_PDD_WriteMaskReg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteMaskReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_MASK_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMaskReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Mask Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MASK.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadMaskReg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadMaskReg(PeripheralBase) ( \
    PeriphReadReg16(PWM_MASK_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteSoftwareControlledOutputReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Software Controlled Output Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      PWM_PDD_WriteSoftwareControlledOutputReg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteSoftwareControlledOutputReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_SWCOUT_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadSoftwareControlledOutputReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Software Controlled Output Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_SWCOUT.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadSoftwareControlledOutputReg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadSoftwareControlledOutputReg(PeripheralBase) ( \
    PeriphReadReg16(PWM_SWCOUT_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WritePWMSourceSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes PWM Source Select Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      PWM_PDD_WritePWMSourceSelectReg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WritePWMSourceSelectReg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_DTSRCSEL_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadPWMSourceSelectReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the PWM Source Select Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_DTSRCSEL.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadPWMSourceSelectReg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadPWMSourceSelectReg(PeripheralBase) ( \
    PeriphReadReg16(PWM_DTSRCSEL_REG(PeripheralBase)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultControlReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultControlReg(PeripheralBase, FChIndex, Value) ( \
    PeriphWriteReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FCTRL[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultControlReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultControlReg(PeripheralBase, FChIndex) ( \
    PeriphReadReg16(PWM_FCTRL_REG(PeripheralBase,(FChIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Status Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultStatusReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultStatusReg(PeripheralBase, FChIndex, Value) ( \
    PeriphWriteReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultStatusReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Status Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FSTS[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultStatusReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultStatusReg(PeripheralBase, FChIndex) ( \
    PeriphReadReg16(PWM_FSTS_REG(PeripheralBase,(FChIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Filter Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultFilterReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultFilterReg(PeripheralBase, FChIndex, Value) ( \
    PeriphWriteReg16(PWM_FFILT_REG(PeripheralBase,(FChIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultFilterReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Filter Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FFILT[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultFilterReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultFilterReg(PeripheralBase, FChIndex) ( \
    PeriphReadReg16(PWM_FFILT_REG(PeripheralBase,(FChIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFaultTestReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fault Test Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FTST[FChIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFaultTestReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFaultTestReg(PeripheralBase, FChIndex, Value) ( \
    PeriphWriteReg16(PWM_FTST_REG(PeripheralBase,(FChIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFaultTestReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fault Test Register.
 * @param PeripheralBase Peripheral base address.
 * @param FChIndex Fault cannel index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FTST[FChIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFaultTestReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFaultTestReg(PeripheralBase, FChIndex) ( \
    PeriphReadReg16(PWM_FTST_REG(PeripheralBase,(FChIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFractionalDelayCircuit
   ---------------------------------------------------------------------------- */

/**
 * @brief Function is used to power up the fractional delay analog block. The
 * fractional delay block takes 25 us to power up after the first FRAC_PU bit in
 * any submodule is set. The fractional delay block only powers down when the
 * FRAC_PU bits in all submodules are 0. The fractional delay logic can only be used
 * when the IPBus clock is running at 100 MHz. When turned off, fractional
 * placement is disabled.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the fractional delay analog block
 *        will be enabled (powered up) or disabled (turned off). This parameter is
 *        of "Global enumeration used for specifying general enable/disable states
 *        (PDD_DISABLE and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFractionalDelayCircuit(deviceID, periphID, PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFractionalDelayCircuit(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC_PU_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC_PU_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalDelayCircuitEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of fractional delay analog block.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetFractionalDelayCircuitEnabled(deviceID,
 *      periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalDelayCircuitEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_FRCTRL_FRAC_PU_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFractionalCyclePWM_APlacement
   ---------------------------------------------------------------------------- */

/**
 * @brief Function is used to enable the fractional cycle edge placement of
 * PWM_A using the FRACVAL2 and FRACVAL3 registers. When disabled, the fractional
 * cycle edge placement of PWM_A is bypassed.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the fractional cycle placement for
 *        PWM_A will be enabled or disabled. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFractionalCyclePWM_APlacement(deviceID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFractionalCyclePWM_APlacement(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC23_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC23_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFractionalCyclePWM_BPlacement
   ---------------------------------------------------------------------------- */

/**
 * @brief Function is used to enable the fractional cycle edge placement of
 * PWM_B using the FRACVAL4 and FRACVAL5 registers. When disabled, the fractional
 * cycle edge placement of PWM_B is bypassed.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the fractional cycle placement for
 *        PWM_A will be enabled or disabled. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE and
 *        PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFractionalCyclePWM_BPlacement(deviceID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFractionalCyclePWM_BPlacement(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC45_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC45_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- EnableFractionalCyclePWM_PeriodPlacement
   ---------------------------------------------------------------------------- */

/**
 * @brief Function is used to enable the fractional cycle edge placement of the
 * FRACVAL1 register. When disabled, the fractional cycle length of the PWM
 * period is bypassed.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param State Parameter is specifying if the fractional cycle length for the
 *        PWM period will be enabled or disabled. This parameter is of "Global
 *        enumeration used for specifying general enable/disable states (PDD_DISABLE
 *        and PDD_ENABLE defined in PDD_Types.h)" type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_EnableFractionalCyclePWM_PeriodPlacement(deviceID, periphID,
 *      PDD_DISABLE);
 *      @endcode
 */
#define PWM_PDD_EnableFractionalCyclePWM_PeriodPlacement(PeripheralBase, SmIndex, State) ( \
    ((State) == PDD_DISABLE) ? ( \
      PeriphClearBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC1_EN_MASK \
      )) : ( \
      PeriphSetBitMask16( \
       PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
       PWM_FRCTRL_FRAC1_EN_MASK \
      )) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalCyclePWM_APlacementEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of the fractional cycle placement for PWM_A.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      PWM_PDD_GetFractionalCyclePWM_APlacementEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalCyclePWM_APlacementEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_FRCTRL_FRAC23_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalCyclePWM_BPlacementEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of the fractional cycle placement for PWM_A.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      PWM_PDD_GetFractionalCyclePWM_BPlacementEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalCyclePWM_BPlacementEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_FRCTRL_FRAC45_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalCyclePWM_PeriodPlacementEnabled
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of the fractional cycle length for the PWM
 * period.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result =
 *      PWM_PDD_GetFractionalCyclePWM_PeriodPlacementEnabled(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalCyclePWM_PeriodPlacementEnabled(PeripheralBase, SmIndex) ( \
    PeriphGetBitMask16( \
     PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), \
     PWM_FRCTRL_FRAC1_EN_MASK \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalValue1
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines a fractional addition to the value in the VAL1 register which
 * controls the PWM period width. The PWM period is computed in terms of IPBus
 * clock cycles. This fractional portion is accumulated at the end of every cycle
 * until an additional whole IPBus cycle is reached. At this time the value being
 * used for VAL1 is temporarily incremented and the PWM cycle is extended by one
 * clock period to compensate for the accumulated fractional values.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 1 value. This parameter is a 5-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFractionalValue1(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFractionalValue1(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_FRACVAL1_REG(PeripheralBase,(SmIndex)), \
     PWM_FRACVAL1_FRACVAL1_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_FRACVAL1_FRACVAL1_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalValue1
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the Fractional Value 1 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FRACVAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFractionalValue1(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalValue1(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_FRACVAL1_REG(PeripheralBase,(SmIndex)), \
      PWM_FRACVAL1_FRACVAL1_MASK \
     )) >> ( \
     PWM_FRACVAL1_FRACVAL1_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalValue2
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines a fractional addition to the value in the VAL2 register which
 * controls the PWM_A turn on timing. It is also used to control the fractional
 * addition to the turn off delay of PWM_B when MCTRL[IPOLx]=0 in complementary
 * mode, CTRL2[INDEP]=0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 2 value. This parameter is a 5-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFractionalValue2(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFractionalValue2(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_FRACVAL2_REG(PeripheralBase,(SmIndex)), \
     PWM_FRACVAL2_FRACVAL2_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_FRACVAL2_FRACVAL2_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalValue3
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines a fractional addition to the value in the VAL3 register which
 * controls the PWM_A turn off timing. It is also used to control the fractional
 * addition to the turn on delay of PWM_B when MCTRL[IPOLx]=0 in complementary
 * mode, CTRL2[INDEP]=0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 3 value. This parameter is a 5-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL3[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFractionalValue3(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFractionalValue3(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_FRACVAL3_REG(PeripheralBase,(SmIndex)), \
     PWM_FRACVAL3_FRACVAL3_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_FRACVAL3_FRACVAL3_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalValue4
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines a fractional addition to the value in the VAL4 register which
 * controls the PWM_B turn on timing. It is also used to control the fractional
 * addition to the turn off delay of PWM_A when MCTRL[IPOLx]=0 in complementary
 * mode, CTRL2[INDEP]=0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 4 value. This parameter is a 5-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL4[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFractionalValue4(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFractionalValue4(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_FRACVAL4_REG(PeripheralBase,(SmIndex)), \
     PWM_FRACVAL4_FRACVAL4_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_FRACVAL4_FRACVAL4_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- SetFractionalValue5
   ---------------------------------------------------------------------------- */

/**
 * @brief Defines a fractional addition to the value in the VAL5 register which
 * controls the PWM_B turn off timing. It is also used to control the fractional
 * addition to the turn on delay of PWM_A when MCTRL[IPOLx]=0 in complementary
 * mode, CTRL2[INDEP]=0.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Defines Fractional Value 5 value. This parameter is a 5-bit
 *        value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL5[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_SetFractionalValue5(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_SetFractionalValue5(PeripheralBase, SmIndex, Value) ( \
    PeriphSetBits16( \
     PWM_FRACVAL5_REG(PeripheralBase,(SmIndex)), \
     PWM_FRACVAL5_FRACVAL5_MASK, \
     (uint16_t)((uint16_t)(Value) << PWM_FRACVAL5_FRACVAL5_SHIFT) \
    ) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalValue2
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the Fractional Value 2 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FRACVAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFractionalValue2(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalValue2(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_FRACVAL2_REG(PeripheralBase,(SmIndex)), \
      PWM_FRACVAL2_FRACVAL2_MASK \
     )) >> ( \
     PWM_FRACVAL2_FRACVAL2_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalValue3
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the Fractional Value 3 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FRACVAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFractionalValue3(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalValue3(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_FRACVAL3_REG(PeripheralBase,(SmIndex)), \
      PWM_FRACVAL3_FRACVAL3_MASK \
     )) >> ( \
     PWM_FRACVAL3_FRACVAL3_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalValue4
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the Fractional Value 4 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FRACVAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFractionalValue4(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalValue4(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_FRACVAL4_REG(PeripheralBase,(SmIndex)), \
      PWM_FRACVAL4_FRACVAL4_MASK \
     )) >> ( \
     PWM_FRACVAL4_FRACVAL4_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- GetFractionalValue5
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns the Fractional Value 5 value.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 5-bit value. The value is cast to "uint8_t".
 * @remarks The macro accesses the following registers: FRACVAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint8_t result = PWM_PDD_GetFractionalValue5(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_GetFractionalValue5(PeripheralBase, SmIndex) ( \
    (uint8_t)(( \
     PeriphGetBitMask16( \
      PWM_FRACVAL5_REG(PeripheralBase,(SmIndex)), \
      PWM_FRACVAL5_FRACVAL5_MASK \
     )) >> ( \
     PWM_FRACVAL5_FRACVAL5_SHIFT)) \
  )

/* ----------------------------------------------------------------------------
   -- SetMonitorPLL
   ---------------------------------------------------------------------------- */

/**
 * @brief DMA enable control.
 * @param PeripheralBase Peripheral base address.
 * @param State Parameter specifying if DMA requests will be enabled or
 *        disabled. This parameter is of "Independent or complementary pair operation
 *        constants." type.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MCTRL2.
 * @par Example:
 *      @code
 *      PWM_PDD_SetMonitorPLL(deviceID, PWM_PDD_NOT_MONITOR_NOT_LOCKED);
 *      @endcode
 */
#define PWM_PDD_SetMonitorPLL(PeripheralBase, State) ( \
    PeriphWriteReg16(PWM_MCTRL2_REG(PeripheralBase), (uint16_t)(State)) \
  )

/* ----------------------------------------------------------------------------
   -- GetMonitorPLL
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns current state of DMA enable bit.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MCTRL2.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_GetMonitorPLL(deviceID);
 *      @endcode
 */
#define PWM_PDD_GetMonitorPLL(PeripheralBase) ( \
    PeriphGetBitMask16(PWM_MCTRL2_REG(PeripheralBase), PWM_MCTRL2_MONPLL_MASK) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalControlReg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalControlReg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRCTRL_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalControlReg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Control Register.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRCTRL[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalControlReg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalControlReg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRCTRL_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Value Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL1[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalValue1Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalValue1Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRACVAL1_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Value Register 2.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL2[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalValue2Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalValue2Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRACVAL2_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Value Register 3.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL3[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalValue3Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalValue3Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRACVAL3_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Value Register 4.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL4[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalValue4Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalValue4Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRACVAL4_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- WriteFractionalValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Fractional Value Register 5.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: FRACVAL5[SmIndex].
 * @par Example:
 *      @code
 *      PWM_PDD_WriteFractionalValue5Reg(deviceID, periphID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteFractionalValue5Reg(PeripheralBase, SmIndex, Value) ( \
    PeriphWriteReg16(PWM_FRACVAL5_REG(PeripheralBase,(SmIndex)), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalValue1Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Value Register 1.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRACVAL1[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalValue1Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalValue1Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRACVAL1_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalValue2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Value Register 2.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRACVAL2[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalValue2Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalValue2Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRACVAL2_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalValue3Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Value Register 3.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRACVAL3[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalValue3Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalValue3Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRACVAL3_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalValue4Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Value Register 4.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRACVAL4[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalValue4Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalValue4Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRACVAL4_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- ReadFractionalValue5Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Fractional Value Register 5.
 * @param PeripheralBase Peripheral base address.
 * @param SmIndex PWM submodule index. This parameter is of index type.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: FRACVAL5[SmIndex].
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadFractionalValue5Reg(deviceID, periphID);
 *      @endcode
 */
#define PWM_PDD_ReadFractionalValue5Reg(PeripheralBase, SmIndex) ( \
    PeriphReadReg16(PWM_FRACVAL5_REG(PeripheralBase,(SmIndex))) \
  )

/* ----------------------------------------------------------------------------
   -- WriteMasterControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Writes Master Control 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @param Value Register value. This parameter is a 16-bit value.
 * @return Returns a value of void type.
 * @remarks The macro accesses the following registers: PWMA_MCTRL2.
 * @par Example:
 *      @code
 *      PWM_PDD_WriteMasterControl2Reg(deviceID, 1);
 *      @endcode
 */
#define PWM_PDD_WriteMasterControl2Reg(PeripheralBase, Value) ( \
    PeriphWriteReg16(PWM_MCTRL2_REG(PeripheralBase), (uint16_t)(Value)) \
  )

/* ----------------------------------------------------------------------------
   -- ReadMasterControl2Reg
   ---------------------------------------------------------------------------- */

/**
 * @brief Returns value of the Master Control 2 Register.
 * @param PeripheralBase Peripheral base address.
 * @return Returns a 16-bit value.
 * @remarks The macro accesses the following registers: PWMA_MCTRL2.
 * @par Example:
 *      @code
 *      uint16_t result = PWM_PDD_ReadMasterControl2Reg(deviceID);
 *      @endcode
 */
#define PWM_PDD_ReadMasterControl2Reg(PeripheralBase) ( \
    PeriphReadReg16(PWM_MCTRL2_REG(PeripheralBase)) \
  )
#endif  /* #if defined(PWM_PDD_H_) */

/* PWM_PDD.h, eof. */
