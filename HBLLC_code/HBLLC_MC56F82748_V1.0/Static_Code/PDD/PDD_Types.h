/* Common type declarations for PDD layer */
/* Copyright 2010 Freescale, Inc. 
  SPDX-License-Identifier: BSD-3-Clause*/
/* This file is static and it is generated from API-Factory */

#ifndef PDD_TYPES_H_
#define PDD_TYPES_H_

/* --------------------------------------------------------------- */
/* --------- PDD_TBool */
/* --------------------------------------------------------------- */

/* Implementation note for boolean type: */
/* Native "bool" type should be used for boolean values */
/* because the compiler provides optimization and best type representation. */
/* If the compiler does not support native "bool" type, common "Bool" */
/* at each place there should be used the most suitable integer type */
/* and no explicit conversion to 0 and 1 values should be used. */
/* Common "TBool" typedef could have possible performance issue */
/* (there is a problem how wide type to select). It is good to avoid using */
/* explicit "!= 0" because of performance penalty, but without such */
/* explicit conversion to 0/1 the resulting type may be too wide. */
/* Every bool type (native or replaced by integer value) should be always */
/* treated as zero-equal or non-zero (see below for examples). */

/* E.g.: This compiler supports boolean type only in C++ mode. */
/* Out of the C++ mode the boolean type must be simulated. */

#ifndef __cplusplus
typedef uint16_t PDD_TBool;
#else
typedef bool PDD_TBool;
#endif

/* ============================================================================
   ================== General PDD macros ======================================
   ============================================================================ */

#define PDD_DISABLE 0u
#define PDD_ENABLE  1u

#endif  /* #if !defined(PDD_TYPES_H_) */
