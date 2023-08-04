/** ###################################################################
**      Filename    : PE_Const.h
**      Processor   : MC56F82748VLH
**      Version     : 1.0
** 
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     SPDX-License-Identifier: BSD-3-Clause
**     */

#ifndef __PE_Const_H
#define __PE_Const_H

#ifdef __cplusplus
extern "C" {
#endif

/* Constants for detecting running mode */
#define HIGH_SPEED        0x00U        /* High speed */
#define LOW_SPEED         0x01U        /* Low speed */
#define SLOW_SPEED        0x02U        /* Slow speed */

/* Reset cause constants */
#define RSTSRC_POR        0x04U        /* Power-on reset */
#define RSTSRC_PIN        0x08U        /* External reset bit */
#define RSTSRC_COP_LOR    0x10U        /* COP loss of reference reset */
#define RSTSRC_COP_CPU    0x20U        /* CPU time-out reset */
#define RSTSRC_COP_WINDOW 0x40U        /* COP window time-out reset */
#define RSTSRC_SWR        0x80U        /* Software reset */  

/* Memory modes */
#define INT_EXT_MODE      0x00U        /* Internal & external memory mode */
#define EXT_MODE          0x01U        /* External memory mode */

#ifdef __cplusplus
}
#endif

#endif /* _PE_Const_H */
