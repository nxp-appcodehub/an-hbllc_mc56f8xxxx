/** ###################################################################
**      Filename    : PE_Error.h
**      Processor   : MC56F82748VLH
**      Version     : 1.0
** 
**     Copyright : 1997 - 2014 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     SPDX-License-Identifier: BSD-3-Clause
**     */

#ifndef __PE_Error_H
#define __PE_Error_H

#ifdef __cplusplus
extern "C" {
#endif

#define ERR_OK                          0U /* OK */
#define ERR_SPEED                       1U /* This device does not work in the active speed mode. */
#define ERR_RANGE                       2U /* Parameter out of range. */
#define ERR_VALUE                       3U /* Parameter of incorrect value. */
#define ERR_OVERFLOW                    4U /* Timer overflow. */
#define ERR_MATH                        5U /* Overflow during evaluation. */
#define ERR_ENABLED                     6U /* Device is enabled. */
#define ERR_DISABLED                    7U /* Device is disabled. */
#define ERR_BUSY                        8U /* Device is busy. */
#define ERR_NOTAVAIL                    9U /* Requested value or method not available. */
#define ERR_RXEMPTY                     10U /* No data in receiver. */
#define ERR_TXFULL                      11U /* Transmitter is full. */
#define ERR_BUSOFF                      12U /* Bus not available. */
#define ERR_OVERRUN                     13U /* Overrun error is detected. */
#define ERR_FRAMING                     14U /* Framing error is detected. */
#define ERR_PARITY                      15U /* Parity error is detected. */
#define ERR_NOISE                       16U /* Noise error is detected. */
#define ERR_IDLE                        17U /* Idle error is detected. */
#define ERR_FAULT                       18U /* Fault error is detected. */
#define ERR_BREAK                       19U /* Break char is received during communication. */
#define ERR_CRC                         20U /* CRC error is detected. */
#define ERR_ARBITR                      21U /* A node losts arbitration. This error occurs if two nodes start transmission at the same time. */
#define ERR_PROTECT                     22U /* Protection error is detected. */
#define ERR_UNDERFLOW                   23U /* Underflow error is detected. */
#define ERR_UNDERRUN                    24U /* Underrun error is detected. */
#define ERR_COMMON                      25U /* Common error of a device. */
#define ERR_LINSYNC                     26U /* LIN synchronization error is detected. */
#define ERR_FAILED                      27U /* Requested functionality or process failed. */
#define ERR_QFULL                       28U /* Queue is full. */
#define ERR_PARAM_MASK                  128U /* Invalid mask. */
#define ERR_PARAM_MODE                  129U /* Invalid mode. */
#define ERR_PARAM_INDEX                 130U /* Invalid index. */
#define ERR_PARAM_DATA                  131U /* Invalid data. */
#define ERR_PARAM_SIZE                  132U /* Invalid size. */
#define ERR_PARAM_VALUE                 133U /* Invalid value. */
#define ERR_PARAM_RANGE                 134U /* Invalid parameter's range or parameters' combination. */
#define ERR_PARAM_LOW_VALUE             135U /* Invalid value (LOW part). */
#define ERR_PARAM_HIGH_VALUE            136U /* Invalid value (HIGH part). */
#define ERR_PARAM_ADDRESS               137U /* Invalid address. */
#define ERR_PARAM_PARITY                138U /* Invalid parity. */
#define ERR_PARAM_WIDTH                 139U /* Invalid width. */
#define ERR_PARAM_LENGTH                140U /* Invalid length. */
#define ERR_PARAM_ADDRESS_TYPE          141U /* Invalid address type. */
#define ERR_PARAM_COMMAND_TYPE          142U /* Invalid command type. */
#define ERR_PARAM_COMMAND               143U /* Invalid command. */
#define ERR_PARAM_RECIPIENT             144U /* Invalid recipient. */
#define ERR_PARAM_BUFFER_COUNT          145U /* Invalid buffer count. */
#define ERR_PARAM_ID                    146U /* Invalid ID. */
#define ERR_PARAM_GROUP                 147U /* Invalid group. */
#define ERR_PARAM_CHIP_SELECT           148U /* Invalid chip select. */
#define ERR_PARAM_ATTRIBUTE_SET         149U /* Invalid set of attributes. */
#define ERR_PARAM_SAMPLE_COUNT          150U /* Invalid sample count. */
#define ERR_PARAM_CONDITION             151U /* Invalid condition. */
#define ERR_PARAM_TICKS                 152U /* Invalid ticks parameter. */

#ifdef __cplusplus
}
#endif

#endif __PE_Error_H
