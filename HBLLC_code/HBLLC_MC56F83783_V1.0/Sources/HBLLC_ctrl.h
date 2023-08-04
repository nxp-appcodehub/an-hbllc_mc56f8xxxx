/*
* Copyright 2021 NXP
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * HBLLC_Ctrl.h
 *
 *  Created on: Nov 12, 2021
 *      Author: nxa16823
 */

#ifndef HBLLC_CTRL_H_
#define HBLLC_CTRL_H_

/******************************************************************************
* Macros 
******************************************************************************/

/************************************** conditional compilation **************************************/
#define PWM            0  /* no fractional addition to PWM values, when it's 1, PWM_HIGH_RES must be zero */
#define PWM_HIGH_RES   1  /* high resolution PWM, enable fractional circuit,when it's 1, PWM must be zero */

#define SR_ENABLE               1  /* synchronous rectification control */
#define CTRL_FRE_CHANGE_ENABLE  1  /* control and sampling frequency change control */
#define COMM_WITH_PFC_ENABLE    0  /* communicate with front stage control */

/************************************** Control mode ***************************************/
#define OPEN_LOOP               0x01  /* open loop control, in this condition, set all control loops as zero */
#define SINGLE_VOLTAGE_LOOP     0x02  /* output voltage single loop control */
#define DUAL_VOLTAGE_LOOP       0x04  /* output voltage outer loop, primary side resonant current inner loop, dual loop control */
#define DUAL_OUTER_LOOP         0x08  /* output voltage and current dual outer loop,primary side resonant current inner loop, dual loop control */

/*************************************** Voltage&Current Scales ************************************************/
#define VOUT_SCALE        19.8   /* MAX measurable output voltage[V] */
#define IOUT_SCALE        57.0   /* MAX measurable output current[A] */
#define IPRIM_SCALE       5.32   /* MAX measurable primary side resonant cavity current[A] */
#define VPRIM_SCALE       430.8  /* MAX measurable transformer primary side voltage[V] */ 

/*************************************** Fault thresholds ************************************************/
#define VOUT_UP_LIMIT           14   /* MAX output voltage[V] */
#define VOUT_LOW_LIMIT          9    /* MIN output voltage[V] */
#define IOUT_OVER_20_PERCENT    24   /* Overload 20% output current[A] */
#define IOUT_OVER_50_PERCENT    30   /* Overload 50% output current[A] */
#define IOUT_LIMIT              35   /* MAX output current[A] */
#define IPRIM_LIMIT             4    /* MAX primary side resonant cavity current[A] */
#define VPRIM_LOW_LIMIT         330  /* MIN applicable input bus voltage[V] */
#define VPRIM_UP_LIMIT          400  /* MAX applicable input bus voltage[V] */

/*************************************** load connect control ************************************************/
#define LOAD_ON_ACCEPT_ERR      FRAC16(0.1/19.8) /* Compensate state error to ensure load connect*/
#define LOAD_OFF_VOUT           11               /* voltage to disconnect load in stop condition[V] */

/**************************************** frequency limit ***********************************************/
#define FREQ_OPEN_LOOP          200.0  /* open loop switch frequency[kHz] */
#define FREQ_LIM1               200.0  /* MAX switch frequency when control frequency is twice of PWM frequency[kHz]*/ 
#define FREQ_LIM2               100.0  /* MAX switch frequency when control frequency equals PWM frequency[kHz] */ 
#define FREQ_MAX_NORMALRUN      199.0  /* MAX switch frequency in normal run[kHz] */
#define FREQ_MAX_SOFTSTART      250.0  /* MAX switch frequency in soft-start[kHz] */
#define FREQ_MIN                80.0   /* MIN switch frequency[kHz] */
#define FREQ_START              250.0  /* soft_start starting switching frequency[kHz] */

#define CTRLOUT_TO_PEIOD_GAIN   1000   /* controller output to PWM period (PWM VAL register value) gain */

/**************************************** duty limit ***********************************************/
#define DUTY_CYCLE_MAX          0.5    /* MAX allowable duty cycle */ 
#define DUTY_CYCLE_MIN          0.3    /* MIN allowable duty cycle */
#define BURST_ON_DUTY_CYCLE_HYS 0.32   /* when calculated duty cycle is greater than this hysteresis in burst off, switch to burst on */  

/**************************************** resonant cavity Current control ***********************************************/
#define IPRIM_KP            0.1
#define IPRIM_KI            0.023
#define IPRIM_UP_LIMIT()    FRAC16(100/(2*FREQ_MIN)) /* in dual loop mode, iprim pi result*CTRLOUT_TO_PEIOD_GAIN=PWM VAL1/2, so the up limit is PWM_CLK_FREQ/(1000*(2*FREQ/MIN)) */
#define IPRIM_LOW_LIMIT()   FRAC16(DUTY_CYCLE_MIN/2) /* in dual loop mode, duty cycle=2*iprim pi result */

#define IPRIM_REF_LIMIT     FRAC16(0.52)

/*************************************** Voltage(VOUT) control ************************************************/
#define VOUT_REF       12 /* required output voltage[V] */
#define VOUT_SOFT_END  10 /* soft_start loop required output voltage, when reached, switch to normal run[V] */

/* VOUT control parameters in single loop control mode, 1p1z controller */
#define SINGLE_LOOP_VOUT_KP          6.0 
#define SINGLE_LOOP_VOUT_KI          0.2
#define SINGLE_LOOP_VOUT_UP_LIMIT()  FRAC16(100/(2*FREQ_MIN)) /* in single loop mode, vout pi result*CTRLOUT_TO_PEIOD_GAIN=PWM VAL1/2, so the up limit is PWM_CLK_FREQ/(1000*(2*FREQ_MIN))=100/(2*FREQ_MIN) */
#define SINGLE_LOOP_VOUT_LOW_LIMIT() FRAC16(DUTY_CYCLE_MIN/2) /* in single loop mode, duty cycle=2*vout pi result */

/* VOUT control parameters in dual control mode, 2p2z controller */
#define DUAL_LOOP_VOUT_COEFF_B0      0.89
#define DUAL_LOOP_VOUT_COEFF_B1      -0.001
#define DUAL_LOOP_VOUT_COEFF_B2      -0.835
#define DUAL_LOOP_VOUT_COEFF_A1      0.9
#define DUAL_LOOP_VOUT_COEFF_A2      -0.4

/**************************************** Output Current control ***********************************************/
#define IOUT_REF         22 /* required output current[A]*/

/* output current PI controller parameters*/
#define IOUT_KP         4.0//12.0
#define IOUT_KI         0.083
#define IOUT_UP_LIMIT   0.0
#define IOUT_LOW_LIMIT  -0.52 /* -IPRIM_REF_LIMIT *

/**************************************** Application time base ***********************************************/
#define T_5MS         5
#define T_20MS        20
#define T_5S          5000
#define T_10S         10000

#define DT_LIMIT      40 /* MIN allowable deadtime, deadtime = DT_LIMIT * BUS CLOCK = 400ns */
#define SR_OFFDELAY   10 /* SR switch off delay time = SR_OFFDELAY *BUS CLOCK  = 100ns */

#define LLC_MESSAGE_TRANS_DURATION              200  /* [ms], LLC transmit message to front end about every this time */
#define LLC_RECEIVE_MESSAGE_RESTART_DURATION    12   /* [ms], when the time interval between two received message is greater than this
                                                         time, the next received message is considered to be the head of the new message */
/**************************************** Fault control ***********************************************/
#define VOUT_OVER      0x1  /* VOUT over voltage */
#define VOUT_LOW       0x20 /* VOUT under voltage */
#define IOUT_OVER      0x2  /* IOUT over current */
#define IPRIM_OVER     0x4  /* IPRIM over current */
#define IOUT_OVER20    0x8  /* IOUT overload 20% */
#define IOUT_OVER50    0x10 /* IOUT overload 50% */ 
#define HW_IPRIMOVER   0x40 /* IPRIM over current - HW protection */

#define FAULT_SET(faults, faultid)    (faults |= faultid)  /* Sets the fault defined by faultid in the faults variable */
#define FAULT_CLEAR(faults, faultid)  (faults &= ~faultid) /* Clears the fault defined by faultid in the faults variable */
#define FAULT_ALL_CLEAR(faults)       faults = 0; /* Clears all faults in the faults variable */      


#endif /* HBLLC_CTRL_H_ */
