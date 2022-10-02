/*
 * motor.h
 *
 *  Created on: 29. 9. 2020
 *      Author: dosedel
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <msp430.h>
// macros definition

#define H_CONTROL_PORT P8OUT
#define H_FB_PORT P6OUT

#define H_STATUS_PORT P8IN
//#define H_BRAKE_PORT P8OUT
//#define H_ENABLE_PORT P8OUT
//#define H_PHASE_PORT P8OUT


// LED pins definition

#define H_BRAKE_PIN 0x20    // 5 bit
#define H_STATUS_PIN 0x10   // 4 bit
#define H_ENABLE_PIN 0x08   // 3 bit
#define H_PHASE_PIN 0x04    // 2 bit
#define H_FB_PIN 0x08       // 4 bit

void M_init(void);

// functions prototypes

#define M_START() (H_CONTROL_PORT |= H_ENABLE_PIN)
#define M_STOP() (H_CONTROL_PORT &= ~H_ENABLE_PIN)
#define M_DIR_1() (H_CONTROL_PORT |= H_PHASE_PIN)   // dopredu
#define M_DIR_2() (H_CONTROL_PORT &= ~H_PHASE_PIN)  // dozadu

// variables

#endif /* MOTOR_H_ */
