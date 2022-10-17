/*
 * motor.h
 *
 *  Created on: 29. 9. 2020
 *      Author: brabenec, pohorsky
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <msp430.h>
#include <stdint.h>
// macros definition

#define COUNTER_1KHZ 16000      //  5ms ~ 100 Hz      16000 no divider ~ 1 kHz
#define H_CONTROL_PORT P8OUT
#define H_FB_PORT P6OUT



//#define H_BRAKE_PIN BIT5    // 5 bit
//#define H_STATUS_PIN BIT4   // 4 bit
//#define H_ENABLE_PIN BIT3   // 3 bit
#define H_PHASE_PIN BIT2      // 2 bit
//#define H_FB_PIN BIT3       // 3 bit

void M_init(void);
void M_set_duty_cycle(uint8_t d_cycle);

// functions prototypes

//#define M_START() (H_CONTROL_PORT |= H_ENABLE_PIN)
//#define M_STOP() (H_CONTROL_PORT &= ~H_ENABLE_PIN)
#define M_DIR_1() (H_CONTROL_PORT |= H_PHASE_PIN)   // dopredu
#define M_DIR_2() (H_CONTROL_PORT &= ~H_PHASE_PIN)  // dozadu

// variables

#endif /* MOTOR_H_ */
