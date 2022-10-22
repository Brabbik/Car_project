/*
 * motor.c
 *
 *  Created on: 29. 9. 2020
 *      Author: brabenec, pohorsky
 */

#include "include/motor.h"

void M_init(void){
    P8DIR = BIT2 + BIT3;        //pin 2 and pin 3 set output
    P8SEL = BIT3;               //pin 3 is special function
    M_DIR_1();                  //set motor direction

    // setup timer TA0
    TA0CTL |= MC__UP;           //up mode
    TA0CTL |= TASSEL__SMCLK;    //chose clk 16 MHz
    TA0CTL |= ID__1;            //ID divider /1
    TA0CCTL3 |= OUTMOD_7;       //set mode Reset/Set
    TA0CCR0 = COUNTER_1KHZ;     //set frequency of PWM to 1 kHz
    //TA0CCR3 = 160 * 50;         //set actual duty cycle
}

void M_set_duty_cycle(uint8_t d_cycle){
    TA0CCR3 = 160 * d_cycle;    //set actual duty cycle
}
