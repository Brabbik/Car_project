/*
 * LED.c
 *
 *  Created on: 26. 9. 2022
 *      Author: brabenec, pohorsky
 */

#include "include/LED.h"

void LED_init(void)
{
    P1DIR |= BIT2 + BIT3;  //  P1.2 and P1.3 as output
    P4DIR |= BIT2;  //  P4.2 as output
    P3DIR |= BIT6;  //  P3.6 as output
    LED_FL_OFF();
    LED_FR_OFF();
    LED_RL_OFF();
    LED_RR_OFF();
}
