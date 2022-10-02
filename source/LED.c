/*
 * LED.c
 *
 *  Created on: 24. 9. 2020
 *      Author: dosedel
 */

#include "include/LED.h"

void LED_init(void)
{
    P1DIR |= 0x0C;  //  P1.2 and P1.3 as output
    P4DIR |= 0x04;  //  P4.2 as output
    P3DIR |= 0x40;  //  P3.6 as output
    LED_FL_OFF();
    LED_FR_OFF();
    LED_RL_OFF();
    LED_RR_OFF();
}
