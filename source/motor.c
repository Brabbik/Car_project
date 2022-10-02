/*
 * motor.c
 *
 *  Created on: 29. 9. 2020
 *      Author: dosedel
 */

#include "include/motor.h"

void M_init(void){
    P8DIR |= 0x0C;
}
