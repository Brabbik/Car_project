/*
 * ADC.c
 *
 *  Created on: 24. 9. 2020
 *      Author: dosedel
 */

#include "include/ADC.h"

void ADC_init(void)
{
    P6SEL |= 0xF8;      // P6.3 - P6.7 as ADC

    ADC12CTL0 = ADC12ON + ADC12MSC + ADC12SHT0_2; // Turn on ADC12, set sampling time, fs is roughly 20kHz
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1; // Use sampling timer, single sequence
    ADC12MCTL3 = ADC12INCH_3;                 // ref+=AVcc, channel = A3
    ADC12MCTL4 = ADC12INCH_4;                 // ref+=AVcc, channel = A4
    ADC12MCTL5 = ADC12INCH_5;                 // ref+=AVcc, channel = A5
    ADC12MCTL6 = ADC12INCH_6;                 // ref+=AVcc, channel = A6
    ADC12MCTL7 = ADC12INCH_7 + ADC12EOS;    // ref+=AVcc, channel = A7, end seq.
    ADC12IE = ADC12IE7;                       // Enable ADC12IFG.7
    ADC12CTL0 |= ADC12ENC;                    // Enable conversions
}
