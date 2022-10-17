/*
 * Serial.c
 *
 *  Created on: 10. 10. 2022
 *      Author: pbrsa
 */

#include "include/Serial.h"

void SerialWrite(void)
{
     UCA1TXBUF = 0xaa;
}

void SerialInit(void){

    P5SEL |= BIT6;                        // parita, clock, nemusime
    UCA1CTL1 |= UCSWRST;;               // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL__SMCLK;          // CLK = SMCLK
    UCA1BR0 = 0x8a;                     // 16 MHz/115200 = 138.9 (see User's Guide)
    UCA1BR1 = 0x00;
    UCA1MCTL |= (UCBRS0 | UCBRS1);                 // Modulation UCBRSx=3, UCBRFx=0
    UCA1MCTL |= UCBRF0;
    UCA1CTL1 &= ~UCSWRST;               // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                   // Enable USCI_A1 RX interrupt
}
