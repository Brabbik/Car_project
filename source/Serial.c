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
    P5DIR |= BIT6;
    P5SEL |= BIT6;                        // parita, clock, nemusime
    // LOW BYTE
    UCA1CTL1 |= UCSWRST;;               // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL__SMCLK;          // CLK = SMCLK
    // HIGH BYTE
    UCA1CTL0 = 0x00;

    //BRW LOW BYTE
    //UCA1BR0 = 0x08;                     // 16 MHz/115200 = 138.9 (see User's Guide)
    //BRW HIGH BYTE
    UCA1BRW = 0x0008;

    UCA1MCTL |= (UCBRS0 | UCBRS1);                 // Modulation UCBRSx=3, UCBRFx=0
    UCA1MCTL |= UCBRF0;
    UCA1CTL1 &= ~UCSWRST;               // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                   // Enable USCI_A1 RX interrupt
}
