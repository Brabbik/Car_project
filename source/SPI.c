/*
 * ADXL355.c
 *
 *  Created on: 16. 9. 2020
 *      Author: dosedel
 */

// includes
#include "include/SPI.h"


// functions
/*
 *  SPI_init   -   initializes SPI interface at a speed of 4MHz
 *  returns - NA
 */
void SPI_init(void)
{
    // SPI at UCSI B
    // SPI pins setup
    P3SEL |= 0x0E;                  // PIN1-3 - functional module 1 (SPI module), CS controlled by SW
    P3DIR |= 0x01;                  // PIN0 - set to output direction (SE)
    P3OUT |= 0x01;                  // CS to HIGH

    // SPI module setup
    //UCB0CTLW0 |= 0x0001;
    UCB0CTL1 |= UCSWRST;          // Put state machine in reset state
    UCB0CTL0 |= UCMST;            // master mode
    UCB0CTL0 |= UCSYNC;           // synchronous mode
    UCB0CTL0 |= UCMODE_0;         // 4pin SPI, Slave Enable active in 0
    UCB0CTL0 |= UCMSB;            // MSB first
    //UCB0CTL0 |= UCCKPH;           // data is changed on the folowing edge
    UCB0CTL0 |= UCCKPL;           // inactive polarity of clock is high
    UCB0CTL1 |= UCSSEL_2;         // SMCLK clock source
    UCB0BRW = 0x0004;             // divide /4 16MHz / 4 = 4 MHz
    //UCB0IE |= UCTXIE;// + UCTXIE;    // turn on rx and tx interrupt

    UCB0CTL1 &= ~UCSWRST;         // Release state machine from reset state
    UCB0IE |= UCTXIE;// + UCTXIE;    // turn on rx and tx interrupt
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);    //clear rx tx interrupt flags
}
/*
 *  SPI_read_byte - reads byte at the address specified by addr parameter
 *  addr - address for the data reading
 *  returns - uint8 value received by SPI
 */
uint8_t SPI_read_byte(uint8_t addr)
{
    uint8_t data;
    uint8_t timeout = 40;
    UCB0IE &= ~UCTXIE;
    P3OUT &= ~0x01;                 // CS to LOW
    while(!(UCB0IFG & UCTXIFG) && timeout)
        timeout--;
    UCB0TXBUF = addr;               // Transmit first character - address to be read
    timeout = 40;
    while(!(UCB0IFG & UCTXIFG) && timeout)
        timeout--;
    UCB0TXBUF = 0x00;               // Transmit second character - dummy byte
    //SPI_delay();
    timeout = 40;
    while((UCB0STAT & UCBUSY) && timeout)
        timeout--;
    data = UCB0RXBUF;
    P3OUT |= 0x01;                  // CS to HIGH
    UCB0IE |= UCTXIE;// + UCTXIE;
    return data;
}

/*
 *  SPI_write_byte - writes byte to SPI bus
 *  addr - address of the data to be written
 *  data - data to be written at a specific addreess
 *  return - NA
 */
void SPI_write_byte(uint8_t addr, uint8_t data)
{
    uint8_t timeout = 40;
    UCB0IE &= ~UCTXIE;
    P3OUT &= ~0x01;                 // CS to LOW
    while(!(UCB0IFG & UCTXIFG) && timeout)
        timeout--;
    UCB0TXBUF = addr;               // Transmit first character - address to be read
    timeout = 40;
    while(!(UCB0IFG & UCTXIFG) && timeout)
        timeout--;
    UCB0TXBUF = data;               // Transmit second character - data byte
    timeout = 40;
    while((UCB0STAT & UCBUSY) && timeout)
        timeout--;
    P3OUT |= 0x01;                  // CS to HIGH
    UCB0IE |= UCTXIE;
}

void SPI_delay(void)
{
    uint8_t i;
    for (i = 0; i <100 ; i++);
}
