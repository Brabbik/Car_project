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
    UCB0CTL1 |= UCSWRST;          // Put state machine in reset state
    UCB0CTL0 |= UCMST;            // master mode
    UCB0CTL0 |= UCSYNC;           // synchronous mode
    UCB0CTL0 |= UCMODE_0;         // 4pin SPI, Slave Enable active in 0
    UCB0CTL0 |= UCMSB;            // MSB first
    UCB0CTL0 |= UCCKPH;           //
    UCB0CTL1 |= UCSSEL_2;         // SMCLK clock source
    UCB0BRW = 0x0004;              //
    UCB0CTL1 &= ~UCSWRST;         // Release state machine from reset state
}
/*
 *  SPI_read_byte - reads byte at the address specified by addr parameter
 *  addr - address for the data reading
 *  returns - uint8 value received by SPI
 */
uint8_t SPI_read_byte(uint8_t addr)
{
    uint8_t data;
    P3OUT &= ~0x01;                 // CS to LOW
    UCB0TXBUF = addr;               // Transmit first character - address to be read
    SPI_delay();
    UCB0TXBUF = addr;               // Transmit second character - dummy byte
    SPI_delay();
    SPI_delay();
    data = UCB0RXBUF;
    P3OUT |= 0x01;                  // CS to HIGH
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
    P3OUT &= ~0x01;                 // CS to LOW
    UCB0TXBUF = addr;               // Transmit first character - address to be read
    SPI_delay();
    UCB0TXBUF = data;               // Transmit second character - data byte
    SPI_delay();
    SPI_delay();
    P3OUT |= 0x01;                  // CS to HIGH
}

void SPI_delay(void)
{
    uint8_t i;
    for (i = 0; i <1 ; i++);
}
