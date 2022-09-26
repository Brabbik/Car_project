/*
 * I2C.c
 *
 *  Created on: 22. 9. 2020
 *      Author: dosedel
 */

// includes
#include "include/I2C.h"


// functions
/*
 * I2C_Init - initializes I2C interface at a speed of 400 kHz
 */
void I2C_init(uint8_t slave_addr)
{
    P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK
    UCB0BR0 = 40;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = slave_addr;                        // Slave Address is 048h
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

}

/* uint8_t I2C_read_byte(uint8_t r_adr)
 * desc: function reads the value from I2C slave from the register with dedicated address
 *
 * params:
 *      r_adr - address of the register to be read from the slave
 *      count - number of data bytes to be read (max. I2C_RX_BUFFER_SIZE is allowed !!!)
 * return:
 *      NA, but fills RX_buffer in interrupt
 */
void I2C_read_byte(uint8_t r_adr, uint8_t count)
{
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C start condition
    while(!(UCB0IFG & UCTXIFG));            // data transfered
    UCB0TXBUF = r_adr;
    while(!(UCB0IFG & UCTXIFG));            // data transfered
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    while((UCB0CTL1 & UCTXSTP));            // stop condition ended
    I2C_delay();
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
    UCB0CTL1 &= ~UCTR;                      // receiver (= READ data from slave)

    RXByteCtr = count;                      // number of bytes to be read
    UCB0IE |= UCRXIE;                       // waiting for reception
    // polling method
    /*
    while(!(UCB0IFG & UCRXIFG));          // data transfered
    RX_buffer[0] = UCB0RXBUF;
    while(!(UCB0IFG & UCRXIFG));          // data transfered
    RX_buffer[1] = UCB0RXBUF;
    while(!(UCB0IFG & UCRXIFG));          // data transfered
    RX_buffer[2] = UCB0RXBUF;
    while(!(UCB0IFG & UCRXIFG));          // data transfered
    RX_buffer[3] = UCB0RXBUF;

    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    while((UCB0CTL1 & UCTXSTP));            // stop condition ended
    RX_buffer[4] = UCB0RXBUF;*/
}

/* void I2C_write_byte(uint8_t r_adr, uint8_t data)
 * desc: function writes the data to the register with dedicated address
 *
 * params:
 *      r_adr - register address inside the slave
 *      data - data to be written to the register with r_adr
 * return:
 *      NA
 */
void I2C_write_byte(uint8_t r_adr, uint8_t data)
{
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C start condition
    while(!(UCB0IFG & UCTXIFG));          // data transfered
    UCB0TXBUF = r_adr;
    while(!(UCB0IFG & UCTXIFG));            // data transfered
    UCB0TXBUF = data;
    while(!(UCB0IFG & UCTXIFG));            // data transfered
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    while((UCB0CTL1 & UCTXSTP));            // stop condition ended
}

void I2C_delay(void)
{
    uint8_t i;
    for (i = 0; i <100 ; i++);
}
