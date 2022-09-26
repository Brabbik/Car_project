/*
 * I2C.h
 *
 *  Created on: 22. 9. 2020
 *      Author: dosedel
 */

#ifndef I2C_H_
#define I2C_H_


#include "stdint.h"
#include <msp430.h>
// macros definition

// functions prototypes
void I2C_init(uint8_t slave_addr);
void I2C_read_byte(uint8_t r_adr, uint8_t count);
void I2C_write_byte(uint8_t r_adr, uint8_t data);
void I2C_delay(void);
// variables
extern uint8_t RX_buffer[5];
extern uint8_t RXByteCtr;
#endif /* I2C_H_ */
