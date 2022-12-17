/*
 * SPI.h
 *
 *  Created on: 16. 11. 2022
 *      Author: brabenec, pohorsky
 */

#ifndef SPI_H_
#define SPI_H_


#include "stdint.h"
#include <msp430.h>
// macros definition

// functions prototypes
void SPI_init(void);
uint8_t SPI_read_byte(uint8_t addr);
void SPI_write_byte(uint8_t addr, uint8_t data);
void SPI_delay(void);

// variables

#endif /* SPI_H_ */
