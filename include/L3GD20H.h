/*
 * L3GD20.h
 *
 *  Created on: 16. 9. 2020
 *      Author: dosedel
 */

#ifndef L3GD20_H_
#define L3GD20_H_

// macros definition
// for detailes see the table at https://www.st.com/resource/en/datasheet/l3gd20h.pdf, page 34
#define WHO_AM_I    0x0F
#define CTRL1       0x20
#define CTRL2       0x21
#define CTRL3       0x22
#define CTRL4       0x23
#define CTRL5       0x24
#define REFERENCE   0x25
#define OUT_TEMP    0x26
#define STATUS      0x27
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D
#define FIFO_CTRL   0x2E
#define FIFO_SRC    0x2F
#define IG_CFG      0x30
#define IG_SRC      0x31
#define IG_THS_XH   0x32
#define IG_THS_XL   0x33
#define IG_THS_YH   0x34
#define IG_THS_YL   0x35
#define IG_THS_ZH   0x36
#define IG_THS_ZL   0x37
#define IG_DURATION 0x38
#define LOW_ODR     0x39

#define L3GD20H_READ        0x80
#define L3GD20H_WRITE       0x00

#define L3GD20H_I2C_ADDR    0x6A    // address for SA0 in LOW
// functions prototypes

// variables

#endif /* L3GD20_H_ */
