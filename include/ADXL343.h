/*
 * ADXL343.h
 *
 *  Created on: 23. 9. 2020
 *      Author: dosedel
 */

#ifndef ADXL343_H_
#define ADXL343_H_

// macros definition
// for detailes see the table at https://www.st.com/resource/en/datasheet/l3gd20h.pdf, page 34
#define DEVID       0x00
#define THRESH_TAP  0x1D    //  Tap threshold
#define OFSX        0x1E    //  X-axis  offset
#define OFSY        0x1F    //  Y-axis  offset
#define OFSZ        0x20    //  Z-axis  offset
#define DUR         0x21    //  Tap duration
#define LATENT      0x22    //  Tap latency
#define WINDOW      0x23    //  Tap window
#define THRESH_ACT  0x24    //  Activity    threshold
#define THRESH_INACT    0x25    //  Inactivity  threshold
#define TIME_INACT  0x26    //  Inactivity  time
#define ACT_INACT_CTL   0x27    //  Axis    enable  control for activity    and inactivity  detection
#define THRESH_FF   0x28    //  Free-fall   threshold
#define TIME_FF     0x29    //  Free-fall   time
#define TAP_AXES    0x2A    //  Axis    control for single  tap/double  tap
#define ACT_TAP_STATUS  0x2B    //  Source  of  single  tap/double  tap
#define BW_RATE     0x2C    //  Data    rate    and power   mode    control
#define POWER_CTL   0x2D    //  Power-saving    features    control
#define INT_ENABLE  0x2E    //  Interrupt   enable  control
#define INT_MAP     0x2F    //  Interrupt   mapping control
#define INT_SOURCE  0x30    //  Source  of  interrupts
#define DATA_FORMAT 0x31    //  Data    format  control
#define DATAX0      0x32    //  X-Axis  Data
#define DATAX1      0x33    //  X-Axis  Data
#define DATAY0      0x34    //  Y-Axis  Data
#define DATAY1      0x35    //  Y-Axis  Data
#define DATAZ0      0x36    //  Z-Axis  Data
#define DATAZ1      0x37    //  Z-Axis  Data
#define FIFO_CTL    0x38    //  FIFO    control
#define FIFO_STATUS 0x39    //  FIFO    status




#define ADXL343_I2C_RD        0xA7
#define ADXL343_I2C_WR        0xA6

#define L3GD20H_I2C_ADDR    0x6A    // address for SA0 in LOW
// functions prototypes

// variables

#endif /* ADXL343_H_ */
