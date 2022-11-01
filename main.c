#include <msp430.h>
#include "include/SPI.h"
#include "include/I2C.h"
#include "include/LED.h"
#include "include/ADC.h"
#include "include/Serial.h"
#include "include/L3GD20H.h"
#include "include/ADXL343.h"
#include "include/motor.h"
#include <stdbool.h>

#define SPI
//#define I2C
#define I2C_RX_BUFFER_SIZE 5
#define COUNTER_VALUE 2048      // 2048 0.5s ~ 1 Hz //32 768
#define GYRO_PERIOD 328       // 32 768 Hz, divider /1 ~ 100.2 Hz

void initClockTo16MHz(void);

uint16_t results[5];
uint16_t index;
int real_temp = 0;
bool run;

// I2C
uint8_t RX_buffer[I2C_RX_BUFFER_SIZE] = {0};
volatile uint8_t SPI_state = 0;
volatile uint16_t x_accel_t, y_accel_t, z_accel_t;
/*
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;
*/

/******************************************************************/
/*************************** main function ************************/
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	//stop watchdog timer
	uint8_t duty_cycle = 50;    //initialize of duty_cycle
	volatile uint8_t SPIData;
	volatile uint8_t sens_id, status, temp;
	volatile int x_pos, x_accel, y_accel, z_accel;
	volatile uint8_t integr_samples;

    #ifdef SPI
        SPI_init();

        SPI_write_byte(0x20, 0x0F);
        //SPI_write_byte(0x20, 0x0F);
        //sens_id = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
        //__delay_cycles(160000);
        //sens_id = SPI_read_byte(CTRL1 | L3GD20H_READ);
        //SPI_write_byte(CTRL2 | L3GD20H_WRITE, 0x80);
        //__delay_cycles(160000);

        //status = SPI_read_byte(STATUS | L3GD20H_READ);

    #endif

	initClockTo16MHz();
	LED_init();                 //initialize of LEDs
	M_init();                   //initialize of H bridge and PWM
	SerialInit();

	TB0CCTL0 = CCIE;            // timer 0 interrupt enable LED
	TB0CCR0 = COUNTER_VALUE;
	TB0CTL = TASSEL__ACLK + MC__UP + ID__8;
	TB0CTL |= TBIE;

    TA1CCTL0 = CCIE;            //timer A1 interrupt enable GYRO start measure
    TA1CCR0 = GYRO_PERIOD;      //timer A1 freq 100 Hz
    TA1CTL = TASSEL__ACLK + MC__UP + ID__1;
    TA1CTL |= TAIE;

    _BIS_SR(GIE);


	P1DIR |= 0x04;
	ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger

/******************************************************************/
/************************** main loop *****************************/
	    while(true)
	    {
	        run = false;
	        /*if(duty_cycle >= 50)
	            slowdown = true;
	        if(duty_cycle <= 40)
	            slowdown = false;*/
	        M_set_duty_cycle(duty_cycle);
	        //SerialWrite();

	#ifdef SPI

	        //SPI_write_byte(0x21, 0xA7);
	        //SPI_write_byte(CTRL1 | L3GD20H_WRITE, 0x0F);
	        //sens_id = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
	        //status = SPI_read_byte(STATUS | L3GD20H_READ);
	        if(sens_id == 0xD7)
	            run = false;

	        //SPI_write_byte(0x20, 0x0F);
	        //x_accel_t = SPI_read_byte(OUT_X_L | L3GD20H_READ);
	        //x_accel_t += SPI_read_byte(OUT_X_H | L3GD20H_READ) << 8;
	        //y_accel_t = SPI_read_byte(OUT_Y_L | L3GD20H_READ);
	        //y_accel_t += SPI_read_byte(OUT_Y_H | L3GD20H_READ) << 8;
	        //z_accel_t = SPI_read_byte(OUT_Z_L | L3GD20H_READ);
	        //z_accel_t += SPI_read_byte(OUT_Z_H | L3GD20H_READ) << 8;
	        if(integr_samples > 100){
	            x_pos = 0;
	            integr_samples = 0;
	        }
	        x_accel = (int)x_accel_t;
	        y_accel = (int)y_accel_t;
	        z_accel = (int)z_accel_t;
	        x_pos += x_accel;

	        integr_samples++;

	        /*if(z_accel > 5000){
	            LED_FL_ON();
	            LED_FR_OFF();
	            LED_RL_ON();
	            LED_RR_ON();
	            duty_cycle = 45;
	        }else if(z_accel < -5000){
	            LED_FR_ON();
	            LED_FL_OFF();
	            LED_RL_ON();
	            LED_RR_ON();
	            duty_cycle = 45;
	        }else{
	            LED_FR_OFF();
	            LED_FL_OFF();
	            LED_RL_OFF();
	            LED_RR_OFF();
	            duty_cycle = 55;
	        }*/
	        //temp = SPI_read_byte(OUT_TEMP | L3GD20H_READ);
	        real_temp = (int)temp;
	        //__delay_cycles(3200000);
	#endif

    #ifdef I2C  // i2c init
	    I2C_init(0x20>>1);
	    _BIS_SR(GIE);
	    P3DIR &= ~0x04;                            // P3.2 as input
	 /* only for ADXL343
	    P3DIR |= 0x09;                            // P3.2 as output for I2C address specification
	    P3OUT &= ~0x08;                           // P3.2 to LOW (for 1101010b of L3GD20H address selection)
	    P3OUT |= 0x01;                            // ADXL343 CS pin to HIGH for I2C enable
	    */
	#endif
	#ifdef I2C  // i2c read/write
	        __delay_cycles(1000);                     // Delay required between transaction
	        I2C_write_byte(0x00, 0x00);             // wake up signal at address 0x00
	        I2C_read_byte(0x00, 5);
	        // for ADXL343
	        /*
	        index = I2C_read_byte(BW_RATE);
	        I2C_write_byte(BW_RATE, index^0x01);
	        index = I2C_read_byte(BW_RATE);
	        */
	#endif
        }       // end of main loop
	}           // end of main function

void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer_A(){
    run = true;
    /*if(slowdown)
        duty_cycle -=2;
    else
        duty_cycle += 2;*/

    SPI_state = 0;
    UCB0IE |= UCTXIE;
    P3OUT &= ~0x01;
    UCB0TXBUF = (L3GD20H_READ + 0x40 + OUT_X_L);

    TA1CTL |= TACLR;
}
#pragma vector = TIMER0_B0_VECTOR   //isr for period
__interrupt void ISR_TB0_CCR0(void){
    LED_FL_TOGGLE();
    LED_FR_TOGGLE();
    LED_RL_TOGGLE();
    LED_RR_TOGGLE();
    TB0CTL |= TBCLR;    //clear flag CCR0
}

/*#pragma vector = TIMER0_B0_VECTOR   //isr for period
__interrupt void ISR_TB0_CCR0(void){
    run = true;
    TB0CCTL0 &= ~CCIFG;    //clear flag CCR0
}*/
/*
#pragma vector = TIMER0_A1_VECTOR       // isr flag duty cycle
__interrupt void ISR_TA0_CCR1(void){
    //M_STOP();
    TA0CCTL2 &= ~CCIFG;    //clear flag CCR1
}*/

void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Select DCO range 16MHz operation
    UCSCTL2 = FLLD_0 + 487;                   // Set DCO Multiplier for 16MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (487 + 1) * 32768 = 16MHz
                                              // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);//
    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                          // Clear fault flags
    }while (SFRIFG1&OFIFG);                         // Test oscillator fault flag
}

/*#pragma vector = USCI_UCTXIFG
__interrupt void USCI_B0_ISR1(void)
    {
    UCB0TXBUF = 0xFF;
    }*/
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    UCB0IFG &= ~UCTXIFG;
    UCB0IE &= ~UCTXIE;
    //UCB0TXBUF = 0xFF;
    switch(SPI_state)
      {
      case 0:
          UCB0TXBUF = 0xFF;
          SPI_state++;
          break;
      case 1:
      {
          UCB0TXBUF = 0xFF;
          x_accel_t = UCB0RXBUF;
          SPI_state++;
          //UCB0IE |= UCTXIE;
          break;
      }
      case 2:
      {
          UCB0TXBUF = 0xFF;
          x_accel_t += UCB0RXBUF << 8;
          //UCB0IFG &= ~UCTXIFG;
          SPI_state++;
          break;
      }
      case 3:
      {
          UCB0TXBUF = 0xFF;
          y_accel_t = UCB0RXBUF;
          //UCB0IFG &= ~UCTXIFG;
          SPI_state++;
          break;
      }
      case 4:
      {
          UCB0TXBUF = 0xFF;
          y_accel_t += UCB0RXBUF << 8;
          //UCB0IFG &= ~UCTXIFG;
          SPI_state++;
          break;
      }
      case 5:
      {
          UCB0TXBUF = 0xFF;
          z_accel_t = UCB0RXBUF;
          //UCB0IFG &= ~UCTXIFG;
          SPI_state++;
          break;
      }
      case 6:
      {
          //UCB0TXBUF = 0xFF;
          z_accel_t += UCB0RXBUF << 8;
          P3OUT |= 0x01;
          //UCB0IFG &= ~UCTXIFG;
          //UCB0IE &= ~UCTXIE;
          SPI_state = 0;
          break;
      }
      default: break;
      }
    UCB0IE |= UCTXIE;
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6: break;                           // Vector  6:  ADC12IFG0
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                            // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20:                            // Vector 20:  ADC12IFG7
      LED_FL_ON();
      ADC12CTL0 &=~ADC12SC;                // For sequence-of-Channels mode, ADC12SC must be cleared by software after each sequence to trigger another sequence
      results[0] = ADC12MEM3;                 // Move results, IFG is cleared
      results[1] = ADC12MEM4;                 // Move results, IFG is cleared
      results[2] = ADC12MEM5;                 // Move results, IFG is cleared
      results[3] = ADC12MEM6;                 // Move results, IFG is cleared
      results[4] = ADC12MEM7;                 // Move results, IFG is cleared
      LED_FL_OFF();
      ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
