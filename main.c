#include <msp430.h>
#include "include/SPI.h"
#include "include/I2C.h"
#include "include/LED.h"
#include "include/ADC.h"
#include "include/L3GD20H.h"
#include "include/ADXL343.h"
#include <stdbool.h>

//#define SPI
//#define I2C
#define I2C_RX_BUFFER_SIZE 5
/**
 * main.c
 */
void initClockTo16MHz(void);

uint16_t results[5];
uint16_t i, index, t_1ms;
bool run, led;
// I2C
uint8_t RX_buffer[I2C_RX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
/*
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;
*/
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	initClockTo16MHz();
	#ifdef SPI
	    uint8_t SPIData;
	    SPI_init();
	#endif
	    LED_init();
	    led = true;
	    t_1ms = 0;
	  //  ADC_init();
	    _BIS_SR(GIE);
	#ifdef I2C
	    I2C_init(0x20>>1);
	    _BIS_SR(GIE);
	    P3DIR &= ~0x04;                            // P3.2 as input
	 /* only for ADXL343
	    P3DIR |= 0x09;                            // P3.2 as output for I2C address specification
	    P3OUT &= ~0x08;                           // P3.2 to LOW (for 1101010b of L3GD20H address selection)
	    P3OUT |= 0x01;                            // ADXL343 CS pin to HIGH for I2C enable
	    */
	#endif
	    P1DIR |= 0x04;
	    ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger
	//    P8DIR |= 0b101110;    // H bridge
	    if(t_1ms > 16000000)
	        {
	        run = true;
	        t_1ms = 0;
	        led = !led;
	        }
	    else
	        run = false;
	    t_1ms++;


	    while(run)
	    {
	        if (led)
	          LED_FL_ON();
	        else
	          LED_FL_OFF();
	 //       P8OUT = 0b001100;
	 //       for (i = 5000; i!=0; i--);
	 //       P8OUT = 0b0;    // OFF
	 //       for (i = 5000; i!=0; i--);
	//        P8OUT = 0b0001100;
	        //for (i = 1000; i!=0; i--);
	        //P8OUT = 0b0;    // OFF
	        //for (i = 1000; i!=0; i--);

	#ifdef SPI
	        SPIData = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
	        SPIData = SPI_read_byte(CTRL1 | L3GD20H_READ);
	        SPI_write_byte(CTRL1 | L3GD20H_WRITE, 0x0F);
	        SPIData = SPI_read_byte(CTRL1 | L3GD20H_READ);
	#endif
	#ifdef I2C
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
	     }
	}

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

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  uint8_t rx_val = 0;

  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: break;                           // Vector  4: NACKIFG
  case  6: break;                           // Vector  6: STTIFG
  case  8: break;                           // Vector  8: STPIFG
  case USCI_I2C_UCRXIFG:
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          RX_buffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTL1 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
        }
        break;                      // Interrupt Vector: I2C Mode: UCRXIFG
  case 12: break;                           // Vector 12: TXIFG
  default: break;
  }
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