#include <msp430.h>
#include "include/SPI.h"
#include "include/I2C.h"
#include "include/LED.h"
#include "include/ADC.h"
#include "include/L3GD20H.h"
#include "include/ADXL343.h"
#include "include/motor.h"
#include <stdbool.h>

#define SPI
//#define I2C
#define I2C_RX_BUFFER_SIZE 5
#define BLINK_PERIOD 1024 //2048      // 2048 0.5s ~ 1 Hz //32 768
#define GYRO_PERIOD 364       // 32 768 Hz, divider /1 ~ 100.2 Hz
#define AVERAGING 4     // 4
#define N_LAP 90       // pattern lenght for correlation 90*5cm = 4,5 m
//#define DEBUG

void initClockTo16MHz(void);
void timer_B0_init(int period);
void timer_A1_init(int period);
void timer_A1_change_period(int period);
int avg_calc(unsigned int act_index, unsigned int length); //function for average calculation

uint16_t index;
unsigned int duty_cycle = 45;    //initialize of duty_cycle

//int x_speed_buf[AVERAGING];
//int y_speed_buf[AVERAGING];
int z_speed_buf[AVERAGING]; // averaging buffer
int index_delay = 0;
int last_breaking = 0;
int main_buffer[8000] = {0};         // main buffer for angular speed
unsigned int main_buffer_index = 0;  // actual index of saved acc to main buffer
unsigned int first_lap_index = 0;    // repeating indexes of data from 1 lap
long long z_speed_four_avg = 0; // average of sensor data 4 values
long long z_all_avg = 0; // average of all main buffer data
long long r_xx = 0; // corelation coeficient
int number_lap = 0; // actual number lap of race
int look_up[71] = {500,502,502,502,502,  // timer for data reading start
                   330,502,502,502,502,502,502,502,502,502, //5 rychlejsi posun realne na draze == vetsi perioda
                   502,502,502,502,502,502,502,502,502,502, //15
                   502,502,502,502,502,502,502,502,502,502, //25
                   502,48,465,461,448,434,420,406,393,379,  //35
                   364,364,364,364,345,/*50*/342,277,266,256,247, //45
                   238,230,222,215,209,/*60*/240,197,191,186,181, //55
                   176,172,168,164,160,156};                //65 - 70
bool save_calc; // main program flag
bool scanning = true; //

// I2C
uint8_t RX_buffer[I2C_RX_BUFFER_SIZE] = {0};
volatile uint8_t SPI_state = 0;
volatile uint16_t x_speed_t, y_speed_t, z_speed_t;
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
	volatile uint8_t SPIData;
	volatile uint8_t sens_id, status, temp;
	volatile int x_pos, x_speed, y_speed, z_speed;
	int breaking = 10;

    #ifdef SPI
        SPI_init();
        SPI_write_byte(0x20, 0x0F);
        //sens_id = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
        //status = SPI_read_byte(STATUS | L3GD20H_READ);
    #endif

	initClockTo16MHz();
	LED_init();                 // initialize of LEDs
	M_init();                   // initialize of H bridge and PWM
#ifdef DEBUG
	LED_RL_ON();
	LED_RR_ON();
#endif
	timer_B0_init(BLINK_PERIOD);    // timer B0 init
	timer_A1_init(GYRO_PERIOD);     // timer A1 init
    _BIS_SR(GIE);
	//P1DIR |= 0x04;
	//ADC12CTL0 |= ADC12SC;                   // Start convn - software trigger

/******************************************************************/
/************************** main loop *****************************/
	    while(true)
	    {
	        //duty_cycle = 45;
	        if(save_calc){
	            main_buffer[main_buffer_index] = z_speed_four_avg;
                main_buffer_index++;
                first_lap_index++;
                index_delay++;
                last_breaking++;

                if(main_buffer_index > 0)
                    z_all_avg = ((z_all_avg * main_buffer_index) + z_speed_four_avg)/(main_buffer_index + 1);
                else
                    z_all_avg = z_speed_four_avg;


                if(scanning){
                    if(avg_calc(main_buffer_index, 21) > 30)
                        duty_cycle = 47;
                    else
                        duty_cycle = 45;
                }

                if(main_buffer_index >= N_LAP)
                {
#ifdef DEBUG
                    LED_RL_OFF();
                    LED_RR_OFF();
#endif
                }

                if(main_buffer_index >= 2*N_LAP)   // po projeti useku zacneme korelovat
                {
                unsigned int i = 0; unsigned int n = main_buffer_index - N_LAP;
                long long a = 0;
                long long b = 0;
                for(i = 0; i < N_LAP; i++){         // auto korelace
                    a = a + ((main_buffer[i]-z_all_avg) * (main_buffer[i + n]-z_all_avg));
                    b = b + ((main_buffer[i]-z_all_avg) * (main_buffer[i]-z_all_avg));
                    }
                r_xx = ((100*a)/b);

                if(r_xx > 95 && index_delay > 20){
                    index_delay = 0;
                    number_lap++;
                    first_lap_index = N_LAP;
                    scanning = false;
#ifdef DEBUG
                    LED_FL_TOGGLE();
                    LED_FR_TOGGLE();

#endif
                    }
                }
#ifndef DEBUG
                    if(z_speed_four_avg > 25){
                        LED_FL_ON();
                        LED_FR_OFF();
                    }
                    else if(z_speed_four_avg < -25){
                        LED_FR_ON();
                        LED_FL_OFF();
                    }
                    else{
                        LED_FL_OFF();
                        LED_FR_OFF();
                    }
#endif


                if(number_lap > 0){
                    if(avg_calc(main_buffer_index, 15) < 3 && last_breaking > 18)
                        breaking = 4;
                    else if(avg_calc(main_buffer_index, 22) < 3 && last_breaking > 18)
                        breaking = 0;

                    //if(-20 > avg_calc(first_lap_index,20) && avg_calc(first_lap_index,20) < 20){    // avg udelat v abs hodnote
                        //if(-30 < main_buffer[first_lap_index + (est_duty-25)>>2] && main_buffer[first_lap_index + (est_duty-25)>>2] < 30){
                    if(-25 < main_buffer[first_lap_index+5] && main_buffer[first_lap_index+5] < 25 && last_breaking > 14 && -35 < z_speed_four_avg && z_speed_avg < 35){
                            duty_cycle = 60;       // pro duty_cycle nabihajici po rampe 45/5 vpred, 65/10 vpred
                            LED_RL_OFF();
                            LED_RR_OFF();
                            }
                        else{
                            duty_cycle = 45;    // kdyby vyletl 44
                            LED_RL_OFF();
                            LED_RR_OFF();
                            if(avg_calc(main_buffer_index, 21) > 30){
                                duty_cycle = 50;    // kdyby vyletl 47
                            }
                            if(breaking < 8){
                                LED_RL_ON();
                                LED_RR_ON();
                                duty_cycle = 0;
                                breaking++;
                                last_breaking = 0;
                            }
                            }
                    /*}else{
                        if(-25 < main_buffer[first_lap_index + 6] && main_buffer[first_lap_index + 6] < 25){
                         duty_cycle = 65;
                         LED_RL_OFF();
                         LED_RR_OFF();
                        }
                        else{
                        duty_cycle = 45;
                        LED_RL_ON();
                        LED_RR_ON();
                        }
                    }*/
                }
                timer_A1_change_period(look_up[duty_cycle]);
                M_set_duty_cycle(duty_cycle);
                save_calc = false;
	        }       // end of save_calc if


	#ifdef SPI
	        //SPI_write_byte(CTRL1 | L3GD20H_WRITE, 0x0F);
	        //sens_id = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
	        //status = SPI_read_byte(STATUS | L3GD20H_READ);
	        //x_speed_t = SPI_read_byte(OUT_X_L | L3GD20H_READ);    x_speed_t += SPI_read_byte(OUT_X_H | L3GD20H_READ) << 8;
	        //y_speed_t = SPI_read_byte(OUT_Y_L | L3GD20H_READ);    y_speed_t += SPI_read_byte(OUT_Y_H | L3GD20H_READ) << 8;
	        //z_speed_t = SPI_read_byte(OUT_Z_L | L3GD20H_READ);    z_speed_t += SPI_read_byte(OUT_Z_H | L3GD20H_READ) << 8;
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

// ############ ISRs ############## //
#pragma vector = TIMER1_A0_VECTOR
__interrupt void ISR_TA1_GYRO(void){
    SPI_state = 0;
    if(index >= AVERAGING){
        index = 0;
        save_calc = true;
    }
    UCB0IE |= UCTXIE;
    P3OUT &= ~0x01;
    UCB0TXBUF = (L3GD20H_READ + 0x40 + OUT_X_L);
    TA1CTL |= TACLR;
}

#pragma vector = TIMER0_B0_VECTOR   //isr for period
__interrupt void ISR_TB0_CCR0(void){
    if(scanning)
        {
#ifndef DEBUG
        LED_RL_TOGGLE();
        LED_RR_TOGGLE();
#endif
#ifdef DEBUG
        LED_FL_TOGGLE();
        LED_FR_TOGGLE();
#endif
        }
    TB0CTL |= TBCLR;    //clear flag CCR0
}

#pragma vector = USCI_B0_VECTOR
__interrupt void ISR_GYRO_MEASURE(void){
    UCB0IFG &= ~UCTXIFG;
    UCB0IE &= ~UCTXIE;
    switch(SPI_state){
      case 0:
          UCB0TXBUF = 0xFF;
          SPI_state++;
          break;
      case 1:
          UCB0TXBUF = 0xFF;
          x_speed_t = UCB0RXBUF;
          SPI_state++;
          break;
      case 2:
          UCB0TXBUF = 0xFF;
          x_speed_t += UCB0RXBUF << 8;
          SPI_state++;
          break;
      case 3:
          UCB0TXBUF = 0xFF;
          y_speed_t = UCB0RXBUF;
          SPI_state++;
          break;
      case 4:
          UCB0TXBUF = 0xFF;
          y_speed_t += UCB0RXBUF << 8;
          SPI_state++;
          break;
      case 5:
          UCB0TXBUF = 0xFF;
          z_speed_t = UCB0RXBUF;
          SPI_state++;
          break;
      case 6:
          z_speed_t += UCB0RXBUF << 8;
          P3OUT |= 0x01;
          SPI_state = 0;
          z_speed_buf[index] = (int)z_speed_t >> 8;
          index++;
          unsigned i = AVERAGING;
          z_speed_four_avg = 0;
          for(;i > 0; i--){
              z_speed_four_avg += z_speed_buf[AVERAGING - i];
          }
          z_speed_four_avg = z_speed_four_avg >> 2;       // pro 4 musi byt 2
          break;
      default: break;
    }
    UCB0IE |= UCTXIE;
}

// ######## functions ########### //
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

void timer_B0_init(int period){
    TB0CCTL0 = CCIE;            // timer 0 interrupt enable LED
    TB0CCR0 = period;           // set timer period 500 ms
    TB0CTL = TASSEL__ACLK + MC__UP + ID__8 + TBIE;
}
void timer_A1_init(int period){
    TA1CCTL0 = CCIE;            // timer for GYRO start measure period
    TA1CCR0 = period;           // set timer period 10 ms ~ freq 100 Hz
    TA1CTL = TASSEL__ACLK + MC__UP + ID__1 + TAIE;
}
void timer_A1_change_period(int period){
    TA1CCR0 = period;
}
void timer_B0_change_period(int period){
    TB0CCR0 = period;
}

int avg_calc(unsigned int act_index, unsigned int length){
    unsigned int tmp = 0;
    unsigned int i = act_index;
    for (; i > act_index - length; i--)
        tmp += abs(main_buffer[i]);
    return tmp/length;
}

