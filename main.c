#include <msp430.h>
#include "include/SPI.h"
#include "include/LED.h"
#include "include/ADC.h"
#include "include/L3GD20H.h"
#include "include/ADXL343.h"
#include "include/motor.h"
#include <stdbool.h>

#define SPI
#define BLINK_PERIOD 1024   //2048      // 2048 0.5s ~ 1 Hz //32 768
#define GYRO_PERIOD 364     // 32 768 Hz, divider /1 ~ 100.2 Hz
#define AVERAGING 4         // 4
#define PATTERN_L 90            // pattern lenght for correlation 90*5cm = 4,5 m
//#define DEBUG

void initClockTo16MHz(void);
void timer_B0_init(int period);
void timer_A1_init(int period);
void timer_A1_change_period(int period);
int avg_calc(unsigned int act_index, unsigned int length); //function for average calculation

uint16_t index;                         // index of new four data
unsigned int duty_cycle = 45;           // initialize of duty_cycle
int z_speed_buf[AVERAGING];             // averaging buffer
int index_delay = 0;                    // help variable for correlation coef
int last_breaking = 0;                  // number of cycles of last breaking
int main_buffer[8000] = {0};            // main buffer for angular speed
unsigned int main_buffer_index = 0;     // actual index of saved angular speed to main buffer
unsigned int first_lap_index = 0;       // repeating indexes of data from 1 lap
long long z_speed_four_avg = 0;         // average of sensor data 4 values
long long z_all_avg = 0;                // average of all main buffer data
long long r_xx = 0;                     // corelation coeficient
int number_lap = 0;                     // actual number lap of race

int look_up[71] = {500,502,502,502,502,  // timer for data reading start
                   330,502,502,502,502,502,502,502,502,502, //5 rychlejsi posun realne na draze == vetsi perioda
                   502,502,502,502,502,502,502,502,502,502, //15
                   502,502,502,502,502,502,502,502,502,502, //25
                   502,48,465,461,448,434,420,406,393,379,  //35
                   364,364,364,364,345,/*50*/342,277,266,256,247, //45
                   238,230,222,215,209,/*60*/240,197,191,186,181, //55
                   176,172,168,164,160,156};                //65 - 70

bool save_calc;                     // main program flag
bool scanning = true;               // flag of all track scanning
volatile uint8_t SPI_state = 0;     // SPI RX data state machine
volatile uint16_t x_speed_t, y_speed_t, z_speed_t;        // raw angular data (two complement)

/******************************************************************/
/*************************** main function ************************/
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	volatile uint8_t sens_id;   // id of sensor model
	int breaking = 10;          // breaking length

    #ifdef SPI
        SPI_init();                     // initialize of SPI communication
        SPI_write_byte(0x20, 0x0F);     // setting of sensor (axis enable)
        //sens_id = SPI_read_byte(WHO_AM_I | L3GD20H_READ);
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
    _BIS_SR(GIE);                   // global interrupt enable

/******************************************************************/
/************************** main loop *****************************/
while(true)
{
    if(save_calc){      // cycle condition, new z_speed_four_avg data = 1x execute
        main_buffer[main_buffer_index] = z_speed_four_avg;      // feeding main_buffer
        main_buffer_index++;
        first_lap_index++;
        index_delay++;
        last_breaking++;

        if(main_buffer_index > 0)       // next 4 lines are for calculating the moving average of all data
            z_all_avg = ((z_all_avg * main_buffer_index) + z_speed_four_avg)/(main_buffer_index + 1);
        else
            z_all_avg = z_speed_four_avg;

        if(scanning){                   // controlling car speed in first lap
            if(avg_calc(main_buffer_index, 21) > 30)
                duty_cycle = 47;
            else
                duty_cycle = 45;
        }

        if(main_buffer_index >= PATTERN_L)  // turn off rear lights after pattern length
        {
#ifdef DEBUG
        LED_RL_OFF();
        LED_RR_OFF();
#endif
        }

        if(main_buffer_index >= 2*PATTERN_L)   // after 2 correlation pattern correlations started
        {
        unsigned int i = 0; unsigned int n = main_buffer_index - PATTERN_L;
        long long a = 0;
        long long b = 0;
        for(i = 0; i < PATTERN_L; i++){         // auto correlation
            a = a + ((main_buffer[i]-z_all_avg) * (main_buffer[i + n]-z_all_avg));
            b = b + ((main_buffer[i]-z_all_avg) * (main_buffer[i]-z_all_avg));
            }
        r_xx = ((100*a)/b);         // calculation of coefficient

        if(r_xx > 95 && index_delay > 20){      // limit for index same pattern && multiple execution treatment
            index_delay = 0;
            number_lap++;
            first_lap_index = PATTERN_L;
            scanning = false;
#ifdef DEBUG
            LED_FL_TOGGLE();
            LED_FR_TOGGLE();

#endif
            }
        }
#ifndef DEBUG
        if(z_speed_four_avg > 25){  // left turn led indicate
            LED_FL_ON();
            LED_FR_OFF();
        }
        else if(z_speed_four_avg < -25){    // right turn led indicate
            LED_FR_ON();
            LED_FL_OFF();
        }
        else{
            LED_FL_OFF();
            LED_FR_OFF();
        }
#endif
        if(number_lap > 0){     // controlling car speed after first lap
            if(avg_calc(main_buffer_index, 15) < 3 && last_breaking > 18)   // set breaking flag after 3 straight
                breaking = 4;
            else if(avg_calc(main_buffer_index, 22) < 3 && last_breaking > 18)  // set breaking flag after 4 straight
                breaking = 0;

            if(-25 < main_buffer[first_lap_index+5] && main_buffer[first_lap_index+5] < 25 && last_breaking > 14 && -35 < z_speed_four_avg && z_speed_four_avg < 35){
                duty_cycle = 60;       // high speed 60 if 25 cm before car isn't turn && last breaking flag end && car currently isn't in turn
                LED_RL_OFF();
                LED_RR_OFF();
                }
            else{
                duty_cycle = 45;        // slow speed
                LED_RL_OFF();
                LED_RR_OFF();
                if(avg_calc(main_buffer_index, 21) > 30)    // after 3 turn we speed up
                    duty_cycle = 50;    // medium speed
                if(breaking < 8){       // breaking
                    LED_RL_ON();
                    LED_RR_ON();
                    duty_cycle = 0;
                    breaking++;
                    last_breaking = 0;
                    }
                }
            }
        timer_A1_change_period(look_up[duty_cycle]);        // change of data reading speed
        M_set_duty_cycle(duty_cycle);                       // change duty cycle
        save_calc = false;                                  // set flag for 1 cycle
        }           // end of save_calc if
    }           // end of main loop
}           // end of main function

// ############ Interrupt SubRoutines ############## //
#pragma vector = TIMER1_A0_VECTOR
__interrupt void ISR_TA1_GYRO(void){    // ISR for data reading start
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

#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void){        // ISR for LED blinking
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
__interrupt void ISR_GYRO_MEASURE(void){        // ISR  SPI data receive
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
          z_speed_buf[index] = (int)z_speed_t >> 8;     // angular speed divided by 256
          index++;
          unsigned i = AVERAGING;
          z_speed_four_avg = 0;
          for(;i > 0; i--){                             // averaging of last 4 data
              z_speed_four_avg += z_speed_buf[AVERAGING - i];
          }
          z_speed_four_avg = z_speed_four_avg >> 2;       // logical divide by 4
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
