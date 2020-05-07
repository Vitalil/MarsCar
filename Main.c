#include <msp430.h>
//#define RECEIVE_ECHO 0x01 //[P3,P2.0]Receive echo from ultra-sonic sensor
#define FORWARD 0x02      //[P4,P2.1]Trigger forward wave
#define LEFT 0x04         //[P5,P2.2]Trigger left wave
#define RIGHT 0x08        //[P6,P2.3]Trigger right wave
#define MOTOR_LF   0x10   //[P7,P2.4]Activate left motor - forward Yellow
#define MOTOR_RF   0x08   //[P8,P4.3]Activate right motor - forward Red
#define MOTOR_LR   0x10   //[P9,P4.4]Activate left motor - reverse Green
#define MOTOR_RR   0x20   //[P10,P4.5]Activate right motor - reverse BLUE
#define READ_PERIOD 5     //Time to wait between trigger wave and receive echo
#define TIMEOUT 30000     //Maximum time to wait if no ehco is received
#define DRIVE_TIME 1000000
#define MIN_DIS 20        //The minimal distance in which the vehicle can respond
#define AMOUNT_SAMPLE 5   //The number of pulses the ultra-sonic sensor transmit in order to detect blockage
#define MIN_DIS_TURN 5    //The minimal distance in which the vehicle can make a turn
#define CONVERSION_CONST 58

int currentEcho = 0;
int miliseconds = 0;      //A counter to track how many milliseconds have passed
int sensor = 0;           //value from ultra-sonic sensor

int detect(int direction);  //return the distance in cm from the next blockage, the parameter 'trigger' represent which sensor to activate



void main(void)
{
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;                    // submainclock 1mhz
  WDTCTL = WDTPW + WDTHOLD;                // Stop WDT

  CCTL0 = CCIE;                            // CCR0 interrupt enabled
  CCR0 = 1000;                             // 1ms at 1mhz
  TACTL = TASSEL_2 + MC_1;                 // SMCLK, upmode

  P2IFG  = 0x00;                           //clear all interrupt flags
  P1IFG  = 0x00;                           //clear all interrupt flags
  P1DIR |= 0x03;                           // P1.0 P1.1 as output for LEDs

  P4DIR |= MOTOR_RR + MOTOR_RF + MOTOR_LR; // set pins' direction  as output
  P2DIR |= MOTOR_LF;                       //set direction pin as output


  _BIS_SR(GIE);                            // global interrupt enable

  int left_average = 0,right_average = 0,i;

  P4OUT &= ~MOTOR_RF;
  P4OUT &= ~MOTOR_RR;
  P2OUT &= ~MOTOR_LF;
  P4OUT &= ~MOTOR_LR;

  int samplesInRow = 0;
  while(1)
    {
      P1OUT &= ~0x03;
      P4OUT &= ~MOTOR_RF;
      P4OUT &= ~MOTOR_RR;
      P2OUT &= ~MOTOR_LF;
      P4OUT &= ~MOTOR_LR;

    //activate motors - start moving ahead!
    P4OUT |= MOTOR_RF;
    P2OUT |= MOTOR_LF;
    //keep moving & detect forward as long as there is no blockage
    samplesInRow = 0;
    while(samplesInRow < AMOUNT_SAMPLE)
    {
        if(detect(FORWARD) < MIN_DIS)
            samplesInRow++;
        else
            samplesInRow = 0;
    }

    //stop moving forword in order to make a turn
    P2OUT &= ~MOTOR_LF;
    P4OUT &= ~MOTOR_RF;
    samplesInRow = 0;

    right_average = 0;
    left_average = 0;

    //gather samples from the sides
    for(i=0; i<AMOUNT_SAMPLE;i++)
    {
        //left
        left_average += detect(LEFT);
        //right
        right_average += detect(RIGHT);
    }

    //calc average for improved accuracy
    right_average /= AMOUNT_SAMPLE;
    left_average  /= AMOUNT_SAMPLE;


    //__delay_cycles(100000);
    //if a turn is not an option goto reverse
    if(right_average<MIN_DIS_TURN  && left_average<MIN_DIS_TURN)
    {
        //reverse
        P4OUT |= MOTOR_LR;
        P4OUT |= MOTOR_RR;
        __delay_cycles(DRIVE_TIME);
        continue;
    }
    if(right_average < left_average)
    {
     //turn left
        //activate left motor - forward
        P2OUT |= MOTOR_LF;              // activate motor to go faorward
        //activate right motor - reverse
        P4OUT |= MOTOR_RR;              // activate motor to go faorward
        P1OUT |= 0x01;
    }
    else
    {
     //turn right
        //activate left motor - reverse
        P4OUT |= MOTOR_LR;              // activate motor to go faorward
        //activate right motor - forward
        P4OUT |= MOTOR_RF;              // activate motor to go faorward
        P1OUT |= 0x02;
    }
    __delay_cycles(DRIVE_TIME);
  }//while(1)
}//main

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if( P2IFG & currentEcho )           //is there interrupt pending?
        {
          if(!( P2IES & currentEcho ))  // is this the rising edge?
          {
            TACTL|=TACLR;               // clears timer A
            miliseconds = 0;
            P2IES |= currentEcho;       //falling edge
          }
          else
          {
              sensor = (long)miliseconds*1000 + (long)TAR; //calculating RECEIVE_ECHO lenght
          }
    P2IFG &= ~currentEcho;        //clear flag
    }
    P2IFG = 0x00;
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
  miliseconds++;
}
int detect(int trigger)
{
    currentEcho = trigger;
    P2IE &= ~currentEcho;              // disable interrupt
    P2DIR |= trigger;                  // make pin P2.0 output (trigger)[p3]
    P2OUT |= trigger;                  // generate pulse
    __delay_cycles(READ_PERIOD);       // for 10us
    P2OUT &= ~trigger;                 // stop pulse
    P2DIR &= ~currentEcho;             // make pin P2.1 input (RECEIVE_ECHO)[p4]
    P2IFG = 0x00;                      // clear flag just in case anything happened before
    P2IE |= currentEcho;               // enable interrupt on RECEIVE_ECHO pin
    P2IES &= ~currentEcho;             // rising edge on RECEIVE_ECHO pin
    __delay_cycles(TIMEOUT);           // delay for 30ms (after this time echo times out if there is no object detected)
    currentEcho = 0;
    return sensor / CONVERSION_CONST;  // converting RECEIVE_ECHO lenght into cm
}

