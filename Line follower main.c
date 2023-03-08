#include <msp430.h> 
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "MPU6050.h"
#include "I2C.h"
#define LED BIT0


int zAccel;
int yAccel;
int xAccel;
int zAngle;
int yAngle;
int xAngle;
int period;
float D;
unsigned int adc[8];
int Left_IR,Right_IR;
void straight (void);
void turnright (void);
void turnleft (void);
void stop (void);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ;//sets clock to 1 MHz - up to discretion and application
    DCOCTL  = CALDCO_1MHZ;
    P1SEL |= BIT6;
    P2SEL |=  BIT1 + BIT2 + BIT6;               //P1SEL1 |= BIT3 + BIT2;      // Assign I2C pins to USCI_B0
    P2SEL2|=  BIT1 + BIT2 + BIT6;               //P1SEL0 |= BIT3 + BIT2;      // Assign I2C pins to USCI_B0
    P1DIR |= 0xE7;
    P2DIR |= 0xFF;
    period = 0x03FF;  //PWM period
    D = 0.62; //duty cycle
    TACCR0 = period-1;  //PWM period
    TACCR1 = period*D;  //CCR1 PWM Duty Cycle
    TACCTL1 = OUTMOD_7;  //CCR1 selection reset-set
    TACTL = TASSEL_2|MC_1;   //SMCLK submain clock,upmode
    ADC10CTL1 = INCH_7 + ADC10DIV_0 + CONSEQ_3 + SHS_0;
    ADC10CTL0 = SREF_0 + ADC10SHT_2 + MSC + ADC10ON; //ADC10IE
    ADC10AE0 =  BIT4 + BIT3 ;
    ADC10DTC1 = 8;

    slaveAddress = 0x68;    // Set slave address for MPU-6050

    i2cInit();

    // Wake up the MPU-6050
    slaveAddress = 0x68;                    // MPU-6050 address
    TX_Data[1] = 0x6B;                      // address of PWR_MGMT_1 register
    TX_Data[0] = 0x00;                      // set register to zero (wakes up the MPU-6050)
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    //placing the accelerometer in mode 3
    //which is +/- 16g
    //Note:
    //

    TX_Data[1] = 0x1C;
    TX_Data[0] = 0x18;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    //gyro handling
    TX_Data[1] = 0x1B;
    TX_Data[0] = 0x18;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    //low pass filter handling
    TX_Data[1] = 0x1A;
    TX_Data[0] = 0x06;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);
    while(1)
        {
             /////// Get MPU Values ////////
            getMPU6050();//Updates acceleration values
            zAccel = getZAccel();
            yAccel = getYAccel();
            xAccel = getXAccel();
            zAngle = getZAngle();
            yAngle = getYAngle();
            xAngle = getXAngle();

            ADC10CTL0 &= ~ENC;
            while (ADC10CTL1 & BUSY);
            ADC10CTL0 |= ENC + ADC10SC;
            ADC10SA = (unsigned int)adc;
            Left_IR = adc[4];
            Right_IR = adc[3];
            __delay_cycles(100);

             if (Left_IR>=1000 && Right_IR>=1000)
               stop();
             else if (Left_IR<=100 && Right_IR>=1000)
               turnright();
             else if (Left_IR>=1000 && Right_IR<=100)
               turnleft();
             else if (Left_IR<=100 && Right_IR<=100)
               straight() ;
            }

        }

void straight(void){
    P2OUT = 0x00;
    P1OUT = 0x00;
    P2OUT |= 0x28;
}
void turnright(void){
    P2OUT = 0x00;
    P1OUT = 0x00;
    P2OUT |= 0x20;
    P1OUT |= 0x80;
}
void turnleft(void){
    D = 1.00;
    P2OUT = 0x00;
    P1OUT = 0x00;
    P2OUT |= 0x018;
}
void stop(void){
    D = 1.00;
    P2OUT = 0x00;
    P1OUT = 0x00;
    P2OUT |= 0x00;
}

