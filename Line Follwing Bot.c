#include <msp430.h> 
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "MPU6050.h"
#include "I2C.h"
#include "FlashMemory.h"
#define LED BIT0


int zAccel;
int yAccel;
int xAccel;
int zAngle;
int yAngle;
int xAngle;


int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ;//sets clock to 1 MHz - up to discretion and application
    DCOCTL  = CALDCO_1MHZ;
    P1SEL |= BIT6 + BIT7;               //P1SEL1 |= BIT3 + BIT2;      // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;               //P1SEL0 |= BIT3 + BIT2;      // Assign I2C pins to USCI_B0
    P1DIR |= 0x09;
    //P1OUT |= LED;

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
            if(xAngle>0){
                P1OUT = LED;
            }


    if(zAccel > 1*1024)
    {
        //this is for writing temperature and acceleration to flash
        //Although this can be modified to write other quantites
        char *Flash_ptr = (char *)0xC714;       //Flash pointer starting position - this is determined based on where your code ends in memory
        write_flash(Flash_ptr);                 //Call function to clear/write temp to flash
    }
        }
}
