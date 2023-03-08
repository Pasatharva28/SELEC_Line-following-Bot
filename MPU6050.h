/*
 * MPU6050.h
 *
 *  Created on: Feb 12, 2023
 *      Author: ATHARV
 */

#ifndef MPU6050_H_
#define MPU6050_H_

extern unsigned char RX_ByteCtr;
extern unsigned char TX_ByteCtr;

void initMPU6050();
void getMPU6050();
int getXAccel();
int getYAccel();
int getZAccel();
int getXAngle();
int getYAngle();
int getZAngle();
int getTemp();



#endif /* MPU6050_H_ */
