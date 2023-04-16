#ifndef __MOVEMENT_H_
#define __MOVEMENT_H_

#include <M5StickCPlus.h>

void Movement_Setup();
void Movement_Loop();

void imuInit();
void resetMotor();
void resetPara();
void resetVar();
void calib1();
void drvMotor(byte ch, int8_t sp);
void drvMotorL(int16_t pwm);
void drvMotorR(int16_t pwm);
void setMode(bool inc);
void dispBatVolt();
void getGyro();
void checkButtonP();
void calib2();
void startDemo();
void startDemo();
void drive();
void calDelay(int n);
void startDemo();
void sendStatus();
void readGyro();
void readGyro();

#endif 