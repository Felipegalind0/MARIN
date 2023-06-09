#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <M5StickCPlus.h>

void Movement_Setup();
void Movement_Loop();

void imuInit();
void resetMotor();
void calib1();
void drvMotor(byte ch, int8_t sp);
void drvMotorL(int16_t pwm);
void drvMotorR(int16_t pwm);
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

void dispAngle();

void Movement_UpdateRotation(int x);
void Movement_UpdateMovement(int x);

void report_DEG_Abort();

void report_PWR_Abort();

void Abort(String MSG);


#endif 