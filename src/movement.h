#ifndef _MOVEMENT_H
#define _MOVEMENT_H

#include <M5StickCPlus.h>
#include "variables.h"
#include "PID_v1.h"
#include "IO.h"
#include "LCD.h"
#include <MadgwickAHRS.h>
#include "speaker.h"

#include "lfs.h"

// Calibration status for each side
// enum Side { ROBOT_TOP_SIDE, BOTTOM, LEFT, RIGHT, ROBOT_FRONT_SIDE, ROBOT_BACK_SIDE, NONE };





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
void readRAWGyro();
void readRAWGyro();

void dispAngle();

void Movement_UpdateRotation(int x);
void Movement_UpdateMovement(int x);

void report_DEG_Abort();

void report_PWR_Abort();

void Abort(String MSG);

void calibrateIMU();

// extern struct CalibrationData;





void addIMUReadingsToBuffer();


void calculateOffsetsAndScalingFactors();

void calibrateIMU();




#endif 