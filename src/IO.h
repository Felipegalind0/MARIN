#ifndef _IO_H
#define _IO_H

#include <M5StickCPlus.h>

#define LED      10
#define LED_CH    0

#define SPEAKER      2
#define SPEAKER_CH   1
#define N_CAL1   100
#define N_CAL2   100

void drvMotor(byte ch, int8_t sp);
void drvMotorR(int16_t pwm);
void drvMotorL(int16_t pwm);
void CheckButtons();
void RED_LED(bool state);
#endif