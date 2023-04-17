#ifndef __LCD_H
#define __LCD_H

#define LCDV_MID 60

#define LCD_BTv_X 35
#define LCD_BTv_Y 220

#define SM_X 30
#define SM_Y 170

#define CLB1_M_X 35
#define CLB1_M_Y 100


#define CORE_SM_X 10
#define CORE_SM_Y 185

#define LCD_IMU_SM_X 5
#define LCD_IMU_SM_Y 10

#define LCD_FELG_SM_X 40
#define LCD_FELG_SM_Y 190

void LCD_Setup();
void CORE_Message();
void IMU_Message();
void Felg_Message();
void calib1_Message();

#endif