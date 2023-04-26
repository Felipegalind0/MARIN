#ifndef __LCD_H
#define __LCD_H


#define LCDV_MID 60


#define LCD_BTv_X 5
#define LCD_BTv_Y 5 //220

#define LCD_BTv_W 50
#define LCD_BTv_H 20
#define LCD_BTv_R 5


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


#define LCD_ANGLE_X 55
#define LCD_ANGLE_Y 10

#define LCD_ANGLE_W 80
#define LCD_ANGLE_H 60
#define LCD_ANGLE_R 10 


void LCD_Setup();
void LCD_CORE_Message();
void LCD_IMU_Message();
void LCD_Felg_Message();
void LCD_calib1_Message();

void LCD_DispBatVolt();
void LCD_DispAngle();
void LCD_Update_Mode();


#endif