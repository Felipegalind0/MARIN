#ifndef __LCD_H
#define __LCD_H


#define LCDV_MID 60


// #define LCD_BTv_X 5
// #define LCD_BTv_Y 5 //220

// #define LCD_BTv_W 50
// #define LCD_BTv_H 20
// #define LCD_BTv_R 5


#define SM_X 30
#define SM_Y 170


#define Middle_M_X 35
#define Middle_M_Y 150


#define CORE_SM_X 10
#define CORE_SM_Y 185


#define LCD_IMU_SM_X 5
#define LCD_IMU_SM_Y 10


#define LCD_FELG_SM_X 40
#define LCD_FELG_SM_Y 190


#define LCD_ANGLE_X 45
#define LCD_ANGLE_Y 40

#define LCD_ANGLE_W 80
#define LCD_ANGLE_H 60
#define LCD_ANGLE_R 10 

#define ABORT_M_W 125
#define ABORT_M_H 60
#define ABORT_M_R 5

void LCD_loop();

void LCD_UI_Setup();
void LCD_CORE_Message();
void LCD_IMU_Message();
void LCD_Felg_Message();
void LCD_calib1_Message();
void LCD_calib1_complete_Message(bool isCalibrated);

void LCD_calib2_Message(void);

void LCD_DispBatVolt();
void LCD_DispAngle();
void LCD_DispRotation();
void LCD_Update_Mode();

void LCD_Abort_Message();

void LCD_Abort_PWR_Message();
void LCD_Abort_DEG_Message();

void LCD_Resume_from_Abort_Message();

void LCD_Western_Artificial_Horizon();

void LCD_IMU_Message();

void LCD_Status_Message();

void LCD_flush();

#endif