#include <M5StickCPlus.h>
#include "pinout.h"
#include "variables.h"
#include "LCD.h"

void LCD_CORE_Message(void){
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(CORE_SM_X, CORE_SM_Y);
    M5.Lcd.print("Core");
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(CORE_SM_X, CORE_SM_Y + 15);
    M5.Lcd.print(xPortGetCoreID());
}

void LCD_IMU_Message(void){
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(LCD_IMU_SM_X, LCD_IMU_SM_Y);
    M5.Lcd.print("Please Lay");
    M5.Lcd.setCursor(LCD_IMU_SM_X+3, LCD_IMU_SM_Y + 35);
    M5.Lcd.print("Robot Flat");
}

void LCD_Felg_Message(void){
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);

    M5.Lcd.setCursor(LCD_FELG_SM_X+5, LCD_FELG_SM_Y);
    M5.Lcd.print("Designed By");

    M5.Lcd.setCursor(LCD_FELG_SM_X, LCD_FELG_SM_Y + 15);
    M5.Lcd.print("Felipe Galindo");

    M5.Lcd.setCursor(LCD_FELG_SM_X+3, LCD_FELG_SM_Y + 30);
    M5.Lcd.print("in Minnesota");
}

void LCD_calib1_Message(void){
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(RED);

    M5.Lcd.setCursor(CLB1_M_X, CLB1_M_Y);
    M5.Lcd.print("Executing");

    M5.Lcd.setCursor(CLB1_M_X, CLB1_M_Y + 15);
    M5.Lcd.print("Stationary");

    M5.Lcd.setCursor(CLB1_M_X, CLB1_M_Y + 30);
    M5.Lcd.print("Calibration");

    M5.Lcd.setTextColor(WHITE);
}

void LCD_Setup(){
    M5.Axp.ScreenBreath(11);
    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(WHITE);

    M5.Lcd.setCursor(SM_X, SM_Y);
    M5.Lcd.print("Starting Systems");
    
    LCD_CORE_Message();

    LCD_IMU_Message();

    LCD_Felg_Message();
}

void LCD_DispBatVolt() {
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);

    M5.Lcd.setCursor(LCD_BTv_X, LCD_BTv_Y);

    M5.Lcd.fillRoundRect(LCD_BTv_X-5, LCD_BTv_Y-2,
     LCD_BTv_W, LCD_BTv_H, LCD_BTv_R, BLUE);

    M5.Lcd.printf("%4.2fv ", vBatt);
}

void LCD_DispAngle() {
    M5.Lcd.setTextFont(6);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(LCD_ANGLE_X-15, LCD_ANGLE_Y);

    M5.Lcd.fillRoundRect(LCD_ANGLE_X, LCD_ANGLE_Y-10,
     LCD_ANGLE_W, LCD_ANGLE_H, LCD_ANGLE_R, GREEN);

    M5.Lcd.printf("%5.0f   ", (-aveAccZ)*90.0);
}

void LCD_Update_Mode(){
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(1);

    M5.Lcd.fillScreen(BLACK);

    M5.Lcd.setCursor(30, 5);
    
    if (demoMode == 0)
        M5.Lcd.print("Stand ");

    else if (demoMode == 1)
        M5.Lcd.print("Demo ");

}