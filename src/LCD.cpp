#include <M5StickCPlus.h>
#include "pinout.h"
#include "LCD.h"

void LCD_Setup(){
    M5.Axp.ScreenBreath(11);
    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(SM_X, SM_Y);
    M5.Lcd.print("Starting Systems");
}

void CORE_Message(void){
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(CORE_SM_X, CORE_SM_Y);
    M5.Lcd.print("Core");
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(CORE_SM_X, CORE_SM_Y + 15);
    M5.Lcd.print(xPortGetCoreID());
}

void IMU_Message(void){
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(LCD_IMU_SM_X, LCD_IMU_SM_Y);
    M5.Lcd.print("Please Lay");
    M5.Lcd.setCursor(LCD_IMU_SM_X+3, LCD_IMU_SM_Y + 35);
    M5.Lcd.print("Robot Flat");
}

void Felg_Message(void){
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(LCD_FELG_SM_X+5, LCD_FELG_SM_Y);
    M5.Lcd.print("Designed By");
    M5.Lcd.setCursor(LCD_FELG_SM_X, LCD_FELG_SM_Y + 15);
    M5.Lcd.print("Felipe Galindo");
    M5.Lcd.setCursor(LCD_FELG_SM_X+3, LCD_FELG_SM_Y + 30);
    M5.Lcd.print("in Minnesota");
}

void calib1_Message(void){

    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(CLB1_M_X, CLB1_M_Y);
    M5.Lcd.print("Calibration");

    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(CLB1_M_X+20, CLB1_M_Y + 15);
    M5.Lcd.print("1");


}
