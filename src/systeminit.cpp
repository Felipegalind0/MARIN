#include <M5StickCPlus.h>
#include "pinout.h"

void SysInit_Setup(void){
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    M5.begin();
    Wire.begin(0, 26);  // SDA,SCL
    M5.Axp.ScreenBreath(11);
    M5.Lcd.setRotation(2);
    M5.Lcd.setTextFont(4);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
}

void SysInit_Check(void){
    ;;
}