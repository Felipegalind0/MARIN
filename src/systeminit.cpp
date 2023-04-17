#include <M5StickCPlus.h>
#include "systeminit.h"
#include "speaker.h"
#include "pinout.h"
#include "LCD.h"

void SysInit_Setup(void){

    //disableCore0WDT();

    M5.begin();
    Wire.begin(0, 26);  // SDA,SCL

    pinMode(LED, OUTPUT);
    pinMode(SPEAKER, OUTPUT);

    //ledcSetup(LED_CH, 5000, 8);
    ledcSetup(SPEAKER_CH, 5000, 8);
    //ledcAttachPin(LED, LED_CH);
    ledcAttachPin(SPEAKER, SPEAKER_CH);
    digitalWrite(LED, HIGH);

    Serial.println("");
    Serial.println("Starting Up Systems");

    LCD_Setup();

    CORE_Message();

    IMU_Message();

    Felg_Message();

    calib1_Message();

    StartUp_Sound();
    
    Serial.print("...");
    digitalWrite(LED, LOW);
}


void SysInit_Check(void){
    Serial.println("");
    Serial.println("StartUp Complete!");
}