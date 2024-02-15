#include <M5StickCPlus.h>
#include "systeminit.h"
#include "speaker.h"
#include "IO.h"
#include "LCD.h"
#include "wireless.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "COMS.h"
#include "IO.h"
#include "variables.h"
#include "FileSystem.h"


void SysInit_Setup(void){

    M5.begin();

    serial_Init();

    Serial.println("SysInit_Setup() running on cpu#" + String(xPortGetCoreID()));

    fs_setup();


    //disableCore0WDT();

    
    Wire.begin(0, 26);  // SDA,SCL

    pinMode(LED, OUTPUT);
    pinMode(SPEAKER, OUTPUT);

    //ledcSetup(LED_CH, 5000, 8);
    ledcSetup(SPEAKER_CH, 5000, 8);
    //ledcAttachPin(LED, LED_CH);
    ledcAttachPin(SPEAKER, SPEAKER_CH);
    
    RED_LED(true);
    Serial.println("");
    Serial.println("Starting Up Systems");
    // Serial.print("Setup() running on core ");
    // Serial.println(xPortGetCoreID());
    RED_LED(false);
    
    LCD_UI_Setup();

    StartUp_Sound();

    RED_LED(true);
    Wireless_Setup();
    RED_LED(false);
    
    Serial.print("...");

    is_booted = true;
}


void SysInit_Check(void){
    digitalWrite(LED, HIGH);
    Serial.println("");
    Serial.println("StartUp Complete!");
    digitalWrite(LED, LOW);
}