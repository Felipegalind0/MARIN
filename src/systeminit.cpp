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
#include "movement.h"


void SysInit_Setup(void){

    M5.begin();

    serial_Init();

    Serial.println("SysInit_Setup() running on cpu#" + String(xPortGetCoreID()));

    //disableCore0WDT();

    
    Wire.begin(0, 26);  // SDA,SCL

    pinMode(LED, OUTPUT);
    
    Setup_Speaker();


    
    RED_LED(true);
    Serial.println("");
    Serial.println("Starting Up Systems");
    // Serial.print("Setup() running on core ");
    // Serial.println(xPortGetCoreID());
    RED_LED(false);
    
    LCD_UI_Setup();

    // Make backgroundCore Play the StartUp Sound in the background
    xTaskCreatePinnedToCore(
        StartUp_Sound,   /* Function to implement the task */
        "StartUp_Sound", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        -2,          /* Priority of the task */
        NULL,       /* Task handle. */
    BackgroundCore);  /* Core where the task should run */

    fs_setup();

    RED_LED(true);
    Wireless_Setup();
    RED_LED(false);

    //Zero Motors
    resetMotor();

    // Reset to Default Parameters
    resetPara();

    //Zero Out Variables
    resetVar();
    
    Serial.print("...");

    is_booted = true;
}


void SysInit_Check(void){
    digitalWrite(LED, HIGH);
    Serial.println("");
    Serial.println("StartUp Complete!");
    digitalWrite(LED, LOW);
}