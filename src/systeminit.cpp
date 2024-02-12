#include <M5StickCPlus.h>
#include "systeminit.h"
#include "speaker.h"
#include "IO.h"
#include "LCD.h"
#include "wireless.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


void SysInit_Setup(void){

    Serial.println("SysInit_Setup() running on cpu#" + String(xPortGetCoreID()));


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
    // Serial.print("Setup() running on core ");
    // Serial.println(xPortGetCoreID());
    digitalWrite(LED, LOW);
    
    LCD_UI_Setup();

    StartUp_Sound();

    digitalWrite(LED, HIGH);

    Wireless_Setup();
    
    Serial.print("...");


    // UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    // TaskStatus_t *pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));


    // if (pxTaskStatusArray != NULL) {
    //     // Generate raw status information about each task.
    //     UBaseType_t uxNumberOfTasks = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

    //     // Iterate over each task to print its details.
    //     for (UBaseType_t x = 0; x < uxNumberOfTasks; x++) {
    //         // Print task name, task number, task state, and core ID.
    //         printf("Task: %s, Task Number: %u, Task State: %u, Priority: %d\n",
    //             pxTaskStatusArray[x].pcTaskName,
    //             (unsigned int)pxTaskStatusArray[x].xTaskNumber,
    //             (unsigned int)pxTaskStatusArray[x].eCurrentState,
    //             (int)pxTaskStatusArray[x].uxCurrentPriority);
    //     }

    //     // Free the allocated memory once done.
    //     vPortFree(pxTaskStatusArray);
    // }


    digitalWrite(LED, LOW);
}


void SysInit_Check(void){
    digitalWrite(LED, HIGH);
    Serial.println("");
    Serial.println("StartUp Complete!");
    digitalWrite(LED, LOW);
}