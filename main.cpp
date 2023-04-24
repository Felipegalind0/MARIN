/*
    BalaC balancing robot (IMU:MPU6886)

Robot Setup instructions:

1. Lay the robot flat, and power on.
2. Wait until Gal-1 (Pitch Gyro calibration) completes.
3. Hold still the robot upright in balance until Cal-2 (Accel & Yaw Gyro cal)
completes.

       short push of power button: Gyro calibration
long push (>1sec) of power button: switch mode between standig and demo(circle)

*/
#include "movement.h"
#include "systeminit.h"

#include <esp_now.h>
#include <WiFi.h>

TaskHandle_t Task0, Task1;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[7];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
}


void RealTcode( void * pvParameters ){
  Serial.print("RT_loop() running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    Movement_Loop(); //Main Loop
  }

}

void setup() {
  // Start systems
    SysInit_Setup();

    WiFi.mode(WIFI_AP);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);

    Serial.print("Setup() running on core ");
    Serial.println(xPortGetCoreID());

    Movement_Setup();

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                      RealTcode,   /* Task function. */
                      "Task0",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      1,           /* priority of the task */
                      &Task0,      /* Task handle to keep track of created task */
                      1);          /* pin task to core 0 */                  
    vTaskDelay(500); 

    

  // Validate StartUp
    SysInit_Check();
  
}

void loop() {
  // Serial.print("loop() running on core ");
  // Serial.println(xPortGetCoreID());
  //Movement_Loop();
}