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
#include <Arduino.h>
#include "creds.h"

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80);



/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

TaskHandle_t Task0;


// void RealTcode( void * pvParameters ){

//   Movement_Loop(); //Main Loop

// }

void setup() {
  // Start systems
    SysInit_Setup();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.msgCallback(recvMsg);
    server.begin();

    Movement_Setup();

    // //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    // xTaskCreatePinnedToCore(
    //                   RealTcode,   /* Task function. */
    //                   "Task0",     /* name of task. */
    //                   10000,       /* Stack size of task */
    //                   NULL,        /* parameter of the task */
    //                   1,           /* priority of the task */
    //                   &Task1,      /* Task handle to keep track of created task */
    //                   0);          /* pin task to core 0 */                  
    // delay(500); 

    

  // Validate StartUp
    SysInit_Check();
}

void loop() {
  Movement_Loop();
}