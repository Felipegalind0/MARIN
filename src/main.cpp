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

TaskHandle_t Task0, Task1;



/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  //WebSerial.println(d);
}

#include <esp_now.h>
#include <WiFi.h>



// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[7];
} struct_message;

int x = 0;
int y = 0;

// Create a struct_message called myData
struct_message myData;

void processCharArray() {
  if (myData.a[1] == '-' || myData.a[1] == '+') {
    if (myData.a[2] == '0' || myData.a[2] == '1') {
      if (myData.a[3] >= '0' && myData.a[3] <= '9') {
        int signX = (myData.a[1] == '-') ? -1 : 1;
        int digit1X = myData.a[2] - '0';
        int digit2X = myData.a[3] - '0';
        x = signX * (digit1X * 10 + digit2X);
      }
    }
  }
  if (myData.a[4] == '-' || myData.a[4] == '+') {
    if (myData.a[5] == '0' || myData.a[5] == '1') {
      if (myData.a[6] >= '0' && myData.a[6] <= '9') {
        int signY = (myData.a[4] == '-') ? -1 : 1;
        int digit1Y = myData.a[5] - '0';
        int digit2Y = myData.a[6] - '0';
        y = signY * -(digit1Y * 10 + digit2Y);
      }
    }
  }

  // Update the rotation and movement of the robot based on the X and Y values
  Movement_UpdateRotation(x);
  Movement_UpdateMovement(y);
}



// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  memcpy(&myData, incomingData, sizeof(myData));

  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Char: ");
  // Serial.println(myData.a);

  processCharArray(); // Call the function to process the received data

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("    Y: ");
  Serial.println(y);

}


void RealTcode( void * pvParameters ){ //Code that needs to run in real time
  Serial.print("RT_loop() running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    Movement_Loop(); //Main Loop
  }

}

void setup() {
  // Start systems
    SysInit_Setup();

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WiFi.softAP(AP_ssid);
    //WiFi.softAP(AP_ssid, AP_password);

    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.msgCallback(recvMsg);
    server.begin();

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
                      &Task1,      /* Task handle to keep track of created task */
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