
#include <Arduino.h>

#include "creds.h"

#include "variables.h"

#include "movement.h"

#include <esp_now.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  //#include <AsyncTCP.h>
#endif
//#include <ESPAsyncWebServer.h>
//#include <WebSerial.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[7];
} struct_message;


//AsyncWebServer server(80);

// Create a struct_message called myData
struct_message myData;

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  //Serial.println("Received Data...");

  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  
  //Serial.println(d);
}



//BAD
// void processCharArray() {
//   if (myData.a[1] == '-' || myData.a[1] == '+') {
//     if (myData.a[2] == '0' || myData.a[2] == '1') {
//       if (myData.a[3] >= '0' && myData.a[3] <= '9') {
//         int signX = (myData.a[1] == '-') ? -1 : 1;
//         int digit1X = myData.a[2] - '0';
//         int digit2X = myData.a[3] - '0';
//         x = signX * (digit1X * 10 + digit2X);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid X value");
//   }
//   if (myData.a[4] == '-' || myData.a[4] == '+') {
//     if (myData.a[5] == '0' || myData.a[5] == '1') {
//       if (myData.a[6] >= '0' && myData.a[6] <= '9') {
//         int signY = (myData.a[4] == '-') ? -1 : 1;
//         int digit1Y = myData.a[5] - '0';
//         int digit2Y = myData.a[6] - '0';
//         y = signY * -(digit1Y * 10 + digit2Y);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid Y value");
//   }

//   // Update the rotation and movement of the robot based on the X and Y values
//   Movement_UpdateRotation(x);
//   Movement_UpdateMovement(y);
// }


// BAD, DOES NOT WORK
// void processCharArray() {
//   if (myData.a[1] == '-' || myData.a[1] == '+') {
//     if (myData.a[2] == '0' || myData.a[2] == '1') {
//       if (myData.a[3] >= '0' && myData.a[3] <= '9') {
//         int signX = (myData.a[1] == '-') ? -1 : 1;
//         int digit1X = myData.a[2] - '0';
//         int digit2X = myData.a[3] - '0';
//         x = signX * (digit1X * 10 + digit2X);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid X value: " + myData.a[1]);
//     return;
//   }
//   if (myData.a[4] == '-' || myData.a[4] == '+') {
//     if (myData.a[5] == '0' || myData.a[5] == '1') {
//       if (myData.a[6] >= '0' && myData.a[6] <= '9') {
//         int signY = (myData.a[4] == '-') ? -1 : 1;
//         int digit1Y = myData.a[5] - '0';
//         int digit2Y = myData.a[6] - '0';
//         y = signY * -(digit1Y * 10 + digit2Y);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid Y value");
//     return;
//   }

//   x = 2 * x -100;
//   y = 2 * y -100; 

//   // Update the rotation and movement of the robot based on the X and Y values
//   Movement_UpdateRotation(x);
//   Movement_UpdateMovement(y);
// }
// @OnDataRecv(d4:d4:da:96:7e:0, c+11+83, 7 X: -78    Y: -3260)
// @OnDataRecv(d4:d4:da:96:7e:0, c+58+75, 7 X: -256    Y: -6620)
// @OnDataRecv(d4:d4:da:96:7e:0, c+99+50, 7 X: -612    Y: -13340)
// @OnDataRecv(d4:d4:da:96:7e:0, c+99+31, 7 X: -1324    Y: -26780)
// @OnDataRecv(d4:d4:da:96:7e:0, c+93+19, 7 X: -2748    Y: -138)
// @OnDataRecv(d4:d4:da:96:7e:0, c+80+08, 7 X: -5596    Y: -116)
// @OnDataRecv(d4:d4:da:96:7e:0, c+35+02, 7 X: -11292    Y: -104)
// @OnDataRecv(d4:d4:da:96:7e:0, c+18+07, 7 X: -64    Y: -114)
// @OnDataRecv(d4:d4:da:96:7e:0, c+04+22, 7 X: -92    Y: -328)
// @OnDataRecv(d4:d4:da:96:7e:0, c+00+57, 7 X: -100    Y: -756)
// @OnDataRecv(d4:d4:da:96:7e:0, c+06+75, 7 X: -88    Y: -1612)
// @OnDataRecv(d4:d4:da:96:7e:0, c+11+84, 7 X: -78    Y: -3324)
// @OnDataRecv(d4:d4:da:96:7e:0, c+32+94, 7 X: -256    Y: -6748)
// @OnDataRecv(d4:d4:da:96:7e:0, c+97+64, 7 X: -612    Y: -13596)
// @OnDataRecv(d4:d4:da:96:7e:0, c+99+50, 7 X: -1324    Y: -27292)
// @OnDataRecv(d4:d4:da:96:7e:0, c+96+25, 7 X: -2748    Y: -54684)
// @OnDataRecv(d4:d4:da:96:7e:0, c+38+23, 7 X: -5596    Y: -109468)
// @OnDataRecv(d4:d4:da:96:7e:0, c+03+25, 7 X: -94    Y: -219036)


# define command_index 0  // c
# define x_sign_index 1   // + or -
# define x_p10_index 2    // 0-9
# define x_p1_index 3     // 0-9
# define y_sign_index 4   // + or -
# define y_p10_index 5    // 0-9
# define y_p1_index 6     // 0-9

// fixed function

float JoYC_X_Sensitivity = 0.1;
float JoYC_Y_Sensitivity = -0.1;
void processCharArray() {
  
  // myData.a contains a string of 7 characters

  // myData.a[0] is the first character if it is a 'c' then it is a valid x_y command
  // examples: c+11+83, c+58+75, c+99+50, c+99+31, c+93+19, c+80+08, c+35+02, c+18+07, c+04+22, c+00+57, c+06+75, c+11+84, c+32+94, c+97+64, c+99+50, c+96+25, c+38+23, c+03+25

  // decode the first int from the string ( myData.a[1] and myData.a[2] ) and save it as x
  // By subtracting '0' from the char, we convert the char to an int, and then multiply by 10 to get the tens place
  JoyC_X = (myData.a[x_p10_index]- '0') * 10 + (myData.a[x_p1_index] - '0');  

  // decode the second int from the string and save it as y
  JoyC_Y = (myData.a[y_p10_index] - '0') * 10 + (myData.a[y_p1_index] - '0');

  // x = 2*JoyC_X - 100;
  // y = 2*JoyC_Y - 100;
  x = JoYC_X_Sensitivity * (2*JoyC_X - 100);
  y = JoYC_Y_Sensitivity * (2*JoyC_Y - 100);

  // Update the rotation and movement of the robot based on the X and Y values
  Movement_UpdateRotation(x);
  Movement_UpdateMovement(y);
}




# define PRINT_MAC_RECEIVED 0
# define PRINT_BYTES_RECEIVED 0
# define PRINT_CPU_RECIVED 0
# define PRINT_X_Y 0
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  // print CPU# OnDataRecv(A8:0D:3A:3B:3C:3D, "example msg", 11)

  #if PRINT_CPU_RECIVED
  Serial.print("CPU" + String(xPortGetCoreID()) + " ");
  #endif

  #if PRINT_BYTES_RECEIVED || PRINT_X_Y || PRINT_MAC_RECEIVED
  Serial.print("@OnDataRecv(");
  #endif

  #if PRINT_MAC_RECEIVED
  Serial.print(String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[5], HEX) + ", ");
  #endif

  memcpy(&myData, incomingData, sizeof(myData));

  #if PRINT_BYTES_RECEIVED
  Serial.print(myData.a);
  Serial.print(", ");
  Serial.print(len);
  #endif


  processCharArray(); // Call the function to process the received data


  #if PRINT_X_Y
  Serial.print(" X: ");
  Serial.print(x);
  Serial.print("    Y: ");
  Serial.print(y);
  #endif

  #if PRINT_BYTES_RECEIVED || PRINT_X_Y || PRINT_MAC_RECEIVED
  Serial.println(")");
  #endif

}

void Wireless_Setup(){

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

    //WebSerial.begin(&server);
    /* Attach Message Callback */
    //WebSerial.msgCallback(recvMsg);
    //server.begin();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);

}
