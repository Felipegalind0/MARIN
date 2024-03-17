// wireless.cpp
#include <Arduino.h>

#include "creds.h"

#include "variables.h"
#include "IO.h"

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

#define msg_str_len 64

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message_r {
    byte i;
    char a[msg_str_len];
} struct_message_r;



typedef struct struct_message_s {
    byte i;
    char a[msg_str_len];
    //float robot_pitch;
} struct_message_s;


//AsyncWebServer server(80);

// Create a struct_message_r called myDataR
struct_message_r myDataR;
struct_message_s myDataS;

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  //Serial.println("Received Data...");

  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  
  //Serial.println(d);
}




void cartesianToPolar() {
    // Normalize x and y to have 0,0 at the center
    float x_normalized = JoyC_X - 50.0;
    float y_normalized = JoyC_Y - 50.0;

    // Calculate the radius
    JoyC_r = sqrt(x_normalized * x_normalized + y_normalized * y_normalized);

    // Calculate the angle in radians
    JoyC_Phi = atan2(y_normalized, x_normalized);

    // for angle in degrees, uncomment the following line
    JoyC_Phi = JoyC_Phi * (180.0 / M_PI);

    //Serial.println("r: " + String(JoyC_r) + " φ: " + String(JoyC_Phi));
}



//BAD
// void processCharArray() {
//   if (myDataR.a[1] == '-' || myDataR.a[1] == '+') {
//     if (myDataR.a[2] == '0' || myDataR.a[2] == '1') {
//       if (myDataR.a[3] >= '0' && myDataR.a[3] <= '9') {
//         int signX = (myDataR.a[1] == '-') ? -1 : 1;
//         int digit1X = myDataR.a[2] - '0';
//         int digit2X = myDataR.a[3] - '0';
//         x = signX * (digit1X * 10 + digit2X);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid X value");
//   }
//   if (myDataR.a[4] == '-' || myDataR.a[4] == '+') {
//     if (myDataR.a[5] == '0' || myDataR.a[5] == '1') {
//       if (myDataR.a[6] >= '0' && myDataR.a[6] <= '9') {
//         int signY = (myDataR.a[4] == '-') ? -1 : 1;
//         int digit1Y = myDataR.a[5] - '0';
//         int digit2Y = myDataR.a[6] - '0';
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
//   if (myDataR.a[1] == '-' || myDataR.a[1] == '+') {
//     if (myDataR.a[2] == '0' || myDataR.a[2] == '1') {
//       if (myDataR.a[3] >= '0' && myDataR.a[3] <= '9') {
//         int signX = (myDataR.a[1] == '-') ? -1 : 1;
//         int digit1X = myDataR.a[2] - '0';
//         int digit2X = myDataR.a[3] - '0';
//         x = signX * (digit1X * 10 + digit2X);
//       }
//     }
//   }
//   else {
//     Serial.println("Invalid X value: " + myDataR.a[1]);
//     return;
//   }
//   if (myDataR.a[4] == '-' || myDataR.a[4] == '+') {
//     if (myDataR.a[5] == '0' || myDataR.a[5] == '1') {
//       if (myDataR.a[6] >= '0' && myDataR.a[6] <= '9') {
//         int signY = (myDataR.a[4] == '-') ? -1 : 1;
//         int digit1Y = myDataR.a[5] - '0';
//         int digit2Y = myDataR.a[6] - '0';
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

  if(remote_msg == "ARM") {
    abortWasHandled = true;
    isArmed = true;
    return;

  }
  else if(remote_msg == "DISARM") {
    isArmed = false;
    abortWasHandled = true;
    return;

  }
  else if(remote_msg == "REQUEST_TAKEOFF") {
    isArmed = true;
    abortWasHandled = true;
    takeoffRequested = true;
    return;

  }
  else if (myDataR.a[command_index] == 'c') {
    //Serial.println("Valid command: " + String(myDataR.a[command_index]));
    JoyC_X = (myDataR.a[x_p10_index]- '0') * 10 + (myDataR.a[x_p1_index] - '0');  

    // decode the second int from the string and save it as y
    JoyC_Y = (myDataR.a[y_p10_index] - '0') * 10 + (myDataR.a[y_p1_index] - '0');

    // x = 2*JoyC_X - 100;
    // y = 2*JoyC_Y - 100;
    x = JoYC_X_Sensitivity * (2*JoyC_X - 100);
    y = JoYC_Y_Sensitivity * (2*JoyC_Y - 100);

    // Update the rotation and movement of the robot based on the X and Y values
    Movement_UpdateRotation(x);
    Movement_UpdateMovement(y);

    cartesianToPolar();
  }
  else {
    Serial.println("Invalid command: " + remote_msg);
    return;
  }
  
  // myDataR.a contains a string of 7 characters

  // myDataR.a[0] is the first character if it is a 'c' then it is a valid x_y command
  // examples: c+11+83, c+58+75, c+99+50, c+99+31, c+93+19, c+80+08, c+35+02, c+18+07, c+04+22, c+00+57, c+06+75, c+11+84, c+32+94, c+97+64, c+99+50, c+96+25, c+38+23, c+03+25

  // decode the first int from the string ( myDataR.a[1] and myDataR.a[2] ) and save it as x
  // By subtracting '0' from the char, we convert the char to an int, and then multiply by 10 to get the tens place
  
}

# define PRINT_SENT 1

# define DEBUG_BUFF 0
char buffer[6]; // Buffer to hold the formatted string, including the sign, three digits, and null terminator

String robot_pitch_str = "";

void sendData() {
  RED_LED(1);
  // Structure and data to send as before
  struct_message_r myDataR;

  // Create a string formatted as (+/-)XX(+/-)YY

  String message = "";

  if (remote_msg == "ARM") {
    message = "ARMED";
  }
  else if (remote_msg == "DISARM") {
    message = "DISARMED";
  }
  else if (remote_msg == "REQUEST_TAKEOFF") {
    message = "TAKING_OFF";
  }


  else {


    if (i_msg % 100 == 0) {
      message += "s";

      sprintf(buffer, "%+02d", isArmed);

      message += buffer;

      sprintf(buffer, "%+03d", perCentBatt);

      message += buffer;

    }
    else {
      message += "c";
    }


    if (JoyC_X < 10) {
      message += "+0";
    }
    else {
      message += "+";
    }
    message += String(JoyC_X);



    if (JoyC_Y < 10) {
      message += "+0";
    }
    else {
      message += "+";
    }
    message += String(JoyC_Y);

    sprintf(buffer, "%+04.0f", Avg_IMU_Z_deg_per_sec);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print("b: " + String(buffer));

    #endif


    sprintf(buffer, "%+04.0f", varAng);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print(" b: " + String(buffer));

    #endif


    sprintf(buffer, "%+04d", Lmotor);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print(" b: " + String(buffer));

    #endif


    sprintf(buffer, "%+04d", Rmotor);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print(" b: " + String(buffer));

    #endif

    sprintf(buffer, "%+03d", BackgroundTask_CPU_load);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print(" b: " + String(buffer));

    #endif

    sprintf(buffer, "%+03d", RealTcode_CPU_load);

    message += buffer;

    #if DEBUG_BUFF

    Serial.print(" b: " + String(buffer));

    #endif


  }

  

  // robot_pitch_str = " " + String(int(varAng));
  //robot_pitch_str = " " + buffer;
  //myDataS.robot_pitch = varAng;
  //robot_pitch_str = sscanf(robot_pitch_str.c_str(), " %hhx");
  //strcpy(myDataR.a, (if (x < 0) ? "" : "+") + String(x) + (if (y < 0) ? "" : "+") + String(y));

  // Move message to myDataR
  message.toCharArray(myDataS.a, msg_str_len);
  myDataS.i = i_msg;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataS, sizeof(myDataS));
  
  if (result == ESP_OK) {
    #if PRINT_SENT
    Serial.print("Sent: " + String(myDataS.a));
    #endif
  }
  else {
    Serial.print("Error sending the data: " + String(result) + "myDataR: " + String(myDataR.a));
    //Serial.print(String(*sender_mac, HEX) + ":" + String(*(sender_mac + 1), HEX) + ":" + String(*(sender_mac + 2), HEX) + ":" + String(*(sender_mac + 3), HEX) + ":" + String(*(sender_mac + 4), HEX) + ":" + String(*(sender_mac + 5), HEX));
  }
  RED_LED(0);

  should_reply_to_C_cmd = false;
}


#define DEBUG_converMacAddress 0
void convertMacAddress(const String &macStr, uint8_t *macAddr) {

   // Assumes macStr is in the format "XX:XX:XX:XX:XX:XX"
    sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
           &macAddr[0], &macAddr[1], &macAddr[2], &macAddr[3], &macAddr[4], &macAddr[5]);

    #if DEBUG_converMacAddress
    Serial.println("@convertMacAddress(" + macStr + ") = " + String(macAddr[0], HEX) + ":" + String(macAddr[1], HEX) + ":" + String(macAddr[2], HEX) + ":" + String(macAddr[3], HEX) + ":" + String(macAddr[4], HEX) + ":" + String(macAddr[5], HEX));
    #endif

   
}

String macToString(const uint8_t* mac) {
  char macStr[18]; // 17 for MAC + 1 for null terminator
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

# define PRINT_MAC_RECEIVED 0
# define PRINT_BYTES_RECEIVED 1
# define PRINT_CPU_RECIVED 0
# define PRINT_X_Y 0
// callback function that will be executed when data is receiveD

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  
  if (!remote_connected && sender_mac == 0x00) {
    
    //sender_mac_str = String(*mac, HEX); // BAD, DOES NOT WORK = 'c' instead of the MAC
    //sender_mac_str = String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[5], HEX);
    
    sender_mac_str = macToString(mac);

    Serial.println("Remote found :D MAC: " + sender_mac_str + "\n");
    
    // Convert Robot_MAC String to byte array
    convertMacAddress(sender_mac_str, broadcastAddress);
    
    // Setup ESPNOW peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;


    // Add peer
    esp_err_t addPeerResult = esp_now_add_peer(&peerInfo);
    if (addPeerResult != ESP_OK){
      Serial.println("Failed to add peer:" + sender_mac_str + "\n");
      return;
    }
    Serial.println("SUCCESS: Remote found :D MAC: " + sender_mac_str + "\n");
    sender_mac = *mac;
    remote_connected = true;

    Serial.println("Peer added");

    // robot_connected = true;

  }

  // if () {
  //   sender_mac = *mac;
  // }

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

  memcpy(&myDataR, incomingData, sizeof(myDataR));

  remote_msg = myDataR.a;
  i_msg = myDataR.i;

  #if PRINT_BYTES_RECEIVED
  Serial.print(myDataR.a);
  Serial.print(", ");
  Serial.print(len);
  #endif



  processCharArray(); // Call the function to process the received data

  should_reply_to_C_cmd = true;
  //sendData(); // Send the processed data back to the sender




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

void exec_Wireless_Setup( void * pvParameters) {
  RED_LED(1);
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

    wireless_status = INITIALIZED;
    RED_LED(0);
    vTaskDelete(NULL);
    return;
}

void Wireless_Setup(){

  if(wireless_status == OFF) {
    wireless_status = INITIALIZING;
    xTaskCreatePinnedToCore(
      exec_Wireless_Setup,   /* Function to implement the task */
      "exec_Wireless_Setup", /* Name of the task */
      10000,      /* Stack size in words */
      NULL,       /* Task input parameter */
      -2,          /* Priority of the task */
      NULL,      /* Task handle. */
      BackgroundCore);         /* Core where the task should run */

    
  }



}
