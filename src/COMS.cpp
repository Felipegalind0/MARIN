#include "variables.h"
//#include <WebSerial.h>
#include <HardwareSerial.h>

//HardwareSerial Serial2(2); // Use hardware serial 2 for the second connection

void serial_Init() {

  // Initialize the physical serial interface for debugging
  Serial.begin(115200);


  Serial2.begin(115200, SERIAL_8N1, 32, 33);
  // Serial2.setRxBufferSize(1024);
  // Serial2.setDebugOutput(true);
  // Serial2.println("Serial2 Initialized");
}

// void logData() {
//   yield();
//   Serial.print(gyroXdata);
//   Serial.print(",");
//   Serial.print(gyroYdata);
//   Serial.print(",");
//   Serial.print(gyroZdata);
//   Serial.print(",");
//   Serial.print(IMU_X_acceleration);
//   Serial.print(",");
//   Serial.print(IMU_Y_acceleration);
//   Serial.print(",");
//   Serial.println(IMU_Z_acceleration);
//   yield();
// }

void logData() {
  yield();
  Serial2.print(gyroXdata);
  Serial2.print(",");
  Serial2.print(gyroYdata);
  Serial2.print(",");
  Serial2.print(gyroZdata);
  Serial2.print(",");
  Serial2.print(IMU_X_acceleration);
  Serial2.print(",");
  Serial2.print(IMU_Y_acceleration);
  Serial2.print(",");
  Serial2.println(IMU_Z_acceleration);
  yield();
}



void sendStatus() {
    yield();
    Serial.print(" s");
    Serial.println(standing);
    Serial.print(" p");
    Serial.println(power);
    Serial.print(" a");
    Serial.println(varAng);
    yield();

    Serial.print(String(millis() - time0)+'\n');
}


