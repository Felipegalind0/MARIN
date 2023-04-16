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

void setup() {
  // Start systems
    SysInit_Setup();

    Movement_Setup();

  // Validate StartUp
    SysInit_Check();
}

void loop() {
    Movement_Loop();
}