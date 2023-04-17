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

TaskHandle_t Task0;

// void RealTcode( void * pvParameters ){

//   Movement_Loop(); //Main Loop

// }

void setup() {
  // Start systems
    SysInit_Setup();

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