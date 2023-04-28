/*
    BalaC balancing robot with IMU:MPU6886 

By Felipe Galindo



This code is based of the example BalaC with the following changes:

  - Improved Controller

      General Improvements to the PID, position estimation and balanceing 
      systems that enable the robot to be controlled at a high level. 

      Robot will stand up and minimize movement, then can be controlled by 2
      numbers. X = Rotation, Y = Foward/Backward.

      Both X and Y are numbers betweel -10 and+10


    * X represents (L/R) rotation rate on the robot. 

      By setting it to 0 robot will attempt to mantain heading. 

      Positive numbers make it turn right. Negatives make it turn left. 


    * Y represents (FWD/BACK) advance rate on the robot. 

      At 0 robot attempts to stay upright and stationary.

      Positive numbers make it move foward. Negative makes it move backwards. 


  - ESPNOW communications & remote control:
      Enables to robot to be remotely controlled by other robot or a human 
      with a remote controller. 

  - WebSerial: 
      Robot will create it's own WiFi Network and attempt connect to a 
      pre-defined network on which the Serial is avaliable at <ip-address>/webserial
      and at ? on the robot hosted WiFi

  - FreeRTOS Optimizations: 
      The arduino framework comes with a Default implementation of freRTOS.

      We optimized code 1 to run as a 'real-time' core, running code that 
      needs to be exectuted with tight deadlines or bad stuff will happen.
      Here we run code that 

      Core 0 is used by FreeRTOS for wireless COM & other background stuff. 
      A lot of the time this core is not busy so background tasks are delegated to
      this core whenever possible. 


Robot Setup instructions:

1. Lay the robot flat, and power on. DO NOT MOVE THE ROBOT AT ALL. 
    Robot will Display Startup Message, and "Executing Stationary Calibration" in red
    If you moved the robot please press the PWR/RESET button, to restart device. 

2. Wait until you see a "Stationary Calibration Complete" message. DO NOT MOVE ROBOT UNTIL COMPLETE
    The Stationary calibration is important, and your robot will not work if not calibrated properly.

3. Shortly after the first calibration is done Wireless Status when calibration is complete. 
    You may now move the robot if you'd like. On the display of the robot and the Physical serial interface(USB serial)
    you will see the IP address of the extenal network the robot connected to.
    
    WebSerial is now avaliable at <ip-address>/webserial
    and at ? on the robot hosted WiFi

3. Complete Balance calibration. 
    Your robot has an angle at which it balances itself. We need to find it to be able to
    make the robot stand up and not fall over. 

    Hold the robot at as close as possible to the balancing point. 

    Wait until the motors start up and the robot is balacing itself, then quickly let go. 

    Robot should stay upright. 

4. Web Camera
    After your robot is standing up the external web camera module should start to stream images. 
    Note that the WebCam module runs separate code on a separate micro controller.
    It might take longer or shorter for the WebCam to start
    The two systems are independent, this code runs on ESP 32, which hopefully will be able to communicate in the future and 
    receive instructions from the external camera but that is not added yet.

5. Remote controller 
    This robot can receive comments from a ESPN now enabled device. I haven't designed code for the M5stack MiniJoyC.
    This is a Joystick ESP32 MCU, and it sends commands formated like this:

    c-00-00 This is the 'zeroed' meesage, it includes '-' even through 0 is not negative for readability

    c-03-05 This would mean   x = -3  y = -5

    c+03-05 This would mean   x = 3  y = -5

    c-03+05 This would mean   x = -3  y = 5

    c+03+05 This would mean   x = 3  y = 5

    c means command, I might setup other letters to do other stuff

     -11 < (Both X & Y) < 11
        AKA -10 to 10 


short push of power button: Gyro calibration

long push (>1sec) of power button: switch mode between standig and demo(circle)

*/

#include "systeminit.h"
#include "IO.h"
#include "movement.h"
#include "variables.h"
#include "LCD.h"


TaskHandle_t Task0, Task1;

SemaphoreHandle_t syncSemaphore;


// Function that contains code that needs to run in real-time
void RealTcode( void * pvParameters ){ 
  for(;;){ // Infinite loop for continuous execution
    Movement_Loop(); // Call the movement loop for balancing and motion control

    counter += 1; // Increment the counter variable by 1 each iteration
    
    // Give the semaphore to allow background tasks to run
    xSemaphoreGive(syncSemaphore);
    
    // Delay the task for a specific interval (in milliseconds) to control execution frequency
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}



void BackgroundTask( void * pvParameters ){
  for(;;){ // Infinite loop for background task
    // Attempt to take the semaphore. Block indefinitely if it is not available.
    if(xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE){
      // Execute CheckButtons() if the counter is a multiple of 4
      if ((counter % 4) == 0) CheckButtons();
      // Execute the LCD_loop() function to handle the LCD display
      LCD_loop();
    }
    // Yield control to other tasks, allowing them to execute
    yield();
  }
}




// Setup Code
void Movement_Setup() {

    //StartUp IMU
    imuInit();

    //Zero Motors
    resetMotor();

    // Reset to Default Parameters
    resetPara();

    //Zero Out Variables
    resetVar();

    // Run Calibration1
    LCD_calib1_Message();
    calib1();
    LCD_calib1_complete_Message();
#ifdef DEBUG
    debugSetup();
#else
    setMode(false);
#endif
}






void setup() {
  // Start systems
  //setCpuFrequencyMhz(512);

  // Start Physical Serial
  Serial.begin(115200);

  syncSemaphore = xSemaphoreCreateBinary();

  // Start WebSerial, I2c, LCD, speaker, mic and other systems
  SysInit_Setup();

  // Start Movement Systems, Motors, Gyro, etc
  Movement_Setup();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    RealTcode,   /* Task function. */
                    "Task0",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */                  
  
  vTaskDelay(100); 

  xTaskCreatePinnedToCore(
                      BackgroundTask,   /* Task function. */
                      "Task1",          /* name of task. */
                      10000,            /* Stack size of task */
                      NULL,             /* parameter of the task */
                      -1,                /* priority of the task (lower than RealTcode) */
                      NULL,             /* Task handle to keep track of created task */
                      1);               /* pin task to core 1 */


  vTaskDelay(100);

  

  // Validate StartUp
    SysInit_Check();
  
}


void loop() {
  // Delete this task to save resources 
  vTaskDelete(NULL);
  // Serial.print("loop() running on core ");
  // Serial.println(xPortGetCoreID());
  //Movement_Loop();
}