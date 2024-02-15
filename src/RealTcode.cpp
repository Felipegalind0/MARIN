// RealTcode.cpp

#include "RealTcode.h"



// Function that contains code that needs to run in real-time
void exec_RealTcode(){ 

    if (!(is_booted)) { // If Device is not booted, run the setup code
    
    // Initialize web serial, I2C, LCD, speaker, microphone, and other systems
    SysInit_Setup();

    // Initialize movement systems, motors, gyro, etc.
    Movement_Setup();
    return;
  }

  else if (is_booted) { // If Device is booted, run the main code
    Movement_Loop(); // Call the movement loop for balancing and motion control
  }

  
}





// Function that contains code that needs to run in real-time
void RealTcode( void * pvParameters ){ 
  for(;;){ // Infinite loop for continuous execution

    //RealTcode_no_execution_time_end = esp_timer_get_time(); // Record the end time of the loop

    //RealTcode_start_time = micros(); // Record the start time of the loop
    RealTcode_start_time = esp_timer_get_time();; // Record the start time of the loop

    RealTcode_no_execution_time = RealTcode_start_time - RealTcode_end_time; // Calculate the time not spent executing the loop


    // Execute the real-time code
    exec_RealTcode();

    yield(); // Yield the processor to other tasks

    //RealTcode_end_time = micros(); // Record the end time of the loop
    RealTcode_end_time = esp_timer_get_time(); // Record the end time of the loop

    //Serial.print("@RealTcode: end_time = ");
    //Serial.println(RealTcode_end_time);

    // Calculate the time taken to execute the loop
    RealTcode_execution_time = RealTcode_end_time - RealTcode_start_time;

    //Serial.print("@RealTcode: Execution time = ");
    //Serial.println(RealTcode_execution_time);
    //Serial.println();

    counter += 1; // Increment the counter variable by 1 each iteration
    

    //RealTcode_no_execution_time_start = esp_timer_get_time(); // Record the start time of the loop

    // Give the semaphore to allow background tasks to run
    xSemaphoreGive(syncSemaphore);


    // Delay the task for a specific interval (in milliseconds) to control execution frequency
    vTaskDelay(pdMS_TO_TICKS(interval));

    //RealTcode_no_execution_time = RealTcode_no_execution_time_end - RealTcode_no_execution_time_start; // Calculate the time not spent executing the loop

    RealTcode_total_execution_time = RealTcode_execution_time + RealTcode_no_execution_time;

    if (RealTcode_total_execution_time != 0 && RealTcode_execution_time != 0) {

      // Calculate the CPU load
      RealTcode_CPU_load = (RealTcode_execution_time * 100/ RealTcode_total_execution_time);

      // Serial.print("@RealTcode_no_execution_time_start: ");
      // Serial.print(RealTcode_no_execution_time_start);
      // Serial.print(" @RealTcode_no_execution_time_end: ");
      // Serial.print(RealTcode_no_execution_time_end);

      // Serial.print("@RealTcode_no_execution_time: "); 
      // Serial.print(RealTcode_no_execution_time);
      // Serial.print(" @RealTcode_total_execution_time: ");
      // Serial.print(RealTcode_total_execution_time);
      // Serial.print(" @RealTcode_execution_time: ");
      // Serial.print(RealTcode_execution_time);
      // Serial.print(" @RealTcode: CPU load = ");
      // Serial.println(RealTcode_CPU_load);

    }
    
  }
}