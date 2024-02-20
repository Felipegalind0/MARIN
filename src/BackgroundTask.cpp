# include "BackgroundTask.h"
#include "wireless.h"

# define SEND_STATUS_OVER_SERIAL 0
void exec_BackgroundTask() {
  // Wait for the syncSemaphore to be given by the RealTcode task
  if (xSemaphoreTake(syncSemaphore, portMAX_DELAY) == pdTRUE) {



    // 
    if ((counter % btnCounter) == 0) {
      CheckButtons();
    }

    // 
    if (counter % logCounter == 0) {
      if (SEND_STATUS_OVER_SERIAL) {
        sendStatus();
        logData();
      }
    }

    M5.update();

    // Update the LCD display
    LCD_loop();

    if(should_reply_to_C_cmd){
      vTaskDelay(1000);
      sendData();
      should_reply_to_C_cmd = false;
    }
    

    // M5.BtnA.read();
    // M5.BtnB.read();

    if(M5.BtnA.wasPressed()){
      Serial.println("Button A was pressed");
      Abtn = 1;
      isArmed = !isArmed;
    }
    else {
      Abtn = 0;
    }

    if(M5.BtnB.wasPressed()){
      Serial.println("Button B was pressed");
      Bbtn = 1;
    }
    else {
      Bbtn = 0;
    }
  }
}




void BackgroundTask( void * pvParameters ) {
  for (;;) {  // Infinite loop for background task

    BackgroundTask_execution_time_start = esp_timer_get_time(); // Record the start time of the loop

    BackgroundTask_no_execution_time = BackgroundTask_execution_time_start - BackgroundTask_execution_time_end; // Calculate the time not spent executing the loop

    yield(); // Yield the processor to other tasks
    // Wait for the syncSemaphore to be given by the RealTcode task

    exec_BackgroundTask(); // Execute the background task

    // Record the end time of the loop
    BackgroundTask_execution_time_end = esp_timer_get_time();

    // Calculate the time taken to execute the loop
    BackgroundTask_execution_time = BackgroundTask_execution_time_end - BackgroundTask_execution_time_start;

    // Calculate the total execution time and CPU load
    BackgroundTask_total_execution_time = BackgroundTask_execution_time + BackgroundTask_no_execution_time;

    if (BackgroundTask_total_execution_time != 0 && BackgroundTask_execution_time != 0) {

      // Calculate the CPU load
      BackgroundTask_CPU_load = (BackgroundTask_execution_time * 100/ BackgroundTask_total_execution_time);

      // Serial.print("@BackgroundTask_no_execution_time_start: ");
      // Serial.print(BackgroundTask_no_execution_time_start);
      // Serial.print(" @BackgroundTask_no_execution_time_end: ");
      // Serial.print(BackgroundTask_no_execution_time_end);

      // Serial.print("@BackgroundTask_no_execution_time: "); 
      // Serial.print(BackgroundTask_no_execution_time);
      // Serial.print(" @BackgroundTask_total_execution_time: ");
      // Serial.print(BackgroundTask_total_execution_time);
      // Serial.print(" @BackgroundTask_execution_time: ");
      // Serial.print(BackgroundTask_execution_time);
      // Serial.print(" @BackgroundTask: CPU load = ");
      // Serial.println(BackgroundTask_CPU_load);

    }

    vTaskDelay(pdMS_TO_TICKS(interval)); // Delay the task for a specific interval (in milliseconds) to control execution frequency  
  }
}







