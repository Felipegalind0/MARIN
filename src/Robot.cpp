# include "Robot.h"
# include "ArduinoJson.h"

void loadRobotJSON() {
    
    // read the file

    String robot_config_json = readFile(LittleFS, "/robot_config.json");

    // Parse the JSON object

    JsonDocument doc;


    DeserializationError error = deserializeJson(doc, robot_config_json);    
    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }

    robot_config_counter = doc["counter"];
    //gyro_deg_per_sec_X_offset = doc["Robot_gyro_deg_per_sec_X_offset"];
    gyro_deg_per_sec_Y_offset = doc["Robot_gyro_deg_per_sec_Y_offset"];
    gyro_deg_per_sec_Z_offset = doc["Robot_gyro_deg_per_sec_Z_offset"];
    
}



void SaveRobotJSON() {
    // Create a new JSON document
    JsonDocument doc;

    // Set the values in the document
    doc["counter"] = robot_config_counter+1;
    //doc["Robot_gyro_deg_per_sec_X_offset"] = gyro_deg_per_sec_X_offset;
    doc["Robot_gyro_deg_per_sec_Y_offset"] = gyro_deg_per_sec_Y_offset;
    doc["Robot_gyro_deg_per_sec_Z_offset"] = gyro_deg_per_sec_Z_offset;

    
}