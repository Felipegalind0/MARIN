// Adapted from: https://github.com/espressif/arduino-esp32/blob/master/libraries/LittleFS/examples/LITTLEFS_test/LITTLEFS_test.ino
// Project details: https://RandomNerdTutorials.com/esp32-write-data-littlefs-arduino/

//  You only need to format LittleFS the first time you run a
//  test or else use the LITTLEFS plugin to create a partition 
//  https://github.com/lorol/arduino-esp32littlefs-plugin

#include "FileSystem.h"

#define FORMAT_LITTLEFS_IF_FAILED true

bool robot_config_exists(){
    return LittleFS.exists("/robot_config.json");
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    //Serial.printf("Listing directory: %s\r\n", dirname);
    Serial.printf("\n@listDir(%s, %d)\n", dirname, levels);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    //Serial.printf("Creating Dir: %s\n", path);
    Serial.printf("\n@createDir(%s)\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    //Serial.printf("Removing Dir: %s\n", path);
    Serial.printf("\n@removeDir(%s)\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

String readFile(fs::FS &fs, const char * path){
    //Serial.printf("Reading file: %s\r\n", path);
    Serial.printf("\n@readFile(%s)\n", path);

    String file_contents = "";

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return "FAILED TO OPEN FILE: " + String(path);
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
        file_contents += file.read();
    }
    file.close();

    return file_contents;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\r\n", path);
    Serial.printf("\n@writeFile(%s, %s)\n", path, message);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\r\n", path);
    Serial.printf("\n@appendFile(%s, %s)\n", path, message);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    //Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    Serial.printf("\n@renameFile(%s, %s)\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("\n@testFileIO(%s)\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}

void fs_setup(){

    //vTaskDelay(4000);

    //deleteFile(LittleFS, "/robot_config/robot_config.txt");

    if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
        Serial.println("LittleFS Mount Failed");
        return;
    }

    if (robot_config_exists()){
        Serial.println("Robot is configured");

    } else {
        Serial.println("Robot is not configured");
    }



    listDir(LittleFS, "/", 2); // List the root directory up to 2 levels


    //createDir(LittleFS, "/robot_config"); // Create a robot_config folder

    //writeFile(LittleFS, "/robot_config/robot_config.txt", "Robot Configuration File"); // Create and write a new file in the robot_config folder

    // createDir(LittleFS, "/mydir"); // Create a mydir folder
    // writeFile(LittleFS, "/mydir/hello1.txt", "Hello1"); // Create a hello1.txt file with the content "Hello1"
    // listDir(LittleFS, "/", 1); // List the directories up to one level beginning at the root directory
    // deleteFile(LittleFS, "/mydir/hello1.txt"); //delete the previously created file
    // removeDir(LittleFS, "/mydir"); //delete the previously created folder
    // listDir(LittleFS, "/", 1); // list all directories to make sure they were deleted
    
    // writeFile(LittleFS, "/hello.txt", "Hello "); //Create and write a new file in the root directory
    // appendFile(LittleFS, "/hello.txt", "World!\r\n"); //Append some text to the previous file
    // readFile(LittleFS, "/hello.txt"); // Read the complete file
    // renameFile(LittleFS, "/hello.txt", "/foo.txt"); //Rename the previous file
    // readFile(LittleFS, "/foo.txt"); //Read the file with the new name
    // deleteFile(LittleFS, "/foo.txt"); //Delete the file
    // testFileIO(LittleFS, "/test.txt"); //Testin
    // deleteFile(LittleFS, "/test.txt"); //Delete the file
  
    Serial.println( "Test complete" ); 
}