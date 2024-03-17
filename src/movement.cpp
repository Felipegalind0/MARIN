// ---------------------movement.cpp---------------------


#include "movement.h"

#include "SensorFusion.h"

SF fusion;



//#include <WebSerial.h>

// import NVS library

//Madgwick filter;

// enum Robot_Side {
//     ROBOT_TOP_SIDE,
//     ROBOT_BOTTOM_SIDE,
//     ROBOT_FRONT_SIDE,
//     ROBOT_BACK_SIDE,
//     ROBOT_BACK_LEFT,
//     ROBOT_BACK_RIGHT,
//     ROBOT_UNKNOWN_SIDE
// };


// using standard cube side enum
enum Robot_Side {
    ROBOT_FRONT_SIDE,   // 0
    ROBOT_BACK_SIDE,    // 1
    ROBOT_LEFT_SIDE,    // 2
    ROBOT_RIGHT_SIDE,   // 3
    ROBOT_TOP_SIDE,     // 4
    ROBOT_BOTTOM_SIDE,  // 5
    ROBOT_UNKNOWN_SIDE  // 6
};


// enum CalibrationState {
//     INIT,
//     WAIT_FOR_STABILITY,
//     TOP_SIDE_STATE,
//     BOTTOM_SIDE_STATE,
//     FRONT_SIDE_STATE,
//     BACK_SIDE_STATE,
//     BACK_LEFT_STATE,
//     BACK_RIGHT_STATE,
//     COMPLETED
// };



// using standard cube side enum
enum CalibrationState {
    FRONT_SIDE_STATE,
    BACK_SIDE_STATE,
    LEFT_SIDE_STATE,
    RIGHT_SIDE_STATE,
    TOP_SIDE_STATE,
    BOTTOM_SIDE_STATE,
    UNKNOWN_SIDE_STATE,
    INIT,
    WAIT_FOR_STABILITY,
    COMPLETED
};




// Setup Code
void Movement_Setup() {



    //calibrateIMU();





    //StartUp IMU
    //imuInit();

    

    // Run Calibration1
    //LCD_calib1_complete_Message(false);
    calib1();
    

    // Add to Movement_Setup() in movement.cpp

    // // Instantiate PID controllers
    // PID pitchPID(&inputPitch, &outputPitch, &setpointPitch, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);
    // PID yawPID(&inputYaw, &outputYaw, &setpointYaw, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);

    // // Initialize PID controllers
    // pitchPID.SetMode(AUTOMATIC);
    // yawPID.SetMode(AUTOMATIC);

  

// #ifdef DEBUG
//     debugSetup();
// #else
//     setMode(false);
// #endif
}




// Update the rotation rate and direction based on the input rotation value
void Movement_UpdateRotation(int rotation) {
  moveRate = 0.0;
  spinContinuous = false;
  spinStep = 0.0;

  // Normalize the rotation value to be within the range of -1.0 and 1.0
  float normalizedRotation = rotation / 10.0;

  // Set the movement rate based on the normalized rotation value
  moveRate = abs(normalizedRotation);

  // Set the spinContinuous flag to true if the rotation value is non-zero
  spinContinuous = rotation != 0;

  // Set the spinStep based on the normalized rotation value and the clock value
  spinStep = -69.0 * normalizedRotation * clk;
}

// Update the movement rate and direction based on the input movement value
void Movement_UpdateMovement(int movement) {
  // Normalize the movement value to be within the range of -2.0 and 2.0
  float normalizedMovement = movement / 5.0;

  // Set the movement rate based on the normalized movement value
  moveRate = normalizedMovement;
}

boolean pointedUp = false;

#define Takeoff_Pitch_Up_Time 1000
// Main Loop 
void Movement_Loop() {
    
    #ifdef DEBUG
        if (debugLoop1()) return;
    #endif

    getGyro();



    if(isArmed){

        pointedUp = abs(Avg_IMU_Z_Gs) > 0.9 && abs(Avg_Robot_Z_deg_per_sec) < 1.5;

        if (takeoffRequested) {
            if (!takeoffTime){ // if has not taken off
                takeoffTime = millis();
                // drive both motors forward
                drvMotorR(200);
                drvMotorL(200);
                
            }

            else if (pointedUp) {
                takeoffRequested = false;
                abortWasHandled = false;
                standing = true;
                drive(); 
            }

            // else if (millis() - takeoffTime < Takeoff_Pitch_Up_Time){

            // }

            else if (millis() - takeoffTime > Takeoff_Pitch_Up_Time ){ //if failed to pitch up in time 
                // drive both motors forward
                drvMotorR(0);
                drvMotorL(0);
                takeoffRequested = false;
                takeoffTime = 0;
                isArmed = false;
            }
        }

        
        else if (!standing) { // If Robot is not Standing

            //Avg_Robot_Z_deg_per_sec = Avg_Robot_Z_deg_per_sec * 0.9 + IMU_Y_deg_per_sec * 0.1; // update average angular velocity
            

            

            // 0.9*90= 72
            //if Pointed 72 deg up 
            if (pointedUp) {

                // Run Second Calibration
                LCD_calib2_Message();
                
                calib2();

                // if (demoMode == 1) startDemo();

                standing = true;
                abortWasHandled = false;
            }
        } 





        else { // Check if Robot has fallen and disable

            if (abs(varAng) > maxAngle) {
                Abort("Max DEG");
            } 

            else if (counterOverPwr > maxOvp) { 
                Abort("Max PWR");
            } 

            else { //Robot is okay to drive

                drive(); // What makes it move
            }

        }


    }
    else { // If Robot is not armed
        resetMotor();
        standing = false;
    }

    

}


void report_DEG_Abort(){
    LCD_Abort_DEG_Message();
    LCD_flush();
}

void report_PWR_Abort(){
    LCD_Abort_PWR_Message();
    LCD_flush();
}

void Abort(String MSG){

    exec_status = "ABORT: " + MSG;

    LCD_Abort_Message();

    if(MSG == "Max DEG") report_DEG_Abort();

    else if(MSG == "Max PWR") report_PWR_Abort();

    resetMotor();

    Serial.print(MSG);

    standing = false;
    isArmed = false;
    hasFallen = true;

    while(!abortWasHandled){

        M5.update();
        vTaskDelay(500);
        ledcWriteTone(SPEAKER_CH,911);
        digitalWrite(LED, HIGH);
        CheckButtons();
        vTaskDelay(50);
        digitalWrite(LED, LOW);
        ledcWriteTone(SPEAKER_CH,0);

        if(M5.BtnA.wasPressed()){
            Serial.println("Button A was pressed");
            Abtn = 1;
            abortWasHandled = true;
        }
    }

    LCD_Resume_from_Abort_Message();
    vTaskDelay(1000);


    Movement_Setup();
    
}


    

// First Calibration
void calib1() {


    //LCD_calib1_Message();
    //exec_status = "IMU Calibration";

    
    update_exec_status("Starting IMU Calibration");
    LCD_Status_Message();
    RED_LED(1);

    calDelay(150);

    gyro_deg_per_sec_X_offset = 0.0;
    gyro_deg_per_sec_Y_offset = 0.0;
    gyro_deg_per_sec_Z_offset = 0.0;

    
    for (int i = 0; i < N_CAL1; i++) {
        readRAWGyro();
        gyro_deg_per_sec_X_offset += IMU_RAW_X_dps;
        gyro_deg_per_sec_Y_offset += IMU_RAW_Y_dps;
        gyro_deg_per_sec_Z_offset += IMU_RAW_Z_dps;
        update_exec_status("g_Y = " + String(IMU_RAW_Y_dps));
        LCD_Status_Message();
        vTaskDelay(9);
    }

    gyro_deg_per_sec_X_offset /= (float)N_CAL1;
    gyro_deg_per_sec_Y_offset /= (float)N_CAL1;
    gyro_deg_per_sec_Z_offset /= (float)N_CAL1;

    update_exec_status("g_Y_offset = " + String(gyro_deg_per_sec_Y_offset));
    LCD_Status_Message();
    
    M5.Lcd.fillScreen(BLACK);

    RED_LED(0);
    //LCD_calib1_complete_Message(true);

    
    IMU_has_been_calibrated = true;
}

// Second Calibration
void 
calib2() {
    resetVar();
    resetMotor();
    digitalWrite(LED, HIGH);
    calDelay(80);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.println(" Cal-2  ");
    accXoffset  = 0.0;
    gyro_deg_per_sec_Z_offset = 0.0;
    //gyro_deg_per_sec_Z_offset = 0.0;
    for (int i = 0; i < N_CAL2; i++) {
        readRAWGyro();
        accXoffset += IMU_RAW_X_Gs;
        gyro_deg_per_sec_Z_offset += IMU_RAW_Z_dps;
        vTaskDelay(9);
    }
    accXoffset /= (float)N_CAL2;
    gyro_deg_per_sec_Z_offset /= (float)N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, LOW);
}



void calDelay(int n) {
    for (int i = 0; i < n; i++) {
        readRAWGyro();
        vTaskDelay(9);
    }
}

// void startDemo() {
//     moveRate       = 1.0;
//     spinContinuous = true;
//     spinStep       = -40.0 * clk;
// }

float deltat;

// float prev_yaw = 0;

// float delta_yaw = 0;


void getGyro() {
    readRAWGyro();
    
    IMU_X_deg_per_sec = IMU_RAW_X_dps - gyro_deg_per_sec_X_offset; // unit:deg/sec
    IMU_Y_deg_per_sec = IMU_RAW_Y_dps - gyro_deg_per_sec_Y_offset;// unit:deg/sec AKA var0mg
    IMU_Z_deg_per_sec = IMU_RAW_Z_dps - gyro_deg_per_sec_Z_offset; // unit:deg/sec AKA yawAngle

    Avg_IMU_X_deg_per_sec = Avg_IMU_X_deg_per_sec * 0.9 + IMU_X_deg_per_sec * 0.1;  // update average y angular velocity
    Avg_IMU_Y_deg_per_sec = Avg_IMU_Y_deg_per_sec * 0.9 + IMU_Y_deg_per_sec * 0.1;  // update average y angular velocity
    Avg_IMU_Z_deg_per_sec = Avg_IMU_Z_deg_per_sec * 0.9 + IMU_Z_deg_per_sec * 0.1;  // update average z angular velocity

    Avg_IMU_X_Gs   = Avg_IMU_X_Gs * 0.9 + IMU_RAW_X_Gs * 0.1;  // update average x acceleration
    Avg_IMU_Y_Gs   = Avg_IMU_Y_Gs * 0.9 + IMU_RAW_Y_Gs * 0.1;  // update average y acceleration 
    Avg_IMU_Z_Gs   = Avg_IMU_Z_Gs * 0.9 + IMU_RAW_Z_Gs * 0.1;  // update average z acceleration


    //calculate the x, y, z angles
    //robot_Y_deg = (((Avg_IMU_X_Gs) ? -1 : 1) * 90 * Avg_IMU_X_Gs);  // unit:deg

    yawAngle += Avg_IMU_Z_deg_per_sec * clk;  // unit:deg

    // calculate the heading, as yawAngle mod 360
    heading = (byte)(round(yawAngle));


    varAng += (IMU_Y_deg_per_sec + ((IMU_RAW_X_Gs - accXoffset) * 57.3 - varAng) * cutoff) *
              clk;  // complementary filter



    // // Convert gyroscope data from degrees per second to radians per second
    //   gX = gX * PI / 180.0;
    //   gY = gY * PI / 180.0;
    //   gZ = gZ * PI / 180.0;

        // // Convert gyro data from degrees per second to radians per second
        // gX = gX * RAD_TO_DEG;
        // gY = gY * RAD_TO_DEG;
        // gZ = gZ * RAD_TO_DEG;

    //BAD, need to print constant length strings otherwise it is impossible to read
    //Serial.println("gX: " + String(gX) + " gY: " + String(gY) + " gZ: " + String(gZ) + " aX: " + String(aX) + " aY: " + String(aY) + " aZ: " + String(aZ));

        // Fixed, using ranges: (+/-)XXX for gX and gY, +/-(XX) for aX, aY, and aZ

        // char print_buffer[100];

        // //String print_buffer;

        // sprintf(print_buffer, "gX: %04d gY: %04d gZ: %04d aX: %02.02f aY: %02.02f aZ: %02.02f", (int)gX, (int)gY, (int)gZ, aX, aY, aZ);

        //Serial.println(print_buffer);


    // filter.updateIMU(gX, gY, gZ, aX, aY, aZ);

    // // Get the roll, pitch, and yaw from the filter
    //   roll = filter.getRoll();
    //   pitch = filter.getPitch();
    //   yaw = filter.getYaw();

    //Serial.println("R: " + String(roll) + " P: " + String(pitch) + " Y: " + String(yaw));

    //Serial.println(" P: " + String(pitch) + " Y: " + String(delta_yaw));

    //Serial.println(" varAng: " + String(varAng) + " p: " + String(pitch));


    #define RAW_DATA 1

    #if RAW_DATA
    // Serial << "From last Update:\t"; Serial.println(deltat, 6);
    // Serial << "GYRO:\tx:" << gX << "\t\ty:" << gY << "\t\tz:" << gZ << '\n';
    // Serial << "ACC:\tx:" << aX << "\t\ty:" << aY << "\t\tz:" << aZ << '\n';
    // Serial << "MAG:\tx:" << mX << "\t\ty:" << mY << "\t\tz:" << mZ << '\n';
    //Serial << "TEMP:\t" << temp << newl << newl;
    #endif

    deltat = fusion.deltatUpdate();
    //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
    //fusion.MadgwickUpdate(gX, gY, gZ, aX, aY, aZ, deltat);  //else use the magwick
    fusion.MadgwickUpdate(IMU_X_deg_per_sec * DEG_TO_RAD, IMU_Y_deg_per_sec * DEG_TO_RAD, IMU_Z_deg_per_sec * DEG_TO_RAD, Avg_IMU_X_Gs, Avg_IMU_Y_Gs, Avg_IMU_Z_Gs, deltat);  //else use the magwick

    //prev_yaw = yaw;

    roll = fusion.getRoll();
    pitch = fusion.getPitch();
    yaw = fusion.getYaw();

    // delta_yaw = yaw - prev_yaw;

    // // check if 0 

    // if (abs(delta_yaw) > 180) {
    //     if (delta_yaw > 0) {
    //         delta_yaw -= 360;
    //     } else {
    //         delta_yaw += 360;
    //     }
    // }

    // robot_yaw += delta_yaw;

    #define EULER_DATA 0

    #if EULER_DATA
    Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << newl << newl;
    #endif

    #define PROCESSING 0

    #if PROCESSING
    roll = fusion.getRollRadians();
    pitch = fusion.getPitchRadians();
    yaw = fusion.getYawRadians();
    Serial  << pitch << ":" << roll << ":" << yaw << newl;
    #endif


    #define SERIAL_PLOTER 1

    // #ifdef SERIAL_PLOTER
    // Serial << String(pitch) << " " << String(roll) << " " << String(yaw) << endl;
    // #endif


    //Serial.println("Pitch: " + String(pitch) + " Roll: " + String(roll) + " Yaw: " + String(yaw));
    




}







void readRAWGyro() {
    // Declare variables for gyro and accelerometer data
    //read the MPU6886

    // Get the gyro data for the X, Y, and Z axes
    M5.Imu.getGyroData(&gX, &gY, &gZ);

    // Get the accelerometer data for the X, Y, and Z axes
    M5.Imu.getAccelData(&aX, &aY, &aZ);


      // Assign the gyro data to the corresponding global variables
    // (Note the sign changes to match the robot's coordinate system)
    IMU_RAW_Y_dps = gX;  // y = x
    IMU_RAW_Z_dps = -gY; // z = -y
    IMU_RAW_X_dps = -gZ; // x = -z

    // Assign the accelerometer data to the corresponding global variables
    // (Note the change in variable usage for aX)
    IMU_RAW_X_Gs = aZ; // x = z
    IMU_RAW_Y_Gs = aX; // y = x
    IMU_RAW_Z_Gs = aY; // z = y
}


void drive() { //spinStep and moveRate map to X and Y in the JoyC
#ifdef DEBUG
    debugDrive();
#endif
    if (abs(moveRate) > 0.1)
        spinFact = constrain(-(powerR + powerL) / 10.0, -1.0, 1.0);  // moving
    else
        spinFact = 1.0;  // standing
    if (spinContinuous)
        spinTarget += spinStep * spinFact;
    else {
        if (spinTarget < spinDest) spinTarget += spinStep;
        if (spinTarget > spinDest) spinTarget -= spinStep;
    }
    moveTarget += moveStep * (moveRate + (float)fbBalance / 100.0);
    varSpd += power * clk;
    varDst += Kdst * (varSpd * clk - moveTarget);
    varIang += KIang * varAng * clk;
    I_y_dps += KI_y_dps * IMU_Y_deg_per_sec * clk;
    power =
        varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * IMU_Y_deg_per_sec) + I_y_dps;
    if (abs(power) > 1000.0)
        counterOverPwr += 1;
    else
        counterOverPwr = 0;
    if (counterOverPwr > maxOvp) return;
    power    = constrain(power, -maxPwr, maxPwr);


    yawPower = (yawAngle - spinTarget) * Kyaw;


    //yawPower = 0;


    powerR   = power - yawPower;
    powerL   = power + yawPower;

    ipowerL      = (int16_t)constrain(powerL * mechFactL, -maxPwr, maxPwr);
    int16_t mdbn = -motorDeadband;
    int16_t pp2n = -punchPwr2;
    if (ipowerL > 0) {
        if (motorLdir == 1)
            punchCountL = constrain(++punchCountL, 0, 100);
        else
            punchCountL = 0;
        motorLdir = 1;
        if (punchCountL < punchDur)
            drvMotorL(max(ipowerL, punchPwr2));
        else
            drvMotorL(max(ipowerL, motorDeadband));
    } else if (ipowerL < 0) {
        if (motorLdir == -1)
            punchCountL = constrain(++punchCountL, 0, 100);
        else
            punchCountL = 0;
        motorLdir = -1;
        if (punchCountL < punchDur)
            drvMotorL(min(ipowerL, pp2n));
        else
            drvMotorL(min(ipowerL, mdbn));
    } else {
        drvMotorL(0);
        motorLdir = 0;
    }

    ipowerR = (int16_t)constrain(powerR * mechFactR, -maxPwr, maxPwr);
    if (ipowerR > 0) {
        if (motorRdir == 1)
            punchCountR = constrain(++punchCountR, 0, 100);
        else
            punchCountR = 0;
        motorRdir = 1;
        if (punchCountR < punchDur)
            drvMotorR(max(ipowerR, punchPwr2));
        else
            drvMotorR(max(ipowerR, motorDeadband));
    } else if (ipowerR < 0) {
        if (motorRdir == -1)
            punchCountR = constrain(++punchCountR, 0, 100);
        else
            punchCountR = 0;
        motorRdir = -1;
        if (punchCountR < punchDur)
            drvMotorR(min(ipowerR, pp2n));
        else
            drvMotorR(min(ipowerR, mdbn));
    } else {
        drvMotorR(0);
        motorRdir = 0;
    }
}

void resetMotor() {
    drvMotorR(0);
    drvMotorL(0);
    counterOverPwr = 0;
}







const int BUFFER_SIZE = 300; // Buffer size for readings
const int NUM_SIDES = 6; // Number of sides on the robot
const int NUM_IMU_VALUES = 6; // Number of IMU values (3 gyro, 3 accel

const int GYRO_IMU_VALUES = 3;
const int ACC_IMU_VALUES = 3;


// Structure to hold calibration data
struct CalibrationData {

    float gyroOffsets[GYRO_IMU_VALUES] = {0, 0, 0}; // Gyro offsets for X, Y, Z
    float accelOffsets[ACC_IMU_VALUES] = {0, 0, 0}; // Accelerometer offsets for X, Y, Z
    float accelScale[ACC_IMU_VALUES] = {1, 1, 1}; // Accelerometer scaling factors for X, Y, Z
    bool sideCalibrated[NUM_SIDES] = {false, false, false, false, false, false}; // Calibration status for each side

    float readings[BUFFER_SIZE][NUM_IMU_VALUES]; // Buffer for the last 100 readings (6 IMU values)
    Robot_Side orientation_history[BUFFER_SIZE]; // Buffer for the last 100 robot orientations

    float StandardDeviations[NUM_SIDES][NUM_IMU_VALUES]; // Standard deviations for each side and each IMU value ie like the noise 
    float Means[NUM_SIDES][NUM_IMU_VALUES]; // Means for each side and each IMU value ie the g value for that axis

    int currentReadingIndex = 0; // Index for the current reading in the buffer
    //Side currentSide = NONE; // Current side being calibrated
    bool calibrationComplete = false; // Flag to indicate if calibration is complete
};

CalibrationData calibData;


CalibrationState calibrationState = INIT; // Initial state

Robot_Side currentSide = ROBOT_UNKNOWN_SIDE; // Current side being calibrated

int reading_buff_index = calibData.currentReadingIndex % BUFFER_SIZE;

//CalibrationData calibData;

# define DEBUG_calcRobotAlignment 0
# define RobotAlignmentPreActicationThreshold 0.85
# define RobotAlignmentActicationThreshold 0.9
# define RobotAlignmentDeactivationThreshold 0.1
# define RobotAlignmentPreDeactivationThreshold 0.2

float speaker_offset = 0;

void updateRobotOrientation(Robot_Side side, String sideString, bool isAtCenter, float Gs){

        if (side == ROBOT_UNKNOWN_SIDE) {
            RED_LED(0);
            ledcWriteTone(SPEAKER_CH,0);
        }else if(calibData.sideCalibrated[side] == false){

            float speaker_offset = ((Gs - RobotAlignmentActicationThreshold) * 50000)*0.1 + 0.9*speaker_offset;
            Serial.print("Speaker offset: ");
            Serial.println(speaker_offset);

            ledcWriteTone(SPEAKER_CH,2000 + (int)(speaker_offset));
            
            M5.Beep.setVolume(10);
            if (isAtCenter){
                //ledcWriteTone(SPEAKER_CH,1000);
                RED_LED(1);

            }
            else {
                //ledcWriteTone(SPEAKER_CH,500);
                RED_LED(0);
            }
            
        }

        if (isAtCenter){
            currentSide = side;
            calibData.orientation_history[reading_buff_index] = side;
        }
        else{
            currentSide = ROBOT_UNKNOWN_SIDE;
            calibData.orientation_history[reading_buff_index] = ROBOT_UNKNOWN_SIDE;
        }

        #if DEBUG_calcRobotAlignment

        //vTaskDelay(100);

        Serial.println("IMU: X:" + String(IMU_RAW_X_Gs) + "  Y:" + String(IMU_RAW_Y_Gs) + " Z:" + String(IMU_RAW_Z_Gs) + "Robot is pointed " + sideString + " side" + " isAtCenter: " + String(isAtCenter));
        #endif
}


// Checks IMU_RAW_X_Gs, IMU_RAW_Y_Gs, IMU_RAW_Z_Gs to determine the robot's alignment
void calculateRobotAlignment(){

    //Pointed front / back
    if (abs(IMU_RAW_X_Gs) > RobotAlignmentActicationThreshold && 
        abs(IMU_RAW_Y_Gs) < RobotAlignmentDeactivationThreshold && 
        abs(IMU_RAW_Z_Gs) < RobotAlignmentDeactivationThreshold) {

        // determine front or back
        if (IMU_RAW_X_Gs > 0) {
            // updateRobotOrientation(ROBOT_FRONT_SIDE, "front", true);
            updateRobotOrientation(ROBOT_BACK_SIDE, "back", true, abs(Avg_IMU_X_Gs));
        } else {
            // updateRobotOrientation(ROBOT_BACK_SIDE, "back", true);
            updateRobotOrientation(ROBOT_FRONT_SIDE, "front", true, abs(Avg_IMU_X_Gs));
        }

    }

    

    //Pointed left / right
    else if (abs(IMU_RAW_Y_Gs) > RobotAlignmentActicationThreshold &&
             abs(IMU_RAW_X_Gs) < RobotAlignmentDeactivationThreshold &&
             abs(IMU_RAW_Z_Gs) < RobotAlignmentDeactivationThreshold) {
        
        // determine left or right
        if (IMU_RAW_Y_Gs > 0) {
            updateRobotOrientation(ROBOT_LEFT_SIDE, "left", true, abs(IMU_RAW_Y_Gs));
        } else {
            updateRobotOrientation(ROBOT_RIGHT_SIDE, "right", true, abs(IMU_RAW_Y_Gs));
        }

    }


    //Pointed up / down
    else if (abs(IMU_RAW_Z_Gs) > RobotAlignmentActicationThreshold   &&
             abs(IMU_RAW_X_Gs) < RobotAlignmentDeactivationThreshold &&
             abs(IMU_RAW_Y_Gs) < RobotAlignmentDeactivationThreshold) {
        
        if (IMU_RAW_Z_Gs < 0) {
            updateRobotOrientation(ROBOT_TOP_SIDE, "up", true, abs(IMU_RAW_Z_Gs));
        } else {
            updateRobotOrientation(ROBOT_BOTTOM_SIDE, "down", true, abs(IMU_RAW_Z_Gs));
        }

    }




    
    //Pointed front / back
    else if (abs(IMU_RAW_X_Gs) > RobotAlignmentPreActicationThreshold &&
             abs(IMU_RAW_Y_Gs) < RobotAlignmentPreDeactivationThreshold &&
             abs(IMU_RAW_Z_Gs) < RobotAlignmentPreDeactivationThreshold) {

// determine front or back
        if (IMU_RAW_X_Gs > 0) {
            updateRobotOrientation(ROBOT_BACK_SIDE, "back", false, abs(Avg_IMU_X_Gs));
        } else {
            updateRobotOrientation(ROBOT_FRONT_SIDE, "front", false, abs(Avg_IMU_X_Gs));
        }

    }

    // }

    //Pointed left / right
    else if (abs(IMU_RAW_Y_Gs) > RobotAlignmentPreActicationThreshold &&
             abs(IMU_RAW_X_Gs) < RobotAlignmentPreDeactivationThreshold &&
             abs(IMU_RAW_Z_Gs) < RobotAlignmentPreDeactivationThreshold) {
        
        // determine left or right
        if (IMU_RAW_Y_Gs > 0) {
            updateRobotOrientation(ROBOT_LEFT_SIDE, "left", false, abs(Avg_IMU_Y_Gs));
        } else {
            updateRobotOrientation(ROBOT_RIGHT_SIDE, "right", false, abs(Avg_IMU_Y_Gs));
        }

    }


    //Pointed up / down
    else if (abs(IMU_RAW_Z_Gs) > RobotAlignmentPreActicationThreshold &&
             abs(IMU_RAW_X_Gs) < RobotAlignmentPreDeactivationThreshold &&
             abs(IMU_RAW_Y_Gs) < RobotAlignmentPreDeactivationThreshold) {

        if (IMU_RAW_Z_Gs < 0) {
            updateRobotOrientation(ROBOT_TOP_SIDE, "up", false, abs(Avg_IMU_Z_Gs));
        } else {
            updateRobotOrientation(ROBOT_BOTTOM_SIDE, "down", false, abs(Avg_IMU_Z_Gs));
        }
    }


    else {
        updateRobotOrientation(ROBOT_UNKNOWN_SIDE, "unknown", false, 0);
    }

}




void addIMUReadingsToBuffer(){



    calibData.readings[reading_buff_index][0] = IMU_RAW_X_dps;
    calibData.readings[reading_buff_index][1] = IMU_RAW_Y_dps;
    calibData.readings[reading_buff_index][2] = IMU_RAW_Z_dps;

    calibData.readings[reading_buff_index][3] = IMU_RAW_X_Gs;
    calibData.readings[reading_buff_index][4] = IMU_RAW_Y_Gs;
    calibData.readings[reading_buff_index][5] = IMU_RAW_Z_Gs;


    // Increment the current reading index
    calibData.currentReadingIndex++; 

};

# define DEBUG_calculate_buff_stats 1

void calculate_buff_stats(){


    #if DEBUG_calculate_buff_stats
    Serial.println("@calculate_buff_stats");

    Serial.println("Buffer Readings:");
    for(int i = 0; i < BUFFER_SIZE; i++) {
        for (int j = 0; j < 6; j++) {
            Serial.print(calibData.readings[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }
    #endif

    // Calculate the standard deviation and mean for each side

    // for(int i = 0; i < 6; i++) {
        
    // }
    for(int j = 0; j < 6; j++) {
        calibData.StandardDeviations[currentSide][j] = 0;
        calibData.Means[currentSide][j] = 0;
    }

    // for(int i = 0; i < 6; i++) {
    //     for(int j = 0; j < 6; j++) {
    //         for(int k = 0; k < BUFFER_SIZE; k++) {
    //             calibData.Means[i][j] += calibData.readings[k][j];
    //         }
    //         calibData.Means[i][j] /= BUFFER_SIZE;
    //     }
    // }
    for(int j = 0; j < 6; j++) {
        for(int k = 0; k < BUFFER_SIZE; k++) {
            calibData.Means[currentSide][j] += calibData.readings[k][j];
        }
        calibData.Means[currentSide][j] /= BUFFER_SIZE;
    }


    // for(int i = 0; i < 6; i++) {
    //     for(int j = 0; j < 6; j++) {
    //         for(int k = 0; k < BUFFER_SIZE; k++) {
    //             calibData.StandardDeviations[i][j] += pow(calibData.readings[k][j] - calibData.Means[i][j], 2);
    //         }
    //         calibData.StandardDeviations[i][j] = sqrt(calibData.StandardDeviations[i][j] / BUFFER_SIZE);
    //     }
    // }
    for(int j = 0; j < 6; j++) {
        for(int k = 0; k < BUFFER_SIZE; k++) {
            calibData.StandardDeviations[currentSide][j] += pow(calibData.readings[k][j] - calibData.Means[currentSide][j], 2);
        }
        calibData.StandardDeviations[currentSide][j] = sqrt(calibData.StandardDeviations[currentSide][j] / BUFFER_SIZE);
    }

    #if DEBUG_calculate_buff_stats
    Serial.println("Standard Deviations:");
    for(int i = 0; i < 6; i++) {
        if (i == 0){Serial.print("FRONT  ");};
        if (i == 1){Serial.print("BACK   ");};
        if (i == 2){Serial.print("LEFT   ");};
        if (i == 3){Serial.print("RIGHT  ");};
        if (i == 4){Serial.print("TOP    ");};
        if (i == 5){Serial.print("BOTTOM ");};
        for(int j = 0; j < 6; j++) {

            if(j == 0){Serial.print("gX");}
            if(j == 1){Serial.print("gY");}
            if(j == 2){Serial.print("gZ");}

            if(j == 3){Serial.print("aX");}
            if(j == 4){Serial.print("aY");}
            if(j == 5){Serial.print("aZ");}

            Serial.print(calibData.StandardDeviations[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }

    Serial.println("Means:");
    for(int i = 0; i < 6; i++) {
        // if (i == 0){Serial.print("TOP    ");};
        // if (i == 1){Serial.print("BOTTOM ");};
        // if (i == 2){Serial.print("FRONT  ");};
        // if (i == 3){Serial.print("BACK   ");};
        // if (i == 4){Serial.print("LEFT   ");};
        // if (i == 5){Serial.print("RIGHT  ");};
        if (i == 0){Serial.print("FRONT  ");};
        if (i == 1){Serial.print("BACK   ");};
        if (i == 2){Serial.print("LEFT   ");};
        if (i == 3){Serial.print("RIGHT  ");};
        if (i == 4){Serial.print("TOP    ");};
        if (i == 5){Serial.print("BOTTOM ");};

        for(int j = 0; j < 6; j++) {

            if(j == 0){Serial.print("gX");}
            if(j == 1){Serial.print("gY");}
            if(j == 2){Serial.print("gZ");}

            if(j == 3){Serial.print("aX");}
            if(j == 4){Serial.print("aY");}
            if(j == 5){Serial.print("aZ");}

            Serial.print(calibData.Means[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }

    #endif





};

# define DEBUG_robotHasBeenStableFor100Cycles 0

boolean robotHasBeenAlignedWithAxisForLongerThanThreshold(){


    if(currentSide == ROBOT_UNKNOWN_SIDE)
    {
        #if DEBUG_robotHasBeenStableFor100Cycles
        Serial.println("currentSide is ROBOT_UNKNOWN_SIDE, returning false");
        #endif
        return false;
    }

    //ledcWriteTone(SPEAKER_CH,0);

    #if DEBUG_robotHasBeenStableFor100Cycles
    Serial.println("@robotHasBeenAlignedWithAxisForLongerThanThreshold");
    #endif

    // first check if currentReadingIndex is greater than BUFFER_SIZE, if not return false
    if(calibData.currentReadingIndex < BUFFER_SIZE) {
        #if DEBUG_robotHasBeenStableFor100Cycles
        Serial.println("currentReadingIndex is less than BUFFER_SIZE, returning false");
        #endif
        return false;
    }

    // check if orientation_history is the same for the last 100 cycles, aka the same as currentSide
    for(int i = 0; i < BUFFER_SIZE; i++) {
        if(calibData.orientation_history[i] != currentSide) {
            #if DEBUG_robotHasBeenStableFor100Cycles
            Serial.println("orientation_history is not the same as currentSide, returning false");
            #endif
            return false;
        
        }

    }



    return true;

    


};


// void calculateOffsetsAndScalingFactors() {

//     /*  

//     StandardDeviations and Means will look like this:
//     Standard Deviations:
//     FRONT  gX0.54 gY1.26 gZ1.29 aX0.01 aY0.01 aZ0.01
//     BACK   gX3.75 gY3.32 gZ2.67 aX0.01 aY0.02 aZ0.01 
//     LEFT   gX0.64 gY4.20 gZ0.58 aX0.01 aY0.01 aZ0.01
//     RIGHT  gX0.67 gY1.24 gZ0.49 aX0.01 aY0.01 aZ0.01
//     TOP    gX1.05 gY6.61 gZ0.50 aX0.01 aY0.01 aZ0.01 
//     BOTTOM gX1.26 gY1.27 gZ0.47 aX0.01 aY0.01 aZ0.01
//     Means:
//     FRONT  gX3.91 gY-5.45 gZ-7.51 aX-0.00 aY-0.02 aZ-0.94
//     BACK   gX2.86 gY-5.18 gZ-7.60 aX-0.03 aY-0.01 aZ1.06
//     LEFT   gX3.31 gY-5.70 gZ-7.06 aX0.99 aY-0.02 aZ0.04 
//     RIGHT  gX3.72 gY-5.98 gZ-7.46 aX-1.00 aY-0.02 aZ0.07
//     TOP    gX3.72 gY-3.48 gZ-7.36 aX-0.03 aY-1.00 aZ0.08
//     BOTTOM gX3.96 gY-4.83 gZ-7.44 aX-0.05 aY0.99 aZ0.05

//     */

//     // calculate 

//      // Placeholder arrays for gyro and accelerometer offsets
//     float gyroOffsets[3] = {0.0, 0.0, 0.0};
//     float accelOffsets[3] = {0.0, 0.0, 0.0};

//     // Placeholder arrays for accelerometer scaling factors
//     float accelScaleFactors[3] = {1.0, 1.0, 1.0};

//     // Calculating gyro offsets as the mean gyro readings for each axis
//     gyroOffsets[0] = (calibData.Means[ROBOT_FRONT_SIDE][0] + calibData.Means[ROBOT_BACK_SIDE][0]) / 2;
//     gyroOffsets[1] = (calibData.Means[ROBOT_LEFT_SIDE][1] + calibData.Means[ROBOT_RIGHT_SIDE][1]) / 2;
//     gyroOffsets[2] = (calibData.Means[ROBOT_TOP_SIDE][2] + calibData.Means[ROBOT_BOTTOM_SIDE][2]) / 2;

//     // Assuming the Z axis is aligned with gravity, and using FRONT and BACK for X, LEFT and RIGHT for Y
//     accelOffsets[0] = (calibData.Means[ROBOT_FRONT_SIDE][3] + calibData.Means[ROBOT_BACK_SIDE][3]) / 2;
//     accelOffsets[1] = (calibData.Means[ROBOT_LEFT_SIDE][4] + calibData.Means[ROBOT_RIGHT_SIDE][4]) / 2;
//     // For Z, averaging TOP and BOTTOM should give a value close to 1g or -1g depending on orientation
//     accelOffsets[2] = (calibData.Means[ROBOT_TOP_SIDE][5] + calibData.Means[ROBOT_BOTTOM_SIDE][5]) / 2 - 1.0;

//     // Calculating scaling factors for the accelerometer
//     // Ideally, we would adjust these based on the expected 1g reading for each axis when aligned with gravity
//     accelScaleFactors[0] = 1.0 / fabs(calibData.Means[ROBOT_FRONT_SIDE][3] - accelOffsets[0]);
//     accelScaleFactors[1] = 1.0 / fabs(calibData.Means[ROBOT_LEFT_SIDE][4] - accelOffsets[1]);
//     accelScaleFactors[2] = 1.0 / fabs((calibData.Means[ROBOT_TOP_SIDE][5] - accelOffsets[2]) / 1.0);  // Divided by 1g

//     // Printing the calculated offsets and scaling factors
//     Serial.println("Gyro Offsets: ");
//     Serial.print("X: "); Serial.println(gyroOffsets[0]);
//     Serial.print("Y: "); Serial.println(gyroOffsets[1]);
//     Serial.print("Z: "); Serial.println(gyroOffsets[2]);

//     Serial.println("Accelerometer Offsets: ");
//     Serial.print("X: "); Serial.println(accelOffsets[0]);
//     Serial.print("Y: "); Serial.println(accelOffsets[1]);
//     Serial.print("Z: "); Serial.println(accelOffsets[2]);

//     Serial.println("Accelerometer Scaling Factors: ");
//     Serial.print("X: "); Serial.println(accelScaleFactors[0]);
//     Serial.print("Y: "); Serial.println(accelScaleFactors[1]);
//     Serial.print("Z: "); Serial.println(accelScaleFactors[2]);


// };





# define DEBUG_CalculateOffsetsAndScalingFactors 1

void calculateOffsetsAndScalingFactors() {


    /*  EXAMPLE DATA

    
    Standard Deviations:
    FRONT  gX1.61 gY30.86 gZ1.09 aX0.01 aY0.01 aZ0.01
    BACK   gX1.56 gY42.93 gZ9.21 aX0.01 aY0.01 aZ0.01
    LEFT   gX0.48 gY7.93 gZ0.63 aX0.01 aY0.01 aZ0.01
    RIGHT  gX1.84 gY2.79 gZ0.66 aX0.01 aY0.01 aZ0.01
    TOP    gX1.39 gY15.68 gZ0.72 aX0.01 aY0.01 aZ0.01
    BOTTOM gX0.95 gY1.74 gZ1.69 aX0.01 aY0.01 aZ0.01
    Means:
    FRONT  gX3.73 gY25.66 gZ-7.79 aX0.03 aY-0.04 aZ-0.93
    BACK   gX3.63 gY38.13 gZ-9.38 aX-0.04 aY0.01 aZ1.06
    LEFT   gX3.53 gY-2.15 gZ-7.14 aX0.99 aY0.00 aZ0.05
    RIGHT  gX3.87 gY-6.18 gZ-7.42 aX-1.00 aY-0.02 aZ0.07
    TOP    gX3.88 gY1.64 gZ-7.68 aX-0.01 aY-1.00 aZ0.05
    BOTTOM gX3.82 gY-4.86 gZ-7.74 aX0.04 aY0.99 aZ0.04
    
    */
   


    // Gyro offsets are the average of the means for opposing sides
    // float gyroOffsets[3] = {
    //     (calibData.Means[ROBOT_FRONT_SIDE][0] + calibData.Means[ROBOT_BACK_SIDE][0]) / 2,
    //     (calibData.Means[ROBOT_LEFT_SIDE][1] + calibData.Means[ROBOT_RIGHT_SIDE][1]) / 2,
    //     (calibData.Means[ROBOT_TOP_SIDE][2] + calibData.Means[ROBOT_BOTTOM_SIDE][2]) / 2
    // };


    // float gyroOffsets[3] = {
    //     (   0.16 * (calibData.Means[ROBOT_FRONT_SIDE][0] + calibData.Means[ROBOT_BACK_SIDE][0]) +
    //         0.16 * (calibData.Means[ROBOT_LEFT_SIDE][0] + calibData.Means[ROBOT_RIGHT_SIDE][0]) +
    //         0.16 * (calibData.Means[ROBOT_TOP_SIDE][0] + calibData.Means[ROBOT_BOTTOM_SIDE][0])),

    //     (   0.16 * (calibData.Means[ROBOT_FRONT_SIDE][1] + calibData.Means[ROBOT_BACK_SIDE][1]) +
    //         0.16 * (calibData.Means[ROBOT_LEFT_SIDE][1] + calibData.Means[ROBOT_RIGHT_SIDE][1]) +
    //         0.16 * (calibData.Means[ROBOT_TOP_SIDE][1] + calibData.Means[ROBOT_BOTTOM_SIDE][1])),

    //     (   0.16 * (calibData.Means[ROBOT_FRONT_SIDE][2] + calibData.Means[ROBOT_BACK_SIDE][2]) +
    //         0.16 * (calibData.Means[ROBOT_LEFT_SIDE][2] + calibData.Means[ROBOT_RIGHT_SIDE][2]) +
    //         0.16 * (calibData.Means[ROBOT_TOP_SIDE][2] + calibData.Means[ROBOT_BOTTOM_SIDE][2]) )
    // };

        gyro_deg_per_sec_X_offset = (calibData.Means[ROBOT_FRONT_SIDE][0] + calibData.Means[ROBOT_BACK_SIDE][0] +
                                     calibData.Means[ROBOT_LEFT_SIDE][0] + calibData.Means[ROBOT_RIGHT_SIDE][0] +
                                     calibData.Means[ROBOT_TOP_SIDE][0] + calibData.Means[ROBOT_BOTTOM_SIDE][0])/ 6;

        gyro_deg_per_sec_Y_offset = (calibData.Means[ROBOT_FRONT_SIDE][1] + calibData.Means[ROBOT_BACK_SIDE][1] +
                                     calibData.Means[ROBOT_LEFT_SIDE][1] + calibData.Means[ROBOT_RIGHT_SIDE][1] +
                                     calibData.Means[ROBOT_TOP_SIDE][1] + calibData.Means[ROBOT_BOTTOM_SIDE][1])/ 6;

        gyro_deg_per_sec_Z_offset = (calibData.Means[ROBOT_FRONT_SIDE][2] + calibData.Means[ROBOT_BACK_SIDE][2] +
                                     calibData.Means[ROBOT_LEFT_SIDE][2] + calibData.Means[ROBOT_RIGHT_SIDE][2] +
                                     calibData.Means[ROBOT_TOP_SIDE][2] + calibData.Means[ROBOT_BOTTOM_SIDE][2])/ 6;

    //     float gyroOffsets[3] = {
    //     (  ,

    //     (  ,

    //     (  
    // };

    // Accelerometer offsets for X and Y are the averages of the means for opposing sides
    // For Z, it's the average adjusted for gravity
    // float accelOffsets[3] = {
    //     (calibData.Means[ROBOT_FRONT_SIDE][3] + calibData.Means[ROBOT_BACK_SIDE][3]) / 2,
    //     (calibData.Means[ROBOT_LEFT_SIDE][4] + calibData.Means[ROBOT_RIGHT_SIDE][4]) / 2,
    //     ((calibData.Means[ROBOT_TOP_SIDE][5] + calibData.Means[ROBOT_BOTTOM_SIDE][5]) / 2)
    // };
    
    float accelOffsets[3] = {
        (calibData.Means[ROBOT_FRONT_SIDE][3] + calibData.Means[ROBOT_BACK_SIDE][3]) * 0.1 +
        (calibData.Means[ROBOT_LEFT_SIDE][3] + calibData.Means[ROBOT_RIGHT_SIDE][3]) * 0.3 +
        (calibData.Means[ROBOT_TOP_SIDE][3] + calibData.Means[ROBOT_BOTTOM_SIDE][3]) * 0.1,

        (calibData.Means[ROBOT_FRONT_SIDE][4] + calibData.Means[ROBOT_BACK_SIDE][4]) * 0.1 +
        (calibData.Means[ROBOT_LEFT_SIDE][4] + calibData.Means[ROBOT_RIGHT_SIDE][4]) * 0.1 +
        (calibData.Means[ROBOT_TOP_SIDE][4] + calibData.Means[ROBOT_BOTTOM_SIDE][4]) * 0.3,

        (calibData.Means[ROBOT_FRONT_SIDE][5] + calibData.Means[ROBOT_BACK_SIDE][5]) * 0.3 +
        (calibData.Means[ROBOT_LEFT_SIDE][5] + calibData.Means[ROBOT_RIGHT_SIDE][5]) * 0.1 +
        (calibData.Means[ROBOT_TOP_SIDE][5] + calibData.Means[ROBOT_BOTTOM_SIDE][5]) * 0.1

    };


    // BAD BAD CODE, u suk felpie

    // // Scaling factors for accelerometers are based on the deviation from 1g in the Z-axis 
    // float accelScaleFactors[3] = {
    //     1.0 / fabs(calibData.Means[ROBOT_TOP_SIDE][3] - accelOffsets[0]),
    //     1.0 / fabs(calibData.Means[ROBOT_TOP_SIDE][4] - accelOffsets[1]),
    //     1.0 / fabs((calibData.Means[ROBOT_TOP_SIDE][5] - accelOffsets[2]) / 1.0)
    // };

    // calculate scaling factors for the accelerometer by averaging 1/(measured g for + and - g) for each axis
    // float accelScaleFactors[3] = {
    //     (1.0 / fabs(calibData.Means[ROBOT_TOP_SIDE][3] - accelOffsets[0]) + 1.0 / fabs(calibData.Means[ROBOT_BOTTOM_SIDE][3] - accelOffsets[0])) / 2,
    //     (1.0 / fabs(calibData.Means[ROBOT_TOP_SIDE][4] - accelOffsets[1]) + 1.0 / fabs(calibData.Means[ROBOT_BOTTOM_SIDE][4] - accelOffsets[1])) / 2,
    //     (1.0 / fabs((calibData.Means[ROBOT_TOP_SIDE][5] - accelOffsets[2]) / 1.0) + 1.0 / fabs((calibData.Means[ROBOT_BOTTOM_SIDE][5] - accelOffsets[2]) / 1.0)) / 2
    // };

    // BAD PRODUCES:

    //     Standard Deviations:
    // FRONT  gX6.08 gY1.31 gZ0.89 aX0.01 aY0.02 aZ0.01
    // BACK   gX6.85 gY1.07 gZ0.91 aX0.01 aY0.02 aZ0.01 
    // LEFT   gX4.56 gY1.38 gZ0.66 aX0.01 aY0.01 aZ0.01
    // RIGHT  gX4.77 gY1.07 gZ0.62 aX0.01 aY0.01 aZ0.01
    // TOP    gX5.81 gY1.39 gZ0.70 aX0.01 aY0.01 aZ0.01
    // BOTTOM gX2.71 gY0.80 gZ2.49 aX0.01 aY0.01 aZ0.01 
    // Means:
    // FRONT  gX-2.19 gY4.88 gZ-13.65 aX-0.04 aY-0.00 aZ-0.93
    // BACK   gX-1.21 gY4.78 gZ-12.78 aX-0.04 aY-0.07 aZ1.08
    // LEFT   gX-0.76 gY5.52 gZ-14.81 aX0.98 aY-0.03 aZ0.07 
    // RIGHT  gX-0.82 gY4.97 gZ-13.29 aX-1.01 aY-0.00 aZ0.07
    // TOP    gX-0.33 gY4.84 gZ-13.17 aX-0.02 aY-1.03 aZ0.07
    // BOTTOM gX-0.47 gY5.07 gZ-13.34 aX-0.05 aY0.97 aZ0.06 
    // Calibration of 0 side complete
    // Calibration complete. Calculated Offsets and Scaling Factors:
    // Gyro Offsets: X: -1.70, Y: 5.24, Z: -13.26
    // Accelerometer Offsets: X: -0.04, Y: -0.02, Z: 0.06
    // Accelerometer Scaling Factors: X: 76.54, Y: 1.00, Z: 197.54



        // calculate scaling factors for the accelerometer by averaging 1/(measured g for + and - g) for each axis
    // float accelScaleFactors[3] = {
    //     // (1.0 / fabs(calibData.Means[ROBOT_FRONT_SIDE][3] - accelOffsets[0]) + 1.0 / fabs(calibData.Means[ROBOT_BACK_SIDE][3] - accelOffsets[0])) / 2,
    //     // (1.0 / fabs(calibData.Means[ROBOT_LEFT_SIDE][4] - accelOffsets[1]) + 1.0 / fabs(calibData.Means[ROBOT_RIGHT_SIDE][4] - accelOffsets[1])) / 2,

    //     ((1.0 / abs(calibData.Means[ROBOT_FRONT_SIDE][3] - accelOffsets[0])) + (1.0 / abs(calibData.Means[ROBOT_BACK_SIDE][3] - accelOffsets[0])) / 2),
    //     ((1.0 / abs(calibData.Means[ROBOT_LEFT_SIDE][4]  - accelOffsets[1])) + (1.0 / abs(calibData.Means[ROBOT_RIGHT_SIDE][4] - accelOffsets[1])) / 2),
    //     ((1.0 / abs(calibData.Means[ROBOT_TOP_SIDE][5]   - accelOffsets[2])) + (1.0 / abs(calibData.Means[ROBOT_BOTTOM_SIDE][5] - accelOffsets[2])) / 2)
    // };



    Serial.println(String(calibData.Means[ROBOT_FRONT_SIDE][3]) +  " - " + String(accelOffsets[0]));

    Serial.println(String(0.5 / abs(calibData.Means[ROBOT_FRONT_SIDE][3]  - accelOffsets[0])) + " + " + String(0.5 / abs(calibData.Means[ROBOT_BACK_SIDE][3] - accelOffsets[0])) + " = " + String((0.5 / abs(calibData.Means[ROBOT_LEFT_SIDE][3]  - accelOffsets[0])) + (0.5 / abs(calibData.Means[ROBOT_RIGHT_SIDE][3] - accelOffsets[0]))));

    float accelScaleFactors[3] = {

        ((0.5 / abs(calibData.Means[ROBOT_FRONT_SIDE][3]  - accelOffsets[0])) + (0.5 / abs(calibData.Means[ROBOT_BACK_SIDE][3] -  accelOffsets[0]))),
        ((0.5 / abs(calibData.Means[ROBOT_LEFT_SIDE][4]   - accelOffsets[1])) + (0.5 / abs(calibData.Means[ROBOT_RIGHT_SIDE][4] - accelOffsets[1]))),
        ((0.5 / abs(calibData.Means[ROBOT_TOP_SIDE][5] - accelOffsets[2])) + (0.5 / abs(calibData.Means[ROBOT_BOTTOM_SIDE][5] -   accelOffsets[2])))
        
    };
    










    #if DEBUG_CalculateOffsetsAndScalingFactors
    // Printing the calculated offsets and scaling factors
    Serial.println("Calibration complete. Calculated Offsets and Scaling Factors:");
    Serial.print("Gyro Offsets: X: "); Serial.print(gyro_deg_per_sec_X_offset);
    Serial.print(", Y: "); Serial.print(gyro_deg_per_sec_Y_offset);
    Serial.print(", Z: "); Serial.println(gyro_deg_per_sec_Z_offset);

    Serial.print("Accelerometer Offsets: X: "); Serial.print(accelOffsets[0]);
    Serial.print(", Y: "); Serial.print(accelOffsets[1]);
    Serial.print(", Z: "); Serial.println(accelOffsets[2]);

    Serial.print("Accelerometer Scaling Factors: X: "); Serial.print(accelScaleFactors[0]);
    Serial.print(", Y: "); Serial.print(accelScaleFactors[1]);
    Serial.print(", Z: "); Serial.println(accelScaleFactors[2]);
    #endif
}






/*  Function to calibrate the IMU

It has the followins states:

INIT: The initial state
WAIT_FOR_STABILITY: Wait for the IMU to stabilize
TOP: Calibrate the top side
ROBOT_BOTTOM_SIDE: Calibrate the bottom side
ROBOT_FRONT_SIDE: Calibrate the front side
BACK: Calibrate the back side
LEFT: Calibrate the left side
RIGHT: Calibrate the right side
COMPLETED: The calibration is complete, calculate the offsets and scaling factors

*/
void calibrateIMU() {
    //Serial.println("@calibrateIMU");

    reading_buff_index = calibData.currentReadingIndex % BUFFER_SIZE;


    // Read the gyro and accelerometer data
    getGyro();

    // Add the readings to the buffer
    addIMUReadingsToBuffer();



    switch (calibrationState)
    {
        // Initial state
        case INIT:

            Serial.println("Starting IMU Calibration...");
            Serial.println("x,y,z offsets will be calculated");
            Serial.println("for gyro and accel as well as scaling factors for accel");
            Serial.println("You will be prompted to orientate the robot in 6 different positions");
            Serial.println("The robot must be held stable for 5s while orientated:");
            Serial.println("UP, DOWN, FRONT, BACK, LEFT, RIGHT");

            // Set the current side to the top
            //calibData.currentSide = ROBOT_TOP_SIDE;
            // Set the calibration state to wait for stability
            calibrationState = WAIT_FOR_STABILITY;
            break;


        // Will stay in this state until the robot has been stable for 100 cycles,
        // then switch to the appropriate side calibration state
        case WAIT_FOR_STABILITY:

            // Calculate robot alignment
            calculateRobotAlignment();

            // Check if the robot has been stable for 100 cycles

            // this sets currentSide to the side facing up
            if(robotHasBeenAlignedWithAxisForLongerThanThreshold()){

                switch (currentSide)
                {
                case ROBOT_TOP_SIDE: case ROBOT_BOTTOM_SIDE: case ROBOT_FRONT_SIDE: case ROBOT_BACK_SIDE: case ROBOT_LEFT_SIDE: case ROBOT_RIGHT_SIDE:
                    if (calibData.sideCalibrated[currentSide] == false) {

                        vTaskDelay(1000);
                        
                        calibrationState = CalibrationState(currentSide);
                        currentSide = ROBOT_UNKNOWN_SIDE; // reset currentSide
                        //Serial.println("Calibrating top side...");

                        

                    }


                    break;
                
                default:
                    break;
                }

            }


            break;

        //case TOP_SIDE_STATE || BOTTOM_SIDE_STATE || FRONT_SIDE_STATE || BACK_SIDE_STATE || LEFT_SIDE_STATE || RIGHT_SIDE_STATE: how to handle these states?



        case TOP_SIDE_STATE: case BOTTOM_SIDE_STATE: case FRONT_SIDE_STATE: case BACK_SIDE_STATE: case LEFT_SIDE_STATE: case RIGHT_SIDE_STATE:
            
            ledcWriteTone(SPEAKER_CH,1200);

            calculateRobotAlignment();

            if(robotHasBeenAlignedWithAxisForLongerThanThreshold()){

                ledcWriteTone(SPEAKER_CH,1400);

                vTaskDelay(1000);

                // Calculate the standard deviation and mean 
                calculate_buff_stats();

                // Set the calibration status for the top side to true
                calibData.sideCalibrated[currentSide] = true;

                // check if all sides have been calibrated
                if(calibData.sideCalibrated[ROBOT_FRONT_SIDE] == true &&
                calibData.sideCalibrated[ROBOT_BACK_SIDE] == true &&
                calibData.sideCalibrated[ROBOT_LEFT_SIDE] == true &&
                calibData.sideCalibrated[ROBOT_RIGHT_SIDE] == true &&
                calibData.sideCalibrated[ROBOT_TOP_SIDE] == true &&
                calibData.sideCalibrated[ROBOT_BOTTOM_SIDE] == true) {
                    // Set the calibration state to completed
                    calibrationState = COMPLETED;
                } else {
                    // Set the calibration state to wait for stability
                    calibrationState = WAIT_FOR_STABILITY;
                }

                Serial.println("Calibration of " + String(currentSide) + " side complete");
                RED_LED(0);

                ledcWriteTone(SPEAKER_CH,0);

            }
        



            break;
    

        case COMPLETED:

            // play completion sound
            //ledcWriteTone(SPEAKER_CH,1600);


            // Calculate the offsets and scaling factors
            calculateOffsetsAndScalingFactors();

            // print the IMU values

            Serial.println("Calibration complete");














            //vTaskDelay(1000);




            IMU_has_been_calibrated = true;
            break;


  
    }
}


void imuInit() {

    M5.Imu.Init();
    // if (M5.Imu.imuType = M5.Imu.IMU_MPU6886) {
    //     M5.Mpu6886.SetGyroFsr(
    //         M5.Mpu6886.GFS_250DPS);  // 250DPS 500DPS 1000DPS 2000DPS
    //     M5.Mpu6886.SetAccelFsr(M5.Mpu6886.AFS_4G);  // 2G 4G 8G 16G
    //     if (serialMonitor) Serial.println("MPU6886 found");
    // } else if (serialMonitor)
    //     Serial.println("MPU6886 not found");

    //filter.begin(25);



    M5.Imu.SetGyroFsr(
    M5.Imu.GFS_250DPS);  // 250DPS 500DPS 1000DPS 2000DPS
    //M5.Imu.SetAccelFsr(M5.Imu.AFS_4G);  // 2G 4G 8G 16G
    M5.Imu.SetAccelFsr(M5.Imu.AFS_2G);  // 2G 4G 8G 16G
    if (serialMonitor) Serial.println("MPU6886 configured");

    IMU_has_been_init = true;

    //calibrateIMU(gyroXoffset, gyro_deg_per_sec_Y_offset, gyro_deg_per_sec_Z_offset, accXoffset, accYoffset, accZoffset);
}



