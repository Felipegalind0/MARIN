#include <M5StickCPlus.h>
#include "variables.h"
#include "movement.h"
#include "IO.h"
#include "LCD.h"
//#include <WebSerial.h>


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
  spinStep = -40.0 * normalizedRotation * clk;
}

// Update the movement rate and direction based on the input movement value
void Movement_UpdateMovement(int movement) {
  // Normalize the movement value to be within the range of -2.0 and 2.0
  float normalizedMovement = movement / 5.0;

  // Set the movement rate based on the normalized movement value
  moveRate = normalizedMovement;
}


// Main Loop 
void Movement_Loop() {
    
    #ifdef DEBUG
        if (debugLoop1()) return;
    #endif

    getGyro();

    if (!standing) { // If Robot is not Standing

        Avg_Robot_Z_deg_per_sec = Avg_Robot_Z_deg_per_sec * 0.9 + abs(IMU_Y_deg_per_sec) * 0.1; // update average angular velocity
        

        

        // 0.9*90= 72
        //if Pointed 72 deg up 
        if (abs(Avg_IMU_Z_acceleration) > 0.9 && Avg_Robot_Z_deg_per_sec < 1.5) {

            // Run Second Calibration
            LCD_calib2_Message();
            calib2();

            if (demoMode == 1) startDemo();

            standing = true;
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


void report_DEG_Abort(){
    LCD_Abort_DEG_Message();
}

void report_PWR_Abort(){
    LCD_Abort_PWR_Message();
}

void Abort(String MSG){

    LCD_Abort_Message();

    if(MSG == "Max DEG") report_DEG_Abort();

    if(MSG == "Max PWR") report_PWR_Abort();

    resetMotor();

    Serial.print(MSG);

    standing = false;
    hasFallen = true;

    while(!abortWasHandled){
        vTaskDelay(500);
        ledcWriteTone(SPEAKER_CH,911);
        digitalWrite(LED, HIGH);
        CheckButtons();
        vTaskDelay(50);
        digitalWrite(LED, LOW);
        ledcWriteTone(SPEAKER_CH,0);

        if (Abtn) {
            abortWasHandled = true;
            break;
        };
    }

    LCD_Resume_from_Abort_Message();
    vTaskDelay(1000);


    Movement_Setup();
    
}


    

// First Calibration
void calib1() {
    calDelay(30);
    digitalWrite(LED, HIGH);
    calDelay(80);
    LCD_calib1_Message();
    gyroYoffset = 0.0;
    for (int i = 0; i < N_CAL1; i++) {
        readGyro();
        gyroYoffset += gyroYdata;
        vTaskDelay(9);
    }
    gyroYoffset /= (float)N_CAL1;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, LOW);
}

// Second Calibration
void calib2() {
    resetVar();
    resetMotor();
    digitalWrite(LED, HIGH);
    calDelay(80);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.println(" Cal-2  ");
    accXoffset  = 0.0;
    gyroZoffset = 0.0;
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();
        accXoffset += IMU_X_acceleration;
        gyroZoffset += gyroZdata;
        vTaskDelay(9);
    }
    accXoffset /= (float)N_CAL2;
    gyroZoffset /= (float)N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, LOW);
}



void calDelay(int n) {
    for (int i = 0; i < n; i++) {
        getGyro();
        vTaskDelay(9);
    }
}

void startDemo() {
    moveRate       = 1.0;
    spinContinuous = true;
    spinStep       = -40.0 * clk;
}



void getGyro() {
    readGyro();
    IMU_Y_deg_per_sec = (gyroYdata - gyroYoffset);// unit:deg/sec
    IMU_Z_deg_per_sec = gyroZdata - gyroZoffset; // unit:deg/sec

    Avg_IMU_Y_deg_per_sec = Avg_IMU_Y_deg_per_sec * 0.9 + IMU_Y_deg_per_sec * 0.1;  // update average y angular velocity
    Avg_IMU_Z_deg_per_sec = Avg_IMU_Z_deg_per_sec * 0.9 + IMU_Z_deg_per_sec * 0.1;  // update average z angular velocity

    Avg_IMU_X_acceleration   = Avg_IMU_X_acceleration * 0.9 + IMU_X_acceleration * 0.1;  // update average x acceleration
    Avg_IMU_Y_acceleration   = Avg_IMU_Y_acceleration * 0.9 + IMU_Y_acceleration * 0.1;  // update average y acceleration 
    Avg_IMU_Z_acceleration   = Avg_IMU_Z_acceleration * 0.9 + IMU_Z_acceleration * 0.1;  // update average z acceleration


    //calculate the x, y, z angles
    robot_Y_deg = (((Avg_IMU_X_acceleration) ? -1 : 1) * 90 * Avg_IMU_X_acceleration);  // unit:deg

    yawAngle += IMU_Z_deg_per_sec * clk;  // unit:deg

    // calculate the heading, as yawAngle mod 360
    heading = (byte)(round(yawAngle));


    varAng += (IMU_Y_deg_per_sec + ((IMU_X_acceleration - accXoffset) * 57.3 - varAng) * cutoff) *
              clk;  // complementary filter
}

void readGyro() {
  // Declare variables for gyro and accelerometer data
  float gX, gY, gZ, aX, aY, aZ;

  // Get the gyro data for the X, Y, and Z axes
  M5.Imu.getGyroData(&gX, &gY, &gZ);

  // Get the accelerometer data for the X, Y, and Z axes
  M5.Imu.getAccelData(&aX, &aY, &aZ);

  // Assign the gyro data to the corresponding global variables
  // (Note the sign changes to match the robot's coordinate system)
  gyroYdata = gX;  // y = x
  gyroZdata = -gY; // z = -y
  gyroXdata = -gZ; // x = -z

  // Assign the accelerometer data to the corresponding global variables
  // (Note the change in variable usage for aX)
  IMU_X_acceleration = aZ; // x = z
  IMU_Y_acceleration = aX; // y = x
  IMU_Z_acceleration = aY; // z = y
}


void drive() {
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
    power =
        varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * IMU_Y_deg_per_sec);
    if (abs(power) > 1000.0)
        counterOverPwr += 1;
    else
        counterOverPwr = 0;
    if (counterOverPwr > maxOvp) return;
    power    = constrain(power, -maxPwr, maxPwr);
    yawPower = (yawAngle - spinTarget) * Kyaw;
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

void imuInit() {
    M5.Imu.Init();
    // if (M5.Imu.imuType = M5.Imu.IMU_MPU6886) {
    //     M5.Mpu6886.SetGyroFsr(
    //         M5.Mpu6886.GFS_250DPS);  // 250DPS 500DPS 1000DPS 2000DPS
    //     M5.Mpu6886.SetAccelFsr(M5.Mpu6886.AFS_4G);  // 2G 4G 8G 16G
    //     if (serialMonitor) Serial.println("MPU6886 found");
    // } else if (serialMonitor)
    //     Serial.println("MPU6886 not found");

    M5.Imu.SetGyroFsr(
    M5.Imu.GFS_250DPS);  // 250DPS 500DPS 1000DPS 2000DPS
    M5.Imu.SetAccelFsr(M5.Imu.AFS_4G);  // 2G 4G 8G 16G
    if (serialMonitor) Serial.println("MPU6886 found");
}


