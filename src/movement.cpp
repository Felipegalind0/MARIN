#include <M5StickCPlus.h>
#include "variables.h"
#include "movement.h"
#include "pinout.h"
#include "LCD.h"
#include <WebSerial.h>


void Movement_UpdateRotation(int rotation) {
  moveRate = 0.0;
  spinContinuous = false;
  spinStep = 0.0;
  int s = abs(rotation);
  if (s > 0) {
    moveRate = -(s);
    spinContinuous = true;
    spinStep = (rotation > 4 ? -40.0 : 40.0) * clk;
  }
}

void Movement_UpdateMovement(int movement) {
  moveRate = 0.0;

  if (abs(movement) > 0) {
    moveRate = (movement > 4 ? -2.0 : 2.0);
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


// Main Loop 
void Movement_Loop() {
    
    #ifdef DEBUG
        if (debugLoop1()) return;
    #endif

    getGyro();

    if (hasFallen){
        ;;
    }

    else if (!standing) { // If Robot is not Standing

        aveAbsOmg = aveAbsOmg * 0.9 + abs(varOmg) * 0.1;
        aveAccZ   = aveAccZ * 0.9 + accZdata * 0.1;

        updateBatVolt();
        LCD_DispAngle();

        // 0.9*90= 72
        //if Pointed 72 deg up 
        if (abs(aveAccZ) > 0.9 && aveAbsOmg < 1.5) {

            // Run Second Calibration
            LCD_calib2_Message();
            calib2();

            if (demoMode == 1) startDemo();

            standing = true;
        }
    } 

    else { // Check if Robot has fallen and disable

        if (abs(varAng) > 45.0) {
            Abort("Max DEG");
        } 

        if (counterOverPwr > maxOvp) { 
            Abort("Max PWR");
        } 

        else { //Robot is okay to drive

            drive(); // What makes it move
        }

    }

}

void Abort(String MSG){
    WebSerial.print(MSG);

    standing = false;
    hasFallen = true;

    resetMotor();

    resetVar();

    standing = false;

    setMode(false);

    LCD_Abort_Message(MSG);
}


// First Calibration
void calib1() {
    calDelay(30);
    digitalWrite(LED, LOW);
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
    digitalWrite(LED, HIGH);
}

// Second Calibration
void calib2() {
    resetVar();
    resetMotor();
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.setCursor(30, LCDV_MID);
    M5.Lcd.println(" Cal-2  ");
    accXoffset  = 0.0;
    gyroZoffset = 0.0;
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();
        accXoffset += accXdata;
        gyroZoffset += gyroZdata;
        vTaskDelay(9);
    }
    accXoffset /= (float)N_CAL2;
    gyroZoffset /= (float)N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
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

void resetPara() {
    Kang          = 37.0;
    Komg          = 0.84;
    KIang         = 800.0;
    Kyaw          = 4.0;
    Kdst          = 85.0;
    Kspd          = 2.7;
    mechFactL     = 0.45;
    mechFactR     = 0.45;
    punchPwr      = 20;
    punchDur      = 1;
    fbBalance     = -3;
    motorDeadband = 10;
    maxPwr        = 120;
    punchPwr2     = max(punchPwr, motorDeadband);
}

void getGyro() {
    readGyro();
    varOmg = (gyroYdata - gyroYoffset);           // unit:deg/sec
    yawAngle += (gyroZdata - gyroZoffset) * clk;  // unit:g
    varAng += (varOmg + ((accXdata - accXoffset) * 57.3 - varAng) * cutoff) *
              clk;  // complementary filter
}

void readGyro() {
    float gX, gY, gZ, aX, aY, aZ;
    M5.Imu.getGyroData(&gX, &gY, &gZ);
    M5.Imu.getAccelData(&aX, &aY, &aZ);
    gyroYdata = gX;
    gyroZdata = -gY;
    gyroXdata = -gZ;
    accXdata  = aZ;
    accZdata  = aY;
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
        varIang + varDst + (Kspd * varSpd) + (Kang * varAng) + (Komg * varOmg);
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

void drvMotorL(int16_t pwm) {
    drvMotor(0, (int8_t)constrain(pwm, -127, 127));
}

void drvMotorR(int16_t pwm) {
    drvMotor(1, (int8_t)constrain(-pwm, -127, 127));
}

void drvMotor(byte ch, int8_t sp) {
    Wire.beginTransmission(0x38);
    Wire.write(ch);
    Wire.write(sp);
    Wire.endTransmission();
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
    if (serialMonitor) WebSerial.println("MPU6886 found");
}


