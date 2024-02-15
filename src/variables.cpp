//----------------variables.cpp----------------
#include "variables.h"
#include "LCD.h"

// These are the 2 important global variables that are linked by variables.h

int x = 0;  // Defines robot rotation rate    + = R     &   - = L
int y = 0;  // Defines robot FWD/BACK         + = FWD   &   - = BACK

float deviceTemp = -1.0;


//-----------------System Variables-----------------
byte robot_config_counter = 0;
boolean is_booted = false;
String exec_status = "OFF";
boolean exec_status_has_changed = false;

TaskHandle_t Task0, Task1;

SemaphoreHandle_t syncSemaphore;

//-----------------Battery Variables-----------------


int perCentBatt = -1;

float vBatt, voltAve             = -1.0;

float vBatt_min = 3.3, vBatt_max = 4.1;

boolean isCharging = false;




//-----------------Time Variables-----------------
int64_t RealTcode_start_time = 0, RealTcode_end_time = 0, RealTcode_execution_time = 0, RealTcode_no_execution_time = 0, RealTcode_total_execution_time = 0;
int64_t BackgroundTask_execution_time_start = 0, BackgroundTask_execution_time_end = 0, BackgroundTask_execution_time = 0, BackgroundTask_total_execution_time = 0, BackgroundTask_no_execution_time = 0;

double RealTcode_CPU_load = 0.0, BackgroundTask_CPU_load = 0.0;



//-----------------Joystick Variables-----------------
// uint32_t Joyc_X_min = 500, Joyc_X_center = 2000, Joyc_X_max = 3500, Joyc_Y_min = 500, Joyc_Y_center = 2000, Joyc_Y_max = 3500, JoyC_X_raw = 2000, JoyC_Y_raw = 2000, JoyC_X_raw_prev = 2000, JoyC_Y_raw_prev = 2000;

// byte JoyC_X_deadzone = 200, JoyC_Y_deadzone = 200;

byte JoyC_X = 50, JoyC_Y = 50;

float JoyC_r = 0;

float JoyC_Phi = 0;

byte JoyC_X_Cycles_In_Deadzone = 0, JoyC_Y_Cycles_In_Deadzone = 0;


boolean JoyC_In_X_DeadZone = true;
boolean JoyC_In_y_DeadZone = true;

int JoyC_X_left_right = 0;
int JoyC_Y_up_down    = 0;

boolean JoyC_btn = false;

boolean JoyC_left = false;
boolean JoyC_right = false;
boolean JoyC_up = false;
boolean JoyC_down = false;

boolean JoyC_needs_to_return_to_center = false;

boolean JoyC_Xinput = false;


//-----------------LCD Variables-----------------
uint8_t lcd_brightness = 12;


//-----------------IMU Variables-----------------
float varAng, IMU_Y_deg_per_sec, IMU_Z_deg_per_sec, varSpd, varDst, varIang;

float gyroXoffset, gyro_deg_per_sec_Y_offset, gyro_deg_per_sec_Z_offset, accXoffset;

float gyroXdata, gyro_Y_data, gyroZdata, IMU_X_acceleration, IMU_Y_acceleration, IMU_Z_acceleration;

float Avg_IMU_X_deg_per_sec = 0.0, Avg_IMU_Y_deg_per_sec = 0.0, Avg_IMU_Z_deg_per_sec = 0.0;
float Avg_IMU_X_acceleration = 0.0, Avg_IMU_Y_acceleration = 0.0, Avg_IMU_Z_acceleration = 0.0;
float Avg_Robot_Z_deg_per_sec = 0.0;
float robot_X_deg = 0.0, robot_Y_deg = 0.0, robot_Z_deg = 0.0;











// Variables and stuff
boolean serialMonitor   = true;


boolean standing        = false;
boolean hasFallen       = false;
boolean isArmed         = false;
boolean abortWasHandled = false;

int16_t counter       = 0;
uint32_t time0 = 0, time1 = 0;
int16_t counterOverPwr = 0, maxOvp = 80, maxAngle = 30;
float power, powerR, powerL, yawPower;




float cutoff            = 0.1;                     //~=2 * pi * f (Hz)
const float clk         = 0.01;                    // in sec,
const uint32_t interval = (uint32_t)(clk * 1000);  // in msec
float Kang, Komg, KIang, Kyaw, Kdst, Kspd;
int16_t maxPwr;

float yawAngle = 0.0;
byte heading = 0;



float moveDestination, moveTarget;
float moveRate        = 0.0;
const float moveStep  = 0.2 * clk;
int16_t fbBalance     = 0;
int16_t motorDeadband = 0;
float mechFactR, mechFactL;
int8_t motorRDir = 0, motorLDir = 0;
bool spinContinuous = false;
float spinDest, spinTarget, spinFact = 1.0;
float spinStep  = 0.0;  // deg per 10msec
int16_t ipowerL = 0, ipowerR = 0;
int16_t motorLdir = 0, motorRdir = 0;  // 0:stop 1:+ -1:-

int Rmotor = 0, Lmotor = 0;

int16_t punchPwr, punchPwr2, punchDur, punchCountL = 0, punchCountR = 0;

byte demoMode = 0;

byte Abtn = 0;
byte Bbtn = 0;

void resetVar() {
    abortWasHandled = false;
    power               = 0.0;
    moveTarget          = 0.0;
    moveRate            = 0.0;
    spinContinuous      = false;
    spinDest            = 0.0;
    spinTarget          = 0.0;
    spinStep            = 0.0;
    yawAngle            = 0.0;
    varAng              = 0.0;
    IMU_Y_deg_per_sec   = 0.0;
    varDst              = 0.0;
    varSpd              = 0.0;
    varIang             = 0.0;
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


void update_exec_status(String status){
    exec_status = status;
    //LCD_Status_Message();
    exec_status_has_changed = true;
    Serial.println(status);
}

// -------Functions that should be run on second core-------

// Update mode 
// void setMode(bool inc) {
//     if (inc) demoMode = ++demoMode % 2;

//     LCD_Update_Mode();
// }

// // Update & show Battery Voltage On Display
// void updateBatVolt(){
//     vBatt = M5.Axp.GetBatVoltage();
//     LCD_DispBatVolt();
// }
