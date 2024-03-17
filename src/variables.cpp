//----------------variables.cpp----------------
#include "variables.h"
#include "LCD.h"

// These are the 2 important global variables that are linked by variables.h

int x = 0;  // Defines robot rotation rate    + = R     &   - = L
int y = 0;  // Defines robot FWD/ROBOT_BACK_SIDE         + = FWD   &   - = ROBOT_BACK_SIDE

float deviceTemp = -1.0;


byte wireless_status = OFF;
//-----------------System Variables-----------------
byte robot_config_counter = 0;
boolean is_booted = false;
String exec_status = "OFF";
boolean exec_status_has_changed = false;

TaskHandle_t Task0, Task1;

SemaphoreHandle_t syncSemaphore;

String remote_msg = "";

byte i_msg = 0;

//-----------------Battery Variables-----------------


int perCentBatt = -1;

float vBatt, voltAve             = -1.0;

float vBatt_min = 3.3, vBatt_max = 4.1;

boolean isCharging = false;


uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Array to store the MAC address


String sender_mac_str = "";
uint8_t sender_mac = 0x00;
boolean remote_connected = false;
boolean should_reply_to_C_cmd = false;



//-----------------Time Variables-----------------
int64_t RealTcode_start_time = 0, RealTcode_end_time = 0, RealTcode_execution_time = 0, RealTcode_no_execution_time = 0, RealTcode_total_execution_time = 0;
int64_t BackgroundTask_execution_time_start = 0, BackgroundTask_execution_time_end = 0, BackgroundTask_execution_time = 0, BackgroundTask_total_execution_time = 0, BackgroundTask_no_execution_time = 0;

byte RealTcode_CPU_load = 0.0, BackgroundTask_CPU_load = 0.0;



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
float varAng = 0.0;

float IMU_X_deg_per_sec = 0.0, IMU_Y_deg_per_sec = 0.0, IMU_Z_deg_per_sec = 0.0;

float varSpd, varDst, varIang, I_y_dps;



float gyroXoffset;

// float accXoffset, gyro_deg_per_sec_Z_offset; 

float accXoffset, gyro_deg_per_sec_Z_offset; 

float gyro_deg_per_sec_X_offset, gyro_deg_per_sec_Y_offset, accYoffset, accZoffset;

float IMU_RAW_X_dps, IMU_RAW_Y_dps, IMU_RAW_Z_dps, IMU_RAW_X_Gs, IMU_RAW_Y_Gs, IMU_RAW_Z_Gs;

boolean IMU_has_been_calibrated = false, IMU_has_been_init = false, IMU_balance_point_has_been_set = false;

byte Wireless_status = OFF;
float Avg_IMU_X_deg_per_sec = 0.0, Avg_IMU_Y_deg_per_sec = 0.0, Avg_IMU_Z_deg_per_sec = 0.0;
float Avg_IMU_X_Gs = 0.0, Avg_IMU_Y_Gs = 0.0, Avg_IMU_Z_Gs = 0.0;

float Avg_Robot_Z_deg_per_sec = 0.0;

float robot_X_deg = 0.0, robot_Y_deg = 0.0, robot_Z_deg = 0.0;

float roll = 0.0, pitch = 0.0, yaw = 0.0;



float robot_yaw = 0.0;

float gX = 0.0, gY = 0.0, gZ = 0.0, aX = 0.0, aY = 0.0, aZ = 0.0;










// Variables and stuff
boolean serialMonitor   = true;


boolean standing        = false;
boolean hasFallen       = false;
boolean isArmed         = false;
boolean abortWasHandled = false;

boolean takeoffRequested = false;

int16_t counter       = 0;
uint32_t time0 = 0, time1 = 0;
int16_t counterOverPwr = 0, maxOvp = 80, maxAngle = 40;
float power, powerR, powerL, yawPower;




float cutoff            = 0.1;                     //~=2 * pi * f (Hz)
const float clk         = 0.01;                    // in sec,
const uint32_t interval = (uint32_t)(clk * 1000);  // in msec
float Kang, Komg, KIang, KI_y_dps, Kyaw, Kdst, Kspd;
int16_t maxPwr;

float yawAngle = 0.0;
byte heading = 0;
int takeoffTime = 0;



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
    I_y_dps             = 0.0;
    IMU_Y_deg_per_sec   = 0.0;
    varDst              = 0.0;
    varSpd              = 0.0;
    varIang             = 0.0;
}

// void resetPara() {
//     Kang          = 37.0;
//     Komg          = 0.84;
//     KIang         = 800.0;
//     Kyaw          = 4.0;
//     Kdst          = 85.0;
//     Kspd          = 2.7;
//     mechFactL     = 0.45;
//     mechFactR     = 0.45;
//     punchPwr      = 20;
//     punchDur      = 1;
//     fbBalance     = -3;
//     motorDeadband = 10;
//     maxPwr        = 126;
//     punchPwr2     = max(punchPwr, motorDeadband);
// }

void resetPara() {
    //Kang          = 20.0;
    //Kang          = 37.0;
    //Kang          = 50.0;
    // Kang          = 60.0;
    Kang          = 65.0;
    //Kang          = 70.0; BAD makes the robot quickly ocilate in pitch, it does not fall, but you can hear the motors working hard

    //Komg          = 0.84;
    //Komg          = 1;
    //Komg          = 1.5;
    //Komg          = 1.7;
    Komg          = 2.0;
    //Komg          = 2.2;
    //Komg          = 3; //BAD makes the robot very quickly ocilate in pitch, it does not fall, but you can hear the motors working hard

    //KIang         = 400.0;
    //KIang         = 800.0;
    //KIang         = 1000.0; 
    //KIang         = 1200.0; //better
    //KIang         = 1400.0; //Maybe better
    //KIang         = 2200.0; //Maybe better
    KIang         = 2400.0; //Maybe better

    KI_y_dps         = 10.0; //Maybe better

    //Kyaw          = 1.0;
    Kyaw          = 4.0;


    //kdst seems to scale the response of the robot to forward/backward 
    //(X axis) movement
    //Kdst          = 5.0;
    //Kdst          = 85.0;
    //Kdst          = 100.0; // best
    //Kdst          = 130.0;
    // Kdst          = 200.0;
    Kdst          = 300.0;
    //Kdst          = 385.0; // BAD: Medium speed ocilations

    //Kspd          = 2.7;
    //Kspd          = 4.7;
    //Kspd          = 7.0;
    //Kspd          = 8.0;
    Kspd          = 9.0;
    //Kspd          = 10.0; // too much

    mechFactL     = 0.5;
    mechFactR     = 0.5;
    // punchPwr      = 10;
    punchPwr      = 2;
    punchDur      = 1;
    //fbBalance     = -3;
    fbBalance     = 0;
    motorDeadband = 5;
    //motorDeadband = 10;
    maxPwr        = 126;
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
