//-----------------Variables.h-----------------

#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#define btnCounter 10
#define logCounter 100

#include <M5StickCPlus.h>
#include <LCD.h>

extern int x, y;

extern float deviceTemp;

//-----------------System Variables-----------------


#define RealTcore 1
#define BackgroundCore 0

extern byte robot_config_counter;
extern boolean is_booted;
extern String exec_status;
extern boolean exec_status_has_changed;

extern TaskHandle_t Task0, Task1;

extern SemaphoreHandle_t syncSemaphore;

extern uint8_t broadcastAddress[6]; // Array to store the MAC address

extern String sender_mac_str;


//-----------------Battery Variables-----------------

extern int perCentBatt;

extern float vBatt, voltAve;

extern float vBatt_min, vBatt_max;

extern boolean isCharging;


extern uint8_t sender_mac;
extern boolean remote_connected;
extern boolean should_reply_to_C_cmd;


//-----------------Time Variables-----------------
extern int64_t RealTcode_start_time, RealTcode_end_time, RealTcode_execution_time, RealTcode_no_execution_time, RealTcode_total_execution_time;
extern int64_t BackgroundTask_execution_time_start, BackgroundTask_execution_time_end, BackgroundTask_execution_time, BackgroundTask_total_execution_time, BackgroundTask_no_execution_time;

extern double RealTcode_CPU_load, BackgroundTask_CPU_load;



//-----------------Joystick Variables-----------------

extern byte JoyC_X, JoyC_Y;

extern float JoyC_r;

extern float JoyC_Phi;

extern byte JoyC_X_Cycles_In_Deadzone, JoyC_Y_Cycles_In_Deadzone;


extern boolean JoyC_In_X_DeadZone, JoyC_In_y_DeadZone;

extern int JoyC_X_left_right, JoyC_Y_up_down;

extern boolean JoyC_btn;

extern boolean JoyC_left, JoyC_right, JoyC_up, JoyC_down;

extern boolean JoyC_needs_to_return_to_center;

extern boolean JoyC_Xinput;

//-----------------LCD Variables-----------------
extern uint8_t lcd_brightness;

extern boolean standing, hasFallen, isArmed, abortWasHandled;

extern boolean serialMonitor;
extern int16_t counter;
extern uint32_t time0, time1;
extern int16_t counterOverPwr, maxOvp, maxAngle;
extern float power, powerR, powerL, yawPower;

extern float varAng;

extern float IMU_X_deg_per_sec, IMU_Y_deg_per_sec, IMU_Z_deg_per_sec;
extern float varSpd, varDst, varIang;

extern float gyroXoffset;
extern float gyro_deg_per_sec_X_offset, gyro_deg_per_sec_Y_offset, gyro_deg_per_sec_Z_offset, accXoffset, accYoffset, accZoffset;

extern boolean IMU_has_been_calibrated, IMU_has_been_init;

extern float IMU_RAW_X_dps, IMU_RAW_Y_dps, IMU_RAW_Z_dps, IMU_RAW_X_Gs, IMU_RAW_Y_Gs, IMU_RAW_Z_Gs;
extern float Avg_IMU_X_deg_per_sec, Avg_IMU_Y_deg_per_sec, Avg_IMU_Z_deg_per_sec;
extern float Avg_IMU_X_Gs, Avg_IMU_Y_Gs, Avg_IMU_Z_Gs;
extern float Avg_Robot_X_deg_per_sec, Avg_Robot_Y_deg_per_sec, Avg_Robot_Z_deg_per_sec;
extern float robot_X_deg, robot_Y_deg, robot_Z_deg;

extern float roll, pitch, yaw;

extern float gX, gY, gZ, aX, aY, aZ;


extern float cutoff;
extern const float clk;
extern const uint32_t interval;
extern float Kang, Komg, KIang, Kyaw, Kdst, Kspd;
extern int16_t maxPwr;
extern float yawAngle;
extern byte heading;
extern float moveDestination, moveTarget;
extern float moveRate;
extern const float moveStep;
extern int16_t fbBalance;
extern int16_t motorDeadband;
extern float mechFactR, mechFactL;
extern int8_t motorRDir, motorLDir;
extern bool spinContinuous;
extern float spinDest, spinTarget, spinFact;
extern float spinStep;
extern int16_t ipowerL, ipowerR;
extern int16_t motorLdir, motorRdir;
extern int Rmotor, Lmotor;
extern int16_t punchPwr, punchPwr2, punchDur, punchCountL, punchCountR;

extern byte demoMode;
extern byte Abtn;
extern byte Bbtn;

void updateBatVolt();
void setMode(bool inc);

void resetVar();
void resetPara();

void update_exec_status(String status);


#endif // _VARIABLES_H_
