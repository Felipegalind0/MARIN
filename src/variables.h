#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#define btnCounter 10
#define logCounter 100

#include <M5StickCPlus.h>

extern int x, y;

extern float deviceTemp;

//-----------------Battery Variables-----------------

extern int perCentBatt;

extern float vBatt, voltAve;

extern float vBatt_min, vBatt_max;

extern boolean isCharging;


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

extern boolean standing, hasFallen, abortWasHandled;

extern boolean serialMonitor;
extern int16_t counter;
extern uint32_t time0, time1;
extern int16_t counterOverPwr, maxOvp, maxAngle;
extern float power, powerR, powerL, yawPower;
extern float varAng, IMU_Y_deg_per_sec, IMU_Z_deg_per_sec, varSpd, varDst, varIang;
extern float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;
extern float gyroXdata, gyroYdata, gyroZdata, IMU_X_acceleration, IMU_Y_acceleration, IMU_Z_acceleration;
extern float Avg_IMU_X_deg_per_sec, Avg_IMU_Y_deg_per_sec, Avg_IMU_Z_deg_per_sec;
extern float Avg_IMU_X_acceleration, Avg_IMU_Y_acceleration, Avg_IMU_Z_acceleration;
extern float Avg_Robot_X_deg_per_sec, Avg_Robot_Y_deg_per_sec, Avg_Robot_Z_deg_per_sec;
extern float robot_X_deg, robot_Y_deg, robot_Z_deg;
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


#endif // _VARIABLES_H_
