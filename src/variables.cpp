#include "variables.h"
#include "LCD.h"

// These are the 2 important global variables that are linked by variables.h

int x = 0;  // Defines robot rotation rate    + = R     &   - = L
int y = 0;  // Defines robot FWD/BACK         + = FWD   &   - = BACK

float vBatt, voltAve             = 3.7;

// Variables and stuff
boolean serialMonitor   = true;
boolean standing        = false;
boolean hasFallen       = false;
boolean abortWasHandled = false;
int16_t counter       = 0;
uint32_t time0 = 0, time1 = 0;
int16_t counterOverPwr = 0, maxOvp = 80, maxAngle = 30;
float power, powerR, powerL, yawPower;
float varAng, varOmg, varSpd, varDst, varIang;
float gyroXoffset, gyroYoffset, gyroZoffset, accXoffset;
float gyroXdata, gyroYdata, gyroZdata, accXdata, accYdata, accZdata;
float aveAccX = 0.0, aveAccZ = 0.0, aveAbsOmg = 0.0;
float cutoff            = 0.1;                     //~=2 * pi * f (Hz)
const float clk         = 0.01;                    // in sec,
const uint32_t interval = (uint32_t)(clk * 1000);  // in msec
float Kang, Komg, KIang, Kyaw, Kdst, Kspd;
int16_t maxPwr;
float yawAngle = 0.0;
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

int16_t punchPwr, punchPwr2, punchDur, punchCountL = 0, punchCountR = 0;

byte demoMode = 0;

byte Abtn = 0;
byte Bbtn = 0;

void resetVar() {
    abortWasHandled = false;
    power          = 0.0;
    moveTarget     = 0.0;
    moveRate       = 0.0;
    spinContinuous = false;
    spinDest       = 0.0;
    spinTarget     = 0.0;
    spinStep       = 0.0;
    yawAngle       = 0.0;
    varAng         = 0.0;
    varOmg         = 0.0;
    varDst         = 0.0;
    varSpd         = 0.0;
    varIang        = 0.0;
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

// -------Functions that should be run on second core-------

// Update mode 
void setMode(bool inc) {
    if (inc) demoMode = ++demoMode % 2;

    LCD_Update_Mode();
}

// Update & show Battery Voltage On Display
void updateBatVolt(){
    vBatt = M5.Axp.GetBatVoltage();
    LCD_DispBatVolt();
}
