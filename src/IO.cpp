//#include <WebSerial.h>
#include "variables.h"
#include "speaker.h"
#include "IO.h"

void RED_LED(bool state){
    digitalWrite(LED, !state);
}

//Check if button has been pressed
void CheckButtons() {
    byte pbtn = M5.Axp.GetBtnPress();
    //byte Abtn = 
    M5.BtnA.read();
    M5.BtnB.read();
    Abtn = (M5.BtnA.wasPressed());
    Bbtn = (M5.BtnB.wasPressed());

    if (pbtn == 2){

        Shutdown_Sound();

        esp_restart();

    }
        
        
    // else if (pbtn == 1){
    //     setMode(true);  // long push
    // }

    if (Abtn){
      Serial.println("A Button Pressed");
    }

    if (Bbtn){
      Serial.println("B Button Pressed");
    }

}

void drvMotorL(int16_t pwm) {
    drvMotor(0, (int8_t)constrain(pwm, -127, 127));

    Lmotor = pwm/1.27;
}

void drvMotorR(int16_t pwm) {
    drvMotor(1, (int8_t)constrain(-pwm, -127, 127));

    Rmotor = pwm/1.27;
}

void drvMotor(byte ch, int8_t sp) {
    Wire.beginTransmission(0x38);
    Wire.write(ch);
    Wire.write(sp);
    Wire.endTransmission();
}