#include "variables.h"
#include "LCD.h"

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


boolean print_web_serial_not_initialized = false;

void updateBatVolt(){

    //Serial.println("");

    //Serial.print("isCharging = M5.Axp.GetVBusVoltage() > 4.2");

    isCharging = M5.Axp.GetVBusVoltage() > 4.2;

    //Serial.print("isChargeing = ");
    //Serial.println(isCharging);

    //Serial.println("deviceTemp = M5.Axp.GetTempInAXP192();");

    deviceTemp = M5.Axp.GetTempInAXP192();


    //Serial.println("WebSerial");

    // if (WebSerial.availableForWrite()) {
    //     Serial.println("WebSerial INITIALIZED");
    //     WebSerial.println(" Battery Voltage: " + String(voltAve) + "V" + " | " + String(perCentBatt) + "%");
        
    // }
    // else {
    //     if (print_web_serial_not_initialized) {
    //         Serial.println("WebSerial NOT INITIALIZED");
    //     }
        
    // }

    
    if (counter > 100) {
    //if(vBatt < 0.0) { // If the battery voltage is not available
        //vBatt = M5.Axp.GetVBusVoltage();
        vBatt = M5.Axp.GetBatVoltage();
        voltAve = vBatt; // Set the average voltage to the current voltage
        
    } else {
        //vBatt = M5.Axp.GetVBusVoltage();
        vBatt = M5.Axp.GetBatVoltage();
        voltAve = (voltAve * 0.5) + (vBatt * 0.5);
    }


    if (vBatt < vBatt_min) {

        vBatt_min = vBatt;

        Serial.println("NEW vBatt_min = " + String(vBatt_min));
    
    }
    else if (vBatt > vBatt_max) {

        //vBatt_max = vBatt;

        //Serial.println("NEW vBatt_max = " + String(vBatt_max));
    }
    
    //perCentBatt = map(vBatt, vBatt_min, vBatt_max, 0, 100);

    if (counter < 100){
        perCentBatt = -1;
    }
    else {
        perCentBatt = mapfloat(voltAve, vBatt_min, vBatt_max, 0, 100);
        if (perCentBatt > 100) perCentBatt = 100;
        else if (perCentBatt < 0) perCentBatt = -1;
    }

    

    //isCharging = M5.Axp.GetBatChargeCurrent() > 0;


}