//---------------------LCD.cpp---------------------
#include <M5StickCPlus.h>
#include "IO.h"
#include "variables.h"
#include "LCD.h"
#include "COMS.h"

uint16_t invertColor16(uint16_t color) {
    // Invert the color: 0xFFFF is max value for 16-bit, color is the original color
    return 0xFFFF - color;
}

TFT_eSprite canvas = TFT_eSprite(&M5.Lcd);

void LCD_flush(){
    canvas.pushSprite(0, 0);
    canvas.fillSprite(BLACK);
} 

void LCD_Status_Message(){

    String status_message = exec_status;

    // get the length of status_message
    //int status_message_length = status_message.length();

    canvas.setTextFont(1);
    canvas.setTextSize(1);
    //canvas.setTextColor(BLACK);
    
    //define the status box UI variables
    const int STATUS_M_X = 0;
    const int STATUS_M_Y = 230;
    const int STATUS_M_W = 135;
    const int STATUS_M_H = 10;
    const int STATUS_M_R = 5;

    const int STATUS_M_COLOR = WHITE;
    int STATUS_M_TEXT_COLOR = invertColor16(STATUS_M_COLOR);
    //int STATUS_M_TEXT_COLOR = BLACK;



    // define color for the status box as white
    

    canvas.fillRoundRect(STATUS_M_X, STATUS_M_Y,
     STATUS_M_W, STATUS_M_H, STATUS_M_R, STATUS_M_COLOR);
    
    canvas.setTextColor(STATUS_M_TEXT_COLOR);
    
    canvas.setCursor(STATUS_M_X+2, STATUS_M_Y+1);
    canvas.print(status_message);
}

void LCD_CPU_Widget(){
    canvas.setTextFont(1);
    canvas.setTextSize(1);
    
    //define the status box UI variables
    const int COUNTER_M_X = 33;
    const int COUNTER_M_Y = 122;
    const int COUNTER_M_W = 100;
    const int COUNTER_M_H = 10;
    const int COUNTER_M_R = 5;

    const int COUNTER_M_COLOR = WHITE;
    int COUNTER_M_TEXT_COLOR = invertColor16(COUNTER_M_COLOR);

    canvas.fillRoundRect(COUNTER_M_X, COUNTER_M_Y, COUNTER_M_W, COUNTER_M_H, COUNTER_M_R, COUNTER_M_COLOR);
    
    canvas.setTextColor(COUNTER_M_TEXT_COLOR);

    canvas.setCursor(COUNTER_M_X+5, COUNTER_M_Y+2);
    canvas.print("CPU");


    canvas.setCursor(COUNTER_M_X+30, COUNTER_M_Y+2);

    String S_CPU_load = "0:"+String(int(BackgroundTask_CPU_load))+"%"+ " 1:";

    if (RealTcode_CPU_load < 10){
        S_CPU_load += "0";
    }
    
    
    S_CPU_load += String(int(RealTcode_CPU_load))+"%";
    
    canvas.print(S_CPU_load);

}





void LCD_UI_Setup(){
    M5.Axp.ScreenBreath(11);
    //canvas.setRotation(2); // 2 = landscape inverted 0 = landscape
    M5.lcd.setRotation(2);
    M5.lcd.fillScreen(BLACK);

    canvas.createSprite(135, 240);
    canvas.fillRect(0, 0, 135, 240, BLACK);

    

    // set TFT_eSPI LCD Brightness
    M5.Axp.ScreenBreath(lcd_brightness);


    canvas.setTextFont(2);
    canvas.setTextSize(1);
    canvas.setTextColor(WHITE);

    canvas.setCursor(SM_X, SM_Y);
    canvas.print("Starting Systems");

    LCD_CORE_Message();

    LCD_IMU_Message();

    LCD_Felg_Message();

    LCD_flush();
}



boolean print_LCD_Loop = false;

boolean print_updating_battery_voltage = false;

boolean print_RealT_Times = true;

void LCD_loop(){

    LCD_Western_Artificial_Horizion();
    
    LCD_CPU_Widget();

    LCD_DispBatVolt();


    if ((counter % 100) == 0) {
            updateBatVolt();
            // Serial.print("COM() running on core ");
            // Serial.println(xPortGetCoreID());
            
    }
    // LCD_DispAngle();
    // LCD_DispRotation();

    LCD_Status_Message();


    canvas.pushSprite(0, 0);
}


void LCD_CORE_Message(void){
   canvas.setTextFont(2);
   canvas.setTextSize(1);
   canvas.setCursor(CORE_SM_X, CORE_SM_Y);
   canvas.print("Core");
   canvas.setTextFont(4);
   canvas.setTextSize(2);
   canvas.setCursor(CORE_SM_X, CORE_SM_Y + 15);
   canvas.print(xPortGetCoreID());
}

void LCD_IMU_Message(void){
   canvas.setTextFont(4);
   canvas.setTextSize(1);
   canvas.setCursor(LCD_IMU_SM_X, LCD_IMU_SM_Y);
   canvas.print("Please Lay");
   canvas.setCursor(LCD_IMU_SM_X+3, LCD_IMU_SM_Y + 35);
   canvas.print("Robot Flat");
}

void LCD_Felg_Message(void){
   canvas.setTextFont(2);
   canvas.setTextSize(1);

   canvas.setCursor(LCD_FELG_SM_X+5, LCD_FELG_SM_Y);
   canvas.print("Designed By");

   canvas.setCursor(LCD_FELG_SM_X, LCD_FELG_SM_Y + 15);
   canvas.print("Felipe Galindo");

   canvas.setCursor(LCD_FELG_SM_X+3, LCD_FELG_SM_Y + 30);
   canvas.print("in Minnesota");
}

// void LCD_calib1_Message(void){
//    canvas.setTextFont(2);
//    canvas.setTextSize(1);
//    canvas.setTextColor(RED);

//    canvas.setCursor(Middle_M_X, Middle_M_Y);
//    canvas.print("Executing");

//    canvas.setCursor(Middle_M_X, Middle_M_Y + 15);
//    canvas.print("Stationary");

//    canvas.setCursor(Middle_M_X, Middle_M_Y + 30);
//    canvas.print("Calibration");

//    canvas.setTextColor(WHITE);
// }

void LCD_calib1_Message(void){

}

void LCD_calib1_complete_Message(void){
   canvas.setTextFont(2);
   canvas.setTextSize(1);
   canvas.setTextColor(RED);

   canvas.setCursor(Middle_M_X, Middle_M_Y);
   canvas.print("Stationary");


   canvas.setCursor(Middle_M_X, Middle_M_Y + 15);
   canvas.print("Calibration");

   canvas.setCursor(Middle_M_X, Middle_M_Y + 30);
   canvas.print("Complete");

   canvas.setTextColor(WHITE);
}

void LCD_calib2_Message(void){
   canvas.setTextFont(2);
   canvas.setTextSize(1);
   canvas.setTextColor(RED);

   canvas.setCursor(Middle_M_X, Middle_M_Y);
   canvas.print("Executing");

   canvas.setCursor(Middle_M_X, Middle_M_Y + 15);
   canvas.print("Balance");

   canvas.setCursor(Middle_M_X, Middle_M_Y + 30);
   canvas.print("Calibration");

   canvas.setTextColor(WHITE);
}

// void LCD_Setup(){
//     M5.Axp.ScreenBreath(11);
//    canvas.setRotation(2);
//    canvas.fillScreen(BLACK);
//    canvas.setTextFont(2);
//    canvas.setTextSize(1);
//    canvas.setTextColor(WHITE);

//    canvas.setCursor(SM_X, SM_Y);
//    canvas.print("Starting Systems");
    
//     LCD_CORE_Message();

//     LCD_IMU_Message();

//     LCD_Felg_Message();
// }



void LCD_DispBatVolt() {

    const int LCD_BTv_X = 0;
    const int LCD_BTv_Y = 122; //220
    const int LCD_BTv_W = 35;
    const int LCD_BTv_H = 13;
    const int LCD_BTv_R = 5;
    
    int LCD_BTv_C = TFT_LIGHTGREY;

    if(perCentBatt > 80){
        LCD_BTv_C = GREEN;
    }
    else if(isCharging){
        LCD_BTv_C = BLUE;
    }
    else if (perCentBatt == -1){
        LCD_BTv_C = TFT_LIGHTGREY;
    }
    else if (perCentBatt < 20){
        LCD_BTv_C = RED;
    }
    else if(perCentBatt < 40){
        LCD_BTv_C = YELLOW;
    }

    canvas.setTextFont(1);
    canvas.setTextSize(1);

    canvas.setCursor(LCD_BTv_X+3, LCD_BTv_Y+3);

    canvas.fillRoundRect(LCD_BTv_X, LCD_BTv_Y,
     LCD_BTv_W, LCD_BTv_H, LCD_BTv_R, LCD_BTv_C);
    canvas.fillRoundRect(LCD_BTv_X+LCD_BTv_W, LCD_BTv_Y+2,
     4, 8, 2, LCD_BTv_C);

    String text = "";

    if (perCentBatt <= -1){
        text = " ... ";
    }
    else if (perCentBatt == 100){
        text = String(perCentBatt) + "%";
    }
        
    else{
        text = " " + String(perCentBatt) + "%";
    }


        
    

    //canvas.printf("%4.2fv ", vBatt);
    canvas.setTextColor(BLACK);
    canvas.print(text);
    //canvas.printf("%1.2fv ", vBatt);
    //canvas.printf("%1.0fv ", perCentBatt);

}


// void LCD_DispBatVolt() {
//    canvas.setTextFont(2);
//    canvas.setTextSize(1);

//    canvas.setCursor(LCD_BTv_X, LCD_BTv_Y);

//    canvas.fillRoundRect(LCD_BTv_X-5, LCD_BTv_Y-2,
//      LCD_BTv_W, LCD_BTv_H, LCD_BTv_R, BLUE);

//    canvas.printf("%4.2fv ", vBatt);
// }


// uint32_t interpolateColor(int value, int minRange, int midRange, int maxRange, uint32_t startColor, uint32_t midColor, uint32_t endColor) {
//     // Normalize value to 0-1
//     float normalized = 0;
//     uint32_t color = 0;
    
//     if (value <= midRange) {
//         normalized = (float)(value - minRange) / (midRange - minRange);
//         uint8_t r = ((uint16_t)(((startColor >> 16) & 0xFF) * (1 - normalized) + ((midColor >> 16) & 0xFF) * normalized)) & 0xFF;
//         uint8_t g = ((uint16_t)(((startColor >> 8) & 0xFF) * (1 - normalized) + ((midColor >> 8) & 0xFF) * normalized)) & 0xFF;
//         uint8_t b = ((uint16_t)((startColor & 0xFF) * (1 - normalized) + (midColor & 0xFF) * normalized)) & 0xFF;
//         color = M5.Lcd.color565(r, g, b);
//     } else {
//         normalized = (float)(value - midRange) / (maxRange - midRange);
//         uint8_t r = ((uint16_t)(((midColor >> 16) & 0xFF) * (1 - normalized) + ((endColor >> 16) & 0xFF) * normalized)) & 0xFF;
//         uint8_t g = ((uint16_t)(((midColor >> 8) & 0xFF) * (1 - normalized) + ((endColor >> 8) & 0xFF) * normalized)) & 0xFF;
//         uint8_t b = ((uint16_t)((midColor & 0xFF) * (1 - normalized) + (endColor & 0xFF) * normalized)) & 0xFF;
//         color = M5.Lcd.color565(r, g, b);
//     }

//     return color;
// }

// Extract RGB components from RGB565
auto extractRGB565 = [](uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
    r = (color >> 11) & 0x1F;
    g = (color >> 5) & 0x3F;
    b = color & 0x1F;
};

// Function to mix two RGB565 colors based on a weight
auto mixRGB565 = [](uint16_t color1, uint16_t color2, float weight, uint16_t &result) {
    uint8_t r1, g1, b1, r2, g2, b2;
    extractRGB565(color1, r1, g1, b1);
    extractRGB565(color2, r2, g2, b2);

    // Corrected: Cast to uint8_t after computing the blend, before applying the mask
    uint8_t r = static_cast<uint8_t>(r1 * (1 - weight) + r2 * weight) & 0x1F;
    uint8_t g = static_cast<uint8_t>(g1 * (1 - weight) + g2 * weight) & 0x3F;
    uint8_t b = static_cast<uint8_t>(b1 * (1 - weight) + b2 * weight) & 0x1F;

    result = (r << 11) | (g << 5) | b;
};



uint16_t interpolateColor(int value, int minRange, int midRange, int maxRange, uint16_t startColor, uint16_t midColor, uint16_t endColor) {

    // make sure value is within range

    if (value < minRange){
        value = minRange;
    }
    else if (value > maxRange){
        value = maxRange;
    }


    // Normalize value to 0-1
    float normalized = 0;
    uint16_t color = 0;
    


    if (value <= midRange) {
        normalized = (float)(value - minRange) / (midRange - minRange);
        mixRGB565(startColor, midColor, normalized, color);
    } else {
        normalized = (float)(value - midRange) / (maxRange - midRange);
        mixRGB565(midColor, endColor, normalized, color);
    }

    return color;
}




void LCD_DispRotation() {

    int LCD_DispRotation_X = 45;
    int LCD_DispRotation_Y = 80;

    int LCD_DispRotation_W = 80;
    int LCD_DispRotation_H = 30;
    int LCD_DispRotation_R = 10;
    
    canvas.setTextFont(4);
    canvas.setTextSize(1);
    //canvas.setCursor(LCD_ANGLE_X-5, LCD_ANGLE_Y);
    canvas.setCursor(LCD_DispRotation_X, LCD_DispRotation_Y);

    // canvas.fillRoundRect(LCD_ANGLE_X+10, LCD_ANGLE_Y-10,
    //     LCD_ANGLE_W, LCD_ANGLE_H, LCD_ANGLE_R, GREEN);
    canvas.fillRoundRect(LCD_DispRotation_X, LCD_DispRotation_Y,
        LCD_DispRotation_W, LCD_DispRotation_H, LCD_DispRotation_R, RED);

    
    //canvas.printf("%2+.2f   ", (int(yawAngle) % 360));
    canvas.print(String(Avg_IMU_Z_deg_per_sec));
    //canvas.printf("%2+.2f   ", yawAngle);
}

void LCD_Western_Artificial_Horizion(){

    int AH_X = 18;
    int AH_Y = 20;
    int AH_W = 100;
    int AH_H = 100;

    const int JoyCircle_X = AH_X + AH_W/2;
    const int JoyCircle_Y = AH_Y + AH_H/2;
    const int JoyCircle_R = 10;
    const int JoyCircle_IR = 8;
    
    int JoyCircle_OC = BLACK;

    // // Define colors for interpolation
    // uint32_t red = tft.color565(255, 0, 0);
    // uint32_t yellow = tft.color565(255, 255, 0);
    // uint32_t blue = tft.color565(0, 0, 255);
    // uint32_t lightBlue = tft.color565(0, 128, 255);
    // uint32_t green = tft.color565(0, 255, 0);




    int Sky_C = 0x871F;
    //int Sky_C = 0x111F;
    //int Sky_C = TFT_BLUE;

    int Ground_C = 0x87CE;
    //int Ground_C = TFT_GREEN;


    // RGB565(255, 255, 150) = 0xFF9F

    int LIGHT_YELLOW = 0xFFFF69; // ;)







    // canvas.fillRoundRect(AH_X-10, AH_Y-10, AH_W+20, AH_H+20, 10, TFT_LIGHTGREY);


    // Calculate point offsets

    // int AH_Y_Offset = robot_Y_deg;
    // int AH_dif_offset = Avg_IMU_Z_deg_per_sec/4;
    
    int AH_Y_Offset = robot_Y_deg;
    int AH_dif_offset = Avg_IMU_Z_deg_per_sec;

    // Calculate the horizon line


    int AH_Y_center = AH_Y + AH_H/2;

    int AH_Horizon_x0 = AH_X;
    int AH_Horizon_y0 = AH_Y_center - AH_Y_Offset + AH_dif_offset;

    int AH_Horizon_x1 = AH_X + AH_W;
    int AH_Horizon_y1 = AH_Y_center - AH_Y_Offset - AH_dif_offset;


    // make sure the horizon line is within the screen

    if (AH_Horizon_y0 < AH_Y){
        AH_Horizon_y0 = AH_Y;
    }
    else if (AH_Horizon_y0 > (AH_Y + AH_H)){
        AH_Horizon_y0 = AH_Y + AH_H;
    }


    if (AH_Horizon_y1 < AH_Y){
        AH_Horizon_y1 = AH_Y;
    }
    else if (AH_Horizon_y1 > (AH_Y + AH_H)){
        AH_Horizon_y1 = AH_Y + AH_H;
    }




    // draw the 4 triangles

    // draw the sky

    canvas.fillTriangle(AH_X         , AH_Y,    (AH_X + AH_W),          AH_Y,   AH_Horizon_x0, AH_Horizon_y0, Sky_C);
    canvas.fillTriangle((AH_X + AH_W), AH_Y,    AH_Horizon_x1, AH_Horizon_y1,   AH_Horizon_x0, AH_Horizon_y0, Sky_C);

    // draw the ground
    canvas.fillTriangle(AH_X, (AH_Y + AH_H), AH_Horizon_x1, AH_Horizon_y1, AH_Horizon_x0, AH_Horizon_y0, Ground_C);
    canvas.fillTriangle(AH_X, (AH_Y + AH_H),    (AH_X + AH_W), (AH_Y + AH_H),   AH_Horizon_x1, AH_Horizon_y1, Ground_C);


        
    int JoyCircle_IC = TFT_LIGHTGREY;

    if (JoyC_btn){
        JoyCircle_IC = RED;
    }
    else if (JoyC_r < 5){
        JoyCircle_IC = TFT_LIGHTGREY;
    }
    else if (JoyC_r > 40){
        JoyCircle_IC = BLACK;
    }
    else{
        JoyCircle_IC = TFT_DARKGREY;
    }
    
    float inner_JoyCircle_X = JoyCircle_X + ((JoyC_X)) - (AH_W/2);
    float inner_JoyCircle_Y = JoyCircle_Y - ((JoyC_Y)) + (AH_H/2); // y is inverted

    if (JoyC_X == 50 && JoyC_Y == 50){
        JoyCircle_OC = BLACK;
    }
    else{
        JoyCircle_OC = TFT_LIGHTGREY;
    }

    canvas.fillCircle(inner_JoyCircle_X, inner_JoyCircle_Y, JoyCircle_IR, JoyCircle_IC);





    // draw the horizon line

    canvas.drawLine(AH_X, (AH_Y + (AH_H/2)), (AH_X + AH_W), (AH_Y + (AH_H/2)), TFT_LIGHTGREY);


    int AH_Motors_Y_Offset = (Rmotor + Lmotor)/2;
    int AH_Motors_dif_offset = 2 * (Rmotor - Lmotor);

    // make sure that offsets are not more than .5 height

    if (AH_Motors_Y_Offset > AH_H/2){
        AH_Motors_Y_Offset = AH_H/2;
    }
    else if (AH_Motors_Y_Offset < -AH_H/2){
        AH_Motors_Y_Offset = -AH_H/2;
    }

    if (AH_Motors_dif_offset > AH_H/2){
        AH_Motors_dif_offset = AH_H/2;
    }
    else if (AH_Motors_dif_offset < -AH_H/2){
        AH_Motors_dif_offset = -AH_H/2;
    }


    // draw the motor indicator lines

    // canvas.drawLine(AH_X, (AH_Y + (AH_H/2) + Rmotor)-1, (AH_X + AH_W), (AH_Y + (AH_H/2) + Lmotor)-1, BLACK); 
    // canvas.drawLine(AH_X, (AH_Y + (AH_H/2) + Rmotor)+1, (AH_X + AH_W), (AH_Y + (AH_H/2) + Lmotor)+1, BLACK); 

    canvas.drawLine(AH_X, (AH_Y + (AH_H/2) + AH_Motors_Y_Offset + AH_Motors_dif_offset)-1, (AH_X + AH_W), (AH_Y + (AH_H/2) + AH_Motors_Y_Offset - AH_Motors_dif_offset)-1, BLACK);
    canvas.drawLine(AH_X, (AH_Y + (AH_H/2) + AH_Motors_Y_Offset + AH_Motors_dif_offset)+1, (AH_X + AH_W), (AH_Y + (AH_H/2) + AH_Motors_Y_Offset - AH_Motors_dif_offset)+1, BLACK);


    // draw the aircraft

    // canvas.setTextColor(BLACK);

    // canvas.setCursor(AH_X+AH_W/2-10, AH_Y+AH_H/2-5);
    // canvas.print("--w--"); 




    canvas.drawCircle(JoyCircle_X, JoyCircle_Y, JoyCircle_R, JoyCircle_OC);


    // print the joystick values

    byte AH_JoyC_X = (AH_X)-18;
    byte AH_JoyC_Y = (AH_Y)-20;
    byte AH_JoyC_W = 30;
    byte AH_JoyC_H = 20;
    int AH_JoyC_C = TFT_LIGHTGREY;

    canvas.setTextFont(1);
    canvas.setTextSize(2);

    //canvas.fillRect(AH_JoyC_X, AH_JoyC_Y, AH_JoyC_W, AH_JoyC_H, AH_JoyC_C);

    int Rect_color = interpolateColor(JoyC_X, 0, 50, 100, BLUE, LIGHT_YELLOW, RED);
    canvas.setTextColor(BLACK);

    

    canvas.fillRoundRect(AH_JoyC_X, AH_JoyC_Y, AH_JoyC_W, AH_JoyC_H, 5, Rect_color);
    canvas.setCursor(AH_JoyC_X+2, AH_JoyC_Y+2);

    // if (JoyC_X == 50){
    //     canvas.setTextColor(TFT_LIGHTGREY);
    // }
    // else if (5 >= JoyC_X || JoyC_X >= 95){
    //     canvas.setTextColor(BLACK);
    // }
    // else{
    //     canvas.setTextColor(TFT_DARKGREY);
    // }

    if (JoyC_X < 10){
        canvas.print("0"+String(JoyC_X));
    }
    else{
        canvas.print(JoyC_X);
    }

    
    Rect_color = interpolateColor(int(IMU_Z_deg_per_sec), -100, 0, 100, RED, LIGHT_YELLOW, BLUE);
    //canvas.setTextColor(invertColor16(Rect_color));

    canvas.fillRoundRect((AH_JoyC_X + AH_JoyC_W), AH_JoyC_Y, AH_JoyC_W+10, AH_JoyC_H, 5, Rect_color);
    canvas.setCursor(AH_JoyC_X-4 + AH_JoyC_W, AH_JoyC_Y + 3);
    // if (IMU_Z_deg_per_sec == 50){
    //     canvas.setTextColor(TFT_LIGHTGREY);
    // }
    // else if (5 >= IMU_Z_deg_per_sec || IMU_Z_deg_per_sec >= 95){
    //     canvas.setTextColor(BLACK);
    // }
    // else{
    //     canvas.setTextColor(TFT_DARKGREY);
    // }

    if (IMU_Z_deg_per_sec > 0){
        canvas.print(" "+String(int(IMU_Z_deg_per_sec)));
    }
    else{
        canvas.print(int(IMU_Z_deg_per_sec));
    }


    Rect_color = interpolateColor(JoyC_Y, 0, 50, 100, GREEN, Sky_C, BLUE);
    //canvas.setTextColor(invertColor16(Rect_color));
    
    canvas.fillRoundRect((AH_JoyC_X + 2 * AH_JoyC_W+10), AH_JoyC_Y, AH_JoyC_W, AH_JoyC_H, 5, Rect_color);

    canvas.setCursor((AH_JoyC_X + 2 * AH_JoyC_W + 13), AH_JoyC_Y + 3);
    // if (JoyC_Y == 50){
    //     canvas.setTextColor(TFT_LIGHTGREY);
    // }
    // else if (5 >= JoyC_Y || JoyC_Y >= 95){
    //     canvas.setTextColor(BLACK);
    // }
    // else{
    //     canvas.setTextColor(TFT_DARKGREY);
    // }


    if (JoyC_Y < 10){
        canvas.print("0"+String(JoyC_Y));
    }
    else{
        canvas.print(JoyC_Y);
    }

    Rect_color = interpolateColor(int(robot_Y_deg), -100, 0, 100, RED, BLUE, RED);
    //canvas.setTextColor(invertColor16(Rect_color));

    canvas.fillRoundRect((AH_JoyC_X + 3 * AH_JoyC_W) + 10, AH_JoyC_Y , AH_JoyC_W + 5, AH_JoyC_H, 5, Rect_color);
    canvas.setCursor((AH_JoyC_X + 3 * AH_JoyC_W + 5), AH_JoyC_Y + 3);
    // if (JoyC_Y == 50){
    //     canvas.setTextColor(TFT_LIGHTGREY);
    // }
    // else if (5 >= JoyC_Y || JoyC_Y >= 95){
    //     canvas.setTextColor(BLACK);
    // }
    // else{
    //     canvas.setTextColor(TFT_DARKGREY);
    // }

    if (robot_Y_deg > 0){
        canvas.print(" "+String(int(robot_Y_deg)));
    }
    else{
        canvas.print(int(robot_Y_deg));
    }

}




void LCD_DispAngle() {
   canvas.setTextFont(6);
   canvas.setTextSize(1);
   canvas.setCursor(LCD_ANGLE_X-5, LCD_ANGLE_Y);
   //canvas.draw

   canvas.fillRoundRect(LCD_ANGLE_X+10, LCD_ANGLE_Y-10,
     LCD_ANGLE_W, LCD_ANGLE_H, LCD_ANGLE_R, GREEN);

   //canvas.printf("%5.0f   ", (-Avg_IMU_Z_acceleration)*90.0);

    canvas.printf("%5.0f   ", (robot_Y_deg));
}

void LCD_Update_Mode(){
   canvas.setTextFont(4);
   canvas.setTextSize(1);

   canvas.fillScreen(BLACK);

   canvas.setCursor(30, 5);
    
    if (demoMode == 0)
       canvas.print("Stand ");

    else if (demoMode == 1)
       canvas.print("Demo ");

}

void LCD_Abort_Message(){
   canvas.setTextFont(2);
   canvas.setTextSize(1);
   canvas.setTextColor(BLACK);

   canvas.fillRoundRect(Middle_M_X-30, Middle_M_Y-3,
     ABORT_M_W, ABORT_M_H, ABORT_M_R, RED);

   canvas.setCursor(Middle_M_X-25, Middle_M_Y);
   canvas.print("Robot Deactivated");

   canvas.setCursor(Middle_M_X-25, Middle_M_Y + 15);
   canvas.print("Reason:");

}

void LCD_Abort_DEG_Message(){

   canvas.setCursor(Middle_M_X-25, Middle_M_Y + 30);
   canvas.print("Max DEG:");
   canvas.println(maxAngle);

   canvas.setCursor(Middle_M_X+25, Middle_M_Y + 15);
   canvas.print("DEG:");
   canvas.print(varAng);

   canvas.setTextColor(WHITE);
}

void LCD_Abort_PWR_Message(){

   canvas.setCursor(Middle_M_X-25, Middle_M_Y + 30);
   canvas.print("Max PWR:");
   canvas.println(maxOvp);

   canvas.setCursor(Middle_M_X+25, Middle_M_Y + 15);
   canvas.print("PWR:");
   canvas.print(counterOverPwr);

   canvas.setTextColor(WHITE);
}

void LCD_Resume_from_Abort_Message(){
    LCD_IMU_Message();
}