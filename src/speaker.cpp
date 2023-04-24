#include <M5StickCPlus.h>
#include "pinout.h"

void StartUp_Sound(){
    ledcWriteTone(SPEAKER_CH,1000);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1400);
    vTaskDelay(600);
    ledcWriteTone(SPEAKER_CH,0);
}
void Shutdown_Sound(){
    ledcWriteTone(SPEAKER_CH,1400);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1000);
    vTaskDelay(400);
    ledcWriteTone(SPEAKER_CH,0);
}