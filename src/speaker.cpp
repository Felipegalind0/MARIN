#include <M5StickCPlus.h>
#include "pinout.h"

void StartUp_Sound(){
    ledcWriteTone(SPEAKER_CH,1000);
    delay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    delay(200);
    ledcWriteTone(SPEAKER_CH,1400);
    delay(600);
    ledcWriteTone(SPEAKER_CH,0);
}
void Shutdown_Sound(){
    ledcWriteTone(SPEAKER_CH,1400);
    delay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    delay(200);
    ledcWriteTone(SPEAKER_CH,1000);
    delay(400);
    ledcWriteTone(SPEAKER_CH,0);
}