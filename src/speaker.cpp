#include <M5StickCPlus.h>
#include "IO.h"

void Setup_Speaker(){
    pinMode(SPEAKER, OUTPUT);
    ledcSetup(SPEAKER_CH, 5000, 8);
    ledcAttachPin(SPEAKER, SPEAKER_CH);
}

void Remote_Sound() {
    const int remoteMelody[] = {1200, 1500, 1200, 1500, 1200}; // A sequence of higher-pitched tones
    const int duration = 50; // Short duration for each tone

    for (int i = 0; i < sizeof(remoteMelody) / sizeof(remoteMelody[0]); i++) {
        ledcWriteTone(SPEAKER_CH, remoteMelody[i]);
        vTaskDelay(duration / portTICK_PERIOD_MS);
    }

    ledcWriteTone(SPEAKER_CH, 0); // Turn off the sound
}

void StartUp_Sound( void * pvParameters ){

    Remote_Sound();

    vTaskDelay(200);
    
    ledcWriteTone(SPEAKER_CH,1000);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1400);
    vTaskDelay(600);
    ledcWriteTone(SPEAKER_CH,0);

    vTaskDelete(NULL);

    return;
}

// void StartUp_Sound(){
//     ledcWriteTone(SPEAKER_CH,1000);
//     vTaskDelay(200);
//     ledcWriteTone(SPEAKER_CH,1200);
//     vTaskDelay(200);
//     ledcWriteTone(SPEAKER_CH,1400);
//     vTaskDelay(600);
//     ledcWriteTone(SPEAKER_CH,0);
// }
void Shutdown_Sound(){
    ledcWriteTone(SPEAKER_CH,1400);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1200);
    vTaskDelay(200);
    ledcWriteTone(SPEAKER_CH,1000);
    vTaskDelay(400);
    ledcWriteTone(SPEAKER_CH,0);
}