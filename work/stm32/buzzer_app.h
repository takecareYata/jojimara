#include "buzzer.h"

#ifndef BUZZER_APP_H
#define BUZZER_APP_H

void app_buzzer_init();

// 소리만 잠시 끄는 함수 (PWM OFF)
void app_buzzer_mute();

// 경고음 완전히 시작 (명령 수신 시 1회 호출)
void app_start_buzzer();

// 경고음 완전히 정지 (명령 수신 시 1회 호출)
void app_stop_buzzer();
void app_buzzer_interrupt();

#endif
