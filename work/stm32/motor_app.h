#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "motor.h"
#include "motor_hw.h"

// 시스템 동작 API
void app_motor_init();           // 모터 핀 할당 및 하드웨어 초기화 실행
void app_aircon_start();         // 에어컨 가동 (10초 카운트 다운 시작 10초 후 정지)
void app_airpurifier_start();    // 공기청정 가동
void app_airpurifier_stop();     // 공기청정 정지

// 타이머 인터럽트(1ms 주기 등)에서 호출할 함수
void app_motor_1ms_ISR();

#endif /* MOTOR_APP_H */