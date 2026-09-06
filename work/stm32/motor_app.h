#include "motor.h"
#include "motor_hw.h"

// 시스템 동작 API
void App_Motor_Init(void);           // 모터 핀 할당 및 하드웨어 초기화 실행
void App_AirCon_Start(void);         // 에어컨 가동 (10초 카운트 다운 시작)
void App_AirCon_Stop(void);          // 에어컨 즉시 정지
void App_AirPurifier_Start(void);    // 공기청정 가동
void App_AirPurifier_Stop(void);     // 공기청정 정지

// 타이머 인터럽트(1ms 주기 등)에서 호출할 함수
void App_Motor_1ms_ISR(void);