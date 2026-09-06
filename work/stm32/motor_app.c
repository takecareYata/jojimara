#include "motor_app.h"

static DCMotor_t motor_ac;
static DCMotor_t motor_purifier;

static volatile int ac_running_tick = 0;
static volatile int is_ac_running = 0;


void App_Motor_Init(){
    Motor_HW_Init();

    // 핀과 레지스터 바인딩
    DCMotor_Init(&motor_ac, GPIOA, 6, 7, &(TIM2->CCR3));
    DCMotor_Init(&motor_purifier, GPIOB, 8, 9, &(TIM2->CCR2));
}

void App_AirCon_Start(){
    DCMotor_Start(&motor_ac, 1000);
    ac_running_tick = 0;
    is_ac_running = 1;
}

void App_AirCon_Stop(){
    DCMotor_Stop(&motor_ac);
}

void App_AirPurifier_Start(){
    DCMotor_Start(&motor_purifier, 1000);
}

void App_AirPurifier_Stop(){
    DCMotor_Stop(&motor_purifier);
}    

// 타이머 인터럽트(1ms 주기 등)에서 호출할 함수
void App_Motor_1ms_ISR(){
    if (is_ac_running) {
        ac_running_tick++;
        if (ac_running_tick >= 10000) {
            DCMotor_Stop(&motor_ac);
            is_ac_running = 0;
        }
    }
}



