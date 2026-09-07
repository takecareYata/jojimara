#include "motor_app.h"

static DCMotor_t motor_ac;
static DCMotor_t motor_purifier;

static volatile int ac_running_tick = 0;
static volatile int is_ac_running = 0;


void app_motor_init(){
    motor_hw_init();

    // 핀과 레지스터 바인딩
    dcmotor_init(&motor_ac, GPIOA, 6, 7, &(TIM2->CCR3));
    dcmotor_init(&motor_purifier, GPIOB, 8, 9, &(TIM2->CCR2));
}

void app_aircon_start(){
    dcmotor_start(&motor_ac, 1000);
    ac_running_tick = 0;
    is_ac_running = 1;
}

void app_airpurifier_start(){
    dcmotor_start(&motor_purifier, 1000);
}

void app_airpurifier_stop(){
    dcmotor_stop(&motor_purifier);
}    

// 타이머 인터럽트(1ms 주기 등)에서 호출할 함수
void app_motor_1ms_ISR(){
    if (is_ac_running) {
        ac_running_tick++;
        if (ac_running_tick >= 10000) {
            dcmotor_stop(&motor_ac);
            is_ac_running = 0;
        }
    }
}



