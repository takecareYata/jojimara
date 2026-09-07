#include "buzzer_app.h"
#include "buzzer.h"

#define BZ_WARNING_HZ (3000)
#define BZ_TOGGLE_MS  (250)
#define BUZZER_CHANNEL        3
#define BUZZER_TIM_CLK_HZ     1000000U

static Buzzer_t buzzer;

volatile static unsigned int warning_tick = 0;
volatile static int warning_state = 0;
volatile static int is_bz_running_tick = 0;

void app_buzzer_init(){
    buzzer_hw_init();
    buzzer_time_init(&buzzer, TIM3, BUZZER_CHANNEL, BUZZER_TIM_CLK_HZ);
}

// 소리만 잠시 끄는 함수 (PWM OFF)
void app_buzzer_mute()
{
    buzzer_insert_hz(&buzzer, 0);
}

// 경고음 완전히 시작 (명령 수신 시 1회 호출)
void app_start_buzzer()
{
    warning_tick = 0;
    buzzer_insert_hz(&buzzer, BZ_WARNING_HZ);
    is_bz_running_tick = 1;
}

// 경고음 완전히 정지 (명령 수신 시 1회 호출)
void app_stop_buzzer()
{
    is_bz_running_tick = 0;
    app_buzzer_mute();                   // PWM 출력 정지
}

void app_buzzer_interrupt(){
    if(is_bz_running_tick){
        warning_tick++;

        if (warning_tick >= BZ_TOGGLE_MS) // 250ms 주기마다 On/Off 토글
        {
            warning_tick = 0;    
            warning_state ^= 1; 

            if (warning_state)
            {
                // 소리만 잠시 끔 (TIM1 카운터는 계속 돌아야 함)
                app_buzzer_mute();
            }
            else
            {
                // 소리 켬
                buzzer_insert_hz(&buzzer, BZ_WARNING_HZ); // 삐- 삐- 단속음  
            }
        }
    }
    
}