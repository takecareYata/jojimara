#include "buzzer.h"

void buzzer_time_init(Buzzer_t *buzzer, TIM_TypeDef *htim, uint32_t channel, uint32_t timer_clock){
    buzzer->htim = htim;
    buzzer->channel = channel;
    buzzer->timer_clock = timer_clock;
    buzzer_insert_hz(buzzer, 0);
}

void buzzer_insert_hz(Buzzer_t *buzzer, uint32_t freq_hz){

    if (freq_hz == 0)
    {
        Macro_Clear_Bit(buzzer->htim->CCER, 8);
        return;
    }

    uint32_t arr = (buzzer->timer_clock / freq_hz) - 1;
    uint32_t ccr = (arr + 1) / 2;

    tim_set_auto_reload(buzzer, arr);
    tim_set_compare(buzzer->htim,buzzer->channel, ccr);

    // EGR: UG (Bit 0) 세트 -> 설정값 즉시 갱신
    Macro_Set_Bit(buzzer->htim->EGR, 0);

    // CCER: CC3E (Bit 8) 세트하여 PWM 출력 켜기
    Macro_Set_Bit(buzzer->htim->CCER, 8);
}

void tim_set_auto_reload(Buzzer_t *buzzer, uint32_t arr){
    buzzer->htim->ARR = arr;
}

void tim_set_compare(TIM_TypeDef *htim, uint32_t channel, uint32_t ccr) {
    switch (channel) {
        case 1: htim->CCR1 = ccr; break;
        case 2: htim->CCR2 = ccr; break;
        case 3: htim->CCR3 = ccr; break;
        case 4: htim->CCR4 = ccr; break;
        default: break;
    }
}

