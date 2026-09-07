#include "device_driver.h"

#ifndef BUZZER_H
#define BUZZER_H

typedef struct {
    TIM_TypeDef *htim;
    uint32_t channel;
    uint32_t timer_clock; // 타이머 카운터 클록 (예: 1MHz 등)
} Buzzer_t;

void buzzer_time_init(Buzzer_t *buzzer, TIM_TypeDef *htim, uint32_t channel, uint32_t timer_clock);
void buzzer_insert_hz(Buzzer_t *buzzer, uint32_t freq_hz);
void tim_set_auto_reload(Buzzer_t *buzzer, uint32_t arr);
void tim_set_compare(TIM_TypeDef *htim, uint32_t channel, uint32_t ccr);

#endif