#include "device_driver.h"

void Buzzer_Init()
{
    // 1. GPIOB, TIM3 
    Macro_Set_Bit(RCC->AHB1ENR, 1); 
    Macro_Set_Bit(RCC->APB1ENR, 1); 

    // 2. PB0 핀 (AF2: TIM3_CH3) 설정
    // MODER0: 2비트 폭, 0b10  쓰기
    Macro_Write_Block(GPIOB->MODER, 0x3, 0x2, 0);

    // AFR[0] (AFRL0): 4비트 폭, AF2(0x2) 쓰기
    Macro_Write_Block(GPIOB->AFR[0], 0xF, 0x2, 0);

    // 3. TIM3 시간축 설정 (SYSCLK 96MHz 기준 1MHz 카운터)
    TIM3->PSC = 96 - 1;
    TIM3->ARR = 250- 1; // 기본 4kHz
    TIM3->CCR3 = 125;    // 50% 듀티비

    // TIM3 Channel 3 PWM Mode 1 설정 & Preload 활성화

    Macro_Write_Block(TIM3->CCMR2, 0x7, 0x6, 4);
    Macro_Set_Bit(TIM3->CCMR2, 3);

    // CR1: ARPE (Bit 7) -> ARR Preload Enable
    Macro_Set_Bit(TIM3->CR1, 7);

    // CCER: CC3E (Bit 8) -> PWM 출력 비활성화 상태로 대기
    Macro_Clear_Bit(TIM3->CCER, 8);

    // 5. 카운터 시작 (CR1: CEN, Bit 0)
    Macro_Set_Bit(TIM3->CR1, 0);
}

void Buzzer_Play(uint32_t freq_hz)
{
    if (freq_hz == 0)
    {
        // CCER: CC3E (Bit 8) 클리어하여 소리 끄기
        Macro_Clear_Bit(TIM3->CCER, 8);
        return;
    }

    uint32_t arr = (1000000 / freq_hz) - 1;
    uint32_t ccr = (arr + 1) / 2;

    TIM3->ARR = arr;
    TIM3->CCR3 = ccr;

    // EGR: UG (Bit 0) 세트 -> 설정값 즉시 갱신
    Macro_Set_Bit(TIM3->EGR, 0);

    // CCER: CC3E (Bit 8) 세트하여 PWM 출력 켜기
    Macro_Set_Bit(TIM3->CCER, 8);
}

volatile uint32_t warning_tick = 0;

// 소리만 잠시 끄는 함수 (PWM OFF)
void Buzzer_Mute()
{
    Macro_Clear_Bit(TIM3->CCER, 8);
}

// 경고음 완전히 시작 (명령 수신 시 1회 호출)
void start_buzzer()
{
    warning_tick = 0;
    Buzzer_Play(3000);
    Macro_Set_Bit(TIM1->CR1, 0);     // TIM1 카운터
}

// 경고음 완전히 정지 (명령 수신 시 1회 호출)
void stop_buzzer()
{
    Macro_Clear_Bit(TIM1->CR1, 0);   // TIM1 카운터 정지 (인터럽트 중단)
    Buzzer_Mute();                   // PWM 출력 정지
}