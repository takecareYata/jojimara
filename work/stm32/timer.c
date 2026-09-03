#include "device_driver.h"


void TIM1_1ms_Interrupt_Init()
{
    Macro_Set_Bit(RCC->APB2ENR, 0);

    // 2. 1ms 주기 설정 (SYSCLK 96MHz 기준)
    // 96MHz / (95 + 1) = 1MHz 카운터 클럭
    TIM1->PSC = 96 - 1;
    // 1MHz / 1000 = 1kHz (1ms)
    TIM1->ARR = 1000 - 1;

    // 3. 인터럽트 설정
    Macro_Set_Bit(TIM1->DIER, 0);   // Update Interrupt Enable (UIE, Bit 0)
    Macro_Set_Bit(TIM1->EGR, 0);    // 설정값 반영을 위한 UG 발생
    Macro_Clear_Bit(TIM1->SR, 0);   // 발생한 플래그 초기화

    // 4. NVIC 인터럽트 활성화
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2); // 통신 인터럽트보다 우선순위를 낮게 설정 권장

    Macro_Set_Bit(TIM1->CR1, 0);     // TIM1 카운터
}