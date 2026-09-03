#include "device_driver.h"

// 타이머 1번 딜레이 설정
#define TIM1_TIC        20                          // usec (1 tick = 20us)
#define TIM1_FREQ       (1000000. / TIM1_TIC)       // 50,000 Hz (50kHz)
#define TIM1_1ms_PLS    (TIM1_FREQ / 1000.)         // 1ms당 50 펄스
#define TIM1_MAX        (0xFFFFu)


void TIM1_Delay_Init()
{
    Macro_Set_Bit(RCC->APB2ENR, 0); 

    // Down count (DIR=1, Bit 4), One-pulse mode (OPM=1, Bit 3)
    TIM1->CR1 = (0x1 << 4) | (0x1 << 3);

    // 20usec tick (50kHz) 설정 
    TIM1->PSC = (int)(TIMXCLK / TIM1_FREQ + 0.5) - 1;

    Macro_Set_Bit(TIM1->EGR, 0);    // PSC 적용을 위한 UG 발생
    Macro_Clear_Bit(TIM1->SR, 0);   
}

/* 딜레이 함수 */
void TIM1_Delay(int time)
{
    if (time <= 0) return;

    unsigned int pls = (unsigned int)(time * TIM1_1ms_PLS);
    int n = pls / TIM1_MAX; 
    int m = pls % TIM1_MAX;

    for (int i = 0; i < n; i++) {
        TIM1->ARR = TIM1_MAX;
        Macro_Set_Bit(TIM1->EGR, 0);    
        Macro_Clear_Bit(TIM1->SR, 0);   
        Macro_Set_Bit(TIM1->CR1, 0);    
        while (!(TIM1->SR & 0x1));      
    }

    // 2. 자투리 시간(m) 대기 (m이 0보다 클 때만 실행해야 멈추지 않음)
    if (m > 0) {
        TIM1->ARR = m;
        Macro_Set_Bit(TIM1->EGR, 0);
        Macro_Clear_Bit(TIM1->SR, 0);
        Macro_Set_Bit(TIM1->CR1, 0);
        while (!(TIM1->SR & 0x1));
    }
}