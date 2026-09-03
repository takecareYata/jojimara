#include "device_driver.h"

void window_init(){
    //GPIOB 및 TIM4 클럭 인가
    Macro_Set_Bit(RCC->AHB1ENR, 1);     // GPIOB Clock Enable
    Macro_Set_Bit(RCC->APB1ENR, 2);     // TIM4 Clock Enable

    //PB6 핀을 Alternate Function 모드로 설정
    Macro_Write_Block(GPIOB->MODER, 0x3, 2, 12);     // PB6 -> Mode 2 (AF)
    Macro_Write_Block(GPIOB->AFR[0], 0xF, 2, 24);    // PB6 -> AF2 (TIM4_CH1, 4비트 폭)

    TIM4->PSC = 95;
    TIM4->ARR = 19999;
    TIM4->CCR1 = 500;                              

    // PWM 모드 1 설정 및 채널 1 출력 켜기
    // OC1M = 110 (PWM Mode 1, 비트 4~6), OC1PE = 1 (Preload Enable, 비트 3)
    Macro_Write_Block(TIM4->CCMR1, 0x7, 0x6, 4);
    Macro_Set_Bit(TIM4->CCMR1, 3);

    Macro_Set_Bit(TIM4->CCER, 0);                        // CC1E = 1 (Channel 1 Output Enable)

    Macro_Set_Bit(TIM4->CR1, 0);        
}

void window_open(){
    TIM4->CCR1 = 500;   
}

void window_close(){
    TIM4->CCR1 = 2500;   
}