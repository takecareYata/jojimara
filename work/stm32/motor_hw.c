#include "motor_hw.h"

void Motor_HW_Init() {
    // 1. 클럭 활성화
    Macro_Set_Bit(RCC->AHB1ENR, 0); // GPIOA
    Macro_Set_Bit(RCC->AHB1ENR, 1); // GPIOB
    Macro_Set_Bit(RCC->APB1ENR, 0); // TIM2

    // 2. GPIO Output 설정
    Macro_Write_Block(GPIOA->MODER, 0x3, 1, 12); // PA6
    Macro_Write_Block(GPIOA->MODER, 0x3, 1, 14); // PA7
    Macro_Write_Block(GPIOB->MODER, 0x3, 1, 16); // PB8
    Macro_Write_Block(GPIOB->MODER, 0x3, 1, 18); // PB9

    // 3. Alternate Function (PWM)
    Macro_Write_Block(GPIOB->MODER, 0x3, 2, 20); // PB10
    Macro_Write_Block(GPIOB->AFR[1], 0xF, 1, 8);
    Macro_Write_Block(GPIOA->MODER, 0x3, 2, 2);  // PA1
    Macro_Write_Block(GPIOA->AFR[0], 0xF, 1, 4);

    // 4. TIM2 베이스 및 채널 설정
    TIM2->PSC = 96 - 1;
    TIM2->ARR = 999;
    Macro_Write_Block(TIM2->CCMR2, 0x7, 0x6, 4);
    Macro_Set_Bit(TIM2->CCMR2, 3);
    Macro_Write_Block(TIM2->CCMR1, 0x7, 0x6, 12);
    Macro_Set_Bit(TIM2->CCMR1, 11);
    Macro_Set_Bit(TIM2->CCER, 8);
    Macro_Set_Bit(TIM2->CCER, 4);
    Macro_Set_Bit(TIM2->CR1, 0);
}