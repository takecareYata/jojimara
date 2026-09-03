#include "device_driver.h"
#include <stdio.h>
// 모터 통합 초기화
// 에어컨:   PA6 (IN1), PA7 (IN2), PB10 (ENA -> TIM2_CH3)
// 공기청정: PB8 (IN3), PB9 (IN4), PA1 (ENB -> TIM2_CH2)

void motors_init()
{
    // 1. 클럭 활성화 (GPIOA, GPIOB, TIM2)
    Macro_Set_Bit(RCC->AHB1ENR, 0); // GPIOA
    Macro_Set_Bit(RCC->AHB1ENR, 1); // GPIOB
    Macro_Set_Bit(RCC->APB1ENR, 0); // TIM2

    // 2. 방향 제어 핀(GPIO Output) 설정
    Macro_Write_Block(GPIOA->MODER, 0x3, 1, 12); // PA6 (IN1)
    Macro_Write_Block(GPIOA->MODER, 0x3, 1, 14); // PA7 (IN2)
    Macro_Write_Block(GPIOB->MODER, 0x3, 1, 16); // PB8 (IN3)
    Macro_Write_Block(GPIOB->MODER, 0x3, 1, 18); // PB9 (IN4)

    // 3. PWM 핀(AF1) 설정
    // PB10 -> Mode 2 (AF), AF1 (TIM2_CH3)
    Macro_Write_Block(GPIOB->MODER, 0x3, 2, 20);
    Macro_Write_Block(GPIOB->AFR[1], 0xF, 1, 8);

    // PA1 -> Mode 2 (AF), 1번 핀 (비트 2)
    // PA1 -> AF1 (TIM2_CH2), AFR[0]의 1번 핀 (비트 4)
    Macro_Write_Block(GPIOA->MODER, 0x3, 2, 2);
    Macro_Write_Block(GPIOA->AFR[0], 0xF, 1, 4);

    // 4. 타이머 공통 클럭 주기 설정 (딱 1번만 설정)
    TIM2->PSC = 96 - 1;   // 96MHz / 96 = 1MHz
    TIM2->ARR = 999;      // 1MHz / 1000 = 1kHz 주기

    // 5. CH3(에어컨), CH2(공기청정) PWM Mode 1 설정
    // CH3 (TIM2->CCMR2: OC3M = 0b110 [비트 6:4], OC3PE = 1 [비트 3])
    Macro_Write_Block(TIM2->CCMR2, 0x7, 0x6, 4);
    Macro_Set_Bit(TIM2->CCMR2, 3);

    // CH2 (TIM2->CCMR1: OC2M = 0b110 [비트 14:12], OC2PE = 1 [비트 11])
    Macro_Write_Block(TIM2->CCMR1, 0x7, 0x6, 12);
    Macro_Set_Bit(TIM2->CCMR1, 11);

    // 6. CH3, CH4 출력 활성화 및 타이머 시작
    Macro_Set_Bit(TIM2->CCER, 8);  // CC3E = 1 (에어컨 ENA)
    Macro_Set_Bit(TIM2->CCER, 4);  // CC2E = 1 (비트 4 : 공기청정 ENB - PA1)
    Macro_Set_Bit(TIM2->CR1, 0);   // CEN = 1 (타이머 카운터 가동)

    // 7. 초기 상태: 모터 듀티비 0 및 방향 LOW (정지)
    TIM2->CCR3 = 0;
    TIM2->CCR2 = 0;
    Macro_Clear_Bit(GPIOA->ODR, 6);
    Macro_Clear_Bit(GPIOA->ODR, 7);
    Macro_Clear_Bit(GPIOB->ODR, 8);
    Macro_Clear_Bit(GPIOB->ODR, 9);
}

// [에어컨] 가동 및 정지
void air_con_motor_start()
{
    printf("6666\n");
    Macro_Clear_Bit(GPIOA->ODR, 6);   // IN1 = 0
    Macro_Set_Bit(GPIOA->ODR, 7); // IN2 = 1
    TIM2->CCR3 = 1000;               
}

void air_con_motor_stop()
{
    Macro_Clear_Bit(GPIOA->ODR, 6);
    Macro_Clear_Bit(GPIOA->ODR, 7);
    TIM2->CCR3 = 0;
}

// [공기청정] 가동 및 정지
void air_purification_motor_start()
{
    printf("5555\n");
    Macro_Clear_Bit(GPIOB->ODR, 8);   // IN3 = 1
    Macro_Set_Bit(GPIOB->ODR, 9); // IN4 = 0
    TIM2->CCR2 = 1000;               
}

void air_purification_motor_stop()
{
    Macro_Clear_Bit(GPIOB->ODR, 8);
    Macro_Clear_Bit(GPIOB->ODR, 9);
    TIM2->CCR2 = 0;
}