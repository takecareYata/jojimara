#include "device_driver.h"


//에어컨 모터
//PA6 (IN1), PA7 (IN2), PB10(ENA)
// pwm -> TIM2 CH3( 중간 세기로 설정)


void air_con_motor_init(){
   Macro_Set_Bit(RCC->AHB1ENR, 0);
   Macro_Set_Bit(RCC->AHB1ENR, 1);

   Macro_Set_Bit(RCC->APB1ENR, 0);

   Macro_Write_Block(GPIOA->MODER, 0x3, 1, 12);
   Macro_Write_Block(GPIOA->MODER, 0x3, 1, 14);

   Macro_Write_Block(GPIOB->MODER, 0x3, 2, 20);     // PB10 -> Mode 2 (AF)
   Macro_Write_Block(GPIOB->AFR[1], 0xF, 1, 8);    // PB10 -> AF1 (TIM2_CH3, 4비트 폭)

   //타이머 주파수 설정 (시스템 클럭 96MHz 기준 -> 1kHz PWM 생성)
    TIM2->PSC = 96 - 1;   // 96MHz / 96 = 1MHz 카운팅 속도
    TIM2->ARR = 999;      // 1MHz / 1000 = 1kHz PWM 주기

    //TIM2 채널 3 PWM Mode 1 설정
    Macro_Write_Block(TIM2->CCMR2, 0x7, 0x6, 4); // OC3M = 0b110 (PWM Mode 1)
    Macro_Set_Bit(TIM2->CCMR2, 3);                // OC3PE = 1 (Preload 활성화)

    //TIM2 채널 3 출력 활성화 및 타이머 시작
    Macro_Set_Bit(TIM2->CCER, 8); // CC3E = 1 (CH3 활성화)
    Macro_Set_Bit(TIM2->CR1, 0);  

    TIM2->CCR3 = 0;             

    Macro_Clear_Bit(GPIOA->ODR, 6);   
    Macro_Clear_Bit(GPIOA->ODR, 7); 
}

// 임시 속도 제어: speed (0 ~ 1000)
void air_con_set_speed(uint32_t speed)
{
    if (speed > 1000) speed = 1000;
    TIM2->CCR3 = speed;
}

//에어컨 가동
void air_con_start()
{
    Macro_Set_Bit(GPIOA->ODR, 6); // IN1 = 1
    Macro_Clear_Bit(GPIOA->ODR, 7); // IN2 = 0
    TIM2->CCR3 = 1000;
}

// 정지
void air_con_stop()
{
    Macro_Clear_Bit(GPIOA->ODR, 6); // IN1 = 0
    Macro_Clear_Bit(GPIOA->ODR, 7); // IN2 = 0
    TIM2->CCR3 = 0;
}