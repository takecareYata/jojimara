#include "device_driver.h"
#include <stdio.h>
void _Invalid_ISR()
{
	unsigned int r = Macro_Extract_Area(SCB->ICSR, 0x1ff, 0);
	printf("\nInvalid_Exception: %d!\n", r);
	printf("Invalid_ISR: %d!\n", r - 16);
	for(;;);
}

extern char cmd_buf[64];
extern volatile int Uart_Data_In;
extern volatile unsigned char Uart_Data;

void USART1_IRQHandler()
{
    static int rx_idx = 0;

    // 1. RXNE (수신 데이터 있음) 플래그 확인
    if (USART1->SR & (1 << 5))
    {
        // DR 레지스터를 읽어야 RXNE 인터럽트 플래그가 자동으로 클리어됨
        char ch = (char)(USART1->DR);

        // 개행 문자 수신 시 한 문장 완성 처리
        if (ch == '\n' || ch == '\r')
        {
            if (rx_idx > 0)
            {
                cmd_buf[rx_idx] = '\0';
                Uart_Data_In = 1;   
                rx_idx = 0;
            }
        }
        else
        {
            if (rx_idx < sizeof(cmd_buf) - 1)
            {
                cmd_buf[rx_idx++] = ch;
            }
        }
    }

    NVIC_ClearPendingIRQ(37);
}

void TIM1_UP_TIM10_IRQHandler()
{
    // TIM1 인터럽트 플래그 확인 및 클리어
    if (Macro_Check_Bit_Set(TIM1->SR, 0))
    {
        Macro_Clear_Bit(TIM1->SR, 0); // 플래그를 지워야 무한 루프에 안 빠짐

        air_con_running_interrupt();
        buzzer_interrupt();
        led_interrupt();
    }
}
