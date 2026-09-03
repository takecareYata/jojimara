#include "device_driver.h"

#include <stdio.h>
static void Sys_Init(int baud) 
{
	SCB->CPACR |= (0x3 << 10*2)|(0x3 << 11*2); 
    Clock_Init();
	Uart2_Init(baud);
    Uart1_Init(baud);
    motors_init();
    Buzzer_Init();
    TIM1_1ms_Interrupt_Init();
    window_init();
	setvbuf(stdout, NULL, _IONBF, 0);
}


char cmd_buf[64];
volatile int Uart_Data_In;

void Main(){
    Sys_Init(115200);
    Uart1_RX_Interrupt_Enable(1);
    printf("test_start\n");
    while(1){
        if(Uart_Data_In){
            printf("before[Jetson -> MCU]: %s\r\n", cmd_buf);
            app_process_command(UART_ParseCommand(cmd_buf));
            printf("2\n");
            Uart_Data_In = 0;
        }
    }
}

