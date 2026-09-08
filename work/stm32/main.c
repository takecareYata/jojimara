#include "device_driver.h"
#include "motor_app.h"
#include "buzzer_app.h"
#include <stdio.h>
static void Sys_Init(int baud) 
{
	SCB->CPACR |= (0x3 << 10*2)|(0x3 << 11*2); 
    Clock_Init();
	Uart2_Init(baud);
    //Uart1_Init(baud);
    app_motor_init();
    app_buzzer_init();
    TIM1_1ms_Interrupt_Init();
    window_init();
    led_init();
	setvbuf(stdout, NULL, _IONBF, 0);
}


char cmd_buf[64];
char ack_cmd_buf[64];

volatile int Uart_Data_In;

void Main(){
    Sys_Init(115200);
    Uart2_RX_Interrupt_Enable(1);
    
    while(1){
        if(Uart_Data_In){
            app_process_command(UART_ParseCommand(cmd_buf), ack_cmd_buf, sizeof(ack_cmd_buf));
            UART2_Ack_SendString(ack_cmd_buf);
            Uart_Data_In = 0;
        }
    }
}

