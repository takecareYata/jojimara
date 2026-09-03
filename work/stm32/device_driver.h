#include "stm32f4xx.h"
#include "option.h"
#include "macro.h"
#include "malloc.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
// Uart.c
extern void Uart2_Init(int baud);

extern void Uart1_Init(int baud);
extern void UART1_GetCommand(char *out_buf);
extern void UART1_SendChar(char c);
extern void UART1_SendString(const char *str);
extern void Uart1_RX_Interrupt_Enable(int en);

//protocol.c

typedef enum {
    CMD_NONE,
    CMD_DROWSY_WARN,
    CMD_DROWSY_OK,
    CMD_VENT_ON,
    CMD_VENT_OFF,
    CMD_WIN_CLOSE,
    CMD_WIN_OPEN,
    CMD_SIDE_WARN
} CommandType;

typedef struct {
    const char *cmd_compare_str;
    int length;
    CommandType cmd_type;
} CommandMap;

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))
extern CommandType UART_ParseCommand(char *cmd);


//app_process.c
extern void app_process_command(CommandType cmd);

//eception.c
extern void USART1_IRQHandler();
extern void TIM1_UP_TIM10_IRQHandler();
extern void _Invalid_ISR();


//clock.c
extern void Clock_Init();

//motor.c
extern void motors_init();
extern void air_con_motor_start();
extern void air_con_motor_stop();
extern void air_purification_motor_start();
extern void air_purification_motor_stop();

//buzzer.c
extern void Buzzer_Init();
extern void Buzzer_Play(uint32_t freq_hz);
extern void Buzzer_Mute();
extern void start_buzzer();
extern void stop_buzzer();


//timer.c
extern void TIM1_1ms_Interrupt_Init();