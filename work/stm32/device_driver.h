#include "stm32f4xx.h"
#include "option.h"
#include "macro.h"
#include "malloc.h"

// Uart.c

extern void Uart2_Init(int baud);
extern void Uart1_Init(int baud);
extern void Uart1_Send_Byte(char data);
extern char Uart1_Get_Char(void);
extern char Uart1_Get_Pressed(void);
extern void Uart2_RX_Interrupt_Enable(int en);

// Led.c

extern void LED_Init(void);
extern void LED_On(void);
extern void LED_Off(void);
extern void LED_Display(int on);
extern void LED_SetFloor(uint8_t floor, uint8_t state);
extern void LED_UpdateFromSlots(void);

// Clock.c

extern void Clock_Init(void);

// Timer.c
extern void TIM3_Delay(int time_ms);

