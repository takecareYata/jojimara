#include "device_driver.h"


#define UART_RX_BUFFER_SIZE 64
volatile bool s_cmd_ready = false;
static char s_cmd_buf[UART_RX_BUFFER_SIZE];

#if 1
void Uart2_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA2,3
  Macro_Set_Bit(RCC->APB1ENR, 17);                   // USART2 ON
  Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 4);     // PA2,3 => ALT
  Macro_Write_Block(GPIOA->AFR[0], 0xff, 0x77, 8);  // PA2,3 => AF07
  Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 4);     // PA2,3 => Pull-Up  

  div = PCLK1/(16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16. + 0.5);
  mant += frac >> 4;
  frac &= 0xf;

  USART2->BRR = (mant<<4)|(frac<<0);
  USART2->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  USART2->CR2 = 0<<12;
  USART2->CR3 = 0;
}

void UART2_GetCommand(char *out_buf) {
    if (out_buf != NULL) {
        strcpy(out_buf, s_cmd_buf);
    }
    s_cmd_ready = false;
}

void UART2_SendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

void UART2_SendString(const char *str) {
    while (*str) {
        UART2_SendChar(*str++);
    }
}

void UART2_Ack_SendString(char *ack_buf) {
    if (ack_buf != NULL) {
        UART2_SendString(ack_buf);
    }
}

void Uart2_RX_Interrupt_Enable(int en)
{
    if (en)
    {
        Macro_Set_Bit(USART2->CR1, 5); // RXNEIE 비트 활성화
        NVIC_ClearPendingIRQ(38);      // USART2_IRQn (38) 팬딩 클리어
        NVIC_EnableIRQ(38);            // USART2_IRQn (38) 인터럽트 허용
    }
    else
    {
        Macro_Clear_Bit(USART2->CR1, 5); // RXNEIE 비트 비활성화
        NVIC_DisableIRQ(38);             // USART2_IRQn (38) 인터럽트 금지
    }
}

#else

void Uart1_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA9,10
  Macro_Set_Bit(RCC->APB2ENR, 4);                   // USART1 ON
  Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 18);    // PA9,10 => ALT
  Macro_Write_Block(GPIOA->AFR[1], 0xff, 0x77, 4);  // PA9,10 => AF07
  Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 18);    // PA9,10 => Pull-Up

  div = PCLK2 / (16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16 + 0.5);
  mant += frac >> 4;
  frac &= 0xf;
  USART1->BRR = (mant<<4)|(frac<<0);

  USART1->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  USART1->CR2 = 0 << 12;
  USART1->CR3 = 0;
}



void UART1_GetCommand(char *out_buf) {
    if (out_buf != NULL) {
        strcpy(out_buf, s_cmd_buf);
    }
    s_cmd_ready = false;
}

void UART1_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
}

void UART1_SendString(const char *str) {
    while (*str) {
        UART1_SendChar(*str++);
    }
}

void UART1_Ack_SendString(bool is_success){
    if(is_success){
      // 명확하게 8글자, 9글자로 늘려서 테스트
      UART1_SendString("ACKOKK\r\n");   // 8바이트
    }
    else {
        UART1_SendString("NACK\r\n");
    }
}

void Uart1_RX_Interrupt_Enable(int en)
{
  if(en)
  {
    Macro_Set_Bit(USART1->CR1, 5);
    NVIC_ClearPendingIRQ(37);
    NVIC_EnableIRQ(37);
  }
  else
  {
    Macro_Clear_Bit(USART1->CR1, 5);
    NVIC_DisableIRQ(37);
  }
}
  #endif