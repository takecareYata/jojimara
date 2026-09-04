#include "device_driver.h"

void led_init(){
    Macro_Set_Bit(RCC->AHB1ENR, 2);
    
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, 10);
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, 12);
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, 16);

    Macro_Clear_Bit(GPIOC->OTYPER, 5);
    Macro_Clear_Bit(GPIOC->OTYPER, 6);
    Macro_Clear_Bit(GPIOC->OTYPER, 8);

    Macro_Clear_Bit(GPIOC->ODR, 5);
    Macro_Clear_Bit(GPIOC->ODR, 6);
    Macro_Clear_Bit(GPIOC->ODR, 8);
}

volatile LED_STATE target_led = NONE;
static volatile int led_count = 0;

void led_center_off(){
    Macro_Clear_Bit(target_led, 0);
    Macro_Clear_Bit(GPIOC->ODR, 8);
}

void led_right_off(){
    Macro_Clear_Bit(target_led, 1);
    Macro_Clear_Bit(GPIOC->ODR, 5);
}

void led_left_off(){
    Macro_Clear_Bit(target_led, 2);
    Macro_Clear_Bit(GPIOC->ODR, 6);
}

void set_led_warning(LED_STATE led_state){
    target_led |= led_state;
}

void led_interrupt(){

    if (target_led == NONE) {
        return;
    }

    led_count++;
    if (led_count >= 250) { // 250ms 주기
        led_count = 0;

        if (target_led & CENTER) Macro_Invert_Bit(GPIOC->ODR, 8);
        if (target_led & LEFT)   Macro_Invert_Bit(GPIOC->ODR, 6);
        if (target_led & RIGHT)  Macro_Invert_Bit(GPIOC->ODR, 5);
    }
}