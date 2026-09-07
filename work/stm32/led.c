#include "device_driver.h"

#define LED_RIGHT_PIN            5
#define LED_LEFT_PIN             6
#define LED_CENTER_PIN           8
#define LED_TOGGLE_MS  250

void led_init(){
    Macro_Set_Bit(RCC->AHB1ENR, 2);
    
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, LED_RIGHT_PIN * 2);
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, LED_LEFT_PIN * 2);
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, LED_CENTER_PIN * 2);

    Macro_Clear_Bit(GPIOC->OTYPER, LED_RIGHT_PIN);
    Macro_Clear_Bit(GPIOC->OTYPER, LED_LEFT_PIN);
    Macro_Clear_Bit(GPIOC->OTYPER, LED_CENTER_PIN);

    Macro_Clear_Bit(GPIOC->ODR, LED_RIGHT_PIN);
    Macro_Clear_Bit(GPIOC->ODR, LED_LEFT_PIN);
    Macro_Clear_Bit(GPIOC->ODR, LED_CENTER_PIN);
}

volatile LED_STATE target_led = NONE;
static volatile int led_count = 0;

void led_center_off(){
    target_led &= ~CENTER;
    Macro_Clear_Bit(GPIOC->ODR, LED_CENTER_PIN);
}

void led_right_off(){
    target_led &= ~RIGHT;
    Macro_Clear_Bit(GPIOC->ODR, LED_RIGHT_PIN);
}

void led_left_off(){
    target_led &= ~LEFT;
    Macro_Clear_Bit(GPIOC->ODR, LED_LEFT_PIN);
}

void set_led_warning(LED_STATE led_state){
    target_led |= led_state;
}

void led_interrupt(){

    if (target_led == NONE) {
        return;
    }

    led_count++;
    if (led_count >= LED_TOGGLE_MS) { // 250ms 주기
        led_count = 0;

        if (target_led & CENTER) Macro_Invert_Bit(GPIOC->ODR, LED_CENTER_PIN);
        if (target_led & LEFT)   Macro_Invert_Bit(GPIOC->ODR, LED_LEFT_PIN);
        if (target_led & RIGHT)  Macro_Invert_Bit(GPIOC->ODR, LED_RIGHT_PIN );
    }
}