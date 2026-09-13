#include "device_driver.h"

#define LED_RIGHT_PIN            5
#define LED_LEFT_PIN             6
#define LED_CENTER_PIN           8
#define LED_TOGGLE_MS  250

volatile LED_STATE target_led = NONE;
static volatile int led_count = 0;

void total_led_init(){
    Macro_Set_Bit(RCC->AHB1ENR, 2);
    
    led_init(LED_RIGHT_PIN);
    led_init(LED_LEFT_PIN);
    led_init(LED_CENTER_PIN);
}

void led_init(int pin_num){
    Macro_Write_Block(GPIOC->MODER, 0x3, 1, pin_num * 2);
    Macro_Clear_Bit(GPIOC->OTYPER, pin_num);
    Macro_Clear_Bit(GPIOC->ODR, pin_num);
}

void led_off(int dir, int pin_num){
    target_led &= ~dir;
    Macro_Clear_Bit(GPIOC->ODR, pin_num);
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