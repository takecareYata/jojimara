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
static LED_STATE prev_led = NONE;
static volatile int led_count = 0;

void led_all_off() {
    Macro_Clear_Bit(GPIOC->ODR, 5);
    Macro_Clear_Bit(GPIOC->ODR, 6);
    Macro_Clear_Bit(GPIOC->ODR, 8);
}

void set_led_warning(LED_STATE led_state){
    target_led = led_state;
}

void led_interrupt(){
    // 모드가 바뀌면 기존 핀들을 끄고 카운터 리셋
    if (prev_led != target_led) {
        led_all_off();
        led_count = 0;
        prev_led = target_led;
    }

    if (target_led == NONE) {
        return;
    }

    led_count++;
    if (led_count >= 250) { // 250ms 주기
        led_count = 0;

        switch (target_led) {
            case CENTER: // PC8
                Macro_Invert_Bit(GPIOC->ODR, 8);
                break;
            case LEFT:   // PC6
                Macro_Invert_Bit(GPIOC->ODR, 6);
                break;
            case RIGHT:  // PC5
                Macro_Invert_Bit(GPIOC->ODR, 5);
                break;
            default:
                break;
        }
    }
}