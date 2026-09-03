#include "device_driver.h"


void app_process_command(CommandType cmd){
    bool is_success = true;
    
    switch(cmd){
        case CMD_DROWSY_WARN: start_buzzer(); break;
        case CMD_DROWSY_OK: stop_buzzer(); break;
        case CMD_VENT_ON: air_con_motor_start(); break;
        case CMD_WARN_CENTER: set_led_warning(CENTER); break;
        case CMD_WARN_RIGHT: set_led_warning(RIGHT); break;
        case CMD_WARN_LEFT: set_led_warning(LEFT); break;
        case CMD_WIN_CLOSE:
            window_close();
            air_purification_motor_start();
            break;
        case CMD_WIN_OPEN:
            window_open();
            air_purification_motor_stop( );
            break;
        default:
            is_success = false;
            break;
    }

    // 작업 수행후 응답
    if(is_success){
        UART1_SendString("ACKOK\r\n");
    }
    else if(cmd != CMD_NONE){
        UART1_SendString("NACK\r\n");
    }
    
}