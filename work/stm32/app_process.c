#include "device_driver.h"


void app_process_command(CommandType cmd){
    bool is_success = false;
    printf("3\n");
    switch(cmd){
        case CMD_DROWSY_WARN:
            //졸음 경고 부저
            start_buzzer();
            is_success = true;
            break;
        case CMD_DROWSY_OK:
            //경고부저 off
            stop_buzzer();
            is_success = true;
            break;
        case CMD_VENT_ON:
            air_con_motor_start();
            is_success = true;
            break;
        case CMD_WIN_CLOSE:
            //서보모터 창문닫기
            window_close();
            air_purification_motor_start();
            is_success = true;
            break;
        case CMD_WIN_OPEN:
            window_open();
            air_purification_motor_stop( );
            is_success = true;
            break;
        case CMD_WARN_CENTER:
            set_led_warning(CENTER);
            is_success = true;
            break;
        case CMD_WARN_RIGHT:
            set_led_warning(RIGHT);
            is_success = true;
            break;
        case CMD_WARN_LEFT:
            set_led_warning(LEFT);
            is_success = true;
            break;

        default:
            printf("ERROR\n");
            break;
    }

    // 작업 수행후 응답
    if(is_success){
        printf("4\n");
        UART1_SendString("ACKOK\r\n");
    }
    else if(cmd != CMD_NONE){
        UART1_SendString("NACK\r\n");
    }
    
}