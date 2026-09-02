#include "device_driver.h"


void app_process_command(CommandType cmd){
    bool is_success = false;
    printf("3\n");
    switch(cmd){
        case CMD_DROWSY_WARN:
            //경고 부저
            is_success = true;
            break;
        case CMD_DROWSY_OK:
            //경고부저 off
            break;
        case CMD_VENT_ON:
            air_con_motor_start();
            is_success = true;
            break;
        case CMD_WIN_CLOSE:
            //서보모터 창문닫기
            air_purification_motor_start();
            is_success = true;
            break;
        case CMD_WIN_OPEN:
            air_purification_motor_stop( );
            is_success = true;
            break;
        case CMD_SIDE_WARN:
            // 경고 LED
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