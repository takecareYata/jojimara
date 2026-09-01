#include "device_driver.h"


void app_process_command(CommandType cmd){
    bool is_success = false;
    printf("3\n");
    switch(cmd){
        case CMD_DROWSY_WARN:
            //작동 함수 작성
            printf("CMD_DROWSY_WARN receive\n");
            is_success = true;
            break;
        case CMD_DROWSY_OK:
            break;
        case CMD_VENT_ON:
            break;
        case CMD_VENT_OFF:
            break;
        case CMD_WIN_CLOSE:
            break;
        case CMD_WIN_OPEN:
            break;
        case CMD_SIDE_WARN:
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