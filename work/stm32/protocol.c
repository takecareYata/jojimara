#include "device_driver.h"


const CommandMap cmd_table[] = {
    {"DROWSY_WARN", 11, CMD_DROWSY_WARN},
    {"DROWSY_OK",    8, CMD_DROWSY_OK},
    {"VENT_ON",      7, CMD_VENT_ON},
    {"VENT_OFF",     8, CMD_VENT_OFF},
    {"WIN_CLOSE",    9, CMD_WIN_CLOSE},
    {"WIN_OPEN",     8, CMD_WIN_OPEN},
    {"SIDE_WARN",    9, CMD_SIDE_WARN},
    {"NONE",         4, CMD_NONE}
};

CommandType UART_ParseCommand(char *cmd) {
    printf("1\n");
    for (int i = 0; i < CMD_TABLE_SIZE; i++) {
        if (strncmp(cmd, cmd_table[i].cmd_compare_str, cmd_table[i].length) == 0) {
            return cmd_table[i].cmd_type;
        }
    }
    
    return CMD_NONE;
}
    