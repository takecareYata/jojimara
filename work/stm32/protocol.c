#include "device_driver.h"


const CommandMap cmd_table[] = {
    {"DROWSY_WARN", 11, CMD_DROWSY_WARN},
    {"DROWSY_OK",    8, CMD_DROWSY_OK},
    {"VENT_ON",      7, CMD_VENT_ON},
    {"VENT_OFF",     8, CMD_VENT_OFF},
    {"WIN_CLOSE",    9, CMD_WIN_CLOSE},
    {"WIN_OPEN",     8, CMD_WIN_OPEN},
    {"WARN_CENTER",    11, CMD_WARN_CENTER},
    {"WARN_RIGHT",    10, CMD_WARN_RIGHT},
    {"WARN_LEFT",    9, CMD_WARN_LEFT},
    {"CENTOR_OK",    9, CMD_CENTOR_OK},
    {"RIGHT_OK",    8, CMD_RIGHT_OK},
    {"LEFT_OK",    7, CMD_LEFT_OK},
    {"NONE",         4, CMD_NONE}
};

CommandType UART_ParseCommand(char *cmd) {
    for (int i = 0; i < CMD_TABLE_SIZE; i++) {
        if (strncmp(cmd, cmd_table[i].cmd_compare_str, cmd_table[i].length) == 0) {
            return cmd_table[i].cmd_type;
        }
    }
    
    return CMD_NONE;
}
    