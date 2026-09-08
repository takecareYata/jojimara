#include "device_driver.h"
#include "motor_app.h"
#include "buzzer_app.h"

// 복합 동작(예: 윈도우+공기청정기)은 래퍼 함수로 묶어 등록
static void action_win_close() {
    window_close();
    app_airpurifier_start();
}

static void action_win_open() {
    window_open();
    app_airpurifier_stop();
}

static void action_warn_center() { set_led_warning(CENTER); }
static void action_warn_right()  { set_led_warning(RIGHT); }
static void action_warn_left()   { set_led_warning(LEFT); }

// 테이블 정의: 여기에 한 줄씩만 추가/수정하면 모든 로직이 자동 연동됨
static const CommandEntry cmd_table[] = {
    { CMD_DROWSY_WARN, "DROWSY_WARN", 11, app_start_buzzer },
    { CMD_DROWSY_OK,   "DROWSY_OK",    9, app_stop_buzzer },
    { CMD_VENT_ON,     "VENT_ON",      7, app_aircon_start },
    { CMD_WARN_CENTER, "WARN_CENTER", 11, action_warn_center },
    { CMD_WARN_RIGHT,  "WARN_RIGHT",  10, action_warn_right },
    { CMD_WARN_LEFT,   "WARN_LEFT",    9, action_warn_left },
    { CMD_CENTER_OK,   "CENTER_OK",    9, led_center_off },
    { CMD_RIGHT_OK,    "RIGHT_OK",     8, led_right_off },
    { CMD_LEFT_OK,     "LEFT_OK",      7, led_left_off },
    { CMD_WIN_CLOSE,   "WIN_CLOSE",    9, action_win_close },
    { CMD_WIN_OPEN,    "WIN_OPEN",     8, action_win_open },
};

#define CMD_TABLE_SIZE (sizeof(cmd_table) / sizeof(cmd_table[0]))


void app_process_command(CommandType in_cmd, char *out_ack, int max_len) {
    for (int i = 0; i < CMD_TABLE_SIZE; i++) {
        if (cmd_table[i].cmd_type == in_cmd) {
            if (cmd_table[i].handler != NULL) {
                cmd_table[i].handler();
            }

            snprintf(out_ack, max_len, "%s\r\n", cmd_table[i].cmd_str);
            return;
        }
    }

    // 일치하는 enum 값이 없을 때 (또는 CMD_INVALID 등)
    snprintf(out_ack, max_len, "NACK_INVALID_CMD\r\n");
}