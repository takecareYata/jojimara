
#  Driver Monitoring (STM32_M4)

상위 제어 시스템(jetson_orin_nano)으로부터 UART로 전송되는 안전 경고 및 차량 제어 명령을 파싱하여 부저, 팬, 창문 모터 등 하드웨어를 실시간 제어하는 시스템입니다.

---

##  1. Hardware Architecture & Pin Map

| 구분 | 장치 / 기능 | STM32M4 Pin | 인터페이스 / 모드 | 비고 |
| :--- | :--- | :--- | :--- | :--- |
| **통신** | Host UART | `PA9` (TX), `PA10` (RX) | UART1 (115200 bps, 8-N-1) | 수신 패킷 파싱 및 상태 응답 |
| **경고** | 메인 부저 (Buzzer) | `PB0` | TIM3_CH3 (PWM) | 졸음(지속 패턴), 사각지대(단발 비프) |
| **경고** | 차량 근접 경고 (Led) | `PC5`, `PC6`, `PC8` | GPIO Output | Led 토글 |
| **환기** | 공기청정 모터 | `PB8` (IN3), `PB9` (IN4), `PA1`(ENB) | TIM2_CH2(GPIO / PWM)  | |
| **졸음** | 에어컨 모터 | `PA6` (IN1), `PA7` (IN2), `PB10`(ENA) | TIM2_CH3(GPIO / PWM)  | |
| **창문** | 서보 모터 | `PB6` | TIM4_CH1(GPIO / PWM)  | 정방향(Up) / 역방향(Down) |



---

##  2. UART Protocol Specification

- **Baud Rate:** `115200 bps`
- **Data / Parity / Stop:** `8-N-1`
- **Packet Delimiter:** `\\n`  또는 `\\r\\n`

|카메라 번호 | 전송 패킷 (ASCII) | 기능 정의 | STM32F4 액추에이터 처리 동작 |
| :--- | :--- | :--- | :--- |
| Cam 2 | `DROWSY_WARN` | 졸음 경고 발생 | 경고 부저 지속 패턴 출력 (`BUZZER_STATE_ALERT`) |
| Cam 2 | `DROWSY_OK` | 졸음 상태 해제 | 경고 부저 즉시 Off (`BUZZER_STATE_IDLE`) |
| Cam 2 | `VENT_ON` | 에어컨 요청 (하품 감지) | 에어컨 모터 정방향 구동( 10초만 작동 후 정지 )  |
| Cam 1 | `WIN_CLOSE` | 창문 닫기 (터널 진입) | 창문(서보모터 정방향), 공기청정모터 정방향 구동 (`IN1=HIGH, IN2=LOW`) |
| Cam 1 | `WIN_OPEN` | 창문 열기 (터널 탈출) | 창문(서보모터 역방향), 공기청정모터 정지 |
| Cam 1 | `WARN_CENTER` | 앞차 근접 경고 | 경고 LED 토글 |
| Cam 1 | `CENTER_OK` | 앞차 경고 해제 | 경고 LED off |
| Cam 1 | `WARN_RIGHT` | 오른쪽 차선 근접 경고 | 경고 LED 토글 |
| Cam 1 | `RIGHT_OK` | 오른쪽 차선 경고 해제 | 경고 LED off |
| Cam 1 | `WARN_LEFT` | 왼쪽 차선 근접 경고 | 경고 LED 토글 |
| Cam 1 | `LEFT_OK` | 왼쪽 차선 경고 해제 | 경고 LED off |

---

##  3. Software Architecture & Flow

```c
+-----------------------------------------------------------------------------------+
|                           Jetson Orin Nano (Host System)                          |
|                                                                                   |
|   +------------------------------------+   +----------------------------------+   |
|   | Cam 1: External Perception         |   | Cam 2: Driver Monitoring (DMS)   |   |
|   | - Tunnel (WIN_CLOSE / WIN_OPEN)    |   | - Drowsy (DROWSY_WARN / OK)      |   |
|   | - Proximity (WARN/OK_C, L, R)      |   | - Yawn (VENT_ON)                 |   |
|   +-----------------+------------------+   +-----------------+----------------+   |
|                     |                                        |                    |
|                     +--------------------+-------------------+                    |
|                                          |                                        |
|                          [ UART Protocol Generator ]                              |
|                          - 115200 bps, 8-N-1, '\n' Delimiter                      |
+------------------------------------------+----------------------------------------+
                                           |
                                           | UART2 (PA9:TX, PA10:RX)
                                           v
+-----------------------------------------------------------------------------------+
|                        STM32F4 Core (Peripheral Controller)                       |
|                                                                                   |
|  [ Comm Subsystem ]                                                               |
|    +----------------------+      +-------------------------------------------+    |
|    | UART2 RX Interrupt   | ---> | ASCII Packet Parser                       |    |
|    | (Ring Buffer / Line) |      | (DROWSY_*, VENT_*, WIN_*, WARN/OK_*)      |    |
|    +----------------------+      +---------------------+---------------------+    |
|                                                        |                          |
|  [ Real-Time Control ]                                 v                          |
|    +-------------------------------------------------------------------------+    |
|    | Non-Blocking Actuator State Machine                                     |    |
|    | - Tick Scheduler (TIM3 1ms / Systick)                                   |    |
|    | - Buzzer Pattern Engine (Continuous Alert / Single Beep)                |    |
|    | - 10s Non-blocking Countdown Timer (A/C Motor)                          |    |
|    | - Direction & Speed Controllers (Air Cleaner, Window Servo, LEDs)       |    |
|    +-----+---------------+--------------------+---------------+--------------+    |
+----------|---------------|--------------------|---------------|-------------------+
           |               |                    |               |
           | TIM3_CH3      | GPIO Output        | TIM2 (PWM/IO) | TIM4_CH1 (PWM)
           | (PB0)         | (PC5, PC6, PC8)    | (PA/PB Pins)  | (PB6)
           v               v                    v               v
     +-----------+   +-----------+        +-----------+   +-----------+
     |  Buzzer   |   | Proximity |        |  DC Motor |   |   Servo   |
     |           |   |   LEDs    |        |  Drivers  |   |   Motor   |
     +-----------+   +-----------+        +-----+-----+   +-----+-----+
     - Drowsy Alert  - Center (PC5)             |               |
     - Proximity     - Left   (PC6)             |               +- Window Up/Down
       Beep          - Right  (PC8)             |
                                                +-- Air Cleaner (PA1, PB8, PB9)
                                                +-- A/C (PB10, PA6, PA7 - 10s Run)
 ```

### 1) Non-Blocking Data Reception
- `UART Rx Interrupt` 기반 구성
- 개행 문자(`\\n`) 수신 시 패킷을 분리하여 커맨드 파서(Parser)로 전달

### 2) Non-Blocking Actuator State Machine
- `Delay()`를 배제하고 `Tim3` 하드웨어 타이머 인터럽트를 적용
- 부저 주기 패턴 및 환기 팬 10초 카운트다운 중에도 추가 UART 명령 수신 및 모터 제어가 끊김 없이 병렬 실행

##  4. Class Diagram

```c
+-----------------------------------------------------------------------------------------+
|                                      <<Program Entry>>                                  |
|                                            Main                                         |
+-----------------------------------------------------------------------------------------+
| - cmd_buf[64]: char                                                                     |
| - ack_cmd_buf[64]: char                                                                 |
| - Uart_Data_In: volatile int                                                            |
+-----------------------------------------------------------------------------------------+
| + Sys_Init(baud: int): void                                                             |
| + Main(): void                                                                          |
+-----------------------------------------------------------------------------------------+
         |                             |                                 |
         | uses                        | calls                           | calls
         v                             v                                 v
+-------------------------------+ +-------------------------------+ +-----------------------+
|          Protocol             | |          AppProcess           | |        UartComm       |
+-------------------------------+ +-------------------------------+ +-----------------------+
| - cmd_table[]: CommandMap     | | - cmd_table[]: CommandEntry   | | - s_cmd_ready: bool   |
+-------------------------------+ +-------------------------------+ | - s_cmd_buf[64]: char |
| + UART_ParseCommand(          | | + app_process_command(        | +-----------------------+
|     cmd: char*): CommandType  | |     in_cmd, *out_ack, max_len)| | + Uart2_Init(baud)    |
+-------------------------------+ | - action_win_close(): void    | | + UART2_SendChar()    |
                                  | - action_win_open(): void     | | + UART2_SendString()  |
                                  | - action_warn_center(): void  | | + UART2_Ack_SendString|
                                  | - action_warn_right(): void   | | + Uart2_RX_Interrupt_ |
                                  | - action_warn_left(): void    | |   Enable(en: int)     |
                                  +-------------------------------+ +-----------------------+
                                                  |
                                                  v
         +----------------------------------------+----------------------------------------+
         |                                        |                                        |
         v                                        v                                        v
+--------------------+                   +--------------------+                   +--------------------+
|      MotorApp      |                   |     BuzzerApp      |                   |       LedApp       |
+--------------------+                   +--------------------+                   +--------------------+
| - motor_ac:        |                   | - buzzer: Buzzer_t |                   | - target_led:      |
|   DCMotor_t        |                   | - warning_tick: int|                   |   volatile LED_STAT|
| - motor_purifier:  |                   | - warning_state:   |                   | - led_count:       |
|   DCMotor_t        |                   |   volatile int     |                   |   volatile int     |
| - ac_running_tick: |                   | - is_bz_running_   |                   +--------------------+
|   volatile int     |                   |   tick: volatile   |                   | + led_init(): void |
| - is_ac_running:   |                   +--------------------+                   | + led_interrupt(): |
|   volatile int     |                   | + app_buzzer_init()|                   |   void             |
+--------------------+                   | + app_start_buzzer |                   | + set_led_warning()|
| + app_motor_init() |                   | + app_stop_buzzer()|                   | + led_center_off() |
| + app_aircon_start |                   | + app_buzzer_mute()|                   | + led_right_off()  |
| + app_airpurifier_ |                   | + app_buzzer_      |                   | + led_left_off()   |
|   start() / stop() |                   |   interrupt(): void|                   +--------------------+
| + app_motor_1ms_   |                   +--------------------+                             |
|   ISR(): void      |                             |                                        |
+--------------------+                             | controls                               |
         |                                         v                                        |
         | controls                       +--------------------+                            |
         v                                |      Buzzer_t      |                            |
+--------------------+                    +--------------------+                            |
|    DCMotor_t       |                    | + htim: TIM_TypeDef|                            |
+--------------------+                    | + channel: uint32_t|                            |
| + dir_port: GPIO*  |                    | + timer_clock: uint|                            |
| + pin_in1: uint8_t |                    +--------------------+                            |
| + pin_in2: uint8_t |                    | + buzzer_time_init |                            |
| + ccr: uint32_t*   |                    | + buzzer_insert_hz |                            |
+--------------------+                    | + tim_set_compare()|                            |
| + dcmotor_init()   |                    | + tim_set_auto_    |                            |
| + dcmotor_start()  |                    |   reload()         |                            |
| + dcmotor_stop()   |                    +--------------------+                            |
+--------------------+                                                                      |
         |                                                                                  |
         +------------------------------------+                                             |
                                              |                                             |
                                              v                                             |
+------------------------------+     +------------------------------------------------------+----+
|      WindowServoMotor        |     |                    ExceptionISR                           |
+------------------------------+     +-----------------------------------------------------------+
| + window_init(): void        |     | + USART2_IRQHandler(): void  // rx_idx -> cmd_buf         |
| + window_open(): void        |     | + TIM1_UP_TIM10_IRQHandler(): void                        |
| + window_close(): void       |     |   - app_motor_1ms_ISR()                                   |
+------------------------------+     |   - app_buzzer_interrupt()                                |
               |                     |   - led_interrupt()                                       |
               v                     +-----------------------------------------------------------+
+------------------------------------------------------------------------------------------------+
|                                    Hardware Layer (STM32F4)                                    |
+------------------------------------------------------------------------------------------------+
| - USART2 : PA2 (TX), PA3 (RX)          [AF07, 115200bps]                                       |
| - TIM1   : 1ms System Tick Interrupt   [Update ISR]                                            |
| - TIM2   : PA1 (CH2-Purifier ENB), PB10 (CH3-AC ENA), PA6/PA7 (AC DIR), PB8/PB9 (Purifier DIR) |
| - TIM3   : PB0 (CH3-Buzzer PWM)        [AF02, 1MHz base]                                       |
| - TIM4   : PB6 (CH1-Window Servo PWM)  [AF02, 50Hz, CCR 500~2500]                              |
| - GPIOC  : PC5 (Right LED), PC6 (Left LED), PC8 (Center LED)                                   |
+------------------------------------------------------------------------------------------------+
```
