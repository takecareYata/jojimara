
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
[ Jetson Orin Nano (Dual AI Pipeline) ]
 ├─ Cam 1 (전방 카메라) : 터널 감지 (`WIN_CLOSE`/`OPEN`), 사각지대/옆차선 감지 (`SIDE_WARN`)
 └─ Cam 2 (내부 운전자) : 졸음 안면 인식 (`DROWSY_WARN`/`OK`), 하품 감지 (`VENT_ON`/`OFF`)
             │
             │ UART (115200 bps, ASCII + '\n')
             ▼
[ STM32F4 Peripheral Controller ]
 ├─ Buzzer (TIM3) / LED ── 졸음 경고 패턴 및 측면 경고음
 ├─ Ventilation Motor   ── 환기 구동
 ├─ Airconditioner Motor── 에어컨 구동
 └─ Window Motor        ── 터널 출입 시 창문 개폐
 ```

### 1) Non-Blocking Data Reception
- `UART Rx Interrupt` 기반 구성
- 개행 문자(`\\n`) 수신 시 패킷을 분리하여 커맨드 파서(Parser)로 전달

### 2) Non-Blocking Actuator State Machine
- `Delay()`를 배제하고 `Tim3` 하드웨어 타이머 인터럽트를 적용
- 부저 주기 패턴 및 환기 팬 10초 카운트다운 중에도 추가 UART 명령 수신 및 모터 제어가 끊김 없이 병렬 실행


