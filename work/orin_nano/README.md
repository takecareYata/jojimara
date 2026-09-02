work/

└── zolzimala/

    └── orin_nano/

        ├── uart.py : stm32 와의 uart 통신제어

        ├── cam0.py : cam0 통신 및 안면인식, 구강인식후 uart통신 전송

        ├── cam1.py : cam1 통신 및 차간거리 인식 및 터널인식 후 uart통신 전송

        ├── gui.py : 제어 GUI 송출 제어

        ├── video.py : video 송출 제어

        └── main.py : GUI 실행 및 STM32 연동실행

trtexec --onnx=rps_yolo11n.onnx --saveEngine=yolo11n.engine

## PyQt5 설치 및 가상환경 연결

### 1. PyQt5 설치

패키지 목록을 업데이트한 후 PyQt5와 관련 도구를 설치한다.

```bash
sudo apt update
sudo apt install python3-pyqt5 pyqt5-dev-tools qttools5-dev-tools
```

### 2. 가상환경 설정 변경

다음 명령어로 가상환경의 `pyvenv.cfg` 파일을 연다.

```bash
vi /home/aidl/work/venv/pyvenv.cfg
```

아래 항목을 찾는다.

```text
include-system-site-packages = false
```

`false`를 `true`로 변경한다.

```text
include-system-site-packages = true
```

변경한 후 `Esc` 키를 누르고 다음 명령어를 입력하여 저장하고 종료한다.

```text
:wq
```

### 3. PyQt5 연결 확인

가상환경이 활성화된 상태에서 다음 명령어를 실행한다.

```bash
python -c "from PyQt5.QtWidgets import QApplication; print('가상환경 PyQt5 연결 성공')"
```

다음 메시지가 출력되면 PyQt5가 정상적으로 연결된 것이다.

```text
가상환경 PyQt5 연결 성공
```

## UART 포트 연결 오류 해결

프로그램 실행 로그에 다음과 같은 오류가 나타날 수 있다.

```text
UART 연결 실패 (/dev/ttyTHS1): ...
```

이 오류는 젯슨이 `/dev/ttyTHS1` UART 포트를 정상적으로 열지 못했다는 의미이다. 다음 순서대로 장치, PySerial 설치 상태, 사용자 권한을 확인한다.

### 1. UART 장치명 확인

다음 명령어를 실행하여 젯슨에서 사용할 수 있는 시리얼 장치를 확인한다.

```bash
ls -l /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

실행 결과에 다음 장치가 있어야 현재 코드의 UART 설정과 일치한다.

```text
/dev/ttyTHS1
```

`/dev/ttyTHS1`이 없다면 코드에서 설정한 UART 장치명과 젯슨에서 실제로 인식된 장치명이 일치하지 않는 상태이다.

### 2. PySerial 설치 확인

가상환경이 활성화된 상태에서 다음 명령어를 실행한다.

```bash
python -c "import serial; print(serial.__file__)"
```

정상이라면 현재 파이썬이 사용하는 `serial` 모듈의 파일 경로가 출력된다.

예시:

```text
/home/aidl/work/venv/lib/python3.10/site-packages/serial/__init__.py
```

`ModuleNotFoundError` 등의 오류가 발생하면 PySerial을 설치한다.

```bash
python -m pip install pyserial
```

설치 후 다시 확인한다.

```bash
python -c "import serial; print(serial.__file__)"
```

### 3. 사용자 권한 확인

UART 장치와 PySerial이 모두 정상이라면 현재 사용자에게 UART 포트를 열 수 있는 권한이 있는지 확인한다.

```bash
groups
```

출력된 그룹 목록에 `dialout`이 포함되어 있어야 한다.

예시:

```text
aidl adm dialout sudo
```

`dialout`이 없다면 현재 `aidl` 계정에 UART 접근 권한이 없는 상태이므로 다음 명령어를 실행한다.

```bash
sudo usermod -aG dialout aidl
```

그룹 권한을 적용하기 위해 반드시 젯슨을 재부팅한다.

```bash
sudo reboot
```

재부팅 후 다시 접속하여 `dialout` 그룹이 적용되었는지 확인한다.

```bash
groups
```

### 4. 다른 프로그램의 포트 점유 확인

다른 프로그램이 `/dev/ttyTHS1`을 사용 중인지 확인한다.

```bash
sudo lsof /dev/ttyTHS1
```

아무 결과도 나오지 않으면 다른 프로그램이 해당 포트를 사용하고 있지 않은 상태이다.

프로그램 정보가 출력된다면 해당 프로그램이 `/dev/ttyTHS1`을 사용 중이므로, 기존 프로그램을 종료한 후 다시 시도해야 한다.

### 5. UART 포트 열기 테스트

`groups` 결과에 `dialout`이 포함되어 있으면 프로젝트 폴더로 이동한 후 다음 명령어로 UART 포트를 시험한다.

```bash
python -c "import serial; s=serial.Serial('/dev/ttyTHS1',115200,timeout=1); print('UART OPEN:',s.is_open); s.close()"
```

정상적으로 포트가 열리면 다음과 같이 출력된다.

```text
UART OPEN: True
```

위 메시지가 확인되면 UART 장치와 사용자 권한이 정상적으로 설정된 것이므로 프로젝트 프로그램을 실행한다.

```bash
python main.py
```
