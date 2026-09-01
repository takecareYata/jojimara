work/

└── zolzimala/

    └── orin_nano/

        ├── uart.py : stm32 와의 uart 통신제어

        ├── cam0.py : cam0 통신 및 안면인식, 구강인식후 uart통신 전송

        ├── cam1.py : cam1 통신 및 차간거리 인식 및 터널인식 후 uart통신 전송

        ├── gui.py : 제어 GUI 송출 제어

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