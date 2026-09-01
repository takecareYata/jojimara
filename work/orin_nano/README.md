work/

└── zolzimala/

    └── orin_nano/

        ├── uart.py : stm32 와의 uart 통신제어

        ├── cam0.py : cam0 통신 및 안면인식, 구강인식후 uart통신 전송

        ├── cam1.py : cam1 통신 및 차간거리 인식 및 터널인식 후 uart통신 전송

        ├── gui.py : 제어 GUI 송출 제어

        └── main.py : GUI 실행 및 STM32 연동실행

trtexec --onnx=rps_yolo11n.onnx --saveEngine=yolo11n.engine
