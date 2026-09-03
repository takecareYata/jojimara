#!/usr/bin/env python3
import os
import subprocess
import sys
import time
from pathlib import Path
from threading import Thread

import cv2

# OpenCV가 포함한 Qt 플러그인과 PyQt5 플러그인이 충돌하지 않도록
# GUI 관련 모듈을 불러오기 전에 환경변수를 제거한다.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_FONTDIR", None)

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QKeySequence,
)
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QShortcut,
)

import cam0
import cam1
from uart_comm import UARTCommunication


BASE_DIR = Path(__file__).resolve().parent
UI_FILE = BASE_DIR / "gui.ui"
PY_FILE = BASE_DIR / "gui.py"
YOLO_ENGINE_FILE = BASE_DIR / "yolo11n.engine"

# ==========================================
# 전방 영상 입력 설정
# True  : road_test.mp4 사용
# False : USB 웹캠(/dev/video2) 사용
# ==========================================
USE_ROAD_VIDEO = True

ROAD_CAMERA_INDEX = 2
ROAD_VIDEO_FILE = BASE_DIR / "road_rain_test.mp4"
DRIVER_CAMERA_INDEX = 0

# cam1.py에 전달할 영상 입력을 결정한다.
if USE_ROAD_VIDEO:
    ROAD_VIDEO_SOURCE = str(ROAD_VIDEO_FILE)
else:
    ROAD_VIDEO_SOURCE = ROAD_CAMERA_INDEX

# Jetson과 STM32 사이의 UART 설정
# 실제 Jetson에서 확인된 장치명이 다르면 이 값만 변경하면 된다.
UART_PORT = "/dev/ttyTHS1"
UART_BAUD_RATE = 115200

# Tunnel Exit가 사라진 후 창문을 열기까지 기다리는 시간
EXIT_CLEAR_DELAY_SECONDS = 1.5


if not UI_FILE.is_file():
    raise FileNotFoundError(f"GUI 파일이 없습니다: {UI_FILE}")

subprocess.run(
    [
        sys.executable,
        "-m",
        "PyQt5.uic.pyuic",
        str(UI_FILE),
        "-o",
        str(PY_FILE),
    ],
    check=True,
)

from gui import Ui_MainWindow


class DriverMonitoringSystem(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Integrated Driver Monitoring System")

        # GUI 초기 상태
        self.warningFrame.hide()
        self.txtStmLog.setReadOnly(True)
        self.txtStmLog.document().setMaximumBlockCount(500)
        self.lblWarningText.setText(
            "DROWSINESS ALERT!\nWAKE UP!"
        )
        self.lblWarningText.setAlignment(Qt.AlignCenter)

        # 운전자 감지 상태
        self.warning_active = False
        self.drowsy_alarm_running = False
        self.yawn_active = False

        # 전방 카메라 상태 변화 확인용 변수
        self.previous_side_warning = False
        self.previous_tunnel_type = None

        # Tunnel Exit 검출 종료 판단용 변수
        self.exit_tracking_active = False #출구 통과 중인지 표시
        self.last_exit_seen_time = None # 마지막으로 출구를 인식한 시간
        self.exit_open_sent = False # win_open 중복 전송 방지

        # ==========================================
        # MP4 영상 이동 단축키
        # 오른쪽 방향키: 5초 앞으로
        # 왼쪽 방향키: 5초 뒤로
        # ==========================================

        self.seek_forward_shortcut = QShortcut(
            QKeySequence(Qt.Key_Right),
            self
        )
        self.seek_forward_shortcut.setContext(
            Qt.WindowShortcut
        )
        self.seek_forward_shortcut.activated.connect(
            lambda: self.seek_road_video(5)
        )

        self.seek_backward_shortcut = QShortcut(
            QKeySequence(Qt.Key_Left),
            self
        )
        self.seek_backward_shortcut.setContext(
            Qt.WindowShortcut
        )
        self.seek_backward_shortcut.activated.connect(
            lambda: self.seek_road_video(-5)
        )

        # UART는 main에서 한 번만 열고 모든 카메라가 공유한다.
        self.uart = UARTCommunication(
            port=UART_PORT,
            baud_rate=UART_BAUD_RATE,
        )
        self.uart.start()

        # 카메라 감지 모듈 초기화
        cam0.cam0_init()
        #영상경로 전달
        
        cam1.cam1_start(
            video_source=ROAD_VIDEO_SOURCE,
            engine_path=str(YOLO_ENGINE_FILE),
        )

        # 운전자 카메라는 GUI 스레드의 타이머에서 읽는다.
        self.driver_cap = cv2.VideoCapture(
            DRIVER_CAMERA_INDEX,
            cv2.CAP_V4L2,
        )
        self.driver_cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"),
        )
        self.driver_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.driver_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.driver_cap.set(cv2.CAP_PROP_FPS, 30)
        self.driver_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.add_stm_log("시스템 시작 및 카메라 초기화 완료")

        # 약 30ms마다 카메라 화면과 UART 로그를 갱신한다.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(30)

        QTimer.singleShot(0, self.update_camera_geometry)

    def update_camera_geometry(self):
        """창 크기에 맞춰 카메라와 오버레이 위치를 조절한다."""
        frame_width = max(1, self.cameraFrame.width())
        frame_height = max(1, self.cameraFrame.height())
        margin = 20

        self.lblRoadCamera.setGeometry(
            0,
            0,
            frame_width,
            frame_height,
        )

        driver_width = max(
            200,
            min(int(frame_width * 0.28), 400),
        )
        driver_height = int(driver_width * 3 / 4)
        driver_x = max(
            0,
            frame_width - driver_width - margin,
        )
        driver_y = max(
            0,
            frame_height - driver_height - margin,
        )
        self.lblDriverCamera.setGeometry(
            driver_x,
            driver_y,
            driver_width,
            driver_height,
        )

        warning_width = max(
            300,
            min(int(frame_width * 0.38), 550),
        )
        warning_height = max(
            120,
            min(int(frame_height * 0.22), 220),
        )
        warning_x = max(
            0,
            (frame_width - warning_width) // 2,
        )
        warning_y = max(
            0,
            (frame_height - warning_height) // 2,
        )
        self.warningFrame.setGeometry(
            warning_x,
            warning_y,
            warning_width,
            warning_height,
        )

        log_width = max(
            260,
            min(int(frame_width * 0.30), 450),
        )
        log_height = max(
            100,
            min(int(frame_height * 0.18), 180),
        )
        log_x = min(
            margin,
            max(0, frame_width - log_width),
        )
        log_y = max(
            0,
            frame_height - log_height - margin,
        )
        self.txtStmLog.setGeometry(
            log_x,
            log_y,
            log_width,
            log_height,
        )

        self.lblRoadCamera.lower()
        self.warningFrame.raise_()
        self.txtStmLog.raise_()
        self.lblDriverCamera.raise_()

    def resizeEvent(self, event):
        """창 크기가 바뀌면 카메라 배치를 다시 계산한다."""
        super().resizeEvent(event)
        self.update_camera_geometry()

    def add_stm_log(self, message):
        """STM32 통신 및 감지 로그를 GUI에 표시한다."""
        current_time = time.strftime("%H:%M:%S")
        self.txtStmLog.appendPlainText(
            f"[{current_time}] {message}"
        )
    def seek_road_video(self, seconds):
        """전방 영상의 재생 위치를 이동한다."""

        seek_success = cam1.cam1_seek(seconds)

        if not seek_success:
            return

        # 같은 장면에서 UART 명령을 다시 시험할 수 있도록 초기화
        self.previous_side_warning = False
        self.previous_tunnel_type = None

        # 영상 이동을 출구 검출 종료로 잘못 판단하지 않도록 초기화
        self.exit_tracking_active = False
        self.last_exit_seen_time = None
        self.exit_open_sent = False
    def update_uart_logs(self):
        """UART 백그라운드 스레드의 로그를 GUI에 출력한다."""
        for message_type, content in self.uart.get_messages():
            if message_type == "TX":
                self.add_stm_log(f"TX → {content}")
            elif message_type == "RX":
                self.add_stm_log(f"RX ← {content}")
            elif message_type == "ERROR":
                self.add_stm_log(f"UART 오류: {content}")
            else:
                self.add_stm_log(content)

    def drowsiness_alarm(self):
        """졸음 경고 상태 동안 콘솔 경고를 반복 출력한다."""
        while self.drowsy_alarm_running:
            print("[ALARM] Wake up sir!")
            time.sleep(1.5)

    def yawn_alarm(self):
        """하품 감지 콘솔 경고를 한 번 출력한다."""
        print("[ALARM] Take some fresh air sir!")

    def process_road_status(self, road_frame, road_status):
        """전방 카메라 상태 변화에 따라 UART 명령을 전송한다."""
        side_warning = road_status["roi_warning"]

        if side_warning:
            cv2.putText(
                road_frame,
                "STATUS: VEHICLE IN ROI!",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

        # 차량이 ROI 밖에서 안으로 들어온 순간 한 번만 전송한다.
        if side_warning and not self.previous_side_warning:
            self.uart.send_command("SIDE_WARN")

        self.previous_side_warning = side_warning

        # ==========================================
        # 터널 입구 및 출구 처리
        # ==========================================
        tunnel_type = road_status["tunnel_type"]
        current_time = time.monotonic()

        if tunnel_type == "entrance":
            # 터널 입구가 처음 검출된 순간 창문을 닫는다.
            if self.previous_tunnel_type != "entrance":
                self.uart.send_command("WIN_CLOSE")

            self.previous_tunnel_type = "entrance"

            # 새로운 터널에 진입했으므로 출구 상태를 초기화한다.
            self.exit_tracking_active = False
            self.last_exit_seen_time = None
            self.exit_open_sent = False

        elif tunnel_type == "exit":
            # Tunnel Exit가 보이는 동안에는 창문을 열지 않는다.
            # 마지막으로 출구가 보인 시간만 계속 갱신한다.
            if not self.exit_open_sent:
                self.exit_tracking_active = True
                self.last_exit_seen_time = current_time

            self.previous_tunnel_type = "exit"

        else:
            # tunnel_type이 None이면 현재 Exit가 보이지 않는 상태다.
            if (
                self.exit_tracking_active
                and not self.exit_open_sent
                and self.last_exit_seen_time is not None
                and (
                    current_time - self.last_exit_seen_time
                    >= EXIT_CLEAR_DELAY_SECONDS
                )
            ):
                # Exit가 1.5초 동안 검출되지 않았으므로
                # 터널을 완전히 빠져나간 것으로 판단한다.
                self.uart.send_command("WIN_OPEN")

                # 같은 출구에서 WIN_OPEN이 반복되지 않도록 상태 변경
                self.exit_tracking_active = False
                self.last_exit_seen_time = None
                self.exit_open_sent = True

    def process_driver_status(self, status):
        """운전자 상태 변화에 따라 GUI와 UART 명령을 처리한다."""
        if status["trigger_drowsy"] and not self.warning_active:
            self.warning_active = True
            self.warningFrame.show()
            self.drowsy_alarm_running = True

            # 졸음 경고가 시작되는 순간 한 번만 보낸다.
            self.uart.send_command("DROWSY_WARN")
            Thread(
                target=self.drowsiness_alarm,
                daemon=True,
            ).start()

        elif status["release_drowsy"] and self.warning_active:
            self.warning_active = False
            self.warningFrame.hide()
            self.drowsy_alarm_running = False

            # 졸음 경고가 해제되는 순간 한 번만 보낸다.
            self.uart.send_command("DROWSY_OK")

        if status["yawn_detected"]:
            if not self.yawn_active:
                self.yawn_active = True

                # 회의 결과에 따라 하품 시작 시 VENT_ON만 보낸다.
                # 입을 닫아도 VENT_OFF는 전송하지 않는다.
                self.uart.send_command("VENT_ON")
                Thread(
                    target=self.yawn_alarm,
                    daemon=True,
                ).start()
        else:
            # 다음 하품을 새 이벤트로 인식하기 위한 내부 상태만
            # 해제하며 UART 명령은 전송하지 않는다.
            self.yawn_active = False

    def update_frames(self):
        """두 카메라 프레임과 UART 로그를 주기적으로 갱신한다."""
        # UART 수신 스레드가 저장한 로그를 GUI에서 안전하게 출력한다.
        self.update_uart_logs()

        # 전방 카메라 및 YOLO 감지 결과 처리
        road_frame = cam1.cam1_get_frame()

        if road_frame is not None:
            road_status = cam1.cam1_get_status()
            self.process_road_status(road_frame, road_status)
            self.show_frame(self.lblRoadCamera, road_frame)

        # 운전자 카메라 및 졸음·하품 감지 결과 처리
        if self.driver_cap.isOpened():
            ret, driver_frame = self.driver_cap.read()

            if ret:
                driver_frame = cv2.flip(driver_frame, 1)
                processed_driver, status = cam0.cam0_process_frame(
                    driver_frame,
                    self.warning_active,
                )

                self.process_driver_status(status)
                self.show_frame(
                    self.lblDriverCamera,
                    processed_driver,
                )

    def show_frame(self, label, frame):
        """OpenCV 프레임을 QLabel 크기에 맞춰 표시한다."""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        q_image = QImage(
            rgb_frame.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()
        pixmap = QPixmap.fromImage(q_image).scaled(
            label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        label.setPixmap(pixmap)

    def closeEvent(self, event):
        """프로그램 종료 시 카메라와 UART 자원을 해제한다."""
        self.drowsy_alarm_running = False
        self.timer.stop()

        cam1.cam1_stop()
        cam0.cam0_close()

        if hasattr(self, "driver_cap"):
            self.driver_cap.release()

        if hasattr(self, "uart"):
            self.uart.close()

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DriverMonitoringSystem()
    window.show()
    sys.exit(app.exec_())

