import os
import sys
import time
import subprocess
from pathlib import Path
from threading import Thread

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QShortcut

import cv2
import numpy as np
import cam1
import cam0
import video
from uart import UARTCommunication

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_FONTDIR", None)

BASE_DIR = Path(__file__).resolve().parent
UI_FILE = BASE_DIR / "gui.ui"
PY_FILE = BASE_DIR / "gui.py"

if not UI_FILE.is_file():
    raise FileNotFoundError(f"GUI 파일이 없습니다: {UI_FILE}")

subprocess.run([sys.executable, "-m", "PyQt5.uic.pyuic", str(UI_FILE), "-o", str(PY_FILE)], check=True)
from gui import Ui_MainWindow

ROAD_CAMERA_INDEX = 2
DRIVER_CAMERA_INDEX = 0

FALLBACK_VIDEO_PATH = str(BASE_DIR / "test_video.mp4")
YOLO_ENGINE_PATH = str(BASE_DIR / "yolo11n.engine")

# Jetson - STM32 UART 설정
UART_PORT = "/dev/ttyTHS1"
UART_BAUD_RATE = 115200

# 터널 출구 통과 판단 지연 시간 (초)
EXIT_CLEAR_DELAY_SECONDS = 1.5

# 테스트 영상 A/S 키 이동 시간
VIDEO_SEEK_SECONDS = 5


class DriverMonitoringSystem(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Integrated Driver Monitoring System")

        self.warningFrame.hide()
        self.txtStmLog.setReadOnly(True)
        self.txtStmLog.document().setMaximumBlockCount(500)
        self.lblWarningText.setText("DROWSINESS ALERT!\nWAKE UP!")
        self.lblWarningText.setAlignment(Qt.AlignCenter)

        # 로그창 등에 포커스가 있어도 영상 단축키가 동작하도록 설정한다.
        self.setup_video_shortcuts()

        # 운전자 상태 플래그
        self.warning_active = False
        self.drowsy_alarm_running = False
        self.yawn_active = False

        # 비디오 폴백 상태 관리 및 대기 카운터
        self.using_video_fallback = False
        self.cam1_retry_count = 0
        self.MAX_CAM1_RETRY = 100  # 30ms * 100회 = 약 3초간 초기화 대기

        # 전방 상태 변화 추적 플래그 (Edge Triggering) - 3가지 ROI 개별 추적
        self.previous_roi_left = False
        self.previous_roi_center = False
        self.previous_roi_right = False
        self.previous_tunnel_type = None

        # 터널 출구 추적 상태
        self.exit_tracking_active = False
        self.last_exit_seen_time = None
        self.exit_open_sent = False

        # UART 통신 객체 생성 및 스레드 시작
        self.uart = UARTCommunication(
            port=UART_PORT,
            baud_rate=UART_BAUD_RATE,
        )
        self.uart.start()

        # cam0, cam1 모듈 초기화 및 실행 시도
        cam0.cam0_init()
        cam1.cam1_start(ROAD_CAMERA_INDEX, YOLO_ENGINE_PATH)
        
        # 운전자 카메라 (CAM0 영상 수집용)
        self.driver_cap = cv2.VideoCapture(DRIVER_CAMERA_INDEX, cv2.CAP_V4L2)
        self.driver_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.driver_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.driver_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.driver_cap.set(cv2.CAP_PROP_FPS, 30)
        self.driver_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.add_stm_log("시스템 시작 및 카메라 연결 시도 완료")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(30)

        QTimer.singleShot(0, self.update_camera_geometry)

    def update_camera_geometry(self):
        frame_width = max(1, self.cameraFrame.width())
        frame_height = max(1, self.cameraFrame.height())
        margin = 20

        self.lblRoadCamera.setGeometry(0, 0, frame_width, frame_height)

        driver_width = max(200, min(int(frame_width * 0.28), 400))
        driver_height = int(driver_width * 3 / 4)
        driver_x = max(0, frame_width - driver_width - margin)
        driver_y = max(0, frame_height - driver_height - margin)
        self.lblDriverCamera.setGeometry(driver_x, driver_y, driver_width, driver_height)

        warning_width = max(300, min(int(frame_width * 0.38), 550))
        warning_height = max(120, min(int(frame_height * 0.22), 220))
        warning_x = max(0, (frame_width - warning_width) // 2)
        warning_y = max(0, (frame_height - warning_height) // 2)
        self.warningFrame.setGeometry(warning_x, warning_y, warning_width, warning_height)

        log_width = max(260, min(int(frame_width * 0.30), 450))
        log_height = max(100, min(int(frame_height * 0.18), 180))
        log_x = min(margin, max(0, frame_width - log_width))
        log_y = max(0, frame_height - log_height - margin)
        self.txtStmLog.setGeometry(log_x, log_y, log_width, log_height)

        self.lblRoadCamera.lower()
        self.warningFrame.raise_()
        self.txtStmLog.raise_()
        self.lblDriverCamera.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_camera_geometry()

    def add_stm_log(self, message):
        """STM32 통신 및 감지 로그를 GUI에 출력한다."""
        current_time = time.strftime("%H:%M:%S")
        self.txtStmLog.appendPlainText(f"[{current_time}] {message}")

    def update_uart_logs(self):
        """UART 백그라운드 스레드의 TX/RX/오류 로그를 GUI에 출력한다."""
        for message_type, content in self.uart.get_messages():
            if message_type == "TX":
                self.add_stm_log(f"TX → {content}")
            elif message_type == "RX":
                self.add_stm_log(f"RX ← {content}")
            elif message_type == "ERROR":
                self.add_stm_log(f"UART 오류: {content}")
            else:
                self.add_stm_log(content)

    def _reset_detection_states(self):
        """영상 시크/이동 시 중복 명령 및 판단 오류 방지를 위한 상태 초기화"""
        self.previous_roi_left = False
        self.previous_roi_center = False
        self.previous_roi_right = False
        self.previous_tunnel_type = None
        self.exit_tracking_active = False
        self.last_exit_seen_time = None
        self.exit_open_sent = False

    def setup_video_shortcuts(self):
        """하위 위젯의 포커스와 관계없이 동작하는 영상 단축키를 만든다."""
        self.video_shortcuts = []

        shortcut_actions = (
            ("Space", self.toggle_video_pause),
            ("D", self.step_video_frame),
            ("A", lambda: self.seek_video(-VIDEO_SEEK_SECONDS)),
            ("S", lambda: self.seek_video(VIDEO_SEEK_SECONDS)),
            ("Q", self.close_video_mode),
        )

        for key_text, action in shortcut_actions:
            shortcut = QShortcut(QKeySequence(key_text), self)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.activated.connect(action)
            self.video_shortcuts.append(shortcut)

    def toggle_video_pause(self):
        """테스트 영상 모드일 때 재생과 일시정지를 전환한다."""
        if self.using_video_fallback:
            video.video_toggle_pause()

    def step_video_frame(self):
        """일시정지된 테스트 영상을 한 프레임 진행한다."""
        if self.using_video_fallback:
            video.video_step_frame()
            self._reset_detection_states()

    def seek_video(self, seconds):
        """테스트 영상을 지정한 초만큼 이동한다."""
        if self.using_video_fallback:
            video.video_seek_seconds(seconds)
            self._reset_detection_states()

    def close_video_mode(self):
        """테스트 영상 모드에서 Q키로 프로그램을 종료한다."""
        if self.using_video_fallback:
            self.close()

    def drowsiness_alarm(self):
        while self.drowsy_alarm_running:
            print("[ALARM] Wake up sir!")
            time.sleep(1.5)

    def yawn_alarm(self):
        print("[ALARM] Take some fresh air sir!")

    def process_road_status(self, frame, status_dict):
        """전방 감지 상태(3개 ROI 분할)에 따라 OSD 오버레이와 UART 명령을 처리한다."""
        # 3가지 ROI 경고 상태 읽기
        warn_left = status_dict.get("roi_warning_left", False)
        warn_center = status_dict.get("roi_warning_center", False)
        warn_right = status_dict.get("roi_warning_right", False)

        # 1. 화면 OSD 오버레이 (영역별 표시)
        active_rois = []
        if warn_left: active_rois.append("LEFT")
        if warn_center: active_rois.append("CENTER")
        if warn_right: active_rois.append("RIGHT")

        if active_rois:
            roi_str = ", ".join(active_rois)
            cv2.putText(
                frame,
                f"WARNING : {roi_str}",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
            )

        # 2. UART 전송 Edge Trigger (각 영역별 상태가 False -> True로 변할 때만 명령 전송)
        if warn_left and not self.previous_roi_left:
            self.uart.send_command("WARN_LEFT")
        if warn_center and not self.previous_roi_center:
            self.uart.send_command("WARN_CENTER")
        if warn_right and not self.previous_roi_right:
            self.uart.send_command("WARN_RIGHT")

        # 이전 ROI 상태 갱신
        self.previous_roi_left = warn_left
        self.previous_roi_center = warn_center
        self.previous_roi_right = warn_right

        # 3. 터널 입구/출구 처리
        tunnel_entrance = status_dict.get("tunnel_entrance", False)
        tunnel_exit = status_dict.get("tunnel_exit", False)
        current_time = time.monotonic()

        if tunnel_entrance:
            if self.previous_tunnel_type != "entrance":
                self.uart.send_command("WIN_CLOSE")

            self.previous_tunnel_type = "entrance"
            self.exit_tracking_active = False
            self.last_exit_seen_time = None
            self.exit_open_sent = False

        elif tunnel_exit:
            if not self.exit_open_sent:
                self.exit_tracking_active = True
                self.last_exit_seen_time = current_time

            self.previous_tunnel_type = "exit"

        else:
            if (
                self.exit_tracking_active
                and not self.exit_open_sent
                and self.last_exit_seen_time is not None
                and (current_time - self.last_exit_seen_time >= EXIT_CLEAR_DELAY_SECONDS)
            ):
                self.uart.send_command("WIN_OPEN")
                self.exit_tracking_active = False
                self.last_exit_seen_time = None
                self.exit_open_sent = True

    def process_driver_status(self, status):
        """운전자 감지 상태에 따라 알람 및 UART 명령을 처리한다."""
        if status["trigger_drowsy"] and not self.warning_active:
            self.warning_active = True
            self.warningFrame.show()
            self.drowsy_alarm_running = True

            self.uart.send_command("DROWSY_WARN")
            Thread(target=self.drowsiness_alarm, daemon=True).start()

        elif status["release_drowsy"] and self.warning_active:
            self.warning_active = False
            self.warningFrame.hide()
            self.drowsy_alarm_running = False

            self.uart.send_command("DROWSY_OK")

        if status["yawn_detected"]:
            if not self.yawn_active:
                self.yawn_active = True
                self.uart.send_command("VENT_ON")
                Thread(target=self.yawn_alarm, daemon=True).start()
        else:
            self.yawn_active = False

    def update_frames(self):
        # 백그라운드 수신된 UART 로그 출력
        self.update_uart_logs()

        # 1. 전방 카메라(cam1) 또는 비디오(video) 영상 처리
        if not self.using_video_fallback:
            # cam1.cam1_get_frame()이 (main_frame, debug_frame) 튜플을 반환하므로 언패킹 처리
            road_frame, _ = cam1.cam1_get_frame(include_debug=False)
            if road_frame is not None:
                road_status = cam1.cam1_get_status()
                self.process_road_status(road_frame, road_status)
                self.show_frame(self.lblRoadCamera, road_frame)
                self.cam1_retry_count = 0  # 프레임 수신 성공 시 대기 카운터 초기화
            else:
                self.cam1_retry_count += 1
                # 대기 시간(약 3초) 동안 프레임이 안 나올 경우에만 Video Fallback 전환
                if self.cam1_retry_count > self.MAX_CAM1_RETRY:
                    # Cam1의 카메라와 TensorRT 자원을 먼저 정리한 뒤 영상을 시작한다.
                    cam1.cam1_stop()
                    self.using_video_fallback = True
                    self.add_stm_log("cam1 미작동: 비디오 영상 모드로 전환 (키보드 단축키 사용 가능)")
                    video.video_start(video_path=FALLBACK_VIDEO_PATH, engine_path=YOLO_ENGINE_PATH)

        if self.using_video_fallback:
            # GUI에 표시하지 않는 차선 디버그 영상은 복사하지 않는다.
            video_frame, _, video_status = video.video_get_data(include_debug=False)
            if video_frame is not None:
                self.process_road_status(video_frame, video_status)
                self.show_frame(self.lblRoadCamera, video_frame)

        # 2. 운전자 카메라(cam0) 프레임 처리
        if self.driver_cap.isOpened():
            ret, driver_frame = self.driver_cap.read()
            if ret:
                driver_frame = cv2.flip(driver_frame, 1)
                processed_driver, status = cam0.cam0_process_frame(
                    driver_frame, self.warning_active
                )

                self.process_driver_status(status)
                self.show_frame(self.lblDriverCamera, processed_driver)

    def show_frame(self, label, frame):
        # 방어 코드: frame이 None이거나 NumPy 배열이 아닌 경우 무시
        if frame is None or not isinstance(frame, np.ndarray):
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        q_image = QImage(rgb_frame.data, w, h, ch * w, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(q_image).scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.drowsy_alarm_running = False
        self.timer.stop()

        cam1.cam1_stop()
        if self.using_video_fallback:
            video.video_stop()
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
