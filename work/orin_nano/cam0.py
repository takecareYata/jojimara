#!/usr/bin/env python3
"""Jetson의 카메라 2대를 사용하는 운전자 상태 감지 GUI.

gui.ui에 필요한 objectName:
    cameraFrame, lblRoadCamera, lblDriverCamera,
    warningFrame, lblWarningText, txtStmLog
"""

import os
import subprocess
import sys
import time
from pathlib import Path
from threading import Thread

# OpenCV에 포함된 Qt가 먼저 로드되지 않도록 PyQt5를 먼저 불러온다.
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow

import cv2

# OpenCV와 PyQt5의 Qt 플러그인 경로 충돌을 방지한다.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_FONTDIR", None)

import mediapipe as mp
import numpy as np
from scipy.spatial import distance as dist


# ---------------------------------------------------------------------------
# 카메라 설정: Jetson에서 확인한 실제 카메라 번호
# ---------------------------------------------------------------------------
ROAD_CAMERA_INDEX = 2
DRIVER_CAMERA_INDEX = 0

ROAD_WIDTH = 1280
ROAD_HEIGHT = 720
DRIVER_WIDTH = 640
DRIVER_HEIGHT = 480
CAMERA_FPS = 30


# ---------------------------------------------------------------------------
# 졸음 및 하품 판단 기준
# ---------------------------------------------------------------------------
EYE_AR_THRESH = 0.25
EYES_CLOSED_WARNING_SECONDS = 2.0
EYES_OPEN_RELEASE_SECONDS = 5.0
RECOVERY_BLINK_GRACE_SECONDS = 0.5
YAWN_THRESH = 25

LEFT_EYE_IDX = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]
MOUTH_TOP_BOTTOM = (13, 14)


# ---------------------------------------------------------------------------
# 프로그램을 실행할 때 gui.ui를 gui.py로 자동 변환한다.
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent
UI_FILE = BASE_DIR / "gui.ui"
PY_FILE = BASE_DIR / "gui.py"

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


def calculate_ear(eye_points):
    """눈의 랜드마크 6개를 이용해 EAR 값을 계산한다."""
    vertical_1 = dist.euclidean(eye_points[1], eye_points[5])
    vertical_2 = dist.euclidean(eye_points[2], eye_points[4])
    horizontal = dist.euclidean(eye_points[0], eye_points[3])

    if horizontal == 0:
        return 0.0

    return (vertical_1 + vertical_2) / (2.0 * horizontal)


def open_usb_camera(index, width, height, fps):
    """V4L2 USB 카메라를 열고 MJPG 촬영 설정을 적용한다."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class Form(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Driver Monitoring System")

        self.validate_ui_widgets()

        # 눈 감음, 눈 뜸, 짧은 깜빡임의 시간을 측정하는 변수
        self.eyes_closed_since = None
        self.eyes_open_since = None
        self.recovery_blink_since = None
        self.warning_active = False
        self.drowsy_alarm_running = False
        self.yawn_active = False

        self.warningFrame.hide()
        self.txtStmLog.setReadOnly(True)
        self.txtStmLog.document().setMaximumBlockCount(500)

        # MediaPipe Face Mesh 초기화
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing = mp.solutions.drawing_utils
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.lip_drawing_spec = self.mp_drawing.DrawingSpec(
            color=(0, 255, 255),
            thickness=2,
            circle_radius=1,
        )

        # 전방 카메라와 운전자 카메라 열기
        self.road_cap = open_usb_camera(
            ROAD_CAMERA_INDEX, ROAD_WIDTH, ROAD_HEIGHT, CAMERA_FPS
        )
        self.driver_cap = open_usb_camera(
            DRIVER_CAMERA_INDEX, DRIVER_WIDTH, DRIVER_HEIGHT, CAMERA_FPS
        )

        self.report_camera_status()
        self.add_stm_log("STM 연결 대기 중")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(30)

        # GUI가 표시된 직후 위젯의 위치와 크기를 계산한다.
        QTimer.singleShot(0, self.update_camera_geometry)

    def validate_ui_widgets(self):
        """Qt Designer에서 만든 위젯을 확인하고 부모 관계를 정리한다."""
        mandatory = (
            "cameraFrame",
            "lblRoadCamera",
            "lblDriverCamera",
            "warningFrame",
            "lblWarningText",
            "txtStmLog",
        )
        missing = [name for name in mandatory if not hasattr(self, name)]
        if missing:
            names = ", ".join(missing)
            raise RuntimeError(
                f"gui.ui에 다음 objectName이 필요합니다: {names}"
            )

        # Designer에서 만든 위젯을 그대로 사용한다.
        # 실수로 centralwidget 아래에 배치했다면 cameraFrame 아래로 옮긴다.
        for widget in (
            self.lblRoadCamera,
            self.lblDriverCamera,
            self.warningFrame,
            self.txtStmLog,
        ):
            if widget.parent() is not self.cameraFrame:
                widget.setParent(self.cameraFrame)
                widget.show()

        if self.lblWarningText.parent() is not self.warningFrame:
            raise RuntimeError(
                "gui.ui에서 lblWarningText를 warningFrame 안에 배치하세요."
            )

        self.lblWarningText.setText("DROWSINESS ALERT!\nWAKE UP!")
        self.lblWarningText.setAlignment(Qt.AlignCenter)

    def resizeEvent(self, event):
        """창 크기가 바뀌면 카메라와 오버레이 위치를 다시 계산한다."""
        super().resizeEvent(event)
        if all(
            hasattr(self, name)
            for name in (
                "cameraFrame",
                "lblRoadCamera",
                "lblDriverCamera",
                "warningFrame",
                "txtStmLog",
            )
        ):
            self.update_camera_geometry()

    def update_camera_geometry(self):
        """전방 화면 크기와 모든 오버레이 위젯의 위치를 조절한다."""
        frame_width = max(1, self.cameraFrame.width())
        frame_height = max(1, self.cameraFrame.height())
        margin = 20

        self.lblRoadCamera.setGeometry(0, 0, frame_width, frame_height)

        # 운전자 카메라: 오른쪽 아래, 4:3 비율
        driver_width = max(200, min(int(frame_width * 0.28), 400))
        driver_width = min(driver_width, max(1, frame_width - 2 * margin))
        driver_height = int(driver_width * 3 / 4)
        if driver_height > frame_height - 2 * margin:
            driver_height = max(1, frame_height - 2 * margin)
            driver_width = int(driver_height * 4 / 3)

        driver_x = max(0, frame_width - driver_width - margin)
        driver_y = max(0, frame_height - driver_height - margin)
        self.lblDriverCamera.setGeometry(
            driver_x, driver_y, driver_width, driver_height
        )

        # 졸음 경고창: 화면 중앙
        warning_width = max(300, min(int(frame_width * 0.38), 550))
        warning_height = max(120, min(int(frame_height * 0.22), 220))
        warning_width = min(warning_width, max(1, frame_width - 2 * margin))
        warning_height = min(warning_height, max(1, frame_height - 2 * margin))
        warning_x = max(0, (frame_width - warning_width) // 2)
        warning_y = max(0, (frame_height - warning_height) // 2)
        self.warningFrame.setGeometry(
            warning_x, warning_y, warning_width, warning_height
        )

        # STM 로그창: 왼쪽 아래
        log_width = max(260, min(int(frame_width * 0.30), 450))
        log_height = max(100, min(int(frame_height * 0.18), 180))
        log_width = min(log_width, max(1, frame_width - 2 * margin))
        log_height = min(log_height, max(1, frame_height - 2 * margin))
        log_x = min(margin, max(0, frame_width - log_width))
        log_y = max(0, frame_height - log_height - margin)
        self.txtStmLog.setGeometry(log_x, log_y, log_width, log_height)

        # 전방 화면은 가장 뒤에 두고 나머지 위젯을 앞으로 올린다.
        self.lblRoadCamera.lower()
        self.warningFrame.raise_()
        self.txtStmLog.raise_()
        self.lblDriverCamera.raise_()

    def report_camera_status(self):
        """실제로 적용된 카메라 설정을 터미널과 로그창에 출력한다."""
        if self.road_cap.isOpened():
            width = int(self.road_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.road_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.road_cap.get(cv2.CAP_PROP_FPS)
            message = (
                f"전방 카메라 연결됨: /dev/video{ROAD_CAMERA_INDEX} "
                f"{width}x{height}, {fps:.1f} FPS"
            )
            print(f"[ROAD] {message}")
            self.add_stm_log(message)
        else:
            message = f"전방 카메라 연결 실패: /dev/video{ROAD_CAMERA_INDEX}"
            print(f"[ERROR] {message}")
            self.lblRoadCamera.setText(message)
            self.add_stm_log(message)

        if self.driver_cap.isOpened():
            width = int(self.driver_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.driver_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self.driver_cap.get(cv2.CAP_PROP_FPS)
            message = (
                f"운전자 카메라 연결됨: /dev/video{DRIVER_CAMERA_INDEX} "
                f"{width}x{height}, {fps:.1f} FPS"
            )
            print(f"[DRIVER] {message}")
            self.add_stm_log(message)
        else:
            message = f"운전자 카메라 연결 실패: /dev/video{DRIVER_CAMERA_INDEX}"
            print(f"[ERROR] {message}")
            self.lblDriverCamera.setText(message)
            self.add_stm_log(message)

    def add_stm_log(self, message):
        """사용자의 스크롤 위치를 보존하며 시간과 로그를 추가한다."""
        current_time = time.strftime("%H:%M:%S")
        scroll_bar = self.txtStmLog.verticalScrollBar()
        was_at_bottom = scroll_bar.value() >= scroll_bar.maximum() - 2

        self.txtStmLog.appendPlainText(f"[{current_time}] {message}")

        if was_at_bottom:
            scroll_bar.setValue(scroll_bar.maximum())

    def drowsiness_alarm(self):
        """졸음 경고가 유지되는 동안 터미널 알람을 반복한다."""
        while self.drowsy_alarm_running:
            print("[ALARM] Wake up sir!")
            time.sleep(1.5)

    def yawn_alarm(self):
        print("[ALARM] Take some fresh air sir!")

    def show_drowsiness_warning(self):
        """눈을 2초 동안 감으면 졸음 경고창을 표시한다."""
        if self.warning_active:
            return

        self.warning_active = True
        self.eyes_open_since = None
        self.recovery_blink_since = None
        self.warningFrame.show()
        self.warningFrame.raise_()
        self.lblDriverCamera.raise_()

        if not self.drowsy_alarm_running:
            self.drowsy_alarm_running = True
            Thread(target=self.drowsiness_alarm, daemon=True).start()

        self.add_stm_log("졸음 경고")
        print("[WARNING] Drowsiness warning shown")

    def hide_drowsiness_warning(self):
        """회복 상태가 5초 동안 유지되면 졸음 경고창을 숨긴다."""
        self.warning_active = False
        self.warningFrame.hide()
        self.drowsy_alarm_running = False
        self.eyes_closed_since = None
        self.eyes_open_since = None
        self.recovery_blink_since = None

        self.add_stm_log("졸음 경고 해제")
        print("[INFO] Drowsiness warning hidden")

    def update_eye_state(self, ear):
        """2초 눈 감음과 깜빡임을 허용하는 5초 회복 조건을 처리한다."""
        current_time = time.monotonic()
        eyes_closed = ear < EYE_AR_THRESH

        # 경고가 아직 발생하지 않은 경우에는 2초 연속 눈 감음을 측정한다.
        if not self.warning_active:
            self.eyes_open_since = None
            self.recovery_blink_since = None

            if eyes_closed:
                if self.eyes_closed_since is None:
                    self.eyes_closed_since = current_time

                closed_duration = current_time - self.eyes_closed_since
                if closed_duration >= EYES_CLOSED_WARNING_SECONDS:
                    self.show_drowsiness_warning()
            else:
                self.eyes_closed_since = None

            return

        # 경고가 켜진 뒤에는 5초 회복 시간을 측정한다.
        self.eyes_closed_since = None

        if not eyes_closed:
            # 눈을 다시 뜨면 깜빡임 시간 측정을 끝낸다.
            self.recovery_blink_since = None

            if self.eyes_open_since is None:
                self.eyes_open_since = current_time

            open_duration = current_time - self.eyes_open_since
            if open_duration >= EYES_OPEN_RELEASE_SECONDS:
                self.hide_drowsiness_warning()
        else:
            # 회복 중 눈을 감으면 짧은 깜빡임인지 시간을 측정한다.
            if self.recovery_blink_since is None:
                self.recovery_blink_since = current_time

            blink_duration = current_time - self.recovery_blink_since

            # 0.5초 이하의 깜빡임은 허용한다.
            # 0.5초를 넘겨 감으면 5초 회복 시간을 처음부터 다시 측정한다.
            if blink_duration > RECOVERY_BLINK_GRACE_SECONDS:
                self.eyes_open_since = None

    def update_frames(self):
        """두 카메라의 영상을 읽고 각각의 QLabel에 표시한다."""
        if self.road_cap.isOpened():
            road_ok, road_frame = self.road_cap.read()
            if road_ok:
                self.show_frame(self.lblRoadCamera, road_frame)

        if self.driver_cap.isOpened():
            driver_ok, driver_frame = self.driver_cap.read()
            if driver_ok:
                driver_frame = cv2.flip(driver_frame, 1)
                driver_frame = self.process_driver_frame(driver_frame)
                self.show_frame(self.lblDriverCamera, driver_frame)

    def process_driver_frame(self, frame):
        """운전자 얼굴 랜드마크, 졸음, 하품을 감지한다."""
        frame = cv2.resize(frame, (DRIVER_WIDTH, DRIVER_HEIGHT))
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb_frame)

        if not results.multi_face_landmarks:
            # 얼굴이 보이지 않는 상태를 눈을 뜬 것으로 처리하지 않는다.
            # 기존 경고는 얼굴이 돌아오고 회복 조건을 만족할 때까지 유지한다.
            self.eyes_closed_since = None
            self.eyes_open_since = None
            self.recovery_blink_since = None
            self.yawn_active = False

            cv2.putText(
                frame,
                "NO FACE",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return frame

        face_landmarks = results.multi_face_landmarks[0]
        landmarks = np.array(
            [
                (int(point.x * DRIVER_WIDTH), int(point.y * DRIVER_HEIGHT))
                for point in face_landmarks.landmark
            ],
            dtype=np.int32,
        )

        left_eye = landmarks[LEFT_EYE_IDX]
        right_eye = landmarks[RIGHT_EYE_IDX]
        ear = (calculate_ear(left_eye) + calculate_ear(right_eye)) / 2.0

        top_lip = landmarks[MOUTH_TOP_BOTTOM[0]]
        bottom_lip = landmarks[MOUTH_TOP_BOTTOM[1]]
        lip_distance = abs(top_lip[1] - bottom_lip[1])

        cv2.polylines(frame, [left_eye], True, (0, 255, 0), 2)
        cv2.polylines(frame, [right_eye], True, (0, 255, 0), 2)
        self.mp_drawing.draw_landmarks(
            image=frame,
            landmark_list=face_landmarks,
            connections=self.mp_face_mesh.FACEMESH_LIPS,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.lip_drawing_spec,
        )
        cv2.line(frame, tuple(top_lip), tuple(bottom_lip), (0, 0, 255), 2)

        # 프레임 수가 아닌 실제 시간을 기준으로 졸음 상태를 판단한다.
        self.update_eye_state(ear)

        if self.warning_active:
            cv2.putText(
                frame,
                "DROWSINESS ALERT!",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if lip_distance > YAWN_THRESH:
            cv2.putText(
                frame,
                "YAWN ALERT!",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 165, 255),
                2,
                cv2.LINE_AA,
            )

            if not self.yawn_active:
                self.yawn_active = True
                Thread(target=self.yawn_alarm, daemon=True).start()
        else:
            self.yawn_active = False

        cv2.putText(
            frame,
            f"EAR: {ear:.2f}",
            (DRIVER_WIDTH - 180, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"YAWN: {lip_distance:.2f}",
            (DRIVER_WIDTH - 180, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return frame

    def show_frame(self, label, frame):
        """OpenCV BGR 영상을 변환해 지정한 QLabel에 표시한다."""
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_frame.shape
        bytes_per_line = channel * width

        q_image = QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()

        pixmap = QPixmap.fromImage(q_image).scaled(
            label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        label.setPixmap(pixmap)

    def closeEvent(self, event):
        """GUI가 닫힐 때 카메라와 MediaPipe 자원을 해제한다."""
        self.drowsy_alarm_running = False

        if hasattr(self, "timer"):
            self.timer.stop()
        if hasattr(self, "road_cap"):
            self.road_cap.release()
        if hasattr(self, "driver_cap"):
            self.driver_cap.release()
        if hasattr(self, "face_mesh"):
            self.face_mesh.close()

        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Form()
    window.show()
    sys.exit(app.exec_())
