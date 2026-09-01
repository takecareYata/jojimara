#!/usr/bin/env python3
import os
import sys
import time
import subprocess
from pathlib import Path
from threading import Thread

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow

import cv2
import cam1
import cam0

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

        self.warning_active = False
        self.drowsy_alarm_running = False
        self.yawn_active = False

        # cam0, cam1 모듈 함수를 통한 초기화 및 실행
        cam0.cam0_init()
        cam1.cam1_start(cam_id=ROAD_CAMERA_INDEX, engine_path="yolo11n.engine")

        # 운전자 카메라 (CAM0 영상 수집용)
        self.driver_cap = cv2.VideoCapture(DRIVER_CAMERA_INDEX, cv2.CAP_V4L2)
        self.driver_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.driver_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.driver_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.driver_cap.set(cv2.CAP_PROP_FPS, 30)
        self.driver_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.add_stm_log("시스템 시작 및 카메라 연결 완료")

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
        current_time = time.strftime("%H:%M:%S")
        self.txtStmLog.appendPlainText(f"[{current_time}] {message}")

    def drowsiness_alarm(self):
        while self.drowsy_alarm_running:
            print("[ALARM] Wake up sir!")
            time.sleep(1.5)

    def yawn_alarm(self):
        print("[ALARM] Take some fresh air sir!")

    def update_frames(self):
        # 1. cam1 모듈 함수 호출
        road_frame = cam1.cam1_get_frame()
        if road_frame is not None:
            road_status = cam1.cam1_get_status()
            if road_status["roi_warning"]:
                cv2.putText(road_frame, "STATUS: VEHICLE IN ROI!", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.show_frame(self.lblRoadCamera, road_frame)

        # 2. cam0 모듈 함수 호출
        if self.driver_cap.isOpened():
            ret, driver_frame = self.driver_cap.read()
            if ret:
                driver_frame = cv2.flip(driver_frame, 1)
                processed_driver, status = cam0.cam0_process_frame(driver_frame, self.warning_active)

                if status["trigger_drowsy"] and not self.warning_active:
                    self.warning_active = True
                    self.warningFrame.show()
                    self.drowsy_alarm_running = True
                    Thread(target=self.drowsiness_alarm, daemon=True).start()
                    self.add_stm_log("졸음 경고 발생")

                elif status["release_drowsy"] and self.warning_active:
                    self.warning_active = False
                    self.warningFrame.hide()
                    self.drowsy_alarm_running = False
                    self.add_stm_log("졸음 경고 해제")

                if status["yawn_detected"]:
                    if not self.yawn_active:
                        self.yawn_active = True
                        Thread(target=self.yawn_alarm, daemon=True).start()
                        self.add_stm_log("하품 감지")
                else:
                    self.yawn_active = False

                self.show_frame(self.lblDriverCamera, processed_driver)

    def show_frame(self, label, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        q_image = QImage(rgb_frame.data, w, h, ch * w, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(q_image).scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.drowsy_alarm_running = False
        self.timer.stop()
        
        # cam0, cam1 모듈 자원 해제 함수 호출
        cam1.cam1_stop()
        cam0.cam0_close()

        if hasattr(self, "driver_cap"):
            self.driver_cap.release()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DriverMonitoringSystem()
    window.show()
    sys.exit(app.exec_())