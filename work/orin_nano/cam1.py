import cv2
import numpy as np
import threading
import time

# ==========================================
# [CONSTANTS / CONFIGURATION]
# ==========================================
CAM1_DEFAULT_CAM_ID = 1
CAM1_FRAME_WIDTH = 1280
CAM1_FRAME_HEIGHT = 720

# ROI 가이드라인 비율 ('ㅈ' 형태 사다리꼴 좌표)
CAM1_ROI_TOP_LEFT_X_RATIO = 0.18 
CAM1_ROI_TOP_LEFT_Y_RATIO = 0.75
CAM1_ROI_TOP_RIGHT_X_RATIO = 0.82 
CAM1_ROI_TOP_RIGHT_Y_RATIO = 0.75
CAM1_ROI_BOTTOM_RIGHT_X_RATIO = 0.90
CAM1_ROI_BOTTOM_RIGHT_Y_RATIO = 0.95
CAM1_ROI_BOTTOM_LEFT_X_RATIO = 0.10
CAM1_ROI_BOTTOM_LEFT_Y_RATIO = 0.95


class Cam(threading.Thread):
    def __init__(self, cam_id=CAM1_DEFAULT_CAM_ID):
        super().__init__()
        self.cam_id = cam_id
        self.running = False
        self.frame = None
        self.warning_triggered = False
        self.lock = threading.Lock()

    def run(self):
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM1_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM1_FRAME_HEIGHT)

        if not cap.isOpened():
            print(f"[CAM1 Error] Camera {self.cam_id}를 열 수 없습니다.")
            return

        self.running = True

        while self.running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            height, width = frame.shape[:2]

            # 'ㅈ' 형태 ROI 가이드라인 좌표 계산
            roi_pts = np.array([
                [int(width * CAM1_ROI_TOP_LEFT_X_RATIO), int(height * CAM1_ROI_TOP_LEFT_Y_RATIO)],
                [int(width * CAM1_ROI_TOP_RIGHT_X_RATIO), int(height * CAM1_ROI_TOP_RIGHT_Y_RATIO)],
                [int(width * CAM1_ROI_BOTTOM_RIGHT_X_RATIO), int(height * CAM1_ROI_BOTTOM_RIGHT_Y_RATIO)],
                [int(width * CAM1_ROI_BOTTOM_LEFT_X_RATIO), int(height * CAM1_ROI_BOTTOM_LEFT_Y_RATIO)]
            ], np.int32)

            # 'ㅈ' 형태 가이드라인 시각화 (청록색)
            cv2.polylines(frame, [roi_pts], isClosed=True, color=(255, 255, 0), thickness=2)

            with self.lock:
                self.frame = frame

        cap.release()

    def cam1_get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def cam1_stop(self):
        self.running = False


def cam1_func(cam_id=CAM1_DEFAULT_CAM_ID):
    thread_obj = Cam(cam_id=cam_id)
    return thread_obj