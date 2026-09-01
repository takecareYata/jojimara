import time
import cv2
import numpy as np
import threading
from ultralytics import YOLO

# [CONSTANTS]
CAM1_FRAME_WIDTH = 1280
CAM1_FRAME_HEIGHT = 720

CAM1_ROI_TOP_LEFT_X_RATIO = 0.18 
CAM1_ROI_TOP_LEFT_Y_RATIO = 0.75
CAM1_ROI_TOP_RIGHT_X_RATIO = 0.82 
CAM1_ROI_TOP_RIGHT_Y_RATIO = 0.75
CAM1_ROI_BOTTOM_RIGHT_X_RATIO = 0.90
CAM1_ROI_BOTTOM_RIGHT_Y_RATIO = 0.95
CAM1_ROI_BOTTOM_LEFT_X_RATIO = 0.10
CAM1_ROI_BOTTOM_LEFT_Y_RATIO = 0.95

CLASS_VEHICLE = 0
CLASS_TUNNEL_ENTRANCE = 1
CLASS_TUNNEL_EXIT = 2


class Cam1Thread(threading.Thread):
    def __init__(self, cam_id=2, engine_path="yolo11n.engine"):
        super().__init__()
        self.cam_id = cam_id
        self.engine_path = engine_path
        self.running = False
        
        self.processed_frame = None
        self.warning_triggered = False
        self.tunnel_detected = False
        self.lock = threading.Lock()
        self.model = None

    def run(self):
        try:
            print(f"[YOLO] Loading TensorRT Engine: {self.engine_path}")
            self.model = YOLO(self.engine_path, task='detect')
            print("[YOLO] Model loaded successfully.")
        except Exception as e:
            print(f"[YOLO Error] Failed to load model: {e}")

        cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM1_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM1_FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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

            roi_pts = np.array([
                [int(width * CAM1_ROI_TOP_LEFT_X_RATIO), int(height * CAM1_ROI_TOP_LEFT_Y_RATIO)],
                [int(width * CAM1_ROI_TOP_RIGHT_X_RATIO), int(height * CAM1_ROI_TOP_RIGHT_Y_RATIO)],
                [int(width * CAM1_ROI_BOTTOM_RIGHT_X_RATIO), int(height * CAM1_ROI_BOTTOM_RIGHT_Y_RATIO)],
                [int(width * CAM1_ROI_BOTTOM_LEFT_X_RATIO), int(height * CAM1_ROI_BOTTOM_LEFT_Y_RATIO)]
            ], np.int32)

            display_frame = frame.copy()
            roi_warning_flag = False
            tunnel_flag = False

            if self.model is not None:
                results = self.model(frame, verbose=False, conf=0.5, imgsz=320)[0]
                
                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    if cls_id in [CLASS_TUNNEL_ENTRANCE, CLASS_TUNNEL_EXIT]:
                        tunnel_flag = True
                        label_text = "Tunnel Entrance" if cls_id == CLASS_TUNNEL_ENTRANCE else "Tunnel Exit"
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(display_frame, f"{label_text} {conf:.2f}", (x1, max(y1 - 10, 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    elif cls_id == CLASS_VEHICLE:
                        vehicle_bottom_center = (int((x1 + x2) / 2), y2)
                        is_inside = cv2.pointPolygonTest(roi_pts, vehicle_bottom_center, False)

                        if is_inside >= 0:
                            roi_warning_flag = True
                            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                            cv2.putText(display_frame, f"WARNING VEHICLE {conf:.2f}", (x1, max(y1 - 10, 20)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

            line_color = (0, 0, 255) if roi_warning_flag else (255, 255, 0)
            cv2.polylines(display_frame, [roi_pts], isClosed=True, color=line_color, thickness=2)

            with self.lock:
                self.processed_frame = display_frame
                self.warning_triggered = roi_warning_flag
                self.tunnel_detected = tunnel_flag

        cap.release()


# Global 스레드 객체
_cam1_thread = None

# ==========================================
# [CAM1 PUBLIC FUNCTIONS]
# ==========================================
def cam1_start(cam_id=2, engine_path="yolo11n.engine"):
    """cam1 스레드 시작 함수"""
    global _cam1_thread
    _cam1_thread = Cam1Thread(cam_id=cam_id, engine_path=engine_path)
    _cam1_thread.start()

def cam1_get_frame():
    """cam1 프레임 가져오기 함수"""
    if _cam1_thread is None:
        return None
    with _cam1_thread.lock:
        return _cam1_thread.processed_frame.copy() if _cam1_thread.processed_frame is not None else None

def cam1_get_status():
    """cam1 상태 정보 가져오기 함수"""
    if _cam1_thread is None:
        return {"roi_warning": False, "tunnel_detected": False}
    with _cam1_thread.lock:
        return {
            "roi_warning": _cam1_thread.warning_triggered,
            "tunnel_detected": _cam1_thread.tunnel_detected
        }

def cam1_stop():
    """cam1 스레드 중지 함수"""
    global _cam1_thread
    if _cam1_thread is not None:
        _cam1_thread.running = False
        _cam1_thread.join()
        _cam1_thread = None