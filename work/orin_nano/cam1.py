import time
import cv2
import numpy as np
import threading
from ultralytics import YOLO

# [CONSTANTS]
CAM1_FRAME_WIDTH = 1280
CAM1_FRAME_HEIGHT = 720

DEFAULT_ROI_RATIOS = np.array([
    [0.32, 0.85],  # Top-Left
    [0.68, 0.85],  # Top-Right
    [0.95, 0.95],  # Bottom-Right
    [0.05, 0.95]   # Bottom-Left
], dtype=np.float32)

CLASS_VEHICLE = 0
CLASS_TUNNEL_ENTRANCE = 1
CLASS_TUNNEL_EXIT = 2


class LaneDetector:
    """차선 검출 및 가변 ROI 계산 클래스"""
    def __init__(self):
        self.prev_slopes_bs = None
        self.debug_edges_frame = None

    def detect_rois(self, frame, y_top_ratio=0.85, y_bottom_ratio=0.95):
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 130])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array([12, 40, 80])
        upper_yellow = np.array([32, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        color_mask = cv2.bitwise_or(mask_white, mask_yellow)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        combined = cv2.bitwise_and(edges, color_mask)

        search_roi_pts = np.array([
            [int(w * 0.05), int(h * 0.98)],
            [int(w * 0.20), int(h * 0.55)],
            [int(w * 0.80), int(h * 0.55)],
            [int(w * 0.95), int(h * 0.98)]
        ], np.int32)

        roi_mask = np.zeros_like(combined)
        cv2.fillPoly(roi_mask, [search_roi_pts], 255)
        masked_edges = cv2.bitwise_and(combined, roi_mask)
        
        self.debug_edges_frame = masked_edges.copy()

        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=25, minLineLength=25, maxLineGap=110)
        if lines is None:
            self.prev_slopes_bs = None
            return None

        left_slopes, left_bs = [], []
        right_slopes, right_bs = [], []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1)
            b = y1 - slope * x1

            if -1.8 < slope < -0.2:
                left_slopes.append(slope)
                left_bs.append(b)
            elif 0.2 < slope < 1.8:
                right_slopes.append(slope)
                right_bs.append(b)

        if not left_slopes or not right_slopes:
            self.prev_slopes_bs = None
            return None

        curr_il_slope, curr_il_b = np.mean(left_slopes), np.mean(left_bs)
        curr_ir_slope, curr_ir_b = np.mean(right_slopes), np.mean(right_bs)

        if self.prev_slopes_bs is None:
            il_slope, il_b, ir_slope, ir_b = curr_il_slope, curr_il_b, curr_ir_slope, curr_ir_b
        else:
            alpha = 0.35
            il_slope = alpha * curr_il_slope + (1 - alpha) * self.prev_slopes_bs[0]
            il_b = alpha * curr_il_b + (1 - alpha) * self.prev_slopes_bs[1]
            ir_slope = alpha * curr_ir_slope + (1 - alpha) * self.prev_slopes_bs[2]
            ir_b = alpha * curr_ir_b + (1 - alpha) * self.prev_slopes_bs[3]

        y_top = int(h * y_top_ratio)
        y_bottom = int(h * y_bottom_ratio)

        try:
            x_il_top = int((y_top - il_b) / il_slope)
            x_il_bot = int((y_bottom - il_b) / il_slope)

            x_ir_top = int((y_top - ir_b) / ir_slope)
            x_ir_bot = int((y_bottom - ir_b) / ir_slope)

            # 위변 최소 간격 제한
            min_lane_width_top = int(w * 0.15)
            if (x_ir_top - x_il_top) < min_lane_width_top:
                center_x = (x_il_top + x_ir_top) // 2
                x_il_top = center_x - (min_lane_width_top // 2)
                x_ir_top = center_x + (min_lane_width_top // 2)

            # 위변 X 좌표 위치 제한
            x_il_top = max(int(w * 0.1), min(x_il_top, int(w * 0.4)))
            x_ir_top = max(int(w * 0.6), min(x_ir_top, int(w * 0.9)))

            edge_margin = int(w * 0.05)
            
            if (x_il_top <= edge_margin or x_il_top >= w - edge_margin or
                x_ir_top <= edge_margin or x_ir_top >= w - edge_margin or
                x_il_bot <= edge_margin or x_il_bot >= w - edge_margin or
                x_ir_bot <= edge_margin or x_ir_bot >= w - edge_margin or
                x_il_top >= x_ir_top or x_il_bot >= x_ir_bot):
                
                self.prev_slopes_bs = None
                return None

            self.prev_slopes_bs = (il_slope, il_b, ir_slope, ir_b)

            center_pts = np.array([
                [x_il_top, y_top],
                [x_ir_top, y_top],
                [x_ir_bot, y_bottom],
                [x_il_bot, y_bottom]
            ], np.int32)

            lane_width_top = x_ir_top - x_il_top
            lane_width_bot = x_ir_bot - x_il_bot

            offset_top = int(lane_width_top * 0.75)
            offset_bot = int(lane_width_bot * 0.75)

            left_pts = np.array([
                [max(0, x_il_top - offset_top), y_top],
                [x_il_top, y_top],
                [x_il_bot, y_bottom],
                [max(0, x_il_bot - offset_bot), y_bottom]
            ], np.int32)

            right_pts = np.array([
                [x_ir_top, y_top],
                [min(w, x_ir_top + offset_top), y_top],
                [min(w, x_ir_bot + offset_bot), y_bottom],
                [x_ir_bot, y_bottom]
            ], np.int32)

            return center_pts, left_pts, right_pts

        except ZeroDivisionError:
            pass

        self.prev_slopes_bs = None
        return None

    def reset(self):
        self.prev_slopes_bs = None
        self.debug_edges_frame = None


class Cam1Thread(threading.Thread):
    def __init__(self, cam_id=2, engine_path="yolo11n.engine"):
        super().__init__()
        self.cam_id = cam_id
        self.engine_path = engine_path
        self.running = False
        
        self.tracked_roi_vehicles = {}
        self.tracked_tunnels = {}
        
        self.MAX_MISS_VEHICLE = 20  
        self.MAX_MISS_TUNNEL = 30   

        self.processed_frame = None
        self.debug_frame = None
        self.warning_triggered = False
        
        self.tunnel_entrance_detected = False
        self.tunnel_exit_detected = False
        
        self.lock = threading.Lock()
        self.model = None
        self.lane_detector = LaneDetector()

    def run(self):
        try:
            print(f"[YOLO] Loading TensorRT Engine: {self.engine_path}")
            self.model = YOLO(self.engine_path, task='detect')
        except Exception as e:
            print(f"[YOLO Error] Failed to load TensorRT engine: {e}")

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

            rois = self.lane_detector.detect_rois(frame, y_top_ratio=0.85, y_bottom_ratio=0.95)
            
            if rois is not None:
                center_roi, left_roi, right_roi = rois
                is_lane_detected = True
            else:
                center_roi = (DEFAULT_ROI_RATIOS * [width, height]).astype(np.int32)
                left_roi = None
                right_roi = None
                is_lane_detected = False

            display_frame = frame.copy()
            current_frame_roi_track_ids = set()
            current_frame_tunnel_track_ids = set()

            if self.model is not None:
                results = self.model.track(
                    frame, 
                    persist=True, 
                    tracker="bytetrack.yaml", 
                    classes=[CLASS_VEHICLE, CLASS_TUNNEL_ENTRANCE, CLASS_TUNNEL_EXIT],
                    conf=0.45, 
                    verbose=False, 
                    imgsz=320
                )[0]

                if results.boxes is not None and len(results.boxes) > 0:
                    boxes = results.boxes.xyxy.cpu().numpy()
                    cls_ids = results.boxes.cls.int().cpu().numpy()
                    confs = results.boxes.conf.cpu().numpy()
                    
                    if results.boxes.id is not None:
                        track_ids = results.boxes.id.int().cpu().numpy()
                    else:
                        track_ids = [-1] * len(boxes)

                    for box, track_id, cls_id, conf in zip(boxes, track_ids, cls_ids, confs):
                        x1, y1, x2, y2 = map(int, box)

                        if cls_id in (CLASS_TUNNEL_ENTRANCE, CLASS_TUNNEL_EXIT):
                            effective_id = track_id if track_id != -1 else f"tunnel_{cls_id}_{x1}_{y1}"
                            current_frame_tunnel_track_ids.add(effective_id)
                            
                            self.tracked_tunnels[effective_id] = {
                                "miss_count": 0,
                                "cls_id": cls_id,
                                "last_box": (x1, y1, x2, y2),
                                "last_conf": conf
                            }

                        elif cls_id == CLASS_VEHICLE:
                            vehicle_bottom_center = (int((x1 + x2) / 2), y2)

                            in_center = cv2.pointPolygonTest(center_roi, vehicle_bottom_center, False) >= 0
                            in_left = cv2.pointPolygonTest(left_roi, vehicle_bottom_center, False) >= 0 if left_roi is not None else False
                            in_right = cv2.pointPolygonTest(right_roi, vehicle_bottom_center, False) >= 0 if right_roi is not None else False

                            if in_center or in_left or in_right:
                                effective_id = track_id if track_id != -1 else f"temp_{x1}_{y1}"
                                current_frame_roi_track_ids.add(effective_id)
                                
                                self.tracked_roi_vehicles[effective_id] = {
                                    "miss_count": 0,
                                    "last_box": (x1, y1, x2, y2),
                                    "last_conf": conf
                                }

            expired_tunnel_ids = []
            has_entrance = False
            has_exit = False

            for track_id, info in self.tracked_tunnels.items():
                cls_id = info["cls_id"]

                if track_id in current_frame_tunnel_track_ids:
                    if cls_id == CLASS_TUNNEL_ENTRANCE:
                        has_entrance = True
                    elif cls_id == CLASS_TUNNEL_EXIT:
                        has_exit = True
                else:
                    info["miss_count"] += 1
                    if info["miss_count"] <= self.MAX_MISS_TUNNEL:
                        if cls_id == CLASS_TUNNEL_ENTRANCE:
                            has_entrance = True
                        elif cls_id == CLASS_TUNNEL_EXIT:
                            has_exit = True
                    else:
                        expired_tunnel_ids.append(track_id)

            for track_id in expired_tunnel_ids:
                del self.tracked_tunnels[track_id]

            expired_vehicle_ids = []
            for track_id, info in self.tracked_roi_vehicles.items():
                if track_id not in current_frame_roi_track_ids:
                    info["miss_count"] += 1
                    if info["miss_count"] > self.MAX_MISS_VEHICLE:
                        expired_vehicle_ids.append(track_id)

            for track_id in expired_vehicle_ids:
                del self.tracked_roi_vehicles[track_id]

            final_warning_flag = len(self.tracked_roi_vehicles) > 0

            # ROI 시각화
            if is_lane_detected:
                color_roi = (0, 0, 255) if final_warning_flag else (0, 255, 0)
                cv2.polylines(display_frame, [center_roi], isClosed=True, color=color_roi, thickness=2)
                if left_roi is not None:
                    cv2.polylines(display_frame, [left_roi], isClosed=True, color=(255, 200, 0), thickness=1)
                if right_roi is not None:
                    cv2.polylines(display_frame, [right_roi], isClosed=True, color=(255, 200, 0), thickness=1)
            else:
                color_default = (0, 0, 255) if final_warning_flag else (255, 255, 0)
                cv2.polylines(display_frame, [center_roi], isClosed=True, color=color_default, thickness=2)

            with self.lock:
                self.processed_frame = display_frame
                self.debug_frame = self.lane_detector.debug_edges_frame
                self.warning_triggered = final_warning_flag
                
                self.tunnel_entrance_detected = has_entrance
                self.tunnel_exit_detected = has_exit

        cap.release()


# Global 스레드 객체
_cam1_thread = None


# =========================================================
# [main.py 호환용 PUBLIC FUNCTIONS]
# =========================================================

def cam1_start(cam_id=2, engine_path="yolo11n.engine"):
    """main.py에서 스레드를 개시할 때 호출"""
    global _cam1_thread
    if _cam1_thread is None or not _cam1_thread.is_alive():
        _cam1_thread = Cam1Thread(cam_id=cam_id, engine_path=engine_path)
        _cam1_thread.start()

def cam1_get_frame():
    """main.py에서 도로 프레임을 가져올 때 호출"""
    if _cam1_thread is None:
        return None
    with _cam1_thread.lock:
        return _cam1_thread.processed_frame.copy() if _cam1_thread.processed_frame is not None else None

def cam1_get_status():
    """main.py에서 감지 상태 정보를 가져올 때 호출"""
    if _cam1_thread is None:
        return {
            "roi_warning": False,
            "tunnel_entrance": False,
            "tunnel_exit": False
        }
    with _cam1_thread.lock:
        return {
            "roi_warning": _cam1_thread.warning_triggered,
            "tunnel_entrance": _cam1_thread.tunnel_entrance_detected,
            "tunnel_exit": _cam1_thread.tunnel_exit_detected
        }

def cam1_stop():
    """main.py 종료 시 호출하여 자원 해제"""
    global _cam1_thread
    if _cam1_thread is not None:
        _cam1_thread.running = False
        _cam1_thread.join()
        _cam1_thread = None