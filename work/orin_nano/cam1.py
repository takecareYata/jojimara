import time
import cv2
import numpy as np
import threading
from ultralytics import YOLO

# [CONSTANTS]
CAM1_FRAME_WIDTH = 1280
CAM1_FRAME_HEIGHT = 720

# [CONSTANTS] 고정 사다리꼴 비율 (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
DEFAULT_ROI_RATIOS = np.array([
    [0.40, 0.85],  # Top-Left
    [0.60, 0.85],  # Top-Right
    [0.65, 0.95],  # Bottom-Right
    [0.35, 0.95]   # Bottom-Left
], dtype=np.float32)

# 양옆 평행사변형 너비 조절 비율 (기본 차선 폭의 0.75배 → 이 값을 변경하여 크기 조절 가능)
SIDE_ROI_OFFSET_RATIO = 0.90

CLASS_VEHICLE = 0
CLASS_TUNNEL_ENTRANCE = 1
CLASS_TUNNEL_EXIT = 2

CLASS_NAMES = {
    CLASS_VEHICLE: "Vehicle",
    CLASS_TUNNEL_ENTRANCE: "Tunnel Entrance",
    CLASS_TUNNEL_EXIT: "Tunnel Exit"
}


def check_bbox_roi_intersection_opencv(box_coords, roi_pts):
    """
    OpenCV를 사용하여 축정렬 바운딩 박스(BBox)와 다각형(ROI) 간의
    면적/선분 겹침 유무를 검사합니다.
    """
    if roi_pts is None:
        return False

    x1, y1, x2, y2 = box_coords

    # 1. 바운딩 박스 모서리 4개 점 중 하나라도 ROI 내부에 있는지 검사
    bbox_corners = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]
    if any(cv2.pointPolygonTest(roi_pts, pt, False) >= 0 for pt in bbox_corners):
        return True

    # 2. ROI 모서리 점 중 하나라도 바운딩 박스 내부에 있는지 검사 (ROI보다 차량이 클 때)
    for pt in roi_pts:
        rx, ry = pt[0], pt[1]
        if x1 <= rx <= x2 and y1 <= ry <= y2:
            return True

    # 3. 바운딩 박스 4개 변과 ROI 4개 변 사이의 선분 교차(Intersection) 검사
    roi_len = len(roi_pts)
    bbox_lines = [
        ((x1, y1), (x2, y1)),  # 상단 변
        ((x2, y1), (x2, y2)),  # 우측 변
        ((x2, y2), (x1, y2)),  # 하단 변
        ((x1, y2), (x1, y1)),  # 좌측 변
    ]

    def is_line_intersect(p1, p2, p3, p4):
        """두 선분 p1-p2와 p3-p4가 교차하는지 CCW 알고리즘으로 검사"""

        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (
            ccw(p1, p2, p3) != ccw(p1, p2, p4)
        )

    for b_p1, b_p2 in bbox_lines:
        for i in range(roi_len):
            r_p1 = tuple(roi_pts[i])
            r_p2 = tuple(roi_pts[(i + 1) % roi_len])
            if is_line_intersect(b_p1, b_p2, r_p1, r_p2):
                return True

    return False


class LaneDetector:
    """고정 비율 좌표 배열을 사용하여 고정 ROI(중앙, 좌, 우) 영역 계산"""
    def __init__(self):
        self.debug_edges_frame = None

    def get_default_rois(self, width, height, side_offset_ratio=SIDE_ROI_OFFSET_RATIO):
        center_pts = (DEFAULT_ROI_RATIOS * [width, height]).astype(np.int32)
        
        top_left, top_right, bot_right, bot_left = center_pts
        lane_width_top = top_right[0] - top_left[0]
        lane_width_bot = bot_right[0] - bot_left[0]

        # 설정된 side_offset_ratio 비율에 맞추어 양옆 평행사변형 크기 계산
        offset_top = int(lane_width_top * side_offset_ratio)
        offset_bot = int(lane_width_bot * side_offset_ratio)

        y_top = top_left[1]
        y_bottom = bot_left[1]

        left_pts = np.array([
            [max(0, top_left[0] - offset_top), y_top],
            [top_left[0], y_top],
            [bot_left[0], y_bottom],
            [max(0, bot_left[0] - offset_bot), y_bottom]
        ], np.int32)

        right_pts = np.array([
            [top_right[0], y_top],
            [min(width, top_right[0] + offset_top), y_top],
            [min(width, bot_right[0] + offset_bot), y_bottom],
            [bot_right[0], y_bottom]
        ], np.int32)

        return center_pts, left_pts, right_pts

    def detect_rois(self, frame, y_top_ratio=0.85, y_bottom_ratio=0.95):
        h, w = frame.shape[:2]
        return self.get_default_rois(w, h)

    def reset(self):
        self.debug_edges_frame = None


class Cam1Thread(threading.Thread):
    def __init__(self, cam_id=2, engine_path="yolo11n.engine"):
        super().__init__(daemon=True)
        self.cam_id = cam_id
        self.engine_path = engine_path
        self.running = False
        self.stop_event = threading.Event()
        
        self.tracked_roi_vehicles = {}
        self.tracked_tunnels = {}
        
        self.MAX_MISS_VEHICLE = 30  
        self.MAX_MISS_TUNNEL = 30   

        self.processed_frame = None
        self.debug_frame = None
        
        self.warning_left = False
        self.warning_center = False
        self.warning_right = False
        self.warning_triggered = False
        
        self.tunnel_entrance_detected = False
        self.tunnel_exit_detected = False
        
        self.lock = threading.Lock()
        self.model = None
        self.lane_detector = LaneDetector()

    def stop(self):
        self.running = False
        self.stop_event.set()

    def draw_bbox(self, frame, box, label, color, thickness=2):
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        
        (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, max(0, y1 - h - 6)), (x1 + w + 4, max(h + 6, y1)), color, -1)
        cv2.putText(frame, label, (x1 + 2, max(h + 2, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def run(self):
        cap = cv2.VideoCapture(self.cam_id, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM1_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM1_FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print(f"[CAM1 Error] Camera {self.cam_id}를 열 수 없습니다.")
            return

        if self.stop_event.is_set():
            cap.release()
            return

        try:
            print(f"[YOLO] Loading TensorRT Engine: {self.engine_path}")
            self.model = YOLO(self.engine_path, task='detect')
        except Exception as e:
            print(f"[YOLO Error] Failed to load TensorRT engine: {e}")

        if self.stop_event.is_set():
            cap.release()
            return

        self.running = True

        while self.running and not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            center_roi, left_roi, right_roi = self.lane_detector.detect_rois(frame)

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
                            box_coords = (x1, y1, x2, y2)

                            # 바운딩 박스 면적의 일부라도 ROI 영역과 겹치면 감지 (OpenCV 방식)
                            in_center = check_bbox_roi_intersection_opencv(box_coords, center_roi)
                            in_left = check_bbox_roi_intersection_opencv(box_coords, left_roi)
                            in_right = check_bbox_roi_intersection_opencv(box_coords, right_roi)

                            if in_center or in_left or in_right:
                                effective_id = track_id if track_id != -1 else f"temp_{x1}_{y1}"
                                current_frame_roi_track_ids.add(effective_id)

                                self.tracked_roi_vehicles[effective_id] = {
                                    "miss_count": 0,
                                    "last_box": (x1, y1, x2, y2),
                                    "last_conf": conf,
                                    "in_center": in_center,
                                    "in_left": in_left,
                                    "in_right": in_right
                                }

            # [1] 터널 상태 업데이트 및 시각화
            expired_tunnel_ids = []
            has_entrance = False
            has_exit = False

            for track_id, info in self.tracked_tunnels.items():
                cls_id = info["cls_id"]
                box = info["last_box"]
                conf = info["last_conf"]

                if track_id in current_frame_tunnel_track_ids:
                    info["miss_count"] = 0
                else:
                    info["miss_count"] += 1

                if info["miss_count"] <= self.MAX_MISS_TUNNEL:
                    if cls_id == CLASS_TUNNEL_ENTRANCE:
                        has_entrance = True
                        color = (0, 165, 255)
                    else:
                        has_exit = True
                        color = (255, 255, 0)
                    
                    label_text = f"{CLASS_NAMES.get(cls_id, 'Tunnel')} {conf:.2f}"
                    self.draw_bbox(display_frame, box, label_text, color)
                else:
                    expired_tunnel_ids.append(track_id)

            for track_id in expired_tunnel_ids:
                del self.tracked_tunnels[track_id]

            # [2] 차량 상태 업데이트 및 시각화
            expired_vehicle_ids = []
            flag_left = False
            flag_center = False
            flag_right = False

            for track_id, info in self.tracked_roi_vehicles.items():
                box = info["last_box"]
                conf = info["last_conf"]

                if track_id in current_frame_roi_track_ids:
                    info["miss_count"] = 0
                else:
                    info["miss_count"] += 1

                if info["miss_count"] <= self.MAX_MISS_VEHICLE:
                    in_danger = info["in_left"] or info["in_center"] or info["in_right"]
                    if info["in_left"]: flag_left = True
                    if info["in_center"]: flag_center = True
                    if info["in_right"]: flag_right = True

                    color = (0, 0, 255) if in_danger else (255, 100, 0)
                    label_text = f"Vehicle {conf:.2f}"
                    self.draw_bbox(display_frame, box, label_text, color)
                else:
                    expired_vehicle_ids.append(track_id)

            for track_id in expired_vehicle_ids:
                del self.tracked_roi_vehicles[track_id]

            # [3] ROI 영역별 시각화
            color_center = (0, 0, 255) if flag_center else (0, 255, 0)
            cv2.polylines(display_frame, [center_roi], isClosed=True, color=color_center, thickness=2)

            if left_roi is not None:
                color_left = (0, 0, 255) if flag_left else (255, 200, 0)
                cv2.polylines(display_frame, [left_roi], isClosed=True, color=color_left, thickness=1 if not flag_left else 2)

            if right_roi is not None:
                color_right = (0, 0, 255) if flag_right else (255, 200, 0)
                cv2.polylines(display_frame, [right_roi], isClosed=True, color=color_right, thickness=1 if not flag_right else 2)

            with self.lock:
                self.processed_frame = display_frame
                self.debug_frame = None
                
                self.warning_left = flag_left
                self.warning_center = flag_center
                self.warning_right = flag_right
                self.warning_triggered = flag_left or flag_center or flag_right
                
                self.tunnel_entrance_detected = has_entrance
                self.tunnel_exit_detected = has_exit

        cap.release()
        self.running = False


_cam1_thread = None

def cam1_start(cam_id=2, engine_path="yolo11n.engine"):
    global _cam1_thread
    if _cam1_thread is None or not _cam1_thread.is_alive():
        _cam1_thread = Cam1Thread(cam_id=cam_id, engine_path=engine_path)
        _cam1_thread.start()

def cam1_get_frame(include_debug=True):
    if _cam1_thread is None:
        return None, None
    with _cam1_thread.lock:
        main_f = _cam1_thread.processed_frame.copy() if _cam1_thread.processed_frame is not None else None
        debug_f = (
            _cam1_thread.debug_frame.copy()
            if include_debug and _cam1_thread.debug_frame is not None
            else None
        )
        return main_f, debug_f

def cam1_get_status():
    if _cam1_thread is None:
        return {
            "roi_warning": False,
            "roi_warning_left": False,
            "roi_warning_center": False,
            "roi_warning_right": False,
            "tunnel_entrance": False,
            "tunnel_exit": False
        }
    with _cam1_thread.lock:
        return {
            "roi_warning": _cam1_thread.warning_triggered,
            "roi_warning_left": _cam1_thread.warning_left,
            "roi_warning_center": _cam1_thread.warning_center,
            "roi_warning_right": _cam1_thread.warning_right,
            "tunnel_entrance": _cam1_thread.tunnel_entrance_detected,
            "tunnel_exit": _cam1_thread.tunnel_exit_detected
        }

def cam1_stop():
    global _cam1_thread
    if _cam1_thread is not None:
        _cam1_thread.stop()
        _cam1_thread.join(timeout=3.0)
        _cam1_thread = None