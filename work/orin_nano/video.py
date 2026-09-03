import time
import cv2
import numpy as np
import threading
from ultralytics import YOLO

# [CONSTANTS]
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
    """차선 검출 및 가변 ROI 계산을 담당하는 클래스"""
    def __init__(self):
        self.prev_slopes_bs = None
        self.debug_edges_frame = None

    def detect_rois(self, frame, y_top_ratio=0.85, y_bottom_ratio=0.95):
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 흰색 차선 범위 (조명 조건에 맞춰 조정 가능)
        lower_white = np.array([0, 0, 130])
        upper_white = np.array([180, 50, 255])
        # lower_white_strict = np.array([0, 0, 200]) # [Backup] 엄격한 흰색 마스크 조건
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 노란색 차선 범위
        lower_yellow = np.array([12, 40, 80])
        upper_yellow = np.array([32, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        color_mask = cv2.bitwise_or(mask_white, mask_yellow)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny Edge 검출
        edges = cv2.Canny(blur, 50, 150)
        # edges = cv2.Canny(blur, 70, 210) # [Backup] 높은 임계값 설정
        
        combined = cv2.bitwise_and(edges, color_mask)

        # 차선 탐색 영역(Search ROI) 설정
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

        # 허프 변환을 통한 직선 검출
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=25, minLineLength=25, maxLineGap=110)
        # lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=40, minLineLength=40, maxLineGap=80) # [Backup] 엄격한 파라미터

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

            # 기울기 필터링 (좌/우 차선 분리)
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

        # 지수 이동 평균(EMA)을 활용한 보정
        if self.prev_slopes_bs is None:
            il_slope, il_b, ir_slope, ir_b = curr_il_slope, curr_il_b, curr_ir_slope, curr_ir_b
        else:
            alpha = 0.35
            # alpha = 0.20 # [Backup] 더 부드러운 필터링 사용 시
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

            # 평행사변형/사다리꼴 위변(Top)의 위치 한계 설정
            min_lane_width_top = int(w * 0.15)
            if (x_ir_top - x_il_top) < min_lane_width_top:
                center_x = (x_il_top + x_ir_top) // 2
                x_il_top = center_x - (min_lane_width_top // 2)
                x_ir_top = center_x + (min_lane_width_top // 2)

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


class VideoProcessingThread(threading.Thread):
    def __init__(self, video_path="test_video.mp4", engine_path="yolo11n.engine"):
        super().__init__()
        self.video_path = video_path
        self.engine_path = engine_path
        self.running = False
        
        self.is_paused = False
        self.step_frame = False
        self.seek_offset = 0
        
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
            print(f"[YOLO] Loading TensorRT Custom Engine: {self.engine_path}")
            self.model = YOLO(self.engine_path, task='detect')
            print(f"[YOLO] Loaded Custom Classes: {self.model.names}")
        except Exception as e:
            print(f"[YOLO Error] Failed to load TensorRT engine: {e}")

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print(f"[Video Error] 비디오 파일 '{self.video_path}'를 열 수 없습니다.")
            return

        self.running = True

        while self.running:
            if self.seek_offset != 0:
                curr_pos = cap.get(cv2.CAP_PROP_POS_FRAMES)
                new_pos = max(0, curr_pos + self.seek_offset)
                cap.set(cv2.CAP_PROP_POS_FRAMES, new_pos)
                self.seek_offset = 0

            if self.is_paused and not self.step_frame:
                time.sleep(0.03)
                continue

            self.step_frame = False

            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.tracked_roi_vehicles.clear()
                self.tracked_tunnels.clear()
                self.lane_detector.reset()
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
                    # conf=0.30, # [Backup] 더 낮은 신뢰도 사용 시
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

                        # [Debug] 원본 바운딩 박스 그리기
                        # cv2.rectangle(display_frame, (x1, y1), (x2, y2), (255, 0, 0), 1)

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

            # [1] 터널 상태 업데이트 및 주석 처리된 시각화 로직
            for track_id, info in self.tracked_tunnels.items():
                x1, y1, x2, y2 = info["last_box"]
                conf = info["last_conf"]
                cls_id = info["cls_id"]
                
                label_prefix = "TUNNEL ENTRANCE" if cls_id == CLASS_TUNNEL_ENTRANCE else "TUNNEL EXIT"
                box_color = (0, 255, 0) if cls_id == CLASS_TUNNEL_ENTRANCE else (255, 255, 0)

                if track_id in current_frame_tunnel_track_ids:
                    # =========================================================================
                    # 터널 입구/출구 박스 및 신뢰도/클래스 이름 텍스트 그리기 비활성화
                    # =========================================================================
                    # cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
                    # cv2.putText(display_frame, f"{label_prefix} {conf:.2f}", (x1, max(y1 - 10, 20)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    
                    if cls_id == CLASS_TUNNEL_ENTRANCE:
                        has_entrance = True
                    elif cls_id == CLASS_TUNNEL_EXIT:
                        has_exit = True
                else:
                    info["miss_count"] += 1
                    if info["miss_count"] <= self.MAX_MISS_TUNNEL:
                        # =========================================================================
                        # 터널 검출 일시 누락(HOLDING) 상태의 주황색 박스 및 텍스트 비활성화
                        # =========================================================================
                        # cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 140, 255), 1, cv2.LINE_AA)
                        # cv2.putText(display_frame, f"{label_prefix} HOLD ({info['miss_count']}/{self.MAX_MISS_TUNNEL})", 
                        #             (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 255), 1)
                        
                        if cls_id == CLASS_TUNNEL_ENTRANCE:
                            has_entrance = True
                        elif cls_id == CLASS_TUNNEL_EXIT:
                            has_exit = True
                    else:
                        expired_tunnel_ids.append(track_id)

            for track_id in expired_tunnel_ids:
                del self.tracked_tunnels[track_id]

            # [2] 차량 상태 업데이트 및 주석 처리된 시각화 로직
            expired_vehicle_ids = []
            for track_id, info in self.tracked_roi_vehicles.items():
                x1, y1, x2, y2 = info["last_box"]
                conf = info["last_conf"]

                if track_id in current_frame_roi_track_ids:
                    # =========================================================================
                    # ROI 내부 감지 차량의 빨간색 바운딩 박스 및 WARNING 텍스트 비활성화
                    # =========================================================================
                    # id_label = f"ID:{track_id}" if not str(track_id).startswith("temp_") else "VEHICLE"
                    # cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # cv2.putText(display_frame, f"{id_label} WARNING {conf:.2f}", (x1, max(y1 - 10, 20)),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    pass
                else:
                    info["miss_count"] += 1
                    if info["miss_count"] <= self.MAX_MISS_VEHICLE:
                        # =========================================================================
                        # 차량 검출 일시 누락(HOLDING) 상태의 주황색 박스 및 텍스트 비활성화
                        # =========================================================================
                        # cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 140, 255), 1, cv2.LINE_AA)
                        # cv2.putText(display_frame, f"HOLDING ({info['miss_count']}/{self.MAX_MISS_VEHICLE})", 
                        #             (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 255), 1)
                        pass
                    else:
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

            if self.is_paused:
                cv2.putText(display_frame, "PAUSED (Space: Resume, D: 1 Step)", (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            with self.lock:
                self.processed_frame = display_frame
                self.debug_frame = self.lane_detector.debug_edges_frame
                self.warning_triggered = final_warning_flag
                
                self.tunnel_entrance_detected = has_entrance
                self.tunnel_exit_detected = has_exit

            time.sleep(0.01)

        cap.release()


# Global thread handle
_video_thread = None

def video_start(video_path="test_video.mp4", engine_path="yolo11n.engine"):
    global _video_thread
    if _video_thread is None or not _video_thread.is_alive():
        _video_thread = VideoProcessingThread(video_path=video_path, engine_path=engine_path)
        _video_thread.start()

def video_get_data():
    if _video_thread is None:
        return None, None, {
            "roi_warning": False, 
            "tunnel_entrance": False, 
            "tunnel_exit": False
        }
    with _video_thread.lock:
        frame = _video_thread.processed_frame.copy() if _video_thread.processed_frame is not None else None
        debug_frame = _video_thread.debug_frame.copy() if _video_thread.debug_frame is not None else None
        status = {
            "roi_warning": _video_thread.warning_triggered,
            "tunnel_entrance": _video_thread.tunnel_entrance_detected,
            "tunnel_exit": _video_thread.tunnel_exit_detected
        }
        return frame, debug_frame, status

def video_toggle_pause():
    if _video_thread is not None:
        _video_thread.is_paused = not _video_thread.is_paused

def video_step_frame():
    if _video_thread is not None:
        _video_thread.step_frame = True

def video_seek(frames):
    if _video_thread is not None:
        _video_thread.seek_offset += frames

def video_stop():
    global _video_thread
    if _video_thread is not None:
        _video_thread.running = False
        _video_thread.join()
        _video_thread = None


# ==========================================
# [MAIN EXECUTABLE]
# ==========================================
# if __name__ == "__main__":
#     TEST_VIDEO_PATH = "test_video.mp4" 
#     TEST_ENGINE_PATH = "yolo11n.engine" 

#     print(f"[VIDEO] Starting Test with Custom Engine: {TEST_ENGINE_PATH}")
#     print(" [Controls] Space: Pause/Resume | D: Next Frame | A: -30 Frames | S: +30 Frames | Q: Quit")
    
#     video_start(video_path=TEST_VIDEO_PATH, engine_path=TEST_ENGINE_PATH)

#     try:
#         while True:
#             frame, debug_frame, status = video_get_data()

#             if frame is not None:
#                 status_text = (
#                     f"VEHICLE: {status['roi_warning']} | "
#                     f"ENTRANCE: {status['tunnel_entrance']} | "
#                     f"EXIT: {status['tunnel_exit']}"
#                 )
#                 cv2.putText(frame, status_text, (20, 40),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if status['roi_warning'] else (0, 255, 0), 2)

#                 cv2.imshow("Video Processing Multi-ROI Test", frame)

#             if debug_frame is not None:
#                 cv2.imshow("Debug - Detected Lane Edges", debug_frame)
#                 # pass

#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q'):
#                 break
#             elif key == ord(' '):
#                 video_toggle_pause()
#             elif key == ord('d'):
#                 video_step_frame()
#             elif key == ord('a'):
#                 video_seek(-30)
#             elif key == ord('s'):
#                 video_seek(30)

#             time.sleep(0.01)

#     except KeyboardInterrupt:
#         pass

#     finally:
#         video_stop()
#         cv2.destroyAllWindows()
#         print("[VIDEO] Processing finished.")