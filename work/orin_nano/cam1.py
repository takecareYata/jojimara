import threading
import time

import cv2
import numpy as np
from ultralytics import YOLO


# 전방 카메라 설정
CAM1_FRAME_WIDTH = 1280
CAM1_FRAME_HEIGHT = 720

# 옆 차량 경고 영역(ROI) 비율
CAM1_ROI_TOP_LEFT_X_RATIO = 0.18
CAM1_ROI_TOP_LEFT_Y_RATIO = 0.75
CAM1_ROI_TOP_RIGHT_X_RATIO = 0.82
CAM1_ROI_TOP_RIGHT_Y_RATIO = 0.75
CAM1_ROI_BOTTOM_RIGHT_X_RATIO = 0.90
CAM1_ROI_BOTTOM_RIGHT_Y_RATIO = 0.95
CAM1_ROI_BOTTOM_LEFT_X_RATIO = 0.10
CAM1_ROI_BOTTOM_LEFT_Y_RATIO = 0.95

# 학습된 YOLO 클래스 번호
CLASS_VEHICLE = 0
CLASS_TUNNEL_ENTRANCE = 1
CLASS_TUNNEL_EXIT = 2


class Cam1Thread(threading.Thread):
    """전방 카메라 수집과 YOLO 추론을 담당하는 스레드다."""

    def __init__(
        self,
        video_source=2,
        engine_path="yolo11n.engine",
    ):
        super().__init__(daemon=True)
        # 정수이면 USB 카메라 번호, 문자열이면 영상 파일 경로다.
        self.video_source = video_source
        self.engine_path = engine_path
        self.running = False

        self.processed_frame = None
        self.warning_triggered = False
        self.tunnel_type = None

        #영상 이동
        #양수는 앞으로, 음수는 뒤로 이동
        self.seek_request_seconds = 0.0

        self.lock = threading.Lock()
        self.model = None

    def request_seek(self, seconds):
        """영상 파일의 재생 위치 이동을 요청한다."""

        # USB 카메라는 재생 위치를 이동할 수 없다.
        if isinstance(self.video_source, int):
            return False

        # 방향키를 빠르게 여러 번 누르면 이동 시간을 누적한다.
        with self.lock:
            self.seek_request_seconds += seconds

        return True
    
    def run(self):
        try:
            print(
                f"[YOLO] TensorRT 엔진을 불러옵니다: "
                f"{self.engine_path}"
            )
            self.model = YOLO(self.engine_path, task="detect")
            print("[YOLO] 모델을 정상적으로 불러왔습니다.")
        except Exception as error:
            print(f"[YOLO 오류] 모델 로드 실패: {error}")

        using_camera = isinstance(self.video_source, int)

        if using_camera:
            # USB 카메라 번호가 전달된 경우 기존 V4L2 방식으로 연다.
            cap = cv2.VideoCapture(
                self.video_source,
                cv2.CAP_V4L2,
            )
            cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG"),
            )
            cap.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                CAM1_FRAME_WIDTH,
            )
            cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                CAM1_FRAME_HEIGHT,
            )
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            # 문자열 경로가 전달된 경우 MP4 영상 파일을 연다.
            cap = cv2.VideoCapture(str(self.video_source))

        if not cap.isOpened():
            print(
                f"[CAM1 오류] 영상 입력 {self.video_source}을(를) "
                "열 수 없습니다."
            )
            return

        # 파일 영상은 원본 FPS에 맞춰 재생한다.
        source_fps = cap.get(cv2.CAP_PROP_FPS)
        if source_fps <= 0 or source_fps > 120:
            source_fps = 30.0
        frame_interval = 1.0 / source_fps

        if using_camera:
            print(
                f"[CAM1] USB 카메라 사용: "
                f"/dev/video{self.video_source}"
            )
        else:
            print(
                f"[CAM1] 영상 파일 사용: {self.video_source} "
                f"({source_fps:.1f} FPS)"
            )

        self.running = True

        while self.running:
            # ==========================================
            # MP4 영상 재생 위치 이동 처리
            # ==========================================
            if not using_camera:
                with self.lock:
                    seek_seconds = self.seek_request_seconds
                    self.seek_request_seconds = 0.0

                if seek_seconds != 0:
                    # 현재 영상 위치를 밀리초 단위로 가져온다.
                    current_position_ms = cap.get(
                        cv2.CAP_PROP_POS_MSEC
                    )

                    # 영상의 전체 길이를 계산한다.
                    total_frame_count = cap.get(
                        cv2.CAP_PROP_FRAME_COUNT
                    )

                    if source_fps > 0:
                        video_duration_ms = (
                            total_frame_count
                            / source_fps
                            * 1000.0
                        )
                    else:
                        video_duration_ms = 0.0

                    # 이동할 위치 계산
                    target_position_ms = (
                        current_position_ms
                        + seek_seconds * 1000.0
                    )

                    # 영상 시작 위치보다 앞으로 갈 수 없게 제한
                    target_position_ms = max(
                        0.0,
                        target_position_ms
                    )

                    # 영상의 마지막 위치를 넘지 않게 제한
                    if video_duration_ms > 0:
                        target_position_ms = min(
                            target_position_ms,
                            max(
                                0.0,
                                video_duration_ms
                                - frame_interval * 1000.0
                            )
                        )

                    # 실제 영상 위치 변경
                    cap.set(
                        cv2.CAP_PROP_POS_MSEC,
                        target_position_ms
                    )

            frame_started_at = time.monotonic()
            ret, frame = cap.read()

            if not ret:
                if not using_camera:
                    # 영상 파일이 끝나면 첫 프레임으로 돌아간다.
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    time.sleep(0.01)
                    continue

                time.sleep(0.01)
                continue

            height, width = frame.shape[:2]
            roi_points = np.array(
                [
                    [
                        int(width * CAM1_ROI_TOP_LEFT_X_RATIO),
                        int(height * CAM1_ROI_TOP_LEFT_Y_RATIO),
                    ],
                    [
                        int(width * CAM1_ROI_TOP_RIGHT_X_RATIO),
                        int(height * CAM1_ROI_TOP_RIGHT_Y_RATIO),
                    ],
                    [
                        int(width * CAM1_ROI_BOTTOM_RIGHT_X_RATIO),
                        int(height * CAM1_ROI_BOTTOM_RIGHT_Y_RATIO),
                    ],
                    [
                        int(width * CAM1_ROI_BOTTOM_LEFT_X_RATIO),
                        int(height * CAM1_ROI_BOTTOM_LEFT_Y_RATIO),
                    ],
                ],
                dtype=np.int32,
            )

            display_frame = frame.copy()
            roi_warning = False
            tunnel_type = None
            best_tunnel_confidence = -1.0

            if self.model is not None:
                result = self.model(
                    frame,
                    verbose=False,
                    conf=0.5,
                    imgsz=320,
                )[0]

                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])

                    if class_id in (
                        CLASS_TUNNEL_ENTRANCE,
                        CLASS_TUNNEL_EXIT,
                    ):
                        # 한 프레임에 여러 터널 객체가 검출되면
                        # 신뢰도가 가장 높은 결과를 사용한다.
                        if confidence > best_tunnel_confidence:
                            best_tunnel_confidence = confidence
                            tunnel_type = (
                                "entrance"
                                if class_id == CLASS_TUNNEL_ENTRANCE
                                else "exit"
                            )

                        label_text = (
                            "Tunnel Entrance"
                            if class_id == CLASS_TUNNEL_ENTRANCE
                            else "Tunnel Exit"
                        )
                        cv2.rectangle(
                            display_frame,
                            (x1, y1),
                            (x2, y2),
                            (0, 255, 0),
                            2,
                        )
                        cv2.putText(
                            display_frame,
                            f"{label_text} {confidence:.2f}",
                            (x1, max(y1 - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )

                    elif class_id == CLASS_VEHICLE:
                        vehicle_bottom_center = (
                            int((x1 + x2) / 2),
                            y2,
                        )
                        is_inside = cv2.pointPolygonTest(
                            roi_points,
                            vehicle_bottom_center,
                            False,
                        )

                        if is_inside >= 0:
                            roi_warning = True
                            cv2.rectangle(
                                display_frame,
                                (x1, y1),
                                (x2, y2),
                                (0, 165, 255),
                                2,
                            )
                            cv2.putText(
                                display_frame,
                                f"WARNING VEHICLE {confidence:.2f}",
                                (x1, max(y1 - 10, 20)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 165, 255),
                                2,
                            )

            line_color = (
                (0, 0, 255)
                if roi_warning
                else (255, 255, 0)
            )
            cv2.polylines(
                display_frame,
                [roi_points],
                isClosed=True,
                color=line_color,
                thickness=2,
            )

            with self.lock:
                self.processed_frame = display_frame
                self.warning_triggered = roi_warning
                self.tunnel_type = tunnel_type

            if not using_camera:
                # 추론이 원본 영상 FPS보다 빠른 경우에만 기다린다.
                processing_time = (
                    time.monotonic() - frame_started_at
                )
                remaining_time = frame_interval - processing_time

                if remaining_time > 0:
                    time.sleep(remaining_time)

        cap.release()


_cam1_thread = None


def cam1_start(
    video_source=2,
    engine_path="yolo11n.engine",
):
    """USB 카메라 또는 영상 파일 처리 스레드를 시작한다."""
    global _cam1_thread

    if _cam1_thread is not None:
        cam1_stop()

    _cam1_thread = Cam1Thread(
        video_source=video_source,
        engine_path=engine_path,
    )
    _cam1_thread.start()


def cam1_get_frame():
    """가장 최근에 처리된 전방 카메라 프레임을 반환한다."""
    if _cam1_thread is None:
        return None

    with _cam1_thread.lock:
        if _cam1_thread.processed_frame is None:
            return None
        return _cam1_thread.processed_frame.copy()


def cam1_get_status():
    """측면 차량 경고와 터널 종류를 반환한다."""
    if _cam1_thread is None:
        return {
            "roi_warning": False,
            "tunnel_type": None,
        }

    with _cam1_thread.lock:
        return {
            "roi_warning": _cam1_thread.warning_triggered,
            "tunnel_type": _cam1_thread.tunnel_type,
        }

def cam1_seek(seconds):
    """영상 파일을 지정한 시간만큼 이동한다."""
    if _cam1_thread is None:
        return False

    return _cam1_thread.request_seek(seconds)

def cam1_stop():
    """전방 카메라 처리 스레드를 종료한다."""
    global _cam1_thread

    if _cam1_thread is not None:
        _cam1_thread.running = False
        _cam1_thread.join(timeout=3.0)
        _cam1_thread = None
