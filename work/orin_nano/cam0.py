import time
import cv2
import numpy as np
import mediapipe as mp
from scipy.spatial import distance as dist

DRIVER_WIDTH = 640
DRIVER_HEIGHT = 480

EYE_AR_THRESH = 0.25
EYES_CLOSED_WARNING_SECONDS = 2.0
EYES_OPEN_RELEASE_SECONDS = 5.0
RECOVERY_BLINK_GRACE_SECONDS = 0.5
YAWN_THRESH = 25

LEFT_EYE_IDX = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]
MOUTH_TOP_BOTTOM = (13, 14)

# Global 자원 및 상태 변수
_mp_face_mesh = None
_face_mesh = None
_mp_drawing = None
_lip_drawing_spec = None

_eyes_closed_since = None
_eyes_open_since = None
_recovery_blink_since = None


# ==========================================
# [CAM0 PUBLIC FUNCTIONS]
# ==========================================
def cam0_init():
    """cam0 자원 초기화 함수"""
    global _mp_face_mesh, _face_mesh, _mp_drawing, _lip_drawing_spec
    _mp_face_mesh = mp.solutions.face_mesh
    _mp_drawing = mp.solutions.drawing_utils
    _face_mesh = _mp_face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    _lip_drawing_spec = _mp_drawing.DrawingSpec(
        color=(0, 255, 255), thickness=2, circle_radius=1
    )

def cam0_calculate_ear(eye_points):
    """EAR(눈 종횡비) 계산 함수"""
    vertical_1 = dist.euclidean(eye_points[1], eye_points[5])
    vertical_2 = dist.euclidean(eye_points[2], eye_points[4])
    horizontal = dist.euclidean(eye_points[0], eye_points[3])
    if horizontal == 0:
        return 0.0
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

def cam0_update_eye_state(ear, is_warning_active):
    """졸음 및 회복 시간 판별 함수"""
    global _eyes_closed_since, _eyes_open_since, _recovery_blink_since
    current_time = time.monotonic()
    eyes_closed = ear < EYE_AR_THRESH
    trigger_warning = False
    release_warning = False

    if not is_warning_active:
        _eyes_open_since = None
        _recovery_blink_since = None
        if eyes_closed:
            if _eyes_closed_since is None:
                _eyes_closed_since = current_time
            if current_time - _eyes_closed_since >= EYES_CLOSED_WARNING_SECONDS:
                trigger_warning = True
        else:
            _eyes_closed_since = None
    else:
        _eyes_closed_since = None
        if not eyes_closed:
            _recovery_blink_since = None
            if _eyes_open_since is None:
                _eyes_open_since = current_time
            if current_time - _eyes_open_since >= EYES_OPEN_RELEASE_SECONDS:
                release_warning = True
        else:
            if _recovery_blink_since is None:
                _recovery_blink_since = current_time
            if current_time - _recovery_blink_since > RECOVERY_BLINK_GRACE_SECONDS:
                _eyes_open_since = None

    return trigger_warning, release_warning

def cam0_process_frame(frame, is_warning_active):
    """운전자 프레임 처리 및 인식 함수"""
    global _eyes_closed_since, _eyes_open_since, _recovery_blink_since
    
    frame = cv2.resize(frame, (DRIVER_WIDTH, DRIVER_HEIGHT))
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = _face_mesh.process(rgb_frame)

    status_info = {
        "no_face": False,
        "trigger_drowsy": False,
        "release_drowsy": False,
        "yawn_detected": False
    }

    if not results.multi_face_landmarks:
        _eyes_closed_since = None
        _eyes_open_since = None
        _recovery_blink_since = None
        status_info["no_face"] = True

        cv2.putText(frame, "NO FACE", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
        return frame, status_info

    face_landmarks = results.multi_face_landmarks[0]
    landmarks = np.array([
        (int(point.x * DRIVER_WIDTH), int(point.y * DRIVER_HEIGHT))
        for point in face_landmarks.landmark
    ], dtype=np.int32)

    left_eye = landmarks[LEFT_EYE_IDX]
    right_eye = landmarks[RIGHT_EYE_IDX]
    ear = (cam0_calculate_ear(left_eye) + cam0_calculate_ear(right_eye)) / 2.0

    top_lip = landmarks[MOUTH_TOP_BOTTOM[0]]
    bottom_lip = landmarks[MOUTH_TOP_BOTTOM[1]]
    lip_distance = abs(top_lip[1] - bottom_lip[1])

    cv2.polylines(frame, [left_eye], True, (0, 255, 0), 2)
    cv2.polylines(frame, [right_eye], True, (0, 255, 0), 2)
    _mp_drawing.draw_landmarks(
        image=frame,
        landmark_list=face_landmarks,
        connections=_mp_face_mesh.FACEMESH_LIPS,
        landmark_drawing_spec=None,
        connection_drawing_spec=_lip_drawing_spec,
    )
    cv2.line(frame, tuple(top_lip), tuple(bottom_lip), (0, 0, 255), 2)

    trigger, release = cam0_update_eye_state(ear, is_warning_active)
    status_info["trigger_drowsy"] = trigger
    status_info["release_drowsy"] = release

    if is_warning_active or trigger:
        cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    if lip_distance > YAWN_THRESH:
        status_info["yawn_detected"] = True
        cv2.putText(frame, "YAWN ALERT!", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2, cv2.LINE_AA)

    cv2.putText(frame, f"EAR: {ear:.2f}", (DRIVER_WIDTH - 180, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"YAWN: {lip_distance:.2f}", (DRIVER_WIDTH - 180, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    return frame, status_info

def cam0_close():
    """cam0 자원 해제 함수"""
    global _face_mesh
    if _face_mesh is not None:
        _face_mesh.close()