import time

import cv2
import mediapipe as mp
import numpy as np
from scipy.spatial import distance as dist


# 운전자 카메라 처리 해상도
DRIVER_WIDTH = 640
DRIVER_HEIGHT = 480

# 졸음 및 하품 판단 기준
EYE_AR_THRESH = 0.25
EYES_CLOSED_WARNING_SECONDS = 2.0
EYES_OPEN_RELEASE_SECONDS = 5.0
RECOVERY_BLINK_GRACE_SECONDS = 0.5
YAWN_THRESH = 25

# MediaPipe Face Mesh 랜드마크 번호
LEFT_EYE_IDX = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]
MOUTH_TOP_BOTTOM = (13, 14)

# MediaPipe 자원
_mp_face_mesh = None
_face_mesh = None
_mp_drawing = None
_lip_drawing_spec = None

# 졸음 판단용 시간 상태
_eyes_closed_since = None
_eyes_open_since = None
_recovery_blink_since = None


def cam0_init():
    """운전자 감지에 필요한 MediaPipe 자원을 초기화한다."""
    global _mp_face_mesh, _face_mesh, _mp_drawing, _lip_drawing_spec
    global _eyes_closed_since, _eyes_open_since, _recovery_blink_since

    _eyes_closed_since = None
    _eyes_open_since = None
    _recovery_blink_since = None

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
        color=(0, 255, 255),
        thickness=2,
        circle_radius=1,
    )


def cam0_calculate_ear(eye_points):
    """6개의 눈 좌표로 EAR(눈 종횡비)을 계산한다."""
    vertical_1 = dist.euclidean(eye_points[1], eye_points[5])
    vertical_2 = dist.euclidean(eye_points[2], eye_points[4])
    horizontal = dist.euclidean(eye_points[0], eye_points[3])

    if horizontal == 0:
        return 0.0

    return (vertical_1 + vertical_2) / (2.0 * horizontal)


def cam0_update_eye_state(ear, is_warning_active):
    """눈 감김 2초와 경고 후 눈 뜸 누적 5초를 판별한다."""
    global _eyes_closed_since, _eyes_open_since, _recovery_blink_since

    current_time = time.monotonic()
    eyes_closed = ear < EYE_AR_THRESH
    trigger_warning = False
    release_warning = False

    if not is_warning_active:
        # 경고 발생 전에는 눈을 연속으로 감은 시간을 측정한다.
        _eyes_open_since = None
        _recovery_blink_since = None

        if eyes_closed:
            if _eyes_closed_since is None:
                _eyes_closed_since = current_time

            if (
                current_time - _eyes_closed_since
                >= EYES_CLOSED_WARNING_SECONDS
            ):
                trigger_warning = True
        else:
            _eyes_closed_since = None

    else:
        # 경고 발생 후에는 눈을 뜬 누적 시간을 측정한다.
        # 0.5초 이내의 짧은 깜빡임은 회복 시간을 초기화하지 않는다.
        _eyes_closed_since = None

        if not eyes_closed:
            _recovery_blink_since = None

            if _eyes_open_since is None:
                _eyes_open_since = current_time

            if (
                current_time - _eyes_open_since
                >= EYES_OPEN_RELEASE_SECONDS
            ):
                release_warning = True
        else:
            if _recovery_blink_since is None:
                _recovery_blink_since = current_time

            if (
                current_time - _recovery_blink_since
                > RECOVERY_BLINK_GRACE_SECONDS
            ):
                _eyes_open_since = None

    return trigger_warning, release_warning


def cam0_process_frame(frame, is_warning_active):
    """운전자 영상을 처리하고 화면과 감지 상태를 반환한다."""
    global _eyes_closed_since, _eyes_open_since, _recovery_blink_since

    if _face_mesh is None:
        raise RuntimeError("cam0_init()을 먼저 호출해야 합니다.")

    frame = cv2.resize(frame, (DRIVER_WIDTH, DRIVER_HEIGHT))
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = _face_mesh.process(rgb_frame)

    status_info = {
        "no_face": False,
        "trigger_drowsy": False,
        "release_drowsy": False,
        "yawn_detected": False,
    }

    if not results.multi_face_landmarks:
        _eyes_closed_since = None
        _eyes_open_since = None
        _recovery_blink_since = None
        status_info["no_face"] = True

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
        return frame, status_info

    face_landmarks = results.multi_face_landmarks[0]
    landmarks = np.array(
        [
            (
                int(point.x * DRIVER_WIDTH),
                int(point.y * DRIVER_HEIGHT),
            )
            for point in face_landmarks.landmark
        ],
        dtype=np.int32,
    )

    left_eye = landmarks[LEFT_EYE_IDX]
    right_eye = landmarks[RIGHT_EYE_IDX]
    ear = (
        cam0_calculate_ear(left_eye)
        + cam0_calculate_ear(right_eye)
    ) / 2.0

    top_lip = landmarks[MOUTH_TOP_BOTTOM[0]]
    bottom_lip = landmarks[MOUTH_TOP_BOTTOM[1]]
    lip_distance = abs(top_lip[1] - bottom_lip[1])

    # 눈과 입 외곽선을 화면에 표시한다.
    cv2.polylines(frame, [left_eye], True, (0, 255, 0), 2)
    cv2.polylines(frame, [right_eye], True, (0, 255, 0), 2)
    _mp_drawing.draw_landmarks(
        image=frame,
        landmark_list=face_landmarks,
        connections=_mp_face_mesh.FACEMESH_LIPS,
        landmark_drawing_spec=None,
        connection_drawing_spec=_lip_drawing_spec,
    )
    cv2.line(
        frame,
        tuple(top_lip),
        tuple(bottom_lip),
        (0, 0, 255),
        2,
    )

    trigger, release = cam0_update_eye_state(
        ear,
        is_warning_active,
    )
    status_info["trigger_drowsy"] = trigger
    status_info["release_drowsy"] = release

    if is_warning_active or trigger:
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
        status_info["yawn_detected"] = True
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

    return frame, status_info


def cam0_close():
    """MediaPipe 자원을 해제한다."""
    global _face_mesh

    if _face_mesh is not None:
        _face_mesh.close()
        _face_mesh = None

