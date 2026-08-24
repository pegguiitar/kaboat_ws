#!/usr/bin/env python3
"""카메라 연결 및 AprilTag/ArUco 실시간 검출 테스트 (수조 4면 전체 1m 실측 격자 로드).

특징:
  1. config/pool_calibration.yaml 에서 4개 변 1m 실측 점 자동 로드
  2. 1m x 1m 정밀 원근 격자망 렌더링 및 보트 실시간 위치/각도 표시
"""

import math
import os
import sys
import yaml
import cv2
import numpy as np

pool_size_x = 10.0
pool_size_y = 5.0

YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'src', 'kaboat_hardware', 'config', 'pool_calibration.yaml'
)

# 4개 변 기본값
bottom_x_pts = [[56.0 + i * (1246.0 - 56.0) / 10.0, 624.0] for i in range(11)]
right_y_pts = [[1246.0, 624.0 - i * (624.0 - 121.0) / 5.0] for i in range(6)]
top_x_pts = [[255.0 + i * (1246.0 - 255.0) / 10.0, 96.0 + i * (121.0 - 96.0) / 10.0] for i in range(11)]
left_y_pts = [[56.0 + i * (255.0 - 56.0) / 5.0, 624.0 - i * (624.0 - 96.0) / 5.0] for i in range(6)]

if os.path.exists(YAML_PATH):
    try:
        with open(YAML_PATH, 'r') as f:
            d = yaml.safe_load(f)
            if 'bottom_x_pts' in d:
                bottom_x_pts = d['bottom_x_pts']
            if 'right_y_pts' in d:
                right_y_pts = d['right_y_pts']
            if 'top_x_pts' in d:
                top_x_pts = d['top_x_pts']
            if 'left_y_pts' in d:
                left_y_pts = d['left_y_pts']
        print(f"✅ 4면 캘리브레이션 파일 로드 완료: {YAML_PATH}")
    except Exception as e:
        print("캘리브레이션 로드 실패:", e)


def get_curve_pt(norm_val, pts_list):
    idx_float = norm_val * (len(pts_list) - 1)
    i0 = int(math.floor(idx_float))
    i1 = min(i0 + 1, len(pts_list) - 1)
    t = idx_float - i0
    return (1.0 - t) * np.array(pts_list[i0], dtype=np.float64) + t * np.array(pts_list[i1], dtype=np.float64)


def coons_patch(u, v):
    c_bot = get_curve_pt(u, bottom_x_pts)
    c_top = get_curve_pt(u, top_x_pts)
    c_left = get_curve_pt(v, left_y_pts)
    c_right = get_curve_pt(v, right_y_pts)

    P0 = np.array(bottom_x_pts[0], dtype=np.float64)
    P1 = np.array(bottom_x_pts[-1], dtype=np.float64)
    P3 = np.array(top_x_pts[0], dtype=np.float64)
    P2 = np.array(top_x_pts[-1], dtype=np.float64)

    corner_blend = (1.0 - u) * (1.0 - v) * P0 + u * (1.0 - v) * P1 + (1.0 - u) * v * P3 + u * v * P2
    return (1.0 - v) * c_bot + v * c_top + (1.0 - u) * c_left + u * c_right - corner_blend


def pixel_to_pool_metric(target_px):
    u = 0.5
    v = 0.5
    eps = 1e-4
    for _ in range(12):
        pos = coons_patch(u, v)
        err = pos - target_px
        if np.linalg.norm(err) < 1e-3:
            break
        du = (coons_patch(u + eps, v) - coons_patch(u - eps, v)) / (2 * eps)
        dv = (coons_patch(u, v + eps) - coons_patch(u, v - eps)) / (2 * eps)
        J = np.column_stack([du, dv])
        delta = np.linalg.lstsq(J, -err, rcond=None)[0]
        u += delta[0]
        v += delta[1]
        u = np.clip(u, -0.3, 1.3)
        v = np.clip(v, -0.3, 1.3)
    return float(u * pool_size_x), float(v * pool_size_y)


def get_detector_params():
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()

    params.minMarkerPerimeterRate = 0.005
    params.maxMarkerPerimeterRate = 4.0
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 3
    params.adaptiveThreshConstant = 7.0

    if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(params, 'aprilTagQuadDecimate'):
        params.aprilTagQuadDecimate = 1.0
    if hasattr(params, 'aprilTagCriticalRad'):
        params.aprilTagCriticalRad = 0.1745
    if hasattr(params, 'aprilTagMinWhiteBlackDiff'):
        params.aprilTagMinWhiteBlackDiff = 5

    return params


def main():
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"카메라 /dev/video{device_idx} 연결 중...")

    cap = cv2.VideoCapture(device_idx)
    if not cap.isOpened():
        print(f"오류: /dev/video{device_idx} 를 열 수 없습니다.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    dict_families = {}
    if hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
        dict_families['tag36h11'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'DICT_APRILTAG_25h9'):
        dict_families['tag25h9'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_25h9) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    if hasattr(cv2.aruco, 'DICT_APRILTAG_16h5'):
        dict_families['tag16h5'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    if hasattr(cv2.aruco, 'DICT_4X4_50'):
        dict_families['ArUco_4x4'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    params = get_detector_params()

    window_name = "AprilTag Camera Test (4-Edge Calibrated Grid)"
    cv2.namedWindow(window_name)

    print("\n=======================================================")
    print(" AprilTag Live Test (4-Edge Calibrated Grid)")
    print(" Run 'python3 scripts/calibrate_pool_grid.py' to recalibrate.")
    print(" Press 'Q' on window to exit.")
    print("=======================================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_info = []

        for fam_name, adict in dict_families.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=params)
            if ids is not None and len(ids) > 0:
                for i, tid in enumerate(ids.flatten()):
                    detected_info.append(f"{fam_name} ID:{tid}")
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[tid]]))

                    c = corners[i][0]
                    u_center = float(c[:, 0].mean())
                    v_center = float(c[:, 1].mean())

                    x_pool, y_pool = pixel_to_pool_metric(np.array([u_center, v_center]))

                    u_fwd = float((c[0][0] + c[1][0]) / 2.0)
                    v_fwd = float((c[0][1] + c[1][1]) / 2.0)
                    x_fwd, y_fwd = pixel_to_pool_metric(np.array([u_fwd, v_fwd]))
                    yaw_deg = math.degrees(math.atan2(y_fwd - y_pool, x_fwd - x_pool))

                    p0_int = tuple(np.array(bottom_x_pts[0], dtype=int))
                    cv2.line(frame, p0_int, (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)
                    info_str = f"Pool [X:{x_pool:.2f}m, Y:{y_pool:.2f}m] Yaw:{yaw_deg:+.1f}deg"
                    cv2.putText(frame, info_str, (int(u_center) - 80, int(v_center) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        # ── 곡면 외곽선 및 격자망 ──
        t_samples = np.linspace(0.0, 1.0, 50)
        bot_pts = np.array([coons_patch(t, 0.0) for t in t_samples], dtype=np.int32)
        top_pts = np.array([coons_patch(t, 1.0) for t in t_samples], dtype=np.int32)
        left_pts = np.array([coons_patch(0.0, t) for t in t_samples], dtype=np.int32)
        right_pts = np.array([coons_patch(1.0, t) for t in t_samples], dtype=np.int32)

        cv2.polylines(frame, [bot_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [top_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [left_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [right_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)

        for gx in range(1, int(pool_size_x)):
            u_norm = gx / pool_size_x
            line_pts = np.array([coons_patch(u_norm, t) for t in t_samples], dtype=np.int32)
            cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 150, 80), thickness=1, lineType=cv2.LINE_AA)

        for gy in range(1, int(pool_size_y)):
            v_norm = gy / pool_size_y
            line_pts = np.array([coons_patch(t, v_norm) for t in t_samples], dtype=np.int32)
            cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 150, 80), thickness=1, lineType=cv2.LINE_AA)

        p0 = tuple(np.array(bottom_x_pts[0], dtype=int))
        cv2.circle(frame, p0, 16, (0, 215, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Origin (0,0)", (p0[0] - 20, p0[1] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

        pt_x_arrow = tuple(coons_patch(0.20, 0.0).astype(int))
        cv2.arrowedLine(frame, p0, pt_x_arrow, (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(frame, "+X (0->10m)", (pt_x_arrow[0] + 10, pt_x_arrow[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        pt_y_arrow = tuple(coons_patch(0.0, 0.25).astype(int))
        cv2.arrowedLine(frame, p0, pt_y_arrow, (0, 255, 0), 3, tipLength=0.2)
        cv2.putText(frame, "+Y (0->5m)", (pt_y_arrow[0] - 30, pt_y_arrow[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, "4-Edge 1m Grid Applied | Run 'python3 scripts/calibrate_pool_grid.py' to recalibrate",
                    (20, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1, cv2.LINE_AA)

        if detected_info:
            cv2.putText(frame, f"Detected: {', '.join(detected_info)}",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Searching for AprilTag / ArUco...",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
