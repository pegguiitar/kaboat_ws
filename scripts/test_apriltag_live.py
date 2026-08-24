#!/usr/bin/env python3
"""카메라 연결 및 AprilTag/ArUco 실시간 검출 테스트 (곡면 서피스 보정 + 잘린 우하단 외삽 지원).

특징:
  1. 광각 렌즈 왜곡 및 수조 곡면 휨을 Coons Patch 곡면 서피스 모델로 정밀 보정
  2. 화면 밖으로 잘린 우하단 꼭짓점(P1)을 원근 기하학(소실점/교점)으로 자동 외삽/보간
  3. 곡면을 따라 자연스럽게 휘어지는 +X, +Y 축 및 곡면 원근 격자망(Curved Grid) 렌더링
  4. 'C' 키를 누르면 마우스 4점 클릭으로 현장에서 1초 만에 곡면 제어점 재보정 가능
"""

import math
import sys
import cv2
import numpy as np

# 수조 크기 [m]
pool_size_x = 10.0
pool_size_y = 5.0

# ── 수조 제어점 (곡면 휨 + 잘린 우하단 외삽) ────────────────
P0 = np.array([85.0, 640.0], dtype=np.float64)       # 좌하단 (0m, 0m)
P3 = np.array([200.0, 95.0], dtype=np.float64)       # 좌상단 (0m, 5m)
P2 = np.array([1160.0, 105.0], dtype=np.float64)     # 우상단 (10m, 5m)
P1 = np.array([1275.0, 650.0], dtype=np.float64)     # 우하단 (10m, 0m) [외삽됨]

M_bot = np.array([640.0, 715.0], dtype=np.float64)   # 하단 변 중심 (아래로 볼록)
M_top = np.array([640.0, 78.0], dtype=np.float64)    # 상단 변 중심 (위로 볼록)
M_left = np.array([100.0, 360.0], dtype=np.float64)  # 좌측 변 중심
M_right = np.array([1240.0, 360.0], dtype=np.float64)# 우측 변 중심

calib_mode = False
calib_step = 0


def extrapolate_missing_corner():
    global P1, M_right, M_left
    vec_left = P0 - P3
    dir_right = np.array([-vec_left[0] * 1.05, vec_left[1]])
    P1 = np.array([P2[0] + dir_right[0], P0[1] + 10.0], dtype=np.float64)
    if P1[0] < 1250:
        P1[0] = 1275.0
    if P1[1] < 640:
        P1[1] = 650.0
    M_right = (P1 + P2) / 2.0
    M_left = (P0 + P3) / 2.0


extrapolate_missing_corner()


def quad_bezier(A, M, B, t):
    return (1.0 - t)**2 * A + 2.0 * (1.0 - t) * t * M + t**2 * B


def coons_patch(u, v):
    c_bot = quad_bezier(P0, M_bot, P1, u)
    c_top = quad_bezier(P3, M_top, P2, u)
    c_left = quad_bezier(P0, M_left, P3, v)
    c_right = quad_bezier(P1, M_right, P2, v)
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


def on_mouse_click(event, x, y, flags, param):
    global calib_mode, calib_step, P0, M_bot, P2, P3
    if event == cv2.EVENT_LBUTTONDOWN and calib_mode:
        pt = np.array([float(x), float(y)], dtype=np.float64)
        if calib_step == 0:
            P0 = pt
            print(f"📍 1. 좌하단(P0) 설정: ({x}, {y})")
            calib_step += 1
        elif calib_step == 1:
            M_bot = pt
            print(f"📍 2. 하단 중간(M_bot) 설정: ({x}, {y})")
            calib_step += 1
        elif calib_step == 2:
            P2 = pt
            print(f"📍 3. 우상단(P2) 설정: ({x}, {y})")
            calib_step += 1
        elif calib_step == 3:
            P3 = pt
            print(f"📍 4. 좌상단(P3) 설정: ({x}, {y})")
            extrapolate_missing_corner()
            calib_mode = False
            calib_step = 0
            print(f"🎉 [곡면 캘리브레이션 완료!] 우하단 외삽: {P1.astype(int)}")


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
    global calib_mode, calib_step
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"카메라 장치 /dev/video{device_idx} 연결 시도 중...")

    cap = cv2.VideoCapture(device_idx)
    if not cap.isOpened():
        print(f"오류: /dev/video{device_idx} 를 열 수 없습니다.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

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

    window_name = "AprilTag Camera Test (Curved Surface Calibrated)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("\n=======================================================")
    print(" AprilTag 실시간 테스트 (곡면 서피스 보정 & 우하단 외삽 모드)")
    print(" [팁] 'C' 키를 누르면 4개 곡면 제어점을 클릭해 재보정할 수 있습니다!")
    print(" 화면 창에서 'q' 키를 누르면 종료됩니다.")
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

                    p0_int = P0.astype(int)
                    cv2.line(frame, (p0_int[0], p0_int[1]), (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)
                    info_str = f"Pool [X:{x_pool:.2f}m, Y:{y_pool:.2f}m] Yaw:{yaw_deg:+.1f}deg"
                    cv2.putText(frame, info_str, (int(u_center) - 80, int(v_center) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        # ── 곡면 외곽선 및 격자망 ──
        t_samples = np.linspace(0.0, 1.0, 40)
        bot_pts = np.array([coons_patch(t, 0.0) for t in t_samples], dtype=np.int32)
        top_pts = np.array([coons_patch(t, 1.0) for t in t_samples], dtype=np.int32)
        left_pts = np.array([coons_patch(0.0, t) for t in t_samples], dtype=np.int32)
        right_pts = np.array([coons_patch(1.0, t) for t in t_samples], dtype=np.int32)

        cv2.polylines(frame, [bot_pts], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [top_pts], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [left_pts], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
        cv2.polylines(frame, [right_pts], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        for gx in range(2, int(pool_size_x), 2):
            u_norm = gx / pool_size_x
            line_pts = np.array([coons_patch(u_norm, t) for t in t_samples], dtype=np.int32)
            cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 140, 80), thickness=1, lineType=cv2.LINE_AA)

        for gy in range(1, int(pool_size_y)):
            v_norm = gy / pool_size_y
            line_pts = np.array([coons_patch(t, v_norm) for t in t_samples], dtype=np.int32)
            cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 140, 80), thickness=1, lineType=cv2.LINE_AA)

        p0 = P0.astype(int)
        cv2.circle(frame, tuple(p0), 18, (0, 215, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, tuple(p0), 5, (0, 215, 255), -1)
        cv2.drawMarker(frame, tuple(p0), (0, 215, 255), cv2.MARKER_CROSS, 32, 2)
        cv2.putText(frame, "Origin (0,0)", (p0[0] - 10, p0[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

        pt_x_arrow = coons_patch(0.20, 0.0).astype(int)
        cv2.arrowedLine(frame, tuple(p0), tuple(pt_x_arrow), (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(frame, "+X (Curved 0->10m)", (pt_x_arrow[0] + 10, pt_x_arrow[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        pt_y_arrow = coons_patch(0.0, 0.25).astype(int)
        cv2.arrowedLine(frame, tuple(p0), tuple(pt_y_arrow), (0, 255, 0), 3, tipLength=0.2)
        cv2.putText(frame, "+Y (Curved 0->5m)", (pt_y_arrow[0] - 30, pt_y_arrow[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        p3 = P3.astype(int)
        p2 = P2.astype(int)
        cv2.putText(frame, f"P3 (0m, {pool_size_y:.0f}m)", (p3[0] - 20, p3[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, f"P2 ({pool_size_x:.0f}m, {pool_size_y:.0f}m)", (p2[0] - 90, p2[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, "[Extrapolated P1 (10m, 0m)]", (actual_w - 240, actual_h - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1, cv2.LINE_AA)

        if calib_mode:
            names = ["1. 좌하단(P0)", "2. 하단변 중간(M_bot)", "3. 우상단(P2)", "4. 좌상단(P3)"]
            calib_str = f"[곡면 캘리브레이션] 클릭해 주세요: {names[calib_step]}"
            cv2.putText(frame, calib_str, (20, actual_h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "[C] 키: 곡면 제어점 4점 클릭 보정 | [Q] 키: 종료",
                        (20, actual_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1, cv2.LINE_AA)

        if detected_info:
            cv2.putText(frame, f"Detected: {', '.join(detected_info)}",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Searching for AprilTag / ArUco...",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') or key == ord('C'):
            calib_mode = True
            calib_step = 0
            print("📐 [곡면 캘리브레이션 시작] '좌하단(P0) -> 하단중간(M_bot) -> 우상단(P2) -> 좌상단(P3)' 순서로 클릭해 주세요.")
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
