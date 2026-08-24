#!/usr/bin/env python3
"""카메라 연결 및 AprilTag/ArUco 실시간 검출 테스트 (4점 호모그래피 수조 기울기 보정 지원).

특징:
  1. 수조의 실제 4개 꼭짓점(좌하단, 우하단, 우상단, 좌상단) 기반 원근/기울어짐 완전 보정
  2. 수조 기울기에 맞춘 +X(아랫변), +Y(왼쪽변) 화살표 및 원근 격자망 렌더링
  3. 'c' 키를 누르면 마우스 4점 클릭으로 현장에서 1초 만에 꼭짓점 재보정 가능
"""

import math
import sys
import cv2
import numpy as np

# 수조 크기 [m]
pool_size_x = 10.0
pool_size_y = 5.0

# 4개 꼭짓점 픽셀 좌표 [P0(좌하단), P1(우하단), P2(우상단), P3(좌상단)]
corners_px = np.array([
    [85.0, 640.0],    # P0: 좌하단 (0.0m, 0.0m)
    [1175.0, 650.0],  # P1: 우하단 (10.0m, 0.0m)
    [1160.0, 105.0],  # P2: 우상단 (10.0m, 5.0m)
    [200.0, 95.0]     # P3: 좌상단 (0.0m, 5.0m)
], dtype=np.float32)

calib_mode = False
calib_points = []
H = None
H_inv = None


def update_homography():
    global H, H_inv
    dst_pts = np.array([
        [0.0, 0.0],
        [pool_size_x, 0.0],
        [pool_size_x, pool_size_y],
        [0.0, pool_size_y]
    ], dtype=np.float32)
    H = cv2.getPerspectiveTransform(corners_px, dst_pts)
    H_inv = cv2.getPerspectiveTransform(dst_pts, corners_px)


update_homography()


def on_mouse_click(event, x, y, flags, param):
    global calib_mode, calib_points, corners_px
    if event == cv2.EVENT_LBUTTONDOWN and calib_mode:
        calib_points.append([float(x), float(y)])
        names = ["P0 (좌하단)", "P1 (우하단)", "P2 (우상단)", "P3 (좌상단)"]
        print(f"📍 {names[len(calib_points)-1]} 선택: 픽셀({x}, {y})")

        if len(calib_points) == 4:
            corners_px = np.array(calib_points, dtype=np.float32)
            update_homography()
            calib_mode = False
            calib_points = []
            print("🎉 [수조 4점 캘리브레이션 완료!] 호모그래피 행렬이 완벽하게 갱신되었습니다.")


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
    global calib_mode, calib_points
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

    window_name = "AprilTag Camera Test (Homography Calibrated)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("\n=======================================================")
    print(" AprilTag 실시간 테스트 시작 (수조 기울기 호모그래피 보정 모드)")
    print(" [팁] 'C' 키를 누르면 마우스로 4개 꼭짓점을 클릭해 즉시 재보정할 수 있습니다!")
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

                    # 호모그래피 수조 좌표 변환
                    px_mat = np.array([[[u_center, v_center]]], dtype=np.float32)
                    mapped = cv2.perspectiveTransform(px_mat, H)[0][0]
                    x_pool = float(mapped[0])
                    y_pool = float(mapped[1])

                    # 수조 기준 헤딩(Yaw) 각도 계산
                    u_fwd = float((c[0][0] + c[1][0]) / 2.0)
                    v_fwd = float((c[0][1] + c[1][1]) / 2.0)
                    fwd_mat = np.array([[[u_fwd, v_fwd]]], dtype=np.float32)
                    mapped_fwd = cv2.perspectiveTransform(fwd_mat, H)[0][0]
                    yaw_rad = math.atan2(float(mapped_fwd[1] - y_pool), float(mapped_fwd[0] - x_pool))
                    yaw_deg = math.degrees(yaw_rad)

                    p0 = corners_px[0].astype(int)
                    cv2.line(frame, (p0[0], p0[1]), (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)
                    info_str = f"Pool [X:{x_pool:.2f}m, Y:{y_pool:.2f}m] Yaw:{yaw_deg:+.1f}deg"
                    cv2.putText(frame, info_str, (int(u_center) - 80, int(v_center) - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        # ── 수조 외곽선 및 원근 격자망 ──
        pts = corners_px.astype(int)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        for gx in range(2, int(pool_size_x), 2):
            m_bottom = cv2.perspectiveTransform(np.array([[[gx, 0.0]]], dtype=np.float32), H_inv)[0][0].astype(int)
            m_top = cv2.perspectiveTransform(np.array([[[gx, pool_size_y]]], dtype=np.float32), H_inv)[0][0].astype(int)
            cv2.line(frame, tuple(m_bottom), tuple(m_top), (80, 140, 80), 1, cv2.LINE_AA)

        for gy in range(1, int(pool_size_y)):
            m_left = cv2.perspectiveTransform(np.array([[[0.0, gy]]], dtype=np.float32), H_inv)[0][0].astype(int)
            m_right = cv2.perspectiveTransform(np.array([[[pool_size_x, gy]]], dtype=np.float32), H_inv)[0][0].astype(int)
            cv2.line(frame, tuple(m_left), tuple(m_right), (80, 140, 80), 1, cv2.LINE_AA)

        p0 = pts[0]  # 좌하단 원점
        p1 = pts[1]  # 우하단
        p3 = pts[3]  # 좌상단

        # 원점 마커
        cv2.circle(frame, tuple(p0), 18, (0, 215, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, tuple(p0), 5, (0, 215, 255), -1)
        cv2.drawMarker(frame, tuple(p0), (0, 215, 255), cv2.MARKER_CROSS, 32, 2)
        cv2.putText(frame, "Origin (0,0)", (p0[0] - 10, p0[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

        # 기울어진 +X 축 화살표 (수조 아랫변)
        vec_x = (p1 - p0).astype(float)
        len_x = np.linalg.norm(vec_x)
        if len_x > 0:
            dir_x = (vec_x / len_x * min(200, len_x * 0.3)).astype(int)
            target_x = p0 + dir_x
            cv2.arrowedLine(frame, tuple(p0), tuple(target_x), (0, 0, 255), 3, tipLength=0.18)
            cv2.putText(frame, "+X (0->10m)", (target_x[0] + 10, target_x[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        # 기울어진 +Y 축 화살표 (수조 왼쪽변)
        vec_y = (p3 - p0).astype(float)
        len_y = np.linalg.norm(vec_y)
        if len_y > 0:
            dir_y = (vec_y / len_y * min(200, len_y * 0.3)).astype(int)
            target_y = p0 + dir_y
            cv2.arrowedLine(frame, tuple(p0), tuple(target_y), (0, 255, 0), 3, tipLength=0.18)
            cv2.putText(frame, "+Y (0->5m)", (target_y[0] - 30, target_y[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, f"P1 ({pool_size_x:.0f}m, 0m)", (pts[1][0] - 90, pts[1][1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, f"P2 ({pool_size_x:.0f}m, {pool_size_y:.0f}m)", (pts[2][0] - 90, pts[2][1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(frame, f"P3 (0m, {pool_size_y:.0f}m)", (pts[3][0] - 20, pts[3][1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        if calib_mode:
            point_names = ["1. 좌하단(P0)", "2. 우하단(P1)", "3. 우상단(P2)", "4. 좌상단(P3)"]
            next_step = point_names[len(calib_points)]
            calib_str = f"[캘리브레이션 모드] 수조 꼭짓점을 차례로 클릭하세요: {next_step}"
            cv2.putText(frame, calib_str, (20, actual_h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "[C] 키: 수조 4점 클릭 캘리브레이션 | [Q] 키: 종료",
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
            calib_points = []
            print("📐 [4점 캘리브레이션 시작] 수조의 4개 꼭짓점을 '좌하단 -> 우하단 -> 우상단 -> 좌상단' 순서로 클릭해 주세요.")
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
