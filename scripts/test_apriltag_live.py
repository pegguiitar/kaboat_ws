#!/usr/bin/env python3
"""카메라 연결 및 AprilTag/ArUco 실시간 검출 테스트 스크립트 (수조 꼭짓점 원점 지원).

기능:
  1. 실제 파란색 수조 좌하단 꼭짓점에 원점 (0,0)과 좌표축 (+X, +Y) 표시
  2. 마우스 클릭으로 수조 꼭짓점 원점을 화면에서 자유롭게 보정 가능
  3. 실시간 보트 위치 [X, Y] 및 거리, 선수각 표시
"""

import math
import sys
import cv2
import numpy as np

# 기본 수조 좌하단 꼭짓점 픽셀 (사용자가 화면 클릭 시 갱신됨)
origin_u = 90
origin_v = 630
ceiling_height = 4.60
fx = 960.0
fy = 960.0
cx = 640.0
cy = 360.0


def on_mouse_click(event, x, y, flags, param):
    global origin_u, origin_v
    if event == cv2.EVENT_LBUTTONDOWN:
        origin_u = x
        origin_v = y
        x_m = (origin_u - cx) * (ceiling_height / fx)
        y_m = (origin_v - cy) * (ceiling_height / fy)
        print(f"🎯 [수조 원점 갱신] 클릭 픽셀: ({x}, {y}) -> 카메라 기준 3D 오프셋: [X:{x_m:+.2f}m, Y:{y_m:+.2f}m]")


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
    global origin_u, origin_v, cx, cy
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
    cx = actual_w / 2.0
    cy = actual_h / 2.0

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
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    window_name = "AprilTag Camera Test (Click to set Pool Origin)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("\n=======================================================")
    print(" AprilTag 실시간 테스트 시작 (수조 꼭짓점 원점 모드)")
    print(" [팁] 화면의 파란색 수조 좌하단 꼭짓점을 마우스로 클릭하면 원점이 보정됩니다!")
    print(" 화면 창에서 'q' 키를 누르면 종료됩니다.")
    print("=======================================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_info = []

        origin_x_cam = (origin_u - cx) * (ceiling_height / fx)
        origin_y_cam = (origin_v - cy) * (ceiling_height / fy)

        for fam_name, adict in dict_families.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=params)
            if ids is not None and len(ids) > 0:
                for i, tid in enumerate(ids.flatten()):
                    detected_info.append(f"{fam_name} ID:{tid}")
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[tid]]))

                    c = corners[i][0]
                    s = 0.30 / 2.0
                    obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
                    _, rvec, tvec = cv2.solvePnP(obj_pts, c.astype(np.float64), camera_matrix, dist_coeffs)

                    tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                    x_odom = tx - origin_x_cam
                    y_odom = -(ty - origin_y_cam)

                    u_center = int(c[:, 0].mean())
                    v_center = int(c[:, 1].mean())

                    cv2.line(frame, (origin_u, origin_v), (u_center, v_center), (0, 255, 255), 2, cv2.LINE_AA)
                    info_str = f"Pool [X:{x_odom:+.2f}m, Y:{y_odom:+.2f}m]"
                    cv2.putText(frame, info_str, (u_center - 70, v_center - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # ── 수조 좌하단 꼭짓점 원점 시각화 ──
        uo = origin_u
        vo = origin_v

        cv2.circle(frame, (uo, vo), 20, (0, 215, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, (uo, vo), 6, (0, 215, 255), -1)
        cv2.drawMarker(frame, (uo, vo), (0, 215, 255), cv2.MARKER_CROSS, 36, 2)

        cv2.arrowedLine(frame, (uo, vo), (uo + 150, vo), (0, 0, 255), 3, tipLength=0.15)
        cv2.putText(frame, "+X (Pool Right)", (uo + 160, vo + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.arrowedLine(frame, (uo, vo), (uo, vo - 150), (0, 255, 0), 3, tipLength=0.15)
        cv2.putText(frame, "+Y (Pool Forward)", (uo - 20, vo - 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, "Origin (0,0) [Pool Bottom-Left Corner]", (uo + 15, vo + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, "[TIP] Click anywhere on the pool corner to adjust Origin (0,0)",
                    (20, actual_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1, cv2.LINE_AA)

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
