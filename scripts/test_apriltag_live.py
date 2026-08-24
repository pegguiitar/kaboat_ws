#!/usr/bin/env python3
"""카메라 연결 및 AprilTag 실시간 인식 단독 테스트 스크립트 (원거리/천장 카메라 최적화).

개선 사항:
  1. HD(1280x720) MJPG 고해상도 캡처 (천장 원거리 태그 픽셀 확보)
  2. 원거리/소형 마커 검출 파라미터 최적화 (minMarkerPerimeterRate, Subpixel Refinement)
  3. tag36h11 외 다른 패밀리(tag25h9, tag16h5, ARUCO) 자동 감지 지원

사용법:
  python3 scripts/test_apriltag_live.py [카메라번호 (기본: 2)]

종료: 영상 창에서 'q' 또는 ESC 키 입력
"""

import sys
import cv2


def get_detector_params():
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()

    # ── 원거리/소형 태그 검출 튜닝 ─────────────────────
    params.minMarkerPerimeterRate = 0.005      # 기본 0.03 -> 0.005로 완화하여 멀리 있는 작은 태그 감지
    params.maxMarkerPerimeterRate = 4.0
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 3
    params.adaptiveThreshConstant = 7.0

    # 서브픽셀 코너 정밀화
    if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # AprilTag 전용 파라미터
    if hasattr(params, 'aprilTagQuadDecimate'):
        params.aprilTagQuadDecimate = 1.0     # 1.0 = 원본 해상도 유지(축소 안 함)
    if hasattr(params, 'aprilTagCriticalRad'):
        params.aprilTagCriticalRad = 0.1745
    if hasattr(params, 'aprilTagMinWhiteBlackDiff'):
        params.aprilTagMinWhiteBlackDiff = 5

    return params


def main():
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"카메라 장치 /dev/video{device_idx} 연결 시도 중...")

    cap = cv2.VideoCapture(device_idx)
    if not cap.isOpened():
        print(f"오류: /dev/video{device_idx} 를 열 수 없습니다. (장치가 사용 중이거나 연결 안 됨)")
        sys.exit(1)

    # 1280x720 MJPG 고해상도 설정
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"카메라 해상도: {actual_w}x{actual_h}")

    # 패밀리별 딕셔너리
    dict_families = {}
    if hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
        dict_families['tag36h11 (표준)'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'DICT_APRILTAG_25h9'):
        dict_families['tag25h9'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_25h9) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    if hasattr(cv2.aruco, 'DICT_APRILTAG_16h5'):
        dict_families['tag16h5'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    if hasattr(cv2.aruco, 'DICT_4X4_50'):
        dict_families['ArUco_4x4'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    params = get_detector_params()

    print("\n=======================================================")
    print(" AprilTag 실시간 테스트 시작 (1280x720 고해상도 모드)")
    print(" 화면 창에서 'q' 키를 누르면 종료됩니다.")
    print("=======================================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected_info = []

        # 패밀리별 검출 시도 (tag36h11 우선)
        for fam_name, adict in dict_families.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=params)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for tag_id in ids.flatten():
                    detected_info.append(f"{fam_name} ID:{tag_id}")

        cx_int = actual_w // 2
        cy_int = actual_h // 2

        # ── 1. 수조 중심 원점 (0, 0) 및 격자선 오버레이 ───────────
        cv2.line(frame, (0, cy_int), (actual_w, cy_int), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.line(frame, (cx_int, 0), (cx_int, actual_h), (80, 80, 80), 1, cv2.LINE_AA)

        cv2.circle(frame, (cx_int, cy_int), 18, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, (cx_int, cy_int), 4, (0, 255, 255), -1)
        cv2.drawMarker(frame, (cx_int, cy_int), (0, 255, 255), cv2.MARKER_CROSS, 32, 2)

        # ── 2. 좌표축 (+X, +Y) 화살표 ──────────────────────────
        cv2.arrowedLine(frame, (cx_int, cy_int), (cx_int + 120, cy_int), (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(frame, "+X (Right)", (cx_int + 130, cy_int + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.arrowedLine(frame, (cx_int, cy_int), (cx_int, cy_int - 120), (0, 255, 0), 3, tipLength=0.2)
        cv2.putText(frame, "+Y (Up)", (cx_int - 25, cy_int - 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, "Origin (0, 0) [Pool Center]", (cx_int + 10, cy_int + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        # OSD 정보 표시
        if detected_info:
            text = f"Detected: {', '.join(detected_info)}"
            cv2.putText(frame, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "Searching for AprilTag / ArUco...",
                        (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(frame, f"Res: {actual_w}x{actual_h} | Press 'q' to exit",
                    (20, actual_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("AprilTag Camera Test (HD 720p)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("테스트 종료.")


if __name__ == '__main__':
    main()
