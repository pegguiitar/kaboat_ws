#!/usr/bin/env python3
"""실내 수조 4면 전체 1m 간격 정밀 클릭 캘리브레이션 툴.

사용법:
  python3 scripts/calibrate_pool_grid.py [카메라장치번호: 기본 2]

진행 순서 (총 4단계):
  1단계: 하단 변 (Bottom) - 좌하단(X=0m) -> 우하단(X=10m) 1m 간격 [11점]
  2단계: 우측 변 (Right)  - 우하단(Y=0m) -> 우상단(Y=5m) 1m 간격 [6점]
  3단계: 상단 변 (Top)    - 좌상단(X=0m) -> 우상단(X=10m) 1m 간격 [11점]
  4단계: 좌측 변 (Left)   - 좌하단(Y=0m) -> 좌상단(Y=5m) 1m 간격 [6점]

완료 시 config/pool_calibration.yaml 에 자동 저장되며,
이후 ceiling_apriltag_node 실행 시 4면 전체 1m 실측 좌표계를 자동 적용합니다.
"""

import os
import sys
import yaml
import cv2
import numpy as np

YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'src', 'kaboat_hardware', 'config', 'pool_calibration.yaml'
)

# 4개 변 1m 점들
bottom_x_pts = []  # 11점 (X: 0m -> 10m, Y=0m)
right_y_pts = []   # 6점  (Y: 0m -> 5m, X=10m)
top_x_pts = []     # 11점 (X: 0m -> 10m, Y=5m)
left_y_pts = []    # 6점  (Y: 0m -> 5m, X=0m)

current_step = 1   # 1: 하단, 2: 우측, 3: 상단, 4: 좌측, 5: 완료


def on_mouse_click(event, x, y, flags, param):
    global current_step, bottom_x_pts, right_y_pts, top_x_pts, left_y_pts
    if event == cv2.EVENT_LBUTTONDOWN:
        pt = [float(x), float(y)]

        # ── 1단계: 하단 변 (11점) ──
        if current_step == 1:
            bottom_x_pts.append(pt)
            m = len(bottom_x_pts) - 1
            print(f"📍 [1/4 하단] X={m}m (Y=0m) 클릭: ({x}, {y}) [총 {len(bottom_x_pts)}/11]")
            if len(bottom_x_pts) == 11:
                current_step = 2
                print("\n🎉 [1단계 완료!] 이제 2단계: 우측 변(Right)을 클릭하세요.")
                print("   (우하단 Y=0m부터 우상단 Y=5m까지 아래->위 6번 클릭)\n")

        # ── 2단계: 우측 변 (6점) ──
        elif current_step == 2:
            right_y_pts.append(pt)
            m = len(right_y_pts) - 1
            print(f"📍 [2/4 우측] Y={m}m (X=10m) 클릭: ({x}, {y}) [총 {len(right_y_pts)}/6]")
            if len(right_y_pts) == 6:
                current_step = 3
                print("\n🎉 [2단계 완료!] 이제 3단계: 상단 변(Top)을 클릭하세요.")
                print("   (좌상단 X=0m부터 우상단 X=10m까지 왼쪽->오른쪽 11번 클릭)\n")

        # ── 3단계: 상단 변 (11점) ──
        elif current_step == 3:
            top_x_pts.append(pt)
            m = len(top_x_pts) - 1
            print(f"📍 [3/4 상단] X={m}m (Y=5m) 클릭: ({x}, {y}) [총 {len(top_x_pts)}/11]")
            if len(top_x_pts) == 11:
                current_step = 4
                print("\n🎉 [3단계 완료!] 이제 마지막 4단계: 좌측 변(Left)을 클릭하세요.")
                print("   (좌하단 Y=0m부터 좌상단 Y=5m까지 아래->위 6번 클릭)\n")

        # ── 4단계: 좌측 변 (6점) ──
        elif current_step == 4:
            left_y_pts.append(pt)
            m = len(left_y_pts) - 1
            print(f"📍 [4/4 좌측] Y={m}m (X=0m) 클릭: ({x}, {y}) [총 {len(left_y_pts)}/6]")
            if len(left_y_pts) == 6:
                current_step = 5
                print("\n🎉🎉 [모든 4면 캘리브레이션 완료!] 수조 격자망이 생성되었습니다.")
                save_calibration()


def save_calibration():
    global bottom_x_pts, right_y_pts, top_x_pts, left_y_pts
    data = {
        'pool_size_x': 10.0,
        'pool_size_y': 5.0,
        'bottom_x_pts': [[float(p[0]), float(p[1])] for p in bottom_x_pts],
        'right_y_pts': [[float(p[0]), float(p[1])] for p in right_y_pts],
        'top_x_pts': [[float(p[0]), float(p[1])] for p in top_x_pts],
        'left_y_pts': [[float(p[0]), float(p[1])] for p in left_y_pts],
    }
    os.makedirs(os.path.dirname(YAML_PATH), exist_ok=True)
    with open(YAML_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
    print(f"\n💾 [저장 완료!] 파일 경로: {YAML_PATH}")
    print("   이제 ceiling_apriltag.launch.py 를 실행하면 4면 전체 1m 실측 격자가 자동 로드됩니다.\n")


def get_curve_pt(norm_val, pts_list):
    idx_float = norm_val * (len(pts_list) - 1)
    i0 = int(np.floor(idx_float))
    i1 = min(i0 + 1, len(pts_list) - 1)
    t = idx_float - i0
    return (1.0 - t) * np.array(pts_list[i0]) + t * np.array(pts_list[i1])


def coons_patch_4edge(u, v):
    c_bot = get_curve_pt(u, bottom_x_pts)
    c_top = get_curve_pt(u, top_x_pts)
    c_left = get_curve_pt(v, left_y_pts)
    c_right = get_curve_pt(v, right_y_pts)

    P0 = np.array(bottom_x_pts[0])
    P1 = np.array(bottom_x_pts[-1])
    P3 = np.array(top_x_pts[0])
    P2 = np.array(top_x_pts[-1])

    corner_blend = (1.0 - u) * (1.0 - v) * P0 + u * (1.0 - v) * P1 + (1.0 - u) * v * P3 + u * v * P2
    return (1.0 - v) * c_bot + v * c_top + (1.0 - u) * c_left + u * c_right - corner_blend


def main():
    global current_step, bottom_x_pts, right_y_pts, top_x_pts, left_y_pts
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"카메라 /dev/video{device_idx} 연결 중...")

    cap = cv2.VideoCapture(device_idx)
    if not cap.isOpened():
        print(f"오류: /dev/video{device_idx} 를 열 수 없습니다.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 기존 설정 로드
    if os.path.exists(YAML_PATH):
        try:
            with open(YAML_PATH, 'r') as f:
                d = yaml.safe_load(f)
                if 'bottom_x_pts' in d and 'right_y_pts' in d and 'top_x_pts' in d and 'left_y_pts' in d:
                    bottom_x_pts = d['bottom_x_pts']
                    right_y_pts = d['right_y_pts']
                    top_x_pts = d['top_x_pts']
                    left_y_pts = d['left_y_pts']
                    if len(bottom_x_pts) == 11 and len(right_y_pts) == 6 and len(top_x_pts) == 11 and len(left_y_pts) == 6:
                        current_step = 5
                        print(f"기존 4면 캘리브레이션 파일 로드 완료 ({YAML_PATH})")
        except Exception as e:
            print("기존 파일 로드 실패:", e)

    window_name = "4-Edge Pool Grid Calibrator (1280x720)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("\n==================================================================")
    print(" 🎯 수조 4면 전체 1m 간격 캘리브레이션 툴")
    print(" ------------------------------------------------------------------")
    print(" [1단계: 하단] 좌하단(X=0) -> 우하단(X=10m) 1m 간격 [11번 클릭]")
    print(" [2단계: 우측] 우하단(Y=0) -> 우상단(Y=5m)  1m 간격 [6번 클릭]")
    print(" [3단계: 상단] 좌상단(X=0) -> 우상단(X=10m) 1m 간격 [11번 클릭]")
    print(" [4단계: 좌측] 좌하단(Y=0) -> 좌상단(Y=5m)  1m 간격 [6번 클릭]")
    print(" ------------------------------------------------------------------")
    print(" [U] 키: 직전 점 취소(Undo) | [R] 키: 전체 초기화(Reset)")
    print(" [S] 키: 저장(Save)        | [Q] 키: 종료(Quit)")
    print("==================================================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ── 1. 각 변의 점들 렌더링 ───────────────────────────────────
        # 1) 하단 변 점들 (Red/Orange)
        for idx, pt in enumerate(bottom_x_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 10, (0, 100, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"B:{idx}m", (p_int[0] - 15, p_int[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
        if len(bottom_x_pts) >= 2:
            cv2.polylines(frame, [np.array(bottom_x_pts, dtype=np.int32)], False, (0, 0, 255), 2, cv2.LINE_AA)

        # 2) 우측 변 점들 (Magenta)
        for idx, pt in enumerate(right_y_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 6, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 10, (200, 0, 200), 2, cv2.LINE_AA)
            cv2.putText(frame, f"R:{idx}m", (p_int[0] + 12, p_int[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
        if len(right_y_pts) >= 2:
            cv2.polylines(frame, [np.array(right_y_pts, dtype=np.int32)], False, (255, 0, 255), 2, cv2.LINE_AA)

        # 3) 상단 변 점들 (Green)
        for idx, pt in enumerate(top_x_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 6, (0, 255, 100), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 10, (0, 200, 100), 2, cv2.LINE_AA)
            cv2.putText(frame, f"T:{idx}m", (p_int[0] - 15, p_int[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 1, cv2.LINE_AA)
        if len(top_x_pts) >= 2:
            cv2.polylines(frame, [np.array(top_x_pts, dtype=np.int32)], False, (0, 255, 100), 2, cv2.LINE_AA)

        # 4) 좌측 변 점들 (Cyan/Yellow)
        for idx, pt in enumerate(left_y_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 6, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 10, (0, 200, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"L:{idx}m", (p_int[0] - 45, p_int[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if len(left_y_pts) >= 2:
            cv2.polylines(frame, [np.array(left_y_pts, dtype=np.int32)], False, (0, 255, 255), 2, cv2.LINE_AA)

        # ── 2. 전체 완벽 격자망 렌더링 (4면 모두 완료 시) ────────────
        if len(bottom_x_pts) == 11 and len(right_y_pts) == 6 and len(top_x_pts) == 11 and len(left_y_pts) == 6:
            t_samples = np.linspace(0.0, 1.0, 40)
            # 세로 1m 격자선 (X = 1m ~ 9m)
            for gx in range(1, 10):
                u_norm = gx / 10.0
                line_pts = np.array([coons_patch_4edge(u_norm, t) for t in t_samples], dtype=np.int32)
                cv2.polylines(frame, [line_pts], False, (80, 180, 80), 1, cv2.LINE_AA)

            # 가로 1m 격자선 (Y = 1m ~ 4m)
            for gy in range(1, 5):
                v_norm = gy / 5.0
                line_pts = np.array([coons_patch_4edge(t, v_norm) for t in t_samples], dtype=np.int32)
                cv2.polylines(frame, [line_pts], False, (80, 180, 80), 1, cv2.LINE_AA)

            # 원점 마커 표시
            p0 = tuple(np.array(bottom_x_pts[0], dtype=int))
            cv2.circle(frame, p0, 16, (0, 215, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, "Origin (0,0)", (p0[0] - 20, p0[1] + 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

        # ── 3. OSD 단계별 가이드 배너 ────────────────────────────────
        if current_step == 1:
            m = len(bottom_x_pts)
            guide = f"[1/4 하단변] 좌하단(X=0) -> 우하단(X=10m): X = {m}.0m 지점 클릭 ({m+1}/11)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        elif current_step == 2:
            m = len(right_y_pts)
            guide = f"[2/4 우측변] 우하단(Y=0) -> 우상단(Y=5m): Y = {m}.0m 지점 클릭 ({m+1}/6)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 255), 2, cv2.LINE_AA)
        elif current_step == 3:
            m = len(top_x_pts)
            guide = f"[3/4 상단변] 좌상단(X=0) -> 우상단(X=10m): X = {m}.0m 지점 클릭 ({m+1}/11)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 100), 2, cv2.LINE_AA)
        elif current_step == 4:
            m = len(left_y_pts)
            guide = f"[4/4 좌측변] 좌하단(Y=0) -> 좌상단(Y=5m): Y = {m}.0m 지점 클릭 ({m+1}/6)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "🎉 [4면 전체 캘리브레이션 완료!] pool_calibration.yaml 저장됨",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, "[U] Undo | [R] Reset | [S] Save | [Q] Quit",
                    (20, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 1, cv2.LINE_AA)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('u') or key == ord('U'):
            if current_step == 5:
                current_step = 4
                if len(left_y_pts) > 0:
                    left_y_pts.pop()
            elif current_step == 4:
                if len(left_y_pts) > 0:
                    left_y_pts.pop()
                else:
                    current_step = 3
                    if len(top_x_pts) > 0:
                        top_x_pts.pop()
            elif current_step == 3:
                if len(top_x_pts) > 0:
                    top_x_pts.pop()
                else:
                    current_step = 2
                    if len(right_y_pts) > 0:
                        right_y_pts.pop()
            elif current_step == 2:
                if len(right_y_pts) > 0:
                    right_y_pts.pop()
                else:
                    current_step = 1
                    if len(bottom_x_pts) > 0:
                        bottom_x_pts.pop()
            elif current_step == 1 and len(bottom_x_pts) > 0:
                bottom_x_pts.pop()
            print("직전 점 취소 (Undo)")

        elif key == ord('r') or key == ord('R'):
            bottom_x_pts = []
            right_y_pts = []
            top_x_pts = []
            left_y_pts = []
            current_step = 1
            print("전체 캘리브레이션 초기화 (Reset)")

        elif key == ord('s') or key == ord('S'):
            if len(bottom_x_pts) == 11 and len(right_y_pts) == 6 and len(top_x_pts) == 11 and len(left_y_pts) == 6:
                save_calibration()
            else:
                print("경고: 4개 변(하단 11, 우측 6, 상단 11, 좌측 6)을 모두 찍어야 저장됩니다.")

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
