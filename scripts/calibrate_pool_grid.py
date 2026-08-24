#!/usr/bin/env python3
"""실내 수조 정밀 클릭 캘리브레이션 툴 (좌측 6점 + 상단 11점 저장).

사용법:
  python3 scripts/calibrate_pool_grid.py [카메라번호: 기본 2]

기능:
  1단계: 좌측 모서리 Y=0m부터 Y=5m까지 1m 간격으로 6번 클릭
  2단계: 상단 모서리 X=0m부터 X=10m까지 1m 간격으로 11번 클릭
  완료 시 config/pool_calibration.yaml 파일로 자동 저장되며,
  이후 ceiling_apriltag_node 실행 시 저장된 좌표계를 자동으로 불러옵니다.
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

# 1280x720 기본값
y_pts = []       # 6개 점 (0m ~ 5m)
top_x_pts = []   # 11개 점 (0m ~ 10m)
current_phase = 1  # 1: Y축 (6점), 2: 상단 X축 (11점), 3: 완료/미리보기


def on_mouse_click(event, x, y, flags, param):
    global current_phase, y_pts, top_x_pts
    if event == cv2.EVENT_LBUTTONDOWN:
        pt = [float(x), float(y)]
        if current_phase == 1:
            y_pts.append(pt)
            m = len(y_pts) - 1
            print(f"📍 [좌측 Y축 {len(y_pts)}/6] Y={m}m 클릭: ({x}, {y})")
            if len(y_pts) == 6:
                current_phase = 2
                print("\n🎉 좌측 모서리(6점) 완료! 이제 상단 모서리(11점)를 클릭하세요.")
                print("   (P3 좌상단 X=0m부터 P2 우상단 X=10m까지 왼쪽->오른쪽 순서)\n")

        elif current_phase == 2:
            top_x_pts.append(pt)
            m = len(top_x_pts) - 1
            print(f"📍 [상단 X축 {len(top_x_pts)}/11] X={m}m 클릭: ({x}, {y})")
            if len(top_x_pts) == 11:
                current_phase = 3
                print("\n🎉 [모든 캘리브레이션 완료!] 수조 격자망이 생성되었습니다.")
                save_calibration()


def save_calibration():
    global y_pts, top_x_pts
    data = {
        'pool_size_x': 10.0,
        'pool_size_y': 5.0,
        'y_control_pts': [[float(p[0]), float(p[1])] for p in y_pts],
        'top_x_control_pts': [[float(p[0]), float(p[1])] for p in top_x_pts]
    }
    os.makedirs(os.path.dirname(YAML_PATH), exist_ok=True)
    with open(YAML_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=None)
    print(f"\n💾 [저장 완료!] 파일 경로: {YAML_PATH}")
    print("   이제 ceiling_apriltag.launch.py 를 실행하면 이 좌표계가 자동으로 적용됩니다.\n")


def quad_bezier(A, M, B, t):
    return (1.0 - t)**2 * A + 2.0 * (1.0 - t) * t * M + t**2 * B


def get_left_pt(v_norm, pts_list):
    idx_float = v_norm * (len(pts_list) - 1)
    i0 = int(np.floor(idx_float))
    i1 = min(i0 + 1, len(pts_list) - 1)
    t = idx_float - i0
    return (1.0 - t) * pts_list[i0] + t * pts_list[i1]


def get_top_pt(u_norm, pts_list):
    idx_float = u_norm * (len(pts_list) - 1)
    i0 = int(np.floor(idx_float))
    i1 = min(i0 + 1, len(pts_list) - 1)
    t = idx_float - i0
    return (1.0 - t) * pts_list[i0] + t * pts_list[i1]


def main():
    global current_phase, y_pts, top_x_pts
    device_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    print(f"카메라 /dev/video{device_idx} 연결 중...")

    cap = cv2.VideoCapture(device_idx)
    if not cap.isOpened():
        print(f"오류: /dev/video{device_idx} 를 열 수 없습니다.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 기존 설정 파일이 있으면 기본값으로 불러오기
    if os.path.exists(YAML_PATH):
        try:
            with open(YAML_PATH, 'r') as f:
                loaded = yaml.safe_load(f)
                if 'y_control_pts' in loaded and 'top_x_control_pts' in loaded:
                    y_pts = [np.array(p, dtype=np.float64) for p in loaded['y_control_pts']]
                    top_x_pts = [np.array(p, dtype=np.float64) for p in loaded['top_x_control_pts']]
                    if len(y_pts) == 6 and len(top_x_pts) == 11:
                        current_phase = 3
                        print(f"기존 캘리브레이션 파일 로드 완료 ({YAML_PATH})")
        except Exception as e:
            print("기존 파일 로드 실패:", e)

    window_name = "Pool Grid Calibrator (Interactive 1m Step)"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse_click)

    print("\n==================================================================")
    print(" 🎯 수조 정밀 캘리브레이션 툴 시작")
    print(" ------------------------------------------------------------------")
    print(" [1단계] 좌측 모서리를 따라 아래(Y=0m)부터 위(Y=5m)까지 6번 클릭")
    print(" [2단계] 상단 모서리를 따라 왼쪽(X=0m)부터 오른쪽(X=10m)까지 11번 클릭")
    print(" ------------------------------------------------------------------")
    print(" [U] 키: 마지막 클릭 취소(Undo) | [R] 키: 처음부터 다시(Reset)")
    print(" [S] 키: 저장(Save)            | [Q] 키: 종료(Quit)")
    print("==================================================================\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ── 1. 지금까지 찍은 점들 시각화 ─────────────────────────────
        # 좌측 Y축 점들
        for idx, pt in enumerate(y_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 7, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 11, (0, 200, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Y={idx}m", (p_int[0] + 12, p_int[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

        if len(y_pts) >= 2:
            pts_arr = np.array(y_pts, dtype=np.int32)
            cv2.polylines(frame, [pts_arr], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        # 상단 X축 점들
        for idx, pt in enumerate(top_x_pts):
            p_int = tuple(np.array(pt, dtype=int))
            cv2.circle(frame, p_int, 7, (0, 255, 100), -1, cv2.LINE_AA)
            cv2.circle(frame, p_int, 11, (0, 200, 100), 2, cv2.LINE_AA)
            cv2.putText(frame, f"X={idx}m", (p_int[0] - 15, p_int[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2, cv2.LINE_AA)

        if len(top_x_pts) >= 2:
            pts_arr = np.array(top_x_pts, dtype=np.int32)
            cv2.polylines(frame, [pts_arr], isClosed=False, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        # ── 2. 전체 격자망 미리보기 (Phase 3) ─────────────────────────
        if len(y_pts) == 6 and len(top_x_pts) == 11:
            P0 = np.array(y_pts[0])
            P3 = np.array(y_pts[-1])
            P2 = np.array(top_x_pts[-1])
            vec_left = P0 - P3
            P1 = np.array([P2[0] - vec_left[0] * 1.08, P0[1] + 30.0])
            M_bot = np.array([640.0, 716.0])

            y_arr = [np.array(p) for p in y_pts]
            x_arr = [np.array(p) for p in top_x_pts]

            t_samples = np.linspace(0.0, 1.0, 40)
            # 1m 격자선 그리기
            for gx in range(1, 10):
                u_norm = gx / 10.0
                pt_top = get_top_pt(u_norm, x_arr)
                pt_bot = quad_bezier(P0, M_bot, P1, u_norm)
                cv2.line(frame, tuple(pt_top.astype(int)), tuple(pt_bot.astype(int)), (80, 160, 80), 1, cv2.LINE_AA)

            for gy in range(1, 5):
                v_norm = gy / 5.0
                pt_l = get_left_pt(v_norm, y_arr)
                pt_r = (1.0 - v_norm) * P1 + v_norm * P2
                cv2.line(frame, tuple(pt_l.astype(int)), tuple(pt_r.astype(int)), (80, 160, 80), 1, cv2.LINE_AA)

            # 하단 변 곡선
            bot_samples = np.array([quad_bezier(P0, M_bot, P1, t) for t in t_samples], dtype=np.int32)
            cv2.polylines(frame, [bot_samples], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)

        # ── 3. OSD 상태 바 ───────────────────────────────────────────
        if current_phase == 1:
            curr_m = len(y_pts)
            guide = f"[Step 1/2: Left Wall] Click point Y = {curr_m}.0m ({curr_m+1}/6)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
        elif current_phase == 2:
            curr_m = len(top_x_pts)
            guide = f"[Step 2/2: Top Wall] Click point X = {curr_m}.0m ({curr_m+1}/11)"
            cv2.putText(frame, guide, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "🎉 [Calibration Complete] Saved to pool_calibration.yaml",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.putText(frame, "[U] Undo | [R] Reset | [S] Save | [Q] Quit",
                    (20, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 1, cv2.LINE_AA)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('u') or key == ord('U'):
            if current_phase == 2 and len(top_x_pts) > 0:
                top_x_pts.pop()
            elif current_phase == 2 and len(top_x_pts) == 0:
                current_phase = 1
                if len(y_pts) > 0:
                    y_pts.pop()
            elif current_phase == 1 and len(y_pts) > 0:
                y_pts.pop()
            elif current_phase == 3:
                current_phase = 2
                if len(top_x_pts) > 0:
                    top_x_pts.pop()
            print("마지막 점 취소 (Undo)")

        elif key == ord('r') or key == ord('R'):
            y_pts = []
            top_x_pts = []
            current_phase = 1
            print("캘리브레이션 초기화 (Reset)")

        elif key == ord('s') or key == ord('S'):
            if len(y_pts) == 6 and len(top_x_pts) == 11:
                save_calibration()
            else:
                print(f"경고: 점을 모두 찍어야 저장됩니다. (Y축 {len(y_pts)}/6, X축 {len(top_x_pts)}/11)")

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
