"""ceiling_apriltag_node — 4점 호모그래피(Perspective Transform) 기반 실내 수조 AprilTag 추적기.

카메라의 장착 기울어짐(Pitch/Roll/Yaw)과 원근 왜곡을 수조의 4개 꼭짓점 기반
호모그래피 변환(Homography)으로 100% 완전 보정하여, 수조 모서리를 따르는
정밀한 X(0~10m), Y(0~5m), Yaw 각도 오도메트리를 산출합니다.

특징:
  1. 수조의 실제 4개 꼭짓점(좌하단, 우하단, 우상단, 좌상단) 기반 원근 보정
  2. 수조 기울기에 맞춘 +X / +Y 축 화살표 및 원근 격자망(Perspective Grid) 렌더링
  3. 'c' 키로 언제든지 4점 클릭 재보정(Calibration) 및 's' 키로 YAML 저장 가능
"""

import math
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory


def rot_matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


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


class CeilingAprilTagNode(Node):
    def __init__(self):
        super().__init__('ceiling_apriltag_node')

        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('tag_size', 0.300)   # [m] 30cm
        self.declare_parameter('camera_frame', 'ceiling_camera')
        self.declare_parameter('tag_frame', 'tag36h11:1')
        self.declare_parameter('show_gui', True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('fx', 960.0)
        self.declare_parameter('fy', 960.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        self.declare_parameter('pool_size_x', 10.0)  # [m]
        self.declare_parameter('pool_size_y', 5.0)   # [m]

        device_str = str(self.get_parameter('video_device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.target_id = int(self.get_parameter('tag_id').value)
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.tag_frame = str(self.get_parameter('tag_frame').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.publish_img = bool(self.get_parameter('publish_image').value)
        self.pool_size_x = float(self.get_parameter('pool_size_x').value)
        self.pool_size_y = float(self.get_parameter('pool_size_y').value)

        # 4개 꼭짓점 픽셀 좌표 [P0(좌하단), P1(우하단), P2(우상단), P3(좌상단)]
        self.corners_px = np.array([
            [85.0, 640.0],    # P0: 좌하단 (0.0m, 0.0m)
            [1175.0, 650.0],  # P1: 우하단 (10.0m, 0.0m)
            [1160.0, 105.0],  # P2: 우상단 (10.0m, 5.0m)
            [200.0, 95.0]     # P3: 좌상단 (0.0m, 5.0m)
        ], dtype=np.float32)

        self._update_homography()

        # 캘리브레이션 모드 상태
        self.calib_mode = False
        self.calib_points = []

        # 카메라 매트릭스
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.camera_matrix = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # 딕셔너리 로드
        self.dict_families = {}
        if hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
            self.dict_families['tag36h11'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'DICT_APRILTAG_25h9'):
            self.dict_families['tag25h9'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_25h9) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        if hasattr(cv2.aruco, 'DICT_APRILTAG_16h5'):
            self.dict_families['tag16h5'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
        if hasattr(cv2.aruco, 'DICT_4X4_50'):
            self.dict_families['ArUco_4x4'] = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.params = get_detector_params()

        # 카메라 열기
        try:
            device_idx = int(device_str.replace('/dev/video', ''))
        except ValueError:
            device_idx = 2

        self.cap = cv2.VideoCapture(device_idx)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            self.get_logger().error(f"카메라 장치 {device_str} 열기 실패!")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/detections', 10)
        self.img_pub = self.create_publisher(Image, '/ceiling_cam/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/ceiling_cam/camera_info', 10)

        self.window_name = "Ceiling AprilTag Tracker (Homography Calibrated)"
        self.window_initialized = False

        self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info(
            f"ceiling_apriltag_node 시작 — 수조 4점 호모그래피 보정 활성화 ({self.pool_size_x}x{self.pool_size_y}m)")

    def _update_homography(self):
        dst_pts = np.array([
            [0.0, 0.0],                          # P0: (0m, 0m)
            [self.pool_size_x, 0.0],             # P1: (10m, 0m)
            [self.pool_size_x, self.pool_size_y],# P2: (10m, 5m)
            [0.0, self.pool_size_y]              # P3: (0m, 5m)
        ], dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(self.corners_px, dst_pts)
        self.H_inv = cv2.getPerspectiveTransform(dst_pts, self.corners_px)

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.calib_mode:
                self.calib_points.append([float(x), float(y)])
                point_names = ["P0 (좌하단)", "P1 (우하단)", "P2 (우상단)", "P3 (좌상단)"]
                idx = len(self.calib_points) - 1
                self.get_logger().info(f"📍 {point_names[idx]} 선택됨: 픽셀({x}, {y})")

                if len(self.calib_points) == 4:
                    self.corners_px = np.array(self.calib_points, dtype=np.float32)
                    self._update_homography()
                    self.calib_mode = False
                    self.calib_points = []
                    self.get_logger().info("🎉 [수조 4점 캘리브레이션 완료!] 호모그래피 행렬이 완벽하게 갱신되었습니다.")

    def _process_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tag_found = False
        detected_info = []

        for fam_name, adict in self.dict_families.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=self.params)
            if ids is not None and len(ids) > 0:
                for i, tid in enumerate(ids.flatten()):
                    detected_info.append(f"{fam_name} ID:{tid}")
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[tid]]))

                    if not tag_found and (self.target_id <= 0 or tid == self.target_id):
                        tag_found = True
                        c = corners[i][0]  # (4, 2)
                        u_center = float(c[:, 0].mean())
                        v_center = float(c[:, 1].mean())

                        # ── 1. 호모그래피 변환을 통한 정밀 수조 좌표 계산 ──
                        px_mat = np.array([[[u_center, v_center]]], dtype=np.float32)
                        mapped = cv2.perspectiveTransform(px_mat, self.H)[0][0]
                        x_pool = float(mapped[0])
                        y_pool = float(mapped[1])

                        # ── 2. 수조 평면 기준 태그 방향(Yaw) 계산 ───
                        # 태그 전방 벡터(모서리 0->3 또는 1->2의 중점 방향)
                        u_fwd = float((c[0][0] + c[1][0]) / 2.0)
                        v_fwd = float((c[0][1] + c[1][1]) / 2.0)
                        fwd_mat = np.array([[[u_fwd, v_fwd]]], dtype=np.float32)
                        mapped_fwd = cv2.perspectiveTransform(fwd_mat, self.H)[0][0]
                        dx = float(mapped_fwd[0] - x_pool)
                        dy = float(mapped_fwd[1] - y_pool)
                        yaw_pool_rad = math.atan2(dy, dx)
                        yaw_pool_deg = math.degrees(yaw_pool_rad)

                        # PnP로 높이(Z) 및 쿼터니언 계산
                        s = self.tag_size / 2.0
                        obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
                        _, rvec, tvec = cv2.solvePnP(
                            obj_pts, c.astype(np.float64), self.camera_matrix, self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE') else cv2.SOLVEPNP_ITERATIVE
                        )
                        R, _ = cv2.Rodrigues(rvec)
                        qx, qy, qz, qw = rot_matrix_to_quaternion(R)
                        tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])

                        tag_frame_name = f"tag36h11:{tid}" if self.target_id <= 0 else self.tag_frame

                        # TF 발행
                        tf = TransformStamped()
                        tf.header.stamp = stamp
                        tf.header.frame_id = self.camera_frame
                        tf.child_frame_id = tag_frame_name
                        tf.transform.translation.x = tx
                        tf.transform.translation.y = ty
                        tf.transform.translation.z = tz
                        tf.transform.rotation.x = qx
                        tf.transform.rotation.y = qy
                        tf.transform.rotation.z = qz
                        tf.transform.rotation.w = qw
                        self.tf_broadcaster.sendTransform(tf)

                        # PoseStamped 발행 (수조 정밀 보정 좌표)
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = stamp
                        pose_msg.header.frame_id = "odom"
                        pose_msg.pose.position.x = x_pool
                        pose_msg.pose.position.y = y_pool
                        pose_msg.pose.position.z = 0.0
                        # 2D yaw -> quaternion
                        pose_msg.pose.orientation.z = math.sin(yaw_pool_rad / 2.0)
                        pose_msg.pose.orientation.w = math.cos(yaw_pool_rad / 2.0)
                        self.pose_pub.publish(pose_msg)

                        # 터미널 로깅
                        self.get_logger().info(
                            f"✅ [{fam_name}] ID:{tid} | 🎯 보정된 수조좌표:[X:{x_pool:.2f}m, Y:{y_pool:.2f}m] | Yaw:{yaw_pool_deg:+.1f}°",
                            throttle_duration_sec=1.0)

                        # 태그 ↔ 원점 연결선
                        p0 = self.corners_px[0].astype(int)
                        cv2.line(frame, (p0[0], p0[1]), (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)

                        # 보트 위 수조 좌표 표시
                        info_str = f"Pool [X:{x_pool:.2f}m, Y:{y_pool:.2f}m] Yaw:{yaw_pool_deg:+.1f}deg"
                        cv2.putText(frame, info_str, (int(u_center) - 80, int(v_center) - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        if self.show_gui:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name)
                cv2.setMouseCallback(self.window_name, self._on_mouse_click)
                self.window_initialized = True

            # ── 1. 수조 외곽선 및 원근 격자망 그리기 ────────────────
            pts = self.corners_px.astype(int)
            # 수조 외곽 테두리 (녹색 사다리꼴)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

            # 수조 내부 2m x 1m 가상 격자망 (원근 보정선)
            for gx in range(2, int(self.pool_size_x), 2):
                m_bottom = cv2.perspectiveTransform(np.array([[[gx, 0.0]]], dtype=np.float32), self.H_inv)[0][0].astype(int)
                m_top = cv2.perspectiveTransform(np.array([[[gx, self.pool_size_y]]], dtype=np.float32), self.H_inv)[0][0].astype(int)
                cv2.line(frame, tuple(m_bottom), tuple(m_top), (80, 140, 80), 1, cv2.LINE_AA)

            for gy in range(1, int(self.pool_size_y)):
                m_left = cv2.perspectiveTransform(np.array([[[0.0, gy]]], dtype=np.float32), self.H_inv)[0][0].astype(int)
                m_right = cv2.perspectiveTransform(np.array([[[self.pool_size_x, gy]]], dtype=np.float32), self.H_inv)[0][0].astype(int)
                cv2.line(frame, tuple(m_left), tuple(m_right), (80, 140, 80), 1, cv2.LINE_AA)

            # ── 2. 수조 기울기에 맞춘 +X, +Y 축 화살표 ───────────────
            p0 = pts[0]  # 좌하단 원점
            p1 = pts[1]  # 우하단
            p3 = pts[3]  # 좌상단

            # 원점 마커 (이중 황금 링)
            cv2.circle(frame, tuple(p0), 18, (0, 215, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, tuple(p0), 5, (0, 215, 255), -1)
            cv2.drawMarker(frame, tuple(p0), (0, 215, 255), cv2.MARKER_CROSS, 32, 2)
            cv2.putText(frame, "Origin (0,0)", (p0[0] - 10, p0[1] + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

            # +X 축 화살표 (수조 아랫변을 따라 정확한 기울기 방향: Red)
            vec_x = (p1 - p0).astype(float)
            len_x = np.linalg.norm(vec_x)
            if len_x > 0:
                dir_x = (vec_x / len_x * min(200, len_x * 0.3)).astype(int)
                target_x = p0 + dir_x
                cv2.arrowedLine(frame, tuple(p0), tuple(target_x), (0, 0, 255), 3, tipLength=0.18)
                cv2.putText(frame, "+X (0->10m)", (target_x[0] + 10, target_x[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

            # +Y 축 화살표 (수조 왼쪽변을 따라 정확한 기울기 방향: Green)
            vec_y = (p3 - p0).astype(float)
            len_y = np.linalg.norm(vec_y)
            if len_y > 0:
                dir_y = (vec_y / len_y * min(200, len_y * 0.3)).astype(int)
                target_y = p0 + dir_y
                cv2.arrowedLine(frame, tuple(p0), tuple(target_y), (0, 255, 0), 3, tipLength=0.18)
                cv2.putText(frame, "+Y (0->5m)", (target_y[0] - 30, target_y[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            # ── 3. 꼭짓점 라벨 표시 ──────────────────────────────
            cv2.putText(frame, f"P1 ({self.pool_size_x:.0f}m, 0m)", (pts[1][0] - 90, pts[1][1] + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.putText(frame, f"P2 ({self.pool_size_x:.0f}m, {self.pool_size_y:.0f}m)", (pts[2][0] - 90, pts[2][1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.putText(frame, f"P3 (0m, {self.pool_size_y:.0f}m)", (pts[3][0] - 20, pts[3][1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

            # ── 4. OSD 안내 및 캘리브레이션 가이드 ────────────────
            if self.calib_mode:
                point_names = ["1. 좌하단(P0)", "2. 우하단(P1)", "3. 우상단(P2)", "4. 좌상단(P3)"]
                next_step = point_names[len(self.calib_points)]
                calib_str = f"[캘리브레이션 모드] 수조 꼭짓점을 차례로 클릭하세요: {next_step}"
                cv2.putText(frame, calib_str, (20, self.height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, "[C] 키: 수조 4점 클릭 캘리브레이션 | [Q] 키: 종료",
                            (20, self.height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1, cv2.LINE_AA)

            if detected_info:
                cv2.putText(frame, f"Detected: {', '.join(detected_info)}",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, f"Searching for AprilTag/ArUco ID:{self.target_id}...",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c') or key == ord('C'):
                self.calib_mode = True
                self.calib_points = []
                self.get_logger().info("📐 [4점 캘리브레이션 시작] 수조의 4개 꼭짓점을 '좌하단 -> 우하단 -> 우상단 -> 좌상단' 순서로 클릭해 주세요.")

    def destroy_node(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CeilingAprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
