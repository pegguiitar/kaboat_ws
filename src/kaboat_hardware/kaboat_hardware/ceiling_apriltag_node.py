"""ceiling_apriltag_node — 실내 수조 정밀 캘리브레이션(좌측 6점 + 상단 11점) 기반 AprilTag 추적기.

특징:
  1. config/pool_calibration.yaml 에서 좌측 6점(0~5m) 및 상단 11점(0~10m)을 자동 로드
  2. 수조 좌측 및 상단 벽면의 1m 실측 간격을 완벽하게 반영한 Coons Patch 곡면 서피스 모델
  3. 화면 밖으로 잘린 우하단 꼭짓점(P1: 10m, 0m) 자동 외삽/보간
  4. 2D Newton-Raphson 역투영으로 보트의 정확한 X(0~10m), Y(0~5m), Yaw 오도메트리 산출
"""

import math
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory


def quad_bezier(A, M, B, t):
    """2차 베지어 곡선 보간."""
    return (1.0 - t)**2 * A + 2.0 * (1.0 - t) * t * M + t**2 * B


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

        # 기본 제어점 초기화
        self.y_control_pts = [
            np.array([78.0, 645.0]),   # Y=0m
            np.array([92.0, 530.0]),   # Y=1m
            np.array([110.0, 415.0]),  # Y=2m
            np.array([135.0, 300.0]),  # Y=3m
            np.array([162.0, 185.0]),  # Y=4m
            np.array([195.0, 78.0])    # Y=5m
        ]
        self.top_x_control_pts = [
            np.array([195.0 + i * (1175.0 - 195.0) / 10.0, 78.0 + i * (95.0 - 78.0) / 10.0])
            for i in range(11)
        ]

        self.calib_file = self._find_calibration_file()
        self._load_calibration()

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

        self.window_name = "Ceiling AprilTag Tracker"
        self.window_initialized = False

        self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info(
            f"ceiling_apriltag_node 시작 — 수조 캘리브레이션 적용됨 ({self.pool_size_x}x{self.pool_size_y}m)")

    def _find_calibration_file(self):
        # 1) 소스 디렉토리
        ws_path = os.path.join(os.path.expanduser('~'), 'Desktop', '2026KABOAT_REAL',
                               'src', 'kaboat_hardware', 'config', 'pool_calibration.yaml')
        if os.path.exists(ws_path):
            return ws_path
        # 2) share 디렉토리
        try:
            share_dir = get_package_share_directory('kaboat_hardware')
            share_path = os.path.join(share_dir, 'config', 'pool_calibration.yaml')
            if os.path.exists(share_path):
                return share_path
        except Exception:
            pass
        return ws_path

    def _load_calibration(self):
        if os.path.exists(self.calib_file):
            try:
                with open(self.calib_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if 'y_control_pts' in data:
                        self.y_control_pts = [np.array(p, dtype=np.float64) for p in data['y_control_pts']]
                    if 'top_x_control_pts' in data:
                        self.top_x_control_pts = [np.array(p, dtype=np.float64) for p in data['top_x_control_pts']]
                self.get_logger().info(f"✅ 수조 캘리브레이션 로드 성공: {self.calib_file}")
            except Exception as e:
                self.get_logger().warn(f"캘리브레이션 파일 로드 오류: {e}")

        self.P0 = self.y_control_pts[0].copy()
        self.P3 = self.y_control_pts[-1].copy()
        self.P2 = self.top_x_control_pts[-1].copy()
        vec_left = self.P0 - self.P3
        self.P1 = np.array([self.P2[0] - vec_left[0] * 1.08, self.P0[1] + 30.0], dtype=np.float64)
        self.M_bot = np.array([640.0, 716.0], dtype=np.float64)

    def _get_left_pt(self, v_norm):
        idx_float = v_norm * (len(self.y_control_pts) - 1)
        i0 = int(math.floor(idx_float))
        i1 = min(i0 + 1, len(self.y_control_pts) - 1)
        t = idx_float - i0
        return (1.0 - t) * self.y_control_pts[i0] + t * self.y_control_pts[i1]

    def _get_top_pt(self, u_norm):
        idx_float = u_norm * (len(self.top_x_control_pts) - 1)
        i0 = int(math.floor(idx_float))
        i1 = min(i0 + 1, len(self.top_x_control_pts) - 1)
        t = idx_float - i0
        return (1.0 - t) * self.top_x_control_pts[i0] + t * self.top_x_control_pts[i1]

    def _coons_patch(self, u, v):
        c_bot = quad_bezier(self.P0, self.M_bot, self.P1, u)
        c_top = self._get_top_pt(u)
        c_left = self._get_left_pt(v)
        c_right = (1.0 - v) * self.P1 + v * self.P2
        corner_blend = (1.0 - u) * (1.0 - v) * self.P0 + u * (1.0 - v) * self.P1 + (1.0 - u) * v * self.P3 + u * v * self.P2
        return (1.0 - v) * c_bot + v * c_top + (1.0 - u) * c_left + u * c_right - corner_blend

    def _pixel_to_pool_metric(self, target_px):
        u = 0.5
        v = 0.5
        eps = 1e-4
        for _ in range(12):
            pos = self._coons_patch(u, v)
            err = pos - target_px
            if np.linalg.norm(err) < 1e-3:
                break
            du = (self._coons_patch(u + eps, v) - self._coons_patch(u - eps, v)) / (2 * eps)
            dv = (self._coons_patch(u, v + eps) - self._coons_patch(u, v - eps)) / (2 * eps)
            J = np.column_stack([du, dv])
            delta = np.linalg.lstsq(J, -err, rcond=None)[0]
            u += delta[0]
            v += delta[1]
            u = np.clip(u, -0.3, 1.3)
            v = np.clip(v, -0.3, 1.3)

        return float(u * self.pool_size_x), float(v * self.pool_size_y)

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
                        c = corners[i][0]
                        u_center = float(c[:, 0].mean())
                        v_center = float(c[:, 1].mean())

                        # 수조 실제 좌표 산출
                        x_pool, y_pool = self._pixel_to_pool_metric(np.array([u_center, v_center]))

                        # 수조 평면 기준 Yaw 산출
                        u_fwd = float((c[0][0] + c[1][0]) / 2.0)
                        v_fwd = float((c[0][1] + c[1][1]) / 2.0)
                        x_fwd, y_fwd = self._pixel_to_pool_metric(np.array([u_fwd, v_fwd]))
                        yaw_rad = math.atan2(y_fwd - y_pool, x_fwd - x_pool)
                        yaw_deg = math.degrees(yaw_rad)

                        # PnP 3D 위치
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

                        # PoseStamped 발행
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = stamp
                        pose_msg.header.frame_id = "odom"
                        pose_msg.pose.position.x = x_pool
                        pose_msg.pose.position.y = y_pool
                        pose_msg.pose.position.z = 0.0
                        pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
                        pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
                        self.pose_pub.publish(pose_msg)

                        # 터미널 로깅
                        self.get_logger().info(
                            f"✅ [{fam_name}] ID:{tid} | 🌊 수조좌표:[X:{x_pool:.2f}m, Y:{y_pool:.2f}m] | Yaw:{yaw_deg:+.1f}°",
                            throttle_duration_sec=1.0)

                        # 원점 -> 보트 연결선
                        p0_int = self.P0.astype(int)
                        cv2.line(frame, (p0_int[0], p0_int[1]), (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)

                        # 보트 위 수조 좌표 표시
                        info_str = f"Pool [X:{x_pool:.2f}m, Y:{y_pool:.2f}m] Yaw:{yaw_deg:+.1f}deg"
                        cv2.putText(frame, info_str, (int(u_center) - 80, int(v_center) - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        if self.show_gui:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name)
                self.window_initialized = True

            # ── 1. 수조 4변 곡면 테두리 렌더링 ─────────────────────
            t_samples = np.linspace(0.0, 1.0, 50)
            bot_pts = np.array([self._coons_patch(t, 0.0) for t in t_samples], dtype=np.int32)
            top_pts = np.array([self._coons_patch(t, 1.0) for t in t_samples], dtype=np.int32)
            left_pts = np.array([self._coons_patch(0.0, t) for t in t_samples], dtype=np.int32)
            right_pts = np.array([self._coons_patch(1.0, t) for t in t_samples], dtype=np.int32)

            cv2.polylines(frame, [bot_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
            cv2.polylines(frame, [top_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
            cv2.polylines(frame, [left_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)
            cv2.polylines(frame, [right_pts], isClosed=False, color=(0, 255, 0), thickness=3, lineType=cv2.LINE_AA)

            # ── 2. 곡면 내부 원근 격자망 (1m x 1m 격자) ────────────
            for gx in range(1, int(self.pool_size_x)):
                u_norm = gx / self.pool_size_x
                line_pts = np.array([self._coons_patch(u_norm, t) for t in t_samples], dtype=np.int32)
                cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 150, 80), thickness=1, lineType=cv2.LINE_AA)

            for gy in range(1, int(self.pool_size_y)):
                v_norm = gy / self.pool_size_y
                line_pts = np.array([self._coons_patch(t, v_norm) for t in t_samples], dtype=np.int32)
                cv2.polylines(frame, [line_pts], isClosed=False, color=(80, 150, 80), thickness=1, lineType=cv2.LINE_AA)

            # ── 3. Y축 1m 간격 제어점 마커 표시 ────────────────────
            for m_idx, pt in enumerate(self.y_control_pts):
                p_int = pt.astype(int)
                cv2.circle(frame, tuple(p_int), 5, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.putText(frame, f"Y={m_idx}m", (p_int[0] + 10, p_int[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

            # ── 4. 상단 X축 1m 간격 제어점 마커 표시 ───────────────
            for m_idx, pt in enumerate(self.top_x_control_pts):
                p_int = pt.astype(int)
                cv2.circle(frame, tuple(p_int), 5, (0, 255, 100), -1, cv2.LINE_AA)
                cv2.putText(frame, f"X={m_idx}m", (p_int[0] - 12, p_int[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 100), 1, cv2.LINE_AA)

            # ── 5. 좌표축 화살표 ─────────────────────────────────
            p0 = self.P0.astype(int)
            pt_x_arrow = self._coons_patch(0.20, 0.0).astype(int)
            cv2.arrowedLine(frame, tuple(p0), tuple(pt_x_arrow), (0, 0, 255), 3, tipLength=0.2)
            cv2.putText(frame, "+X Axis (0->10m)", (pt_x_arrow[0] + 10, pt_x_arrow[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

            pt_y_arrow = self.y_control_pts[1].astype(int)
            cv2.arrowedLine(frame, tuple(p0), tuple(pt_y_arrow), (0, 255, 0), 3, tipLength=0.2)
            cv2.putText(frame, "+Y Axis (0->5m)", (pt_y_arrow[0] - 25, pt_y_arrow[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            # ── 6. OSD 안내 ─────────────────────────────────────
            cv2.putText(frame, "Loaded from pool_calibration.yaml | Run scripts/calibrate_pool_grid.py to calibrate",
                        (20, self.height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1, cv2.LINE_AA)

            if detected_info:
                cv2.putText(frame, f"Detected: {', '.join(detected_info)}",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, f"Searching for AprilTag/ArUco ID:{self.target_id}...",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

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
