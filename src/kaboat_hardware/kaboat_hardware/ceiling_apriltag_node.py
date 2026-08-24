"""ceiling_apriltag_node — 수조 4면 전체 1m 실측 캘리브레이션 기반 AprilTag 추적기.

특징:
  1. config/pool_calibration.yaml 에서 4개 변(하단 11점, 우측 6점, 상단 11점, 좌측 6점) 자동 로드
  2. 4면 전체 1m 실측 간격을 완벽히 반영한 Coons Patch 곡면 서피스 좌표계 생성
  3. 2D Newton-Raphson 역투영으로 보트의 정확한 X(0~10m), Y(0~5m), Yaw 오도메트리 산출
  4. TF(ceiling_camera -> tag36h11:1) 및 PoseStamped(/detections) 실시간 발행
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

        # 4개 변 기본 제어점 초기화
        self.bottom_x_pts = [[56.0 + i * (1246.0 - 56.0) / 10.0, 624.0] for i in range(11)]
        self.right_y_pts = [[1246.0, 624.0 - i * (624.0 - 121.0) / 5.0] for i in range(6)]
        self.top_x_pts = [[255.0 + i * (1246.0 - 255.0) / 10.0, 96.0 + i * (121.0 - 96.0) / 10.0] for i in range(11)]
        self.left_y_pts = [[56.0 + i * (255.0 - 56.0) / 5.0, 624.0 - i * (624.0 - 96.0) / 5.0] for i in range(6)]

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

        self.window_name = "Ceiling AprilTag Tracker (4-Edge Calibrated)"
        self.window_initialized = False

        self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info(
            f"ceiling_apriltag_node 시작 — 수조 4면 전체 1m 실측 좌표계 적용됨 ({self.pool_size_x}x{self.pool_size_y}m)")

    def _find_calibration_file(self):
        ws_path = os.path.join(os.path.expanduser('~'), 'Desktop', '2026KABOAT_REAL',
                               'src', 'kaboat_hardware', 'config', 'pool_calibration.yaml')
        if os.path.exists(ws_path):
            return ws_path
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
                    d = yaml.safe_load(f)
                    if 'bottom_x_pts' in d:
                        self.bottom_x_pts = d['bottom_x_pts']
                    if 'right_y_pts' in d:
                        self.right_y_pts = d['right_y_pts']
                    if 'top_x_pts' in d:
                        self.top_x_pts = d['top_x_pts']
                    if 'left_y_pts' in d:
                        self.left_y_pts = d['left_y_pts']
                self.get_logger().info(f"✅ 수조 4면 캘리브레이션 로드 완료: {self.calib_file}")
            except Exception as e:
                self.get_logger().warn(f"캘리브레이션 파일 로드 오류: {e}")

    def _get_curve_pt(self, norm_val, pts_list):
        n = len(pts_list)
        if n == 0:
            return np.zeros(2, dtype=np.float64)
        if n == 1:
            return np.array(pts_list[0], dtype=np.float64)

        idx_float = norm_val * (n - 1)
        if idx_float <= 0.0:
            p0 = np.array(pts_list[0], dtype=np.float64)
            p1 = np.array(pts_list[1], dtype=np.float64)
            return p0 + idx_float * (p1 - p0)
        elif idx_float >= (n - 1):
            p_last = np.array(pts_list[-1], dtype=np.float64)
            p_prev = np.array(pts_list[-2], dtype=np.float64)
            return p_last + (idx_float - (n - 1)) * (p_last - p_prev)
        else:
            i0 = int(math.floor(idx_float))
            i1 = min(i0 + 1, n - 1)
            t = idx_float - i0
            return (1.0 - t) * np.array(pts_list[i0], dtype=np.float64) + t * np.array(pts_list[i1], dtype=np.float64)

    def _coons_patch(self, u, v):
        c_bot = self._get_curve_pt(u, self.bottom_x_pts)
        c_top = self._get_curve_pt(u, self.top_x_pts)
        c_left = self._get_curve_pt(v, self.left_y_pts)
        c_right = self._get_curve_pt(v, self.right_y_pts)

        P0 = np.array(self.bottom_x_pts[0], dtype=np.float64)
        P1 = np.array(self.bottom_x_pts[-1], dtype=np.float64)
        P3 = np.array(self.top_x_pts[0], dtype=np.float64)
        P2 = np.array(self.top_x_pts[-1], dtype=np.float64)

        corner_blend = (1.0 - u) * (1.0 - v) * P0 + u * (1.0 - v) * P1 + (1.0 - u) * v * P3 + u * v * P2
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
                        p0_int = tuple(np.array(self.bottom_x_pts[0], dtype=int))
                        cv2.line(frame, p0_int, (int(u_center), int(v_center)), (0, 255, 255), 2, cv2.LINE_AA)

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

            # ── 3. 원점 및 축 화살표 ──────────────────────────────
            p0 = tuple(np.array(self.bottom_x_pts[0], dtype=int))
            cv2.circle(frame, p0, 16, (0, 215, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, p0, 5, (0, 215, 255), -1)
            cv2.putText(frame, "Origin (0,0)", (p0[0] - 20, p0[1] + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

            pt_x_arrow = tuple(self._coons_patch(0.20, 0.0).astype(int))
            cv2.arrowedLine(frame, p0, pt_x_arrow, (0, 0, 255), 3, tipLength=0.2)
            cv2.putText(frame, "+X (0->10m)", (pt_x_arrow[0] + 10, pt_x_arrow[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

            pt_y_arrow = tuple(self._coons_patch(0.0, 0.25).astype(int))
            cv2.arrowedLine(frame, p0, pt_y_arrow, (0, 255, 0), 3, tipLength=0.2)
            cv2.putText(frame, "+Y (0->5m)", (pt_y_arrow[0] - 30, pt_y_arrow[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

            # ── 4. OSD 안내 ─────────────────────────────────────
            cv2.putText(frame, "4-Edge 1m Grid Applied | Run 'python3 scripts/calibrate_pool_grid.py' to recalibrate",
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
