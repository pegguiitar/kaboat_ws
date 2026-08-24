"""ceiling_apriltag_node — OpenCV 기반 천장 AprilTag/ArUco 검출 및 TF 발행 ROS 2 노드.

특징:
  1. 수조의 실제 파란색 직사각형 좌하단 꼭짓점을 원점 (0,0)으로 설정
  2. 마우스 클릭으로 수조 꼭짓점을 실시간으로 보정(Calibration) 가능
  3. 실시간 Odom 좌표 (X, Y >= 0), 선수각(Yaw), 거리 시각화
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

        # 수조 좌하단 꼭짓점 픽셀 좌표 (기본값: 사진 상의 파란색 수조 좌하단 모서리)
        self.declare_parameter('origin_pixel_u', 90)
        self.declare_parameter('origin_pixel_v', 630)
        self.declare_parameter('ceiling_height', 4.60)  # [m]

        device_str = str(self.get_parameter('video_device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.target_id = int(self.get_parameter('tag_id').value)
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.tag_frame = str(self.get_parameter('tag_frame').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.publish_img = bool(self.get_parameter('publish_image').value)
        self.ceiling_height = float(self.get_parameter('ceiling_height').value)

        self.origin_u = int(self.get_parameter('origin_pixel_u').value)
        self.origin_v = int(self.get_parameter('origin_pixel_v').value)

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

        # 지원 딕셔너리 로드
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

        self.window_name = "Ceiling AprilTag Tracker (Click to set Pool Origin)"
        self.window_initialized = False

        self.create_timer(1.0 / 30.0, self._process_frame)
        self.get_logger().info(
            f"ceiling_apriltag_node 시작 — 장치 {device_str} @ {self.width}x{self.height}, "
            f"수조 좌하단 꼭짓점 원점: ({self.origin_u}, {self.origin_v})")

    def _on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.origin_u = x
            self.origin_v = y
            # 3D 위치 오프셋 계산 (m)
            x_m = (self.origin_u - self.cx) * (self.ceiling_height / self.fx)
            y_m = (self.origin_v - self.cy) * (self.ceiling_height / self.fy)
            self.get_logger().info(
                f"🎯 [수조 원점 갱신] 클릭 위치: 픽셀({x}, {y}) -> 카메라 기준 3D 오프셋: [X:{x_m:+.2f}m, Y:{y_m:+.2f}m]")

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

        # 원점 3D 좌표 (카메라 기준 [m])
        origin_x_cam = (self.origin_u - self.cx) * (self.ceiling_height / self.fx)
        origin_y_cam = (self.origin_v - self.cy) * (self.ceiling_height / self.fy)

        for fam_name, adict in self.dict_families.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, adict, parameters=self.params)
            if ids is not None and len(ids) > 0:
                for i, tid in enumerate(ids.flatten()):
                    detected_info.append(f"{fam_name} ID:{tid}")
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[tid]]))

                    if not tag_found and (self.target_id <= 0 or tid == self.target_id):
                        tag_found = True
                        c = corners[i][0]
                        s = self.tag_size / 2.0
                        obj_pts = np.array([
                            [-s,  s, 0.0],
                            [ s,  s, 0.0],
                            [ s, -s, 0.0],
                            [-s, -s, 0.0]
                        ], dtype=np.float64)

                        _, rvec, tvec = cv2.solvePnP(
                            obj_pts, c.astype(np.float64), self.camera_matrix, self.dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, 'SOLVEPNP_IPPE_SQUARE') else cv2.SOLVEPNP_ITERATIVE
                        )

                        R, _ = cv2.Rodrigues(rvec)
                        qx, qy, qz, qw = rot_matrix_to_quaternion(R)
                        tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])

                        # 수조 좌하단 꼭짓점 기준 보트 Odom 좌표 계산
                        x_odom = tx - origin_x_cam
                        y_odom = -(ty - origin_y_cam)  # 카메라 Y(아래) -> 수조 Y(위)

                        tag_frame_name = f"tag36h11:{tid}" if self.target_id <= 0 else self.tag_frame

                        # 1. TF 발행
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

                        # 2. PoseStamped 발행 (수조 원점 기준)
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = stamp
                        pose_msg.header.frame_id = self.camera_frame
                        pose_msg.pose.position.x = tx
                        pose_msg.pose.position.y = ty
                        pose_msg.pose.position.z = tz
                        pose_msg.pose.orientation.x = qx
                        pose_msg.pose.orientation.y = qy
                        pose_msg.pose.orientation.z = qz
                        pose_msg.pose.orientation.w = qw
                        self.pose_pub.publish(pose_msg)

                        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                        yaw_deg = math.degrees(yaw_rad)

                        # 터미널 로깅
                        self.get_logger().info(
                            f"✅ [{fam_name}] ID:{tid} | 수조좌표:[X:{x_odom:+.2f}, Y:{y_odom:+.2f}]m | Yaw:{yaw_deg:+.1f}°",
                            throttle_duration_sec=1.0)

                        u_center = int(c[:, 0].mean())
                        v_center = int(c[:, 1].mean())

                        # 좌하단 원점 -> 보트 연결선
                        cv2.line(frame, (self.origin_u, self.origin_v), (u_center, v_center), (0, 255, 255), 2, cv2.LINE_AA)

                        # 보트 위 수조 좌표 표시
                        info_str = f"Pool [X:{x_odom:+.2f}m, Y:{y_odom:+.2f}m] Yaw:{yaw_deg:+.1f}deg"
                        cv2.putText(frame, info_str, (u_center - 80, v_center - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        if self.show_gui:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name)
                cv2.setMouseCallback(self.window_name, self._on_mouse_click)
                self.window_initialized = True

            # ── 1. 수조 좌하단 꼭짓점 원점 (0,0) 시각화 ───────────────
            uo = self.origin_u
            vo = self.origin_v

            # 원점 마커 (황금색 타겟 링)
            cv2.circle(frame, (uo, vo), 20, (0, 215, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, (uo, vo), 6, (0, 215, 255), -1)
            cv2.drawMarker(frame, (uo, vo), (0, 215, 255), cv2.MARKER_CROSS, 36, 2)

            # ── 2. 꼭짓점 기준 좌표축 (+X: 우측, +Y: 상단) ─────────
            # +X 축 (수조 하단 변을 따라 오른쪽: Red)
            cv2.arrowedLine(frame, (uo, vo), (uo + 150, vo), (0, 0, 255), 3, tipLength=0.15)
            cv2.putText(frame, "+X (Pool Right)", (uo + 160, vo + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

            # +Y 축 (수조 좌측 변을 따라 위쪽: Green)
            cv2.arrowedLine(frame, (uo, vo), (uo, vo - 150), (0, 255, 0), 3, tipLength=0.15)
            cv2.putText(frame, "+Y (Pool Forward)", (uo - 20, vo - 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

            # 원점 라벨
            cv2.putText(frame, "Origin (0,0) [Pool Bottom-Left Corner]", (uo + 15, vo + 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2, cv2.LINE_AA)

            # ── 3. OSD 상태 바 및 클릭 안내 ────────────────────────
            cv2.putText(frame, "[TIP] Click on the image to set/adjust Pool Bottom-Left Corner (0,0)",
                        (20, self.height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1, cv2.LINE_AA)

            if detected_info:
                cv2.putText(frame, f"Detected: {', '.join(detected_info)}",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, f"Searching for AprilTag/ArUco ID:{self.target_id}...",
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)

        # 4. 디버그 이미지 발행
        if self.publish_img and self.img_pub.get_subscription_count() > 0:
            img_msg = Image()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.camera_frame
            img_msg.height = frame.shape[0]
            img_msg.width = frame.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = frame.shape[1] * 3
            img_msg.data = frame.tobytes()
            self.img_pub.publish(img_msg)

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
