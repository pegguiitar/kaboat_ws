#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Twist

class RVizInteractiveMarkerNode(Node):
    def __init__(self):
        super().__init__('rviz_interactive_marker')
        
        # Publisher for Twist messages
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize Interactive Marker Server
        self.server = InteractiveMarkerServer(self, 'wamv_joystick')
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.is_active = False
        
        # Create and initialize interactive marker
        self.init_marker()
        
        # Publish timer (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("RViz Interactive Marker Node initialized. Frame: wamv/base_link")

    def init_marker(self):
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "wamv/base_link"
        int_marker.name = "joystick"
        int_marker.description = "WAM-V virtual joystick (Drag to move)"
        int_marker.scale = 1.5
        
        # Position at the center of the boat
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.5  # Float slightly above the deck
        
        # Create a visual marker (cyan/blue sphere representing the joystick)
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.6
        sphere_marker.scale.y = 0.6
        sphere_marker.scale.z = 0.6
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.8
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 0.8  # Semi-transparent cyan
        
        # Control for dragging in the 2D plane (XY Plane)
        control_plane = InteractiveMarkerControl()
        control_plane.name = "move_xy"
        control_plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # Align y-z plane of control with the global x-y plane (rotate 90 deg around Y axis)
        control_plane.orientation.w = 0.707
        control_plane.orientation.x = 0.0
        control_plane.orientation.y = 0.707
        control_plane.orientation.z = 0.0
        control_plane.markers.append(sphere_marker)
        control_plane.always_visible = True
        int_marker.controls.append(control_plane)
        
        # Add 1D Translation arrows for visual guidance (X-axis: forward/backward)
        control_x = InteractiveMarkerControl()
        control_x.name = "move_x"
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_x.orientation.w = 1.0
        control_x.orientation.x = 0.0
        control_x.orientation.y = 0.0
        control_x.orientation.z = 0.0
        int_marker.controls.append(control_x)
        
        # Add 1D Translation arrows for visual guidance (Y-axis: left/right turn)
        control_y = InteractiveMarkerControl()
        control_y.name = "move_y"
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # Rotate local X-axis to align with global Y-axis (90 deg around Z-axis)
        control_y.orientation.w = 0.707
        control_y.orientation.x = 0.0
        control_y.orientation.y = 0.0
        control_y.orientation.z = 0.707
        int_marker.controls.append(control_y)
        
        # Insert marker and associate feedback callback
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.is_active = True
            
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.is_active = True
            # Read position offset relative to base_link
            dx = feedback.pose.position.x
            dy = feedback.pose.position.y
            
            # Map displacement (e.g. up to 2.0 meters) to Twist commands.
            # 명령 = 최대추력 비율 (twist2thrust 정규화, 2026-07-17) — 계수는
            # 구 scale=60 시절과 추력 N 동등: 45 N/m (클램프 ±90N), 30 N/m (±60N).
            # ⚠️ 이 클램프(±90N)는 검증된 안전값(12N)보다 훨씬 큼 — 정규화
            # 이전부터 그랬던 기존 위험(마커를 끝까지 끌면 전복 가능성). 동작
            # 불변 원칙으로 유지하나, 손볼 거면 별도 커밋으로.
            MAX_THRUST = 118.6
            self.linear_vel = float(max(-90.0, min(90.0, dx * 45.0))) / MAX_THRUST
            self.angular_vel = float(max(-60.0, min(60.0, dy * 30.0))) / MAX_THRUST
            
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Spring-loaded behavior: reset marker pose back to the origin
            feedback.pose.position.x = 0.0
            feedback.pose.position.y = 0.0
            feedback.pose.position.z = 0.5
            feedback.pose.orientation.w = 1.0
            feedback.pose.orientation.x = 0.0
            feedback.pose.orientation.y = 0.0
            feedback.pose.orientation.z = 0.0
            self.server.setPose(feedback.marker_name, feedback.pose)
            self.server.applyChanges()
            
            # Publish a final stop command
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            
            # Deactivate control to release /cmd_vel topic ownership
            self.is_active = False
            self.linear_vel = 0.0
            self.angular_vel = 0.0

    def timer_callback(self):
        # Only publish Twist commands when the user is actively manipulating the marker
        if self.is_active:
            msg = Twist()
            msg.linear.x = self.linear_vel
            msg.angular.z = self.angular_vel
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RVizInteractiveMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
