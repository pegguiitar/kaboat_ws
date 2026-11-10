"""Use the existing avoidance follower, switching from entry to live stop goal."""
import math
import time
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from kaboat_hardware.real_obstacle_avoid_test import RealObstacleAvoidTest


class DockCtrl(RealObstacleAvoidTest):
    def __init__(self):
        super().__init__()
        self.declare_parameter('dock_final_switch_distance_m',3.0)
        self.declare_parameter('dock_stop_goal_topic','/dock/stop_point')
        self.declare_parameter('dock_final_replan_distance_m',.10)
        self.switch_distance=float(self.get_parameter('dock_final_switch_distance_m').value)
        self.replan_distance=float(self.get_parameter('dock_final_replan_distance_m').value)
        if not all(math.isfinite(v) and v>0 for v in (self.switch_distance,self.replan_distance)):
            raise ValueError('docking switch/replan distances must be positive')
        self.final_stage=False
        self.stop_goal=None
        self.stop_received=None
        self.planned_stop_goal=None
        self.create_subscription(PointStamped,str(self.get_parameter('dock_stop_goal_topic').value),
                                 self._on_stop_goal,qos_profile_sensor_data)
        self.create_service(Trigger,'/dock/start',self._on_start_service)
        self.create_service(Trigger,'/dock/stop',self._on_stop_service)

    def _on_stop_goal(self,msg):
        p=msg.point
        expected=self.odom.header.frame_id if self.odom is not None else 'odom'
        if (msg.header.frame_id!=(expected or 'odom') or abs(p.z)>1e-6
                or not all(math.isfinite(v) for v in (p.x,p.y,p.z))):
            self.stop_goal=self.stop_received=None
            return
        age=self.get_clock().now().nanoseconds*1e-9-self._goal_stamp(msg)
        if self._goal_stamp(msg)<=0 or not -.1<=age<=self.goal_timeout:return
        if self.stop_goal is not None and self._goal_stamp(msg)<=self._goal_stamp(self.stop_goal):return
        self.stop_goal=msg
        self.stop_received=time.monotonic()

    def _stop_goal_error(self,now):
        if self.stop_goal is None or self.stop_received is None:return 'waiting for final yellow stop goal'
        age=self.get_clock().now().nanoseconds*1e-9-self._goal_stamp(self.stop_goal)
        if now-self.stop_received>self.goal_timeout or not -.1<=age<=self.goal_timeout:
            return 'final yellow stop goal stale'
        if self.stop_goal.header.frame_id!=(self.odom.header.frame_id or 'odom'):
            return 'final stop goal and odom frame IDs differ'
        if self.tank_enabled and self._tank_edge(self.stop_goal.point.x,self.stop_goal.point.y)<self.tank_stop_margin:
            return 'final stop goal near tank boundary'
        return None

    def _input_error(self,now,*,require_goal=True):
        error=super()._input_error(now,require_goal=require_goal)
        if error:return error
        if require_goal and self.started and self.final_stage:
            return self._stop_goal_error(now)
        return None

    def _tick(self):
        if self.started and self.odom is not None and self.goal is not None:
            boat=self.odom.pose.pose.position
            switching=(not self.final_stage and math.hypot(boat.x-self.goal[0],boat.y-self.goal[1])<=self.switch_distance)
            if switching:self.final_stage=True
            if self.final_stage and self._stop_goal_error(time.monotonic()) is None:
                target=(self.stop_goal.point.x,self.stop_goal.point.y)
                if not switching and math.dist(target,self.goal)>self.goal_max_shift:
                    self._disarm('final yellow stop goal jumped')
                    self._publish_status('STOP','final yellow stop goal jumped; reacquire')
                    return
                self.goal=target
                if switching or self.planned_stop_goal is None or math.dist(target,self.planned_stop_goal)>=self.replan_distance:
                    self.fsm.reset()
                    self.planned_stop_goal=target
                if switching:self.get_logger().info(f'Within {self.switch_distance:.2f}m of entry: follow live final yellow stop goal')
        super()._tick()

    def _is_final_segment(self,plan):
        return self.final_stage or super()._is_final_segment(plan)

    def _publish_status(self,state,detail):
        stage='FINAL' if getattr(self,'final_stage',False) else 'ENTRY'
        super()._publish_status(state,f'{detail}, stage={stage}')

    def _disarm(self,reason):
        super()._disarm(reason)
        self.final_stage=False
        self.planned_stop_goal=None

    def _on_stop_service(self,request,response):
        self._disarm('dock stop requested');self._publish_status('STOP','dock stop requested')
        response.success=True;response.message='dock_ctrl stopped'
        return response


def main(args=None):
    rclpy.init(args=args);node=DockCtrl()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node._publish_command(Twist());node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
