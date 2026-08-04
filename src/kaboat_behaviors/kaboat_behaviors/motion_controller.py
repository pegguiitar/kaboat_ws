"""미션 공통 속도벡터 조종기 (ROS 무의존).

각 behavior/planner 는 odom 월드 좌표계의 목표 속도벡터만 만든다. 이 모듈은
그 벡터를 선체가 실제로 따라가도록 목표 선수각, yaw PD, 전진/후진 추력
명령으로 변환한다. Gate/Search/Station의 충돌 반발벡터도 조종기 앞에서
명목 속도벡터에 합성하면 같은 제어기를 그대로 사용할 수 있다.

좌표/단위 계약:
  DesiredVelocity.vx, vy : odom frame [m/s]
  yaw, yaw_rate          : odom 기준 선수각 [rad], 요레이트 [rad/s]
  ControlCommand         : 현재 스택의 정규화 추력 명령 linear.x/angular.z
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DesiredVelocity:
    """월드 좌표계 목표 속도벡터.

    reverse=True이면 벡터는 선미가 움직여야 할 월드 방향이다. 속도벡터만으로는
    선수를 돌려 전진할지 현재 자세로 후진할지가 모호하므로 이를 명시한다.
    """

    vx: float
    vy: float
    reverse: bool = False


@dataclass(frozen=True)
class MotionControllerParams:
    yaw_kp: float
    yaw_kd: float
    cmd_full: float
    v_full: float
    cmd_exp: float
    turn_slow_angle: float = math.pi / 3


@dataclass(frozen=True)
class ControlCommand:
    linear_x: float
    angular_z: float
    heading_error: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def speed_to_command(speed: float, p: MotionControllerParams) -> float:
    """물리 속도[m/s]를 현재 스택의 정규화 추력 명령으로 변환한다."""
    if p.v_full <= 0.0:
        raise ValueError('v_full must be positive')
    return p.cmd_full * (max(0.0, speed) / p.v_full) ** p.cmd_exp


def control_velocity(desired: DesiredVelocity, yaw: float, yaw_rate: float,
                     p: MotionControllerParams,
                     turn_speed_limit: float | None = None) -> ControlCommand:
    """목표 속도벡터를 정규화된 전후진·회전 명령으로 바꾼다.

    turn_speed_limit은 큰 방위 오차에서의 속도 상한[m/s]이다. None이면 후진
    ESCAPE처럼 방향 오차와 무관하게 요청 속도를 유지한다.
    """
    speed = math.hypot(desired.vx, desired.vy)
    if speed <= 1e-12:
        return ControlCommand(0.0, 0.0, 0.0)

    travel_yaw = math.atan2(desired.vy, desired.vx)
    reference_yaw = yaw + math.pi if desired.reverse else yaw
    error = wrap_angle(travel_yaw - reference_yaw)

    if (turn_speed_limit is not None
            and abs(error) > p.turn_slow_angle):
        speed = min(speed, max(0.0, turn_speed_limit))

    linear = speed_to_command(speed, p)
    if desired.reverse:
        linear = -linear
    angular = p.yaw_kp * error - p.yaw_kd * yaw_rate
    return ControlCommand(linear, angular, error)
