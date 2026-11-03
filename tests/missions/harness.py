"""L2 미션 시험 공통 폐루프 — 선체 모델 ↔ behavior 노드.

매 제어 주기(0.1 s):
  1) 가짜 시계를 0.1 s 진행한다.
  2) 선체 상태에 odom 잡음을 더해 /odom 으로 넣는다 (odom 끊김 주입 시 생략).
  3) 미션별 센서 hook 이 scan·격자·도킹 좌표 등을 넣는다.
  4) node._tick() 을 부른다 — 실제 10 Hz timer 와 같은 경로라 명령 상한·추력 비율이 그대로 적용된다.
  5) 발행된 /cmd/<mission> 명령으로 선체 모델을 0.1 s 적분한다.
     발행이 없으면 cmd_mux 워치독처럼 정지(0)로 본다.

노드는 spin 하지 않으므로 ROS 실행 없이 결정적으로 재현된다.
"""
from dataclasses import dataclass, field
import os

import rclpy
from std_msgs.msg import String

from tests.sim.boat_model import BoatModel, BoatState, OdomNoise
from tests.sim.recorder import Recorder, figure_dir
from tests.sim.ros_adapters import FakeClock, odom_msg, pose_msg

DT = 0.1
DEFAULT_SEEDS = 5


def seeds():
    """S2 반복 seed 목록. KABOAT_SEEDS 로 늘릴 수 있다 (TEST_PLAN.md 4.3)."""
    return list(range(int(os.environ.get('KABOAT_SEEDS', DEFAULT_SEEDS))))


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@dataclass
class Scenario:
    """S1/S2/S3 조건. S2 는 seed 마다 잡음·조류·초기 자세가 달라진다."""
    name: str = 'S1'
    seed: int = 0
    position_std: float = 0.0
    yaw_std: float = 0.0
    current: tuple = (0.0, 0.0)
    start_offset: tuple = (0.0, 0.0, 0.0)   # (dx, dy, dyaw)

    @classmethod
    def nominal(cls):
        return cls('S1')

    @classmethod
    def disturbed(cls, seed: int):
        import numpy as np
        rng = np.random.default_rng(1000 + seed)
        angle = rng.uniform(-3.1416, 3.1416)
        speed = 0.05 + 0.05 * rng.random()          # 조류 0.05~0.10 m/s
        return cls('S2', seed, position_std=0.03, yaw_std=0.01,
                   current=(speed * float(np.cos(angle)), speed * float(np.sin(angle))),
                   start_offset=(float(rng.uniform(-0.5, 0.5)), float(rng.uniform(-0.5, 0.5)),
                                 float(rng.uniform(-0.2, 0.2))))

    @classmethod
    def failure(cls):
        return cls('S3')


@dataclass
class MissionSim:
    node: object
    start: BoatState
    scenario: Scenario = field(default_factory=Scenario.nominal)
    world: object = None
    hooks: list = field(default_factory=list)
    clock: FakeClock = field(default_factory=FakeClock)
    status_fn: object = None      # node → 기록할 상태 문자열 (없으면 node._status_text)

    def __post_init__(self):
        dx, dy, dyaw = self.scenario.start_offset
        s = self.start
        self.boat = BoatModel(BoatState(x=s.x + dx, y=s.y + dy, yaw=s.yaw + dyaw),
                              current=self.scenario.current)
        self.noise = OdomNoise(self.scenario.position_std, self.scenario.yaw_std,
                               seed=self.scenario.seed)
        self.rec = Recorder()
        self.node.cmd_pub = Spy()
        self.node.complete_pub = Spy()
        self.odom_alive = True
        self.last_odom = None
        self.cmd = (0.0, 0.0)

    @property
    def t(self):
        return self.boat.time

    def activate(self, goal_xy, goal_yaw=0.0):
        self.node._on_state(String(data=self.node.STATE_NAME))
        self.node._on_goal(pose_msg(*goal_xy, goal_yaw))

    def step(self):
        self.clock.advance(DT)
        if self.odom_alive:
            msg = odom_msg(self.noise.measure(self.boat.state))
            msg.header.stamp = self.clock_msg()
            self.node._on_odom(msg)
            self.last_odom = msg
        for hook in self.hooks:
            hook(self)
        sent = len(self.node.cmd_pub.msgs)
        self.node._tick()
        if len(self.node.cmd_pub.msgs) > sent:
            cmd = self.node.cmd_pub.msgs[-1]
            self.cmd = (float(cmd.linear.x), float(cmd.angular.z))
        else:
            self.cmd = (0.0, 0.0)                   # 침묵 → cmd_mux 워치독 정지와 같다
        self.boat.step(*self.cmd, DT)
        status = (self.status_fn(self.node) if self.status_fn is not None
                  else getattr(self.node, '_status_text', '') or '')
        self.rec.record(self.t, self.boat.state, *self.cmd, status)
        if self.node.complete_pub.msgs and self.rec.first_event_time('complete') is None:
            self.rec.event(self.t, 'complete')

    def clock_msg(self):
        from rclpy.time import Time
        return Time(nanoseconds=int(self.clock.now * 1e9)).to_msg()

    def run(self, max_time=120.0, until=None, after_complete=0.0):
        """완료 신호 뒤 after_complete 초 더 돌리거나, until(sim) 이 참이 될 때까지 돌린다."""
        while self.t < max_time:
            self.step()
            done_t = self.rec.first_event_time('complete')
            if until is not None and until(self):
                break
            if until is None and done_t is not None and self.t >= done_t + after_complete:
                break
        return self.rec

    @property
    def completed(self):
        return self.rec.first_event_time('complete') is not None


class LidarGrid:
    """가짜 라이다 → 실제 occupancy_grid 노드 → behavior 의 /occupancy_grid (와 /scan).

    world 를 광선 추적한 scan 을 occupancy_grid 노드에 넣고, 노드가 발행한 격자를
    그대로 behavior 에 넘긴다. feed_scan_to_behavior 면 behavior 의 _on_scan 도 부른다.
    scan_alive=False 로 바꾸면 라이다 끊김이 된다 (S3).
    """

    def __init__(self, world, feed_scan_to_behavior=False):
        from kaboat_perception.occupancy_grid import OccupancyGridNode
        self.world = world
        self.grid_node = OccupancyGridNode()
        self.grid_node.pub = Spy()
        self.feed_scan = feed_scan_to_behavior
        self.scan_alive = True

    def __call__(self, sim):
        from tests.sim.fake_sensors import scan_ranges
        from tests.sim.ros_adapters import scan_msg
        if sim.last_odom is not None:
            self.grid_node._on_odom(sim.last_odom)
        if self.scan_alive:
            s = sim.boat.state
            scan = scan_msg(scan_ranges(s.x, s.y, s.yaw, self.world))
            self.grid_node._on_scan(scan)
            self.grid_node._publish()
            sim.node._on_grid(self.grid_node.pub.msgs[-1])
            if self.feed_scan:
                sim.node._on_scan(scan)

    def close(self):
        self.grid_node.destroy_node()


def tank_walls(xmin, ymin, xmax, ymax):
    """수조 벽 4면 — 라이다 빔이 돌아와야 빈 칸도 '관측됨'이 된다."""
    from tests.sim.fake_sensors import Segment
    return [Segment(xmin, ymin, xmax, ymin), Segment(xmax, ymin, xmax, ymax),
            Segment(xmax, ymax, xmin, ymax), Segment(xmin, ymax, xmin, ymin)]


def make_node(cls, monkeypatch, clock, params=()):
    args = ['--ros-args']
    for p in params:
        args += ['-p', p]
    rclpy.init(args=args)
    node = cls()
    clock.install(monkeypatch, node)
    return node


def close(*nodes):
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def save_figure(name, rec, world=None, goal=None, extra=None):
    """KABOAT_TEST_FIGURES 가 있을 때만 궤적 그림을 저장한다."""
    out = figure_dir()
    if out is None:
        return None
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 5))
    if world is not None:
        for c in world.circles:
            ax.add_patch(plt.Circle((c.x, c.y), c.radius, color='0.35'))
        for s in world.segments:
            ax.plot([s.x0, s.x1], [s.y0, s.y1], color='0.2', lw=2)
    xs = [s.x for s in rec.samples]
    ys = [s.y for s in rec.samples]
    ax.plot(xs, ys, color='tab:blue', lw=1.5, label='boat')
    ax.plot(xs[0], ys[0], 'o', color='tab:blue')
    if goal is not None:
        ax.plot(*goal, '*', color='tab:green', ms=12, label='goal')
    done = rec.first_event_time('complete')
    if done is not None:
        k = min(range(len(rec.samples)), key=lambda i: abs(rec.samples[i].t - done))
        ax.plot(xs[k], ys[k], 'x', color='tab:red', ms=10, label=f'complete {done:.1f}s')
    if extra is not None:
        extra(ax)
    ax.set_aspect('equal', adjustable='datalim')
    ax.autoscale_view()
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    half = max(x1 - x0, y1 - y0, 6.0) / 2 + 0.5     # 한 변 최소 6 m — 직선 궤적도 납작하지 않게
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(loc='best', fontsize=8)
    ax.grid(alpha=0.3)
    path = out / f'{name}.png'
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
