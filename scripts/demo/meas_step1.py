"""물리 재캘리브레이션 — 배를 정해진 순서로 몰면서 실측값을 뽑는다.

**실물 전환 때 제일 먼저 돌릴 스크립트.** AVOIDANCE.MD §4 의 속도·회전 관련
값(v_max, omega_max, cmd_exp, v_escape)과 SKELETON §6 의 물리 결합 상수가
전부 여기서 나온다. 새 선체·새 스러스터면 값이 통째로 바뀌므로 재측정 필수.

측정 항목:
  1. /odom twist 좌표계 — body vs world (헤딩 튼 뒤 전진, twist 벡터 각 비교)
  2. 후진 추력 — cmd -0.101 에서 속도·자세(roll/pitch) 안정성
  3. v↔cmd 중간점 — cmd 0.0506 (6N) 의 정상 속도 (speed_to_cmd 보정용)
  4. 후진 중 조향 부호 — 전진과 같은가 (차동추력 이론 확인)

시간은 전부 odom stamp (sim time, RTF≈0.3 이라 wall 시계 금지).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_pitch(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                      1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    s = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return roll, math.asin(s)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


PHASES = [   # (이름, cmd_lin, cmd_ang, 지속 sim 초)
    ('fwd_full',  0.1012, 0.0,    10.0),
    ('fwd_mid',   0.0506, 0.0,    10.0),
    ('coast',     0.0,    0.0,     3.0),
    ('rev_full', -0.1012, 0.0,    12.0),
    ('rev_turn', -0.06,   0.0253,  6.0),
    ('fwd_turn',  0.06,   0.0253,  6.0),
    ('fwd_skew',  0.1012, 0.0,     8.0),  # 헤딩 튼 상태 전진 — frame 판별
]


class Meas(Node):
    def __init__(self):
        super().__init__('meas_step1')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.t0 = None
        self.phase_i = 0
        self.phase_t0 = None
        self.samples = {n: [] for n, *_ in PHASES}
        self.done = False
        self.timer = self.create_timer(0.1, self.tick)  # cmd 재발행 10Hz wall
        self.last = None

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
            self.phase_t0 = t
        self.last = m
        if self.done or self.phase_i >= len(PHASES):
            return
        name, _, _, dur = PHASES[self.phase_i]
        tw, q = m.twist.twist, m.pose.pose.orientation
        r, p = roll_pitch(q)
        self.samples[name].append(dict(
            t=t - self.phase_t0, vx=tw.linear.x, vy=tw.linear.y,
            wz=tw.angular.z, yaw=yaw_of(q), roll=r, pitch=p,
            x=m.pose.pose.position.x, y=m.pose.pose.position.y))
        if t - self.phase_t0 >= dur:
            self.report_phase(name)
            self.phase_i += 1
            self.phase_t0 = t
            if self.phase_i >= len(PHASES):
                self.pub.publish(Twist())
                self.done = True

    def tick(self):
        if self.done or self.phase_i >= len(PHASES) or self.t0 is None:
            if self.done:
                self.pub.publish(Twist())
            return
        _, lin, ang, _ = PHASES[self.phase_i]
        c = Twist()
        c.linear.x = lin
        c.angular.z = ang
        self.pub.publish(c)

    def report_phase(self, name):
        ss = self.samples[name]
        tail = [s for s in ss if s['t'] >= ss[-1]['t'] - 4.0]
        mean = lambda k, arr: sum(s[k] for s in arr) / max(1, len(arr))
        vx, vy = mean('vx', tail), mean('vy', tail)
        wz = mean('wz', tail)
        mr = max(abs(s['roll']) for s in ss)
        mp = max(abs(s['pitch']) for s in ss)
        # frame 판별: 속도 벡터 각 vs 0(body 가설) vs yaw(world 가설)
        moving = [s for s in tail if math.hypot(s['vx'], s['vy']) > 0.3]
        frame = ''
        if moving:
            db = mean('_', [dict(_=abs(wrap(math.atan2(s['vy'], s['vx']))))
                            for s in moving])
            dw = mean('_', [dict(_=abs(wrap(math.atan2(s['vy'], s['vx'])
                                            - s['yaw'])))
                            for s in moving])
            frame = f' | frame: |Δbody|={db:.3f} |Δworld−yaw|={dw:.3f}'
        p0, p1 = ss[0], ss[-1]
        print(f"[{name}] tail4s: vx={vx:+.3f} vy={vy:+.3f} m/s  wz={wz:+.3f} rad/s"
              f"  yaw {p0['yaw']:+.2f}->{p1['yaw']:+.2f}"
              f"  max|roll|={mr:.3f} max|pitch|={mp:.3f}"
              f"  pos ({p0['x']:.1f},{p0['y']:.1f})->({p1['x']:.1f},{p1['y']:.1f})"
              f"{frame}", flush=True)


def main():
    rclpy.init()
    n = Meas()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=1.0)
        for _ in range(5):
            rclpy.spin_once(n, timeout_sec=0.2)  # 마지막 정지 cmd 몇 번 더
    except KeyboardInterrupt:
        pass
    print('MEAS DONE', flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
