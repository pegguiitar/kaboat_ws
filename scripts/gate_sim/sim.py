"""게이트 폐루프 시뮬레이션 — rec_bag 에서 잰 값(calibrate.py)으로 가상 라이다·배를 만들고
실제 occupancy_grid·gate_follower 코드를 그대로 돌린다.

  python3 scripts/gate_sim/sim.py calib.json gates.json --scenario base --seed 0 [-p gate.x:=v …]

세계: gates.json 의 게이트 부표(원, 반지름 post_radius) + calib 의 먼 구조물 칸(맞으면 free 가 칠해짐).
라이다: 2030 빔, 뒤 120°~240° 가림(inf), 부표는 거리별 검출률(calib)로 스캔마다 보이거나 무반사,
         배 근처 잡음 히트(calib 분포), 거리 잡음. 위치·선수각 잡음은 odom 에.
배: 정상 속도 표(명령→m/s, 1차 지연 τ), 회전 (g + g_u·u)·w (1차 지연), 바람 표류.
출력: JSON 한 줄 — 완료, 시간, 통과 게이트(정답과 맞춤), 정지 시간, 부표 최소 거리, 상태 시간.
"""

import argparse
import json
import math
import sys
import time as _time

import numpy as np

SCENARIOS = {
    'base': {},
    'wind10': {'wind': (0.0, 0.10)},             # 북쪽으로 0.1 m/s 표류 (현장 정지 중 2~4 m/30 s)
    'wind20': {'wind': (0.0, 0.20)},
    'wind20_head': {'wind': (-0.20, 0.0)},       # 맞바람 (진행 방향 반대)
    'weak_lidar': {'detect_scale': 0.7},          # 부표 검출률 70 %
    'clutter3': {'clutter_scale': 3.0},           # 배 근처 잡음 3배
    'start_off': {'start_dyaw': 25.0, 'start_dy': 1.0},   # 시작 선수 25°·옆 1 m 틀어짐
    'noisy_pose': {'pose_sigma': 0.15, 'yaw_sigma_deg': 4.0},
    # 첫 게이트 쌍이 안 잡히는 경우 (2026-11-14 첫 게이트 지키기·가까이 다가가기 검증)
    # post_detect: 부표 번호(gates.json 순서, 0·1 = 게이트 1 왼·오) → 검출률 배수
    'start_back3': {'start_back': 3.0},                          # 첫 게이트 약 6.7 m
    'start_back6': {'start_back': 6.0},                          # 첫 게이트 약 9.7 m
    'faint_g1L': {'post_detect': {0: 0.15}},                     # 게이트 1 왼 부표가 잘 안 보임
    'faint_g1R': {'post_detect': {1: 0.15}},                     # 게이트 1 오른 부표가 잘 안 보임
    'faint_g1R_wind10': {'post_detect': {1: 0.15}, 'wind': (0.0, 0.10)},
    'faint_g1R_back3': {'post_detect': {1: 0.15}, 'start_back': 3.0},
    'dead_g1R': {'post_detect': {1: 0.0}},                       # 게이트 1 오른 부표가 영영 안 보임
    # post_range: 부표 번호 → 이 거리 [m] 안에서만 보임 (작고 낮은 부표 — 가까이 가야 잡힘)
    'short_g1L': {'post_range': {0: 4.0}},
    'short_g1R': {'post_range': {1: 4.0}},
    'short_g1R_wind10': {'post_range': {1: 4.0}, 'wind': (0.0, 0.10)},
    'short_g1R_back3': {'post_range': {1: 4.0}, 'start_back': 3.0},
    'short3_g1R': {'post_range': {1: 3.0}},                      # 더 심함: 3 m 안에서만
}


def interp_rate(bins, rates, r):
    centers = [(a + b) / 2 for a, b in zip(bins[:-1], bins[1:])]
    return float(np.interp(r, [0.0] + centers + [40.0], [rates[0]] + list(rates) + [0.0]))


class World:
    def __init__(self, calib, gates, radius, radii=None):
        self.gates = [np.asarray(g, float) for g in gates]
        self.posts = np.vstack(self.gates)
        self.mids = np.array([g.mean(0) for g in self.gates])
        self.radius = (np.asarray(radii, float).ravel() if radii is not None
                       else np.full(len(self.posts), radius))
        cell = calib['far_cell']
        self.far_cell = cell
        self.far = set(map(tuple, calib['far_cells']))
        # 게이트 통로 안 먼 구조물 칸은 빼 둔다 (지도 잔상 — 실제 물)
        self.far = {c for c in self.far
                    if np.min(np.linalg.norm(self.posts - ((c[0] + .5) * cell, (c[1] + .5) * cell),
                                             axis=1)) > 3.0}


class Lidar:
    N = 2030

    def __init__(self, calib, rng, sc):
        self.rng = rng
        self.bins, self.rates = calib['detect_bins'], np.asarray(calib['detect_rate'])
        self.rates = np.clip(self.rates * sc.get('detect_scale', 1.0), 0, 1)
        self.clutter_counts = np.asarray(calib['clutter_counts'])
        self.clutter_r = np.asarray(calib['clutter_ranges'])
        self.clutter_scale = sc.get('clutter_scale', 1.0)
        self.inc = 2 * math.pi / self.N
        self.ang = -math.pi + np.arange(self.N) * self.inc
        rel = np.mod(np.degrees(self.ang), 360.0)
        self.masked = (rel >= 120.0) & (rel <= 240.0)
        self.far_p = 0.6
        self.post_detect = sc.get('post_detect', {})
        self.post_range = sc.get('post_range', {})

    def scan(self, world, x, y, yaw, lidar_x=0.4):
        ox, oy = x + lidar_x * math.cos(yaw), y + lidar_x * math.sin(yaw)
        wa = yaw + self.ang
        dx, dy = np.cos(wa), np.sin(wa)
        ranges = np.full(self.N, np.inf)
        # 먼 구조물: 0.25 m 걸음으로 40 m 까지
        steps = np.arange(8.0, 40.0, 0.25)
        px = ox + dx[:, None] * steps[None, :]
        py = oy + dy[:, None] * steps[None, :]
        c = world.far_cell
        keys = np.floor(px / c).astype(np.int64) * 1_000_003 + np.floor(py / c).astype(np.int64)
        farkeys = np.array([a * 1_000_003 + b for a, b in world.far], dtype=np.int64)
        hit = np.isin(keys, farkeys)
        first = np.where(hit.any(1), hit.argmax(1), -1)
        ok = (first >= 0) & (self.rng.random(self.N) < self.far_p)
        ranges[ok] = steps[first[ok]] + self.rng.normal(0, 0.1, ok.sum())
        # 부표: 스캔마다 거리별 확률로 보임
        for i, ((px_, py_), rad) in enumerate(zip(world.posts, world.radius)):
            vx, vy = px_ - ox, py_ - oy
            d = math.hypot(vx, vy)
            rate = interp_rate(self.bins, self.rates, d) * self.post_detect.get(i, 1.0)
            if d > self.post_range.get(i, math.inf):
                rate = 0.0
            if d < 0.3 or self.rng.random() > rate:
                continue
            proj = vx * dx + vy * dy
            perp2 = d * d - proj * proj
            inside = (proj > 0) & (perp2 < rad ** 2)
            if not inside.any():
                continue
            rr = proj[inside] - np.sqrt(np.maximum(rad ** 2 - perp2[inside], 0))
            rr = rr + self.rng.normal(0, 0.03, len(rr))
            idx = np.nonzero(inside)[0]
            ranges[idx] = np.minimum(ranges[idx], rr)
        # 배 근처 잡음 히트
        k = int(self.rng.choice(self.clutter_counts) * self.clutter_scale)
        if k and len(self.clutter_r):
            front = np.nonzero(np.abs(np.arctan2(np.sin(self.ang), np.cos(self.ang))) < math.radians(110))[0]
            idx = self.rng.choice(front, size=min(k, len(front)), replace=False)
            ranges[idx] = np.minimum(ranges[idx], self.rng.choice(self.clutter_r, size=len(idx)))
        ranges[self.masked] = np.inf
        ranges[(ranges < 0.4)] = np.inf
        return ranges


class Boat:
    SPEED_V = [0.0, 0.03, 0.085, 0.225, 0.355, 0.6, 1.0]
    SPEED_U = [0.0, 0.10, 0.36, 0.71, 0.99, 1.35, 1.8]

    def __init__(self, calib, x, y, yaw, wind):
        self.x, self.y, self.yaw = x, y, yaw
        self.u, self.r = 0.0, 0.0
        self.tau_u = max(calib['tau_u'], 0.5)
        self.tau_r = max(calib['tau_r'], 0.3)
        self.g_r, self.g_ru = calib['gain_r'], calib['gain_ru']
        self.wind = np.asarray(wind, float)

    def step(self, v, w, dt, rng):
        uss = math.copysign(np.interp(abs(v), self.SPEED_V, self.SPEED_U), v) * (0.6 if v < 0 else 1.0)
        rss = (self.g_r + self.g_ru * max(self.u, 0.0)) * w
        self.u += (uss - self.u) * dt / self.tau_u
        self.r += (rss - self.r) * dt / self.tau_r + rng.normal(0, 0.01)
        self.yaw += self.r * dt
        self.x += (self.u * math.cos(self.yaw) + self.wind[0]) * dt
        self.y += (self.u * math.sin(self.yaw) + self.wind[1]) * dt


def _post_blob_sizes(grid, posts):
    """지도에서 정답 부표 0.7 m 안 덩어리들의 크기 [m] (현장 녹화와 비교용)."""
    from scipy import ndimage
    res, wdt, hgt = grid.info.resolution, grid.info.width, grid.info.height
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    lab, n = ndimage.label(np.asarray(grid.data, np.int16).reshape(hgt, wdt) >= 95,
                           structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        iy, ix = np.nonzero(lab == i)
        if len(ix) < 2:
            continue
        c = (ox + (ix.mean() + .5) * res, oy + (iy.mean() + .5) * res)
        if np.linalg.norm(posts - c, axis=1).min() < 0.7:
            out.append(res * max(ix.max() - ix.min() + 1, iy.max() - iy.min() + 1))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('calib')
    ap.add_argument('gates')
    ap.add_argument('--scenario', default='base')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--duration', type=float, default=150.0)
    ap.add_argument('--radius', type=float, default=0.25)
    ap.add_argument('--params')
    ap.add_argument('--label', default='')
    ap.add_argument('-p', dest='overrides', action='append', default=[])
    ap.add_argument('--trace')
    ap.add_argument('--verbose', action='store_true', help='게이트 노드 로그를 시뮬 시각과 함께 (stderr)')
    args = ap.parse_args(argv)
    calib = json.load(open(args.calib))
    spec = json.load(open(args.gates))
    sc = SCENARIOS[args.scenario]
    rng = np.random.default_rng(args.seed)

    clock = [0.0]
    _time.monotonic = lambda: clock[0]
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from kaboat_behaviors.gate_follower import GateFollower
    from kaboat_perception.occupancy_grid import OccupancyGridNode as Occ

    world = World(calib, spec['gates'], args.radius, spec.get('radii'))
    lidar = Lidar(calib, rng, sc)
    sx, sy, syaw = spec.get('start', (108.80, -14.45, -6.4))
    gx, gy = spec.get('goal', (131.57, -22.08))
    back = sc.get('start_back', 0.0)       # 처음 선수 방향 뒤로 [m]
    sx, sy = sx - back * math.cos(math.radians(syaw)), sy - back * math.sin(math.radians(syaw))
    yaw0 = math.radians(syaw + sc.get('start_dyaw', 0.0))
    boat = Boat(calib, sx - math.sin(yaw0) * sc.get('start_dy', 0.0),
                sy + math.cos(yaw0) * sc.get('start_dy', 0.0), yaw0, sc.get('wind', (0.0, 0.0)))

    # 한 번에 init — 이름 바꾸기는 게이트 노드에만, 게이트 파라미터는 gate_follower 키에만 들어간다
    ros_args = ['--ros-args', '-r', 'gate_behavior:__node:=gate_follower',
                '-p', 'gate.motion_enabled:=true',
                '-p', 'virtual_walls_file:=none', '-p', 'enable_tilt_gate:=false']
    if args.params:
        ros_args[1:1] = ['--params-file', args.params]
    for o in args.overrides:
        ros_args += ['-p', o]
    rclpy.init(args=ros_args)
    occ = Occ()
    gate = GateFollower()
    grids = []
    occ.pub.publish = grids.append
    cmds = []
    gate.cmd_pub.publish = cmds.append
    statuses = []
    orig_status = gate._status

    def status(v):
        if v != gate._status_text:
            statuses.append((clock[0], v))
        orig_status(v)
    gate._status = status
    if args.verbose:
        from types import SimpleNamespace

        class _Log:
            def __init__(self):
                self.last = {}

            def _p(self, lv, msg, throttle_duration_sec=None, **_):
                key = (lv, str(msg)[:30])
                if throttle_duration_sec and clock[0] - self.last.get(key, -1e9) < throttle_duration_sec:
                    return
                self.last[key] = clock[0]
                print(f'{clock[0]:6.1f} {lv} {msg}  [배 ({boat.x:.2f},{boat.y:.2f}) {math.degrees(boat.yaw):.0f}°]',
                      file=sys.stderr)
            info = lambda self, m, **k: self._p('I', m, **k)
            warn = lambda self, m, **k: self._p('W', m, **k)
            warning = warn
            error = lambda self, m, **k: self._p('E', m, **k)
            debug = lambda self, m, **k: None
        _logger = _Log()
        gate.get_logger = lambda: _logger
    else:
        gate.get_logger().set_level(40)
    occ.get_logger().set_level(40)
    gate._on_state(String(data='gate'))
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = gx, gy
    gate._on_goal(goal)

    pose_s = sc.get('pose_sigma', 0.08)
    yaw_s = math.radians(sc.get('yaw_sigma_deg', 2.0))
    dt = 0.1
    trace = []
    blob_sizes = []
    min_clear = 99.0
    stop_time = 0.0
    done_t = None
    v = w = 0.0
    for k in range(int(args.duration / dt)):
        clock[0] = k * dt
        odom = Odometry()
        odom.header.frame_id = 'odom'
        ex, ey = rng.normal(0, pose_s, 2)
        eyaw = rng.normal(0, yaw_s)
        odom.pose.pose.position.x, odom.pose.pose.position.y = boat.x + ex, boat.y + ey
        oy_ = boat.yaw + eyaw
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(oy_ / 2), math.cos(oy_ / 2)
        odom.twist.twist.linear.x = boat.u
        odom.twist.twist.angular.z = boat.r
        gate._on_odom(odom)
        occ._on_odom(odom)
        scan = LaserScan()
        scan.header.frame_id = 'onboard_laser_frame'
        scan.angle_min, scan.angle_increment = -math.pi, lidar.inc
        scan.angle_max = scan.angle_min + lidar.inc * (Lidar.N - 1)
        scan.range_min, scan.range_max = 0.4, 50.0
        scan.ranges = lidar.scan(world, boat.x, boat.y, boat.yaw).astype(np.float32).tolist()
        occ._on_scan(scan)
        occ._publish()
        gate._on_grid(grids[-1])
        if k % 5 == 0:
            blob_sizes.extend(_post_blob_sizes(grids[-1], world.posts))
        gate._on_scan(scan)
        n_before = len(cmds)
        gate._tick()
        if len(cmds) > n_before:
            v, w = cmds[-1].linear.x, cmds[-1].angular.z
        else:
            v = w = 0.0
        if gate._status_text == 'COMPLETE' and done_t is None:
            done_t = clock[0]
            break
        if abs(v) < 0.01:
            stop_time += dt
        for _ in range(2):
            boat.step(v, w, dt / 2, rng)
        d = float((np.linalg.norm(world.posts - (boat.x, boat.y), axis=1) - world.radius).min())
        min_clear = min(min_clear, d)
        if args.trace:
            trace.append((round(clock[0], 1), round(boat.x, 2), round(boat.y, 2),
                          round(math.degrees(boat.yaw), 1), round(v, 3), round(w, 3),
                          gate._status_text))
    # 통과한 게이트를 정답 게이트와 맞춤
    passed = [tuple(m) for m in gate._passed_midpoints]
    matched = []
    for m in passed:
        dd = np.linalg.norm(world.mids - m, axis=1)
        matched.append(int(dd.argmin()) + 1 if dd.min() < 1.0 else 0)
    hist = {}
    for i, (t, s) in enumerate(statuses):
        end = statuses[i + 1][0] if i + 1 < len(statuses) else clock[0]
        key = s.split('_GATE')[0] if s.startswith(('FOLLOW_GATE', 'GATE_')) else s
        key = 'FOLLOW' if s.startswith('FOLLOW_GATE') and not s.endswith('_SLOW') else key
        hist[key] = round(hist.get(key, 0.0) + end - t, 1)
    result = {
        'label': args.label, 'scenario': args.scenario, 'seed': args.seed,
        'complete': done_t is not None, 'time': round(done_t if done_t else clock[0], 1),
        'passed': len(passed), 'gates': matched,
        'in_order': matched == list(range(1, len(matched) + 1)),
        'stop_time': round(stop_time, 1), 'min_clear': round(float(min_clear), 2),
        'status_time': hist,
        'blob_p50_90_99': [round(float(v), 2) for v in np.percentile(blob_sizes, [50, 90, 99])]
        if blob_sizes else None,
    }
    print(json.dumps(result, ensure_ascii=False))
    if args.trace:
        json.dump({'trace': trace, 'statuses': statuses, 'result': result}, open(args.trace, 'w'))
    gate.destroy_node()
    occ.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
