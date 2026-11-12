#!/usr/bin/env python3
"""사진 기반 회피 경기장(photo_arena.json)에서 실제 회피 코드를 Gazebo 없이 폐루프로 돌린다.

실제 노드 클래스를 그대로 쓴다: occupancy_grid(라이다 지도 + 가상 벽) → real_obstacle_avoid_test
(test_avoid_outdoor 와 같은 config/outdoor_avoid_test.yaml + 즉시 출발). 시계는 가짜(0.1 s 걸음).

  source /opt/ros/<distro>/setup.bash && source install/setup.bash
  PYTHONPATH=src/kaboat_hardware:src/kaboat_behaviors:src/kaboat_perception:src/kaboat_mission:$PYTHONPATH \\
    python3 scripts/arena_sim/avoid_arena_sim.py --scenario field --seed 0 --plot out.png

세계 (경기장 로컬 좌표 = odom, 원점 남서 모서리, x 동·y 북):
  - 라이다(2030 빔, 선체 중심보다 선수 0.4 m, 뒤 120°~240° 가림)는 스캔 면 높이(기본 수면 위
    0.14 m — 카메라 0.775 m − 라이다→카메라 0.635 m)를 지나는 물체만 본다: 흰 부표, 모서리 큰 부표.
    붐(수면 위 0.12 m)·노란 소형 부표(0.075 m)·줄은 안 보인다 — 현장과 같은 상황.
    --booms-visible 은 카메라 층이 붐을 완벽히 잡는다고 보고 붐을 라이다에 넣는다(상한).
  - 먼 구조물(물가): 빔의 far_hit_prob 가 35 m 안팎에서 맞는다 — 맞은 빔은 지도의 빈 칸을 칠함.
  - 가상 벽: 현장처럼 경기장 둘레를 지도에 박고 짧은 두 변 가운데 3 m 만 비운다(--no-walls 로 끔).
  - 배: 현장 녹화로 잰 명령→속도 표·1차 지연·회전 이득(field_calib.json). 바람 표류 --wind.
  - 시나리오 ideal: 부표 항상 보임, 잡음 없음 / field: 현장 거리별 검출률·배 근처 잡음 히트·위치 잡음.
선체(1.1 × 0.8 m 사각형)와 물체(흰·모서리 부표, 붐, 경계 줄)가 겹치면 충돌로 센다.
출력: JSON 한 줄 (도착, 시간, 충돌, 최소 거리, 상태 시간), --plot 경로 그림, --trace 궤적 JSON.
"""

import argparse
import json
import math
import os
import sys
import time as _time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
ARENA_JSON = os.path.join(WS, 'src', 'kaboat_sim', 'config', 'photo_arena.json')
AVOID_YAML = os.path.join(WS, 'src', 'kaboat_hardware', 'config', 'outdoor_avoid_test.yaml')
CALIB_JSON = os.path.join(HERE, 'field_calib.json')
sys.path.insert(0, os.path.join(WS, 'scripts'))
import generate_photo_arena as arena_gen  # noqa: E402

LIDAR_X = 0.4                       # base_link → 라이다 (선수 +)
HULL_L, HULL_W = 1.1, 0.8           # 선체 사각형 [m] (sim 선체 1.1 m, 폭은 가정)
N_BEAMS = 2030

SCENARIOS = {
    'ideal': dict(detect=False, clutter=False, pose_sigma=0.0, yaw_sigma_deg=0.0, range_sigma=0.01),
    'field': dict(detect=True, clutter=True, pose_sigma=0.05, yaw_sigma_deg=1.5, range_sigma=0.03),
}


# ── 경기장 ─────────────────────────────────────────────────────────────

class Arena:
    """photo_arena.json → 원(부표)·선분(붐·줄) 목록. 라이다에 보이는 것과 부딪히는 것을 나눈다."""

    def __init__(self, config, lidar_height, booms_visible):
        self.config = config
        self.width, self.depth = config['size']
        white, corner, boom = config['white_buoy'], config['corner'], config['boom']
        self.white = np.array([config['buoys'][n] for n in config['buoys']], float)
        self.white_r = white['radius']
        w = self.width
        self.corners = np.array([(0, 0), (w, 0), (w, self.depth), (0, self.depth)], float)
        self.corner_r = corner['radius']
        self.booms = [(np.array(arena_gen.endpoint(config, b['start']), float),
                       np.array(arena_gen.endpoint(config, b['end']), float), b['name'])
                      for b in config['booms']]
        self.boom_half = boom['width'] / 2
        self.ropes = [(np.array(a, float), np.array(b, float), name)
                      for name, a, b in arena_gen.boundary_segments(config)]
        self.rope_r = config['boundary']['float_radius']   # 줄 위 노란 부표까지 (충돌용)
        # 라이다에 보이는 원: 스캔 면에서 자른 반지름
        dz = lidar_height - white['center_z']
        circles, radii = [], []
        if abs(dz) < self.white_r:
            r = math.sqrt(self.white_r ** 2 - dz ** 2)
            circles += list(self.white)
            radii += [r] * len(self.white)
        if lidar_height <= corner['freeboard']:
            circles += list(self.corners)
            radii += [self.corner_r] * 4
        # 붐: 스캔 면보다 높거나(아님 0.12 m) 카메라가 잡았다고 볼 때만 — 0.1 m 간격 원 사슬
        if booms_visible or boom['freeboard'] >= lidar_height:
            for a, b, _ in self.booms:
                n = max(2, int(math.ceil(np.linalg.norm(b - a) / 0.1)) + 1)
                for t in np.linspace(0, 1, n):
                    circles.append(a + t * (b - a))
                    radii.append(self.boom_half)
        self.lidar_circles = np.array(circles, float).reshape(-1, 2)
        self.lidar_radii = np.array(radii, float)

    def walls(self, opening):
        """현장 가상 벽과 같은 모양: 둘레 + 짧은 두 변(동·서) 가운데 opening m 비움."""
        from kaboat_perception.virtual_walls import arena_walls
        corners = [(0, 0), (self.width, 0), (self.width, self.depth), (0, self.depth)]
        return arena_walls('arena', corners, {1: opening, 3: opening}, 0.0)


def hull_points(x, y, yaw, n_side=6):
    """선체 사각형 둘레 점들 (충돌 검사용)."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = HULL_L / 2, HULL_W / 2
    pts = []
    for u in np.linspace(-hl, hl, n_side):
        pts += [(u, -hw), (u, hw)]
    for v in np.linspace(-hw, hw, 4):
        pts += [(-hl, v), (hl, v)]
    local = np.array(pts)
    return np.column_stack((x + local[:, 0] * c - local[:, 1] * s,
                            y + local[:, 0] * s + local[:, 1] * c))


def _seg_dist(p, a, b):
    d = b - a
    t = np.clip(((p - a) @ d) / (d @ d), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * d), axis=1)


def _inside_hull(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    rel = points - (x, y)
    u = rel[:, 0] * c + rel[:, 1] * s
    v = -rel[:, 0] * s + rel[:, 1] * c
    return (np.abs(u) <= HULL_L / 2) & (np.abs(v) <= HULL_W / 2)


def clearances(arena, x, y, yaw):
    """선체 둘레에서 물체 종류별 최소 거리 [m] (음수 = 겹침)."""
    pts = hull_points(x, y, yaw)
    out = {}
    for name, centers, r in (('white', arena.white, arena.white_r),
                             ('corner', arena.corners, arena.corner_r)):
        d = np.linalg.norm(pts[:, None, :] - centers[None, :, :], axis=2).min() - r
        if _inside_hull(centers, x, y, yaw).any():
            d = -r
        out[name] = float(d)
    for name, segs, r in (('boom', arena.booms, arena.boom_half), ('rope', arena.ropes, arena.rope_r)):
        best = math.inf
        for a, b, _ in segs:
            best = min(best, float(_seg_dist(pts, a, b).min()) - r)
            mid = np.linspace(a, b, 12)
            if _inside_hull(mid, x, y, yaw).any():
                best = min(best, -r)
        out[name] = best
    return out


# ── 라이다 ─────────────────────────────────────────────────────────────

class Lidar:
    def __init__(self, arena, calib, sc, rng):
        self.arena, self.calib, self.sc, self.rng = arena, calib, sc, rng
        self.inc = 2 * math.pi / N_BEAMS
        self.ang = -math.pi + np.arange(N_BEAMS) * self.inc
        rel = np.mod(np.degrees(self.ang), 360.0)
        self.masked = (rel >= 120.0) & (rel <= 240.0)
        counts = calib['clutter_count_prob']
        self.clutter_k = np.array([int(k) for k in counts])
        self.clutter_p = np.array(list(counts.values()), float)
        self.clutter_p /= self.clutter_p.sum()

    def detect_prob(self, d):
        bins, rates = self.calib['detect_bins'], self.calib['detect_rate']
        centers = [(a + b) / 2 for a, b in zip(bins[:-1], bins[1:])]
        return float(np.interp(d, [0.0] + centers + [40.0], [rates[0]] + list(rates) + [0.0]))

    def scan(self, x, y, yaw):
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        wa = yaw + self.ang
        dx, dy = np.cos(wa), np.sin(wa)
        ranges = np.full(N_BEAMS, np.inf)
        far = self.rng.random(N_BEAMS) < self.calib['far_hit_prob']
        ranges[far] = 35.0 + self.rng.normal(0, 1.0, far.sum())       # 물가·먼 구조물
        seen_obj = {}
        for i, ((px, py), rad) in enumerate(zip(self.arena.lidar_circles, self.arena.lidar_radii)):
            vx, vy = px - ox, py - oy
            d = math.hypot(vx, vy)
            if d < 0.3 or d > 45.0:
                continue
            if self.sc['detect']:
                key = (round(px, 1), round(py, 1)) if rad > 0.2 else None
                p = self.detect_prob(d)
                if key is not None:      # 부표는 스캔마다 통째로 보이거나 안 보임 (현장 측정 방식)
                    if key not in seen_obj:
                        seen_obj[key] = self.rng.random() < p
                    if not seen_obj[key]:
                        continue
                elif self.rng.random() > p:
                    continue
            proj = vx * dx + vy * dy
            perp2 = d * d - proj * proj
            hit = (proj > 0) & (perp2 < rad * rad)
            if not hit.any():
                continue
            rr = proj[hit] - np.sqrt(np.maximum(rad * rad - perp2[hit], 0.0))
            rr = rr + self.rng.normal(0, self.sc['range_sigma'], len(rr))
            idx = np.nonzero(hit)[0]
            ranges[idx] = np.minimum(ranges[idx], rr)
        if self.sc['clutter']:
            k = int(self.rng.choice(self.clutter_k, p=self.clutter_p))
            if k:
                front = np.nonzero(np.abs(np.arctan2(np.sin(self.ang), np.cos(self.ang)))
                                   < math.radians(110))[0]
                idx = self.rng.choice(front, size=min(k, len(front)), replace=False)
                ranges[idx] = np.minimum(ranges[idx],
                                         self.rng.choice(self.calib['clutter_ranges'], size=len(idx)))
        ranges[self.masked] = np.inf
        ranges[ranges < 0.4] = np.inf
        return ranges


# ── 배 ────────────────────────────────────────────────────────────────

class Boat:
    """현장 녹화로 잰 1차 응답 — 명령(비율) → 정상 속도 표, 회전 = (g + g_u·u)·w."""

    def __init__(self, calib, x, y, yaw, wind):
        self.calib = calib
        self.x, self.y, self.yaw = x, y, yaw
        self.u = self.r = 0.0
        self.wind = np.asarray(wind, float)

    def step(self, v, w, dt, rng):
        c = self.calib
        uss = math.copysign(np.interp(abs(v), c['speed_cmd'], c['speed_mps']), v) * (0.6 if v < 0 else 1.0)
        rss = (c['gain_r'] + c['gain_ru'] * max(self.u, 0.0)) * w
        self.u += (uss - self.u) * dt / max(c['tau_u'], 0.5)
        self.r += (rss - self.r) * dt / max(c['tau_r'], 0.3) + rng.normal(0, 0.005)
        self.yaw += self.r * dt
        self.x += (self.u * math.cos(self.yaw) + self.wind[0]) * dt
        self.y += (self.u * math.sin(self.yaw) + self.wind[1]) * dt


# ── 실행 ───────────────────────────────────────────────────────────────

def run(args):
    config = json.load(open(args.arena))
    calib = json.load(open(CALIB_JSON))
    sc = SCENARIOS[args.scenario]
    rng = np.random.default_rng(args.seed)
    arena = Arena(config, args.lidar_height, args.booms_visible)
    lidar = Lidar(arena, calib, sc, rng)
    w, h = arena.width, arena.depth
    sx, sy, syaw = args.start if args.start else (-3.0, h / 2, 0.0)
    gx, gy = args.goal if args.goal else (w + 4.0, h / 2)
    boat = Boat(calib, sx, sy, math.radians(syaw), args.wind)

    clock = [0.0]
    _time.monotonic = lambda: clock[0]
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from kaboat_hardware.real_obstacle_avoid_test import RealObstacleAvoidTest
    from kaboat_perception.occupancy_grid import OccupancyGridNode

    ros_args = ['--ros-args', '--params-file', AVOID_YAML,
                '-p', 'drive_enabled:=true', '-p', 'wait_for_start:=false',
                '-p', f'fixed_start_xy:=[{sx:.3f},{sy:.3f}]',
                '-p', f'fixed_goal_xy:=[{gx:.3f},{gy:.3f}]',
                '-p', 'virtual_walls_file:=none', '-p', 'enable_tilt_gate:=false',
                '-p', 'size:=15.0', '-p', 'publish_rate:=10.0']
    for o in args.overrides:
        ros_args += ['-p', o]
    rclpy.init(args=ros_args)
    occ = OccupancyGridNode()
    avoid = RealObstacleAvoidTest()
    for node in (occ, avoid):
        node.get_logger().set_level(40)
    if not args.no_walls:
        occ.walls_xy = arena.walls(args.wall_opening)
    grids, cmds, statuses = [], [], []
    occ.pub.publish = grids.append
    avoid.cmd_pub.publish = cmds.append
    original_status = avoid._publish_status

    def status(state, detail):
        if not statuses or statuses[-1][1] != state:
            statuses.append((round(clock[0], 1), state, detail))
        original_status(state, detail)
    avoid._publish_status = status

    dt = 0.1
    t0 = 1000.0                       # odom 스탬프 = 시뮬 시각 + t0 (0 이면 노드가 벽시계로 대체)
    trace, contacts = [], {}
    min_clear = {k: math.inf for k in ('white', 'corner', 'boom', 'rope')}
    finished_t = None
    v = wcmd = 0.0
    for k in range(int(args.duration / dt)):
        clock[0] = k * dt
        odom = Odometry()
        odom.header.frame_id = 'odom'
        stamp = t0 + clock[0]
        odom.header.stamp.sec, odom.header.stamp.nanosec = int(stamp), int((stamp % 1) * 1e9)
        ex, ey = rng.normal(0, sc['pose_sigma'], 2) if sc['pose_sigma'] else (0.0, 0.0)
        eyaw = rng.normal(0, math.radians(sc['yaw_sigma_deg'])) if sc['yaw_sigma_deg'] else 0.0
        odom.pose.pose.position.x, odom.pose.pose.position.y = boat.x + ex, boat.y + ey
        oyaw = boat.yaw + eyaw
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(oyaw / 2), math.cos(oyaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = boat.u, boat.r
        occ._on_odom(odom)
        avoid._on_odom(odom)
        scan = LaserScan()
        scan.header.frame_id = 'onboard_laser_frame'
        scan.header.stamp = odom.header.stamp
        scan.angle_min, scan.angle_increment = -math.pi, lidar.inc
        scan.angle_max = scan.angle_min + lidar.inc * (N_BEAMS - 1)
        scan.range_min, scan.range_max = 0.4, 50.0
        scan.ranges = lidar.scan(boat.x, boat.y, boat.yaw).astype(np.float32).tolist()
        occ._on_scan(scan)
        occ._publish()
        avoid._on_grid(grids[-1])
        avoid._on_scan(scan)
        n_before = len(cmds)
        avoid._tick()
        if len(cmds) > n_before:
            v, wcmd = cmds[-1].linear.x, cmds[-1].angular.z
        if statuses and statuses[-1][1] == 'FINISHED':
            finished_t = clock[0]
            break
        for _ in range(2):
            boat.step(v, wcmd, dt / 2, rng)
        cl = clearances(arena, boat.x, boat.y, boat.yaw)
        for key, d in cl.items():
            min_clear[key] = min(min_clear[key], d)
            if d < 0:
                contacts[key] = contacts.get(key, 0) + 1
        trace.append((round(clock[0], 1), round(boat.x, 3), round(boat.y, 3),
                      round(math.degrees(boat.yaw), 1), round(v, 3), round(wcmd, 3),
                      statuses[-1][1] if statuses else ''))
    hist = {}
    for i, (t, state, _) in enumerate(statuses):
        end = statuses[i + 1][0] if i + 1 < len(statuses) else clock[0]
        hist[state] = round(hist.get(state, 0.0) + end - t, 1)
    path = np.array([(p[1], p[2]) for p in trace]) if trace else np.zeros((0, 2))
    result = {
        'scenario': args.scenario, 'seed': args.seed, 'booms_visible': args.booms_visible,
        'walls': not args.no_walls, 'wind': list(args.wind),
        'finished': finished_t is not None, 'time': round(finished_t or clock[0], 1),
        'final_xy': [round(boat.x, 2), round(boat.y, 2)],
        'path_m': round(float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()), 1) if len(path) > 1 else 0.0,
        'contact_steps': contacts,
        'min_clear': {k: round(v, 2) for k, v in min_clear.items()},
        'status_time': hist,
        'escape_count': sum(1 for _, s, _ in statuses if s.startswith('ESCAPE')),
        'last_status': statuses[-1][1:] if statuses else None,
    }
    if args.trace:
        json.dump({'result': result, 'trace': trace, 'statuses': statuses}, open(args.trace, 'w'),
                  ensure_ascii=False)
    if args.plot:
        plot(arena, path, trace, result, args, (sx, sy), (gx, gy))
    avoid.destroy_node()
    occ.destroy_node()
    rclpy.shutdown()
    return result


def plot(arena, path, trace, result, args, start, goal):
    import cv2
    scale, pad = 30, 5.0
    x0, y0 = -pad, -2.5
    W, H = arena.width + 2 * pad, arena.depth + 5.0
    img = np.full((int(H * scale), int(W * scale), 3), 60, np.uint8)

    def px(x, y):
        return int((x - x0) * scale), int((H - (y - y0)) * scale)
    if not args.no_walls:
        for xy in arena.walls(args.wall_opening).values():
            cv2.polylines(img, [np.array([px(*p) for p in xy], np.int32)], False, (140, 70, 140), 2)
    for a, b, _ in arena.ropes:
        cv2.line(img, px(*a), px(*b), (40, 190, 230), 1)
    for a, b, _ in arena.booms:
        cv2.line(img, px(*a), px(*b), (30, 40, 230), max(2, int(arena.boom_half * 2 * scale)))
    for c in arena.white:
        cv2.circle(img, px(*c), int(arena.white_r * scale), (235, 235, 235), -1)
    for c in arena.corners:
        cv2.circle(img, px(*c), int(arena.corner_r * scale), (20, 110, 255), -1)
    for (t, x, y, yaw, v, w, state), nxt in zip(trace[:-1], trace[1:]):   # 초록 = FOLLOW, 주황 = ESCAPE
        col = (0, 140, 255) if state.startswith('ESCAPE') else (90, 220, 90)
        cv2.line(img, px(x, y), px(nxt[1], nxt[2]), col, 2)
    for t, x, y, yaw, *_ in trace:
        cl = clearances(arena, x, y, math.radians(yaw))
        if min(cl.values()) < 0:
            cv2.drawMarker(img, px(x, y), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 10, 2)
    if trace:
        t, x, y, yaw = trace[-1][:4]
        hp = hull_points(x, y, math.radians(yaw))
        cv2.polylines(img, [cv2.convexHull(np.array([px(*p) for p in hp], np.int32))], True, (255, 255, 0), 2)
    cv2.circle(img, px(*start), 5, (255, 255, 0), -1)
    cv2.drawMarker(img, px(*goal), (255, 255, 0), cv2.MARKER_STAR, 16, 2)
    label = (f"{result['scenario']} seed {result['seed']}{' booms_visible' if result['booms_visible'] else ''}"
             f" | {'FINISHED' if result['finished'] else 'NOT finished'} {result['time']} s"
             f" | contacts {result['contact_steps'] or 0}")
    cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1)
    cv2.imwrite(args.plot, img)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', choices=sorted(SCENARIOS), default='field')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--duration', type=float, default=240.0)
    ap.add_argument('--arena', default=ARENA_JSON)
    ap.add_argument('--lidar-height', type=float, default=0.14)
    ap.add_argument('--booms-visible', action='store_true', help='카메라 층이 붐을 완벽히 잡는다고 봄')
    ap.add_argument('--no-walls', action='store_true', help='가상 벽 끔')
    ap.add_argument('--wall-opening', type=float, default=3.0)
    ap.add_argument('--wind', type=float, nargs=2, default=(0.0, 0.0), metavar=('VX', 'VY'))
    ap.add_argument('--start', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--goal', type=float, nargs=2, metavar=('X', 'Y'))
    ap.add_argument('-p', dest='overrides', action='append', default=[], help='노드 파라미터 name:=value')
    ap.add_argument('--trace')
    ap.add_argument('--plot')
    args = ap.parse_args(argv)
    print(json.dumps(run(args), ensure_ascii=False))


if __name__ == '__main__':
    main()
