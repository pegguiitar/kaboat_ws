#!/usr/bin/env python3
"""KABOAT 종합임무 경기장 end-to-end 시뮬 노드 (센서·모터 자리만 대신한다).

실제 스택(autonomy.launch.py, occupancy_grid, thruster_driver dummy)은 그대로 띄우고 이 노드가
  /cmd_vel → (실제 ThrusterMixer: 슬루·불감대·워치독) → 좌우 PWM → Nomoto 배 모델 → /odom·TF
  경기장(arena.json) → /scan, /scan/level (가상 라이다), /camera/color/image_raw (부표 색), /imu/data, /gps/fix
  도킹 표지 → /dock/detection_status (+ /dock/markers/<도형>/ray) — 노트북에 YOLO(ultralytics)가 없어 검출기 출력만 흉내
를 실제 시각(wall clock)으로 낸다. 정답 궤적·접촉·상태는 trace.jsonl / events.jsonl 에 남긴다.

배 모델 (bag 맞춤, cache/nomoto.json):
  선회 Nomoto 1차  T·ṙ + r = K(u)·sgn(δ)|δ|^q,  K(u) = K0 + K1·|u|,  δ = (우 − 좌)/2
  전진 1차        τ·u̇ + u = U(f̄),  U = a·|f̄|^pw (후진 × krev),  τ = tau_u(가속)/tau_dn(감속),  f̄ = (좌 + 우)/2
  좌·우 = (|PWM − 1500| − 25)/375 부호 포함, 입력 지연 delay. 물살 = 일정 벡터 + 돌풍(OU), 바람 요 외란(OU).
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from kaboat_hardware.thruster_backend import ThrusterConfig, ThrusterMixer
from kaboat_perception.buoy_localize import BuoyGeometry, level_to_pixel

HULL_L, HULL_W = 1.1, 0.7
LIDAR_X = 0.4
N_BEAMS = 2030
K_CAM = (381.67, 381.25, 316.82, 245.23)
IMG_W, IMG_H = 640, 480
# green = HSV (73, 160, 150): 실제 초록 부표 H 중앙 73 — 예전 (60,150,60) 은 H 60 이라 2026-11-14 색 튜닝(초록 H ≥ 64)에서 unknown
BGR = {'red': (30, 30, 200), 'green': (97, 150, 56), 'black': (35, 35, 35), 'orange': (0, 140, 255),
       'white': (235, 235, 235), 'yellow': (0, 220, 230)}
SHAPES = ('triangle', 'circle', 'square')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class World:
    def __init__(self, arena, sc):
        self.arena = arena
        drop = set(sc.get('remove', []))
        rm_prefix = tuple(sc.get('remove_prefix', []))
        circles = [c for c in arena['circles'] if c['name'] not in drop and not (rm_prefix and c['name'].startswith(rm_prefix))]
        for extra in sc.get('add_circles', []):
            circles.append(extra)
        segs = [s for s in arena['segments'] if s['name'] not in drop and not (rm_prefix and s['name'].startswith(rm_prefix))]
        polys = [p for p in arena['polys'] if p['name'] not in drop]
        shift = sc.get('shift', {})          # 이름 접두어 → (dx, dy): 물체가 밀려 있는 경우
        def sh(name):
            for k, v in shift.items():
                if name.startswith(k):
                    return v
            return (0.0, 0.0)
        self.c_name = [c['name'] for c in circles]
        self.c_xy = np.array([[c['x'] + sh(c['name'])[0], c['y'] + sh(c['name'])[1]] for c in circles], float)
        self.c_r = np.array([c['r'] for c in circles], float)
        self.c_cls = [sc.get('bar_end_cls', c['cls']) if False else c['cls'] for c in circles]
        self.c_color = [c['color'] for c in circles]
        # 선분(두께 있는 바) → 사각형 4변, 다각형 → 변
        edges, e_obj, self.o_name, self.o_cls, self.o_center = [], [], [], [], []
        for s in segs:
            d = sh(s['name'])
            p0 = np.array(s['p0'], float) + d
            p1 = np.array(s['p1'], float) + d
            u = (p1 - p0) / np.linalg.norm(p1 - p0)
            nrm = np.array([-u[1], u[0]]) * s['hw']
            pts = [p0 + nrm, p1 + nrm, p1 - nrm, p0 - nrm]
            self._add_poly(pts, s['name'], sc.get('bar_cls', s['cls']), edges, e_obj)
        for p in polys:
            d = sh(p['name'])
            self._add_poly([np.array(q, float) + d for q in p['pts']], p['name'], p['cls'], edges, e_obj)
        self.e = np.array(edges, float).reshape(-1, 4)
        self.e_obj = np.array(e_obj, int)
        self.o_center = np.array(self.o_center, float).reshape(-1, 2)
        # 접촉 판정용 변 위 점 (0.1 m 간격)
        pts, pobj = [], []
        for (x0, y0, x1, y1), o in zip(self.e, self.e_obj):
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.1) + 1)
            for k in np.linspace(0, 1, n):
                pts.append((x0 + k * (x1 - x0), y0 + k * (y1 - y0)))
                pobj.append(o)
        self.ep = np.array(pts, float).reshape(-1, 2)
        self.ep_obj = np.array(pobj, int)
        det = arena['detect']
        self.bins = np.array(det['bins'])
        scale = sc.get('detect_scale', {})
        self.seen = {k: np.clip(np.array(v['seen']) * scale.get(k, 1.0), 0, 1) for k, v in det.items() if isinstance(v, dict)}
        self.beam = {k: v['beam'] for k, v in det.items() if isinstance(v, dict)}
        # 도킹 표지
        dshift = np.array(sh('dock'))
        self.markers = {k: np.array(v) + dshift for k, v in arena['dock']['markers'].items()}
        self.dock_n = np.array(arena['dock']['n'])

    def _add_poly(self, pts, name, cls, edges, e_obj):
        oid = len(self.o_name)
        self.o_name.append(name)
        self.o_cls.append(cls)
        self.o_center.append(np.mean(pts, axis=0))
        for a, b in zip(pts, pts[1:] + pts[:1]):
            edges.append((a[0], a[1], b[0], b[1]))
            e_obj.append(oid)

    def p_seen(self, cls, dist):
        tab = self.seen[cls]
        i = np.clip(np.searchsorted(self.bins, dist) - 1, 0, len(tab) - 1)
        return np.where(dist > self.bins[-1], 0.0, tab[i])


class Lidar:
    def __init__(self, world, rng, sc):
        self.w, self.rng = world, rng
        self.inc = 2 * math.pi / N_BEAMS
        self.ang = -math.pi + np.arange(N_BEAMS) * self.inc
        self.front = np.nonzero(np.abs(np.degrees(self.ang)) <= 118.0)[0]
        self.clutter_p = sc.get('clutter_p', 0.26)     # 스캔에 배 근처 잡음 히트가 생길 확률 (2026-11-12 실측 0.26)
        self.c_cls = np.array(world.c_cls)
        self.o_cls = np.array(world.o_cls)

    def scan(self, x, y, yaw):
        w, rng = self.w, self.rng
        ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        a = yaw + self.ang[self.front]
        dx, dy = np.cos(a), np.sin(a)
        best = np.full(len(a), np.inf)
        # 원
        v = w.c_xy - (ox, oy)
        dist = np.hypot(v[:, 0], v[:, 1])
        near = dist - w.c_r < 22.0
        ps = np.zeros(len(dist))
        for cls in set(w.c_cls):
            m = (self.c_cls == cls) & near
            ps[m] = w.p_seen(cls, np.maximum(dist[m] - w.c_r[m], 0.0))
        vis = np.nonzero(near & (rng.random(len(dist)) < ps))[0]
        if len(vis):
            vv = v[vis]
            proj = dx[:, None] * vv[None, :, 0] + dy[:, None] * vv[None, :, 1]
            perp2 = (dist[vis] ** 2)[None, :] - proj ** 2
            r2 = (w.c_r[vis] ** 2)[None, :]
            inside = (proj > 0) & (perp2 < r2)
            t = np.where(inside, proj - np.sqrt(np.maximum(r2 - perp2, 0.0)), np.inf)
            keep = np.array([w.beam[w.c_cls[i]] for i in vis])[None, :]
            t = np.where(rng.random(t.shape) < keep, t, np.inf)
            best = np.minimum(best, t.min(axis=1))
        # 변 (바·도킹장)
        if len(w.e):
            od = np.hypot(w.o_center[:, 0] - ox, w.o_center[:, 1] - oy)
            pso = np.zeros(len(od))
            for cls in set(w.o_cls):
                m = self.o_cls == cls
                pso[m] = w.p_seen(cls, np.maximum(od[m] - 2.0, 0.0))
            ovis = rng.random(len(od)) < pso
            ev = np.nonzero(ovis[w.e_obj])[0]
            if len(ev):
                e = w.e[ev]
                ex, ey = e[:, 2] - e[:, 0], e[:, 3] - e[:, 1]
                wx, wy = e[:, 0] - ox, e[:, 1] - oy
                den = dx[:, None] * ey[None, :] - dy[:, None] * ex[None, :]
                with np.errstate(divide='ignore', invalid='ignore'):
                    t = (wx * ey - wy * ex)[None, :] / den
                    s = (wx[None, :] * dy[:, None] - wy[None, :] * dx[:, None]) / den
                ok = (t > 0.05) & (s >= 0) & (s <= 1) & (np.abs(den) > 1e-9)
                t = np.where(ok, t, np.inf)
                keep = np.array([w.beam[w.o_cls[o]] for o in w.e_obj[ev]])[None, :]
                # 물체 뒤쪽 변은 앞쪽 변에 가려진다 — 빔 탈락은 물체 단위 최소값에 적용
                tmin = np.full((len(a), len(w.o_name)), np.inf)
                np.minimum.at(tmin, (slice(None), w.e_obj[ev]), t)
                ko = np.array([w.beam[c] for c in w.o_cls])[None, :]
                tmin = np.where(rng.random(tmin.shape) < ko, tmin, np.inf)
                best = np.minimum(best, tmin.min(axis=1))
        best = best + rng.normal(0, 0.02, len(best))
        ranges = np.full(N_BEAMS, np.inf, np.float32)
        ranges[self.front] = best
        # 배 근처 잡음 (물보라·수면)
        if rng.random() < self.clutter_p:
            k = int(rng.integers(1, 4))
            idx = rng.choice(self.front, k, replace=False)
            ranges[idx] = np.minimum(ranges[idx], rng.uniform(0.45, 2.9, k))
        ranges[ranges < 0.4] = np.inf
        ranges[ranges > 50.0] = np.inf
        return ranges


class Boat:
    def __init__(self, p, x, y, yaw, sc, rng):
        self.p, self.rng = p, rng
        self.x, self.y, self.yaw = x, y, yaw
        self.u = self.r = 0.0
        self.cur = np.array(sc.get('current', [0.0, 0.0]), float)     # 일정 물살 [m/s] (동, 북)
        self.gust_sigma = float(sc.get('gust_sigma', 0.04))
        self.gust = np.zeros(2)
        self.yaw_dist_sigma = float(sc.get('yaw_dist_sigma', 0.02))  # 바람 요 외란 [rad/s]
        self.yaw_dist = 0.0
        self.speed_scale = float(sc.get('speed_scale', 1.0))
        self.buf = collections.deque()      # (시각, fbar, delta) — 입력 지연
        self.f = (0.0, 0.0)

    @staticmethod
    def ratio(pwm):
        d = pwm - 1500.0
        return math.copysign(max(abs(d) - 25.0, 0.0) / 375.0, d)

    def step(self, left_pwm, right_pwm, dt, now):
        p = self.p
        fl, fr = self.ratio(left_pwm), self.ratio(right_pwm)
        self.buf.append((now, (fl + fr) / 2, (fr - fl) / 2))
        while len(self.buf) > 1 and self.buf[1][0] <= now - p['delay']:
            self.buf.popleft()
        _, fb, de = self.buf[0]
        self.f = (fb, de)
        uss = p['a'] * abs(fb) ** p['pw'] * (1.0 if fb >= 0 else -p['krev']) * self.speed_scale
        tau = p['tau_u'] if abs(uss) >= abs(self.u) else p['tau_dn']
        self.u += dt * (uss - self.u) / tau
        k = max(p['K0'] + p['K1'] * abs(self.u), 0.3)
        rss = k * math.copysign(abs(de) ** p['q'], de)
        # OU 외란
        th = dt / 8.0
        self.gust += -th * self.gust + self.gust_sigma * math.sqrt(2 * th) * self.rng.normal(0, 1, 2)
        self.yaw_dist += -dt / 3.0 * self.yaw_dist + self.yaw_dist_sigma * math.sqrt(2 * dt / 3.0) * self.rng.normal()
        self.r += dt * (rss + self.yaw_dist - self.r) / p['T']
        self.yaw = wrap(self.yaw + self.r * dt)
        vx = self.u * math.cos(self.yaw) + self.cur[0] + self.gust[0]
        vy = self.u * math.sin(self.yaw) + self.cur[1] + self.gust[1]
        self.x += vx * dt
        self.y += vy * dt
        return vx, vy


class SimNode(Node):
    def __init__(self, args):
        super().__init__('arena_sim')
        self.out = args.out
        os.makedirs(self.out, exist_ok=True)
        arena = json.load(open(args.arena))
        sc = json.load(open(args.scenario)) if args.scenario else {}
        self.sc = sc
        self.rng = np.random.default_rng(args.seed)
        self.world = World(arena, sc)
        self.lidar = Lidar(self.world, self.rng, sc)
        nom = json.load(open(args.nomoto))
        sx, sy, syaw = sc.get('start', arena['start'])
        self.boat = Boat(nom, sx, sy, math.radians(syaw), sc, self.rng)
        self.origin = (arena['origin']['lat'], arena['origin']['lon'])
        self.max_time = float(sc.get('max_time', args.max_time))
        self.pose_sigma = float(sc.get('pose_sigma', 0.02))
        self.yaw_sigma = math.radians(float(sc.get('yaw_sigma_deg', 0.3)))
        self.marker_hidden = set(sc.get('marker_hidden', []))
        self.marker_p_scale = float(sc.get('marker_p_scale', 1.0))
        self.cmd_drop = sc.get('cmd_drop')          # [t0, t1] run 시각 동안 /cmd_vel 무시 (모터 고장 흉내)
        # 모터: 실제 믹서 + 드라이버와 같은 워치독
        self.mixer = ThrusterMixer(ThrusterConfig())
        self.cmd = Twist()
        self.cmd_at = 0.0
        self.pwm = (1500, 1500)

        self.t0 = time.monotonic()
        self.last_step = self.t0
        self.run_t0 = None                 # 처음 모터가 돈 시각 (경기 시계)
        self.state = ''
        self.done_at = None
        self.detector_on = False
        self.pose_hist = collections.deque(maxlen=200)
        self.contact = {}                  # 물체 이름 → 시작 시각
        self.min_clear = {}
        self.geom = BuoyGeometry()
        self.trace = open(os.path.join(self.out, 'trace.jsonl'), 'w')
        self.events = open(os.path.join(self.out, 'events.jsonl'), 'w')
        self._last_status = {}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, '/mission/state', self._on_state, latched)
        self.create_subscription(Bool, '/detector/enable', self._on_det_enable, latched)
        for topic in ('/mission/complete', '/mission/give_up', '/gate/status', '/station/status', '/search/status',
                      '/teleop/status'):
            self.create_subscription(String, topic, lambda m, t=topic: self._on_status(t, m.data), 20)
        self.create_subscription(String, '/dock/status', self._on_dock_status, 20)
        self.create_subscription(Log, '/rosout', self._on_rosout, 200)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.level_pub = self.create_publisher(LaserScan, '/scan/level', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
        self.img_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.det_pub = self.create_publisher(String, '/dock/detection_status', 10)
        ray_qos = QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.ray_pubs = {c: self.create_publisher(PointStamped, f'/dock/markers/{c}/ray', ray_qos) for c in SHAPES}
        self._static()

        self.create_timer(0.02, self._physics)
        self.create_timer(0.1, self._scan)
        self.create_timer(0.1, self._image)
        self.create_timer(0.145, self._dock_detect)
        self.create_timer(0.5, self._gps)
        self.create_timer(0.2, self._log)
        self.k = 0
        self._event('start', pose=[sx, sy, syaw], scenario=sc, seed=args.seed)
        self.get_logger().info(f'arena_sim: 시작 ({sx:.2f}, {sy:.2f}, {syaw:.1f}°), 물체 원 {len(self.world.c_r)}·변 {len(self.world.e)}')

    # ── 기록 ─────────────────────────────────────────────
    def now(self):
        return time.monotonic() - self.t0

    def run_time(self):
        return None if self.run_t0 is None else time.monotonic() - self.run_t0

    def _event(self, kind, **kw):
        rt = self.run_time()
        self.events.write(json.dumps({'t': round(self.now(), 2), 'rt': None if rt is None else round(rt, 2),
                                      'kind': kind, **kw}, ensure_ascii=False) + '\n')
        self.events.flush()

    def _on_state(self, m):
        if m.data != self.state:
            self._event('state', value=m.data, pose=[round(self.boat.x, 2), round(self.boat.y, 2), round(math.degrees(self.boat.yaw), 1)])
        self.state = m.data
        if m.data == 'done' and self.done_at is None:
            self.done_at = time.monotonic()

    def _on_det_enable(self, m):
        self.detector_on = bool(m.data)

    def _on_status(self, topic, text):
        if topic == '/teleop/status':
            if text != self._last_status.get(topic):
                self._last_status[topic] = text
                self._event('motor_mode', value=text)
            return
        key = text[:60]
        if self._last_status.get(topic) != key:
            self._last_status[topic] = key
            self._event('status', topic=topic, value=text[:300])

    def _on_dock_status(self, m):
        try:
            j = json.loads(m.data)
        except ValueError:
            return
        key = (j.get('phase'), (j.get('detail') or '')[:28])
        if self._last_status.get('dock') != key:
            self._last_status['dock'] = key
            self._event('dock', phase=j.get('phase'), detail=(j.get('detail') or '')[:200])

    def _on_rosout(self, m):
        if m.name in ('arena_sim',):
            return
        if m.level >= 30 or m.name in ('mission_manager', 'cmd_mux', 'mission_approach'):
            self._event('log', node=m.name, level=int(m.level), msg=m.msg[:400])

    # ── 모터·물리 ────────────────────────────────────────
    def _on_cmd(self, m):
        self.cmd = m
        self.cmd_at = time.monotonic()

    def _physics(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now
        rt = self.run_time()
        dropped = self.cmd_drop is not None and rt is not None and self.cmd_drop[0] <= rt <= self.cmd_drop[1]
        if now - self.cmd_at <= 0.3 and not dropped:
            l, r, _, _ = self.mixer.mix(self.cmd.linear.x, self.cmd.angular.z, now)
        else:
            self.mixer.reset()
            l, r, _, _ = self.mixer.mix(0.0, 0.0, now)
        self.pwm = (l, r)
        if self.run_t0 is None and (l != 1500 or r != 1500):
            self.run_t0 = now
            self._event('run_start')
        b = self.boat
        px, py = b.x, b.y
        vx, vy = b.step(l, r, dt, now)
        blocked = self._contacts()
        if blocked:                       # 도킹장·바는 뚫고 지나가지 못한다
            b.x, b.y = px, py
            b.u *= 0.5
        t = self.get_clock().now()
        self.pose_hist.append((now, b.x, b.y, b.yaw))
        self.k += 1
        if self.k % 2 == 0:               # 25 Hz
            ex, ey = self.rng.normal(0, self.pose_sigma, 2)
            yaw = b.yaw + self.rng.normal(0, self.yaw_sigma)
            od = Odometry()
            od.header.stamp = t.to_msg()
            od.header.frame_id, od.child_frame_id = 'odom', 'base_link'
            od.pose.pose.position.x, od.pose.pose.position.y = b.x + ex, b.y + ey
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            # 선체 속도 (대지 속도를 선체 축으로)
            od.twist.twist.linear.x = vx * math.cos(b.yaw) + vy * math.sin(b.yaw)
            od.twist.twist.linear.y = -vx * math.sin(b.yaw) + vy * math.cos(b.yaw)
            od.twist.twist.angular.z = b.r + self.rng.normal(0, 0.005)
            self.odom_pub.publish(od)
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = od.pose.pose.position.x, od.pose.pose.position.y
            tf.transform.rotation = od.pose.pose.orientation
            self.tf.sendTransform(tf)
        imu = Imu()
        imu.header.stamp = t.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation.z, imu.orientation.w = math.sin(b.yaw / 2), math.cos(b.yaw / 2)
        imu.angular_velocity.z = b.r
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)
        # 끝
        if (self.done_at is not None and now - self.done_at > 5.0) or \
                (rt is not None and rt > self.max_time) or (rt is None and self.now() > 120.0):
            self._event('end', reason='done' if self.done_at else 'max_time', run_time=rt)
            self.trace.close()
            self.events.close()
            raise SystemExit(0)

    def _contacts(self):
        """선체 사각형과 물체 사이 여유. 겹치면 접촉 기록. 도킹장·바와 겹치면 True (막힘)."""
        b, w = self.boat, self.world
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        def clearance(pts, rad):
            dx, dy = pts[:, 0] - b.x, pts[:, 1] - b.y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            qx, qy = np.abs(lx) - HULL_L / 2, np.abs(ly) - HULL_W / 2
            outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
            inside = np.minimum(np.maximum(qx, qy), 0)
            return outside + inside - rad
        near = np.nonzero(np.hypot(w.c_xy[:, 0] - b.x, w.c_xy[:, 1] - b.y) < 3.0)[0]
        touching, blocked = set(), False
        if len(near):
            cl = clearance(w.c_xy[near], w.c_r[near])
            for i, d in zip(near, cl):
                cls = w.c_cls[i] if w.c_cls[i] != 'buoy' else 'buoy'
                key = w.c_name[i].rstrip('0123456789ab') if w.c_cls[i] == 'bound' else w.c_name[i]
                self.min_clear[key] = min(self.min_clear.get(key, 99.0), float(d))
                if d <= 0:
                    touching.add(key)
        if len(w.ep):
            nearp = np.nonzero(np.hypot(w.ep[:, 0] - b.x, w.ep[:, 1] - b.y) < 2.0)[0]
            if len(nearp):
                cl = clearance(w.ep[nearp], 0.0)
                for o in set(w.ep_obj[nearp].tolist()):
                    d = float(cl[w.ep_obj[nearp] == o].min())
                    name = w.o_name[o]
                    self.min_clear[name] = min(self.min_clear.get(name, 99.0), d)
                    if d <= 0:
                        touching.add(name)
                        blocked = True
        now = self.now()
        for name in touching - set(self.contact):
            self.contact[name] = now
            self._event('contact_start', obj=name, pose=[round(b.x, 2), round(b.y, 2), round(math.degrees(b.yaw), 1)], state=self.state)
        for name in set(self.contact) - touching:
            self._event('contact_end', obj=name, dur=round(now - self.contact.pop(name), 2))
        return blocked

    # ── 센서 ─────────────────────────────────────────────
    def _static(self):
        tfs = []
        for parent, child, xyz, q in (
                ('base_link', 'onboard_laser_frame', (LIDAR_X, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                ('camera_link', 'camera_color_optical_frame', (0.0, 0.0, 0.0), (-0.5, 0.5, -0.5, 0.5)),
                ('base_link', 'imu_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            tfs.append(t)
        self.static_tf.sendTransform(tfs)

    def _scan(self):
        b = self.boat
        stamp = self.get_clock().now().to_msg()
        ranges = self.lidar.scan(b.x, b.y, b.yaw)
        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = 'onboard_laser_frame'
        m.angle_min, m.angle_increment = -math.pi, self.lidar.inc
        m.angle_max = m.angle_min + self.lidar.inc * (N_BEAMS - 1)
        m.range_min, m.range_max = 0.4, 50.0
        m.scan_time = 0.1
        m.ranges = ranges.tolist()
        self.scan_pub.publish(m)
        if self.rng.random() < 0.97:       # /scan/level 은 배가 수평일 때만 (bag: 95~98 %)
            self.level_pub.publish(m)

    def _image(self):
        b, w = self.boat, self.world
        img = np.empty((IMG_H, IMG_W, 3), np.uint8)
        img[:245] = (205, 200, 195)
        img[245:] = (120, 110, 92)
        c, s = math.cos(b.yaw), math.sin(b.yaw)
        dx, dy = w.c_xy[:, 0] - b.x, w.c_xy[:, 1] - b.y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        cam = self.geom.cam_offset_x
        order = np.argsort(-(lx - cam))
        for i in order:
            fwd = lx[i] - cam
            if fwd < 0.5 or fwd > 30.0 or w.c_cls[i] == 'bound' or abs(ly[i]) > fwd:
                continue
            r = w.c_r[i]
            h = 0.55 if r < 0.4 else 0.75
            pts = np.array([[lx[i], ly[i] - r, 0.0], [lx[i], ly[i] + r, 0.0], [lx[i], ly[i] - r, h], [lx[i], ly[i] + r, h]])
            u, v, d = level_to_pixel(pts, K_CAM, self.geom)
            if np.any(d <= 0):
                continue
            u0, u1, v0, v1 = int(np.floor(u.min())), int(np.ceil(u.max())), int(np.floor(v.min())), int(np.ceil(v.max()))
            if u1 < 0 or u0 >= IMG_W or v1 < 0 or v0 >= IMG_H:
                continue
            cv2.rectangle(img, (u0, v0), (u1, v1), BGR[w.c_color[i]], -1)
        stamp = self.get_clock().now().to_msg()
        m = Image()
        m.header.stamp = stamp
        m.header.frame_id = 'camera_color_optical_frame'
        m.height, m.width, m.encoding, m.step = IMG_H, IMG_W, 'bgr8', IMG_W * 3
        m.data = img.tobytes()
        self.img_pub.publish(m)
        ci = CameraInfo()
        ci.header = m.header
        ci.width, ci.height = IMG_W, IMG_H
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0] * 5
        ci.k = [K_CAM[0], 0.0, K_CAM[2], 0.0, K_CAM[1], K_CAM[3], 0.0, 0.0, 1.0]
        self.info_pub.publish(ci)

    def _dock_detect(self):
        """dock_mark_detector(YOLO) 출력 흉내 — 0.2 s 전 영상 기준, 방위 잡음은 화면 가장자리일수록 큼."""
        now = time.monotonic()
        msg = String()
        if not self.detector_on:
            return   # 실제 검출기처럼 도킹이 아닐 때(/detector/enable false)는 상태를 안 낸다 (2026-11-14)
        pose = None
        for p in self.pose_hist:
            if p[0] >= now - 0.2:
                pose = p
                break
        if pose is None:
            return
        _, x, y, yaw = pose
        stamp_s = self.get_clock().now().nanoseconds * 1e-9 - (now - pose[0])
        cx, cy = x + self.geom.cam_offset_x * math.cos(yaw), y + self.geom.cam_offset_x * math.sin(yaw)
        rays = {}
        for name, mk in self.world.markers.items():
            if name in self.marker_hidden:
                continue
            v = mk - (cx, cy)
            rng_ = float(np.hypot(*v))
            bearing = wrap(math.atan2(v[1], v[0]) - yaw)
            off_normal = math.degrees(math.acos(max(-1.0, min(1.0, float(-(v / rng_) @ self.world.dock_n)))))
            if bearing > math.radians(39.5) or bearing < -math.radians(40.0) or off_normal > 72.0 or rng_ > 22.0 or rng_ < 0.8:
                continue
            p = 0.92 if rng_ <= 12.0 else 0.92 - 0.62 * (rng_ - 12.0) / 10.0
            p *= (1.0 if off_normal < 50 else 0.6) * self.marker_p_scale
            if self.rng.random() > p:
                continue
            noise = math.radians(0.3 + 2.3 * (bearing / math.radians(40.0)) ** 2)
            bm = bearing + self.rng.normal(0, noise)
            conf = float(self.rng.uniform(0.3, 0.55)) if self.rng.random() < 0.08 else float(self.rng.uniform(0.8, 0.96))
            ray = np.array([-math.tan(bm), -0.05, 1.0])
            ray /= np.linalg.norm(ray)
            u = K_CAM[2] - K_CAM[0] * math.tan(bm)
            rays[name] = dict(pixel_uv=[u, 215.0], optical_ray=ray.tolist(), bearing_left_deg=math.degrees(bm),
                              confidence=conf, depth_used=False)
            pt = PointStamped()
            pt.header.stamp = rclpy.time.Time(seconds=stamp_s).to_msg()
            pt.header.frame_id = 'camera_color_optical_frame'
            pt.point.x, pt.point.y, pt.point.z = map(float, ray)
            self.ray_pubs[name].publish(pt)
        msg.data = json.dumps({'valid': False, 'reason': 'target_missing_or_invalid_depth', 'target_class': 'sim',
                               'ground_valid': False, 'markers': {}, 'sight_rays': rays, 'image_stamp_s': stamp_s,
                               'image_half_fov_deg': {'left': 40.0, 'right': 40.6}, 'depth_quality': {}})
        self.det_pub.publish(msg)

    def _gps(self):
        b = self.boat
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gnss_1_antenna'
        m.latitude = self.origin[0] + math.degrees(b.y / 6378137.0)
        m.longitude = self.origin[1] + math.degrees(b.x / (6378137.0 * math.cos(math.radians(self.origin[0]))))
        m.status.status = 2
        self.gps_pub.publish(m)

    def _log(self):
        b = self.boat
        rt = self.run_time()
        self.trace.write(json.dumps({
            't': round(self.now(), 2), 'rt': None if rt is None else round(rt, 2),
            'x': round(b.x, 3), 'y': round(b.y, 3), 'yaw': round(math.degrees(b.yaw), 1), 'u': round(b.u, 3),
            'r': round(b.r, 3), 'L': int(self.pwm[0]), 'R': int(self.pwm[1]),
            'cl': round(self.cmd.linear.x, 3), 'ca': round(self.cmd.angular.z, 3), 'state': self.state}) + '\n')
        if self.k % 250 == 0:
            self.trace.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arena', required=True)
    ap.add_argument('--nomoto', required=True)
    ap.add_argument('--scenario', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--max-time', type=float, default=960.0)
    args = ap.parse_args()
    rclpy.init()
    node = SimNode(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        json.dump({'min_clear': {k: round(v, 3) for k, v in node.min_clear.items() if v < 1.0}},
                  open(os.path.join(args.out, 'clearance.json'), 'w'), ensure_ascii=False)
        try:
            node.trace.close()
            node.events.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
