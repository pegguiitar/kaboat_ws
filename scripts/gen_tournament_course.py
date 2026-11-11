#!/usr/bin/env python3
"""토너먼트 경기장(kaboat_2026_tournament) model.sdf 생성기.

현장 사진 기준 구성물:
  - 빨간 사각 기둥 4개 : 외곽 경계 사각형의 네 꼭짓점.
  - 노란 구슬 로프      : 기둥과 기둥 사이를 잇는 경계선. 실물도 작은 부표라
                          라이다에 거의 안 잡히므로 여기서도 visual 전용
                          (충돌 형상 없음) 으로 둔다. --rope-solid 로 켤 수 있다.
  - 붐 세그먼트         : 흰 부표 + 주황 막대 + 흰 부표 한 벌. 실제 회피 대상이라
                          collision + visual 둘 다 가진다.

좌표계: 모델 원점 = 경계 사각형 좌하단 꼭짓점, +x = 긴 방향, +y = 짧은 방향.
world 배치 오프셋은 worlds/perception_task.sdf 의 <include><pose> 가 정한다.

경기장 전체를 <static>true</static> 모델의 링크로 찍는다. 기존
kaboat_2026_comprehensive_mission 의 dock_structure 와 같은 방식이고, 부표마다
<include> 로 별도 모델을 띄우는 것보다 엔티티 수가 훨씬 적다. 정적이라 파도에
떠밀려 배치가 틀어지지 않아서 같은 코스를 반복 주행하기에도 좋다.
(실물처럼 출렁이게 하려면 붐을 basic_buoy_white 처럼 부력 플러그인 가진 개별
모델로 빼야 한다 — 지금은 재현성을 택했다.)

사용:
    python3 scripts/gen_tournament_course.py                 # 기본값으로 재생성
    python3 scripts/gen_tournament_course.py --length 30 --width 20
    python3 scripts/gen_tournament_course.py --check-only    # 통로 폭만 확인
"""

import argparse
import math
import os

# ── 기본 치수 ──────────────────────────────────────────────────────────────
DEF_LENGTH = 20.0        # [m] 경계 사각형 가로 (+x)
DEF_WIDTH = 15.0         # [m] 경계 사각형 세로 (+y)

POST_SIDE = 0.35         # [m] 빨간 기둥 한 변
POST_HEIGHT = 1.2        # [m] 기둥 전체 높이
POST_FREEBOARD = 0.7     # [m] 수면 위로 나온 높이 (나머지는 물속)

BEAD_RADIUS = 0.09       # [m] 경계 로프 구슬 반지름
BEAD_SPACING = 0.8       # [m] 구슬 간격
BEAD_Z = 0.02            # [m] 구슬 중심 높이 (수면에 살짝 잠김)

BAR_RADIUS = 0.14        # [m] 주황 막대 반지름
BAR_Z = 0.13             # [m] 막대 중심 높이
FLOAT_RADIUS = 0.40      # [m] 흰 부표 반지름. 라이다(선체중심+0.4m)에 걸리도록
                         #      실물보다 약간 키웠다. 더 키우려면 이 값만 올린다.
FLOAT_LENGTH = 0.55      # [m] 흰 부표 원통부 길이 (캡슐이라 양끝 반구가 더 붙음)
FLOAT_Z = 0.10           # [m] 흰 부표 중심 높이
FLOAT_GAP = 0.18         # [m] 막대 끝과 흰 부표 몸통 사이 여유

# 색 (ambient/diffuse 공용)
C_POST = (0.90, 0.10, 0.10)
C_BEAD = (0.95, 0.75, 0.15)
C_BAR = (1.00, 0.30, 0.05)
C_FLOAT = (0.95, 0.95, 0.95)
C_GOAL = (0.05, 0.85, 0.15)

# ── 붐 세그먼트 배치 ───────────────────────────────────────────────────────
# (x, y, yaw_deg, bar_len) — x/y 는 경계 사각형 좌하단 기준 [m], 기본 20×15 에 맞춘 값.
# yaw 0 = 막대가 +x 방향. 사진처럼 긴 방향과 나란한 막대들을 x/y 로 엇갈리게 깔아
# 배가 좌우로 짜며 통과하게 했다. 크기를 바꾸면 BOOMS 도 같이 손봐야 한다
# (--length/--width 는 경계와 로프만 늘리고 이 좌표는 그대로 쓴다).
BOOMS = [
    (4.0,  10.0,  0.0, 2.5),
    (4.5,   4.5,  0.0, 2.5),
    (8.5,   7.5,  0.0, 2.5),
    (9.0,  12.5,  0.0, 2.0),
    (13.0,  4.0,  0.0, 2.5),
    (13.5, 10.5,  0.0, 2.5),
    (17.0,  7.0,  0.0, 2.0),
]

# 막대 없이 홀로 떠 있는 흰 부표 (사진에도 몇 개 보인다)
LOOSE_FLOATS = [
    (6.8, 12.8),
    (11.0, 1.8),
    (15.5, 13.2),
    (18.3, 2.5),
]

ENTRY = (0.0, 7.5)       # 배가 들어오는 지점 (좌변 중앙)
GOAL = (18.8, 11.0)      # 목적지


def rgba(c):
    return f'{c[0]} {c[1]} {c[2]} 1.0'


def material(c):
    return (f'        <material>\n'
            f'          <ambient>{rgba(c)}</ambient>\n'
            f'          <diffuse>{rgba(c)}</diffuse>\n'
            f'          <specular>0.3 0.3 0.3 1.0</specular>\n'
            f'        </material>\n')


def shape(name, pose, geom, color, collision=True):
    """collision(선택) + visual 한 쌍을 찍는다."""
    out = ''
    if collision:
        out += (f'      <collision name="{name}_collision">\n'
                f'        <pose>{pose}</pose>\n'
                f'        <geometry>{geom}</geometry>\n'
                f'      </collision>\n')
    out += (f'      <visual name="{name}_visual">\n'
            f'        <pose>{pose}</pose>\n'
            f'        <geometry>{geom}</geometry>\n'
            f'{material(color)}'
            f'      </visual>\n')
    return out


def box(sx, sy, sz):
    return f'<box><size>{sx} {sy} {sz}</size></box>'


def sphere(r):
    return f'<sphere><radius>{r}</radius></sphere>'


def cylinder(r, ln):
    return f'<cylinder><radius>{r}</radius><length>{ln}</length></cylinder>'


def capsule(r, ln):
    return f'<capsule><radius>{r}</radius><length>{ln}</length></capsule>'


def corner_posts(length, width):
    """네 꼭짓점의 빨간 기둥. 수면 위 POST_FREEBOARD 만큼 나오게 z 를 잡는다."""
    z = POST_FREEBOARD - POST_HEIGHT / 2.0
    out = '    <link name="corner_posts">\n'
    corners = [('sw', 0.0, 0.0), ('se', length, 0.0),
               ('ne', length, width), ('nw', 0.0, width)]
    for tag, x, y in corners:
        out += shape(f'post_{tag}', f'{x} {y} {z:.3f} 0 0 0',
                     box(POST_SIDE, POST_SIDE, POST_HEIGHT), C_POST)
    out += '    </link>\n'
    return out


def boundary_rope(length, width, solid):
    """기둥 사이를 잇는 노란 구슬 줄. 기본은 visual 전용."""
    out = '    <link name="boundary_rope">\n'
    edges = [((0.0, 0.0), (length, 0.0)), ((length, 0.0), (length, width)),
             ((length, width), (0.0, width)), ((0.0, width), (0.0, 0.0))]
    n = 0
    for (x0, y0), (x1, y1) in edges:
        seg = math.hypot(x1 - x0, y1 - y0)
        count = max(1, int(round(seg / BEAD_SPACING)))
        # 양 끝(기둥 자리)은 빼고 사이만 채운다
        for i in range(1, count):
            t = i / count
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            out += shape(f'bead_{n}', f'{x:.3f} {y:.3f} {BEAD_Z} 0 0 0',
                         sphere(BEAD_RADIUS), C_BEAD, collision=solid)
            n += 1
    out += '    </link>\n'
    return out, n


def booms():
    """흰 부표 + 주황 막대 + 흰 부표 한 벌씩.

    SDF 의 cylinder/capsule 은 축이 +z 라, 막대를 바닥에 눕히려면 pitch=90°
    를 먼저 주고 yaw 로 방향을 돌린다 (회전 적용 순서가 Rz·Ry·Rx 라
    rpy=(0, 90°, yaw) 이면 축이 (cos yaw, sin yaw, 0) 으로 간다).
    """
    out = '    <link name="booms">\n'
    for i, (cx, cy, yaw_deg, bar_len) in enumerate(BOOMS):
        yaw = math.radians(yaw_deg)
        rpy = f'0 {math.pi / 2:.6f} {yaw:.6f}'
        ux, uy = math.cos(yaw), math.sin(yaw)

        out += shape(f'boom{i}_bar', f'{cx:.3f} {cy:.3f} {BAR_Z} {rpy}',
                     cylinder(BAR_RADIUS, bar_len), C_BAR)

        # 흰 부표 중심 = 막대 끝에서 (여유 + 부표 몸통 절반) 만큼 더 바깥
        d = bar_len / 2.0 + FLOAT_GAP + FLOAT_LENGTH / 2.0
        for sign, side in ((-1.0, 'a'), (1.0, 'b')):
            fx = cx + ux * d * sign
            fy = cy + uy * d * sign
            out += shape(f'boom{i}_float_{side}',
                         f'{fx:.3f} {fy:.3f} {FLOAT_Z} {rpy}',
                         capsule(FLOAT_RADIUS, FLOAT_LENGTH), C_FLOAT)
    for i, (x, y) in enumerate(LOOSE_FLOATS):
        out += shape(f'loose_float{i}', f'{x:.3f} {y:.3f} {FLOAT_Z} 0 0 0',
                     capsule(FLOAT_RADIUS, FLOAT_LENGTH), C_FLOAT)
    out += '    </link>\n'
    return out


def goal_marker():
    """목적지 표식 — visual 전용이라 충돌·라이다 경로를 막지 않는다."""
    return ('    <link name="goal_marker">\n'
            + shape('goal', f'{GOAL[0]:.3f} {GOAL[1]:.3f} 0.03 0 0 0',
                    cylinder(0.6, 0.06), C_GOAL, collision=False)
            + '    </link>\n')


def boom_extent(cx, cy, yaw_deg, bar_len):
    """붐 한 벌이 차지하는 선분의 양 끝 (흰 부표 바깥면까지)."""
    yaw = math.radians(yaw_deg)
    half = bar_len / 2.0 + FLOAT_GAP + FLOAT_LENGTH + FLOAT_RADIUS
    ux, uy = math.cos(yaw), math.sin(yaw)
    return (cx - ux * half, cy - uy * half), (cx + ux * half, cy + uy * half)


def check_clearance(length, width):
    """x 를 훑으며 그 자리에서 배가 지나갈 수 있는 제일 넓은 y 틈을 잰다.

    세그먼트를 y 구간으로 투영하는 대략적인 검사다. 실제 통과 가능 여부는
    경로계획이 정하지만, 치수를 바꿨을 때 코스가 막혔는지 바로 보려는 용도.
    """
    print(f'  통로 폭 점검 ({length}×{width} m, 0.25m 간격으로 x 훑기)')
    worst = (1e9, 0.0)
    x = 0.0
    while x <= length + 1e-9:
        blocked = []
        for cx, cy, yaw_deg, bar_len in BOOMS:
            (x0, y0), (x1, y1) = boom_extent(cx, cy, yaw_deg, bar_len)
            if min(x0, x1) - FLOAT_RADIUS <= x <= max(x0, x1) + FLOAT_RADIUS:
                blocked.append((min(y0, y1) - FLOAT_RADIUS,
                                max(y0, y1) + FLOAT_RADIUS))
        for fx, fy in LOOSE_FLOATS:
            if abs(fx - x) <= FLOAT_LENGTH / 2.0 + FLOAT_RADIUS:
                blocked.append((fy - FLOAT_RADIUS, fy + FLOAT_RADIUS))
        # 막힌 구간들을 뺀 나머지 중 제일 넓은 틈
        free, lo = 0.0, 0.0
        best = 0.0
        for b0, b1 in sorted(blocked):
            if b0 > lo:
                best = max(best, b0 - lo)
            lo = max(lo, b1)
        best = max(best, width - lo)
        free = best
        if free < worst[0]:
            worst = (free, x)
        x += 0.25
    print(f'  → 가장 좁은 곳: x={worst[1]:.2f} m 에서 {worst[0]:.2f} m')
    if worst[0] < 2.0:
        print('  ⚠ 2m 미만이다. 배 폭을 생각하면 BOOMS 좌표나 bar_len 을 줄이는 게 좋다.')
    return worst



def write_svg(path, length, width):
    """배치를 위에서 내려다본 평면도 SVG. 사진과 대조해 BOOMS 를 손볼 때 쓴다.

    외부 라이브러리 없이 문자열로 찍는다. +y 가 위로 가도록 y 축을 뒤집는다.
    """
    pad, scale = 1.5, 40.0          # [m] 여백, [px/m]
    w = (length + pad * 2) * scale
    h = (width + pad * 2) * scale

    def sx(x):
        return (x + pad) * scale

    def sy(y):
        return (width - y + pad) * scale

    def hexc(c):
        return '#%02x%02x%02x' % tuple(int(round(v * 255)) for v in c)

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
         f'viewBox="0 0 {w:.0f} {h:.0f}">',
         f'<rect width="{w:.0f}" height="{h:.0f}" fill="#13304d"/>']

    # 경계 사각형
    o.append(f'<rect x="{sx(0):.1f}" y="{sy(width):.1f}" '
             f'width="{length * scale:.1f}" height="{width * scale:.1f}" '
             f'fill="none" stroke="{hexc(C_BEAD)}" stroke-width="2" stroke-dasharray="4 4"/>')

    # 경계 구슬
    edges = [((0.0, 0.0), (length, 0.0)), ((length, 0.0), (length, width)),
             ((length, width), (0.0, width)), ((0.0, width), (0.0, 0.0))]
    for (x0, y0), (x1, y1) in edges:
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / BEAD_SPACING)))
        for i in range(1, n):
            t = i / n
            o.append(f'<circle cx="{sx(x0 + (x1 - x0) * t):.1f}" '
                     f'cy="{sy(y0 + (y1 - y0) * t):.1f}" '
                     f'r="{BEAD_RADIUS * scale:.1f}" fill="{hexc(C_BEAD)}"/>')

    # 붐 세그먼트
    for cx, cy, yaw_deg, bar_len in BOOMS:
        yaw = math.radians(yaw_deg)
        ux, uy = math.cos(yaw), math.sin(yaw)
        hb = bar_len / 2.0
        o.append(f'<line x1="{sx(cx - ux * hb):.1f}" y1="{sy(cy - uy * hb):.1f}" '
                 f'x2="{sx(cx + ux * hb):.1f}" y2="{sy(cy + uy * hb):.1f}" '
                 f'stroke="{hexc(C_BAR)}" stroke-width="{BAR_RADIUS * 2 * scale:.1f}" '
                 f'stroke-linecap="round"/>')
        d = hb + FLOAT_GAP + FLOAT_LENGTH / 2.0
        for sign in (-1.0, 1.0):
            o.append(f'<circle cx="{sx(cx + ux * d * sign):.1f}" '
                     f'cy="{sy(cy + uy * d * sign):.1f}" '
                     f'r="{FLOAT_RADIUS * scale:.1f}" fill="{hexc(C_FLOAT)}"/>')

    for x, y in LOOSE_FLOATS:
        o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" '
                 f'r="{FLOAT_RADIUS * scale:.1f}" fill="{hexc(C_FLOAT)}"/>')

    # 빨간 기둥
    for x, y in ((0, 0), (length, 0), (length, width), (0, width)):
        s_px = POST_SIDE * scale * 2.2        # 작아서 안 보이므로 과장해 그린다
        o.append(f'<rect x="{sx(x) - s_px / 2:.1f}" y="{sy(y) - s_px / 2:.1f}" '
                 f'width="{s_px:.1f}" height="{s_px:.1f}" fill="{hexc(C_POST)}"/>')

    # 진입 / 목적지
    o.append(f'<circle cx="{sx(GOAL[0]):.1f}" cy="{sy(GOAL[1]):.1f}" '
             f'r="{0.6 * scale:.1f}" fill="none" stroke="{hexc(C_GOAL)}" stroke-width="3"/>')
    o.append(f'<text x="{sx(GOAL[0]) + 14:.1f}" y="{sy(GOAL[1]) + 5:.1f}" '
             f'fill="{hexc(C_GOAL)}" font-family="sans-serif" font-size="14">GOAL</text>')
    o.append(f'<polygon points="{sx(ENTRY[0]) - 18:.1f},{sy(ENTRY[1]) - 9:.1f} '
             f'{sx(ENTRY[0]) - 18:.1f},{sy(ENTRY[1]) + 9:.1f} '
             f'{sx(ENTRY[0]) - 4:.1f},{sy(ENTRY[1]):.1f}" fill="#4fc3f7"/>')
    o.append(f'<text x="{sx(ENTRY[0]) - 46:.1f}" y="{sy(ENTRY[1]) - 16:.1f}" '
             f'fill="#4fc3f7" font-family="sans-serif" font-size="13">ENTRY</text>')
    o.append(f'<text x="10" y="{h - 10:.0f}" fill="#9fb6cc" font-family="sans-serif" '
             f'font-size="13">{length:g} x {width:g} m  |  +x right, +y up  |  '
             f'generated by gen_tournament_course.py</text>')
    o.append('</svg>')
    open(path, 'w').write('\n'.join(o) + '\n')

def build(length, width, rope_solid):
    rope, n_beads = boundary_rope(length, width, rope_solid)
    body = (
        '<?xml version="1.0" ?>\n'
        '<sdf version="1.7">\n'
        '  <model name="kaboat_2026_tournament">\n'
        '    <static>true</static>\n'
        '\n'
        '    <!-- =====================================================================\n'
        '         ⚠ 이 파일은 scripts/gen_tournament_course.py 가 찍어낸다.\n'
        '           손으로 고치지 말고 스크립트의 치수/BOOMS 를 고쳐 다시 돌릴 것.\n'
        '\n'
        '         현장 사진 기준 토너먼트 경기장.\n'
        f'         경계 사각형 {length} × {width} m, 모델 원점 = 좌하단 꼭짓점,\n'
        '         +x = 긴 방향, +y = 짧은 방향.\n'
        f'         진입 {ENTRY[0]}, {ENTRY[1]} → 목적지 {GOAL[0]}, {GOAL[1]} (모델 좌표).\n'
        '         world 좌표로 쓰려면 perception_task.sdf 의 include pose 를 더할 것.\n'
        '\n'
        f'         구성: 빨간 기둥 4개 / 노란 경계 구슬 {n_beads}개'
        f'{" (충돌 있음)" if rope_solid else " (visual 전용)"}\n'
        f'                 붐 세그먼트 {len(BOOMS)}벌 / 홀로 뜬 흰 부표 {len(LOOSE_FLOATS)}개\n'
        '    ===================================================================== -->\n'
        '\n'
    )
    body += corner_posts(length, width)
    body += '\n' + rope
    body += '\n' + booms()
    body += '\n' + goal_marker()
    body += '  </model>\n</sdf>\n'
    return body, n_beads


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_out = os.path.join(
        here, 'src/kaboat_sim/models/kaboat_2026_tournament/model.sdf')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--length', type=float, default=DEF_LENGTH, help='경계 가로 [m]')
    ap.add_argument('--width', type=float, default=DEF_WIDTH, help='경계 세로 [m]')
    ap.add_argument('--rope-solid', action='store_true',
                    help='경계 구슬에 충돌 형상을 준다 (기본: visual 전용)')
    ap.add_argument('--out', default=default_out, help='출력 model.sdf 경로')
    ap.add_argument('--check-only', action='store_true',
                    help='파일은 안 쓰고 통로 폭만 확인')
    ap.add_argument('--no-svg', action='store_true',
                    help='평면도 layout.svg 를 안 쓴다 (기본: model.sdf 옆에 같이 쓴다)')
    args = ap.parse_args()

    check_clearance(args.length, args.width)
    if args.check_only:
        return

    sdf, n_beads = build(args.length, args.width, args.rope_solid)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(sdf)
    print(f'  기둥 4 / 구슬 {n_beads} / 붐 {len(BOOMS)}벌 / 흰 부표 {len(LOOSE_FLOATS)}')
    print(f'  → {args.out} ({len(sdf.splitlines())} 줄)')

    if not args.no_svg:
        svg = os.path.join(os.path.dirname(args.out), 'layout.svg')
        write_svg(svg, args.length, args.width)
        print(f'  → {svg} (평면도)')


if __name__ == '__main__':
    main()
