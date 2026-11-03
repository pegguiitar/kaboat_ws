#!/usr/bin/env python3
"""pytest 결과 집계 — junit XML 과 L2 궤적 데이터로 보고서용 표·그림을 만든다.

  scripts/run_tests.sh                          # test_output/junit.xml
  KABOAT_TEST_FIGURES=1 scripts/run_tests.sh -m level_mission   # 궤적 JSON 도 저장
  python scripts/test_report.py                 # test_output/report/ 에 결과

Docker 이미지에는 한글 글꼴이 없어서 그림은 이 스크립트가 호스트(Windows)에서 그린다.
분류(단·묶음·기능·체크리스트)는 conftest.py 가 junit 속성으로 남긴 값을 그대로 쓴다.
"""
import argparse
import collections
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from tests.testmap import GROUPS  # noqa: E402

GROUP_ORDER = ['mission_mgmt', 'common', 'gate', 'station', 'dock', 'search', 'avoid',
               'perception', 'localization', 'control']
MISSIONS = [('gate', 'Gate'), ('station', 'Station'), ('dock', 'Dock'),
            ('search', 'Search'), ('avoid', 'Avoid')]
CHECKS = ['A', 'B', 'C', 'D']
CHECK_LABEL = {'A': 'A 정상', 'B': 'B 경계', 'C': 'C 실패 안전', 'D': 'D 노드 입출력'}

# dataviz 기준 팔레트 (밝은 면)
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
NEUTRAL = '#f0efec'
SERIES_1 = '#2a78d6'
GOOD = '#0ca30c'
CRITICAL = '#d03b3b'
BLUE_RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95']


# ── junit 읽기 ─────────────────────────────────────────

def load_cases(junit_path):
    cases = []
    for tc in ET.parse(junit_path).getroot().iter('testcase'):
        props = {p.get('name'): p.get('value') for p in tc.iter('property')}
        failure = tc.find('failure')
        if failure is None:
            failure = tc.find('error')
        outcome = ('failed' if failure is not None
                   else 'skipped' if tc.find('skipped') is not None else 'passed')
        message = (failure.get('message') or '').splitlines()[0] if failure is not None else ''
        cases.append({
            'file': tc.get('classname', '').split('.')[-1],
            'name': tc.get('name'),
            'outcome': outcome,
            'message': message,
            'level': props.get('level', 'unclassified'),
            'group': props.get('group', ''),
            'function': props.get('function', ''),
            'check': props.get('check', 'untagged'),
        })
    return cases


def mission_result(cases, mission, test_name):
    for c in cases:
        if c['file'] == f'test_mission_{mission}' and c['name'] == test_name:
            return c
    return None


def l2_columns(cases):
    seeds = sorted({int(c['name'].split('[')[1].rstrip(']')) for c in cases
                    if c['level'] == 'mission' and '[' in c['name']})
    cols = [('S1', 'nominal', 'S1')]
    cols += [(f'S2-{k}', f'disturbed[{k}]', 'S2') for k in seeds]
    return cols


def l2_test_name(mission, key, cases):
    if key == 'nominal':
        return f'test_{mission}_nominal'
    if key.startswith('disturbed'):
        return f'test_{mission}_{key}'
    s3 = [c['name'] for c in cases if c['file'] == f'test_mission_{mission}'
          and c['check'] == 'S3']
    return s3[0] if s3 else None


# ── 표 (Markdown) ──────────────────────────────────────

def summary_markdown(cases):
    lines = ['# pytest 결과 집계', '']
    by_level = collections.OrderedDict(
        (lv, collections.Counter()) for lv in ('unit', 'mission', 'system', 'tool', 'excluded'))
    for c in cases:
        by_level.setdefault(c['level'], collections.Counter())[c['outcome']] += 1
    label = {'unit': 'L1 기능', 'mission': 'L2 미션', 'system': 'L3 전체 흐름',
             'tool': '시험 도구 검증', 'excluded': '범위 밖 (수조 시험 도구)'}
    lines += ['## 단별 결과', '', '| 단 | 시험 수 | 통과 | 실패 |', '|---|---|---|---|']
    for lv, cnt in by_level.items():
        total = sum(cnt.values())
        if total:
            lines.append(f"| {label.get(lv, lv)} | {total} | {cnt['passed']} | {cnt['failed']} |")
    lines.append('')

    lines += ['## L1 기능 × 체크리스트', '',
              '| 묶음 | 기능 | A 정상 | B 경계 | C 실패 시 안전 | D 노드 입출력 | 실패 |',
              '|---|---|---|---|---|---|---|']
    for group, function, counts, failed in l1_matrix(cases):
        cells = [str(counts[k]) if counts[k] else '–' for k in CHECKS]
        lines.append(f"| {GROUPS[group]} | {function} | " + ' | '.join(cells) + f' | {failed} |')
    lines.append('')

    cols = l2_columns(cases) + [('S3', None, 'S3')]
    lines += ['## L2 미션 시험', '',
              '| 미션 | ' + ' | '.join(c[0] for c in cols) + ' | 통과 |',
              '|---|' + '---|' * (len(cols) + 1)]
    for key, title in MISSIONS:
        row, ok = [], 0
        for col, k, _ in cols:
            r = mission_result(cases, key, l2_test_name(key, k or 'S3', cases))
            mark = '통과' if r and r['outcome'] == 'passed' else '실패' if r else '–'
            ok += mark == '통과'
            row.append(mark)
        lines.append(f'| {title} | ' + ' | '.join(row) + f' | {ok}/{len(cols)} |')
    lines.append('')

    lines += ['## L3 전체 흐름', '', '| 시험 | 결과 |', '|---|---|']
    for c in cases:
        if c['level'] == 'system':
            lines.append(f"| {c['name']} | {'통과' if c['outcome'] == 'passed' else '실패'} |")
    lines.append('')

    failed = [c for c in cases if c['outcome'] == 'failed']
    lines += [f'## 실패 목록 ({len(failed)}개)', '']
    for c in failed:
        lines.append(f"- `{c['file']}::{c['name']}` — {c['message']}")
    lines.append('')
    return '\n'.join(lines)


def l1_matrix(cases):
    table = collections.OrderedDict()
    for c in cases:
        if c['level'] != 'unit':
            continue
        key = (c['group'], c['function'])
        counts, failed = table.setdefault(key, (collections.Counter(), [0]))
        counts[c['check']] += 1
        failed[0] += c['outcome'] == 'failed'
    rows = [(g, f, counts, failed[0]) for (g, f), (counts, failed) in table.items()]
    rows.sort(key=lambda r: (GROUP_ORDER.index(r[0]) if r[0] in GROUP_ORDER else 99, r[1]))
    return rows


# ── 그림 ──────────────────────────────────────────────

def setup_matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    for font in ('Malgun Gothic', 'NanumGothic', 'Noto Sans CJK KR', 'AppleGothic'):
        if font in names:
            plt.rcParams['font.family'] = font
            break
    plt.rcParams.update({
        'axes.unicode_minus': False, 'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK_2, 'xtick.color': INK_2,
        'ytick.color': INK_2, 'text.color': INK, 'font.size': 9,
    })
    return plt


def ramp_color(n, top):
    if n <= 0:
        return NEUTRAL
    i = min(len(BLUE_RAMP) - 1, int((n - 1) / max(top, 1) * (len(BLUE_RAMP) - 1) + 0.5))
    return BLUE_RAMP[i]


def text_on(color):
    return '#ffffff' if color in BLUE_RAMP[3:] or color in (GOOD, CRITICAL) else INK


def fig_l1_checklist(plt, cases, out):
    rows = l1_matrix(cases)
    top = max(max(counts.values()) for _, _, counts, _ in rows)
    h = 0.28 * len(rows) + 1.2
    fig, ax = plt.subplots(figsize=(7.2, h))
    prev = None
    for i, (group, function, counts, failed) in enumerate(rows):
        y = len(rows) - 1 - i
        for j, k in enumerate(CHECKS):
            n = counts[k]
            color = ramp_color(n, top)
            ax.add_patch(plt.Rectangle((j + 0.04, y + 0.06), 0.92, 0.88, color=color, lw=0))
            ax.text(j + 0.5, y + 0.5, str(n) if n else '–', ha='center', va='center',
                    color=text_on(color) if n else MUTED, fontsize=8)
        label = function + (f'  (실패 {failed})' if failed else '')
        ax.text(-0.15, y + 0.5, label, ha='right', va='center', fontsize=8,
                color=CRITICAL if failed else INK)
        if group != prev:
            ax.text(-4.9, y + 0.5, GROUPS[group], ha='left', va='center', fontsize=8,
                    color=INK_2, fontweight='bold')
            if prev is not None:
                ax.axhline(y + 1, xmin=0, xmax=1, color=GRID, lw=0.8)
            prev = group
    for j, k in enumerate(CHECKS):
        ax.text(j + 0.5, len(rows) + 0.3, CHECK_LABEL[k], ha='center', va='bottom',
                fontsize=8, color=INK_2)
    ax.set_xlim(-5.0, len(CHECKS))
    ax.set_ylim(-0.2, len(rows) + 1.0)
    ax.axis('off')
    fig.text(0.01, 0.005, '칸의 숫자는 시험 수, – 는 해당 항목 시험 없음 (D 는 노드만 해당)',
             fontsize=7, color=MUTED)
    fig.tight_layout()
    path = out / 'fig_L1_checklist.png'
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def fig_l2_results(plt, cases, out):
    cols = l2_columns(cases) + [('S3', None, 'S3')]
    fig, ax = plt.subplots(figsize=(7.2, 2.6))
    for i, (key, title) in enumerate(MISSIONS):
        y = len(MISSIONS) - 1 - i
        ok = 0
        for j, (col, k, _) in enumerate(cols):
            r = mission_result(cases, key, l2_test_name(key, k or 'S3', cases))
            passed = r is not None and r['outcome'] == 'passed'
            ok += passed
            color = GOOD if passed else CRITICAL if r else NEUTRAL
            ax.add_patch(plt.Rectangle((j + 0.04, y + 0.08), 0.92, 0.84, color=color, lw=0))
            ax.text(j + 0.5, y + 0.5, '통과' if passed else '실패' if r else '–',
                    ha='center', va='center', color=text_on(color), fontsize=8)
        ax.text(-0.15, y + 0.5, title, ha='right', va='center', fontsize=9)
        ax.text(len(cols) + 0.15, y + 0.5, f'{ok}/{len(cols)}', ha='left', va='center',
                fontsize=9, color=INK_2)
    for j, (col, _, _) in enumerate(cols):
        ax.text(j + 0.5, len(MISSIONS) + 0.15, col, ha='center', va='bottom', fontsize=8,
                color=INK_2)
    ax.set_xlim(-1.3, len(cols) + 0.9)
    ax.set_ylim(-0.1, len(MISSIONS) + 0.7)
    ax.axis('off')
    fig.text(0.01, 0.01, 'S1 기본, S2 잡음·조류(seed 별), S3 실패 주입', fontsize=7, color=MUTED)
    fig.tight_layout()
    path = out / 'fig_L2_results.png'
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def _on_bounds(x0, y0, x1, y1, bounds):
    """선분이 수조 경계 위에 놓였는지 — 수조 벽은 연하게 그리고 축 범위에서 뺀다."""
    xmin, ymin, xmax, ymax = bounds
    return ((x0 == x1 and x0 in (xmin, xmax)) or (y0 == y1 and y0 in (ymin, ymax)))


def fig_l2_trajectory(plt, cases, runs_dir, mission, title, out):
    runs = sorted(runs_dir.glob(f'L2_{mission}_*.json'))
    if not runs:
        return None
    fig, ax = plt.subplots(figsize=(6.0, 4.8))
    world_drawn = False
    handles = {}
    focus = []        # 축 범위를 정할 점들 — 수조 벽은 빼고 궤적·장애물·목표만
    for path in runs:
        data = json.loads(path.read_text(encoding='utf-8'))
        if not world_drawn:
            bounds = data['bounds']
            for x0, y0, x1, y1 in data['segments']:
                wall = bounds is not None and _on_bounds(x0, y0, x1, y1, bounds)
                ax.plot([x0, x1], [y0, y1], color=MUTED if wall else INK_2,
                        lw=1.5 if wall else 2.5, solid_capstyle='round', zorder=1)
                if not wall:
                    focus += [(x0, y0), (x1, y1)]
            for x, y, r in data['circles']:
                ax.add_patch(plt.Circle((x, y), r, color=MUTED, zorder=2))
                focus.append((x, y))
            if data['goal']:
                h, = ax.plot(*data['goal'], marker='*', ms=13, color=INK, ls='none', zorder=5)
                handles['목표'] = h
                focus.append(tuple(data['goal']))
            world_drawn = True
        name = data['name']
        if name.endswith('_S1'):
            test = f'test_{mission}_nominal'
        else:
            test = f"test_{mission}_disturbed[{name.rsplit('seed', 1)[1]}]"
        r = mission_result(cases, mission, test)
        passed = r is not None and r['outcome'] == 'passed'
        xs = [s[1] for s in data['samples']]
        ys = [s[2] for s in data['samples']]
        focus += list(zip(xs, ys))
        if name.endswith('_S1'):
            h, = ax.plot(xs, ys, color=SERIES_1, lw=2.0, zorder=4)
            key = 'S1 기본' + ('' if passed else ' (실패, × 끝 위치)')
            if not passed:
                ax.plot(xs[-1], ys[-1], marker='x', ms=9, mew=2.2, color=SERIES_1, zorder=5)
        elif passed:
            h, = ax.plot(xs, ys, color=MUTED, lw=1.0, zorder=3)
            key = 'S2 잡음·조류 (통과)'
        else:
            h, = ax.plot(xs, ys, color=CRITICAL, lw=1.2, ls=(0, (4, 2)), zorder=3)
            ax.plot(xs[-1], ys[-1], marker='x', ms=8, mew=2, color=CRITICAL, zorder=4)
            key = 'S2 잡음·조류 (실패, × 끝 위치)'
        if not name.endswith('_S1') or key not in handles:
            handles.setdefault(key, h)
        ax.plot(xs[0], ys[0], marker='o', ms=5, color=h.get_color(), zorder=4)
    fx = [p[0] for p in focus]
    fy = [p[1] for p in focus]
    margin = 1.5
    x0, x1 = min(fx) - margin, max(fx) + margin
    y0, y1 = min(fy) - margin, max(fy) + margin
    if y1 - y0 < 4.0:                      # 직선 궤적도 납작하지 않게 세로 최소 4 m
        cy = (y0 + y1) / 2
        y0, y1 = cy - 2.0, cy + 2.0
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.grid(color=GRID, lw=0.6)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.set_title(f'{title} 미션 궤적', fontsize=10, color=INK, loc='left')
    order = [k for k in ('S1 기본', 'S1 기본 (실패, × 끝 위치)', 'S2 잡음·조류 (통과)',
                         'S2 잡음·조류 (실패, × 끝 위치)', '목표') if k in handles]
    ax.legend([handles[k] for k in order], order, fontsize=8, frameon=False,
              loc='upper center', bbox_to_anchor=(0.5, -0.26), ncol=min(len(order), 3))
    path = out / f'fig_L2_traj_{mission}.png'
    fig.savefig(path, dpi=200, bbox_inches='tight', pad_inches=0.15)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--junit', default=REPO / 'test_output' / 'junit.xml', type=Path)
    ap.add_argument('--runs', default=REPO / 'test_output' / 'figures', type=Path)
    ap.add_argument('--out', default=REPO / 'test_output' / 'report', type=Path)
    args = ap.parse_args()
    cases = load_cases(args.junit)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'summary.md').write_text(summary_markdown(cases), encoding='utf-8')
    plt = setup_matplotlib()
    made = [args.out / 'summary.md', fig_l1_checklist(plt, cases, args.out),
            fig_l2_results(plt, cases, args.out)]
    for key, title in MISSIONS:
        made.append(fig_l2_trajectory(plt, cases, args.runs, key, title, args.out))
    for path in made:
        if path is not None:
            print(path.relative_to(REPO))


if __name__ == '__main__':
    main()
