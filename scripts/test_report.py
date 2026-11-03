#!/usr/bin/env python3
"""pytest L1 결과 집계 — junit XML 로 보고서용 표·그림을 만든다.

  scripts/run_tests.sh            # test_output/junit.xml
  python scripts/test_report.py   # test_output/report/ 에 결과

보고서에는 L1 기능 시험만 넣는다. L2·L3 시험은 저장소에 남아 있지만 집계하지 않는다.
분류(묶음·기능·체크리스트)는 conftest.py 가 junit 속성으로 남긴 값을 그대로 쓴다.
Docker 이미지에는 한글 글꼴이 없어서 그림은 이 스크립트가 호스트(Windows)에서 그린다.
"""
import argparse
import collections
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from tests.testmap import GROUPS  # noqa: E402

GROUP_ORDER = ['mission_mgmt', 'common', 'gate', 'station', 'dock', 'search', 'avoid',
               'perception', 'localization', 'control']
CHECKS = ['A', 'B', 'C', 'D']
CHECK_LABEL = {'A': 'A 정상', 'B': 'B 경계', 'C': 'C 실패 안전', 'D': 'D 노드 입출력'}
MINIMUM = {'A': 2, 'B': 2, 'C': 2, 'D': 1}      # TEST_PLAN.md 3.2 — D 는 노드만

# dataviz 기준 팔레트 (밝은 면)
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
NEUTRAL = '#f0efec'
GOOD = '#0ca30c'
CRITICAL = '#d03b3b'
BLUE_RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95']


def load_cases(junit_path):
    cases = []
    for tc in ET.parse(junit_path).getroot().iter('testcase'):
        props = {p.get('name'): p.get('value') for p in tc.iter('property')}
        failure = tc.find('failure')
        if failure is None:
            failure = tc.find('error')
        outcome = ('failed' if failure is not None
                   else 'skipped' if tc.find('skipped') is not None else 'passed')
        cases.append({
            'file': tc.get('classname', '').split('.')[-1],
            'name': tc.get('name'),
            'outcome': outcome,
            'message': (failure.get('message') or '').splitlines()[0] if failure is not None else '',
            'level': props.get('level', 'unclassified'),
            'group': props.get('group', ''),
            'function': props.get('function', ''),
            'check': props.get('check', 'untagged'),
        })
    return cases


def l1_rows(cases):
    """[(group, function, Counter(check), failed, is_node)] — 묶음 순서대로."""
    table = collections.OrderedDict()
    for c in cases:
        if c['level'] != 'unit':
            continue
        counts, failed = table.setdefault((c['group'], c['function']),
                                          (collections.Counter(), [0]))
        counts[c['check']] += 1
        failed[0] += c['outcome'] == 'failed'
    rows = [(g, f, counts, failed[0], counts['D'] > 0)
            for (g, f), (counts, failed) in table.items()]
    rows.sort(key=lambda r: (GROUP_ORDER.index(r[0]) if r[0] in GROUP_ORDER else 99, r[1]))
    return rows


def below_minimum(counts, is_node, check):
    if check == 'D' and not is_node:
        return False
    return counts[check] < MINIMUM[check]


def summary_markdown(cases):
    unit = [c for c in cases if c['level'] == 'unit']
    passed = sum(c['outcome'] == 'passed' for c in unit)
    failed = [c for c in unit if c['outcome'] == 'failed']
    rows = l1_rows(cases)
    lines = ['# pytest L1 기능 시험 결과', '',
             f'기능 {len(rows)}개, 시험 {len(unit)}개 중 {passed}개 통과, {len(failed)}개 실패.',
             '최소 기준: A·B·C 각 2개 이상, D 는 노드만 1개 이상.', '',
             '| 묶음 | 기능 | A 정상 | B 경계 | C 실패 시 안전 | D 노드 입출력 | 합계 | 결과 |',
             '|---|---|---|---|---|---|---|---|']
    for group, function, counts, n_failed, is_node in rows:
        cells = []
        for k in CHECKS:
            if k == 'D' and not is_node:
                cells.append('–')
            else:
                mark = ' (기준 미달)' if below_minimum(counts, is_node, k) else ''
                cells.append(f'{counts[k]}{mark}')
        result = '통과' if n_failed == 0 else f'실패 {n_failed}'
        lines.append(f"| {GROUPS[group]} | {function} | " + ' | '.join(cells)
                     + f' | {sum(counts.values())} | {result} |')
    lines += ['', f'## 실패한 시험 ({len(failed)}개)', '']
    for c in failed:
        lines.append(f"- `{c['file']}::{c['name']}` — {c['message']}")
    lines.append('')
    return '\n'.join(lines)


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
        'text.color': INK, 'font.size': 9,
    })
    return plt


def ramp_color(n, top):
    if n <= 0:
        return NEUTRAL
    i = min(len(BLUE_RAMP) - 1, int((n - 1) / max(top - 1, 1) * (len(BLUE_RAMP) - 1) + 0.5))
    return BLUE_RAMP[i]


def text_on(color):
    return '#ffffff' if color in BLUE_RAMP[3:] or color in (GOOD, CRITICAL) else INK


def fig_l1_checklist(plt, cases, out):
    rows = l1_rows(cases)
    top = max(max(counts[k] for k in CHECKS) for _, _, counts, _, _ in rows)
    n_cols = len(CHECKS) + 1                     # A~D + 결과
    fig, ax = plt.subplots(figsize=(7.6, 0.28 * len(rows) + 1.3))
    prev = None
    for i, (group, function, counts, n_failed, is_node) in enumerate(rows):
        y = len(rows) - 1 - i
        for j, k in enumerate(CHECKS):
            if k == 'D' and not is_node:
                color, label, ink = NEUTRAL, '–', MUTED
            else:
                n = counts[k]
                color = ramp_color(n, top)
                label, ink = str(n), text_on(color)
            ax.add_patch(plt.Rectangle((j + 0.04, y + 0.06), 0.92, 0.88, color=color, lw=0))
            if below_minimum(counts, is_node, k):
                ax.add_patch(plt.Rectangle((j + 0.06, y + 0.08), 0.88, 0.84, fill=False,
                                           ec=CRITICAL, lw=1.6))
            ax.text(j + 0.5, y + 0.5, label, ha='center', va='center', color=ink, fontsize=8)
        ok = n_failed == 0
        color = GOOD if ok else CRITICAL
        ax.add_patch(plt.Rectangle((len(CHECKS) + 0.04, y + 0.06), 0.92, 0.88, color=color, lw=0))
        ax.text(len(CHECKS) + 0.5, y + 0.5, '통과' if ok else f'실패 {n_failed}',
                ha='center', va='center', color='#ffffff', fontsize=8)
        ax.text(-0.15, y + 0.5, function, ha='right', va='center', fontsize=8)
        if group != prev:
            ax.text(-4.9, y + 0.5, GROUPS[group], ha='left', va='center', fontsize=8,
                    color=INK_2, fontweight='bold')
            if prev is not None:
                ax.plot([-4.9, n_cols], [y + 1, y + 1], color=GRID, lw=0.8)
            prev = group
    for j, label in enumerate([CHECK_LABEL[k] for k in CHECKS] + ['결과']):
        ax.text(j + 0.5, len(rows) + 0.3, label, ha='center', va='bottom', fontsize=8, color=INK_2)
    ax.set_xlim(-5.0, n_cols)
    ax.set_ylim(-0.2, len(rows) + 1.0)
    ax.axis('off')
    unit = [c for c in cases if c['level'] == 'unit']
    passed = sum(c['outcome'] == 'passed' for c in unit)
    fig.text(0.01, 0.005,
             f'칸의 숫자는 시험 수 (총 {len(unit)}개, 통과 {passed}개). '
             '최소 기준 A·B·C 2개, D 1개(노드만) — 미달 칸은 빨간 테두리, – 는 해당 없음',
             fontsize=7, color=MUTED)
    fig.tight_layout()
    path = out / 'fig_L1_checklist.png'
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--junit', default=REPO / 'test_output' / 'junit.xml', type=Path)
    ap.add_argument('--out', default=REPO / 'test_output' / 'report', type=Path)
    args = ap.parse_args()
    cases = load_cases(args.junit)
    args.out.mkdir(parents=True, exist_ok=True)
    summary = args.out / 'summary.md'
    summary.write_text(summary_markdown(cases), encoding='utf-8')
    plt = setup_matplotlib()
    for path in (summary, fig_l1_checklist(plt, cases, args.out)):
        print(path)


if __name__ == '__main__':
    main()
