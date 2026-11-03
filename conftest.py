"""저장소 루트 pytest 설정 — 시험 분류 marker 와 junit 속성.

분류표는 tests/testmap.py 에 있다. 저장소 루트에서 pytest 를 실행할 때만
적용되고, 패키지 폴더 안에서 단독 실행하면 이 파일은 읽히지 않는다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from tests.checkmap import lookup as lookup_check  # noqa: E402
from tests.testmap import GROUPS, LEVELS, TEST_FILES  # noqa: E402


def pytest_configure(config):
    for level in LEVELS:
        config.addinivalue_line('markers', f'level_{level}: 시험 단 {level}')
    for group, label in GROUPS.items():
        config.addinivalue_line('markers', f'group_{group}: {label}')
    config.addinivalue_line('markers', 'unclassified: tests/testmap.py 에 없는 시험 파일')
    config.addinivalue_line('markers', 'check(item): 체크리스트 항목 A/B/C/D 또는 L2 항목')


def pytest_collection_modifyitems(config, items):
    import pytest
    for item in items:
        name = os.path.splitext(os.path.basename(str(item.fspath)))[0]
        entry = TEST_FILES.get(name)
        if entry is None:
            item.add_marker(pytest.mark.unclassified)
            item.user_properties.append(('level', 'unclassified'))
            continue
        level, group, function = entry
        item.add_marker(getattr(pytest.mark, f'level_{level}'))
        item.add_marker(getattr(pytest.mark, f'group_{group}'))
        item.user_properties.append(('level', level))
        item.user_properties.append(('group', group))
        item.user_properties.append(('function', function))
        # 체크리스트 항목: 시험 함수의 marker 가 우선, 없으면 tests/checkmap.py
        marker = item.get_closest_marker('check')
        if marker is not None and marker.args:
            check = str(marker.args[0])
        else:
            check = lookup_check(name, getattr(item, 'originalname', None) or item.name)
        item.user_properties.append(('check', check or 'untagged'))


def _reports_with(stats, key, value):
    return sorted({
        f'{os.path.basename(str(r.fspath))}::{r.head_line}' if value == 'untagged'
        else os.path.basename(str(r.fspath))
        for outcome in ('passed', 'failed', 'skipped')
        for r in stats.get(outcome, [])
        if (key, value) in getattr(r, 'user_properties', [])
    })


def pytest_terminal_summary(terminalreporter):
    stats = terminalreporter.stats
    unclassified = _reports_with(stats, 'level', 'unclassified')
    if unclassified:
        terminalreporter.write_line(
            'tests/testmap.py 에 분류되지 않은 시험 파일: ' + ', '.join(unclassified), yellow=True)
    untagged = _reports_with(stats, 'check', 'untagged')
    if untagged:
        terminalreporter.write_line(
            f'체크리스트 항목(A~D)이 없는 시험 {len(untagged)}개 — '
            '@pytest.mark.check 또는 tests/checkmap.py 에 추가:', yellow=True)
        for line in untagged:
            terminalreporter.write_line('  ' + line, yellow=True)
