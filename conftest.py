"""저장소 루트 pytest 설정 — 시험 분류 marker 와 junit 속성.

분류표는 tests/testmap.py 에 있다. 저장소 루트에서 pytest 를 실행할 때만
적용되고, 패키지 폴더 안에서 단독 실행하면 이 파일은 읽히지 않는다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

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
        check = item.get_closest_marker('check')
        if check is not None and check.args:
            item.user_properties.append(('check', str(check.args[0])))


def pytest_terminal_summary(terminalreporter):
    stats = terminalreporter.stats
    unclassified = sorted({
        os.path.basename(str(r.fspath)) for key in ('passed', 'failed', 'skipped')
        for r in stats.get(key, [])
        if any(k == 'level' and v == 'unclassified' for k, v in getattr(r, 'user_properties', []))
    })
    if unclassified:
        terminalreporter.write_line(
            'tests/testmap.py 에 분류되지 않은 시험 파일: ' + ', '.join(unclassified), yellow=True)
