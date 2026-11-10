"""rec_point.py 요청·결과 문자열 (ROS 없이)."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rec_point  # noqa: E402


def test_request_is_short_json_with_id():
    req = json.loads(rec_point.make_request('ab12cd34', '게이트1'))
    assert req == {'id': 'ab12cd34', 'cmd': 'record', 'name': '게이트1'}
    assert json.loads(rec_point.make_request('x', list_only=True))['cmd'] == 'list'
    assert len(rec_point.make_request('ab12cd34', 'gate_entry')) < 80


def test_only_matching_result_is_taken():
    ok = json.dumps({'id': 'mine', 'success': True, 'message': 'gate: x=+1.00'})
    assert rec_point.match_result(ok, 'mine')['success'] is True
    assert rec_point.match_result(ok, 'other') is None              # 다른 노트북의 요청
    assert rec_point.match_result('not json', 'mine') is None
    assert rec_point.match_result('[1, 2]', 'mine') is None
