#!/usr/bin/env python3
"""젯슨 point_recorder 에 '지금 위치 기록' 요청을 보내고 결과를 출력한다 (Zenoh 경유).

  python3 scripts/rec_point.py gate_entry     # 지금 위치를 gate_entry 로 기록
  python3 scripts/rec_point.py                # 이름 생략 → p1, p2 …
  python3 scripts/rec_point.py --list         # 저장된 지점 보기

노트북(set_kaboat, DOMAIN 43)에서 Zenoh 브리지가 켜져 있으면 젯슨으로 간다. 젯슨에서
직접 실행해도 된다(DOMAIN 42, 단축어 rec_point / list_points). 워크스페이스 빌드 없이
rclpy·std_msgs 만 쓴다.

요청은 /mission_points/request 로 한 번 보내고 /mission_points/result 를 기다린다.
결과가 안 오면 같은 id 로 1.5초마다 다시 보낸다 — 젯슨은 같은 id 를 두 번 기록하지 않고
앞서 낸 결과를 다시 보낸다. 끝 상태: 0 = 기록 성공, 1 = 젯슨이 거부, 2 = 응답 없음.
"""

import argparse
import json
import sys
import time
import uuid

REQUEST_TOPIC = '/mission_points/request'
RESULT_TOPIC = '/mission_points/result'
RESEND_SEC = 1.5


def make_request(req_id: str, name: str = '', list_only: bool = False) -> str:
    if list_only:
        return json.dumps({'id': req_id, 'cmd': 'list'})
    return json.dumps({'id': req_id, 'cmd': 'record', 'name': name}, ensure_ascii=False)


def match_result(data: str, req_id: str):
    """결과 문자열이 이 요청(id)의 것이면 dict, 아니면 None."""
    try:
        result = json.loads(data)
    except ValueError:
        return None
    if isinstance(result, dict) and result.get('id') == req_id:
        return result
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description='젯슨에 지금 위치 기록 요청')
    parser.add_argument('name', nargs='?', default='', help='지점 이름 (생략하면 p1, p2 …)')
    parser.add_argument('--list', action='store_true', help='저장된 지점 보기')
    parser.add_argument('--timeout', type=float, default=8.0, help='응답 기다리는 시간 [s]')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    req_id = uuid.uuid4().hex[:8]
    request = String(data=make_request(req_id, args.name.strip(), args.list))
    results = []

    rclpy.init()
    node = Node(f'rec_point_{req_id}')
    try:
        pub = node.create_publisher(String, REQUEST_TOPIC, 10)
        node.create_subscription(
            String, RESULT_TOPIC,
            lambda msg: results.extend(r for r in [match_result(msg.data, req_id)] if r), 10)

        deadline = time.monotonic() + args.timeout
        # 받는 쪽(젯슨 point_recorder 또는 Zenoh 브리지)이 보일 때까지 기다린 뒤 보낸다
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print('✗ 받는 쪽이 없음 — 젯슨 run_points·run_zenoh, 노트북 set_kaboat 를 확인',
                  file=sys.stderr)
            return 2

        next_send = 0.0
        while not results and time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                pub.publish(request)
                next_send = time.monotonic() + RESEND_SEC
            rclpy.spin_once(node, timeout_sec=0.1)
        if not results:
            print(f'✗ {args.timeout:.0f}초 동안 응답 없음 — 젯슨 point_recorder 가 켜져 있나?',
                  file=sys.stderr)
            return 2

        result = results[0]
        print(('✓ ' if result.get('success') else '✗ ') + str(result.get('message', '')))
        return 0 if result.get('success') else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
