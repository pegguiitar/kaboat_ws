"""rtcm_guard: GQ7 AUX 쓰기 실패 로그 폭주를 끊고 RTK 상태를 알린다 (2026-11-13 토너먼트: 11:36~11:59
'Failed to write RTCM to aux port' 3,293줄, RTK Fixed → SBAS 를 아무도 모름)."""

import pytest

from kaboat_hardware.rtcm_guard import GuardParams, RtcmGuardLogic


def _paused(g, t=0.0):
    out = []
    for k in range(5):
        out += g.on_failure_log(t + 0.1 * k)
    return out


def test_forwards_until_failures_burst_then_pauses_with_one_clear_message():
    g = RtcmGuardLogic()
    assert all(g.forward(0.1 * k) for k in range(20))         # 평소엔 그대로 중계
    out = _paused(g, 3.0)
    assert len(out) == 1 and out[0][0] == 'error'              # 실패 5번에 안내 1번
    assert 'run_gq7_rtk' in out[0][1] and '/dev/microstrain_aux' in out[0][1]
    assert not g.forward(3.6) and not g.forward(3.7)            # 중계 멈춤 → 드라이버 로그도 멈춤
    assert g.status().endswith('보정 중계 멈춤')


def test_scattered_failures_do_not_pause():
    g = RtcmGuardLogic()
    for k in range(20):
        assert g.on_failure_log(k * 2.0) == []                  # 2 초에 하나 — 5 초 안 5번 안 됨
    assert g.forward(41.0)


def test_probe_fails_stays_paused_and_reports_periodically():
    p = GuardParams()
    g = RtcmGuardLogic(p)
    _paused(g)
    assert not g.forward(5.0)
    assert g.forward(10.5)                                       # 10 s 뒤 하나 넣어 봄
    assert not g.forward(10.6)
    assert g.on_failure_log(10.7) == []                         # 또 실패 — 조용히 계속 멈춤
    assert g.tick(13.0) == []
    assert not g.forward(15.0)
    out = g.tick(30.5)                                           # 30 s 마다 다시 안내
    assert [lvl for lvl, _ in out] == ['error']
    assert g.tick(31.0) == []


def test_probe_succeeds_resumes_forwarding():
    g = RtcmGuardLogic()
    _paused(g)
    for t in (2.0, 4.0, 6.0):
        g.forward(t)
    assert g.forward(10.5)                                       # 넣어 봄
    out = g.tick(13.0)                                           # 2 s 안에 실패 없음 → 복구
    assert [lvl for lvl, _ in out] == ['info'] and '다시 시작' in out[0][1]
    assert g.forward(13.1) and g.forward(13.2)
    assert g.status().endswith('보정 중계 중')


def test_warns_when_corrections_flow_but_no_rtk():
    g = RtcmGuardLogic()
    g.on_gnss_state('RTK Fixed', 0.0)
    for t in range(0, 300):
        g.forward(float(t))
        if t == 10:
            g.on_gnss_state('SBAS', float(t))
        out = g.tick(float(t))
        if t < 131:
            assert out == [], (t, out)
        elif t == 131:
            assert out and out[0][0] == 'warn' and 'RTK 아님' in out[0][1]
    g.on_gnss_state('RTK Float', 300.0)
    g.forward(399.5)                                             # 보정은 계속 들어옴
    assert g.tick(400.0) == []


def test_warns_when_no_ntrip_corrections_arrive():
    g = RtcmGuardLogic()
    g.tick(0.0)
    assert g.tick(5.0) == []
    out = g.tick(11.0)
    assert out and 'NTRIP 보정' in out[0][1]
    assert g.tick(30.0) == []                                    # 60 s 마다만


def test_node_relays_and_stops_on_driver_failure_logs():
    """실제 rclpy 노드: /rtcm → /rtcm/gq7 중계, 드라이버 실패 로그가 쏟아지면 멈춤."""
    import rclpy
    from rcl_interfaces.msg import Log
    from rclpy.executors import SingleThreadedExecutor
    from rtcm_msgs.msg import Message
    from kaboat_hardware.rtcm_guard import build_node

    rclpy.init()
    try:
        guard = build_node()
        peer = rclpy.create_node('rtcm_guard_test_peer')
        got = []
        peer.create_subscription(Message, '/rtcm/gq7', got.append, 100)
        rtcm = peer.create_publisher(Message, '/rtcm', 100)
        log = peer.create_publisher(Log, '/rosout', 100)
        ex = SingleThreadedExecutor()
        ex.add_node(guard)
        ex.add_node(peer)

        def spin(n=20):
            for _ in range(n):
                ex.spin_once(timeout_sec=0.02)

        spin()
        for _ in range(3):
            rtcm.publish(Message(message=[1, 2, 3]))
        spin()
        assert len(got) == 3
        for _ in range(6):
            log.publish(Log(name='microstrain_inertial_driver', level=40,
                            msg='Failed to write RTCM to aux port'))
        spin()
        assert not guard.logic.forwarding
        for _ in range(3):
            rtcm.publish(Message(message=[4]))
        spin()
        assert len(got) == 3                                     # 멈춘 뒤 안 넘김
        ex.remove_node(guard)
        ex.remove_node(peer)
        guard.destroy_node()
        peer.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('limit', [3, 5])
def test_fail_limit_parameter(limit):
    g = RtcmGuardLogic(GuardParams(fail_limit=limit))
    for k in range(limit - 1):
        assert g.on_failure_log(k * 0.1) == []
    assert g.on_failure_log(1.0)
