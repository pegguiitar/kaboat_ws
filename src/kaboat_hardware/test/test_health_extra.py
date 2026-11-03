"""센서 상태 판정 — 정상 동작(A)·실패 시 안전(C) 보강."""
import pytest

from kaboat_hardware.health import OK, WARN, TopicTracker


@pytest.mark.check('A')
def test_steady_ten_hertz_topic_is_ok_with_measured_rate():
    tracker = TopicTracker(enabled=True, topic='/scan', min_rate_hz=5.0, timeout_sec=1.0)
    for k in range(11):
        tracker.observe(100.0 + 0.1 * k, 'laser')
    level, message, age, rate = tracker.evaluate(101.05, 99.0, 2.0)
    assert (level, message) == (OK, 'receiving')
    assert rate == pytest.approx(10.0)
    assert age == pytest.approx(0.05)


@pytest.mark.check('C')
def test_empty_frame_id_is_not_reported_ok():
    tracker = TopicTracker(enabled=True, topic='/scan', min_rate_hz=0.0, timeout_sec=1.0)
    for k in range(5):
        tracker.observe(100.0 + 0.1 * k, '')
    level, message, _, _ = tracker.evaluate(100.45, 99.0, 2.0)
    assert (level, message) == (WARN, 'empty frame_id')
