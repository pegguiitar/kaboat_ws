from kaboat_hardware.health import ERROR, OK, WARN, TopicTracker


def test_disabled_topic_is_ok():
    tracker = TopicTracker(False, '/scan', 5.0, 1.0)
    assert tracker.evaluate(20.0, 0.0, 10.0)[0] == OK


def test_missing_topic_transitions_after_grace_period():
    tracker = TopicTracker(True, '/scan', 5.0, 1.0)
    assert tracker.evaluate(5.0, 0.0, 10.0)[0] == WARN
    assert tracker.evaluate(11.0, 0.0, 10.0)[0] == ERROR


def test_stale_topic_is_error():
    tracker = TopicTracker(True, '/imu/data', 20.0, 0.5)
    tracker.observe(1.0, 'imu_link')
    assert tracker.evaluate(1.6, 0.0, 0.0)[1] == 'stale'


def test_rate_and_frame_are_reported():
    tracker = TopicTracker(True, '/scan', 5.0, 1.0)
    tracker.observe(1.0, 'laser_link')
    tracker.observe(1.1, 'laser_link')
    tracker.observe(1.2, 'laser_link')
    level, message, _age, rate = tracker.evaluate(1.25, 0.0, 0.0)
    assert level == OK
    assert message == 'receiving'
    assert rate == 10.0
